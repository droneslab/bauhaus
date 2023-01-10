
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../core/robust_kernel_impl.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../core/optimization_algorithm_gauss_newton.h"
#include "../solvers/linear_solver_dense.h"
#include "../types/se3quat.h"
#include "../types/types_six_dof_expmap.h"
#include "../../../target/cxxbridge/g2o/src/lib.rs.h"
#include "../core/hyper_graph.h"

namespace g2o {
    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer(int opt_type, std::array<double,4> camera_param) {
        return std::unique_ptr<BridgeSparseOptimizer>(new BridgeSparseOptimizer(opt_type, camera_param));
    }

    BridgeSparseOptimizer::BridgeSparseOptimizer(int opt_type, std::array<double,4> camera_param) {
        if (opt_type == 1) {
            // For Optimizer::PoseOptimization and GlobalBundleAdjustemnt
            optimizer = new SparseOptimizer();
            BlockSolver_6_3::LinearSolverType * linearSolver = new LinearSolverDense<BlockSolver_6_3::PoseMatrixType>();
            BlockSolver_6_3* solver_ptr = new BlockSolver_6_3(linearSolver);
            OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer->setAlgorithm(solver);

            deltaMono = sqrt(5.991);
            deltaStereo = sqrt(7.815);
            optimizer_type = 1;

            fx = camera_param[0];
            fy = camera_param[1];
            cx = camera_param[2];
            cy = camera_param[3];
        } else {
            // For Optimizer::PoseInertialOptimizationLastFrame and Optimizer::PoseInertialOptimizationLastKeyFrame
            optimizer = new SparseOptimizer();
            BlockSolverX::LinearSolverType * linearSolver = new LinearSolverDense<BlockSolverX::PoseMatrixType>();
            BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);

            OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(solver_ptr);
            optimizer->setVerbose(false);
            optimizer->setAlgorithm(solver);

            thHuberMono = sqrt(5.991);
            thHuberStereo = sqrt(7.815);
            thHuber2D = sqrt(5.99);
            thHuber3D = sqrt(7.815);
            optimizer_type = 2;
            
            fx = camera_param[0];
            fy = camera_param[1];
            cx = camera_param[2];
            cy = camera_param[3];
        }
    }
 
    //* Vertices *//
    bool BridgeSparseOptimizer::has_vertex(int id) const {
        return optimizer->vertex(id) != NULL;
    }

    void BridgeSparseOptimizer::remove_vertex (int vertex_id) {
        optimizer->removeVertex(optimizer->vertex(vertex_id));
    }

    void BridgeSparseOptimizer::add_frame_vertex (
        int vertex_id,  Pose pose, bool set_fixed
    ) {
        if (optimizer_type == 1) {
            VertexSE3Expmap * vSE3 = new VertexSE3Expmap();
            vSE3->setEstimate(this->format_pose(pose));
            // std::cout << "Set frame to " << vSE3->estimate() << std::endl;
            vSE3->setId(vertex_id);
            vSE3->setFixed(set_fixed);
            optimizer->addVertex(vSE3);
            // return vSE3;
        } else {
            // For Optimizer::PoseInertialOptimizationLastFrame and Optimizer::PoseInertialOptimizationLastKeyFrame
            // TODO (IMU)
            // Following variables rely on types/G2oTypes and types/ImuTypes, from ORBSLAM3
            // but those have dependency on Frame and KeyFrame. Need to go through and figure out how to
            // remove the dependency by just passing minimal info instead of the whole object.
            // VertexPose* VP = new VertexPose(pFrame);
            // VP->setId(0);
            // VP->setFixed(false);
            // optimizer.addVertex(VP);
            // VertexVelocity* VV = new VertexVelocity(pFrame);
            // VV->setId(1);
            // VV->setFixed(false);
            // optimizer->addVertex(VV);
            // VertexGyroBias* VG = new VertexGyroBias(pFrame);
            // VG->setId(2);
            // VG->setFixed(false);
            // optimizer->addVertex(VG);
            // VertexAccBias* VA = new VertexAccBias(pFrame);
            // VA->setId(3);
            // VA->setFixed(false);
            // optimizer->addVertex(VA);
        }
    }

    void BridgeSparseOptimizer::add_mappoint_vertex(int vertex_id,  Pose pose) {
        VertexSBAPointXYZ * vPoint = new VertexSBAPointXYZ();
        Eigen::Vector3d translation(pose.translation.data());
        vPoint->setEstimate(translation);
        // std::cout << "Set mp to " << vPoint->estimate() << std::endl;
        vPoint->setId(vertex_id);
        vPoint->setMarginalized(true);
        optimizer->addVertex(vPoint);
    }

    SE3Quat BridgeSparseOptimizer::format_pose(Pose pose) const {
        Eigen::Vector3d trans_vec(pose.translation.data());
        //Pranay : quaternion in nlabegra [w,x,y,z] while in eigen [x,y,z,w]
        auto rot_quat_val = pose.rotation.data();
        Eigen::Quaterniond rot_quat(rot_quat_val[0], rot_quat_val[1], rot_quat_val[2],rot_quat_val[3]);
        // Rotation is already a quaternion in Darvis!!
        // Don't need to do the conversion in ORBSLAM3
        return SE3Quat(rot_quat, trans_vec);
    }

    void BridgeSparseOptimizer::set_vertex_estimate(int vertex_id, Pose pose) {
        VertexSE3Expmap* v = dynamic_cast<VertexSE3Expmap*>(optimizer->vertex(vertex_id));
        v->setEstimate(format_pose(pose));
    }

    //* Edges *//
    int BridgeSparseOptimizer::num_edges() const {
        return optimizer->edges().size();
    }

    unique_ptr<EdgeSE3ProjectXYZOnlyPose> BridgeSparseOptimizer::add_edge_monocular_unary(
        bool robust_kernel, int vertex_id,
        int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float invSigma2,
        array<double, 3> mp_world_position
    ) {
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        EdgeSE3ProjectXYZOnlyPose * edge = new EdgeSE3ProjectXYZOnlyPose();
        edge->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id)));

        edge->setMeasurement(obs);
        edge->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        if (robust_kernel) {
            RobustKernelHuber * rk = new RobustKernelHuber();
            edge->setRobustKernel(rk);
            rk->setDelta(thHuber2D);
        }

        // Pranay : Important camera settings
        edge->fx = fx;
        edge->fy = fy;
        edge->cx = cx;
        edge->cy = cy;

        Eigen::Vector3d worldpos_vec(mp_world_position.data());
        edge->Xw = worldpos_vec;

        optimizer->addEdge(edge);

        unique_ptr<EdgeSE3ProjectXYZOnlyPose> ptr_edge(edge);
        return ptr_edge;
    }

    std::unique_ptr<g2o::EdgeSE3ProjectXYZ> BridgeSparseOptimizer::add_edge_monocular_binary(
        bool robust_kernel, int vertex1, int vertex2,
        int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float invSigma2,
        int huber_delta
    ) {
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        EdgeSE3ProjectXYZ * edge = new EdgeSE3ProjectXYZ();
        edge->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex1)));
        edge->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex2)));
        edge->setMeasurement(obs);
        edge->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        if (robust_kernel) {
            RobustKernelHuber * rk = new RobustKernelHuber;
            edge->setRobustKernel(rk);
            if (huber_delta == 0) {
                rk->setDelta(thHuber2D);
            } else if (huber_delta == 1) {
                rk->setDelta(thHuberMono);
            }
        }

        // Pranay : Important camera settings
        edge->fx = fx;
        edge->fy = fy;
        edge->cx = cx;
        edge->cy = cy;

        optimizer->addEdge(edge);

        unique_ptr<EdgeSE3ProjectXYZ> ptr_edge(edge);
        return ptr_edge;
    }

    //** Optimization *//
    void BridgeSparseOptimizer::optimize(int iterations) {
        optimizer->initializeOptimization(0);
        optimizer->optimize(iterations);
    }

    Pose BridgeSparseOptimizer::recover_optimized_frame_pose(int vertex_id) const {
        // std::cout << "recover_optimized_frame_pose" << std::endl;
        const VertexSE3Expmap* v = dynamic_cast<const VertexSE3Expmap*>(optimizer->vertex(vertex_id));

        g2o::SE3Quat SE3quat_recov = v->estimate();
        Vector3d translation = SE3quat_recov.translation();
        Quaterniond rotation = SE3quat_recov.rotation();

        // TODO (verify): make sure that quaternion order of (w,x,y,z)
        // is the order that we use for quaternions in darvis
        // Also, feel like there should be a cleaner way to do this?
        Pose pose;
        pose.translation = {
            (double) translation.x(),
            (double) translation.y(),
            (double) translation.z()
        };
        pose.rotation = {
            (double) rotation.w(), 
            (double) rotation.x(),
            (double) rotation.y(),
            (double) rotation.z()
        };
        return pose;
    }

    Pose BridgeSparseOptimizer::recover_optimized_mappoint_pose(int vertex_id) const {
        // std::cout<< "recover_optimized_mappoint_pose" << std::endl;
        const VertexSBAPointXYZ* v = dynamic_cast<const VertexSBAPointXYZ*>(optimizer->vertex(vertex_id));

        Eigen::Vector3f pos = v->estimate().cast<float>();

        // TODO (verify): make sure that quaternion order of (w,x,y,z)
        // is the order that we use for quaternions in darvis
        // Also, feel like there should be a cleaner way to do this?
        Pose pose;
        pose.translation = {
            (double) pos.x(),
            (double) pos.y(),
            (double) pos.z()
        };
        pose.rotation = {
            (double) 1.0,
            (double) 0.0,
            (double) 0.0,
            (double) 0.0
        };
        return pose;
    }

    // }
    // Sofiya note: This might already be deleted when g2o deletes the sparseoptimizer?
    // BridgeSparseOptimizer::~BridgeSparseOptimizer() {
    //     delete linearSolver;
    //     delete solver_ptr;
    //     delete solver;
    // }

} // end namespace
