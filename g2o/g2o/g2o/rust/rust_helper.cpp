
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../core/robust_kernel_impl.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../core/optimization_algorithm_gauss_newton.h"
#include "../solvers/linear_solver_dense.h"
#include "../types/se3quat.h"
#include "../types/types_six_dof_expmap.h"
#include "../../../target/cxxbridge/g2o/src/lib.rs.h"

namespace g2o {
    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer(int opt_type) {
        return std::unique_ptr<BridgeSparseOptimizer>(new BridgeSparseOptimizer(opt_type));
    }

    BridgeSparseOptimizer::BridgeSparseOptimizer(int opt_type) {
        if (opt_type == 1) {
            // For Optimizer::PoseOptimization and GlobalBundleAdjustemnt
            optimizer = std::make_unique<SparseOptimizer>();
            BlockSolver_6_3::LinearSolverType * linearSolver = new LinearSolverDense<BlockSolver_6_3::PoseMatrixType>();
            BlockSolver_6_3* solver_ptr = new BlockSolver_6_3(linearSolver);
            OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer->setAlgorithm(solver);

            deltaMono = sqrt(5.991);
            deltaStereo = sqrt(7.815);
            optimizer_type = 1;
        } else {
            // For Optimizer::PoseInertialOptimizationLastFrame and Optimizer::PoseInertialOptimizationLastKeyFrame
            optimizer = std::make_unique<SparseOptimizer>();
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
        }
    }
 
    //* Vertices *//
    bool BridgeSparseOptimizer::has_vertex(int id) const {
        return optimizer->vertex(id) != NULL;
    }

    void BridgeSparseOptimizer::remove_vertex (std::shared_ptr<VertexSBAPointXYZ> vertex) {
        cout << "remove vertex " << vertex->id() << endl;
        optimizer->removeVertex(vertex.get());
    }

    std::shared_ptr<VertexSE3Expmap> BridgeSparseOptimizer::add_frame_vertex (
        int vertex_id,  Pose pose, bool set_fixed
    ) {
        if (optimizer_type == 1) {
            std::shared_ptr<VertexSE3Expmap> vSE3 = std::make_shared<VertexSE3Expmap>();
            vSE3->setEstimate(this->format_pose(pose));
            vSE3->setId(vertex_id);
            vSE3->setFixed(set_fixed);
            optimizer->addVertex(vSE3.get());
            return vSE3;
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

    std::shared_ptr<VertexSBAPointXYZ> BridgeSparseOptimizer::add_mappoint_vertex(int vertex_id,  Pose pose) {
        VertexSBAPointXYZ * vPoint = new VertexSBAPointXYZ();
        Eigen::Vector3d translation(pose.translation.data());
        vPoint->setEstimate(translation);
        vPoint->setId(vertex_id);
        vPoint->setMarginalized(true);
        optimizer->addVertex(vPoint);
        shared_ptr<VertexSBAPointXYZ> other_ptr(vPoint);
        return other_ptr;
    }

    SE3Quat BridgeSparseOptimizer::format_pose(Pose pose) const {
        Eigen::Vector3d trans_vec(pose.translation.data());
        Eigen::Quaterniond rot_quat(pose.rotation.data());
        // Rotation is already a quaternion in Darvis!!
        // Don't need to do the conversion in ORBSLAM3
        return SE3Quat(rot_quat, trans_vec);
    }

    void BridgeSparseOptimizer::set_vertex_estimate(std::shared_ptr<VertexSE3Expmap> vertex, Pose pose) {
        vertex->setEstimate(format_pose(pose));
    }

    //* Edges *//
    int BridgeSparseOptimizer::num_edges() const {
        return optimizer->edges().size();
    }

    std::unique_ptr<EdgeSE3ProjectXYZOnlyPose> BridgeSparseOptimizer::add_edge_monocular_unary(
        bool robust_kernel, int vertex_id,
        int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float invSigma2,
        array<double, 3> mp_world_position
    ) {
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        std::unique_ptr<EdgeSE3ProjectXYZOnlyPose> edge = std::make_unique<EdgeSE3ProjectXYZOnlyPose>();
        edge->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id)));

        edge->setMeasurement(obs);
        edge->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        if (robust_kernel) {
            std::unique_ptr<RobustKernelHuber> rk(new RobustKernelHuber);
            edge->setRobustKernel(rk.get());
            rk->setDelta(thHuber2D);
        }
        // TODO (need?)
        // e->pCamera = pKF->mpCamera;

        Eigen::Vector3d worldpos_vec(mp_world_position.data());
        edge->Xw = worldpos_vec;

        optimizer->addEdge(edge.get());

        return edge;
    }
    std::unique_ptr<EdgeSE3ProjectXYZ> BridgeSparseOptimizer::add_edge_monocular_binary(
        bool robust_kernel, std::shared_ptr<VertexSBAPointXYZ> vertex1, std::shared_ptr<VertexSE3Expmap> vertex2,
        int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float invSigma2
    ) {
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        EdgeSE3ProjectXYZ * edge = new EdgeSE3ProjectXYZ();
        edge->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(vertex1.get()));
        edge->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(vertex2.get()));
        edge->setMeasurement(obs);
        edge->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        if (robust_kernel) {
            std::unique_ptr<RobustKernelHuber> rk(new RobustKernelHuber);
            edge->setRobustKernel(rk.get());
            rk->setDelta(thHuber2D);
        }
        // TODO (need?)
        // e->pCamera = pKF->mpCamera;

        optimizer->addEdge(edge);

        unique_ptr<EdgeSE3ProjectXYZ> other_ptr(edge);
        return other_ptr;
    }

    //** Optimization *//
    void BridgeSparseOptimizer::optimize(int iterations) {
        cout << optimizer->vertices().size() << endl;
        optimizer->initializeOptimization(0);
        optimizer->optimize(iterations);
    }

    Pose BridgeSparseOptimizer::recover_optimized_frame_pose(int vertex_id) const {
        g2o::VertexSE3Expmap* vSE3_recov = static_cast<VertexSE3Expmap*>(optimizer->vertex(vertex_id));
        g2o::SE3Quat SE3quat_recov = vSE3_recov->estimate();
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
        g2o::VertexSBAPointXYZ* vertex = static_cast<VertexSBAPointXYZ*>(optimizer->vertex(vertex_id));
        Eigen::Vector3f pos = vertex->estimate().cast<float>();

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
            (double) 0.0,
            (double) 0.0,
            (double) 0.0,
            (double) 0.0
        };
        return pose;
    }

    //** Functions on edge *//
    // void BridgeEdgeSE3ProjectXYZOnlyPose::set_level(int level) {edge->setLevel(1);}
    // void BridgeEdgeSE3ProjectXYZOnlyPose::compute_error() const {edge->computeError();}
    // double BridgeEdgeSE3ProjectXYZOnlyPose::chi2() const {return edge->chi2();}
    void EdgeSE3ProjectXYZOnlyPose::set_robust_kernel(bool reset) {
        // Note: setRobustKernel takes a RobustKernelHuber pointer
        // ORBSLAM3 usually does this but occasionally passes in a 0 instead
        // Here is an alternative implementation that takes a boolean:
        // http://docs.ros.org/en/fuerte/api/re_vision/html/optimizable__graph_8h_source.html
        // although this implementation isn't in the ORBSLAM3 modified g2o...
        // so I have no idea how they are passing in a 0 and compiling it correctly.
        // I *think* that passing in a 0 is equivalent to removing the robust kernel pointer.
        if (reset) {
            setRobustKernel(NULL);
        } else {
            RobustKernelHuber* rk = new RobustKernelHuber;
            setRobustKernel(rk);
        }
    }

    // Sofiya note: This might already be deleted when g2o deletes the sparseoptimizer?
    // BridgeSparseOptimizer::~BridgeSparseOptimizer() {
    //     delete linearSolver;
    //     delete solver_ptr;
    //     delete solver;
    // }

} // end namespace
