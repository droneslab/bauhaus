
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../core/robust_kernel_impl.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../core/optimization_algorithm_gauss_newton.h"
#include "../solvers/linear_solver_dense.h"
#include "../solvers/linear_solver_eigen.h"
#include "../types/se3quat.h"
#include "../types/types_six_dof_expmap.h"
#include "../types/types_seven_dof_expmap.h"
#include "/home/nitin/Downloads/Darvis_Upd/darvis/darvis/target/cxxbridge/g2o/src/lib.rs.h"
#include "../core/hyper_graph.h"
#include <numeric>

namespace g2o {
    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer(int opt_type, std::array<double,4> camera_param) {
        BridgeSparseOptimizer * optimizer = new BridgeSparseOptimizer(opt_type, camera_param);
        unique_ptr<BridgeSparseOptimizer> ptr(optimizer);
        return ptr;
    }
 
    BridgeSparseOptimizer::BridgeSparseOptimizer(int opt_type, std::array<double,4> camera_param) {
        this->xyz_edges = std::vector<RustXYZEdge>();
        this->xyz_onlypose_edges = std::vector<RustXYZOnlyPoseEdge>();
        this->sim3_edges = std::vector<RustSim3Edge>();

        if (opt_type == 1) {
            // For GlobalBundleAdjustemnt
            optimizer = new SparseOptimizer();
            BlockSolver_6_3::LinearSolverType * linearSolver = new LinearSolverEigen<BlockSolver_6_3::PoseMatrixType>();
            BlockSolver_6_3* solver_ptr = new BlockSolver_6_3(linearSolver);
            OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer->setAlgorithm(solver);
            optimizer->setVerbose(false);

            optimizer_type = 1;
        } else if (opt_type == 2) {
            // For PoseOptimization
            optimizer = new SparseOptimizer();
            g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
            g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer->setAlgorithm(solver);

            optimizer_type = 2;
        } else if (opt_type == 3) {
            // For OptimizeSim3
            optimizer = new SparseOptimizer();
            BlockSolverX::LinearSolverType * linearSolver = new LinearSolverDense<BlockSolverX::PoseMatrixType>();
            BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);

            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer->setVerbose(false);
            optimizer->setAlgorithm(solver);

            optimizer_type = 3;
        } else {
            // For Optimizer::PoseInertialOptimizationLastFrame, Optimizer::PoseInertialOptimizationLastKeyFrame,
            optimizer = new SparseOptimizer();
            BlockSolverX::LinearSolverType * linearSolver = new LinearSolverDense<BlockSolverX::PoseMatrixType>();
            BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);

            OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(solver_ptr);
            optimizer->setVerbose(false);
            optimizer->setAlgorithm(solver);

            optimizer_type = 4;
        }

        thHuberMono = sqrt(5.991);
        thHuberStereo = sqrt(7.815);
        thHuber2D = sqrt(5.99);
        thHuber3D = sqrt(7.815);

        fx = camera_param[0];
        fy = camera_param[1];
        cx = camera_param[2];
        cy = camera_param[3];

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
        if (optimizer_type == 1 || optimizer_type == 2) {
            VertexSE3Expmap * vSE3 = new VertexSE3Expmap();
            vSE3->setEstimate(this->format_pose(pose));
            // std::cout << "Set frame to " << vSE3->estimate() << std::endl;
            vSE3->setId(vertex_id);
            vSE3->setFixed(set_fixed);
            optimizer->addVertex(vSE3);
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

    void BridgeSparseOptimizer::add_mappoint_vertex(
        int vertex_id,  Pose pose, bool set_fixed, bool set_marginalized
    ) {
        VertexSBAPointXYZ * vPoint = new VertexSBAPointXYZ();
        Eigen::Vector3d translation(pose.translation.data());
        vPoint->setEstimate(translation);
        // std::cout << "Set mp to " << vPoint->estimate() << std::endl;
        vPoint->setId(vertex_id);
        vPoint->setMarginalized(set_marginalized);
        vPoint->setFixed(set_fixed);
        optimizer->addVertex(vPoint);
    }

    SE3Quat BridgeSparseOptimizer::format_pose(Pose pose) const {
        Eigen::Vector3d trans_vec(pose.translation.data());
        auto rot_quat_val = pose.rotation.data();
        Eigen::Quaterniond rot_quat(rot_quat_val[0], rot_quat_val[1], rot_quat_val[2],rot_quat_val[3]);
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

    // Note: see explanation under get_mut_edges in lib.rs for why we do this
    // This function is being called
    std::vector<RustXYZOnlyPoseEdge>& BridgeSparseOptimizer::get_mut_xyz_onlypose_edges() {
        //std::cout << "get_mut_xyz_onlypose_edges is being called" << std::endl;
        return this->xyz_onlypose_edges;
    }
    std::vector<RustXYZEdge>& BridgeSparseOptimizer::get_mut_xyz_edges() {
        //std::cout << "get_mut_xyz_edges is being called" << std::endl;
        return this->xyz_edges;
    }
    std::vector<RustSim3Edge>& BridgeSparseOptimizer::get_mut_sim3_edges() {
        //std::cout << "get_mut_sim3_edges is being called" << std::endl;
        return this->sim3_edges;
    }

    void BridgeSparseOptimizer::add_edge_monocular_unary(
        bool robust_kernel, int vertex_id,
        int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float inv_sigma2,
        array<double, 3> mp_world_position,
        int mappoint_id,
        float huber_delta
    ) {

        // This function is being called
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;
        EdgeSE3ProjectXYZOnlyPose * edge = new EdgeSE3ProjectXYZOnlyPose();

        // This is where the edges are being pushed back into the container
        edge_container_all_types.push_back(edge);

        //if(dynamic_cast<EdgeSE3ProjectXYZOnlyPose*>(edge_container_all_types.at(1)))
        //{

        //   //std::cout << "This is the right types" << std::endl;  
        //   EdgeSE3ProjectXYZOnlyPose * edge12 = dynamic_cast<EdgeSE3ProjectXYZOnlyPose*>(edge_container_all_types.at(1));
        //   std::cout << edge12->fx << " This is the answer after casting" << std::endl;

        //}
        edge->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id)));

        edge->setMeasurement(obs);
        edge->setInformation(Eigen::Matrix2d::Identity()*inv_sigma2);

        if (robust_kernel) {
            RobustKernelHuber * rk = new RobustKernelHuber();
            edge->setRobustKernel(rk);
            rk->setDelta(huber_delta);
        }

        // Pranay : Important camera settings
        edge->fx = fx;
        edge->fy = fy;
        edge->cx = cx;
        edge->cy = cy;

        Eigen::Vector3d worldpos_vec(mp_world_position.data());
        edge->Xw = worldpos_vec;

        optimizer->addEdge(edge);

        // Note: see explanation under get_mut_edges in lib.rs for why we do this
        unique_ptr<EdgeSE3ProjectXYZOnlyPose> ptr_edge(edge);
        RustXYZOnlyPoseEdge rust_edge;
        rust_edge.inner = std::move(ptr_edge);
        rust_edge.mappoint_id = mappoint_id;
        this->xyz_onlypose_edges.emplace(this->xyz_onlypose_edges.end(), std::move(rust_edge));
    }

    void BridgeSparseOptimizer::add_edge_monocular_binary(
        bool robust_kernel, int vertex1, int vertex2,
        float keypoint_pt_x, float keypoint_pt_y, float inv_sigma2,
        float huber_delta
    ) {
        // This function is being called 
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        //std::cout << "Add Edge binary is being called " << std::endl;
        EdgeSE3ProjectXYZ * edge = new EdgeSE3ProjectXYZ();
        
        // This is where the edges are being moved back into the container
        edge_container_all_types.push_back(edge);

        edge->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex1)));
        edge->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex2)));
        edge->setMeasurement(obs);
        edge->setInformation(Eigen::Matrix2d::Identity()*inv_sigma2);

        if (robust_kernel) {
            RobustKernelHuber * rk = new RobustKernelHuber();
            edge->setRobustKernel(rk);
            rk->setDelta(huber_delta);
        }

        edge->fx = fx;
        edge->fy = fy;
        edge->cx = cx;
        edge->cy = cy;

        auto success = optimizer->addEdge(edge);
        if (!success) {
            std::cout << "Optimizer failed to add edge!!" << std::endl;
        }
        // Note: see explanation under get_mut_edges in lib.rs for why we do this
        unique_ptr<EdgeSE3ProjectXYZ> ptr_edge(edge);
        RustXYZEdge rust_edge {std::move(ptr_edge)};
        this->xyz_edges.emplace(this->xyz_edges.end(), std::move(rust_edge));
    }

    // ** SIM3 ** //
    // Functions similar to those above, just specific to sim3 optimization. Keeping them all in one place.
    void BridgeSparseOptimizer::add_sim3_vertex (
        Pose pose, double scale, bool fix_scale
    ) {
        g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();    
        vSim3->_fix_scale=fix_scale;
        vSim3->setEstimate(this->format_sim3(pose, scale));
        vSim3->setId(0);
        vSim3->setFixed(false);
        vSim3->_principle_point1[0] = cx;
        vSim3->_principle_point1[1] = cy;
        vSim3->_focal_length1[0] = fx;
        vSim3->_focal_length1[1] = fy;
        vSim3->_principle_point2[0] = cx;
        vSim3->_principle_point2[1] = cy;
        vSim3->_focal_length2[0] = fx;
        vSim3->_focal_length2[1] = fy;
        optimizer->addVertex(vSim3);
    }

    Sim3 BridgeSparseOptimizer::format_sim3(Pose pose, float scale) const {
        Eigen::Vector3d trans_vec(pose.translation.data());
        auto rot_quat_val = pose.rotation.data();
        Eigen::Quaterniond rot_quat(rot_quat_val[0], rot_quat_val[1], rot_quat_val[2],rot_quat_val[3]);
        return Sim3(rot_quat, trans_vec, scale);
    }

    void BridgeSparseOptimizer::add_both_sim_edges(
        int vertex_id1, float kp_pt_x_1, float kp_pt_y_1, float inv_sigma1,
        int vertex_id2, float kp_pt_x_2, float kp_pt_y_2, float inv_sigma2,
        float huber_delta
    ) {
        // Set edge x1 = S12*X2
        Eigen::Matrix<double,2,1> obs1;
        obs1 << kp_pt_x_1, kp_pt_y_1;

        EdgeSim3ProjectXYZ* e12 = new EdgeSim3ProjectXYZ();
    
        // This is where the edge goes into the vector
        edge_container_all_types.push_back(e12);

        e12->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id1)));
        e12->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(0)));
        e12->setMeasurement(obs1);
        e12->setInformation(Eigen::Matrix2d::Identity()*inv_sigma2);

        RobustKernelHuber * rk1 = new RobustKernelHuber();
        e12->setRobustKernel(rk1);
        rk1->setDelta(huber_delta);

        optimizer->addEdge(e12);

        // Set edge x2 = S21*X1
        Eigen::Matrix<double,2,1> obs2;
        obs2 << kp_pt_x_2, kp_pt_y_2;

        EdgeInverseSim3ProjectXYZ* e21 = new EdgeInverseSim3ProjectXYZ();

        // This is where the edge goes into the vector
        edge_container_all_types.push_back(e12);

        e21->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id2)));
        e21->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(0)));
        e21->setMeasurement(obs2);
        e21->setInformation(Eigen::Matrix2d::Identity()*inv_sigma2);

        RobustKernelHuber * rk2 = new RobustKernelHuber();
        e21->setRobustKernel(rk2);
        rk2->setDelta(huber_delta);

        optimizer->addEdge(e21);

        // Note: see explanation under get_mut_edges in lib.rs for why we do this
        RustSim3Edge rust_edge;
        unique_ptr<EdgeSim3ProjectXYZ> edge1(e12);
        unique_ptr<EdgeInverseSim3ProjectXYZ> edge2(e21);

        rust_edge.edge1 = std::move(edge1);
        rust_edge.edge2 = std::move(edge2);
        this->sim3_edges.emplace(this->sim3_edges.end(), std::move(rust_edge));
    }

    rust::Vec<int> BridgeSparseOptimizer::remove_sim3_edges_with_chi2 (float chi2_threshold) {

        std::cout << "The remove_sim3_edges_with_chi2 function is being called" << std::endl;
        rust::vec<int> * deleted_indexes = new rust::Vec<int>();
        for (int i = 0; i < this->sim3_edges.size(); i++) {
            if (!this->sim3_edges[i].edge1 && !this->sim3_edges[i].edge2) {
                continue;
            }
            if (this->sim3_edges[i].edge1->chi2() > chi2_threshold || this->sim3_edges[i].edge2->chi2() > chi2_threshold) {
                optimizer->removeEdge(this->sim3_edges[i].edge1.get());
                optimizer->removeEdge(this->sim3_edges[i].edge2.get());

                // After removing from graph, set sim3_edges element to null
                RustSim3Edge null_rust_edge;
                null_rust_edge.edge1 = std::unique_ptr<EdgeSim3ProjectXYZ>(nullptr);
                null_rust_edge.edge2 = std::unique_ptr<EdgeInverseSim3ProjectXYZ>(nullptr);
                this->sim3_edges[i] = std::move(null_rust_edge);
                deleted_indexes->push_back(i);
            }
        }
        return *deleted_indexes;
    }

    RustSim3 BridgeSparseOptimizer::recover_optimized_sim3() const {
        g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer->vertex(0));

        g2o::Sim3 g2oS12= vSim3_recov->estimate();

        Vector3d translation = g2oS12.translation();
        Quaterniond rotation = g2oS12.rotation();
        double scale = g2oS12.scale();

        RustSim3 format_sim3;
        format_sim3.translation = {
            (double) translation[0],
            (double) translation[1],
            (double) translation[2]
        };
        format_sim3.rotation = {
            (double) rotation.w(), 
            (double) rotation.x(),
            (double) rotation.y(),
            (double) rotation.z()
        };
        format_sim3.scale = scale;
        return format_sim3;
    }


    //** Optimization *//
    void BridgeSparseOptimizer::optimize(int iterations, bool online) {
        optimizer->initializeOptimization();
        optimizer->optimize(iterations, online);
    }

    Pose BridgeSparseOptimizer::recover_optimized_frame_pose(int vertex_id) const {
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer->vertex(vertex_id));

        // const VertexSE3Expmap* v = dynamic_cast<const VertexSE3Expmap*>(optimizer->vertex(vertex_id));

        g2o::SE3Quat SE3quat = vSE3->estimate();
        Vector3d translation = SE3quat.translation();
        Quaterniond rotation = SE3quat.rotation();

        Pose pose;
        pose.translation = {
            (double) translation[0],
            (double) translation[1],
            (double) translation[2]
        };
        pose.rotation = {
            (double) rotation.w(), 
            (double) rotation.x(),
            (double) rotation.y(),
            (double) rotation.z()
        };
        return pose;
    }

    Position BridgeSparseOptimizer::recover_optimized_mappoint_pose(int vertex_id) const {
        // std::cout<< "recover_optimized_mappoint_pose" << std::endl;
        VertexSBAPointXYZ* v = static_cast<VertexSBAPointXYZ*>(optimizer->vertex(vertex_id));

        Eigen::Vector3f pos = v->estimate().cast<float>();

        Position position;
        position.translation = {
            (double) pos.x(),
            (double) pos.y(),
            (double) pos.z()
        };
        return position;
    }

     BridgeSparseOptimizer::~BridgeSparseOptimizer() {
 
    /****************************************************************************LEAK FIX*****************************************************************************************************************
    *
    * The sparse optimizer object inside this object stores the edges of a graph. In C++ land, things are fine, when this object is no longer required, we can call delete on the sparse optimizer. This w
    * will delete all the edges for the sparse optimizer, the edges are stored within a vector. However things take a turn when we use ffi bindings to pass unique pointers from Cpp to Rust. We require ** Rust to have access to the above mentioned edges.
    *
    * As a result they will have to be wrapped up in unique pointers. Why does this pose an issue? When we attempt to delete the BridgeSparseOptimizer, there are going to be two attempts to delete the
    * same edges. One from the normal destructor within the library and another when the vector of edges belonging to the BridgeSparseOptimizer object gets deleted when the BridgeSparseObject destructor* gets called. To remedy this, the unique pointers let go of their allegiance to their individual pointers so that the normal destructor calls just delete the edges. This is instrumented with releas* e().  
    * 
    *****************************************************************************************************************************************************************************************************/
         for(auto it = xyz_onlypose_edges.begin(); it != xyz_onlypose_edges.end(); it++)
         {
            it->inner.release();
         }
        
         for(auto it = xyz_edges.begin(); it != xyz_edges.end(); it++)
         {
            it->inner.release();
         }

       for(auto it = sim3_edges.begin(); it != sim3_edges.end(); it++)
       {
          it->edge1.release();
          it->edge2.release();
       }

          xyz_onlypose_edges.clear();
          xyz_edges.clear();
          sim3_edges.clear();

         delete optimizer;
     }

} // end namespace
