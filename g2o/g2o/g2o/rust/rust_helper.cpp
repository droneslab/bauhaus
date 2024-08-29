
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
#include "/home/f1tenth/Downloads/Darvis_Git_Latest/darvis/darvis/target/cxxbridge/g2o/src/lib.rs.h"
#include "../core/hyper_graph.h"
#include "../orbslam_types/Pinhole.h"
#include "../types/G2oTypes.h"


namespace g2o {
    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer(int opt_type, std::array<double,4> camera_param, float lambda_init) {
        // Note: send 0 as lambda_init if you don't want to set a specific one
        BridgeSparseOptimizer * optimizer = new BridgeSparseOptimizer(opt_type, camera_param, lambda_init);
        unique_ptr<BridgeSparseOptimizer> ptr(optimizer);
        return ptr;
    }

    BridgeSparseOptimizer::BridgeSparseOptimizer(int opt_type, std::array<double,4> camera_param, float lambda_init) {
        this->xyz_edges = std::vector<RustXYZEdge>();
        this->xyz_onlypose_edges = std::vector<RustXYZOnlyPoseEdge>();
        this->sim3_projxyz_edges = std::vector<RustSim3ProjectXYZEdge>();
        this->sim3_edges = std::vector<RustSim3Edge>();

        if (opt_type == 1) {
            // For GlobalBundleAdjustemnt
            optimizer = new SparseOptimizer();
            BlockSolver_6_3::LinearSolverType * linearSolver = new LinearSolverEigen<BlockSolver_6_3::PoseMatrixType>();
            BlockSolver_6_3* solver_ptr = new BlockSolver_6_3(linearSolver);
            OptimizationAlgorithmLevenberg* solver = new OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer->setAlgorithm(solver);
            optimizer->setVerbose(false);

            if (lambda_init > 0) {
                solver->setUserLambdaInit(lambda_init);
            }

            optimizer_type = 1;
        } else if (opt_type == 2) {
            // For PoseOptimization
            optimizer = new SparseOptimizer();
            g2o::BlockSolver_6_3::LinearSolverType * linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();
            g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer->setAlgorithm(solver);

            if (lambda_init > 0) {
                solver->setUserLambdaInit(lambda_init);
            }

            optimizer_type = 2;
        } else if (opt_type == 3) {
            // For OptimizeSim3
            optimizer = new SparseOptimizer();
            BlockSolverX::LinearSolverType * linearSolver = new LinearSolverDense<BlockSolverX::PoseMatrixType>();
            BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);

            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer->setVerbose(false);
            optimizer->setAlgorithm(solver);

            if (lambda_init > 0) {
                solver->setUserLambdaInit(lambda_init);
            }

            optimizer_type = 3;
        } else if (opt_type == 4) {
            // For OptimizeEssentialGraph
            optimizer = new SparseOptimizer();

            g2o::BlockSolver_7_3::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_7_3::PoseMatrixType>();
            g2o::BlockSolver_7_3 * solver_ptr= new g2o::BlockSolver_7_3(linearSolver);

            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
            optimizer->setVerbose(false);
            optimizer->setAlgorithm(solver);

            if (lambda_init > 0) {
                solver->setUserLambdaInit(lambda_init);
            }

            optimizer_type = 4;
        } else if (opt_type == 5) {
            optimizer = new SparseOptimizer();
            g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

            g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

            g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

            if (lambda_init > 0) {
                solver->setUserLambdaInit(lambda_init);
            }

            optimizer->setAlgorithm(solver);
            optimizer_type = 5;
        } else if (opt_type == 6) {
            optimizer = new SparseOptimizer();
            g2o::BlockSolverX::LinearSolverType * linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>();

            g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);

            g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);

            optimizer->setAlgorithm(solver);
            optimizer_type = 6;
        } else {
            // For Optimizer::PoseInertialOptimizationLastFrame, Optimizer::PoseInertialOptimizationLastKeyFrame,
            optimizer = new SparseOptimizer();
            BlockSolverX::LinearSolverType * linearSolver = new LinearSolverDense<BlockSolverX::PoseMatrixType>();
            BlockSolverX * solver_ptr = new BlockSolverX(linearSolver);

            OptimizationAlgorithmGaussNewton* solver = new OptimizationAlgorithmGaussNewton(solver_ptr);
            optimizer->setVerbose(false);
            optimizer->setAlgorithm(solver);

            optimizer_type = 7;
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
 
    // void BridgeSparseOptimizer::enable_stop_flag() {
    //     // Allows the optimizer to be turned off with a pointer to a stop flag
    //     stopFlag = false;
    //     optimizer->setForceStopFlag(*stopFlag);
    // }

    // void BridgeSparseOptimizer::set_stop_flag(bool should_stop) {
    //     // If true, turns off optimizer
    //     stopFlag = should_stop;
    // }

    //* Vertices *//
    bool BridgeSparseOptimizer::has_vertex(int id) const {
        return optimizer->vertex(id) != NULL;
    }

    void BridgeSparseOptimizer::remove_vertex (int vertex_id) {
        optimizer->removeVertex(optimizer->vertex(vertex_id));
    }

    void BridgeSparseOptimizer::add_vertex_se3_expmap (
        int vertex_id,  Pose pose, bool set_fixed
    ) {
        if (optimizer_type == 1 || optimizer_type == 2) {
            VertexSE3Expmap * vSE3 = new VertexSE3Expmap();
            vSE3->setEstimate(this->format_pose(pose));
            // std::cout << "Set frame to " << vSE3->estimate() << std::endl;
            vSE3->setId(vertex_id);
            vSE3->setFixed(set_fixed);
            optimizer->addVertex(vSE3);
        } else if (optimizer_type == 3 || optimizer_type == 4) {
            std::cout << "WARNING: Use add_vertex_sim3_expmap instead of add_vertex_se3_expmap for OptimizeEssentialGraph or OptimizeSim3" << std::endl;
        } else {
            std::cout << "Error!! For inertial optimizations, call ``add_vertex_pose`` instead of ``add_vertex_se3_expmap`" << std::endl;
        }
    }

    void BridgeSparseOptimizer::add_vertex_pose (
        int vertex_id, bool set_fixed,
        int num_cams, 
        array<double, 3> imu_position, array<double, 4> imu_rotation,
        array<double, 3> translation, array<double, 4> rotation,
        array<double, 3> tcb_translation, array<double, 4> tcb_rotation,
        array<double, 3> tbc_translation,
        float bf
    ) {
        // Used by inertial optimizations

        vector<float> camera_calib{fx,fy,cx,cy};
        ORB_SLAM3::Pinhole * camera = new ORB_SLAM3::Pinhole(camera_calib);

        Eigen::Vector3d imu_position2(imu_position.data());
        Eigen::Matrix3d imu_rotation2 = Eigen::Quaterniond(imu_rotation.data()).toRotationMatrix();
        Eigen::Vector3d translation2(translation.data());
        Eigen::Matrix3d rotation2 = Eigen::Quaterniond(rotation.data()).toRotationMatrix();
        Eigen::Vector3d tcb_translation2(tcb_translation.data());
        Eigen::Matrix3d tcb_rotation2 = Eigen::Quaterniond(tcb_rotation.data()).toRotationMatrix();
        Eigen::Vector3d tbc_translation2(tbc_translation.data());


        g2o::VertexPose* VP = new g2o::VertexPose(
            1, camera,
            imu_position2, imu_rotation2,
            translation2, rotation2,
            tcb_translation2, tcb_rotation2,
            tbc_translation2,
            bf
        );

        VP->setId(vertex_id);
        VP->setFixed(set_fixed);
        optimizer->addVertex(VP);

        std::cout << "(pose) add vertex " << vertex_id << std::endl;
    }

    void BridgeSparseOptimizer::add_vertex_velocity(
        int vertex_id, bool set_fixed,
        array<double, 3> velocity
    ) {
        Eigen::Vector3d vel(velocity.data());
        g2o::VertexVelocity* VV = new g2o::VertexVelocity(vel);
        VV->setId(vertex_id);
        VV->setFixed(set_fixed);
        optimizer->addVertex(VV);
        std::cout << "(vel) add vertex " << vertex_id << std::endl;
    }

    void BridgeSparseOptimizer::add_vertex_gyro_bias(
        int vertex_id, bool set_fixed,
        array<double, 3> gyro_bias
    ) {
        Eigen::Vector3d bias(gyro_bias.data());
        g2o::VertexGyroBias* VG = new g2o::VertexGyroBias(bias);
        VG->setId(vertex_id);
        VG->setFixed(set_fixed);
        optimizer->addVertex(VG);
        std::cout << "(gyro) add vertex " << vertex_id << std::endl;
    }

    void BridgeSparseOptimizer::add_vertex_acc_bias(
        int vertex_id, bool set_fixed,
        array<double, 3> acc_bias
    ) {
        Eigen::Vector3d bias(acc_bias.data());
        g2o::VertexAccBias* VA = new g2o::VertexAccBias(bias);
        VA->setId(vertex_id);
        VA->setFixed(set_fixed);
        optimizer->addVertex(VA);
        std::cout << "(acc) add vertex " << vertex_id << std::endl;
    }

    void BridgeSparseOptimizer::add_gravity_and_scale_vertex(
        int vertex_id1, bool set_fixed1, array<array<double, 3>, 3> rwg,
        int vertex_id2, bool set_fixed2, double scale
    ) {
        Eigen::Matrix3d rwg_eig;
        rwg_eig << rwg[0][0], rwg[0][1], rwg[0][2],
                   rwg[1][0], rwg[1][1], rwg[1][2],
                   rwg[2][0], rwg[2][1], rwg[2][2];
        g2o::VertexGDir* VGDir = new g2o::VertexGDir(rwg_eig);
        VGDir->setId(vertex_id1);
        VGDir->setFixed(set_fixed1);
        optimizer->addVertex(VGDir);

        g2o::VertexScale* VS = new g2o::VertexScale(scale);
        VS->setId(vertex_id2);
        VS->setFixed(set_fixed2);
        optimizer->addVertex(VS);
        std::cout << "(gravity) add vertex " << vertex_id1 << std::endl;
        std::cout << "(scale) add vertex " << vertex_id2 << std::endl;

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
        // std::cout << "set frame vertex pose to: "<< SE3Quat(rot_quat, trans_vec) << std::endl;
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
    std::vector<RustXYZOnlyPoseEdge>& BridgeSparseOptimizer::get_mut_xyz_onlypose_edges() {
        return this->xyz_onlypose_edges;
    }
    std::vector<RustXYZEdge>& BridgeSparseOptimizer::get_mut_xyz_edges() {
        return this->xyz_edges;
    }
    std::vector<RustSim3ProjectXYZEdge>& BridgeSparseOptimizer::get_mut_sim3_edges() {
        return this->sim3_projxyz_edges;
    }
    // std::vector<RustEdgeMono>& BridgeSparseOptimizer::get_mut_mono_edges() {
    //     return this->mono_edges;
    // }

    void BridgeSparseOptimizer::add_edge_se3_project_xyz_monocular_unary(
        bool robust_kernel, int vertex_id,
        int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float inv_sigma2,
        array<double, 3> mp_world_position,
        int mappoint_id,
        float huber_delta
    ) {
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        EdgeSE3ProjectXYZOnlyPose * edge = new EdgeSE3ProjectXYZOnlyPose();
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

    void BridgeSparseOptimizer::add_edge_se3_project_xyz_monocular_binary(
        bool robust_kernel, int vertex1, int vertex2, int mp_id,
        float keypoint_pt_x, float keypoint_pt_y, float inv_sigma2,
        float huber_delta
    ) {
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        EdgeSE3ProjectXYZ * edge = new EdgeSE3ProjectXYZ();
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
        RustXYZEdge rust_edge;
        rust_edge.inner = std::move(ptr_edge);
        rust_edge.mappoint_id = mp_id;
        this->xyz_edges.emplace(this->xyz_edges.end(), std::move(rust_edge));
    }

    void BridgeSparseOptimizer::add_edge_mono_binary(
        bool robust_kernel, int vertex1, int vertex2, int mp_id,
        float keypoint_pt_x, float keypoint_pt_y, float inv_sigma2,
        float huber_delta
    ) {
        Eigen::Matrix<double,2,1> obs;
        obs << keypoint_pt_x, keypoint_pt_y;

        g2o::EdgeMono * edge = new g2o::EdgeMono(0);
        edge->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex1)));
        edge->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex2)));
        edge->setMeasurement(obs);
        edge->setInformation(Eigen::Matrix2d::Identity()*inv_sigma2);

        if (robust_kernel) {
            RobustKernelHuber * rk = new RobustKernelHuber();
            edge->setRobustKernel(rk);
            rk->setDelta(huber_delta);
        }

        auto success = optimizer->addEdge(edge);
        if (!success) {
            std::cout << "Optimizer failed to add edge!!" << std::endl;
        }
        // // Note: see explanation under get_mut_edges in lib.rs for why we do this
        // unique_ptr<EdgeMono> ptr_edge(edge);
        // RustEdgeMono rust_edge;
        // rust_edge.inner = std::move(ptr_edge);
        // rust_edge.mappoint_id = mp_id;
        // this->mono_edges.emplace(this->mono_edges.end(), std::move(rust_edge));
    }


    void BridgeSparseOptimizer::add_edge_prior_for_imu(
        int vertex_id1, int vertex_id2,
        array<double, 3> bprior,
        double priorA, double priorG
    ) {
        Eigen::Vector3d bprior_eig(bprior.data());
        g2o::EdgePriorAcc* epa = new g2o::EdgePriorAcc(bprior_eig);
        epa->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id1)));
        double infoPriorA = priorA;
        epa->setInformation(infoPriorA*Eigen::Matrix3d::Identity());
        optimizer->addEdge(epa);
        g2o::EdgePriorGyro* epg = new g2o::EdgePriorGyro(bprior_eig);
        epg->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id2)));
        double infoPriorG = priorG;
        epg->setInformation(infoPriorG*Eigen::Matrix3d::Identity());
        optimizer->addEdge(epg);
    }

    void BridgeSparseOptimizer::add_graph_edges_inertial(
        int vertex_P1_id, int vertex_V1_id,
        int vertex_G_id, int vertex_A_id, int vertex_P2_id,
        int vertex_V2_id, int vertex_GDir_id, int vertex_S_id,
        RustImuPreintegrated imu_preintegrated,
        bool set_robust_kernel, float delta
    ) {
        g2o::IMU::Preintegrated* pre_ = new g2o::IMU::Preintegrated(& imu_preintegrated);
        g2o::EdgeInertialGS* ei = new g2o::EdgeInertialGS(pre_);
        ei->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_P1_id)));
        ei->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_V1_id)));
        ei->setVertex(2,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_G_id)));
        ei->setVertex(3,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_A_id)));
        ei->setVertex(4,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_P2_id)));
        ei->setVertex(5,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_V2_id)));

        if (vertex_GDir_id != -1) {
            ei->setVertex(6,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_GDir_id)));
            std::cout << "(special gdir) edge " << ei->id() << " added vertex " << vertex_GDir_id << "..." << (optimizer->vertex(vertex_GDir_id) == NULL) << std::endl;

        }
        if (vertex_S_id != -1) {
            ei->setVertex(7,dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer->vertex(vertex_S_id)));
            std::cout << "(special s) edge " << ei->id() << " added vertex " << vertex_S_id << "..." << (optimizer->vertex(vertex_S_id) == NULL) << std::endl;

        }

        if (set_robust_kernel) {
            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            ei->setRobustKernel(rk);
            rk->setDelta(delta);
        }
        ei->setId(optimizer->edges().size());
        std::cout << "THIS WAS EDGE " << ei->id() << std::endl;

        std::cout << "edge " << ei->id() << " added vertex " << vertex_P1_id << "..." << (optimizer->vertex(vertex_P1_id) == NULL) << std::endl;
        std::cout << "edge " << ei->id() << " added vertex " << vertex_V1_id << "..." << (optimizer->vertex(vertex_V1_id) == NULL) << std::endl;
        std::cout << "edge " << ei->id() << " added vertex " << vertex_G_id << "..." << (optimizer->vertex(vertex_G_id) == NULL) << std::endl;
        std::cout << "edge " << ei->id() << " added vertex " << vertex_A_id << "..." << (optimizer->vertex(vertex_A_id) == NULL) << std::endl;
        std::cout << "edge " << ei->id() << " added vertex " << vertex_P2_id << "..." << (optimizer->vertex(vertex_P2_id) == NULL) << std::endl;
        std::cout << "edge " << ei->id() << " added vertex " << vertex_V2_id << "..." << (optimizer->vertex(vertex_V2_id) == NULL) << std::endl;

        optimizer->addEdge(ei);

    }

    // void BridgeSparseOptimizer::add_graph_edges_gyro_and_acc(
    //     int 
    //     RustImuPreintegrated imu_preintegrated,
    // ) {
    //     g2o::IMU::Preintegrated* pre_ = new g2o::IMU::Preintegrated(& imu_preintegrated);

    //     EdgeGyroRW* egr= new EdgeGyroRW();
    //     egr->setVertex(0,vertex_G_id);
    //     egr->setVertex(1,vertex_GDir_id);
    //     Eigen::Matrix3d InfoG = pre_->C.block<3,3>(9,9).cast<double>().inverse();
    //     egr->setInformation(InfoG);
    //     egr->computeError();
    //     optimizer.addEdge(egr);

    //     EdgeAccRW* ear = new EdgeAccRW();
    //     ear->setVertex(0,vertex_A_id);
    //     ear->setVertex(1,vertex_S_id);
    //     Eigen::Matrix3d InfoA = pre_->C.block<3,3>(12,12).cast<double>().inverse();
    //     ear->setInformation(InfoA);
    //     ear->computeError();
    //     optimizer.addEdge(ear);
    // }

    void BridgeSparseOptimizer::add_edge_gyro_and_acc(
        int vertex1_id, int vertex2_id,
        int vertex3_id, int vertex4_id,
        RustImuPreintegrated imu_preintegrated
    ) {
        g2o::IMU::Preintegrated* pre_ = new g2o::IMU::Preintegrated(& imu_preintegrated);

        EdgeGyroRW* egr= new EdgeGyroRW();
        egr->setVertex(0, optimizer->vertex(vertex1_id));
        egr->setVertex(1, optimizer->vertex(vertex2_id));
        Eigen::Matrix3d InfoG = pre_->C.block<3,3>(9,9).cast<double>().inverse();
        egr->setInformation(InfoG);
        egr->computeError();
        optimizer->addEdge(egr);

        EdgeAccRW* ear = new EdgeAccRW();
        ear->setVertex(0, optimizer->vertex(vertex3_id));
        ear->setVertex(1, optimizer->vertex(vertex4_id));
        Eigen::Matrix3d InfoA = pre_->C.block<3,3>(12,12).cast<double>().inverse();
        ear->setInformation(InfoA);
        ear->computeError();
        optimizer->addEdge(ear);

        std::cout << "(gyro) edge  added vertex " << vertex1_id << "..." << (optimizer->vertex(vertex1_id) == NULL) << std::endl;
        std::cout << "(gyro) edge  added vertex " << vertex2_id << "..." << (optimizer->vertex(vertex2_id) == NULL) << std::endl;

        std::cout << "(acc) edge  added vertex " << vertex3_id << "..." << (optimizer->vertex(vertex3_id) == NULL) << std::endl;
        std::cout << "(acc) edge  added vertex " << vertex4_id << "..." << (optimizer->vertex(vertex4_id) == NULL) << std::endl;


    }


    // ** SIM3 ** //
    // Functions similar to those above, just specific to sim3 optimization. Keeping them all in one place.
    void BridgeSparseOptimizer::add_vertex_sim3_expmap (
        int vertex_id, RustSim3 sim3, bool fix_scale, bool set_fixed, bool set_camera_params
    ) {
        g2o::VertexSim3Expmap * vSim3 = new g2o::VertexSim3Expmap();
        vSim3->setEstimate(this->format_sim3(sim3));
        vSim3->setId(vertex_id);
        vSim3->setFixed(set_fixed);
        vSim3->setMarginalized(false);
        vSim3->_fix_scale=fix_scale;

        if (set_camera_params) {
            vSim3->_principle_point1[0] = cx;
            vSim3->_principle_point1[1] = cy;
            vSim3->_focal_length1[0] = fx;
            vSim3->_focal_length1[1] = fy;
            vSim3->_principle_point2[0] = cx;
            vSim3->_principle_point2[1] = cy;
            vSim3->_focal_length2[0] = fx;
            vSim3->_focal_length2[1] = fy;
        }
        else
        {
            vSim3->_principle_point1[0] = 0;
            vSim3->_principle_point1[1] = 0;
            vSim3->_focal_length1[0] = 0;
            vSim3->_focal_length1[1] = 0;
            vSim3->_principle_point2[0] = 0;
            vSim3->_principle_point2[1] = 0;
            vSim3->_focal_length2[0] = 0;
            vSim3->_focal_length2[1] = 0; 
        }
        optimizer->addVertex(vSim3);
    }

    Sim3 BridgeSparseOptimizer::format_sim3(RustSim3 sim3) const {
        Eigen::Vector3d trans_vec(sim3.translation.data());
        auto rot_quat_val = sim3.rotation.data();
        Eigen::Quaterniond rot_quat(rot_quat_val[0], rot_quat_val[1], rot_quat_val[2],rot_quat_val[3]);
        return Sim3(rot_quat, trans_vec, sim3.scale);
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
        e21->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id2)));
        e21->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(0)));
        e21->setMeasurement(obs2);
        e21->setInformation(Eigen::Matrix2d::Identity()*inv_sigma2);

        RobustKernelHuber * rk2 = new RobustKernelHuber();
        e21->setRobustKernel(rk2);
        rk2->setDelta(huber_delta);

        optimizer->addEdge(e21);

        // Note: see explanation under get_mut_edges in lib.rs for why we do this
        RustSim3ProjectXYZEdge rust_edge;
        unique_ptr<EdgeSim3ProjectXYZ> edge1(e12);
        unique_ptr<EdgeInverseSim3ProjectXYZ> edge2(e21);

        rust_edge.edge1 = std::move(edge1);
        rust_edge.edge2 = std::move(edge2);
        this->sim3_projxyz_edges.emplace(this->sim3_projxyz_edges.end(), std::move(rust_edge));
    }

    void BridgeSparseOptimizer::add_one_sim3_edge(
        int vertex_id_i, int vertex_id_j,
        RustSim3 observation
    ) {
        const Eigen::Matrix<double,7,7> matLambda = Eigen::Matrix<double,7,7>::Identity();

        EdgeSim3* e = new EdgeSim3();
        e->setVertex(0, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id_i)));
        e->setVertex(1, dynamic_cast<OptimizableGraph::Vertex*>(optimizer->vertex(vertex_id_j)));
        e->setMeasurement(this->format_sim3(observation));
        e->information() = matLambda;

        // std::cout << "Sofiya essential graph Add edge " << vertex_id_i << "->" << vertex_id_j << " with observation: " << this->format_sim3(observation) << std::endl;

        optimizer->addEdge(e);

        // Note: see explanation under get_mut_edges in lib.rs for why we do this
        RustSim3Edge rust_edge;
        unique_ptr<EdgeSim3> edge(e);
        rust_edge.edge = std::move(edge);
        this->sim3_edges.emplace(this->sim3_edges.end(), std::move(rust_edge));
    }

    rust::Vec<int> BridgeSparseOptimizer::remove_sim3_edges_with_chi2 (float chi2_threshold) {
        rust::vec<int> deleted_indexes;
        for (int i = 0; i < this->sim3_projxyz_edges.size(); i++) {
            if (!this->sim3_projxyz_edges[i].edge1 && !this->sim3_projxyz_edges[i].edge2) {
                continue;
            }
            if (this->sim3_projxyz_edges[i].edge1->chi2() > chi2_threshold || this->sim3_projxyz_edges[i].edge2->chi2() > chi2_threshold) {
                optimizer->removeEdge(this->sim3_projxyz_edges[i].edge1.release());
                optimizer->removeEdge(this->sim3_projxyz_edges[i].edge2.release());

                // After removing from graph, set sim3_edges element to null
                RustSim3ProjectXYZEdge null_rust_edge;
                null_rust_edge.edge1 = std::unique_ptr<EdgeSim3ProjectXYZ>(nullptr);
                null_rust_edge.edge2 = std::unique_ptr<EdgeInverseSim3ProjectXYZ>(nullptr);
                this->sim3_projxyz_edges[i] = std::move(null_rust_edge);
                deleted_indexes.push_back(i);
            }
        }
        return deleted_indexes;
    }

    RustSim3 BridgeSparseOptimizer::recover_optimized_sim3(int vertex_id) const {
        g2o::VertexSim3Expmap* vSim3_recov = static_cast<g2o::VertexSim3Expmap*>(optimizer->vertex(vertex_id));

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

    void BridgeSparseOptimizer::save(rust::Str filename, int save_id) const
    {   
        bool is_saved = optimizer->save((std::to_string(save_id)+"_"+filename.data()).c_str()); 


    }
    //** Optimization *//
    void BridgeSparseOptimizer::optimize(int iterations, bool online, bool compute_active_errors) {
        optimizer->initializeOptimization();

        if (compute_active_errors) {
            optimizer->computeActiveErrors();
        }

        optimizer->optimize(iterations, online);

        if (compute_active_errors) {
            optimizer->computeActiveErrors();
        }
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

        // std::cout << "Optimized pose in c++: t " << SE3quat.translation().transpose() << " r " << SE3quat.rotation().vec().transpose() << std::endl;

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

    InertialEstimate BridgeSparseOptimizer::recover_optimized_inertial(int vg, int va, int vs, int vgdir) const {
        // Sometimes we don't care about getting the values associated with a subset of these, so in those cases we pass -1

        Vector6d vb;
        Eigen::Vector3d bg;
        Eigen::Vector3d ba;
        if (vg != -1 && va != -1) {
            g2o::VertexGyroBias* VG = static_cast<g2o::VertexGyroBias*>(optimizer->vertex(vg));
            g2o::VertexAccBias* VA = static_cast<g2o::VertexAccBias*>(optimizer->vertex(va));
            ba << VA->estimate();
            vb << VG->estimate(), VA->estimate();
            bg << VG->estimate();
        } else {
            vb << 0, 0, 0, 0, 0, 0;
            bg << 0, 0, 0;
        }

        double scale;
        Eigen::Matrix3d Rwg;
        if (vs != -1 && vgdir != -1) {
            g2o::VertexScale* VS = static_cast<g2o::VertexScale*>(optimizer->vertex(vs));
            g2o::VertexGDir* VGDir = static_cast<g2o::VertexGDir*>(optimizer->vertex(vgdir));
            scale = VS->estimate();
            Rwg = VGDir->estimate().Rwg;
        } else {
            scale = 0.0;
            Rwg = Eigen::Matrix3d::Identity();
        }


        InertialEstimate estimate;
        estimate.vb = {vb[0], vb[1], vb[2], vb[3], vb[4], vb[5]};
        estimate.bg = {bg[0], bg[1], bg[2]};
        estimate.ba = {ba[0], ba[1], ba[2]};
        estimate.scale = scale;
        estimate.rwg = {
            Rwg(0,0), Rwg(0,1), Rwg(0,2),
            Rwg(1,0), Rwg(1,1), Rwg(1,2),
            Rwg(2,0), Rwg(2,1), Rwg(2,2)
        };
        return estimate;
    }

    array<double, 3> BridgeSparseOptimizer::recover_optimized_vertex_velocity(int vertex_id) const {
        g2o::VertexVelocity* VV = static_cast<g2o::VertexVelocity*>(optimizer->vertex(vertex_id));
        Eigen::Vector3d Vw = VV->estimate();
        return {Vw[0], Vw[1], Vw[2]};
    }

    BridgeSparseOptimizer::~BridgeSparseOptimizer() {
        // Drop pointers to edges, these are already deleted when the optimizer is deleted
        for(auto it = xyz_onlypose_edges.begin(); it != xyz_onlypose_edges.end(); it++) {
            it->inner.release();
        }
        for(auto it = xyz_edges.begin(); it != xyz_edges.end(); it++) {
            it->inner.release();
        }
        for(auto it = sim3_projxyz_edges.begin(); it != sim3_projxyz_edges.end(); it++) {
            it->edge1.release();
            it->edge2.release();
        }
        for(auto it = sim3_edges.begin(); it != sim3_edges.end(); it++) {
            it->edge.release();
        }
        // for(auto it = mono_edges.begin(); it != mono_edges.end(); it++) {
        //     it->edge.release();
        // }
        
        xyz_onlypose_edges.clear();
        xyz_edges.clear();
        sim3_projxyz_edges.clear();
        sim3_edges.clear();
        // mono_edges.clear();
        
        delete optimizer;
    }

} // end namespace
