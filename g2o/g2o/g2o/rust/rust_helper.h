#ifndef RUSTHELPER_H
#define RUSTHELPER_H

#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../core/base_unary_edge.h"
#include "../core/base_binary_edge.h"
#include "../solvers/linear_solver_dense.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../types/types_six_dof_expmap.h"
#include "../types/types_seven_dof_expmap.h"
#include "../types/sim3.h"
#include "../types/G2oTypes.h"

#include "rust/cxx.h"

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT


namespace g2o {
    // struct EdgeMono;
    // using Edge = OptimizableGraph::Edge;
    struct Pose;
    struct Position;
    struct RustSim3;
    struct RustXYZEdge;
    struct RustEdgeMono;
    struct RustXYZOnlyPoseEdge;
    struct RustSim3ProjectXYZEdge;
    struct RustSim3Edge;
    struct RustImuPreintegrated;
    struct InertialEstimate;
    struct RustEdgeMonoOnlyPose;
    struct RustEdgeInertial;
    enum class VertexPoseRecoverType : std::uint8_t;

    class BridgeSparseOptimizer
    {
        public :
            BridgeSparseOptimizer(int opt_type, std::array<double, 4> camera_param, float lambda_init);
        ~BridgeSparseOptimizer();

        // vertices
        bool has_vertex(int id) const;
        void remove_vertex(int vertex_id);
        void add_vertex_sim3expmap(int vertex_id, RustSim3 sim3, bool fix_scale, bool set_fixed, bool set_camera_params);
        void add_vertex_se3expmap(int vertex_id, Pose pose, bool set_fixed);
        void update_estimate_vertex_se3xpmap(int vertex_id, Pose pose);
        void add_vertex_pose(int vertex_id, bool set_fixed, int num_cams,
                             array<double, 3> imu_position, array<array<double, 3>, 3> imu_rotation,
                             array<double, 3> translation, array<array<double, 3>, 3> rotation,
                             array<double, 3> tcb_translation, array<array<double, 3>, 3> tcb_rotation,
                             array<double, 3> tbc_translation,
                             float bf);
        void add_vertex_velocity(int vertex_id, bool set_fixed, array<double, 3> velocity);
        void add_vertex_gyrobias(int vertex_id, bool set_fixed, array<double, 3> gyro_bias);
        void add_vertex_accbias(int vertex_id, bool set_fixed, array<double, 3> acc_bias);
        void add_vertex_sbapointxyz(int vertex_id, Pose pose, bool set_fixed, bool set_marginalized);
        void add_vertex_gdir(
            int vertex_id, bool set_fixed, array<array<double, 3>, 3> rwg);
        void add_vertex_scale(
            int vertex_id, bool set_fixed, double scale);

        // edges
        void add_both_sim_edges(
            int vertex_id1, float kp_pt_x_1, float kp_pt_y_1, float inv_sigma1,
            int vertex_id2, float kp_pt_x_2, float kp_pt_y_2, float inv_sigma2,
            float huber_delta);
        void add_one_sim3_edge(int vertex_id_i, int vertex_id_j, RustSim3 observation);
        void add_edge_se3_project_xyz_monocular_unary(
            bool robust_kernel, int vertex_id,
            int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float inv_sigma2,
            array<double, 3> mp_world_position,
            int mappoint_id,
            float huber_delta);
        void add_edge_se3_project_xyz_monocular_binary(
            bool robust_kernel, int vertex1, int vertex2, int mp_id,
            float keypoint_pt_x, float keypoint_pt_y, float inv_sigma2,
            float huber_delta);
        void add_edge_mono_binary(
            bool robust_kernel, int vertex1, int vertex2, int mp_id,
            float keypoint_pt_x, float keypoint_pt_y, float inv_sigma2,
            float huber_delta);
        void add_edge_mono_only_pose(
            bool robust_kernel,
            int vertex, int mp_id,
            array<double, 3> mp_world_position,
            float keypoint_pt_x, float keypoint_pt_y,
            float inv_sigma2, float huber_delta);

        void add_edge_prior_for_imu(int vertex_id1, int vertex_id2, array<double, 3> bprior, double priorA, double priorG);
        void add_edge_inertial_gs(
            int vertex_P1_id, int vertex_V1_id,
            int vertex_G_id, int vertex_A_id, int vertex_P2_id,
            int vertex_V2_id, int vertex_GDir_id, int vertex_S_id,
            RustImuPreintegrated imu_preintegrated,
            bool set_robust_kernel, float delta,
            bool set_information, float information_weight);
        void add_edge_inertial(
            int vertex_P1_id, int vertex_V1_id,
            int vertex_G_id, int vertex_A_id, int vertex_P2_id,
            int vertex_V2_id,
            RustImuPreintegrated imu_preintegrated,
            bool set_robust_kernel, float delta);
        void add_edge_prior_pose_imu(
            int vertex_id1, int vertex_id2, int vertex_id3, int vertex_id4,
            array<array<double, 3>, 3> Rwb, array<double, 3> twb, array<double, 3> vwb,
            array<double, 3> bg, array<double, 3> ba,
            array<array<double, 15>, 15> H,
            bool set_robust_kernel, float delta);
        void add_edge_gyro_and_acc(
            int vertex1_id, int vertex2_id,
            int vertex3_id, int vertex4_id,
            RustImuPreintegrated imu_preintegrated,
            bool compute_error
        );

        void _add_edge_stereo() const;
        int num_edges() const;
        rust::Vec<int>
            remove_sim3_edges_with_chi2(float chi2_threshold);

        // optimization
        void optimize(int iterations, bool online, bool compute_active_errors);
        Pose recover_optimized_frame_pose(int vertex_id) const;
        Pose recover_optimized_vertex_pose(int vertex_id, VertexPoseRecoverType recover_type) const;
        Position recover_optimized_mappoint_pose(int vertex_id) const;
        void recover_optimized_sim3(int vertex_id, RustSim3 & sim3) const;
        InertialEstimate recover_optimized_inertial(int vg, int va, int vs, int vgdir) const;
        array<double, 3>
            recover_optimized_vertex_velocity(int vertex_id) const;

        void print_optimized_vertex_pose(int vertex_id, VertexPoseRecoverType recover_type) const;
        
        array<array<double, 24>, 24>
            get_hessian_from_edge_inertial(int edge_idx) const;
        array<array<double, 9>, 9>
            get_hessian2_from_edge_inertial(int edge_idx) const;
        array<array<double, 6>, 6>
            get_hessian_from_edge_gyro() const;
        array<array<double, 3>, 3>
            get_hessian2_from_edge_gyro() const;
        array<array<double, 6>, 6>
            get_hessian_from_edge_acc() const;
        array<array<double, 3>, 3>
            get_hessian2_from_edge_acc() const;
        array<array<double, 15>, 15>
            get_hessian_from_edge_prior() const;

        void save(rust::Str filename, int save_id) const;

        // Note: see explanation under get_mut_edges in lib.rs for why we do this
        std::vector<RustXYZEdge>
            xyz_edges;
        std::vector<RustXYZOnlyPoseEdge>
            xyz_onlypose_edges;
        std::vector<RustSim3ProjectXYZEdge>
            sim3_projxyz_edges;
        std::vector<RustSim3Edge>
            sim3_edges;
        std::vector<RustEdgeMono>
            mono_edges;
        std::vector<RustEdgeMonoOnlyPose>
            mono_onlypose_edges;
        std::vector<RustEdgeInertial>
            inertial_edges;
        EdgeGyroRW * gyro_edge;
        EdgeAccRW * acc_edge;
        EdgePriorPoseImu * edge_prior_pose_imu;
        std::vector<RustXYZEdge> & get_mut_xyz_edges();
        std::vector<RustXYZOnlyPoseEdge> & get_mut_xyz_onlypose_edges();
        std::vector<RustSim3ProjectXYZEdge> & get_mut_sim3_edges();
        std::vector<RustEdgeMono> & get_mut_mono_edges();
        std::vector<RustEdgeMonoOnlyPose> & get_mut_mono_onlypose_edges();

        // void enable_stop_flag();
        // void set_stop_flag(bool should_stop);

        void set_robust_kernel_for_edge(int edge_id, bool reset);

        private :
            SparseOptimizer * optimizer;
        int optimizer_type; // see constructor
        float deltaMono;
        float deltaStereo;
        float thHuberMono;
        float thHuberStereo;
        float thHuber2D;
        float thHuber3D;
        vector<size_t>
            vnIndexEdgeMono;
        bool stopFlag;

        SE3Quat format_pose(Pose pose) const;
        Sim3 format_sim3(RustSim3 sim3) const;

        // Camera Parameters for Optimization
        float fx,
        fy,
        cx,
        cy;
        public :
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    // Note: send 0 as lambda_init if you don't want to set a specific one
    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer(int opt_type, std::array<double,4> camera_param, float lambda_init);
} // end namespace

#endif