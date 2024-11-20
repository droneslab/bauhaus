#[cxx::bridge(namespace = "g2o")]
pub mod ffi {
    // Shared structs with fields visible to both languages.

    //* Pose types */
    struct Pose {
        translation: [f64; 3], // in C++: array<double, 3>,
        rotation: [f64; 4] // in C++: array<double, 4> 
    }
    struct Position {
        translation: [f64; 3],
    }
    struct RustSim3 {
        translation: [f64; 3],
        rotation: [f64; 4],
        scale: f64,
    }

    //* Edges */
    // Note: Workaround to have a vec of shared ptrs 
    // https://github.com/dtolnay/cxx/issues/741
    // Not thread safe!! Don't make this shared.
    // See explanation below for get_mut_xyz_edges for why we do this.
    struct RustXYZEdge {
        inner: UniquePtr<EdgeSE3ProjectXYZ>,
        mappoint_id: i32,
    }
    struct RustXYZOnlyPoseEdge {
        inner: UniquePtr<EdgeSE3ProjectXYZOnlyPose>,
        mappoint_id: i32,
    }
    struct RustSim3ProjectXYZEdge {
        edge1: UniquePtr<EdgeSim3ProjectXYZ>,
        edge2: UniquePtr<EdgeInverseSim3ProjectXYZ>,
    }
    struct RustSim3Edge {
        edge: UniquePtr<EdgeSim3>,
    }
    struct RustEdgeMono {
        inner: UniquePtr<EdgeMono>,
        mappoint_id: i32,
    }
    struct RustEdgeMonoOnlyPose {
        inner: UniquePtr<EdgeMonoOnlyPose>,
        mappoint_id: i32,
    }
    struct RustEdgeInertial {
        inner: UniquePtr<EdgeInertial>,
    }

    //* Inertial types *//
    struct RustImuBias {
        b_acc_x: f32,
        b_acc_y: f32,
        b_acc_z: f32,
        b_ang_vel_x: f32,
        b_ang_vel_y: f32,
        b_ang_vel_z: f32,
    }
    struct RustImuPreintegrated {
        jrg: [[f32; 3]; 3],
        jvg: [[f32; 3]; 3],
        jpg: [[f32; 3]; 3],
        jva: [[f32; 3]; 3],
        jpa: [[f32; 3]; 3],

        dv: [f32; 3],
        db: [f32; 6],
        dr: [[f32; 3]; 3],
        dp: [f32; 3],
        avga: [f32; 3],
        avgw: [f32; 3],
        c: [[f32; 15]; 15],
        bias: RustImuBias,
        t: f64,
    }
    struct InertialEstimate {
        vb: [f64; 6],
        bg: [f64; 3],
        ba: [f64; 3],
        scale: f64,
        rwg: [[f64; 3]; 3],
    }

    unsafe extern "C++" {
        include!("rust_helper.h");

        // Opaque types which both languages can pass around
        // but only C++ can see the fields.
        type BridgeSparseOptimizer;
        type VertexSE3Expmap;
        type VertexSBAPointXYZ;
        type EdgeSE3ProjectXYZOnlyPose;
        type EdgeSE3ProjectXYZ;
        type EdgeSim3ProjectXYZ;
        type EdgeInverseSim3ProjectXYZ;
        type EdgeSim3;
        type EdgeMono;
        type EdgeMonoOnlyPose;
        type EdgeInertial;

        fn new_sparse_optimizer(opt_type: i32, camera_param: [f64;4], lambda_init: f32) -> UniquePtr<BridgeSparseOptimizer>;
        // fn set_stop_flag(self: Pin<&mut BridgeSparseOptimizer>, should_stop: bool);
        // fn enable_stop_flag(self: Pin<&mut BridgeSparseOptimizer>);

        //* Adding and modifying vertices */
        fn has_vertex(self: &BridgeSparseOptimizer, id: i32) -> bool;
        fn remove_vertex(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
        );
        fn add_vertex_sim3expmap(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            sim3: RustSim3,
            fix_scale: bool,
            set_fixed: bool,
            set_camera_params: bool
        );
        fn add_vertex_se3expmap(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            pose: Pose,
            set_fixed: bool
        );
        fn add_vertex_pose(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            set_fixed: bool,
            num_cams: i32,
            imu_position: [f64; 3],
            imu_rotation: [f64; 4],
            translation: [f64; 3],
            rotation: [f64; 4],
            tcb_translation: [f64; 3],
            tcb_rotation: [f64; 4],
            tbc_translation: [f64; 3],
            bf: f32,
        );
        fn add_vertex_velocity(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            set_fixed: bool,
            velocity: [f64; 3],
        );
        fn add_vertex_gyrobias(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            set_fixed: bool,
            gyro_bias: [f64; 3],
        );
        fn add_vertex_accbias(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            set_fixed: bool,
            acc_bias: [f64; 3],
        );
        fn add_vertex_gdir(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            set_fixed: bool,
            rwg: [[f64; 3]; 3],
        );
        fn add_vertex_scale(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            set_fixed: bool,
            scale: f64,
        );
        fn add_vertex_sbapointxyz(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            pose: Pose,
            set_fixed: bool,
            set_marginalized: bool
        );
        fn update_estimate_vertex_se3xpmap(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex: i32,
            pose: Pose,
        );

        //* Adding and modifying edges */
        fn num_edges(self: &BridgeSparseOptimizer) -> i32;
        fn add_both_sim_edges(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id1: i32, keypoint_pt_x1: f32, keypoint_pt_y1: f32, inv_sigma1: f32,
            vertex_id2: i32, keypoint_pt_x2: f32, keypoint_pt_y2: f32, inv_sigma2: f32,
            huber_delta: f32
        );
        fn add_one_sim3_edge(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id_i: i32,
            vertex_id_j: i32,
            observation: RustSim3,
        );
        fn add_edge_se3_project_xyz_monocular_unary(
            self: Pin<&mut BridgeSparseOptimizer>,
            robust_kernel: bool,
            vertex_id: i32,
            keypoint_octave: i32,
            keypoint_pt_x: f32,
            keypoint_pt_y: f32,
            inv_sigma2: f32,
            mp_world_position: [f64; 3],
            mappoint_id: i32,
            huber_delta: f32
        );
        fn add_edge_se3_project_xyz_monocular_binary(
            self: Pin<&mut BridgeSparseOptimizer>,
            robust_kernel: bool,
            vertex_id_1: i32,
            vertex_id_2: i32,
            mp_id: i32,
            keypoint_pt_x: f32,
            keypoint_pt_y: f32,
            inv_sigma2: f32,
            huber_delta: f32
        );
        fn add_edge_mono_binary(
            self: Pin<&mut BridgeSparseOptimizer>,
            robust_kernel: bool,
            vertex_id_1: i32,
            vertex_id_2: i32,
            mp_id: i32,
            keypoint_pt_x: f32,
            keypoint_pt_y: f32,
            inv_sigma2: f32,
            huber_delta: f32
        );
        fn add_edge_mono_only_pose(
            self: Pin<&mut BridgeSparseOptimizer>,
            robust_kernel: bool,
            vertex_id: i32,
            mp_id: i32,
            mp_world_position: [f64; 3],
            keypoint_pt_x: f32,
            keypoint_pt_y: f32,
            inv_sigma2: f32,
            huber_delta: f32
        );
        fn add_edge_prior_for_imu(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id1: i32,
            vertex_id2: i32,
            bprior: [f64; 3],
            prior_a: f64,
            prior_g: f64
        );
        fn add_edge_gyro_and_acc(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex1_id: i32,
            vertex2_id: i32,
            vertex3_id: i32,
            vertex4_id: i32,
            imu_preintegrated: RustImuPreintegrated,
        );
        fn add_edge_inertial_gs(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_p1_id: i32,
            vertex_v1_id: i32,
            vertex_g_id: i32,
            vertex_a_id: i32,
            vertex_p2_id: i32,
            vertex_v2_id: i32,
            vertex_gdir_id: i32,
            vertex_s_id: i32,
            imu_preintegrated: RustImuPreintegrated,
            set_robust_kernel: bool,
            delta: f32,
            set_information: bool,
            information_weight: f32
        );
        fn add_edge_inertial(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_p1_id: i32,
            vertex_v1_id: i32,
            vertex_g_id: i32,
            vertex_a_id: i32,
            vertex_p2_id: i32,
            vertex_v2_id: i32,
            imu_preintegrated: RustImuPreintegrated,
            set_robust_kernel: bool,
            delta: f32
        );
        fn add_edge_prior_pose_imu(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id1: i32,
            vertex_id2: i32,
            vertex_id3: i32,
            vertex_id4: i32,
            rwb: [[f64; 3]; 3],
            twb: [f64; 3],
            vwb: [f64; 3],
            bg: [f64; 3],
            ba: [f64; 3],
            h: [[f64; 15]; 15],
            set_robust_kernel: bool,
            delta: f32
        );
        fn _add_edge_stereo(self: &BridgeSparseOptimizer);
        fn remove_sim3_edges_with_chi2(
            self: Pin<&mut BridgeSparseOptimizer>,
            chi2_threshold: f32
        ) -> Vec<i32>;

        //* Optimization */
        fn optimize(
            self: Pin<&mut BridgeSparseOptimizer>,
            iterations: i32,
            online: bool,
            compute_active_errors: bool
        );
        fn recover_optimized_frame_pose(
            self: &BridgeSparseOptimizer,
            vertex: i32,
        ) -> Pose;
        fn recover_optimized_vertex_pose(
            self: &BridgeSparseOptimizer,
            vertex: i32,
        ) -> Pose;
        fn recover_optimized_mappoint_pose(
            self: &BridgeSparseOptimizer,
            vertex: i32,
        ) -> Position;
        fn recover_optimized_sim3(
            self: &BridgeSparseOptimizer,
            vertex: i32,
        ) -> RustSim3;
        fn recover_optimized_inertial(
            self: &BridgeSparseOptimizer,
            vg: i32,
            va: i32,
            vs: i32,
            vgdir: i32,
        ) -> InertialEstimate;
        fn recover_optimized_vertex_velocity(
            self: &BridgeSparseOptimizer,
            vertex_id: i32,
        ) -> [f64; 3];
        fn get_hessian_from_edge_inertial(
            self: &BridgeSparseOptimizer,
            edge_idx: i32
        ) -> [[f64; 24]; 24];
        fn get_hessian2_from_edge_inertial(
            self: &BridgeSparseOptimizer,
            edge_idx: i32
        ) -> [[f64; 9]; 9];
        fn get_hessian_from_edge_gyro(
            self: &BridgeSparseOptimizer,
        ) -> [[f64; 6]; 6];
        fn get_hessian2_from_edge_gyro(
            self: &BridgeSparseOptimizer,
        ) -> [[f64; 3]; 3];
        fn get_hessian_from_edge_acc(
            self: &BridgeSparseOptimizer,
        ) -> [[f64; 6]; 6];
        fn get_hessian2_from_edge_acc(
            self: &BridgeSparseOptimizer,
        ) -> [[f64; 3]; 3];
        fn get_hessian_from_edge_prior(
            self: &BridgeSparseOptimizer,
        ) -> [[f64; 15]; 15];

        fn save(
            self: &BridgeSparseOptimizer,
            filename: &str, 
            save_id: i32,
        );

        // Note: BridgeSparseOptimizer has vector of RustEdge types, call get_mut_edges to access
        // the vec and then iterate through it and call functions on each edge inside.
        // Kind of an annoying workaround for this problem:
        // All items returned from C++ need to be a Unique or Shared ptr. We can return one to
        // the edges directly and have rust manage the vector, instead of keeping the vec on
        // the C++ side. But some functions don't need to access the edges and therefore don't
        // do anything meaningful with the edge vec, causing the compiler to free the memory of
        // the shared/unique ptr. But we don't want the memory freed, because it is still being
        // used by the optimizer!
        fn get_mut_xyz_edges(self: Pin<&mut BridgeSparseOptimizer>) -> Pin<&mut CxxVector<RustXYZEdge>>;
        fn get_mut_xyz_onlypose_edges(self: Pin<&mut BridgeSparseOptimizer>) -> Pin<&mut CxxVector<RustXYZOnlyPoseEdge>>;
        fn get_mut_sim3_edges(self: Pin<&mut BridgeSparseOptimizer>) -> Pin<&mut CxxVector<RustSim3ProjectXYZEdge>>;
        fn get_mut_mono_edges(self: Pin<&mut BridgeSparseOptimizer>) -> Pin<&mut CxxVector<RustEdgeMono>>;
        fn get_mut_mono_onlypose_edges(self: Pin<&mut BridgeSparseOptimizer>) -> Pin<&mut CxxVector<RustEdgeMonoOnlyPose>>;

        //* Functions on an edge, not on the optimizer */
        #[rust_name = "set_level"]
        fn setLevel(
            self: Pin<&mut EdgeSE3ProjectXYZOnlyPose>,
            level: i32,
        );
        #[rust_name = "set_level"]
        fn setLevel(
            self: Pin<&mut EdgeMonoOnlyPose>,
            level: i32,
        );
        #[rust_name = "compute_error"]
        fn computeError(self: Pin<&mut EdgeSE3ProjectXYZOnlyPose>);
        #[rust_name = "compute_error"]
        fn computeError(self: Pin<&mut EdgeMonoOnlyPose>);
        fn chi2(self: &EdgeSE3ProjectXYZOnlyPose) -> f64;
        fn chi2(self: &EdgeSE3ProjectXYZ) -> f64;
        fn chi2(self: &EdgeSim3ProjectXYZ) -> f64;
        fn chi2(self: &EdgeInverseSim3ProjectXYZ) -> f64;
        fn chi2(self: &EdgeMonoOnlyPose) -> f64;
        #[rust_name = "is_depth_positive"]
        fn isDepthPositive(self: &EdgeSE3ProjectXYZOnlyPose) -> bool;
        #[rust_name = "is_depth_positive"]
        fn isDepthPositive(self: &EdgeMonoOnlyPose) -> bool;
        #[rust_name = "is_depth_positive"]
        fn isDepthPositive(self: &EdgeSE3ProjectXYZ) -> bool;
        fn set_robust_kernel(self: Pin<&mut EdgeMonoOnlyPose>, reset: bool, delta: f32);
        fn set_robust_kernel(self: Pin<&mut EdgeSE3ProjectXYZOnlyPose>, reset: bool, delta: f32);
        #[rust_name = "get_hessian"]
        fn GetHessianRust(self: Pin<&mut EdgeMonoOnlyPose>) -> [[f64; 6]; 6];
        // #[rust_name = "get_hessian"]
        // fn GetHessian(self: Pin<&mut EdgeMonoOnlyPose>) -> [[f64; 15]; 15];

    }
}
