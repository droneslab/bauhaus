pub type TranslationVector = [f64; 3];
pub type RotationVector = [f64; 4];

#[cxx::bridge(namespace = "g2o")]
pub mod ffi {
    // Shared structs with fields visible to both languages.

    // TODO (SLAM with respect to rigid body)
    // struct BridgeEdgeSE3ProjectXYZOnlyPose {
    //     edge: SharedPtr<EdgeSE3ProjectXYZOnlyPose>
    // } 
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

        fn new_sparse_optimizer(opt_type: i32, camera_param: [f64;4]) -> UniquePtr<BridgeSparseOptimizer>;
        // fn set_stop_flag(self: Pin<&mut BridgeSparseOptimizer>, should_stop: bool);
        // fn enable_stop_flag(self: Pin<&mut BridgeSparseOptimizer>);

        // creating/adding vertices to graph
        fn add_vertex_sim3_expmap(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            pose: RustSim3,
            fix_scale: bool,
            set_fixed: bool,
            set_camera_params: bool
        );
        fn add_frame_vertex(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            pose: Pose,
            set_fixed: bool
        );
        fn add_mappoint_vertex(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            pose: Pose,
            set_fixed: bool,
            set_marginalized: bool
        );
        fn set_vertex_estimate(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex: i32,
            pose: Pose,
        );
        fn remove_vertex(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
        );
        fn has_vertex(self: &BridgeSparseOptimizer, id: i32) -> bool;

        // creating/adding edges to graph
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
        fn add_edge_monocular_unary(
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
        fn add_edge_monocular_binary(
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
        // fn set_edge_worldpos(
        //     self: &BridgeSparseOptimizer,
        //     mp_world_index: i32,
        //     edge: SharedPtr<EdgeSE3ProjectXYZOnlyPose>,
        //     mp_world_position: [f64; 3]
        // );
        // fn add_edge(
        //     self: &BridgeSparseOptimizer,
        //     mp_world_index: i32,
        //     edge: SharedPtr<EdgeSE3ProjectXYZOnlyPose>,
        //     mp_world_position: [f64; 3]
        // );

        fn _add_edge_stereo(self: &BridgeSparseOptimizer);
        fn num_edges(self: &BridgeSparseOptimizer) -> i32;
        fn remove_sim3_edges_with_chi2(
            self: Pin<&mut BridgeSparseOptimizer>,
            chi2_threshold: f32
        ) -> Vec<i32>;

        // optimization
        fn optimize(
            self: Pin<&mut BridgeSparseOptimizer>,
            iterations: i32,
            online: bool,
        );
        fn recover_optimized_frame_pose(
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

        // optimization within edge
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
        #[rust_name = "set_level"]
        fn setLevel(
            self: Pin<&mut EdgeSE3ProjectXYZOnlyPose>,
            level: i32,
        );
        #[rust_name = "compute_error"]
        fn computeError(self: Pin<&mut EdgeSE3ProjectXYZOnlyPose>);
        fn chi2(self: &EdgeSE3ProjectXYZOnlyPose) -> f64;
        fn chi2(self: &EdgeSE3ProjectXYZ) -> f64;
        fn chi2(self: &EdgeSim3ProjectXYZ) -> f64;
        fn chi2(self: &EdgeInverseSim3ProjectXYZ) -> f64;

        #[rust_name = "is_depth_positive"]
        fn isDepthPositive(self: &EdgeSE3ProjectXYZOnlyPose) -> bool;
        fn set_robust_kernel(self: Pin<&mut EdgeSE3ProjectXYZOnlyPose>, reset: bool);

        #[rust_name = "is_depth_positive"]
        fn isDepthPositive(self: &EdgeSE3ProjectXYZ) -> bool;

    }
}
