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

    unsafe extern "C++" {
        // Note: can't use relative path because cargo hates it :(

        include!("rust_helper.h");
        // Opaque types which both languages can pass around
        // but only C++ can see the fields.
        type BridgeSparseOptimizer;
        type VertexSE3Expmap;
        type VertexSBAPointXYZ;
        type EdgeSE3ProjectXYZOnlyPose;
        type EdgeSE3ProjectXYZ;

        fn new_sparse_optimizer(opt_type: i32) -> UniquePtr<BridgeSparseOptimizer>;

        // creating/adding vertices to graph
        fn add_frame_vertex(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            pose: Pose,
            set_fixed: bool
        );
        fn add_mappoint_vertex(
            self: Pin<&mut BridgeSparseOptimizer>,
            vertex_id: i32,
            pose: Pose
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
        fn add_edge_monocular_unary(
            self: Pin<&mut BridgeSparseOptimizer>,
            robust_kernel: bool,
            vertex_id: i32,
            keypoint_octave: i32,
            keypoint_pt_x: f32,
            keypoint_pt_y: f32,
            invSigma2: f32,
            mp_world_position: [f64; 3]
        );
        fn add_edge_monocular_binary(
            self: Pin<&mut BridgeSparseOptimizer>,
            robust_kernel: bool,
            vertex_id_1: i32,
            vertex_id_2: i32,
            keypoint_octave: i32,
            keypoint_pt_x: f32,
            keypoint_pt_y: f32,
            invSigma2: f32,
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

        // optimization
        fn optimize(
            self: Pin<&mut BridgeSparseOptimizer>,
            iterations: i32,
        );
        fn recover_optimized_frame_pose(
            self: &BridgeSparseOptimizer,
            vertex: i32,
        ) -> Pose;
        fn recover_optimized_mappoint_pose(
            self: &BridgeSparseOptimizer,
            vertex: i32,
        ) -> Pose;

        // optimization within edge
        #[rust_name = "set_level"]
        fn setLevel(
            self: Pin<&mut EdgeSE3ProjectXYZOnlyPose>,
            level: i32,
        );
        #[rust_name = "compute_error"]
        fn computeError(self: Pin<&mut EdgeSE3ProjectXYZOnlyPose>);
        fn chi2(self: &EdgeSE3ProjectXYZOnlyPose) -> f64;
        fn set_robust_kernel(self: Pin<&mut EdgeSE3ProjectXYZOnlyPose>, reset: bool);
    }
}