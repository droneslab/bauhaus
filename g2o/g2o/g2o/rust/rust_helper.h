
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../core/base_unary_edge.h"
#include "../core/base_binary_edge.h"
#include "../solvers/linear_solver_dense.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../types/types_six_dof_expmap.h"

#define EIGEN_DONT_VECTORIZE
#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

namespace g2o {
    // using Edge = OptimizableGraph::Edge;
    struct Pose;
        // SOFIYA TEST

    // struct RustXYZEdge;
    // struct RustXYZOnlyPoseEdge;

    class BridgeSparseOptimizer {
    public:
        BridgeSparseOptimizer(int opt_type, std::array<double,4> camera_param);
        // ~BridgeSparseOptimizer();

        // vertices
        bool has_vertex(int id) const;
        void remove_vertex(int vertex_id);
        void add_frame_vertex(int vertex_id, Pose pose, bool set_fixed);
        void add_mappoint_vertex(int vertex_id,  Pose pose);
        void set_vertex_estimate(int vertex_id, Pose pose);

        // edges
        void add_edge_monocular_unary(
            bool robust_kernel, int vertex_id,
            int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float invSigma2,
            array<double, 3> mp_world_position,
            int mappoint_id
        );
        void add_edge_monocular_binary(
            bool robust_kernel, int vertex1, int vertex2,
            int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float invSigma2,
            int huber_delta
        );
        void _add_edge_stereo() const;
        int num_edges() const;

        // optimization
        void optimize(int iterations);
        Pose recover_optimized_frame_pose(int vertex_id) const;
        Pose recover_optimized_mappoint_pose(int vertex_id) const;

    // SOFIYA TEST

        // Note: see explanation under get_mut_edges in lib.rs for why we do this
        // std::vector<RustXYZEdge> xyz_edges;
        // std::vector<RustXYZOnlyPoseEdge> xyz_onlypose_edges;
        // std::vector<RustXYZEdge>& get_mut_xyz_edges();
        // std::vector<RustXYZOnlyPoseEdge>& get_mut_xyz_onlypose_edges();

        // void set_robust_kernel_for_edge(int edge_id, bool reset);

    private:
        SparseOptimizer * optimizer;
        int optimizer_type; // see constructor 
        float deltaMono;
        float deltaStereo;
        float thHuberMono;
        float thHuberStereo;
        float thHuber2D;
        float thHuber3D;
        vector<size_t> vnIndexEdgeMono;

        SE3Quat format_pose(Pose pose) const;

        // Camera Parameters for Optimization
        float fx,fy,cx,cy;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer(int opt_type, std::array<double,4> camera_param);
} // end namespace
