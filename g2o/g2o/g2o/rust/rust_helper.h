
#include "../core/sparse_optimizer.h"
#include "../core/block_solver.h"
#include "../core/base_unary_edge.h"
#include "../core/base_binary_edge.h"
#include "../solvers/linear_solver_dense.h"
#include "../core/optimization_algorithm_levenberg.h"
#include "../types/types_six_dof_expmap.h"

namespace g2o {
    struct Pose;

    // Note: it would be nice to combine these two objects into one
    // and return a BaseEdge pointer instead, but that requires making it
    // a template, and I'm not sure how that would work with the rust bindings.
    class BridgeEdgeSE3ProjectXYZOnlyPose{
        public:
            std::shared_ptr<EdgeSE3ProjectXYZOnlyPose> edge;

            void set_level(int level) const;
            void compute_error() const;
            double chi2() const;
            void set_robust_kernel(bool reset) const;
    };

    class BridgeEdgeSE3ProjectXYZ{
        public:
            std::shared_ptr<EdgeSE3ProjectXYZ> edge;
    };

    class BridgeSparseOptimizer {
    public:
        BridgeSparseOptimizer(int opt_type);
        // ~BridgeSparseOptimizer();

        // vertices
        void remove_vertex (std::shared_ptr<VertexSBAPointXYZ> vertex) const;
        std::shared_ptr<VertexSE3Expmap> add_frame_vertex (
            int vertex_id, Pose pose, bool set_fixed
        ) const;
        std::shared_ptr<VertexSBAPointXYZ> add_mappoint_vertex (
            int vertex_id,  Pose pose
        ) const;
        void set_vertex_estimate(
            std::shared_ptr<VertexSE3Expmap> vertex, 
            Pose pose
        ) const;

        // edges
        std::shared_ptr<BridgeEdgeSE3ProjectXYZOnlyPose> add_edge_monocular_unary(
            bool robust_kernel, int vertex_id,
            int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float invSigma2,
            array<double, 3> mp_world_position
        ) const;
        std::shared_ptr<BridgeEdgeSE3ProjectXYZ> add_edge_monocular_binary(
            bool robust_kernel, int vertex_id_1, int vertex_id_2,
            int keypoint_octave, float keypoint_pt_x, float keypoint_pt_y, float invSigma2
        ) const;

        // void add_edge_monocular(
        //     int mp_world_index, std::shared_ptr<BridgeEdgeSE3ProjectXYZOnlyPose> edge,
        //     array<double, 3> mp_world_position
        // ) const;
        void _add_edge_stereo() const;
        int num_edges() const;

        // optimization
        void optimize(int iterations) const;
        Pose recover_optimized_frame_pose(int vertex_id) const;
        Pose recover_optimized_mappoint_pose(int vertex_id) const;

    private:
        std::unique_ptr<SparseOptimizer> optimizer;
        int optimizer_type; // see constructor 
        float deltaMono;
        float deltaStereo;
        float thHuberMono;
        float thHuberStereo;
        float thHuber2D;
        float thHuber3D;
        vector<size_t> vnIndexEdgeMono;

        SE3Quat format_pose(Pose pose) const;
    };

    std::unique_ptr<BridgeSparseOptimizer> new_sparse_optimizer(int opt_type);
} // end namespace
