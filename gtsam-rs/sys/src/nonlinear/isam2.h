#include "rust/cxx.h"
#include <memory>
#include <gtsam/nonlinear/ISAM2.h>

namespace gtsam {
    struct DoubleVec;

    std::unique_ptr<ISAM2> default_isam2();

    void update_noresults(ISAM2 &isam2, const NonlinearFactorGraph &graph, const Values &initial_values);
    std::unique_ptr<Values> calculate_estimate(const ISAM2 &isam2);
    rust::Vec<DoubleVec> get_marginal_covariance(const ISAM2 &isam2, const Key key);
}