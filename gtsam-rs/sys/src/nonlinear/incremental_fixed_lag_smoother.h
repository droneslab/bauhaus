// #include "rust/cxx.h"
// #include <memory>
// // #include <gtsam/nonlinear/IncrementalFixedLagSmoother.h>

// namespace gtsam {
//     // struct DoubleVec;
//     // struct ISAM2ResultRust;

//     std::unique_ptr<IncrementalFixedLagSmoother> default_incremental_fixed_lag_smoother(double smootherLag);

//     // ISAM2ResultRust update(ISAM2 &isam2, const NonlinearFactorGraph &graph, const Values &initial_values, const rust::Vec<DoubleVec> & new_affected_keys, const rust::Vec<unsigned long int> & keys_to_remove);
//     // std::unique_ptr<Values> calculate_estimate(const ISAM2 &isam2);
//     // rust::Vec<DoubleVec> get_marginal_covariance(const ISAM2 &isam2, const Key key);
// }