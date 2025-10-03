#include "rust/cxx.h"
#include <memory>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

namespace gtsam {
    struct DoubleVec;
    struct ISAM2ResultRust;

    void get_foobar(void);

    std::unique_ptr<IncrementalFixedLagSmoother> new_incremental_fixed_lag_smoother(
        double smootherLag,
        double relinearizeThreshold, int relinearizeSkip,
        bool cacheLinearizedFactors, bool enableDetailedResults
    );

    // Same as update()... for some reason rust won't let me name it update since isam2
    // already defines update
    ISAM2ResultRust update_smoother(
        IncrementalFixedLagSmoother &smoother, const NonlinearFactorGraph &graph,
        const Values &initial_values);
    std::unique_ptr<Values> calculate_estimate(const IncrementalFixedLagSmoother &smoother);


    // rust::Vec<DoubleVec> get_marginal_covariance(const ISAM2 &isam2, const Key key);
}