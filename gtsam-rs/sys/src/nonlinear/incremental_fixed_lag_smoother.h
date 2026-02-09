#include "rust/cxx.h"
#include <memory>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

namespace gtsam {
    struct DoubleVec;
    struct ISAM2ResultRust;

    void get_foobar(void);

    std::unique_ptr<IncrementalFixedLagSmoother> new_incremental_fixed_lag_smoother(
        double relinearizeThreshold, int relinearizeSkip,
        bool cacheLinearizedFactors, bool enableDetailedResults,
        float wildfire_threshold,
        bool find_unused_factor_slots,
        int nr_states
    );

    // Same as update()... for some reason rust won't let me name it update since isam2
    // already defines update
    ISAM2ResultRust update_smoother(
        IncrementalFixedLagSmoother &smoother,
        const gtsam::NonlinearFactorGraph& new_factors,
        const gtsam::Values& new_values,
        // const rust::Vec<DoubleVec> & timestamps,
        const double cur_id,
        const rust::Vec<unsigned long int>&  delete_slots
    );

    std::unique_ptr<Values> calculate_estimate_smoother(const IncrementalFixedLagSmoother &smoother);

    bool slot_exists_in_smoother(
        const IncrementalFixedLagSmoother &smoother,
        const size_t slot);


    // rust::Vec<DoubleVec> get_marginal_covariance(const ISAM2 &isam2, const Key key);
}