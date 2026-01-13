#include "incremental_fixed_lag_smoother.h"
#include "../base/rust.hpp"
#include <gtsam/inference/Symbol.h>
#include "../../target/cxxbridge/gtsam-sys/src/lib.rs.h"

namespace gtsam
{
    using symbol_shorthand::X;

    std::unique_ptr<IncrementalFixedLagSmoother> new_incremental_fixed_lag_smoother(
        double relinearizeThreshold, int relinearizeSkip,
        bool cacheLinearizedFactors, bool enableDetailedResults,
        float wildfire_threshold,
        bool find_unused_factor_slots,
        int nr_states
    )
    {
        ISAM2Params parameters;
        parameters.cacheLinearizedFactors = cacheLinearizedFactors;
        parameters.relinearizeThreshold = relinearizeThreshold;
        parameters.relinearizeSkip = relinearizeSkip;
        parameters.findUnusedFactorSlots = find_unused_factor_slots;
        parameters.enableDetailedResults = enableDetailedResults;
        parameters.factorization = gtsam::ISAM2Params::CHOLESKY;

        gtsam::ISAM2GaussNewtonParams gauss_newton_params;
        gauss_newton_params.wildfireThreshold = wildfire_threshold;
        parameters.optimizationParams = gauss_newton_params;

        parameters.print();
        IncrementalFixedLagSmoother * smoother = new IncrementalFixedLagSmoother(nr_states, parameters);

        return std::unique_ptr<IncrementalFixedLagSmoother>(smoother);
    }

    ISAM2ResultRust update_smoother(
        IncrementalFixedLagSmoother &smoother,
        const gtsam::NonlinearFactorGraph& new_factors,
        const gtsam::Values& new_values,
        const double cur_id,
        // const rust::Vec<DoubleVec> & timestamps,
        const rust::Vec<unsigned long int>&  delete_slots
    ) {
        std::map<Key, double> timestamps_gtsam;
        for (const auto& key_value : new_values) {
            timestamps_gtsam[key_value.key] = cur_id;
        }

        // Sofiya uncomment this for further debuggin
        // std::cout << "Before update, smoother has: ";
        // smoother.getLinearizationPoint().print();
        // std::cout << "Initial values: ";
        // new_values.print();
        // std::cout << std::endl << "Graph: ";
        // new_factors.print();

        gtsam::FactorIndices delete_slots_gtsam;
        for (int i = 0; i < delete_slots.size(); i++)
        {
            // std::cout << "C++, removing key: " << keys_to_remove[i] << std::endl;
            delete_slots_gtsam.push_back(delete_slots[i]);
        }

        // Sofiya uncomment this for further debuggin
        // std::cout << "iSAM2 update with " << new_factors.size() << " new factors "
        //    << ", " << new_values.size() << " new values "
        //    << ", and " << delete_slots.size() << " deleted factors.";

        // Sofiya uncomment this for further debuggin
        // std::cout << std::endl << "Delete slots: " ;
        // for (const auto& slot : delete_slots_gtsam) {
        //     std::cout << slot << ", ";
        // }

        smoother.update(new_factors, new_values, timestamps_gtsam, delete_slots_gtsam);
        ISAM2Result result = smoother.getISAM2Result();

        // Sofiya uncomment this for further debuggin
        // std::cout << "After update, isam2 has: ";
        // smoother.getLinearizationPoint().print();

        rust::Vec<std::uint64_t> new_factor_indices;
        for (const auto& value : result.newFactorsIndices) {
            new_factor_indices.push_back(value);
        }
        std::cout << std::endl;

        rust::Vec<Point> points;
        rust::Vec<std::uint64_t> invalid_points;
        ISAM2ResultRust rust_result;
        rust_result.new_factor_indices = new_factor_indices;
        rust_result.points = points;
        rust_result.invalid_points = invalid_points;


        return rust_result;
    }

    std::unique_ptr<Values> calculate_estimate_smoother(const IncrementalFixedLagSmoother &smoother) {
        Values estimate = smoother.calculateEstimate();
    //     std::cout << "SOFIYA! VALUES ALL: ";
    //     estimate.print();

        return std::make_unique<Values>(estimate);
    }

    bool slot_exists_in_smoother(
        const IncrementalFixedLagSmoother &smoother,
        const size_t slot)
    {
        return smoother.getFactors().exists(slot);
    }

    // rust::Vec<DoubleVec> get_marginal_covariance(
    //     const ISAM2 &isam2,
    //     const Key key)
    // {
    //     return eigenmat_to_rustvec(isam2.marginalCovariance(key));
    // }
}