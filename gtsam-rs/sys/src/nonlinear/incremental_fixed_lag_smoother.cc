#include "incremental_fixed_lag_smoother.h"
#include "../base/rust.hpp"
#include <gtsam/inference/Symbol.h>
#include "../../target/cxxbridge/gtsam-sys/src/lib.rs.h"

namespace gtsam
{
    extern int foobar;

    void get_foobar(void) {
        std::cout << "Hello from foobar! " << foobar << std::endl;
    }
    
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
        const rust::Vec<DoubleVec> & timestamps,
        const rust::Vec<int>&  delete_slots
    ) {
        std::map<Key, double> timestamps_gtsam;
        for (int i = 0; i < timestamps.size(); i++)
        {
            rust::Vec<double> row = timestamps[i].vec;
            timestamps_gtsam[row[0]] = row[1];
        }

        std::cout << "Before update, smoother has: ";
        smoother.getLinearizationPoint().print();

        std::cout << "Initial values: ";
        new_values.print();
        std::cout << std::endl << "Graph: ";
        new_factors.print();

        gtsam::FactorIndices delete_slots_gtsam;
        for (int i = 0; i < delete_slots.size(); i++)
        {
            // std::cout << "C++, removing key: " << keys_to_remove[i] << std::endl;
            delete_slots_gtsam.push_back(delete_slots[i]);
        }

        std::cout << "iSAM2 update with " << new_factors.size() << " new factors "
           << ", " << new_values.size() << " new values "
           << ", and " << delete_slots.size() << " deleted factors.";

        smoother.update(new_factors, new_values, timestamps_gtsam, delete_slots_gtsam);
        ISAM2Result result = smoother.getISAM2Result();

        // std::cout << "Detailed results:" << std::endl;
        // for (auto keyedStatus : result.detail->variableStatus) {
        //     const auto& status = keyedStatus.second;
        //     PrintKey(keyedStatus.first);
        //     std::cout << " {" << std::endl;
        //     std::cout << "reeliminated: " << status.isReeliminated << std::endl;
        //     std::cout << "relinearized above thresh: " << status.isAboveRelinThreshold
        //         << std::endl;
        //     std::cout << "relinearized involved: " << status.isRelinearizeInvolved << std::endl;
        //     std::cout << "relinearized: " << status.isRelinearized << std::endl;
        //     std::cout << "observed: " << status.isObserved << std::endl;
        //     std::cout << "new: " << status.isNew << std::endl;
        //     std::cout << "in the root clique: " << status.inRootClique << std::endl;
        //     std::cout << "}" << std::endl;
        // }


        // std::cout << "After update, isam2 has: ";
        // isam2.getLinearizationPoint().print();

        // std::cout << "GET FACTORS! ";
        // smoother.getFactorsUnsafe().print();

        rust::Vec<std::uint64_t> new_factor_indices;
        // for (const auto& value : result.newFactorsIndices) {
        //     new_factor_indices.push_back(value);
        // }
        rust::Vec<Point> points;
        rust::Vec<std::uint64_t> invalid_points;
        // auto factors = smoother.getFactorsUnsafe();
        // for (int i = 0; i < factors.size(); i++) {
        //     auto factor = factors[i];
        //     if (auto smartfactor = dynamic_cast<const SmartProjectionPoseFactor<gtsam::Cal3_S2>*>(factor.get())) {
        //         TriangulationResult res = smartfactor->point();
        //         if (res.valid()) {
        //             // std::cout << "Smart factor point: i " << i << ", " << res.get() << std::endl;
        //             points.push_back(Point{i, res.get()(0), res.get()(1), res.get()(2)});
        //         } else {
        //             // std::cout << "Smart factor point not valid" << std::endl;
        //             invalid_points.push_back(i);
        //         }

        //     }
        // }
        ISAM2ResultRust rust_result;
        rust_result.new_factor_indices = new_factor_indices;
        rust_result.points = points;
        rust_result.invalid_points = invalid_points;


        return rust_result;
    }

    std::unique_ptr<Values> calculate_estimate(const IncrementalFixedLagSmoother &smoother) {
        Values estimate = smoother.calculateEstimate();
    //     std::cout << "SOFIYA! VALUES ALL: ";
    //     estimate.print();

        return std::make_unique<Values>(estimate);
    }

    // rust::Vec<DoubleVec> get_marginal_covariance(
    //     const ISAM2 &isam2,
    //     const Key key)
    // {
    //     return eigenmat_to_rustvec(isam2.marginalCovariance(key));
    // }
}