#include "isam2.h"
#include "../base/rust.hpp"

namespace gtsam
{

    std::unique_ptr<ISAM2> default_isam2()
    {
        return std::make_unique<ISAM2>();
    }

    void update_noresults(ISAM2 &isam2, const NonlinearFactorGraph &graph, const Values &initial_values)
    {
        // std::cout << "Initial values: ";
        // initial_values.print();
        // std::cout << std::endl << "Graph: ";
        // graph.print();

        ISAM2Result result = isam2.update(graph, initial_values);
    }

    std::unique_ptr<Values> calculate_estimate(const ISAM2 &isam2) {
        return std::make_unique<Values>(isam2.calculateEstimate());
    }

    rust::Vec<DoubleVec> get_marginal_covariance(
        const ISAM2 &isam2,
        const Key key)
    {
        return eigenmat_to_rustvec(isam2.marginalCovariance(key));
    }
}