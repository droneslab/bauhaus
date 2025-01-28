#include "isam2.h"

namespace gtsam
{

    std::unique_ptr<ISAM2> default_isam2()
    {
        return std::make_unique<ISAM2>();
    }

    void update_noresults(ISAM2 &isam2, const NonlinearFactorGraph &graph, const Values &initial_values)
    {
        ISAM2Result result = isam2.update(graph, initial_values);
    }

    std::unique_ptr<Values> calculate_estimate(const ISAM2 &isam2) {
        return std::make_unique<Values>(isam2.calculateEstimate());
    }
}