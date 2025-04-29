#include "cal3_s2.h"

namespace gtsam
{

    std::shared_ptr<Cal3_S2> default_cal3_s2() {
        return std::make_shared<Cal3_S2>();
    }

    std::shared_ptr<Cal3_S2> new_cal3_s2(double fx, double fy, double s, double u0, double v0)
    {
        return std::make_shared<Cal3_S2>(fx, fy, s, u0, v0);
    }

} // namespace gtsam
