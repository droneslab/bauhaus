#pragma once

#include "rust/cxx.h"
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Cal3_S2Stereo.h>
#include <memory>

namespace gtsam
{

    std::shared_ptr<Cal3_S2> default_cal3_s2();
    std::shared_ptr<Cal3_S2> new_cal3_s2(double fx, double fy, double s, double u0, double v0);
    std::shared_ptr<gtsam::Cal3_S2Stereo> new_cal3_s2_stereo(
        double fx, double fy, double s, double u0, double v0, double b
    );

} // namespace gtsam
