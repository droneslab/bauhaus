#pragma once

#include "rust/cxx.h"
#include <gtsam/geometry/Cal3_S2.h>
#include <memory>

namespace gtsam
{

    std::shared_ptr<Cal3_S2> default_cal3_s2();

} // namespace gtsam
