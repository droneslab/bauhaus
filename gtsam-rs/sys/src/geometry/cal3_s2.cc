#include "cal3_s2.h"

namespace gtsam
{

    std::shared_ptr<Cal3_S2> default_cal3_s2() {
        return std::make_shared<Cal3_S2>();
    }
} // namespace gtsam
