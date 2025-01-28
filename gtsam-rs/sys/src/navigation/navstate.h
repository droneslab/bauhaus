#include "rust/cxx.h"
#include <memory>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>

namespace gtsam
{
    std::unique_ptr<NavState> new_navstate(const Pose3 &pose, const Vector3 &v);
    const Pose3 &pose(const NavState &navstate);
    const Vector3 &velocity(const NavState &navstate);
}