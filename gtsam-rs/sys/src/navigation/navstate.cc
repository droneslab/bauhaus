#include "navstate.h"

namespace gtsam
{
    std::unique_ptr<NavState> new_navstate(const Pose3 &pose, const Vector3 &v)
    {
        return std::make_unique<NavState>(pose, v);
    }

    const Pose3 &pose(const NavState &navstate)
    {
        return navstate.pose();
    }

    const Vector3 &velocity(const NavState &navstate)
    {
        return navstate.velocity();
    }

}