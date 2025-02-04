#include "navstate.h"

namespace gtsam
{
    std::unique_ptr<NavState> new_navstate(const Pose3 &pose, const Vector3 &v)
    {
        return std::make_unique<NavState>(pose, v);
    }

    std::unique_ptr<Pose3> get_pose(const NavState &navstate)
    {
        return std::make_unique<Pose3>(navstate.pose());
    }

    const Vector3 &get_velocity(const NavState &navstate)
    {
        return navstate.velocity();
    }
}