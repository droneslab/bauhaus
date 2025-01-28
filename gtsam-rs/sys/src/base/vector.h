#pragma once

#include "rust/cxx.h"
#include <gtsam/base/Vector.h>
#include <memory>

namespace gtsam
{

    std::unique_ptr<Vector3> default_vector3();

    std::unique_ptr<Vector3> new_vector3(double x, double y, double z);

    void vector3_to_raw(const Vector3 &src, rust::Slice<double> dst);

} // namespace gtsam
