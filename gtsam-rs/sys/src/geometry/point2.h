#pragma once

#include "rust/cxx.h"
#include <gtsam/geometry/Point2.h>
#include <memory>

namespace gtsam {

std::unique_ptr<Point2> default_point2();

std::unique_ptr<Point2> new_point2(double x, double y);

} // namespace gtsam
