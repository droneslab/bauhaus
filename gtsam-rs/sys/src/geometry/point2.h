#pragma once

#include "rust/cxx.h"
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/StereoPoint2.h>
#include <memory>

namespace gtsam {

std::unique_ptr<Point2> default_point2();
std::unique_ptr<Point2> new_point2(double x, double y);

std::unique_ptr<StereoPoint2> default_stereopoint2();
std::unique_ptr<StereoPoint2> new_stereopoint2(double uL, double uR, double v);
std::unique_ptr<StereoPoint2> new_stereopoint2_nour(double uL, double v);

} // namespace gtsam
