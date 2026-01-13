#include "point2.h"

namespace gtsam {

std::unique_ptr<Point2> default_point2() { return std::make_unique<Point2>(); }

std::unique_ptr<Point2> new_point2(double x, double y) {
  return std::make_unique<Point2>(x, y);
}

std::unique_ptr<StereoPoint2> default_stereopoint2() { return std::make_unique<StereoPoint2>(); }

std::unique_ptr<StereoPoint2> new_stereopoint2(double uL, double uR, double v) {
    return std::make_unique<StereoPoint2>(uL, uR, v);
}

std::unique_ptr<StereoPoint2> new_stereopoint2_nour(double uL, double v) {
    return std::make_unique<StereoPoint2>(uL, std::numeric_limits<double>::quiet_NaN(), v);
}



} // namespace gtsam
