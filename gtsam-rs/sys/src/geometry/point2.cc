#include "point2.h"

namespace gtsam {

std::unique_ptr<Point2> default_point2() { return std::make_unique<Point2>(); }

std::unique_ptr<Point2> new_point2(double x, double y) {
  return std::make_unique<Point2>(x, y);
}


} // namespace gtsam
