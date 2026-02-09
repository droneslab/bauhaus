#include "rot3.h"

namespace gtsam {

std::unique_ptr<Rot3> default_rot3() { return std::make_unique<Rot3>(); }

std::unique_ptr<Rot3> from_rot3_quaternion(double w, double x, double y,
                                           double z) {
    // const gtsam::Rot3 huh = gtsam::Rot3(0.5308, -0.1365, -0.8329, -0.0761);
    // std::cout << "Real inputs: " << huh << std::endl;

    // const gtsam::Rot3 huh2 = gtsam::Rot3(0.530843, -0.136525, -0.832927, -0.076146);
    // std::cout << "Huh?: " << huh2 << std::endl;

    gtsam::Rot3 huh3 = gtsam::Rot3(w, x, y, z);
    // std::cout << "w, x, y, z: " << w << ", " << x << ", " << y << ", " << z << std::endl;
    // std::cout << "Huh?????: " << huh3 << std::endl;

    return std::make_unique<Rot3>(huh3);
}

void rot3_to_raw(const Rot3 &src, rust::Slice<double> dst) {
  const auto matrix = src.matrix();

  const double *p_src = matrix.data();
  double *p_dst = dst.data();

  const size_t size = dst.size();
  for (size_t i = 0; i < size; ++i) {
    p_dst[i] = p_src[i];
  }
}

} // namespace gtsam
