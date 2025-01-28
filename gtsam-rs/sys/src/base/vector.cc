#include "vector.h"

namespace gtsam {


std::unique_ptr<Vector3> default_vector3() { return std::make_unique<Vector3>(); }

std::unique_ptr<Vector3> new_vector3(double x, double y, double z)
{
    return std::make_unique<Vector3>(x, y, z);
}

void vector3_to_raw(const Vector3 &src, rust::Slice<double> dst)
{
    const double *p_src = src.data();
    double *p_dst = dst.data();

    const size_t size = dst.size();
    for (size_t i = 0; i < size; ++i)
    {
        p_dst[i] = p_src[i];
    }
}

} // namespace gtsam
