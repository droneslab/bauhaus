#include <Eigen/Core>
#include "rust/cxx.h"


namespace orb_slam3 {
    struct SVDResult;
    struct DoubleVec;

    std::array<::std::array<double, 3>, 3> normalize_rotation(std::array<std::array<double, 3>, 3> rot_array);
    SVDResult svd(rust::Vec<DoubleVec> mat);

    Eigen::MatrixXd rustvec_to_eigenmat(rust::Vec<DoubleVec> mat);
    rust::Vec<DoubleVec> eigenmat_to_rustvec(Eigen::MatrixXd mat);
    rust::Vec<double> row_eigenmat_to_rustvec(Eigen::MatrixXd mat);
}