#include "rust.hpp"
#include "../../target/cxxbridge/gtsam-sys/src/lib.rs.h"


namespace gtsam {
    Eigen::MatrixXd rustvec_to_eigenmat(rust::Vec<DoubleVec> mat)
    {
        std::size_t rows = mat.size();
        std::size_t cols = mat[0].vec.size();
        Eigen::MatrixXd mat_c = Eigen::MatrixXd::Zero(rows, cols);

        for (int i = 0; i < mat.size(); i++)
        {
            rust::Vec<double> row = mat[i].vec;
            for (int j = 0; j < row.size(); j++)
            {
                mat_c(i, j) = row[j];
            }
        }
        return mat_c;
    }

    rust::Vec<DoubleVec> eigenmat_to_rustvec(Eigen::MatrixXd mat)
    {
        rust::Vec<DoubleVec> mat_rust;

        for (int i = 0; i < mat.rows(); i++)
        {
            rust::Vec<double> row_rust;
            for (int j = 0; j < mat.cols(); j++)
            {
                row_rust.push_back(mat(i, j));
            }
            mat_rust.push_back(DoubleVec{row_rust});
        }
        return mat_rust;
    }
    rust::Vec<double> row_eigenmat_to_rustvec(Eigen::MatrixXd mat)
    {
        rust::Vec<double> vec_rust;

        for (int i = 0; i < mat.rows(); i++)
        {
            vec_rust.push_back(mat(i, 0));
        }
        return vec_rust;
    }
} // namespace gtsam