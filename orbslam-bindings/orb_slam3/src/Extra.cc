#include "Extra.h"
#include<Eigen/Dense>
#include "../../target/cxxbridge/dvos3binding/src/lib.rs.h"

namespace orb_slam3 {

    std::array<::std::array<double, 3>, 3> normalize_rotation(std::array<std::array<double, 3>, 3> rot_array){
        Eigen::Matrix3d R;
        R << rot_array[0][0], rot_array[0][1], rot_array[0][2],
             rot_array[1][0], rot_array[1][1], rot_array[1][2],
             rot_array[2][0], rot_array[2][1], rot_array[2][2];

        Eigen::JacobiSVD<Eigen::Matrix3d> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
        auto temp = svd.matrixU() * svd.matrixV().transpose();

        std::array<std::array<double, 3>, 3> arr  = { {
            {temp(0,0), temp(0,1), temp(0,2)},
            {temp(1,0), temp(1,1), temp(1,2)},
            {temp(2,0), temp(2,1), temp(2,2)}
        } };
        return arr;
    }

    SVDResult svd(rust::Vec<DoubleVec> mat, SVDComputeType compute_type){
        // Returns (U, V, singular values)
        // Need to pass in type of SVD to compute, options below:
        unsigned int compute_type_eigen;
        switch (compute_type) {
            case orb_slam3::SVDComputeType::ThinUThinV:
                compute_type_eigen = Eigen::ComputeThinU | Eigen::ComputeThinV;
                break;
            case orb_slam3::SVDComputeType::FullV:
                compute_type_eigen = Eigen::ComputeFullV;
                break;
        };

        Eigen::MatrixXd mat_c = rustvec_to_eigenmat(mat);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat_c, compute_type_eigen);
        Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType singularValues_inv=svd.singularValues();

        SVDResult result;
        result.u = eigenmat_to_rustvec(svd.matrixU());
        result.v = eigenmat_to_rustvec(svd.matrixV());
        result.singular_values = row_eigenmat_to_rustvec(singularValues_inv);
        return result;
    }

    Eigen::MatrixXd rustvec_to_eigenmat(rust::Vec<DoubleVec> mat){
        std::size_t rows = mat.size();
        std::size_t cols = mat[0].vec.size();
        Eigen::MatrixXd mat_c = Eigen::MatrixXd::Zero(rows, cols);

        for (int i = 0; i < mat.size(); i++) {
            rust::Vec<double> row = mat[i].vec;
            for (int j = 0; j < row.size(); j++) {
                mat_c(i,j) = row[j];
            }
        }
        return mat_c;
    }

    rust::Vec<DoubleVec> eigenmat_to_rustvec(Eigen::MatrixXd mat){
        rust::Vec<DoubleVec> mat_rust;

        for (int i = 0; i < mat.rows(); i++) {
            rust::Vec<double> row_rust;
            for (int j = 0; j < mat.cols(); j++) {
                row_rust.push_back(mat(i,j));
            }
            mat_rust.push_back(DoubleVec { row_rust } );
        }
        return mat_rust;
    }
    rust::Vec<double> row_eigenmat_to_rustvec(Eigen::MatrixXd mat){
        rust::Vec<double> vec_rust;

        for (int i = 0; i < mat.rows(); i++) {
            vec_rust.push_back(mat(i,0) );
        }
        return vec_rust;
    }
}