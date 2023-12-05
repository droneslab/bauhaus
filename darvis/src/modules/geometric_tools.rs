use dvcore::matrix::DVVector3;

use crate::dvmap::pose::DVPose;

pub fn triangulate(
    x_c1: DVVector3<f64>, x_c2: DVVector3<f64>,
    pose1: DVPose, pose2: DVPose
) -> Option<DVVector3<f64>> {
    // bool GeometricTools::Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,3,4> &Tc1w ,Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D)

    let mut matrix = nalgebra::Matrix4::<f64>::default();
    let pose1m: nalgebra::Matrix3x4<f64> = pose1.into();
    let pose2m: nalgebra::Matrix3x4<f64> = pose2.into();

    matrix.set_row(0, &(x_c1[0] * pose1m.row(2) - pose1m.row(0)));
    matrix.set_row(1, &(x_c1[1] * pose1m.row(2) - pose1m.row(1)));
    matrix.set_row(2, &(x_c2[0] * pose2m.row(2) - pose2m.row(0)));
    matrix.set_row(3, &(x_c2[1] * pose2m.row(2) - pose2m.row(1)));

    let svd = nalgebra::SVD::new(matrix, false, true);
    let binding = svd.v_t.unwrap();
    let x_3d_h = binding.row(3);

    if x_3d_h[3] == 0.0 {
        return None;
    }
    // Euclidean coordinates
    Some(DVVector3::new_with(
        x_3d_h[0] / x_3d_h[3],
        x_3d_h[1] / x_3d_h[3],
        x_3d_h[2] / x_3d_h[3]
    ))
}