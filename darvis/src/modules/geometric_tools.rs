use dvcore::matrix::DVVector3;

use crate::dvmap::pose::Pose;

pub fn triangulate(
    x_c1: DVVector3<f64>, x_c2: DVVector3<f64>,
    pose1: Pose, pose2: Pose
) -> Option<DVVector3<f64>> {
    // bool GeometricTools::Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,3,4> &Tc1w ,Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D)
    // TODO (verify) this might be wrong

    let mut matrix = nalgebra::Matrix4::<f64>::default();
    let pose1m = pose1.as_matrix();
    let pose2m = pose2.as_matrix();
    matrix.set_row(0, &((pose1m.row(2) - pose2m.row(0)) * x_c1[0]));
    matrix.set_row(1, &((pose1m.row(2) - pose2m.row(1)) * x_c1[0]));
    matrix.set_row(2, &((pose1m.row(2) - pose2m.row(0)) * x_c2[0]));
    matrix.set_row(3, &((pose1m.row(2) - pose2m.row(1)) * x_c2[0]));

    let svd = nalgebra::SVD::new(matrix, false, true);
    let binding = svd.v_t.unwrap();
    let x_3d_h = binding.column(3);
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