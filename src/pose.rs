pub mod pose {

    extern crate nalgebra as na;

    pub struct Pose {
        // 3-Vector
        pos: na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
        // 3x3 Matrix
        rot: na::Matrix<f64, na::U3, na::U3, na::base::storage::Owned<f64, na::U3, na::U3>>,
    }
}
