use nalgebra;

mod Pose {
    pub struct Pose {
        pos: nalgebra::Point3,
        rot: nalgebra::Matrix3x3
    }
}
