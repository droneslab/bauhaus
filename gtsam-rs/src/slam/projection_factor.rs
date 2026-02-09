use cxx::SharedPtr;

use crate::{geometry::{cal3_s2::{Cal3S2, Cal3S2Stereo}, point2::{Point2, StereoPoint2}, pose3::Pose3}, inference::key::IntoKey, linear::noise_model::IsotropicNoiseModel};

pub struct SmartProjectionPoseFactorCal3S2 {
    pub(crate) inner: SharedPtr<::sys::SmartProjectionPoseFactorCal3_S2>,
}

impl SmartProjectionPoseFactorCal3S2 {
    pub fn new(
        measurement_noise: &IsotropicNoiseModel,
        k: &Cal3S2,
        sensor_p_body: &Pose3,
    ) -> Self {
        Self {
            inner: ::sys::new_smart_projection_pose_factor(
                &measurement_noise.inner,
                &k.inner,
                &sensor_p_body.inner,
            ),
        }
    }

    pub fn copy(&self) -> Self {
        Self {
            inner: ::sys::clone_smart_projection_pose_factor(&self.inner)
        }
    }

    pub fn add(
        self: &mut Self,
        point: &Point2,
        key: impl IntoKey
    ) {
        ::sys::add_smart(
            &mut self.inner,
            &point.inner,
            key.into_key(),
        )
    }
}

pub struct SmartStereoProjectionPoseFactor {
    pub(crate) inner: SharedPtr<::sys::SmartStereoProjectionPoseFactor>,
}

impl SmartStereoProjectionPoseFactor {
    pub fn new(
        measurement_noise: &IsotropicNoiseModel,
        sensor_p_body: &Pose3,
    ) -> Self {
        Self {
            inner: ::sys::new_smart_stereo_projection_pose_factor(
                &measurement_noise.to_base_model().inner,
                &sensor_p_body.inner,
            ),
        }
    }

    pub fn copy(&self) -> Self {
        Self {
            inner: ::sys::clone_smart_stereo_projection_pose_factor(&self.inner)
        }
    }

    pub fn add(
        self: &mut Self,
        point: &StereoPoint2,
        key: impl IntoKey,
        k: &Cal3S2Stereo,
    ) {
        ::sys::add_smartstereo(
            &mut self.inner,
            &point.inner,
            key.into_key(),
            &k.inner,
        )
    }
}



pub struct GenericProjectionFactorPose3Point3Cal3S2 {
    pub(crate) inner: SharedPtr<::sys::GenericProjectionFactorPose3Point3Cal3_S2>,
}

impl GenericProjectionFactorPose3Point3Cal3S2 {
    pub fn new(
        point: &Point2,
        measurement_noise: &IsotropicNoiseModel,
        pose_key: impl IntoKey,
        point_key: impl IntoKey,
        k: &Cal3S2,
        sensor_p_body: &Pose3,
    ) -> Self {
        // From gtsam:
            // measured	is the 2 dimensional location of point in image (the measurement)
            // model	is the standard deviation
            // poseKey	is the index of the camera
            // pointKey	is the index of the landmark
            // K	shared pointer to the constant calibration
            // body_P_sensor	is the transform from body to sensor frame (default identity)
        Self {
            inner: ::sys::new_generic_projection_factor(
                &point.inner,
                &measurement_noise.inner,
                pose_key.into_key(),
                point_key.into_key(),
                &k.inner,
                &sensor_p_body.inner,
            ),
        }
    }
}