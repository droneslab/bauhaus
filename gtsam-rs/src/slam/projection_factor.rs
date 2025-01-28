use cxx::{SharedPtr, UniquePtr};

use crate::{geometry::{cal3_s2::Cal3S2, point2::Point2, pose3::Pose3}, inference::key::IntoKey, linear::noise_model::IsotropicNoiseModel};

pub struct SmartProjectionPoseFactorCal3S2 {
    pub(crate) inner: UniquePtr<::sys::SmartProjectionPoseFactorCal3_S2>,
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

    pub fn add(
        self: &mut Self,
        point: &Point2,
        key: impl IntoKey
    ) {
        ::sys::add(
            self.inner.pin_mut(),
            &point.inner,
            key.into_key(),
        )
    }
}