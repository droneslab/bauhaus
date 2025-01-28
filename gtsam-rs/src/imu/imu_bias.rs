use cxx::UniquePtr;

use crate::base::vector::{Vector3, Vector3Ref};

pub struct ConstantBias {
    pub(crate) inner: UniquePtr<::sys::ConstantBias>,
}

impl Default for ConstantBias {
    fn default() -> Self {
        Self {
            inner: ::sys::default_constantbias(),
        }
    }
}

impl ConstantBias {
    pub fn new(
        accel_bias: &Vector3, gyro_bias: &Vector3
    ) -> Self {
        Self {
            inner: ::sys::new_constantbias(&accel_bias.inner, &gyro_bias.inner),
        }
    }
}


pub struct ConstantBiasRef<'a> {
    pub(crate) inner: &'a ::sys::ConstantBias,
}

impl ConstantBiasRef<'_> {
    pub fn accel_bias(&self) -> Vector3Ref {
        Vector3Ref {
            inner: ::sys::accel_bias(&self.inner)
        }
    }

    pub fn gyro_bias(&self) -> Vector3Ref {
        Vector3Ref {
            inner: ::sys::gyro_bias(&self.inner)
        }
    }
}

impl<'a> From<ConstantBiasRef<'a>> for ConstantBias {
    fn from(value: ConstantBiasRef<'a>) -> Self {
        Self::new(& value.accel_bias().into(), & value.gyro_bias().into())
    }
}