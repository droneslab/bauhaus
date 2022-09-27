use opencv::{
    types::{VectorOfKeyPoint, VectorOfPoint3f},
};
use std::collections::HashMap;
use crate::map::{pose::Pose, map::Id};
use crate::utils::twoviewreconstruction::TwoViewReconstruction;

#[derive(Debug, Clone)]
pub enum CameraType {
    CamPinhole
}

#[derive(Debug, Clone)]
pub struct CalibParam {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub inv_fx: f64,
    pub inv_fy: f64,
    pub bf: f64,
    pub baseline: f64,
    pub th_depth: i32,
}

#[derive(Debug, Clone)]
pub struct DVCamera {
    pub cam_type: CameraType,
    pub calibration_params: CalibParam,
    tvr: Option<TwoViewReconstruction>,
}

impl DVCamera {
    pub fn default(fx: f64, fy: f64, cx: f64, cy: f64, th_depth: i32, bf: f64) -> DVCamera {
        let inv_fx = 1.0 / fx;
        let inv_fy = 1.0 / fy;
        let baseline = bf / fx;
        let calibration_params = CalibParam {
            fx, fy, cx, cy, inv_fx, inv_fy, bf, baseline, th_depth
        };

        DVCamera {
            cam_type: CameraType::CamPinhole,
            calibration_params: calibration_params,
            tvr: None,
        }
    }

    pub fn new(fx: f64, fy: f64, cx: f64, cy: f64, th_depth: i32, bf: f64) -> DVCamera {
        let inv_fx = 1.0 / fx;
        let inv_fy = 1.0 / fy;
        let baseline = bf / fx;
        let calibration_params = CalibParam {
            fx, fy, cx, cy, inv_fx, inv_fy, bf, baseline, th_depth
        };

        DVCamera {
            cam_type: CameraType::CamPinhole,
            calibration_params: calibration_params,
            tvr: None,
        }
    }

    pub fn reconstruct_with_two_views(
        &mut self, 
        vKeys1: &VectorOfKeyPoint, 
        vKeys2: &VectorOfKeyPoint,
        vMatches12: &HashMap<i32, Id>,
        T21: &mut Pose,
        vP3D: &mut VectorOfPoint3f,
        vbTriangulated: &mut Vec<bool>
    ) -> bool {
        if self.tvr.is_none() {
            self.tvr = Some(TwoViewReconstruction::default());
        }
        return self.tvr.clone().unwrap().reconstruct(vKeys1,vKeys2,vMatches12,T21, vP3D,vbTriangulated);
    }
}

