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
pub struct DVCamera {
    pub parameters: Vec<f32>,
    pub cam_type: CameraType,
    tvr: Option<TwoViewReconstruction>,
}

impl DVCamera {
    pub fn default() -> DVCamera {
        DVCamera {
            parameters : vec![0.0;4],
            cam_type: CameraType::CamPinhole,
            tvr: None,
        }
    }

    pub fn new(parameter : &Vec<f32>) -> DVCamera {
        assert!(parameter.len() ==4);
        DVCamera {
            parameters : parameter.clone(),
            cam_type: CameraType::CamPinhole,
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

