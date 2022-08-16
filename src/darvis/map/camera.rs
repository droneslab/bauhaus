use opencv::{
    prelude::*, types::{VectorOfKeyPoint, VectorOfPoint3f},
};

use super::{pose::Pose, twoviewreconstruction::TwoViewReconstruction};



#[derive(Debug, Clone)]
pub enum CameraType {
    CAM_PINHOLE
}

#[derive(Debug, Clone)]
pub struct DVCamera {
    pub parameters: Vec<f32>,
    pub cam_type: CameraType,
    //id: u32,
    tvr: Option<TwoViewReconstruction>,
}

impl DVCamera {
    pub fn default() -> DVCamera {
        DVCamera {
            parameters : vec![0.0;4],
            cam_type: CameraType::CAM_PINHOLE,
            //id: 
            tvr: None,
        }
    }

    pub fn new(parameter : &Vec<f32>) -> DVCamera {
        assert!(parameter.len() ==4);
        DVCamera {
            parameters : parameter.clone(),
            cam_type: CameraType::CAM_PINHOLE,
            //id: 
            tvr: None,
        }
    }

    //bool Pinhole::ReconstructWithTwoViews(const std::vector<cv::KeyPoint>& vKeys1, const std::vector<cv::KeyPoint>& vKeys2, const std::vector<int> &vMatches12, Sophus::SE3f &T21, std::vector<cv::Point3f> &vP3D, std::vector<bool> &vbTriangulated)
    pub fn reconstruct_with_two_views(&mut self, vKeys1: &VectorOfKeyPoint, vKeys2: &VectorOfKeyPoint,  vMatches12: &Vec<i32>, T21: &mut Pose, vP3D: &mut VectorOfPoint3f, vbTriangulated: &mut Vec<bool>) -> bool
    {
        if self.tvr.is_none()
        {
            // Eigen::Matrix3f K = this->toK_();
            // self.to
            // tvr = new TwoViewReconstruction(K);
            self.tvr = Some(TwoViewReconstruction::default());
        }

        return self.tvr.clone().unwrap().reconstruct(vKeys1,vKeys2,vMatches12,T21, vP3D,vbTriangulated);
}


// cv::Mat Pinhole::toK() {
// cv::Mat K = (cv::Mat_<float>(3, 3)
// << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f);
// return K;
// }

// Eigen::Matrix3f Pinhole::toK_() {
// Eigen::Matrix3f K;
// K << mvParameters[0], 0.f, mvParameters[2], 0.f, mvParameters[1], mvParameters[3], 0.f, 0.f, 1.f;
// return K;
// }


}

