// Accepts a message of two image extracted data (kps, des), computes homography
use opencv::{
    prelude::*,
    core,
    features2d::{BFMatcher, FlannBasedMatcher},
    types::{VectorOfKeyPoint, VectorOfDMatch, VectorOfPoint2f},
};
use opencv::core::CV_32FC1;
use std::convert::TryInto;

use axiom::prelude::*;
use serde::{Deserialize, Serialize};
use crate::dvutils::*;
use crate::base::*;
use crate::vis::*;
use crate::config::*;



// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct TrackerMsgKLT {
    // Vector of image paths to read in/extract
    pub img1: DVMatrixGrayscale,
    pub img1_kps: DVVectorOfKeyPoint,
    pub img1_des: DVMatrixGrayscale,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl TrackerMsgKLT {
    pub fn new(image1: DVMatrixGrayscale, kps1: DVVectorOfKeyPoint, des1: DVMatrixGrayscale, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            img1 : image1,
            img1_kps: kps1,
            img1_des: des1,
            actor_ids: ids,
        }
    }
}




#[derive(Debug, Clone)]
pub struct DarvisTrackerKLT
{
    prev_frame: Mat,
    prev_kps: VectorOfKeyPoint,
    prev_des: Mat,
    prev_points: VectorOfPoint2f,
    first_frame: bool,
    min_num_feat: i32
}

impl DarvisTrackerKLT
{
    pub fn new() -> DarvisTrackerKLT {
      DarvisTrackerKLT {
            prev_frame : Mat::default(),
            prev_kps: VectorOfKeyPoint::new(),
            prev_des: Mat::default(),
            prev_points: VectorOfPoint2f::new(),
            first_frame: true,
            min_num_feat: 2000            
        }
    }
    // This is the handler that will be used by the actor.
// This is the handler that will be used by the actor.
pub fn align(&mut self, _context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<TrackerMsgKLT>() {
        // Convert back to cv structures

        let img1 = msg.img1.grayscale_to_cv_mat();
        let kp1 =  msg.img1_kps.cv_vector_of_keypoint();
        let des1 = msg.img1_des.grayscale_to_cv_mat();
        

        if !self.first_frame
        {
            
            let mut prev_points = VectorOfPoint2f::from_iter(&self.prev_points);

            let curr_features = self.track_features(&self.prev_frame, &img1, &mut prev_points);
            
            let (rotation, translation) = self.calculate_transform(&curr_features, &mut prev_points);



            // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
            if prev_points.len() < self.min_num_feat as usize 
            {
                println!("Current Feature count {:?}", curr_features.len());
                let pt_indx = opencv::types::VectorOfi32::new();
                core::KeyPoint::convert(&self.prev_kps, &mut prev_points, &pt_indx).unwrap();
            }

            self.prev_points = prev_points;
            let pose = self.get_pose(&rotation, &translation);


            let vis_id = msg.actor_ids.get(VISUALIZER).unwrap();
            vis_id.send_new(VisMsg::new(pose, msg.actor_ids.clone())).unwrap();
            
        }
        else
        {
            self.first_frame = false;
            let pt_indx = opencv::types::VectorOfi32::new();
            let mut prev_points = VectorOfPoint2f::new();
            core::KeyPoint::convert(&kp1, &mut prev_points, &pt_indx).unwrap();

            self.prev_points = prev_points;
        }
        //println!("Tracking done!");
        self.prev_frame = img1;
        self.prev_kps = kp1;
        self.prev_des = des1;
    }
    Ok(Status::done(()))
}

pub fn get_pose(&self, r: &Mat, t: &Mat) -> Pose
{
    let mut pose = Pose::default_ones();
    pose.pos[0] = *t.at::<f64>(0).unwrap();
    pose.pos[1] = *t.at::<f64>(1).unwrap();
    pose.pos[2] = *t.at::<f64>(2).unwrap();

    pose.rot[(0,0)] = *r.at_2d::<f64>(0,0).unwrap();
    pose.rot[(0,1)] = *r.at_2d::<f64>(0,1).unwrap();
    pose.rot[(0,2)] = *r.at_2d::<f64>(0,2).unwrap();
    pose.rot[(1,0)] = *r.at_2d::<f64>(1,0).unwrap();
    pose.rot[(1,1)] = *r.at_2d::<f64>(1,1).unwrap();
    pose.rot[(1,2)] = *r.at_2d::<f64>(1,2).unwrap();
    pose.rot[(2,0)] = *r.at_2d::<f64>(2,0).unwrap();
    pose.rot[(2,1)] = *r.at_2d::<f64>(2,1).unwrap();
    pose.rot[(2,2)] = *r.at_2d::<f64>(2,2).unwrap();

    pose
}


pub fn calculate_transform(
  &self,
  curr_features: &opencv::types::VectorOfPoint2f,
  prev_features: &opencv::types::VectorOfPoint2f,
) -> (Mat, Mat) {
  //recovering the pose and the essential matrix
  let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
  let essential_mat = opencv::calib3d::find_essential_mat_2(
    &curr_features,
    &prev_features,
    718.8560, // self.focal,
    core::Point2d::new(607.1928, 185.2157) , //self.pp,
    opencv::calib3d::RANSAC,
    0.999,
    1.0,
    &mut mask,
  )
  .unwrap();
  opencv::calib3d::recover_pose(
    &essential_mat,
    &curr_features,
    &prev_features,
    &mut recover_r,
    &mut recover_t,
    718.8560, // self.focal,
    core::Point2d::new(607.1928, 185.2157) , //self.pp,
    &mut mask,
  )
  .unwrap();

  (recover_r, recover_t)
}

pub fn scale_transform(&self,
    scale: f64,
    rotation: &Mat,
    translation: &Mat,
    recover_mat: &Mat,
    t: &Mat,
  ) -> (Mat, Mat) {
    //R = recover_mat, R_f= rotation, t_f =translation , t
    if scale > 0.1
      && (t.at::<f64>(2).unwrap() > t.at::<f64>(0).unwrap())
      && (t.at::<f64>(2).unwrap() > t.at::<f64>(1).unwrap())
    {
      // t_f = t_f + scale*(R_f*t);
      let mut rf_cross_t = opencv::core::mul_mat_mat(&rotation, &t).unwrap();
      rf_cross_t = opencv::core::mul_matexpr_f64(&rf_cross_t, scale).unwrap();
      let t_f = opencv::core::add_mat_matexpr(&translation, &rf_cross_t)
        .unwrap()
        .to_mat()
        .unwrap();
  
      // R_f = R*R_f;
      let r_f = opencv::core::mul_mat_mat(&recover_mat, &rotation)
        .unwrap()
        .to_mat()
        .unwrap();
  
      (r_f, t_f)
    } else {
      println!("scale below 0.1, or incorrect translation");
      (rotation.clone(), translation.clone())
    }
  }

  pub fn track_features(
    &self,
    src_image: &Mat,
    dst_image: &Mat,
    src_image_points: &mut VectorOfPoint2f,
  ) -> VectorOfPoint2f {
    let mut status = opencv::types::VectorOfu8::new();
    let mut dst_image_points = VectorOfPoint2f::new(); //vectors to store the coordinates of the feature points
    self.feature_tracking(
      &src_image,
      &dst_image,
      src_image_points,
      &mut dst_image_points,
      &mut status,
    ); //track those features to img_2
    dst_image_points
  }

  pub fn feature_tracking(
    &self,
    img_1: &opencv::core::Mat,
    img_2: &opencv::core::Mat,
    points1: &mut VectorOfPoint2f,
    points2: &mut VectorOfPoint2f,
    status: &mut opencv::types::VectorOfu8,
  ) {
    //this function automatically gets rid of points for which tracking fails
    let mut err = opencv::types::VectorOff32::default();
    let win_size = core::Size::new(21, 21);
    let termcrit = opencv::core::TermCriteria {
      typ: 3,
      max_count: 30,
      epsilon: 0.01,
    };
    let max_level = 3;
    opencv::video::calc_optical_flow_pyr_lk(
      img_1, img_2, points1, points2, status, &mut err, win_size, max_level, termcrit, 0, 0.001,
    )
    .unwrap();
    //getting rid of points for which the KLT tracking failed or those who have gone outside the framed
    let mut indez_correction = 0;
    for i in 0..status.len() {
      let pt = points2.get(i - indez_correction).unwrap();
      if (status.get(i).unwrap() == 0) || pt.x < 0.0 || pt.y < 0.0 {
        if pt.x < 0.0 || pt.y < 0.0 {
          status.set(i, 0).unwrap();
        }
        points1.remove(i - indez_correction).unwrap();
        points2.remove(i - indez_correction).unwrap();
        indez_correction = indez_correction + 1;
      }
    }
  }
}


use crate::pluginfunction::Function;

impl Function for DarvisTrackerKLT {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.align(_context, message).unwrap();
        Ok(Status::done(()))
    }

}