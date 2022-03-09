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
use crate::actornames::*;


// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct TrackerMsg {
    // Vector of image paths to read in/extract
    img1: DVMatrixGrayscale,
    img1_kps: DVVectorOfKeyPoint,
    img1_des: DVMatrixGrayscale,
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl TrackerMsg {
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
pub struct DarvisTracker
{
    prev_frame: Mat,
    prev_kps: VectorOfKeyPoint,
    prev_des: Mat,
    first_frame: bool
}

impl DarvisTracker
{
    pub fn new() -> DarvisTracker {
        DarvisTracker {
            prev_frame : Mat::default(),
            prev_kps: VectorOfKeyPoint::new(),
            prev_des: Mat::default(),
            first_frame: true            
        }
    }
    // This is the handler that will be used by the actor.
// This is the handler that will be used by the actor.
pub fn align(&mut self, _context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<TrackerMsg>() {
        // Convert back to cv structures

        let img1 = msg.img1.grayscale_to_cv_mat();
        let kp1 =  msg.img1_kps.cv_vector_of_keypoint();
        let des1 = msg.img1_des.grayscale_to_cv_mat();
        

        if !self.first_frame
        {
            let pt_indx = opencv::types::VectorOfi32::new();
            let mut prev_points = VectorOfPoint2f::new();
            core::KeyPoint::convert(&self.prev_kps, &mut prev_points, &pt_indx).unwrap();

            let curr_features = self.track_features(&self.prev_frame, &img1, &mut prev_points);

            println!("Current Feature count {:?}", curr_features.len());
            let (rotation, translation) = self.calculate_transform(&curr_features, &mut prev_points);


            // //println!("Tracking ");
            // let (p2f1, p2f2) =self.find_best_match(&des1, &self.prev_des, &kp1, &self.prev_kps);

            // //println!("{:.2}", ((p2f1.len() as f64)/(kp1.len() as f64)));
            // let (R, t) = self.calculate_transform(&p2f1, &p2f2);

            let pose = self.get_pose(&rotation, &translation);

            // println!("{}", pose.pos);
            // println!("{}", pose.rot);

            let vis_id = msg.actor_ids.get(VISUALIZER).unwrap();
            vis_id.send_new(VisMsg::new(pose, msg.actor_ids.clone())).unwrap();
            
        }
        else
        {
            println!("First Image ");
            self.first_frame = false;
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

pub fn find_best_match(&self,
     des1 :&Mat,
    des2 :&Mat,
    kp1: &VectorOfKeyPoint, 
    kp2: & VectorOfKeyPoint) -> 
    (opencv::types::VectorOfPoint2f, opencv::types::VectorOfPoint2f)
{
        // BFMatcher to get good matches
        let bfmtch = BFMatcher::create(6 , true).unwrap(); 
        let mut mask = Mat::default(); 
        let mut matches = VectorOfDMatch::new();
        bfmtch.train_match(&des2, &des1, &mut matches, &mut mask).unwrap(); 

        // Sort the matches based on the distance in ascending order
        // Using O(n^2) sort here. Need to make the code use cv sort function
        // by providing custom comparator

        let mut sortedMatches = VectorOfDMatch::new();
        let mut added = vec![false; matches.len()];
        for i in 0.. matches.len() {
            if added[i] == true {
                continue;
            }
            let mut mn = i;
            let mut dist = matches.get(i).unwrap().distance;
            for j in 0..matches.len() {
                let dmatch2 = matches.get(j).unwrap();
                if dist > dmatch2.distance && !added[j] {
                    mn = j;
                    dist = dmatch2.distance;
                }        
            }      
            let dmatch1 = matches.get(mn).unwrap();
            sortedMatches.push(dmatch1);
            added[mn] = true;
        }

        // Point vectors to hold the corresponding matched points 
        let mut p2f1 = core::Vector::<core::Point2f>::new(); 
        let mut p2f2 = core::Vector::<core::Point2f>::new();

        for i in 0..sortedMatches.len(){
            p2f1.push(kp1.get(sortedMatches.get(i).unwrap().train_idx.try_into().unwrap()).unwrap().pt);
            p2f2.push(kp2.get(sortedMatches.get(i).unwrap().query_idx.try_into().unwrap()).unwrap().pt);

        }
        (p2f1, p2f2)
}

pub fn find_best_match_flann(&self,
    des1 :&Mat,
   des2 :&Mat,
   kp1: &VectorOfKeyPoint, 
   kp2: & VectorOfKeyPoint) -> 
   (opencv::types::VectorOfPoint2f, opencv::types::VectorOfPoint2f)
{
       // BFMatcher to get good matches
       let bfmtch = FlannBasedMatcher::create().unwrap(); 
       let mut mask = Mat::default(); 
       let mut knn_matches = opencv::types::VectorOfVectorOfDMatch::new();

       //fn knn_train_match(&self, query_descriptors: &dyn core::ToInputArray, 
        // train_descriptors: &dyn core::ToInputArray,
        //  matches: &mut core::Vector::<core::Vector::<core::DMatch>>, 
        //  k: i32, 
        //  mask: &dyn core::ToInputArray, 
        //  compact_result: bool) 
       let k=3;
       let compact_result=true;
       let mut des1_out = Mat::default();
       des1.convert_to(&mut des1_out, CV_32FC1, 1.0,0.0).unwrap();
       let mut des2_out = Mat::default();
       des2.convert_to(&mut des2_out, CV_32FC1, 1.0,0.0).unwrap();
       bfmtch.knn_train_match(&des2_out, &des1_out, &mut knn_matches, k, &mut mask, compact_result).unwrap(); 


           //-- Filter matches using the Lowe's ratio test
    let ratio_thresh = 0.7;
    let mut matches = VectorOfDMatch::new();
    for i in 0..knn_matches.len()
    {
        if knn_matches.get(i).unwrap().get(0).unwrap().distance < ratio_thresh * knn_matches.get(i).unwrap().get(1).unwrap().distance
        {
            matches.push(knn_matches.get(i).unwrap().get(0).unwrap());
        }
    }
        
       // Sort the matches based on the distance in ascending order
       // Using O(n^2) sort here. Need to make the code use cv sort function
       // by providing custom comparator

       let mut sortedMatches = VectorOfDMatch::new();
       let mut added = vec![false; matches.len()];
       for i in 0.. matches.len() {
           if added[i] == true {
               continue;
           }
           let mut mn = i;
           let mut dist = matches.get(i).unwrap().distance;
           for j in 0..matches.len() {
               let dmatch2 = matches.get(j).unwrap();
               if dist > dmatch2.distance && !added[j] {
                   mn = j;
                   dist = dmatch2.distance;
               }        
           }      
           let dmatch1 = matches.get(mn).unwrap();
           sortedMatches.push(dmatch1);
           added[mn] = true;
       }

       // Point vectors to hold the corresponding matched points 
       let mut p2f1 = core::Vector::<core::Point2f>::new(); 
       let mut p2f2 = core::Vector::<core::Point2f>::new();

       for i in 0..sortedMatches.len() {
           p2f1.push(kp1.get(sortedMatches.get(i).unwrap().train_idx.try_into().unwrap()).unwrap().pt);
           p2f2.push(kp2.get(sortedMatches.get(i).unwrap().query_idx.try_into().unwrap()).unwrap().pt);

       }
       (p2f1, p2f2)
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

impl Function for DarvisTracker {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.align(_context, message).unwrap();
        Ok(Status::done(()))
    }

}