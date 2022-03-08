// Accepts a message of two image extracted data (kps, des), computes homography
use opencv::{
    prelude::*,
    core,
    features2d,
    features2d::{Feature2DTrait, BFMatcher, FlannBasedMatcher},
    highgui,
    imgproc,
    videoio,
    imgcodecs,
    calib3d,
    types::{VectorOfKeyPoint, PtrOfBFMatcher, VectorOfDMatch},
};
use opencv::core::CV_32FC1;
use std::convert::TryInto;

use axiom::prelude::*;
use serde::{Deserialize, Serialize};
use crate::dvutils::*;
use crate::base::*;
use crate::vis::*;





// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct AlignMsg {
    // Vector of image paths to read in/extract
    img1_kps: DVVectorOfKeyPoint,
    img1_des: DVMatrixGrayscale,
    img2_kps: DVVectorOfKeyPoint,
    img2_des: DVMatrixGrayscale,
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl AlignMsg {
    pub fn new(kps1: DVVectorOfKeyPoint, des1: DVMatrixGrayscale, kps2: DVVectorOfKeyPoint, des2: DVMatrixGrayscale, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            img1_kps: kps1,
            img1_des: des1,
            img2_kps: kps2,
            img2_des: des2,
            actor_ids: ids,
        }
    }
}




#[derive(Debug, Clone)]
pub struct DarvisAlign;

impl DarvisAlign
{
    pub fn new() -> DarvisAlign {
        DarvisAlign {}
    }
    // This is the handler that will be used by the actor.
// This is the handler that will be used by the actor.
pub fn align(&mut self, context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<AlignMsg>() {
        // Convert back to cv structures
        let kp1 =  msg.img1_kps.cv_vector_of_keypoint();
        let des1 = msg.img1_des.grayscale_to_cv_mat();
        let kp2 = msg.img2_kps.cv_vector_of_keypoint();
        let des2 = msg.img2_des.grayscale_to_cv_mat();

        
        let (p2f1, p2f2) =self.find_best_match(&des1, &des2, &kp1, &kp2);

        //println!("{:.2}", ((p2f1.len() as f64)/(kp1.len() as f64)));
        let (R, t) = self.calculate_transform(&p2f1, &p2f2);


        let mut pose = Pose::default_ones();
        pose.pos[0] = *t.at::<f64>(0).unwrap();
        pose.pos[1] = *t.at::<f64>(1).unwrap();
        pose.pos[2] = *t.at::<f64>(2).unwrap();

        pose.rot[(0,0)] = *R.at_2d::<f64>(0,0).unwrap();
        pose.rot[(0,1)] = *R.at_2d::<f64>(0,1).unwrap();
        pose.rot[(0,2)] = *R.at_2d::<f64>(0,2).unwrap();
        pose.rot[(1,0)] = *R.at_2d::<f64>(1,0).unwrap();
        pose.rot[(1,1)] = *R.at_2d::<f64>(1,1).unwrap();
        pose.rot[(1,2)] = *R.at_2d::<f64>(1,2).unwrap();
        pose.rot[(2,0)] = *R.at_2d::<f64>(2,0).unwrap();
        pose.rot[(2,1)] = *R.at_2d::<f64>(2,1).unwrap();
        pose.rot[(2,2)] = *R.at_2d::<f64>(2,2).unwrap();


        // println!("{}", pose.pos);
        // println!("{}", pose.rot);

        //let vis_id = context.system.find_aid_by_name("visulization").unwrap();
        let vis_id = msg.actor_ids.get("visulization").unwrap();
        vis_id.send_new(VisMsg::new(pose, msg.actor_ids.clone())).unwrap();
    }
    Ok(Status::done(()))
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

  

}


use crate::pluginfunction::Function;

impl Function for DarvisAlign {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.align(_context, message).unwrap();
        Ok(Status::done(()))
    }

}