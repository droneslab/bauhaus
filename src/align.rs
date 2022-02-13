// Accepts a message of two image extracted data (kps, des), computes homography
use opencv::{
    prelude::*,
    core,
    features2d,
    features2d::{Feature2DTrait, BFMatcher},
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

        // BFMatcher to get good matches
        let bfmtch = BFMatcher::create(4, true).unwrap(); 
        let mut mask = Mat::default(); 
        let mut matches = VectorOfDMatch::new();
        bfmtch.train_match(&des2, &des1, &mut matches, &mut mask)?; 

        // Sort the matches based on the distance in ascending order
        // Using O(n^2) sort here. Need to make the code use cv sort function
        // by providing custom comparator

        let mut sortedMatches = VectorOfDMatch::new();
        let mut added = vec![false; matches.len()];
        for i in 0..matches.len() {
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
         
        // Find essential matrix using K
        let mut K = Mat::new_rows_cols_with_default(3, 3 ,CV_32FC1, opencv::core::Scalar::all(0.0)).unwrap();
        // Mat K = (Mat_<double>(3,3) << 718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1 );
        unsafe {
            *K.at_2d_unchecked_mut::<f32>(0,0).unwrap() = 718.8560;
            *K.at_2d_unchecked_mut::<f32>(0,1).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(0,2).unwrap() = 607.1928;
            *K.at_2d_unchecked_mut::<f32>(1,0).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(1,1).unwrap() = 718.8560;
            *K.at_2d_unchecked_mut::<f32>(1,2).unwrap() = 185.2157;
            *K.at_2d_unchecked_mut::<f32>(2,0).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(2,1).unwrap() = 0.0;
            *K.at_2d_unchecked_mut::<f32>(2,2).unwrap() = 1.0;
        }
        let mut ess_mat = calib3d::find_essential_mat_matrix(&mut p2f1, &mut p2f2, &mut K, calib3d::FM_RANSAC, 0.999, 1.0, &mut Mat::default()).unwrap();

        // Recover pose using the matrix
        let mut R = Mat::default();
        let mut t = Mat::default();
        let inliers = calib3d::recover_pose_camera(&mut ess_mat, &mut p2f1, &mut p2f2, &mut K, &mut R, &mut t, &mut Mat::default());
        // println!("Number of inliers:{:}", inliers.unwrap());
        // print_matrix(&R);
        // print_matrix(&t);

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
}


use crate::pluginfunction::Function;

impl Function for DarvisAlign {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.align(_context, message).unwrap();
        Ok(Status::done(()))
    }

}