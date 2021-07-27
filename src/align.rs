// Accepts a message of two image extracted data (kps, des), computes homography
#[allow(unused_imports)]
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
extern crate nalgebra as na;
use na::*;
use axiom::prelude::*;
use serde::{Deserialize, Serialize};
use crate::utils::*;

// Message type for this actor
#[derive(Debug, Serialize, Deserialize)]
pub struct AlignMsg {
    // Vector of image paths to read in/extract
    img1_kps: Vec<DmatKeyPoint>,
    img1_des: na::DMatrix<u8>,
    img2_kps: Vec<DmatKeyPoint>,
    img2_des: na::DMatrix<u8>,
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl AlignMsg {
    pub fn new(kps1: Vec<DmatKeyPoint>, des1: DMatrix<u8>, kps2: Vec<DmatKeyPoint>, des2: na::DMatrix<u8>, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            img1_kps: kps1,
            img1_des: des1,
            img2_kps: kps2,
            img2_des: des2,
            actor_ids: ids,
        }
    }
}

// This is the handler that will be used by the actor.
pub async fn align(_: (), context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<AlignMsg>() {
        println!("{:?}", context);

        // Convert back to cv structures
        let mut kp1 = na_keypoint_to_cv_vector_of_keypoint(&msg.img1_kps);
        let mut des1 = na_grayscale_to_cv_mat(&msg.img1_des);
        let mut kp2 = na_keypoint_to_cv_vector_of_keypoint(&msg.img2_kps);
        let mut des2 = na_grayscale_to_cv_mat(&msg.img2_des);

        // BFMatcher to get good matches
        let mut bfmtch = BFMatcher::create(features2d::DescriptorMatcher_BRUTEFORCE_HAMMING, true).unwrap(); 
        let mut mask = core::Mat::default(); 
        let mut matches = VectorOfDMatch::new();
        bfmtch.train_match(&des2, &des1, &mut matches, &mut mask); 

        // Sort the matches based on the distance in ascending order
        // Using O(n^2) sort here. Need to make the code use cv sort function
        // by providing custom comparator

        let mut sortedMatches = VectorOfDMatch::new();
        let mut added = vec![false; matches.len()];
        for i in 0..matches.len() {
            if(added[i] == true) {
                continue;
            }
            let mut mn = i;
            let mut dist = matches.get(i).unwrap().distance;
            for j in 0..matches.len() {
                let dmatch2 = matches.get(j).unwrap();
                if(dist > dmatch2.distance && !added[j]) {
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
        let mut ess_mat = calib3d::find_essential_mat_matrix(&mut p2f1, &mut p2f2, &mut K, calib3d::CV_RANSAC, 0.999, 1.0, &mut Mat::default()).unwrap();

        // Recover pose using the matrix
        let mut R = Mat::default();
        let mut t = Mat::default();
        let mut thresh: f32 = 1.0;
        let inliers = calib3d::recover_pose_camera(&mut ess_mat, &mut p2f1, &mut p2f2, &mut K, &mut R, &mut t, &mut Mat::default());
        println!("Number of inliers:{:}", inliers.unwrap());
        print_matrix(&R);
        print_matrix(&t);

        //TODO: convert R,t to NA matrices and make Pose object, send to vis
        
    }
    Ok(Status::done(()))
}
