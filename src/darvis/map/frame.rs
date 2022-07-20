use chrono::{DateTime, Utc};

use crate::{
    map::{pose::Pose, map::Id}, 
    dvutils::*
};

use super::misc::IMUBias;

unsafe impl Sync for Frame {}

#[derive(Debug, Clone)]
pub struct Frame {
    pub id: Id,
    pub timestamp: DateTime<Utc>,
    pub key_points: opencv::types::VectorOfKeyPoint,
    pub descriptors: opencv::core::Mat,
    pub pose: Option<Pose>,
    pub imu_bias: Option<IMUBias>,
    pub reference_keyframe_id: Option<Id>,
    // depth_threshold: u64,
    // cam_params: opencv::core::Mat,
}

impl Frame {
    pub fn new(id: i32, timestamp: DateTime<Utc>, keypoints_vec: opencv::types::VectorOfKeyPoint, descriptors_vec: opencv::core::Mat) -> Self {
        let keypoints = keypoints_vec.cv_vector_of_keypoint();
        let descriptors = descriptors_vec.grayscale_to_cv_mat();

        let frame = Frame{
            id: id,
            timestamp: timestamp,
            key_points: keypoints,
            descriptors: descriptors,
            pose: None,
            imu_bias: None,
            reference_keyframe_id: None,

            // depth_threshold:
            // cam_params: 
        };

        // frame.calculate_pose();

        frame
    }

    // TODO: These functions moved out of tracking, but need to figure out what should be here and what should go back in tracking
    // Need to compare this current method of tracking to ORBSLAM3, probably the track reference keyframe function?

    // fn calculate_pose(&self, r: &Mat, t: &Mat) -> Pose {
    //     let mut pose = Pose::default_ones();
    //     pose.pos[0] = *t.at::<f64>(0).unwrap();
    //     pose.pos[1] = *t.at::<f64>(1).unwrap();
    //     pose.pos[2] = *t.at::<f64>(2).unwrap();

    //     pose.rot[(0,0)] = *r.at_2d::<f64>(0,0).unwrap();
    //     pose.rot[(0,1)] = *r.at_2d::<f64>(0,1).unwrap();
    //     pose.rot[(0,2)] = *r.at_2d::<f64>(0,2).unwrap();
    //     pose.rot[(1,0)] = *r.at_2d::<f64>(1,0).unwrap();
    //     pose.rot[(1,1)] = *r.at_2d::<f64>(1,1).unwrap();
    //     pose.rot[(1,2)] = *r.at_2d::<f64>(1,2).unwrap();
    //     pose.rot[(2,0)] = *r.at_2d::<f64>(2,0).unwrap();
    //     pose.rot[(2,1)] = *r.at_2d::<f64>(2,1).unwrap();
    //     pose.rot[(2,2)] = *r.at_2d::<f64>(2,2).unwrap();

    //     pose
    // }

    //     fn align(&mut self, _context: Context, msg: &Arc<TrackerMsg>) {
    //     // Convert back to cv structures
    //     let kp1 =  msg.frame.key_points;
    //     let des1 = msg.img1_des.descriptors;

    //     if !self.first_frame {
    //         let (p2f1, p2f2) = self.find_best_match(&des1, &self.prev_des, &kp1, &self.prev_kps);

    //         let (rotation, translation) = self.calculate_transform(&p2f1, &p2f2);
    //         let pose = self.get_pose(&rotation, &translation);

    //         // println!("{}", pose.pos);
    //         // println!("{}", pose.rot);

    //         let vis_id = msg.actor_ids.get(VISUALIZER).unwrap();
    //         vis_id.send_new(VisMsg::new(pose, msg.actor_ids.clone())).unwrap();
    //     } else {
    //         println!("First Image ");
    //         self.first_frame = false;
    //     }
    //     //println!("Tracking done!");
    //     self.prev_kps = kp1;
    //     self.prev_des = des1;
    // }


    // fn find_best_match(&self,
    //     des1 :&Mat,
    //     des2 :&Mat,
    //     kp1: &VectorOfKeyPoint, 
    //     kp2: & VectorOfKeyPoint
    // ) -> (opencv::types::VectorOfPoint2f, opencv::types::VectorOfPoint2f) {
    //     // BFMatcher to get good matches
    //     let bfmtch = BFMatcher::create(6 , true).unwrap(); 
    //     let mut mask = Mat::default(); 
    //     let mut matches = VectorOfDMatch::new();
    //     bfmtch.train_match(&des2, &des1, &mut matches, &mut mask).unwrap(); 

    //     // Sort the matches based on the distance in ascending order
    //     // Using O(n^2) sort here. Need to make the code use cv sort function
    //     // by providing custom comparator

    //     let mut sorted_matches = VectorOfDMatch::new();
    //     let mut added = vec![false; matches.len()];
    //     for i in 0.. matches.len() {
    //         if added[i] == true {
    //             continue;
    //         }
    //         let mut mn = i;
    //         let mut dist = matches.get(i).unwrap().distance;
    //         for j in 0..matches.len() {
    //             let dmatch2 = matches.get(j).unwrap();
    //             if dist > dmatch2.distance && !added[j] {
    //                 mn = j;
    //                 dist = dmatch2.distance;
    //             }
    //         }
    //         let dmatch1 = matches.get(mn).unwrap();
    //         sorted_matches.push(dmatch1);
    //         added[mn] = true;
    //     }

    //     // Point vectors to hold the corresponding matched points 
    //     let mut p2f1 = core::Vector::<core::Point2f>::new(); 
    //     let mut p2f2 = core::Vector::<core::Point2f>::new();

    //     for i in 0..sorted_matches.len(){
    //         p2f1.push(kp1.get(sorted_matches.get(i).unwrap().train_idx.try_into().unwrap()).unwrap().pt);
    //         p2f2.push(kp2.get(sorted_matches.get(i).unwrap().query_idx.try_into().unwrap()).unwrap().pt);

    //     }
    //     (p2f1, p2f2)
    // }

    // fn calculate_transform(
    //     &self,
    //     curr_features: &opencv::types::VectorOfPoint2f,
    //     prev_features: &opencv::types::VectorOfPoint2f,
    // ) -> (Mat, Mat) {
    //     //recovering the pose and the essential matrix
    //     let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
    //     let essential_mat = opencv::calib3d::find_essential_mat_2(
    //         &curr_features,
    //         &prev_features,
    //         718.8560, // self.focal,
    //         core::Point2d::new(607.1928, 185.2157) , //self.pp,
    //         opencv::calib3d::RANSAC,
    //         0.999,
    //         1.0,
    //         &mut mask,
    //     ).unwrap();
    //     opencv::calib3d::recover_pose(
    //         &essential_mat,
    //         &curr_features,
    //         &prev_features,
    //         &mut recover_r,
    //         &mut recover_t,
    //         718.8560, // self.focal,
    //         core::Point2d::new(607.1928, 185.2157) , //self.pp,
    //         &mut mask,
    //     ).unwrap();

    //     (recover_r, recover_t)
    // }

    // fn scale_transform(&self,
    //     scale: f64,
    //     rotation: &Mat,
    //     translation: &Mat,
    //     recover_mat: &Mat,
    //     t: &Mat,
    // ) -> (Mat, Mat) {
    //     //R = recover_mat, R_f= rotation, t_f =translation , t
    //     if scale > 0.1
    //         && (t.at::<f64>(2).unwrap() > t.at::<f64>(0).unwrap())
    //         && (t.at::<f64>(2).unwrap() > t.at::<f64>(1).unwrap())
    //     {
    //         // t_f = t_f + scale*(R_f*t);
    //         let mut rf_cross_t = opencv::core::mul_mat_mat(&rotation, &t).unwrap();
    //         rf_cross_t = opencv::core::mul_matexpr_f64(&rf_cross_t, scale).unwrap();
    //         let t_f = opencv::core::add_mat_matexpr(&translation, &rf_cross_t)
    //             .unwrap()
    //             .to_mat()
    //             .unwrap();

    //         // R_f = R*R_f;
    //         let r_f = opencv::core::mul_mat_mat(&recover_mat, &rotation)
    //             .unwrap()
    //             .to_mat()
    //             .unwrap();

    //         (r_f, t_f)
    //     } else {
    //         println!("scale below 0.1, or incorrect translation");
    //         (rotation.clone(), translation.clone())
    //     }
    // }

    // Safe to call these (instead of going through map) because
    // only the tracking thread has these data structures
    // when these functions are called
    #[allow(non_snake_case)]
    pub fn clean_VO_matches(&self) {
        // TODO
        // For each mappoint in mvpmappoints vector, delete it if the num of observations is < 1
        // First have to find out why frames even HAVE mvpmappoints vectors...
    }

    #[allow(non_snake_case)]
    pub fn delete_VO_matches_if_not_outliers(&self) {
        // TODO
        // for(int i=0; i<mCurrentFrame.N;i++)
        // {
        //     if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
        //         mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
        // }
    }
}