// Accepts a message of two image extracted data (kps, des), computes homography
use opencv::{
    prelude::*,
    core,
    features2d::{BFMatcher, FlannBasedMatcher},
    types::{VectorOfKeyPoint, VectorOfDMatch, VectorOfPoint2f,VectorOfMat, VectorOfPoint3f}
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
pub struct TriangulationMsg {
    pub pts1: DVVectorOfPoint2d,
    pub pts2: DVVectorOfPoint2d,
    pub pose: Pose,
    pub kp_match_indices: DVVectorOfMatchPair,
    pub actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl TriangulationMsg {
    pub fn new(pt1: DVVectorOfPoint2d, pt2: DVVectorOfPoint2d, cam_pos : Pose, match_indices: DVVectorOfMatchPair, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            pts1: pt1,
            pts2: pt2,
            pose: cam_pos,
            kp_match_indices: match_indices,
            actor_ids: ids,
        }
    }
}




#[derive(Debug, Clone)]
pub struct DarvisTriangulator
{
    prev_poses: core::Vector::<Mat>,
    prev_points: core::Vector::<VectorOfPoint2f>,
    traj_img: Mat,
    P_nk: Mat,
    prev_points3d: core::Vector::<VectorOfPoint3f>,
    T: Mat,
    N: i32,
    n: i32,

    traj_pos: DVVector3, 
    /// Built up trajectory rotation
    traj_rot: DVMatrix3, 
    frame_idx: i32,
    kp_match_indices: DVVectorOfMatchPair,
    all_prev_points: DVVectorOfPoint2d,
    prev_poses_mat: VectorOfMat,

}

impl DarvisTriangulator
{
    pub fn new() -> DarvisTriangulator {
        DarvisTriangulator {
            prev_poses: core::Vector::<Mat>::default(),
            prev_points: core::Vector::<VectorOfPoint2f>::new(),
            traj_img: Mat::new_rows_cols_with_default(900, 900, core::CV_8UC3, core::Scalar::all(0.0)).unwrap(),
            P_nk: Mat::eye(3, 4, CV_32FC1).unwrap().to_mat().unwrap(),
            prev_points3d: core::Vector::<VectorOfPoint3f>::new(),
            T: Mat::eye(4, 4, CV_32FC1).unwrap().to_mat().unwrap(),
            N: 3,
            n: 3,
            
            traj_pos: DVVector3::zeros(),
            traj_rot: DVMatrix3::zeros(),
            frame_idx:0,
            kp_match_indices: DVVectorOfMatchPair::from_element(0,0),
            all_prev_points : DVVectorOfPoint2d::from_element(0,2,0.0),
            prev_poses_mat: VectorOfMat::new()
        }
    }



pub fn get_pose(&self, r: &DVMatrix3, t: &DVVector3) -> Pose
{
    let mut pose = Pose::default_ones();
    pose.pos[0] = t[(0)];
    pose.pos[1] = t[(1)];
    pose.pos[2] = t[(2)];

    pose.rot[(0,0)] = r[(0,0)];
    pose.rot[(0,1)] = r[(0,1)];
    pose.rot[(0,2)] = r[(0,2)];
    pose.rot[(1,0)] = r[(1,0)];
    pose.rot[(1,1)] = r[(1,1)];
    pose.rot[(1,2)] = r[(1,2)];
    pose.rot[(2,0)] = r[(2,0)];
    pose.rot[(2,1)] = r[(2,1)];
    pose.rot[(2,2)] = r[(2,2)];

    pose
}



   // This is the handler that will be used by the actor.
// This is the handler that will be used by the actor.
pub fn triangulate(&mut self, _context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<TriangulationMsg>() {

        if self.traj_pos == DVVector3::zeros() && self.traj_rot == DVMatrix3::zeros() {
            self.traj_pos = msg.pose.pos;
            self.traj_rot = msg.pose.rot;
            return Ok(Status::done(()));
        }

        // Convert back to cv structures

        //https://python.hotexamples.com/examples/cv2/-/triangulatePoints/python-triangulatepoints-function-examples.html
        // '''
        // Takes pts1, pts2, k, r, t
        // Returns triangulated points using openCV implementation
        // '''
        // proj1 = np.array([[1, 0, 0, 0],
        //                   [0, 1, 0, 0],
        //                   [0, 0, 1, 0]])
        // proj2 = np.append(r, t, 1)
        // pts1 = np.array(pts1).transpose()
        // pts2 = np.array(pts2).transpose()
    
        // homogeneous_4d_coords = cv2.triangulatePoints(proj1, proj2, pts1, pts2)
        // # return triangulatePoints(proj1, proj2, pts1, pts2)
    
        // threeD_coords = cv2.convertPointsFromHomogeneous(homogeneous_4d_coords.transpose())
    
        // output_points = []
    
        // # print threeD_coords
        // for point in threeD_coords:
        //     output_points.append((point[0][0], point[0][1], point[0][2]))
        //     # output_points.append(point[0])
        // # for coord in homogeneous_4d_coords:
    
        // return output_points
        
        //remove previous data out of window size of N
        if self.prev_points.len()>self.N as usize
        {
            self.prev_points.remove(0)?;
            self.prev_poses.remove(0)?;
        }
        

        let pts1 = msg.pts1.clone();
        let pts2 = msg.pts2.clone();
        let pose = msg.pose.clone();

        let mut proj_points1 = VectorOfPoint2f::new();
        let mut proj_points2 = VectorOfPoint2f::new();
        
        for i in 0..pts1.nrows() 
        { 
            proj_points1.push(core::Point2f::new(pts1[(i, 0)], pts1[(i, 1)]));
            proj_points2.push(core::Point2f::new(pts2[(i, 0)], pts2[(i, 1)]));
            
        }
        
        
        // form calibration matrix
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

        //==============================================================================

        let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
        let mut triangulated_points = Mat::default() ;//opencv::types::VectorOfPoint3f::new();
        let essential_mat = opencv::calib3d::find_essential_mat(
          &proj_points2,
          &proj_points1,
          718.8560, // self.focal,
          core::Point2d::new(607.1928, 185.2157) , //self.pp,
          opencv::calib3d::RANSAC,
          0.999,
          1.0,
          &mut mask,
        )
        .unwrap();

        // let mut essential_mat = Mat::default();

        
        // let mut curr_traj_pos = self.traj_pos + self.traj_rot*&msg.pose.pos;
        // let mut curr_traj_rot =   &msg.pose.rot * self.traj_rot; 
        
        // // curr_traj_rot = curr_traj_rot.transpose();
        // // curr_traj_pos = - curr_traj_rot  * curr_traj_pos;

        // let curr_pose_new = self.get_pose(&curr_traj_rot, &curr_traj_pos);
        // let r2 = curr_pose_new.get_rotation_matrix();
        // let t2 = curr_pose_new.get_translation_matrix();

        // let mut prev_pose = self.get_pose(&self.traj_rot, &self.traj_pos);   

        // // prev_pose.rot = prev_pose.rot.transpose();
        // // prev_pose.pos = - prev_pose.rot  * prev_pose.pos;


        // let r1 = prev_pose.get_rotation_matrix();
        // let t1 = prev_pose.get_translation_matrix();


        // let curr_pose_new = self.get_pose(&msg.pose.rot, &msg.pose.pos);
        // let r2 = curr_pose_new.get_rotation_matrix();
        // let t2 = curr_pose_new.get_translation_matrix();

        // let r1 = Mat::eye(3, 3, core::CV_64FC1).unwrap().to_mat().unwrap();
        // let t1 = Mat::zeros(3, 1, core::CV_64FC1).unwrap().to_mat().unwrap();

        // // opencv::sfm::essential_from_rt(
        // //     &r2,
        // //     &t2,
        // //     &r1, 
        // //     &t1,   

          
        // //     &mut essential_mat).unwrap();


        let distance_thresh = 10000.0;
        opencv::calib3d::recover_pose_triangulated(
            &essential_mat,
            &proj_points2,  
            &proj_points1,
            &K,
          &mut recover_r,
          &mut recover_t,
          distance_thresh,
          &mut mask,
            &mut triangulated_points).unwrap();



        
        triangulated_points =    triangulated_points.t().unwrap().to_mat().unwrap();
        
        let mut points_3d = Mat::default();
        opencv::calib3d::convert_points_from_homogeneous(&triangulated_points, &mut points_3d)?;

        //==============================================================

        points_3d = points_3d.reshape(1, 0).unwrap();
        let mut points_3d_new = points_3d.clone();
        points_3d.convert_to(&mut points_3d_new, CV_32FC1, 1.0, 0.0).unwrap();
        points_3d = points_3d_new;

        // unsafe{
        //     points_3d.create_nd( &[points_3d.rows(), 3], CV_32FC1)?;
        // }
        println!("{:?}", mask);
        let mut filtered_points = VectorOfPoint3f::new();
        

        let limit_pt = 1000000.0;

        
        for i in 0..mask.rows()
        {
            //if *mask.at_2d::<u8>(i,0).unwrap()==0
            {
                //println!("{}", *mask.at_2d::<u8>(i,0).unwrap());
                let curr_pt = core::Point3f::new(*points_3d.at_2d::<f32>(i,0).unwrap(),*points_3d.at_2d::<f32>(i,1).unwrap(),*points_3d.at_2d::<f32>(i,2).unwrap());

                //if curr_pt.x.abs() <  limit_pt && curr_pt.y.abs() <limit_pt // && curr_ptf.z.abs() <limit_pt 
                {
                    filtered_points.push(curr_pt.clone());
                }

            }
            
        }
        //println!("{:?}", filtered_points);

        if points_3d.rows()>0
        {
            
            let curr_pose = self.get_pose(&self.traj_rot, &self.traj_pos);    
            let relative_proj_mat = curr_pose.get_relative_projection_matrix();    
            let mut tri_trans_points = Mat::default();


            triangulated_points = triangulated_points.reshape(4,0).unwrap();
            let mut tmp_triangulated_points = triangulated_points.clone();

            // unsafe{
            //     tmp_triangulated_points.create_nd( &[tmp_triangulated_points.rows(), 1], core::CV_32FC4)?;
            // }
            
            
            //triangulated_points.convert_to(&mut tmp_triangulated_points, core::CV_32FC4, 1.0, 0.0).unwrap();
            
            //triangulated_points = tmp_triangulated_points.clone();

            println!("{:?}", triangulated_points);
            println!("{:?}", tmp_triangulated_points);
            core::transform(&triangulated_points, &mut tri_trans_points, &relative_proj_mat).unwrap();
            points_3d = tri_trans_points;
            
            let mut new_3dPoints = Mat::default();
            opencv::calib3d::convert_points_from_homogeneous(&points_3d, &mut new_3dPoints)?;
            
            println!("New points {:?}", points_3d);
            //points_3d = new_3dPoints;

            new_3dPoints = points_3d.reshape(1, 0).unwrap();
            new_3dPoints.convert_to(&mut points_3d, CV_32FC1, 1.0, 0.0).unwrap();
            
            println!("New points {:?}", points_3d);
            // unsafe{
            //         points_3d.create_nd( &[points_3d.rows(), 3], CV_32FC1)?;
            //     }
    
            
    
    
            let mut filtered_points = VectorOfPoint3f::new();
            for i in 0..points_3d.rows()
            {
                let curr_pt = core::Point3f::new(*points_3d.at_2d::<f32>(i,0).unwrap(),*points_3d.at_2d::<f32>(i,1).unwrap(),*points_3d.at_2d::<f32>(i,2).unwrap());
                filtered_points.push(curr_pt.clone());
                
            }
            //==============================================================
    
            
    
            println!("{:?}", filtered_points);
    //======================================================================================
    
            //TODO: remove the below code: just for debugging
            // let x_offset = 500 ;
            // let y_offset = 375 ;
            let x_offset = 500 ;
    
            let y_offset = 375 ;
            let imtitle = "Estimated Trajectory".to_string();
            let limit_pt = 1000000.0;
            for i in 0..filtered_points.len()
            {   
                
                    let curr_ptf = filtered_points.get(i).unwrap();
                    if curr_ptf.x.is_finite() && curr_ptf.y.is_finite() && curr_ptf.z.is_finite() //&&  curr_ptf.z<0.0
                    //if curr_ptf.x.abs() <  limit_pt && curr_ptf.z.abs() <limit_pt  && curr_ptf.y.abs() <limit_pt  && curr_ptf.z >0.0// 
                    {
                        println!("{:?}", curr_ptf);
                        // let (x,_) = (curr_ptf.y   as i32).overflowing_add(x_offset);
                        // let (y,_)  = (-curr_ptf.x as i32).overflowing_add(y_offset);
                        // let curr_pt1 = core::Point_::new(x,y); //curr_pt.x as i32 +x_offset, curr_pt.y as i32 +y_offset);
                        // opencv::imgproc::circle(&mut self.traj_img, curr_pt1, 1, core::Scalar::new(0.0, 0.0, 255.0, 0.0), -1, 8, 0)?;

                        let (x,_) = (curr_ptf.y   as i32).overflowing_add(x_offset);
                        let (y,_)  = (-curr_ptf.z as i32).overflowing_add(y_offset);
                        let curr_pt1 = core::Point_::new(x,y); //curr_pt.x as i32 +x_offset, curr_pt.y as i32 +y_offset);
                        opencv::imgproc::circle(&mut self.traj_img, curr_pt1, 1, core::Scalar::new(0.0, 255.0, 255.0, 0.0), -1, 8, 0)?;

                        let (x,_) = (curr_ptf.x   as i32).overflowing_add(x_offset);
                        let (y,_)  = (-curr_ptf.z as i32).overflowing_add(y_offset);
                        let curr_pt1 = core::Point_::new(x,y); //curr_pt.x as i32 +x_offset, curr_pt.y as i32 +y_offset);
                        opencv::imgproc::circle(&mut self.traj_img, curr_pt1, 1, core::Scalar::new(255.0, 0.0, 255.0, 0.0), -1, 8, 0)?;


                    }
    
    
            }
        }
        let x_offset = 500 ;
    
        let y_offset = 375 ;

        opencv::imgproc::circle(&mut self.traj_img, core::Point_::new(self.traj_pos.x as i32 +x_offset, -self.traj_pos.z as i32 +y_offset), 1, core::Scalar::new(0.0, 255.0, 0.0, 0.0), -1, 8, 0)?;


        let imtitle = "Estimated 3d points".to_string();
        opencv::highgui::imshow(&imtitle, &self.traj_img)?;
        opencv::highgui::wait_key(1)?; 




    //-------------------------------------------------------


    if self.traj_pos == DVVector3::zeros() && self.traj_rot == DVMatrix3::zeros() {
        self.traj_pos = msg.pose.pos;
        self.traj_rot = msg.pose.rot;
    }
    else {
        // self.traj_pos = self.traj_pos + self.traj_rot*&msg.pose.pos;
        // self.traj_rot = &msg.pose.rot * self.traj_rot;

        self.traj_pos = self.traj_pos + self.traj_rot*&msg.pose.pos;
        self.traj_rot =  self.traj_rot * &msg.pose.rot;

    }    

    }
    Ok(Status::done(()))
}



}


use crate::pluginfunction::Function;

impl Function for DarvisTriangulator {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.triangulate(_context, message).unwrap();
        Ok(Status::done(()))
    }

}