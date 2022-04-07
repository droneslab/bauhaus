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
            traj_img: Mat::new_rows_cols_with_default(700, 700, core::CV_8UC3, core::Scalar::all(0.0)).unwrap(),
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


pub fn find_prev_matches(&mut self, pts_prev: & VectorOfPoint2f, pts_curr: &VectorOfPoint2f) -> Vec<(i32, i32)>
{
    let mut indices : Vec<(i32, i32)> = Vec::new();
    let mut ind = 0;
    for pt_curr in pts_curr{
        let index_of_found = pts_prev.iter().position(|x| x == pt_curr);
        if ! index_of_found.is_none()
        {
            
            indices.push((index_of_found.unwrap() as i32, ind));
        }
        ind+=1; 
    }
    

    return indices;
}



pub fn find_prev_matches1(&mut self, pts_idx_prev: & DVVectorOfMatchPair, pts_idx_curr: &DVVectorOfMatchPair) -> Vec<(i32, i32)>
{
    let mut indices : Vec<(i32, i32)> = Vec::new();
    let mut ind = 0;
    //println!("{}, {}",pts_idx_prev.nrows(),pts_idx_curr.nrows());
    for i in 0..pts_idx_curr.nrows()
    {

        for j in 0..pts_idx_prev.nrows()
        {
            if pts_idx_prev[(j,1)]== pts_idx_curr[(i,0)]
            {
                indices.push((j as i32, i as i32));
            }
        }

        ind+=1; 
    }
    

    return indices;
}


pub fn find_scale(&mut self, pts_prev: & VectorOfPoint3f, pts_curr: &VectorOfPoint3f) -> f64
{

    let mut scale = 0.0;
    let mut count=0;
    for i in 0..pts_prev.len()
    {
        for j in 0..pts_prev.len()
        {
            let pt_prev1 = pts_prev.get(i).unwrap();
            let pt_prev2 = pts_prev.get(j).unwrap();
            let pt_curr1 = pts_curr.get(i).unwrap();
            let pt_curr2 = pts_curr.get(j).unwrap();
            let dnr = (pt_curr1 - pt_curr2).norm();
            if dnr != 0.0
            {
                let s = (pt_prev1 - pt_prev2).norm() / dnr;
                scale+=s;
                count+=1;
            }
            
        }
        
    }
    if count > 0
    {
        scale/=count as f64;
    }
    else
    {
        scale=0.7;
    }

    if scale ==0.0 || scale.is_nan()
    {
        scale=0.7;
    }
        

    return scale;
}

pub fn compute_projection(&mut self, rot: &DVMatrix3, pos: &DVVector3) -> Mat
{
    let t = self.traj_pos + self.traj_rot* pos;
    let R =    self.traj_rot  * rot ;  // 

    //=======================================================

    let curr_r_t = R.transpose();
    let curr_t_t = -curr_r_t * t;

    let new_pose = self.get_pose(&curr_r_t, & curr_t_t);

    let curr_proj = new_pose.get_projection_matrix();

    return curr_proj;
}

pub fn get_triangulate_pts(&mut self, curr_proj: &Mat, prev_proj: &Mat, K: &Mat,  proj_points1: &VectorOfPoint2f ,  proj_points2: &VectorOfPoint2f ) -> VectorOfPoint3f
{
        // //============================================================================
        // let t = self.traj_pos + self.traj_rot* pos;
        // let R =   self.traj_rot * rot ; //&msg.pose.rot*  self.traj_rot  ;
    
        // //=======================================================
    
    
        // let curr_r_t = R.transpose();
        // let curr_t_t = -curr_r_t * t;
    
        // let new_pose = self.get_pose(&curr_r_t, & curr_t_t);
    
        // let curr_proj = new_pose.get_projection_matrix();
    
        let curr_p = opencv::core::mul_mat_mat(&K, &curr_proj).unwrap().to_mat().unwrap();
    
        let prev_p = opencv::core::mul_mat_mat(&K, &prev_proj).unwrap().to_mat().unwrap();
        //=====================================================
    
        let mut points4_d = Mat::default();
            
            
        //TODO: use followingtriangulate_pointstriangulate_points
        //opencv::sfm::triangulate_points()
        //TODO: convert 2d points to 3d 
        let mut points2d = core::Vector::<VectorOfPoint2f>::new();

        points2d.push(proj_points1.clone());
        points2d.push(proj_points2.clone());

        let mut projection_matrices = VectorOfMat::new();
        projection_matrices.push(prev_p.clone());
        projection_matrices.push(curr_p.clone());


        let mut points3dmat = Mat::default();


        let mut points3d = VectorOfPoint3f::default();


        opencv::sfm::triangulate_points(&points2d, 
            &projection_matrices, 
            &mut points3dmat);

        println!("{:?}", points3dmat);
        // opencv::calib3d::triangulate_points(
        //     &prev_p,
        //     &curr_p, 
        //     &proj_points1,
        //     &proj_points2,
        //     &mut points4_d).unwrap();
    
        // let mut points3d = VectorOfPoint3f::default();
        
        // let mut points4_dnew = Mat::default();
        // opencv::core::transpose(&points4_d, &mut points4_dnew).unwrap();
        
        // opencv::calib3d::convert_points_from_homogeneous(&points4_dnew, &mut points3d).unwrap();

        return points3d;

}





pub fn find_match_3_frame(&mut self, pts_idx_prev: & DVVectorOfMatchPair, pts_idx_curr: &DVVectorOfMatchPair) -> Vec<(i32, i32, i32)>
{
    let mut indices : Vec<(i32, i32, i32)> = Vec::new();
    let mut ind = 0;
    for i in 0..pts_idx_curr.nrows()
    {
        for j in 0..pts_idx_prev.nrows()
        {
            if pts_idx_prev[(j,1)]== pts_idx_curr[(i,0)]
            {
                indices.push((pts_idx_prev[(j,0)], pts_idx_prev[(j,1)], pts_idx_curr[(i,1)]));
            }
        }

        ind+=1; 
    }
    

    return indices;
}

pub fn get_triangulate_pts_sfm(&mut self, points2d: &core::Vector::<VectorOfPoint2f>, projection_matrices: &VectorOfMat) -> VectorOfPoint3f
{


        let mut points3dmat = Mat::default();

        let mut points3d = VectorOfPoint3f::default();


        opencv::sfm::triangulate_points(&points2d, 
            &projection_matrices, 
            &mut points3dmat);

        println!("{:?}", points3dmat);


        return points3d;

}


pub fn triangulate(&mut self, _context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<TriangulationMsg>() {

    
    if self.prev_points.len()>self.N as usize
    {
        self.prev_points.remove(0)?;
        self.prev_poses.remove(0)?;
        self.prev_points3d.remove(0)?;
    }

    
    //============================================================================
    let pts1 = msg.pts1.clone();
    let pts2 = msg.pts2.clone();
    let pose = msg.pose.clone();
    let kp_match_indices = msg.kp_match_indices.clone();


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

    let mut curr_proj = self.compute_projection(&msg.pose.rot, &msg.pose.pos);

 
    // //=================================================================

    //     let indices_3frames = self.find_match_3_frame(&self.kp_match_indices.clone(), &kp_match_indices);
        
    //     //let prev_points3d = self.prev_points3d.get(self.prev_points3d.len() -1).unwrap();

    //     let mut pts2d1 = VectorOfPoint2f::new();
    //     let mut pts2d2 = VectorOfPoint2f::new();
    //     let mut  pts2d3 = VectorOfPoint2f::new();        

    //     for i in 0..indices_3frames.len()
    //     {
    //         let (i1, i2, i3 )= indices_3frames.get(i).unwrap();
        
            
    //         pts2d1.push(core::Point2f::new(self.all_prev_points[(*i1 as usize, 0)], self.all_prev_points[(*i1 as usize, 1)]));
    //         pts2d2.push(core::Point2f::new(pts1[(*i2 as usize, 0)], pts1[(*i2 as usize, 1)]));
    //         pts2d3.push(core::Point2f::new(pts2[(*i3 as usize, 0)], pts2[(*i3 as usize, 1)]));
  
    //     }


    // //================================================================

  


    let curr_p = opencv::core::mul_mat_mat(&K, &curr_proj).unwrap().to_mat().unwrap();
    
    let prev_p = opencv::core::mul_mat_mat(&K, &self.P_nk.clone()).unwrap().to_mat().unwrap();


    let mut points2d = core::Vector::<VectorOfPoint2f>::new();
    points2d.push(proj_points1.clone());
    points2d.push(proj_points2.clone());
    //points2d.push(pts2d3.clone());


    let mut projection_matrices = VectorOfMat::new();
    
    //let old_p = self.prev_poses_mat.get(self.prev_poses_mat.len()-1).unwrap();
    //projection_matrices.push(old_p.clone());
    projection_matrices.push(prev_p.clone());
    projection_matrices.push(curr_p.clone());


    // let is_projective = true;
    // let mut ps = Mat::default();
    // let mut points3d = Mat::default();
    
    // opencv::sfm::reconstruct(&points2d, &mut ps,  &mut points3d, &mut K, is_projective);

    let mut points3d  = self.get_triangulate_pts_sfm(&points2d, &projection_matrices);

    
    //let mut points3d  = self.get_triangulate_pts(&curr_proj, &self.P_nk.clone(), &K, &proj_points1, &proj_points2);


    //================================================================================
    //println!("{:?}", points3d.get(0).unwrap());

    println!("{:?}", points3d.len());

    let traing_point2d = self.get_2d_points_from_3d(&points3d);

    
    //TODO: remove the below code: just for debugging
    // let x_offset = 500 ;
    // let y_offset = 375 ;
    let x_offset = 500 ;
    let y_offset = 375 ;
    //let imtitle = "Estimated Trajectory".to_string();
    for i in 0..points3d.len()
    {   
        let curr_pt = points3d.get(i).unwrap();
        opencv::imgproc::circle(&mut self.traj_img, core::Point_::new(curr_pt.x as i32 +x_offset, -curr_pt.z as i32 +y_offset), 1, core::Scalar::new(0.0, 0.0, 255.0, 0.0), -1, 8, 0)?;
    }
    
    opencv::imgproc::circle(&mut self.traj_img, core::Point_::new(self.traj_pos.x as i32 +x_offset, -self.traj_pos.z as i32 +y_offset), 1, core::Scalar::new(0.0, 255.0, 0.0, 0.0), -1, 8, 0)?;


    let imtitle = "Estimated 3d points".to_string();
    opencv::highgui::imshow(&imtitle, &self.traj_img)?;
    opencv::highgui::wait_key(1)?; 

    self.prev_points3d.push(points3d.clone());

    //=====================================================

    //-------------------------------------------------------
    self.prev_points.push(proj_points2.clone());

    self.P_nk = curr_proj.clone();



    self.prev_poses.push(curr_proj.clone());

    self.kp_match_indices = kp_match_indices.clone();
    self.all_prev_points = msg.pts1.clone();

    self.prev_poses_mat.push(self.P_nk.clone());
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

    self.frame_idx+=1;

}
Ok(Status::done(()))
}

// pub fn triangulate(&mut self, _context: Context, message: Message) -> ActorResult<()> {
//     if let Some(msg) = message.content_as::<TriangulationMsg>() {

    
//     if self.prev_points.len()>self.N as usize
//     {
//         self.prev_points.remove(0)?;
//         self.prev_poses.remove(0)?;
//         self.prev_points3d.remove(0)?;
//     }

    
//     //============================================================================
//     let pts1 = msg.pts1.clone();
//     let pts2 = msg.pts2.clone();
//     let pose = msg.pose.clone();
//     let kp_match_indices = msg.kp_match_indices.clone();


//     let mut proj_points1 = VectorOfPoint2f::new();
//     let mut proj_points2 = VectorOfPoint2f::new();
    

//     for i in 0..kp_match_indices.nrows()
//     {
//         let tidx= kp_match_indices[(i, 0)] as usize;
//         let qidx= kp_match_indices[(i, 1)] as usize;

//         proj_points1.push(core::Point2f::new(pts1[(tidx, 0)], pts1[(tidx, 1)]));
//         proj_points2.push(core::Point2f::new(pts2[(qidx, 0)], pts2[(qidx, 1)]));   
//     }


//     // form calibration matrix
//     let mut K = Mat::new_rows_cols_with_default(3, 3 ,CV_32FC1, opencv::core::Scalar::all(0.0)).unwrap();
//     // Mat K = (Mat_<double>(3,3) << 718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1 );
//     unsafe {
//         *K.at_2d_unchecked_mut::<f32>(0,0).unwrap() = 718.8560;
//         *K.at_2d_unchecked_mut::<f32>(0,1).unwrap() = 0.0;
//         *K.at_2d_unchecked_mut::<f32>(0,2).unwrap() = 607.1928;
//         *K.at_2d_unchecked_mut::<f32>(1,0).unwrap() = 0.0;
//         *K.at_2d_unchecked_mut::<f32>(1,1).unwrap() = 718.8560;
//         *K.at_2d_unchecked_mut::<f32>(1,2).unwrap() = 185.2157;
//         *K.at_2d_unchecked_mut::<f32>(2,0).unwrap() = 0.0;
//         *K.at_2d_unchecked_mut::<f32>(2,1).unwrap() = 0.0;
//         *K.at_2d_unchecked_mut::<f32>(2,2).unwrap() = 1.0;
//     }

//     let mut curr_proj = self.compute_projection(&msg.pose.rot, &msg.pose.pos);

//     if self.frame_idx >0
//     {
//     //=================================================================

//         let indices_3frames = self.find_match_3_frame(&self.kp_match_indices.clone(), &kp_match_indices);
        
//         //let prev_points3d = self.prev_points3d.get(self.prev_points3d.len() -1).unwrap();

//         let mut pts2d1 = VectorOfPoint2f::new();
//         let mut pts2d2 = VectorOfPoint2f::new();
//         let mut  pts2d3 = VectorOfPoint2f::new();        

//         for i in 0..indices_3frames.len()
//         {
//             let (i1, i2, i3 )= indices_3frames.get(i).unwrap();
        
            
//             pts2d1.push(core::Point2f::new(self.all_prev_points[(*i1 as usize, 0)], self.all_prev_points[(*i1 as usize, 1)]));
//             pts2d2.push(core::Point2f::new(pts1[(*i2 as usize, 0)], pts1[(*i2 as usize, 1)]));
//             pts2d3.push(core::Point2f::new(pts2[(*i3 as usize, 0)], pts2[(*i3 as usize, 1)]));
  
//         }


//     //================================================================

  


//     let curr_p = opencv::core::mul_mat_mat(&K, &curr_proj).unwrap().to_mat().unwrap();
    
//     let prev_p = opencv::core::mul_mat_mat(&K, &self.P_nk.clone()).unwrap().to_mat().unwrap();


//     let mut points2d = core::Vector::<VectorOfPoint2f>::new();
//     points2d.push(pts2d1.clone());
//     points2d.push(pts2d2.clone());
//     points2d.push(pts2d3.clone());


//     let mut projection_matrices = VectorOfMat::new();
    
//     let old_p = self.prev_poses_mat.get(self.prev_poses_mat.len()-1).unwrap();
//     projection_matrices.push(old_p.clone());
//     projection_matrices.push(prev_p.clone());
//     projection_matrices.push(curr_p.clone());



//     let mut points3d  = self.get_triangulate_pts_sfm(&points2d, &projection_matrices);

    
//     //let mut points3d  = self.get_triangulate_pts(&curr_proj, &self.P_nk.clone(), &K, &proj_points1, &proj_points2);


//     //================================================================================
//     //println!("{:?}", points3d.get(0).unwrap());

//     println!("{:?}", points3d.len());

//     let traing_point2d = self.get_2d_points_from_3d(&points3d);

    
//     //TODO: remove the below code: just for debugging
//     // let x_offset = 500 ;
//     // let y_offset = 375 ;
//     let x_offset = 500 ;
//     let y_offset = 375 ;
//     //let imtitle = "Estimated Trajectory".to_string();
//     for i in 0..points3d.len()
//     {   
//         let curr_pt = points3d.get(i).unwrap();
//         opencv::imgproc::circle(&mut self.traj_img, core::Point_::new(curr_pt.x as i32 +x_offset, -curr_pt.z as i32 +y_offset), 1, core::Scalar::new(0.0, 0.0, 255.0, 0.0), -1, 8, 0)?;
//     }
    
//     opencv::imgproc::circle(&mut self.traj_img, core::Point_::new(self.traj_pos.x as i32 +x_offset, -self.traj_pos.z as i32 +y_offset), 1, core::Scalar::new(0.0, 255.0, 0.0, 0.0), -1, 8, 0)?;


//     let imtitle = "Estimated 3d points".to_string();
//     opencv::highgui::imshow(&imtitle, &self.traj_img)?;
//     opencv::highgui::wait_key(1)?; 

//     self.prev_points3d.push(points3d.clone());

// }

//     //=====================================================

//     //-------------------------------------------------------
//     self.prev_points.push(proj_points2.clone());

//     self.P_nk = curr_proj.clone();



//     self.prev_poses.push(curr_proj.clone());

//     self.kp_match_indices = kp_match_indices.clone();
//     self.all_prev_points = msg.pts1.clone();

//     self.prev_poses_mat.push(self.P_nk.clone());
//     //-------------------------------------------------------


//     if self.traj_pos == DVVector3::zeros() && self.traj_rot == DVMatrix3::zeros() {
//         self.traj_pos = msg.pose.pos;
//         self.traj_rot = msg.pose.rot;
//     }
//     else {
//         self.traj_pos = self.traj_pos + self.traj_rot*&msg.pose.pos;
//         self.traj_rot = &msg.pose.rot * self.traj_rot;

//         // self.traj_pos = self.traj_pos + self.traj_rot*&msg.pose.pos;
//         // self.traj_rot =  self.traj_rot * &msg.pose.rot;

//     }

//     self.frame_idx+=1;

// }
// Ok(Status::done(()))
// }



pub fn triangulate_old1(&mut self, _context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<TriangulationMsg>() {


    if self.prev_points.len()>self.N as usize
    {
        self.prev_points.remove(0)?;
        self.prev_poses.remove(0)?;
        self.prev_points3d.remove(0)?;
    }

    
    //============================================================================
    let pts1 = msg.pts1.clone();
    let pts2 = msg.pts2.clone();
    let pose = msg.pose.clone();
    let kp_match_indices = msg.kp_match_indices.clone();


    let mut proj_points1 = VectorOfPoint2f::new();
    let mut proj_points2 = VectorOfPoint2f::new();
    

    for i in 0..kp_match_indices.nrows()
    {
        let tidx= kp_match_indices[(i, 0)] as usize;
        let qidx= kp_match_indices[(i, 1)] as usize;

        proj_points1.push(core::Point2f::new(pts1[(tidx, 0)], pts1[(tidx, 1)]));
        proj_points2.push(core::Point2f::new(pts2[(qidx, 0)], pts2[(qidx, 1)]));   
    }
    // for i in 0..pts1.nrows() 
    // { 
    //     proj_points1.push(core::Point2f::new(pts1[(i, 0)], pts1[(i, 1)]));
    //     proj_points2.push(core::Point2f::new(pts2[(i, 0)], pts2[(i, 1)]));        
    // }

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


    let mut curr_proj = self.compute_projection(&msg.pose.rot, &msg.pose.pos);

    let mut points3d  = self.get_triangulate_pts(&curr_proj, &self.P_nk.clone(), &K, &proj_points1, &proj_points2);

    
    // //============================================================================
    // let t = self.traj_pos + self.traj_rot*&msg.pose.pos;
    // let R =   self.traj_rot * &msg.pose.rot ; //&msg.pose.rot*  self.traj_rot  ;

    // //=======================================================


    // let curr_r_t = R.transpose();
    // let curr_t_t = -curr_r_t * t;

    // let new_pose = self.get_pose(&curr_r_t, & curr_t_t);

    // let curr_proj = new_pose.get_projection_matrix();

    // let curr_P = opencv::core::mul_mat_mat(&K, &curr_proj).unwrap().to_mat().unwrap();

    // let prev_P = opencv::core::mul_mat_mat(&K, &self.P_nk).unwrap().to_mat().unwrap();
    // //=====================================================

    // let mut points4_d = Mat::default();
        
        
    // //TODO: use followingtriangulate_pointstriangulate_points
    // //opencv::sfm::triangulate_points()
    // //TODO: convert 2d points to 3d 
    // opencv::calib3d::triangulate_points(
    //     &prev_P,
    //     &curr_P, 
    //     &proj_points1,
    //     &proj_points2,
    //     &mut points4_d)?;

    // let mut points3d = VectorOfPoint3f::default();
    
    // let mut points4_dnew = Mat::default();
    // opencv::core::transpose(&points4_d, &mut points4_dnew)?;
    
    // opencv::calib3d::convert_points_from_homogeneous(&points4_dnew, &mut points3d)?;

    
    // TODO: fix relative scale====================================

    // few new points and existing pts
    //

    //println!("{:?}", points3d.get(0).unwrap());

    // if self.frame_idx >0
    // {
    //     //let indices_pair = self.find_prev_matches(&self.prev_points.get(self.prev_points.len() -1).unwrap(), &proj_points1);
    //     let indices_pair = self.find_prev_matches1(&self.kp_match_indices.clone(), &kp_match_indices);
        
    //     let prev_points3d = self.prev_points3d.get(self.prev_points3d.len() -1).unwrap();

    //     let mut match_prev_points3d = VectorOfPoint3f::new();
    //     let mut match_curr_points3d = VectorOfPoint3f::new();
    //     for i in 0..indices_pair.len()
    //     {
    //         let (i1, i2 )= indices_pair.get(i).unwrap();
    //         match_prev_points3d.push(prev_points3d.get(*i1 as usize).unwrap());
    //         match_curr_points3d.push(points3d.get(*i2 as usize).unwrap());        
    //     }

    //     println!("matches {:?}", indices_pair.len());

    //     let scale = self.find_scale(&match_prev_points3d, &match_curr_points3d);

    //     //---------------------------------------------------------------------
    //     // re triangulate using scaled translation
    //     let pos = msg.pose.pos*scale;
    //     curr_proj = self.compute_projection(&msg.pose.rot, &pos);


    //     points3d  = self.get_triangulate_pts(&curr_proj, &self.P_nk.clone(), &K, &proj_points1, &proj_points2);
    //     //---------------------------------------------------------------------

    //     println!("{:?}", scale);
    // }
    //================================================================================
    //println!("{:?}", points3d.get(0).unwrap());

    println!("{:?}", points3d.len());

    let traing_point2d = self.get_2d_points_from_3d(&points3d);

    
    //TODO: remove the below code: just for debugging
    // let x_offset = 500 ;
    // let y_offset = 375 ;
    let x_offset = 500 ;
    let y_offset = 375 ;
    //let imtitle = "Estimated Trajectory".to_string();
    for i in 0..points3d.len()
    {   
        let curr_pt = points3d.get(i).unwrap();
        opencv::imgproc::circle(&mut self.traj_img, core::Point_::new(curr_pt.x as i32 +x_offset, -curr_pt.z as i32 +y_offset), 1, core::Scalar::new(0.0, 0.0, 255.0, 0.0), -1, 8, 0)?;
    }
    
    opencv::imgproc::circle(&mut self.traj_img, core::Point_::new(self.traj_pos.x as i32 +x_offset, -self.traj_pos.z as i32 +y_offset), 1, core::Scalar::new(0.0, 255.0, 0.0, 0.0), -1, 8, 0)?;


    let imtitle = "Estimated 3d points".to_string();
    opencv::highgui::imshow(&imtitle, &self.traj_img)?;
    opencv::highgui::wait_key(1)?; 

    

    //=====================================================

    //-------------------------------------------------------
    self.prev_points.push(proj_points2.clone());

    self.P_nk = curr_proj.clone();

    self.prev_points3d.push(points3d.clone());

    self.prev_poses.push(curr_proj.clone());

    self.kp_match_indices = kp_match_indices.clone();
    self.all_prev_points = msg.pts2.clone();
    //-------------------------------------------------------


    if self.traj_pos == DVVector3::zeros() && self.traj_rot == DVMatrix3::zeros() {
        self.traj_pos = msg.pose.pos;
        self.traj_rot = msg.pose.rot;
    }
    else {
        self.traj_pos = self.traj_pos + self.traj_rot*&msg.pose.pos;
        self.traj_rot = &msg.pose.rot * self.traj_rot;

        // self.traj_pos = self.traj_pos + self.traj_rot*&msg.pose.pos;
        // self.traj_rot =  self.traj_rot * &msg.pose.rot;

    }

    self.frame_idx+=1;

}
Ok(Status::done(()))
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


/*
//     // This is the handler that will be used by the actor.
// // This is the handler that will be used by the actor.
// pub fn triangulate_old(&mut self, _context: Context, message: Message) -> ActorResult<()> {
//     if let Some(msg) = message.content_as::<TriangulationMsg>() {

//         // Convert back to cv structures

//         //https://python.hotexamples.com/examples/cv2/-/triangulatePoints/python-triangulatepoints-function-examples.html
//         // '''
//         // Takes pts1, pts2, k, r, t
//         // Returns triangulated points using openCV implementation
//         // '''
//         // proj1 = np.array([[1, 0, 0, 0],
//         //                   [0, 1, 0, 0],
//         //                   [0, 0, 1, 0]])
//         // proj2 = np.append(r, t, 1)
//         // pts1 = np.array(pts1).transpose()
//         // pts2 = np.array(pts2).transpose()
    
//         // homogeneous_4d_coords = cv2.triangulatePoints(proj1, proj2, pts1, pts2)
//         // # return triangulatePoints(proj1, proj2, pts1, pts2)
    
//         // threeD_coords = cv2.convertPointsFromHomogeneous(homogeneous_4d_coords.transpose())
    
//         // output_points = []
    
//         // # print threeD_coords
//         // for point in threeD_coords:
//         //     output_points.append((point[0][0], point[0][1], point[0][2]))
//         //     # output_points.append(point[0])
//         // # for coord in homogeneous_4d_coords:
    
//         // return output_points
        
//         //remove previous data out of window size of N
//         if self.prev_points.len()>self.N as usize
//         {
//             self.prev_points.remove(0)?;
//             self.prev_poses.remove(0)?;
//         }
        

//         let pts1 = msg.pts1.clone();
//         let pts2 = msg.pts2.clone();
//         let pose = msg.pose.clone();

//         let mut proj_points1 = VectorOfPoint2f::new();
//         let mut proj_points2 = VectorOfPoint2f::new();
        
//         for i in 0..pts1.nrows() 
//         { 
//             proj_points1.push(core::Point2f::new(pts1[(i, 0)], pts1[(i, 1)]));
//             proj_points2.push(core::Point2f::new(pts2[(i, 0)], pts2[(i, 1)]));
            
//         }
        
        
//         // form calibration matrix
//         let mut K = Mat::new_rows_cols_with_default(3, 3 ,CV_32FC1, opencv::core::Scalar::all(0.0)).unwrap();
//         // Mat K = (Mat_<double>(3,3) << 718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1 );
//         unsafe {
//             *K.at_2d_unchecked_mut::<f32>(0,0).unwrap() = 718.8560;
//             *K.at_2d_unchecked_mut::<f32>(0,1).unwrap() = 0.0;
//             *K.at_2d_unchecked_mut::<f32>(0,2).unwrap() = 607.1928;
//             *K.at_2d_unchecked_mut::<f32>(1,0).unwrap() = 0.0;
//             *K.at_2d_unchecked_mut::<f32>(1,1).unwrap() = 718.8560;
//             *K.at_2d_unchecked_mut::<f32>(1,2).unwrap() = 185.2157;
//             *K.at_2d_unchecked_mut::<f32>(2,0).unwrap() = 0.0;
//             *K.at_2d_unchecked_mut::<f32>(2,1).unwrap() = 0.0;
//             *K.at_2d_unchecked_mut::<f32>(2,2).unwrap() = 1.0;
//         }

//         // TODO:  nghiaho.com/?p=2379
//         // 
//         // Filter points that do not appear in previous point set
//         // Idea: pts2 for previous should match pts1, if the frames are in sequence

//         //let proj_matr1 = self.P.clone() ;//Mat::eye(3, 4, CV_32FC1).unwrap().to_mat().unwrap();
//         //let proj_matr2 = pose.get_relative_projection_matrix(); //self.get_projection(&pose);

//         // fill the list
        
//         self.prev_points.push(proj_points2.clone());
//         //self.prev_poses.push(proj_matr2.clone());

//         // Create all the projection matrix
//         let T = pose.get_transformation_matrix();

     
//         //println!("{:?}", T);

//         let curr_T = opencv::core::mul_mat_mat(&T, &self.T).unwrap().to_mat().unwrap();
        
        
//         let R = Mat::rowscols(&curr_T, &opencv::core::Range::new(0,3).unwrap(), &opencv::core::Range::new(0,3).unwrap()).unwrap();
//         let t = Mat::rowscols(&curr_T, &opencv::core::Range::new(0,3).unwrap(), &opencv::core::Range::new(3,4).unwrap()).unwrap();

//         //============================================================================================
//         let R_t = R.t().unwrap().to_mat().unwrap();
//         let mut t_t =   opencv::core::mul_mat_mat(&R_t, &t).unwrap().to_mat().unwrap();

//         unsafe {
//             *t_t.at_2d_unchecked_mut::<f32>(0, 0).unwrap() = *t_t.at_2d::<f32>(0, 0).unwrap() *-1.0;
//             *t_t.at_2d_unchecked_mut::<f32>(1, 0).unwrap() = *t_t.at_2d::<f32>(1, 0).unwrap()*-1.0;
//             *t_t.at_2d_unchecked_mut::<f32>(2, 0).unwrap() = *t_t.at_2d::<f32>(2, 0).unwrap()*-1.0;
//         }

//         let mut matrices = opencv::types::VectorOfMat::default();
//         println!("{},{}", R_t.rows(), R_t.cols());
//         println!("{},{}", t_t.rows(), t_t.cols());
//         matrices.push(R_t.clone());
//         matrices.push(t_t.clone());
//         let mut T_nk = Mat::default();

//         core::hconcat(&matrices, &mut T_nk )?;


//         //let mut P_nk =  Mat::rowscols(&curr_T, &opencv::core::Range::new(0,3).unwrap(), &opencv::core::Range::new(0,4).unwrap()).unwrap(); 

//         // =========================================================

//         //let mut P_nk =  pose.get_relative_projection_matrix();
//         let mut P_nk =   Mat::rowscols(&T_nk, &opencv::core::Range::new(0,3).unwrap(), &opencv::core::Range::new(0,4).unwrap()).unwrap(); 

//         println!("{},{}", P_nk.rows(), P_nk.cols());
//         //let mut P_nk = Mat::rowscols(&curr_T, &opencv::core::Range::new(0,3).unwrap(), &opencv::core::Range::new(0,4).unwrap()).unwrap();        

//         let P = opencv::core::mul_mat_mat(&K, &P_nk).unwrap().to_mat().unwrap();

//         println!("{},{}", P.rows(), P.cols());
//         let curr_P = P.clone();

//         let mut prev_P = opencv::core::mul_mat_mat(&K, &self.P_nk).unwrap().to_mat().unwrap();

//         //

//         let mut points4_d = Mat::default();
        
        
//         //TODO: use followingtriangulate_pointstriangulate_points
//         //opencv::sfm::triangulate_points()
//         //TODO: convert 2d points to 3d 
//         opencv::calib3d::triangulate_points(
//             &prev_P,
//             &curr_P, 
//             &proj_points1,
//             &proj_points2,
//             &mut points4_d)?;
//         let mut points3d = VectorOfPoint3f::default();
        

//         // Set new projection
//         self.P_nk = P_nk.clone();
//         self.T = curr_T.clone();

//         let mut points4_dnew = Mat::default();
//         opencv::core::transpose(&points4_d, &mut points4_dnew)?;
        
//         opencv::calib3d::convert_points_from_homogeneous(&points4_dnew, &mut points3d)?;

//         println!("{:?}", points3d);
        
//         let traing_point2d = self.get_2d_points_from_3d(&points3d);

        
//         //TODO: remove the below code: just for debugging
//         // let x_offset = 500 ;
//         // let y_offset = 375 ;
//         let x_offset = 500 ;

//         let y_offset = 375 ;
//         //let imtitle = "Estimated Trajectory".to_string();
//         for i in 0..points3d.len()
//         {   
//             let curr_pt = points3d.get(i).unwrap();
//             opencv::imgproc::circle(&mut self.traj_img, core::Point_::new(curr_pt.x as i32 +x_offset, curr_pt.y as i32 +y_offset), 1, core::Scalar_([0.0, 0.0, 255.0, 0.0]), -1, 8, 0)?;
//         }
        
//         let imtitle = "Estimated 3d points".to_string();
//         opencv::highgui::imshow(&imtitle, &self.traj_img)?;
//         opencv::highgui::wait_key(1)?; 

//     }
//     Ok(Status::done(()))
// }

*/

pub fn get_2d_points_from_3d(&self, points3d: &VectorOfPoint3f) -> VectorOfPoint2f
{
    let mut points2d = VectorOfPoint2f::new();
    for i in 0..points3d.len()
    {
        let curr_pt = points3d.get(i).unwrap();
        points2d.push(core::Point2f::new(curr_pt.x, curr_pt.y));        
    }
    return points2d;

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