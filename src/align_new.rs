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
pub struct AlignImgMsg {
    // Vector of image paths to read in/extract
    img1_kps: DVVectorOfKeyPoint,
    img1_des: DVMatrixGrayscale,
    img2_kps: DVVectorOfKeyPoint,
    img2_des: DVMatrixGrayscale,
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>
}

impl AlignImgMsg {
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


pub fn scale_transform(
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


#[derive(Debug, Clone)]
pub struct DarvisAlignFlow
{
  R: Mat,
  t: Mat,
  first_frame: bool  
}

impl DarvisAlignFlow
{
    pub fn new() -> DarvisAlignFlow {
      DarvisAlignFlow {
          R: Mat::default(),
          t: Mat::default(),
          first_frame: true
        }
    }
    // This is the handler that will be used by the actor.
// This is the handler that will be used by the actor.
pub fn align(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<AlignImgMsg>() {

        println!("testsetingsegsegdsf -------------------");
        // Convert back to cv structures
        let kp1 =  msg.img1_kps.cv_vector_of_keypoint();
        //let des1 = msg.img1_des.grayscale_to_cv_mat();
        let kp2 = msg.img2_kps.cv_vector_of_keypoint();
        //let des2 = msg.img2_des.grayscale_to_cv_mat();
        let curr_image = msg.img1_des.grayscale_to_cv_mat();
        let prev_image = msg.img2_des.grayscale_to_cv_mat();

        println!("testing....KeyPoint::convert");
        let mut prev_features  = opencv::types::VectorOfPoint2f::new();
        let pt_indx = opencv::types::VectorOfi32::new();
        core::KeyPoint::convert(&kp2, &mut prev_features, &pt_indx).unwrap();
        println!("testing....track_features");
        let curr_features = self.track_features(&prev_image, &curr_image, &mut prev_features);
        println!("testing....calculate_transform");
        let (R, t) = self.calculate_transform(&curr_features, &mut prev_features);
        
        println!("testing....");
        


        // // BFMatcher to get good matches
        // let bfmtch = BFMatcher::create(4, true).unwrap(); 
        // let mut mask = Mat::default(); 
        // let mut matches = VectorOfDMatch::new();
        // bfmtch.train_match(&des2, &des1, &mut matches, &mut mask)?; 

        // // Sort the matches based on the distance in ascending order
        // // Using O(n^2) sort here. Need to make the code use cv sort function
        // // by providing custom comparator

        // let mut sortedMatches = VectorOfDMatch::new();
        // let mut added = vec![false; matches.len()];
        // for i in 0..matches.len() {
        //     if added[i] == true {
        //         continue;
        //     }
        //     let mut mn = i;
        //     let mut dist = matches.get(i).unwrap().distance;
        //     for j in 0..matches.len() {
        //         let dmatch2 = matches.get(j).unwrap();
        //         if dist > dmatch2.distance && !added[j] {
        //             mn = j;
        //             dist = dmatch2.distance;
        //         }        
        //     }      
        //     let dmatch1 = matches.get(mn).unwrap();
        //     sortedMatches.push(dmatch1);
        //     added[mn] = true;
        // }

        // // Point vectors to hold the corresponding matched points 
        // let mut p2f1 = core::Vector::<core::Point2f>::new(); 
        // let mut p2f2 = core::Vector::<core::Point2f>::new();

        // for i in 0..sortedMatches.len() {
        //     p2f1.push(kp1.get(sortedMatches.get(i).unwrap().train_idx.try_into().unwrap()).unwrap().pt);
        //     p2f2.push(kp2.get(sortedMatches.get(i).unwrap().query_idx.try_into().unwrap()).unwrap().pt);
        // }
         
        // //getting rid of points for which the KLT tracking failed or those who have gone outside the framed
        // let mut indez_correction = 0;
        // for i in 0..p2f2.len() {
        //   let pt = p2f2.get(i - indez_correction).unwrap();
        //   if pt.x < 0.0 || pt.y < 0.0 {

        //     p2f1.remove(i - indez_correction).unwrap();
        //     p2f2.remove(i - indez_correction).unwrap();
        //     indez_correction = indez_correction + 1;
        //   }
        // }


        // // Find essential matrix using K
        // let mut K = Mat::new_rows_cols_with_default(3, 3 ,CV_32FC1, opencv::core::Scalar::all(0.0)).unwrap();
        // // Mat K = (Mat_<double>(3,3) << 718.8560, 0, 607.1928, 0, 718.8560, 185.2157, 0, 0, 1 );
        // unsafe {
        //     *K.at_2d_unchecked_mut::<f32>(0,0).unwrap() = 718.8560;
        //     *K.at_2d_unchecked_mut::<f32>(0,1).unwrap() = 0.0;
        //     *K.at_2d_unchecked_mut::<f32>(0,2).unwrap() = 607.1928;
        //     *K.at_2d_unchecked_mut::<f32>(1,0).unwrap() = 0.0;
        //     *K.at_2d_unchecked_mut::<f32>(1,1).unwrap() = 718.8560;
        //     *K.at_2d_unchecked_mut::<f32>(1,2).unwrap() = 185.2157;
        //     *K.at_2d_unchecked_mut::<f32>(2,0).unwrap() = 0.0;
        //     *K.at_2d_unchecked_mut::<f32>(2,1).unwrap() = 0.0;
        //     *K.at_2d_unchecked_mut::<f32>(2,2).unwrap() = 1.0;
        // }
        // let mut ess_mat = calib3d::find_essential_mat_matrix(&mut p2f1, &mut p2f2, &mut K, calib3d::FM_RANSAC, 0.999, 1.0, &mut Mat::default()).unwrap();

        // // Recover pose using the matrix
        // let mut R = Mat::default();
        // let mut t = Mat::default();
        // let inliers = calib3d::recover_pose_camera(&mut ess_mat, &mut p2f1, &mut p2f2, &mut K, &mut R, &mut t, &mut Mat::default());
        // // println!("Number of inliers:{:}", inliers.unwrap());
        // // print_matrix(&R);
        // // print_matrix(&t);



        // // let focal =  718.8560;
        // // let pp =  core::Point2d::new(607.1928, 185.2157);

        // // let inliers = calib3d::recover_pose( &ess_mat, &p2f1, &p2f2, &mut K, &mut t, focal, pp, &mut Mat::default());    

        // // if !self.first_frame
        // // {
        // //   let (R, t) = scale_transform(0.71, &self.R, &self.t, &R, &t);
        // //   self.R = R;
        // //   self.t = t;
        // // }
        // self.first_frame = false;
        self.R = R;
        self.t = t;

        let mut pose = Pose::default_ones();
        pose.pos[0] = *self.t.at::<f64>(0).unwrap();
        pose.pos[1] = *self.t.at::<f64>(1).unwrap();
        pose.pos[2] = *self.t.at::<f64>(2).unwrap();

        pose.rot[(0,0)] = *self.R.at_2d::<f64>(0,0).unwrap();
        pose.rot[(0,1)] = *self.R.at_2d::<f64>(0,1).unwrap();
        pose.rot[(0,2)] = *self.R.at_2d::<f64>(0,2).unwrap();
        pose.rot[(1,0)] = *self.R.at_2d::<f64>(1,0).unwrap();
        pose.rot[(1,1)] = *self.R.at_2d::<f64>(1,1).unwrap();
        pose.rot[(1,2)] = *self.R.at_2d::<f64>(1,2).unwrap();
        pose.rot[(2,0)] = *self.R.at_2d::<f64>(2,0).unwrap();
        pose.rot[(2,1)] = *self.R.at_2d::<f64>(2,1).unwrap();
        pose.rot[(2,2)] = *self.R.at_2d::<f64>(2,2).unwrap();

        // println!("{}", pose.pos);
        // println!("{}", pose.rot);

        //let vis_id = context.system.find_aid_by_name("visulization").unwrap();
        let vis_id = msg.actor_ids.get("visulization").unwrap();
        vis_id.send_new(VisMsg::new(pose, msg.actor_ids.clone())).unwrap();
    }
    Ok(Status::done(()))
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

  pub fn track_features(
    &self,
    src_image: &Mat,
    dst_image: &Mat,
    src_image_points: &mut opencv::types::VectorOfPoint2f,
  ) -> opencv::types::VectorOfPoint2f {
    let mut status = opencv::types::VectorOfu8::new();
    let mut dst_image_points = opencv::types::VectorOfPoint2f::new(); //vectors to store the coordinates of the feature points
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
    points1: &mut opencv::types::VectorOfPoint2f,
    points2: &mut opencv::types::VectorOfPoint2f,
    status: &mut opencv::types::VectorOfu8,
  ) {
    //this function automatically gets rid of points for which tracking fails
    let mut err = opencv::types::VectorOff32::default();
    let win_sz = 21; //21
    let win_size = core::Size::new(win_sz, win_sz);
    let termcrit = opencv::core::TermCriteria {
      typ: 3,
      max_count: 30,
      epsilon: 0.01,
    };
    let max_level = 1;
    println!("before calc optical");
    opencv::video::calc_optical_flow_pyr_lk(
      img_1, img_2, points1, points2, status, &mut err, win_size, max_level, termcrit, 0, 0.001,
    )
    .unwrap();

    println!("after calc optical");
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

impl Function for DarvisAlignFlow {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.align(_context, message).unwrap();
        Ok(Status::done(()))
    }

}



// use opencv::core::*;

// use opencv::imgproc::*;

// fn main1() {
//     let file_path = "/home/pranayspeed/Work/git_repos/00/image_2";
//     let scan_file_path = "/home/pranayspeed/Work/git_repos";
//     let mut vo = VisualOdometry::new(file_path, scan_file_path);
  
//     let frame1 = vo.get_bw_frame(0);
//     let frame2 = vo.get_bw_frame(1);
  
//     let mut points1 = vo.extract_features(&frame1);
//     let points2 = vo.track_features(&frame1, &frame2, &mut points1);
//     let (recover_r, recover_t) = vo.calculate_transform(&points1, &points2);
  
//     let mut prev_image = frame2;
//     //let mut curr_image: Mat;
//     let mut prev_features = points2;
  
//     vo.set_rotation(recover_r.clone());
//     vo.set_translation(recover_t.clone());
  
//     let font_face: i32 = FONT_HERSHEY_PLAIN;
//     let font_scale: f64 = 1.0;
//     let thickness: i32 = 1;
//     let text_org = Point::new(10, 50);
  
//     opencv::highgui::named_window("Road facing camera", opencv::highgui::WINDOW_AUTOSIZE).unwrap(); // Create a window for display.
//     opencv::highgui::named_window("Trajectory", opencv::highgui::WINDOW_AUTOSIZE).unwrap(); // Create a window for display.
  
//     let mut traj = opencv::core::Mat::zeros(600, 600, CV_8UC3)
//       .unwrap()
//       .to_mat()
//       .unwrap();
  
//     for num_frame in 2..vo.get_max_frame() {
//       let curr_image_c = vo.get_frame(num_frame);
//       let curr_image = vo.get_bw_from_color(&curr_image_c);
//       //let mut curr_features = vo.extract_features(&curr_image);
//       let mut curr_features = vo.track_features(&prev_image, &curr_image, &mut prev_features);
  
//       let (recover_r, recover_t) = vo.calculate_transform(&curr_features, &mut prev_features);
  
//       let scale = vo.get_scale(num_frame, &vo.get_translation());
//       let (rotation, translation) =
//         vo.scale_transform(scale, &vo.get_rotation(), &vo.get_translation(), &recover_r, &recover_t);
//         vo.set_rotation(rotation);
//         vo.set_translation(translation);
  
//       // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
//       if prev_features.len() < vo.get_min_num_feat() as usize {
//         //cout << "Number of tracked features reduced to " << prev_features.len() << endl;
//         //cout << "trigerring redection" << endl;
//         prev_features = vo.extract_features(&prev_image);
//         curr_features = vo.track_features(&prev_image, &curr_image, &mut prev_features);
//       }
  
//       prev_image = curr_image.clone();
//       prev_features = curr_features;
  
//       ////////////////////////////////////Show on GUI///////////////////////////////////////////////////
  
//       let x_c = *vo.get_translation().at::<f64>(0).unwrap() as i32 + 300;
//       let y_c = *vo.get_translation().at::<f64>(2).unwrap() as i32 + 100;
  
//       circle(
//         &mut traj,
//         Point { x: x_c, y: y_c },
//         1,
//         Scalar::new(255.0, 0.0, 0.0, 0.0),
//         2,
//         LINE_8,
//         0,
//       )
//       .unwrap();
  
//       rectangle(
//         &mut traj,
//         Rect2i {
//           x: 10,
//           y: 30,
//           width: 550,
//           height: 50,
//         },
//         Scalar::new(0.0, 0.0, 0.0, 0.0),
//         CV_FILLED,
//         0,
//         0,
//       )
//       .unwrap();
//       let text = format!(
//         "Coordinates: x = {:.2}m y = {:.2}m z = {:.2}m",
//         vo.get_translation().at::<f64>(0).unwrap(),
//         vo.get_translation().at::<f64>(1).unwrap(),
//         vo.get_translation().at::<f64>(2).unwrap()
//       );
  
//       opencv::imgproc::put_text(
//         &mut traj,
//         &text,
//         text_org,
//         font_face,
//         font_scale,
//         Scalar::new(255.0, 255.0, 255.0, 255.0),
//         thickness,
//         8,
//         false,
//       )
//       .unwrap();
  
//       opencv::highgui::imshow("Road facing camera", &curr_image_c).unwrap();
//       opencv::highgui::imshow("Trajectory", &traj).unwrap();
  
//       opencv::highgui::wait_key(1).unwrap();
//     }
//   }



// pub struct VisualOdometry {
//   rotation: Mat,
//   translation: Mat,
//   file_path: String,
//   scale_file_path: String,
//   max_frame: i32,
//   min_num_feat: i32,
//   focal: f64,
//   pp: Point2d,
// }

// impl VisualOdometry {
//   //Construct person
//   pub fn new(filepath: &str, scale_filepath: &str) -> VisualOdometry {
//     VisualOdometry {
//       rotation: Mat::default(),
//       translation: Mat::default(),
//       file_path: String::from(filepath),
//       scale_file_path: String::from(scale_filepath),
//       max_frame: 1000,
//       min_num_feat: 2000,
//       focal: 718.8560,
//       pp: Point2d::new(607.1928, 185.2157),
//     }
//   }

  
//   // //TODO: prune this function if not needed 
//   // pub fn get_frame(&self, frame_id: i32) -> Mat {
//   //   let filename = format!("{}{}", self.file_path, format!("/{:01$}.png", frame_id, 6));
//   //   imread(&filename, imgcodecs::IMREAD_COLOR).unwrap()
//   // }
//   // //TODO: prune this function if not needed 
//   // pub fn get_bw_frame(&self, frame_id: i32) -> Mat {
//   //   let filename = format!("{}{}", self.file_path, format!("/{:01$}.png", frame_id, 6));
//   //   let curr_image_c = imread(&filename, imgcodecs::IMREAD_COLOR).unwrap();
//   //   let mut img_1 = Mat::default();
//   //   cvt_color(&curr_image_c, &mut img_1, COLOR_BGR2GRAY, 0).unwrap();
//   //   img_1
//   // }
//   // //TODO: prune this function if not needed 
//   // pub fn get_bw_from_color(&self, img_color: &Mat) -> Mat {
//   //   let mut img_1 = Mat::default();
//   //   cvt_color(img_color, &mut img_1, COLOR_BGR2GRAY, 0).unwrap();
//   //   img_1
//   // }
//   // //TODO: prune this function if not needed 
//   // pub fn extract_features(&self, curr_image: &Mat) -> VectorOfPoint2f {
//   //   let mut points1 = VectorOfPoint2f::new(); //vectors to store the coordinates of the feature points
//   //   self.feature_detection(curr_image, &mut points1); //detect features in img_1
//   //   points1
//   // }
  

//   //TODO: prune this function if not needed 
//   // pub fn track_features(
//   //   &self,
//   //   src_image: &Mat,
//   //   dst_image: &Mat,
//   //   src_image_points: &mut VectorOfPoint2f,
//   // ) -> VectorOfPoint2f {
//   //   let mut status = VectorOfu8::new();
//   //   let mut dst_image_points = VectorOfPoint2f::new(); //vectors to store the coordinates of the feature points
//   //   self.feature_tracking(
//   //     &src_image,
//   //     &dst_image,
//   //     src_image_points,
//   //     &mut dst_image_points,
//   //     &mut status,
//   //   ); //track those features to img_2
//   //   dst_image_points
//   // }

//   //TODO: absoulte scale estimation using features 
//   // Below implementation use the ground truth for absoulte scale estimation
//   // pub fn get_scale(&self, frame_id: i32, transform: &Mat) -> f64 {
//   //   self.get_absolute_scale(
//   //     frame_id,
//   //     "00",
//   //     *transform.at::<f64>(2).unwrap(),
//   //     &self.scale_file_path,
//   //   )
//   // }

//   // pub fn get_absolute_scale(
//   //   &self,
//   //   frame_id: i32,
//   //   sequence_id: &str,
//   //   z_cal: f64,
//   //   file_path: &str,
//   // ) -> f64 {
//   //   let mut i = 0;

//   //   let filepath = format!("{}/{}.txt", &file_path, sequence_id);
//   //   let filepath_path = Path::new(&filepath);

//   //   let (mut x, mut y, mut z): (f64, f64, f64) = (0.0, 0.0, 0.0);

//   //   let (mut x_prev, mut y_prev, mut z_prev): (f64, f64, f64) = (0.0, 0.0, 0.0);
//   //   // Open the path in read-only mode, returns `io::Result<File>`
//   //   if let Ok(lines) = read_lines(filepath_path) {
//   //     // Consumes the iterator, returns an (Optional) String
//   //     for line in lines {
//   //       if let Ok(val) = line {
//   //         z_prev = z;
//   //         x_prev = x;
//   //         y_prev = y;

//   //         println!("x : {}, y: {}, z:{} ", x, y, z);
//   //         let values: Vec<&str> = val.split(" ").collect();
//   //         for (j, s) in values.iter().enumerate() {
//   //           z = s.parse::<f64>().unwrap();
//   //           if j == 7 {
//   //             y = z;
//   //           }
//   //           if j == 3 {
//   //             x = z;
//   //           }
//   //         }
//   //       }
//   //       i = i + 1;
//   //       if i > frame_id {
//   //         break;
//   //       }
//   //     }
//   //   }

//   //   f64::sqrt(
//   //     (x - x_prev) * (x - x_prev) + (y - y_prev) * (y - y_prev) + (z - z_prev) * (z - z_prev),
//   //   )
//   // }

//   pub fn calculate_transform(
//     &self,
//     curr_features: &VectorOfPoint2f,
//     prev_features: &VectorOfPoint2f,
//   ) -> (Mat, Mat) {
//     //recovering the pose and the essential matrix
//     let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
//     let essential_mat = opencv::calib3d::find_essential_mat(
//       &curr_features,
//       &prev_features,
//       self.focal,
//       self.pp,
//       opencv::calib3d::RANSAC,
//       0.999,
//       1,
//       &mut mask,
//     )
//     .unwrap();
//     opencv::calib3d::recover_pose(
//       &essential_mat,
//       &curr_features,
//       &prev_features,
//       &mut recover_r,
//       &mut recover_t,
//       self.focal,
//       self.pp,
//       &mut mask,
//     )
//     .unwrap();

//     (recover_r, recover_t)
//   }

//   pub fn scale_transform(
//     &self,
//     scale: f64,
//     rotation: &Mat,
//     translation: &Mat,
//     recover_mat: &Mat,
//     t: &Mat,
//   ) -> (Mat, Mat) {
//     //R = recover_mat, R_f= rotation, t_f =translation , t
//     if scale > 0.1
//       && (t.at::<f64>(2).unwrap() > t.at::<f64>(0).unwrap())
//       && (t.at::<f64>(2).unwrap() > t.at::<f64>(1).unwrap())
//     {
//       // t_f = t_f + scale*(R_f*t);
//       let mut rf_cross_t = opencv::core::mul_mat_mat(&rotation, &t).unwrap();
//       rf_cross_t = opencv::core::mul_matexpr_f64(&rf_cross_t, scale).unwrap();
//       let t_f = opencv::core::add_mat_matexpr(&translation, &rf_cross_t)
//         .unwrap()
//         .to_mat()
//         .unwrap();

//       // R_f = R*R_f;
//       let r_f = opencv::core::mul_mat_mat(&recover_mat, &rotation)
//         .unwrap()
//         .to_mat()
//         .unwrap();

//       (r_f, t_f)
//     } else {
//       println!("scale below 0.1, or incorrect translation");
//       (rotation.clone(), translation.clone())
//     }
//   }

//   pub fn feature_tracking(
//     &self,
//     img_1: &opencv::core::Mat,
//     img_2: &opencv::core::Mat,
//     points1: &mut VectorOfPoint2f,
//     points2: &mut VectorOfPoint2f,
//     status: &mut VectorOfu8,
//   ) {
//     //this function automatically gets rid of points for which tracking fails
//     let mut err = opencv::types::VectorOff32::default();
//     let win_size = Size::new(21, 21);
//     let termcrit = opencv::core::TermCriteria {
//       typ: 3,
//       max_count: 30,
//       epsilon: 0.01,
//     };
//     let max_level = 3;
//     opencv::video::calc_optical_flow_pyr_lk(
//       img_1, img_2, points1, points2, status, &mut err, win_size, max_level, termcrit, 0, 0.001,
//     )
//     .unwrap();
//     //getting rid of points for which the KLT tracking failed or those who have gone outside the framed
//     let mut indez_correction = 0;
//     for i in 0..status.len() {
//       let pt = points2.get(i - indez_correction).unwrap();
//       if (status.get(i).unwrap() == 0) || pt.x < 0.0 || pt.y < 0.0 {
//         if pt.x < 0.0 || pt.y < 0.0 {
//           status.set(i, 0).unwrap();
//         }
//         points1.remove(i - indez_correction).unwrap();
//         points2.remove(i - indez_correction).unwrap();
//         indez_correction = indez_correction + 1;
//       }
//     }
//   }

//   pub fn feature_detection(&self, img_1: &Mat, points1: &mut Vector<Point2f>) {
//     //uses FAST as of now, modify parameters as necessary
//     let mut keypoints_1 = opencv::types::VectorOfKeyPoint::new();
//     let fast_threshold: i32 = 20;
//     let non_max_suppression: bool = true;
//     opencv::features2d::FAST(img_1, &mut keypoints_1, fast_threshold, non_max_suppression).unwrap();

//     let pt_indx = opencv::types::VectorOfi32::new();
//     KeyPoint::convert(&keypoints_1, points1, &pt_indx).unwrap();
//   }

//   pub fn keypoint_to_point2f(&self, keypoints_1: &VectorOfKeyPoint, points1: &mut Vector<Point2f>) {
//     //Convert VectorOfKeyPoint to Vector of Points2f
//     let pt_indx = opencv::types::VectorOfi32::new();
//     KeyPoint::convert(&keypoints_1, points1, &pt_indx).unwrap();
//   }


//   pub fn set_rotation(&mut self, rotation: Mat)
//   {
//     self.rotation = rotation;
//   }

//   pub fn set_translation(&mut self, translation: Mat)
//   {
//     self.translation = translation;
//   }

//   pub fn get_rotation(&self) -> &Mat
//   {
//     &self.rotation
//   }

//   pub fn get_translation(&self) -> &Mat
//   {
//     &self.translation
//   }

//   pub fn get_max_frame(&self) -> i32
//   {
//     self.max_frame
//   }

//   pub fn get_min_num_feat(&self) -> i32
//   {
//     self.min_num_feat
//   }

// }
  