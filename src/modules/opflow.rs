use axiom::prelude::*;

use opencv::{
    prelude::*,
    core,
    imgcodecs,
    types::{VectorOfKeyPoint, VectorOfPoint2f, VectorOfu8},
};

use darvis::base::*;
use darvis::plugin_functions::*;

use crate::modules::frameloader::*;
use crate::registered_modules::{VISUALIZER};
use crate::modules::vis::VisPathMsg;


#[derive(Debug, Clone)]
pub struct DarvisOpFlow
{
  focal: f64,
  pp: core::Point2d,
  min_num_feat: i32
}

impl DarvisOpFlow
{
    pub fn new() -> DarvisOpFlow {
        DarvisOpFlow {
          focal: 718.8560,
          pp: core::Point2d::new(607.1928, 185.2157),
          min_num_feat: 2000
        }
    }


pub fn opflow_extract(&mut self, context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<ImagesMsg>() {


        let mut curr_image = Mat::default();
        let mut prev_image = Mat::default();
        let mut curr_features = VectorOfPoint2f::new();
        let mut prev_features = VectorOfPoint2f::new();
        let recover_r = Mat::default();
        let recover_t = Mat::default();
        let mut first_img = true;
        for path in msg.get_img_paths() {
          curr_image = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE)?;

            if first_img 
            {
              curr_features = self.extract_features(&curr_image);
              first_img = false;
              
            }
            else //points1.len() > 0 && points2.len() > 0 
            {

              curr_features = self.track_features(&prev_image, &curr_image, &mut prev_features);

              let (recover_r, recover_t) = self.calculate_transform(&curr_features, &mut prev_features);
        
              //// TODO: Scaling not implemented
              // let mut curr_pos = Point2d::new(0.0,0.0);
              // let scale = self.get_scale(num_frame, &self.get_translation(), &mut curr_pos);         
              //let (rotation, translation) = self.scale_transform(scale, &self.get_rotation(), &self.get_translation(), &recover_r, &recover_t);
                //self.set_rotation(rotation);
                //self.set_translation(translation);
          
              // a redetection is triggered in case the number of feautres being trakced go below a particular threshold
              if prev_features.len() < self.get_min_num_feat() as usize {
                //cout << "Number of tracked features reduced to " << prev_features.len() << endl;
                //cout << "trigerring redection" << endl;
                prev_features = self.extract_features(&prev_image);
                curr_features = self.track_features(&prev_image, &curr_image, &mut prev_features);
              }


              // let align_id = msg.actor_ids.get(TRACKER).unwrap();
              // let vis_id = msg.actor_ids.get(VISUALIZER).unwrap();
              // // println!("{}", align_id);
              // // TODO: This is just a test send for now. Need to change message to accept the custom DarvisKeyPoint type
              // println!("Processed image: {}", path);
              // vis_id.send_new(VisPathMsg::new(path.to_string())).unwrap();
              // align_id.send_new(AlignMsg::new(kpvec1, nades1, kpvec2, nades2, msg.actor_ids.clone())).unwrap();

              let t = recover_t;
              let R = recover_r;

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
      
              //let vis_id = context.system.find_aid_by_name(VISUALIZER).unwrap();
              //let vis_id = msg.get_actor_ids().get(VISUALIZER).unwrap();
              //vis_id.send_new(VisMsg::new(pose, msg.get_actor_ids().clone())).unwrap();




            }
            let vis_id = msg.get_actor_ids().get(VISUALIZER).unwrap();
            // println!("{}", align_id);
            // TODO: This is just a test send for now. Need to change message to accept the custom DarvisKeyPoint type
            println!("Processed image: {}", path);
            vis_id.send_new(VisPathMsg::new(path.to_string())).unwrap();

            
              prev_image = curr_image.clone();
              prev_features = curr_features;


              
            }

          // context.system.trigger_shutdown();
        }
        
        Ok(Status::done(()))
    }
    


pub fn feature_tracking(
    &self,
    img_1: &opencv::core::Mat,
    img_2: &opencv::core::Mat,
    points1: &mut VectorOfPoint2f,
    points2: &mut VectorOfPoint2f,
    status: &mut VectorOfu8,
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

  pub fn extract_features(&self, curr_image: &Mat) -> VectorOfPoint2f {
    let mut points1 = VectorOfPoint2f::new(); //vectors to store the coordinates of the feature points
    
    self.feature_detection(curr_image, &mut points1); //detect features in img_1
    points1
  }

  pub fn feature_detection(&self, img_1: &Mat, points1: &mut VectorOfPoint2f) {
    //uses FAST as of now, modify parameters as necessary
    //let mut keypoints_1 = opencv::types::VectorOfKeyPoint::new();
    let fast_threshold: i32 = 20;
    let non_max_suppression: bool = true;
    let mut keypoints_1 = VectorOfKeyPoint::new();
    opencv::features2d::FAST(img_1, &mut keypoints_1, fast_threshold, non_max_suppression).unwrap();

    let pt_indx = opencv::types::VectorOfi32::new();
    core::KeyPoint::convert(&keypoints_1, points1, &pt_indx).unwrap();
  }


  pub fn calculate_transform(
    &self,
    curr_features: &VectorOfPoint2f,
    prev_features: &VectorOfPoint2f,
  ) -> (Mat, Mat) {
    //recovering the pose and the essential matrix
    let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
    let essential_mat = opencv::calib3d::find_essential_mat_2(
      &curr_features,
      &prev_features,
      self.focal,
      self.pp,
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
      self.focal,
      self.pp,
      &mut mask,
    )
    .unwrap();

    (recover_r, recover_t)
  }

  pub fn track_features(
    &self,
    src_image: &Mat,
    dst_image: &Mat,
    src_image_points: &mut VectorOfPoint2f,
  ) -> VectorOfPoint2f {
    let mut status = VectorOfu8::new();
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

  pub fn get_min_num_feat(&self) -> i32
  {
    self.min_num_feat
  }

}


impl Function for DarvisOpFlow {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.opflow_extract(_context, message).unwrap();
        Ok(Status::done(()))
    }

}
