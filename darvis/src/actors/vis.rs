/* OLD VISUALIZER */

use opencv::{prelude::*, core, highgui, imgproc, imgcodecs};
use dvcore::{plugin_functions::Function,matrix::*,};
use crate::{actors::messages::VisMsg, dvmap::pose::{Translation, Rotation},};

use super::messages::VisPathMsg;

#[derive(Debug, Clone)]
/// Vis state data
pub struct DarvisVis {
    /// Trajectory image for visualization
    traj_img: Mat, 
    /// Camera image for visualization
    cam_img: Mat,
    /// Built up trajectory translation 
    traj_pos: Translation, 
    /// Built up trajectory rotation
    traj_rot: Rotation, 
}

impl DarvisVis {
    /// Constructor
    pub fn new() -> DarvisVis {
        DarvisVis {
            //traj_img: Mat::new_rows_cols_with_default(750, 1000, core::CV_8UC3, core::Scalar::all(0.0)).unwrap(),
            traj_img: Mat::new_rows_cols_with_default(376, 500, core::CV_8UC3, core::Scalar::all(0.0)).unwrap(),
            cam_img: Mat::default(),
            traj_pos: DVVector3::zeros::<f64>(),
            traj_rot: DVMatrix3::zeros::<f64>(),
        }
    }

    /// Visualize core function to be called by the action framework that process VisMsg type message.
    pub fn visualize(&mut self, _context: Context, message: Message) -> ActorResult<()> {
        if let Some(msg) = message.content_as::<VisMsg>() {
            // rotate and translate matrices from pose to track trajectory (R and t from pose)
            if self.traj_pos.is_zero() && self.traj_rot.is_zero() {
                self.traj_pos = msg.new_pose.get_translation();
                self.traj_rot = msg.new_pose.get_rotation();
            }
            else {
                self.traj_pos = DVVector3::new((*self.traj_pos) + (*self.traj_rot) * (*msg.new_pose.get_translation()));
                self.traj_rot = DVMatrix3::new((*msg.new_pose.get_rotation()) * (*self.traj_rot));
            }

            // Draw new circle on image and show
            let x = self.traj_pos[0] as i32 / 2;
            let y = -self.traj_pos[2] as i32 / 2 ;

            // let x_offset = 500 ;
            // let y_offset = 375 ;
            let x_offset = 200 ;
            let y_offset = 350 ;
            //let imtitle = "Estimated Trajectory".to_string();

            imgproc::circle(&mut self.traj_img, core::Point_::new(x+x_offset, y+y_offset), 3, core::Scalar::new(0.0, 0.0, 255.0, 0.0), -1, 8, 0)?;

            //highgui::imshow(&imtitle, &self.traj_img)?;
            //highgui::wait_key(1)?;   
        }
        else if let Some(msg) = message.content_as::<VisPathMsg>() {
            //let imtitle = "Camera Frame".to_string();
            self.cam_img = imgcodecs::imread(&msg.last_img_path, imgcodecs::IMREAD_COLOR)?;
            //highgui::imshow(&imtitle, &self.cam_img)?;
            //highgui::wait_key(1)?;   
        }

        if self.cam_img.rows()>0 && self.traj_img.rows() >0 {
            let mut out = Mat::default();
            let mut matrices = opencv::types::VectorOfMat::default();
            matrices.push(self.cam_img.clone());
            matrices.push(self.traj_img.clone());

            core::hconcat( &matrices, &mut out )?;
            let imtitle = "Estimated Trajectory".to_string();
            highgui::imshow(&imtitle, &out)?;
            highgui::wait_key(1)?;
        }

        Ok(Status::done(()))
    }
}

impl Function for DarvisVis {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.visualize(_context, message).unwrap();
        Ok(Status::done(()))
    }

}