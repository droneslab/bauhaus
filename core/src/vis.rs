use opencv::{
    prelude::*,
    core,
    features2d,
    features2d::{Feature2DTrait, ORB},
    highgui,
    imgproc,
    videoio,
    imgcodecs,
    types::{PtrOfORB, VectorOfKeyPoint},
};
use axiom::prelude::*;
use serde::{Deserialize, Serialize};
extern crate nalgebra as na;
use na::*;
use crate::base::Pose;


// Public message struct for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct VisMsg {
    // Pose of image paths to read in/extract, Poses take 2 matrixes, pos and rot <int type, # rows, # col, data storage?>
    new_pose: Pose,
    // all actor ids
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>
}

impl VisMsg {
    pub fn new(pose: Pose, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            new_pose: pose,
            actor_ids: ids,
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct VisPathMsg {
    last_img_path: String
}

impl VisPathMsg {
    pub fn new(img_path: String) -> Self {
        Self {
            last_img_path: img_path,
        }
    }
}

#[derive(Debug, Clone)]
// Vis state data
pub struct Vis {
    traj_img: Mat, // Trajectory image for visualization
    cam_img: Mat, // Camera image for visualization
    traj_pos: Vector3<f64>, // Built up trajectory translation
    traj_rot: Matrix3<f64>, // Built up trajectory rotation
    // actor_ids: std::collections::HashMap<String, axiom::actors::Aid>, // Collection of all spawned actor ids
}



impl Vis {
    pub fn new() -> Vis {
        Vis {
            traj_img: Mat::new_rows_cols_with_default(750, 1000, core::CV_8UC3, core::Scalar::all(0.0)).unwrap(),
            cam_img: Mat::default(),
            traj_pos: Vector3::zeros(),
            traj_rot: Matrix3::zeros(),
            // actor_ids: ids,
        }
    }

    pub async fn visualize(mut self, context: Context, message: Message) -> ActorResult<Self> {
        if let Some(msg) = message.content_as::<VisMsg>() {
            // rotate and translate matrices from pose to track trajectory (R and t from pose)
            if self.traj_pos == Vector3::zeros() && self.traj_rot == Matrix3::zeros() {
                self.traj_pos = msg.new_pose.pos;
                self.traj_rot = msg.new_pose.rot;
            }
            else {
                self.traj_pos = self.traj_pos + self.traj_rot*&msg.new_pose.pos;
                self.traj_rot = &msg.new_pose.rot * self.traj_rot;
            }
                        
            // Draw new circle on image and show
            let x = self.traj_pos[0] as i32;
            let y = -self.traj_pos[2] as i32;

            let x_offset = 500;
            let y_offset = 375;

            imgproc::circle(&mut self.traj_img, core::Point_::new(x+x_offset, y+y_offset), 3, core::Scalar_([0.0, 0.0, 255.0, 0.0]), -1, 8, 0)?;
            highgui::imshow("Trajectory", &self.traj_img)?;
            highgui::wait_key(1)?;   
        }
        else if let Some(msg) = message.content_as::<VisPathMsg>() {
            self.cam_img = imgcodecs::imread(&msg.last_img_path, imgcodecs::IMREAD_COLOR)?;
            highgui::imshow("Camera Image", &self.cam_img)?;
            highgui::wait_key(1)?;   
        }
        Ok(Status::done(self))
    }

    pub fn visualize1(&mut self, context: Context, message: Message) -> ActorResult<&Self> {
        if let Some(msg) = message.content_as::<VisMsg>() {
            // rotate and translate matrices from pose to track trajectory (R and t from pose)
            if self.traj_pos == Vector3::zeros() && self.traj_rot == Matrix3::zeros() {
                self.traj_pos = msg.new_pose.pos;
                self.traj_rot = msg.new_pose.rot;
            }
            else {
                self.traj_pos = self.traj_pos + self.traj_rot*&msg.new_pose.pos;
                self.traj_rot = &msg.new_pose.rot * self.traj_rot;
            }
                        
            // Draw new circle on image and show
            let x = self.traj_pos[0] as i32;
            let y = -self.traj_pos[2] as i32;

            let x_offset = 500;
            let y_offset = 375;

            imgproc::circle(&mut self.traj_img, core::Point_::new(x+x_offset, y+y_offset), 3, core::Scalar_([0.0, 0.0, 255.0, 0.0]), -1, 8, 0)?;
            highgui::imshow("Trajectory", &self.traj_img)?;
            highgui::wait_key(1)?;   
        }
        else if let Some(msg) = message.content_as::<VisPathMsg>() {
            self.cam_img = imgcodecs::imread(&msg.last_img_path, imgcodecs::IMREAD_COLOR)?;
            highgui::imshow("Camera Image", &self.cam_img)?;
            highgui::wait_key(1)?;   
        }
        Ok(Status::done(self))
    }
}

unsafe impl Send for Vis {}
unsafe impl Sync for Vis {}

