#[allow(unused_imports)]
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
use opencv::core::CV_32FC1;
extern crate nalgebra as na;
use na::*;
use crate::base::Pose;


// Public message struct for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct VisMsg {
    // The ID of the actor that sent this message
    aid: Aid,
    // Pose of image paths to read in/extract, Poses take 2 matrixes, pos and rot <int type, # rows, # col, data storage?>
    new_pose: Pose,
    // all actor ids
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>
}

impl VisMsg {
    pub fn new(aid: axiom::actors::Aid, pose: Pose, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
            aid,
            new_pose: pose,
            actor_ids: ids,
        }
    }
}

// Vis state data
pub struct Vis {
    img: Mat, // Image for visualization
    traj_pos: Vector3<f64>, // Built up trajectory translation
    traj_rot: Matrix3<f64>, // Built up trajectory rotation
    // actor_ids: std::collections::HashMap<String, axiom::actors::Aid>, // Collection of all spawned actor ids
}

impl Vis {
    pub fn new() -> Vis {
        Vis {
            img: Mat::new_rows_cols_with_default(400, 400, core::CV_8UC3, core::Scalar::all(0.0)).unwrap(),
            traj_pos: Vector3::new(1.0,1.0,1.0),
            traj_rot: Matrix3::new(1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0),
            // actor_ids: ids,
        }
    }

    pub async fn visualize(mut self, context: Context, message: Message) -> ActorResult<Self> {
        // println!("{:?}", context);
        if let Some(msg) = message.content_as::<VisMsg>() {
            println!("{:?}", context);
    
            // let mut img = na_grayscale_to_cv_mat(&msg.img);
    
           
    
            // // rotate and translate matrices from pose to track trajectory (R and t from pose)
            // // Rpos = RRpos
            // let new_Rpos = &msg.img_pose.rot * Rpos;
            // // tpos = tpose + tRpos
            // let new_tpos = tpos + (msg.img_pose.pos * Rpos);
            
            // // update image with a small red square (there is no 'circle struct in opencv::core)
            
            // let x = new_Rpos[0] as i32;
            // let y = new_Rpos[1] as i32;
    
            // // use x, y coordinates until we can extract them from Rpos and/or tpos
            // opencv::imgproc::circle( &mut opencv::core::ToInputOutputArray::input_output_array(&mut img).unwrap(), core::Point_::new(x, y), 40, core::Scalar_([4.0, 3.0, 4.0, 5.0]) , -1, 8, 0);
    
            
            // opencv::highgui::imshow("image", &img);
            
            // context.system.trigger_shutdown();
        }

        Ok(Status::done(self))
    }
}

unsafe impl Send for Vis {}
unsafe impl Sync for Vis {}

