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

// Axiom stuff                                  //dunno what any of this is
use axiom::prelude::*;
use serde::{Deserialize, Serialize};

use opencv::core::CV_32FC1;
extern crate nalgebra as na;
use na::*;
use crate::base::Pose;

#[derive(Serialize, Deserialize)]
#[serde(remote = "Pose")]
struct PoseDef {
    // 3-Vector
    pub pos: na::Matrix<f64, na::U3, na::U1, na::base::storage::Owned<f64, na::U3, na::U1>>,
    // 3x3 Matrix
    pub rot: na::Matrix<f64, na::U3, na::U3, na::base::storage::Owned<f64, na::U3, na::U3>>,
}


// Public message struct for the actor
//#[derive(Debug, Serialize, Deserialize)]
#[derive(Serialize, Deserialize)]
pub struct VisMsg {
    // The ID of the actor that sent this message
    aid: Aid,
    // Pose of image paths to read in/extract, Poses take 2 matrixes, pos and rot <int type, # rows, # col, data storage?>
    #[serde(with = "PoseDef")]
    img_pose: Pose
}

impl VisMsg {
    pub fn new(aid: axiom::actors::Aid, pose: Pose) -> Self {
        Self {
            aid,
            img_pose: pose,
        }
    }
}

//temporary "global" identity variables
//identity matrix for Rpos and tpos
static mut Rpos: DMatrix::<f64> = DMatrix::<f64>::identity(3, 3);
static mut tpos: DMatrix::<f64> = DMatrix::<f64>::identity(1, 3);              //aren't identity matrices squares? with a diagonal of 1's


//blank image for graph using opencv
static mut img: Mat = Mat::new_rows_cols_with_default(400, 400, CV_32FC1, opencv::core::Scalar::all(0.0)).unwrap();                // create 400x400 image


// This is the handler that will be used by the actor.
// couldn't find the stuff for Message and Context type - gonna assume we take in a single VisMsg at a time for now
pub async fn Vis_extract(_: (), context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<VisMsg>() {
        println!("{:?}", context);
       

        // rotate and translate matrices from pose to track trajectory (R and t from pose)
        // Rpos = RRpos
        let new_Rpos = &msg.img_pose.rot * &Rpos;
        // tpos = tpose + tRpos
        let new_tpos = &tpos + (&msg.img_pose.pos * &Rpos);
        
        // update image with a small red square (there is no 'circle struct in opencv::core)
        
        //let x = ???
        //let y = ???

        // use (200, 200) for x, y coordinates until we can extract them from Rpos and tpos
        opencv::imgproc::circle( &mut opencv::core::ToInputOutputArray::input_output_array(&mut img).unwrap(), core::Point_::new(200, 200), 40, core::Scalar_([4.0, 3.0, 4.0, 5.0]) , -1, 8, 0);

        
        opencv::highgui::imshow("image", & img);
        
        context.system.trigger_shutdown();
    }
    Ok(Status::done(()))
}

