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

// Public message struct for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct VisMsg {
    // The ID of the actor that sent this message
    aid: Aid,
    // Pose of image paths to read in/extract, Poses take 2 matrixes, pos and rot <int type, # rows, # col, data storage?>
    img_pose: Pose
}

impl VisMsg {
    pub fn new(aid: Aid, pose: Pose) -> Self {          //dunno what this is used for, maybe for after the math?
        Self {
            aid,
            img_pose: Pose,
        }
    }
}

//temporary "global" identity variables
//identity matrix for Rpos and tpos
let Rpos = DMatrix::<f32>::identity(3, 3);
let tpos = DMatrix::<f32>::identity(1, 3);              //aren't identity matrices squares? with a diagonal of 1's


//blank image for graph using opencv
Mat img(Size(320,240),CV_8UC3);                 // create 320x240 image
Mat roi(img, Rect(0,0,320,240));                // select roi
roi = Scalar(0,0,0);                            // fill entire 320x240 image roi with black color


// This is the handler that will be used by the actor.
// couldn't find the stuff for Message and Context type - gonna assume we take in a single VisMsg at a time for now
pub async fn Vis_extract(_: (), context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<VisMsg>() {
        println!("{:?}", context);
       

        // rotate and translate matrices from pose to track trajectory (R and t from pose)
        // Rpos = RRpos
        Rpos = &msg.img_pose.rot * Rpos;
        // tpos = tpose + tRpos
        tpos = tpos + (&msg.img_pose.pos * Rpos);
        
        // update image with a small red square (there is no 'circle struct in opencv::core)
        let x = ???
        let y = ???
        Mat roi(img, Rect(x, y, 20, 20));       // select square roi on the graph by using x, y
        roi = Scalar(255, 0, 0);                // fill square roi with red color
        }
        context.system.trigger_shutdown();
    }
    Ok(Status::done(()))
}

//assumed simple input scenario
pub async fn Vis_extract(input_pose: VisMsg) -> ActorResult<()> {
    
    }
    Ok(Status::done(()))
}
