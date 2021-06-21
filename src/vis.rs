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

// Axiom stuff
use axiom::prelude::*;
use serde::{Deserialize, Serialize};

// Message type for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct VisMsg {
    // The ID of the actor that sent this message
    aid: Aid,
    // Vector of image paths to read in/extract
    img_pose: Pose,
}

impl VisMsg {
    pub fn new(aid: Aid, vec: Vec<String>) -> Self {
        Self {
            aid,
            img_pose: pose,
        }
    }
}

// This is the handler that will be used by the actor.
pub async fn Vis_extract(_: (), context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<VisMsg>() {
        println!("{:?}", context);
        
        }
        context.system.trigger_shutdown();
    }
    Ok(Status::done(()))
}
