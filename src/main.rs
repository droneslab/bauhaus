use std::env;
use glob::glob;

// Axiom stuff
use axiom::prelude::*;
use serde::{Deserialize, Serialize};

// #[allow(unused_imports)]
use opencv::{
    // core,
    // features2d,
    features2d::{Feature2DTrait, ORB},
    // highgui,
    // imgproc,
    prelude::*,
    // videoio,
    imgcodecs,
    types::{PtrOfORB, VectorOfKeyPoint},
};

mod base;
// mod orb;

/// The message type that `Game` instances send to the `GameManager` when they are finished
/// with their work.
#[derive(Debug, Serialize, Deserialize)]
struct OrbMsg {
    /// The ID of the actor that sent this message. This is used by the `GameManager` to
    /// associate game results with the actor that created them.
    aid: Aid,
    img_paths: Vec<String>,
}

impl OrbMsg {
    fn new(aid: Aid, vec: Vec<String>) -> Self {
        Self {
            aid,
            img_paths: vec,
        }
    }
}

/// This is the handler that will be used by the actor.
async fn orb_extract(_: (), context: Context, message: Message) -> ActorResult<()> {
    if let Some(msg) = message.content_as::<OrbMsg>() {
        println!("{:?}", context);
        for path in &msg.img_paths {
            let img = imgcodecs::imread(path, imgcodecs::IMREAD_COLOR).unwrap();
            let mut orb: PtrOfORB = ORB::default().unwrap();
            let mut kp = VectorOfKeyPoint::new();
            let mut des = Mat::default().unwrap();

            orb.detect_and_compute(&img,&Mat::default().unwrap(), &mut kp, &mut des, false).unwrap();
            println!("Processed {}, found {} keypoints", path, kp.len());
        }
        context.system.trigger_shutdown();
    }
    Ok(Status::done(()))
}

fn main() {
    let args: Vec<String> = env::args().collect();
    let img_dir = args[1].to_owned();
    let mut glob_str = img_dir.to_owned();
    glob_str.push_str("/*.png");

    let mut img_paths = Vec::new();

    for entry in glob(&glob_str).expect("Failed to read glob pattern") {
        match entry {
            Ok(path) => match path.to_str() {
                Some(path_str) => img_paths.push(path_str.to_owned()),
                None => println!("Invalid path found!"),
            },
            Err(e) => println!("{:?}", e),
        }
    }

    // First we initialize the actor system using the default config.
    let config = ActorSystemConfig::default();
    let system = ActorSystem::create(config);

    // Spawn the actor and send the message.
    let aid = system.spawn().name("orb_extract").with((), orb_extract).unwrap();
    aid.send_new(OrbMsg::new(aid.clone(), img_paths)).unwrap();

    // The actor will trigger shutdown, we just wait for it.
    system.await_shutdown(None);
}
