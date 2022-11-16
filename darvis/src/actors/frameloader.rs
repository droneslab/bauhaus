use axiom::prelude::*;
use log::info;
use opencv::imgcodecs;
use dvcore::{config::*, matrix::*, plugin_functions::Function};
use crate::{
    actors::vis::*,
    registered_modules::{VISUALIZER, TRACKING_FRONTEND},
    actors::messages::{ImageMsg, ImagesMsg,}
};

#[derive(Debug, Clone)]
pub struct DarvisFrameLoader {
    fps: f64
}

impl DarvisFrameLoader {
    pub fn new() -> DarvisFrameLoader {
        DarvisFrameLoader {fps: GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "fps") }
    }

    pub fn load_frames(&mut self, context: Context, message: Message) -> ActorResult<()> {
        // TODO - run at specific fps
        if let Some(msg) = message.content_as::<ImagesMsg>() {
            let use_visualizer = GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "show_ui");
            for path in &msg.img_paths {
                let img = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE)?;
                let tracking_frontend = context.system.find_aid_by_name(TRACKING_FRONTEND).unwrap();

                info!("Processed image;{}", path);
                if use_visualizer {
                    let vis_id = context.system.find_aid_by_name(VISUALIZER).unwrap();
                    vis_id.send_new(VisPathMsg::new(path.to_string())).unwrap();
                }

                // TODO (need?) Check ORBSLAM3 frame loader, image gets scaled and resized
                // https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/RGB-D/rgbd_tum.cc#L89

                // Kickoff the pipeline by sending the feature extraction module images
                tracking_frontend.send_new(ImageMsg{
                    frame: img.into(),
                }).unwrap();
            }
        }
        Ok(Status::done(()))
    }
}

impl Function for DarvisFrameLoader {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()> {
        self.load_frames(_context, message).unwrap();
        Ok(Status::done(()))
    }
}
