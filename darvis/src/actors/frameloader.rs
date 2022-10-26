use axiom::prelude::*;
use opencv::imgcodecs;
use dvcore::{global_params::*, matrix::*, plugin_functions::Function};
use crate::{
    actors::vis::*,
    registered_modules::{VISUALIZER, TRACKING_FRONTEND},
    actors::messages::{ImageMsg, ImagesMsg,}
};

#[derive(Debug, Clone)]
pub struct DarvisFrameLoader {
    fps: i32
}

impl DarvisFrameLoader {
    pub fn new() -> DarvisFrameLoader {
        DarvisFrameLoader {fps: 20}
    }

    pub fn load_frames(&mut self, context: Context, message: Message) -> ActorResult<()> {
        if let Some(msg) = message.content_as::<ImagesMsg>() {
            let use_visualizer = GLOBAL_PARAMS.get::<bool>(SYSTEM_SETTINGS, "show_ui");
            for path in &msg.img_paths {
                let img = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE)?;
                let vis_id = context.system.find_aid_by_name(VISUALIZER).unwrap();
                let feat_aid = context.system.find_aid_by_name(TRACKING_FRONTEND).unwrap();
                println!("Processed image: {}", path);
                if use_visualizer {
                    vis_id.send_new(VisPathMsg::new(path.to_string())).unwrap();
                }

                // TODO: Check ORBSLAM3 frame loader, image gets scaled and resized. Do we need this?
                // https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/RGB-D/rgbd_tum.cc#L89

                // Kickoff the pipeline by sending the feature extraction module images
                feat_aid.send_new(ImageMsg{
                    frame: img.grayscale_mat(),
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
