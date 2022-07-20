use axiom::prelude::*;
// Accepts a message of two image extracted data (kps, des), computes homography
use opencv::{
    imgcodecs,
};
use darvis::{
    global_params::*,
    dvutils::*,
    plugin_functions::Function
};
use crate::{
    modules::vis::*,
    registered_modules::{VISUALIZER, FEATURE_EXTRACTOR},
    modules::messages::{
        frame_msg::FrameMsg,
        images_msg::ImagesMsg,
    }
};

#[derive(Debug, Clone)]
pub struct DarvisFrameLoader {
    fps: i32
}

impl DarvisFrameLoader {
    pub fn new() -> DarvisFrameLoader {
        DarvisFrameLoader {
            fps: 20
        }
    }

    // This is the handler that will be used by the actor.
    pub fn load_frames(&mut self, _context: Context, message: Message) -> ActorResult<()> {
        if let Some(msg) = message.content_as::<ImagesMsg>() {
            let use_visualizer = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "show_ui");
            for path in msg.get_img_paths() {
                let img = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE)?;
                let vis_id = msg.get_actor_ids().get(VISUALIZER).unwrap();
                let feat_aid = msg.actor_ids.get(FEATURE_EXTRACTOR).unwrap();
                println!("Processed image: {}", path);
                if use_visualizer {
                    vis_id.send_new(VisPathMsg::new(path.to_string())).unwrap();
                }

                // TODO: Check ORBSLAM3 frame loader, image gets scaled and resized. Do we need this?
                // https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Examples/RGB-D/rgbd_tum.cc#L89

                // Kickoff the pipeline by sending the feature extraction module images
                feat_aid.send_new(FrameMsg::new(img.grayscale_mat(), msg.actor_ids.clone())).unwrap();
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
