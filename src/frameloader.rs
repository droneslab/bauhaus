// Accepts a message of two image extracted data (kps, des), computes homography
use opencv::{
    imgcodecs,
};


use axiom::prelude::*;
use serde::{Deserialize, Serialize};
use crate::dvutils::*;
use crate::base::*;
use crate::vis::*;
use crate::actornames::*;

use std::time::{Duration, Instant};



// Message type for the actor
#[derive(Debug, Serialize, Deserialize)]
pub struct ImagesMsg {
    // Vector of image paths to read in/extract
    img_paths: Vec<String>,
    actor_ids: std::collections::HashMap<String, axiom::actors::Aid>,
}

impl ImagesMsg {
    pub fn new(vec: Vec<String>, ids: std::collections::HashMap<String, axiom::actors::Aid>) -> Self {
        Self {
             img_paths: vec,
             actor_ids: ids,
        }
    }

    pub fn get_img_paths(&self ) -> &Vec<String>
    {
        &self.img_paths
    }
    pub fn get_actor_ids(&self ) -> &std::collections::HashMap<String, axiom::actors::Aid>
    {
        &self.actor_ids
    }
}




#[derive(Debug, Clone)]
pub struct DarvisFrameLoader
{
    fps: f32
}

impl DarvisFrameLoader
{
    pub fn new() -> DarvisFrameLoader {
        DarvisFrameLoader {
            fps: 20.0
        }
    }
    // This is the handler that will be used by the actor.
// This is the handler that will be used by the actor.
pub fn load_frames(&mut self, _context: Context, message: Message) -> ActorResult<()> {

    let mut prev = Instant::now();
    if let Some(msg) = message.content_as::<ImagesMsg>() {
        for path in msg.get_img_paths() {

            let mut processed = false;

            let img = imgcodecs::imread(&path, imgcodecs::IMREAD_GRAYSCALE)?;

            let vis_id = msg.get_actor_ids().get(VISUALIZER).unwrap();

            let feat_aid = msg.actor_ids.get(FEATURE_EXTRACTOR).unwrap();
            println!("Processed image: {}", path);
            vis_id.send_new(VisPathMsg::new(path.to_string())).unwrap();
            // Kickoff the pipeline by sending the feature extraction module images
            feat_aid.send_new(FrameMsg::new(img.grayscale_mat(), msg.actor_ids.clone())).unwrap();


            while !processed
            {
                let time_elapsed = prev.elapsed();            
                if time_elapsed.as_secs_f32()> 1.0/self.fps
                {
                    processed = true;    
                    prev = Instant::now();          
    
                }                
            }            


        }
    }
        Ok(Status::done(()))
    }

    
}




use crate::pluginfunction::Function;

impl Function for DarvisFrameLoader {

    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.load_frames(_context, message).unwrap();
        Ok(Status::done(()))
    }

}