use derivative::Derivative;
use core::{config::{SETTINGS, SYSTEM}, system::Timestamp};
use crate::{map::{frame::Frame, map::{Id, Map}}, modules::orbslam_matcher};

use super::module_definitions::RelocalizationModule;

#[derive(Debug, Clone, Derivative)]
#[derivative(Default)]
pub struct Relocalization {
    pub last_reloc_frame_id: Id,
    pub timestamp_lost: Option<Timestamp>,
}
impl RelocalizationModule for Relocalization {
    type Frame = Frame;
    type Timestamp = Timestamp;
    type Map = Map;

    fn run(&self, current_frame: &mut Frame, map: &Map) -> Result<bool, Box<dyn std::error::Error>> {
        // // Relocalization is performed when tracking is lost
        // // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        // let candidate_kfs = map.detect_relocalization_candidates(current_frame);

        // if candidate_kfs.is_empty() {
        //     return Ok(false);
        // }

        // // We perform first an ORB matching with each candidate
        // // If enough matches are found we setup a PnP solver

        // for candidate_kf_id in candidate_kfs {
        //     let candidate_kf = map.keyframes.get(&candidate_kf_id).unwrap();
        //     let matches = orbmatcher::search_by_bow_f(candidate_kf, current_frame, true, 0.75)?;
        //     if matches < 15 {
        //         continue;
        //     } else {

        //     }

        // }


        todo!("Relocalization");
    }
    fn sec_since_lost(&self, current_frame: &Frame) -> Timestamp {
        current_frame.timestamp - self.timestamp_lost.unwrap()
    }

    fn frames_since_lost(&self, current_frame: &Frame) -> i32 {
        current_frame.frame_id - self.last_reloc_frame_id
    }
}

impl Relocalization {
    pub fn past_cutoff(&self, current_frame: &Frame) -> bool {
        self.sec_since_lost(current_frame) > SETTINGS.get::<i32>(SYSTEM, "recently_lost_cutoff") as f64
    }

}