use derivative::Derivative;
use dvcore::config::{GLOBAL_PARAMS, SYSTEM_SETTINGS};
use crate::dvmap::{map::Id, keyframe::{Frame, InitialFrame}};

#[derive(Debug, Clone, Derivative)]
#[derivative(Default)]
pub struct Relocalization {
    pub last_reloc_frame_id: Id,
    pub timestamp_lost: Option<u64>,
}

impl Relocalization {
    pub fn sec_since_lost(&self, current_frame: &Frame<InitialFrame>) -> u64 {
        current_frame.timestamp - self.timestamp_lost.unwrap()
    }

    pub fn frames_since_lost(&self, current_frame: &Frame<InitialFrame>) -> i32 {
        current_frame.frame_id - self.last_reloc_frame_id
    }

    pub fn past_cutoff(&self, current_frame: &Frame<InitialFrame>) -> bool {
        self.sec_since_lost(current_frame) > GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "recently_lost_cutoff") as u64
    }

    pub fn _run(&self) -> bool {
        todo!("Relocalization");
    }
}