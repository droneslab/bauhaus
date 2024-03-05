use derivative::Derivative;
use core::{config::{SETTINGS, SYSTEM}, system::Timestamp};
use crate::map::{map::Id, frame::Frame};

#[derive(Debug, Clone, Derivative)]
#[derivative(Default)]
pub struct Relocalization {
    pub last_reloc_frame_id: Id,
    pub timestamp_lost: Option<Timestamp>,
}

impl Relocalization {
    pub fn sec_since_lost(&self, current_frame: &Frame) -> Timestamp {
        current_frame.timestamp - self.timestamp_lost.unwrap()
    }

    pub fn frames_since_lost(&self, current_frame: &Frame) -> i32 {
        current_frame.frame_id - self.last_reloc_frame_id
    }

    pub fn past_cutoff(&self, current_frame: &Frame) -> bool {
        self.sec_since_lost(current_frame) > SETTINGS.get::<i32>(SYSTEM, "recently_lost_cutoff") as f64
    }

    pub fn _run(&self) -> bool {
        todo!("Relocalization");
    }
}