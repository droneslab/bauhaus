use chrono::{DateTime, Utc, Duration};
use derivative::Derivative;
use dvcore::config::{GLOBAL_PARAMS, SYSTEM_SETTINGS};
use crate::dvmap::{map::Id, keyframe::{Frame, InitialFrame}};

#[derive(Debug, Clone, Derivative)]
#[derivative(Default)]
pub struct Relocalization {
    pub last_reloc_frame_id: Id,
    pub timestamp_lost: Option<DateTime<Utc>>,
}

impl Relocalization {
    pub fn sec_since_lost(&self, current_frame: &Frame<InitialFrame>) -> Duration {
        current_frame.timestamp - self.timestamp_lost.unwrap()
    }

    pub fn frames_since_lost(&self, current_frame: &Frame<InitialFrame>) -> i32 {
        current_frame.frame_id - self.last_reloc_frame_id
    }

    pub fn past_cutoff(&self, current_frame: &Frame<InitialFrame>) -> bool {
        self.sec_since_lost(current_frame) > Duration::seconds(GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "recently_lost_cutoff").into())
    }

    pub fn run(&self) -> bool {
        todo!("Relocalization");
    }
}