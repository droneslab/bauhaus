use chrono::{DateTime, Utc, Duration};
use derivative::Derivative;
use dvcore::global_params::{GLOBAL_PARAMS, SYSTEM_SETTINGS};
use crate::dvmap::{map::Id, frame::Frame, sensor::SensorType};

#[derive(Debug, Clone, Derivative)]
#[derivative(Default)]
pub struct Relocalization {
    // Defaults set from global variables
    // Never change, so just get once instead of having to look up each time
    #[derivative(Default(value = "Duration::seconds(1)"))]
    recently_lost_cutoff: Duration,

    pub last_reloc_frame_id: Id, // last_reloc_frame_id
    pub timestamp_lost: Option<DateTime<Utc>>,
}

impl Relocalization {
    pub fn sec_since_lost<S: SensorType>(&self, current_frame: &Frame<S>) -> Duration {
        current_frame.timestamp - self.timestamp_lost.unwrap()
    }

    pub fn frames_since_lost<S: SensorType>(&self, current_frame: &Frame<S>) -> i32 {
        current_frame.id - self.last_reloc_frame_id
    }

    pub fn past_cutoff<S: SensorType>(&self, current_frame: &Frame<S>) -> bool {
        self.sec_since_lost(current_frame) > Duration::seconds(GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "recently_lost_cutoff").into())
    }

    pub fn run(&self) -> bool {
        todo!("Relocalization");
    }
}