use std::sync::Arc;
use axiom::prelude::*;

use dvcore::global_params::{Sensor, GLOBAL_PARAMS};
use dvcore::{
    plugin_functions::Function,
    lockwrap::ReadOnlyWrapper,
};
use crate::dvmap::{map::Map, map_actor::MAP_ACTOR};
use crate::actors::messages::KeyFrameMsg;
use crate::modules::imu::ImuModule;

use super::messages::Reset;

#[derive(Debug, Clone, Default)]
pub struct DarvisLoopClosing {
    // Map
    map: ReadOnlyWrapper<Map>,

    // IMU
    imu: ImuModule,

    matches_inliers: i32,
}


impl DarvisLoopClosing {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisLoopClosing {
        DarvisLoopClosing {
            map: map,
            ..Default::default()
        }
    }
}

impl Function for DarvisLoopClosing {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {

        Ok(Status::done(()))
    }
}