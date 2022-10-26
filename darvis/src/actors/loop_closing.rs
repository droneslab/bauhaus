use std::sync::Arc;
use axiom::prelude::*;

use dvcore::global_params::{Sensor, GLOBAL_PARAMS};
use dvcore::{
    plugin_functions::Function,
    lockwrap::ReadOnlyWrapper,
};
use crate::dvmap::{map::Map, map_actor::MAP_ACTOR, sensor::SensorType};
use crate::actors::messages::KeyFrameMsg;
use crate::modules::imu::ImuModule;

use super::messages::Reset;

#[derive(Debug, Clone, Default)]
pub struct DarvisLoopClosing<S: SensorType> {
    // Map
    map: ReadOnlyWrapper<Map<S>>,

    // IMU
    imu: ImuModule<S>,

    matches_inliers: i32,
}


impl<S: SensorType + 'static> DarvisLoopClosing<S> {
    pub fn new(map: ReadOnlyWrapper<Map<S>>) -> DarvisLoopClosing<S> {
        DarvisLoopClosing {
            map: map,
            ..Default::default()
        }
    }
}

impl<S: SensorType + 'static> Function for DarvisLoopClosing<S> {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {

        Ok(Status::done(()))
    }
}