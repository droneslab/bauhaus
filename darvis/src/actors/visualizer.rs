use axiom::{message::ActorMessage, prelude::*};
use dvcore::{plugin_functions::Function,matrix::*,};
use rerun::{
    components::{ColorRGBA, Point3D, Radius},
    demo_util::grid,
    external::glam,
    MsgSender, RecordingStreamBuilder, RecordingStream,
};

use crate::{
    actors::messages::VisMsg, 
    dvmap::{map::Map, pose::{Translation, Rotation}},
    lockwrap::ReadOnlyWrapper
};


#[derive(Debug, Clone)]
pub struct DarvisVisualizer {
    map: ReadOnlyWrapper<Map>,
    rec_stream: RecordingStream
}

impl DarvisVisualizer {
    pub fn new(map: ReadOnlyWrapper<Map>, rec_stream: RecordingStream) -> DarvisVisualizer {
        let points = grid(glam::Vec3::splat(-10.0), glam::Vec3::splat(10.0), 10)
            .map(Point3D::from)
            .collect::<Vec<_>>();
        let colors = grid(glam::Vec3::ZERO, glam::Vec3::splat(255.0), 10)
            .map(|v| ColorRGBA::from_rgb(v.x as u8, v.y as u8, v.z as u8))
            .collect::<Vec<_>>();

        MsgSender::new("my_points")
            .with_component(&points).unwrap()
            .with_component(&colors).unwrap()
            .with_splat(Radius(0.5)).unwrap()
            .send(&rec_stream).unwrap();


        return DarvisVisualizer{map, rec_stream};
    }

    pub fn visualize(&mut self, _context: Context, message: Message) -> ActorResult<()> {
        let points = grid(glam::Vec3::splat(-10.0), glam::Vec3::splat(10.0), 10)
            .map(Point3D::from)
            .collect::<Vec<_>>();
        let colors = grid(glam::Vec3::ZERO, glam::Vec3::splat(255.0), 10)
            .map(|v| ColorRGBA::from_rgb(v.x as u8, v.y as u8, v.z as u8))
            .collect::<Vec<_>>();

        MsgSender::new("my_points")
            .with_component(&points).unwrap()
            .with_component(&colors).unwrap()
            .with_splat(Radius(0.5)).unwrap()
            .send(&self.rec_stream).unwrap();

        Ok(Status::done(()))
    }
}

impl Function for DarvisVisualizer {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()> {
        self.visualize(_context, message).unwrap();
        Ok(Status::done(()))
    }

}


pub struct VisualizeMsg {
    // pub new_pose: Pose,
}
impl ActorMessage for VisualizeMsg {}

