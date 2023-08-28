use std::{sync::Arc, path::Path};

use axiom::{message::ActorMessage, prelude::*};
use dvcore::{plugin_functions::Function,matrix::*,};
use opencv::{imgcodecs, viz::WCloud};
use rerun::{
    components::{ColorRGBA, Point3D, Radius},
    demo_util::grid,
    external::glam,
    MsgSender, RecordingStreamBuilder, RecordingStream, time::{Timeline, TimeType},
};

use crate::{
    actors::messages::{ImageMsg}, 
    dvmap::{map::Map, pose::{Translation, Rotation, Pose}},
    lockwrap::ReadOnlyWrapper
};



#[derive(Debug, Clone)]
pub struct DarvisVisualizer {
    map: ReadOnlyWrapper<Map>,
    rec_stream: RecordingStream
}

impl DarvisVisualizer {
    pub fn new(map: ReadOnlyWrapper<Map>, rec_stream: RecordingStream) -> DarvisVisualizer {

        // Using opencv...
        // let cw = WCloud::new(cloud, viz::Color::red());
        // // Display it in a window
        // myWindow.showWidget("CloudWidget1", cw);
        // // Modify it, and it will be modified in the window.
        // cw.set_color(viz::Color::yellow());

        return DarvisVisualizer{map, rec_stream};
    }

    // fn timepoint(index: usize, time: f64) -> TimePoint {
    //     let timeline_time = Timeline::new("time", TimeType::Time);
    //     let timeline_frame = Timeline::new("frame", TimeType::Sequence);
    //     let time = Time::from_seconds_since_epoch(time);
    //     [
    //         (timeline_time, time.into()),
    //         (timeline_frame, (index as i64).into()),
    //     ]
    //     .into()
    // }


    pub fn draw_image(&mut self, _context: Context, message: Arc<ImageMsg>) -> ActorResult<()> {
        // let tensor = rerun::components::Tensor::from_image_file(Path::new(&message.image_path))?;
        // // map/robot/camera/img
        // MsgSender::new("world/camera")
        //     // .with_timepoint(Timeline::new("time", TimeType::Time).into())
        //     .with_component(&[tensor])?
        //     .send(&self.rec_stream)?;

        Ok(Status::done(()))
    }

    pub fn draw_features(&mut self, _context: Context, message: Arc<VisFeaturesMsg>) -> ActorResult<()> {
        todo!()
        // Ok(Status::done(()))
    }

    // Transforms: https://www.rerun.io/docs/concepts/spaces-and-transforms#space-transformations

    pub fn draw_new_keyframe(&mut self, _context: Context, message: Arc<VisKeyFrameMsg>) -> ActorResult<()> {
        // Box3D : https://www.rerun.io/docs/reference/data_types/box3d
        todo!()
    }

    pub fn draw_mappoints(&mut self, _context: Context, message: Arc<VisMapPointsMsg>) -> ActorResult<()> {
        // map/robot/camera/points

        // let points = grid(glam::Vec3::splat(-10.0), glam::Vec3::splat(10.0), 10)
        //     .map(Point3D::from)
        //     .collect::<Vec<_>>();
        // let colors = grid(glam::Vec3::ZERO, glam::Vec3::splat(255.0), 10)
        //     .map(|v| ColorRGBA::from_rgb(v.x as u8, v.y as u8, v.z as u8))
        //     .collect::<Vec<_>>();

        // MsgSender::new("my_points")
        //     .with_component(&points).unwrap()
        //     .with_component(&colors).unwrap()
        //     .with_splat(Radius(0.5)).unwrap()
        //     .send(&rec_stream).unwrap();

        todo!()
    }

    pub fn draw_odometry() -> ActorResult<()> {
        // odometry/vel
        // python ... rr.log_scalar("odometry/vel", odom.twist.twist.linear.x)
        // object ... LineStrip3D https://docs.rs/rerun/latest/rerun/components/struct.LineStrip3D.html
        todo!()
    }
}

impl Function for DarvisVisualizer {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()> {
        // Background: https://www.rerun.io/docs/getting-started/logging-rust
        if let Some(image_msg) = message.content_as::<ImageMsg>() {
            self.draw_image(_context, image_msg).expect("Visualizer could not draw image!");
        } else if let Some(features_msg) = message.content_as::<VisFeaturesMsg>() {
            self.draw_features(_context, features_msg).expect("Visualizer could not draw features!");
        } else if let Some(kf_msg) = message.content_as::<VisKeyFrameMsg>() {
            self.draw_new_keyframe(_context, kf_msg).expect("Visualizer could not draw keyframe!");
        } else if let Some(mps_msg) = message.content_as::<VisMapPointsMsg>() {
            self.draw_mappoints(_context, mps_msg).expect("Visualizer could not draw mappoints!");
        }

        Ok(Status::done(()))
    }
}

pub struct VisFeaturesMsg {
    pub keypoints: DVVectorOfKeyPoint,
}
impl ActorMessage for VisFeaturesMsg {}

pub struct VisKeyFrameMsg {
    pub pose: Pose,
}
impl ActorMessage for VisKeyFrameMsg {}

pub struct VisMapPointsMsg {}
impl ActorMessage for VisMapPointsMsg {}
