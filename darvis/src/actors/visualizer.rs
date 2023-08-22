use axiom::{message::ActorMessage, prelude::*};
use dvcore::{plugin_functions::Function,matrix::*,};
use r2r::builtin_interfaces::msg::Duration;
use r2r::std_msgs::msg::Int32;
use r2r::trajectory_msgs::msg::*;
use r2r::QosProfile;

use crate::{
    actors::messages::VisMsg, 
    dvmap::{map::Map, pose::{Translation, Rotation}},
    lockwrap::ReadOnlyWrapper
};

use super::messages::VisPathMsg;

#[derive(Debug, Clone)]
pub struct DarvisVisualizer {
    map: ReadOnlyWrapper<Map>,
    // node: r2r::Node,
    // publisher: r2r::Publisher<r2r::std_msgs::msg::String>,
    // ctx: r2r::Context
}

impl DarvisVisualizer {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisVisualizer {
        return DarvisVisualizer{map};

        // let ctx = r2r::Context::create().unwrap();
        // let mut node = r2r::Node::create(r2r::Context::create().unwrap(), "testnode", "").unwrap();
        // // let publisher = node.create_publisher::<JointTrajectoryPoint>("/jtp", QosProfile::default()).unwrap();
        // let publisher2 = node.create_publisher::<r2r::std_msgs::msg::String>("/test", QosProfile::default()).unwrap();


        // let visualizer = DarvisVisualizer {
        //     map,
        //     node,
        //     publisher: publisher2,
        //     ctx
        // };

        // let msg = r2r::std_msgs::msg::String { data: format!("Hello, world!") };
        // publisher2.publish(&msg).unwrap();

        // // // run for 10 seconds
        // // let mut count = 0;
        // // let mut positions: Vec<f64> = Vec::new();
        // // while count < 100 {
        // //     positions.push(count as f64);
        // //     let to_send = JointTrajectoryPoint {
        // //         positions: positions.clone(),
        // //         time_from_start: Duration {
        // //             sec: count,
        // //             nanosec: 0,
        // //         },
        // //         ..Default::default()
        // //     };
        // //     let mut native = r2r::NativeMsg::<Int32>::new();
        // //     native.data = count;

        // //     publisher.publish(&to_send).unwrap();

        // //     std::thread::sleep(std::time::Duration::from_millis(100));
        // //     count += 1;
        // // }

        // visualizer
    }

    pub fn visualize(&mut self, _context: Context, message: Message) -> ActorResult<()> {

        // if let Some(msg) = message.content_as::<VisMsg>()  {}

        Ok(Status::done(()))
    }
}

impl Function for DarvisVisualizer {
    fn handle(&mut self, _context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        self.visualize(_context, message).unwrap();
        Ok(Status::done(()))
    }

}


pub struct VisualizeMsg {
    // pub new_pose: Pose,
}
impl ActorMessage for VisualizeMsg {}

