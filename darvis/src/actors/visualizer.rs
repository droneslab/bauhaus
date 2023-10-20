use std::{borrow::Cow, fs::File, sync::Arc, collections::BTreeMap, fs, io::BufWriter};
use serde_json::json;
use log::warn;
use mcap::{Schema, Channel, records::MessageHeader, Writer};
use opencv::prelude::{Mat, MatTraitConst, MatTraitConstManual};

use dvcore::{
    vis_schemas::{POSE_SCHEMA, IMAGE_SCHEMA, VisSchema},
    base::{ActorChannels, Actor},
};
use crate::{
    dvmap::map::Map,
    lockwrap::ReadOnlyWrapper,
    modules::image,
    actors::messages::{ShutdownMessage, TrajectoryMessage, VisKeyFrameMsg, VisMapPointsMsg, VisFeaturesMsg}
};

pub struct DarvisVisualizer {
    actor_system: ActorChannels,
    map: ReadOnlyWrapper<Map>,
    writer: McapWriter,
    
    current_frame_id: u64,

    image_channel: u16,
    pose_channel: u16,
}

impl Actor for DarvisVisualizer {
    fn run(&mut self) {
        loop {
            let message = self.actor_system.receive().unwrap();

            if let Some(msg) = message.downcast_ref::<VisFeaturesMsg>() {
                // self.draw_image(msg).expect("Visualizer could not draw image!");
                // self.draw_features(msg).expect("Visualizer could not draw features!");
                self.write_image_to_file(msg).expect("Could not write image to file");
                self.current_frame_id += 1;
            } else if let Some(msg) = message.downcast_ref::<VisKeyFrameMsg>() {
                // self.draw_new_keyframe(msg).expect("Visualizer could not draw keyframe!");
            } else if let Some(msg) = message.downcast_ref::<VisMapPointsMsg>() {
                // self.draw_mappoints(msg).expect("Visualizer could not draw mappoints!");
            } else if let Some(msg) = message.downcast_ref::<TrajectoryMessage>() {
                self.draw_pose(msg).expect("Visualizer could not draw pose!");
            } else if let Some(_) = message.downcast_ref::<ShutdownMessage>() {
                println!("Closing mcap file");
                self.writer.finish().expect("Could not close file");
                break;
            } else {
                warn!("Visualizer received unknown message type!");
            }

        }
    }
}

impl DarvisVisualizer {
    pub fn new(actor_system: ActorChannels, map: ReadOnlyWrapper<Map>) -> DarvisVisualizer {
        // Set up file writers and channels
        let mut writer = McapWriter::new("results/out.mcap").unwrap();

        let image_channel = writer.create_and_add_channel(
            &dvcore::vis_schemas::IMAGE_SCHEMA, "/camera/image_raw",
        ).expect("Could not make image channel");

        let pose_channel = writer.create_and_add_channel(
            &dvcore::vis_schemas::SPHERE_SCHEMA, "/pose",
        ).expect("Could not make pose channel");

        return DarvisVisualizer{
            actor_system,
            map,
            writer,
            current_frame_id: 0,
            image_channel,
            pose_channel,
        };
    }

    fn write_image_to_file(&mut self, msg: &VisFeaturesMsg) -> Result<(), Box<dyn std::error::Error>> {
        let path = format!("results/images/{}-kp.png", msg.image_filename);
        image::write_features(&msg.image, &msg.keypoints, &path);
        Ok(())
    }

    fn write_image_to_mcap(&mut self, image: &Mat, timestamp: u64) -> Result<(), Box<dyn std::error::Error>> {
        // Doesn't work...shows up as all black in foxglove for some reason
        let data = json!({
            "timestamp": {
                "sec": 0,
                "nsec": 1
            },
            "frame_id": self.current_frame_id,
            "width": image.cols(),
            "height": image.rows(),
            "encoding": "mono8",
            // Step is image cols * size of each element in bytes
            // For mono8 (CV_8UC1) each element is u8 -> size is 1
            // You can check this with ```size_of::<u8>()```
            "step": image.cols(),
            "data": Mat::data_bytes(&image)?
        });

        println!("Mat!!...{:?}",
            image
        );

        println!("Mat!!...{:?}",
            &image.data_typed::<u8>()?
        );

        println!("total bytes...{}, cols...{}, rows...{}",
            Mat::data_bytes(&image)?.len(),
            image.cols(),
            image.rows()
        );

        self.writer.write(self.image_channel, &data, timestamp, 0)?;

        Ok(())
    }


    fn draw_pose(&mut self, message: &TrajectoryMessage) -> Result<(), Box<dyn std::error::Error>> {
        let translation = message.pose.get_translation();

        let msg = json!({
            "position": {
                "x": translation.x,
                "y": translation.y,
                "z": translation.z
            },
            "orientation": {
                "x": 0,
                "y": 0,
                "z": 0,
                "w": 1.0
            },
        });

        self.writer.write(self.pose_channel, &msg, message.timestamp, 0)?;
        Ok(())
    }


    fn draw_features(&mut self, message: &VisFeaturesMsg) -> Result<(), Box<dyn std::error::Error>> {
        todo!()
    }


    fn draw_new_keyframe(&mut self, message: &VisKeyFrameMsg) -> Result<(), Box<dyn std::error::Error>> {
        todo!()
    }

    fn draw_mappoints(&mut self, message: &VisMapPointsMsg) -> Result<(), Box<dyn std::error::Error>> {

        todo!()
    }
}

pub struct McapWriter {
    writer: Writer<'static, BufWriter<File>>,

}

impl McapWriter {
    pub fn new(path: &str) -> Result<McapWriter, Box<dyn std::error::Error>> {
        Ok(McapWriter{
            writer: Writer::new(BufWriter::new(fs::File::create(path)?))?
        })
    }

    pub fn create_and_add_channel(
        &mut self,
        schema: &VisSchema,
        topic: &str,
    ) -> Result<u16, Box<dyn std::error::Error>> {
        let schema = Schema {
            name: String::from(schema.name),
            encoding: String::from("jsonschema"),
            data: Cow::Borrowed(schema.json.as_bytes()),
        };
        let channel = Channel {
            topic: String::from(topic),
            schema: Some(Arc::new(schema)),
            message_encoding: String::from("json"),
            metadata: BTreeMap::default()
        };
        let channel_id = self.writer.add_channel(&channel)?;

        Ok(channel_id)
    }

    pub fn write(
        &mut self,
        channel_id: u16,
        data: &serde_json::Value,
        timestamp: u64,
        sequence: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        warn!("timestamp {}", timestamp);
        Ok(
            self.writer.write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence,
                    log_time: timestamp,
                    publish_time: timestamp,
                },
                &serde_json::to_vec(&data)? // https://github.com/twistedfall/opencv-rust/issues/267
                // test_msg.to_string().as_bytes()
            )?
        )
    }

    pub fn finish(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        Ok(self.writer.finish()?)
    }
}
