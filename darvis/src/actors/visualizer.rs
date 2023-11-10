use std::{borrow::Cow, fs::File, sync::Arc, collections::{BTreeMap, HashMap, HashSet}, fs, io::BufWriter};
use log::{warn, debug};
use mcap::{Schema, Channel, records::MessageHeader, Writer};
use opencv::prelude::{Mat, MatTraitConst, MatTraitConstManual};
use foxglove::{foxglove::items::{SceneEntity, SceneUpdate, SpherePrimitive, Vector3, Quaternion, Color, FrameTransform, LinePrimitive, line_primitive, Point3, RawImage, ArrowPrimitive}, get_file_descriptor_set_bytes, make_pose};

use dvcore::{
    base::{ActorChannels, Actor}, matrix::DVVectorOfKeyPoint, config::GLOBAL_PARAMS,
};
use prost_types::Timestamp;
use crate::{
    dvmap::{map::{Map, Id}, pose::DVPose},
    lockwrap::ReadOnlyWrapper,
    modules::image,
    actors::messages::{ShutdownMsg, TrajectoryMsg, VisFeaturesMsg, VisInitialMapMsg}, registered_actors::VISUALIZER
};

use super::messages::VisFeatureMatchMsg;

// Default visual stuff //
// Trajectory and frame
pub static FRAME_COLOR: Color = Color { r: 1.0, g: 0.0, b: 0.0, a: 1.0 };
pub static TRAJECTORY_COLOR: Color = Color { r: 1.0, g: 0.0, b: 0.0, a: 1.0 };
// Mappoints and mappoint matches
pub static MAPPOINT_COLOR: Color = Color { r: 0.0, g: 0.0, b: 1.0, a: 1.0 };
pub static MAPPOINT_MATCH_COLOR: Color = Color { r: 0.0, g: 1.0, b: 0.0, a: 1.0 };
pub static MAPPOINT_SIZE: Vector3 = Vector3 { x: 0.01, y: 0.01, z: 0.01 };
//Keyframes
pub static KEYFRAME_COLOR: Color = Color { r: 0.5, g: 0.0, b: 0.0, a: 1.0 };

enum ImageDrawType {
    NONE,
    PLAIN,
    FEATURES,
    FEATURESANDMATCHES
}

pub struct DarvisVisualizer {
    actor_system: ActorChannels,
    map: ReadOnlyWrapper<Map>,
    writer: McapWriter,

    // For drawing images
    image_channel: u16,
    image_draw_type: ImageDrawType,
    current_image: Option<Mat>,
    prev_image: Option<Mat>,
    current_keypoints: Option<DVVectorOfKeyPoint>,
    prev_keypoints: Option<DVVectorOfKeyPoint>,

    // For drawing current frame and trajectory
    trajectory_channel: u16,
    transform_channel: u16,
    current_update_id: u64,
    prev_pose: Point3,

    // For drawing mappoints, mappoint matches, and keyframes
    keyframes_channel: u16,
    mappoints_channel: u16,
}

impl Actor for DarvisVisualizer {
    fn run(&mut self) {
        loop {
            let message = self.actor_system.receive().unwrap();

            if let Some(msg) = message.downcast_ref::<VisFeaturesMsg>() {

                // DRAW IMAGE!
                match self.image_draw_type {
                    ImageDrawType::NONE => Ok(()),
                    ImageDrawType::PLAIN => {
                        // Greyscale images are encoded as mono8
                        // For mono8 (CV_8UC1) each element is u8 -> size is 1
                        self.draw_image(&msg.image, "mono8", 1, msg.timestamp)
                    },
                    ImageDrawType::FEATURES => {
                        let image_with_features = image::write_features(&msg.image, &msg.keypoints).expect("Could not draw features to mat");
                        // This is a color image encoded as bgr8
                        // Each element is u8 (size is 1) * 3 (for 3 channels)
                        self.draw_image(&image_with_features, "bgr8", 3, msg.timestamp)
                    },
                    ImageDrawType::FEATURESANDMATCHES => Ok(()), // Draw when matches received from tracking backend
                }.expect("Visualizer could not draw image!");

                // If drawing with features and matches, need to save current image and keypoints for when matches are received
                //TODO (memory)... remove these clones
                self.prev_keypoints = self.current_keypoints.clone();
                self.prev_image = self.current_image.clone();
                self.current_image = Some(msg.image.clone());
                self.current_keypoints = Some(msg.keypoints.clone());

            } else if let Some(msg) = message.downcast_ref::<VisFeatureMatchMsg>() {

                // DRAW IMAGE WITH FEATURE MATCHES!
                if matches!(self.image_draw_type, ImageDrawType::FEATURESANDMATCHES) && self.prev_image.is_some() {
                    let image_with_matches = image::write_feature_matches(
                        &self.prev_image.as_ref().unwrap(),
                        &self.current_image.as_ref().unwrap(),
                        &self.prev_keypoints.as_ref().unwrap(),
                        &self.current_keypoints.as_ref().unwrap(),
                        &msg.matches
                    ).expect("Could not write feature matches to mat");

                    // Encoding and mat element size same as for ImageDrawType::FEATURES
                    self.draw_image(&image_with_matches, "bgr8", 3, msg.timestamp).expect("Could not draw image with matches to mat");
                };

            } else if let Some(msg) = message.downcast_ref::<TrajectoryMsg>() {

                self.draw_trajectory(msg.pose, msg.timestamp).expect("Visualizer could not draw trajectory!");
                self.draw_mappoints(&msg.mappoint_matches, msg.timestamp).expect("Visualizer could not draw mappoints!");
                self.draw_keyframes(msg.timestamp).expect("Visualizer could not draw map!");
                self.current_update_id += 1;
                self.prev_pose = msg.pose.into();
            } else if let Some(_) = message.downcast_ref::<ShutdownMsg>() {

                // SHUTDOWN
                warn!("Closing mcap file");
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
        let mcap_file_path = GLOBAL_PARAMS.get::<String>(VISUALIZER, "mcap_file_path");
        let mut writer = McapWriter::new(&mcap_file_path).unwrap();

        let image_channel = writer.create_and_add_channel(
            "foxglove.RawImage", "/camera",
        ).expect("Could not make image channel");
        let transform_channel = writer.create_and_add_channel(
            "foxglove.FrameTransform", "/frame_transforms",
        ).expect("Could not make frame transform channel");
        let trajectory_channel = writer.create_and_add_channel(
            "foxglove.SceneUpdate", "/trajectory",
        ).expect("Could not make trajectory channel");
        let mappoints_channel = writer.create_and_add_channel(
            "foxglove.SceneUpdate", "/mappoints",
        ).expect("Could not make mappoints channel");
        let keyframes_channel = writer.create_and_add_channel(
            "foxglove.SceneUpdate", "/keyframes",
        ).expect("Could not make keyframes channel");

        let transform = FrameTransform {
            timestamp: Some(Timestamp { seconds: 0,  nanos: 0 }),
            parent_frame_id: "world".to_string(),
            child_frame_id: "camera".to_string(),
            translation: Some(Vector3{ x: 0.0, y: 0.0, z: 0.0 }),
            rotation: Some(Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }),
        };
        writer.write(transform_channel, transform, 0, 0).unwrap();

        // Trajectory starts at 0,0
        let prev_pose = Point3 { x: 0.0, y: 0.0, z: 0.0 };

        let image_draw_type = match GLOBAL_PARAMS.get::<String>(VISUALIZER, "image_draw_type").as_str() {
            "none" => ImageDrawType::NONE,
            "plain" => ImageDrawType::PLAIN,
            "features" => ImageDrawType::FEATURES,
            "featuresandmatches" => ImageDrawType::FEATURESANDMATCHES,
            _ => {
                warn!("Invalid image_draw_type specified in config. Options are none, plain, features, or featuresandmatches. Selecting plain.");
                ImageDrawType::PLAIN
            }
        };

        return DarvisVisualizer{
            actor_system,
            map,
            image_draw_type,
            writer,
            current_update_id: 0,
            prev_pose,
            image_channel,
            trajectory_channel,
            mappoints_channel,
            keyframes_channel,
            transform_channel,
            current_image: None,
            prev_image: None,
            current_keypoints: None,
            prev_keypoints: None,
        };
    }

    fn draw_trajectory(&mut self, pose: DVPose, timestamp: u64) -> Result<(), Box<dyn std::error::Error>> {
        debug!("Drawing trajectory at timestamp {} with pose {:?}", timestamp, pose);

        // Entity 1 ... current pose
        // This entity is overwritten at every update, so only one camera is in view at all times
        let current_pose = self.create_arrow(&pose, FRAME_COLOR.clone());
        let pose_entity = self.create_scene_entity(
            timestamp, "world",
            format!("cam"),
            vec![current_pose],
            vec![], vec![]
        ); // TODO (vis) make this a camera frame and publish new transform, but will this mess up the pose of previous camera objects?

        // Entity 2 ... trajectory
        // This should never be deleted or replaced
        let points = vec![self.prev_pose.clone(), pose.into()];
        let trajectory_line = self.create_line(points, TRAJECTORY_COLOR.clone());
        let traj_entity = self.create_scene_entity(
            timestamp, "world",
            format!("t{}", self.current_update_id.to_string()),
            vec![],
            vec![trajectory_line], 
            vec![]
        );

        let sceneupdate = SceneUpdate {
            deletions: vec![],
            entities: vec![pose_entity, traj_entity]
        };

        self.writer.write(self.trajectory_channel, sceneupdate, timestamp, 0)?;
        Ok(())
    }

    fn draw_keyframes(&mut self, timestamp: u64) -> Result<(), Box<dyn std::error::Error>> {
        // Draw each keyframe
        // Should be deleted when keyframes are deleted
        let map = self.map.read();
        let keyframes = map.get_all_keyframes();
        let mut entities = Vec::<SceneEntity>::new();
        for (id, keyframe) in keyframes {
            let kf_arrow = self.create_arrow(&keyframe.pose.unwrap(), KEYFRAME_COLOR.clone());
            entities.push(self.create_scene_entity(
                timestamp, "world",
                format!("kf {}", id),
                vec![kf_arrow],
                vec![], vec![]
            ));
        }

        let sceneupdate = SceneUpdate {
            deletions: vec![], // todo add deletions
            entities
        };

        self.writer.write(self.keyframes_channel, sceneupdate, timestamp, 0)?;

        Ok(())
    }

    fn draw_mappoints(&mut self, mappoint_matches: &HashMap<u32, (Id, bool)>, timestamp: u64) -> Result<(), Box<dyn std::error::Error>> {
        // Mappoints in map (different color if they are matches)
        // Should be overwritten when there is new info for a mappoint
        let mut entities = Vec::new();

        let mut mappoint_match_ids = HashSet::<Id>::new();
        for (_, (mappoint_id, _)) in mappoint_matches {
            mappoint_match_ids.insert(*mappoint_id);
        }

        let map = self.map.read();
        let mappoints = map.get_all_mappoints();
        for (mappoint_id, mappoint) in mappoints {
            let pose = DVPose::new_with_default_rot(mappoint.position);

            let mp_sphere = match mappoint_match_ids.contains(mappoint_id) {
                true => self.create_sphere(&pose, MAPPOINT_MATCH_COLOR.clone(), MAPPOINT_SIZE.clone()),
                false => self.create_sphere(&pose, MAPPOINT_COLOR.clone(), MAPPOINT_SIZE.clone()),
            };
            let mp_entity = self.create_scene_entity(
                timestamp, "world",
                format!("mp {}", mappoint_id),
                vec![],
                vec![],
                vec![mp_sphere]
            );
            entities.push(mp_entity);
        }


        let sceneupdate = SceneUpdate {
            deletions: vec![],
            entities
        };

        self.writer.write(self.mappoints_channel, sceneupdate, timestamp, 0)?;

        Ok(())
    }

    fn draw_image(
        &mut self, 
        image: &Mat, encoding: &str, item_byte_size: u32, timestamp: u64
    ) -> Result<(), Box<dyn std::error::Error>> {
        let image_msg = RawImage {
            timestamp: Some(Timestamp { seconds: timestamp as i64, nanos: 0 }),
            frame_id: "world".to_string(),
            width: image.cols() as u32,
            height: image.rows() as u32,
            // To figure out the encoding you need, heading 2 in this link might be useful
            // https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
            encoding: encoding.to_string(),
            // Step is image cols * size of each element in bytes
            // If you know the type of each mat element, you can get the size in bytes with ```size_of::<T>()```
            step: image.cols() as u32 * item_byte_size,
            data: Mat::data_bytes(&image)?.to_vec()
        };

        self.writer.write(self.image_channel, image_msg, timestamp, 0)?;

        Ok(())
    }

    fn create_sphere(&self, pose: &DVPose, color: Color, size: Vector3) -> SpherePrimitive {
        SpherePrimitive {
            pose: Some(pose.into()),
            size: Some(size),
            color: Some(color),
        }
    }

    fn create_arrow(&self, pose: &DVPose, color: Color) -> ArrowPrimitive {
        ArrowPrimitive { 
            pose: Some(pose.into()),
            color: Some(color),
            shaft_length: 0.05,
            shaft_diameter: 0.05,
            head_length: 0.05,
            head_diameter: 0.1
        }
    }

    fn create_line(&self, points: Vec<Point3>, color: Color) -> LinePrimitive {
        LinePrimitive {
            r#type: line_primitive::Type::LineStrip as i32,
            pose: make_pose(0.0, 0.0, 0.0),
            thickness: 1.0,
            scale_invariant: true,
            points,
            color: Some(color),
            colors: vec![],
            indices: vec![],
        }
    }

    fn create_scene_entity(
        &self, timestamp: u64, frame_id: &str, entity_id: String,
        arrows: Vec<ArrowPrimitive>, lines: Vec<LinePrimitive>, spheres: Vec<SpherePrimitive>
    ) -> SceneEntity {
        SceneEntity {
            timestamp: Some(Timestamp { seconds: timestamp as i64, nanos: 0 }),
            frame_id: frame_id.to_string(),
            id: entity_id,
            lifetime: None,
            frame_locked: false,
            metadata: vec![],
            arrows,
            cubes: vec![],
            spheres,
            cylinders: vec![],
            lines,
            triangles: vec![],
            texts: vec![],
            models: vec![],
        }
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
        schema: &str,
        topic: &str,
    ) -> Result<u16, Box<dyn std::error::Error>> {
        let schema = Schema {
            name: String::from(schema),
            encoding: String::from("protobuf"),
            data: Cow::Borrowed(get_file_descriptor_set_bytes()),
        };
        let channel = Channel {
            topic: String::from(topic),
            schema: Some(Arc::new(schema)),
            message_encoding: String::from("protobuf"),
            metadata: BTreeMap::default()
        };
        let channel_id = self.writer.add_channel(&channel)?;

        Ok(channel_id)
    }

    pub fn write<T: prost::Message>(
        &mut self,
        channel_id: u16,
        data: T,
        timestamp: u64,
        sequence: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let time_in_nsec = timestamp as u64 * 1000000000;
        Ok(
            self.writer.write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence,
                    log_time: time_in_nsec,
                    publish_time: time_in_nsec,
                },
                &data.encode_to_vec()
            )?
        )
    }

    pub fn finish(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        Ok(self.writer.finish()?)
    }
}


impl From<&DVPose> for foxglove::foxglove::items::Pose {
    fn from(pose: &DVPose) -> foxglove::foxglove::items::Pose {
        let trans = pose.get_translation();
        let position = Vector3 { 
            x: trans[0], 
            y: trans[1], 
            z: trans[2]
        };

        let quat = pose.get_quaternion();
        let orientation = Quaternion {
            x: quat[0],
            y: quat[1],
            z: quat[2],
            w: quat[3]
        };

        foxglove::foxglove::items::Pose {
            position: Some(position),
            orientation: Some(orientation),
        }
    }
}

impl From<DVPose> for foxglove::foxglove::items::Point3 {
    fn from(pose: DVPose) -> foxglove::foxglove::items::Point3 {
        let trans = pose.get_translation();
        Point3 { 
            x: trans[0], 
            y: trans[1], 
            z: trans[2]
        }
    }
}
