use std::{borrow::Cow, fs::File, sync::Arc, collections::{BTreeMap, HashSet}, fs, io::BufWriter};
use log::{warn, debug};
use mcap::{Schema, Channel, records::MessageHeader, Writer};
use opencv::prelude::{Mat, MatTraitConst, MatTraitConstManual};
use foxglove::{foxglove::items::{SceneEntity, SceneUpdate, SpherePrimitive, Vector3, Quaternion, Color, FrameTransform, LinePrimitive, line_primitive, Point3, RawImage, ArrowPrimitive, SceneEntityDeletion, scene_entity_deletion}, get_file_descriptor_set_bytes, make_pose};

use core::{
    actor::{ActorChannels, Actor}, matrix::DVVectorOfKeyPoint, config::SETTINGS,
};
use crate::{
    map::{map::Id, pose::Pose, misc::Timestamp},
    modules::image,
    actors::messages::{ShutdownMsg, VisFeaturesMsg, VisFeatureMatchMsg, VisTrajectoryMsg},
    registered_actors::VISUALIZER, MapLock
};

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
    map: MapLock,
    writer: McapWriter,

    // For drawing images
    image_channel: u16,
    image_draw_type: ImageDrawType,
    current_image: Option<Mat>,
    prev_image: Option<Mat>,
    current_keypoints: Option<DVVectorOfKeyPoint>,
    prev_keypoints: Option<DVVectorOfKeyPoint>,
    previous_mappoints: HashSet<Id>,
    previous_keyframes: HashSet<Id>,

    trajectory_channel: u16,
    mappoints_channel: u16,
    connected_kfs_channel: u16,
    current_update_id: u64,
    prev_pose: Point3,
}

impl Actor for DarvisVisualizer {
    type MapRef = MapLock;

    fn new_actorstate(actor_system: ActorChannels, map: Self::MapRef) -> DarvisVisualizer {
        let mcap_file_path = SETTINGS.get::<String>(VISUALIZER, "mcap_file_path");
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
        let connected_kfs_channel = writer.create_and_add_channel(
            "foxglove.SceneUpdate", "/connected_kfs",
        ).expect("Could not make connected kfs channel");

        let transform = FrameTransform {
            timestamp: Some(prost_types::Timestamp { seconds: 0,  nanos: 0 }),
            parent_frame_id: "world".to_string(),
            child_frame_id: "camera".to_string(),
            translation: Some(Vector3{ x: 0.0, y: 0.0, z: 0.0 }),
            rotation: Some(Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }),
        };
        writer.write(transform_channel, transform, 0.0, 0).unwrap();

        // Trajectory starts at 0,0
        let prev_pose = Point3 { x: 0.0, y: 0.0, z: 0.0 };

        let image_draw_type = match SETTINGS.get::<String>(VISUALIZER, "image_draw_type").as_str() {
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
            connected_kfs_channel,
            current_image: None,
            prev_image: None,
            current_keypoints: None,
            prev_keypoints: None,
            previous_mappoints: HashSet::new(),
            previous_keyframes: HashSet::new(),
        };
    }

    fn spawn(actor_channels: ActorChannels, map: Self::MapRef) {
        let mut actor = DarvisVisualizer::new_actorstate(actor_channels, map);
        loop {
            let message = actor.actor_system.receive().unwrap();

            if message.is::<VisFeaturesMsg>() {
                let msg = message.downcast::<VisFeaturesMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
                actor.update_draw_image(*msg);
            } else if message.is::<VisFeatureMatchMsg>() {
                let msg = message.downcast::<VisFeatureMatchMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
                actor.update_draw_image_with_matches(*msg);
            } else if message.is::<VisTrajectoryMsg>() {
                let msg = message.downcast::<VisTrajectoryMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
                actor.update_draw_map(*msg);
            } else if message.is::<ShutdownMsg>() {
                // SHUTDOWN
                warn!("Closing mcap file");
                actor.writer.finish().expect("Could not close file");
                break;
            } else {
                warn!("Visualizer received unknown message type!");
            }
        }
    }
}

impl DarvisVisualizer {
    fn update_draw_image(&mut self, msg: VisFeaturesMsg) {
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
        //TODO (timing)... remove these clones. Can use same pointer swap technique as in beginning of tracking backend
        self.prev_keypoints = self.current_keypoints.clone();
        self.prev_image = self.current_image.clone();
        self.current_image = Some(msg.image.clone());
        self.current_keypoints = Some(msg.keypoints.clone());
    }

    fn update_draw_image_with_matches(&mut self, msg: VisFeatureMatchMsg) {
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
    }

    fn update_draw_map(&mut self, msg: VisTrajectoryMsg) {
        self.draw_trajectory(msg.pose, msg.timestamp).expect("Visualizer could not draw trajectory!");
        self.draw_mappoints(&msg.mappoint_matches, msg.timestamp).expect("Visualizer could not draw mappoints!");
        // Not calling this right now, need to figure out some nicer way to distribute the points so it's actually readable
        // self.draw_connected_kfs(msg.timestamp).expect("Visualizer could not draw connected kfs!");
        self.current_update_id += 1;
        self.prev_pose = msg.pose.into();
    }

    fn draw_trajectory(&mut self, frame_pose: Pose, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // debug!("Drawing trajectory at timestamp {} with pose {:?}", timestamp, frame_pose.inverse());

        // self.clear_scene(timestamp, self.trajectory_channel)?;

        let mut entities = vec![];
        let inverse_frame_pose = frame_pose.inverse();

        // Draw current pose
        // This entity is overwritten at every update, so only one camera is in view at all times
        entities.push(
            self.create_scene_entity(
                timestamp, 
                "world",
                format!("cam"),
                vec![self.create_arrow(&inverse_frame_pose, FRAME_COLOR.clone())],
                vec![], vec![]
            )
        );

        let map = self.map.read();

        // Draw keyframes and trajectory
        // Re-draw each time because kf poses may have neen optimized since last drawing
        let mut sorted_kfs = map.keyframes.keys().collect::<Vec<&Id>>();
        sorted_kfs.sort();
        let mut prev_pose: Point3 = Pose::default().into();
        let mut prev_id = 0;
        for id in sorted_kfs {
            let curr_pose = map.keyframes.get(id).unwrap().pose.inverse();

            if curr_pose == inverse_frame_pose {
                // Skip over drawing a kf if it was created at this frame, 
                // so that it doesn't cover up the drawing of the current frame. 
                continue;
            }

            let points = vec![prev_pose, curr_pose.into()];
            entities.push(
                self.create_scene_entity(
                    timestamp,
                    "world",
                    format!("t {}->{}", prev_id, id),
                    vec![],
                    vec![self.create_line(points, TRAJECTORY_COLOR.clone())], 
                    vec![]
                )
            );

            entities.push(
                self.create_scene_entity(
                    timestamp, 
                    "world",
                    format!("kf {}", id),
                    vec![self.create_arrow(&curr_pose, KEYFRAME_COLOR.clone())],
                    vec![],
                    vec![]
                )
            );
            prev_pose = curr_pose.into();
            prev_id = *id;
        }

        // Delete keyframes that are no longer in the map but were previously drawn
        let curr_keyframes = map.keyframes.keys().map(|id| *id).collect::<HashSet<Id>>();
        let deleted_keyframes = self.previous_keyframes.difference(&curr_keyframes);
        let deletions = deleted_keyframes.map(|id|
            self.create_scene_entity_deletion(timestamp, format!("kf {}", id),
        )).collect();
        self.previous_keyframes = curr_keyframes;

        // Lastly, add the trajectory line from the last keyframe to the current pose
        let points = vec![prev_pose, frame_pose.inverse().into()];
        entities.push(
            self.create_scene_entity(
                timestamp, 
                "world",
                format!("t to frame"),
                vec![],
                vec![self.create_line(points, TRAJECTORY_COLOR.clone())], 
                vec![]
            )
        );

        let sceneupdate = SceneUpdate {
            deletions,
            entities
        };

        self.writer.write(self.trajectory_channel, sceneupdate, timestamp, 0)?;
        Ok(())
    }

    fn draw_mappoints(&mut self, mappoint_matches: &Vec<Option<(Id, bool)>>, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // Mappoints in map (different color if they are matches)
        // Should be overwritten when there is new info for a mappoint
        // self.clear_scene(timestamp, self.mappoints_channel)?;

        let mut entities = Vec::new();

        let mut mappoint_match_ids = HashSet::<Id>::new();
        for item in mappoint_matches {
            if let Some((mappoint_id, _)) = item {
                mappoint_match_ids.insert(*mappoint_id);
            }
        }

        let map = self.map.read();
        let mut curr_mappoints = HashSet::new();
        for (mappoint_id, mappoint) in &map.mappoints {
            let pose = Pose::new_with_default_rot(mappoint.position);

            let mp_sphere = match mappoint_match_ids.contains(&mappoint_id) {
                true => self.create_sphere(&pose, MAPPOINT_MATCH_COLOR.clone(), MAPPOINT_SIZE.clone()),
                false => {
                    if SETTINGS.get::<bool>(VISUALIZER, "draw_all_mappoints") || !self.previous_mappoints.contains(&mappoint_id) {
                        // Mappoint is not a match in current frame. Only draw if we are drawing all mappoints, or if the mappoint was recently created/seen.
                        // TODO (mvp) : I'm not sure the logic behind using self.previous_mappoints is correct. I think more mps get included than should be?
                        self.create_sphere(&pose, MAPPOINT_COLOR.clone(), MAPPOINT_SIZE.clone())
                    } else {
                        continue;
                    }
                },
            };
            let mp_entity = self.create_scene_entity(
                timestamp, "world",
                format!("mp {}", mappoint_id),
                vec![],
                vec![],
                vec![mp_sphere]
            );
            entities.push(mp_entity);
            curr_mappoints.insert(*mappoint_id);
        }

        // Delete mappoints that are no longer in the map but were previously drawn
        let deleted_mappoints = self.previous_mappoints.difference(&curr_mappoints);
        let deletions = deleted_mappoints.map(|id|
            self.create_scene_entity_deletion(timestamp, format!("mp {}", id),
        )).collect();
        self.previous_mappoints = curr_mappoints;

        let sceneupdate = SceneUpdate {
            deletions,
            entities
        };

        self.writer.write(self.mappoints_channel, sceneupdate, timestamp, 0)?;

        Ok(())
    }

    fn draw_connected_kfs(&mut self, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // You can call this but the results don't look too good because I draw each KF at its pose
        // so it's hard to see the connections, they overlap too much. Need to find some algorithm to
        // distribute items in a graph so they're spread apart and the lines between are more visible.
        // Maybe there's a crate for this?

        // Draw connected keyframes graph
        // Will be re-drawn each time because kf may gain/lose connections over time
        // But won't update if a keyframe is deleted! Need to keep track of that and delete manually

        let mut entities = Vec::new();
        let map = self.map.read();
        for kf_id in map.keyframes.keys() {
            let kf = map.keyframes.get(&kf_id).unwrap();
            let pose = kf.pose.inverse();

            let mut spheres = Vec::new();
            let mut lines = Vec::new();

            for connected_kf_id in &kf.get_covisibility_keyframes(i32::MAX) {
                let connected_kf = map.keyframes.get(connected_kf_id).unwrap();
                let connected_pose = connected_kf.pose.inverse();
                let points = vec![pose.into(), connected_pose.into()];
                lines.push(self.create_line(points, TRAJECTORY_COLOR.clone()));
                spheres.push(self.create_sphere(&connected_pose, KEYFRAME_COLOR.clone(), MAPPOINT_SIZE.clone()));
            }

            let kf_entity = self.create_scene_entity(
                timestamp, "world",
                format!("kf {}", kf_id),
                vec![], lines, spheres
            );
            entities.push(kf_entity);
        }

        // Delete keyframes that are no longer in the map but were previously drawn
        let curr_keyframes = map.keyframes.keys().map(|id| *id).collect::<HashSet<Id>>();
        let deleted_keyframes = self.previous_keyframes.difference(&curr_keyframes);
        let deletions = deleted_keyframes.map(|id|
            self.create_scene_entity_deletion(timestamp, format!("kf {}", id),
        )).collect();
        self.previous_keyframes = curr_keyframes;

        let sceneupdate = SceneUpdate {
            deletions,
            entities
        };

        self.writer.write(self.connected_kfs_channel, sceneupdate, timestamp, 0)?;

        Ok(())
    }

    fn draw_image(
        &mut self, 
        image: &Mat, encoding: &str, item_byte_size: u32, timestamp: Timestamp
    ) -> Result<(), Box<dyn std::error::Error>> {
        let (seconds, nanos) = convert_timestamp(timestamp);

        let image_msg = RawImage {
            timestamp: Some(prost_types::Timestamp { seconds, nanos }),
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

    fn create_sphere(&self, pose: &Pose, color: Color, size: Vector3) -> SpherePrimitive {
        SpherePrimitive {
            pose: Some(pose.into()),
            size: Some(size),
            color: Some(color),
        }
    }

    fn create_arrow(&self, pose: &Pose, color: Color) -> ArrowPrimitive {
        ArrowPrimitive { 
            pose: Some(pose.into()),
            color: Some(color),
            shaft_length: 0.025,
            shaft_diameter: 0.025,
            head_length: 0.05,
            head_diameter: 0.035
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

    fn create_scene_entity_deletion(&self, timestamp: Timestamp, entity_id: String) -> SceneEntityDeletion {
        let (seconds, nanos) = convert_timestamp(timestamp);
        SceneEntityDeletion {
            timestamp: Some(prost_types::Timestamp { seconds, nanos }),
            r#type: scene_entity_deletion::Type::MatchingId as i32,
            id: entity_id,
        }
    }

    fn clear_scene(&mut self, timestamp: Timestamp, channel: u16) -> Result<(), Box<dyn std::error::Error>> {
        // Deletes all entities in the channel
        let (seconds, nanos) = convert_timestamp(timestamp);
        let sceneupdate = SceneUpdate {
            deletions: vec![
                SceneEntityDeletion {
                    timestamp: Some(prost_types::Timestamp { seconds, nanos }),
                    r#type: scene_entity_deletion::Type::All as i32,
                    id: "".to_string(),
                }
            ],
            entities: vec![]
        };
        self.writer.write(channel, sceneupdate, timestamp, 0)
    }

    fn create_scene_entity(
        &self, timestamp: Timestamp, frame_id: &str, entity_id: String,
        arrows: Vec<ArrowPrimitive>, lines: Vec<LinePrimitive>, spheres: Vec<SpherePrimitive>
    ) -> SceneEntity {
        let (seconds, nanos) = convert_timestamp(timestamp);
        SceneEntity {
            timestamp: Some(prost_types::Timestamp { seconds, nanos }),
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

fn convert_timestamp(timestamp: Timestamp) -> (i64, i32) {
    let timestamp = timestamp * 10.0; // Note: Can remove this later, putting this in here for now so foxglove can show the results a little slower. If removing, also modify write() function
    let seconds = timestamp.floor();
    let nanos = (timestamp - seconds) * 1000000000.0;
    (seconds as i64, nanos as i32)
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
        timestamp: Timestamp,
        sequence: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let time_in_nsec = timestamp * 10000000000.0; // Note: remove one 0 if removing line in convert_timestamp that makes it go slower
        Ok(
            self.writer.write_to_known_channel(
                &MessageHeader {
                    channel_id,
                    sequence,
                    log_time: time_in_nsec as u64,
                    publish_time: time_in_nsec as u64,
                },
                &data.encode_to_vec()
            )?
        )
    }

    pub fn finish(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        Ok(self.writer.finish()?)
    }
}


impl From<&Pose> for foxglove::foxglove::items::Pose {
    fn from(pose: &Pose) -> foxglove::foxglove::items::Pose {
        let trans = pose.get_translation();
        let position = Vector3 { 
            x: trans[0], 
            y: trans[1], 
            z: trans[2]
        };

        // TODO (mvp): These always show pointing to the right when they shouldn't be
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

impl From<Pose> for foxglove::foxglove::items::Point3 {
    fn from(pose: Pose) -> foxglove::foxglove::items::Point3 {
        let trans = pose.get_translation();
        Point3 { 
            x: trans[0], 
            y: trans[1], 
            z: trans[2]
        }
    }
}
