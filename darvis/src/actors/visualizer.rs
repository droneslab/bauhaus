use std::{borrow::Cow, collections::{BTreeMap, BTreeSet, HashMap, HashSet}, fs::{self, File}, io::BufWriter, sync::Arc, thread::sleep, time::Duration};
use log::{debug, warn};
use mcap::{Schema, Channel, records::MessageHeader, Writer};
use opencv::prelude::{Mat, MatTraitConst, MatTraitConstManual};
use foxglove::{foxglove::items::{line_primitive, scene_entity_deletion, ArrowPrimitive, Color, FrameTransform, LinePrimitive, MapInfo, Point3, Quaternion, RawImage, SceneEntity, SceneEntityDeletion, SceneUpdate, SpherePrimitive, Vector3}, get_file_descriptor_set_bytes, make_pose};
use base64::{engine::general_purpose, Engine as _};

use core::{
    config::SETTINGS, matrix::{DVVector3, DVVectorOfKeyPoint}, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::messages::{ShutdownMsg, VisFeatureMatchMsg, VisFeaturesMsg, VisTrajectoryMsg}, map::{map::Id, pose::{DVRotation, Pose}, read_only_lock::ReadWriteMap}, modules::image, registered_actors::VISUALIZER
};

use super::messages::{LoopClosureEssentialGraphMsg, LoopClosureGBAMsg, LoopClosureMapPointFusionMsg, VisTrajectoryTrackingMsg};

// Default visual stuff //
// Trajectory and frame
static TRAJECTORY_COLOR: Color = Color { r: 1.0, g: 0.0, b: 0.0, a: 1.0 };
static CONNECTED_KF_COLOR: Color = Color { r: 0.0, g: 1.0, b: 0.0, a: 1.0 };
static CURRENT_FRAME_COLOR: Color = Color { r: 0.0, g: 1.0, b: 0.0, a: 1.0 };
static KEYFRAME_COLOR: Color = Color { r: 0.0, g: 0.0, b: 1.0, a: 1.0 };
// Mappoints and mappoint matches
static MAPPOINT_COLOR: Color = Color { r: 1.0, g: 1.0, b: 1.0, a: 1.0 };
static MAPPOINT_MATCH_COLOR: Color = Color { r: 0.0, g: 1.0, b: 0.0, a: 1.0 };
static MAPPOINT_LOCAL_COLOR: Color = Color { r: 0.0, g: 0.0, b: 1.0, a: 1.0 };
static MAPPOINT_SIZE: Vector3 = Vector3 {  x: 0.01, y: 0.01, z: 0.01 };
static MAPPOINT_LARGE_SIZE: Vector3 = Vector3 { x: 0.1, y: 0.1, z: 0.1 };

static TRACKING_TRAJECTORY_COLOR: Color = Color { r: 0.0, g: 0.8, b: 0.0, a: 1.0 };
static TRACKING_TRAJECTORY_COLOR2: Color = Color { r: 0.8, g: 0.0, b: 0.0, a: 1.0 };
static TRACKING_TRAJECTORY_COLOR3: Color = Color { r: 0.0, g: 0.0, b: 0.8, a: 1.0 };

enum ImageDrawType {
    NONE,
    PLAIN,
    FEATURES,
    FEATURESANDMATCHES
}

pub const IMAGE_CHANNEL: &str = "/camera";
pub const TRANSFORM_CHANNEL: &str = "/frame_transforms";
pub const TRAJECTORY_CHANNEL: &str = "/trajectory";
pub const MAPPOINTS_CHANNEL: &str = "/mappoints";
//Debugging:
pub const LOOP_CLOSURE_CHANNEL: &str = "/loop_closure";
pub const CONNECTED_KFS_CHANNEL: &str = "/connected_kfs";
pub const MAP_INFO_CHANNEL: &str = "/map_info";
pub const DEBUG_CHANNEL: &str = "/debug";
pub const TRACKING_TRAJECTORY_CHANNEL: &str = "/trajectory_tracking";

pub struct DarvisVisualizer {
    system: System,
    map: ReadWriteMap,

    writer: FoxGloveWriter,

    // For drawing images
    image_draw_type: ImageDrawType,
    current_image: Option<Mat>,
    prev_image: Option<Mat>,
    current_keypoints: Option<DVVectorOfKeyPoint>,
    prev_keypoints: Option<DVVectorOfKeyPoint>,
    previous_mappoints: HashSet<Id>,
    previous_keyframes: HashSet<Id>,

    current_update_id: u64,
    prev_poses: Vec<Pose>,
}

impl Actor for DarvisVisualizer {
    type MapRef = ReadWriteMap;

    #[tokio::main]
    async fn spawn(system: System, map: Self::MapRef) {
        // Trajectory starts at 0,0
        let _prev_pose = Point3 { x: 0.0, y: 0.0, z: 0.0 };

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


        // Create foxglove writer, also spawns server if streaming
        // Code in here instead of inside DarvisVisualizer to avoid making new_actorstate async
        let channels = HashMap::from([
            (IMAGE_CHANNEL, "foxglove.RawImage"),
            (TRANSFORM_CHANNEL, "foxglove.FrameTransform"),
            (TRAJECTORY_CHANNEL, "foxglove.SceneUpdate"),
            (MAPPOINTS_CHANNEL, "foxglove.SceneUpdate"),
            (LOOP_CLOSURE_CHANNEL, "foxglove.SceneUpdate"),
            (CONNECTED_KFS_CHANNEL, "foxglove.SceneUpdate"),
            (MAP_INFO_CHANNEL, "foxglove.MapInfo"),
            (DEBUG_CHANNEL, "foxglove.SceneUpdate"),
            (TRACKING_TRAJECTORY_CHANNEL, "foxglove.SceneUpdate"),
        ]);

        let mut writer = if SETTINGS.get::<bool>(VISUALIZER, "stream") {
        let server = foxglove_ws::FoxgloveWebSocket::new();
            tokio::spawn({
                let server = server.clone();
                async move { server.serve(([127, 0, 0, 1], SETTINGS.get::<i32>(VISUALIZER, "port") as u16)).await }
            });
            FoxGloveWriter::new_with_server(server, channels).await.expect("Could not create foxglove server")
        } else {
            let mcap_file_path = SETTINGS.get::<String>(VISUALIZER, "mcap_file_path");
            FoxGloveWriter::new_with_mcap_file(&mcap_file_path, channels).expect("Could not create foxglove writer")
        };

        let transform = FrameTransform {
            timestamp: Some(prost_types::Timestamp { seconds: 0,  nanos: 0 }),
            parent_frame_id: "world".to_string(),
            child_frame_id: "camera".to_string(),
            translation: Some(Vector3{ x: 0.0, y: 0.0, z: 0.0 }),
            rotation: Some(Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }),
        };
        writer.write(TRANSFORM_CHANNEL, transform, 0.0, 0).await.expect("Could not write transform");


        let mut actor = DarvisVisualizer{
            system,
            map,
            image_draw_type,
            current_update_id: 0,
            prev_poses: vec![],
            current_image: None,
            prev_image: None,
            current_keypoints: None,
            prev_keypoints: None,
            previous_mappoints: HashSet::new(),
            previous_keyframes: HashSet::new(),
            writer
        };

        loop {
            let message = actor.system.receive().unwrap();
            if actor.handle_message(message).await.unwrap_or(false) {
                break;
            }
            actor.map.match_map_version();
        }
    }

}

impl DarvisVisualizer {
    async fn handle_message(&mut self, message: MessageBox) -> Result<bool, Box<dyn std::error::Error>> {
        if message.is::<VisFeaturesMsg>() {
            let msg = message.downcast::<VisFeaturesMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
            self.update_draw_image(*msg).await;
        } else if message.is::<VisFeatureMatchMsg>() {
            let msg = message.downcast::<VisFeatureMatchMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
            self.update_draw_image_with_matches(*msg).await;
        } else if message.is::<VisTrajectoryMsg>() {
            let msg = message.downcast::<VisTrajectoryMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
            self.update_draw_map(*msg).await?;
        } else if message.is::<VisTrajectoryTrackingMsg>() {
            let msg = message.downcast::<VisTrajectoryTrackingMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
            self.draw_frame_trajectory(msg.pose, msg.timestamp).await?;
        } else if message.is::<LoopClosureMapPointFusionMsg>() {
            let msg = message.downcast::<LoopClosureMapPointFusionMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
            self.update_loop_closure_mappoint_fusion(*msg).await?;
        } else if message.is::<LoopClosureGBAMsg>() {
            let msg = message.downcast::<LoopClosureGBAMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
            self.debug_gba( msg.kf_id, msg.timestamp).await.expect("Visualizer could not draw mappoints!");

        } else if message.is::<LoopClosureEssentialGraphMsg>() {
            // let msg = message.downcast::<LoopClosureEssentialGraphMsg>().unwrap_or_else(|_| panic!("Could not downcast visualizer message!"));
            // self.update_loop_closure_essential_graph(&mut writer,*msg).await;
        } else if message.is::<ShutdownMsg>() {
            // SHUTDOWN
            // Sleep a little to allow other threads to finish
            sleep(Duration::from_millis(100));
            warn!("Closing mcap file");
            self.writer.finish().expect("Could not close file");
            return Ok(true);
        } else {
            warn!("Visualizer received unknown message type!");
        }
        return Ok(false);
    }

    async fn _update_loop_closure_essential_graph(
        &mut self, msg: LoopClosureEssentialGraphMsg
    ) -> Result<(), Box<dyn std::error::Error>> {
        // This is for debugging
        self.clear_scene(msg.timestamp, LOOP_CLOSURE_CHANNEL).await?;
        let mut entities = Vec::new();
        let lock = self.map.read()?;

        for kf_id in &msg.relevant_keyframes {
            let kf = lock.get_keyframe(*kf_id);
            let pose = kf.get_pose().inverse();
            let kf_entity = self.create_scene_entity(
                msg.timestamp, "world",
                format!("kf {}", kf_id),
                vec![],
                vec![],
                vec![self.create_sphere(&pose, Color { r: 1.0, g: 0.0, b: 0.0, a: 1.0 }, MAPPOINT_LARGE_SIZE.clone())]
            );
            entities.push(kf_entity);
        }

        let sceneupdate = SceneUpdate {
            deletions: vec![],
            entities
        };

        self.writer.write(LOOP_CLOSURE_CHANNEL, sceneupdate, msg.timestamp, 1).await?;
        Ok(())
    }

    async fn update_loop_closure_mappoint_fusion(
        &mut self, msg: LoopClosureMapPointFusionMsg
    ) -> Result<(), Box<dyn std::error::Error>> {
        // This is for debugging
        self.clear_scene(msg.timestamp, LOOP_CLOSURE_CHANNEL).await?;
        let mut entities = Vec::new();
        let lock = self.map.read()?;

        for (mappoint_id, mappoint) in &lock.mappoints {
            let mp_sphere = {
                if msg.mappoint_matches.contains(&mappoint_id) {
                    let pose = Pose::new_with_default_rot(mappoint.position);
                    Some(self.create_sphere(&pose, MAPPOINT_MATCH_COLOR.clone(), MAPPOINT_LARGE_SIZE.clone()))
                } else if msg.loop_mappoints.contains(&mappoint_id) {
                    let pose = Pose::new_with_default_rot(mappoint.position);
                    Some(self.create_sphere(&pose, MAPPOINT_LOCAL_COLOR.clone(), MAPPOINT_LARGE_SIZE.clone()))
                } else {
                    // let pose = Pose::new_with_default_rot(mappoint.position);
                    // self.create_sphere(&pose, MAPPOINT_COLOR.clone(), MAPPOINT_SIZE.clone())
                    None
                }
            };

            match mp_sphere {
                Some(mp_sphere) => {
                    let mp_entity = self.create_scene_entity(
                        msg.timestamp, "world",
                        format!("mp {}", mappoint_id),
                        vec![],
                        vec![],
                        vec![mp_sphere]
                    );
                    entities.push(mp_entity);
                },
                None => continue
            }
        }

        for kf_id in &msg.keyframes_affected {
            let kf = lock.get_keyframe(*kf_id);
            let pose = kf.get_pose().inverse();
            let kf_entity = self.create_scene_entity(
                msg.timestamp, "world",
                format!("kf {}", kf_id),
                vec![],
                vec![],
                vec![self.create_sphere(&pose, KEYFRAME_COLOR.clone(), MAPPOINT_LARGE_SIZE.clone())]
            );
            entities.push(kf_entity);
        }

        let sceneupdate = SceneUpdate {
            deletions: vec![],
            entities
        };

        self.writer.write(LOOP_CLOSURE_CHANNEL, sceneupdate, msg.timestamp, 0).await?;
        Ok(())
    }

    async fn debug_gba(&mut self, kf_id: Id, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // This is for debugging
        // Draws all mappoints in map (different color if they are matches), simplified version of draw_mappoints
        // Highlights mappoint matches for given kf_id

        self.clear_scene(timestamp, DEBUG_CHANNEL).await?;

        let mut entities = Vec::new();

        let map = self.map.read()?;
        let mut curr_mappoints = HashSet::new();
        for (mappoint_id, mappoint) in &map.mappoints {
            let pose = Pose::new_with_default_rot(mappoint.position);
            curr_mappoints.insert(*mappoint_id);

            let mp_sphere = {
                if map.get_keyframe(kf_id).get_mp_match_index(mappoint_id).is_some() {
                    self.create_sphere(&pose, MAPPOINT_MATCH_COLOR.clone(), MAPPOINT_SIZE.clone())
                } else {
                    self.create_sphere(&pose, MAPPOINT_COLOR.clone(), MAPPOINT_SIZE.clone())
                }
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

        // Delete mappoints that are no longer in the map but were previously drawn
        let deleted_mappoints = self.previous_mappoints.difference(&curr_mappoints);

        let deletions: Vec<SceneEntityDeletion> = deleted_mappoints.map(|id|
            self.create_scene_entity_deletion(timestamp, format!("mp {}", id),
        )).collect();
        self.previous_mappoints = curr_mappoints;

        let sceneupdate = SceneUpdate {
            deletions,
            entities
        };

        self.writer.write(DEBUG_CHANNEL, sceneupdate, timestamp, 0).await?;

        Ok(())
    }

    async fn update_draw_image(&mut self, msg: VisFeaturesMsg) {
        match self.image_draw_type {
            ImageDrawType::NONE => Ok(()),
            ImageDrawType::PLAIN => {
                // Greyscale images are encoded as mono8
                // For mono8 (CV_8UC1) each element is u8 -> size is 1
                self.draw_image(&msg.image, "mono8", 1, msg.timestamp).await
            },
            ImageDrawType::FEATURES => {
                let image_with_features = image::write_features(&msg.image, &msg.keypoints).expect("Could not draw features to mat");
                // This is a color image encoded as bgr8
                // Each element is u8 (size is 1) * 3 (for 3 channels)
                self.draw_image(&image_with_features, "bgr8", 3, msg.timestamp).await
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

    async fn update_draw_image_with_matches(&mut self, msg: VisFeatureMatchMsg) {
        if matches!(self.image_draw_type, ImageDrawType::FEATURESANDMATCHES) && self.prev_image.is_some() {
            let image_with_matches = image::write_feature_matches(
                &self.prev_image.as_ref().unwrap(),
                &self.current_image.as_ref().unwrap(),
                &self.prev_keypoints.as_ref().unwrap(),
                &self.current_keypoints.as_ref().unwrap(),
                &msg.matches
            ).expect("Could not write feature matches to mat");

            // Encoding and mat element size same as for ImageDrawType::FEATURES
            self.draw_image(&image_with_matches, "bgr8", 3, msg.timestamp).await.expect("Could not draw image with matches to mat");
        };
    }

    async fn update_draw_map(&mut self, msg: VisTrajectoryMsg) -> Result<(), Box<dyn std::error::Error>> {
        self.draw_trajectory(msg.pose, msg.timestamp, SETTINGS.get::<bool>(VISUALIZER, "draw_graph")).await?;

        println!("Update draw map??");

        self.plot_map_info(&msg.mappoint_matches, msg.timestamp).await?;

        if !SETTINGS.get::<bool>(VISUALIZER, "draw_only_trajectory") {
            self.draw_mappoints(&msg.mappoints_in_tracking, &msg.mappoint_matches, msg.timestamp).await?;
        }

        // if SETTINGS.get::<bool>(VISUALIZER, "draw_graph") {
        //     self.draw_connected_kfs(msg.timestamp).await;
        // }
        self.current_update_id += 1;
        // self.prev_pose = msg.pose.into();
        Ok(())
    }

    async fn draw_frame_trajectory(&mut self, frame_pose: Pose, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // Instead of the regular draw_trajectory, which draws the current frame pose and all previous keyframes,
        // this function draws only the trajectory of reported current frame poses

        let mut entities_traj = vec![];
        let inverse_frame_pose = frame_pose.inverse();

        // Draw current pose
        // This entity is overwritten at every update, so only one camera is in view at all times
        entities_traj.push(
            self.create_frame_scene_entity(
                timestamp, 
                "world",
                format!("cam {}", timestamp),
                &inverse_frame_pose,
                TRACKING_TRAJECTORY_COLOR.clone()
            )
        );

        println!("TRAJECTORY POSE: {:?}", inverse_frame_pose);

        if self.prev_poses.len() > 0 {
            let mut curr_pose = frame_pose;
            let mut i = 0;
            for prev_pose in self.prev_poses.iter().rev() {
                // Draw line from current pose to previous pose
                let points = vec![(*prev_pose).into(), curr_pose.into()];
                entities_traj.push(
                    self.create_scene_entity(
                        timestamp,
                        "world",
                        format!("t {}->{}", i, i+1),
                        vec![],
                        vec![self.create_line(points, TRACKING_TRAJECTORY_COLOR2.clone(), 3.0),], 
                        vec![]
                    )
                );

                entities_traj.push(
                    self.create_frame_scene_entity(
                        timestamp, 
                        "world",
                        format!("kf {}", i),
                        &prev_pose,
                        TRACKING_TRAJECTORY_COLOR3.clone(),
                    )
                );
                curr_pose = prev_pose.clone();
                i += 1;
            }
        }

        let sceneupdate = SceneUpdate {
            deletions: vec![],
            entities: entities_traj
        };

        self.writer.write(TRACKING_TRAJECTORY_CHANNEL, sceneupdate, timestamp, 0).await.expect("Could not write trajectory to foxglove");


        self.prev_poses.push(frame_pose.into());

        Ok(())
    }

    async fn draw_trajectory(&mut self, frame_pose: Pose, timestamp: Timestamp, draw_graph: bool) -> Result<(), Box<dyn std::error::Error>> {
        // debug!("Drawing trajectory at timestamp {} with pose {:?}", timestamp, frame_pose.inverse());

        self.clear_scene(timestamp, TRAJECTORY_CHANNEL).await.expect("Could not clear scene");

        let mut entities_graph = vec![]; // connected keyframes graph

        let mut entities_traj = vec![]; // keyframes, frames, and trajectory
        let inverse_frame_pose = frame_pose.inverse();

        // Draw current pose
        // This entity is overwritten at every update, so only one camera is in view at all times
        entities_traj.push(
            self.create_frame_scene_entity(
                timestamp, 
                "world",
                format!("cam"),
                &inverse_frame_pose,
                CURRENT_FRAME_COLOR.clone()
            )
        );

        // Write transform from world to camera
        let trans = inverse_frame_pose.get_translation();
        let rot = inverse_frame_pose.get_quaternion();
        let transform = FrameTransform {
            timestamp: Some(prost_types::Timestamp { seconds: 0,  nanos: 0 }),
            parent_frame_id: "world".to_string(),
            child_frame_id: "camera".to_string(),
            translation: Some(Vector3{ x: trans[0], y: trans[1], z: trans[2] }),
            rotation: Some(Quaternion { x: rot[0], y: rot[1], z: rot[2], w: rot[3] }),

        };
        self.writer.write(TRANSFORM_CHANNEL, transform, timestamp, 0).await.expect("Could not write transform");

        let map = self.map.read()?;

        // Draw keyframes and trajectory
        // Re-draw each time because kf poses may have neen optimized since last drawing
        let mut sorted_kfs = map.get_keyframes_iter().map(|(id, _)| id).collect::<Vec<&Id>>();
        sorted_kfs.sort();
        let mut prev_pose: Point3 = Pose::default().into();
        let mut prev_id = 0;
        for id in sorted_kfs {
            let kf = map.get_keyframe(*id);
            let curr_pose = kf.get_pose().inverse();

            if curr_pose == inverse_frame_pose {
                // Skip over drawing a kf if it was created at this frame, 
                // so that it doesn't cover up the drawing of the current frame. 
                continue;
            }

            let points = vec![prev_pose, curr_pose.into()];
            entities_traj.push(
                self.create_scene_entity(
                    timestamp,
                    "world",
                    format!("t {}->{}", prev_id, id),
                    vec![],
                    vec![self.create_line(points, TRAJECTORY_COLOR.clone(), 3.0),], 
                    vec![]
                )
            );

            entities_traj.push(
                self.create_frame_scene_entity(
                    timestamp, 
                    "world",
                    format!("kf {}", id),
                    &curr_pose,
                    KEYFRAME_COLOR.clone(),
                )
            );

            if draw_graph {
                let mut lines = vec![];
                // Print connected keyframes
                for connected_kf_id in &kf.get_covisibles_by_weight(100) {
                    let connected_kf = map.get_keyframe(*connected_kf_id);
                    let connected_pose = connected_kf.get_pose().inverse();
                    let points = vec![curr_pose.into(), connected_pose.into()];
                    lines.push(self.create_line(points, CONNECTED_KF_COLOR.clone(), 1.0));
                }

                let kf_entity = self.create_scene_entity(
                    timestamp, "world",
                    format!("kf connections {}", kf.id),
                    vec![], lines, vec![]
                );
                entities_graph.push(kf_entity);
            }

            prev_pose = curr_pose.into();
            prev_id = *id;
        }

        // Delete keyframes that are no longer in the map but were previously drawn
        let curr_keyframes = map.get_keyframes_iter().map(|(id, _)| *id).collect::<HashSet<Id>>();
        let deleted_keyframes = self.previous_keyframes.difference(&curr_keyframes);
        let mut deletions_traj = vec![];
        let mut deletions_graph = vec![];
        for deleted_kf in deleted_keyframes {
            deletions_traj.push(self.create_scene_entity_deletion(timestamp, format!("kf {}", deleted_kf)));
            if draw_graph {
                deletions_graph.push(self.create_scene_entity_deletion(timestamp, format!("kf connections {}", deleted_kf)));
            }
        }
        self.previous_keyframes = curr_keyframes;

        // Lastly, add the trajectory line from the last keyframe to the current pose
        let points = vec![prev_pose, frame_pose.inverse().into()];
        entities_traj.push(
            self.create_scene_entity(
                timestamp, 
                "world",
                format!("t to frame"),
                vec![],
                vec![self.create_line(points, TRAJECTORY_COLOR.clone(), 3.0)], 
                vec![]
            )
        );

        let sceneupdate = SceneUpdate {
            deletions: deletions_traj,
            entities: entities_traj
        };

        self.writer.write(TRAJECTORY_CHANNEL, sceneupdate, timestamp, 0).await.expect("Could not write trajectory to foxglove");


        if draw_graph {
            // Draw graph of connected keyframes
            let sceneupdate = SceneUpdate {
                deletions: deletions_graph,
                entities: entities_graph
            };

            self.writer.write(CONNECTED_KFS_CHANNEL, sceneupdate, timestamp, 0).await.expect("Could not write trajectory to foxglove");
        }

        Ok(())
    }

    async fn plot_map_info(&mut self, tracked_mappoints: &Vec<std::option::Option<(i32, bool)>>, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // Write map info to foxglove
        let map = self.map.read()?;

        let map_info = MapInfo {
            keyframes: map.num_keyframes() as f64,
            mappoints: map.mappoints.len() as f64,
            tracked_mappoints: tracked_mappoints.iter().filter(|item| item.is_some()).count() as f64,
        };

        self.writer.write(MAP_INFO_CHANNEL, map_info, timestamp, 0).await?;
        Ok(())

    }

    async fn draw_mappoints(&mut self, local_mappoints: &BTreeSet<Id>, mappoint_matches: &Vec<Option<(Id, bool)>>, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // Mappoints in map (different color if they are matches)
        // Should be overwritten when there is new info for a mappoint

        // I'm turning off the clear scene for now
        // it seems like it has better performance but it flashes every time it clears the screen which is annoying
        self.clear_scene(timestamp, MAPPOINTS_CHANNEL).await?;

        let mut entities = Vec::new();

        let mappoint_match_ids = mappoint_matches.iter().filter_map(|item| item.map(|(id, _)| id)).collect::<HashSet<Id>>();

        let map = self.map.read()?;
        let mut curr_mappoints = HashSet::new();
        for (mappoint_id, mappoint) in &map.mappoints {
            let pose = Pose::new_with_default_rot(mappoint.position);
            curr_mappoints.insert(*mappoint_id);

            let mp_sphere = {
                if mappoint_match_ids.contains(&mappoint_id) {
                    // If mappoint is a match with the current frame
                    self.create_sphere(&pose, MAPPOINT_MATCH_COLOR.clone(), MAPPOINT_SIZE.clone())
                } else if local_mappoints.contains(&mappoint_id) && SETTINGS.get::<bool>(VISUALIZER, "draw_local_mappoints") {
                    // If mappoint is not a match, but tracking has it as a local mappoint
                    self.create_sphere(&pose, MAPPOINT_LOCAL_COLOR.clone(), MAPPOINT_SIZE.clone())
                } else if SETTINGS.get::<bool>(VISUALIZER, "draw_all_mappoints") || !self.previous_mappoints.contains(&mappoint_id) {
                    // All other mappoints.
                    // Only draw if we are drawing all mappoints, or if the mappoint was recently created/seen.
                    self.create_sphere(&pose, MAPPOINT_COLOR.clone(), MAPPOINT_SIZE.clone())
                } else {
                    continue;
                }
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

        // Delete mappoints that are no longer in the map but were previously drawn
        let deleted_mappoints = self.previous_mappoints.difference(&curr_mappoints);

        let deletions: Vec<SceneEntityDeletion> = deleted_mappoints.map(|id|
            self.create_scene_entity_deletion(timestamp, format!("mp {}", id),
        )).collect();
        self.previous_mappoints = curr_mappoints;

        let sceneupdate = SceneUpdate {
            deletions,
            entities
        };

        self.writer.write(MAPPOINTS_CHANNEL, sceneupdate, timestamp, 0).await?;

        Ok(())
    }

    async fn _draw_connected_kfs(&mut self, timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // Draw connected keyframes graph
        // Will be re-drawn each time because kf may gain/lose connections over time
        // But won't update if a keyframe is deleted! Need to keep track of that and delete manually

        let mut entities = Vec::new();
        let map = self.map.read()?;
        for (kf_id, kf) in map.get_keyframes_iter() {
            let pose = kf.get_pose().inverse();

            let mut lines = Vec::new();

            for connected_kf_id in &kf.get_covisibility_keyframes(100) {
                let connected_kf = map.get_keyframe(*connected_kf_id);
                let connected_pose = connected_kf.get_pose().inverse();
                let points = vec![pose.into(), connected_pose.into()];
                lines.push(self.create_line(points, CONNECTED_KF_COLOR.clone(), 1.0));
            }

            let kf_entity = self.create_scene_entity(
                timestamp, "world",
                format!("kf {}", kf_id),
                vec![], lines, vec![]
            );
            entities.push(kf_entity);
        }

        // Delete keyframes that are no longer in the map but were previously drawn
        let curr_keyframes = map.get_keyframes_iter().map(|(id, _)| *id).collect::<HashSet<Id>>();
        let deleted_keyframes = self.previous_keyframes.difference(&curr_keyframes);
        let deletions = deleted_keyframes.map(|id|
            self.create_scene_entity_deletion(timestamp, format!("kf {}", id),
        )).collect();
        self.previous_keyframes = curr_keyframes;

        let sceneupdate = SceneUpdate {
            deletions,
            entities
        };

        self.writer.write(CONNECTED_KFS_CHANNEL, sceneupdate, timestamp, 0).await?;

        Ok(())
    }

    async fn draw_image(
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

        self.writer.write(IMAGE_CHANNEL, image_msg, timestamp, 0).await?;

        Ok(())
    }

    fn create_sphere(&self, pose: &Pose, color: Color, size: Vector3) -> SpherePrimitive {
        SpherePrimitive {
            pose: Some(pose.into()),
            size: Some(size),
            color: Some(color),
        }
    }

    fn _create_arrow(&self, pose: &Pose, color: Color) -> ArrowPrimitive {
        ArrowPrimitive { 
            pose: Some(pose.into()),
            color: Some(color),
            shaft_length: 0.005,
            shaft_diameter: 0.005,
            head_length: 0.01,
            head_diameter: 0.01
        }
    }

    fn create_line(&self, points: Vec<Point3>, color: Color, thickness: f64) -> LinePrimitive {
        LinePrimitive {
            r#type: line_primitive::Type::LineStrip as i32,
            pose: make_pose(0.0, 0.0, 0.0),
            thickness,
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

    async fn clear_scene(&mut self, timestamp: Timestamp, channel_name: &str) -> Result<(), Box<dyn std::error::Error>> {
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
        self.writer.write(channel_name, sceneupdate, timestamp, 0).await?;
        Ok(())
    }

    fn create_frame_scene_entity(&self, timestamp: Timestamp, frame_id: &str, entity_id: String, pose: &Pose, color: Color) -> SceneEntity {
        fn create_line(init_point1: (f64, f64, f64), init_point2: (f64, f64, f64), rot: &DVRotation, trans: &DVVector3<f64>, color: &Color) -> LinePrimitive {
            let point1 = {
                let temp = **rot * *DVVector3::new_with(init_point1.0, init_point1.1, init_point1.2);
                Point3{x: temp.x + trans.x, y: temp.y + trans.y, z: temp.z + trans.z}
            };
            let point2 = {
                let temp = **rot * *DVVector3::new_with(init_point2.0, init_point2.1, init_point2.2);
                Point3{x: temp.x + trans.x, y: temp.y + trans.y, z: temp.z + trans.z}
            };

            LinePrimitive {
                r#type: line_primitive::Type::LineStrip as i32,
                pose: make_pose(0.0, 0.0, 0.0),
                thickness: 2.0,
                scale_invariant: true,
                points: vec![point1, point2],
                color: Some(color.clone()),
                colors: vec![],
                indices: vec![],
            }
        }

        let (seconds, nanos) = convert_timestamp(timestamp);
        let trans = pose.get_translation();
        let rot = pose.get_rotation();
        let w = 0.075;
        let h = 0.05;
        let z = 0.025;

        let lines = vec![
            // Rectangle
            create_line((w, h, z), (w, - h, z), &rot, &trans, &color),
            create_line((- w, h, z), (- w, - h, z), &rot, &trans, &color),
            create_line((- w, h, z), (w, h, z), &rot, &trans, &color),
            create_line((- w, - h, z), (w, - h, z), &rot, &trans, &color),
            // Diagonals
            // create_line((- w, h, z), (w, - h, z), &rot, &trans, &color),
            // create_line((w, h, z), (- w, - h, z), &rot, &trans, &color),
            // Protruding pyramid
            create_line((0.0, 0.0, 0.0), (w, h, z), &rot, &trans, &color),
            create_line((0.0, 0.0, 0.0), (w, - h, z), &rot, &trans, &color),
            create_line((0.0, 0.0, 0.0), (- w, - h, z), &rot, &trans, &color),
            create_line((0.0, 0.0, 0.0), (- w, h, z), &rot, &trans, &color),
            ];

        SceneEntity {
            timestamp: Some(prost_types::Timestamp { seconds, nanos }),
            frame_id: frame_id.to_string(),
            id: entity_id,
            lifetime: None,
            frame_locked: false,
            metadata: vec![],
            arrows: vec![],
            cubes: vec![],
            spheres: vec![],
            cylinders: vec![],
            lines,
            triangles: vec![],
            texts: vec![],
            models: vec![],
        }
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
    let nanos = 1e9 - timestamp;
    (1 as i64, nanos as i32)
}

pub enum FoxGloveWriter {
    McapFile { 
        writer: Writer<'static, BufWriter<File>>,
        channels: HashMap<String, u16>,
    },
    Stream {
        server: foxglove_ws::FoxgloveWebSocket,
        channels: HashMap<String, foxglove_ws::Channel>,
    }
}

impl FoxGloveWriter {
    pub fn new_with_mcap_file(path: &str, channels_to_add: HashMap<&str, &str>) -> Result<FoxGloveWriter, Box<dyn std::error::Error>> {
        let mut writer = Writer::new(BufWriter::new(fs::File::create(path)?))?;
        let mut channels = HashMap::new();
        for (topic, schema) in channels_to_add {
            let schema = Schema {
                name: schema.to_string(),
                encoding: String::from("protobuf"),
                data: Cow::Borrowed(get_file_descriptor_set_bytes()),
            };
            let channel = Channel {
                topic: topic.to_string(),
                schema: Some(Arc::new(schema)),
                message_encoding: String::from("protobuf"),
                metadata: BTreeMap::default()
            };
            channels.insert(topic.to_string(), writer.add_channel(&channel)?);
        }
        Ok(FoxGloveWriter::McapFile { writer, channels })
    }

    pub async fn new_with_server(server: foxglove_ws::FoxgloveWebSocket, channels_to_add: HashMap<&str, &str>) -> Result<FoxGloveWriter, Box<dyn std::error::Error>> {
        let mut channels = HashMap::new();
        for (topic, schema) in channels_to_add {
            let channel = server
                .publish(
                    topic.to_string(),
                    "protobuf".to_string(),
                    schema.to_string(),
                    // Foxglove expects the schema data to be a binary FileDescriptorSet. For Foxglove WebSocket connections, 
                    // the schema must additionally be base64-encoded because it is represented as a string.
                    // https://docs.foxglove.dev/docs/connecting-to-data/frameworks/custom/#foxglove-websocket
                    general_purpose::STANDARD.encode(&get_file_descriptor_set_bytes()),
                    "protobuf".to_string(),
                    false,
                )
                .await?;

            channels.insert(topic.to_string(), channel);
        }
        Ok(FoxGloveWriter::Stream { server, channels })
    }

    pub async fn write<T: prost::Message>(
        &mut self,
        channel_name: &str,
        data: T,
        timestamp: Timestamp,
        sequence: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let time_in_nsec = timestamp * 1000000000.0;

        match self {
            FoxGloveWriter::McapFile { writer, channels } => {
                let channel_id = channels.get(channel_name).expect("Could not find channel");
                Ok(
                    writer.write_to_known_channel(
                        &MessageHeader {
                            channel_id: *channel_id,
                            sequence,
                            log_time: time_in_nsec as u64,
                            publish_time: time_in_nsec as u64,
                        },
                        &data.encode_to_vec()
                    )?
                )
            },
            FoxGloveWriter::Stream { server: _, channels } => {
                let channel = channels.get(channel_name).expect("Could not find channel");
                Ok(
                    channel.send(
                        timestamp as u64,
                        &data.encode_to_vec()
                    )
                    .await?
                )
            }
        }

    }

    pub fn finish(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        match self {
            FoxGloveWriter::McapFile { writer, channels: _ } => {
                Ok(writer.finish()?)
            },
            FoxGloveWriter::Stream { server: _, channels: _ } => { Ok(()) }
        }
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

        // TODO (visualizer): These always show pointing to the right when they shouldn't be
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
