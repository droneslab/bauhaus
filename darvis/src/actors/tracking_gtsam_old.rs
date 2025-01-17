extern crate g2o;
use log::{warn, info, debug, error};
use std::{collections::{BTreeMap, BTreeSet, HashMap}, sync::atomic::Ordering, thread::sleep, time::Duration};
use opencv::{prelude::*, types::VectorOfKeyPoint,types::VectorOfPoint2f, types::VectorOfu8, types::VectorOfMat};

use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{
        messages::{ImageMsg, LastKeyFrameUpdatedMsg, ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg},
        tracking_backend::TrackingState,
    }, map::{features::Features, frame::Frame, keyframe::MapPointMatches, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{image, imu::{ImuBias, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{FeatureExtractionModule, ImuModule}, optimizer, orbslam_extractor::ORBExtractor, relocalization::Relocalization}, registered_actors::{CAMERA_MODULE, FEATURE_MATCHING_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, VISUALIZER}, MAP_INITIALIZED
};

use super::{local_mapping::LOCAL_MAPPING_IDLE, messages::{ImagePathMsg, InitKeyFrameMsg, NewKeyFrameMsg, TrajectoryMsg, VisTrajectoryMsg}, tracking_backend::TrackedMapPointData};
use crate::modules::module_definitions::MapInitializationModule;

struct FeatureMetaData {
    id: i64,
    // response: f64,
    lifetime: i32,
    point: opencv::core::Point2f,
}
type GridFeatures = HashMap<i32, Vec<FeatureMetaData>>;

pub struct TrackingGTSAM {
    system: System,
    sensor: Sensor,

    /// Frontend
    orb_extractor_left: Box<dyn FeatureExtractionModule>,
    // _orb_extractor_right: Option<Box<dyn FeatureExtractionModule>>,
    // orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>>,
    // init_id: Id,

    // Frames
    last_frame: Option<Frame>,
    curr_frame_id: i32,
    current_frame: Frame,

    /// Backend
    state: TrackingState,
    // KeyFrames
    ref_kf_id: Option<Id>,
    frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
    last_kf_timestamp: Option<Timestamp>,

    // Modules 
    imu: Option<IMU>,
    // relocalization: Relocalization,

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses

    /// References to map
    map_initialized: bool,
    map: ReadWriteMap,
    initialization: Option<MapInitialization>, // data sent to map actor to initialize new map
    map_scale: f64,

    // Local data used for different stages of tracking
    matches_inliers : i32, // mnMatchesInliers ... Current matches in frame
    local_keyframes: BTreeSet<Id>, //mvpLocalKeyFrames 
    local_mappoints: BTreeSet<Id>, //mvpLocalMapPoints
    // track_in_view: HashMap::<Id, TrackedMapPointData>, // mbTrackInView , member variable in Mappoint
    // track_in_view_r: HashMap::<Id, TrackedMapPointData>, // mbTrackInViewR, member variable in Mappoint
    // kf_track_reference_for_frame: HashMap::<Id, Id>, // mnTrackReferenceForFrame, member variable in Keyframe
    // mp_track_reference_for_frame: HashMap::<Id, Id>,  // mnTrackReferenceForFrame, member variable in Mappoint
    // last_frame_seen: HashMap::<Id, Id>, // mnLastFrameSeen, member variable in Mappoint
    // non_tracked_mappoints: HashMap<Id, i32>, // just for testing


    // Specific to optical flow module
    last_keyframe_mappoint_matches: Option<MapPointMatches>, // mnLastFrameMatches

    /// Global defaults
    // localization_only_mode: bool,
    max_frames_to_insert_kf : i32 , //mMaxFrames , Max Frames to insert keyframes and to check relocalisation
    min_frames_to_insert_kf: i32, // mMinFrames, Min Frames to insert keyframes and to check relocalisation
}

impl Actor for TrackingGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let _orb_extractor_right: Option<Box<dyn FeatureExtractionModule>> = match sensor.frame() {
            FrameSensor::Stereo => Some(Box::new(ORBExtractor::new(false))),
            FrameSensor::Mono | FrameSensor::Rgbd => None,
        };
        let orb_extractor_ini: Option<Box<dyn FeatureExtractionModule>> = match sensor.is_mono() {
            true => Some(Box::new(ORBExtractor::new(true))),
            false => None
        };
        // let imu = match sensor.imu() {
        //     ImuSensor::Some => Some(IMU::new()),
        //     _ => None,
        // };
        let imu = Some(IMU::new());

        let mut actor = TrackingGTSAM {
            system,
            orb_extractor_left: Box::new(ORBExtractor::new(false)),
            // _orb_extractor_right,
            // orb_extractor_ini,
            map_initialized: false,
            // init_id: 0,
            sensor,
            map,
            initialization: Some(MapInitialization::new()),
            map_scale: 1.0,
            // localization_only_mode: SETTINGS.get::<bool>(SYSTEM, "localization_only_mode"),
            max_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "max_frames_to_insert_kf"),
            min_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "min_frames_to_insert_kf"),
            state: TrackingState::NotInitialized,
            ref_kf_id: None,
            frames_since_last_kf: 0,
            last_kf_timestamp: None,
            imu,
            // relocalization: Relocalization{last_reloc_frame_id: 0, timestamp_lost: None},
            trajectory_poses: Vec::new(),
            // min_num_features: SETTINGS.get::<i32>(TRACKING_BACKEND, "min_num_features")  as u32,
            matches_inliers: 0,
            local_keyframes: BTreeSet::new(),
            local_mappoints: BTreeSet::new(),
            // track_in_view: HashMap::new(),
            // track_in_view_r: HashMap::new(),
            // kf_track_reference_for_frame: HashMap::new(),
            // mp_track_reference_for_frame: HashMap::new(),
            // last_frame_seen: HashMap::new(),
            // non_tracked_mappoints: HashMap::new(),
            last_keyframe_mappoint_matches: None,
            last_frame: None,
            current_frame: Frame::new_no_features(-1, None, 0.0, None).expect("Should be able to make dummy frame"),
            curr_frame_id: -1
        };
        tracy_client::set_thread_name!("tracking gtsam");

        loop {
            let message = actor.system.receive().unwrap();
            if actor.handle_message(message) {
                break;
            }
            actor.map.match_map_version();
        }
    }

}

impl TrackingGTSAM {
    fn handle_message(&mut self, message: MessageBox) -> bool {
        if message.is::<ImagePathMsg>() || message.is::<ImageMsg>() {
            if self.system.queue_full() {
                // Abort additional work if there are too many frames in the msg queue.
                info!("Tracking gtsam dropped 1 frame");
                return false;
            }

            let (image, timestamp, imu_measurements) = if message.is::<ImagePathMsg>() {
                let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (image::read_image_file(&msg.image_path), msg.timestamp, msg.imu_measurements)
            } else {
                let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
                (msg.image, msg.timestamp, msg.imu_measurements)
            };

            self.process_image(image);

            // match self.track(image, timestamp, imu_measurements) {
            //     Ok((mut curr_frame, created_kf)) => { 
            //         curr_frame.ref_kf_id = self.ref_kf_id;

            //         match self.state {
            //             TrackingState::Ok | TrackingState::RecentlyLost => {
            //                 self.update_trajectory_in_logs(created_kf).expect("Could not save trajectory")
            //             },
            //             _ => {},
            //         };
            //         self.last_frame = Some(curr_frame);
            //         self.curr_frame_id += 1;
            //         self.frames_since_last_kf = if created_kf { 0 } else { self.frames_since_last_kf + 1 };
            //         },
            //     Err(e) => {
            //         error!("Error in tracking gtsam: {}", e);
            //         return false;
            //     }
            // };
        } else if message.is::<InitKeyFrameMsg>() {
            // Received from local mapping after it inserts a keyframe
            let msg = message.downcast::<InitKeyFrameMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

            self.ref_kf_id = Some(msg.kf_id);
        } else if message.is::<LastKeyFrameUpdatedMsg>() {
            // Received from local mapping after it culls and creates new MPs for the last inserted KF
            self.state = TrackingState::Ok;
        } else if message.is::<UpdateFrameIMUMsg>() {
            todo!("IMU ... process message from local mapping!");
        } else if message.is::<ShutdownMsg>() {
            return true;
        } else {
            warn!("Tracking backend received unknown message type!");
        }
        return false;
    }

    fn track(
        &mut self,
        curr_img: opencv::core::Mat, timestamp: Timestamp,
        mut imu_measurements: ImuMeasurements,
    ) -> Result<(Frame, bool), Box<dyn std::error::Error>> {

            // OLD CODE FROM OPTICAL FLOW!!!!
        // let mut current_frame: Frame;

        // let mut created_keyframe = false;

        // if matches!(self.state, TrackingState::NotInitialized) {
        //     // If map is not initialized yet, just extract features and try to initialize
        //     let (keypoints, descriptors) = self.extract_features(curr_img.clone(), self.curr_frame_id);

        //     current_frame = Frame::new(
        //         self.curr_frame_id, 
        //         keypoints,
        //         descriptors,
        //         curr_img.cols() as u32,
        //         curr_img.rows() as u32,
        //         Some(curr_img.clone()),
        //         self.last_frame.as_ref(),
        //         self.map.read()?.imu_initialized,
        //         timestamp,
        //     ).expect("Could not create frame!");

        //     if self.initialization.is_none() {
        //         self.initialization = Some(MapInitialization::new());
        //     }
        //     let init_success = self.initialization.as_mut().unwrap().try_initialize(&current_frame, &mut self.imu.as_mut().unwrap().imu_preintegrated_from_last_kf)?;
        //     if init_success {
        //         let (ini_kf_id, curr_kf_id);
        //         {
        //             (ini_kf_id, curr_kf_id) = match self.initialization.as_mut().unwrap().create_initial_map_monocular(&mut self.map,  &mut self.imu.as_mut().unwrap().imu_preintegrated_from_last_kf)? {
        //                 Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, curr_kf_timestamp, map_scale)) => {
        //                     // Map needs to be initialized before tracking can begin. Received from map actor
        //                     self.frames_since_last_kf = 0;
        //                     self.local_keyframes.insert(curr_kf_id);
        //                     self.local_keyframes.insert(ini_kf_id);
        //                     self.local_mappoints = local_mappoints;
        //                     self.ref_kf_id = Some(curr_kf_id);
        //                     self.last_kf_timestamp = Some(curr_kf_timestamp);
        //                     self.map_scale = map_scale;
        //                     if self.sensor.is_imu() {
        //                         self.imu.as_mut().unwrap().imu_preintegrated_from_last_kf = ImuPreIntegrated::new(self.map.read()?.get_keyframe( curr_kf_id).imu_data.imu_preintegrated.as_ref().unwrap().get_updated_bias());
        //                     }

        //                     {
        //                         // Set current frame's updated info from map initialization
        //                         current_frame.pose = Some(curr_kf_pose);
        //                         current_frame.ref_kf_id = Some(curr_kf_id);
        //                         self.last_keyframe_mappoint_matches = Some(self.map.read()?.get_keyframe(curr_kf_id).clone_matches());
        //                     }
        //                     self.state = TrackingState::Ok;

        //                     // Log initial pose in shutdown actor
        //                     self.system.send(SHUTDOWN_ACTOR, 
        //                     Box::new(TrajectoryMsg{
        //                             pose: self.map.read()?.get_keyframe(ini_kf_id).get_pose(),
        //                             ref_kf_id: ini_kf_id,
        //                             timestamp: self.map.read()?.get_keyframe(ini_kf_id).timestamp,
        //                             map_version: self.map.read()?.version
        //                         })
        //                     );

        //                     (ini_kf_id, curr_kf_id)
        //                 },
        //                 None => {
        //                     panic!("Could not create initial map");
        //                 }
        //             };
        //         }

        //         MAP_INITIALIZED.store(true, Ordering::SeqCst);
        //         self.map_initialized = true;

        //         // Send first two keyframes to local mapping
        //         self.system.send(LOCAL_MAPPING, Box::new(
        //             InitKeyFrameMsg { kf_id: ini_kf_id, map_version: self.map.read()?.version }
        //         ));
        //         self.system.send(LOCAL_MAPPING,Box::new(
        //             InitKeyFrameMsg { kf_id: curr_kf_id, map_version: self.map.read()?.version }
        //         ));

        //         self.state = TrackingState::Ok;
        //         sleep(Duration::from_millis(50)); // Sleep just a little to allow local mapping to process first keyframe
        //     } else {
        //         self.state = TrackingState::NotInitialized;
        //     }
        // } else {
        //     // If this is not the first image, track features from the last image

        //     // let (keypoints, descriptors) = self.extract_features(curr_img.clone(), *curr_frame_id);
        //     current_frame = Frame::new_no_features(
        //         self.curr_frame_id, 
        //         Some(curr_img.clone()),
        //         timestamp,
        //         Some(& self.last_frame.as_mut().unwrap())
        //     ).expect("Could not create frame!");

        //     let status = self.get_feature_tracks()?;

        //     debug!("Optical flow, tracked {} from original {}", current_frame.features.num_keypoints, status.len());

        //     let transform = self.calculate_transform()?;
        //     // self.associate_feature_tracks_with_mappoints(&mut current_frame, last_frame);

        //     // Scale translation with median depth

        //     // Old scale using distance between last keyframes
        //     // let try_scale = {
        //     //     let lock = self.map.read()?;
        //     //     let ref_kf_id = self.ref_kf_id.unwrap();

        //     //     let mut count = 1;
        //     //     while lock.get_keyframe(&(ref_kf_id - count)).is_none() {
        //     //         count += 1;
        //     //     }
        //     //     let second_to_last = lock.get_keyframe((ref_kf_id - count));
        //     //     let last_kf = lock.get_keyframe(ref_kf_id);
        //     //     (*second_to_last.get_camera_center() - *last_kf.get_camera_center()).norm()
        //     // };

        //     let new_trans = *transform.get_translation() * (self.map_scale);
        //     current_frame.pose = Some(
        //         Pose::new(new_trans, * transform.get_rotation()) * self.last_frame.as_ref().unwrap().pose.unwrap()
        //     );

        //     // Sofiya: old extracting features
        //     // if current_frame.features.num_keypoints < self.min_num_features {
        //     //     debug!("Re-extracting features");
        //     //     self.extract_features_and_add_to_existing(&mut current_frame, *curr_frame_id)?;

        //     //     // let mut status = VectorOfu8::new();
        //     //     // current_frame.descriptors = opencv::core::Mat::default();
        //     //     // current_frame.keypoints = VectorOfKeyPoint::new();

        //     //     // Self::get_feature_tracks(last_flow_frame, &mut current_frame)?;

        //     //     // debug!("Tracked {} from original {}, desc rows {}", current_frame.keypoints.len(), status.len(), current_frame.descriptors.rows());

        //     //     // debug!("Flow tracking keypoints extracted {} from original {}, desc rows {}", current_frame.keypoints.len(), status.len(), current_frame.descriptors.rows());
        //     // }

        //     // Decide if we should create a new keyframe

        //     // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        //     let c1a = self.frames_since_last_kf >= (self.max_frames_to_insert_kf as i32);
        //     // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        //     let c1b = self.frames_since_last_kf >= (self.min_frames_to_insert_kf as i32) && LOCAL_MAPPING_IDLE.load(Ordering::SeqCst);
        //     //Condition 1c: tracking is weak
        //     let c1c = current_frame.features.num_keypoints < self.min_num_features;


        //     // println!("Need new kf? ({} || {} || {})", c1a, c1b, c1c);

        //     // if c1c {
        //     //     debug!("Re-extracting features");
        //     //     self.extract_features_and_add_to_existing(&mut last_frame, *curr_frame_id - 1)?;
                
        //     //     let status = Self::get_feature_tracks(last_frame, &mut current_frame)?;

        //     //     debug!("Optical flow, tracked {} from original {}", current_frame.features.num_keypoints, status.len());

        //     //     let transform = Self::calculate_transform(&last_frame, &current_frame)?;

        //     //     let new_trans = *transform.get_translation() * (self.map_scale);
        //     //     current_frame.pose = Some(
        //     //         Pose::new(new_trans, * transform.get_rotation()) * last_frame.pose.unwrap()
        //     //     );
        //     // }

        //     if self.sensor.is_imu() {
        //         let lock = self.map.read()?;
        //         if let Some(ref_kf_id) = self.ref_kf_id {
        //             let ref_kf = lock.get_keyframe(ref_kf_id);
        //             current_frame.imu_data.set_new_bias(ref_kf.imu_data.get_imu_bias());
        //         }
        //         self.imu.as_mut().unwrap().preintegrate(&mut imu_measurements, &mut current_frame, self.last_frame.as_mut().unwrap(), lock.last_kf_id);
        //     }


        //     if c1a || c1b || c1c {
        //         self.extract_features_and_add_to_existing()?;

        //         println!("Current pose guess: {:?}", current_frame.pose.unwrap());

        //         // let matches = self.track_reference_keyframe();

        //         // println!("Matches with reference keyframe: {} (kf is {})", matches?, self.ref_kf_id.unwrap());

        //         // // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        //         // // let c2 = (((self.matches_inliers as f32) < tracked_mappoints * th_ref_ratio || need_to_insert_close)) && self.matches_inliers > 15;
        //         // let c2 = matches < self.map.read()?.get_keyframe(&self.ref_kf_id.unwrap()).unwrap().mp_match_len() as f32 * 0.5;

        //         // if true {
        //             debug!("Creating new keyframe!");
        //             created_keyframe = true;

        //             // let (track_success, matches) = self.track_local_map(&mut current_frame)?;
        //             // if !track_success {
        //             //     panic!("Tracking lost with {} matches to local map! Need to relocalize", matches);
        //             // } else {
        //                 self.create_new_keyframe(&mut current_frame)?;
        //             // }
        //         // }
        //     }
        // };

        // self.system.send(VISUALIZER, Box::new(VisFeaturesMsg {
        //     keypoints: DVVectorOfKeyPoint::empty(),
        //     image: curr_img,
        //     timestamp,
        // }));


        todo!();
        // Ok((current_frame, created_keyframe))
    }

    fn vio_imu(&mut self) {
        // Add node value for current pose with initial estimate being previous pose
        if self.curr_frame_id < 2 {
            // prev_camera_pose = Pose3() * Pose3(T_cam_imu_mat);
        }
        // newNodes.insert(Symbol('x', pose_id), prev_camera_pose);

        // Use ImageProcessor to retrieve subscribed features ids and (u,v) image locations for this pose
        // std::vector<FeatureMeasurement> feature_vector = camera_msg->features; 

        //     // Print info about this pose to console
        //     ROS_INFO("frame %d, %lu total features, camera position: (%f, %f, %f)", pose_id, feature_vector.size(), prev_camera_pose.x(), prev_camera_pose.y(), prev_camera_pose.z());
            
        //     // Create object to publish PointCloud of landmarks
        //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr landmark_cloud_msg_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
            
        //     // Convert features from image_processor to landmarks with 3D coordinates and add to ISAM2 graph/point cloud
        //     for (unsigned int i = 0; i < feature_vector.size(); i++) { 
        //     featureToLandmark(feature_vector[i], prev_camera_pose, landmark_cloud_msg_ptr);
        //     }
            
        //     // Publish landmark PointCloud message (in world frame)
        //     landmark_cloud_msg_ptr->header.frame_id = lv.world_frame_id;
        //     landmark_cloud_msg_ptr->height = 1;
        //     landmark_cloud_msg_ptr->width = landmark_cloud_msg_ptr->points.size();
        //     this->landmark_cloud_pub.publish(landmark_cloud_msg_ptr); 
                
        //     if (pose_id == 0) {

        //     // Add prior on pose x0 (zero pose is used to set world frame)
        //     graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', 0), Pose3(), pose_noise);

        //     // Indicate that all node values seen in pose 0 have been seen for next iteration 
        //     optimizedNodes = newNodes; 

        //     } else {
            
        //     // Update ISAM2 graph with new nodes and factors from this pose, optimize graphs
        //     isam->update(graph, newNodes); 

        //     // Each call to iSAM2 update(*) performs one iteration of the iterative nonlinear solver.
        //     // If accuracy is desired at the expense of time, update(*) can be called additional times
        //     // to perform multiple optimizer iterations every step.
        // //      isam->update();

        //     // Update the node values that have been seen up to this point
        //     optimizedNodes = isam->calculateEstimate();
        // //      optimizedNodes.print("Current estimate: ");

        // //      // Print graph to graphviz dot file (render to PDF using "fdp filname.dot -Tpdf > filename.pdf")
        // //      if (pose_id == 1) {
        // //        ofstream os("/home/vkopli/Documents/GRASP/Graphs/VisualISAMActualGraph_1pose_2019-09-18.dot");
        // //        graph.saveGraph(os, newNodes);
        // //        isam->saveGraph("/home/vkopli/Documents/GRASP/Graphs/VisualISAMGraph_1pose_2019-09-05.dot"); 
        // //      }

        //     // Clear the objects holding new factors and node values for the next iteration
        //     graph.resize(0);
        //     newNodes.clear();
            
        //     // Get optimized nodes for next iteration 
        //     prev_camera_pose = optimizedNodes.at<Pose3>(Symbol('x', pose_id));
        //     }

        //     ros::Time timestamp = camera_msg->header.stamp;
        //     pose_id++;
            
        //     publishTf(prev_camera_pose, timestamp);


    }

    fn process_image(&mut self, image: opencv::core::Mat) {
        // Create image pyramids
        let mut pyramid = opencv::core::Mat::default();
        opencv::video::build_optical_flow_pyramid(
            &image, &mut pyramid, opencv::core::Size::new(31, 31), 3, true, opencv::core::BORDER_REFLECT_101, opencv::core::BORDER_CONSTANT, false
        ).expect("Failed to build optical flow pyramid");

        // Detect features in the first frame.
        if self.curr_frame_id == -1 {
            self.initialize_first_frame(image);
        } else {
            // Track the feature in the previous image.
            self.track_features();

            // Add new features into the current image.
            self.add_new_features();

            // Add new features into the current image.
            self.prune_grid_features();
        }

        // Update the previous image and previous features.
        // cam0_prev_img_ptr = cam0_curr_img_ptr;
        // prev_features_ptr = curr_features_ptr;
        // std::swap(prev_cam0_pyramid_, curr_cam0_pyramid_);

        // // Initialize the current features to empty vectors.
        // curr_features_ptr.reset(new GridFeatures());
        // for (int code = 0; code <
        //     processor_config.grid_row*processor_config.grid_col; ++code) {
        //     (*curr_features_ptr)[code] = vector<FeatureMetaData>(0);
        // }
    }

    fn initialize_first_frame(&mut self, image: opencv::core::Mat) {
        // Size of each grid.
        let grid_height = image.rows() / 10;
        let grid_width = image.cols() / 10;

        // Detect new features on the frist image.
        // vector<KeyPoint> new_features(0);
        // detector_ptr->detect(img, new_features);
        let (keypoints, descriptors) = self.orb_extractor_left.extract(DVMatrix::new(image)).unwrap();

        // Not doing this because not using stereo:
        // Find the stereo matched points for the newly detected features.
        // vector<cv::Point2f> cam0_points(new_features.size());
        // for (int i = 0; i < new_features.size(); ++i)
        //     cam0_points[i] = new_features[i].pt;

        // vector<cv::Point2f> cam1_points(0);
        // vector<unsigned char> inlier_markers(0);
        // stereoMatch(cam0_points, cam1_points, inlier_markers);

        // vector<cv::Point2f> cam0_inliers(0);
        // vector<cv::Point2f> cam1_inliers(0);
        // vector<float> response_inliers(0);
        // for (int i = 0; i < inlier_markers.size(); ++i) {
        //     if (inlier_markers[i] == 0) continue;
        //     cam0_inliers.push_back(cam0_points[i]);
        //     cam1_inliers.push_back(cam1_points[i]);
        //     response_inliers.push_back(new_features[i].response);
        // }

        // Group the features into grids
        let mut grid_new_features = GridFeatures::new();
        for i in 0..100 {
            grid_new_features.insert(i, Vec::new());
        }

        // for (int i = 0; i < cam0_inliers.size(); ++i) {
        //     const cv::Point2f& cam0_point = cam0_inliers[i];
        //     const cv::Point2f& cam1_point = cam1_inliers[i];
        //     const float& response = response_inliers[i];

        //     int row = static_cast<int>(cam0_point.y / grid_height);
        //     int col = static_cast<int>(cam0_point.x / grid_width);
        //     int code = row*processor_config.grid_col + col;

        //     FeatureMetaData new_feature;
        //     new_feature.response = response;
        //     new_feature.cam0_point = cam0_point;
        //     new_feature.cam1_point = cam1_point;
        //     grid_new_features[code].push_back(new_feature);
        // }

        // // Sort the new features in each grid based on its response.
        // for (auto& item : grid_new_features)
        //     std::sort(item.second.begin(), item.second.end(),
        //         &ImageProcessor::featureCompareByResponse);

        // // Collect new features within each grid with high response.
        // for (int code = 0; code <
        //     processor_config.grid_row*processor_config.grid_col; ++code) {
        //     vector<FeatureMetaData>& features_this_grid = (*curr_features_ptr)[code];
        //     vector<FeatureMetaData>& new_features_this_grid = grid_new_features[code];

        //     for (int k = 0; k < processor_config.grid_min_feature_num &&
        //         k < new_features_this_grid.size(); ++k) {
        //     features_this_grid.push_back(new_features_this_grid[k]);
        //     features_this_grid.back().id = next_feature_id++;
        //     features_this_grid.back().lifetime = 1;
        //     }
        // }

        // return;

    }

    fn track_features(&mut self) {
        // Size of each grid.
        // static int grid_height =
        //     cam0_curr_img_ptr->image.rows / processor_config.grid_row;
        // static int grid_width =
        //     cam0_curr_img_ptr->image.cols / processor_config.grid_col;

        // // Compute a rough relative rotation which takes a vector
        // // from the previous frame to the current frame.
        // Matx33f cam0_R_p_c;
        // Matx33f cam1_R_p_c;
        // integrateImuData(cam0_R_p_c, cam1_R_p_c);

        // // Organize the features in the previous image.
        // vector<FeatureIDType> prev_ids(0);
        // vector<int> prev_lifetime(0);
        // vector<Point2f> prev_cam0_points(0);
        // vector<Point2f> prev_cam1_points(0);

        // for (const auto& item : *prev_features_ptr) {
        //     for (const auto& prev_feature : item.second) {
        //     prev_ids.push_back(prev_feature.id);
        //     prev_lifetime.push_back(prev_feature.lifetime);
        //     prev_cam0_points.push_back(prev_feature.cam0_point);
        //     prev_cam1_points.push_back(prev_feature.cam1_point);
        //     }
        // }

        // // Number of the features before tracking.
        // before_tracking = prev_cam0_points.size();

        // // Abort tracking if there is no features in
        // // the previous frame.
        // if (prev_ids.size() == 0) return;

        // // Track features using LK optical flow method.
        // vector<Point2f> curr_cam0_points(0);
        // vector<unsigned char> track_inliers(0);

        // predictFeatureTracking(prev_cam0_points,
        //     cam0_R_p_c, cam0_intrinsics, curr_cam0_points);

        // calcOpticalFlowPyrLK(
        //     prev_cam0_pyramid_, curr_cam0_pyramid_,
        //     prev_cam0_points, curr_cam0_points,
        //     track_inliers, noArray(),
        //     Size(processor_config.patch_size, processor_config.patch_size),
        //     processor_config.pyramid_levels,
        //     TermCriteria(TermCriteria::COUNT+TermCriteria::EPS,
        //         processor_config.max_iteration,
        //         processor_config.track_precision),
        //     cv::OPTFLOW_USE_INITIAL_FLOW);

        // // Mark those tracked points out of the image region
        // // as untracked.
        // for (int i = 0; i < curr_cam0_points.size(); ++i) {
        //     if (track_inliers[i] == 0) continue;
        //     if (curr_cam0_points[i].y < 0 ||
        //         curr_cam0_points[i].y > cam0_curr_img_ptr->image.rows-1 ||
        //         curr_cam0_points[i].x < 0 ||
        //         curr_cam0_points[i].x > cam0_curr_img_ptr->image.cols-1)
        //     track_inliers[i] = 0;
        // }

        // // Collect the tracked points.
        // vector<FeatureIDType> prev_tracked_ids(0);
        // vector<int> prev_tracked_lifetime(0);
        // vector<Point2f> prev_tracked_cam0_points(0);
        // vector<Point2f> prev_tracked_cam1_points(0);
        // vector<Point2f> curr_tracked_cam0_points(0);

        // removeUnmarkedElements(
        //     prev_ids, track_inliers, prev_tracked_ids);
        // removeUnmarkedElements(
        //     prev_lifetime, track_inliers, prev_tracked_lifetime);
        // removeUnmarkedElements(
        //     prev_cam0_points, track_inliers, prev_tracked_cam0_points);
        // removeUnmarkedElements(
        //     prev_cam1_points, track_inliers, prev_tracked_cam1_points);
        // removeUnmarkedElements(
        //     curr_cam0_points, track_inliers, curr_tracked_cam0_points);

        // // Number of features left after tracking.
        // after_tracking = curr_tracked_cam0_points.size();


        // // Outlier removal involves three steps, which forms a close
        // // loop between the previous and current frames of cam0 (left)
        // // and cam1 (right). Assuming the stereo matching between the
        // // previous cam0 and cam1 images are correct, the three steps are:
        // //
        // // prev frames cam0 ----------> cam1
        // //              |                |
        // //              |ransac          |ransac
        // //              |   stereo match |
        // // curr frames cam0 ----------> cam1
        // //
        // // 1) Stereo matching between current images of cam0 and cam1.
        // // 2) RANSAC between previous and current images of cam0.
        // // 3) RANSAC between previous and current images of cam1.
        // //
        // // For Step 3, tracking between the images is no longer needed.
        // // The stereo matching results are directly used in the RANSAC.

        // // Step 1: stereo matching.
        // vector<Point2f> curr_cam1_points(0);
        // vector<unsigned char> match_inliers(0);
        // stereoMatch(curr_tracked_cam0_points, curr_cam1_points, match_inliers);

        // vector<FeatureIDType> prev_matched_ids(0);
        // vector<int> prev_matched_lifetime(0);
        // vector<Point2f> prev_matched_cam0_points(0);
        // vector<Point2f> prev_matched_cam1_points(0);
        // vector<Point2f> curr_matched_cam0_points(0);
        // vector<Point2f> curr_matched_cam1_points(0);

        // removeUnmarkedElements(
        //     prev_tracked_ids, match_inliers, prev_matched_ids);
        // removeUnmarkedElements(
        //     prev_tracked_lifetime, match_inliers, prev_matched_lifetime);
        // removeUnmarkedElements(
        //     prev_tracked_cam0_points, match_inliers, prev_matched_cam0_points);
        // removeUnmarkedElements(
        //     prev_tracked_cam1_points, match_inliers, prev_matched_cam1_points);
        // removeUnmarkedElements(
        //     curr_tracked_cam0_points, match_inliers, curr_matched_cam0_points);
        // removeUnmarkedElements(
        //     curr_cam1_points, match_inliers, curr_matched_cam1_points);

        // // Number of features left after stereo matching.
        // after_matching = curr_matched_cam0_points.size();

        // // Step 2 and 3: RANSAC on temporal image pairs of cam0 and cam1.
        // vector<int> cam0_ransac_inliers(0);
        // twoPointRansac(prev_matched_cam0_points, curr_matched_cam0_points,
        //     cam0_R_p_c, cam0_intrinsics, cam0_distortion_model,
        //     cam0_distortion_coeffs, processor_config.ransac_threshold,
        //     0.99, cam0_ransac_inliers);

        // vector<int> cam1_ransac_inliers(0);
        // twoPointRansac(prev_matched_cam1_points, curr_matched_cam1_points,
        //     cam1_R_p_c, cam1_intrinsics, cam1_distortion_model,
        //     cam1_distortion_coeffs, processor_config.ransac_threshold,
        //     0.99, cam1_ransac_inliers);

        // // Number of features after ransac.
        // after_ransac = 0;

        // for (int i = 0; i < cam0_ransac_inliers.size(); ++i) {
        //     if (cam0_ransac_inliers[i] == 0 ||
        //         cam1_ransac_inliers[i] == 0) continue;
        //     int row = static_cast<int>(
        //         curr_matched_cam0_points[i].y / grid_height);
        //     int col = static_cast<int>(
        //         curr_matched_cam0_points[i].x / grid_width);
        //     int code = row*processor_config.grid_col + col;
        //     (*curr_features_ptr)[code].push_back(FeatureMetaData());

        //     FeatureMetaData& grid_new_feature = (*curr_features_ptr)[code].back();
        //     grid_new_feature.id = prev_matched_ids[i];
        //     grid_new_feature.lifetime = ++prev_matched_lifetime[i];
        //     grid_new_feature.cam0_point = curr_matched_cam0_points[i];
        //     grid_new_feature.cam1_point = curr_matched_cam1_points[i];

        //     ++after_ransac;
        // }

        // // Compute the tracking rate.
        // int prev_feature_num = 0;
        // for (const auto& item : *prev_features_ptr)
        //     prev_feature_num += item.second.size();

        // int curr_feature_num = 0;
        // for (const auto& item : *curr_features_ptr)
        //     curr_feature_num += item.second.size();

        // ROS_INFO_THROTTLE(0.5,
        //     "\033[0;32m candidates: %d; track: %d; match: %d; ransac: %d/%d=%f\033[0m",
        //     before_tracking, after_tracking, after_matching,
        //     curr_feature_num, prev_feature_num,
        //     static_cast<double>(curr_feature_num)/
        //     (static_cast<double>(prev_feature_num)+1e-5));
        // //printf(
        // //    "\033[0;32m candidates: %d; raw track: %d; stereo match: %d; ransac: %d/%d=%f\033[0m\n",
        // //    before_tracking, after_tracking, after_matching,
        // //    curr_feature_num, prev_feature_num,
        // //    static_cast<double>(curr_feature_num)/
        // //    (static_cast<double>(prev_feature_num)+1e-5));

        // return;

    }

    fn add_new_features(&mut self) {
        //   const Mat& curr_img = cam0_curr_img_ptr->image;

        // // Size of each grid.
        // static int grid_height =
        //     cam0_curr_img_ptr->image.rows / processor_config.grid_row;
        // static int grid_width =
        //     cam0_curr_img_ptr->image.cols / processor_config.grid_col;

        // // Create a mask to avoid redetecting existing features.
        // Mat mask(curr_img.rows, curr_img.cols, CV_8U, Scalar(1));

        // for (const auto& features : *curr_features_ptr) {
        //     for (const auto& feature : features.second) {
        //     const int y = static_cast<int>(feature.cam0_point.y);
        //     const int x = static_cast<int>(feature.cam0_point.x);

        //     int up_lim = y-2, bottom_lim = y+3,
        //         left_lim = x-2, right_lim = x+3;
        //     if (up_lim < 0) up_lim = 0;
        //     if (bottom_lim > curr_img.rows) bottom_lim = curr_img.rows;
        //     if (left_lim < 0) left_lim = 0;
        //     if (right_lim > curr_img.cols) right_lim = curr_img.cols;

        //     Range row_range(up_lim, bottom_lim);
        //     Range col_range(left_lim, right_lim);
        //     mask(row_range, col_range) = 0;
        //     }
        // }

        // // Detect new features.
        // vector<KeyPoint> new_features(0);
        // detector_ptr->detect(curr_img, new_features, mask);

        // // Collect the new detected features based on the grid.
        // // Select the ones with top response within each grid afterwards.
        // vector<vector<KeyPoint> > new_feature_sieve(
        //     processor_config.grid_row*processor_config.grid_col);
        // for (const auto& feature : new_features) {
        //     int row = static_cast<int>(feature.pt.y / grid_height);
        //     int col = static_cast<int>(feature.pt.x / grid_width);
        //     new_feature_sieve[
        //     row*processor_config.grid_col+col].push_back(feature);
        // }

        // new_features.clear();
        // for (auto& item : new_feature_sieve) {
        //     if (item.size() > processor_config.grid_max_feature_num) {
        //     std::sort(item.begin(), item.end(),
        //         &ImageProcessor::keyPointCompareByResponse);
        //     item.erase(
        //         item.begin()+processor_config.grid_max_feature_num, item.end());
        //     }
        //     new_features.insert(new_features.end(), item.begin(), item.end());
        // }

        // int detected_new_features = new_features.size();

        // // Find the stereo matched points for the newly
        // // detected features.
        // vector<cv::Point2f> cam0_points(new_features.size());
        // for (int i = 0; i < new_features.size(); ++i)
        //     cam0_points[i] = new_features[i].pt;

        // vector<cv::Point2f> cam1_points(0);
        // vector<unsigned char> inlier_markers(0);
        // stereoMatch(cam0_points, cam1_points, inlier_markers);

        // vector<cv::Point2f> cam0_inliers(0);
        // vector<cv::Point2f> cam1_inliers(0);
        // vector<float> response_inliers(0);
        // for (int i = 0; i < inlier_markers.size(); ++i) {
        //     if (inlier_markers[i] == 0) continue;
        //     cam0_inliers.push_back(cam0_points[i]);
        //     cam1_inliers.push_back(cam1_points[i]);
        //     response_inliers.push_back(new_features[i].response);
        // }

        // int matched_new_features = cam0_inliers.size();

        // if (matched_new_features < 5 &&
        //     static_cast<double>(matched_new_features)/
        //     static_cast<double>(detected_new_features) < 0.1)
        //     ROS_WARN("Images at [%f] seems unsynced...",
        //         cam0_curr_img_ptr->header.stamp.toSec());

        // // Group the features into grids
        // GridFeatures grid_new_features;
        // for (int code = 0; code <
        //     processor_config.grid_row*processor_config.grid_col; ++code)
        //     grid_new_features[code] = vector<FeatureMetaData>(0);

        // for (int i = 0; i < cam0_inliers.size(); ++i) {
        //     const cv::Point2f& cam0_point = cam0_inliers[i];
        //     const cv::Point2f& cam1_point = cam1_inliers[i];
        //     const float& response = response_inliers[i];

        //     int row = static_cast<int>(cam0_point.y / grid_height);
        //     int col = static_cast<int>(cam0_point.x / grid_width);
        //     int code = row*processor_config.grid_col + col;

        //     FeatureMetaData new_feature;
        //     new_feature.response = response;
        //     new_feature.cam0_point = cam0_point;
        //     new_feature.cam1_point = cam1_point;
        //     grid_new_features[code].push_back(new_feature);
        // }

        // // Sort the new features in each grid based on its response.
        // for (auto& item : grid_new_features)
        //     std::sort(item.second.begin(), item.second.end(),
        //         &ImageProcessor::featureCompareByResponse);

        // int new_added_feature_num = 0;
        // // Collect new features within each grid with high response.
        // for (int code = 0; code <
        //     processor_config.grid_row*processor_config.grid_col; ++code) {
        //     vector<FeatureMetaData>& features_this_grid = (*curr_features_ptr)[code];
        //     vector<FeatureMetaData>& new_features_this_grid = grid_new_features[code];

        //     if (features_this_grid.size() >=
        //         processor_config.grid_min_feature_num) continue;

        //     int vacancy_num = processor_config.grid_min_feature_num -
        //     features_this_grid.size();
        //     for (int k = 0;
        //         k < vacancy_num && k < new_features_this_grid.size(); ++k) {
        //     features_this_grid.push_back(new_features_this_grid[k]);
        //     features_this_grid.back().id = next_feature_id++;
        //     features_this_grid.back().lifetime = 1;

        //     ++new_added_feature_num;
        //     }
        // }

    }

    fn prune_grid_features(&mut self) {
        //         for (auto& item : *curr_features_ptr) {
        //     auto& grid_features = item.second;
        //     // Continue if the number of features in this grid does
        //     // not exceed the upper bound.
        //     if (grid_features.size() <=
        //         processor_config.grid_max_feature_num) continue;
        //     std::sort(grid_features.begin(), grid_features.end(),
        //         &ImageProcessor::featureCompareByLifetime);
        //     grid_features.erase(grid_features.begin()+
        //         processor_config.grid_max_feature_num,
        //         grid_features.end());
        // }
        // return;

    }

    // fn extract_features_and_add_to_existing(&mut self) -> Result<(), Box<dyn std::error::Error>> {
    //     let _span = tracy_client::span!("extract features");

    //     let (keypoints, descriptors) = self.extract_features(self.current_frame.image.as_ref().unwrap().clone(), self.curr_frame_id);

    //     let mut new_keypoints = VectorOfKeyPoint::new();
    //     let mut new_descriptor = Mat::default();
    //     let mut new_descriptor_vec = VectorOfMat::new();

    //     for i in 0..self.current_frame.features.num_keypoints {
    //         new_keypoints.push(self.current_frame.features.get_keypoint(i as usize).0);
    //         new_descriptor_vec.push((* self.current_frame.features.descriptors.row(i as u32)).clone());
    //     }

    //     for i in 0..keypoints.len() {
    //         new_keypoints.push(keypoints.get(i as usize)?);
    //         new_descriptor_vec.push((*descriptors).row(i as i32)?);
    //     }

    //     opencv::core::vconcat(&new_descriptor_vec, &mut new_descriptor).expect("Failed to concatenate");

    //     // println!("Re-extracting features, before: {}, after: {}", frame.features.num_keypoints, new_keypoints.len());

    //     self.current_frame.replace_features(DVVectorOfKeyPoint::new(new_keypoints), DVMatrix::new(new_descriptor))?;
    //     Ok(())
    // }

    // fn get_feature_tracks(&mut self) -> Result<VectorOfu8, Box<dyn std::error::Error>> {
    //     let mut status = VectorOfu8::new();

    //     let frame1 = self.last_frame.as_mut().unwrap();
    //     let mut frame2 = &mut self.current_frame;

    //     let mut points1: VectorOfPoint2f = frame1.features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();
    //     let mut points2 = VectorOfPoint2f::new();

    //     //this function automatically gets rid of points for which tracking fails
    //     let mut err = opencv::types::VectorOff32::default();
    //     let win_size = opencv::core::Size::new(24, 24);
    //     let termcrit = opencv::core::TermCriteria {
    //       typ: 3,
    //       max_count: 30,
    //       epsilon: 0.1,
    //     };
    //     let max_level = 3;
    //     opencv::video::calc_optical_flow_pyr_lk(
    //       & frame1.image.as_ref().unwrap(), & frame2.image.as_ref().unwrap(), &mut points1, &mut points2, &mut status, &mut err, win_size, max_level, termcrit, 0, 0.001,
    //     )?;

    //     //getting rid of points for which the KLT tracking failed or those who have gone outside the framed
    //     let mut indez_correction = 0;
    //     let mut new_descriptors = VectorOfMat::new();
    //     let mut new_keypoints = VectorOfKeyPoint::new();

    //     for i in 0..status.len() {
    //       let pt = points2.get(i - indez_correction)?;
    //       if (status.get(i)? == 0) || pt.x < 0.0 || pt.y < 0.0 {
    //         if pt.x < 0.0 || pt.y < 0.0 {
    //           status.set(i, 0)?;
    //         }
    //         points1.remove(i - indez_correction)?;
    //         points2.remove(i - indez_correction)?;
    //         frame1.features.remove_keypoint_and_descriptor(i - indez_correction)?;

    //         indez_correction = indez_correction + 1;
    //         } else {
    //             let curr_kp = frame1.features.get_keypoint(i - indez_correction).0;
    //             new_keypoints.push(opencv::core::KeyPoint::new_coords(pt.x, pt.y, curr_kp.size(), curr_kp.angle(), curr_kp.response(), curr_kp.octave(), curr_kp.class_id()).expect("Failed to create keypoint"));

    //             new_descriptors.push((* frame1.features.descriptors.row(i as u32)).clone());
    //         }
    //     }

    //     let mut new_descs_as_mat = opencv::core::Mat::default();
    //     opencv::core::vconcat(&new_descriptors, &mut new_descs_as_mat).expect("Failed to concatenate");

    //     frame2.features = Features::new(DVVectorOfKeyPoint::new(new_keypoints), DVMatrix::new(new_descs_as_mat), frame2.image.as_ref().unwrap().cols() as u32, frame2.image.as_ref().unwrap().rows() as u32, self.sensor)?;
    //     // frame2.replace_features(DVVectorOfKeyPoint::new(new_keypoints), DVMatrix::new(new_descs_as_mat))?;

    //     Ok(status)
    // }

    // fn calculate_transform(&self) -> Result<Pose, Box<dyn std::error::Error>> {
    //     // recovering the pose and the essential matrix
    //     let prev_features: VectorOfPoint2f = self.last_frame.as_ref().unwrap().features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();
    //     let curr_features: VectorOfPoint2f = self.current_frame.features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();

    //     let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
    //     let essential_mat = opencv::calib3d::find_essential_mat(
    //       &prev_features,
    //       &curr_features,
    //       &CAMERA_MODULE.k_matrix.mat(),
    //       opencv::calib3d::RANSAC,
    //       0.999,
    //       1.0,
    //       1000,
    //       &mut mask,
    //     )?;
    //     opencv::calib3d::recover_pose_estimated(
    //       &essential_mat,
    //       &prev_features,
    //       &curr_features,
    //       &CAMERA_MODULE.k_matrix.mat(),
    //       &mut recover_r,
    //       &mut recover_t,
    //       &mut mask,
    //     )?;

    //     let recover_t = nalgebra::Vector3::<f64>::new(
    //         *recover_t.at_2d::<f64>(0, 0)?,
    //         *recover_t.at_2d::<f64>(1, 0)?,
    //         *recover_t.at_2d::<f64>(2, 0)?
    //     );
    //     let recover_r = nalgebra::Matrix3::<f64>::new(
    //         *recover_r.at_2d::<f64>(0, 0)?, *recover_r.at_2d::<f64>(0, 1)?, *recover_r.at_2d::<f64>(0, 2)?,
    //         *recover_r.at_2d::<f64>(1, 0)?, *recover_r.at_2d::<f64>(1, 1)?, *recover_r.at_2d::<f64>(1, 2)?,
    //         *recover_r.at_2d::<f64>(2, 0)?, *recover_r.at_2d::<f64>(2, 1)?, *recover_r.at_2d::<f64>(2, 2)?
    //     );

    //     Ok(Pose::new(recover_t,recover_r))
    // }

    fn update_trajectory_in_logs(
        &mut self, created_new_kf: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let relative_pose = {
            let reference_pose = if created_new_kf {
                // If we created a new kf this round, the relative keyframe pose is the same as the current frame pose.
                self.current_frame.pose.unwrap()
            } else if self.last_frame.as_ref().unwrap().pose.is_none() {
                // This condition should only happen for the first frame after initialization
                self.map.read()?.get_keyframe(self.ref_kf_id.unwrap()).get_pose().inverse()
            } else {
                self.last_frame.as_ref().unwrap().pose.unwrap()
            };

            self.current_frame.pose.unwrap() * reference_pose.inverse()
        };

        self.trajectory_poses.push(relative_pose);

        info!("Frame {} pose: {:?}", self.current_frame.frame_id, self.current_frame.pose);

        self.system.send(
            SHUTDOWN_ACTOR, 
            Box::new(TrajectoryMsg{
                pose: self.current_frame.pose.unwrap().inverse(),
                ref_kf_id: self.current_frame.ref_kf_id.unwrap(),
                timestamp: self.current_frame.timestamp,
                map_version: self.map.read()?.version
            })
        );

        self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
            pose: self.current_frame.pose.unwrap(),
            mappoint_matches: vec![],
            nontracked_mappoints: HashMap::new(),
            mappoints_in_tracking: self.local_mappoints.clone(),
            timestamp: self.current_frame.timestamp,
            map_version: self.map.read()?.version
        }));

        Ok(())
    }

    fn create_new_keyframe(&mut self, current_frame: &mut Frame) -> Result<(), Box<dyn std::error::Error>>{
        let _span = tracy_client::span!("create_new_keyframe");

        self.imu.as_mut().unwrap().imu_preintegrated_from_last_kf = ImuPreIntegrated::new(ImuBias::new());

        tracy_client::Client::running()
        .expect("message! without a running Client")
        .message("create new keyframe", 2);

        // KeyFrame created here and inserted into map
        self.system.send(
            LOCAL_MAPPING,
            Box::new( NewKeyFrameMsg{
                keyframe: current_frame.clone(),
                tracking_state: self.state,
                matches_in_tracking: self.matches_inliers,
                tracked_mappoint_depths: todo!("Keep track of mappoint depths!"),
                map_version: self.map.read()?.version
            } )
        );

        Ok(())
    }


}
