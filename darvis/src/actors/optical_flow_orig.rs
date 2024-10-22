// extern crate g2o;
// use cxx::UniquePtr;
// use log::{warn, info, debug, error};
// use std::{collections::{BTreeMap, BTreeSet, HashMap, HashSet}, fmt::{self, Debug}, sync::atomic::Ordering, thread::sleep, time::Duration};
// use opencv::{prelude::*, types::VectorOfKeyPoint,types::VectorOfPoint2f, types::VectorOfu8, types::VectorOfMat};

// use core::{
//     config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, System, Timestamp}
// };
// use crate::{
//     actors::{
//         messages::{IMUInitializedMsg, ImageMsg, LastKeyFrameUpdatedMsg, ShutdownMsg, VisFeaturesMsg},
//         tracking_backend::TrackingState,
//     }, map::{features::Features, frame::Frame, map::Id, pose::Pose}, modules::{image, imu::DVImu, map_initialization::DVInitialization, module::FeatureExtractionModule, optimizer, orbextractor::DVORBextractor, orbmatcher, relocalization::DVRelocalization}, registered_actors::{CAMERA, CAMERA_MODULE, FEATURE_DETECTION, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, TRACKING_FRONTEND, VISUALIZER}, ReadWriteMap, MAP_INITIALIZED
// };
// use crate::modules::module::ImuModule;
// use super::{local_mapping::LOCAL_MAPPING_IDLE, messages::{ImagePathMsg, InitKeyFrameMsg, NewKeyFrameMsg, TrackingStateMsg, TrajectoryMsg, VisTrajectoryMsg}, tracking_backend::TrackedMapPointData};
// use crate::modules::module::MapInitializationModule;
// use crate::modules::module::RelocalizationModule;

// pub struct TrackingOpticalFlow {
//     system: System,
//     sensor: Sensor,

//     /// Frontend
//     orb_extractor_left: DVORBextractor,
//     orb_extractor_right: Option<DVORBextractor>,
//     orb_extractor_ini: Option<DVORBextractor>,
//     init_id: Id,

//     /// Backend
//     state: TrackingState,
//     // KeyFrames
//     ref_kf_id: Option<Id>,
//     frames_since_last_kf: i32, // used instead of mnLastKeyFrameId, don't directly copy because the logic is kind of flipped
//     last_kf_timestamp: Option<Timestamp>,

//     // Local data used for different stages of tracking
//     matches_inliers : i32, // mnMatchesInliers ... Current matches in frame
//     local_keyframes: BTreeSet<Id>, //mvpLocalKeyFrames 
//     local_mappoints: BTreeSet<Id>, //mvpLocalMapPoints
//     track_in_view: HashMap::<Id, TrackedMapPointData>, // mbTrackInView , member variable in Mappoint
//     track_in_view_r: HashMap::<Id, TrackedMapPointData>, // mbTrackInViewR, member variable in Mappoint
//     kf_track_reference_for_frame: HashMap::<Id, Id>, // mnTrackReferenceForFrame, member variable in Keyframe
//     mp_track_reference_for_frame: HashMap::<Id, Id>,  // mnTrackReferenceForFrame, member variable in Mappoint
//     last_frame_seen: HashMap::<Id, Id>, // mnLastFrameSeen, member variable in Mappoint
//     // IMU 
//     imu: DVImu,
//     // Relocalization
//     relocalization: DVRelocalization,
//     // Poses in trajectory
//     trajectory_poses: Vec<Pose>, //mlRelativeFramePoses

//     /// References to map
//     map_initialized: bool,
//     map_updated : bool,  // TODO (mvp) I'm not sure we want to use this
//     map: ReadWriteMap,
//     initialization: Option<DVInitialization>, // data sent to map actor to initialize new map

//     /// Global defaults
//     localization_only_mode: bool,
//     frames_to_reset_imu: u32, //mnFramesToResetIMU
//     insert_kfs_when_lost: bool,
//     max_frames_to_insert_kf : i32 , //mMaxFrames , Max Frames to insert keyframes and to check relocalisation
//     min_frames_to_insert_kf: i32, // mMinFrames, Min Frames to insert keyframes and to check relocalisation

//     min_num_features: u32, // Optical flow re-extraction of features if less than this number

//     last_kf_image: Option<opencv::core::Mat>,
//     current_image: Option<opencv::core::Mat>,
// }

// impl Actor for TrackingOpticalFlow {
//     type MapRef = ReadWriteMap;

//     fn new_actorstate(system: System, map: Self::MapRef) -> TrackingOpticalFlow {
//         let max_features = SETTINGS.get::<i32>(FEATURE_DETECTION, "max_features");
//         let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
//         let orb_extractor_right = match sensor.frame() {
//             FrameSensor::Stereo => Some(DVORBextractor::new(max_features)),
//             FrameSensor::Mono | FrameSensor::Rgbd => None,
//         };
//         let orb_extractor_ini = match sensor.is_mono() {
//             true => Some(DVORBextractor::new(max_features*5)),
//             false => None
//         };
//         TrackingOpticalFlow {
//             system,
//             orb_extractor_left: DVORBextractor::new(max_features),
//             orb_extractor_right,
//             orb_extractor_ini,
//             map_initialized: false,
//             init_id: 0,
//             sensor,
//             map,
//             initialization: Some(DVInitialization::new()),
//             localization_only_mode: SETTINGS.get::<bool>(SYSTEM, "localization_only_mode"),
//             frames_to_reset_imu: SETTINGS.get::<i32>(TRACKING_BACKEND, "frames_to_reset_IMU") as u32,
//             insert_kfs_when_lost: SETTINGS.get::<bool>(TRACKING_BACKEND, "insert_KFs_when_lost"),
//             max_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "max_frames_to_insert_kf"),
//             min_frames_to_insert_kf: SETTINGS.get::<i32>(TRACKING_BACKEND, "min_frames_to_insert_kf"),
//             state: TrackingState::NotInitialized,
//             ref_kf_id: None,
//             frames_since_last_kf: 0,
//             last_kf_timestamp: None,
//             matches_inliers: 0,
//             local_keyframes: BTreeSet::new(),
//             local_mappoints: BTreeSet::new(),
//             track_in_view: HashMap::new(),
//             track_in_view_r: HashMap::new(),
//             kf_track_reference_for_frame: HashMap::new(),
//             mp_track_reference_for_frame: HashMap::new(),
//             last_frame_seen: HashMap::new(),
//             imu: DVImu::new(None, None, sensor, false, false),
//             relocalization: DVRelocalization{last_reloc_frame_id: 0, timestamp_lost: None},
//             map_updated: false,
//             trajectory_poses: Vec::new(),
//             min_num_features: SETTINGS.get::<i32>(TRACKING_BACKEND, "min_num_features")  as u32,
//             last_kf_image: None,
//             current_image: None,
//         }
//     }

//     fn spawn(system: System, map: Self::MapRef) {
//         let mut actor = TrackingOpticalFlow::new_actorstate(system, map);
//         let max_queue_size = actor.system.receiver_bound.unwrap_or(100);
//         let mut last_frame: Option<Frame> = None; // Keep last_frame here instead of in tracking object to avoid putting it in an option and doing a ton of unwraps

//         let mut last_img:Option<opencv::core::Mat> = None;

//         let mut last_kps: Option<VectorOfKeyPoint> = None;
//         let mut curr_frame_id = 0;
//         let mut last_pose = Pose::new(nalgebra::Vector3::new(0.0, 0.0, 0.0), nalgebra::Matrix3::identity());
//         let mut last_kf_id = 0;

//         tracy_client::set_thread_name!("tracking full");

//         'outer: loop {
//             let message = actor.system.receive().unwrap();

//             if message.is::<ImagePathMsg>() || message.is::<ImageMsg>(){
//                 if actor.system.queue_len() > max_queue_size {
//                     // Abort additional work if there are too many frames in the msg queue.
//                     info!("Tracking optical flow dropped 1 frame");
//                     continue;
//                 }

//                 let (image, image_cols, image_rows, timestamp) = if message.is::<ImagePathMsg>() {
//                     let msg = message.downcast::<ImagePathMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

//                     let image = image::read_image_file(&msg.image_path);
//                     let image_cols = image.cols() as u32;
//                     let image_rows = image.rows() as u32;

//                     (image, image_cols, image_rows, msg.timestamp)
//                 }
//                 else {
//                     let msg = message.downcast::<ImageMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));

//                     let image = msg.image.clone();
//                     let image_cols = msg.image.cols() as u32;
//                     let image_rows = msg.image.rows() as u32;

//                     (image, image_cols, image_rows, msg.timestamp)
//                 };

//                 let mut current_frame = if last_img.is_none(){
//                     // If this is the first image, just extract features
//                     let (keypoints, descriptors) = actor.extract_features(image.clone(), curr_frame_id);

//                     let mut current_frame = Frame::new(
//                         curr_frame_id, 
//                         DVVectorOfKeyPoint::new(keypoints.clone()),
//                         DVMatrix::new(descriptors),
//                         image_cols,
//                         image_rows,
//                         timestamp
//                     ).expect("Could not create frame!");

//                     info!("First frame keypoints extracted {}", keypoints.len());
//                     last_kps = Some(keypoints);

//                     current_frame
//                 } else {
//                     // If this is not the first image, track features from the last image

//                     let (mut last_keypoints, mut last_desc) = {
//                         let mut last_kps = last_kps.clone();
//                         let mut last_descriptor = last_frame.as_ref().unwrap().features.descriptors.clone();

//                         (last_kps.unwrap(), last_descriptor)
//                     };

//                     let last_kf_img = last_img.clone().unwrap();
 
//                     let mut prev_kps = last_keypoints.clone();
//                     let mut curr_kps = VectorOfKeyPoint::new();
//                     let mut prev_descriptors : Option<Mat>= Some(last_desc.mat().clone());
//                     let mut new_descriptor = Some(opencv::core::Mat::default());
//                     let mut status = VectorOfu8::new();

//                     // debug!("Before flow tracking keypoints  {} , descriptor {}", prev_kps.len(), prev_descriptors.as_ref().unwrap().rows());

//                     let prev_kps_copy = prev_kps.clone();
//                     let prev_descriptors_copy = prev_descriptors.clone();

//                     actor.feature_tracking_kps(&last_kf_img, &image, &mut prev_kps, &mut curr_kps, &mut prev_descriptors, &mut new_descriptor, &mut status);

//                     // debug!("Flow tracking keypoints extracted {} from original {}, desc rows {}", curr_kps.len(), status.len(), new_descriptor.as_ref().unwrap().rows());

//                     let mut points1 = prev_kps.clone().into_iter().map(|kp| kp.pt()).collect();
//                     let mut points2 = curr_kps.clone().into_iter().map(|kp| kp.pt()).collect();

//                     let pose =  actor.calculate_transform(&points2, &points1);

//                     last_pose = pose.clone();

//                     println!("SOFIYA Pose: {:?}", pose);

//                     // check if we need to re-extract features
//                     if prev_kps.len() < actor.min_num_features as usize 
//                     {
//                         info!("Re-extracting features");
//                         // let (mut keypoints_prev, mut descriptors_prev) = actor.extract_features(last_kf_img.clone(), curr_frame_id-1);

//                         let (mut keypoints_prev, mut descriptors_prev) = actor.extract_features_and_add_to_existing(
//                             last_kf_img.clone(), 
//                             curr_frame_id-1,
//                              &prev_kps_copy, 
//                              &prev_descriptors_copy.as_ref().unwrap()
//                         );

//                         println!("Last frame now has {} features", keypoints_prev.len());


//                         curr_kps = VectorOfKeyPoint::new();
//                         status = VectorOfu8::new();

//                         let mut desc_prev = Some(descriptors_prev.clone());
//                         new_descriptor = Some(opencv::core::Mat::default());

//                         actor.feature_tracking_kps(&last_kf_img, &image, &mut keypoints_prev, &mut curr_kps, &mut desc_prev, &mut new_descriptor, &mut status);

//                         debug!("Flow tracking keypoints extracted {} from original {}, desc rows {}", curr_kps.len(), status.len(), new_descriptor.as_ref().unwrap().rows());
//                     }

//                     let mut current_frame = Frame::new(
//                         curr_frame_id, 
//                         DVVectorOfKeyPoint::new(curr_kps.clone()),
//                         DVMatrix::new(new_descriptor.unwrap()),
//                         image_cols,
//                         image_rows,
//                         timestamp
//                     ).expect("Could not create frame!");

//                     last_kps = Some(curr_kps.clone());
//                     current_frame.pose = Some(pose);
//                     current_frame
//                 };
//                 if !last_frame.is_none() {
//                     info!("Tracking between frames {} and {}", last_frame.as_ref().unwrap().frame_id, current_frame.frame_id);
//                 }

//                 actor.current_image = Some(image.clone());
//                 match actor.track(&mut current_frame, &mut last_frame) {
//                     Ok(created_kf) => {
//                         if current_frame.ref_kf_id.is_none() {
//                             current_frame.ref_kf_id = actor.ref_kf_id;
//                         }
//                         info!("Tracking successful with keyframe id {:?} actor kf id {:?}, current frame id {:?}", current_frame.ref_kf_id, actor.ref_kf_id, current_frame.frame_id);

//                         if last_frame.is_some() && last_frame.as_ref().unwrap().ref_kf_id.is_some()
//                         {
//                             let relative_pose = {
//                                 let map_read = actor.map.read().unwrap();
//                                 let mut last_frame_map = map_read.keyframes.get(&last_kf_id).unwrap().pose.clone();
//                                 last_frame_map
//                             };

//                             // last_pose= last_pose * relative_pose;
//                             current_frame.pose = Some(current_frame.pose.unwrap() * relative_pose);

//                             warn!("Current frame id: {}, ref_kf_id: {:?}", curr_frame_id, current_frame.ref_kf_id);
//                             warn!("Current Frame Pose: {:?}", current_frame.pose);
//                             // last_pose = current_frame.pose.unwrap();

//                             if last_kf_id != actor.ref_kf_id.unwrap() {
//                                 // last_pose = Pose::new(nalgebra::Vector3::new(0.0, 0.0, 0.0), nalgebra::Matrix3::identity());

//                                 // last_pose = actor.map.read().unwrap().keyframes.get(&last_kf_id).unwrap().pose.clone();
//                                 last_kf_id = actor.ref_kf_id.unwrap();
//                             }
//                         }

//                         last_frame = Some(current_frame);
//                         warn!("Last frame id is {:?}, kf id is {:?}", last_frame.as_ref().unwrap().frame_id, last_frame.as_ref().unwrap().ref_kf_id);
//                         curr_frame_id += 1;

//                         match actor.state {
//                             TrackingState::Ok | TrackingState::RecentlyLost => {
//                                 actor.update_trajectory_in_logs(last_frame.as_mut().unwrap(), created_kf).expect("Could not save trajectory")
//                             },
//                             _ => {},
//                         };
//                     },
//                     Err(e) => {
//                         panic!("Error in Tracking Backend: {}", e);
//                     }
//                 };

//                 last_img = Some(image.clone());


//             } else if message.is::<InitKeyFrameMsg>() {
//                 // Received from the map actor after it inserts a keyframe
//                 let msg = message.downcast::<InitKeyFrameMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking frontend message!"));
//                 last_frame.as_mut().unwrap().ref_kf_id = Some(msg.kf_id);
//                 info!("Received ref kf id from map actor: {:?}", msg.kf_id);
//             } else if message.is::<LastKeyFrameUpdatedMsg>() {
//                 // Received from local mapping after it culls and creates new MPs for the last inserted KF
//                 actor.state = TrackingState::Ok;
//             } else if message.is::<IMUInitializedMsg>() {
//                 // TODO (IMU) process message from local mapping!
//             } else if message.is::<ShutdownMsg>() {
//                 break 'outer;
//             } else {
//                 warn!("Tracking frontend received unknown message type!");
//             }
//         }
//     }
// }

// impl TrackingOpticalFlow {
//     fn extract_features(&mut self, image: opencv::core::Mat, curr_frame_id: i32) -> (VectorOfKeyPoint, Mat) {
//         let _span = tracy_client::span!("extract features");

//         let image_dv: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::new(image)).into();
//         let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
//         let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();

//         // TODO (C++ and Rust optimizations) ... this takes ~70 ms which is way high compared to ORB-SLAM3. I think this is because the rust and C++ bindings are not getting optimized together.
//         if self.map_initialized && (curr_frame_id - self.init_id > self.max_frames_to_insert_kf) {
//             self.orb_extractor_left.extract(&image_dv);
//         } else 
//         if self.sensor.is_mono() {

//             self.orb_extractor_ini.as_mut().unwrap().extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
//         } else {
//             self.orb_extractor_left.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
//         }
//         match self.sensor.frame() {
//             FrameSensor::Stereo => todo!("Stereo"), //Also call extractor_right, see Tracking::GrabImageStereo,
//             _ => {}
//         }
//         (keypoints.kp_ptr.kp_ptr, descriptors.mat_ptr.mat_ptr)
//     }


//     fn extract_features_and_add_to_existing(&mut self, image: opencv::core::Mat, curr_frame_id: i32, keypoints_old: &VectorOfKeyPoint, descriptors_old: &Mat) -> (VectorOfKeyPoint, Mat) {
//         let _span = tracy_client::span!("extract features");

//         let (keypoints, descriptors) = 
//         {
//             let image_dv: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::new(image)).into();
//             let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
//             let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();

//             // TODO (C++ and Rust optimizations) ... this takes ~70 ms which is way high compared to ORB-SLAM3. I think this is because the rust and C++ bindings are not getting optimized together.
//             if self.map_initialized && (curr_frame_id - self.init_id > self.max_frames_to_insert_kf) {
//                 self.orb_extractor_left.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
//             } else 
//             if self.sensor.is_mono() {

//                 self.orb_extractor_ini.as_mut().unwrap().extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
//             } else {
//                 self.orb_extractor_left.extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);
//             }
//             match self.sensor.frame() {
//                 FrameSensor::Stereo => todo!("Stereo"), //Also call extractor_right, see Tracking::GrabImageStereo,
//                 _ => {}
//             }
//             (keypoints.kp_ptr.kp_ptr, descriptors.mat_ptr.mat_ptr)
//         };

//         let mut new_keypoints = VectorOfKeyPoint::new();
//         let mut new_descriptor = Mat::default();
//         let mut new_descriptor_vec = VectorOfMat::new();

//         // for i in 0..keypoints.len() {

//         //     new_keypoints.push(keypoints.get(i).unwrap());
//         //     new_descriptor_vec.push(descriptors.row(i as i32).unwrap());
//         //     let mut found = false;
//         //     for j in 0..keypoints_old.len() {
//         //         if (keypoints.get(i).unwrap().pt().x - keypoints_old.get(j).unwrap().pt().x).abs() < 0.1 && (keypoints.get(i).unwrap().pt().y - keypoints_old.get(j).unwrap().pt().y).abs() < 0.1 {
//         //             found = true;
//         //             break;
//         //         }
//         //     }
//         //     if !found {
//         //         new_keypoints.push(keypoints.get(i).unwrap());
//         //         new_descriptor_vec.push(descriptors.row(i as ila.t32).unwrap());
//         //     }
//         // }

//         for i in 0..keypoints_old.len() {
//             new_keypoints.push(keypoints_old.get(i).unwrap());
//             new_descriptor_vec.push(descriptors_old.row(i as i32).unwrap());
//         }

//         for i in 0..keypoints.len() {

//             new_keypoints.push(keypoints.get(i).unwrap());
//             new_descriptor_vec.push(descriptors.row(i as i32).unwrap());
//         }

        



//         opencv::core::vconcat(&new_descriptor_vec, &mut new_descriptor).expect("Failed to concatenate");

//         (new_keypoints, new_descriptor)


//     }

//     pub fn feature_tracking_kps(
//         &self,
//         img_1: &opencv::core::Mat,
//         img_2: &opencv::core::Mat,
//         kps1: &mut VectorOfKeyPoint,
//         kps2: &mut VectorOfKeyPoint,
//         desc1: &mut Option<Mat>,
//         desc2: &mut Option<Mat>,
//         status: &mut VectorOfu8,
//       ) {

//         let mut points1: VectorOfPoint2f = kps1.clone().into_iter().map(|kp| kp.pt()).collect();
//         let mut points2 = VectorOfPoint2f::new();

//         //this function automatically gets rid of points for which tracking fails
//         let mut err = opencv::types::VectorOff32::default();
//         let win_size = opencv::core::Size::new(24, 24);
//         let termcrit = opencv::core::TermCriteria {
//           typ: 3,
//           max_count: 30,
//           epsilon: 0.1,
//         };
//         let max_level = 3;
//         opencv::video::calc_optical_flow_pyr_lk(
//           img_1, img_2, &mut points1, &mut points2, status, &mut err, win_size, max_level, termcrit, 0, 0.001,
//         )
//         .unwrap();


//         //getting rid of points for which the KLT tracking failed or those who have gone outside the framed
//         let mut indez_correction = 0;

//         let mut desc1_new = desc1.clone().unwrap();

//         let mut new_desc2_vec = VectorOfMat::new();

//         for i in 0..status.len() {
//           let pt = points2.get(i - indez_correction).unwrap();
//           if (status.get(i).unwrap() == 0) || pt.x < 0.0 || pt.y < 0.0 {
//             if pt.x < 0.0 || pt.y < 0.0 {
//               status.set(i, 0).unwrap();
//             }
//             points1.remove(i - indez_correction).unwrap();
//             points2.remove(i - indez_correction).unwrap();

//             kps1.remove(i - indez_correction).unwrap();

            

//             indez_correction = indez_correction + 1;
//           }
//           else
//             {
//                 let curr_kp = kps1.get(i - indez_correction).unwrap();
//                 kps2.push(opencv::core::KeyPoint::new_coords(pt.x, pt.y, curr_kp.size(), curr_kp.angle(), curr_kp.response(), curr_kp.octave(), curr_kp.class_id()).unwrap());

//                 // desc1_new.row(i as i32).unwrap().copy_to(&mut desc2_new.row(j as i32).unwrap()).unwrap();
//                 // j += 1;

//                 new_desc2_vec.push(desc1_new.row(i as i32).unwrap());
                    

//             }
//         }

//         let mut new_descriptor = opencv::core::Mat::default();

//         opencv::core::vconcat(&new_desc2_vec, &mut new_descriptor).expect("Failed to concatenate");

//         *desc2 = Some(new_descriptor);


//       }


//       pub fn feature_tracking(
//         &self,
//         img_1: &opencv::core::Mat,
//         img_2: &opencv::core::Mat,
//         points1: &mut VectorOfPoint2f,
//         points2: &mut VectorOfPoint2f,
//         status: &mut VectorOfu8,
//       ) {
//         //this function automatically gets rid of points for which tracking fails
//         let mut err = opencv::types::VectorOff32::default();
//         let win_size = opencv::core::Size::new(21, 21);
//         let termcrit = opencv::core::TermCriteria {
//           typ: 3,
//           max_count: 30,
//           epsilon: 0.01,
//         };
//         let max_level = 3;
//         opencv::video::calc_optical_flow_pyr_lk(
//           img_1, img_2, points1, points2, status, &mut err, win_size, max_level, termcrit, 0, 0.001,
//         )
//         .unwrap();
//         //getting rid of points for which the KLT tracking failed or those who have gone outside the framed
//         let mut indez_correction = 0;
//         for i in 0..status.len() {
//           let pt = points2.get(i - indez_correction).unwrap();
//           if (status.get(i).unwrap() == 0) || pt.x < 0.0 || pt.y < 0.0 {
//             if pt.x < 0.0 || pt.y < 0.0 {
//               status.set(i, 0).unwrap();
//             }
//             points1.remove(i - indez_correction).unwrap();
//             points2.remove(i - indez_correction).unwrap();
//             indez_correction = indez_correction + 1;
//           }
//         }
//       }



//       pub fn calculate_transform(
//         &self,
//         curr_features: &VectorOfPoint2f,
//         prev_features: &VectorOfPoint2f,
//       ) -> Pose {
//         //recovering the pose and the essential matrix
//         let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
//         let essential_mat = opencv::calib3d::find_essential_mat(
//           &curr_features,
//           &prev_features,
//           &CAMERA_MODULE.k_matrix.mat(),
//           opencv::calib3d::RANSAC,
//           0.999,
//           1.0,
//           1000,
//           &mut mask,
//         )
//         .unwrap();
//         opencv::calib3d::recover_pose_estimated(
//           &essential_mat,
//           &curr_features,
//           &prev_features,
//           &CAMERA_MODULE.k_matrix.mat(),
//           &mut recover_r,
//           &mut recover_t,
//           &mut mask,
//         )
//         .unwrap();
    



//         let recover_t = nalgebra::Vector3::<f64>::new(
//             *recover_t.at_2d::<f64>(0, 0).unwrap(),
//             *recover_t.at_2d::<f64>(1, 0).unwrap(),
//             *recover_t.at_2d::<f64>(2, 0).unwrap());
//         let recover_r = nalgebra::Matrix3::<f64>::new(
//         *recover_r.at_2d::<f64>(0, 0).unwrap(), *recover_r.at_2d::<f64>(0, 1).unwrap(), *recover_r.at_2d::<f64>(0, 2).unwrap(),
//         *recover_r.at_2d::<f64>(1, 0).unwrap(), *recover_r.at_2d::<f64>(1, 1).unwrap(), *recover_r.at_2d::<f64>(1, 2).unwrap(),
//         *recover_r.at_2d::<f64>(2, 0).unwrap(), *recover_r.at_2d::<f64>(2, 1).unwrap(), *recover_r.at_2d::<f64>(2, 2).unwrap()
//             );


//         Pose::new(recover_t,recover_r)


//       }



//  fn track(&mut self, current_frame: &mut Frame, last_frame: &mut Option<Frame>) -> Result<bool, Box<dyn std::error::Error>>  {
//         let _span = tracy_client::span!("track");

//         // TODO (reset): Reset map because local mapper set the bad imu flag
//         // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1808

//         // TODO (multimaps): Create new map if timestamp older than previous frame arrives
//         // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1820

//         // TODO (reset) TODO (multimaps): Timestamp jump detected, either reset active map or create new map
//         // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1828
//         // (entire block)

//         if self.sensor.is_imu() {
//             self.imu.preintegrate();
//             todo!("IMU");
//             // set bias of new frame = to bias of last
//             // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1860
//         }

//         // Initial estimation of camera pose and matching
//         match (self.localization_only_mode, self.state) {
//             (true, _) => {
//                 todo!("Localization only");
//                 // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L1933
//                 // Look for "mbOnlyTracking" in Track() function
//             },
//             (false, TrackingState::NotInitialized) => {
//                 if self.initialization.is_none() {
//                     self.initialization = Some(DVInitialization::new());
//                 }
//                 let init_success = self.initialization.as_mut().unwrap().try_initialize(&current_frame)?;
//                 if init_success {

//                     let (ini_kf_id, curr_kf_id);
//                     {
//                         // TODO (stereo) - Add option to make the map monocular or stereo
//                         (ini_kf_id, curr_kf_id) = match self.initialization.as_mut().unwrap().create_initial_map_monocular(&mut self.map) {
//                                 Some((curr_kf_pose, curr_kf_id, ini_kf_id, local_mappoints, curr_kf_timestamp)) => {
//                                     // Map needs to be initialized before tracking can begin. Received from map actor
//                                     self.frames_since_last_kf = 0;
//                                     self.local_keyframes.insert(curr_kf_id);
//                                     self.local_keyframes.insert(ini_kf_id);
//                                     self.local_mappoints = local_mappoints;
//                                     self.ref_kf_id = Some(curr_kf_id);
//                                     self.last_kf_timestamp = Some(curr_kf_timestamp);

//                                     {
//                                         // Set current frame's updated info from map initialization
//                                         current_frame.ref_kf_id = Some(curr_kf_id);
//                                         current_frame.pose = Some(curr_kf_pose);
//                                         current_frame.mappoint_matches = self.map.read().unwrap().keyframes.get(&curr_kf_id).unwrap().clone_matches();
//                                     }
//                                     self.state = TrackingState::Ok;

//                                     // Log initial pose in shutdown actor
//                                     self.system.send(SHUTDOWN_ACTOR, 
//                                     Box::new(TrajectoryMsg{
//                                             pose: self.map.read().unwrap().keyframes.get(&ini_kf_id).unwrap().pose,
//                                             ref_kf_id: ini_kf_id,
//                                             timestamp: self.map.read().unwrap().keyframes.get(&ini_kf_id).unwrap().timestamp
//                                         })
//                                     );

//                                     (ini_kf_id, curr_kf_id)
//                                 },
//                                 None => {
//                                     panic!("Could not create initial map");
//                                 }
//                             };
//                     }

//                     MAP_INITIALIZED.store(true, Ordering::SeqCst);

//                     // Send first two keyframes to local mapping
//                     self.system.send(LOCAL_MAPPING, Box::new(
//                         InitKeyFrameMsg { kf_id: ini_kf_id }
//                     ));
//                     self.system.send(LOCAL_MAPPING,Box::new(
//                         InitKeyFrameMsg { kf_id: curr_kf_id }
//                     ));


//                     self.state = TrackingState::Ok;
//                     sleep(Duration::from_millis(50)); // Sleep just a little to allow local mapping to process first keyframe

//                     return Ok(false);
//                 } else {
//                     self.state = TrackingState::NotInitialized;
//                     return Ok(false);
//                 }
//             },
//             (false, TrackingState::Ok) => {
//                 let no_motion_model = (self.imu.velocity.is_none() && !self.imu.is_initialized) || self.relocalization.frames_since_lost(&current_frame) < 2;

//                 let track_success = match no_motion_model {
//                     true => self.track_reference_keyframe(current_frame, last_frame.as_mut().unwrap())?,
//                     false => {
//                         match self.track_with_motion_model(current_frame, last_frame.as_mut().unwrap())? {
//                             true => true,
//                             false => self.track_reference_keyframe(current_frame, last_frame.as_mut().unwrap())?
//                         }
//                     }
//                 };
//                 self.state = match track_success {
//                     true => TrackingState::Ok,
//                     false => {
//                         self.relocalization.timestamp_lost = Some(current_frame.timestamp);
//                         match self.map.read().unwrap().num_keyframes() > 10 {
//                             true =>{warn!("State recently lost!"); TrackingState::RecentlyLost},
//                             false => TrackingState::Lost
//                         }
//                     },
//                 }
//             },
//             (false, TrackingState::RecentlyLost) => {
//                 let relocalize_success = match self.imu.ready() {
//                     true => {
//                         let _ok = self.imu.predict_state(); // TODO (IMU): I guess this should be used somewhere?
//                         self.relocalization.past_cutoff(&current_frame)
//                     },
//                     false => {
//                         // TODO (relocalization): remove the call to shutdown actor and uncomment line of code below.
//                         // This is just to gracefully shut down instead of panicking at the to do 
//                         error!("Relocalization! Shutting down for now.");
//                         self.system.send(SHUTDOWN_ACTOR, Box::new(ShutdownMsg{}));
//                         self.state = TrackingState::Lost;
//                         return Ok(false);
//                         // !self.relocalization.run() && self.relocalization.sec_since_lost(&current_frame) > 3
//                     }
//                 };
//                 self.state = match relocalize_success {
//                     true => {
//                         warn!("Relocalization unsuccessful...");
//                         TrackingState::Lost
//                     },
//                     false => {
//                         warn!("Setting state to recently lost!");
//                         TrackingState::RecentlyLost
//                     }
//                 }
//             },
//             (false, TrackingState::Lost) => {
//                 match self.map.read().unwrap().num_keyframes() < 10 {
//                     true => {
//                         warn!("Reseting current map...");
//                         error!("Resetting current map! Shutting down for now.");
//                         self.system.send(SHUTDOWN_ACTOR, Box::new(ShutdownMsg{}));
//                         self.state = TrackingState::Lost;
//                         return Ok(false);

//                         // TODO (RESET)
//                     },
//                     false => {
//                         info!("Creating new map...");
//                         self.create_new_map();
//                     }
//                 }

//                 self.state = TrackingState::Ok;
//                 return Ok(false);
//             }
//         };


//         // Only 3 valid states now
//         // TrackingState::Ok | TrackingState::RecentlyLost | TrackingState::Lost

//         // Track Local Map
//         let (enough_matches, _matches_in_frame) = self.track_local_map(current_frame, last_frame.as_mut().unwrap());
//         if enough_matches {
//             self.state = TrackingState::Ok;
//         } else if matches!(self.state, TrackingState::Ok) {
//             if self.sensor.is_imu() {
//                 warn!("Track lost for less than 1 second,");
//                 warn!("IMU, Reset... Reset map because local mapper set the bad imu flag");
//                 // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2149
//             }
//             self.relocalization.timestamp_lost = Some(current_frame.timestamp);
//             self.state = TrackingState::RecentlyLost;
//             warn!("Setting state to recently lost!");
//         }

//         if self.sensor.is_imu() {
//             todo!("IMU");
//             // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L2167
//         }

//         let mut created_new_kf = false;
//         if enough_matches || matches!(self.state, TrackingState::RecentlyLost) {
//             // Update motion model
//             let last_frame = last_frame.as_ref();
//             if !last_frame.is_none() && !last_frame.unwrap().pose.is_none() && !current_frame.pose.is_none() {
//                 let last_pose = last_frame.expect("No last frame in tracking?").pose.as_ref().expect("Can't get last frame's pose?");
//                 let last_twc = Pose::new(*last_frame.unwrap().get_camera_center().unwrap(), last_pose.get_rotation().try_inverse().unwrap());
//                 self.imu.velocity = Some(*current_frame.pose.as_ref().expect("Can't get current frame?") * last_twc);
//             } else {
//                 self.imu.velocity = None;
//             }

//             // Clean VO matches
//             current_frame.delete_mappoints_without_observations(&self.map.read().unwrap());

//             // Check if we need to insert a new keyframe
//             let insert_if_lost_anyway = self.insert_kfs_when_lost && matches!(self.state, TrackingState::RecentlyLost) && self.sensor.is_imu();
//             let need_new_kf = self.need_new_keyframe(current_frame);
//             if need_new_kf && (matches!(self.state, TrackingState::Ok) || insert_if_lost_anyway) {
//                 self.create_new_keyframe(current_frame);
//                 created_new_kf = true;
//             }

//             // We allow points with high innovation (considererd outliers by the Huber Function)
//             // pass to the new keyframe, so that bundle adjustment will finally decide
//             // if they are outliers or not. We don't want next frame to estimate its position
//             // with those points so we discard them in the frame. Only has effect if lastframe is tracked
//             let _ = current_frame.delete_mappoint_outliers();
//         }

//         // Reset if the camera get lost soon after initialization
//         if matches!(self.state, TrackingState::Lost) {
//             if self.map.read().unwrap().num_keyframes() <= 10  || (self.sensor.is_imu() && !self.imu.is_initialized) {
//                 warn!("tracking_backend::handle_message;Track lost soon after initialization, resetting...",);
//                 // TODO (RESET)
//             } else {
//                 self.create_new_map();
//             }
//         }

//         Ok(created_new_kf)
//     }

//     fn update_trajectory_in_logs(
//         &mut self, current_frame: &Frame, created_new_kf: bool
//     ) -> Result<(), Box<dyn std::error::Error>> {
//         let relative_pose = {
//             let ref_kf_pose = if created_new_kf {
//                 // If we created a new kf this round, the relative keyframe pose is the same as the current frame pose.
//                 // We have to do this because ORBSLAM3 creates a new keyframe in tracking whereas we wait until beginning
//                 // of local mapping. This code is fine, but we might want to make an atomic int in the map containing the
//                 // latest keyframe inserted in local mapping, rather than sending that as a message from LM to T (current
//                 // implementation). The way we have it now, we are always one reference keyframe behind because local mapping
//                 // will insert the kf in the middle of the tracking thread loop, and then tracking will only know about it in
//                 // the next iteration.
//                 current_frame.pose.unwrap()
//             } else {
//                 let map = self.map.read().unwrap();
//                 let ref_kf = map.keyframes.get(&self.ref_kf_id.unwrap()).expect("Can't get ref kf from map");
//                 ref_kf.pose
//             };

//             current_frame.pose.unwrap() * ref_kf_pose.inverse()
//         };
//         self.trajectory_poses.push(relative_pose);

//         info!("Frame {} pose: {:?}", current_frame.frame_id, current_frame.pose.unwrap());

//         self.system.send(
//             SHUTDOWN_ACTOR, 
//             Box::new(TrajectoryMsg{
//                 pose: current_frame.pose.unwrap().inverse(),
//                 ref_kf_id: current_frame.ref_kf_id.unwrap(),
//                 timestamp: current_frame.timestamp
//             })
//         );

//         self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
//             pose: current_frame.pose.unwrap(),
//             mappoint_matches: current_frame.mappoint_matches.matches.clone(),
//             nontracked_mappoints: HashMap::new(),
//             mappoints_in_tracking: self.local_mappoints.clone(),
//             timestamp: current_frame.timestamp
//         }));

//         Ok(())
//     }

//     fn track_reference_keyframe(&mut self, current_frame: &mut Frame, last_frame: &mut Frame) -> Result<bool, Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("track_reference_keyframe");
//         // Tracking::TrackReferenceKeyFrame()
//         // We perform first an ORB matching with the reference keyframe
//         // If enough matches are found we setup a PnP solver

//         current_frame.compute_bow();
//         let nmatches;
//         {
//             let map_read_lock = self.map.read().unwrap();
//             self.ref_kf_id = Some(map_read_lock.last_kf_id);
//             let ref_kf = map_read_lock.keyframes.get(&self.ref_kf_id.unwrap()).unwrap();

//             nmatches = orbmatcher::search_by_bow_f(ref_kf, current_frame, true, 0.7)?;
//             debug!("Tracking search by bow: {} matches / {} matches in keyframe {}. ({} total mappoints in map)", nmatches, ref_kf.get_mp_matches().iter().filter(|item| item.is_some()).count(), ref_kf.id, map_read_lock.mappoints.len());

//         }

//         if nmatches < 15 {
//             warn!("track_reference_keyframe has fewer than 15 matches = {}!!\n", nmatches);
//             return Ok(false);
//         }

//         current_frame.pose = Some(last_frame.pose.unwrap());

//         optimizer::optimize_pose(current_frame, &self.map);

//         // Discard outliers
//         let nmatches_map = self.discard_outliers(current_frame);

//         match self.sensor.is_imu() {
//             true => { return Ok(true); },
//             false => { return Ok(nmatches_map >= 10); }
//         };
//     }

//     fn track_with_motion_model(&mut self, current_frame: &mut Frame, last_frame: &mut Frame) -> Result<bool, Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("track_with_motion_model");
//         // Tracking::TrackWithMotionModel()
//         // If tracking was successful for last frame, we use a constant
//         // velocity motion model to predict the camera pose and perform
//         // a guided search of the map points observed in the last frame. If
//         // not enough matches were found (i.e. motion model is clearly
//         // violated), we use a wider search of the map points around
//         // their position in the last frame. The pose is then optimized
//         // with the found correspondences.

//         // Update last frame pose according to its reference keyframe
//         // Create "visual odometry" points if in Localization Mode
//         self.update_last_frame(last_frame);

//         let enough_frames_to_reset_imu = current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32);
//         if self.imu.is_initialized && enough_frames_to_reset_imu {
//             // Predict state with IMU if it is initialized and it doesnt need reset
//             todo!("IMU");
//             // self.imu.predict_state();
//         } else {
//             current_frame.pose = Some(self.imu.velocity.unwrap() * last_frame.pose.unwrap());
//         }

//         current_frame.mappoint_matches.clear();

//         // Project points seen in previous frame
//         let th = match self.sensor.frame() {
//             FrameSensor::Mono => 15,
//             _ => 7
//         };

//         let mut matches = orbmatcher::search_by_projection_with_threshold(
//             current_frame,
//             last_frame,
//             th,
//             true,
//             &self.map,
//             self.sensor
//         )?;
//         debug!("Tracking search by projection with previous frame: {} matches / {} mappoints in last frame. ({} total mappoints in map)", matches, last_frame.mappoint_matches.debug_count, self.map.read().unwrap().mappoints.len());

//         // If few matches, uses a wider window search
//         if matches < 20 {
//             info!("tracking_backend::track_with_motion_model;not enough matches, wider window search");
//             current_frame.mappoint_matches.clear();
//             matches = orbmatcher::search_by_projection_with_threshold(
//                 current_frame,
//                 last_frame,
//                 2 * th,
//                 self.sensor.is_mono(),
//                 &self.map,
//                 self.sensor
//             )?;
//             debug!("Tracking search by projection with previous frame: {} matches / {} mappoints in last frame. ({} total mappoints in map)", matches, last_frame.mappoint_matches.debug_count, self.map.read().unwrap().mappoints.len());
//         }

//         if matches < 20 {
//             warn!("tracking_backend::track_with_motion_model;not enough matches!!");
//             return Ok(self.sensor.is_imu());
//         }

//         // Optimize frame pose with all matches
//         optimizer::optimize_pose(current_frame, &self.map);

//         // Discard outliers
//         let nmatches_map = self.discard_outliers(current_frame);

//         if self.localization_only_mode {
//             todo!("Localization only");
//             // mbVO = nmatchesMap<10;
//             // return nmatches>20;
//         }

//         match self.sensor.is_imu() {
//             true => { return Ok(true); },
//             false => { return Ok(nmatches_map >= 10); }
//         };

//     }

//     fn update_last_frame(&mut self, last_frame: &mut Frame) {
//         // Tracking::UpdateLastFrame()

//         // Update pose according to reference keyframe
//         let ref_kf_id = last_frame.ref_kf_id.unwrap();
//         let map_lock = self.map.read().unwrap();
//         let ref_kf_pose = map_lock.keyframes.get(&ref_kf_id).expect(format!("Reference kf {} should be in map", ref_kf_id).as_str()).pose;
//         last_frame.pose = Some(*self.trajectory_poses.last().unwrap() * ref_kf_pose);

//         if self.sensor.is_mono() || self.frames_since_last_kf == 0 {
//             return;
//         }

//         todo!("Stereo, RGBD");
//         // Create "visual odometry" MapPoints
//         // We sort points according to their measured depth by the stereo/RGB-D sensor
//         // vector<pair<float,int> > vDepthIdx;
//         // const int Nfeat = mLastFrame.Nleft == -1? mLastFrame.N : mLastFrame.Nleft;
//         // vDepthIdx.reserve(Nfeat);
//         // for(int i=0; i<Nfeat;i++)
//         // {
//         //     float z = mLastFrame.mvDepth[i];
//         //     if(z>0)
//         //     {
//         //         vDepthIdx.push_back(make_pair(z,i));
//         //     }
//         // }

//         // if(vDepthIdx.empty())
//         //     return;

//         // sort(vDepthIdx.begin(),vDepthIdx.end());

//         // // We insert all close points (depth<mThDepth)
//         // // If less than 100 close points, we insert the 100 closest ones.
//         // int nPoints = 0;
//         // for(size_t j=0; j<vDepthIdx.size();j++)
//         // {
//         //     int i = vDepthIdx[j].second;

//         //     bool bCreateNew = false;

//         //     MapPoint* pMP = mLastFrame.mvpMapPoints[i];

//         //     if(!pMP)
//         //         bCreateNew = true;
//         //     else if(pMP->Observations()<1)
//         //         bCreateNew = true;

//         //     if(bCreateNew)
//         //     {
//         //         Eigen::Vector3f x3D;

//         //         if(mLastFrame.Nleft == -1){
//         //             mLastFrame.UnprojectStereo(i, x3D);
//         //         }
//         //         else{
//         //             x3D = mLastFrame.UnprojectStereoFishEye(i);
//         //         }

//         //         MapPoint* pNewMP = new MapPoint(x3D,mpAtlas->GetCurrentMap(),&mLastFrame,i);
//         //         mLastFrame.mvpMapPoints[i]=pNewMP;

//         //         mlpTemporalPoints.push_back(pNewMP);
//         //         nPoints++;
//         //     }
//         //     else
//         //     {
//         //         nPoints++;
//         //     }

//         //     if(vDepthIdx[j].first>mThDepth && nPoints>100)
//         //         break;

//         // }
//     }

//     fn track_local_map(&mut self, current_frame: &mut Frame, last_frame: &mut Frame) -> (bool, i32) {
//         let _span = tracy_client::span!("track_local_map");
//         // bool Tracking::TrackLocalMap()
//         // We have an estimation of the camera pose and some map points tracked in the frame.
//         // We retrieve the local map and try to find matches to points in the local map.
//         self.update_local_keyframes(current_frame, last_frame);
//         self.update_local_points(current_frame);
//         match self.search_local_points(current_frame) {
//             Ok(res) => res,
//             Err(e) => warn!("Error in search_local_points: {}", e)
//         }

//         if !self.imu.is_initialized || (current_frame.frame_id <= self.relocalization.last_reloc_frame_id + (self.frames_to_reset_imu as i32)) {
//             optimizer::optimize_pose(current_frame, &self.map);
//         } else if !self.map_updated {
//             optimizer::pose_inertial_optimization_last_frame(current_frame, &self.map);
//         } else {
//             optimizer::pose_inertial_optimization_last_keyframe(current_frame);
//         }

//         self.matches_inliers = 0;
//         // Update MapPoints Statistics
//         for index in 0..current_frame.mappoint_matches.len() {
//             if let Some((mp_id, is_outlier)) = current_frame.mappoint_matches.get(index) {
//                 if !is_outlier {
//                     if let Some(mp) = self.map.read().unwrap().mappoints.get(&mp_id) {
//                         mp.increase_found(1);
//                     }

//                     if self.localization_only_mode {
//                         let map_read_lock = self.map.read().unwrap();
//                         if let Some(mp) = map_read_lock.mappoints.get(&mp_id) {
//                             if mp.get_observations().len() > 0 {
//                                 self.matches_inliers+=1;
//                             }
//                         } else {
//                             current_frame.mappoint_matches.delete_at_indices((index as i32, -1));
//                         }
//                     } else {
//                         self.matches_inliers += 1;
//                     }

//                 } else if !self.sensor.is_mono() {
//                     todo!("Stereo");
//                     //mCurrentFrame.mappoint_matches[i] = static_cast<MapPoint*>(NULL);
//                     // current_frame.as_mut().unwrap().mappoint_matches.remove(&index);
//                 }
//             }
//         }

//         // Decide if the tracking was succesful
//         // More restrictive if there was a relocalization recently
//         if current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames_to_insert_kf as i32) && self.matches_inliers<50 {
//             warn!("track_local_map unsuccessful; matches in frame < 50 : {}",self.matches_inliers);
//             return (false, self.matches_inliers);
//         }

//         match self.state {
//             TrackingState::RecentlyLost => { 
//                 return (self.matches_inliers > 10, self.matches_inliers);
//             },
//             _ => {}
//         }

//         match self.sensor {
//             Sensor(FrameSensor::Mono, ImuSensor::Some) => { 
//                 return (
//                     !(self.matches_inliers<15 && self.imu.is_initialized) && !(self.matches_inliers<50 && !self.imu.is_initialized),
//                     self.matches_inliers
//                 )
//             },
//             Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => {
//                 return (self.matches_inliers >= 15, self.matches_inliers);
//             },
//             _ => { return (self.matches_inliers > 30, self.matches_inliers) }
//         }
//     }

//     fn update_local_keyframes(&mut self, current_frame: &mut Frame, last_frame: &mut Frame) -> Option<()> {
//         let _span = tracy_client::span!("update_local_keyframes");
//         // void Tracking::UpdateLocalKeyFrames()
//         // Each map point votes for the keyframes in which it has been observed

//         // BTreeMap so items are sorted, so we have the same output as orbslam for testing. Can probably revert to regular hashmap later.
//         let mut kf_counter = BTreeMap::<Id, i32>::new();
//         let lock = self.map.read().unwrap();
//         // TODO (IMU) ... Check below should be !self.imu.is_initialized || current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2
//         if current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
//             for i in 0..current_frame.mappoint_matches.len() {
//                 if let Some((mp_id, _is_outlier)) = current_frame.mappoint_matches.get(i as usize) {
//                     if let Some(mp) = lock.mappoints.get(&mp_id) {
//                         for kf_id in mp.get_observations().keys() {
//                             *kf_counter.entry(*kf_id).or_insert(0) += 1;
//                         }
//                     } else {
//                         current_frame.mappoint_matches.delete_at_indices((i as i32, -1));
//                     }
//                 }
//             }

//         } else {
//             // Using lastframe since current frame has no matches yet
//             for i in 0..last_frame.mappoint_matches.len() {
//                 if let Some((mp_id, _is_outlier)) = last_frame.mappoint_matches.get(i as usize) {
//                     if let Some(mp) = lock.mappoints.get(&mp_id) {
//                         for kf_id in mp.get_observations().keys() {
//                             *kf_counter.entry(*kf_id).or_insert(0) += 1;
//                         }
//                     } else {
//                         last_frame.mappoint_matches.delete_at_indices((i as i32, -1));
//                     }
//                 }
//             }
//         }


//         let (mut max, mut max_kf_id) = (0, 0);
//         self.local_keyframes.clear();
//         let mut local_kf_vec = Vec::<Id>::new();

//         // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
//         for (kf_id, count) in kf_counter {
//             if count > max {
//                 max = count;
//                 max_kf_id = kf_id;
//             }
//             local_kf_vec.push(kf_id);
//             self.kf_track_reference_for_frame.insert(kf_id, current_frame.frame_id);
//         }

//         // Also include some keyframes that are neighbors to already-included keyframes
//         let mut i = 0;
//         while local_kf_vec.len() <= 80 && i < local_kf_vec.len() { // Limit the number of keyframes
//             let next_kf_id = local_kf_vec[i];
//             let kf = lock.keyframes.get(&next_kf_id);
//             match kf {
//                 Some(kf) => {
//                     for kf_id in &kf.get_covisibility_keyframes(10) {
//                         if self.kf_track_reference_for_frame.get(kf_id) != Some(&current_frame.frame_id) {
//                             local_kf_vec.push(*kf_id);
//                             self.kf_track_reference_for_frame.insert(*kf_id, current_frame.frame_id);
//                             break;
//                         }
//                     }
//                     for kf_id in &kf.children {
//                         if self.kf_track_reference_for_frame.get(kf_id) != Some(&current_frame.frame_id) {
//                             local_kf_vec.push(*kf_id);
//                             self.kf_track_reference_for_frame.insert(*kf_id, current_frame.frame_id);
//                             break;
//                         }
//                     }

//                     if let Some(parent_id) = kf.parent {
//                         if self.kf_track_reference_for_frame.get(&parent_id) != Some(&current_frame.frame_id) {
//                             local_kf_vec.push(parent_id);
//                             self.kf_track_reference_for_frame.insert(parent_id, current_frame.frame_id);
//                         }
//                     };
//                 },
//                 None => {
//                     // KF could have been deleted since constructing local_kf_vec
//                     local_kf_vec.remove(i);
//                 },
//             };

//             i += 1;
//         }
//         self.local_keyframes = local_kf_vec.iter().map(|x| *x).collect();

//         // Add 10 last temporal KFs (mainly for IMU)
//         if self.sensor.is_imu() && self.local_num_keyframes() < 80 {
//             todo!("IMU");
//             // For the last 20 created keyframes, save into new_local_keyframes
//             // until one of them was not in self.local_keyframes. Then, quit.
//         }

//         if max_kf_id != 0 {
//             current_frame.ref_kf_id = Some(max_kf_id);
//             self.ref_kf_id = Some(max_kf_id);
//         }
//         None
//     }

//     fn update_local_points(&mut self, current_frame: &mut Frame) {
//         let _span = tracy_client::span!("update_local_points");
//         // void Tracking::UpdateLocalPoints()
//         self.local_mappoints.clear();
//         let lock = self.map.read().unwrap();
//         let mut kfs_to_remove = vec![];
//         for kf_id in self.local_keyframes.iter().rev() {
//             match lock.keyframes.get(kf_id) {
//                 Some(kf) => {
//                     let mp_ids = kf.get_mp_matches();
//                     for item in mp_ids {
//                         if let Some((mp_id, _)) = item {
//                             if self.mp_track_reference_for_frame.get(mp_id) != Some(&current_frame.frame_id) {
//                                 self.local_mappoints.insert(*mp_id);
//                                 self.mp_track_reference_for_frame.insert(*mp_id, current_frame.frame_id);
//                             }
//                         }
//                     }
//                 },
//                 None => kfs_to_remove.push(*kf_id),
//             }
//         }

//         // KF could have been deleted since construction of local_keyframes
//         for kf_id in kfs_to_remove {
//             self.local_keyframes.remove(&kf_id);
//         }
//     }

//     fn search_local_points(&mut self, current_frame: &mut Frame) -> Result<(), Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("search_local_points");
//         //void Tracking::SearchLocalPoints()

//         // Do not search map points already matched
//         {
//             let mut increased_visible = vec![];
//             let lock = self.map.read().unwrap();
//             for index in 0..current_frame.mappoint_matches.len() {
//                 if let Some((id, _)) = current_frame.mappoint_matches.get(index as usize) {
//                     if let Some(mp) = lock.mappoints.get(&id) {
//                         mp.increase_visible(1);
//                         increased_visible.push(id);

//                         self.last_frame_seen.insert(id, current_frame.frame_id);
//                         self.track_in_view.remove(&id);
//                         self.track_in_view_r.remove(&id);
//                     } else {
//                         current_frame.mappoint_matches.delete_at_indices((index as i32, -1));
//                     }
//                 }
//             }
//         }

//         // Project points in frame and check its visibility
//         let mut to_match = 0;
//         {
//             let lock = self.map.read().unwrap();
//             for mp_id in &self.local_mappoints {
//                 if self.last_frame_seen.get(mp_id) == Some(&current_frame.frame_id) {
//                     continue;
//                 }

//                 match lock.mappoints.get(mp_id) {
//                     Some(mp) => {
//                         // Project (this fills MapPoint variables for matching)
//                         let (tracked_data_left, tracked_data_right) = current_frame.is_in_frustum(mp, 0.5);
//                         if tracked_data_left.is_some() || tracked_data_right.is_some() {
//                             lock.mappoints.get(&mp_id).unwrap().increase_visible(1);
//                             to_match += 1;
//                         }
//                         if let Some(d) = tracked_data_left {
//                             self.track_in_view.insert(*mp_id, d);
//                         } else {
//                             self.track_in_view.remove(mp_id);
//                         }
//                         if let Some(d) = tracked_data_right {
//                             self.track_in_view_r.insert(*mp_id, d);
//                         } else {
//                             self.track_in_view_r.remove(mp_id);
//                         }
//                     },
//                     None => {
//                         continue;
//                     }
//                 };
//             }
//         }

//         if to_match > 0 {
//             let mut th = match self.sensor.frame() {
//                 FrameSensor::Rgbd => 3,
//                 _ => 1
//             };
//             if self.imu.is_initialized {
//                 if self.imu.imu_ba2 {
//                     th = 2;
//                 } else {
//                     th = 6;
//                 }
//             } else if !self.imu.is_initialized && self.sensor.is_imu() {
//                 th = 10;
//             }

//             // If the camera has been relocalised recently, perform a coarser search
//             if current_frame.frame_id < self.relocalization.last_reloc_frame_id + 2 {
//                 th = 5;
//             }
//             match self.state {
//                 TrackingState::RecentlyLost | TrackingState::Lost => th = 15,
//                 _ => {}
//             }

//             let (non_tracked_points, matches) = orbmatcher::search_by_projection(
//                 current_frame,
//                 &mut self.local_mappoints,
//                 th, 0.8,
//                 &self.track_in_view, &self.track_in_view_r,
//                 &self.map, self.sensor
//             )?;
//             debug!("Tracking search local points: {} matches / {} local points. ({} total mappoints in map)", matches, self.local_mappoints.len(), self.map.read().unwrap().mappoints.len());

//         }

//         Ok(())
//     }

//     fn create_new_keyframe(&mut self, current_frame: &mut Frame) {
//         let _span = tracy_client::span!("create_new_keyframe");
//         //CreateNewKeyFrame
//         // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3216
//         let new_kf = Frame::new_clone(&current_frame);

//         if self.sensor.is_imu() {
//             todo!("IMU"); //Reset preintegration from last KF (Create new object)
//         //     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKF->GetImuBias(),pKF->mImuCalib);
//         }

//         if !self.sensor.is_mono() {
//             todo!("Stereo");
//         //     mCurrentFrame.UpdatePoseMatrices();
//         //     // cout << "create new MPs" << endl;
//         //     // We sort points by the measured depth by the stereo/RGBD sensor.
//         //     // We create all those MapPoints whose depth < mThDepth.
//         //     // If there are less than 100 close points we create the 100 closest.
//         //     int maxPoint = 100;
//         //     if(mSensor == System::IMU_STEREO || mSensor == System::IMU_RGBD)
//         //         maxPoint = 100;

//         //     vector<pair<float,int> > vDepthIdx;
//         //     int N = (mCurrentFrame.Nleft != -1) ? mCurrentFrame.Nleft : mCurrentFrame.N;
//         //     vDepthIdx.reserve(mCurrentFrame.N);
//         //     for(int i=0; i<N; i++)
//         //     {
//         //         float z = mCurrentFrame.mvDepth[i];
//         //         if(z>0)
//         //         {
//         //             vDepthIdx.push_back(make_pair(z,i));
//         //         }
//         //     }

//         //     if(!vDepthIdx.empty())
//         //     {
//         //         sort(vDepthIdx.begin(),vDepthIdx.end());

//         //         int nPoints = 0;
//         //         for(size_t j=0; j<vDepthIdx.size();j++)
//         //         {
//         //             int i = vDepthIdx[j].second;

//         //             bool bCreateNew = false;

//         //             MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
//         //             if(!pMP)
//         //                 bCreateNew = true;
//         //             else if(pMP->Observations()<1)
//         //             {
//         //                 bCreateNew = true;
//         //                 mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
//         //             }

//         //             if(bCreateNew)
//         //             {
//         //                 Eigen::Vector3f x3D;

//         //                 if(mCurrentFrame.Nleft == -1){
//         //                     mCurrentFrame.UnprojectStereo(i, x3D);
//         //                 }
//         //                 else{
//         //                     x3D = mCurrentFrame.UnprojectStereoFishEye(i);
//         //                 }

//         //                 MapPoint* pNewMP = new MapPoint(x3D,pKF,mpAtlas->GetCurrentMap());
//         //                 pNewMP->AddObservation(pKF,i);

//         //                 //Check if it is a stereo observation in order to not
//         //                 //duplicate mappoints
//         //                 if(mCurrentFrame.Nleft != -1 && mCurrentFrame.mvLeftToRightMatch[i] >= 0){
//         //                     mCurrentFrame.mvpMapPoints[mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]]=pNewMP;
//         //                     pNewMP->AddObservation(pKF,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
//         //                     pKF->AddMapPoint(pNewMP,mCurrentFrame.Nleft + mCurrentFrame.mvLeftToRightMatch[i]);
//         //                 }

//         //                 pKF->AddMapPoint(pNewMP,i);
//         //                 pNewMP->ComputeDistinctiveDescriptors();
//         //                 pNewMP->UpdateNormalAndDepth();
//         //                 mpAtlas->AddMapPoint(pNewMP);

//         //                 mCurrentFrame.mvpMapPoints[i]=pNewMP;
//         //                 nPoints++;
//         //             }
//         //             else
//         //             {
//         //                 nPoints++;
//         //             }

//         //             if(vDepthIdx[j].first>mThDepth && nPoints>maxPoint)
//         //             {
//         //                 break;
//         //             }
//         //         }
//         //     }
//         }

//         // mnLastKeyFrameId = mCurrentFrame.mnId;
//         // mpLastKeyFrame = pKF;

//         self.last_kf_timestamp = Some(new_kf.timestamp);

//         tracy_client::Client::running()
//         .expect("message! without a running Client")
//         .message("create new keyframe", 2);

//         // KeyFrame created here and inserted into map
//         self.system.send(
//             LOCAL_MAPPING,
//             Box::new( NewKeyFrameMsg{  keyframe: new_kf, } )
//         );
//     }

//     fn need_new_keyframe(&mut self, current_frame: &mut Frame) -> bool {
//         let num_kfs = self.map.read().unwrap().num_keyframes();
//         let not_enough_frames_since_last_reloc = (current_frame.frame_id < self.relocalization.last_reloc_frame_id + (self.max_frames_to_insert_kf as i32)) && (num_kfs as i32 > self.max_frames_to_insert_kf);
//         let imu_not_initialized = self.sensor.is_imu() && !self.imu.is_initialized;

//         let too_close_to_last_kf = match self.last_kf_timestamp {
//             Some(timestamp) => current_frame.timestamp - timestamp < 0.25, // 250 milliseconds
//             None => false
//         };

//         if self.localization_only_mode || not_enough_frames_since_last_reloc || (imu_not_initialized && too_close_to_last_kf) {
//             return false;
//         } else if imu_not_initialized && !too_close_to_last_kf {
//             return true;
//         }

//         // Tracked MapPoints in the reference keyframe
//         let min_observations = match num_kfs <= 2 {
//             true => 2,
//             false => 3
//         };

//         let map_lock = self.map.read().unwrap();
//         let tracked_mappoints = map_lock.keyframes.get(&self.ref_kf_id.unwrap())
//             .map(|kf| kf.get_tracked_mappoints(&*map_lock, min_observations) as f32)
//             .unwrap_or(0.0);

//         // Check how many "close" points are being tracked and how many could be potentially created.
//         let (tracked_close, non_tracked_close) = current_frame.check_close_tracked_mappoints();
//         let need_to_insert_close = (tracked_close<100) && (non_tracked_close>70);

//         // Thresholds
//         let th_ref_ratio = match self.sensor {
//             Sensor(FrameSensor::Mono, ImuSensor::None) => 0.9,
//             Sensor(FrameSensor::Mono, ImuSensor::Some) => {
//                 // Points tracked from the local map
//                 if self.matches_inliers > 350 { 0.75 } else { 0.90 }
//             },
//             Sensor(FrameSensor::Stereo, _) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => 0.75,
//             Sensor(FrameSensor::Rgbd, ImuSensor::None) => { if num_kfs < 2 { 0.4 } else { 0.75 } }
//         };

//         // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
//         let c1a = self.frames_since_last_kf >= (self.max_frames_to_insert_kf as i32);
//         // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
//         let c1b = self.frames_since_last_kf >= (self.min_frames_to_insert_kf as i32) && LOCAL_MAPPING_IDLE.load(Ordering::SeqCst);
//         //Condition 1c: tracking is weak
//         let sensor_is_right = match self.sensor {
//             Sensor(FrameSensor::Mono, _) | Sensor(FrameSensor::Stereo, ImuSensor::Some) | Sensor(FrameSensor::Rgbd, ImuSensor::Some) => false,
//             _ => true
//         }; // I do not know why they just select for RGBD or Stereo without IMU
//         let c1c = sensor_is_right && ((self.matches_inliers as f32) < tracked_mappoints * 0.25 || need_to_insert_close) ;
//         // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
//         let c2 = (((self.matches_inliers as f32) < tracked_mappoints * th_ref_ratio || need_to_insert_close)) && self.matches_inliers > 15;

//         // Temporal condition for Inertial cases
//         let close_to_last_kf = match self.last_kf_timestamp {
//             Some(timestamp) => current_frame.timestamp - timestamp < 0.5, // 500 milliseconds
//             None => false
//         };

//         let c3 = self.sensor.is_imu() && close_to_last_kf;

//         let recently_lost = match self.state {
//             TrackingState::RecentlyLost => true,
//             _ => false
//         };
//         let sensor_is_imumono = match self.sensor {
//             Sensor(FrameSensor::Rgbd, ImuSensor::Some) => true,
//             _ => false
//         };
//         let c4 = ((self.matches_inliers < 75 && self.matches_inliers > 15) || recently_lost) && sensor_is_imumono;

//         // Note: removed code here about checking for idle local mapping and/or interrupting bundle adjustment
//         let create_new_kf =  ((c1a||c1b||c1c) && c2)||c3 ||c4;

//         tracy_client::Client::running()
//             .expect("message! without a running Client")
//             .message(format!("need new kf: {} {}", create_new_kf, LOCAL_MAPPING_IDLE.load(Ordering::SeqCst)).as_str(), 2);

//         if LOCAL_MAPPING_IDLE.load(Ordering::SeqCst) && create_new_kf {
//             self.frames_since_last_kf = 0;
//             return true;
//         } else {
//             self.frames_since_last_kf += 1;
//             return false;
//         }
//     }

//     //* Helper functions */
//     fn discard_outliers(&mut self, current_frame: &mut Frame) -> i32 {
//         let discarded = current_frame.delete_mappoint_outliers();
//         // Note: orbslam removes mappoint from track_in_view if the index < mCurrentFrame.Nleft but Nleft is always -1!
//         // So, commenting out for now.
//         for (mp_id, _index) in discarded {
//             // if (index as u32) < current_frame.features.num_keypoints {
//             //     self.track_in_view.remove(&mp_id);
//             //     // TODO (Stereo) ... need to remove this from track_in_view_r if the mp is seen in the right camera
//             // }
//             self.last_frame_seen.insert(mp_id, current_frame.frame_id);
//         }
//         current_frame.mappoint_matches.tracked_mappoints(&*self.map.read().unwrap(), 1)
//     }

//     //* Next steps */
//     fn create_new_map(&self) -> bool {
//         todo!("Multimaps: Atlas::CreateNewMap");
//     }
//     fn send_to_visualizer(&mut self, keypoints: VectorOfKeyPoint, image: Mat, timestamp: Timestamp) {
//         // Send image and features to visualizer
//         self.system.find_actor(VISUALIZER).send(Box::new(VisFeaturesMsg {
//             keypoints: DVVectorOfKeyPoint::new(keypoints),
//             image,
//             timestamp,
//         })).unwrap();
//     }
// }
