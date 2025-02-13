// extern crate g2o;
// use log::{warn, info, debug, error};
// use nalgebra::{Isometry3, Quaternion, Vector3, Vector6};
// use std::{collections::{HashMap, VecDeque}, f64::INFINITY};
// use opencv::{prelude::*, types::{VectorOfKeyPoint, VectorOfMat, VectorOfPoint2f}};
// use gtsam::{
//     inference::symbol::Symbol, navigation::combined_imu_factor::{CombinedImuFactor, PreintegratedCombinedMeasurements, PreintegrationCombinedParams}, nonlinear::{
//         isam2::ISAM2, nonlinear_factor_graph::NonlinearFactorGraph, values::Values
//     },
// };
// use std::fmt::Debug;
// use core::{
//     config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, MessageBox, System, Timestamp}
// };
// use crate::{
//     actors::messages::{ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg}, map::{features::Features, frame::Frame, keyframe::{KeyFrame, MapPointMatches}, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{geometric_tools, good_features_to_track::GoodFeaturesExtractor, image, imu::{ImuBias, ImuCalib, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{CameraModule, FeatureExtractionModule, ImuModule}, optimizer::{self, LEVEL_SIGMA2}, orbslam_extractor::ORBExtractor, orbslam_matcher::SCALE_FACTORS, relocalization::Relocalization}, registered_actors::{CAMERA_MODULE, FEATURE_DETECTION, FEATURE_MATCHER, FEATURE_MATCHING_MODULE, SHUTDOWN_ACTOR, TRACKING_BACKEND, VISUALIZER}
// };

// use super::messages::{FeatureTracksAndIMUMsg, TrajectoryMsg, VisTrajectoryMsg};
// use crate::registered_actors::IMU;

// pub struct TrackingBackendGTSAM {
//     system: System,
//     map: ReadWriteMap,
//     sensor: Sensor,

//     // Frames
//     last_kf_id: Id,
//     current_kf_id: Id,

//     // Modules 
//     imu: IMU,
//     graph_solver: GraphSolver,

//     // Poses in trajectory
//     trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
// }

// impl Actor for TrackingBackendGTSAM {
//     type MapRef = ReadWriteMap;

//     fn spawn(system: System, map: Self::MapRef) {
//         let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
//         let imu = IMU::new();

//         let mut actor = TrackingBackendGTSAM {
//             system,
//             graph_solver: GraphSolver::new(),
//             sensor,
//             map,
//             imu,
//             trajectory_poses: Vec::new(),
//             last_kf_id: -1,
//             current_kf_id: -1,
//         };
//         tracy_client::set_thread_name!("tracking backend gtsam");

//         loop {
//             let message = actor.system.receive().unwrap();
//             if actor.handle_message(message) {
//                 break;
//             }
//             actor.map.match_map_version();
//         }
//     }

// }

// impl TrackingBackendGTSAM {
//     fn handle_message(&mut self, message: MessageBox) -> bool {
//         if message.is::<FeatureTracksAndIMUMsg>() {
//             if self.system.queue_full() {
//                 // Abort additional work if there are too many frames in the msg queue.
//                 info!("Tracking gtsam dropped 1 frame");
//                 return false;
//             }

//             let msg = message.downcast::<FeatureTracksAndIMUMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
//             self.handle_regular_message(*msg).unwrap();
//         // } else if message.is::<UpdateFrameIMUMsg>() {

//         } else if message.is::<ShutdownMsg>() {
//             return true;
//         } else {
//             warn!("Tracking backend received unknown message type!");
//         }
//         return false;
//     }

//     fn handle_regular_message(&mut self, mut msg: FeatureTracksAndIMUMsg) -> Result<(), Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("track");

//         if self.last_kf_id == -1 {
//             // If this is the first frame, don't do anything
//             // If we are receiving it here, it was already put into the map as the latest keyframe
//             self.last_kf_id = 1;
//         } else {
//             // If we have previous frames already, can track normally
//             let mut current_frame = msg.frame;

//             // if !self.map.read()?.imu_initialized {
//             //     let (prior_g, prior_a, fiba) = match self.sensor.frame() {
//             //         FrameSensor::Mono => (1e2, 1e10, true),
//             //         FrameSensor::Stereo | FrameSensor::Rgbd => (1e2, 1e5, true),
//             //     };

//             //     self.imu.initialize(&mut self.map, 1, prior_g, prior_a, fiba, Some(self.system.find_actor(TRACKING_BACKEND)))?;
//             // }

//             // At this point we only have feature matches from front end, not the matches to mappoints. Add the matches here
//             self.map_feature_tracks_to_mappoints(&mut current_frame, & msg.mappoint_ids)?;

//             // Solve VIO graph. Includes preintegration
//             let optimized = self.graph_solver.solve(&mut current_frame, self.last_kf_id, &mut msg.imu_measurements, & self.map)?;
//             if !optimized {
//                 warn!("Could not optimize graph");
//             }

//             // Create new keyframe! Forget the old frame.
//             self.current_kf_id = self.create_new_keyframe(current_frame).unwrap();
//             // Create new mappoints and cull existing
//             let created_mps = self.create_new_mappoints().unwrap();
//             println!("Created {} mps", created_mps);

//             self.update_trajectory_in_logs().expect("Could not save trajectory");
//             self.last_kf_id = self.current_kf_id;
//         }

//         return Ok(());
//     }

//     fn update_trajectory_in_logs(
//         &mut self,
//     ) -> Result<(), Box<dyn std::error::Error>> {
//         let map = self.map.read()?;
//         let curr_kf = map.get_keyframe(self.current_kf_id);
//         let last_kf = map.get_keyframe(self.last_kf_id);

//         let relative_pose = curr_kf.get_pose() * last_kf.get_pose();

//         self.trajectory_poses.push(relative_pose);

//         println!("Sanity check... curr kf is {}, ref kf is {}, last kf is {}", curr_kf.id, curr_kf.ref_kf_id.unwrap(), last_kf.id);

//         self.system.send(
//             SHUTDOWN_ACTOR, 
//             Box::new(TrajectoryMsg{
//                 pose: curr_kf.get_pose().inverse(),
//                 ref_kf_id: curr_kf.ref_kf_id.unwrap(),
//                 timestamp: curr_kf.timestamp,
//                 map_version: self.map.read()?.version
//             })
//         );

//         let map = self.map.read()?;
//         let bla = last_kf.get_mp_matches().iter().filter_map(|v| match v { 
//            Some((id, _is_outlier)) => Some(*id),
//            None => None
//         });
//         self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
//             pose: curr_kf.get_pose(),
//             mappoint_matches: vec![],
//             nontracked_mappoints: HashMap::new(),
//             mappoints_in_tracking: bla.collect(),
//             timestamp: curr_kf.timestamp,
//             map_version: map.version
//         }));

//         Ok(())
//     }

//     fn create_new_keyframe(&mut self, mut current_frame: Frame) -> Result<i32, Box<dyn std::error::Error>>{
//         let _span = tracy_client::span!("create_new_keyframe");

//         self.imu.imu_preintegrated_from_last_kf = ImuPreIntegrated::new(ImuBias::new());

//         debug!("SOFIYA FEATURES. In backend, frame has N {}, features {}, mappoint matches {}", current_frame.features.num_keypoints, current_frame.features.get_all_keypoints().len(), current_frame.mappoint_matches.len(), );

//         debug!("SOFIYA FEATURES. After mapping feature tracks, frame has N {}, features {}, mappoint matches {}", current_frame.features.num_keypoints, current_frame.features.get_all_keypoints().len(), current_frame.mappoint_matches.len());

//         current_frame.ref_kf_id = Some(self.last_kf_id);

//         let kf_id = self.map.write()?.insert_keyframe_to_map(current_frame, false);

//         let map = self.map.read()?;
//         let curr_kf = map.get_keyframe(kf_id);

//         debug!("SOFIYA FEATURES. After inserting keyframe, frame has N {}, features {}, mappoint matches {}", curr_kf.features.num_keypoints, curr_kf.features.get_all_keypoints().len(), curr_kf.get_mp_matches().len());

//         tracy_client::Client::running()
//         .expect("message! without a running Client")
//         .message("create new keyframe", 2);

//         // TODO SOFIYA probably want to create new mappoints here somehow

//         Ok(kf_id)
//     }

//     fn map_feature_tracks_to_mappoints(&mut self, current_frame: &mut Frame, tracked_mappoint_ids: & Vec<i32>) -> Result<i32, Box<dyn std::error::Error> > {
//         let _span = tracy_client::span!("map_feature_tracks_to_mappoints");

//         let map = self.map.read()?;
//         let ref_kf = map.get_keyframe(self.last_kf_id);
//         let mut added = 0;

//         println!("Current frame features: {:?}", current_frame.features.get_all_keypoints().len());
//         println!("Tracked mappoint ids: {:?}", tracked_mappoint_ids.len());
//         println!("Ref kf mappoint matches: {:?}", ref_kf.get_mp_matches().len());
//         // println!("Ref kf matches: {:?}", ref_kf.get_mp_matches());

//         for idx1 in 0..current_frame.features.get_all_keypoints().len() - 1 {
//             if tracked_mappoint_ids[idx1 as usize] != -1 {
//                 // Most times the id will be a real mappoint
//                 // But when new features are extracted for this frame, there is no associated mappoint yet
//                 // We can create them later when we create the keyframe, but for now ignore.
//                 current_frame.mappoint_matches.add(idx1 as u32, tracked_mappoint_ids[idx1 as usize], false);
//                 added += 1;
//             }
//         }
//         println!("Added {} mappoints", added);

//         Ok(added)
//     }

//     fn create_new_mappoints(&mut self) -> Result<i32, Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("create_new_mappoints");

//         // Retrieve neighbor keyframes in covisibility graph
//         let nn = match self.sensor.is_mono() {
//             true => 30,
//             false => 10
//         };
//         let ratio_factor = 1.5 * SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor");
//         let fpt = SETTINGS.get::<f64>(FEATURE_MATCHER, "far_points_threshold");
//         let far_points_th = if fpt == 0.0 { INFINITY } else { fpt };

//         let mut mps_to_insert = vec![];
//         let mut mps_created = 0;
//         {
//             let _span = tracy_client::span!("create_new_mappoints: before loop");
//             let lock = self.map.read()?;

//             let current_kf = lock.get_keyframe(self.current_kf_id);
//             let mut neighbor_kfs = current_kf.get_covisibility_keyframes(nn);
//             if self.sensor.is_imu() {
//                 let mut count = 0;
//                 let mut pkf = current_kf;
//                 while (neighbor_kfs.len() as i32) < nn && pkf.prev_kf_id.is_some() && count < nn {
//                     let prev_kf = current_kf.prev_kf_id.unwrap();
//                     if !neighbor_kfs.contains(&prev_kf) {
//                         neighbor_kfs.push(prev_kf);
//                     }
//                     pkf = lock.get_keyframe(prev_kf);
//                     count += 1;
//                 }
//             }

//             let mut pose1 = current_kf.get_pose(); // sophTcw1
//             let translation1 = pose1.get_translation(); // tcw1
//             let rotation1 = pose1.get_rotation(); // Rcw1
//             let rotation_transpose1 = rotation1.transpose(); // Rwc1
//             let mut ow1 = current_kf.get_camera_center();

//             drop(_span);
//             let _span = tracy_client::span!("create_new_mappoints: loop");
//             // println!("Create_new_mappoints neighbor_kfs: {:?}", neighbor_kfs);

//             // Search matches with epipolar restriction and triangulate
//             for neighbor_id in neighbor_kfs {
//                 if self.system.queue_len() > 1 {
//                     // Abort additional work if there are too many keyframes in the msg queue.
//                     return Ok(0);
//                 }
//                 let neighbor_kf = lock.get_keyframe(neighbor_id);

//                 // Check first that baseline is not too short
//                 let mut ow2 = neighbor_kf.get_camera_center();
//                 let baseline = (*ow2 - *ow1).norm();
//                 match self.sensor.is_mono() {
//                     true => {
//                         let median_depth_neigh = neighbor_kf.compute_scene_median_depth(&lock.mappoints, 2);
//                         if baseline / median_depth_neigh < 0.01 {
//                             debug!("Local mapping create new mappoints, continuing bc baseline.. baseline {} median scene depth {}", baseline, median_depth_neigh);
//                             continue
//                         }
//                     },
//                     false => {
//                         if baseline < CAMERA_MODULE.stereo_baseline {
//                             continue
//                         }
//                     }
//                 }

//                 // Search matches that fullfil epipolar constraint
//                 let course = false;

//                 let matches = match FEATURE_MATCHING_MODULE.search_for_triangulation(
//                     current_kf,
//                     neighbor_kf,
//                     false, false, course,
//                     self.sensor
//                 ) {
//                     Ok(matches) => matches,
//                     Err(err) => panic!("Problem with search_for_triangulation {}", err)
//                 };

//                 let mut pose2 = neighbor_kf.get_pose();
//                 let translation2 = pose2.get_translation(); // tcw2
//                 let rotation2 = pose2.get_rotation(); // Rcw2
//                 let rotation_transpose2 = rotation2.transpose(); // Rwc2

//                 // Triangulate each match
//                 for (idx1, idx2) in matches {
//                     let (kp1, right1, kp2, right2) = {
//                         let (kp1, right1) = current_kf.features.get_keypoint(idx1);
//                         let (kp2, right2) = neighbor_kf.features.get_keypoint(idx2);
//                         (kp1, right1, kp2, right2)
//                     };

//                     if right1 {
//                         let _kp1_ur = current_kf.features.get_mv_right(idx1);
//                         pose1 = current_kf.get_right_pose();
//                         ow1 = current_kf.get_right_camera_center();
//                         // camera1 = mpCurrentKeyFrame->mpCamera2 TODO (STEREO) .. right now just using global CAMERA
//                     } else {
//                         pose1 = current_kf.get_pose();
//                         ow1 = current_kf.get_camera_center();
//                         // camera1 = mpCurrentKeyFrame->mpCamera TODO (STEREO)
//                     }
//                     if right2 {
//                         let _kp2_ur = neighbor_kf.features.get_mv_right(idx2);
//                         pose2 = neighbor_kf.get_right_pose();
//                         ow2 = neighbor_kf.get_right_camera_center();
//                         // camera2 = neighbor_kf->mpCamera2 TODO (STEREO)
//                     } else {
//                         pose2 = neighbor_kf.get_pose();
//                         ow2 = neighbor_kf.get_camera_center();
//                         // camera2 = neighbor_kf->mpCamera TODO (STEREO)
//                     }

//                     // Check parallax between rays
//                     let xn1 = CAMERA_MODULE.unproject_eig(&kp1.pt());
//                     let xn2 = CAMERA_MODULE.unproject_eig(&kp2.pt());
//                     let ray1 = rotation_transpose1 * (*xn1);
//                     let ray2 = rotation_transpose2 * (*xn2);
//                     let cos_parallax_rays = ray1.dot(&ray2) / (ray1.norm() * ray2.norm());
//                     let (cos_parallax_stereo1, cos_parallax_stereo2) = (cos_parallax_rays + 1.0, cos_parallax_rays + 1.0);
//                     if right1 {
//                         todo!("Stereo");
//                         // cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
//                     } else if right2 {
//                         todo!("Stereo");
//                         //cosParallaxStereo2 = cos(2*atan2(neighbor_kf->mb/2,neighbor_kf->mvDepth[idx2]));
//                     }
//                     let cos_parallax_stereo = cos_parallax_stereo1.min(cos_parallax_stereo2);

//                     let x3_d;
//                     {
//                         let good_parallax_with_imu = cos_parallax_rays < 0.9996 && self.sensor.is_imu();
//                         let good_parallax_wo_imu = cos_parallax_rays < 0.9998 && !self.sensor.is_imu();
//                         if cos_parallax_rays < cos_parallax_stereo && cos_parallax_rays > 0.0 && (right1 || right2 || good_parallax_with_imu || good_parallax_wo_imu) {
//                             x3_d = geometric_tools::triangulate(xn1, xn2, pose1, pose2);
//                         } else if right1 && cos_parallax_stereo1 < cos_parallax_stereo2 {
//                             x3_d = CAMERA_MODULE.unproject_stereo(lock.get_keyframe(self.current_kf_id), idx1);
//                         } else if right2 && cos_parallax_stereo2 < cos_parallax_stereo1 {
//                             x3_d = CAMERA_MODULE.unproject_stereo(lock.get_keyframe(neighbor_id), idx2);
//                         } else {
//                             continue // No stereo and very low parallax
//                         }
//                         if x3_d.is_none() {
//                             continue
//                         }
//                     }


//                     //Check triangulation in front of cameras
//                     let x3_d_nalg = *x3_d.unwrap();
//                     let z1 = rotation1.row(2).transpose().dot(&x3_d_nalg) + (*translation1)[2];
//                     if z1 <= 0.0 {
//                         continue;
//                     }
//                     let z2 = rotation2.row(2).transpose().dot(&x3_d_nalg) + (*translation2)[2];
//                     if z2 <= 0.0 {
//                         continue;
//                     }

//                     //Check reprojection error in first keyframe
//                     let sigma_square1 = LEVEL_SIGMA2[kp1.octave() as usize];
//                     let x1 = rotation1.row(0).transpose().dot(&x3_d_nalg) + (*translation1)[0];
//                     let y1 = rotation1.row(1).transpose().dot(&x3_d_nalg) + (*translation1)[1];

//                     if right1 {
//                         todo!("Stereo");
//                         // let invz1 = 1.0 / z1;
//                         // let u1 = CAMERA_MODULE.fx * x1 * invz1 + CAMERA_MODULE.cx;
//                         // let u1_r = u1 - CAMERA_MODULE.stereo_baseline_times_fx * invz1;
//                         // let v1 = CAMERA_MODULE.fy * y1 * invz1 + CAMERA_MODULE.cy;
//                         // let err_x1 = u1 as f32 - kp1.pt().x;
//                         // let err_y1 = v1 as f32 - kp1.pt().y;
//                         // let err_x1_r = u1_r as f32 - kp1_ur.unwrap();
//                         // if (err_x1 * err_x1  + err_y1 * err_y1 + err_x1_r * err_x1_r) > 7.8 * sigma_square1 {
//                         //     continue
//                         // }
//                     } else {
//                         let uv1 = CAMERA_MODULE.project(DVVector3::new_with(x1, y1, z1));
//                         let err_x1 = uv1.0 as f32 - kp1.pt().x;
//                         let err_y1 = uv1.1 as f32 - kp1.pt().y;
//                         if (err_x1 * err_x1  + err_y1 * err_y1) > 5.991 * sigma_square1 {
//                             continue
//                         }
//                     }

//                     //Check reprojection error in second keyframe
//                     let sigma_square2 = LEVEL_SIGMA2[kp2.octave() as usize];
//                     let x2 = rotation2.row(0).transpose().dot(&x3_d_nalg) + (*translation2)[0];
//                     let y2 = rotation2.row(1).transpose().dot(&x3_d_nalg) + (*translation2)[1];

//                     if right2 {
//                         todo!("Stereo");
//                         // let invz2 = 1.0 / z2;
//                         // let u2 = CAMERA_MODULE.fx * x2 * invz2 + CAMERA_MODULE.cx;// This should be camera2, not camera
//                         // let u2_r = u2 - CAMERA_MODULE.stereo_baseline_times_fx * invz2;
//                         // let v2 = CAMERA_MODULE.fy * y2 * invz2 + CAMERA_MODULE.cy;// This should be camera2, not camera
//                         // let err_x2 = u2 as f32 - kp2.pt().x;
//                         // let err_y2 = v2 as f32 - kp2.pt().y;
//                         // let err_x2_r = u2_r as f32 - kp2_ur.unwrap();
//                         // if (err_x2 * err_x2  + err_y2 * err_y2 + err_x2_r * err_x2_r) > 7.8 * sigma_square2 {
//                         //     continue
//                         // }
//                     } else {
//                         let uv2 = CAMERA_MODULE.project(DVVector3::new_with(x2, y2, z2));
//                         let err_x2 = uv2.0 as f32 - kp2.pt().x;
//                         let err_y2 = uv2.1 as f32 - kp2.pt().y;
//                         if (err_x2 * err_x2  + err_y2 * err_y2) > 5.991 * sigma_square2 {
//                             continue
//                         }
//                     }

//                     //Check scale consistency
//                     let normal1 = *x3_d.unwrap() - *ow1;
//                     let dist1 = normal1.norm();

//                     let normal2 = *x3_d.unwrap() - *ow2;
//                     let dist2 = normal2.norm();

//                     if dist1 == 0.0 || dist2 == 0.0 {
//                         continue;
//                     }

//                     if dist1 >= far_points_th || dist2 >= far_points_th {
//                         continue;
//                     }

//                     let ratio_dist = dist2 / dist1;
//                     let ratio_octave = (SCALE_FACTORS[kp1.octave() as usize] / SCALE_FACTORS[kp2.octave() as usize]) as f64;
//                     if ratio_dist * ratio_factor < ratio_octave || ratio_dist > ratio_octave * ratio_factor {
//                         continue;
//                     }

//                     // Triangulation is successful
//                     let origin_map_id = lock.id;
//                     let observations = vec![
//                         (self.current_kf_id, lock.get_keyframe(self.current_kf_id).features.num_keypoints, idx1),
//                         (neighbor_id, neighbor_kf.features.num_keypoints, idx2)
//                     ];

//                     mps_to_insert.push((x3_d.unwrap(), self.current_kf_id, origin_map_id, observations));

//                 }
//             }
//         }

//         let _span = tracy_client::span!("create_new_mappoints::insert_mappoints");
//         let mut lock = self.map.write()?;
//         for (x3_d, ref_kf_id, origin_map_id, observations) in mps_to_insert {
//             let mp_id = lock.insert_mappoint_to_map(x3_d, ref_kf_id, origin_map_id, observations)
//             ;
//             // self.recently_added_mappoints.insert(mp_id);
//             mps_created += 1;
//         }

//         Ok(mps_created)
//     }
// }

// pub struct GraphSolver {
//     // New graph
//     graph_new: NonlinearFactorGraph, // New factors that have not been optimized yet
//     values_new: Values, // New values that have not been optimized yet

//     // Main graph
//     graph_main: NonlinearFactorGraph, // Main non-linear GTSAM graph, all created factors
//     values_initial: Values, // All created nodes

//     // Misc GTSAM objects
//     isam2: ISAM2, // ISAM2 solvers
//     // preint_gtsam: PreintegratedCombinedMeasurements, // IMU preintegration

//     // IMU preintegration parameters
//     // Initialization
//     // prior_q_g_to_i: Quaternion<f64>, // prior_qGtoI
//     // prior_p_i_in_g: Vector3<f64>, // prior_pIinG
//     // prior_v_i_in_g: [f64; 3], // prior_vIinG
//     prior_ba: [f64; 3], // prior_ba
//     prior_bg: [f64; 3], // prior_bg
//     // Noise values from dataset sensor
//     accel_noise_density: f64, // accelerometer_noise_density, sigma_a
//     gyro_noise_density: f64, // gyroscope_noise_density, sigma_g
//     accel_random_walk: f64, // accelerometer_random_walk, sigma_wa
//     gyro_random_walk: f64, // gyroscope_random_walk, sigma_wg
//     // Noise values for initialization
//     sigma_prior_rotation: [f64; 3],
//     sigma_prior_translation: [f64; 3],
//     sigma_velocity: f64,
//     sigma_bias: f64,
//     // Misc
//     sigma_camera: f64,

//     // Iterations of the map
//     initialized: bool,
//     ct_state: u64,
//     ct_state_lookup: HashMap<i64, u64>,
//     timestamp_lookup: HashMap<u64, i64>,
//     measurement_smart_lookup_left: HashMap<i32, gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2>, // Smart lookup for IMU measurements
// }

// impl GraphSolver {
//     pub fn new() -> Self {

//         // Set type of mono_noise_ for generic projection factors.
//         gtsam::SharedNoiseModel gaussian_dim_2 = gtsam::noiseModel::Isotropic::Sigma(
//             2, regular_vio_params_.monoNoiseSigma_);

//         selectNormType(&mono_noise_,
//                         gaussian_dim_2,
//                         regular_vio_params_.monoNormType_,
//                         regular_vio_params_.monoNormParam_);

//         // Set type of stereo_noise_ for generic stereo projection factors.
//         gtsam::SharedNoiseModel gaussian_dim_3 = gtsam::noiseModel::Isotropic::Sigma(
//             3, regular_vio_params_.stereoNoiseSigma_);

//         selectNormType(&stereo_noise_,
//                         gaussian_dim_3,
//                         regular_vio_params_.stereoNormType_,
//                         regular_vio_params_.stereoNormParam_);

//         // Set type of regularity noise for point plane factors.
//         gtsam::SharedNoiseModel gaussian_dim_1 = gtsam::noiseModel::Isotropic::Sigma(
//             1, regular_vio_params_.regularityNoiseSigma_);

//         selectNormType(&point_plane_regularity_noise_,
//                         gaussian_dim_1,
//                         regular_vio_params_.regularityNormType_,
//                         regular_vio_params_.regularityNormParam_);

//         mono_cal_.reset(new Cal3_S2(stereo_cal_->calibration()));
//         CHECK(mono_cal_->equals(stereo_cal_->calibration()))
//             << "Monocular calibration should match Stereo calibration";

//         // TODO SOFIYA
//             // gtsam::ISAM2GaussNewtonParams gauss_newton_params;
//             // gauss_newton_params.wildfireThreshold = vio_params.wildfire_threshold_;
//             // isam_param->optimizationParams = gauss_newton_params;
//             // // Cache Linearized Factors seems to improve performance.
//             // isam_param->cacheLinearizedFactors = true;
//             // isam_param->relinearizeThreshold = vio_params.relinearizeThreshold_;
//             // isam_param->relinearizeSkip = vio_params.relinearizeSkip_;
//             // isam_param->findUnusedFactorSlots = true;
//             // // isam_param->enablePartialRelinearizationCheck = true;
//             // isam_param->evaluateNonlinearError = false;  // only for debugging
//             // isam_param->enableDetailedResults = false;     // only for debugging.
//             // isam_param->factorization = gtsam::ISAM2Params::CHOLESKY;  // QR
//         let isam2 = IncrementalFixedLagSmoother::default(backend_params.nr_states_, isam_param);

//         setFactorsParams(backend_params,
//                 &smart_noise_,
//                 &smart_factors_params_,
//                 &no_motion_prior_noise_,
//                 &zero_velocity_prior_noise_,
//                 &constant_velocity_prior_noise_);


//         Self {
//             graph_new: NonlinearFactorGraph::default(),
//             values_new: Values::default(),
//             graph_main: NonlinearFactorGraph::default(),
//             values_initial: Values::default(),
//             isam2,
//             preint_gtsam: PreintegratedCombinedMeasurements::default(),
//             initialized: false,

//             // TODO SOFIYA PARAMS
//             sigma_prior_rotation: [0.1, 0.1, 0.1],
//             sigma_prior_translation: [0.3, 0.3, 0.3],
//             sigma_velocity: 0.1,
//             sigma_bias: 0.15,
//             sigma_camera: 0.306555403,
//             accel_noise_density: SETTINGS.get::<f64>(IMU, "noise_acc"),
//             gyro_noise_density: SETTINGS.get::<f64>(IMU, "noise_gyro"),
//             accel_random_walk: SETTINGS.get::<f64>(IMU, "acc_walk"),
//             gyro_random_walk: SETTINGS.get::<f64>(IMU, "gyro_walk"),

//             // TODO SOFIYA IS THIS RIGHT?
//             // prior_q_g_to_i: Quaternion::new(1.0, 0.0, 0.0, 0.0),
//             // prior_p_i_in_g: Vector3::new(0.0, 0.0, 0.0),
//             prior_ba: [1e-3, 1e-3, 1e-3],
//             prior_bg: [1e-5, 1e-3, 1e-3],
//             // prior_v_i_in_g: [0.0 ,0.0, 0.0],

//             ct_state: 0,
//             ct_state_lookup: HashMap::new(),
//             timestamp_lookup: HashMap::new(),
//             measurement_smart_lookup_left: HashMap::new(),
//         }
//     }

//     fn solve(&mut self, current_frame: &mut Frame, last_kf_id: Id, imu_measurements: &mut ImuMeasurements, map: &ReadWriteMap) -> Result<bool, Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("solve");

//         let timestamp = (current_frame.timestamp * 1e14) as i64; // Convert to int just so we can hash it

//         // Return if the node already exists in the graph
//         if self.ct_state_lookup.contains_key(&timestamp) {
//             println!("NODE WITH TIMESTAMP {} ALREADY EXISTS", timestamp);
//             return Ok(false);
//         }

//         // self.preintegrate(imu_measurements, current_frame, last_kf_id, map);

//         if !self.initialized {
//             // TODO SOFIYA
//             // initializefromimu:
//                 const VioNavState& initial_state_estimate =
//                     InitializationFromImu::getInitialStateEstimate(
//                         input.imu_acc_gyrs_,
//                         imu_params_.n_gravity_,
//                         backend_params_.roundOnAutoInitialize_);

//                 // Initialize Backend using IMU data.
//                 return initStateAndSetPriors(
//                     VioNavStateTimestamped(input.timestamp_, initial_state_estimate));


//             // self.add_initials_and_priors(timestamp, map)?;
//             // self.initialized = true;
//             // println!("GTSAM optimize... Added initials and priors");
//         } else {
//             println!("GTSAM optimize... Regular");

//                     // Features and IMU line up --> do iSAM update.

//                     // Add initial guess.
//                     self.add_state_values(current_frame, status_smart_stereo_measurements_kf.first, pim);

//                     //* IMU FACTORS *//
//                     // Add imu factors between consecutive keyframe states.
//                     self.add_imu_factor(last_kf_id_, curr_kf_id_, pim);

//                     //* VISION MEASUREMENTS *//
//                     const StereoMeasurements& smart_stereo_measurements_kf =
//                         status_smart_stereo_measurements_kf.second;

//                     // Extract relevant information from stereo frame.
//                     // Get the landmarks visible in current keyframe. (These are not all the lmks
//                     // in time horizon used for the optimization!)
//                     LandmarkIds lmks_kf;
//                     self.add_stereo_measurements_to_feature_tracks(
//                         curr_kf_id_, smart_stereo_measurements_kf, &lmks_kf);


//                     // Decide which factors to add.
//                     TrackingStatus kfTrackingStatus_mono =
//                         status_smart_stereo_measurements_kf.first.kfTrackingStatus_mono_;

//                     gtsam::FactorIndices delete_slots(delete_slots_of_converted_smart_factors_);
//                     switch (kfTrackingStatus_mono) {
//                         case TrackingStatus::LOW_DISPARITY: {
//                             // Vehicle is not moving.
//                             VLOG(0) << "Tracker has a LOW_DISPARITY status.";
//                             VLOG(10) << "Add zero velocity and no motion factors.";
//                             addZeroVelocityPrior(curr_kf_id_);
//                             addNoMotionFactor(last_kf_id_, curr_kf_id_);
//                             // TODO why are we not adding the regularities here as well...?
//                             break;
//                         }
//                         default: {
//                             kfTrackingStatus_mono == TrackingStatus::VALID
//                                 ? VLOG(1) << "Tracker has a VALID status."
//                                 : kfTrackingStatus_mono == TrackingStatus::FEW_MATCHES
//                                         ? VLOG(1) << "Tracker has a FEW_MATCHES status."
//                                         : kfTrackingStatus_mono == TrackingStatus::INVALID
//                                             ? VLOG(1) << "Tracker has a INVALID status."
//                                             : kfTrackingStatus_mono == TrackingStatus::DISABLED
//                                                     ? VLOG(1) << "Tracker has a DISABLED status."
//                                                     : VLOG(10) << "";

//                             if (kfTrackingStatus_mono == TrackingStatus::VALID) {
//                                 // Extract lmk ids that are involved in a regularity.
//                                 VLOG(10) << "Starting extracting lmk ids from set of planes...";
//                                 LandmarkIds lmk_ids_with_regularity;
//                                 switch (regular_vio_params_.backend_modality_) {
//                                 case RegularBackendModality::STRUCTURELESS: {
//                                     // Do nothing, lmk_ids_with_regularity should be empty.
//                                     CHECK_EQ(lmk_ids_with_regularity.size(), 0);
//                                     planes_.clear();
//                                     break;
//                                 }
//                                 case RegularBackendModality::PROJECTION: {
//                                     // Transform all smart factors to projection factors,
//                                     // and clear all planes.
//                                     lmk_ids_with_regularity = lmks_kf;
//                                     planes_.clear();
//                                     break;
//                                 }
//                                 case RegularBackendModality::PROJECTION_AND_REGULARITY: {
//                                     // Keep the planes, but change all smart factors to projection
//                                     // factors.
//                                     lmk_ids_with_regularity = lmks_kf;
//                                     break;
//                                 }
//                                 case RegularBackendModality::STRUCTURELESS_AND_PROJECTION: {
//                                     // Transforms to projection factors only the ones that should
//                                     // have regularities, but do not use the planes anymore.
//                                     extractLmkIdsFromPlanes(planes_, &lmk_ids_with_regularity);
//                                     planes_.clear();
//                                     break;
//                                 }
//                                 case RegularBackendModality::
//                                     STRUCTURELESS_PROJECTION_AND_REGULARITY: {
//                                     // Act as usual, keep planes, and transform smart factors to projj
//                                     // factors for those that will have regularities.
//                                     extractLmkIdsFromPlanes(planes_, &lmk_ids_with_regularity);
//                                     break;
//                                 }
//                                 default: {
//                                     LOG(ERROR)
//                                         << "Backend modality: "
//                                         << static_cast<
//                                             std::underlying_type<RegularBackendModality>::type>(
//                                             regular_vio_params_.backend_modality_)
//                                         << " is not supported.";
//                                     break;
//                                 }
//                                 }
//                                 VLOG(10) << "Finished extracting lmk ids from set of planes, total of "
//                                         << lmk_ids_with_regularity.size()
//                                         << " lmks with regularities.";

//                                 // We add features in VIO.
//                                 VLOG(10) << "Starting adding/updating landmarks to graph...";
//                                 addLandmarksToGraph(lmks_kf, lmk_ids_with_regularity);
//                                 VLOG(10) << "Finished adding/updating landmarks to graph.";

//                                 // Convert all smart factors of lmks in time horizon that have
//                                 // regularities to projection factors.
//                                 // Most conversions from smart to proj are done before,
//                                 // in addLandmarksToGraph, but here we also make sure we have converted
//                                 // the ones with regularities in time horizon.
//                                 if (FLAGS_convert_extra_smart_factors_to_proj_factors) {
//                                 VLOG(10)
//                                     << "Starting converting extra smart factors to proj factors...";
//                                 convertExtraSmartFactorToProjFactor(lmk_ids_with_regularity);
//                                 VLOG(10)
//                                     << "Finished converting extra smart factors to proj factors...";
//                                 }

//                                 if (planes_.size() > 0) {
//                                 /////////////////// REGULARITY FACTORS
//                                 //////////////////////////////////////////
//                                 // Add regularity factor on vertices of the mesh.
//                                 // WARNING if a plane has been removed by the mesher, then we will not
//                                 // remove the plane from the optimization, since it won't be in
//                                 // planes.
//                                 std::map<PlaneId, std::vector<std::pair<Slot, LandmarkId>>>
//                                     idx_of_point_plane_factors_to_add;
//                                 for (const Plane& plane : planes_) {
//                                     const PlaneId& plane_key = plane.getPlaneSymbol().key();

//                                     VLOG(10) << "Adding regularity factors.";
//                                     addRegularityFactors(
//                                         plane,
//                                         // Creates a new entry if the plane key was not found.
//                                         // So make sure you use [plane_key] instead of .at(plane_key).
//                                         &(plane_id_to_lmk_id_reg_type_[plane_key]),
//                                         &(idx_of_point_plane_factors_to_add[plane_key]));
//                                     VLOG(10) << "Finished adding regularity factors.";
//                                 }

//                                 if (FLAGS_remove_old_reg_factors) {
//                                     VLOG(10) << "Removing old regularity factors.";
//                                     gtsam::FactorIndices delete_old_regularity_factors;
//                                     removeOldRegularityFactors_Slow(planes_,
//                                                                     idx_of_point_plane_factors_to_add,
//                                                                     &plane_id_to_lmk_id_reg_type_,
//                                                                     &delete_old_regularity_factors);
//                                     if (delete_old_regularity_factors.size() > 0) {
//                                     delete_slots.insert(delete_slots.end(),
//                                                         delete_old_regularity_factors.begin(),
//                                                         delete_old_regularity_factors.end());
//                                     }
//                                     VLOG(10) << "Finished removing old regularity factors.";
//                                 }
//                                 } else {
//                                 // TODO shouldn't we "removeOldRegularityFactors_Slow" because there
//                                 // are no planes anymore? shouldn't we delete them or something?
//                                 // Not really because the mesher will only add planes, it won't delete
//                                 // an existing plane from planes structure...
//                                 // Log warning only if we are not using a structureless approach
//                                 // (since it does not require planes).
//                                 LOG_IF(WARNING,
//                                         regular_vio_params_.backend_modality_ !=
//                                             RegularBackendModality::STRUCTURELESS)
//                                     << "We are not receiving planes for the Backend. If planes have "
//                                         "been added to the optimization, we are not removing them.";
//                                 }
//                             }
//                             break;
//                         }
//                     }

//                     // Add odometry factors if they're available and have non-zero precision
//                     if (odometry_body_pose && odom_params_ &&
//                         (odom_params_->betweenRotationPrecision_ > 0.0 ||
//                         odom_params_->betweenTranslationPrecision_ > 0.0)) {
//                         VLOG(1) << "Added external factor between " << last_kf_id_ << " and "
//                                 << curr_kf_id_;
//                         addBetweenFactor(last_kf_id_,
//                                         curr_kf_id_,
//                                         *odometry_body_pose,
//                                         odom_params_->betweenRotationPrecision_,
//                                         odom_params_->betweenTranslationPrecision_);
//                     }
//                     if (odometry_vel && odom_params_ && odom_params_->velocityPrecision_ > 0.0) {
//                         LOG_FIRST_N(WARNING, 1)
//                             << "Using velocity priors from external odometry: "
//                             << "This only works if you have velocity estimates in the world frame! "
//                             << "(not provided by typical odometry sensors)";
//                         addVelocityPrior(
//                             curr_kf_id_, *odometry_vel, odom_params_->velocityPrecision_);
//                     }

//                     ///* OPTIMIZE /////////////////////////////////////////////////
//                     // This lags 1 step behind to mimic hw.
//                     imu_bias_prev_kf_ = imu_bias_lkf_;

//                     VLOG(10) << "Starting optimize...";
//                     bool is_smoother_ok = optimize(timestamp_kf_nsec,
//                                                     curr_kf_id_,
//                                                     backend_params_.numOptimize_,
//                                                     delete_slots);
//                     VLOG(10) << "Finished optimize.";

//                     if (is_smoother_ok) {
//                         // Sanity check: ensure no one is removing planes outside
//                         // updatePlaneEstimates.
//                         CHECK_LE(nr_of_planes_, planes_.size());

//                         // Update estimates of planes, and remove planes that are not in the state.
//                         VLOG(10) << "Starting updatePlaneEstimates...";
//                         updatePlaneEstimates(&planes_);
//                         VLOG(10) << "Finished updatePlaneEstimates.";
//                         nr_of_planes_ = planes_.size();

//                         // Reset list of factors to delete.
//                         // These are the smart factors that have been converted to projection
//                         // factors
//                         // and must be deleted from the factor graph.
//                         delete_slots_of_converted_smart_factors_.resize(0);
//                     }

//                     return is_smoother_ok;

//             // OLD:

//             // self.create_imu_factor()?;

//             // // Original models
//             // let new_state = self.get_predicted_state();

//             // // Move node count forward in time
//             // self.ct_state += 1;

//             // // Append to our node vectors
//             // self.values_new.insert_pose3(
//             //     &Symbol::new(b'x', self.ct_state),
//             //     &new_state.pose
//             // );
//             // self.values_new.insert_vector3(
//             //     &Symbol::new(b'v', self.ct_state),
//             //     &new_state.velocity
//             // );
//             // self.values_new.insert_constant_bias(
//             //     &Symbol::new(b'b', self.ct_state),
//             //     &new_state.bias
//             // );
//             // self.values_initial.insert_pose3(
//             //     &Symbol::new(b'x', self.ct_state),
//             //     &new_state.pose
//             // );
//             // self.values_initial.insert_vector3(
//             //     &Symbol::new(b'v', self.ct_state),
//             //     &new_state.velocity
//             // );
//             // self.values_initial.insert_constant_bias(
//             //     &Symbol::new(b'b', self.ct_state),
//             //     &new_state.bias
//             // );

//             // // Add ct state to map
//             // self.ct_state_lookup.insert(timestamp, self.ct_state);
//             // self.timestamp_lookup.insert(self.ct_state, timestamp);
//         }

//         // OLD:
//         // self.process_smart_features(current_frame);

//         // let optimized_pose = self.optimize();

//         // if optimized_pose {
//         //     // Update frame with optimized values
//         //     // TODO there's probably a better way to clean up all this conversion than this
//         //     let updated_pose: Isometry3<f64> = self.values_initial.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into();
//         //     current_frame.pose = Some(Pose::new_from_isometry(updated_pose));
//         //     let velocity: gtsam::base::vector::Vector3 = self.values_initial.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into();
//         //     let vel_raw = velocity.get_raw();
//         //     current_frame.imu_data.velocity = Some(DVVector3::new_with(vel_raw[0], vel_raw[1], vel_raw[2]));
//         //     let bias_ref = self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap();
//         //     let accel_bias = bias_ref.accel_bias().get_raw();
//         //     let gyro_bias = bias_ref.gyro_bias().get_raw();

//         //     // TODO SOFIYA Should this be set_new_bias?
//         //     current_frame.imu_data.imu_bias = ImuBias {
//         //         bax: accel_bias[0],
//         //         bay: accel_bias[1],
//         //         baz: accel_bias[2],
//         //         bwx: gyro_bias[0],
//         //         bwy: gyro_bias[1],
//         //         bwz: gyro_bias[2]
//         //     };

//         //     println!("!!!!!! FINAL FRAME POSE: {:?}", current_frame.pose);
//         // } else {
//         //     error!("Could not optimize pose!");
//         // }


//         Ok(optimized_pose)
//     }

//     fn add_initials_and_priors(&mut self, timestamp: i64, map: &ReadWriteMap) -> Result<(), Box<dyn std::error::Error>> {
//         let _span = tracy_client::span!("add_initials_and_priors");

//         // Create prior factor and add it to the graph
//         let prior_state = {
//             let map = map.read()?;
//             let first_kf = map.get_keyframe(1);
//             let trans = first_kf.get_pose().translation;
//             let rot = first_kf.get_pose().get_quaternion();
//             // let vel = first_kf.imu_data.velocity.unwrap();
//             let vel = [0.0, 0.0, 0.0];
//             GtsamState {
//                 pose: gtsam::geometry::pose3::Pose3::from_parts(
//                     gtsam::geometry::point3::Point3::new(trans.x, trans.y, trans.z),
//                     gtsam::geometry::rot3::Rot3::from(rot)
//                 ),
//                 velocity: gtsam::base::vector::Vector3::new(vel[0], vel[1], vel[2]),
//                 bias: gtsam::imu::imu_bias::ConstantBias::new(
//                     &gtsam::base::vector::Vector3::new(self.prior_ba[0], self.prior_ba[1], self.prior_ba[2]),
//                     &gtsam::base::vector::Vector3::new(self.prior_bg[0], self.prior_bg[1], self.prior_bg[2])
//                 )
//             }
//         };

//         let pose_noise = gtsam::linear::noise_model::DiagonalNoiseModel::from_sigmas(Vector6::new(
//             self.sigma_prior_rotation[0], self.sigma_prior_rotation[1], self.sigma_prior_rotation[2],
//             self.sigma_prior_translation[0], self.sigma_prior_translation[1], self.sigma_prior_translation[2]
//         ));
//         let v_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(3, self.sigma_velocity);
//         let b_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(6, self.sigma_bias);

//         self.graph_new.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);
//         self.graph_new.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);
//         self.graph_new.add_prior_factor_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);
//         self.graph_main.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);
//         self.graph_main.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);
//         self.graph_main.add_prior_factor_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);

//         // Add initial state to the graph
//         self.values_new.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
//         self.values_new.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
//         self.values_new.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);
//         self.values_initial.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
//         self.values_initial.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
//         self.values_initial.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);

//         // Add ct state to map
//         self.ct_state_lookup.insert(timestamp, self.ct_state);
//         self.timestamp_lookup.insert(self.ct_state, timestamp);

//         // Create GTSAM preintegration parameters for use with Foster's version
//         let mut params = PreintegrationCombinedParams::makesharedu();  // Z-up navigation frame: gravity points along negative Z-axis !!!
//         params.set_params(
//             self.accel_noise_density * self.accel_noise_density, // acc white noise in continuous
//             self.gyro_noise_density * self.gyro_noise_density, // gyro white noise in continuous
//             self.accel_random_walk * self.accel_random_walk, // acc bias in continuous
//             self.gyro_random_walk * self.gyro_random_walk, // gyro bias in continuous
//             0.1, // error committed in integrating position from velocities
//             1e-5 // error in the bias used for preintegration
//         );

//         // Actually create the GTSAM preintegration
//         self.preint_gtsam = PreintegratedCombinedMeasurements::new(params, &prior_state.bias);

//         Ok(())
//     }

//     fn get_predicted_state(&self) -> GtsamState {
//         let _span = tracy_client::span!("get_predicted_state");
//         // This function will get the predicted state based on the IMU measurement

//         // Get the current state (t=k)
//         let state_k = GtsamState {
//             pose: self.values_initial.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into(),
//             velocity: self.values_initial.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into(),
//             bias: self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
//         };

//         debug!("... current state: {:?}", state_k);

//         // From this we should predict where we will be at the next time (t=K+1)
//         let state_k1 = self.preint_gtsam.predict(
//             &gtsam::navigation::navstate::NavState::new(
//                 &state_k.pose,
//                 &state_k.velocity
//             ),
//             &state_k.bias
//         );

//         let predicted = GtsamState {
//             pose: state_k1.get_pose().into(),
//             velocity: state_k1.get_velocity().into(),
//             bias: state_k.bias
//         };

//         println!("... predicted pose: {:?}", predicted.pose);

//         return predicted;
//     }

//     fn create_imu_factor(&mut self) -> Result<(), Box<dyn std::error::Error>> {
//         let imu_factor = CombinedImuFactor::new(
//             &Symbol::new(b'x', self.ct_state),
//             &Symbol::new(b'v', self.ct_state),
//             &Symbol::new(b'x', self.ct_state + 1),
//             &Symbol::new(b'v', self.ct_state + 1),
//             &Symbol::new(b'b', self.ct_state),
//             &Symbol::new(b'b', self.ct_state + 1),
//             & self.preint_gtsam
//         );
//         self.graph_new.add_combined_imu_factor(&imu_factor);
//         self.graph_main.add_combined_imu_factor(&imu_factor);

//         // current_frame.imu_data.imu_preintegrated = Some(other_imu.imu_preintegrated_from_last_kf.clone());
//         // current_frame.imu_data.imu_preintegrated_frame = Some(imu_preintegrated_from_last_frame);
//         // current_frame.imu_data.prev_keyframe = Some(1);

//         // let state = other_imu.predict_state_last_frame(current_frame, previous_frame);

//         Ok(())
//     }

//     fn process_smart_features(&mut self, current_frame: &Frame) {
//         let _span = tracy_client::span!("process_smart_features");

//         // let features: VectorOfPoint2f = frame.features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();
//         let features = current_frame.features.get_all_keypoints();
//         for i in 0..features.len() - 1 {
//             let mp_match = current_frame.mappoint_matches.get(i as usize);
//             if mp_match.is_none() {
//                 continue;
//             }
//             let (mp_id, _is_outlier) = mp_match.unwrap();
//             let (kp, _is_outlier) = current_frame.features.get_keypoint(i as usize);

//             // Check to see if it is already in the graph
//             match self.measurement_smart_lookup_left.get_mut(&mp_id) {
//                 Some(smartfactor) => {
//                     // Insert measurements to a smart factor
//                     smartfactor.add(
//                         & gtsam::geometry::point2::Point2::new(kp.pt().x as f64, kp.pt().y as f64),
//                         &Symbol::new(b'x', self.ct_state)
//                     );
//                     continue;
//                 },
//                 None => {
//                     // If we know it is not in the graph
//                     // Create a smart factor for the new feature
//                     let measurement_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(2, self.sigma_camera);
//                     let k = gtsam::geometry::cal3_s2::Cal3S2::default();

//                     // Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
//                     let sensor_p_body = ImuCalib::new().tbc;

//                     let mut smartfactor_left = gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2::new(
//                         &measurement_noise,
//                         &k,
//                         & sensor_p_body.inverse().into()
//                     );

//                     // Insert measurements to a smart factor
//                     smartfactor_left.add(
//                         & gtsam::geometry::point2::Point2::new(kp.pt().x as f64, kp.pt().y as f64),
//                         &Symbol::new(b'x', self.ct_state)
//                     );

//                     // Add smart factor to FORSTER2 model
//                     self.graph_new.add_smartfactor(&smartfactor_left);
//                     self.graph_main.add_smartfactor(&smartfactor_left);

//                     self.measurement_smart_lookup_left.insert(mp_id, smartfactor_left);
//                 }
//             }
//         }
//     }

//     fn optimize(&mut self) -> bool {
//         let _span = tracy_client::span!("optimize");

//         // Return if not initialized
//         if !self.initialized && self.ct_state < 2 {
//             return false;
//         }

//         // Perform smoothing update
//         self.isam2.update_noresults(& self.graph_new, & self.values_new);
//         self.values_initial = self.isam2.calculate_estimate().into();

//         // Remove the used up nodes
//         self.values_new.clear();

//         // Remove the used up factors
//         self.graph_new.resize(0);

//         self.reset_imu_integration();

//         return true;
//     }

//     fn reset_imu_integration(&mut self) {
//         // Use the optimized bias to reset integration
//         if self.values_initial.exists(&Symbol::new(b'b', self.ct_state)) {
//             self.preint_gtsam.reset_integration_and_set_bias(
//                 & self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
//             );
//         }
//     }







    


//     /// Value adders.
//     /* -------------------------------------------------------------------------- */
//     fn add_state_values(&mut self) { 
//         // const FrameId& frame_id,
//         //                             const TrackerStatusSummary& tracker_status,
//         //                             const gtsam::PreintegrationType& pim,
//         //                             std::optional<gtsam::Pose3> odom_pose,
//         //                             std::optional<gtsam::Vector3> odom_vel) {
//     // NOTE: we use the latest state instead of W_Pose_B_lkf_from_increments_
//     // because that one is generated by chaining relative poses from the
//     // optimization, and might be far from the state estimate of the VIO.
//     // Initializing the smoother_ optimization with W_Pose_B_lkf_from_increments_
//     // would cause crashes because it's different from the latest state in
//     // smoother_.
//     // gtsam::NavState navstate_lkf(W_Pose_B_lkf_from_state_, W_Vel_B_lkf_);
//     // const gtsam::NavState& navstate_k = pim.predict(navstate_lkf, imu_bias_lkf_);
//     // debug_info_.navstate_k_ = navstate_k;

//     // switch (backend_params_.pose_guess_source_) {
//     //     case PoseGuessSource::IMU: {
//     //     self.add_state_valuesFromNavState(frame_id, navstate_k);
//     //     break;
//     //     }
//     //     case PoseGuessSource::MONO: {
//     //     if (tracker_status.kfTrackingStatus_mono_ == TrackingStatus::VALID) {
//     //         gtsam::Pose3 W_Pose_B_k_mono =
//     //             W_Pose_B_lkf_from_state_ * B_Pose_leftCamRect_ *
//     //             tracker_status.lkf_T_k_mono_ * B_Pose_leftCamRect_.inverse();
//     //         gtsam::Point3 W_ScaledTranslation_B_k_mono =
//     //             W_Pose_B_k_mono.translation() *
//     //             backend_params_.mono_translation_scale_factor_;
//     //         self.add_state_values(frame_id,
//     //                     gtsam::Pose3(W_Pose_B_k_mono.rotation(),
//     //                                     W_ScaledTranslation_B_k_mono),
//     //                     navstate_k.velocity(),
//     //                     imu_bias_lkf_);
//     //     } else {
//     //         LOG(WARNING) << "Mono tracking failure... Using IMU for pose guess.";
//     //         self.add_state_valuesFromNavState(frame_id, navstate_k);
//     //     }
//     //     break;
//     //     }
//     //     case PoseGuessSource::STEREO: {
//     //     if (tracker_status.kfTrackingStatus_stereo_ == TrackingStatus::VALID) {
//     //         self.add_state_values(frame_id,
//     //                     W_Pose_B_lkf_from_state_ * B_Pose_leftCamRect_ *
//     //                         tracker_status.lkf_T_k_stereo_ *
//     //                         B_Pose_leftCamRect_.inverse(),
//     //                     navstate_k.velocity(),
//     //                     imu_bias_lkf_);
//     //     } else {
//     //         LOG(WARNING) << "Stereo tracking failure... Using IMU for pose guess.";
//     //         self.add_state_valuesFromNavState(frame_id, navstate_k);
//     //     }
//     //     break;
//     //     }
//     //     case PoseGuessSource::PNP: {
//     //     if (tracker_status.kfTracking_status_pnp_ == TrackingStatus::VALID) {
//     //         self.add_state_values(
//     //             frame_id,
//     //             tracker_status.W_T_k_pnp_ * B_Pose_leftCamRect_.inverse(),
//     //             navstate_k.velocity(),
//     //             imu_bias_lkf_);
//     //     } else {
//     //         LOG(WARNING) << "PnP tracking failure... Using IMU for pose guess.";
//     //         self.add_state_valuesFromNavState(frame_id, navstate_k);
//     //     }
//     //     break;
//     //     }
//     //     case PoseGuessSource::EXTERNAL_ODOM: {
//     //     if (odom_pose) {
//     //         // odom_pose is relative (body_lkf_odomPose_body_kf)
//     //         gtsam::Pose3 W_Pose_B_odom =
//     //             W_Pose_B_lkf_from_state_ * odom_pose.value();
//     //         if (odom_vel && odom_params_->velocityPrecision_ > 0.0) {
//     //         LOG(ERROR) << "Using external odometry velocity is not "
//     //                         "recommended! Set odomVelPrecision = 0. Ignore this "
//     //                         "only after serious consideration.";
//     //         self.add_state_values(
//     //             frame_id, W_Pose_B_odom, odom_vel.value(), imu_bias_lkf_);
//     //         } else {
//     //         self.add_state_values(
//     //             frame_id, W_Pose_B_odom, navstate_k.velocity(), imu_bias_lkf_);
//     //         }
//     //     } else {
//     //         LOG(WARNING) << "External odometry tracking failure (no odom pose "
//     //                         "provided)... Using IMU for pose guess.";
//     //         self.add_state_valuesFromNavState(frame_id, navstate_k);
//     //     }
//     //     break;
//     //     }
//     //     default: {
//     //     LOG(FATAL) << "Unrecognized Initial Pose Guess source: "
//     //                 << VIO::to_underlying(backend_params_.pose_guess_source_);
//     //     break;
//     //     }
//     // }
//     }

//     fn add_state_values_from_navstate() {
//     //     const FrameId& frame_id,
//     //                                             const gtsam::NavState& nav_state) {
//     // self.add_state_values(
//     //     frame_id, nav_state.pose(), nav_state.velocity(), imu_bias_lkf_);
//     }

//     fn add_state_values_internal() {
//     //     const FrameId& cur_id,
//     //                                 const gtsam::Pose3& pose,
//     //                                 const gtsam::Velocity3& velocity,
//     //                                 const ImuBias& imu_bias) {
//     // new_values_.insert(gtsam::Symbol(kPoseSymbolChar, cur_id), pose);
//     // new_values_.insert(gtsam::Symbol(kVelocitySymbolChar, cur_id), velocity);
//     // new_values_.insert(gtsam::Symbol(kImuBiasSymbolChar, cur_id), imu_bias);
//     }



//     /// Factor adders.
//     /* -------------------------------------------------------------------------- */
//     fn add_imu_factor(&mut self) {
//         //     const FrameId& from_id,
//         //                             const FrameId& to_id,
//         //                             const gtsam::PreintegrationType& pim) {
//         // switch (imu_params_.imu_preintegration_type_) {
//         //     case ImuPreintegrationType::kPreintegratedCombinedMeasurements: {
//         //     new_imu_prior_and_other_factors_.emplace_shared<gtsam::CombinedImuFactor>(
//         //         gtsam::Symbol(kPoseSymbolChar, from_id),
//         //         gtsam::Symbol(kVelocitySymbolChar, from_id),
//         //         gtsam::Symbol(kPoseSymbolChar, to_id),
//         //         gtsam::Symbol(kVelocitySymbolChar, to_id),
//         //         gtsam::Symbol(kImuBiasSymbolChar, from_id),
//         //         gtsam::Symbol(kImuBiasSymbolChar, to_id),
//         //         safeCastToPreintegratedCombinedImuMeasurements(pim));
//         //     break;
//         //     }
//         //     case ImuPreintegrationType::kPreintegratedImuMeasurements: {
//         //     new_imu_prior_and_other_factors_.emplace_shared<gtsam::ImuFactor>(
//         //         gtsam::Symbol(kPoseSymbolChar, from_id),
//         //         gtsam::Symbol(kVelocitySymbolChar, from_id),
//         //         gtsam::Symbol(kPoseSymbolChar, to_id),
//         //         gtsam::Symbol(kVelocitySymbolChar, to_id),
//         //         gtsam::Symbol(kImuBiasSymbolChar, from_id),
//         //         safeCastToPreintegratedImuMeasurements(pim));

//         //     static const gtsam::imuBias::ConstantBias zero_bias(
//         //         gtsam::Vector3(0.0, 0.0, 0.0), gtsam::Vector3(0.0, 0.0, 0.0));

//         //     // Factor to discretize and move normalize by the interval between
//         //     // measurements:
//         //     CHECK_NE(imu_params_.nominal_sampling_time_s_, 0.0)
//         //         << "Nominal IMU sampling time cannot be 0 s.";
//         //     // See Trawny05 http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf
//         //     // Eq. 130
//         //     const double& sqrt_delta_t_ij = std::sqrt(pim.deltaTij());
//         //     gtsam::Vector6 bias_sigmas;
//         //     bias_sigmas.head<3>().setConstant(sqrt_delta_t_ij *
//         //                                         imu_params_.acc_random_walk_);
//         //     bias_sigmas.tail<3>().setConstant(sqrt_delta_t_ij *
//         //                                         imu_params_.gyro_random_walk_);
//         //     const gtsam::SharedNoiseModel& bias_noise_model =
//         //         gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas);

//         //     new_imu_prior_and_other_factors_
//         //         .emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
//         //             gtsam::Symbol(kImuBiasSymbolChar, from_id),
//         //             gtsam::Symbol(kImuBiasSymbolChar, to_id),
//         //             zero_bias,
//         //             bias_noise_model);
//         //     break;
//         //     }
//         //     default: {
//         //     LOG(FATAL) << "Unknown IMU Preintegration Type.";
//         //     break;
//         //     }
//         // }

//         // debug_info_.imuR_lkf_kf = pim.deltaRij();
//         // debug_info_.numAddedImuF_++;
//     }

//     /* -------------------------------------------------------------------------- */
//     fn add_between_factor(&mut self) {
//         //     const FrameId& from_id,
//         //                                 const FrameId& to_id,
//         //                                 const gtsam::Pose3& from_id_POSE_to_id,
//         //                                 const double& between_rotation_precision,
//         //                                 const double& between_translation_precision) {
//         // // TODO(Toni): make noise models const members of Backend...
//         // Vector6 precisions;
//         // precisions.head<3>().setConstant(between_rotation_precision);
//         // precisions.tail<3>().setConstant(between_translation_precision);
//         // const gtsam::SharedNoiseModel& betweenNoise_ =
//         //     gtsam::noiseModel::Diagonal::Precisions(precisions);

//         // new_imu_prior_and_other_factors_
//         //     .emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
//         //         gtsam::Symbol(kPoseSymbolChar, from_id),
//         //         gtsam::Symbol(kPoseSymbolChar, to_id),
//         //         from_id_POSE_to_id,
//         //         betweenNoise_);

//         // debug_info_.numAddedBetweenStereoF_++;
//     }

//     /* -------------------------------------------------------------------------- */
//     fn add_no_motion_factor(&mut self) {
//         //     const FrameId& from_id,
//         //                                 const FrameId& to_id) {
//         // new_imu_prior_and_other_factors_
//         //     .emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
//         //         gtsam::Symbol(kPoseSymbolChar, from_id),
//         //         gtsam::Symbol(kPoseSymbolChar, to_id),
//         //         gtsam::Pose3(),
//         //         no_motion_prior_noise_);

//         // debug_info_.numAddedNoMotionF_++;

//         // VLOG(10) << "No motion detected, adding no relative motion prior";
//     }

//     /* -------------------------------------------------------------------------- */
//     fn add_zero_velocity_prior() {
//         //     const FrameId& frame_id) {
//         // VLOG(10) << "No motion detected, adding zero velocity prior.";
//         // new_imu_prior_and_other_factors_
//         //     .emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
//         //         gtsam::Symbol(kVelocitySymbolChar, frame_id),
//         //         gtsam::Vector3::Zero(),
//         //         zero_velocity_prior_noise_);
//     }

//     fn add_velocity_prior() {
//         //     const FrameId& frame_id,
//         //                                 const gtsam::Velocity3& vel,
//         //                                 const double& precision) {
//         // VLOG(10) << "Adding odometry pose velocity prior factor.";
//         // gtsam::Vector3 precisions;
//         // precisions.head<3>().setConstant(precision);
//         // const gtsam::SharedNoiseModel& noise_model =
//         //     gtsam::noiseModel::Diagonal::Precisions(precisions);
//         // new_imu_prior_and_other_factors_
//         //     .emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
//         //         gtsam::Symbol(kVelocitySymbolChar, frame_id), vel, noise_model);
//     }


//     fn add_stereo_measurements_to_feature_tracks(&mut self) {
//         // const int& frame_num,
//         // const StereoMeasurements& stereo_meas_kf,
//         // LandmarkIds* landmarks_kf) {
//         // CHECK_NOTNULL(landmarks_kf);

//         // // TODO: feature tracks will grow unbounded.

//         // // Make sure the landmarks_kf vector is empty and has a suitable size.
//         // const size_t& n_stereo_measurements = stereo_meas_kf.size();
//         // landmarks_kf->resize(n_stereo_measurements);

//         // // Store landmark ids.
//         // // TODO(Toni): the concept of feature tracks should not be in the Backend...
//         // for (size_t i = 0u; i < n_stereo_measurements; ++i) {
//         //     const LandmarkId& lmk_id_in_kf_i = stereo_meas_kf[i].first;
//         //     const StereoPoint2& stereo_px_i = stereo_meas_kf[i].second;

//         //     // We filtered invalid lmks in the StereoTracker, so this should not happen.
//         //     CHECK_NE(lmk_id_in_kf_i, -1) << "landmarkId_kf_i == -1?";

//         //     // Thinner structure that only keeps landmarkIds.
//         //     // These landmark ids are only the ones visible in current keyframe,
//         //     // with a valid track...
//         //     // CHECK that we do not have repeated lmk ids!
//         //     DCHECK(std::find(landmarks_kf->begin(),
//         //                     landmarks_kf->end(),
//         //                     lmk_id_in_kf_i) == landmarks_kf->end());
//         //     (*landmarks_kf)[i] = lmk_id_in_kf_i;

//         //     // Add features to vio->featureTracks_ if they are new.
//         //     const FeatureTracks::iterator& feature_track_it =
//         //         feature_tracks_.find(lmk_id_in_kf_i);
//         //     if (feature_track_it == feature_tracks_.end()) {
//         //     // New feature.
//         //     VLOG(20) << "Creating new feature track for lmk: " << lmk_id_in_kf_i
//         //             << '.';
//         //     feature_tracks_.insert(
//         //         std::make_pair(lmk_id_in_kf_i, FeatureTrack(frame_num, stereo_px_i)));
//         //     ++landmark_count_;
//         //     } else {
//         //     // @TODO: It seems that this else condition does not help --
//         //     // conjecture that it creates long feature tracks with low information
//         //     // (i.e. we're not moving)
//         //     // This is problematic in conjunction with our landmark selection
//         //     // mechanism which prioritizes long feature tracks

//         //     // TODO: to avoid making the feature tracks grow unbounded we could
//         //     // use a tmp feature tracks container to which we would add the old
//         //     // feature track plus the new observation on it. (for new tracks, it
//         //     // would be the same as above, using the tmp structure of course).

//         //     // Add observation to existing landmark.
//         //     VLOG(20) << "Updating feature track for lmk: " << lmk_id_in_kf_i << ".";
//         //     feature_track_it->second.obs_.push_back(
//         //         std::make_pair(frame_num, stereo_px_i));

//         //     // TODO(Toni):
//         //     // Mark feature tracks that have been re-observed, so that we can delete
//         //     // the broken feature tracks efficiently.
//         //     }
//         // }
//     }

// }


// #[derive(Debug)]
// struct GtsamState {
//     pub pose: gtsam::geometry::pose3::Pose3,
//     pub velocity: gtsam::base::vector::Vector3,
//     pub bias: gtsam::imu::imu_bias::ConstantBias,
// }
