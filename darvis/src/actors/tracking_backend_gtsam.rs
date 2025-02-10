extern crate g2o;
use log::{warn, info, debug, error};
use nalgebra::{Isometry3, Quaternion, Vector3, Vector6};
use std::collections::{HashMap, VecDeque};
use opencv::{prelude::*, types::{VectorOfKeyPoint, VectorOfMat, VectorOfPoint2f}};
use gtsam::{
    inference::symbol::Symbol, navigation::combined_imu_factor::{CombinedImuFactor, PreintegratedCombinedMeasurements, PreintegrationCombinedParams}, nonlinear::{
        isam2::ISAM2, nonlinear_factor_graph::NonlinearFactorGraph, values::Values
    },
};
use std::fmt::Debug;
use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::messages::{ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg}, map::{features::Features, frame::Frame, keyframe::{KeyFrame, MapPointMatches}, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{good_features_to_track::GoodFeaturesExtractor, image, imu::{ImuBias, ImuCalib, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{FeatureExtractionModule, ImuModule}, optimizer, orbslam_extractor::ORBExtractor, relocalization::Relocalization}, registered_actors::{SHUTDOWN_ACTOR, TRACKING_BACKEND, VISUALIZER}
};

use super::messages::{FeatureTracksAndIMUMsg, TrajectoryMsg, VisTrajectoryMsg};
use crate::registered_actors::IMU;

pub struct TrackingBackendGTSAM {
    system: System,
    map: ReadWriteMap,
    sensor: Sensor,

    // Frames
    last_kf_id: Id,
    current_kf_id: Id,

    // Modules 
    imu: IMU,
    graph_solver: GraphSolver,

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
}

impl Actor for TrackingBackendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let imu = IMU::new();

        let mut actor = TrackingBackendGTSAM {
            system,
            graph_solver: GraphSolver::new(),
            sensor,
            map,
            imu,
            trajectory_poses: Vec::new(),
            last_kf_id: -1,
            current_kf_id: -1,
        };
        tracy_client::set_thread_name!("tracking backend gtsam");

        loop {
            let message = actor.system.receive().unwrap();
            if actor.handle_message(message) {
                break;
            }
            actor.map.match_map_version();
        }
    }

}

impl TrackingBackendGTSAM {
    fn handle_message(&mut self, message: MessageBox) -> bool {
        if message.is::<FeatureTracksAndIMUMsg>() {
            if self.system.queue_full() {
                // Abort additional work if there are too many frames in the msg queue.
                info!("Tracking gtsam dropped 1 frame");
                return false;
            }

            let msg = message.downcast::<FeatureTracksAndIMUMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
            self.handle_regular_message(*msg).unwrap();
        // } else if message.is::<UpdateFrameIMUMsg>() {

        } else if message.is::<ShutdownMsg>() {
            return true;
        } else {
            warn!("Tracking backend received unknown message type!");
        }
        return false;
    }

    fn handle_regular_message(&mut self, mut msg: FeatureTracksAndIMUMsg) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("track");

        if self.last_kf_id == -1 {
            // If this is the first frame, don't do anything
            // If we are receiving it here, it was already put into the map as the latest keyframe
            self.last_kf_id = 1;
        } else {
            // If we have previous frames already, can track normally
            let mut current_frame = msg.frame;

            // if !self.map.read()?.imu_initialized {
            //     let (prior_g, prior_a, fiba) = match self.sensor.frame() {
            //         FrameSensor::Mono => (1e2, 1e10, true),
            //         FrameSensor::Stereo | FrameSensor::Rgbd => (1e2, 1e5, true),
            //     };

            //     self.imu.initialize(&mut self.map, 1, prior_g, prior_a, fiba, Some(self.system.find_actor(TRACKING_BACKEND)))?;
            // }

            // At this point we only have feature matches from front end, not the matches to mappoints. Add the matches here
            self.map_feature_tracks_to_mappoints(&mut current_frame, & msg.mappoint_ids)?;

            // Solve VIO graph. Includes preintegration
            let optimized = self.graph_solver.solve(&mut current_frame, self.last_kf_id, &mut msg.imu_measurements, & self.map)?;
            if !optimized {
                warn!("Could not optimize graph");
            }

            // Create new keyframe! Forget the old frame.
            self.current_kf_id = self.create_new_keyframe(current_frame).unwrap();

            self.update_trajectory_in_logs().expect("Could not save trajectory");
            self.last_kf_id = self.current_kf_id;
        }

        return Ok(());
    }

    fn update_trajectory_in_logs(
        &mut self,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let map = self.map.read()?;
        let curr_kf = map.get_keyframe(self.current_kf_id);
        let last_kf = map.get_keyframe(self.last_kf_id);

        let relative_pose = curr_kf.get_pose() * last_kf.get_pose();

        self.trajectory_poses.push(relative_pose);

        println!("Sanity check... curr kf is {}, ref kf is {}, last kf is {}", curr_kf.id, curr_kf.ref_kf_id.unwrap(), last_kf.id);

        self.system.send(
            SHUTDOWN_ACTOR, 
            Box::new(TrajectoryMsg{
                pose: curr_kf.get_pose().inverse(),
                ref_kf_id: curr_kf.ref_kf_id.unwrap(),
                timestamp: curr_kf.timestamp,
                map_version: self.map.read()?.version
            })
        );

        let map = self.map.read()?;
        let bla = last_kf.get_mp_matches().iter().filter_map(|v| match v { 
           Some((id, _is_outlier)) => Some(*id),
           None => None
        });
        self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
            pose: curr_kf.get_pose(),
            mappoint_matches: vec![],
            nontracked_mappoints: HashMap::new(),
            mappoints_in_tracking: bla.collect(),
            timestamp: curr_kf.timestamp,
            map_version: map.version
        }));

        Ok(())
    }

    fn create_new_keyframe(&mut self, mut current_frame: Frame) -> Result<i32, Box<dyn std::error::Error>>{
        let _span = tracy_client::span!("create_new_keyframe");

        self.imu.imu_preintegrated_from_last_kf = ImuPreIntegrated::new(ImuBias::new());

        debug!("SOFIYA FEATURES. In backend, frame has N {}, features {}, mappoint matches {}", current_frame.features.num_keypoints, current_frame.features.get_all_keypoints().len(), current_frame.mappoint_matches.len(), );

        debug!("SOFIYA FEATURES. After mapping feature tracks, frame has N {}, features {}, mappoint matches {}", current_frame.features.num_keypoints, current_frame.features.get_all_keypoints().len(), current_frame.mappoint_matches.len());

        current_frame.ref_kf_id = Some(self.last_kf_id);

        let kf_id = self.map.write()?.insert_keyframe_to_map(current_frame, false);

        let map = self.map.read()?;
        let curr_kf = map.get_keyframe(kf_id);

        debug!("SOFIYA FEATURES. After inserting keyframe, frame has N {}, features {}, mappoint matches {}", curr_kf.features.num_keypoints, curr_kf.features.get_all_keypoints().len(), curr_kf.get_mp_matches().len());

        tracy_client::Client::running()
        .expect("message! without a running Client")
        .message("create new keyframe", 2);

        // TODO SOFIYA probably want to create new mappoints here somehow

        Ok(kf_id)
    }

    fn map_feature_tracks_to_mappoints(&mut self, current_frame: &mut Frame, tracked_mappoint_ids: & Vec<i32>) -> Result<i32, Box<dyn std::error::Error> > {
        let _span = tracy_client::span!("map_feature_tracks_to_mappoints");

        let map = self.map.read()?;
        let ref_kf = map.get_keyframe(self.last_kf_id);
        let mut added = 0;

        println!("Current frame features: {:?}", current_frame.features.get_all_keypoints().len());
        println!("Tracked mappoint ids: {:?}", tracked_mappoint_ids.len());
        println!("Ref kf mappoint matches: {:?}", ref_kf.get_mp_matches().len());
        // println!("Ref kf matches: {:?}", ref_kf.get_mp_matches());

        for idx1 in 0..current_frame.features.get_all_keypoints().len() - 1 {
            if tracked_mappoint_ids[idx1 as usize] != -1 {
                // Most times the id will be a real mappoint
                // But when new features are extracted for this frame, there is no associated mappoint yet
                // We can create them later when we create the keyframe, but for now ignore.
                current_frame.mappoint_matches.add(idx1 as u32, tracked_mappoint_ids[idx1 as usize], false);
                added += 1;
            }
        }
        println!("Added {} mappoints", added);

        Ok(added)
    }

    // fn create_new_mappoints(&mut self) -> Result<i32, Box<dyn std::error::Error>> {
    //     let _span = tracy_client::span!("create_new_mappoints");

    //     // Retrieve neighbor keyframes in covisibility graph
    //     let nn = match self.sensor.is_mono() {
    //         true => 30,
    //         false => 10
    //     };
    //     let ratio_factor = 1.5 * SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor");
    //     let fpt = SETTINGS.get::<f64>(FEATURE_MATCHER, "far_points_threshold");
    //     let far_points_th = if fpt == 0.0 { INFINITY } else { fpt };

    //     let mut mps_to_insert = vec![];
    //     let mut mps_created = 0;
    //     {
    //         let _span = tracy_client::span!("create_new_mappoints: before loop");
    //         let lock = self.map.read()?;

    //         let current_kf = lock.get_keyframe(self.current_keyframe_id);
    //         let mut neighbor_kfs = current_kf.get_covisibility_keyframes(nn);
    //         if self.sensor.is_imu() {
    //             let mut count = 0;
    //             let mut pkf = current_kf;
    //             while (neighbor_kfs.len() as i32) < nn && pkf.prev_kf_id.is_some() && count < nn {
    //                 let prev_kf = current_kf.prev_kf_id.unwrap();
    //                 if !neighbor_kfs.contains(&prev_kf) {
    //                     neighbor_kfs.push(prev_kf);
    //                 }
    //                 pkf = lock.get_keyframe(prev_kf);
    //                 count += 1;
    //             }
    //         }

    //         let mut pose1 = current_kf.get_pose(); // sophTcw1
    //         let translation1 = pose1.get_translation(); // tcw1
    //         let rotation1 = pose1.get_rotation(); // Rcw1
    //         let rotation_transpose1 = rotation1.transpose(); // Rwc1
    //         let mut ow1 = current_kf.get_camera_center();

    //         drop(_span);
    //         let _span = tracy_client::span!("create_new_mappoints: loop");
    //         // println!("Create_new_mappoints neighbor_kfs: {:?}", neighbor_kfs);

    //         // Search matches with epipolar restriction and triangulate
    //         for neighbor_id in neighbor_kfs {
    //             if self.system.queue_len() > 1 {
    //                 // Abort additional work if there are too many keyframes in the msg queue.
    //                 return Ok(0);
    //             }
    //             let neighbor_kf = lock.get_keyframe(neighbor_id);

    //             // Check first that baseline is not too short
    //             let mut ow2 = neighbor_kf.get_camera_center();
    //             let baseline = (*ow2 - *ow1).norm();
    //             match self.sensor.is_mono() {
    //                 true => {
    //                     let median_depth_neigh = neighbor_kf.compute_scene_median_depth(&lock.mappoints, 2);
    //                     if baseline / median_depth_neigh < 0.01 {
    //                         debug!("Local mapping create new mappoints, continuing bc baseline.. baseline {} median scene depth {}", baseline, median_depth_neigh);
    //                         continue
    //                     }
    //                 },
    //                 false => {
    //                     if baseline < CAMERA_MODULE.stereo_baseline {
    //                         continue
    //                     }
    //                 }
    //             }

    //             // Search matches that fullfil epipolar constraint
    //             let course = match self.sensor.is_imu() {
    //                 true => lock.imu_ba2 && matches!(self.current_tracking_state, TrackingState::RecentlyLost),
    //                 false => false
    //             };

    //             let matches = match FEATURE_MATCHING_MODULE.search_for_triangulation(
    //                 current_kf,
    //                 neighbor_kf,
    //                 false, false, course,
    //                 self.sensor
    //             ) {
    //                 Ok(matches) => matches,
    //                 Err(err) => panic!("Problem with search_for_triangulation {}", err)
    //             };

    //             let mut pose2 = neighbor_kf.get_pose();
    //             let translation2 = pose2.get_translation(); // tcw2
    //             let rotation2 = pose2.get_rotation(); // Rcw2
    //             let rotation_transpose2 = rotation2.transpose(); // Rwc2

    //             // Triangulate each match
    //             for (idx1, idx2) in matches {
    //                 let (kp1, right1, kp2, right2) = {
    //                     let (kp1, right1) = current_kf.features.get_keypoint(idx1);
    //                     let (kp2, right2) = neighbor_kf.features.get_keypoint(idx2);
    //                     (kp1, right1, kp2, right2)
    //                 };

    //                 if right1 {
    //                     let _kp1_ur = current_kf.features.get_mv_right(idx1);
    //                     pose1 = current_kf.get_right_pose();
    //                     ow1 = current_kf.get_right_camera_center();
    //                     // camera1 = mpCurrentKeyFrame->mpCamera2 TODO (STEREO) .. right now just using global CAMERA
    //                 } else {
    //                     pose1 = current_kf.get_pose();
    //                     ow1 = current_kf.get_camera_center();
    //                     // camera1 = mpCurrentKeyFrame->mpCamera TODO (STEREO)
    //                 }
    //                 if right2 {
    //                     let _kp2_ur = neighbor_kf.features.get_mv_right(idx2);
    //                     pose2 = neighbor_kf.get_right_pose();
    //                     ow2 = neighbor_kf.get_right_camera_center();
    //                     // camera2 = neighbor_kf->mpCamera2 TODO (STEREO)
    //                 } else {
    //                     pose2 = neighbor_kf.get_pose();
    //                     ow2 = neighbor_kf.get_camera_center();
    //                     // camera2 = neighbor_kf->mpCamera TODO (STEREO)
    //                 }

    //                 // Check parallax between rays
    //                 let xn1 = CAMERA_MODULE.unproject_eig(&kp1.pt());
    //                 let xn2 = CAMERA_MODULE.unproject_eig(&kp2.pt());
    //                 let ray1 = rotation_transpose1 * (*xn1);
    //                 let ray2 = rotation_transpose2 * (*xn2);
    //                 let cos_parallax_rays = ray1.dot(&ray2) / (ray1.norm() * ray2.norm());
    //                 let (cos_parallax_stereo1, cos_parallax_stereo2) = (cos_parallax_rays + 1.0, cos_parallax_rays + 1.0);
    //                 if right1 {
    //                     todo!("Stereo");
    //                     // cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
    //                 } else if right2 {
    //                     todo!("Stereo");
    //                     //cosParallaxStereo2 = cos(2*atan2(neighbor_kf->mb/2,neighbor_kf->mvDepth[idx2]));
    //                 }
    //                 let cos_parallax_stereo = cos_parallax_stereo1.min(cos_parallax_stereo2);

    //                 let x3_d;
    //                 {
    //                     let good_parallax_with_imu = cos_parallax_rays < 0.9996 && self.sensor.is_imu();
    //                     let good_parallax_wo_imu = cos_parallax_rays < 0.9998 && !self.sensor.is_imu();
    //                     if cos_parallax_rays < cos_parallax_stereo && cos_parallax_rays > 0.0 && (right1 || right2 || good_parallax_with_imu || good_parallax_wo_imu) {
    //                         x3_d = geometric_tools::triangulate(xn1, xn2, pose1, pose2);
    //                     } else if right1 && cos_parallax_stereo1 < cos_parallax_stereo2 {
    //                         x3_d = CAMERA_MODULE.unproject_stereo(lock.get_keyframe(self.current_keyframe_id), idx1);
    //                     } else if right2 && cos_parallax_stereo2 < cos_parallax_stereo1 {
    //                         x3_d = CAMERA_MODULE.unproject_stereo(lock.get_keyframe(neighbor_id), idx2);
    //                     } else {
    //                         continue // No stereo and very low parallax
    //                     }
    //                     if x3_d.is_none() {
    //                         continue
    //                     }
    //                 }


    //                 //Check triangulation in front of cameras
    //                 let x3_d_nalg = *x3_d.unwrap();
    //                 let z1 = rotation1.row(2).transpose().dot(&x3_d_nalg) + (*translation1)[2];
    //                 if z1 <= 0.0 {
    //                     continue;
    //                 }
    //                 let z2 = rotation2.row(2).transpose().dot(&x3_d_nalg) + (*translation2)[2];
    //                 if z2 <= 0.0 {
    //                     continue;
    //                 }

    //                 //Check reprojection error in first keyframe
    //                 let sigma_square1 = LEVEL_SIGMA2[kp1.octave() as usize];
    //                 let x1 = rotation1.row(0).transpose().dot(&x3_d_nalg) + (*translation1)[0];
    //                 let y1 = rotation1.row(1).transpose().dot(&x3_d_nalg) + (*translation1)[1];

    //                 if right1 {
    //                     todo!("Stereo");
    //                     // let invz1 = 1.0 / z1;
    //                     // let u1 = CAMERA_MODULE.fx * x1 * invz1 + CAMERA_MODULE.cx;
    //                     // let u1_r = u1 - CAMERA_MODULE.stereo_baseline_times_fx * invz1;
    //                     // let v1 = CAMERA_MODULE.fy * y1 * invz1 + CAMERA_MODULE.cy;
    //                     // let err_x1 = u1 as f32 - kp1.pt().x;
    //                     // let err_y1 = v1 as f32 - kp1.pt().y;
    //                     // let err_x1_r = u1_r as f32 - kp1_ur.unwrap();
    //                     // if (err_x1 * err_x1  + err_y1 * err_y1 + err_x1_r * err_x1_r) > 7.8 * sigma_square1 {
    //                     //     continue
    //                     // }
    //                 } else {
    //                     let uv1 = CAMERA_MODULE.project(DVVector3::new_with(x1, y1, z1));
    //                     let err_x1 = uv1.0 as f32 - kp1.pt().x;
    //                     let err_y1 = uv1.1 as f32 - kp1.pt().y;
    //                     if (err_x1 * err_x1  + err_y1 * err_y1) > 5.991 * sigma_square1 {
    //                         continue
    //                     }
    //                 }

    //                 //Check reprojection error in second keyframe
    //                 let sigma_square2 = LEVEL_SIGMA2[kp2.octave() as usize];
    //                 let x2 = rotation2.row(0).transpose().dot(&x3_d_nalg) + (*translation2)[0];
    //                 let y2 = rotation2.row(1).transpose().dot(&x3_d_nalg) + (*translation2)[1];

    //                 if right2 {
    //                     todo!("Stereo");
    //                     // let invz2 = 1.0 / z2;
    //                     // let u2 = CAMERA_MODULE.fx * x2 * invz2 + CAMERA_MODULE.cx;// This should be camera2, not camera
    //                     // let u2_r = u2 - CAMERA_MODULE.stereo_baseline_times_fx * invz2;
    //                     // let v2 = CAMERA_MODULE.fy * y2 * invz2 + CAMERA_MODULE.cy;// This should be camera2, not camera
    //                     // let err_x2 = u2 as f32 - kp2.pt().x;
    //                     // let err_y2 = v2 as f32 - kp2.pt().y;
    //                     // let err_x2_r = u2_r as f32 - kp2_ur.unwrap();
    //                     // if (err_x2 * err_x2  + err_y2 * err_y2 + err_x2_r * err_x2_r) > 7.8 * sigma_square2 {
    //                     //     continue
    //                     // }
    //                 } else {
    //                     let uv2 = CAMERA_MODULE.project(DVVector3::new_with(x2, y2, z2));
    //                     let err_x2 = uv2.0 as f32 - kp2.pt().x;
    //                     let err_y2 = uv2.1 as f32 - kp2.pt().y;
    //                     if (err_x2 * err_x2  + err_y2 * err_y2) > 5.991 * sigma_square2 {
    //                         continue
    //                     }
    //                 }

    //                 //Check scale consistency
    //                 let normal1 = *x3_d.unwrap() - *ow1;
    //                 let dist1 = normal1.norm();

    //                 let normal2 = *x3_d.unwrap() - *ow2;
    //                 let dist2 = normal2.norm();

    //                 if dist1 == 0.0 || dist2 == 0.0 {
    //                     continue;
    //                 }

    //                 if dist1 >= far_points_th || dist2 >= far_points_th {
    //                     continue;
    //                 }

    //                 let ratio_dist = dist2 / dist1;
    //                 let ratio_octave = (SCALE_FACTORS[kp1.octave() as usize] / SCALE_FACTORS[kp2.octave() as usize]) as f64;
    //                 if ratio_dist * ratio_factor < ratio_octave || ratio_dist > ratio_octave * ratio_factor {
    //                     continue;
    //                 }

    //                 // Triangulation is successful
    //                 let origin_map_id = lock.id;
    //                 let observations = vec![
    //                     (self.current_keyframe_id, lock.get_keyframe(self.current_keyframe_id).features.num_keypoints, idx1),
    //                     (neighbor_id, neighbor_kf.features.num_keypoints, idx2)
    //                 ];

    //                 mps_to_insert.push((x3_d.unwrap(), self.current_keyframe_id, origin_map_id, observations));

    //             }
    //         }
    //     }

    //     let _span = tracy_client::span!("create_new_mappoints::insert_mappoints");
    //     let mut lock = self.map.write()?;
    //     for (x3_d, ref_kf_id, origin_map_id, observations) in mps_to_insert {
    //         let mp_id = lock.insert_mappoint_to_map(x3_d, ref_kf_id, origin_map_id, observations)
    //         ;
    //         self.recently_added_mappoints.insert(mp_id);
    //         mps_created += 1;
    //     }

    //     Ok(mps_created)
    // }
}

pub struct GraphSolver {
    // New graph
    graph_new: NonlinearFactorGraph, // New factors that have not been optimized yet
    values_new: Values, // New values that have not been optimized yet

    // Main graph
    graph_main: NonlinearFactorGraph, // Main non-linear GTSAM graph, all created factors
    values_initial: Values, // All created nodes

    // Misc GTSAM objects
    isam2: ISAM2, // ISAM2 solvers
    preint_gtsam: PreintegratedCombinedMeasurements, // IMU preintegration

    // IMU preintegration parameters
    // Initialization
    // prior_q_g_to_i: Quaternion<f64>, // prior_qGtoI
    // prior_p_i_in_g: Vector3<f64>, // prior_pIinG
    // prior_v_i_in_g: [f64; 3], // prior_vIinG
    prior_ba: [f64; 3], // prior_ba
    prior_bg: [f64; 3], // prior_bg
    // Noise values from dataset sensor
    accel_noise_density: f64, // accelerometer_noise_density, sigma_a
    gyro_noise_density: f64, // gyroscope_noise_density, sigma_g
    accel_random_walk: f64, // accelerometer_random_walk, sigma_wa
    gyro_random_walk: f64, // gyroscope_random_walk, sigma_wg
    // Noise values for initialization
    sigma_prior_rotation: [f64; 3],
    sigma_prior_translation: [f64; 3],
    sigma_velocity: f64,
    sigma_bias: f64,
    // Misc
    sigma_camera: f64,

    // Iterations of the map
    initialized: bool,
    ct_state: u64,
    ct_state_lookup: HashMap<i64, u64>,
    timestamp_lookup: HashMap<u64, i64>,
    measurement_smart_lookup_left: HashMap<i32, gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2>, // Smart lookup for IMU measurements
}

impl GraphSolver {
    pub fn new() -> Self {
        //           <!-- Rotation and Translation from camera frame to IMU frame -->
        //   <rosparam param="R_C0toI">[0.9999717314190615,    -0.007438121416209933,  0.001100323844221122,
        //                              0.00743200596269379,    0.9999574688631824,    0.005461295826837418,
        //                             -0.0011408988276470173, -0.0054529638303831614, 0.9999844816472552]</rosparam>
        //   <rosparam param="p_IinC0">[-0.03921656415229387,   0.00621263233002485,   0.0012210059575531885]</rosparam>

        //   <!-- Initialization -->
        //   <rosparam param="prior_qGtoI">[0.716147, 0.051158, 0.133778, 0.683096]</rosparam>       
        //   <rosparam param="prior_pIinG">[0.925493, -6.214668, 0.422872]</rosparam>       
        //   <rosparam param="prior_vIinG">[1.128364, -2.280640, 0.213326]</rosparam>       
        //   <rosparam param="prior_ba">[0.00489771688759235973,  0.00800897351104824969, 0.03020588299505782420]</rosparam>       
        //   <rosparam param="prior_bg">[-0.00041196751646696610, 0.00992948457018005999, 0.02188282212555122189]</rosparam>       

        //   <param name="sigma_camera"                 type="double"   value="0.306555403" />
        
        //   <!-- Noise Values for ETH Dataset Sensor -->
        //   <param name="accelerometer_noise_density"  type="double"   value="0.08" />    <!-- sigma_a -->
        //   <param name="gyroscope_noise_density"      type="double"   value="0.004" />   <!-- sigma_g -->
        //   <param name="accelerometer_random_walk"    type="double"   value="0.00004" /> <!-- sigma_wa -->
        //   <param name="gyroscope_random_walk"        type="double"   value="2.0e-6" />  <!-- sigma_wg -->

        //   <!-- Noise Values for Initialization -->
        //   <param name="sigma_prior_rotation"         type="double"   value="0.1" />
        //   <param name="sigma_prior_translation"      type="double"   value="0.3" />
        //   <param name="sigma_velocity"               type="double"   value="0.1" />
        //   <param name="sigma_bias"                   type="double"   value="0.15" />
        //   <param name="sigma_pose_rotation"          type="double"   value="0.1" />
        //   <param name="sigma_pose_translation"       type="double"   value="0.2" />

        Self {
            graph_new: NonlinearFactorGraph::default(),
            values_new: Values::default(),
            graph_main: NonlinearFactorGraph::default(),
            values_initial: Values::default(),
            isam2: ISAM2::default(),
            preint_gtsam: PreintegratedCombinedMeasurements::default(),
            initialized: false,

            // TODO SOFIYA PARAMS
            sigma_prior_rotation: [0.1, 0.1, 0.1],
            sigma_prior_translation: [0.3, 0.3, 0.3],
            sigma_velocity: 0.1,
            sigma_bias: 0.15,
            sigma_camera: 0.306555403,
            accel_noise_density: SETTINGS.get::<f64>(IMU, "noise_acc"),
            gyro_noise_density: SETTINGS.get::<f64>(IMU, "noise_gyro"),
            accel_random_walk: SETTINGS.get::<f64>(IMU, "acc_walk"),
            gyro_random_walk: SETTINGS.get::<f64>(IMU, "gyro_walk"),

            // TODO SOFIYA IS THIS RIGHT?
            // prior_q_g_to_i: Quaternion::new(1.0, 0.0, 0.0, 0.0),
            // prior_p_i_in_g: Vector3::new(0.0, 0.0, 0.0),
            prior_ba: [1e-3, 1e-3, 1e-3],
            prior_bg: [1e-5, 1e-3, 1e-3],
            // prior_v_i_in_g: [0.0 ,0.0, 0.0],

            ct_state: 0,
            ct_state_lookup: HashMap::new(),
            timestamp_lookup: HashMap::new(),
            measurement_smart_lookup_left: HashMap::new(),
        }
    }

    fn solve(&mut self, current_frame: &mut Frame, last_kf_id: Id, imu_measurements: &mut ImuMeasurements, map: &ReadWriteMap) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("solve");

        let timestamp = (current_frame.timestamp * 1e14) as i64; // Convert to int just so we can hash it

        // Return if the node already exists in the graph
        if self.ct_state_lookup.contains_key(&timestamp) {
            println!("NODE WITH TIMESTAMP {} ALREADY EXISTS", timestamp);
            return Ok(false);
        }

        if !self.initialized {
            self.add_initials_and_priors(timestamp, map)?;
            self.initialized = true;
            println!("GTSAM optimize... Added initials and priors");
        } else {
            println!("GTSAM optimize... Regular");
            self.create_imu_factor(imu_measurements, current_frame, last_kf_id, map)?;

            // Original models
            let new_state = self.get_predicted_state();

            // Move node count forward in time
            self.ct_state += 1;

            // Append to our node vectors
            self.values_new.insert_pose3(
                &Symbol::new(b'x', self.ct_state),
                &new_state.pose
            );
            self.values_new.insert_vector3(
                &Symbol::new(b'v', self.ct_state),
                &new_state.velocity
            );
            self.values_new.insert_constant_bias(
                &Symbol::new(b'b', self.ct_state),
                &new_state.bias
            );
            self.values_initial.insert_pose3(
                &Symbol::new(b'x', self.ct_state),
                &new_state.pose
            );
            self.values_initial.insert_vector3(
                &Symbol::new(b'v', self.ct_state),
                &new_state.velocity
            );
            self.values_initial.insert_constant_bias(
                &Symbol::new(b'b', self.ct_state),
                &new_state.bias
            );

            // Add ct state to map
            self.ct_state_lookup.insert(timestamp, self.ct_state);
            self.timestamp_lookup.insert(self.ct_state, timestamp);
        }

        self.process_smart_features(current_frame);

        let optimized_pose = self.optimize();

        if optimized_pose {
            // Update frame with optimized values
            // TODO there's probably a better way to clean up all this conversion than this
            let updated_pose: Isometry3<f64> = self.values_initial.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into();
            current_frame.pose = Some(Pose::new_from_isometry(updated_pose));
            let velocity: gtsam::base::vector::Vector3 = self.values_initial.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into();
            let vel_raw = velocity.get_raw();
            current_frame.imu_data.velocity = Some(DVVector3::new_with(vel_raw[0], vel_raw[1], vel_raw[2]));
            let bias_ref = self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap();
            let accel_bias = bias_ref.accel_bias().get_raw();
            let gyro_bias = bias_ref.gyro_bias().get_raw();

            // TODO SOFIYA Should this be set_new_bias?
            current_frame.imu_data.imu_bias = ImuBias {
                bax: accel_bias[0],
                bay: accel_bias[1],
                baz: accel_bias[2],
                bwx: gyro_bias[0],
                bwy: gyro_bias[1],
                bwz: gyro_bias[2]
            };

            println!("!!!!!! FINAL FRAME POSE: {:?}", current_frame.pose);
        } else {
            error!("Could not optimize pose!");
        }


        Ok(optimized_pose)
    }

    fn add_initials_and_priors(&mut self, timestamp: i64, map: &ReadWriteMap) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("add_initials_and_priors");

        // Create prior factor and add it to the graph
        let prior_state = {
            let map = map.read()?;
            let first_kf = map.get_keyframe(1);
            let trans = first_kf.get_pose().translation;
            let rot = first_kf.get_pose().get_quaternion();
            // let vel = first_kf.imu_data.velocity.unwrap();
            let vel = [0.0, 0.0, 0.0];
            GtsamState {
                pose: gtsam::geometry::pose3::Pose3::from_parts(
                    gtsam::geometry::point3::Point3::new(trans.x, trans.y, trans.z),
                    gtsam::geometry::rot3::Rot3::from(rot)
                ),
                velocity: gtsam::base::vector::Vector3::new(vel[0], vel[1], vel[2]),
                bias: gtsam::imu::imu_bias::ConstantBias::new(
                    &gtsam::base::vector::Vector3::new(self.prior_ba[0], self.prior_ba[1], self.prior_ba[2]),
                    &gtsam::base::vector::Vector3::new(self.prior_bg[0], self.prior_bg[1], self.prior_bg[2])
                )
            }
        };

        let pose_noise = gtsam::linear::noise_model::DiagonalNoiseModel::from_sigmas(Vector6::new(
            self.sigma_prior_rotation[0], self.sigma_prior_rotation[1], self.sigma_prior_rotation[2],
            self.sigma_prior_translation[0], self.sigma_prior_translation[1], self.sigma_prior_translation[2]
        ));
        let v_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(3, self.sigma_velocity);
        let b_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(6, self.sigma_bias);

        self.graph_new.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);
        self.graph_new.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);
        self.graph_new.add_prior_factor_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);
        self.graph_main.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);
        self.graph_main.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);
        self.graph_main.add_prior_factor_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);

        // Add initial state to the graph
        self.values_new.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
        self.values_new.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
        self.values_new.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);
        self.values_initial.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
        self.values_initial.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
        self.values_initial.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);

        // Add ct state to map
        self.ct_state_lookup.insert(timestamp, self.ct_state);
        self.timestamp_lookup.insert(self.ct_state, timestamp);

        // Create GTSAM preintegration parameters for use with Foster's version
        let mut params = PreintegrationCombinedParams::makesharedu();  // Z-up navigation frame: gravity points along negative Z-axis !!!
        params.set_params(
            self.accel_noise_density * self.accel_noise_density, // acc white noise in continuous
            self.gyro_noise_density * self.gyro_noise_density, // gyro white noise in continuous
            self.accel_random_walk * self.accel_random_walk, // acc bias in continuous
            self.gyro_random_walk * self.gyro_random_walk, // gyro bias in continuous
            0.1, // error committed in integrating position from velocities
            1e-5 // error in the bias used for preintegration
        );

        // Actually create the GTSAM preintegration
        self.preint_gtsam = PreintegratedCombinedMeasurements::new(params, &prior_state.bias);

        Ok(())
    }

    fn get_predicted_state(&self) -> GtsamState {
        let _span = tracy_client::span!("get_predicted_state");
        // This function will get the predicted state based on the IMU measurement

        // Get the current state (t=k)
        let state_k = GtsamState {
            pose: self.values_initial.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into(),
            velocity: self.values_initial.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into(),
            bias: self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
        };

        debug!("... current state: {:?}", state_k);

        // From this we should predict where we will be at the next time (t=K+1)
        let state_k1 = self.preint_gtsam.predict(
            &gtsam::navigation::navstate::NavState::new(
                &state_k.pose,
                &state_k.velocity
            ),
            &state_k.bias
        );

        let predicted = GtsamState {
            pose: state_k1.get_pose().into(),
            velocity: state_k1.get_velocity().into(),
            bias: state_k.bias
        };

        println!("... predicted pose: {:?}", predicted.pose);

        return predicted;
    }

    fn create_imu_factor(&mut self, imu_measurements: &mut ImuMeasurements, current_frame: &Frame, last_kf_id: Id, map: &ReadWriteMap) -> Result<(), Box<dyn std::error::Error>> {
        // This function will create a discrete IMU factor using the GTSAM preintegrator class
        // This will integrate from the current state time up to the new update time
        let _span = tracy_client::span!("create_imu_factor");

        let mut imu_from_last_frame = VecDeque::with_capacity(imu_measurements.len()); // mvImuFromLastFrame
        let imu_per = 0.000000000001; // Used to be 0.000000000001 to adjust for different timestamp units, not sure why it needs to be reverted now. 0.001 in orbslam. 

        let prev_ts = {
            let map = map.read()?;
            let previous_kf = map.get_keyframe(last_kf_id);
            previous_kf.timestamp
        };

        while !imu_measurements.is_empty() {
            if imu_measurements.front().unwrap().timestamp < prev_ts - imu_per {
                imu_measurements.pop_front();
            } else if imu_measurements.front().unwrap().timestamp < current_frame.timestamp - imu_per {
                let msmt = imu_measurements.pop_front().unwrap();
                imu_from_last_frame.push_back(msmt);
            } else {
                let msmt = imu_measurements.pop_front().unwrap();
                imu_from_last_frame.push_back(msmt);
                break;
            }
        }
        let n = imu_from_last_frame.len() - 1;

        // let other_imu = IMU::new();
        // let mut imu_preintegrated_from_last_frame = ImuPreIntegrated::new(previous_frame.imu_data.imu_bias);

        for i in 0..n {
            let mut tstep = 0.0;
            let mut acc: Vector3<f64> = Vector3::zeros(); // acc
            let mut ang_vel: Vector3<f64> = Vector3::zeros(); // angVel

            if i == 0 && i < (n - 1) {
                let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                let tini = imu_from_last_frame[i].timestamp - prev_ts;
                acc = (
                    imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
                    (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tini/tab)
                ) * 0.5;
                ang_vel = (
                    imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
                    (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tini/tab)
                ) * 0.5;
                tstep = imu_from_last_frame[i + 1].timestamp - prev_ts;
            } else if i < (n - 1) {
                acc = (imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc) * 0.5;
                ang_vel = (imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel) * 0.5;
                tstep = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
            } else if i > 0 && i == (n - 1) {
                let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                let tend = imu_from_last_frame[i + 1].timestamp - current_frame.timestamp;
                acc = (
                    imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
                    (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tend/tab)
                ) * 0.5;
                ang_vel = (
                    imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
                    (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tend/tab)
                ) * 0.5;
                tstep = current_frame.timestamp - imu_from_last_frame[i].timestamp;
            } else if i == 0 && i == (n - 1) {
                acc = imu_from_last_frame[i].acc;
                ang_vel = imu_from_last_frame[i].ang_vel;
                tstep = current_frame.timestamp - prev_ts;
            }
            tstep = tstep * 1e9;

            self.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);

            // other_imu.imu_preintegrated_from_last_kf.integrate_new_measurement(acc, ang_vel, tstep);

            // imu_preintegrated_from_last_frame.integrate_new_measurement(acc, ang_vel, tstep);
        }

        let imu_factor = CombinedImuFactor::new(
            &Symbol::new(b'x', self.ct_state),
            &Symbol::new(b'v', self.ct_state),
            &Symbol::new(b'x', self.ct_state + 1),
            &Symbol::new(b'v', self.ct_state + 1),
            &Symbol::new(b'b', self.ct_state),
            &Symbol::new(b'b', self.ct_state + 1),
            & self.preint_gtsam
        );
        self.graph_new.add_combined_imu_factor(&imu_factor);
        self.graph_main.add_combined_imu_factor(&imu_factor);

        // current_frame.imu_data.imu_preintegrated = Some(other_imu.imu_preintegrated_from_last_kf.clone());
        // current_frame.imu_data.imu_preintegrated_frame = Some(imu_preintegrated_from_last_frame);
        // current_frame.imu_data.prev_keyframe = Some(1);

        // let state = other_imu.predict_state_last_frame(current_frame, previous_frame);

        Ok(())
    }

    fn process_smart_features(&mut self, current_frame: &Frame) {
        let _span = tracy_client::span!("process_smart_features");

        // let features: VectorOfPoint2f = frame.features.get_all_keypoints().iter().map(|kp| kp.pt()).collect();
        let features = current_frame.features.get_all_keypoints();
        for i in 0..features.len() - 1 {
            let mp_match = current_frame.mappoint_matches.get(i as usize);
            if mp_match.is_none() {
                continue;
            }
            let (mp_id, _is_outlier) = mp_match.unwrap();
            let (kp, _is_outlier) = current_frame.features.get_keypoint(i as usize);

            // Check to see if it is already in the graph
            match self.measurement_smart_lookup_left.get_mut(&mp_id) {
                Some(smartfactor) => {
                    // Insert measurements to a smart factor
                    smartfactor.add(
                        & gtsam::geometry::point2::Point2::new(kp.pt().x as f64, kp.pt().y as f64),
                        &Symbol::new(b'x', self.ct_state)
                    );
                    continue;
                },
                None => {
                    // If we know it is not in the graph
                    // Create a smart factor for the new feature
                    let measurement_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(2, self.sigma_camera);
                    let k = gtsam::geometry::cal3_s2::Cal3S2::default();

                    // Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
                    let sensor_p_body = ImuCalib::new().tbc;

                    let mut smartfactor_left = gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2::new(
                        &measurement_noise,
                        &k,
                        & sensor_p_body.inverse().into()
                    );

                    // Insert measurements to a smart factor
                    smartfactor_left.add(
                        & gtsam::geometry::point2::Point2::new(kp.pt().x as f64, kp.pt().y as f64),
                        &Symbol::new(b'x', self.ct_state)
                    );

                    // Add smart factor to FORSTER2 model
                    self.graph_new.add_smartfactor(&smartfactor_left);
                    self.graph_main.add_smartfactor(&smartfactor_left);

                    self.measurement_smart_lookup_left.insert(mp_id, smartfactor_left);
                }
            }
        }
    }

    fn optimize(&mut self) -> bool {
        let _span = tracy_client::span!("optimize");

        // Return if not initialized
        if !self.initialized && self.ct_state < 2 {
            return false;
        }

        // Perform smoothing update
        self.isam2.update_noresults(& self.graph_new, & self.values_new);
        self.values_initial = self.isam2.calculate_estimate().into();

        // Remove the used up nodes
        self.values_new.clear();

        // Remove the used up factors
        self.graph_new.resize(0);

        self.reset_imu_integration();

        return true;
    }

    fn reset_imu_integration(&mut self) {
        // Use the optimized bias to reset integration
        if self.values_initial.exists(&Symbol::new(b'b', self.ct_state)) {
            self.preint_gtsam.reset_integration_and_set_bias(
                & self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
            );
        }
    }
}


#[derive(Debug)]
struct GtsamState {
    pub pose: gtsam::geometry::pose3::Pose3,
    pub velocity: gtsam::base::vector::Vector3,
    pub bias: gtsam::imu::imu_bias::ConstantBias,
}
