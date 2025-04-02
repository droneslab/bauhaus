use std::cmp::min;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet, VecDeque};
use std::f64::INFINITY;
use std::iter::FromIterator;
use std::sync::atomic::AtomicBool;

use core::system::{Actor, MessageBox};
use core::sensor::{Sensor, FrameSensor, ImuSensor};
use core::{
    config::{SETTINGS, SYSTEM},
    matrix::DVVector3
};
use std::thread::sleep;
use std::time::Duration;
use log::{debug, warn, info};
use opencv::prelude::KeyPointTraitConst;
use crate::map::frame::Frame;
use crate::map::pose::Pose;
use crate::map::read_only_lock::ReadWriteMap;
use crate::modules::local_bundle_adjustment::local_inertial_ba;
use crate::registered_actors::{CAMERA_MODULE, FEATURE_MATCHER, FEATURE_MATCHING_MODULE, LOCAL_MAP_OPTIMIZATION_MODULE, TRACKING_BACKEND};
use crate::System;
use crate::actors::messages::{LastKeyFrameUpdatedMsg, UpdateFrameIMUMsg};
use crate::modules::optimizer::{self, LEVEL_SIGMA2};
use crate::{
    modules::{imu::IMU, orbslam_matcher::SCALE_FACTORS, geometric_tools},
    registered_actors::{FEATURE_DETECTION, LOOP_CLOSING, CAMERA},
    Id,
};
use crate::modules::module_definitions::{CameraModule, FeatureExtractionModule};
use crate::modules::module_definitions::ImuModule;
use super::messages::{InitKeyFrameMsg, InitKeyFrameMsgGTSAM, KeyFrameIdMsg, NewKeyFrameGTSAMMsg, NewKeyFrameMsg, ShutdownMsg};
use super::tracking_backend::{TrackedMapPointData, TrackingState};
use crate::modules::module_definitions::BoWModule;
use crate::actors::tracking_frontend_gtsam::{TrackedFeatures, TrackedFeaturesIndexMap};

// TODO (design, variable locations): It would be nice for this to be a member of LocalMapping instead of floating around in the global namespace, but we can't do that easily because then Tracking would need a reference to the localmapping object.
pub static LOCAL_MAPPING_IDLE: AtomicBool = AtomicBool::new(true);
pub static LOCAL_MAPPING_PAUSE_SWITCH: AtomicBool = AtomicBool::new(false); // sent from loop closing to stop until gba is done, equivalent to calling RequestStop and isStopped

pub struct LocalMappingGTSAM {
    system: System,
    map: ReadWriteMap,
    sensor: Sensor,

    orb_extractor_left: Box<dyn FeatureExtractionModule>,
    current_keyframe_id: Id, //mpCurrentKeyFrame
    current_tracking_state: TrackingState, // mpCurrentKeyFrame->mTrackingState
    recently_added_mappoints: BTreeSet<Id>, //mlpRecentAddedMapPoints

    // list of keyframes to delete sent to map. they might not be deleted until later, so we need to 
    // keep track of them and avoid doing duplicate work with them
    discarded_kfs: HashSet<Id>,  // TODO (design) ... kf culling and rates

    // Modules
    imu_module: Option<IMU>,
}

impl Actor for LocalMappingGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");
        let imu = match sensor.is_imu() {
            true => Some(IMU::new()),
            false => None
        };

        let mut actor = LocalMappingGTSAM {
            system,
            map,
            sensor,
            current_keyframe_id: -1,
            recently_added_mappoints: BTreeSet::new(),
            imu_module: imu,
            discarded_kfs: HashSet::new(),
            current_tracking_state: TrackingState::NotInitialized,
            orb_extractor_left: crate::registered_actors::new_feature_extraction_module(false),
        };

        tracy_client::set_thread_name!("local mapping");

        loop {
            let message = actor.system.receive().unwrap();
            // Actor can early-return if self.map.read() returns an incorrect version of the map
            // see read() in read_only_lock.rs for more info
            if !actor.handle_message(message).is_ok() {
                debug!("LOCAL MAPPING RESET !");
                actor.current_keyframe_id= -1;
                actor.recently_added_mappoints.clear();
                actor.discarded_kfs.clear();
                actor.current_tracking_state = TrackingState::NotInitialized;
                if let Some(ref mut imu) = actor.imu_module {
                    imu.timestamp_init = 0.0;
                }
            }
            actor.map.match_map_version();
            LOCAL_MAPPING_IDLE.store(true, std::sync::atomic::Ordering::SeqCst);
        }
    }
}

impl LocalMappingGTSAM {
    fn handle_message(&mut self, message: MessageBox) -> Result<bool, Box<dyn std::error::Error>> {
        if message.is::<InitKeyFrameMsg>() {
            LOCAL_MAPPING_IDLE.store(false, std::sync::atomic::Ordering::SeqCst);
            let msg = message.downcast::<InitKeyFrameMsg>().unwrap_or_else(| _ | panic !("Could not downcast local mapping message!"));

            // keyframe may have been deleted in map reset by tracking
            if self.map.read()?.has_keyframe(msg.kf_id) {
                self.current_keyframe_id = msg.kf_id
            } else {
                self.current_keyframe_id = -1;
            };

            // Should only happen for the first two keyframes created by the map because that is the only time
            // the keyframe is bringing in mappoints that local mapping hasn't created.
            // TODO (Stereo) ... Tracking can insert new stereo points, so we will also need to add in new points when processing a NewKeyFrameMsg.
            if self.map.read()?.has_keyframe(self.current_keyframe_id) {
                // If because of timing with map reset of tracking, tracking could have deleted map in meantime
                // Really need to think through this because otherwise we will have to check everywhere
                self.recently_added_mappoints.extend(
                    self.map.read()?
                    .get_keyframe(self.current_keyframe_id)
                    .get_mp_matches().iter()
                    .filter(|item| item.is_some())
                    .map(|item| item.unwrap().0)
                    .collect::<Vec<Id>>()
                );
                self.local_mapping(0, HashMap::new())?;
            }

        } else if message.is::<NewKeyFrameGTSAMMsg>() {
            if LOCAL_MAPPING_PAUSE_SWITCH.load(std::sync::atomic::Ordering::SeqCst) {
                LOCAL_MAPPING_IDLE.store(true, std::sync::atomic::Ordering::SeqCst);
                tracy_client::Client::running()
                    .expect("message! without a running Client")
                    .message("local mapping received paused, set idle", 2);
                return Ok(false);
            }

            LOCAL_MAPPING_IDLE.store(false, std::sync::atomic::Ordering::SeqCst);
            if self.system.queue_full() {
                // Abort additional work if there are too many keyframes in the msg queue.
                info!("Local mapping dropped 1 keyframe");
                return Ok(false);
            }

            let mut msg = message.downcast::<NewKeyFrameGTSAMMsg>().unwrap_or_else(|_| panic!("Could not downcast local mapping message!"));
            let track_in_view = self.create_features_and_mappoint_matches_for_keyframe(&mut msg.keyframe).unwrap();
            let kf_id = self.map.write()?.insert_keyframe_to_map(msg.keyframe, false);

            info!("Local mapping working on keyframe {}. Queue length: {}", kf_id, self.system.queue_len());

            self.current_keyframe_id = kf_id;
            self.current_tracking_state = msg.tracking_state;
            self.local_mapping(0, track_in_view)?;

        } else if message.is::<ShutdownMsg>() {
            // Sleep a little to allow other threads to finish
            sleep(Duration::from_millis(100));
            return Ok(true);
        } else {
            warn!("Local Mapping received unknown message type!");
        }

        Ok(false)
    }

    fn create_features_and_mappoint_matches_for_keyframe(&mut self, frame: &mut Frame) -> Result<HashMap<Id, f64>, Box<dyn std::error::Error>> {
        // Feature extraction
        let (keypoints, descriptors) = self.orb_extractor_left.as_mut().extract(& frame.image.as_ref().unwrap()).unwrap();
        frame.replace_features(keypoints, descriptors)?;
        frame.compute_bow();

        let lock = self.map.read()?;

        // Update local points
        let local_keyframes = {
            let mut local_keyframes: HashSet<Id> = lock.get_keyframe(self.current_keyframe_id).get_covisibility_keyframes(10).into_iter().collect();
            let mut count = 0;
            let mut curr_id = self.current_keyframe_id;
            while count <= 3 {
                if curr_id == 0 {
                    break;
                }
                if lock.has_keyframe(curr_id) {
                    local_keyframes.insert(curr_id);
                    count += 1;
                }
                curr_id -= 1;
            }
            local_keyframes
        };
        // println!("Local keyframes: {:?}", local_keyframes);
        let mut local_mappoints = BTreeSet::new();
        for kf_id in local_keyframes.iter() {
            let kf = lock.get_keyframe(*kf_id);
            let mp_ids = kf.get_mp_matches();
            for item in mp_ids {
                if let Some((mp_id, _)) = item {
                    local_mappoints.insert(*mp_id);
                }
            }
        }

        let mut track_in_view = HashMap::new();
        let mut track_in_view_r = HashMap::new();

        for mp_id in &local_mappoints {
            let mp = lock.mappoints.get(mp_id).unwrap();
            // Project (this fills MapPoint variables for matching)
            let (tracked_data_left, tracked_data_right) = frame.is_in_frustum(mp, 0.5);

            if tracked_data_left.is_some() || tracked_data_right.is_some() {
                lock.mappoints.get(&mp_id).unwrap().increase_visible(1);
            }
            if let Some(d) = tracked_data_left {
                track_in_view.insert(*mp_id, d);
            } else {
                track_in_view.remove(mp_id);
            }
            if let Some(d) = tracked_data_right {
                track_in_view_r.insert(*mp_id, d);
            } else {
                track_in_view_r.remove(mp_id);
            }
        }

        let (_non_tracked_points, matches) = FEATURE_MATCHING_MODULE.search_by_projection(
            frame,
            &mut local_mappoints,
            6, 0.8,
            &track_in_view, &track_in_view_r,
            &self.map, self.sensor
        )?;

        println!("Track in view: {:?}", track_in_view.len());
        println!("Matches in local mapping: {}/{}.", matches, local_mappoints.len());

        // let last_kf = lock.get_keyframe(self.current_keyframe_id - 1);

        // for (feature_id, idx_in_frame) in curr_kf_features_map {
        //     let index_in_prev_frame = self.prev_kf_features_map.get(feature_id).unwrap();
        //     println!("Index in curr frame: {}, index in prev frame: {}", idx_in_frame, index_in_prev_frame);

        //     let (mp_id, _is_outlier) = last_kf.get_mp_match(&(*index_in_prev_frame as u32)).unwrap();
        //     // frame.mappoint_matches.add(*idx_in_frame as u32, mp_id, false);
        // }
        Ok(track_in_view.iter().map(|(k, v)| (*k, v.track_depth)).collect())
    }


    fn local_mapping(&mut self, matches_in_tracking: i32, tracked_mappoint_depths: HashMap<Id, f64>) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("local_mapping");

        match self.sensor.frame() {
            FrameSensor::Stereo => {
                todo!("Stereo");
                // LocalMapping::ProcessNewKeyFrame pushes to mlpRecentAddedMapPoints
                // those stereo mappoints which were added in tracking. The rest of the function
                // is redundant here because all of it is already computed when inserting a keyframe,
                // but the part about mlpRecentAddedMapPoints needs to be included.
            },
            _ => {}
        }

        // Check recent MapPoints
        let mps_culled = self.mappoint_culling()?;

        // Triangulate new MapPoints
        let mps_created = self.create_new_mappoints()?;

        if self.system.queue_len() < 1 {
            // Abort additional work if there are too many keyframes in the msg queue.
            // Find more matches in neighbor keyframes and fuse point duplications
            self.search_in_neighbors()?;
        }

        // ORBSLAM will abort additional work if there are too many keyframes in the msg queue (CheckNewKeyFrames)
        // Additionally it will abort if a stop or reset is requested (stopRequested)
        let kfs_processed = self.map.read()?.num_keyframes();
        if kfs_processed > 2 && self.system.queue_len() < 1 {
            match self.sensor.is_imu() {
                true => {
                    if self.map.read()?.imu_initialized {
                        let rec_init = {
                            let lock = self.map.read()?;
                            let current_kf = lock.get_keyframe(self.current_keyframe_id);
                            let previous_kf = lock.get_keyframe(current_kf.prev_kf_id.unwrap());
                            let previous_previous_kf = lock.get_keyframe(previous_kf.prev_kf_id.unwrap());
                            let dist = {
                                let first = (*previous_kf.get_camera_center() - *current_kf.get_camera_center()).norm();
                                let second = (*previous_previous_kf.get_camera_center() - *previous_kf.get_camera_center()).norm();
                                first + second
                            };

                            if dist > 0.05 {
                                self.imu_module.as_mut().unwrap().timestamp_init += current_kf.timestamp - previous_kf.timestamp;
                            }
                            if !lock.imu_ba2 {
                                if self.imu_module.as_ref().unwrap().timestamp_init < 10.0 && dist < 0.02 {
                                    warn!("Not enough motion for IMU initialization. Resetting...");
                                    return Ok(());
                                    // todo!("Multi-maps");
                                    // mbResetRequestedActiveMap = true;
                                    // mpMapToReset = mpCurrentKeyFrame->GetMap();
                                    // mbBadImu = true;
                                }
                            }

                            !lock.imu_ba2
                        };
                        let large = matches_in_tracking > 75 && self.sensor.is_mono() || matches_in_tracking > 100 && !self.sensor.is_mono();

                        // Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                        local_inertial_ba(&mut self.map, self.current_keyframe_id, large, rec_init, tracked_mappoint_depths, self.sensor)?;

                    }
                },
                false => {
                    LOCAL_MAP_OPTIMIZATION_MODULE.optimize(&self.map, self.current_keyframe_id)?;
                }
            }
        } else {
            warn!("Not running LBA. Either we just initialized the map or there are too many keyframes in the queue ({})", self.system.queue_len());
        }

        // Initialize IMU
        if self.sensor.is_imu() && !self.map.read()?.imu_initialized {
            let (prior_g, prior_a, fiba) = match self.sensor.frame() {
                FrameSensor::Mono => (1e2, 1e10, true),
                FrameSensor::Stereo | FrameSensor::Rgbd => (1e2, 1e5, true),
            };

            self.imu_module.as_mut().unwrap().initialize(&mut self.map, self.current_keyframe_id, prior_g, prior_a, fiba, Some(self.system.find_actor(TRACKING_BACKEND)))?;
        }

        // Check redundant local Keyframes
        let kfs_culled = self.keyframe_culling()?;
        // let kfs_culled = 0;
        // warn!("SOFIYA TURNED OFF KF CULLING");

        if self.sensor.is_imu() && self.imu_module.as_ref().unwrap().timestamp_init < 50.0 && matches!(self.current_tracking_state, TrackingState::Ok) {
            // Enter here everytime local-mapping is called
            if self.imu_module.as_mut().unwrap().imu_ba1 {
                if self.imu_module.as_ref().unwrap().timestamp_init > 5.0 {
                    debug!("Start VIBA 1");
                    self.imu_module.as_mut().unwrap().imu_ba1 = true;
                    self.imu_module.as_mut().unwrap().initialize(&mut self.map, self.current_keyframe_id, 1.0, 1e5, true, Some(self.system.find_actor(TRACKING_BACKEND)))?;
                }
            } else if !self.map.read()?.imu_ba2 {
                if self.imu_module.as_ref().unwrap().timestamp_init > 15.0 {
                    debug!("Start VIBA 2");
                    self.map.write()?.imu_ba2 = true;
                    self.imu_module.as_mut().unwrap().initialize(&mut self.map, self.current_keyframe_id, 0.0, 0.0, true, Some(self.system.find_actor(TRACKING_BACKEND)))?;
                }
            }

            // scale refinement
            let timestamp_init = self.imu_module.as_ref().unwrap().timestamp_init;
            if self.map.read()?.num_keyframes() <= 200 &&
                (timestamp_init > 25.0 && timestamp_init < 25.5) ||
                (timestamp_init > 35.0 && timestamp_init < 35.5) ||
                (timestamp_init > 45.0 && timestamp_init < 45.5) ||
                (timestamp_init > 55.0 && timestamp_init < 55.5) ||
                (timestamp_init > 65.0 && timestamp_init < 65.5) ||
                (timestamp_init > 75.0 && timestamp_init < 75.5) {
                if self.sensor.is_mono() {
                    self.scale_refinement()?;
                }
            }
        }

        debug!("For keyframe {}, culled {} mappoints, created {} mappoints, culled {} keyframes", self.current_keyframe_id, mps_culled, mps_created, kfs_culled);
        info!("Map has {} keyframes and {} mappoints" , self.map.read()?.num_keyframes(), self.map.read()?.mappoints.len());

        tracy_client::plot!("MAP INFO: KeyFrames", self.map.read()?.num_keyframes() as f64);
        tracy_client::plot!("MAP INFO: MapPoints", self.map.read()?.mappoints.len() as f64);

        self.system.try_send(LOOP_CLOSING, Box::new(KeyFrameIdMsg{ kf_id: self.current_keyframe_id, map_version: self.map.get_version() }));
        Ok(())
    }

    fn mappoint_culling(&mut self) -> Result<i32, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("mappoint_culling");

        let th_obs = match self.sensor.is_mono() {
            true => 2,
            false => 3
        };

        let current_kf_id = self.current_keyframe_id;
        let mut discard_for_found_ratio = 0;
        let mut discard_for_observations = 0;
        let mut erased_from_recently_added = 0;
        let mut deleted = HashSet::new();

        {
            let mut lock = self.map.write()?;
            self.recently_added_mappoints.retain(|&mp_id| {
                if let Some(mappoint) = lock.mappoints.get(&mp_id) {
                    let found_ratio = mappoint.get_found_ratio();
                    if found_ratio < 0.25 {
                        discard_for_found_ratio += 1;
                        lock.discard_mappoint(&mp_id);
                        deleted.insert(mp_id);
                        false
                    } else if current_kf_id - mappoint.first_kf_id >= 2 && mappoint.get_observations().len() <= th_obs {
                        discard_for_observations += 1;
                        lock.discard_mappoint(&mp_id);
                        deleted.insert(mp_id);
                        false 
                    } else if current_kf_id - mappoint.first_kf_id >= 3 {
                        erased_from_recently_added += 1;
                        false // mappoint should not be deleted, but remove from recently_added_mappoints
                    } else {
                        true // mappoint should not be deleted, keep in recently_added_mappoints
                    }
                } else {
                    // mappoint has been deleted (fused or deleted for low observations when removing keyframe)
                    // remove from recently_added_mappoints
                    false
                }
            });
        }

        return Ok(discard_for_observations + discard_for_found_ratio);
    }

    fn create_new_mappoints(&mut self) -> Result<i32, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("create_new_mappoints");

        // Retrieve neighbor keyframes in covisibility graph
        let nn = match self.sensor.is_mono() {
            true => 30,
            false => 10
        };
        let ratio_factor = 1.5 * SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor");
        let fpt = SETTINGS.get::<f64>(FEATURE_MATCHER, "far_points_threshold");
        let far_points_th = if fpt == 0.0 { INFINITY } else { fpt };

        let mut mps_to_insert = vec![];
        let mut mps_created = 0;
        {
            // let _span = tracy_client::span!("create_new_mappoints: before loop");
            let lock = self.map.read()?;

            let current_kf = lock.get_keyframe(self.current_keyframe_id);
            let mut neighbor_kfs = current_kf.get_covisibility_keyframes(nn);
            if self.sensor.is_imu() {
                let mut count = 0;
                let mut pkf = current_kf;
                while (neighbor_kfs.len() as i32) < nn && pkf.prev_kf_id.is_some() && count < nn {
                    let prev_kf = current_kf.prev_kf_id.unwrap();
                    if !neighbor_kfs.contains(&prev_kf) {
                        neighbor_kfs.push(prev_kf);
                    }
                    pkf = lock.get_keyframe(prev_kf);
                    count += 1;
                }
            }

            let mut pose1 = current_kf.get_pose(); // sophTcw1
            let translation1 = pose1.get_translation(); // tcw1
            let rotation1 = pose1.get_rotation(); // Rcw1
            let rotation_transpose1 = rotation1.transpose(); // Rwc1
            let mut ow1 = current_kf.get_camera_center();

            // drop(_span);
            // let _span = tracy_client::span!("create_new_mappoints: loop");
            // debug!("Create_new_mappoints neighbor_kfs: {:?}", neighbor_kfs);

            // Search matches with epipolar restriction and triangulate
            for neighbor_id in neighbor_kfs {
                if self.system.queue_len() > 1 {
                    // Abort additional work if there are too many keyframes in the msg queue.
                    return Ok(0);
                }
                let neighbor_kf = lock.get_keyframe(neighbor_id);

                // Check first that baseline is not too short
                let mut ow2 = neighbor_kf.get_camera_center();
                let baseline = (*ow2 - *ow1).norm();
                match self.sensor.is_mono() {
                    true => {
                        let median_depth_neigh = neighbor_kf.compute_scene_median_depth(&lock.mappoints, 2);
                        if baseline / median_depth_neigh < 0.01 {
                            debug!("Local mapping create new mappoints, continuing bc baseline.. baseline {} median scene depth {}", baseline, median_depth_neigh);
                            continue
                        }
                    },
                    false => {
                        if baseline < CAMERA_MODULE.stereo_baseline {
                            continue
                        }
                    }
                }

                // Search matches that fullfil epipolar constraint
                let course = match self.sensor.is_imu() {
                    true => lock.imu_ba2 && matches!(self.current_tracking_state, TrackingState::RecentlyLost),
                    false => false
                };

                let matches = match FEATURE_MATCHING_MODULE.search_for_triangulation(
                    current_kf,
                    neighbor_kf,
                    false, false, course,
                    self.sensor
                ) {
                    Ok(matches) => matches,
                    Err(err) => panic!("Problem with search_for_triangulation {}", err)
                };

                let mut pose2 = neighbor_kf.get_pose();
                let translation2 = pose2.get_translation(); // tcw2
                let rotation2 = pose2.get_rotation(); // Rcw2
                let rotation_transpose2 = rotation2.transpose(); // Rwc2

                // Triangulate each match
                for (idx1, idx2) in matches {
                    let (kp1, right1, kp2, right2) = {
                        let (kp1, right1) = current_kf.features.get_keypoint(idx1);
                        let (kp2, right2) = neighbor_kf.features.get_keypoint(idx2);
                        (kp1, right1, kp2, right2)
                    };

                    if right1 {
                        let _kp1_ur = current_kf.features.get_mv_right(idx1);
                        pose1 = current_kf.get_right_pose();
                        ow1 = current_kf.get_right_camera_center();
                        // camera1 = mpCurrentKeyFrame->mpCamera2 TODO (STEREO) .. right now just using global CAMERA
                    } else {
                        pose1 = current_kf.get_pose();
                        ow1 = current_kf.get_camera_center();
                        // camera1 = mpCurrentKeyFrame->mpCamera TODO (STEREO)
                    }
                    if right2 {
                        let _kp2_ur = neighbor_kf.features.get_mv_right(idx2);
                        pose2 = neighbor_kf.get_right_pose();
                        ow2 = neighbor_kf.get_right_camera_center();
                        // camera2 = neighbor_kf->mpCamera2 TODO (STEREO)
                    } else {
                        pose2 = neighbor_kf.get_pose();
                        ow2 = neighbor_kf.get_camera_center();
                        // camera2 = neighbor_kf->mpCamera TODO (STEREO)
                    }

                    // Check parallax between rays
                    let xn1 = CAMERA_MODULE.unproject_eig(&kp1.pt());
                    let xn2 = CAMERA_MODULE.unproject_eig(&kp2.pt());
                    let ray1 = rotation_transpose1 * (*xn1);
                    let ray2 = rotation_transpose2 * (*xn2);
                    let cos_parallax_rays = ray1.dot(&ray2) / (ray1.norm() * ray2.norm());
                    let (cos_parallax_stereo1, cos_parallax_stereo2) = (cos_parallax_rays + 1.0, cos_parallax_rays + 1.0);
                    if right1 {
                        todo!("Stereo");
                        // cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
                    } else if right2 {
                        todo!("Stereo");
                        //cosParallaxStereo2 = cos(2*atan2(neighbor_kf->mb/2,neighbor_kf->mvDepth[idx2]));
                    }
                    let cos_parallax_stereo = cos_parallax_stereo1.min(cos_parallax_stereo2);

                    let x3_d;
                    {
                        let good_parallax_with_imu = cos_parallax_rays < 0.9996 && self.sensor.is_imu();
                        let good_parallax_wo_imu = cos_parallax_rays < 0.9998 && !self.sensor.is_imu();
                        if cos_parallax_rays < cos_parallax_stereo && cos_parallax_rays > 0.0 && (right1 || right2 || good_parallax_with_imu || good_parallax_wo_imu) {
                            x3_d = geometric_tools::triangulate(xn1, xn2, pose1, pose2);
                        } else if right1 && cos_parallax_stereo1 < cos_parallax_stereo2 {
                            x3_d = CAMERA_MODULE.unproject_stereo(lock.get_keyframe(self.current_keyframe_id), idx1);
                        } else if right2 && cos_parallax_stereo2 < cos_parallax_stereo1 {
                            x3_d = CAMERA_MODULE.unproject_stereo(lock.get_keyframe(neighbor_id), idx2);
                        } else {
                            continue // No stereo and very low parallax
                        }
                        if x3_d.is_none() {
                            continue
                        }
                    }


                    //Check triangulation in front of cameras
                    let x3_d_nalg = *x3_d.unwrap();
                    let z1 = rotation1.row(2).transpose().dot(&x3_d_nalg) + (*translation1)[2];
                    if z1 <= 0.0 {
                        continue;
                    }
                    let z2 = rotation2.row(2).transpose().dot(&x3_d_nalg) + (*translation2)[2];
                    if z2 <= 0.0 {
                        continue;
                    }

                    //Check reprojection error in first keyframe
                    let sigma_square1 = LEVEL_SIGMA2[kp1.octave() as usize];
                    let x1 = rotation1.row(0).transpose().dot(&x3_d_nalg) + (*translation1)[0];
                    let y1 = rotation1.row(1).transpose().dot(&x3_d_nalg) + (*translation1)[1];

                    if right1 {
                        todo!("Stereo");
                        // let invz1 = 1.0 / z1;
                        // let u1 = CAMERA_MODULE.fx * x1 * invz1 + CAMERA_MODULE.cx;
                        // let u1_r = u1 - CAMERA_MODULE.stereo_baseline_times_fx * invz1;
                        // let v1 = CAMERA_MODULE.fy * y1 * invz1 + CAMERA_MODULE.cy;
                        // let err_x1 = u1 as f32 - kp1.pt().x;
                        // let err_y1 = v1 as f32 - kp1.pt().y;
                        // let err_x1_r = u1_r as f32 - kp1_ur.unwrap();
                        // if (err_x1 * err_x1  + err_y1 * err_y1 + err_x1_r * err_x1_r) > 7.8 * sigma_square1 {
                        //     continue
                        // }
                    } else {
                        let uv1 = CAMERA_MODULE.project(DVVector3::new_with(x1, y1, z1));
                        let err_x1 = uv1.0 as f32 - kp1.pt().x;
                        let err_y1 = uv1.1 as f32 - kp1.pt().y;
                        if (err_x1 * err_x1  + err_y1 * err_y1) > 5.991 * sigma_square1 {
                            continue
                        }
                    }

                    //Check reprojection error in second keyframe
                    let sigma_square2 = LEVEL_SIGMA2[kp2.octave() as usize];
                    let x2 = rotation2.row(0).transpose().dot(&x3_d_nalg) + (*translation2)[0];
                    let y2 = rotation2.row(1).transpose().dot(&x3_d_nalg) + (*translation2)[1];

                    if right2 {
                        todo!("Stereo");
                        // let invz2 = 1.0 / z2;
                        // let u2 = CAMERA_MODULE.fx * x2 * invz2 + CAMERA_MODULE.cx;// This should be camera2, not camera
                        // let u2_r = u2 - CAMERA_MODULE.stereo_baseline_times_fx * invz2;
                        // let v2 = CAMERA_MODULE.fy * y2 * invz2 + CAMERA_MODULE.cy;// This should be camera2, not camera
                        // let err_x2 = u2 as f32 - kp2.pt().x;
                        // let err_y2 = v2 as f32 - kp2.pt().y;
                        // let err_x2_r = u2_r as f32 - kp2_ur.unwrap();
                        // if (err_x2 * err_x2  + err_y2 * err_y2 + err_x2_r * err_x2_r) > 7.8 * sigma_square2 {
                        //     continue
                        // }
                    } else {
                        let uv2 = CAMERA_MODULE.project(DVVector3::new_with(x2, y2, z2));
                        let err_x2 = uv2.0 as f32 - kp2.pt().x;
                        let err_y2 = uv2.1 as f32 - kp2.pt().y;
                        if (err_x2 * err_x2  + err_y2 * err_y2) > 5.991 * sigma_square2 {
                            continue
                        }
                    }

                    //Check scale consistency
                    let normal1 = *x3_d.unwrap() - *ow1;
                    let dist1 = normal1.norm();

                    let normal2 = *x3_d.unwrap() - *ow2;
                    let dist2 = normal2.norm();

                    if dist1 == 0.0 || dist2 == 0.0 {
                        continue;
                    }

                    if dist1 >= far_points_th || dist2 >= far_points_th {
                        continue;
                    }

                    let ratio_dist = dist2 / dist1;
                    let ratio_octave = (SCALE_FACTORS[kp1.octave() as usize] / SCALE_FACTORS[kp2.octave() as usize]) as f64;
                    if ratio_dist * ratio_factor < ratio_octave || ratio_dist > ratio_octave * ratio_factor {
                        continue;
                    }

                    // Triangulation is successful
                    let origin_map_id = lock.id;
                    let observations = vec![
                        (self.current_keyframe_id, lock.get_keyframe(self.current_keyframe_id).features.num_keypoints, idx1),
                        (neighbor_id, neighbor_kf.features.num_keypoints, idx2)
                    ];

                    mps_to_insert.push((x3_d.unwrap(), self.current_keyframe_id, origin_map_id, observations));

                }
            }
        }

        // let _span = tracy_client::span!("create_new_mappoints::insert_mappoints");
        let mut lock = self.map.write()?;
        for (x3_d, ref_kf_id, origin_map_id, observations) in mps_to_insert {
            let mp_id = lock.insert_mappoint_to_map(x3_d, ref_kf_id, origin_map_id, observations)
            ;
            self.recently_added_mappoints.insert(mp_id);
            mps_created += 1;
        }

        Ok(mps_created)
    }

    fn search_in_neighbors(&self) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("search_in_neighbors");

        // Retrieve neighbor keyframes
        let nn = match self.sensor.frame() {
            FrameSensor::Mono => 30,
            FrameSensor::Stereo | FrameSensor::Rgbd => 10,
        };

        let mut target_kfs;
        let mappoint_matches;
        {
            let map = self.map.read()?;
            let current_kf = map.get_keyframe(self.current_keyframe_id);
            target_kfs = HashSet::<i32>::from_iter(current_kf.get_covisibility_keyframes(nn));

            // Add some covisible of covisible
            // Extend to some second neighbors if abort is not requested
            let mut new_kfs = vec![];
            for kf_id in &target_kfs {
                let kf = map.get_keyframe(*kf_id);
                let covisible = kf.get_covisibility_keyframes(20);
                for kf2_id in covisible {
                    if kf2_id == self.current_keyframe_id || target_kfs.contains(&kf2_id) || new_kfs.contains(&kf2_id) {
                        continue;
                    }
                    new_kfs.push(kf2_id);
                }
            }
            target_kfs.extend(new_kfs);

            if self.system.queue_len() > 1 {
                // Abort additional work if there are too many keyframes in the msg queue.
                return Ok(());
            }

            // Extend to temporal neighbors
            if self.sensor.is_imu() {
                let mut fuse_targets_for_kf: HashMap<Id, Id> = HashMap::new();
                let prev_kf_id = map.get_keyframe(self.current_keyframe_id).prev_kf_id;
                if let Some(mut prev_kf_id) = prev_kf_id {
                    let mut prev_kf = map.get_keyframe(prev_kf_id);
                    while target_kfs.len() < 20 {
                        if fuse_targets_for_kf.contains_key(&prev_kf_id) && *fuse_targets_for_kf.get(&prev_kf_id).unwrap() == self.current_keyframe_id {
                            match prev_kf.prev_kf_id {
                                Some(prev_kf_id2) => {
                                    prev_kf_id = prev_kf_id2;
                                    prev_kf = map.get_keyframe(prev_kf_id);
                                    continue;
                                },
                                None => break
                            }
                        }
                        target_kfs.insert(prev_kf_id);
                        fuse_targets_for_kf.insert(prev_kf_id, self.current_keyframe_id);

                        match prev_kf.prev_kf_id {
                            Some(prev_kf_id2) => {
                                prev_kf_id = prev_kf_id2;
                                prev_kf = map.get_keyframe(prev_kf_id);
                                continue;
                            },
                            None => break
                        }
                    }
                }
            }
            // Clone necessary here so we can use mappoint_matches after the lock is dropped
            // Lock needs to be dropped because orbmatcher::fuse calls write, which causes a deadlock if we keep the read lock
            mappoint_matches = current_kf.get_mp_matches().clone();
        }

        // Search matches by projection from current KF in target KFs
        for kf_id in &target_kfs {
            FEATURE_MATCHING_MODULE.fuse(kf_id, &mappoint_matches, &self.map, 3.0, false)?;
            match self.sensor.frame() {
                FrameSensor::Stereo => FEATURE_MATCHING_MODULE.fuse(kf_id, &mappoint_matches, &self.map, 3.0, true)?,
                _ => {}
            }

        }

        if self.system.queue_len() > 1 {
            // Abort additional work if there are too many keyframes in the msg queue.
            return Ok(());
        }

        // Search matches by projection from target KFs in current KF
        let mut fuse_candidates_set = HashSet::new();
        let mut fuse_candidates_vec = vec![];
        {
            let read = self.map.read()?;
            for kf_id in target_kfs {
                let keyframe = read.get_keyframe(kf_id);
                let mappoints = keyframe.get_mp_matches();
                for mp_match in mappoints {
                    match mp_match {
                        Some((mp_id, is_outlier)) => {
                            if !fuse_candidates_set.contains(&*mp_id) {
                                fuse_candidates_vec.push(Some((*mp_id, *is_outlier)));
                                fuse_candidates_set.insert(*mp_id);
                            }
                        },
                        None => {}
                    }
                }
            }
        }
        FEATURE_MATCHING_MODULE.fuse(&self.current_keyframe_id, &fuse_candidates_vec,  &self.map, 3.0, false)?;
        match self.sensor.frame() {
            FrameSensor::Stereo => FEATURE_MATCHING_MODULE.fuse(&self.current_keyframe_id, &fuse_candidates_vec, &self.map, 3.0, true)?,
            _ => {}
        }

        // Update points
        // Clone needed here for same reason as in map:insert_keyframe_to_map, read explanation there.
        let mappoint_matches = self.map.read()?.get_keyframe(self.current_keyframe_id).get_mp_matches().clone();
        let mut lock = self.map.write()?;
        for mp_match in mappoint_matches {
            match mp_match {
                Some((mp_id, _)) => {
                    lock.update_mappoint(mp_id);
                },
                None => {}
            }
        }
        // Update connections in covisibility graph
        lock.update_connections(self.current_keyframe_id);

        Ok(())
    }

    fn keyframe_culling(&mut self) -> Result<i32, Box<dyn std::error::Error>> {
        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points

        let _span = tracy_client::span!("keyframe_culling");

        //TODO (mvp)... I think we don't need this because the covisibility keyframes struct organizes itself but double check
        // mpCurrentKeyFrame->UpdateBestCovisibles(); 

        let mut to_delete: Vec<(Id, bool)> = vec![]; // Vec<(kf_id, deleted for imu)>
        {
            let read_lock = self.map.read()?;
            let current_kf = read_lock.get_keyframe(self.current_keyframe_id);
            let local_keyframes = current_kf.get_covisibility_keyframes(i32::MAX);

            let redundant_th = match self.sensor {
                Sensor(_, ImuSensor::None) | Sensor(FrameSensor::Mono, _) => 0.9,
                _ => 0.5
            };

            // Compute last KF from optimizable window:
            let last_id = match self.sensor.is_imu() {
                true => {
                    let nd = 21;
                    let mut count = 0;
                    let mut aux_kf = current_kf;
                    while count < nd && aux_kf.prev_kf_id.is_some() {
                        aux_kf = read_lock.get_keyframe(aux_kf.prev_kf_id.unwrap());
                        count += 1;
                    }
                    aux_kf.id
                }
                false => 0
            };

            for i in 0..min(100, local_keyframes.len()) {
                let kf_id = local_keyframes[i];

                if kf_id == read_lock.initial_kf_id {
                    continue
                }

                if kf_id > self.current_keyframe_id - 5 {
                    continue
                }

                let mut num_mps = 0;
                let mut num_redundant_obs = 0;

                let keyframe = read_lock.get_keyframe(kf_id);
                let th_obs = 3;

                for i in 0..keyframe.get_mp_matches().len() {
                    if let Some((mp_id, _is_outlier)) = keyframe.get_mp_match(&(i as u32)) {
                        if !self.sensor.is_mono() {
                            let mv_depth = keyframe.features.get_mv_depth(i as usize).unwrap();
                            let th_depth = SETTINGS.get::<i32>(CAMERA, "thdepth") as f32;
                            if mv_depth > th_depth || mv_depth < 0.0 {
                                continue
                            }
                        }

                        if !read_lock.mappoints.contains_key(&mp_id) {
                            continue
                        }
                        let mp = read_lock.mappoints.get(&mp_id).unwrap();
                        num_mps += 1;

                        if mp.get_observations().len() > th_obs {
                            let scale_level = keyframe.features.get_octave(i as usize);
                            let mut num_obs = 0;
                            for (obs_kf_id, (left_index, right_index)) in mp.get_observations() {
                                if *obs_kf_id == kf_id {
                                    continue
                                }
                                let obs_kf = read_lock.get_keyframe(*obs_kf_id);
                                let scale_level_i = match self.sensor.frame() {
                                    FrameSensor::Stereo => {
                                        let right_level = if *right_index != -1 { obs_kf.features.get_octave(*right_index as usize)} else { -1 };
                                        let left_level = if *left_index != -1 { obs_kf.features.get_octave(*left_index as usize)} else { -1 };
                                        if left_level == -1 || left_level > right_level {
                                            right_level
                                        } else {
                                            left_level
                                        }
                                    },
                                    _ => {
                                        obs_kf.features.get_octave(*left_index as usize)
                                    }
                                };
                                if scale_level_i <= scale_level + 1 {
                                    num_obs += 1;
                                    if num_obs > th_obs { break; }
                                }
                            }

                            if num_obs > th_obs {
                                num_redundant_obs += 1;
                            }
                        }
                    }
                }

                if (num_redundant_obs as f64) > redundant_th * (num_mps as f64) {
                    match self.sensor.is_imu() {
                        true => {
                            // debug!("Keyframe culling, num redundant obs: {}, redudant_th: {}, num_mps: {}", num_redundant_obs, redundant_th, num_mps);
                            if read_lock.num_keyframes() <= 21 {
                                continue;
                            }

                            if kf_id > current_kf.id - 2 {
                                continue;
                            }

                            let kf_prev_id = keyframe.prev_kf_id;
                            let kf_next_id = keyframe.next_kf_id;

                            match (kf_prev_id, kf_next_id) {
                                (Some(prev_kf_id), Some(next_kf_id)) => {
                                    let next_kf = read_lock.get_keyframe(next_kf_id);
                                    let prev_kf = read_lock.get_keyframe(prev_kf_id);
                                    let t = (next_kf.timestamp - prev_kf.timestamp);

                                    if read_lock.imu_initialized && kf_id < last_id && t < 3.0 || t < 0.5 {
                                        self.discarded_kfs.insert(kf_id);
                                        to_delete.push((kf_id, true));
                                        debug!("Discard kf {}, imu #1", kf_id);
                                        debug!("... t: {}, next kf: {}, prev_kf: {}", t, next_kf_id, prev_kf_id);
                                        // patch_imu_data_over_deleted_kf(&self.map, &mut self.discarded_kfs, kf_id, next_kf_id, prev_kf_id);
                                    } else if !read_lock.imu_ba2 {
                                        let kf_imu_pos = *keyframe.get_imu_position();
                                        let prev_kf_imu_pos = *prev_kf.get_imu_position();
                                        if (kf_imu_pos - prev_kf_imu_pos).norm() < 0.02 && t < 3.0 {
                                            to_delete.push((kf_id, true));
                                            self.discarded_kfs.insert(kf_id);
                                            debug!("Discard kf {}, imu #2", kf_id);
                                            debug!("... kf_imu_pos: {:?}, prev_kf_imu_pos: {:?}, norm: {:?}, t: {}", kf_imu_pos, prev_kf_imu_pos, (kf_imu_pos - prev_kf_imu_pos).norm(), t);
                                            // patch_imu_data_over_deleted_kf(&self.map, &mut self.discarded_kfs, kf_id, next_kf_id, prev_kf_id);
                                        }
                                    }
                                },
                                _ => {}
                            }
                        },
                        false => {
                            to_delete.push((kf_id, false));
                            self.discarded_kfs.insert(kf_id);
                        }
                    }
                }
                if i > 20 && self.system.queue_len() > 1 {
                    // Abort additional work if there are too many keyframes in the msg queue.
                    break;
                }
            }
        }

        for (kf_id, deleted_for_imu) in & to_delete {
            if *deleted_for_imu {
                let (kf_imu_preintegrated, next_kf_id, prev_kf_id) = {
                    let lock = self.map.read()?;
                    let kf = lock.get_keyframe(*kf_id);
                    (
                        kf.imu_data.imu_preintegrated.as_ref().unwrap().clone(),
                        kf.next_kf_id.unwrap(),
                        kf.prev_kf_id.unwrap()
                    )
                };
                let mut lock = self.map.write()?;
                {
                    let next_kf = lock.get_keyframe_mut(next_kf_id);
                    next_kf.imu_data.imu_preintegrated.as_mut().unwrap().merge_previous(& kf_imu_preintegrated);
                    next_kf.prev_kf_id = Some(prev_kf_id);
                    let prev_kf = lock.get_keyframe_mut(prev_kf_id);
                    prev_kf.next_kf_id = Some(next_kf_id);
                }
                {
                    let kf = lock.get_keyframe_mut(*kf_id);
                    kf.next_kf_id = None;
                    kf.prev_kf_id = None;
                }
                self.discarded_kfs.insert(*kf_id);

            }
            self.map.write()?.discard_keyframe(*kf_id);
        }

        return Ok(to_delete.len() as i32);
    }

    fn scale_refinement(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Minimum number of keyframes to compute a solution
        // Minimum time (seconds) between first and last keyframe to compute a solution. Make the difference between monocular and stereo
        let _span = tracy_client::span!("imu scale refinement");

        // Retrieve all keyframes in temporal order
        let lock = self.map.read()?;
        let mut kfs: VecDeque<Id> = VecDeque::<Id>::new();
        let mut kf = lock.get_keyframe(self.current_keyframe_id);
        while let Some(prev_kf_id) = kf.prev_kf_id {
            kfs.push_front(kf.id);
            kf = lock.get_keyframe(prev_kf_id);
        }
        kfs.push_front(kf.id);

        // while(CheckNewKeyFrames())
        // {
        //     ProcessNewKeyFrame();
        //     vpKF.push_back(mpCurrentKeyFrame);
        //     lpKF.push_back(mpCurrentKeyFrame);
        // }

        let imu = self.imu_module.as_mut().unwrap();
        imu.rwg = nalgebra::Matrix3::identity();
        imu.scale = 1.0;

        optimizer::inertial_optimization_scale_refinement(&self.map, &mut imu.rwg, &mut imu.scale)?;

        if imu.scale < 1e-1 {
            warn!("Scale too small");
            // bInitializing=false;
            return Ok(());
        }

        // Before this line we are not changing the map
        if (imu.scale - 1.0).abs() > 0.002 || ! self.sensor.is_mono() {
            let tgw = Pose::new_with_default_trans(imu.rwg.transpose());
            self.map.write()?.apply_scaled_rotation(&tgw, imu.scale, true);
            self.system.find_actor(TRACKING_BACKEND).send(Box::new(
                UpdateFrameIMUMsg{
                    scale: imu.scale,
                    imu_bias: kf.imu_data.get_imu_bias().clone(),
                    current_kf_id: self.current_keyframe_id,
                    imu_initialized: true,
                    map_version: self.map.read()?.version
                }
            )).unwrap();
            // mpTracker->UpdateFrameIMU(mScale,mpCurrentKeyFrame->GetImuBias(),mpCurrentKeyFrame);
        }

        // To perform pose-inertial opt w.r.t. last keyframe
        self.map.write()?.map_change_index += 1;
        return Ok(());
    }
 
}
