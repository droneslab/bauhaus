use std::cmp::min;
use std::collections::HashSet;
use std::f64::INFINITY;
use std::iter::FromIterator;
use std::sync::atomic::AtomicBool;

use core::actor::Actor;
use core::sensor::{Sensor, FrameSensor, ImuSensor};
use core::{
    config::{SETTINGS, SYSTEM},
    matrix::DVVector3
};
use log::{debug, warn, info, trace};
use opencv::prelude::KeyPointTraitConst;
use crate::{ActorChannels, MapLock};
use crate::actors::messages::LastKeyFrameUpdatedMsg;
use crate::modules::optimizer::LEVEL_SIGMA2;
use crate::registered_actors::{TRACKING_BACKEND};
use crate::{
    modules::{optimizer, orbmatcher, imu::ImuModule, camera::CAMERA_MODULE, orbmatcher::SCALE_FACTORS, geometric_tools},
    registered_actors::{FEATURE_DETECTION, LOOP_CLOSING, MATCHER, CAMERA},
    Id,
};

use super::messages::{ShutdownMsg, InitKeyFrameMsg, KeyFrameIdMsg, Reset, NewKeyFrameMsg};

// TODO (design): It would be nice for this to be a member of LocalMapping instead of floating around in the global namespace, but we can't do that easily because then Tracking would need a reference to the localmapping object.
pub static LOCAL_MAPPING_IDLE: AtomicBool = AtomicBool::new(true);

#[derive(Debug)]
pub struct LocalMapping {
    actor_channels: ActorChannels,
    map: MapLock,
    sensor: Sensor,

    current_keyframe_id: Id, //mpCurrentKeyFrame
    recently_added_mappoints: HashSet<Id>, //mlpRecentAddedMapPoints

    // list of keyframes to delete sent to map. they might not be deleted until later, so we need to 
    // keep track of them and avoid doing duplicate work with them
    discarded_kfs: HashSet<Id>,  // TODO (design) ... kf culling and rates

    // Modules
    imu: ImuModule,
    
}

impl Actor for LocalMapping {
    type MapRef = MapLock;

    fn new_actorstate(actor_channels: ActorChannels, map: Self::MapRef) -> LocalMapping {
        let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");

        LocalMapping {
            actor_channels,
            map,
            sensor,
            current_keyframe_id: -1,
            recently_added_mappoints: HashSet::new(),
            imu: ImuModule::new(None, None, sensor, false, false),
            discarded_kfs: HashSet::new(),
        }
    }

    fn spawn(actor_channels: ActorChannels, map: Self::MapRef) {
        let mut actor = LocalMapping::new_actorstate(actor_channels, map);
        let max_queue_size = actor.actor_channels.receiver_bound.unwrap_or(100);

        tracy_client::set_thread_name!("local mapping");

        'outer: loop {
            let message = actor.actor_channels.receive().unwrap();

            if message.is::<InitKeyFrameMsg>() {
                LOCAL_MAPPING_IDLE.store(false, std::sync::atomic::Ordering::SeqCst);
                let msg = message.downcast::<InitKeyFrameMsg>().unwrap_or_else(|_| panic!("Could not downcast local mapping message!"));
                actor.current_keyframe_id = msg.kf_id;

                // Should only happen for the first two keyframes created by the map because that is the only time
                // the keyframe is bringing in mappoints that local mapping hasn't created.
                // TODO (Stereo) ... Tracking can insert new stereo points, so we will also need to add in new points when processing a NewKeyFrameMsg.
                actor.recently_added_mappoints.extend(
                    actor.map.read()
                    .keyframes.get(&actor.current_keyframe_id).unwrap()
                    .get_mp_matches().iter()
                    .filter(|item| item.is_some())
                    .map(|item| item.unwrap().0)
                    .collect::<Vec<Id>>()
                );

                actor.local_mapping();
            } else if message.is::<NewKeyFrameMsg>() {
                LOCAL_MAPPING_IDLE.store(false, std::sync::atomic::Ordering::SeqCst);
                if actor.actor_channels.queue_len() > max_queue_size {
                    // Abort additional work if there are too many keyframes in the msg queue.
                    info!("Local mapping dropped 1 keyframe");
                    continue;
                }

                let msg = message.downcast::<NewKeyFrameMsg>().unwrap_or_else(|_| panic!("Could not downcast local mapping message!"));
                let kf_id = actor.map.write().insert_keyframe_to_map(msg.keyframe, false);

                // Send the new keyframe ID directly back to the sender so they can use the ID 
                actor.actor_channels.find(TRACKING_BACKEND).send(Box::new(InitKeyFrameMsg{kf_id})).unwrap();

                debug!("Local mapping working on kf {}", kf_id);

                actor.current_keyframe_id = kf_id;
                actor.local_mapping();
            } else if message.is::<Reset>() {
                // TODO (reset) need to think about how reset requests should be propagated
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Local Mapping received unknown message type!");
            }
            LOCAL_MAPPING_IDLE.store(true, std::sync::atomic::Ordering::SeqCst);
        }
    }
}

impl LocalMapping {
    fn local_mapping(&mut self) {
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
        let mps_culled = self.mappoint_culling();

        // Triangulate new MapPoints
        let mps_created = self.create_new_mappoints();
        self.actor_channels.find(TRACKING_BACKEND).send(Box::new(
            LastKeyFrameUpdatedMsg{}
        )).unwrap();


        if self.actor_channels.queue_len() < 1 {
            // Abort additional work if there are too many keyframes in the msg queue.
            // Find more matches in neighbor keyframes and fuse point duplications
            self.search_in_neighbors();
        }


        let t_init = 0.0;

        // ORBSLAM will abort additional work if there are too many keyframes in the msg queue (CheckNewKeyFrames)
        // Additionally it will abort if a stop or reset is requested (stopRequested)
        let kfs_in_map = self.map.read().keyframes.len();
        if kfs_in_map > 2 && self.actor_channels.queue_len() < 1 {
            match self.sensor.is_imu() {
                true => { // and imu_initialized
                    todo!("IMU");
                    // float dist = (mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()).norm() +
                    //         (mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter()).norm();

                    // if(dist>0.05)
                    //     t_init += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
                    // if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
                    // {
                    //     if((t_init<10.f) && (dist<0.02))
                    //     {
                    //         cout << "Not enough motion for initializing. Reseting..." << endl;
                    //         unique_lock<mutex> lock(mMutexReset);
                    //         mbResetRequestedActiveMap = true;
                    //         mpMapToReset = mpCurrentKeyFrame->GetMap();
                    //         mbBadImu = true;
                    //     }
                    // }

                    // bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);
                    // Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
                },
                false => {
                    optimizer::local_bundle_adjustment(&self.map, self.current_keyframe_id, 0);
                }
            }
        }

        // Initialize IMU
        if self.sensor.is_imu() && !self.imu.is_initialized {
            self.imu.initialize();
        }

        // Check redundant local Keyframes
        let kfs_culled = self.keyframe_culling();

        if self.sensor.is_imu() && t_init < 50.0 {
            todo!("IMU");
            // if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK) // Enter here everytime local-mapping is called
            // {
            //     if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
            //         if (t_init>5.0f)
            //         {
            //             cout << "start VIBA 1" << endl;
            //             mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
            //             if (mbMonocular)
            //                 InitializeIMU(1.f, 1e5, true);
            //             else
            //                 InitializeIMU(1.f, 1e5, true);

            //             cout << "end VIBA 1" << endl;
            //         }
            //     }
            //     else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
            //         if (t_init>15.0f){
            //             cout << "start VIBA 2" << endl;
            //             mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
            //             if (mbMonocular)
            //                 InitializeIMU(0.f, 0.f, true);
            //             else
            //                 InitializeIMU(0.f, 0.f, true);

            //             cout << "end VIBA 2" << endl;
            //         }
            //     }

            //     // scale refinement
            //     if (((mpAtlas->KeyFramesInMap())<=200) &&
            //             ((t_init>25.0f && t_init<25.5f)||
            //             (t_init>35.0f && t_init<35.5f)||
            //             (t_init>45.0f && t_init<45.5f)||
            //             (t_init>55.0f && t_init<55.5f)||
            //             (t_init>65.0f && t_init<65.5f)||
            //             (t_init>75.0f && t_init<75.5f))){
            //         if (mbMonocular)
            //             ScaleRefinement();
            //     }
            // }
        }

        debug!("For keyframe {}, culled {} mappoints, created {} mappoints, culled {} keyframes", self.current_keyframe_id, mps_culled, mps_created, kfs_culled);

        tracy_client::plot!("MAP INFO: KeyFrames", self.map.read().keyframes.len() as f64);
        tracy_client::plot!("MAP INFO: MapPoints", self.map.read().mappoints.len() as f64);
        let avg_mappoints = self.map.read().keyframes.iter().map(|(_, kf)| kf.debug_get_mps_count()).sum::<i32>() as f64 / self.map.read().keyframes.len() as f64;
        tracy_client::plot!("MAP INFO: Avg mp matches for kfs", avg_mappoints as f64);
        trace!("MAP INFO:{},{},{}", self.map.read().keyframes.len(), self.map.read().mappoints.len(), avg_mappoints as f64);

        if self.actor_channels.actors.get(LOOP_CLOSING).is_some() {
            // Only send if loop closing is actually running
            let loopclosing = self.actor_channels.find(LOOP_CLOSING);
            loopclosing.send(Box::new(KeyFrameIdMsg{ kf_id: self.current_keyframe_id })).unwrap();
        }
    }

    fn mappoint_culling(&mut self) -> i32 {
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
        self.recently_added_mappoints.retain(|&mp_id| {
            let mut lock = self.map.write();
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
        debug!("Mappoint culling, {} {} {} {}", discard_for_found_ratio, discard_for_observations, erased_from_recently_added, self.recently_added_mappoints.len());
        return discard_for_observations + discard_for_found_ratio;
    }

    fn create_new_mappoints(&mut self) -> i32 {
        let _span = tracy_client::span!("create_new_mappoints");

        // Retrieve neighbor keyframes in covisibility graph
        let nn = match self.sensor.is_mono() {
            true => 30,
            false => 10
        };
        let ratio_factor = 1.5 * SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor");
        let fpt = SETTINGS.get::<f64>(MATCHER, "far_points_threshold");
        let far_points_th = if fpt == 0.0 { INFINITY } else { fpt };

        let (neighbor_kfs, mut pose1, translation1, rotation1, rotation_transpose1, mut ow1);
        {
            let map = self.map.read();
            let current_kf = map.keyframes.get(&self.current_keyframe_id).unwrap();
            neighbor_kfs = current_kf.get_covisibility_keyframes(nn);
            if self.sensor.is_imu() {
                todo!("IMU");
                // KeyFrame* pKF = mpCurrentKeyFrame;
                // int count=0;
                // while((vpNeighKFs.size()<=nn)&&(pKF->mPrevKF)&&(count++<nn))
                // {
                //     vector<KeyFrame*>::iterator it = std::find(vpNeighKFs.begin(), vpNeighKFs.end(), pKF->mPrevKF);
                //     if(it==vpNeighKFs.end())
                //         vpNeighKFs.push_back(pKF->mPrevKF);
                //     pKF = pKF->mPrevKF;
                // }
            }

            pose1 = current_kf.pose; // sophTcw1
            translation1 = pose1.get_translation(); // tcw1
            rotation1 = pose1.get_rotation(); // Rcw1
            rotation_transpose1 = rotation1.transpose(); // Rwc1
            ow1 = current_kf.get_camera_center();
        }

        // Search matches with epipolar restriction and triangulate
        let mut mps_created = 0;
        for neighbor_id in neighbor_kfs {
            if self.actor_channels.queue_len() > 1 {
                // Abort additional work if there are too many keyframes in the msg queue.
                return 0;
            }

            // Check first that baseline is not too short
            let (mut ow2, baseline);
            {
                let lock = self.map.read();
                ow2 = lock.keyframes.get(&neighbor_id).unwrap().get_camera_center();
                baseline = (*ow2 - *ow1).norm();
                match self.sensor.is_mono() {
                    true => {
                        let median_depth_neigh = lock.keyframes.get(&neighbor_id).unwrap().compute_scene_median_depth(&lock.mappoints, 2);
                        if baseline / median_depth_neigh < 0.01 {
                            debug!("Local mapping create new mappoints, continuing bc baseline");
                            continue
                        }
                    },
                    false => {
                        if baseline < CAMERA_MODULE.stereo_baseline {
                            continue
                        }
                    }
                }
            }

            // Search matches that fullfil epipolar constraint
            let course = match self.sensor.is_imu() {
                true => todo!("IMU"), // should be mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && mpTracker->mState==Tracking::RECENTLY_LOST
                false => false
            };

            let (matches, mut pose2, translation2, rotation2, rotation_transpose2);
            {
                let lock = self.map.read();

                matches = match orbmatcher::search_for_triangulation(
                    lock.keyframes.get(&self.current_keyframe_id).unwrap(),
                    lock.keyframes.get(&neighbor_id).unwrap(),
                    false, false, course,
                    self.sensor
                ) {
                    Ok(matches) => matches,
                    Err(err) => panic!("Problem with search_for_triangulation {}", err)
                };

                pose2 = lock.keyframes.get(&neighbor_id).unwrap().pose;
                translation2 = pose2.get_translation(); // tcw2
                rotation2 = pose2.get_rotation(); // Rcw2
                rotation_transpose2 = rotation2.transpose(); // Rwc2
            }

            // debug!("Neighbor {}, Matches: {}", neighbor_id, matches.len());


            // Triangulate each match
            for (idx1, idx2) in matches {
                let (kp1, right1, kp2, right2) = {
                    let lock = self.map.read();
                    let (kp1, right1) = lock.keyframes.get(&self.current_keyframe_id).unwrap().features.get_keypoint(idx1);
                    let (kp2, right2) = lock.keyframes.get(&neighbor_id).unwrap().features.get_keypoint(idx2);
                    (kp1, right1, kp2, right2)
                };

                let (kp1_ur, kp2_ur);
                if right1 {
                    let lock = self.map.read();
                    kp1_ur = lock.keyframes.get(&self.current_keyframe_id).unwrap().features.get_mv_right(idx1);
                    pose1 = lock.keyframes.get(&self.current_keyframe_id).unwrap().get_right_pose();
                    ow1 = lock.keyframes.get(&self.current_keyframe_id).unwrap().get_right_camera_center();
                    // camera1 = mpCurrentKeyFrame->mpCamera2 TODO (STEREO) .. right now just using global CAMERA
                } else {
                    let lock = self.map.read();
                    pose1 = lock.keyframes.get(&self.current_keyframe_id).unwrap().pose;
                    ow1 = lock.keyframes.get(&self.current_keyframe_id).unwrap().get_camera_center();
                    // camera1 = mpCurrentKeyFrame->mpCamera TODO (STEREO)
                }
                if right2 {
                    let lock = self.map.read();
                    kp2_ur = lock.keyframes.get(&neighbor_id).unwrap().features.get_mv_right(idx2);
                    pose2 = lock.keyframes.get(&neighbor_id).unwrap().get_right_pose();
                    ow2 = lock.keyframes.get(&neighbor_id).unwrap().get_right_camera_center();
                    // camera2 = neighbor_kf->mpCamera2 TODO (STEREO)
                } else {
                    let lock = self.map.read();
                    pose2 = lock.keyframes.get(&neighbor_id).unwrap().pose;
                    ow2 = lock.keyframes.get(&neighbor_id).unwrap().get_camera_center();
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
                    let lock = self.map.read();
                    let good_parallax_with_imu = cos_parallax_rays < 0.9996 && self.sensor.is_imu();
                    let good_parallax_wo_imu = cos_parallax_rays < 0.9998 && !self.sensor.is_imu();
                    if cos_parallax_rays < cos_parallax_stereo && cos_parallax_rays > 0.0 && (right1 || right2 || good_parallax_with_imu || good_parallax_wo_imu) {
                        x3_d = geometric_tools::triangulate(xn1, xn2, pose1, pose2);
                    } else if right1 && cos_parallax_stereo1 < cos_parallax_stereo2 {
                        x3_d = CAMERA_MODULE.unproject_stereo(lock.keyframes.get(&self.current_keyframe_id).unwrap(), idx1);
                    } else if right2 && cos_parallax_stereo2 < cos_parallax_stereo1 {
                        x3_d = CAMERA_MODULE.unproject_stereo(lock.keyframes.get(&neighbor_id).unwrap(), idx2);
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
                {
                    let mut lock = self.map.write();
                    let origin_map_id = lock.id;
                    let observations = vec![
                        (self.current_keyframe_id, lock.keyframes.get(&self.current_keyframe_id).unwrap().features.num_keypoints, idx1),
                        (neighbor_id, lock.keyframes.get(&neighbor_id).unwrap().features.num_keypoints, idx2)
                    ];

                    let mp_id = lock.insert_mappoint_to_map(x3_d.unwrap(), self.current_keyframe_id, origin_map_id, observations);
                    // debug!("Add mp {} to kf {} at index {}", mp_id, self.current_keyframe_id, idx1);
                    mps_created += 1;
                    self.recently_added_mappoints.insert(mp_id);
                }

            }
        }
        mps_created
    }

    fn search_in_neighbors(&self) {
        let _span = tracy_client::span!("search_in_neighbors");

        // Retrieve neighbor keyframes
        let nn = match self.sensor.frame() {
            FrameSensor::Mono => 30,
            FrameSensor::Stereo | FrameSensor::Rgbd => 10,
        };

        let mut target_kfs;
        let mappoint_matches;
        {
            let map = self.map.read();
            let current_kf = map.keyframes.get(&self.current_keyframe_id).unwrap();
            target_kfs = HashSet::<i32>::from_iter(current_kf.get_covisibility_keyframes(nn));

            // Add some covisible of covisible
            // Extend to some second neighbors if abort is not requested
            let mut new_kfs = vec![];
            for kf_id in &target_kfs {
                let kf = map.keyframes.get(&kf_id).unwrap();
                let covisible = kf.get_covisibility_keyframes(20);
                for kf2_id in covisible {
                    if kf2_id == self.current_keyframe_id || target_kfs.contains(&kf2_id) || new_kfs.contains(&kf2_id) {
                        continue;
                    }
                    new_kfs.push(kf2_id);
                }
            }
            target_kfs.extend(new_kfs);

            if self.actor_channels.queue_len() > 1 {
                // Abort additional work if there are too many keyframes in the msg queue.
                return;
            }

            // Extend to temporal neighbors
            match self.sensor.is_imu() {
                true => {
                    todo!("IMU");
                    // KeyFrame* pKFi = mpCurrentKeyFrame->mPrevKF;
                    // while(vpTargetKFs.size()<20 && pKFi)
                    // {
                    //     if(pKFi->isBad() || pKFi->mnFuseTargetForKF==mpCurrentKeyFrame->mnId)
                    //     {
                    //         pKFi = pKFi->mPrevKF;
                    //         continue;
                    //     }
                    //     vpTargetKFs.push_back(pKFi);
                    //     pKFi->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
                    //     pKFi = pKFi->mPrevKF;
                    // }
                }
                false => {},
            }
            // Clone necessary here so we can use mappoint_matches after the lock is dropped
            // Lock needs to be dropped because orbmatcher::fuse calls write, which causes a deadlock if we keep the read lock
            mappoint_matches = current_kf.get_mp_matches().clone();
        }

        // Search matches by projection from current KF in target KFs
        for kf_id in &target_kfs {
            orbmatcher::fuse(kf_id, &mappoint_matches, &self.map, 3.0, false);
            match self.sensor.frame() {
                FrameSensor::Stereo => orbmatcher::fuse(kf_id, &mappoint_matches, &self.map, 3.0, true),
                _ => {}
            }

        }

        if self.actor_channels.queue_len() > 1 {
            // Abort additional work if there are too many keyframes in the msg queue.
            return;
        }

        // Search matches by projection from target KFs in current KF
        let mut fuse_candidates_set = HashSet::new();
        let mut fuse_candidates_vec = vec![];
        {
            let read = self.map.read();
            for kf_id in target_kfs {
                let keyframe = read.keyframes.get(&kf_id).unwrap();
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
        orbmatcher::fuse(&self.current_keyframe_id, &fuse_candidates_vec,  &self.map, 3.0, false);
        match self.sensor.frame() {
            FrameSensor::Stereo => orbmatcher::fuse(&self.current_keyframe_id, &fuse_candidates_vec, &self.map, 3.0, true),
            _ => {}
        }

        // Update points
        // Clone needed here for same reason as in map:insert_keyframe_to_map, read explanation there.
        let mappoint_matches = self.map.read().keyframes.get(&self.current_keyframe_id).unwrap().get_mp_matches().clone();
        for mp_match in mappoint_matches {
            match mp_match {
                Some((mp_id, _)) => {
                    self.map.write().update_mappoint(mp_id);
                },
                None => {}
            }
        }
        // vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        // for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
        // {
        //     MapPoint* pMP=vpMapPointMatches[i];
        //     if(pMP)
        //     {
        //         if(!pMP->isBad())
        //         {
        //             pMP->ComputeDistinctiveDescriptors();
        //             pMP->UpdateNormalAndDepth();
        //         }
        //     }
        // }
        // Update connections in covisibility graph
        // mpCurrentKeyFrame->UpdateConnections();

    }

    fn keyframe_culling(&mut self) -> i32 {
        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points
        let _span = tracy_client::span!("keyframe_culling");

        //TODO (mvp)... I think we don't need this because the covisibility keyframes struct organizes itself but double check
        // mpCurrentKeyFrame->UpdateBestCovisibles(); 

        let mut to_delete = vec![];
        {
            let read_lock = self.map.read();
            let current_kf = read_lock.keyframes.get(&self.current_keyframe_id).unwrap();
            let local_keyframes = current_kf.get_covisibility_keyframes(i32::MAX);

            let redundant_th = match self.sensor {
                Sensor(_, ImuSensor::None) | Sensor(FrameSensor::Mono, _) => 0.9,
                _ => 0.5
            };

            // Compute last KF from optimizable window:
            let _last_id = match self.sensor.is_imu() {
                true => {
                    todo!("Stereo");
                    // let nd = 21;

                    // int count = 0;
                    // KeyFrame* aux_KF = mpCurrentKeyFrame;
                    // while(count<Nd && aux_KF->mPrevKF)
                    // {
                    //     aux_KF = aux_KF->mPrevKF;
                    //     count++;
                    // }
                    // last_ID = aux_KF->mnId;
                }
                false => {} // set to 0 when finishing stereo code
            };

            for i in 0..min(100, local_keyframes.len()) {
                let kf_id = local_keyframes[i];

                if kf_id == read_lock.initial_kf_id {
                    continue
                }

                let mut num_mps = 0;
                let mut num_redundant_obs = 0;

                let keyframe = read_lock.keyframes.get(&kf_id).unwrap();
                let th_obs = 3;

                for i in 0..keyframe.get_mp_matches().len() {
                    if !keyframe.has_mp_match(&(i as u32)) {
                        continue;
                    }
                    let mp_id = keyframe.get_mp_match(&(i as u32));
                    if !self.sensor.is_mono() {
                        let mv_depth = keyframe.features.get_mv_depth(i as usize).unwrap();
                        let th_depth = SETTINGS.get::<i32>(CAMERA, "thdepth") as f32;
                        if mv_depth > th_depth || mv_depth < 0.0 {
                            continue
                        }
                    }

                    if !self.map.read().mappoints.contains_key(&mp_id) {
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
                            let obs_kf = read_lock.keyframes.get(obs_kf_id).unwrap();
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

                if (num_redundant_obs as f64) > redundant_th * (num_mps as f64) {
                    match self.sensor.is_imu() {
                        true => {
                            todo!("IMU");
                            // if (mpAtlas->KeyFramesInMap()<=Nd)
                            //     continue;

                            // if(pKF->mnId>(mpCurrentKeyFrame->mnId-2))
                            //     continue;

                            // if(pKF->mPrevKF && pKF->mNextKF)
                            // {
                            //     const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;

                            //     if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
                            //     {
                            //         pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                            //         pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                            //         pKF->mPrevKF->mNextKF = pKF->mNextKF;
                            //         pKF->mNextKF = NULL;
                            //         pKF->mPrevKF = NULL;
                            //         pKF->SetBadFlag();
                            //     }
                            //     else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && ((pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition()).norm()<0.02) && (t<3))
                            //     {
                            //         pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
                            //         pKF->mNextKF->mPrevKF = pKF->mPrevKF;
                            //         pKF->mPrevKF->mNextKF = pKF->mNextKF;
                            //         pKF->mNextKF = NULL;
                            //         pKF->mPrevKF = NULL;
                            //         pKF->SetBadFlag();
                            //     }
                            // }
                        },
                        false => {
                            to_delete.push(kf_id);
                            self.discarded_kfs.insert(kf_id);
                        }
                    }
                }
                if i > 20 && self.actor_channels.queue_len() > 1 {
                    // Abort additional work if there are too many keyframes in the msg queue.
                    break;
                }
            }
        }

        for kf_id in 0..to_delete.len() {
            self.map.write().discard_keyframe(to_delete[kf_id]);
        }

        return to_delete.len() as i32;
    }
}
