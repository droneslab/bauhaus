use std::cmp::min;
use std::collections::HashSet;
use std::f64::INFINITY;
use std::iter::FromIterator;
use std::sync::atomic::AtomicBool;

use dvcore::actor::Actor;
use dvcore::sensor::{Sensor, FrameSensor, ImuSensor};
use dvcore::{
    config::{SETTINGS, SYSTEM},
    matrix::DVVector3
};
use log::{debug, warn, info};
use opencv::prelude::KeyPointTraitConst;
use crate::{ActorChannels, MapLock};
use crate::actors::loop_closing::LoopClosingMsg;
use crate::actors::messages::LastKeyFrameUpdatedMsg;
use crate::modules::optimizer::LEVEL_SIGMA2;
use crate::registered_actors::TRACKING_BACKEND;
use crate::{
    dvmap::mappoint::{MapPoint, PrelimMapPoint},
    modules::{optimizer, orbmatcher, imu::ImuModule, camera::CAMERA_MODULE, orbmatcher::SCALE_FACTORS, geometric_tools},
    registered_actors::{FEATURE_DETECTION, LOOP_CLOSING, MATCHER, CAMERA},
    Id,
};

use super::messages::{ShutdownMsg, KeyFrameIdMsg, Reset, NewKeyFrameMsg};

// TODO (design): It would be nice for this to be a member of LocalMapping instead of floating around in the global namespace, but we can't do that easily because then Tracking would need a reference to the localmapping object.
pub static LOCAL_MAPPING_IDLE: AtomicBool = AtomicBool::new(true);

#[derive(Debug)]
pub struct LocalMapping {
    actor_channels: ActorChannels,
    map: MapLock,
    sensor: Sensor,

    current_keyframe_id: Id, //mpCurrentKeyFrame
    recently_added_mappoints: Vec<Id>, //mlpRecentAddedMapPoints

    // list of keyframes to erase sent to map. they might not be erased until later, so we need to 
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
            recently_added_mappoints: Vec::new(),
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

            if message.is::<KeyFrameIdMsg>() {
                LOCAL_MAPPING_IDLE.store(false, std::sync::atomic::Ordering::SeqCst);
                let msg = message.downcast::<KeyFrameIdMsg>().unwrap_or_else(|_| panic!("Could not downcast local mapping message!"));
                debug!("Local mapping working on kf {}", msg.kf_id);

                actor.current_keyframe_id = msg.kf_id;
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
                actor.actor_channels.find(TRACKING_BACKEND).send(Box::new(KeyFrameIdMsg{kf_id})).unwrap();

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

        self.recently_added_mappoints = self.map.read()
            .keyframes.get(&self.current_keyframe_id).unwrap()
            .mappoint_matches.matches.iter()
            .filter(|item| item.is_some())
            .map(|item| item.unwrap().0)
            .collect::<Vec<Id>>();
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

        let loopclosing = self.actor_channels.find(LOOP_CLOSING);
        loopclosing.send(Box::new(LoopClosingMsg::KeyFrameIdMsg{ kf_id: self.current_keyframe_id })).unwrap();
    }

    fn mappoint_culling(&mut self) -> i32 {
        let _span = tracy_client::span!("mappoint_culling");

        let th_obs = match self.sensor.is_mono() {
            true => 2,
            false => 3
        };

        let current_kf_id = self.current_keyframe_id;
        let mut num_to_discard = 0;
        self.recently_added_mappoints.retain(|&mp_id| {
            let mut lock = self.map.write();
            if let Some(mappoint) = lock.mappoints.get(&mp_id) {
                if (mappoint.get_found_ratio() < 0.25) || (current_kf_id - mappoint.first_kf_id >= 2 && mappoint.get_observations().len() <= th_obs) {
                    num_to_discard += 1;
                    lock.discard_mappoint(&mp_id);
                    false
                } else if current_kf_id - mappoint.first_kf_id >= 3 {
                    false // mappoint should not be deleted, but remove from recently_added_mappoints
                } else {
                    true // mappoint should not be deleted
                }
            } else {
                false // mappoint has already been deleted, remove from recently_added_mappoints
            }
        });
        return num_to_discard;
    }

    fn create_new_mappoints(&self) -> i32 {
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
            neighbor_kfs = current_kf.connections.get_covisibility_keyframes(nn);
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
                    lock.keyframes.get(&self.current_keyframe_id).unwrap(), lock.keyframes.get(&neighbor_id).unwrap(),
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
                    // let u1 = CAMERA_MODULE.get_fx() * x1 * invz1 + CAMERA_MODULE.get_cx();
                    // let u1_r = u1 - CAMERA_MODULE.stereo_baseline_times_fx * invz1;
                    // let v1 = CAMERA_MODULE.get_fy() * y1 * invz1 + CAMERA_MODULE.get_cy();
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
                    // let u2 = CAMERA_MODULE.get_fx() * x2 * invz2 + CAMERA_MODULE.get_cx();// This should be camera2, not camera
                    // let u2_r = u2 - CAMERA_MODULE.stereo_baseline_times_fx * invz2;
                    // let v2 = CAMERA_MODULE.get_fy() * y2 * invz2 + CAMERA_MODULE.get_cy();// This should be camera2, not camera
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
                    let new_mp = MapPoint::<PrelimMapPoint>::new(x3_d.unwrap(), self.current_keyframe_id, lock.id);
                    let observations = vec![(self.current_keyframe_id, lock.keyframes.get(&self.current_keyframe_id).unwrap().features.num_keypoints, idx1), (neighbor_id, lock.keyframes.get(&neighbor_id).unwrap().features.num_keypoints, idx2)];
                    let _id = lock.insert_mappoint_to_map(new_mp, observations);
                    mps_created += 1;
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
            target_kfs = HashSet::<i32>::from_iter(current_kf.connections.get_covisibility_keyframes(nn));

            // Add some covisible of covisible
            // Extend to some second neighbors if abort is not requested
            let new_kfs = target_kfs.iter().map(|kf_id| {
                map.keyframes.get(&kf_id).unwrap().connections.get_covisibility_keyframes(20)
            }).flatten().collect::<Vec<i32>>();
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

            // Search matches by projection from current KF in target KFs
            mappoint_matches = current_kf.mappoint_matches.matches.iter()
                .filter(|item| item.is_some() )
                .map(|item| item.unwrap().0 )
                .collect::<Vec<Id>>();
        }

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


        let fuse_candidates;
        {
            let read = self.map.read();
            // Search matches by projection from target KFs in current KF
            fuse_candidates = HashSet::<Id>::from_iter(
                target_kfs.iter().map(|kf_id| {
                    let kf = read.keyframes.get(&kf_id).unwrap();
                    let mappoints_kf = &kf.mappoint_matches.matches;
                    mappoints_kf
                        .iter()
                        .filter(|item| item.is_some())
                        .map(|item| item.unwrap().0)
                        .collect::<Vec<_>>()
                }).flatten()
            );
        }
        let fuse_candidates_vec = fuse_candidates.iter().map(|mp| *mp).collect::<Vec<Id>>();
        orbmatcher::fuse(&self.current_keyframe_id, &fuse_candidates_vec,  &self.map, 3.0, false);
        match self.sensor.frame() {
            FrameSensor::Stereo => orbmatcher::fuse(&self.current_keyframe_id, &fuse_candidates_vec, &self.map, 3.0, true),
            _ => {}
        }
    }

    fn keyframe_culling(&mut self) -> i32 {
        let _span = tracy_client::span!("keyframe_culling");

        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points

        //TODO (mvp)... I think we don't need this because the covisibility keyframes struct organizes itself but double check
        // mpCurrentKeyFrame->UpdateBestCovisibles(); 
        let mut lock = self.map.write();

        let current_kf = lock.keyframes.get(&self.current_keyframe_id).unwrap();
        let local_keyframes = current_kf.connections.get_covisibility_keyframes(i32::MAX);

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

        let mut num_deleted = 0;
        for i in 0..min(100, local_keyframes.len()) {
            let kf_id = local_keyframes[i];

            if kf_id == lock.initial_kf_id {
                continue
            }
            // } else if kf_id > self.current_keyframe_id {
            //     // Note: When local mapping processes messages at a slower rate than receiving them, can get into a weird state
            //     // where the map has inserted future keyframes before local mapping has had a chance to process them.
            //     // This means that earlier keyframes might have connections in the map to later keyframes that local mapping
            //     // doesn't know about yet. In this case, later keyframes might actually be culled while working on an earlier kf.
            //     // But we don't want to cull them yet, so skip over for now.
            //     // TODO (design) ... kf culling and rates
            //     continue
            // } else if self.discarded_kfs.contains(&kf_id) {
            //     // Note: Sometimes local mapping will send a keyframe for erasure in the map but it is not erased by the time this
            //     // function comes around again. In that case, we should treat the keyframe as discarded even though it isn't yet.
            //     // TODO (design) ... kf culling and rates
            //     continue
            // }

            let mut num_mps = 0;
            let mut num_redundant_obs = 0;

            let keyframe = lock.keyframes.get(&kf_id).unwrap();
            let th_obs = 3;

            for i in 0..keyframe.mappoint_matches.matches.len() {
                if !keyframe.mappoint_matches.has_mappoint(&(i as u32)) {
                    continue;
                }
                let mp_id = keyframe.mappoint_matches.get_mappoint(&(i as u32));
                if !self.sensor.is_mono() {
                    let mv_depth = keyframe.features.get_mv_depth(i as usize).unwrap();
                    let th_depth = SETTINGS.get::<i32>(CAMERA, "thdepth") as f32;
                    if mv_depth > th_depth || mv_depth < 0.0 {
                        continue
                    }
                }

                if !lock.mappoints.contains_key(&mp_id) {
                    continue
                }
                let mp = lock.mappoints.get(&mp_id).unwrap();
                num_mps += 1;

                if mp.get_observations().len() > th_obs {
                    let scale_level = keyframe.features.get_octave(i as usize);
                    let mut num_obs = 0;
                    for (obs_kf_id, (left_index, right_index)) in mp.get_observations() {
                        if *obs_kf_id == kf_id {
                            continue
                        }
                        let obs_kf = lock.keyframes.get(obs_kf_id).unwrap();
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
                        lock.discard_keyframe(kf_id);
                        num_deleted += 1;
                        self.discarded_kfs.insert(kf_id);
                        debug!("Choosing to discard {} while working on {}", kf_id, self.current_keyframe_id);
                    }
                }
            }
            if i > 20 && self.actor_channels.queue_len() > 1 {
                // Abort additional work if there are too many keyframes in the msg queue.
                return 0;
            }
        }
        return num_deleted;
    }
}
