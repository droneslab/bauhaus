use std::any::Any;
use std::{cmp::min};
use std::collections::HashSet;
use std::f64::INFINITY;
use std::iter::FromIterator;

use derivative::Derivative;
use dvcore::sensor::{Sensor, FrameSensor, ImuSensor};
use dvcore::{
    lockwrap::ReadOnlyWrapper,
    config::{GLOBAL_PARAMS, SYSTEM_SETTINGS},
    matrix::DVVector3
};
use log::{debug};
use crate::ActorSystem;
use crate::registered_modules::LOCAL_MAPPING;
use crate::{
    dvmap::{
        map_actor::MapWriteMsg, map::Map, mappoint::{MapPoint, PrelimMapPoint}
    },
    modules::{optimizer, orbmatcher, imu::ImuModule, camera::CAMERA_MODULE, optimizer::INV_LEVEL_SIGMA2, orbmatcher::SCALE_FACTORS, geometric_tools},
    registered_modules::{FEATURE_DETECTION, LOOP_CLOSING, MATCHER, CAMERA, MAP_ACTOR},
    Id,
    actors::messages::{Reset, KeyFrameIdMsg}
};

use super::messages::ShutdownMessage;

#[derive(Debug, Derivative)]
#[derivative(Default(bound=""))]
pub struct DarvisLocalMapping {
    actor_system: ActorSystem,
    map: ReadOnlyWrapper<Map>,
    sensor: Sensor,

    current_keyframe_id: Id, //mpCurrentKeyFrame
    recently_added_mappoints: Vec<Id>, //mlpRecentAddedMapPoints // TODO (concurrency): race condition w/ map deleting mappoints from the map

    // Modules
    imu: ImuModule,
}

impl DarvisLocalMapping {
    pub fn new(map: ReadOnlyWrapper<Map>, actor_system: ActorSystem) -> DarvisLocalMapping {
        let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");

        DarvisLocalMapping {
            actor_system,
            map,
            sensor,
            current_keyframe_id: -1,
            ..Default::default()
        }
    }

    pub fn run(&mut self) {
        loop {
            let message = self.actor_system.receive();
            if let Some(msg) = <dyn Any>::downcast_ref::<KeyFrameIdMsg>(&message) {
                self.current_keyframe_id = msg.keyframe_id;
                self.local_mapping();
            } else if let Some(_) = <dyn Any>::downcast_ref::<Reset>(&message){
                // TODO (design) need to think about how reset requests should be propagated
            } else if let Some(_) = <dyn Any>::downcast_ref::<ShutdownMessage>(&message) {
                break;
            }
        }
    }

    fn local_mapping(&mut self) {
        debug!("Local mapping working on kf {}", self.current_keyframe_id);

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
        self.mappoint_culling();

        // Triangulate new MapPoints
        let new_mappoints = self.create_new_mappoints();
        // Paper note: what's interesting is axiom will actually give a runtime error if the rate of sending messages
        // exceeds the ability of the actor to handle the messages. Normally we would create a new mappoint in the loop above,
        // but that sends messages to the map actor too frequently. So, we batch all the messages into one and send when we are done.
        self.actor_system.find(MAP_ACTOR).unwrap().send(Box::new(
            MapWriteMsg::create_many_mappoints(new_mappoints, LOCAL_MAPPING.to_string())
        )).unwrap();

        // TODO (design): mbAbortBA
        // But idk how to check the queue size from within the callback
        // if(!CheckNewKeyFrames()) {
                // Find more matches in neighbor keyframes and fuse point duplications
                self.search_in_neighbors();
        // }

        let t_init = 0.0; // Sofiya: idk what this is for but it's used all over the place

        // TODO (design): mbAbortBA
        // ORBSLAM will abort additional work if there are too many keyframes in the msg queue (CheckNewKeyFrames)
        // Additionally it will abort if a stop or reset is requested (stopRequested)
        // But idk how to check the queue size from within the callback, and idk how to "look ahead" at future messages to see if
        // a stop is requested further in the queue. Maybe the skip functionality? Maybe implementing messages with priority?
        // All the rest of this code, up until sending to loop closing, should be under this if statement.
        // if(!CheckNewKeyFrames() && !stopRequested())
        // {
        if self.map.read().num_keyframes() > 2 {
            match self.sensor.is_imu() {
                true => { // and self.map.read().imu_initialized
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
                    // TODO (design): mbAbortBA
                    let force_stop_flag = false; // mbAbortBA
                    let map = self.map.read();
                    let current_keyframe = map.get_keyframe(&self.current_keyframe_id).unwrap();
                    optimizer::local_bundle_adjustment(&self.map.read(), current_keyframe);
                }
            }
        }

        // Initialize IMU
        if self.sensor.is_imu() && !self.map.read().imu_initialized {
            self.imu.initialize();
        }

        // Check redundant local Keyframes
        self.keyframe_culling();

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

        let loopclosing = self.actor_system.find(LOOP_CLOSING).unwrap();
        loopclosing.send(Box::new(KeyFrameIdMsg{ keyframe_id: self.current_keyframe_id })).unwrap();
    }

    fn mappoint_culling(&mut self) {
        let th_obs = match self.sensor.is_mono() {
            true => 2,
            false => 3
        };

        let map = self.map.read();
        let current_kf_id = self.current_keyframe_id;
        let mut to_discard = Vec::new();
        self.recently_added_mappoints.retain(|&mp_id| {
            let mappoint = map.get_mappoint(&mp_id).unwrap();

            if (mappoint.get_found_ratio() < 0.25) ||
            (current_kf_id - mappoint.first_kf_id >= 2 && mappoint.get_observations().len() <= th_obs) {
                to_discard.push(mp_id);
                false
            } else if current_kf_id - mappoint.first_kf_id >= 3 {
                false // mappoint should not be deleted, but remove from recently_added_mappoints
            } else {
                true // mappoint should not be deleted
            }
        });

        self.actor_system.find(MAP_ACTOR).unwrap().send(Box::new(MapWriteMsg::discard_many_mappoints(&to_discard))).unwrap();
    }

    fn create_new_mappoints(&self) -> Vec<(MapPoint<PrelimMapPoint>, Vec<(i32, u32, usize)>)> {
        // Retrieve neighbor keyframes in covisibility graph
        let nn = match self.sensor.is_mono() {
            true => 30,
            false => 10
        };
        let ratio_factor = 1.5 * GLOBAL_PARAMS.get::<f64>(FEATURE_DETECTION, "scale_factor");
        let fpt = GLOBAL_PARAMS.get::<f64>(MATCHER, "far_points_threshold");
        let far_points_th = if fpt == 0.0 { INFINITY } else { fpt };

        let map = self.map.read();
        let current_kf = map.get_keyframe(&self.current_keyframe_id).unwrap();
        let neighbor_kfs = current_kf.get_connections(nn);
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

        let mut pose1 = current_kf.pose.unwrap(); // sophTcw1
        let translation1 = pose1.get_translation(); // tcw1
        let rotation1 = pose1.get_rotation(); // Rcw1
        let rotation_transpose1 = rotation1.transpose(); // Rwc1
        let mut ow1 = current_kf.get_camera_center();

        let mut new_mappoints = Vec::new();
        // Search matches with epipolar restriction and triangulate
        for neighbor_id in neighbor_kfs {
            // TODO (design): ORBSLAM will abort additional work if there are too many keyframes in the msg queue.
            // if(i>0 && CheckNewKeyFrames())
            //     return;
            let neighbor_kf = map.get_keyframe(&neighbor_id).unwrap();

            // Check first that baseline is not too short
            let mut ow2 = neighbor_kf.get_camera_center();
            let baseline = (*ow2 - *ow1).norm();
            match self.sensor.is_mono() {
                true => {
                    let median_depth_neigh = neighbor_kf.compute_scene_median_depth(map.get_all_mappoints(), 2);
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

            // Search matches that fullfil epipolar constraint
            let course = match self.sensor.is_imu() {
                true => todo!("IMU"), // should be mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && mpTracker->mState==Tracking::RECENTLY_LOST
                false => false
            };
            let matches = match orbmatcher::search_for_triangulation(
                current_kf, neighbor_kf,
                false, false, course,
                self.sensor
            ) {
                Ok(matches) => matches,
                Err(err) => panic!("Problem with search_by_bow_f {}", err)
            };

            let mut pose2 = neighbor_kf.pose.unwrap();
            let translation2 = pose2.get_translation(); // tcw2
            let rotation2 = pose2.get_rotation(); // Rcw2
            let rotation_transpose2 = rotation2.transpose(); // Rwc2

            // Triangulate each match
            for (idx1, idx2) in matches {
                let (kp1, right1) = current_kf.features.get_keypoint(idx1);
                let (kp2, right2) = neighbor_kf.features.get_keypoint(idx2);

                let (mut kp1_ur, mut kp2_ur) = (None, None);
                if right1 {
                    kp1_ur = current_kf.features.get_mv_right(idx1);
                    pose1 = current_kf.get_right_pose();
                    ow1 = current_kf.get_right_camera_center();
                    // camera1 = mpCurrentKeyFrame->mpCamera2 TODO (STEREO) .. right now just using global CAMERA
                } else {
                    pose1 = current_kf.pose.unwrap();
                    ow1 = current_kf.get_camera_center();
                    // camera1 = mpCurrentKeyFrame->mpCamera TODO (STEREO)
                }
                if right2 {
                    kp2_ur = neighbor_kf.features.get_mv_right(idx2);
                    pose2 = neighbor_kf.get_right_pose();
                    ow2 = neighbor_kf.get_right_camera_center();
                    // camera2 = neighbor_kf->mpCamera2 TODO (STEREO)
                } else {
                    pose2 = neighbor_kf.pose.unwrap();
                    ow2 = neighbor_kf.get_camera_center();
                    // camera2 = neighbor_kf->mpCamera TODO (STEREO)
                }

                // Check parallax between rays
                let xn1 = CAMERA_MODULE.unproject_eig(&kp1.pt);
                let xn2 = CAMERA_MODULE.unproject_eig(&kp2.pt);
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
                let good_parallax_with_imu = cos_parallax_rays < 0.9996 && self.sensor.is_imu();
                let good_parallax_wo_imu = cos_parallax_rays < 0.9998 && !self.sensor.is_imu();
                if cos_parallax_rays < cos_parallax_stereo && cos_parallax_rays > 0.0 && (right1 || right2 || good_parallax_with_imu || good_parallax_wo_imu) {
                    x3_d = geometric_tools::triangulate(xn1, xn2, pose1, pose2);
                } else if right1 && cos_parallax_stereo1 < cos_parallax_stereo2 {
                    x3_d = CAMERA_MODULE.unproject_stereo(current_kf, idx1);
                } else if right2 && cos_parallax_stereo2 < cos_parallax_stereo1 {
                    x3_d = CAMERA_MODULE.unproject_stereo(neighbor_kf, idx2);
                } else {
                    continue // No stereo and very low parallax
                }
                if x3_d.is_none() {
                    continue
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
                let sigma_square1 = INV_LEVEL_SIGMA2[kp1.octave as usize];
                let x1 = rotation1.row(0).transpose().dot(&x3_d_nalg) + (*translation1)[0];
                let y1 = rotation1.row(1).transpose().dot(&x3_d_nalg) + (*translation1)[1];

                if right1 {
                    todo!("Stereo");
                    // let invz1 = 1.0 / z1;
                    // let u1 = CAMERA_MODULE.get_fx() * x1 * invz1 + CAMERA_MODULE.get_cx();
                    // let u1_r = u1 - CAMERA_MODULE.stereo_baseline_times_fx * invz1;
                    // let v1 = CAMERA_MODULE.get_fy() * y1 * invz1 + CAMERA_MODULE.get_cy();
                    // let err_x1 = u1 as f32 - kp1.pt.x;
                    // let err_y1 = v1 as f32 - kp1.pt.y;
                    // let err_x1_r = u1_r as f32 - kp1_ur.unwrap();
                    // if (err_x1 * err_x1  + err_y1 * err_y1 + err_x1_r * err_x1_r) > 7.8 * sigma_square1 {
                    //     continue
                    // }
                } else {
                    let uv1 = CAMERA_MODULE.project(DVVector3::new_with(x1, y1, z1));
                    let err_x1 = uv1.0 as f32 - kp1.pt.x;
                    let err_y1 = uv1.1 as f32 - kp1.pt.y;
                    if (err_x1 * err_x1  + err_y1 * err_y1) > 5.991 * sigma_square1 {
                        continue
                    }
                }

                //Check reprojection error in second keyframe
                let sigma_square2 = INV_LEVEL_SIGMA2[kp2.octave as usize];
                let x2 = rotation2.row(0).transpose().dot(&x3_d_nalg) + (*translation2)[0];
                let y2 = rotation2.row(1).transpose().dot(&x3_d_nalg) + (*translation2)[1];

                if right2 {
                    todo!("Stereo");
                    // let invz2 = 1.0 / z2;
                    // let u2 = CAMERA_MODULE.get_fx() * x2 * invz2 + CAMERA_MODULE.get_cx();// This should be camera2, not camera
                    // let u2_r = u2 - CAMERA_MODULE.stereo_baseline_times_fx * invz2;
                    // let v2 = CAMERA_MODULE.get_fy() * y2 * invz2 + CAMERA_MODULE.get_cy();// This should be camera2, not camera
                    // let err_x2 = u2 as f32 - kp2.pt.x;
                    // let err_y2 = v2 as f32 - kp2.pt.y;
                    // let err_x2_r = u2_r as f32 - kp2_ur.unwrap();
                    // if (err_x2 * err_x2  + err_y2 * err_y2 + err_x2_r * err_x2_r) > 7.8 * sigma_square2 {
                    //     continue
                    // }
                } else {
                    let uv2 = CAMERA_MODULE.project(DVVector3::new_with(x2, y2, z2));
                    let err_x2 = uv2.0 as f32 - kp2.pt.x;
                    let err_y2 = uv2.1 as f32 - kp2.pt.y;
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
                let ratio_octave = (SCALE_FACTORS[kp1.octave as usize] / SCALE_FACTORS[kp2.octave as usize]) as f64;
                if ratio_dist * ratio_factor < ratio_octave || ratio_dist > ratio_octave * ratio_factor {
                    continue;
                }

                // Triangulation is successful
                new_mappoints.push((
                    MapPoint::<PrelimMapPoint>::new(x3_d.unwrap(), self.current_keyframe_id, map.id),
                    vec![(self.current_keyframe_id, current_kf.features.num_keypoints, idx1), (neighbor_id, neighbor_kf.features.num_keypoints, idx2)]
                ));
            }
        }
        new_mappoints
    }

    fn search_in_neighbors(&self) {
        // Retrieve neighbor keyframes
        let nn = match self.sensor.frame() {
            FrameSensor::Mono => 30,
            FrameSensor::Stereo | FrameSensor::Rgbd => 10,
        };

        let map = self.map.read();
        let current_kf = map.get_keyframe(&self.current_keyframe_id).unwrap();
        let mut target_kfs = HashSet::<i32>::from_iter(current_kf.get_connections(nn));

        // Add some covisible of covisible
        // Extend to some second neighbors if abort is not requested
        let new_kfs = target_kfs.iter().map(|kf_id| {
            map.get_keyframe(&kf_id).unwrap().get_connections(20)
        }).flatten().collect::<Vec<i32>>();
        target_kfs.extend(new_kfs);

        // TODO (design): mbAbortBA.. ORBSLAM will abort additional work if there are too many keyframes in the msg queue.
        // if (mbAbortBA)
        //     break;

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

        let mut to_fuse = Vec::new();
        // Search matches by projection from current KF in target KFs
        let mappoint_matches = current_kf.mappoint_matches.iter().map(|(_, (mp_id, _))| {*mp_id}).collect::<Vec<Id>>();
        for kf_id in &target_kfs {
            to_fuse.extend(orbmatcher::fuse(kf_id, &mappoint_matches, &map, 3.0, false));
            to_fuse.extend(
                match self.sensor.frame() {
                    FrameSensor::Stereo => orbmatcher::fuse(kf_id, &mappoint_matches, &map, 3.0, true),
                    FrameSensor::Mono | FrameSensor::Rgbd => vec![],
                }
            )
        }

        // TODO (design): mbAbortBA
        // if (mbAbortBA)
        //     return;

        // Search matches by projection from target KFs in current KF
        let fuse_candidates = HashSet::<Id>::from_iter(
            target_kfs.iter().map(|kf_id| {
                let kf = map.get_keyframe(&kf_id).unwrap();
                let mappoints_kf = &kf.mappoint_matches;
                mappoints_kf.iter().map(|(_, (mp_id, _))| *mp_id)
            }).flatten()
        );
        let fuse_candidates_vec = fuse_candidates.iter().map(|mp| *mp).collect::<Vec<Id>>();
        to_fuse.extend(orbmatcher::fuse(&self.current_keyframe_id, &fuse_candidates_vec,  &map, 3.0, false));
        to_fuse.extend(
            match self.sensor.frame() {
                FrameSensor::Stereo => orbmatcher::fuse(&self.current_keyframe_id, &fuse_candidates_vec, &map, 3.0, true),
                FrameSensor::Mono | FrameSensor::Rgbd => vec![]
            }
        );

        for msg in to_fuse {
            self.actor_system.find(MAP_ACTOR).unwrap().send(Box::new(msg)).unwrap();
        }
    }

    fn keyframe_culling(&self) {
        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points

        //TODO... I think we don't need this because the covisibility keyframes struct organizes itself but double check
        // mpCurrentKeyFrame->UpdateBestCovisibles(); 

        let map = self.map.read();
        let current_kf = map.get_keyframe(&self.current_keyframe_id).unwrap();
        let local_keyframes = current_kf.get_connections(i32::MAX);

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
            if kf_id == map.initial_kf_id {
                continue
            }
            let keyframe = map.get_keyframe(&kf_id).unwrap();
            let mappoints = &keyframe.mappoint_matches;

            let th_obs = 3;
            let mut num_mps = 0;
            let mut num_redundant_obs = 0;

            for (idx, (mp_id, _)) in mappoints {
                if !self.sensor.is_mono() {
                    let mv_depth = keyframe.features.get_mv_depth(*idx as usize).unwrap();
                    let th_depth = GLOBAL_PARAMS.get::<i32>(CAMERA, "thdepth") as f32;
                    if mv_depth > th_depth || mv_depth < 0.0 {
                        continue
                    }
                }
                let mp = map.get_mappoint(mp_id).unwrap();
                num_mps += 1;

                if mp.get_observations().len() > th_obs {
                    let scale_level = keyframe.features.get_octave(*idx as usize);
                    let mut num_obs = 0;
                    for (obs_kf_id, (left_index, right_index)) in mp.get_observations() {
                        if *obs_kf_id == kf_id {
                            continue
                        }
                        let obs_kf = map.get_keyframe(obs_kf_id).unwrap();
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
                        let map_msg = MapWriteMsg::delete_keyframe(kf_id);
                        self.actor_system.find(MAP_ACTOR).unwrap().send(Box::new(map_msg)).unwrap();
                    }
                }
            }
            // TODO (design): mbAbortBA
            // if((count > 20 && mbAbortBA) { break; }
        }
    }
}
