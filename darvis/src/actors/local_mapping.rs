use std::f64::INFINITY;

use axiom::prelude::*;
use dvcore::config::{Sensor, GLOBAL_PARAMS, SYSTEM_SETTINGS, FrameSensor};
use dvcore::matrix::DVVector3;
use dvcore::{
    plugin_functions::Function,
    lockwrap::ReadOnlyWrapper,
};
use log::debug;
use crate::dvmap::map_actor::MapWriteMsg;
use crate::modules::camera;
use crate::modules::optimizer::INV_LEVEL_SIGMA2;
use crate::modules::orbmatcher::SCALE_FACTORS;
use crate::registered_modules::{FEATURE_DETECTION, MATCHER};
use crate::{
    dvmap::{map::Map, map_actor::MAP_ACTOR, keyframe::{KeyFrame, PrelimKeyFrame}, mappoint::{MapPoint, PrelimMapPoint}},
    modules::{optimizer, orbmatcher, imu::ImuModule, camera::CAMERA_MODULE},
    registered_modules::LOOP_CLOSING,
    Id,
    actors::messages::{Reset, KeyFrameIdMsg, LastKeyFrameUpdatedMsg}
};

#[derive(Debug, Clone, Default)]
pub struct DarvisLocalMapping {
    map: ReadOnlyWrapper<Map>,
    sensor: Sensor,

    current_keyframe_id: Id, //mpCurrentKeyFrame
    recently_added_mappoints: Vec<Id>, //mlpRecentAddedMapPoints // TODO: there is a race condition here with deleting mappoints from the map

    // Modules
    imu: ImuModule,
}

impl DarvisLocalMapping {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisLocalMapping {
        let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");

        DarvisLocalMapping {
            map,
            sensor,
            current_keyframe_id: -1,
            ..Default::default()
        }
    }

    fn local_mapping(&mut self, context: Context) {
        let map_actor = context.system.find_aid_by_name(MAP_ACTOR).unwrap();
        // TODO (Stereo): LocalMapping::ProcessNewKeyFrame pushes to mlpRecentAddedMapPoints
        // those stereo mappoints which were added in tracking. The rest of the function
        // is redundant here because all of it is already computed when inserting a keyframe,
        // but the part about mlpRecentAddedMapPoints needs to be included.

        // Check recent MapPoints
        self.mappoint_culling(&map_actor);

        // Triangulate new MapPoints
        self.create_new_mappoints(&map_actor);
        // Inform tracking that we are done with creating new mappoints. Needed because tracking fails if new points are not created
        // before track_with_reference_keyframe, so to avoid the race condition we have it wait until the new points are ready.
        context.system.find_aid_by_name("TRACKING_BACKEND").unwrap().send_new(LastKeyFrameUpdatedMsg{}).unwrap();
        debug!("Local mapping finished creating new mappoints");

        // TODO (design): ORBSLAM will abort additional work if there are too many keyframes in the msg queue.
        // But idk how to check the queue size from within the callback
        // if(!CheckNewKeyFrames()) {
                // Find more matches in neighbor keyframes and fuse point duplications
                self.search_in_neighbors();
        // }

        let t_init = 0.0; // Sofiya: idk what this is for but it's used all over the place

        // TODO (design): ORBSLAM will abort additional work if there are too many keyframes in the msg queue (CheckNewKeyFrames)
        // Additionally it will abort if a stop or reset is requested (stopRequested)
        // But idk how to check the queue size from within the callback, and idk how to "look ahead" at future messages to see if
        // a stop is requested further in the queue. Maybe the skip functionality? Maybe implementing messages with priority?
        // All the rest of this code, up until sending to loop closing, should be under this if statement.
        // if(!CheckNewKeyFrames() && !stopRequested())
        // {
        if self.map.read().num_keyframes() > 2 {
            match self.sensor.is_imu() {
                true => { // and self.map.read().imu_initialized
                    // TODO (IMU)
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
                    // TODO (design): ORBSLAM will abort additional work if there are too many keyframes in the msg queue.
                    let force_stop_flag = false; // mbAbortBA
                    let map = self.map.read();
                    let current_keyframe = map.get_keyframe(&self.current_keyframe_id).unwrap();
                    optimizer::local_bundle_adjustment(&self.map.read(), current_keyframe, force_stop_flag, 0, 0,0, 0);
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
            // TODO (IMU)
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

        self.send_to_loop_closing(context);
    }

    fn mappoint_culling(&mut self, map_actor: &Aid) {
        let th_obs = match self.sensor.is_mono() {
            true => 2,
            false => 3
        };

        let map = self.map.read();
        let current_kf_id = self.current_keyframe_id;
        self.recently_added_mappoints.retain(|&mp_id| {
            let mappoint = map.get_mappoint(&mp_id).unwrap();

            if (mappoint.get_found_ratio() < 0.25) ||
            (current_kf_id - mappoint.first_kf_id >= 2 && mappoint.get_observations().len() <= th_obs) {
                map_actor.send_new(MapWriteMsg::discard_mappoint(&mp_id)).unwrap();
                false
            } else if current_kf_id - mappoint.first_kf_id >= 3 {
                false // mappoint should not be deleted, but remove from recently_added_mappoints
            } else {
                true // mappoint should not be deleted
            }
        });
    }

    fn create_new_mappoints(&self, map_actor: &Aid) {
        // Retrieve neighbor keyframes in covisibility graph
        let nn = match self.sensor.is_mono() {
            true => 30,
            false => 10
        };
        let thresh = 0.6;
        let ratio_factor = 1.5 * GLOBAL_PARAMS.get::<f64>(FEATURE_DETECTION, "scale_factor");
        let fpt = GLOBAL_PARAMS.get::<f64>(MATCHER, "far_points_threshold");
        let far_points_th = if fpt == 0.0 { INFINITY } else { fpt };

        let map = self.map.read();
        let current_kf = map.get_keyframe(&self.current_keyframe_id).unwrap();
        let neighbor_kfs = current_kf.get_connections(nn);
        if self.sensor.is_imu() {
            // TODO (IMU)
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

        let mut pose1 = current_kf.pose; // sophTcw1
        let mut translation1 = pose1.get_translation(); // tcw1
        let mut rotation1 = pose1.get_rotation(); // Rcw1
        let mut rotation_transpose1 = rotation1.transpose(); // Rwc1
        let mut ow1 = current_kf.get_camera_center();

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
            // TODO (IMU): should replace falses with mpCurrentKeyFrame->GetMap()->GetIniertialBA2() and mpTracker->mState==Tracking::RECENTLY_LOST
            let course = self.sensor.is_imu() && false && false;
            let matches = match orbmatcher::search_for_triangulation(
                current_kf, neighbor_kf,
                false, false, course, thresh,
                &self.map
            ) {
                Ok(matches) => matches,
                Err(err) => panic!("Problem with search_by_bow_f {}", err)
            };

            let mut pose2 = neighbor_kf.pose;
            let mut translation2 = pose2.get_translation(); // tcw2
            let mut rotation2 = pose2.get_rotation(); // Rcw2
            let mut rotation_transpose2 = rotation2.transpose(); // Rwc2

            // Triangulate each match
            for (idx1, idx2) in matches {
                let kp1 = current_kf.features.get_keypoint(idx1);
                let kp2 = neighbor_kf.features.get_keypoint(idx2);

                let (kp1_ur, kp2_ur, stereo1, stereo2, right1, right2) = match self.sensor.frame() {
                    FrameSensor::Mono | FrameSensor::Rgbd => (0.0, 0.0, false, false, false, false),
                    FrameSensor::Stereo => {
                        // TODO (stereo) ... fill in the commented out things in this subsection
                        let kp1_ur = current_kf.features.get_mv_right(idx1);
                        let stereo1 = kp1_ur.is_some(); // && !mpCurrentKeyFrame->mpCamera2
                        let right1 = current_kf.features.has_left_kp().map_or(false, |n_left| (idx1 as u32) >= n_left);

                        let kp2_ur = neighbor_kf.features.get_mv_right(idx2);
                        let stereo2 = kp2_ur.is_some(); // && !neighbor_kf->mpCamera2
                        let right2 = neighbor_kf.features.has_left_kp().map_or(false, |n_left| (idx2 as u32) >= n_left);

                        if right1 {
                            pose1 = current_kf.get_right_pose();
                            ow1 = current_kf.get_right_camera_center();
                            // camera1 = mpCurrentKeyFrame->mpCamera2
                        } else {
                            pose1 = current_kf.pose;
                            ow1 = current_kf.get_camera_center();
                            // camera1 = mpCurrentKeyFrame->mpCamera
                        };
                        if right2 {
                            pose2 = neighbor_kf.get_right_pose();
                            ow2 = neighbor_kf.get_right_camera_center();
                            // camera2 = neighbor_kf->mpCamera2
                        } else {
                            pose2 = neighbor_kf.pose;
                            ow2 = neighbor_kf.get_camera_center();
                            // camera2 = neighbor_kf->mpCamera
                        }

                        translation1 = pose1.get_translation();
                        rotation1 = pose1.get_rotation();
                        rotation_transpose1 = rotation1.transpose();

                        translation2 = pose2.get_translation();
                        rotation2 = pose2.get_rotation();
                        rotation_transpose2 = rotation2.transpose();

                        (kp1_ur.unwrap_or_else(|| 0.0), kp2_ur.unwrap_or_else(|| 0.0), stereo1, stereo2, right1, right2)
                    },
                };

                // Check parallax between rays
                let xn1 = CAMERA_MODULE.unproject_eig(&kp1.pt);
                let xn2 = CAMERA_MODULE.unproject_eig(&kp2.pt);
                let ray1 = rotation_transpose1 * (*xn1);
                let ray2 = rotation_transpose2 * (*xn2);
                let cos_parallax_rays = ray1.dot(&ray2) / (ray1.norm() * ray2.norm());
                let (cos_parallax_stereo1, cos_parallax_stereo2) = (cos_parallax_rays + 1.0, cos_parallax_rays + 1.0);
                if stereo1 {
                    // TODO (Stereo)
                    // cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
                } else if stereo2 {
                    // TODO (Stereo)
                    //cosParallaxStereo2 = cos(2*atan2(neighbor_kf->mb/2,neighbor_kf->mvDepth[idx2]));
                }
                let cos_parallax_stereo = cos_parallax_stereo1.min(cos_parallax_stereo2);

                let x3_d;
                let good_parallax_with_imu = cos_parallax_rays < 0.9996 && self.sensor.is_imu();
                let good_parallax_wo_imu = cos_parallax_rays < 0.9998 && !self.sensor.is_imu();
                if cos_parallax_rays < cos_parallax_stereo && cos_parallax_rays > 0.0 && (stereo1 || stereo2 || good_parallax_with_imu || good_parallax_wo_imu) {
                    x3_d = camera::triangulate(xn1, xn2, translation1, translation2);
                } else if stereo1 && cos_parallax_stereo1 < cos_parallax_stereo2 {
                    x3_d = CAMERA_MODULE.unproject_stereo(current_kf, idx1);
                } else if stereo2 && cos_parallax_stereo2 < cos_parallax_stereo1 {
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
                // float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
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
                let invz1 = 1.0 / z1;
                
                if !stereo1 {
                    let uv1 = CAMERA_MODULE.project(DVVector3::new_with(x1, y1, z1));
                    let err_x1 = uv1.0 as f32 - kp1.pt.x;
                    let err_y1 = uv1.1 as f32 - kp1.pt.y;
                    if (err_x1 * err_x1  + err_y1 * err_y1) > 5.991 * sigma_square1 {
                        continue
                    }
                } else {
                    let u1 = CAMERA_MODULE.get_fx() * x1 * invz1 + CAMERA_MODULE.get_cx();
                    let u1_r = u1 - CAMERA_MODULE.stereo_baseline_times_fx * invz1;
                    let v1 = CAMERA_MODULE.get_fy() * y1 * invz1 + CAMERA_MODULE.get_cy();
                    let err_x1 = u1 as f32 - kp1.pt.x;
                    let err_y1 = v1 as f32 - kp1.pt.y;
                    let err_x1_r = u1_r as f32 - kp1_ur;
                    if (err_x1 * err_x1  + err_y1 * err_y1 + err_x1_r * err_x1_r) > 7.8 * sigma_square1 {
                        continue
                    }
                }

                //Check reprojection error in second keyframe
                let sigma_square2 = INV_LEVEL_SIGMA2[kp2.octave as usize];
                let x2 = rotation2.row(0).transpose().dot(&x3_d_nalg) + (*translation2)[0];
                let y2 = rotation2.row(1).transpose().dot(&x3_d_nalg) + (*translation2)[1];
                let invz2 = 1.0 / z2;

                if !stereo2 {
                    let uv2 = CAMERA_MODULE.project(DVVector3::new_with(x2, y2, z2)); // TODO(STEREO): This should be camera2, not camera
                    let err_x2 = uv2.0 as f32 - kp2.pt.x;
                    let err_y2 = uv2.1 as f32 - kp2.pt.y;
                    if (err_x2 * err_x2  + err_y2 * err_y2) > 5.991 * sigma_square2 {
                        continue
                    }

                } else {
                    let u2 = CAMERA_MODULE.get_fx() * x2 * invz2 + CAMERA_MODULE.get_cx();// TODO(STEREO): This should be camera2, not camera
                    let u2_r = u2 - CAMERA_MODULE.stereo_baseline_times_fx * invz2;
                    let v2 = CAMERA_MODULE.get_fy() * y2 * invz2 + CAMERA_MODULE.get_cy();// TODO(STEREO): This should be camera2, not camera
                    let err_x2 = u2 as f32 - kp2.pt.x;
                    let err_y2 = v2 as f32 - kp2.pt.y;
                    let err_x2_r = u2_r as f32- kp2_ur;
                    if (err_x2 * err_x2  + err_y2 * err_y2 + err_x2_r * err_x2_r) > 7.8 * sigma_square2 {
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

                // Triangulation is succesfull
                // TODO LOCAL MAPPING
                // let new_mp_msg = MapWriteMsg::create_new_mappoint(
                //     MapPoint::<PrelimMapPoint>::new(x3D, self.current_keyframe_id, map.id),
                //     vec![(self.current_keyframe_id, num_keypoints, idx1), (neighbor_id, num_keypoints, idx2)]
                // );
                // map_actor.send_new(new_mp_msg).unwrap();
            }
        }
    }

    fn search_in_neighbors(&self) {
        // TODO LOCAL MAPPING
        // // Retrieve neighbor keyframes
        // int nn = 10;
        // if(mbMonocular)
        //     nn=30;
        // const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
        // vector<KeyFrame*> vpTargetKFs;
        // for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
        // {
        //     KeyFrame* pKFi = *vit;
        //     if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
        //         continue;
        //     vpTargetKFs.push_back(pKFi);
        //     pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;
        // }

        // // Add some covisible of covisible
        // // Extend to some second neighbors if abort is not requested
        // for(int i=0, imax=vpTargetKFs.size(); i<imax; i++)
        // {
        //     const vector<KeyFrame*> vpSecondNeighKFs = vpTargetKFs[i]->GetBestCovisibilityKeyFrames(20);
        //     for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        //     {
        //         KeyFrame* pKFi2 = *vit2;
        //         if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
        //             continue;
        //         vpTargetKFs.push_back(pKFi2);
        //         pKFi2->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
        //     }
        //     if (mbAbortBA)
        //         break;
        // }

        // // Extend to temporal neighbors
        // if(mbInertial)
        // {
        //     KeyFrame* pKFi = mpCurrentKeyFrame->mPrevKF;
        //     while(vpTargetKFs.size()<20 && pKFi)
        //     {
        //         if(pKFi->isBad() || pKFi->mnFuseTargetForKF==mpCurrentKeyFrame->mnId)
        //         {
        //             pKFi = pKFi->mPrevKF;
        //             continue;
        //         }
        //         vpTargetKFs.push_back(pKFi);
        //         pKFi->mnFuseTargetForKF=mpCurrentKeyFrame->mnId;
        //         pKFi = pKFi->mPrevKF;
        //     }
        // }

        // // Search matches by projection from current KF in target KFs
        // ORBmatcher matcher;
        // vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        // for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
        // {
        //     KeyFrame* pKFi = *vit;

        //     matcher.Fuse(pKFi,vpMapPointMatches);
        //     if(pKFi->NLeft != -1) matcher.Fuse(pKFi,vpMapPointMatches,true);
        // }


        // if (mbAbortBA)
        //     return;

        // // Search matches by projection from target KFs in current KF
        // vector<MapPoint*> vpFuseCandidates;
        // vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

        // for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
        // {
        //     KeyFrame* pKFi = *vitKF;

        //     vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

        //     for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        //     {
        //         MapPoint* pMP = *vitMP;
        //         if(!pMP)
        //             continue;
        //         if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
        //             continue;
        //         pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
        //         vpFuseCandidates.push_back(pMP);
        //     }
        // }

        // matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);
        // if(mpCurrentKeyFrame->NLeft != -1) matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates,true);


        // // Update points
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

        // // Update connections in covisibility graph
        // mpCurrentKeyFrame->UpdateConnections();
    }

    fn keyframe_culling(&self) {
        // TODO LOCAL MAPPING
        // // Check redundant keyframes (only local keyframes)
        // // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // // in at least other 3 keyframes (in the same or finer scale)
        // // We only consider close stereo points
        // const int Nd = 21;
        // mpCurrentKeyFrame->UpdateBestCovisibles();
        // vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

        // float redundant_th;
        // if(!mbInertial)
        //     redundant_th = 0.9;
        // else if (mbMonocular)
        //     redundant_th = 0.9;
        // else
        //     redundant_th = 0.5;

        // const bool bInitImu = mpAtlas->isImuInitialized();
        // int count=0;

        // // Compoute last KF from optimizable window:
        // unsigned int last_ID;
        // if (mbInertial)
        // {
        //     int count = 0;
        //     KeyFrame* aux_KF = mpCurrentKeyFrame;
        //     while(count<Nd && aux_KF->mPrevKF)
        //     {
        //         aux_KF = aux_KF->mPrevKF;
        //         count++;
        //     }
        //     last_ID = aux_KF->mnId;
        // }



        // for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
        // {
        //     count++;
        //     KeyFrame* pKF = *vit;

        //     if((pKF->mnId==pKF->GetMap()->GetInitKFid()) || pKF->isBad())
        //         continue;
        //     const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

        //     int nObs = 3;
        //     const int thObs=nObs;
        //     int nRedundantObservations=0;
        //     int nMPs=0;
        //     for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        //     {
        //         MapPoint* pMP = vpMapPoints[i];
        //         if(pMP)
        //         {
        //             if(!pMP->isBad())
        //             {
        //                 if(!mbMonocular)
        //                 {
        //                     if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
        //                         continue;
        //                 }

        //                 nMPs++;
        //                 if(pMP->Observations()>thObs)
        //                 {
        //                     const int &scaleLevel = (pKF -> NLeft == -1) ? pKF->mvKeysUn[i].octave
        //                                                                 : (i < pKF -> NLeft) ? pKF -> mvKeys[i].octave
        //                                                                                     : pKF -> mvKeysRight[i].octave;
        //                     const map<KeyFrame*, tuple<int,int>> observations = pMP->GetObservations();
        //                     int nObs=0;
        //                     for(map<KeyFrame*, tuple<int,int>>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        //                     {
        //                         KeyFrame* pKFi = mit->first;
        //                         if(pKFi==pKF)
        //                             continue;
        //                         tuple<int,int> indexes = mit->second;
        //                         int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        //                         int scaleLeveli = -1;
        //                         if(pKFi -> NLeft == -1)
        //                             scaleLeveli = pKFi->mvKeysUn[leftIndex].octave;
        //                         else {
        //                             if (leftIndex != -1) {
        //                                 scaleLeveli = pKFi->mvKeys[leftIndex].octave;
        //                             }
        //                             if (rightIndex != -1) {
        //                                 int rightLevel = pKFi->mvKeysRight[rightIndex - pKFi->NLeft].octave;
        //                                 scaleLeveli = (scaleLeveli == -1 || scaleLeveli > rightLevel) ? rightLevel
        //                                                                                             : scaleLeveli;
        //                             }
        //                         }

        //                         if(scaleLeveli<=scaleLevel+1)
        //                         {
        //                             nObs++;
        //                             if(nObs>thObs)
        //                                 break;
        //                         }
        //                     }
        //                     if(nObs>thObs)
        //                     {
        //                         nRedundantObservations++;
        //                     }
        //                 }
        //             }
        //         }
        //     }

        //     if(nRedundantObservations>redundant_th*nMPs)
        //     {
        //         if (mbInertial)
        //         {
        //             if (mpAtlas->KeyFramesInMap()<=Nd)
        //                 continue;

        //             if(pKF->mnId>(mpCurrentKeyFrame->mnId-2))
        //                 continue;

        //             if(pKF->mPrevKF && pKF->mNextKF)
        //             {
        //                 const float t = pKF->mNextKF->mTimeStamp-pKF->mPrevKF->mTimeStamp;

        //                 if((bInitImu && (pKF->mnId<last_ID) && t<3.) || (t<0.5))
        //                 {
        //                     pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
        //                     pKF->mNextKF->mPrevKF = pKF->mPrevKF;
        //                     pKF->mPrevKF->mNextKF = pKF->mNextKF;
        //                     pKF->mNextKF = NULL;
        //                     pKF->mPrevKF = NULL;
        //                     pKF->SetBadFlag();
        //                 }
        //                 else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2() && ((pKF->GetImuPosition()-pKF->mPrevKF->GetImuPosition()).norm()<0.02) && (t<3))
        //                 {
        //                     pKF->mNextKF->mpImuPreintegrated->MergePrevious(pKF->mpImuPreintegrated);
        //                     pKF->mNextKF->mPrevKF = pKF->mPrevKF;
        //                     pKF->mPrevKF->mNextKF = pKF->mNextKF;
        //                     pKF->mNextKF = NULL;
        //                     pKF->mPrevKF = NULL;
        //                     pKF->SetBadFlag();
        //                 }
        //             }
        //         }
        //         else
        //         {
        //             pKF->SetBadFlag();
        //         }
        //     }
        //     if((count > 20 && mbAbortBA) || count>100)
        //     {
        //         break;
        //     }
        // }
    }

    fn send_to_loop_closing(&self, context: Context) {
        let loopclosing = context.system.find_aid_by_name(LOOP_CLOSING).unwrap();
        let actor_msg = GLOBAL_PARAMS.get::<String>(LOOP_CLOSING, "actor_message");
        // TODO LOCAL MAPPING
        // match actor_msg.as_ref() {
        //     "KeyFrameMsg" => {
        //         loopclosing.send_new(KeyFrameMsg{ kf: new_kf }).unwrap();
        //     },
        //     _ => {
        //         warn!("tracking_backend::create_new_keyframe;invalid message type, selecting KeyFrameMsg");
        //         loopclosing.send_new(KeyFrameMsg{ kf: new_kf }).unwrap();
        //     },
        // }
    }
}

impl Function for DarvisLocalMapping {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        if let Some(msg) = message.content_as::<KeyFrameIdMsg>() {
            self.current_keyframe_id = msg.keyframe_id;
            self.local_mapping(context);
        } else if let Some(_) = message.content_as::<Reset>() {
            // TODO (design) need to think about how reset requests should be propagated
        }

        Ok(Status::done(()))
    }
}