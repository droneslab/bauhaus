use std::sync::Arc;
use axiom::prelude::*;

use dvcore::config::{Sensor, GLOBAL_PARAMS, SYSTEM_SETTINGS, FrameSensor};
use dvcore::{
    plugin_functions::Function,
    lockwrap::ReadOnlyWrapper,
};
use crate::dvmap::map_actor::MapWriteMsg;
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
        // TODO: send mappoints_to_delete to map actor
    }

    fn create_new_mappoints(&self, map_actor: &Aid) {
        // Retrieve neighbor keyframes in covisibility graph
        let nn = match self.sensor.is_mono() {
            true => 30,
            false => 10
        };
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

        let th = 0.6;

        let kf_pose = current_kf.pose.get_translation();
        // Sophus::SE3<float> sophTcw1 = mpCurrentKeyFrame->GetPose();
        // Eigen::Matrix<float,3,4> eigTcw1 = sophTcw1.matrix3x4();
        // Eigen::Matrix<float,3,3> Rcw1 = eigTcw1.block<3,3>(0,0);
        // Eigen::Matrix<float,3,3> Rwc1 = Rcw1.transpose();
        // Eigen::Vector3f tcw1 = sophTcw1.translation();
        let ow1 = current_kf.get_camera_center();

        // const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

        // Search matches with epipolar restriction and triangulate
        for neighbor_id in neighbor_kfs {
            // if(i>0 && CheckNewKeyFrames())
            //     return;
            let neighbor_kf = map.get_keyframe(&neighbor_id).unwrap();

            // Check first that baseline is not too short
            let ow2 = neighbor_kf.get_camera_center();
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
            // let course = self.sensor.is_imu() && ;
            // bool bCoarse = mbInertial && mpTracker->mState==Tracking::RECENTLY_LOST && mpCurrentKeyFrame->GetMap()->GetIniertialBA2();
            // TODO: this is an error because map lock is taken twice in same thread
            let matches = match orbmatcher::search_for_triangulation(current_kf, neighbor_kf, false, th, &self.map) {
                Ok(matches) => matches,
                Err(err) => panic!("Problem with search_by_bow_f {}", err)
            };

            // Sophus::SE3<float> sophTcw2 = neighbor_kf->GetPose();
            // Eigen::Matrix<float,3,4> eigTcw2 = sophTcw2.matrix3x4();
            // Eigen::Matrix<float,3,3> Rcw2 = eigTcw2.block<3,3>(0,0);
            // Eigen::Matrix<float,3,3> Rwc2 = Rcw2.transpose();
            // Eigen::Vector3f tcw2 = sophTcw2.translation();

            // Triangulate each match
            for (idx1, idx2) in matches {
                let kp1 = current_kf.features.get_keypoint(idx1);
                let kp2 = current_kf.features.get_keypoint(idx2);

                // TODO (stereo)
                // const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
                // bool bStereo1 = (!mpCurrentKeyFrame->mpCamera2 && kp1_ur>=0);
                // const bool bRight1 = (mpCurrentKeyFrame -> NLeft == -1 || idx1 < mpCurrentKeyFrame -> NLeft) ? false : true;
                match self.sensor.frame() {
                    FrameSensor::Mono | FrameSensor::Rgbd => {},
                    FrameSensor::Stereo => {
                        // TODO (stereo)
                        // if(bRight1 && bRight2){
                        //     sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                        //     Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                        //     sophTcw2 = neighbor_kf->GetRightPose();
                        //     Ow2 = neighbor_kf->GetRightCameraCenter();

                        //     pCamera1 = mpCurrentKeyFrame->mpCamera2;
                        //     pCamera2 = neighbor_kf->mpCamera2;
                        // }
                        // else if(bRight1 && !bRight2){
                        //     sophTcw1 = mpCurrentKeyFrame->GetRightPose();
                        //     Ow1 = mpCurrentKeyFrame->GetRightCameraCenter();

                        //     sophTcw2 = neighbor_kf->GetPose();
                        //     Ow2 = neighbor_kf->GetCameraCenter();

                        //     pCamera1 = mpCurrentKeyFrame->mpCamera2;
                        //     pCamera2 = neighbor_kf->mpCamera;
                        // }
                        // else if(!bRight1 && bRight2){
                        //     sophTcw1 = mpCurrentKeyFrame->GetPose();
                        //     Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                        //     sophTcw2 = neighbor_kf->GetRightPose();
                        //     Ow2 = neighbor_kf->GetRightCameraCenter();

                        //     pCamera1 = mpCurrentKeyFrame->mpCamera;
                        //     pCamera2 = neighbor_kf->mpCamera2;
                        // }
                        // else{
                        //     sophTcw1 = mpCurrentKeyFrame->GetPose();
                        //     Ow1 = mpCurrentKeyFrame->GetCameraCenter();

                        //     sophTcw2 = neighbor_kf->GetPose();
                        //     Ow2 = neighbor_kf->GetCameraCenter();

                        //     pCamera1 = mpCurrentKeyFrame->mpCamera;
                        //     pCamera2 = neighbor_kf->mpCamera;
                        // }
                        // eigTcw1 = sophTcw1.matrix3x4();
                        // Rcw1 = eigTcw1.block<3,3>(0,0);
                        // Rwc1 = Rcw1.transpose();
                        // tcw1 = sophTcw1.translation();

                        // eigTcw2 = sophTcw2.matrix3x4();
                        // Rcw2 = eigTcw2.block<3,3>(0,0);
                        // Rwc2 = Rcw2.transpose();
                        // tcw2 = sophTcw2.translation();
                    },
                }

                // Check parallax between rays
                let xn1 = CAMERA_MODULE.unproject_eig(&kp1.pt);
                let xn2 = CAMERA_MODULE.unproject_eig(&kp2.pt);

                // Eigen::Vector3f ray1 = Rwc1 * xn1;
                // Eigen::Vector3f ray2 = Rwc2 * xn2;
                // const float cosParallaxRays = ray1.dot(ray2)/(ray1.norm() * ray2.norm());

                {
                    // TODO (Stereo)
                    // float cosParallaxStereo = cosParallaxRays+1;
                    // float cosParallaxStereo1 = cosParallaxStereo;
                    // float cosParallaxStereo2 = cosParallaxStereo;

                    // if(bStereo1)
                    //     cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
                    // else if(bStereo2)
                    //     cosParallaxStereo2 = cos(2*atan2(neighbor_kf->mb/2,neighbor_kf->mvDepth[idx2]));

                    // if (bStereo1 || bStereo2) totalStereoPts++;

                    // cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);
                }

                // Eigen::Vector3f x3D;

                // bool goodProj = false;
                // bool bPointStereo = false;
                // if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 ||
                //                                                             (cosParallaxRays<0.9996 && mbInertial) || (cosParallaxRays<0.9998 && !mbInertial)))
                // {
                //     goodProj = GeometricTools::Triangulate(xn1, xn2, eigTcw1, eigTcw2, x3D);
                //     if(!goodProj)
                //         continue;
                // }
                // else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
                // {
                //     countStereoAttempt++;
                //     bPointStereo = true;
                //     goodProj = mpCurrentKeyFrame->UnprojectStereo(idx1, x3D);
                // }
                // else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
                // {
                //     countStereoAttempt++;
                //     bPointStereo = true;
                //     goodProj = neighbor_kf->UnprojectStereo(idx2, x3D);
                // }
                // else
                // {
                //     continue; //No stereo and very low parallax
                // }

                // if(goodProj && bPointStereo)
                //     countStereoGoodProj++;

                // if(!goodProj)
                //     continue;

                // //Check triangulation in front of cameras
                // float z1 = Rcw1.row(2).dot(x3D) + tcw1(2);
                // if(z1<=0)
                //     continue;

                // float z2 = Rcw2.row(2).dot(x3D) + tcw2(2);
                // if(z2<=0)
                //     continue;

                // //Check reprojection error in first keyframe
                // const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                // const float x1 = Rcw1.row(0).dot(x3D)+tcw1(0);
                // const float y1 = Rcw1.row(1).dot(x3D)+tcw1(1);
                // const float invz1 = 1.0/z1;

                // if(!bStereo1)
                // {
                //     cv::Point2f uv1 = pCamera1->project(cv::Point3f(x1,y1,z1));
                //     float errX1 = uv1.x - kp1.pt.x;
                //     float errY1 = uv1.y - kp1.pt.y;

                //     if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                //         continue;

                // }
                // else
                // {
                //     float u1 = fx1*x1*invz1+cx1;
                //     float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                //     float v1 = fy1*y1*invz1+cy1;
                //     float errX1 = u1 - kp1.pt.x;
                //     float errY1 = v1 - kp1.pt.y;
                //     float errX1_r = u1_r - kp1_ur;
                //     if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                //         continue;
                // }

                // //Check reprojection error in second keyframe
                // const float sigmaSquare2 = neighbor_kf->mvLevelSigma2[kp2.octave];
                // const float x2 = Rcw2.row(0).dot(x3D)+tcw2(0);
                // const float y2 = Rcw2.row(1).dot(x3D)+tcw2(1);
                // const float invz2 = 1.0/z2;
                // if(!bStereo2)
                // {
                //     cv::Point2f uv2 = pCamera2->project(cv::Point3f(x2,y2,z2));
                //     float errX2 = uv2.x - kp2.pt.x;
                //     float errY2 = uv2.y - kp2.pt.y;
                //     if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                //         continue;
                // }
                // else
                // {
                //     float u2 = fx2*x2*invz2+cx2;
                //     float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                //     float v2 = fy2*y2*invz2+cy2;
                //     float errX2 = u2 - kp2.pt.x;
                //     float errY2 = v2 - kp2.pt.y;
                //     float errX2_r = u2_r - kp2_ur;
                //     if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                //         continue;
                // }

                // //Check scale consistency
                // Eigen::Vector3f normal1 = x3D - Ow1;
                // float dist1 = normal1.norm();

                // Eigen::Vector3f normal2 = x3D - Ow2;
                // float dist2 = normal2.norm();

                // if(dist1==0 || dist2==0)
                //     continue;

                // if(mbFarPoints && (dist1>=mThFarPoints||dist2>=mThFarPoints)) // MODIFICATION
                //     continue;

                // const float ratioDist = dist2/dist1;
                // const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/neighbor_kf->mvScaleFactors[kp2.octave];

                // if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                //     continue;

                // Triangulation is succesfull
                // let new_mp_msg = MapWriteMsg::create_new_mappoint(
                //     MapPoint::<PrelimMapPoint>::new(x3D, self.current_keyframe_id, map.id),
                //     vec![(self.current_keyframe_id, num_keypoints, idx1), (neighbor_id, num_keypoints, idx2)]
                // );
                // map_actor.send_new(new_mp_msg).unwrap();
            }
        }
    }

    fn search_in_neighbors(&self) {
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