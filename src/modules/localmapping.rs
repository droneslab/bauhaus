use std::sync::Arc;
use axiom::prelude::*;
use nalgebra::Vector3;
use serde::{Serialize, Deserialize};

use darvis::{
    plugin_functions::Function,
    map::{map::Id, map::Map, map_actor::MAP_ACTOR},
    lockwrap::ReadOnlyWrapper,
    utils::sensor::SensorType,
};
use crate::modules::messages::keyframe_msg::KeyFrameMsg;

#[derive(Debug, Clone)]
pub struct DarvisLocalMapping<S: SensorType> {
    // Map
    map: ReadOnlyWrapper<Map<S>>,
    map_actor: Option<Aid>,

    matches_inliers: i32,
}


impl<S: SensorType + 'static> DarvisLocalMapping<S> {
    pub fn new(map: ReadOnlyWrapper<Map<S>>) -> DarvisLocalMapping<S> {
        DarvisLocalMapping {
            map: map,
            map_actor: None,
            matches_inliers: 0
        }
    }
    fn local_mapping(&mut self, _context: Context, msg: Arc<KeyFrameMsg>) {
        let map_actor = msg.actor_ids.get(MAP_ACTOR).unwrap();
        self.map_actor = Some(map_actor.clone());

    //     mbFinished = false;

    //     while(1)
    //     {
    //         // Tracking will see that Local Mapping is busy
    //         SetAcceptKeyFrames(false);

    //         // Check if there are keyframes in the queue
    //         if(CheckNewKeyFrames() && !mbBadImu)
    //         {
    // #ifdef REGISTER_TIMES
    //             double timeLBA_ms = 0;
    //             double timeKFCulling_ms = 0;

    //             std::chrono::steady_clock::time_point time_StartProcessKF = std::chrono::steady_clock::now();
    // #endif
    //             // BoW conversion and insertion in Map
    //             ProcessNewKeyFrame();
    // #ifdef REGISTER_TIMES
    //             std::chrono::steady_clock::time_point time_EndProcessKF = std::chrono::steady_clock::now();

    //             double timeProcessKF = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndProcessKF - time_StartProcessKF).count();
    //             vdKFInsert_ms.push_back(timeProcessKF);
    // #endif

    //             // Check recent MapPoints
    //             MapPointCulling();
    // #ifdef REGISTER_TIMES
    //             std::chrono::steady_clock::time_point time_EndMPCulling = std::chrono::steady_clock::now();

    //             double timeMPCulling = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCulling - time_EndProcessKF).count();
    //             vdMPCulling_ms.push_back(timeMPCulling);
    // #endif

    //             // Triangulate new MapPoints
    //             CreateNewMapPoints();

    //             mbAbortBA = false;

    //             if(!CheckNewKeyFrames())
    //             {
    //                 // Find more matches in neighbor keyframes and fuse point duplications
    //                 SearchInNeighbors();
    //             }

    // #ifdef REGISTER_TIMES
    //             std::chrono::steady_clock::time_point time_EndMPCreation = std::chrono::steady_clock::now();

    //             double timeMPCreation = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndMPCreation - time_EndMPCulling).count();
    //             vdMPCreation_ms.push_back(timeMPCreation);
    // #endif

    //             bool b_doneLBA = false;
    //             int num_FixedKF_BA = 0;
    //             int num_OptKF_BA = 0;
    //             int num_MPs_BA = 0;
    //             int num_edges_BA = 0;

    //             if(!CheckNewKeyFrames() && !stopRequested())
    //             {
    //                 if(mpAtlas->KeyFramesInMap()>2)
    //                 {

    //                     if(mbInertial && mpCurrentKeyFrame->GetMap()->isImuInitialized())
    //                     {
    //                         float dist = (mpCurrentKeyFrame->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->GetCameraCenter()).norm() +
    //                                 (mpCurrentKeyFrame->mPrevKF->mPrevKF->GetCameraCenter() - mpCurrentKeyFrame->mPrevKF->GetCameraCenter()).norm();

    //                         if(dist>0.05)
    //                             mTinit += mpCurrentKeyFrame->mTimeStamp - mpCurrentKeyFrame->mPrevKF->mTimeStamp;
    //                         if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2())
    //                         {
    //                             if((mTinit<10.f) && (dist<0.02))
    //                             {
    //                                 cout << "Not enough motion for initializing. Reseting..." << endl;
    //                                 unique_lock<mutex> lock(mMutexReset);
    //                                 mbResetRequestedActiveMap = true;
    //                                 mpMapToReset = mpCurrentKeyFrame->GetMap();
    //                                 mbBadImu = true;
    //                             }
    //                         }

    //                         bool bLarge = ((mpTracker->GetMatchesInliers()>75)&&mbMonocular)||((mpTracker->GetMatchesInliers()>100)&&!mbMonocular);
    //                         Optimizer::LocalInertialBA(mpCurrentKeyFrame, &mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA, bLarge, !mpCurrentKeyFrame->GetMap()->GetIniertialBA2());
    //                         b_doneLBA = true;
    //                     }
    //                     else
    //                     {
    //                         Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpCurrentKeyFrame->GetMap(),num_FixedKF_BA,num_OptKF_BA,num_MPs_BA,num_edges_BA);
    //                         b_doneLBA = true;
    //                     }

    //                 }
    // #ifdef REGISTER_TIMES
    //                 std::chrono::steady_clock::time_point time_EndLBA = std::chrono::steady_clock::now();

    //                 if(b_doneLBA)
    //                 {
    //                     timeLBA_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLBA - time_EndMPCreation).count();
    //                     vdLBA_ms.push_back(timeLBA_ms);

    //                     nLBA_exec += 1;
    //                     if(mbAbortBA)
    //                     {
    //                         nLBA_abort += 1;
    //                     }
    //                     vnLBA_edges.push_back(num_edges_BA);
    //                     vnLBA_KFopt.push_back(num_OptKF_BA);
    //                     vnLBA_KFfixed.push_back(num_FixedKF_BA);
    //                     vnLBA_MPs.push_back(num_MPs_BA);
    //                 }

    // #endif

    //                 // Initialize IMU here
    //                 if(!mpCurrentKeyFrame->GetMap()->isImuInitialized() && mbInertial)
    //                 {
    //                     if (mbMonocular)
    //                         InitializeIMU(1e2, 1e10, true);
    //                     else
    //                         InitializeIMU(1e2, 1e5, true);
    //                 }


    //                 // Check redundant local Keyframes
    //                 KeyFrameCulling();

    // #ifdef REGISTER_TIMES
    //                 std::chrono::steady_clock::time_point time_EndKFCulling = std::chrono::steady_clock::now();

    //                 timeKFCulling_ms = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndKFCulling - time_EndLBA).count();
    //                 vdKFCulling_ms.push_back(timeKFCulling_ms);
    // #endif

    //                 if ((mTinit<50.0f) && mbInertial)
    //                 {
    //                     if(mpCurrentKeyFrame->GetMap()->isImuInitialized() && mpTracker->mState==Tracking::OK) // Enter here everytime local-mapping is called
    //                     {
    //                         if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA1()){
    //                             if (mTinit>5.0f)
    //                             {
    //                                 cout << "start VIBA 1" << endl;
    //                                 mpCurrentKeyFrame->GetMap()->SetIniertialBA1();
    //                                 if (mbMonocular)
    //                                     InitializeIMU(1.f, 1e5, true);
    //                                 else
    //                                     InitializeIMU(1.f, 1e5, true);

    //                                 cout << "end VIBA 1" << endl;
    //                             }
    //                         }
    //                         else if(!mpCurrentKeyFrame->GetMap()->GetIniertialBA2()){
    //                             if (mTinit>15.0f){
    //                                 cout << "start VIBA 2" << endl;
    //                                 mpCurrentKeyFrame->GetMap()->SetIniertialBA2();
    //                                 if (mbMonocular)
    //                                     InitializeIMU(0.f, 0.f, true);
    //                                 else
    //                                     InitializeIMU(0.f, 0.f, true);

    //                                 cout << "end VIBA 2" << endl;
    //                             }
    //                         }

    //                         // scale refinement
    //                         if (((mpAtlas->KeyFramesInMap())<=200) &&
    //                                 ((mTinit>25.0f && mTinit<25.5f)||
    //                                 (mTinit>35.0f && mTinit<35.5f)||
    //                                 (mTinit>45.0f && mTinit<45.5f)||
    //                                 (mTinit>55.0f && mTinit<55.5f)||
    //                                 (mTinit>65.0f && mTinit<65.5f)||
    //                                 (mTinit>75.0f && mTinit<75.5f))){
    //                             if (mbMonocular)
    //                                 ScaleRefinement();
    //                         }
    //                     }
    //                 }
    //             }

    // #ifdef REGISTER_TIMES
    //             vdLBASync_ms.push_back(timeKFCulling_ms);
    //             vdKFCullingSync_ms.push_back(timeKFCulling_ms);
    // #endif

    //             mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);

    // #ifdef REGISTER_TIMES
    //             std::chrono::steady_clock::time_point time_EndLocalMap = std::chrono::steady_clock::now();

    //             double timeLocalMap = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(time_EndLocalMap - time_StartProcessKF).count();
    //             vdLMTotal_ms.push_back(timeLocalMap);
    // #endif
    //         }
    //         else if(Stop() && !mbBadImu)
    //         {
    //             // Safe area to stop
    //             while(isStopped() && !CheckFinish())
    //             {
    //                 usleep(3000);
    //             }
    //             if(CheckFinish())
    //                 break;
    //         }

    //         ResetIfRequested();

    //         // Tracking will see that Local Mapping is busy
    //         SetAcceptKeyFrames(true);

    //         if(CheckFinish())
    //             break;

    //         usleep(3000);
    //     }

    //     SetFinish();
    // }
    }
}
impl<S: SensorType + 'static> Function for DarvisLocalMapping<S> {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        if let Some(msg) = message.content_as::<KeyFrameMsg>() {
            self.local_mapping(context, msg);
        }

        Ok(Status::done(()))
    }
}