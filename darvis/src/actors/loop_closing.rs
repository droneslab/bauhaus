use dvcore::actor::{Actor, ActorMessage};
use dvcore::config::{SETTINGS, SYSTEM};
use dvcore::sensor::Sensor;
use log::warn;
use crate::{ActorChannels, MapLock};
use crate::dvmap::map::{Id, Map};
use crate::modules::imu::ImuModule;

use super::messages::{ShutdownMsg, IMUInitializedMsg};

#[derive(Debug)]
pub struct LoopClosing {
    actor_channels: ActorChannels,
    // Map
    map: MapLock,

    // IMU
    imu: ImuModule,

    matches_inliers: i32,
}

impl Actor for LoopClosing {
    type MapRef = MapLock;

    fn new_actorstate(actor_channels: ActorChannels, map: Self::MapRef) -> LoopClosing {
        let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");

        LoopClosing {
            actor_channels,
            map,
            imu: ImuModule::new(None, None, sensor, false, false),
            matches_inliers: 0,
        }
    }

    fn spawn(actor_channels: ActorChannels, map: Self::MapRef) {
        let mut actor = LoopClosing::new_actorstate(actor_channels, map);
        tracy_client::set_thread_name!("loop closing");

        'outer: loop {
            let message = actor.actor_channels.receive().unwrap();
            if message.is::<LoopClosingMsg>() {
                let msg = message.downcast::<LoopClosingMsg>().unwrap_or_else(|_| panic!("Could not downcast loop closing message!"));
                match *msg {
                    LoopClosingMsg::KeyFrameIdMsg{ kf_id } => {
                        actor.loop_closing(kf_id);
                    },
                };
            } else if message.is::<IMUInitializedMsg>() {
                // TODO (IMU) process message from local mapping!
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Loop Closing received unknown message type!");
            }
        }
    }


}

impl LoopClosing {

    fn loop_closing(&mut self, kf_id: Id) {
        // if(mpLastCurrentKF)
        // {
        //     mpLastCurrentKF->mvpLoopCandKFs.clear();
        //     mpLastCurrentKF->mvpMergeCandKFs.clear();
        // }

        // bool bFindedRegion = NewDetectCommonRegions();

        // if(bFindedRegion)
        // {
        //     if(mbMergeDetected)
        //     {
        //         if ((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
        //             (!mpCurrentKF->GetMap()->isImuInitialized()))
        //         {
        //             cout << "IMU is not initilized, merge is aborted" << endl;
        //         }
        //         else
        //         {
        //             Sophus::SE3d mTmw = mpMergeMatchedKF->GetPose().cast<double>();
        //             g2o::Sim3 gSmw2(mTmw.unit_quaternion(), mTmw.translation(), 1.0);
        //             Sophus::SE3d mTcw = mpCurrentKF->GetPose().cast<double>();
        //             g2o::Sim3 gScw1(mTcw.unit_quaternion(), mTcw.translation(), 1.0);
        //             g2o::Sim3 gSw2c = mg2oMergeSlw.inverse();
        //             g2o::Sim3 gSw1m = mg2oMergeSlw;

        //             mSold_new = (gSw2c * gScw1);


        //             if(mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
        //             {
        //                 cout << "Merge check transformation with IMU" << endl;
        //                 if(mSold_new.scale()<0.90||mSold_new.scale()>1.1){
        //                     mpMergeLastCurrentKF->SetErase();
        //                     mpMergeMatchedKF->SetErase();
        //                     mnMergeNumCoincidences = 0;
        //                     mvpMergeMatchedMPs.clear();
        //                     mvpMergeMPs.clear();
        //                     mnMergeNumNotFound = 0;
        //                     mbMergeDetected = false;
        //                     Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
        //                     continue;
        //                 }
        //                 // If inertial, force only yaw
        //                 if ((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
        //                         mpCurrentKF->GetMap()->GetIniertialBA1())
        //                 {
        //                     Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
        //                     phi(0)=0;
        //                     phi(1)=0;
        //                     mSold_new = g2o::Sim3(ExpSO3(phi),mSold_new.translation(),1.0);
        //                 }
        //             }

        //             mg2oMergeSmw = gSmw2 * gSw2c * gScw1;

        //             mg2oMergeScw = mg2oMergeSlw;

        //             //mpTracker->SetStepByStep(true);

        //             Verbose::PrintMess("*Merge detected", Verbose::VERBOSITY_QUIET);

        //             if (mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD)
        //                 MergeLocal2();
        //             else
        //                 MergeLocal();


        //             Verbose::PrintMess("Merge finished!", Verbose::VERBOSITY_QUIET);
        //         }

        //         vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
        //         vdPR_MatchedTime.push_back(mpMergeMatchedKF->mTimeStamp);
        //         vnPR_TypeRecogn.push_back(1);

        //         // Reset all variables
        //         mpMergeLastCurrentKF->SetErase();
        //         mpMergeMatchedKF->SetErase();
        //         mnMergeNumCoincidences = 0;
        //         mvpMergeMatchedMPs.clear();
        //         mvpMergeMPs.clear();
        //         mnMergeNumNotFound = 0;
        //         mbMergeDetected = false;

        //         if(mbLoopDetected)
        //         {
        //             // Reset Loop variables
        //             mpLoopLastCurrentKF->SetErase();
        //             mpLoopMatchedKF->SetErase();
        //             mnLoopNumCoincidences = 0;
        //             mvpLoopMatchedMPs.clear();
        //             mvpLoopMPs.clear();
        //             mnLoopNumNotFound = 0;
        //             mbLoopDetected = false;
        //         }

        //     }

        //     if(mbLoopDetected)
        //     {
        //         bool bGoodLoop = true;
        //         vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
        //         vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
        //         vnPR_TypeRecogn.push_back(0);

        //         Verbose::PrintMess("*Loop detected", Verbose::VERBOSITY_QUIET);

        //         mg2oLoopScw = mg2oLoopSlw; //*mvg2oSim3LoopTcw[nCurrentIndex];
        //         if(mpCurrentKF->GetMap()->IsInertial())
        //         {
        //             Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
        //             g2o::Sim3 g2oTwc(Twc.unit_quaternion(),Twc.translation(),1.0);
        //             g2o::Sim3 g2oSww_new = g2oTwc*mg2oLoopScw;

        //             Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
        //             cout << "phi = " << phi.transpose() << endl; 
        //             if (fabs(phi(0))<0.008f && fabs(phi(1))<0.008f && fabs(phi(2))<0.349f)
        //             {
        //                 if(mpCurrentKF->GetMap()->IsInertial())
        //                 {
        //                     // If inertial, force only yaw
        //                     if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
        //                             mpCurrentKF->GetMap()->GetIniertialBA2())
        //                     {
        //                         phi(0)=0;
        //                         phi(1)=0;
        //                         g2oSww_new = g2o::Sim3(ExpSO3(phi),g2oSww_new.translation(),1.0);
        //                         mg2oLoopScw = g2oTwc.inverse()*g2oSww_new;
        //                     }
        //                 }

        //             }
        //             else
        //             {
        //                 cout << "BAD LOOP!!!" << endl;
        //                 bGoodLoop = false;
        //             }

        //         }

        //         if (bGoodLoop) {

        //             mvpLoopMapPoints = mvpLoopMPs;

        //             mnNumCorrection += 1;
        //         }

        //         // Reset all variables
        //         mpLoopLastCurrentKF->SetErase();
        //         mpLoopMatchedKF->SetErase();
        //         mnLoopNumCoincidences = 0;
        //         mvpLoopMatchedMPs.clear();
        //         mvpLoopMPs.clear();
        //         mnLoopNumNotFound = 0;
        //         mbLoopDetected = false;
        //     }

        // }
        // mpLastCurrentKF = mpCurrentKF;
    }
}


pub enum LoopClosingMsg {
    KeyFrameIdMsg{ kf_id: Id },
}
impl ActorMessage for LoopClosingMsg {}