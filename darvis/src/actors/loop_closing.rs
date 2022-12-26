use std::sync::Arc;
use axiom::prelude::*;

use dvcore::{
    plugin_functions::Function,
    lockwrap::ReadOnlyWrapper,
};

use dvcore::config::{Sensor, SYSTEM_SETTINGS, GLOBAL_PARAMS};

use log::{warn, info, debug};
use nalgebra::{Matrix3, Similarity3};
use crate::dvmap::keyframe::{KeyFrame, FullKeyFrame};

use crate::dvmap::map_actor::{MAP_ACTOR, KeyframeDatabaseWriteMsg};
use crate::dvmap::mappoint::FullMapPoint;
use crate::dvmap::pose::Pose;
use crate::dvmap::{map::Map};
use crate::modules::imu::ImuModule;

use super::messages::{KeyFrameIdMsg};

#[derive(Debug, Clone, Default)]
pub struct DarvisLoopClosing {
    // Map
    map: ReadOnlyWrapper<Map>,

    // IMU
    imu: ImuModule,

    matches_inliers: i32,
    merge_detected: bool,
    sensor: Sensor,
    current_kf: Option<KeyFrame<FullKeyFrame>>,
    merge_match_kf: Option<KeyFrame<FullKeyFrame>>, 
    mSold_new : Option<Similarity3<f64>>,
    mg2oMergeSlw: Option<Similarity3<f64>>,
    mg2oMergeSmw : Option<Similarity3<f64>>,
    mg2oMergeScw : Option<Similarity3<f64>>,
    mbLoopDetected: bool,
    mg2oLoopSlw : Option<Similarity3<f64>>,
    mg2oLoopScw : Option<Similarity3<f64>>,
    lastcurrent_kf :Option<KeyFrame<FullKeyFrame>>,
    mnLoopNumCoincidences : u32,
    looplastcurrent_kf:Option<KeyFrame<FullKeyFrame>>,
    mvpLoopMPs : Vec<i32>,
    loop_matched_mps: Vec<i32>,
    mnLoopNumNotFound : i32,
    mnMergeNumCoincidences: u32,

    loop_matched_kf: Option<KeyFrame<FullKeyFrame>>, 
    mergelastcurrent_kf:Option<KeyFrame<FullKeyFrame>>,
    merge_matched_mps: Vec<i32>,
    mnMergeNumNotFound : u32,
}


impl DarvisLoopClosing {
    pub fn new(map: ReadOnlyWrapper<Map>) -> DarvisLoopClosing {
        DarvisLoopClosing {
            map: map,
            sensor: GLOBAL_PARAMS.get::<Sensor>(SYSTEM_SETTINGS, "sensor"),
            ..Default::default()
        }
    }

    fn loop_closing(&mut self, context: Context, msg: Arc<KeyFrameIdMsg>) {

            ////Pranay : Why we need this ??
            // if(mpLastCurrentKF)
            // {
            //     mpLastCurrentKF->mvpLoopCandKFs.clear();
            //     mpLastCurrentKF->mvpMergeCandKFs.clear();
            // }

            // setting current keyframe
            self.current_kf = Some(self.map.read().get_keyframe(&msg.keyframe_id).unwrap().clone());

            let b_finded_region = self.new_detect_common_regions(context.clone());

            if b_finded_region
            {
                if self.merge_detected
                {
                    // if ((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
                    //     (!mpCurrentKF->GetMap()->isImuInitialized()))
                    if self.sensor.is_imu() && !self.map.read().imu_initialized 
                    {
                        warn!("loop_closing;IMU is not initilized, merge is aborted...");
                    }
                    else
                    {

                        let mTmw = self.merge_match_kf.as_ref().unwrap().pose;
                        let gSmw2 = Similarity3::from_isometry(mTmw.iso(), 1.0);
                        let mTcw = self.current_kf.as_ref().unwrap().pose;
                        let gScw1 = Similarity3::from_isometry(mTcw.iso(), 1.0);
                        let gSw2c = &self.mg2oMergeSlw.unwrap().inverse();
                        let gSw1m = self.mg2oMergeSlw.unwrap().clone();

                        self.mSold_new = Some(*gSw2c * gScw1);
                        

                        if self.sensor.is_imu()
                        {
                            // Pranay: Not considering multi-map system for now
                            //if(mpCurrentKF->GetMap()->IsInertial() && mpMergeMatchedKF->GetMap()->IsInertial())
                            todo!("Implement for IMU");
                            // cout << "Merge check transformation with IMU" << endl;
                            // if(mSold_new.scale()<0.90||mSold_new.scale()>1.1){
                            //     mpMergeLastCurrentKF->SetErase();
                            //     mpMergeMatchedKF->SetErase();
                            //     mnMergeNumCoincidences = 0;
                            //     mvpMergeMatchedMPs.clear();
                            //     mvpMergeMPs.clear();
                            //     mnMergeNumNotFound = 0;
                            //     mbMergeDetected = false;
                            //     Verbose::PrintMess("scale bad estimated. Abort merging", Verbose::VERBOSITY_NORMAL);
                            //     continue;
                            // }
                            // // If inertial, force only yaw
                            // if ((mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
                            //        mpCurrentKF->GetMap()->GetIniertialBA1())
                            // {
                            //     Eigen::Vector3d phi = LogSO3(mSold_new.rotation().toRotationMatrix());
                            //     phi(0)=0;
                            //     phi(1)=0;
                            //     mSold_new = g2o::Sim3(ExpSO3(phi),mSold_new.translation(),1.0);
                            // }
                        }

                        self.mg2oMergeSmw = Some(gSmw2 * gSw2c * gScw1);

                        self.mg2oMergeScw = self.mg2oMergeSlw.clone();


                        info!("*Merge Detected");

                        if self.sensor.is_imu()
                        {
                            todo!("Implement for IMU");
                            //MergeLocal2();
                        }                            
                        else
                        {
                            self.merge_local();
                        }
                            
                        info!("Merge finished!");
                    }

                    // Reset all variables
                    self.reset_merge_variables();

                    if self.mbLoopDetected
                    {
                        // Reset Loop variables
                        self.reset_loop_variables();
                    }

                }

                if self.mbLoopDetected
                {
                    let mut bGoodLoop = true;
                    // vdPR_CurrentTime.push_back(mpCurrentKF->mTimeStamp);
                    // vdPR_MatchedTime.push_back(mpLoopMatchedKF->mTimeStamp);
                    // vnPR_TypeRecogn.push_back(0);

                    info!("*Loop detected");

                    self.mg2oLoopScw = self.mg2oLoopSlw.clone(); //*mvg2oSim3LoopTcw[nCurrentIndex];

                    if self.sensor.is_imu()
                    //if(mpCurrentKF->GetMap()->IsInertial())
                    {
                        let Twc = &self.current_kf.as_ref().unwrap().pose.inverse();
                        let g2oTwc = Similarity3::from_isometry(Twc.iso(), 1.0);
                        let g2oSww_new = g2oTwc* self.mg2oLoopScw.unwrap();

                        let phi  = g2oSww_new.isometry.rotation.ln();//.ln().vector();
                        //Eigen::Vector3d phi = LogSO3(g2oSww_new.rotation().toRotationMatrix());
                        debug!("phi = {}", phi.vector().transpose());
                        
                        if phi.vector()[0].abs()<0.008 && phi.vector()[1].abs()<0.008 && phi.vector()[2].abs()<0.349
                        {
                            //if(mpCurrentKF->GetMap()->IsInertial())
                            {
                                // If inertial, force only yaw

                                if self.sensor.is_imu()
                                // if ((mpTracker->mSensor==System::IMU_MONOCULAR ||mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD) &&
                                //         mpCurrentKF->GetMap()->GetIniertialBA2())
                                {
                                    todo!(" Implement for IMU");
                                    // phi(0)=0;
                                    // phi(1)=0;
                                    // g2oSww_new = g2o::Sim3(ExpSO3(phi),g2oSww_new.translation(),1.0);
                                    // mg2oLoopScw = g2oTwc.inverse()*g2oSww_new;
                                }
                            }

                        }
                        else
                        {
                            info!(" BAD LOOP !!!");
                            bGoodLoop = false;
                        }

                    }

                    if bGoodLoop {

                        todo!("Pranay : figure out why we need following variables");
                        // mvpLoopMapPoints = mvpLoopMPs;
                        // mnNumCorrection += 1;
                    }

                    // Reset all variables
                    self.reset_loop_variables();

                }

            }
            self.lastcurrent_kf = self.current_kf.clone();


            //ResetIfRequested();
        }

        


        fn new_detect_common_regions(&mut self, context: Context) -> bool
        {
            
            let map_actor = Some(context.system.find_aid_by_name(MAP_ACTOR).unwrap());

            //Pranay : If we have an active loop closing actor, we don't need additional check, if loop closing is active or not
            // // To deactivate placerecognition. No loopclosing nor merging will be performed
            // if(!mbActiveLC)
            //     return false;

            //Pranay : Do we need to set notErase for Keypoint. Below code refers to multi-map case.
            // {
            //     // Avoid that a keyframe can be erased while it is being process by this thread
            //     mpCurrentKF->SetNotErase();
            //     mpCurrentKF->mbCurrentPlaceRecognition = true;

            //     mpLastMap = mpCurrentKF->GetMap();
            // }
            


            if self.sensor.is_imu()
            {
                todo!(" Implement for IMU");
                // if(mpLastMap->IsInertial() && !mpLastMap->GetIniertialBA2())
                // {
                //     mpKeyFrameDB->add(mpCurrentKF);
                //     mpCurrentKF->SetErase();
                //     return false;
                // }
            }


            if !self.sensor.is_mono()
            {
                todo!(" Implement for STEREO");
                // if(mpTracker->mSensor == System::STEREO && mpLastMap->GetAllKeyFrames().size() < 5) //12
                // {
                //     // cout << "LoopClousure: Stereo KF inserted without check: " << mpCurrentKF->mnId << endl;
                //     mpKeyFrameDB->add(mpCurrentKF);
                //     mpCurrentKF->SetErase();
                //     return false;
                // }
            }

            if self.map.read().get_all_keyframes().len() <12 // if(mpLastMap->GetAllKeyFrames().size() < 12)
            {
                // cout << "LoopClousure: Stereo KF inserted without check, map is small: " << mpCurrentKF->mnId << endl;

                // Insert Keyframe in Database
                let map_actor = map_actor.as_ref().unwrap();
                map_actor.send_new(KeyframeDatabaseWriteMsg::add(self.current_kf.as_ref().unwrap().id())).unwrap(); // mpKeyFrameDB->add(mpCurrentKF);                
                //mpCurrentKF->SetErase();
                return false;
            }

            // //cout << "LoopClousure: Checking KF: " << mpCurrentKF->mnId << endl;

            // //Check the last candidates with geometric validation
            // // Loop candidates
            let mut bLoopDetectedInKF = false; // bool bLoopDetectedInKF = false;
            let mut bCheckSpatial = false;// bool bCheckSpatial = false;


            if self.mnLoopNumCoincidences > 0 // if(mnLoopNumCoincidences > 0)
            {
                bCheckSpatial = true;
                // Find from the last KF candidates
                let mTcl = self.current_kf.as_ref().unwrap().pose * self.looplastcurrent_kf.as_ref().unwrap().pose; //Sophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpLoopLastCurrentKF->GetPoseInverse()).cast<double>();
                let gScl = Similarity3::from_isometry(mTcl.iso(), 1.0);//g2o::Sim3 gScl(mTcl.unit_quaternion(),mTcl.translation(),1.0);
                let gScw = gScl *self.mg2oLoopSlw.unwrap();//g2o::Sim3 gScw = gScl * mg2oLoopSlw;
                let mut numProjMatches = 0;//int numProjMatches = 0;
                let mut vpMatchedMPs = Vec::new(); //vector<MapPoint*> vpMatchedMPs;
                let bCommonRegion = DarvisLoopClosing::detect_and_reffine_sim3_from_last_kf(
                &self.current_kf.as_ref().unwrap(), 
                &self.loop_matched_kf.as_ref().unwrap(),
                &gScw,
                numProjMatches,
                &mut self.mvpLoopMPs, 
                &mut vpMatchedMPs );//bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpLoopMatchedKF, gScw, numProjMatches, mvpLoopMPs, vpMatchedMPs);
                if bCommonRegion 
                {

                    bLoopDetectedInKF = true;

                    self.mnLoopNumCoincidences+=1;
                    //mpLoopLastCurrentKF->SetErase();
                    self.looplastcurrent_kf = self.current_kf.clone();//mpLoopLastCurrentKF = mpCurrentKF;
                    self.mg2oLoopSlw = Some(gScw);
                    self.loop_matched_mps = vpMatchedMPs; //mvpLoopMatchedMPs = vpMatchedMPs;


                    self.mbLoopDetected = self.mnLoopNumCoincidences >= 3;
                    self.mnLoopNumNotFound = 0;

                    if !self.mbLoopDetected
                    {
                        warn!("PR: Loop detected with Reffine Sim3"); //cout << "PR: Loop detected with Reffine Sim3" << endl;
                    }
                }
                else
                {
                    bLoopDetectedInKF = false;

                    self.mnLoopNumNotFound+=1;
                    if self.mnLoopNumNotFound >= 2
                    {                       
                        // mpLoopLastCurrentKF->SetErase();
                        // mpLoopMatchedKF->SetErase();
                        self.mnLoopNumCoincidences = 0;
                        self.loop_matched_mps.clear(); //mvpLoopMatchedMPs.clear();
                        self.mvpLoopMPs.clear(); // mvpLoopMPs.clear();
                        self.mnLoopNumNotFound = 0;// mnLoopNumNotFound = 0;

                    }

                }
            }

            // //Merge candidates
            let mut bMergeDetectedInKF = false;// bool bMergeDetectedInKF = false;
            if self.mnMergeNumCoincidences >0 // if(mnMergeNumCoincidences > 0)
            {
                // Find from the last KF candidates
                let mTcl = self.current_kf.as_ref().unwrap().pose * self.mergelastcurrent_kf.as_ref().unwrap().pose;//ophus::SE3d mTcl = (mpCurrentKF->GetPose() * mpMergeLastCurrentKF->GetPoseInverse()).cast<double>();

                let gScl = Similarity3::from_isometry(mTcl.iso(), 1.0);//g2o::Sim3 gScl(mTcl.unit_quaternion(), mTcl.translation(), 1.0);
                let gScw = gScl *self.mg2oMergeSlw.unwrap();// g2o::Sim3 gScw = gScl * mg2oMergeSlw;
                let mut numProjMatches = 0;//int numProjMatches = 0;
                let mut vpMatchedMPs = Vec::new(); //vector<MapPoint*> vpMatchedMPs;
                let bCommonRegion = DarvisLoopClosing::detect_and_reffine_sim3_from_last_kf(
                &self.current_kf.as_ref().unwrap(), 
                &self.merge_match_kf.as_ref().unwrap(),
                &gScw,
                numProjMatches,
                &mut self.mvpLoopMPs, 
                &mut vpMatchedMPs ); //bool bCommonRegion = DetectAndReffineSim3FromLastKF(mpCurrentKF, mpMergeMatchedKF, gScw, numProjMatches, mvpMergeMPs, vpMatchedMPs);
                if bCommonRegion
                {
                    bMergeDetectedInKF = true;

                    self.mnMergeNumCoincidences+=1;
                    //mpMergeLastCurrentKF->SetErase();
                    self.mergelastcurrent_kf = self.current_kf.clone(); //mpMergeLastCurrentKF = mpCurrentKF;
                    self.mg2oMergeSlw = Some(gScw);
                    self.merge_matched_mps = vpMatchedMPs ; //mvpMergeMatchedMPs = vpMatchedMPs;

                    self.merge_detected = self.mnMergeNumCoincidences >= 3 ; //mbMergeDetected = mnMergeNumCoincidences >= 3;
                }
                else
                {
                    self.merge_detected = false;// mbMergeDetected = false;
                    bMergeDetectedInKF = false;

                    self.mnMergeNumNotFound+=1;
                    if(self.mnMergeNumNotFound >= 2)
                    {
                        todo!("fix merge variable clear");
                        // mpMergeLastCurrentKF->SetErase();
                        // mpMergeMatchedKF->SetErase();
                        // mnMergeNumCoincidences = 0;
                        // mvpMergeMatchedMPs.clear();
                        // mvpMergeMPs.clear();
                        self.mnMergeNumNotFound = 0;
                    }


                }
            }  


            // if(mbMergeDetected || mbLoopDetected)
            // {

            //     mpKeyFrameDB->add(mpCurrentKF);
            //     return true;
            // }

            // //TODO: This is only necessary if we use a minimun score for pick the best candidates
            // const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

            // // Extract candidates from the bag of words
            // vector<KeyFrame*> vpMergeBowCand, vpLoopBowCand;
            // if(!bMergeDetectedInKF || !bLoopDetectedInKF)
            // {
            //     // Search in BoW

            //     mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand,3);

            // }


            // // Check the BoW candidates if the geometric candidate list is empty
            // //Loop candidates
            // if(!bLoopDetectedInKF && !vpLoopBowCand.empty())
            // {
            //     mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
            // }
            // // Merge candidates
            // if(!bMergeDetectedInKF && !vpMergeBowCand.empty())
            // {
            //     mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
            // }



            // mpKeyFrameDB->add(mpCurrentKF);

            // if(mbMergeDetected || mbLoopDetected)
            // {
            //     return true;
            // }

            // mpCurrentKF->SetErase();
            // mpCurrentKF->mbCurrentPlaceRecognition = false;

            return false;

        }
    
        fn reset_merge_variables(&mut self) 
        {
            todo!("reset_merge_variables");
            //// Reset all variables
            // mpMergeLastCurrentKF->SetErase();
            // mpMergeMatchedKF->SetErase();
            // mnMergeNumCoincidences = 0;
            // mvpMergeMatchedMPs.clear();
            // mvpMergeMPs.clear();
            // mnMergeNumNotFound = 0;
            // mbMergeDetected = false;
        }
    
        fn reset_loop_variables(&mut self) 
        {
            
            // // Reset Loop variables
            // // Pranay : why we need SetErase for keyframe?
            // mpLoopLastCurrentKF->SetErase();
            // mpLoopMatchedKF->SetErase();
            self.mnLoopNumCoincidences = 0; // mnLoopNumCoincidences = 0;
            self.loop_matched_mps.clear(); //mvpLoopMatchedMPs.clear();
            self.mvpLoopMPs.clear(); // mvpLoopMPs.clear();
            self.mnLoopNumNotFound = 0;// mnLoopNumNotFound = 0;
            self.mbLoopDetected= false;

        }

        fn merge_local(&mut self)
        {
            todo!("Implement merge_local");
        }


        //bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        fn detect_and_reffine_sim3_from_last_kf(current_kf: &KeyFrame<FullKeyFrame>, matched_kf : &KeyFrame<FullKeyFrame>, gScw: &Similarity3<f64>, num_proj_matches: i32, vmMPs: &mut Vec<i32>, vpMatchedMPs : &mut Vec<i32>) -> bool
        {
            todo!("Implement detect_and_reffine_sim3_from_last_kf");
            // set<MapPoint*> spAlreadyMatchedMPs;
            // nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

            // int nProjMatches = 30;
            // int nProjOptMatches = 50;
            // int nProjMatchesRep = 100;

            // if(nNumProjMatches >= nProjMatches)
            // {
            // //Verbose::PrintMess("Sim3 reffine: There are " + to_string(nNumProjMatches) + " initial matches ", Verbose::VERBOSITY_DEBUG);
            // Sophus::SE3d mTwm = pMatchedKF->GetPoseInverse().cast<double>();
            // g2o::Sim3 gSwm(mTwm.unit_quaternion(),mTwm.translation(),1.0);
            // g2o::Sim3 gScm = gScw * gSwm;
            // Eigen::Matrix<double, 7, 7> mHessian7x7;

            // bool bFixedScale = mbFixScale;       // TODO CHECK; Solo para el monocular inertial
            // if(mpTracker->mSensor==System::IMU_MONOCULAR && !pCurrentKF->GetMap()->GetIniertialBA2())
            // bFixedScale=false;
            // int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pMatchedKF, vpMatchedMPs, gScm, 10, bFixedScale, mHessian7x7, true);

            // //Verbose::PrintMess("Sim3 reffine: There are " + to_string(numOptMatches) + " matches after of the optimization ", Verbose::VERBOSITY_DEBUG);

            // if(numOptMatches > nProjOptMatches)
            // {
            // g2o::Sim3 gScw_estimation(gScw.rotation(), gScw.translation(),1.0);

            // vector<MapPoint*> vpMatchedMP;
            // vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));

            // nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
            // if(nNumProjMatches >= nProjMatchesRep)
            // {
            // gScw = gScw_estimation;
            // return true;
            // }
            // }
            // }
            // return false;
        }

    }





impl Function for DarvisLoopClosing {
    fn handle(&mut self, context: axiom::prelude::Context, message: Message) -> ActorResult<()>
    {
        if let Some(msg) = message.content_as::<KeyFrameIdMsg>() {
            self.loop_closing(context, msg);
        }
        Ok(Status::done(()))
    }
}