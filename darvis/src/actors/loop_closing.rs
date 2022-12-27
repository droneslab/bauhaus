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
use crate::registered_modules::LOOP_CLOSING;

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
    mvpMergeMPs : Vec<i32>,
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
                self.add_keyframe_to_database(context, self.current_kf.as_ref().unwrap().id());// mpKeyFrameDB->add(mpCurrentKF);
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
                    if self.mnMergeNumNotFound >= 2
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


            if self.merge_detected || self.mbLoopDetected //if(mbMergeDetected || mbLoopDetected)
            {
                self.add_keyframe_to_database(context, self.current_kf.as_ref().unwrap().id()); //mpKeyFrameDB->add(mpCurrentKF);
                return true;
            }

            // //TODO: This is only necessary if we use a minimun score for pick the best candidates
            let vpConnectedKeyFrames = self.current_kf.as_ref().unwrap().get_connections(30);// const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();

            // // Extract candidates from the bag of words
            let (mut vpMergeBowCand, mut vpLoopBowCand)= (Vec::new(), Vec::new());// vector<KeyFrame*> vpMergeBowCand, vpLoopBowCand;
            if !bMergeDetectedInKF || !bLoopDetectedInKF
            {
                // Search in BoW

                let kf_database = self.map.read().keyframe_database.as_ref().unwrap().clone();
                kf_database.detect_n_best_candidates(&self.current_kf.as_ref().unwrap(), &mut vpLoopBowCand, &mut vpMergeBowCand, 3); //mpKeyFrameDB->DetectNBestCandidates(mpCurrentKF, vpLoopBowCand, vpMergeBowCand,3);

            }


            // // Check the BoW candidates if the geometric candidate list is empty
            // //Loop candidates
            if !bLoopDetectedInKF && !vpLoopBowCand.is_empty()
            {
                self.mbLoopDetected =  DarvisLoopClosing::detect_common_regions_from_bow(&vpLoopBowCand, self.loop_matched_kf.as_ref().unwrap(), self.looplastcurrent_kf.as_ref().unwrap(), self.mg2oLoopSlw.as_ref().unwrap(), self.mnLoopNumCoincidences as i32, &mut self.mvpLoopMPs, &mut self.loop_matched_mps); //mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
            }
            // // Merge candidates
            if !bMergeDetectedInKF && !vpMergeBowCand.is_empty()
            {
                self.merge_detected = DarvisLoopClosing::detect_common_regions_from_bow(&vpMergeBowCand, self.merge_match_kf.as_ref().unwrap(), self.mergelastcurrent_kf.as_ref().unwrap(), self.mg2oMergeSlw.as_ref().unwrap(), self.mnMergeNumCoincidences as i32, &mut self.mvpMergeMPs, &mut self.merge_matched_mps) ;//mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
            }


            self.add_keyframe_to_database(context, self.current_kf.as_ref().unwrap().id()); // mpKeyFrameDB->add(mpCurrentKF);

            if self.merge_detected  || self.mbLoopDetected 
            {
                return true;
            }

            // // Pranay : need to check why we set these variables
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

        fn add_keyframe_to_database(&self, context: Context,  kf_id : i32)
        {
                let map_actor = Some(context.system.find_aid_by_name(MAP_ACTOR).unwrap());
                // Insert Keyframe in Database
                let map_actor = map_actor.as_ref().unwrap();
                map_actor.send_new(KeyframeDatabaseWriteMsg::add(kf_id)).unwrap(); // mpKeyFrameDB->add
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


        //bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF2, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw, int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        fn detect_common_regions_from_bow(vpBowCand : &Vec<i32>, pMatchedKF2 : &KeyFrame<FullKeyFrame>, pLastCurrentKF : &KeyFrame<FullKeyFrame>,  g2oScw: &Similarity3<f64>, nNumCoincidences : i32, vpMPs: &mut Vec<i32>,  vpMatchedMPs : &mut Vec<i32>) -> bool
        {
            todo!("Implement detect_common_regions_from_bow");
            // int nBoWMatches = 20;
            // int nBoWInliers = 15;
            // int nSim3Inliers = 20;
            // int nProjMatches = 50;
            // int nProjOptMatches = 80;

            // set<KeyFrame*> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();

            // int nNumCovisibles = 10;

            // ORBmatcher matcherBoW(0.9, true);
            // ORBmatcher matcher(0.75, true);

            // // Varibles to select the best numbe
            // KeyFrame* pBestMatchedKF;
            // int nBestMatchesReproj = 0;
            // int nBestNumCoindicendes = 0;
            // g2o::Sim3 g2oBestScw;
            // std::vector<MapPoint*> vpBestMapPoints;
            // std::vector<MapPoint*> vpBestMatchedMapPoints;

            // int numCandidates = vpBowCand.size();
            // vector<int> vnStage(numCandidates, 0);
            // vector<int> vnMatchesStage(numCandidates, 0);

            // int index = 0;
            // //Verbose::PrintMess("BoW candidates: There are " + to_string(vpBowCand.size()) + " possible candidates ", Verbose::VERBOSITY_DEBUG);
            // for(KeyFrame* pKFi : vpBowCand)
            // {
            // if(!pKFi || pKFi->isBad())
            // continue;

            // // std::cout << "KF candidate: " << pKFi->mnId << std::endl;
            // // Current KF against KF with covisibles version
            // std::vector<KeyFrame*> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
            // if(vpCovKFi.empty())
            // {
            // std::cout << "Covisible list empty" << std::endl;
            // vpCovKFi.push_back(pKFi);
            // }
            // else
            // {
            // vpCovKFi.push_back(vpCovKFi[0]);
            // vpCovKFi[0] = pKFi;
            // }


            // bool bAbortByNearKF = false;
            // for(int j=0; j<vpCovKFi.size(); ++j)
            // {
            // if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
            // {
            // bAbortByNearKF = true;
            // break;
            // }
            // }
            // if(bAbortByNearKF)
            // {
            // //std::cout << "Check BoW aborted because is close to the matched one " << std::endl;
            // continue;
            // }
            // //std::cout << "Check BoW continue because is far to the matched one " << std::endl;


            // std::vector<std::vector<MapPoint*> > vvpMatchedMPs;
            // vvpMatchedMPs.resize(vpCovKFi.size());
            // std::set<MapPoint*> spMatchedMPi;
            // int numBoWMatches = 0;

            // KeyFrame* pMostBoWMatchesKF = pKFi;
            // int nMostBoWNumMatches = 0;

            // std::vector<MapPoint*> vpMatchedPoints = std::vector<MapPoint*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
            // std::vector<KeyFrame*> vpKeyFrameMatchedMP = std::vector<KeyFrame*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));

            // int nIndexMostBoWMatchesKF=0;
            // for(int j=0; j<vpCovKFi.size(); ++j)
            // {
            // if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
            // continue;

            // int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
            // if (num > nMostBoWNumMatches)
            // {
            // nMostBoWNumMatches = num;
            // nIndexMostBoWMatchesKF = j;
            // }
            // }

            // for(int j=0; j<vpCovKFi.size(); ++j)
            // {
            // for(int k=0; k < vvpMatchedMPs[j].size(); ++k)
            // {
            // MapPoint* pMPi_j = vvpMatchedMPs[j][k];
            // if(!pMPi_j || pMPi_j->isBad())
            // continue;

            // if(spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
            // {
            // spMatchedMPi.insert(pMPi_j);
            // numBoWMatches++;

            // vpMatchedPoints[k]= pMPi_j;
            // vpKeyFrameMatchedMP[k] = vpCovKFi[j];
            // }
            // }
            // }

            // //pMostBoWMatchesKF = vpCovKFi[pMostBoWMatchesKF];

            // if(numBoWMatches >= nBoWMatches) // TODO pick a good threshold
            // {
            // // Geometric validation
            // bool bFixedScale = mbFixScale;
            // if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
            // bFixedScale=false;

            // Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
            // solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers

            // bool bNoMore = false;
            // vector<bool> vbInliers;
            // int nInliers;
            // bool bConverge = false;
            // Eigen::Matrix4f mTcm;
            // while(!bConverge && !bNoMore)
            // {
            // mTcm = solver.iterate(20,bNoMore, vbInliers, nInliers, bConverge);
            // //Verbose::PrintMess("BoW guess: Solver achieve " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
            // }

            // if(bConverge)
            // {
            // //std::cout << "Check BoW: SolverSim3 converged" << std::endl;

            // //Verbose::PrintMess("BoW guess: Convergende with " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
            // // Match by reprojection
            // vpCovKFi.clear();
            // vpCovKFi = pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
            // vpCovKFi.push_back(pMostBoWMatchesKF);
            // set<KeyFrame*> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

            // //std::cout << "There are " << vpCovKFi.size() <<" near KFs" << std::endl;

            // set<MapPoint*> spMapPoints;
            // vector<MapPoint*> vpMapPoints;
            // vector<KeyFrame*> vpKeyFrames;
            // for(KeyFrame* pCovKFi : vpCovKFi)
            // {
            // for(MapPoint* pCovMPij : pCovKFi->GetMapPointMatches())
            // {
            // if(!pCovMPij || pCovMPij->isBad())
            // continue;

            // if(spMapPoints.find(pCovMPij) == spMapPoints.end())
            // {
            // spMapPoints.insert(pCovMPij);
            // vpMapPoints.push_back(pCovMPij);
            // vpKeyFrames.push_back(pCovKFi);
            // }
            // }
            // }

            // //std::cout << "There are " << vpKeyFrames.size() <<" KFs which view all the mappoints" << std::endl;

            // g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(),solver.GetEstimatedTranslation().cast<double>(), (double) solver.GetEstimatedScale());
            // g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),pMostBoWMatchesKF->GetTranslation().cast<double>(),1.0);
            // g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
            // Sophus::Sim3f mScw = Converter::toSophus(gScw);

            // vector<MapPoint*> vpMatchedMP;
            // vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
            // vector<KeyFrame*> vpMatchedKF;
            // vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));
            // int numProjMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);
            // //cout <<"BoW: " << numProjMatches << " matches between " << vpMapPoints.size() << " points with coarse Sim3" << endl;

            // if(numProjMatches >= nProjMatches)
            // {
            // // Optimize Sim3 transformation with every matches
            // Eigen::Matrix<double, 7, 7> mHessian7x7;

            // bool bFixedScale = mbFixScale;
            // if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
            // bFixedScale=false;

            // int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale, mHessian7x7, true);

            // if(numOptMatches >= nSim3Inliers)
            // {
            // g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),pMostBoWMatchesKF->GetTranslation().cast<double>(),1.0);
            // g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
            // Sophus::Sim3f mScw = Converter::toSophus(gScw);

            // vector<MapPoint*> vpMatchedMP;
            // vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
            // int numProjOptMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);

            // if(numProjOptMatches >= nProjOptMatches)
            // {
            // int max_x = -1, min_x = 1000000;
            // int max_y = -1, min_y = 1000000;
            // for(MapPoint* pMPi : vpMatchedMP)
            // {
            // if(!pMPi || pMPi->isBad())
            // {
            //    continue;
            // }

            // tuple<size_t,size_t> indexes = pMPi->GetIndexInKeyFrame(pKFi);
            // int index = get<0>(indexes);
            // if(index >= 0)
            // {
            //    int coord_x = pKFi->mvKeysUn[index].pt.x;
            //    if(coord_x < min_x)
            //    {
            //        min_x = coord_x;
            //    }
            //    if(coord_x > max_x)
            //    {
            //        max_x = coord_x;
            //    }
            //    int coord_y = pKFi->mvKeysUn[index].pt.y;
            //    if(coord_y < min_y)
            //    {
            //        min_y = coord_y;
            //    }
            //    if(coord_y > max_y)
            //    {
            //        max_y = coord_y;
            //    }
            // }
            // }

            // int nNumKFs = 0;
            // //vpMatchedMPs = vpMatchedMP;
            // //vpMPs = vpMapPoints;
            // // Check the Sim3 transformation with the current KeyFrame covisibles
            // vector<KeyFrame*> vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

            // int j = 0;
            // while(nNumKFs < 3 && j<vpCurrentCovKFs.size())
            // {
            // KeyFrame* pKFj = vpCurrentCovKFs[j];
            // Sophus::SE3d mTjc = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
            // g2o::Sim3 gSjc(mTjc.unit_quaternion(),mTjc.translation(),1.0);
            // g2o::Sim3 gSjw = gSjc * gScw;
            // int numProjMatches_j = 0;
            // vector<MapPoint*> vpMatchedMPs_j;
            // bool bValid = DetectCommonRegionsFromLastKF(pKFj,pMostBoWMatchesKF, gSjw,numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

            // if(bValid)
            // {
            //    Sophus::SE3f Tc_w = mpCurrentKF->GetPose();
            //    Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
            //    Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
            //    Eigen::Vector3f vector_dist = Tc_cj.translation();
            //    nNumKFs++;
            // }
            // j++;
            // }

            // if(nNumKFs < 3)
            // {
            // vnStage[index] = 8;
            // vnMatchesStage[index] = nNumKFs;
            // }

            // if(nBestMatchesReproj < numProjOptMatches)
            // {
            // nBestMatchesReproj = numProjOptMatches;
            // nBestNumCoindicendes = nNumKFs;
            // pBestMatchedKF = pMostBoWMatchesKF;
            // g2oBestScw = gScw;
            // vpBestMapPoints = vpMapPoints;
            // vpBestMatchedMapPoints = vpMatchedMP;
            // }
            // }
            // }
            // }
            // }
            // /*else
            // {
            // Verbose::PrintMess("BoW candidate: it don't match with the current one", Verbose::VERBOSITY_DEBUG);
            // }*/
            // }
            // index++;
            // }

            // if(nBestMatchesReproj > 0)
            // {
            // pLastCurrentKF = mpCurrentKF;
            // nNumCoincidences = nBestNumCoindicendes;
            // pMatchedKF2 = pBestMatchedKF;
            // pMatchedKF2->SetNotErase();
            // g2oScw = g2oBestScw;
            // vpMPs = vpBestMapPoints;
            // vpMatchedMPs = vpBestMatchedMapPoints;

            // return nNumCoincidences >= 3;
            // }
            // else
            // {
            // int maxStage = -1;
            // int maxMatched;
            // for(int i=0; i<vnStage.size(); ++i)
            // {
            // if(vnStage[i] > maxStage)
            // {
            // maxStage = vnStage[i];
            // maxMatched = vnMatchesStage[i];
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