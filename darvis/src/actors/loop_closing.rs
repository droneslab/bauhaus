use std::collections::{HashSet, HashMap, VecDeque};
use std::convert::identity;
use std::iter::FromIterator;
use std::ops::IndexMut;
use std::sync::Arc;
use axiom::prelude::*;

use dvcore::{
    plugin_functions::Function,
    lockwrap::ReadOnlyWrapper,
};

use dvcore::config::{Sensor, SYSTEM_SETTINGS, GLOBAL_PARAMS};

use log::{warn, info, debug};
use nalgebra::{Matrix3, Similarity3, MatrixMN, MatrixN, SMatrix, Matrix4};
use crate::actors::messages::InitialMapMsg;
use crate::dvmap::keyframe::{KeyFrame, FullKeyFrame};

use crate::dvmap::map_actor::{MAP_ACTOR, KeyframeDatabaseWriteMsg, MapWriteMsg};

use crate::{
    dvmap::{mappoint::*, pose::Pose, map::Map},
};

use crate::modules::imu::ImuModule;
use crate::modules::sim3solver::Sim3Solver;
use crate::modules::{orbmatcher, optimizer};
use crate::registered_modules::LOOP_CLOSING;

use super::messages::{KeyFrameIdMsg};

use std::thread;

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
    mvpLoopMapPoints : Vec<i32>,
    mnNumCorrection: u32,
    mnCorrectionGBA: u32,
    loop_matched_mps: Vec<i32>,
    mnLoopNumNotFound : i32,
    mnMergeNumCoincidences: u32,

    loop_matched_kf: Option<KeyFrame<FullKeyFrame>>, 
    mergelastcurrent_kf:Option<KeyFrame<FullKeyFrame>>,
    merge_matched_mps: Vec<i32>,
    mnMergeNumNotFound : u32,
    mvpMergeMPs : Vec<i32>,

    mbRunningGBA: bool,
    mb_stop_gba : bool,
    mn_full_ba_idx: u32,
    mb_fix_scale: bool, // Pranay : Need to check why this is set or not

    mvpCurrentConnectedKFs : Vec<i32>,
    mpThreadGBA: Option<thread::JoinHandle<()>>,
    mnFullBAIdx: u32,
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

            ////Pranay : Why we need this ?? I think they are never used
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
                        self.mvpLoopMapPoints = self.mvpLoopMPs.clone(); // mvpLoopMapPoints = mvpLoopMPs;
                        self.correct_loop(context); // CorrectLoop();
                        self.mnNumCorrection += 1; // seems just a counter for number of loop corrections done.
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
                let bCommonRegion = self.detect_and_refine_sim3_from_last_kf(
                &self.current_kf.as_ref().unwrap(), 
                &self.loop_matched_kf.as_ref().unwrap(),
                &mut gScw,
                &mut numProjMatches,
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
                let bCommonRegion = self.detect_and_refine_sim3_from_last_kf(
                &self.current_kf.as_ref().unwrap(), 
                &self.merge_match_kf.as_ref().unwrap(),
                &mut gScw,
                &mut numProjMatches,
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
                self.mbLoopDetected =  self.detect_common_regions_from_bow(&vpLoopBowCand, self.loop_matched_kf.as_ref().unwrap(), self.looplastcurrent_kf.as_ref().unwrap(), self.mg2oLoopSlw.as_ref().unwrap(), self.mnLoopNumCoincidences as i32, &mut self.mvpLoopMPs, &mut self.loop_matched_mps); //mbLoopDetected = DetectCommonRegionsFromBoW(vpLoopBowCand, mpLoopMatchedKF, mpLoopLastCurrentKF, mg2oLoopSlw, mnLoopNumCoincidences, mvpLoopMPs, mvpLoopMatchedMPs);
            }
            // // Merge candidates
            if !bMergeDetectedInKF && !vpMergeBowCand.is_empty()
            {
                self.merge_detected = self.detect_common_regions_from_bow(&vpMergeBowCand, self.merge_match_kf.as_ref().unwrap(), self.mergelastcurrent_kf.as_ref().unwrap(), self.mg2oMergeSlw.as_ref().unwrap(), self.mnMergeNumCoincidences as i32, &mut self.mvpMergeMPs, &mut self.merge_matched_mps) ;//mbMergeDetected = DetectCommonRegionsFromBoW(vpMergeBowCand, mpMergeMatchedKF, mpMergeLastCurrentKF, mg2oMergeSlw, mnMergeNumCoincidences, mvpMergeMPs, mvpMergeMatchedMPs);
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

        //void LoopClosing::MergeLocal()
        fn merge_local(&mut self)
        {
            //todo!("Implement merge_local");
            let numTemporalKFs = 25; //int numTemporalKFs = 25; //Temporal KFs in the local window if the map is inertial.

            //Relationship to rebuild the essential graph, it is used two times, first in the local window and later in the rest of the map
            let mut pNewChild = -1; //KeyFrame* pNewChild;
            let mut pNewParent = -1; //KeyFrame* pNewParent;

            let mut vpLocalCurrentWindowKFs = Vec::new(); //vector<KeyFrame*> vpLocalCurrentWindowKFs;
            let mut vpMergeConnectedKFs = Vec::new(); //vector<KeyFrame*> vpMergeConnectedKFs;

            // Flag that is true only when we stopped a running BA, in this case we need relaunch at the end of the merge
            let mut bRelaunchBA = false; //bool bRelaunchBA = false;

            //Verbose::PrintMess("MERGE-VISUAL: Check Full Bundle Adjustment", Verbose::VERBOSITY_DEBUG);
            // If a Global Bundle Adjustment is running, abort it
            if self.is_running_gba() //(isRunningGBA())
            {
                //unique_lock<mutex> lock(mMutexGBA);

                // Pranay: current gba call is not using this variable, need to 
                self.mb_stop_gba; //mbStopGBA = true;

                self.mnFullBAIdx+=1 ; //mnFullBAIdx++;

                if self.mpThreadGBA.is_some() //(mpThreadGBA)
                {
                    todo!("mpThreadGBA: figure out how to stop the thread");
                    //mpThreadGBA->detach();
                    //delete mpThreadGBA;
                }
                bRelaunchBA = true;
            }

            //Verbose::PrintMess("MERGE-VISUAL: Request Stop Local Mapping", Verbose::VERBOSITY_DEBUG);
            //cout << "Request Stop Local Mapping" << endl;

            // Pranay: [TODO] figure out if we need local mapper to be stopped
            // mpLocalMapper->RequestStop();
            // // Wait until Local Mapping has effectively stopped
            // while(!mpLocalMapper->isStopped())
            // {
            //     usleep(1000);
            // }
            // //cout << "Local Map stopped" << endl;

            // Pranay: [TODO] figure out if we need local mapper to empty the queue
            // mpLocalMapper->EmptyQueue();

            // Merge map will become in the new active map with the local window of KFs and MPs from the current map.
            // Later, the elements of the current map will be transform to the new active map reference, in order to keep real time tracking
            //Map* pCurrentMap = mpCurrentKF->GetMap();
            //Map* pMergeMap = mpMergeMatchedKF->GetMap();

            //std::cout << "Merge local, Active map: " << pCurrentMap->GetId() << std::endl;
            //std::cout << "Merge local, Non-Active map: " << pMergeMap->GetId() << std::endl;


            // [TODO] Pranay : NEED TO CHECK THIS if keyframe is needed to be updated, as pre below commented code
            // // Ensure current keyframe is updated
            // mpCurrentKF->UpdateConnections();

            todo!("Continue merge_local function from here");
            // //Get the current KF and its neighbors(visual->covisibles; inertial->temporal+covisibles)
            // set<KeyFrame*> spLocalWindowKFs;
            // //Get MPs in the welding area from the current map
            // set<MapPoint*> spLocalWindowMPs;
            // if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
            // {
            //     KeyFrame* pKFi = mpCurrentKF;
            //     int nInserted = 0;
            //     while(pKFi && nInserted < numTemporalKFs)
            //     {
            //         spLocalWindowKFs.insert(pKFi);
            //         pKFi = mpCurrentKF->mPrevKF;
            //         nInserted++;

            //         set<MapPoint*> spMPi = pKFi->GetMapPoints();
            //         spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());
            //     }

            //     pKFi = mpCurrentKF->mNextKF;
            //     while(pKFi)
            //     {
            //         spLocalWindowKFs.insert(pKFi);

            //         set<MapPoint*> spMPi = pKFi->GetMapPoints();
            //         spLocalWindowMPs.insert(spMPi.begin(), spMPi.end());

            //         pKFi = mpCurrentKF->mNextKF;
            //     }
            // }
            // else
            // {
            //     spLocalWindowKFs.insert(mpCurrentKF);
            // }

            // vector<KeyFrame*> vpCovisibleKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
            // spLocalWindowKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
            // spLocalWindowKFs.insert(mpCurrentKF);
            // const int nMaxTries = 5;
            // int nNumTries = 0;
            // while(spLocalWindowKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
            // {
            //     vector<KeyFrame*> vpNewCovKFs;
            //     vpNewCovKFs.empty();
            //     for(KeyFrame* pKFi : spLocalWindowKFs)
            //     {
            //         vector<KeyFrame*> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            //         for(KeyFrame* pKFcov : vpKFiCov)
            //         {
            //             if(pKFcov && !pKFcov->isBad() && spLocalWindowKFs.find(pKFcov) == spLocalWindowKFs.end())
            //             {
            //                 vpNewCovKFs.push_back(pKFcov);
            //             }

            //         }
            //     }

            //     spLocalWindowKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
            //     nNumTries++;
            // }

            // for(KeyFrame* pKFi : spLocalWindowKFs)
            // {
            //     if(!pKFi || pKFi->isBad())
            //         continue;

            //     set<MapPoint*> spMPs = pKFi->GetMapPoints();
            //     spLocalWindowMPs.insert(spMPs.begin(), spMPs.end());
            // }

            // //std::cout << "[Merge]: Ma = " << to_string(pCurrentMap->GetId()) << "; #KFs = " << to_string(spLocalWindowKFs.size()) << "; #MPs = " << to_string(spLocalWindowMPs.size()) << std::endl;

            // set<KeyFrame*> spMergeConnectedKFs;
            // if(pCurrentMap->IsInertial() && pMergeMap->IsInertial()) //TODO Check the correct initialization
            // {
            //     KeyFrame* pKFi = mpMergeMatchedKF;
            //     int nInserted = 0;
            //     while(pKFi && nInserted < numTemporalKFs/2)
            //     {
            //         spMergeConnectedKFs.insert(pKFi);
            //         pKFi = mpCurrentKF->mPrevKF;
            //         nInserted++;
            //     }

            //     pKFi = mpMergeMatchedKF->mNextKF;
            //     while(pKFi && nInserted < numTemporalKFs)
            //     {
            //         spMergeConnectedKFs.insert(pKFi);
            //         pKFi = mpCurrentKF->mNextKF;
            //     }
            // }
            // else
            // {
            //     spMergeConnectedKFs.insert(mpMergeMatchedKF);
            // }
            // vpCovisibleKFs = mpMergeMatchedKF->GetBestCovisibilityKeyFrames(numTemporalKFs);
            // spMergeConnectedKFs.insert(vpCovisibleKFs.begin(), vpCovisibleKFs.end());
            // spMergeConnectedKFs.insert(mpMergeMatchedKF);
            // nNumTries = 0;
            // while(spMergeConnectedKFs.size() < numTemporalKFs && nNumTries < nMaxTries)
            // {
            //     vector<KeyFrame*> vpNewCovKFs;
            //     for(KeyFrame* pKFi : spMergeConnectedKFs)
            //     {
            //         vector<KeyFrame*> vpKFiCov = pKFi->GetBestCovisibilityKeyFrames(numTemporalKFs/2);
            //         for(KeyFrame* pKFcov : vpKFiCov)
            //         {
            //             if(pKFcov && !pKFcov->isBad() && spMergeConnectedKFs.find(pKFcov) == spMergeConnectedKFs.end())
            //             {
            //                 vpNewCovKFs.push_back(pKFcov);
            //             }

            //         }
            //     }

            //     spMergeConnectedKFs.insert(vpNewCovKFs.begin(), vpNewCovKFs.end());
            //     nNumTries++;
            // }

            // set<MapPoint*> spMapPointMerge;
            // for(KeyFrame* pKFi : spMergeConnectedKFs)
            // {
            //     set<MapPoint*> vpMPs = pKFi->GetMapPoints();
            //     spMapPointMerge.insert(vpMPs.begin(),vpMPs.end());
            // }

            // vector<MapPoint*> vpCheckFuseMapPoint;
            // vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
            // std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

            // //std::cout << "[Merge]: Mm = " << to_string(pMergeMap->GetId()) << "; #KFs = " << to_string(spMergeConnectedKFs.size()) << "; #MPs = " << to_string(spMapPointMerge.size()) << std::endl;


            // //
            // Sophus::SE3d Twc = mpCurrentKF->GetPoseInverse().cast<double>();
            // g2o::Sim3 g2oNonCorrectedSwc(Twc.unit_quaternion(),Twc.translation(),1.0);
            // g2o::Sim3 g2oNonCorrectedScw = g2oNonCorrectedSwc.inverse();
            // g2o::Sim3 g2oCorrectedScw = mg2oMergeScw; //TODO Check the transformation

            // KeyFrameAndPose vCorrectedSim3, vNonCorrectedSim3;
            // vCorrectedSim3[mpCurrentKF]=g2oCorrectedScw;
            // vNonCorrectedSim3[mpCurrentKF]=g2oNonCorrectedScw;

            // for(KeyFrame* pKFi : spLocalWindowKFs)
            // {
            //     if(!pKFi || pKFi->isBad())
            //     {
            //         Verbose::PrintMess("Bad KF in correction", Verbose::VERBOSITY_DEBUG);
            //         continue;
            //     }

            //     if(pKFi->GetMap() != pCurrentMap)
            //         Verbose::PrintMess("Other map KF, this should't happen", Verbose::VERBOSITY_DEBUG);

            //     g2o::Sim3 g2oCorrectedSiw;

            //     if(pKFi!=mpCurrentKF)
            //     {
            //         Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
            //         g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
            //         //Pose without correction
            //         vNonCorrectedSim3[pKFi]=g2oSiw;

            //         Sophus::SE3d Tic = Tiw*Twc;
            //         g2o::Sim3 g2oSic(Tic.unit_quaternion(),Tic.translation(),1.0);
            //         g2oCorrectedSiw = g2oSic*mg2oMergeScw;
            //         vCorrectedSim3[pKFi]=g2oCorrectedSiw;
            //     }
            //     else
            //     {
            //         g2oCorrectedSiw = g2oCorrectedScw;
            //     }
            //     pKFi->mTcwMerge  = pKFi->GetPose();

            //     // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            //     double s = g2oCorrectedSiw.scale();
            //     pKFi->mfScale = s;
            //     Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(), g2oCorrectedSiw.translation() / s);

            //     pKFi->mTcwMerge = correctedTiw.cast<float>();

            //     if(pCurrentMap->isImuInitialized())
            //     {
            //         Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
            //         pKFi->mVwbMerge = Rcor * pKFi->GetVelocity();
            //     }

            //     //TODO DEBUG to know which are the KFs that had been moved to the other map
            // }

            // int numPointsWithCorrection = 0;

            // //for(MapPoint* pMPi : spLocalWindowMPs)
            // set<MapPoint*>::iterator itMP = spLocalWindowMPs.begin();
            // while(itMP != spLocalWindowMPs.end())
            // {
            //     MapPoint* pMPi = *itMP;
            //     if(!pMPi || pMPi->isBad())
            //     {
            //         itMP = spLocalWindowMPs.erase(itMP);
            //         continue;
            //     }

            //     KeyFrame* pKFref = pMPi->GetReferenceKeyFrame();
            //     if(vCorrectedSim3.find(pKFref) == vCorrectedSim3.end())
            //     {
            //         itMP = spLocalWindowMPs.erase(itMP);
            //         numPointsWithCorrection++;
            //         continue;
            //     }
            //     g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
            //     g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

            //     // Project with non-corrected pose and project back with corrected pose
            //     Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
            //     Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
            //     Eigen::Quaterniond Rcor = g2oCorrectedSwi.rotation() * g2oNonCorrectedSiw.rotation();

            //     pMPi->mPosMerge = eigCorrectedP3Dw.cast<float>();
            //     pMPi->mNormalVectorMerge = Rcor.cast<float>() * pMPi->GetNormal();

            //     itMP++;
            // }
            // /*if(numPointsWithCorrection>0)
            // {
            //     std::cout << "[Merge]: " << std::to_string(numPointsWithCorrection) << " points removed from Ma due to its reference KF is not in welding area" << std::endl;
            //     std::cout << "[Merge]: Ma has " << std::to_string(spLocalWindowMPs.size()) << " points" << std::endl;
            // }*/

            // {
            //     unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            //     unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            //     //std::cout << "Merge local window: " << spLocalWindowKFs.size() << std::endl;
            //     //std::cout << "[Merge]: init merging maps " << std::endl;
            //     for(KeyFrame* pKFi : spLocalWindowKFs)
            //     {
            //         if(!pKFi || pKFi->isBad())
            //         {
            //             //std::cout << "Bad KF in correction" << std::endl;
            //             continue;
            //         }

            //         //std::cout << "KF id: " << pKFi->mnId << std::endl;

            //         pKFi->mTcwBefMerge = pKFi->GetPose();
            //         pKFi->mTwcBefMerge = pKFi->GetPoseInverse();
            //         pKFi->SetPose(pKFi->mTcwMerge);

            //         // Make sure connections are updated
            //         pKFi->UpdateMap(pMergeMap);
            //         pKFi->mnMergeCorrectedForKF = mpCurrentKF->mnId;
            //         pMergeMap->AddKeyFrame(pKFi);
            //         pCurrentMap->EraseKeyFrame(pKFi);

            //         if(pCurrentMap->isImuInitialized())
            //         {
            //             pKFi->SetVelocity(pKFi->mVwbMerge);
            //         }
            //     }

            //     for(MapPoint* pMPi : spLocalWindowMPs)
            //     {
            //         if(!pMPi || pMPi->isBad())
            //             continue;

            //         pMPi->SetWorldPos(pMPi->mPosMerge);
            //         pMPi->SetNormalVector(pMPi->mNormalVectorMerge);
            //         pMPi->UpdateMap(pMergeMap);
            //         pMergeMap->AddMapPoint(pMPi);
            //         pCurrentMap->EraseMapPoint(pMPi);
            //     }

            //     mpAtlas->ChangeMap(pMergeMap);
            //     mpAtlas->SetMapBad(pCurrentMap);
            //     pMergeMap->IncreaseChangeIndex();
            //     //TODO for debug
            //     pMergeMap->ChangeId(pCurrentMap->GetId());

            //     //std::cout << "[Merge]: merging maps finished" << std::endl;
            // }

            // //Rebuild the essential graph in the local window
            // pCurrentMap->GetOriginKF()->SetFirstConnection(false);
            // pNewChild = mpCurrentKF->GetParent(); // Old parent, it will be the new child of this KF
            // pNewParent = mpCurrentKF; // Old child, now it will be the parent of its own parent(we need eliminate this KF from children list in its old parent)
            // mpCurrentKF->ChangeParent(mpMergeMatchedKF);
            // while(pNewChild)
            // {
            //     pNewChild->EraseChild(pNewParent); // We remove the relation between the old parent and the new for avoid loop
            //     KeyFrame * pOldParent = pNewChild->GetParent();

            //     pNewChild->ChangeParent(pNewParent);

            //     pNewParent = pNewChild;
            //     pNewChild = pOldParent;

            // }

            // //Update the connections between the local window
            // mpMergeMatchedKF->UpdateConnections();

            // vpMergeConnectedKFs = mpMergeMatchedKF->GetVectorCovisibleKeyFrames();
            // vpMergeConnectedKFs.push_back(mpMergeMatchedKF);
            // //vpCheckFuseMapPoint.reserve(spMapPointMerge.size());
            // //std::copy(spMapPointMerge.begin(), spMapPointMerge.end(), std::back_inserter(vpCheckFuseMapPoint));

            // // Project MapPoints observed in the neighborhood of the merge keyframe
            // // into the current keyframe and neighbors using corrected poses.
            // // Fuse duplications.
            // //std::cout << "[Merge]: start fuse points" << std::endl;
            // SearchAndFuse(vCorrectedSim3, vpCheckFuseMapPoint);
            // //std::cout << "[Merge]: fuse points finished" << std::endl;

            // // Update connectivity
            // for(KeyFrame* pKFi : spLocalWindowKFs)
            // {
            //     if(!pKFi || pKFi->isBad())
            //         continue;

            //     pKFi->UpdateConnections();
            // }
            // for(KeyFrame* pKFi : spMergeConnectedKFs)
            // {
            //     if(!pKFi || pKFi->isBad())
            //         continue;

            //     pKFi->UpdateConnections();
            // }

            // //std::cout << "[Merge]: Start welding bundle adjustment" << std::endl;

            // bool bStop = false;
            // vpLocalCurrentWindowKFs.clear();
            // vpMergeConnectedKFs.clear();
            // std::copy(spLocalWindowKFs.begin(), spLocalWindowKFs.end(), std::back_inserter(vpLocalCurrentWindowKFs));
            // std::copy(spMergeConnectedKFs.begin(), spMergeConnectedKFs.end(), std::back_inserter(vpMergeConnectedKFs));
            // if (mpTracker->mSensor==System::IMU_MONOCULAR || mpTracker->mSensor==System::IMU_STEREO || mpTracker->mSensor==System::IMU_RGBD)
            // {
            //     Optimizer::MergeInertialBA(mpCurrentKF,mpMergeMatchedKF,&bStop, pCurrentMap,vCorrectedSim3);
            // }
            // else
            // {
            //     Optimizer::LocalBundleAdjustment(mpCurrentKF, vpLocalCurrentWindowKFs, vpMergeConnectedKFs,&bStop);
            // }

            // //std::cout << "[Merge]: Welding bundle adjustment finished" << std::endl;

            // // Loop closed. Release Local Mapping.
            // mpLocalMapper->Release();

            // //Update the non critical area from the current map to the merged map
            // vector<KeyFrame*> vpCurrentMapKFs = pCurrentMap->GetAllKeyFrames();
            // vector<MapPoint*> vpCurrentMapMPs = pCurrentMap->GetAllMapPoints();

            // if(vpCurrentMapKFs.size() == 0){}
            // else {
            //     if(mpTracker->mSensor == System::MONOCULAR)
            //     {
            //         unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information

            //         for(KeyFrame* pKFi : vpCurrentMapKFs)
            //         {
            //             if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
            //             {
            //                 continue;
            //             }

            //             g2o::Sim3 g2oCorrectedSiw;

            //             Sophus::SE3d Tiw = (pKFi->GetPose()).cast<double>();
            //             g2o::Sim3 g2oSiw(Tiw.unit_quaternion(),Tiw.translation(),1.0);
            //             //Pose without correction
            //             vNonCorrectedSim3[pKFi]=g2oSiw;

            //             Sophus::SE3d Tic = Tiw*Twc;
            //             g2o::Sim3 g2oSim(Tic.unit_quaternion(),Tic.translation(),1.0);
            //             g2oCorrectedSiw = g2oSim*mg2oMergeScw;
            //             vCorrectedSim3[pKFi]=g2oCorrectedSiw;

            //             // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            //             double s = g2oCorrectedSiw.scale();

            //             pKFi->mfScale = s;

            //             Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / s);

            //             pKFi->mTcwBefMerge = pKFi->GetPose();
            //             pKFi->mTwcBefMerge = pKFi->GetPoseInverse();

            //             pKFi->SetPose(correctedTiw.cast<float>());

            //             if(pCurrentMap->isImuInitialized())
            //             {
            //                 Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse() * vNonCorrectedSim3[pKFi].rotation()).cast<float>();
            //                 pKFi->SetVelocity(Rcor * pKFi->GetVelocity()); // TODO: should add here scale s
            //             }

            //         }
            //         for(MapPoint* pMPi : vpCurrentMapMPs)
            //         {
            //             if(!pMPi || pMPi->isBad()|| pMPi->GetMap() != pCurrentMap)
            //                 continue;

            //             KeyFrame* pKFref = pMPi->GetReferenceKeyFrame();
            //             g2o::Sim3 g2oCorrectedSwi = vCorrectedSim3[pKFref].inverse();
            //             g2o::Sim3 g2oNonCorrectedSiw = vNonCorrectedSim3[pKFref];

            //             // Project with non-corrected pose and project back with corrected pose
            //             Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
            //             Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oNonCorrectedSiw.map(P3Dw));
            //             pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());

            //             pMPi->UpdateNormalAndDepth();
            //         }
            //     }

            //     mpLocalMapper->RequestStop();
            //     // Wait until Local Mapping has effectively stopped
            //     while(!mpLocalMapper->isStopped())
            //     {
            //         usleep(1000);
            //     }

            //     // Optimize graph (and update the loop position for each element form the begining to the end)
            //     if(mpTracker->mSensor != System::MONOCULAR)
            //     {
            //         Optimizer::OptimizeEssentialGraph(mpCurrentKF, vpMergeConnectedKFs, vpLocalCurrentWindowKFs, vpCurrentMapKFs, vpCurrentMapMPs);
            //     }


            //     {
            //         // Get Merge Map Mutex
            //         unique_lock<mutex> currentLock(pCurrentMap->mMutexMapUpdate); // We update the current map with the Merge information
            //         unique_lock<mutex> mergeLock(pMergeMap->mMutexMapUpdate); // We remove the Kfs and MPs in the merged area from the old map

            //         //std::cout << "Merge outside KFs: " << vpCurrentMapKFs.size() << std::endl;
            //         for(KeyFrame* pKFi : vpCurrentMapKFs)
            //         {
            //             if(!pKFi || pKFi->isBad() || pKFi->GetMap() != pCurrentMap)
            //             {
            //                 continue;
            //             }
            //             //std::cout << "KF id: " << pKFi->mnId << std::endl;

            //             // Make sure connections are updated
            //             pKFi->UpdateMap(pMergeMap);
            //             pMergeMap->AddKeyFrame(pKFi);
            //             pCurrentMap->EraseKeyFrame(pKFi);
            //         }

            //         for(MapPoint* pMPi : vpCurrentMapMPs)
            //         {
            //             if(!pMPi || pMPi->isBad())
            //                 continue;

            //             pMPi->UpdateMap(pMergeMap);
            //             pMergeMap->AddMapPoint(pMPi);
            //             pCurrentMap->EraseMapPoint(pMPi);
            //         }
            //     }
            // }



            // mpLocalMapper->Release();

            // if(bRelaunchBA && (!pCurrentMap->isImuInitialized() || (pCurrentMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1)))
            // {
            //     // Launch a new thread to perform Global Bundle Adjustment
            //     mbRunningGBA = true;
            //     mbFinishedGBA = false;
            //     mbStopGBA = false;
            //     mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this, pMergeMap, mpCurrentKF->mnId);
            // }

            // mpMergeMatchedKF->AddMergeEdge(mpCurrentKF);
            // mpCurrentKF->AddMergeEdge(mpMergeMatchedKF);

            // pCurrentMap->IncreaseChangeIndex();
            // pMergeMap->IncreaseChangeIndex();

            // mpAtlas->RemoveBadMaps();

        }



        //bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)        
        fn detect_and_refine_sim3_from_last_kf(
            &mut self,
            p_current_kf: &KeyFrame<FullKeyFrame>,
            p_matched_kf: &KeyFrame<FullKeyFrame>,
            g_scw: &mut Similarity3<f64>,
            n_num_proj_matches: &mut i32,
            vp_mps: &mut Vec<i32>,
            vp_matched_mps: &mut Vec<i32>,
        ) -> bool 
        {
            let mut sp_already_matched_mps = std::collections::HashSet::new();
            *n_num_proj_matches = self.find_matches_by_projection(
                p_current_kf,
                p_matched_kf,
                g_scw,
                &mut sp_already_matched_mps,
                vp_mps,
                vp_matched_mps,
            );
        
            let n_proj_matches = 30;
            let n_proj_opt_matches = 50;
            let n_proj_matches_rep = 100;
        
            if *n_num_proj_matches >= n_proj_matches {
                let m_twm = p_matched_kf.pose.inverse(); //.get_pose_inverse().cast::<f64>();
                let g_swm = Similarity3::from_isometry(m_twm.iso(),1.0);
                let g_scm = *g_scw * g_swm;
                let mut m_hessian_7x7 =   SMatrix::<f64,7,7>::zeros(); // Eigen::Matrix::<f64, 7, 7>::zeros();

                let mut b_fixed_scale = self.mb_fix_scale;
                if self.sensor.is_imu() && self.sensor.is_mono() && !self.map.read().imu_ba2
                {
                    b_fixed_scale = false;
                }

                // [TODO]: Pranay: Need to implement Optimizer::optimize_sim3
                todo!("Need to implement Optimizer::optimize_sim3 and uncomment below code");
                let num_opt_matches = 0;
                // let num_opt_matches = optimizer::optimize_sim3(
                //     p_current_kf,
                //     p_matched_kf,
                //     vp_matched_mps,
                //     &mut g_scm,
                //     10,
                //     b_fixed_scale,
                //     &mut m_hessian_7x7,
                //     true,
                // );
            
                if num_opt_matches > n_proj_opt_matches {
                    let mut g_scw_estimation = g_scw.clone();//g2o::Sim3::new(g_scw.rotation(), g_scw.translation(), 1.0);
            
                    let mut vp_matched_mps: Vec<i32> = Vec::new();// Vec::<MapPoint*>::new();
                    vp_matched_mps.resize(p_current_kf.mappoint_matches.len(), -1);
                    *n_num_proj_matches = self.find_matches_by_projection(
                        p_current_kf,
                        p_matched_kf,
                        &mut g_scw_estimation,
                        &mut sp_already_matched_mps,
                        vp_mps,
                        &mut vp_matched_mps,
                    );
                    if *n_num_proj_matches >=  n_proj_matches_rep {
                        *g_scw = g_scw_estimation.clone();
                        return true;
                    }
                }
            }
            
            return false;
            
        }      
        

        //int LoopClosing::FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
        //    set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
        //    vector<MapPoint*> &vpMatchedMapPoints)
        fn find_matches_by_projection(
            &self,
            p_current_kf: &KeyFrame<FullKeyFrame>,
            p_matched_kfw: &KeyFrame<FullKeyFrame>,
            g2o_scw: &Similarity3<f64>,
            sp_matched_mp_in_origin: &mut HashSet<i32>,
            vp_map_points: &mut Vec<i32>,
            vp_matched_map_points: &mut Vec<i32>,
        ) -> i32 {
            let n_num_covisibles = 10;
            let vp_cov_kfm = p_matched_kfw.get_connections(n_num_covisibles); //get_best_covisibility_key_frames(n_num_covisibles);
            let n_initial_cov = vp_cov_kfm.len();
            vp_cov_kfm.push(p_matched_kfw.id());
            let mut sp_check_kfs =  Vec::from_iter(vp_cov_kfm.into_iter());
            let sp_current_covisbles = p_current_kf.get_connections(30); //get_connected_key_frames();

            
            if n_initial_cov < n_num_covisibles as usize {
                for i in 0..n_initial_cov {
                    let vp_kfs = self.map.read().get_keyframe(&vp_cov_kfm[i]).unwrap().get_connections(n_num_covisibles);//'.get_best_covisibility_key_frames(n_num_covisibles);
                    let mut n_inserted = 0;
                    let mut j = 0;
                    while j < vp_kfs.len() && n_inserted < n_num_covisibles {
                        if !sp_check_kfs.contains(&vp_kfs[j]) && !sp_current_covisbles.contains(&vp_kfs[j]) {
                            sp_check_kfs.push(vp_kfs[j]);
                            n_inserted += 1;
                        }
                        j += 1;
                    }
                    vp_cov_kfm.extend_from_slice(&vp_kfs);
                }
            }
            let mut sp_map_points = HashSet::new();
            vp_map_points.clear();
            vp_matched_map_points.clear();
            for p_kfi in vp_cov_kfm {
                for (p_mpij , (_,_)) in self.map.read().get_keyframe(&p_kfi).unwrap().mappoint_matches {
                    // Pranay : might not need following checks for our implementation
                    // if p_mpij.is_none() || p_mpij.unwrap().is_bad() {
                    //     continue;
                    // }
        
                    if !sp_map_points.contains(&(p_mpij as i32)) {
                        sp_map_points.insert(p_mpij as i32);
                        vp_map_points.push(p_mpij as i32);
                    }
                }
            }
        
            // let m_scw = Converter::to_sophus(g2o_scw);
            // let mut matcher = ORBmatcher::new(0.9, true);
            // vp_matched_map_points.resize(p_current_kf.mappoint_matches.len(),false);
            // let num_matches = matcher.search_by_projection(p_current_kf, m_scw, vp_map_points, &mut vp_matched_map_points, 3, 1.5);

            let th = 3;
            let num_matches = orbmatcher::search_by_projection_sim3(
                &mut p_current_kf,
                g2o_scw,
                vp_map_points,
                vp_matched_map_points,
                3,
                1.5
            ).unwrap();


            return num_matches;
                
        }




        //bool LoopClosing::DetectCommonRegionsFromBoW(std::vector<KeyFrame*> &vpBowCand, KeyFrame* &pMatchedKF2, KeyFrame* &pLastCurrentKF, g2o::Sim3 &g2oScw, int &nNumCoincidences, std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        fn detect_common_regions_from_bow(&self, vpBowCand : &Vec<i32>, pMatchedKF2 : &KeyFrame<FullKeyFrame>, pLastCurrentKF : &KeyFrame<FullKeyFrame>,  g2oScw: &Similarity3<f64>, nNumCoincidences : i32, vpMPs: &mut Vec<i32>,  vpMatchedMPs : &mut Vec<i32>) -> bool
        {
            //todo!("Implement detect_common_regions_from_bow");
            // Pranay : Interesting heuristics used here
            let bow_matches = 20;// int nBoWMatches = 20;
            let bow_inlilers = 15;// int nBoWInliers = 15;
            let n_sim3_inliers = 20;// int nSim3Inliers = 20;
            let proj_matches = 50;// int nProjMatches = 50;
            let proj_opt_matches = 80;// int nProjOptMatches = 80;
            
            
            //get connected keyframes
            let sp_connected_keyframes = self.current_kf.unwrap().get_connections(10); // set<KeyFrame*> spConnectedKeyFrames = mpCurrentKF->GetConnectedKeyFrames();
            

            let num_covisibles = 10;// int nNumCovisibles = 10;

            // ORBmatcher matcherBoW(0.9, true);
            // ORBmatcher matcher(0.75, true);

            // // Varibles to select the best numbe
            let mut best_matches_kf = -1;// KeyFrame* pBestMatchedKF;
            let mut best_matches_reproj = 0;// int nBestMatchesReproj = 0;
            let mut best_num_coindicendes = 0;// int nBestNumCoindicendes = 0;
            let mut g2o_best_scw = Similarity3::<f64>::default();// g2o::Sim3 g2oBestScw;
            let mut best_mappoints = Vec::new();// std::vector<MapPoint*> vpBestMapPoints;
            let mut best_matches_mappoints = Vec::new();// std::vector<MapPoint*> vpBestMatchedMapPoints;

            let mun_candidtates = vpBowCand.len();// int numCandidates = vpBowCand.size();
            let mut vn_stage = Vec::new(); // vector<int> vnStage(numCandidates, 0);
            vn_stage.resize(mun_candidtates, 0);
            let mut vn_matches_stage = Vec::new(); // vector<int> vnMatchesStage(numCandidates, 0);
            vn_matches_stage.resize(mun_candidtates, 0);

    
            let mut index =0;// int index = 0;
            //info!("BoW candidates: There are " )// //Verbose::PrintMess("BoW candidates: There are " + to_string(vpBowCand.size()) + " possible candidates ", Verbose::VERBOSITY_DEBUG);

            for kfi in vpBowCand // for(KeyFrame* pKFi : vpBowCand)
            {
            
                // Pranay : not sure if this check is needed
                // if(!pKFi || pKFi->isBad())
                // continue;

                // std::cout << "KF candidate: " << pKFi->mnId << std::endl;
                // Current KF against KF with covisibles version
                let mut vpCovKFi = self.map.read().get_keyframe(&kfi).unwrap().get_connections(num_covisibles); // std::vector<KeyFrame*> vpCovKFi = pKFi->GetBestCovisibilityKeyFrames(nNumCovisibles);
                
                if vpCovKFi.is_empty()
                {
                    info!("Covisible list empty"); //std::cout << "Covisible list empty" << std::endl;
                    vpCovKFi.push(*kfi);
                }
                else
                {
                    vpCovKFi.push(vpCovKFi[0]);
                    vpCovKFi[0] = *kfi;
                }


                let abort_by_near_kf = false;//bool bAbortByNearKF = false;
                for j in 0..vpCovKFi.len() //for(int j=0; j<vpCovKFi.size(); ++j)
                {
                    if sp_connected_keyframes.contains(&vpCovKFi[j])//if(spConnectedKeyFrames.find(vpCovKFi[j]) != spConnectedKeyFrames.end())
                    {
                        abort_by_near_kf = true;
                        break;
                    }
                }
                if(abort_by_near_kf)
                {
                    info!("Check BoW aborted because is close to the matched one "); //std::cout << "Check BoW aborted because is close to the matched one " << std::endl;
                    continue;
                }
                info!("Check BoW continue because is far to the matched one ");//std::cout << "Check BoW continue because is far to the matched one " << std::endl;


                let mut vvp_matches_mps = Vec::new();// std::vector<std::vector<MapPoint*> > vvpMatchedMPs;
                vvp_matches_mps.resize(vpCovKFi.len(), Vec::new());// vvpMatchedMPs.resize(vpCovKFi.size());
                
                let mut sp_matches_mpi = Vec::new();// std::set<MapPoint*> spMatchedMPi;
                let mut num_bow_matches = 0;// int numBoWMatches = 0;

                let most_bow_matches_kf = kfi;// KeyFrame* pMostBoWMatchesKF = pKFi;
                let most_bow_num_matches = 0;// int nMostBoWNumMatches = 0;

                let matched_points = Vec::new();// std::vector<MapPoint*> vpMatchedPoints = std::vector<MapPoint*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                matched_points.resize(self.current_kf.unwrap().mappoint_matches.len(), -1);
                let keyframe_matches_mp = Vec::new(); // std::vector<KeyFrame*> vpKeyFrameMatchedMP = std::vector<KeyFrame*>(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));
                keyframe_matches_mp.resize(self.current_kf.unwrap().mappoint_matches.len(), -1);


                let mut index_most_bow_matches_kf=0;// int nIndexMostBoWMatchesKF=0;
                for j in 0..vpCovKFi.len()//for(int j=0; j<vpCovKFi.size(); ++j)
                {
                    // Pranay : not sure if this check is needed
                    // if(!vpCovKFi[j] || vpCovKFi[j]->isBad())
                    //     continue;
                    
                    let matches_bow = orbmatcher::search_by_bow_kf(self.current_kf.as_ref().unwrap(), self.map.read().get_keyframe(&vpCovKFi[j]).unwrap(), true, 0.9, &self.map).unwrap(); //int num = matcherBoW.SearchByBoW(mpCurrentKF, vpCovKFi[j], vvpMatchedMPs[j]);
                    if matches_bow.len() > most_bow_num_matches
                    {
                        most_bow_num_matches = matches_bow.len(); // nMostBoWNumMatches = num;
                        index_most_bow_matches_kf = j; // nIndexMostBoWMatchesKF = j;

                        vvp_matches_mps[j] = matches_bow.values().collect(); // vvpMatchedMPs[j] = vpMatchedMPs;
                    }
                }

                for j in 0..vpCovKFi.len()// for(int j=0; j<vpCovKFi.size(); ++j)
                {
                    for k in 0..vvp_matches_mps[j].len()//for(int k=0; k < vvpMatchedMPs[j].size(); ++k)
                    {
                        let mpi_j = vvp_matches_mps[j][k];//MapPoint* pMPi_j = vvpMatchedMPs[j][k];

                        // Pranay : not sure if this check is needed
                        // if(!pMPi_j || pMPi_j->isBad())
                        // continue;

                        if !sp_matches_mpi.contains(&mpi_j) // if(spMatchedMPi.find(pMPi_j) == spMatchedMPi.end())
                        {
                            sp_matches_mpi.push(&mpi_j); //spMatchedMPi.insert(pMPi_j);
                            num_bow_matches+=1;//numBoWMatches++;

                            matched_points[k] = *mpi_j; //vpMatchedPoints[k]= pMPi_j;
                            keyframe_matches_mp[k] = vpCovKFi[j];//vpKeyFrameMatchedMP[k] = vpCovKFi[j];
                        }
                    }
                }


                
                if num_bow_matches >= bow_matches //if(numBoWMatches >= nBoWMatches) // TODO pick a good threshold
                {
                    // Geometric validation
                    let b_fixed_scale = self.mb_fix_scale;//bool bFixedScale = mbFixScale;                                
                    if self.sensor.is_imu() && self.sensor.is_mono() && !self.map.read().imu_ba2 //if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                    {
                        b_fixed_scale = false;
                    }


                    //Pranay : [TODO] implement Sim3Solver
                    let mut solver = Sim3Solver::new(self.current_kf.as_ref().unwrap(), self.map.read().get_keyframe(&vpCovKFi[index_most_bow_matches_kf]).unwrap(), &matched_points, b_fixed_scale, &keyframe_matches_mp, self.map); // Sim3Solver solver = Sim3Solver(mpCurrentKF, pMostBoWMatchesKF, vpMatchedPoints, bFixedScale, vpKeyFrameMatchedMP);
                    solver.set_ransac_parameters(0.99, bow_inlilers, 300); // at least 15 inliers // solver.SetRansacParameters(0.99, nBoWInliers, 300); // at least 15 inliers
                    
                
                    let mut b_no_more = false; //bool bNoMore = false;
                    let mut vb_inliers = Vec::new(); //vector<bool> vbInliers;
                    let n_inliers = 0;//int nInliers;
                    let b_converage = false; //bool bConverge = false;
                    let mut mTcm = Matrix4::<f64>::default();//Eigen::Matrix4f mTcm;
                    while !b_converage && !b_no_more
                    {
                        mTcm = solver.iterate(20, b_no_more, vb_inliers, n_inliers, b_converage); //mTcm = solver.iterate(20,bNoMore, vbInliers, nInliers, bConverge);
                        //Verbose::PrintMess("BoW guess: Solver achieve " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
                    }

                    if b_converage
                    {
                        //std::cout << "Check BoW: SolverSim3 converged" << std::endl;

                        //Verbose::PrintMess("BoW guess: Convergende with " + to_string(nInliers) + " geometrical inliers among " + to_string(nBoWInliers) + " BoW matches", Verbose::VERBOSITY_DEBUG);
                        // Match by reprojection
                        vpCovKFi.clear();
                        vpCovKFi = self.map.read().get_keyframe(&most_bow_matches_kf).unwrap().get_connections(num_covisibles);// pMostBoWMatchesKF->GetBestCovisibilityKeyFrames(nNumCovisibles);
                        vpCovKFi.push(self.map.read().get_keyframe(&most_bow_matches_kf).unwrap().id());//(pMostBoWMatchesKF);
                        let mut sp_check_KFs = vpCovKFi.clone();//set<KeyFrame*> spCheckKFs(vpCovKFi.begin(), vpCovKFi.end());

                        //std::cout << "There are " << vpCovKFi.size() <<" near KFs" << std::endl;

                        let mut sp_map_points = HashSet::new();//set<MapPoint*> spMapPoints;
                        let mut vp_map_points = Vec::new(); //vector<MapPoint*> vpMapPoints;
                        let mut vp_keyframes = Vec::new();//vector<KeyFrame*> vpKeyFrames;
                        for p_cov_kfi in vpCovKFi//for(KeyFrame* pCovKFi : vpCovKFi)
                        {
                            for (p_cov_mpij, is_match) in self.map.read().get_keyframe(&p_cov_kfi).unwrap().mappoint_matches.values()//for(MapPoint* pCovMPij : pCovKFi->GetMapPointMatches())
                            {
                                if *is_match //if(!pCovMPij || pCovMPij->isBad())
                                {
                                    continue;
                                }

                                if !sp_map_points.contains(p_cov_mpij)//if(spMapPoints.find(pCovMPij) == spMapPoints.end())
                                {
                                    sp_map_points.insert(p_cov_mpij);// spMapPoints.insert(pCovMPij);
                                    vp_map_points.push(*p_cov_mpij); //vpMapPoints.push_back(pCovMPij);
                                    vp_keyframes.push(p_cov_kfi); //vpKeyFrames.push_back(pCovKFi);
                                }
                            }
                        }

                        //std::cout << "There are " << vpKeyFrames.size() <<" KFs which view all the mappoints" << std::endl;

                        let new_pose = Pose::new(&solver.get_estimated_translation(), &solver.get_estimated_rotation());
                        let gScm = Similarity3::from_isometry(new_pose.iso(), solver.get_estimated_scale()); //g2o::Sim3 gScm(solver.GetEstimatedRotation().cast<double>(),solver.GetEstimatedTranslation().cast<double>(), (double) solver.GetEstimatedScale());
                        let gSmw = Similarity3::from_isometry(self.map.read().get_keyframe(&most_bow_matches_kf).unwrap().pose.iso(),1.0); //g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),pMostBoWMatchesKF->GetTranslation().cast<double>(),1.0);
                        let gScw = gScm* gSmw; //g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                        let mScw = gScw; //Sophus::Sim3f mScw = Converter::toSophus(gScw);

                        let mut vp_matched_mp = Vec::new();//vector<MapPoint*> vpMatchedMP;
                        vp_matched_mp.resize(self.current_kf.unwrap().mappoint_matches.len(), -1);//vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
                        let mut vp_matched_kf = Vec::new();//vector<KeyFrame*> vpMatchedKF;
                        vp_matched_kf.resize(self.current_kf.unwrap().mappoint_matches.len(), -1); //vpMatchedKF.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<KeyFrame*>(NULL));

                        let num_proj_matches = orbmatcher::search_by_projection_sim3_kfs(
                            &mut self.current_kf.unwrap(),
                            &mut mScw,
                            &mut vp_map_points,
                            &mut vp_keyframes,
                            &mut vp_matched_mp,
                            &mut vp_matched_kf,
                            8,
                            1.5
                        ).unwrap(); //int numProjMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpKeyFrames, vpMatchedMP, vpMatchedKF, 8, 1.5);
                        //cout <<"BoW: " << numProjMatches << " matches between " << vpMapPoints.size() << " points with coarse Sim3" << endl;

                        if num_proj_matches >= proj_matches 
                        {
                            // Optimize Sim3 transformation with every matches
                            let mut m_hessian_7x7 =   SMatrix::<f64,7,7>::zeros(); //Eigen::Matrix<double, 7, 7> mHessian7x7;

                            let b_fixed_scale = self.mb_fix_scale;//bool bFixedScale = mbFixScale;                                
                            if self.sensor.is_imu() && self.sensor.is_mono() && !self.map.read().imu_ba2 //if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
                            {
                                b_fixed_scale = false;
                            }
        
                            let num_opt_matches = optimizer::optimize_sim3(
                                &self.current_kf.unwrap(),
                                self.map.read().get_keyframe(&kfi).unwrap(),
                                &mut vp_matched_mp,
                                &mut gScm,
                                10.0,
                                b_fixed_scale,
                                &mut m_hessian_7x7,
                                true
                            ); // int numOptMatches = Optimizer::OptimizeSim3(mpCurrentKF, pKFi, vpMatchedMP, gScm, 10, mbFixScale, mHessian7x7, true);

                            //if(numOptMatches >= nSim3Inliers)
                            if num_opt_matches >= n_sim3_inliers
                            {
                                let gSmw = Similarity3::from_isometry(self.map.read().get_keyframe(&most_bow_matches_kf).unwrap().pose.iso(),1.0);  //g2o::Sim3 gSmw(pMostBoWMatchesKF->GetRotation().cast<double>(),pMostBoWMatchesKF->GetTranslation().cast<double>(),1.0);
                                let gScw = gScm* gSmw;  //g2o::Sim3 gScw = gScm*gSmw; // Similarity matrix of current from the world position
                                let mScw = gScw; //Sophus::Sim3f mScw = Converter::toSophus(gScw);
                                
                                let mut vp_matched_mp = Vec::new();//vector<MapPoint*> vpMatchedMP;
                                vp_matched_mp.resize(self.current_kf.unwrap().mappoint_matches.len(), -1); //vpMatchedMP.resize(mpCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));

                                let num_proj_opt_matches = orbmatcher::search_by_projection_sim3(
                                    &mut self.current_kf.unwrap(),
                                    &mScw,
                                    &mut vp_map_points,
                                    &mut vp_matched_mp,
                                    3,
                                    1.5
                                ).unwrap(); // int numProjOptMatches = matcher.SearchByProjection(mpCurrentKF, mScw, vpMapPoints, vpMatchedMP, 5, 1.0);


                                if num_proj_opt_matches >= proj_opt_matches //(numProjOptMatches >= nProjOptMatches)
                                {
                                    let (mut max_x, mut min_x) : (f32,f32)= (-1.0,1000000.0); //int max_x = -1, min_x = 1000000;
                                    let (mut max_y, mut min_y) : (f32,f32)= (-1.0,1000000.0); //int max_y = -1, min_y = 1000000;
                                    for mpi in vp_matched_mp //for(MapPoint* pMPi : vpMatchedMP)
                                    {
                                        if mpi == -1 // if(!pMPi || pMPi->isBad())
                                        {
                                            continue;
                                        }
                                        let p_mp_i = self.map.read().get_mappoint(&mpi).unwrap(); //MapPoint* pMPi = vpMatchedMP[i];
                                        let indexes = p_mp_i.get_observations().get_observation(&kfi); // tuple<size_t,size_t> indexes = pMPi->GetIndexInKeyFrame(pKFi);
                                        
                                        let index = indexes.0; // int index = get<0>(indexes);
                                        if index >= 0 //if(index >= 0)
                                        {
                                             //
                                            let coord_x = self.map.read().get_keyframe(&kfi).unwrap().features.get_keypoint(index as usize).pt.x; //int coord_x = pKFi->mvKeysUn[index].pt.x;
                                            if coord_x < min_x
                                            {
                                                min_x = coord_x;
                                            }
                                            if coord_x > max_x
                                            {
                                                max_x = coord_x;
                                            }
                                            let coord_y = self.map.read().get_keyframe(&kfi).unwrap().features.get_keypoint(index as usize).pt.y; //int coord_y = pKFi->mvKeysUn[index].pt.y;
                                            if coord_y < min_y
                                            {
                                                min_y = coord_y;
                                            }
                                            if coord_y > max_y
                                            {
                                                max_y = coord_y;
                                            }
                                        }
                                    }

                                    let mut n_num_kfs = 0; //int nNumKFs = 0;
                                    //vpMatchedMPs = vpMatchedMP;
                                    //vpMPs = vpMapPoints;
                                    // Check the Sim3 transformation with the current KeyFrame covisibles
                                    let vp_current_cov_kfs = self.current_kf.unwrap().get_connections(num_covisibles); //vector<KeyFrame*> vpCurrentCovKFs = mpCurrentKF->GetBestCovisibilityKeyFrames(nNumCovisibles);

                                    let mut j = 0;//int j = 0;
                                    while n_num_kfs <3 && j < vp_current_cov_kfs.len() //while(nNumKFs < 3 && j<vpCurrentCovKFs.size())
                                    {
                                        let kfj = self.map.read().get_keyframe(&vp_current_cov_kfs[j]).unwrap();//KeyFrame* pKFj = vpCurrentCovKFs[j];
                                        let mTjc = kfj.pose * self.current_kf.unwrap().pose.inverse(); //Sophus::SE3d mTjc = (pKFj->GetPose() * mpCurrentKF->GetPoseInverse()).cast<double>();
                                        let gSjc = Similarity3::from_isometry(mTjc.iso(), 1.0); //g2o::Sim3 gSjc(mTjc.unit_quaternion(),mTjc.translation(),1.0);
                                        let gSjw = gSjc * gScw;//g2o::Sim3 gSjw = gSjc * gScw;
                                        let mut num_proj_matches_j = 0;//int numProjMatches_j = 0;
                                        let mut vp_matched_mps_j = Vec::new();//vector<MapPoint*> vpMatchedMPs_j;
                                        let b_valid = self.detect_common_regions_from_last_kf(
                                            &kfj,
                                            &self.map.read().get_keyframe(&most_bow_matches_kf).unwrap(),
                                            &gSjw,
                                            &mut num_proj_matches_j,
                                            &mut vp_map_points,
                                            &mut vp_matched_mps_j,
                                        ); //bool bValid = DetectCommonRegionsFromLastKF(pKFj,pMostBoWMatchesKF, gSjw,numProjMatches_j, vpMapPoints, vpMatchedMPs_j);

                                        if b_valid //(bValid)
                                        {
                                            let Tc_w = self.current_kf.unwrap().pose; //Sophus::SE3f Tc_w = mpCurrentKF->GetPose();
                                            let Tw_cj =  kfj.pose.inverse();//Sophus::SE3f Tw_cj = pKFj->GetPoseInverse();
                                            let Tc_cj = Tc_w * Tw_cj; //Sophus::SE3f Tc_cj = Tc_w * Tw_cj;
                                            let vector_dist = Tc_cj.get_translation(); // Eigen::Vector3f vector_dist = Tc_cj.translation();
                                            n_num_kfs+=1; //nNumKFs++;
                                        }
                                        j+=1;
                                    }

                                    if n_num_kfs < 3
                                    {
                                        vn_stage[index]=8; //vnStage[index] = 8;
                                        vn_matches_stage[index] = n_num_kfs; //vnMatchesStage[index] = nNumKFs;
                                    }

                                    if best_matches_reproj < num_proj_opt_matches //(nBestMatchesReproj < numProjOptMatches)
                                    {
                                        best_matches_reproj = num_proj_opt_matches; // nBestMatchesReproj = numProjOptMatches;
                                        best_num_coindicendes = n_num_kfs; // nBestNumCoindicendes = nNumKFs;
                                        best_matches_kf = *most_bow_matches_kf;// pBestMatchedKF = pMostBoWMatchesKF;
                                        g2o_best_scw = gScw; // g2oBestScw = gScw;
                                        best_mappoints = vp_map_points; // vpBestMapPoints = vpMapPoints;
                                        best_matches_mappoints = vp_matched_mp; // vpBestMatchedMapPoints = vpMatchedMP;
                                    }
                                }
                            }
                        }
                    }
                /*else
                {
                Verbose::PrintMess("BoW candidate: it don't match with the current one", Verbose::VERBOSITY_DEBUG);
                }*/
                }
                index+=1;
            }

            if best_matches_reproj > 0 // if nBestMatchesReproj > 0
            {
                pLastCurrentKF = self.current_kf.as_ref().unwrap() ; //mpCurrentKF;
                nNumCoincidences = best_num_coindicendes;//nBestNumCoindicendes;
                pMatchedKF2 = self.map.read().get_keyframe(&best_matches_kf).unwrap() ;//pBestMatchedKF;
                //pMatchedKF2->SetNotErase(); // Pranay: check why this is needed??
                *g2oScw = g2o_best_scw; //  g2oBestScw;
                *vpMPs = best_mappoints;//vpBestMapPoints;
                *vpMatchedMPs = best_matches_mappoints;// vpBestMatchedMapPoints;

                return nNumCoincidences >= 3;
            }
            else
            {
                let max_stage = -1; //int maxStage = -1;
                let max_matched = 0;//int maxMatched;
                for i in 0..vn_stage.len() //for(int i=0; i<vnStage.size(); ++i)
                {
                    if vn_stage[i] > max_stage//if(vnStage[i] > maxStage)
                    {
                        max_stage = vn_stage[i];//maxStage = vnStage[i];
                        max_matched = vn_matches_stage[i];//maxMatched = vnMatchesStage[i];
                    }
                }
            }
            return false;
        }

        //bool LoopClosing::DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
        //    std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        fn detect_common_regions_from_last_kf(&self, 
            p_current_kf: &KeyFrame<FullKeyFrame>,
            p_matched_kf: &KeyFrame<FullKeyFrame>, 
            g_scw: &Similarity3<f64>,
            n_num_proj_matches: &mut i32,
            vp_mps: &mut Vec<i32>,
            vp_matched_mps: &mut Vec<i32>
        ) -> bool
        {
            let mut sp_already_matched_mps = std::collections::HashSet::new(); //set<MapPoint*> spAlreadyMatchedMPs(vpMatchedMPs.begin(), vpMatchedMPs.end());
            
            *n_num_proj_matches = self.find_matches_by_projection(
                p_current_kf,
                p_matched_kf,
                g_scw,
                &mut sp_already_matched_mps,
                vp_mps,
                vp_matched_mps,
            ); // nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);

            let n_proj_matches = 30; //int nProjMatches = 30;
            if *n_num_proj_matches >= n_proj_matches //(nNumProjMatches >= nProjMatches)
            {
                return true;
            }

            return false;
        }


        fn is_running_gba(&self) -> bool
        {
            self.mbRunningGBA
        }

        /* 
        fn correct_loop(&mut self) {
            // Send a stop signal to Local Mapping
            // Avoid new keyframes are inserted while correcting the loop

            //Pranay : (TODO) send local mapping msg to stop and empty queue. 
            //self.local_mapper.request_stop();
            //self.local_mapper.empty_queue(); // Process keyframes in the queue
        
            // If a Global Bundle Adjustment is running, abort it
            if self.is_running_gba() {
                debug!("Stopping Global Bundle Adjustment...");
                //let mut lock = self.mutex_gba.lock().unwrap();
                self.mb_stop_gba = true;
                self.mn_full_ba_idx += 1;

                todo!("Implement running GBA as thread");
                // if let Some(ref thread_gba) = self.mp_thread_gba {
                //     thread_gba.detach();
                //     delete(thread_gba);
                // }

                debug!("  Done!!");
            }
        
            //Pranay : (TODO) wait flag for local mapping stopped after stop msg
            // Wait until Local Mapping has effectively stopped
            // while !self.local_mapper.is_stopped() {
            //     thread::sleep(Duration::from_millis(1000));
            // }
        
            //Pranay : [Please check] if following line is needed, if yes need to send an actor msg instead
            // Ensure current keyframe is updated
            //self.current_kf.unwrap().update_connections();
        
            // Retrieve keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
            self.mvp_current_connected_kfs = self.current_kf.unwrap().get_connections(30);
            self.mvp_current_connected_kfs.push(self.current_kf);
        
            let mut corrected_sim3 = HashMap::new();
            let mut non_corrected_sim3 = HashMap::new();
            corrected_sim3.insert(self.current_kf, self.mg2o_loop_scw);
            let twc = self.current_kf.get_pose_inverse();
            let tcw = self.current_kf.get_pose();
            let g2o_scw = g2o::Sim3::new(
                tcw.unit_quaternion().cast::<f64>(),
                tcw.translation().cast::<f64>(),
                1.0,
            );
            non_corrected_sim3.insert(self.current_kf, g2o_scw);
        
            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            let corrected_tcw = se3d::SE3::new(
                self.mg2o_loop_scw.rotation(),
                self.mg2o_loop_scw.translation() / self.mg2o_loop_scw.scale(),
            );
            self.current_kf.set_pose(corrected_tcw.cast::<f32>());
        
            let loop_map = self.current_kf.get_map();
        
            let mut lock = loop_map.mutex_map_update.lock().unwrap();
        
            let b_imu_init = loop_map.is_imu_initialized();
        
            for kf in self.mvp_current_connected_kfs.iter() {
                if kf != self.current_kf {
                    let tiw = kf.get
                    let tic = (tiw * twc).cast::<f64>();
                    let g2o_sic = g2o::Sim3::new(
                        tic.unit_quaternion(),
                        tic.translation(),
                        1.0,
                    );
                    non_corrected_sim3.insert(kf, g2o_sic);
                }
            }
        
            // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
            for kf in self.mvp_current_connected_kfs.iter() {
                if kf != self.current_kf {
                    for mb in kf.map_points.iter() {
                        if let Some(mp) = mb {
                            if !mp.is_corrected() {
                                mp.correct_loop(self.current_kf, non_corrected_sim3, corrected_sim3, b_imu_init);
                            }
                        }
                    }
                }
            }
        
            // Now loop_map.mvpKeyFrameOrigins has the previous loop keyframes, we must set to nullptr the pointers
            // in loop_map.mvpKeyFrameOrigins of keyframes not in the current connected KFs
            for kf in loop_map.key_frames.iter() {
                if !self.current_kf.is_neighbor(kf) {
                    loop_map.key_frame_origins.remove(kf);
                }
            }
        
            // Project MapPoints observed in the neighborhood of the loop keyframe
            // into the current keyframe and neighbors using corrected poses.
            // Fuse duplications.
            for kf in self.mvp_current_connected_kfs.iter() {
                if kf != self.current_kf {
                    for mb in kf.map_points.iter() {
                        if let Some(mp) = mb {
                            if !mp.is_bad() {
                                mp.lock_mut().project(corrected_sim3, Some(&loop_map));
                            }
                        }
                    }
                }
            }

            self.current_kf.lock_mut().project_map_points(Some(&loop_map), 3, Some(&self.loop_map_matches));

            // Attention! Extra loop correction
            if self.b_loop_correction {
                for kf in self.mvp_current_connected_kfs.iter() {
                    if kf != self.current_kf {
                        for mb in kf.map_points.iter() {
                            if let Some(mp) = mb {
                                if !mp.is_bad() {
                                    mp.lock_mut().project(corrected_sim3, Some(&loop_map));
                                }
                            }
                        }
                    }
                }
            }


            loop_map.invalidate();

            // Loop closed. Release Local Mapping.            

            //cout << "Loop closed!" << endl;
            self.mp_local_mapper.lock().unwrap().release();

            // Stop loop if error too large
            if self.b_loop_correction && self.b_stop_loop {
                self.b_loop = false;
                
                //break;
            }

            self.current_kf = kf_loop_closer;
        }
        */      

        //void LoopClosing::CorrectLoop()
        fn correct_loop(&mut self, context: Context)
        {
            //cout << "Loop detected!" << endl;


            // Pranay : NEED TO CHECK THIS if local mapping need to be stopped
            // // Send a stop signal to Local Mapping
            // // Avoid new keyframes are inserted while correcting the loop
            // mpLocalMapper->RequestStop();
            // mpLocalMapper->EmptyQueue(); // Proccess keyframes in the queue

            
            // If a Global Bundle Adjustment is running, abort it
            if self.is_running_gba() //(isRunningGBA())
            {
                info!("Stoping Global Bundle Adjustment...");//cout << "Stoping Global Bundle Adjustment...";

                todo!("Stop Global Bundle Adjustment");
                // unique_lock<mutex> lock(mMutexGBA);
                // mbStopGBA = true;

                // mnFullBAIdx++;

                // if(mpThreadGBA)
                // {
                //     mpThreadGBA->detach();
                //     delete mpThreadGBA;
                // }

                info!("Done!!");//cout << "  Done!!" << endl;
            }


            // Pranay: [TODO] Need to impelement how to check local mapper running status
            // // Wait until Local Mapping has effectively stopped
            // while(!mpLocalMapper->isStopped())
            // {
            //     usleep(1000);
            // }


            // [TODO] Pranay : NEED TO CHECK THIS if keyframe is needed to be updated, as pre below commented code
            // // Ensure current keyframe is updated
            // mpCurrentKF->UpdateConnections();


            // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
            self.mvpCurrentConnectedKFs = self.current_kf.unwrap().get_connections(30);//mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
            self.mvpCurrentConnectedKFs.push(self.current_kf.unwrap().id()); //mvpCurrentConnectedKFs.push_back(mpCurrentKF);

            //std::cout << "Loop: number of connected KFs -> " + to_string(mvpCurrentConnectedKFs.size()) << std::endl;

            let (mut CorrectedSim3, mut NonCorrectedSim3): (HashMap<i32, Similarity3<f64>>, HashMap<i32, Similarity3<f64>>) = (HashMap::new(), HashMap::new());//KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
            CorrectedSim3[mpCurrentKF]=mg2oLoopScw; // CorrectedSim3[mpCurrentKF]=mg2oLoopScw;
            let Twc = self.current_kf.unwrap().pose.inverse(); //Sophus::SE3f Twc = mpCurrentKF->GetPoseInverse();
            let Tcw = self.current_kf.unwrap().pose.clone(); //Sophus::SE3f Tcw = mpCurrentKF->GetPose();
            let g2oScw = Similarity3::from_isometry(Tcw.iso(), 1.0); //g2o::Sim3 g2oScw(Tcw.unit_quaternion().cast<double>(),Tcw.translation().cast<double>(),1.0);
            NonCorrectedSim3[mpCurrentKF]=g2oScw; // NonCorrectedSim3[mpCurrentKF]=g2oScw;

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            let correctedTcw =  Pose::new(&self.mg2oLoopScw.unwrap().isometry.translation.vector, &self.mg2oLoopScw.unwrap().isometry.rotation.to_rotation_matrix().matrix() / self.mg2oLoopScw.unwrap().scaling());//Sophus::SE3d correctedTcw(mg2oLoopScw.rotation(),mg2oLoopScw.translation() / mg2oLoopScw.scale());
            

            let map_actor = Some(context.system.find_aid_by_name(MAP_ACTOR).unwrap());
            // Insert Keyframe in Database
            let map_actor = map_actor.as_ref().unwrap();
            map_actor.send_new(MapWriteMsg::update_keyframe_pose(&self.current_kf.unwrap().id(), &correctedTcw)).unwrap();
            self.current_kf.unwrap().pose = correctedTcw.clone(); //mpCurrentKF->SetPose(correctedTcw.cast<float>());

            //Map* pLoopMap = mpCurrentKF->GetMap();


            {
                // Get Map Mutex
                //unique_lock<mutex> lock(pLoopMap->mMutexMapUpdate);

                let bImuInit = self.map.read().imu_initialized; //const bool bImuInit = pLoopMap->isImuInitialized();

                for kfi in self.mvpCurrentConnectedKFs //for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
                {
                    //KeyFrame* pKFi = *vit;

                    if kfi != self.current_kf.unwrap().id() //(pKFi!=mpCurrentKF)
                    {
                        let Tiw = self.map.read().get_keyframe(&kfi).unwrap().pose.clone(); //Sophus::SE3f Tiw = pKFi->GetPose();
                        let Tic = Tiw * Twc; //Sophus::SE3d Tic = (Tiw * Twc).cast<double>();
                        let g2oSic = Similarity3::from_isometry(Tic.iso(), 1.0); //g2o::Sim3 g2oSic(Tic.unit_quaternion(),Tic.translation(),1.0);
                        let g2oCorrectedSiw = g2oSic*mg2oLoopScw; //g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oLoopScw;
                        //Pose corrected with the Sim3 of the loop closure
                        CorrectedSim3[pKFi]=g2oCorrectedSiw; // CorrectedSim3[pKFi]=g2oCorrectedSiw;

                        // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                        let correctedTiw =  Pose::new(&g2oCorrectedSiw.isometry.translation.vector, &g2oCorrectedSiw.isometry.rotation.to_rotation_matrix().matrix() / g2oCorrectedSiw.scaling());//Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
                        map_actor.send_new(MapWriteMsg::update_keyframe_pose(&kfi, &correctedTiw)).unwrap(); //pKFi->SetPose(correctedTiw.cast<float>());

                        //Pose without correction
                        let g2oSiw = Similarity3::from_isometry(Tiw.iso(), 1.0); //g2o::Sim3 g2oSiw(Tiw.unit_quaternion().cast<double>(),Tiw.translation().cast<double>(),1.0);
                        NonCorrectedSim3[pKFi]=g2oSiw; // NonCorrectedSim3[pKFi]=g2oSiw;
                    }  
                }

                // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
                for (kfi, g2oCorrectedSiw) in CorrectedSim3 //for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
                {
                    //KeyFrame* pKFi = mit->first;
                    //g2o::Sim3 g2oCorrectedSiw = mit->second;
                    let g2oCorrectedSwi = g2oCorrectedSiw.inverse(); //g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

                    let g2oSiw = NonCorrectedSim3[pKFi]; //g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

                    // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
                    /*Sophus::SE3d correctedTiw(g2oCorrectedSiw.rotation(),g2oCorrectedSiw.translation() / g2oCorrectedSiw.scale());
                    pKFi->SetPose(correctedTiw.cast<float>());*/

                    let vpMPsi = self.map.read().get_keyframe(&kfi).unwrap().mappoint_matches.clone(); //vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
                    for (_, (pMPi, _))  in vpMPsi //for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
                    {
                        // MapPoint* pMPi = vpMPsi[iMP];
                        if mpi ==-1 // (!pMPi)
                        {
                            continue;
                        }
                        // if(pMPi->isBad())
                        //     continue;

                        let mpi_mp = self.map.read().get_mappoint(&mpi).unwrap();
                        if mpi_mp.mnCorrectedByKF == self.current_kf.unwrap().id() //(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                        {
                            continue;
                        }
                        
                        // Project with non-corrected pose and project back with corrected pose
                        let P3Dw = mpi_mp.position.clone(); //Eigen::Vector3d P3Dw = pMPi->GetWorldPos().cast<double>();
                        let eigCorrectedP3Dw = g2oCorrectedSwi.map(&g2oSiw.map(P3Dw)); //Eigen::Vector3d eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(P3Dw));

                        mpi_mp.position = eigCorrectedP3Dw; //pMPi->SetWorldPos(eigCorrectedP3Dw.cast<float>());
                        mpi_mp.mnCorrectedByKF = self.current_kf.unwrap().id(); //pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                        mpi_mp.mnCorrectedReference = self.current_kf.unwrap().id(); //pMPi->mnCorrectedReference = pKFi->mnId;
                        //pMPi->UpdateNormalAndDepth();// Pranay : this function call is handled in update_mappoint map call

                        map_actor.send_new(MapWriteMsg::update_mappoint(&mpi, &mpi_mp)).unwrap();

                    }

                    // Correct velocity according to orientation correction
                    if bImuInit //(bImuInit)
                    {
                        todo!("IMU");
                        //Eigen::Quaternionf Rcor = (g2oCorrectedSiw.rotation().inverse()*g2oSiw.rotation()).cast<float>();
                        //pKFi->SetVelocity(Rcor*pKFi->GetVelocity());
                    }


                    // [TODO] Pranay : NEED TO CHECK THIS if keyframe is needed to be updated, as pre below commented code
                    //// Make sure connections are updated
                    //pKFi->UpdateConnections();


                }

                map_actor.send_new(MapWriteMsg::increase_change_index()).unwrap(); //mpAtlas->GetCurrentMap()->IncreaseChangeIndex();


                // Start Loop Fusion
                // Update matched map points and replace if duplicated
                for i in 0..self.loop_matched_mps.len() //for(size_t i=0; i<mvpLoopMatchedMPs.size(); i++)
                {
                    if self.loop_matched_mps[i] !=-1 //if(mvpLoopMatchedMPs[i])
                    {
                        let loop_mpi = self.loop_matched_mps[i]; //MapPoint* pLoopMP = mvpLoopMatchedMPs[i];
                        let curr_mpi = self.current_kf.unwrap().get_mappoint(i); //MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);

                        let loop_mp = self.map.read().get_mappoint(&loop_mpi).unwrap();
                        if curr_mpi != -1 // (pCurMP)
                        {
                            map_actor.send_new(MapWriteMsg::update_mappoint(&curr_mpi, &loop_mp)).unwrap(); //pCurMP->Replace(pLoopMP);
                        }
                        else
                        {   
                            //mpCurrentKF->AddMapPoint(pLoopMP,i);
                            //pLoopMP->AddObservation(mpCurrentKF,i);
                            //pLoopMP->ComputeDistinctiveDescriptors();
                            map_actor.send_new(MapWriteMsg::add_mappoint_to_keyframe(&self.current_kf.unwrap().id(), &loop_mp, i)).unwrap(); 
                        }
                    }
                }
                ////cout << "LC: end replacing duplicated" << endl;
            }

            // Project MapPoints observed in the neighborhood of the loop keyframe
            // into the current keyframe and neighbors using corrected poses.
            // Fuse duplications.
            self.search_and_fuse(&CorrectedSim3, &self.mvpLoopMapPoints); //SearchAndFuse(CorrectedSim3, mvpLoopMapPoints);

            // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
            let mut LoopConnections: HashMap<i32, Vec<i32>> = HashMap::new();//map<KeyFrame*, set<KeyFrame*> > LoopConnections;

            for kfi in self.mvpCurrentConnectedKFs //for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
            {
                //KeyFrame* pKFi = *vit;
                let vpPreviousNeighbors = self.map.read().get_keyframe(&kfi).unwrap().get_connections(30); //vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

                // [TODO] Pranay : NEED TO CHECK THIS if keyframe is needed to be updated, as pre below commented code
                //// Update connections. Detect new links.
                //pKFi->UpdateConnections();


                LoopConnections[kfi] = self.map.read().get_keyframe(&kfi).unwrap().get_connections(30); //LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
                for prev in vpPreviousNeighbors //for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
                {                    
                    // let index = LoopConnections.get(&kfi).unwrap().iter().position(|x| *x == prev).unwrap();
                    // LoopConnections.get(&kfi).unwrap().remove(index);
                    LoopConnections.get(&kfi).unwrap().retain(|&x| x != prev);//LoopConnections[pKFi].erase(*vit_prev);
                }
                for prev2 in self.mvpCurrentConnectedKFs //for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
                {
                    LoopConnections.get(&kfi).unwrap().retain(|&x| x != prev2); //LoopConnections[pKFi].erase(*vit2);
                }
            }

            // Optimize graph
            let b_fixed_scale = self.mb_fix_scale;//bool bFixedScale = mbFixScale;                                
            if self.sensor.is_imu() && self.sensor.is_mono() && !self.map.read().imu_ba2 //if(mpTracker->mSensor==System::IMU_MONOCULAR && !mpCurrentKF->GetMap()->GetIniertialBA2())
            {
                b_fixed_scale = false;
            }

            //cout << "Optimize essential graph" << endl;
            if self.sensor.is_imu() //(pLoopMap->IsInertial() && pLoopMap->isImuInitialized())
            {
                todo!("IMU");
                // Optimizer::OptimizeEssentialGraph4DoF(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections);
            }
            else
            {
                //cout << "Loop -> Scale correction: " << mg2oLoopScw.scale() << endl;
                optimizer::optimize_essential_graph(self.map, self.loop_matched_kf.as_ref().unwrap(), self.current_kf.as_ref().unwrap(), NonCorrectedSim3, CorrectedSim3, LoopConnections, b_fixed_scale);//Optimizer::OptimizeEssentialGraph(pLoopMap, mpLoopMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, bFixedScale);
            }

            //[TODO] Pranay : Need to check if this is required
            //mpAtlas->InformNewBigChange();

            // Add loop edge
            let loop_matched_kf_updated = self.loop_matched_kf.unwrap().clone().add_loop_edge(self.current_kf.unwrap().id());//mpLoopMatchedKF->AddLoopEdge(mpCurrentKF);

            let current_kf_updated = self.current_kf.unwrap().clone().add_loop_edge(self.loop_matched_kf.unwrap().id());//mpCurrentKF->AddLoopEdge(mpLoopMatchedKF);

            // Send updated KFs to the map
            map_actor.send_new(MapWriteMsg::update_keyframe(&self.loop_matched_kf.unwrap().id(), self.loop_matched_kf.as_ref().unwrap())).unwrap(); 
            map_actor.send_new(MapWriteMsg::update_keyframe(&self.current_kf.unwrap().id(), self.current_kf.as_ref().unwrap())).unwrap(); 


            // Launch a new thread to perform Global Bundle Adjustment (Only if few keyframes, if not it would take too much time)
            if !self.map.read().imu_initialized || self.map.read().num_keyframes() < 200 //(!pLoopMap->isImuInitialized() || (pLoopMap->KeyFramesInMap()<200 && mpAtlas->CountMaps()==1))
            {
                self.mbRunningGBA = true; //mbRunningGBA = true;
                //mbFinishedGBA = false; //Pranay: this variable never used
                self.mb_stop_gba = false; //mbStopGBA = false;
                self.mnCorrectionGBA = self.mnNumCorrection; //mnCorrectionGBA = mnNumCorrection;

                self.mpThreadGBA = Some(thread::spawn(|| {
                    self.run_global_bundle_adjustment(self.current_kf.unwrap().id()); //LoopClosing::RunGlobalBundleAdjustment(pLoopMap, mpCurrentKF->mnId);
                })); //mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment, this, pLoopMap, mpCurrentKF->mnId);
                
            }

            //Pranay: Decide how to handle following code for mpLocalMapper release call
            // // Loop closed. Release Local Mapping.
            // mpLocalMapper->Release();    

            ////mLastLoopKFid = mpCurrentKF->mnId; //TODO old varible, it is not use in the new algorithm
        }


        //void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint*> &vpMapPoints)
        fn search_and_fuse(&self, corrected_poses_map: &HashMap<i32, Similarity3<f64>>, vp_map_points: &Vec<i32>)
        {
            // ORBmatcher matcher(0.8);

            let mut total_replaces=0;//int total_replaces = 0;

            // //cout << "[FUSE]: Initially there are " << vpMapPoints.size() << " MPs" << endl;
            // //cout << "FUSE: Intially there are " << CorrectedPosesMap.size() << " KFs" << endl;

            for (kfi, Scw) in corrected_poses_map //for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
            {
                let mut num_repalces = 0; //int num_replaces = 0;
                //KeyFrame* pKFi = mit->first;
                //Map* pMap = pKFi->GetMap();

                //g2o::Sim3 g2oScw = mit->second;
                //Sophus::Sim3f Scw = Converter::toSophus(g2oScw);

                let mut vpReplacePoints: Vec<i32> = Vec::new().resize(vp_map_points.len(), -1); //vector<MapPoint*> vpReplacePoints(vpMapPoints.size(),static_cast<MapPoint*>(NULL));

                //Pranay :[TODO] call fuse function from matcher
                todo!("call fuse function from matcher");
                let numFused = 0; //int numFused = matcher.Fuse(pKFi,Scw,vpMapPoints,4,vpReplacePoints);

                // Get Map Mutex
                //unique_lock<mutex> lock(pMap->mMutexMapUpdate);
                
                //const int nLP = vpMapPoints.size();
                for i in 0..vp_map_points.len() //for(int i=0; i<nLP;i++)
                {
                    let rep_mp = vpReplacePoints[i]; //MapPoint* pRep = vpReplacePoints[i];
                    if rep_mp !=-1 //(pRep)
                    {
                        num_replaces += 1;
                        map_actor.send_new(MapWriteMsg::update_mappoint(&rep_mp, &self.map.read().get_mappoint(&vp_map_points[i]).unwrap())).unwrap(); //pRep->Replace(vpMapPoints[i]);

                    }
                }

                total_replaces += num_replaces;
            }
            // //cout << "[FUSE]: " << total_replaces << " MPs had been fused" << endl;
        }



        //void LoopClosing::RunGlobalBundleAdjustment(Map* pActiveMap, unsigned long nLoopKF)
        pub fn run_global_bundle_adjustment(&mut self, loop_kf: i32)
        {  
            info!("Starting Global Bundle Adjustment");//Verbose::PrintMess("Starting Global Bundle Adjustment", Verbose::VERBOSITY_NORMAL);

            let b_imu_init = self.map.read().imu_initialized; //const bool bImuInit = pActiveMap->isImuInitialized();

            if !b_imu_init // (!bImuInit)
            {
                optimizer::global_bundle_adjustment(self.map, loop_kf, 10); // Optimizer::GlobalBundleAdjustemnt(pActiveMap,10,&mbStopGBA,nLoopKF,false);
            }

            else
            {
                //Optimizer::FullInertialBA(pActiveMap,7,false,nLoopKF,&mbStopGBA);
                todo!("FullInertialBA");
            }

            let mut idx = self.mnFullBAIdx; //int idx =  mnFullBAIdx;
            // Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

            // Update all MapPoints and KeyFrames
            // Local Mapping was active during BA, that means that there might be new keyframes
            // not included in the Global BA and they are not consistent with the updated map.
            // We need to propagate the correction through the spanning tree
            {
                //unique_lock<mutex> lock(mMutexGBA);
                if idx != self.mnFullBAIdx //(idx!=mnFullBAIdx)
                {
                    return;
                }

                //Pranay: skipping check for active map, as we are only working on one map
                if !b_imu_init //(!bImuInit && pActiveMap->isImuInitialized())      
                {
                    return;
                }

                if self.mb_stop_gba//(!mbStopGBA)
                {
                    
                    info!("Global Bundle Adjustment finished"); // Verbose::PrintMess("Global Bundle Adjustment finished", Verbose::VERBOSITY_NORMAL);
                    info!("Updating map ..."); // Verbose::PrintMess("Updating map ...", Verbose::VERBOSITY_NORMAL);

                    // Pranay: [TODO] figure out if we need local mapper to be stopped
                    // mpLocalMapper->RequestStop();
                    // // Wait until Local Mapping has effectively stopped

                    // while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
                    // {
                    //     usleep(1000);
                    // }

                    // // Get Map Mutex
                    // //unique_lock<mutex> lock(pActiveMap->mMutexMapUpdate);
                    // // cout << "LC: Update Map Mutex adquired" << endl;

                    // //pActiveMap->PrintEssentialGraph();
                    // // Correct keyframes starting at map first keyframe

                    //Pranay : Hardcoding to 0, as we are only working on one map
                    let mut lpKFtoCheck: VecDeque<i32> = vec![0];//list<KeyFrame*> lpKFtoCheck(pActiveMap->mvpKeyFrameOrigins.begin(),pActiveMap->mvpKeyFrameOrigins.end());

                    while !lpKFtoCheck.is_empty() // (!lpKFtoCheck.empty())
                    {
                        let kf_id = lpKFtoCheck.pop_front().unwrap() ;// KeyFrame* pKF = lpKFtoCheck.front();
                        
                        let mut twc = self.map.read().get_keyframe(&kf_id).unwrap().pose.inverse(); // Sophus::SE3f Twc = pKF->GetPoseInverse();

                        // Pranay: [TODO] Continue this code after merging the rest of the code
                        todo!("Continue this code after merging the rest of the code");
                        //self.map.read().get_keyframe(&kf_id).unwrap().get_children()
                        // .iter().for_each(|child_id| {
                        //     let child = self.map.read().get_keyframe(&child_id).unwrap();

                        //     // Pranay: not sure if we need this check
                        //     // if child.is_bad() {
                        //     //     return;
                        //     // }

                        //     if child.get_ba_global_for_kf() != loop_kf {
                        //         let twc = child.get_pose_inverse();
                        //         let tchildc = child.get_pose() * twc;
                        //         child.set_tcw_gba(tchildc * self.map.read().get_keyframe(&kf_id).unwrap().get_tcw_gba());

                        //         let rcor = child.get_tcw_gba().so3().inverse() * child.get_pose().so3();
                        //         if child.is_velocity_set() {
                        //             child.set_vwb_gba(rcor * child.get_velocity());
                        //         }
                        //         else {
                        //             warn!("Child velocity empty!!");
                        //         }

                        //         child.set_bias_gba(child.get_imu_bias());
                        //         child.set_ba_global_for_kf(loop_kf);
                        //     }

                        //     lpKFtoCheck.push_back(child_id);
                        // });
                        
                        // const set<KeyFrame*> sChilds = pKF->GetChilds();
                        // //cout << "---Updating KF " << pKF->mnId << " with " << sChilds.size() << " childs" << endl;
                        // //cout << " KF mnBAGlobalForKF: " << pKF->mnBAGlobalForKF << endl;
                        // Sophus::SE3f Twc = pKF->GetPoseInverse();
                        // //cout << "Twc: " << Twc << endl;
                        // //cout << "GBA: Correct KeyFrames" << endl;
                        // for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                        // {
                        //     KeyFrame* pChild = *sit;
                        //     if(!pChild || pChild->isBad())
                        //         continue;

                        //     if(pChild->mnBAGlobalForKF!=nLoopKF)
                        //     {
                        //         //cout << "++++New child with flag " << pChild->mnBAGlobalForKF << "; LoopKF: " << nLoopKF << endl;
                        //         //cout << " child id: " << pChild->mnId << endl;
                        //         Sophus::SE3f Tchildc = pChild->GetPose() * Twc;
                        //         //cout << "Child pose: " << Tchildc << endl;
                        //         //cout << "pKF->mTcwGBA: " << pKF->mTcwGBA << endl;
                        //         pChild->mTcwGBA = Tchildc * pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;

                        //         Sophus::SO3f Rcor = pChild->mTcwGBA.so3().inverse() * pChild->GetPose().so3();
                        //         if(pChild->isVelocitySet()){
                        //             pChild->mVwbGBA = Rcor * pChild->GetVelocity();
                        //         }
                        //         else
                        //             Verbose::PrintMess("Child velocity empty!! ", Verbose::VERBOSITY_NORMAL);


                        //         //cout << "Child bias: " << pChild->GetImuBias() << endl;
                        //         pChild->mBiasGBA = pChild->GetImuBias();


                        //         pChild->mnBAGlobalForKF = nLoopKF;

                        //     }
                        //     lpKFtoCheck.push_back(pChild);
                        // }

                        // //cout << "-------Update pose" << endl;
                        // pKF->mTcwBefGBA = pKF->GetPose();
                        // //cout << "pKF->mTcwBefGBA: " << pKF->mTcwBefGBA << endl;
                        // pKF->SetPose(pKF->mTcwGBA);



                        // if(pKF->bImu)
                        // {
                        //     //cout << "-------Update inertial values" << endl;
                        //     pKF->mVwbBefGBA = pKF->GetVelocity();
                        //     //if (pKF->mVwbGBA.empty())
                        //     //    Verbose::PrintMess("pKF->mVwbGBA is empty", Verbose::VERBOSITY_NORMAL);

                        //     //assert(!pKF->mVwbGBA.empty());
                        //     pKF->SetVelocity(pKF->mVwbGBA);
                        //     pKF->SetNewBias(pKF->mBiasGBA);                    
                        // }

                        // lpKFtoCheck.pop_front();
                    }

                    // //cout << "GBA: Correct MapPoints" << endl;
                    // // Correct MapPoints
                    // const vector<MapPoint*> vpMPs = pActiveMap->GetAllMapPoints();

                    // for(size_t i=0; i<vpMPs.size(); i++)
                    // {
                    //     MapPoint* pMP = vpMPs[i];

                    //     if(pMP->isBad())
                    //         continue;

                    //     if(pMP->mnBAGlobalForKF==nLoopKF)
                    //     {
                    //         // If optimized by Global BA, just update
                    //         pMP->SetWorldPos(pMP->mPosGBA);
                    //     }
                    //     else
                    //     {
                    //         // Update according to the correction of its reference keyframe
                    //         KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    //         if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                    //             continue;

                    //         /*if(pRefKF->mTcwBefGBA.empty())
                    //             continue;*/

                    //         // Map to non-corrected camera
                    //         // cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    //         // cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    //         Eigen::Vector3f Xc = pRefKF->mTcwBefGBA * pMP->GetWorldPos();

                    //         // Backproject using corrected camera
                    //         pMP->SetWorldPos(pRefKF->GetPoseInverse() * Xc);
                    //     }
                    // }

                    // pActiveMap->InformNewBigChange();
                    // pActiveMap->IncreaseChangeIndex();

                    // // TODO Check this update
                    // // mpTracker->UpdateFrameIMU(1.0f, mpTracker->GetLastKeyFrame()->GetImuBias(), mpTracker->GetLastKeyFrame());

                    // mpLocalMapper->Release();
                    
                    info!("Map updated!"); // Verbose::PrintMess("Map updated!", Verbose::VERBOSITY_NORMAL);
                }

                //mbFinishedGBA = true; // Pranay: never used anywhere
                self.mbRunningGBA = false; //mbRunningGBA = false;
            }
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