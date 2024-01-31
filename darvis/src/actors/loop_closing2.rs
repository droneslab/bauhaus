use core::actor::Actor;
use core::config::{SETTINGS, SYSTEM};
use core::matrix::{DVMatrix, DVMatrix4};
use core::sensor::{Sensor, FrameSensor, ImuSensor};
use std::collections::{HashMap, HashSet};
use std::thread;
use std::time::Duration;
use log::{warn, debug};
use crate::actors::messages::KeyFrameIdMsg;
use crate::map::bow::VOCABULARY;
use crate::map::pose::{Pose, DVTranslation};
use crate::modules::sim3solver::Sim3Solver;
use crate::modules::{optimizer, orbmatcher};
use crate::registered_actors::LOOP_CLOSING;
use crate::{ActorChannels, MapLock};
use crate::map::map::{Id, Map};
use crate::modules::imu::ImuModule;

use super::messages::{ShutdownMsg, IMUInitializedMsg};
use crate::map::keyframe_database2::{KeyFrameDatabase};

type ConsistentGroup = (HashSet<Id>, i32);

#[derive(Debug)]
pub struct LoopClosing {
    actor_channels: ActorChannels,
    sensor: Sensor,

    map: MapLock,
    keyframe_database: KeyFrameDatabase,
    _imu: ImuModule,

    // Loop detector variables
    current_kf_id: Id, // mpCurrentKF
    matched_kf: Id, // mpMatchedKF
    consistent_groups: Vec<ConsistentGroup>, // mvConsistentGroups
    current_matched_points: Vec<Option<Id>>, // mvpCurrentMatchedPoints
    enough_consistent_candidates: Vec<Id>, // mvpEnoughConsistentCandidates;
    current_connected_kfs: Vec<Id>, // mvpCurrentConnectedKFs
    loop_mappoints: Vec<Id>, // mvpLoopMapPoints
    scw: DVMatrix4<f32>, // mScw
    // g2o::Sim3 mg2oScw;

    last_loop_kf_id: Id, // mLastLoopKFid

    // Variables related to global BA, might be able to get rid of these?
    running_global_ba: bool, // mbRunningGBA
    finished_global_ba: bool, // mbFinishedGBA

    // bool mnFullBAIdx;
}

impl Actor for LoopClosing {
    type MapRef = MapLock;

    fn new_actorstate(actor_channels: ActorChannels, map: Self::MapRef) -> LoopClosing {
        let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");

        LoopClosing {
            actor_channels,
            map,
            _imu: ImuModule::new(None, None, sensor, false, false),
            sensor,
            current_kf_id: -1,
            running_global_ba: false,
            finished_global_ba: false,
            keyframe_database: KeyFrameDatabase::new(),
            consistent_groups: Vec::new(),
            matched_kf: -1,
            last_loop_kf_id: -1,
            current_matched_points: Vec::new(),
            enough_consistent_candidates: Vec::new(),
            current_connected_kfs: Vec::new(),
            loop_mappoints: Vec::new(),
            scw: DVMatrix4::identity(),
        }
    }

    fn spawn(actor_channels: ActorChannels, map: Self::MapRef) {
        let mut actor = LoopClosing::new_actorstate(actor_channels, map);
        tracy_client::set_thread_name!("loop closing");

        'outer: loop {
            let message = actor.actor_channels.receive().unwrap();
            if message.is::<KeyFrameIdMsg>() {
                let msg = message.downcast::<KeyFrameIdMsg>().unwrap_or_else(|_| panic!("Could not downcast loop closing message!"));
                actor.current_kf_id = msg.kf_id;
                actor.loop_closing();
            } else if message.is::<IMUInitializedMsg>() {
                todo!("IMU: Process message from local mapping");
            } else if message.is::<ShutdownMsg>() {
                break 'outer;
            } else {
                warn!("Loop Closing received unknown message type!");
            }
        }
    }


}

impl LoopClosing {
    fn loop_closing(&mut self) {
        // Avoid that a keyframe can be erased while it is being process by this thread
        self.map.write().keyframes.get_mut(&self.current_kf_id).unwrap().dont_delete = true;

        // Detect loop candidates and check covisibility consistency
        if self.detect_loop() {
            self.keyframe_database.add(self.map.read().keyframes.get(&self.current_kf_id).unwrap());

            // Compute similarity transformation [sR|t]
            // In the stereo/RGBD case s=1
            match self.compute_sim3() {
                Ok(has_match) => {
                    if has_match {
                        // Perform loop fusion and pose graph optimization
                        self.correct_loop();
                    }
                },
                Err(e) => {
                    debug!("Loop closing failed to compute sim3: {}", e);
                }
            }
        }
        self.map.write().keyframes.get_mut(&self.current_kf_id).unwrap().dont_delete = false;
        thread::sleep(Duration::from_millis(5000));
    }

    fn detect_loop(&mut self) -> bool {
        //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
        if self.map.read().keyframes.len() < 10 {
            return false;
        }

        // Compute reference BoW similarity score
        // This is the lowest score to a connected keyframe in the covisibility graph
        // We will impose loop candidates to have a higher similarity than this
        let connected_keyframes = self.map.read().keyframes.get(&self.current_kf_id).unwrap().get_covisibility_keyframes(i32::MAX);
        let mut min_score = 1.0;
        for kf_id in connected_keyframes {
            let score = VOCABULARY.score(
                &self.map.read().keyframes.get(&self.current_kf_id).unwrap(),
                &self.map.read().keyframes.get(&kf_id).unwrap()
            );
            if score < min_score {
                min_score = score;
            }
        }

        // Query the database imposing the minimum score
        let candidate_kfs = self.keyframe_database.detect_loop_candidates(&self.map, &self.current_kf_id, min_score);

        // If there are no loop candidates, just add new keyframe and return false
        if candidate_kfs.is_empty() {
        //     mvConsistentGroups.clear();
            return false;
        }

        // For each loop candidate check consistency with previous loop candidates
        // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
        // A group is consistent with a previous group if they share at least a keyframe
        // We must detect a consistent loop in several consecutive keyframes to accept it
        self.enough_consistent_candidates.clear();
        let mut current_consistent_groups = Vec::new(); // vCurrentConsistentGroups
        let mut consistent_group = vec![false; self.consistent_groups.len()]; // vbConsistentGroup

        for cand_kf_id in candidate_kfs {
            let map_read_lock = self.map.read();
            let candidate_kf = map_read_lock.keyframes.get(&cand_kf_id).unwrap();
            let mut candidate_group: HashSet<Id> = HashSet::from_iter(candidate_kf.get_covisibility_keyframes(i32::MAX).iter().cloned());
            candidate_group.insert(cand_kf_id);

            let mut enough_consistent = false;
            let mut consistent_for_some_group = false;
            for i in 0..self.consistent_groups.len() {
                let (previous_group, previous_consistency) = &self.consistent_groups[i];
                let mut consistent = false;
                for kf_id in &candidate_group {
                    if previous_group.contains(kf_id) {
                        consistent = true;
                        consistent_for_some_group = true;
                        break;
                    }
                }

                if consistent {
                    let current_consistency = previous_consistency + 1;
                    if !consistent_group[i] {
                        current_consistent_groups.push((candidate_group.clone(), current_consistency));
                        consistent_group.push(true); //this avoid to include the same group more than once
                    }
                    if current_consistency >= SETTINGS.get::<i32>(LOOP_CLOSING, "covisibility_consistency_threshold") && !enough_consistent {
                        self.enough_consistent_candidates.push(cand_kf_id);
                        enough_consistent = true; //this avoid to insert the same candidate more than once
                    }
                }
            }

            // If the group is not consistent with any previous group insert with consistency counter set to zero
            if !consistent_for_some_group {
                current_consistent_groups.push((candidate_group, 0));
            }
        }

        // Update Covisibility Consistent Groups
        self.consistent_groups = current_consistent_groups;

        return !self.enough_consistent_candidates.is_empty();
    }

    fn compute_sim3(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        // For each consistent loop candidate we try to compute a Sim3
        let initial_candidates = self.enough_consistent_candidates.len();

        // We compute first ORB matches for each candidate
        // If enough matches are found, we setup a Sim3Solver

        let mappoint_matches = vec![vec![];initial_candidates]; //vvpMapPointMatches

        let mut discarded = vec![false; initial_candidates]; // vbDiscarded
        let mut solvers = HashMap::new(); // vpSim3Solvers
        let mut num_candidates = 0; // nCandidates, candidates with enough matches

        {
            let map_read_lock = self.map.read();
            let current_keyframe = map_read_lock.keyframes.get(&self.current_kf_id).unwrap();
            for i in 0..initial_candidates {
                let keyframe = map_read_lock.keyframes.get(&self.enough_consistent_candidates[i]).unwrap();
                let mappoint_matches = orbmatcher::search_by_bow_kf(current_keyframe, keyframe, true, 0.75)?;
                if mappoint_matches.len() < 20 {
                    discarded[i] = true;
                    continue;
                } else {
                    let sim3_solver = Sim3Solver::new(
                        &self.map,
                        current_keyframe.id, keyframe.id, &mappoint_matches, false,
                        0.99, 20, 300
                    );
                    solvers.insert(i, sim3_solver);
                }
                num_candidates += 1;
            }
        }
        let mut has_match = false;

        // Perform alternatively RANSAC iterations for each candidate
        // until one is succesful or all fail
        while num_candidates > 0 && !has_match {
            for i in 0..initial_candidates {
                if discarded[i] {
                    continue;
                }
                let kf_id = self.enough_consistent_candidates[i];

                // Perform 5 Ransac Iterations
                let solver = solvers.get_mut(&i).unwrap();
                let (no_more, sim3_result) = solver.iterate(5)?;

                // If Ransac reachs max. iterations discard keyframe
                if no_more {
                    discarded[i] = true;
                    num_candidates -= 1;
                }

                // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
                if sim3_result.is_some() {
                    let (scm, inliers, num_inliers) = sim3_result.unwrap();
                    let mut mappoint_matches_copy = vec![None; mappoint_matches.len()]; //vpMapPointMatches
                    for j in 0..inliers.len() {
                        if inliers[j] {
                            mappoint_matches_copy[j] = Some(mappoint_matches[i][j]);
                        }
                    }

                    let (R, t, s) = solver.get_estimates();
                    orbmatcher::search_by_sim3(
                        &self.map, self.current_kf_id, kf_id, &mut mappoint_matches_copy, s, &R, &t, 7.5
                    );

                    // g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s); // todo is this necessary
                    let (num_inliers, acum_hessian) = optimizer::optimize_sim3(
                        &self.map, self.current_kf_id, kf_id, &mut mappoint_matches_copy, &R, &t, &s, 10, false
                    );

                    // If optimization is succesful stop ransacs and continue
                    if num_inliers >= 20 {
                        has_match = true;
                        self.matched_kf = kf_id;
                        todo!("Check these lines");
                        // g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                        // mg2oScw = gScm*gSmw;
                        // mScw = Converter::toCvMat(mg2oScw);

                        self.current_matched_points = mappoint_matches_copy;
                        break;
                    }
                }
            }
        }

        if !has_match {
            for i in 0..initial_candidates {
                self.map.write().keyframes.get_mut(&self.enough_consistent_candidates[i]).unwrap().dont_delete = false;
            }
            return Ok(false);
        }

        // Retrieve MapPoints seen in Loop Keyframe and neighbors
        let mut loop_connected_keyframes = self.map.read().keyframes.get(&self.matched_kf).unwrap().get_covisibility_keyframes(i32::MAX); // vpLoopConnectedKFs
        loop_connected_keyframes.push(self.matched_kf);
        self.loop_mappoints.clear();
        let mut loop_point_for_kf = HashMap::new(); // mnLoopPointForKF
        {
            let map_read_lock = self.map.read();
            for kf_id in loop_connected_keyframes {
                let keyframe = map_read_lock.keyframes.get(&kf_id).unwrap();
                let mappoints = keyframe.get_mp_matches();
                for mp in mappoints {
                    match mp {
                        Some((id, _)) => {
                            let has_point = loop_point_for_kf.contains_key(id);
                            if !has_point || (*loop_point_for_kf.get(id).unwrap() != self.current_kf_id) {
                                self.loop_mappoints.push(*id);
                                loop_point_for_kf.insert(*id, self.current_kf_id);
                            }
                        },
                        None => continue
                    }
                }
            }
        }

        // Find more matches projecting with the computed Sim3
        orbmatcher::search_by_projection_for_loop_detection(
            &self.map, &self.current_kf_id, &self.scw, &self.loop_mappoints, &mut self.current_matched_points, 10,
            0.75, true
        );

        // If enough matches accept Loop
        let mut n_total_matches = 0;
        for i in 0..self.current_matched_points.len() {
            if self.current_matched_points[i].is_some() {
                n_total_matches += 1;
            }
        }

        if n_total_matches >= 40 {
            for i in 0..initial_candidates {
                if self.enough_consistent_candidates[i] != self.matched_kf {
                    self.map.write().keyframes.get_mut(&self.enough_consistent_candidates[i]).unwrap().dont_delete = true;
                }
            }
            return Ok(true);
        } else {
            for i in 0..initial_candidates {
                self.map.write().keyframes.get_mut(&self.enough_consistent_candidates[i]).unwrap().dont_delete = false;
            }
            return Ok(false);
        }

    }

    fn correct_loop(&mut self) {
        // cout << "Loop detected!" << endl;

        // // Send a stop signal to Local Mapping
        // // Avoid new keyframes are inserted while correcting the loop
        // mpLocalMapper->RequestStop();

        // // If a Global Bundle Adjustment is running, abort it
        // if(isRunningGBA())
        // {
        //     unique_lock<mutex> lock(mMutexGBA);
        //     mbStopGBA = true;

        //     mnFullBAIdx++;

        //     if(mpThreadGBA)
        //     {
        //         mpThreadGBA->detach();
        //         delete mpThreadGBA;
        //     }
        // }

        // // Wait until Local Mapping has effectively stopped
        // while(!mpLocalMapper->isStopped())
        // {
        //     usleep(1000);
        // }

        // // Ensure current keyframe is updated
        // mpCurrentKF->UpdateConnections();

        // // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
        // mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
        // mvpCurrentConnectedKFs.push_back(mpCurrentKF);

        // KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
        // CorrectedSim3[mpCurrentKF]=mg2oScw;
        // cv::Mat Twc = mpCurrentKF->GetPoseInverse();


        // {
        //     // Get Map Mutex
        //     unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        //     for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        //     {
        //         KeyFrame* pKFi = *vit;

        //         cv::Mat Tiw = pKFi->GetPose();

        //         if(pKFi!=mpCurrentKF)
        //         {
        //             cv::Mat Tic = Tiw*Twc;
        //             cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
        //             cv::Mat tic = Tic.rowRange(0,3).col(3);
        //             g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
        //             g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
        //             //Pose corrected with the Sim3 of the loop closure
        //             CorrectedSim3[pKFi]=g2oCorrectedSiw;
        //         }

        //         cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
        //         cv::Mat tiw = Tiw.rowRange(0,3).col(3);
        //         g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
        //         //Pose without correction
        //         NonCorrectedSim3[pKFi]=g2oSiw;
        //     }

        //     // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        //     for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        //     {
        //         KeyFrame* pKFi = mit->first;
        //         g2o::Sim3 g2oCorrectedSiw = mit->second;
        //         g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

        //         g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

        //         vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
        //         for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
        //         {
        //             MapPoint* pMPi = vpMPsi[iMP];
        //             if(!pMPi)
        //                 continue;
        //             if(pMPi->isBad())
        //                 continue;
        //             if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
        //                 continue;

        //             // Project with non-corrected pose and project back with corrected pose
        //             cv::Mat P3Dw = pMPi->GetWorldPos();
        //             Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
        //             Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

        //             cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
        //             pMPi->SetWorldPos(cvCorrectedP3Dw);
        //             pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
        //             pMPi->mnCorrectedReference = pKFi->mnId;
        //             pMPi->UpdateNormalAndDepth();
        //         }

        //         // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
        //         Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
        //         Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
        //         double s = g2oCorrectedSiw.scale();

        //         eigt *=(1./s); //[R t/s;0 1]

        //         cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

        //         pKFi->SetPose(correctedTiw);

        //         // Make sure connections are updated
        //         pKFi->UpdateConnections();
        //     }

        //     // Start Loop Fusion
        //     // Update matched map points and replace if duplicated
        //     for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        //     {
        //         if(mvpCurrentMatchedPoints[i])
        //         {
        //             MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
        //             MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
        //             if(pCurMP)
        //                 pCurMP->Replace(pLoopMP);
        //             else
        //             {
        //                 mpCurrentKF->AddMapPoint(pLoopMP,i);
        //                 pLoopMP->AddObservation(mpCurrentKF,i);
        //                 pLoopMP->ComputeDistinctiveDescriptors();
        //             }
        //         }
        //     }

        // }

        // // Project MapPoints observed in the neighborhood of the loop keyframe
        // // into the current keyframe and neighbors using corrected poses.
        // // Fuse duplications.
        // SearchAndFuse(CorrectedSim3);


        // // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
        // map<KeyFrame*, set<KeyFrame*> > LoopConnections;

        // for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        // {
        //     KeyFrame* pKFi = *vit;
        //     vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        //     // Update connections. Detect new links.
        //     pKFi->UpdateConnections();
        //     LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        //     for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        //     {
        //         LoopConnections[pKFi].erase(*vit_prev);
        //     }
        //     for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        //     {
        //         LoopConnections[pKFi].erase(*vit2);
        //     }
        // }

        // // Optimize graph
        // Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

        // mpMap->InformNewBigChange();

        // // Add loop edge
        // mpMatchedKF->AddLoopEdge(mpCurrentKF);
        // mpCurrentKF->AddLoopEdge(mpMatchedKF);

        // // Launch a new thread to perform Global Bundle Adjustment
        // mbRunningGBA = true;
        // mbFinishedGBA = false;
        // mbStopGBA = false;
        // mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

        // // Loop closed. Release Local Mapping.
        // mpLocalMapper->Release();    

        // mLastLoopKFid = mpCurrentKF->mnId;   

    }

    fn search_and_fuse(&mut self) {
        // ORBmatcher matcher(0.8);

        // for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
        // {
        //     KeyFrame* pKF = mit->first;

        //     g2o::Sim3 g2oScw = mit->second;
        //     cv::Mat cvScw = Converter::toCvMat(g2oScw);

        //     vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        //     matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        //     // Get Map Mutex
        //     unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        //     const int nLP = mvpLoopMapPoints.size();
        //     for(int i=0; i<nLP;i++)
        //     {
        //         MapPoint* pRep = vpReplacePoints[i];
        //         if(pRep)
        //         {
        //             pRep->Replace(mvpLoopMapPoints[i]);
        //         }
        //     }
        // }

    }
    
    fn run_gba(&mut self) {
        // cout << "Starting Global Bundle Adjustment" << endl;

        // int idx =  mnFullBAIdx;
        // Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

        // // Update all MapPoints and KeyFrames
        // // Local Mapping was active during BA, that means that there might be new keyframes
        // // not included in the Global BA and they are not consistent with the updated map.
        // // We need to propagate the correction through the spanning tree
        // {
        //     unique_lock<mutex> lock(mMutexGBA);
        //     if(idx!=mnFullBAIdx)
        //         return;

        //     if(!mbStopGBA)
        //     {
        //         cout << "Global Bundle Adjustment finished" << endl;
        //         cout << "Updating map ..." << endl;
        //         mpLocalMapper->RequestStop();
        //         // Wait until Local Mapping has effectively stopped

        //         while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
        //         {
        //             usleep(1000);
        //         }

        //         // Get Map Mutex
        //         unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        //         // Correct keyframes starting at map first keyframe
        //         list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

        //         while(!lpKFtoCheck.empty())
        //         {
        //             KeyFrame* pKF = lpKFtoCheck.front();
        //             const set<KeyFrame*> sChilds = pKF->GetChilds();
        //             cv::Mat Twc = pKF->GetPoseInverse();
        //             for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
        //             {
        //                 KeyFrame* pChild = *sit;
        //                 if(pChild->mnBAGlobalForKF!=nLoopKF)
        //                 {
        //                     cv::Mat Tchildc = pChild->GetPose()*Twc;
        //                     pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
        //                     pChild->mnBAGlobalForKF=nLoopKF;

        //                 }
        //                 lpKFtoCheck.push_back(pChild);
        //             }

        //             pKF->mTcwBefGBA = pKF->GetPose();
        //             pKF->SetPose(pKF->mTcwGBA);
        //             lpKFtoCheck.pop_front();
        //         }

        //         // Correct MapPoints
        //         const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

        //         for(size_t i=0; i<vpMPs.size(); i++)
        //         {
        //             MapPoint* pMP = vpMPs[i];

        //             if(pMP->isBad())
        //                 continue;

        //             if(pMP->mnBAGlobalForKF==nLoopKF)
        //             {
        //                 // If optimized by Global BA, just update
        //                 pMP->SetWorldPos(pMP->mPosGBA);
        //             }
        //             else
        //             {
        //                 // Update according to the correction of its reference keyframe
        //                 KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

        //                 if(pRefKF->mnBAGlobalForKF!=nLoopKF)
        //                     continue;

        //                 // Map to non-corrected camera
        //                 cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
        //                 cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
        //                 cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

        //                 // Backproject using corrected camera
        //                 cv::Mat Twc = pRefKF->GetPoseInverse();
        //                 cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
        //                 cv::Mat twc = Twc.rowRange(0,3).col(3);

        //                 pMP->SetWorldPos(Rwc*Xc+twc);
        //             }
        //         }            

        //         mpMap->InformNewBigChange();

        //         mpLocalMapper->Release();

        //         cout << "Map updated!" << endl;
        //     }

        //     mbFinishedGBA = true;
        //     mbRunningGBA = false;
        // }

    }


}

