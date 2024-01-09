use core::actor::Actor;
use core::config::{SETTINGS, SYSTEM};
use core::sensor::{Sensor, FrameSensor, ImuSensor};
use std::collections::HashSet;
use log::{warn, debug};
use crate::actors::messages::KeyFrameIdMsg;
use crate::map::pose::{Pose, DVTranslation};
use crate::modules::{optimizer, orbmatcher};
use crate::{ActorChannels, MapLock};
use crate::map::map::{Id, Map};
use crate::modules::imu::ImuModule;

use super::messages::{ShutdownMsg, IMUInitializedMsg};
use crate::map::keyframe_database::KeyFrameDatabase;


#[derive(Debug)]
pub struct LoopClosing {
    actor_channels: ActorChannels,
    sensor: Sensor,

    map: MapLock,
    keyframe_database: KeyFrameDatabase,
    _imu: ImuModule,

    current_kf_id: Id, // mpCurrentKF
    last_current_kf: Id, // mpLastCurrentKF
    num_loops: i32, // nLoop

    // Variables related to global BA, might be able to get rid of these?
    running_global_ba: bool, // mbRunningGBA
    finished_global_ba: bool, // mbFinishedGBA
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
            num_loops: 0,
            current_kf_id: -1,
            last_current_kf: -1,
            running_global_ba: false,
            finished_global_ba: false,
            keyframe_database: KeyFrameDatabase::new(),
        }
    }

    fn spawn(actor_channels: ActorChannels, map: Self::MapRef) {
        let mut actor = LoopClosing::new_actorstate(actor_channels, map);
        tracy_client::set_thread_name!("loop closing");

        // Todo these might need to be put in the loopclosing struct but I have it here right now because otherwise it messes with ownership
        let mut loop_data: LoopDetectionData = LoopDetectionData::new();
        let mut merge_data: MergeDetectionData = MergeDetectionData::new();


        'outer: loop {
            let message = actor.actor_channels.receive().unwrap();
            if message.is::<KeyFrameIdMsg>() {
                let msg = message.downcast::<KeyFrameIdMsg>().unwrap_or_else(|_| panic!("Could not downcast loop closing message!"));
                actor.current_kf_id = msg.kf_id;
                actor.loop_closing(&mut loop_data, &mut merge_data);
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
    fn loop_closing(&mut self, loop_data: &mut LoopDetectionData, merge_data: &mut MergeDetectionData) {
        loop_data.common.candidate_kfs = Vec::new();
        merge_data.common.candidate_kfs = Vec::new();

        let (merge_detected, loop_detected) = self.new_detect_common_regions(loop_data, merge_data);

        if merge_detected {
            todo!("multimaps");
            // For reference code look for code under " if(mbMergeDetected) " in LoopClosing::Run()
        }

        if loop_detected {
            debug!("Loop detected");
            let good_loop = true;
            loop_data.common.scw = loop_data.common.slw;

            if self.sensor.is_imu() {
                todo!("IMU");
                // For reference code look for code under first " if(mpCurrentKF->GetMap()->IsInertial()) " in LoopClosing::Run()
            }

            if good_loop {
                self.num_loops += 1;
                loop_data.loop_mappoints = loop_data.common.mps.clone(); // todo timing can we avoid the clone?I dont even know why these are two different variables
                self.correct_loop();
            }

            // Reset all variables
            self.map.write().keyframes.get_mut(&self.last_current_kf).unwrap().dont_delete = false;
            self.map.write().keyframes.get_mut(&loop_data.common.matched_kf).unwrap().dont_delete = false;
            loop_data.reset_vars();
        }
    }

    fn new_detect_common_regions(&mut self, loop_data: &mut LoopDetectionData, _merge_data: &mut MergeDetectionData) -> (bool, bool) {
        // bool LoopClosing::NewDetectCommonRegions()

        // Avoid that a keyframe can be erased while it is being process by this thread
        self.map.write().keyframes.get_mut(&self.current_kf_id).unwrap().dont_delete = true;

        // TODO (multimaps)
        // mpLastMap = mpCurrentKF->GetMap();

        if self.sensor.is_imu() {
            todo!("IMU");
            // if(!mpCurrentKF->GetMap()->isImuInitialized())
            // {
            //     mpKeyFrameDB->add(mpCurrentKF);
            //     mpCurrentKF->SetErase();
            //     return false;
            // }
        }

        if matches!(self.sensor.frame(), FrameSensor::Stereo) && self.map.read().keyframes.len() < 5 {
            todo!("Stereo");
            // cout << "LoopClousure: Stereo KF inserted without check: " << mpCurrentKF->mnId << endl;
            // mpKeyFrameDB->add(mpCurrentKF);
            // mpCurrentKF->SetErase();
            // return false;
        }

        if self.map.read().keyframes.len() < 12 {
            self.keyframe_database.add(self.map.read().keyframes.get(&self.current_kf_id).unwrap());

            self.map.write().keyframes.get_mut(&self.current_kf_id).unwrap().dont_delete = false;
            return (false, false);
        }


        // Check the last candidates with geometric validation
        // Loop candidates
        let mut loop_detected = false; // bLoopDetectedInKF
        if loop_data.common.num_coincidences > 0 {
            // Find from the last KF candidates
            // todo double-check this... I guess this section should not get hit until at least 1 kf has passed through LC? otherwise last_current_kf will be -1
            let tcl = self.map.read().keyframes.get(&self.current_kf_id).unwrap().pose * self.map.read().keyframes.get(&loop_data.common.last_current_kf).unwrap().pose.inverse();
            let scw = tcl * loop_data.common.slw; // mg2oLoopSlw
            let common_region = self.detect_and_reffine_sim3_from_last_kf(
                &mut loop_data.common,
                &scw,
            );
            if common_region {
                loop_detected = true;

                loop_data.common.num_coincidences += 1;
                loop_data.common.last_current_kf = self.current_kf_id;
                loop_data.common.slw = scw;
                self.map.write().keyframes.get_mut(&loop_data.common.last_current_kf).unwrap().dont_delete = false;

                loop_data.common.detected = loop_data.common.num_coincidences >= 3;
                loop_data.common.num_not_found = 0;

                if !loop_data.common.detected {
                    debug!("Loop detected with Reffine Sim3");
                }
            } else {
                loop_detected = false;
                loop_data.common.num_not_found += 1;
                if loop_data.common.num_not_found >= 2 {
                    self.map.write().keyframes.get_mut(&loop_data.common.last_current_kf).unwrap().dont_delete = false;
                    self.map.write().keyframes.get_mut(&loop_data.common.matched_kf).unwrap().dont_delete = false;
                    loop_data.common.num_coincidences = 0;
                    loop_data.common.matched_mps.clear();
                    loop_data.common.mps.clear();
                    loop_data.common.num_not_found = 0;
                }
            }
        }

        let merge_detected = false;
        {
            // TODO (multimaps)
            // For reference code look at " if(mnMergeNumCoincidences > 0) " in LoopClosing::NewDetectCommonRegions
        }

        if merge_detected || loop_detected {
            self.keyframe_database.add(self.map.read().keyframes.get(&self.current_kf_id).unwrap());
            return (merge_detected, loop_detected);
        }

        // Extract candidates from the bag of words
        let (merge_bow_cand, loop_bow_cand) = match !merge_detected && !loop_detected {
            true => {
                // Search in BoW
                self.keyframe_database.detect_n_best_candidates(
                    &self.map,
                    self.current_kf_id,
                    3
                )

            },
            false => (Vec::new(), Vec::new())
        };

        // Check the BoW candidates if the geometric candidate list is empty
        // Loop candidates
        if !loop_detected && loop_bow_cand.is_empty() {
            loop_detected = self.detect_common_regions_from_bow(
                &loop_bow_cand,
                &loop_data.common,
            );
        }

        // Merge candidates
        if !merge_detected && !merge_bow_cand.is_empty() {
            todo!("multimaps");
        }

        self.keyframe_database.add(self.map.read().keyframes.get(&self.current_kf_id).unwrap());

        if merge_detected || loop_detected {
            return (merge_detected, loop_detected);
        }

        self.map.write().keyframes.get_mut(&self.current_kf_id).unwrap().dont_delete = false; 

        return (false,false);
    }

    fn detect_and_reffine_sim3_from_last_kf(
        &self, detection_data: &mut CommonDetectionData, scw: &Pose,
    ) -> bool {
        // bool LoopClosing::DetectAndReffineSim3FromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
        //                                          std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        let num_proj_matches = self.find_matches_by_projection(detection_data, scw);

        if num_proj_matches >= 30 {
            let twm = self.map.read().keyframes.get(&self.current_kf_id).unwrap().pose.inverse();
            let scm = *scw * twm;

            let fixed_scale = match self.sensor {
                Sensor(FrameSensor::Mono, ImuSensor::Some) => {
                    todo!("IMU");
                    // if !pCurrentKF->GetMap()->GetIniertialBA2()) { return false }
                },
                _ => false
            };
            // todo matched_mps needs to be used somewhere but hessian_7x7 truly seems unused
            let (num_opt_matches, hessian_7x7) = optimizer::optimize_sim3(
                &self.map, self.current_kf_id, detection_data.matched_kf, &mut detection_data.matched_mps, &scm, 10.0, fixed_scale,
            );

            if num_opt_matches > 50 {
                let num_proj_matches  = self.find_matches_by_projection(detection_data, scw);
                // nNumProjMatches = FindMatchesByProjection(pCurrentKF, pMatchedKF, gScw_estimation, spAlreadyMatchedMPs, vpMPs, vpMatchedMPs);
                if num_proj_matches >= 100 {
                    detection_data.scw = *scw; // todo is this right?
                    return true;
                }
            }
        }
        return false;
    }

    fn detect_common_regions_from_bow(
        &self, bow_cands: &Vec<Id>, detection_data: &CommonDetectionData,
    ) -> bool {
        todo!("MVP LoopClosing::DetectCommonRegionsFromBoW");
    }

    fn detect_common_regions_from_last_kf(
        &self, detection_data: &mut CommonDetectionData, scw: &Pose
    ) -> (bool, i32) {
        // bool LoopClosing::DetectCommonRegionsFromLastKF(KeyFrame* pCurrentKF, KeyFrame* pMatchedKF, g2o::Sim3 &gScw, int &nNumProjMatches,
        //                                         std::vector<MapPoint*> &vpMPs, std::vector<MapPoint*> &vpMatchedMPs)
        let num_proj_matches = self.find_matches_by_projection(detection_data, scw);
        if num_proj_matches >= 30 {
            return (true, num_proj_matches);
        }
        return (false, num_proj_matches);
    }

    fn find_matches_by_projection(
        &self, detection_data: &mut CommonDetectionData, scw: &Pose
    ) -> i32 {
        // int LoopClosing::FindMatchesByProjection(KeyFrame* pCurrentKF, KeyFrame* pMatchedKFw, g2o::Sim3 &g2oScw,
        //                                         set<MapPoint*> &spMatchedMPinOrigin, vector<MapPoint*> &vpMapPoints,
        //                                         vector<MapPoint*> &vpMatchedMapPoints)
    
        let num_covisibles = 10;
        let mut cov_kf_matched = self.map.read().keyframes.get(&detection_data.matched_kf).unwrap().get_covisibility_keyframes(num_covisibles); // vpCovKFm
        let initial_cov = cov_kf_matched.len();
        cov_kf_matched.push(detection_data.matched_kf);

        let mut check_kfs: HashSet<i32> = HashSet::from_iter(cov_kf_matched.iter().cloned()); // spCheckKFs
        let current_covisibles = self.map.read().keyframes.get(&self.current_kf_id).unwrap().get_covisibility_keyframes(i32::MAX); // spCurrentCovisbles

        if (initial_cov as i32) < num_covisibles {
            for i in 0..initial_cov {
                let kfs = self.map.read().keyframes.get(&cov_kf_matched[i]).unwrap().get_covisibility_keyframes(num_covisibles);
                let mut num_inserted = 0;
                let mut j = 0;
                while j < kfs.len() && num_inserted < num_covisibles {
                    if !check_kfs.contains(&kfs[j]) && !current_covisibles.contains(&kfs[j]) {
                        check_kfs.insert(kfs[j]);
                        num_inserted += 1;
                    }
                    j += 1;
                }
                cov_kf_matched.extend(kfs);
            }
        }

        let mut sp_map_points = HashSet::new();
        detection_data.matched_mps.clear();
        detection_data.mps.clear();

        for keyframe in cov_kf_matched {
            let read = self.map.read();
            let kf = read.keyframes.get(&keyframe).unwrap();
            for mp in kf.get_mp_matches() {
                match mp {
                    None => continue,
                    Some((id, _)) => {
                        if !sp_map_points.contains(id) {
                            sp_map_points.insert(*id);
                            detection_data.mps.push(*id);
                        }
                    }
                }
            }
        }
        // todo do we need this?
        // vpMatchedMapPoints.resize(pCurrentKF->GetMapPointMatches().size(), static_cast<MapPoint*>(NULL));
        let num_matches = orbmatcher::search_by_projection_for_loop_detection(&self.map, scw, &detection_data.mps, &detection_data.matched_mps, 3, 1.5);

        return num_matches;
    }

    pub fn correct_loop(&self) {
        todo!("MVP LoopClosing::CorrectLoop");
    }

    pub fn search_and_fuse_corrected_poses_map(
        &self, corrected_poses_map: &Vec<(Id, &Pose)>, mps: &Vec<Id>
    ) {
        // void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap, vector<MapPoint*> &vpMapPoints)

        let mut total_replaced = 0;

        for (kf_id, pose) in corrected_poses_map {
            let mut num_replaced = 0;
            let replace_points = orbmatcher::fuse_from_loop_closing(kf_id, pose, mps, &self.map, 4, 0.8);

            for i in 0..mps.len() {
                let rep = replace_points[i];
                match rep {
                    None => continue,
                    Some(mp_id) => {
                        num_replaced += 1;
                        self.map.write().replace_mappoint(mp_id, mps[i]);
                    }
                }
            }
            total_replaced += num_replaced;
        }
        debug!("Fuse pose: {} MPs had been fused", total_replaced);
        todo!("MVP LoopClosing::SearchAndFuse");
    }

    pub fn search_and_fuse_connected_kfs(&self, connected_kfs: &Vec<Id>, mps: &Vec<Id>) {
        // void LoopClosing::SearchAndFuse(const vector<KeyFrame*> &vConectedKFs, vector<MapPoint*> &vpMapPoints)

        let mut total_replaced = 0;
        for kf_id in connected_kfs {
            let mut num_replaced = 0;
            let scw = self.map.read().keyframes.get(&kf_id).unwrap().pose;

            let replace_points = orbmatcher::fuse_from_loop_closing(kf_id, &scw, mps, &self.map, 4, 0.8);

            for i in 0..mps.len() {
                let rep = replace_points[i];
                match rep {
                    None => continue,
                    Some(mp_id) => {
                        num_replaced += 1;
                        self.map.write().replace_mappoint(mp_id, mps[i]);
                    }
                }
            }
            debug!("Fuse pose: {} MPs had been fused", num_replaced);
            total_replaced += num_replaced;
        }
        debug!("Fuse pose: {} MPs had been fused", total_replaced);
        todo!("MVP LoopClosing::SearchAndFuseConnectedKFs");
    }

    pub fn run_global_ba(&self) {
        todo!("MVP LoopClosing::RunGlobalBundleAdjustment");
    }

    pub fn merge_local(&self) {
        todo!("multimaps ... LoopClosing::MergeLocal");
    }

    pub fn merge_local2(&self) {
        todo!("multimaps ... LoopClosing::MergeLocal2");
    }

}



#[derive(Debug)]
struct CommonDetectionData {
    // Data for loop or merge detection as loop closing runs.
    // In Orbslam this data is saved once per loop and merge in the LoopClosing object
    // but is hard to keep track of so we combine all the loop-related and merge-related vars.
    detected: bool, // mbLoopDetected or mbMergeDetected
    num_coincidences: i32, // mnLoopNumCoincidences or mnMergeNumCoincidences
    num_not_found: i32, // mnLoopNumNotFound or mnMergeNumNotFound
    last_current_kf: Id, // mpLoopLastCurrentKF or mpMergeLastCurrentKF
    slw: Pose, // mg2oLoopSlw or mg2oMergeSlw
    scw: Pose, // mg2oLoopScw or mg2oMergeScw
    candidate_kfs: Vec<Id>, // mvpLoopCandKFs or mvpMergeCandKFs in KeyFrame
    mps: Vec<Id>, // mvpLoopMPs or mvpMergeMPs
    matched_mps: Vec<Id>, // mvpLoopMatchedMPs or mvpMergeMatchedMPs
    matched_kf: Id, // mpLoopMatchedKF or mpMergeMatchedKF
}
impl CommonDetectionData {
    fn new() -> Self {
        Self {
            detected: false,
            num_coincidences: 0,
            num_not_found: 0,
            last_current_kf: -1,
            slw: Pose::identity(),
            scw: Pose::identity(),
            candidate_kfs: vec![],
            mps: vec![],
            matched_mps: vec![],
            matched_kf: -1,
        }
    }
}

#[derive(Debug)]
struct LoopDetectionData {
    common: CommonDetectionData,
    last_loop_kf_id: Id, // mLastLoopKFid
    matched_kf: Id, // mpMatchedKF
    consistent_groups: Vec<(Id, i32)>, // mvConsistentGroups
    enough_consistent_candidates: Vec<Id>, // mvpEnoughConsistentCandidates
    current_connected_kfs: Vec<Id>, // mvpCurrentConnectedKFs
    current_matched_points: Vec<Id>, // mvpCurrentMatchedPoints
    loop_mappoints: Vec<Id>, // mvpLoopMapPoints
    scw: Pose, // mg2oScw
}
impl LoopDetectionData {
    fn new() -> Self {
        Self {
            common: CommonDetectionData::new(),
            last_loop_kf_id: -1,
            loop_mappoints: Vec::new(),
            matched_kf: -1,
            consistent_groups: vec![],
            enough_consistent_candidates: vec![],
            current_connected_kfs: vec![],
            current_matched_points: vec![],
            scw: Pose::identity(),
        }
    }

    fn reset_vars(&mut self) {
        self.common.num_coincidences = 0;
        self.common.matched_mps.clear();
        self.common.mps.clear();
        self.common.num_not_found = 0;
        self.common.detected = false;
    }
}

#[derive(Debug)]
struct MergeDetectionData {
    common: CommonDetectionData,
    smw: Pose, // mg2oMergeSmw
    connected_kfs: Vec<Id>, // mvpMergeConnectedKFs
}
impl MergeDetectionData {
    fn new() -> Self {
        Self {
            common: CommonDetectionData::new(),
            smw: Pose::identity(),
            connected_kfs: vec![],
        }
    }
}