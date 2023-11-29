use std::collections::{HashMap, HashSet};
use log::{info, warn, error, debug};
use dvcore::{matrix::{DVVector3, DVVectorOfPoint2f, DVVectorOfPoint3f}, config::{SETTINGS, SYSTEM}, sensor::{Sensor, FrameSensor, ImuSensor}};
use crate::{
    dvmap::{keyframe::*, mappoint::*, pose::DVPose},
    modules::{map_initialization::Initialization, optimizer::{self}}
};

use super::misc::Timestamp;

pub type Id = i32;

#[derive(Debug, Clone, Default)]
pub struct Map {
    pub id: Id,

    pub keyframes: HashMap<Id, Frame<FullKeyFrame>>, // = mspKeyFrames
    pub mappoints: HashMap<Id, MapPoint<FullMapPoint>>, // = mspMapPoints

    // IMU
    pub imu_initialized: bool, // isImuInitialized(), set true by local mapper
    pub imu_ba2: bool, // mbIMU_BA2

    // KeyFrames
    last_kf_id: Id, // = mnMaxKFid
    pub initial_kf_id: Id, // TODO (multimaps): this is the initial kf id that was added to this map, when this map is made. should tie together map versioning better, maybe in a single struct

    // MapPoints
    last_mp_id: Id,

    sensor: Sensor,
    // TODO (mvp): following are in orbslam3, not sure if we need:
    // mvpKeyFrameOrigins: Vec<KeyFrame>
    // mvBackupKeyFrameOriginsId: Vec<: u32>
    // mpFirstRegionKF: KeyFrame* 
    // mpKFlowerID: KeyFrame*

    // TODO (mvp): following are in orbslam3, probably don't need
    // mbFail: bool
    // nNextId: : u32
    // mbImuInitialized: bool
    // mnMapChange: bool
    // mnMapChangeNotified: bool
    // mnBigChangeIdx: i32
}

impl Map {
    pub fn new() -> Map {
        let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");

        Map {
            id: 0, // TODO (Multimaps): this should increase when new maps are made
            sensor,
            last_kf_id: -1,
            last_mp_id: -1,
            imu_initialized: false,
            ..Default::default()
        }
    }

    pub fn num_keyframes(&self) -> i32 { self.keyframes.len() as i32 }
    pub fn get_keyframe(&self, id: &Id) -> Option<&Frame<FullKeyFrame>> { self.keyframes.get(id) }
    pub fn get_all_keyframes(&self) -> &HashMap<Id, Frame<FullKeyFrame>> { &self.keyframes }
    pub fn get_mappoint(&self, id: &Id) -> Option<&MapPoint<FullMapPoint>> { self.mappoints.get(id) }
    pub fn get_all_mappoints(&self) -> &HashMap<Id, MapPoint<FullMapPoint>> { &self.mappoints }
    pub fn print_current_map(&self, identifier: String) {
        // For debugging
        println!("PRINT MAP START;{}", identifier);
        for (id, kf) in &self.keyframes {
            print!("PRINT MAP KF;{};{:?}", id, kf.pose.unwrap());
            print!(";");
            for connection in &kf.get_covisibility_keyframes(i32::MAX) {
                print!("{},", connection);
            };
            print!(";");
            for (index, (mp_id, _)) in &kf.mappoint_matches {
                print!("{},", mp_id);
            };
            println!("");
        }
        for (id, mp) in &self.mappoints {
            print!("PRINT MAP MP;{};{:?}", id, mp.position);
            print!(";");
            for (kf_id, indexes) in mp.get_observations() {
                print!("{},", kf_id);
            };
            println!("");
        }
        println!("PRINT MAP DONE");
    }

    ////* &mut self ... behind map actor */////////////////////////////////////////////////////
    pub fn insert_mappoint_to_map(&mut self, mp: MapPoint<PrelimMapPoint>, observations_to_add: Vec<(Id, u32, usize)>) -> Id {
        // Note: I would really like this to consume the mp, but this brings up issues
        // with the map actor being able to take ownership of the message.
        self.last_mp_id += 1;
        let new_mp_id = self.last_mp_id;

        let full_mappoint = MapPoint::<FullMapPoint>::new(mp, self.last_mp_id);
        self.mappoints.insert(self.last_mp_id, full_mappoint);

        let mp = self.mappoints.get_mut(&new_mp_id).unwrap();
        for (kf_id, num_keypoints, index) in observations_to_add {
            // Add observation for mp->kf
            mp.add_observation(&kf_id, num_keypoints, index as u32);

            // Add observation kf->mp
            self.keyframes.get_mut(&kf_id).map(|kf| {
                kf.add_mappoint(index as u32, mp.get_id(), false);
            });
        }

        // Compute distinctive descriptors
        let best_descriptor = self.mappoints.get(&new_mp_id)
            .and_then(|mp| mp.compute_distinctive_descriptors(&self)).unwrap();
        self.mappoints.get_mut(&new_mp_id)
            .map(|mp| mp.update_distinctive_descriptors(best_descriptor));

        // Update normal and depth
        let norm_and_depth = self.mappoints.get(&new_mp_id)
            .and_then(|mp| {mp.get_norm_and_depth(&self)}).unwrap();
        self.mappoints.get_mut(&new_mp_id)
            .map(|mp| mp.update_norm_and_depth(norm_and_depth));

        return self.last_mp_id;
    }

    pub fn insert_keyframe_to_map(&mut self, prelim_kf: Frame<PrelimKeyFrame>, is_initialization: bool) -> Id {
        // Note: I would really like this to consume the keyframe, but this brings up issues
        // with the map actor being able to take ownership of the keyframe message.
        self.last_kf_id += 1;
        let new_kf_id = self.last_kf_id;

        let full_keyframe = Frame::<FullKeyFrame>::new(prelim_kf, self.id, new_kf_id);
        let num_keypoints = full_keyframe.features.num_keypoints;
        let matches = full_keyframe.mappoint_matches.clone();
        self.keyframes.insert(new_kf_id, full_keyframe);

        if self.keyframes.is_empty() {
            self.initial_kf_id = new_kf_id;
            // self.lowest_kf = kf; // TODO (mvp): ORBSLAM3:Map.cc:67, used to sort mspKeyFrames. I think we can ignore?
        }

        // Update mp connections after insertion
        // Note: In ORBSLAM, this code is copied in a bunch of places. Pulled into here
        // and generalized a bit so we can guarantee it runs every time it needs to.
        // is_initialization check is because initialization does a very similar task
        // but different enough that it's not worth it to generalize it to here.
        if !is_initialization {
            for (index, (mp_id, _is_outlier)) in matches {
                let mp = self.mappoints.get_mut(&mp_id).unwrap();

                // Add observation for mp->kf
                mp.add_observation(&new_kf_id, num_keypoints, index);

                // Add observation kf->mp
                self.keyframes.get_mut(&new_kf_id).map(|kf| {
                    kf.add_mappoint(index, mp_id, false);
                });

                // Update normal and depth
                let norm_and_depth = self.mappoints.get(&mp_id)
                    .and_then(|mp| {mp.get_norm_and_depth(&self)}).unwrap();
                self.mappoints.get_mut(&mp_id)
                    .map(|mp| mp.update_norm_and_depth(norm_and_depth));

                // Compute distinctive descriptors
                let best_descriptor = self.mappoints.get(&mp_id)
                    .and_then(|mp| mp.compute_distinctive_descriptors(&self)).unwrap();
                self.mappoints.get_mut(&mp_id)
                    .map(|mp| mp.update_distinctive_descriptors(best_descriptor));
            }

            // Update Connections
            Map::update_connections(&self.mappoints, &mut self.keyframes, & self.initial_kf_id, &new_kf_id);
        }

        new_kf_id
    }

    pub fn discard_mappoint(&mut self, id: &Id) {
        self.mappoints
            .remove(id)
            .map(|mappoint| {
                let obs = mappoint.get_observations();
                for (kf_id, indexes) in obs {
                    self.keyframes.get_mut(kf_id).unwrap().erase_mappoint_match(*indexes);
                }
            });

        info!("Discard mappoint {}", id);
    }

    pub fn discard_keyframe(&mut self, kf_id: Id) {
        warn!("TODO... not sure this works (discard_keyframe)");
        if kf_id == self.initial_kf_id {
            return;
        }

        let (connections1, matches1, parent1, mut children1);
        {
            let kf = self.get_keyframe(&kf_id).unwrap();
            connections1 = kf.get_covisibility_keyframes(i32::MAX);
            matches1 = kf.mappoint_matches.clone();
            parent1 = kf.full_kf_info.parent;
            children1 = kf.full_kf_info.children.clone();
        }
        for conn_kf in connections1 {
            self.keyframes.get_mut(&conn_kf).unwrap().erase_connection(&kf_id);
        }

        for (_, (mp_id, _)) in matches1 {
            let delete_mappoint = self.mappoints.get_mut(&mp_id).unwrap().erase_observation(&kf_id);
            if delete_mappoint {
                self.discard_mappoint(&mp_id);
            }
        }

        // Update Spanning Tree
        let mut parent_candidates = HashSet::new();
        if parent1.is_some() {
            parent_candidates.insert(parent1.unwrap());
        } 

        // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
        // Include that children as new parent candidate for the rest
        let mut continue_loop = true;
        warn!("TODO... this might be an infinite loop?");

        while !children1.is_empty() {
            let (mut max, mut child_id, mut parent_id) = (-1, -1, -1);

            for child_kf_id in &children1 {
                let child_kf = self.keyframes.get(&child_kf_id).unwrap();
                let connected_kfs = child_kf.get_covisibility_keyframes(i32::MAX);

                // Check if a parent candidate is connected to the keyframe
                for connected_kf_id in &connected_kfs {
                    for parent_candidate_id in &parent_candidates {
                        if connected_kf_id == parent_candidate_id {
                            let weight = child_kf.get_weight(connected_kf_id);
                            if weight > max {
                                child_id = *child_kf_id;
                                parent_id = *connected_kf_id;
                                max = weight;
                                continue_loop = true;
                            }
                        }
                    }
                }
            }

            if continue_loop {
                self.keyframes.get_mut(&child_id).unwrap().change_parent(Some(parent_id));
                parent_candidates.insert(parent_id);
                children1.remove(&child_id);
            } else {
                break;
            }
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        for child_id in &children1 {
            self.keyframes.get_mut(&child_id).unwrap().change_parent(parent1);
        }

        if parent1.is_some() {
            self.keyframes.get_mut(&parent1.unwrap()).unwrap().erase_child(kf_id);
        }
        self.keyframes.remove(&kf_id);

        info!("Discard keyframe {}", kf_id);
    }

    pub fn create_initial_map_monocular(
        &mut self, mp_matches: Vec<i32>, p3d: DVVectorOfPoint3f, initial_frame: Frame<InitialFrame>, current_frame: Frame<InitialFrame>
    ) -> Option<(DVPose, i32, i32, HashSet<Id>, Timestamp)> {
        // Testing local mapping ... global bundle adjustment gives slightly different pose for second keyframe.
        // TODO (design) - we have to do some pretty gross things with calling functions in this section
        // so that we can have multiple references to parts of the map. This should get cleaned up, but I'm not sure how.
        // TODO (design) - can this function be moved into map_initialization.rs ? Would require a bit of back-and-forth
        // between creating map objects and inserting them
        
        match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => todo!("IMU"), // pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);
            _ => {}
        }

        if self.last_kf_id == 0 {
            self.initial_kf_id = self.last_kf_id + 1;
        }

        let num_keypoints = initial_frame.features.num_keypoints;

        // Create KeyFrames
        let initial_kf_id = self.insert_keyframe_to_map(
            Frame::<PrelimKeyFrame>::new(initial_frame),
            true
        );
        let curr_kf_id = self.insert_keyframe_to_map(
            Frame::<PrelimKeyFrame>::new(current_frame),
            true
        );

        let mut count = 0;
        for kf1_index in 0..mp_matches.len() {
            // Get index for kf1 and kf2
            let kf2_index = mp_matches[kf1_index];
            if kf2_index == -1 {
                continue;
            }
            count +=1;

            // Create mappoint
            let point = p3d.get(kf1_index).unwrap();
            let world_pos = DVVector3::new_with(point.x as f64, point.y as f64, point.z as f64);
            let id = self.insert_mappoint_to_map(
                MapPoint::<PrelimMapPoint>::new(world_pos, curr_kf_id, self.id),
                vec![(initial_kf_id, num_keypoints, kf1_index), (curr_kf_id, num_keypoints, kf2_index as usize)]
            );
        }
        info!("Monocular initialization created new map with {} mappoints", count);
        
        // Update Connections
        Map::update_connections(& self.mappoints, &mut self.keyframes, & self.initial_kf_id, &initial_kf_id);
        Map::update_connections(& self.mappoints, &mut self.keyframes, & self.initial_kf_id, &curr_kf_id);

        // self.print_current_map("after update connections".to_string());

        // Bundle Adjustment
        let optimized_poses = optimizer::global_bundle_adjustment(self, 20);
        self.update_after_ba(optimized_poses, 0);

        // self.print_current_map("after BA".to_string());

        let median_depth = self.keyframes.get_mut(&initial_kf_id)?.compute_scene_median_depth(& self.mappoints, 2);
        let inverse_median_depth = match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => 4.0 / median_depth,
            _ => 1.0 / median_depth
        };

        if median_depth < 0.0 || self.keyframes.get(&curr_kf_id)?.tracked_mappoints(&self, 1) < 50 {
            // reset active map
            warn!("map::create_initial_map_monocular;wrong initialization");
            return None;
        }

        // Scale initial baseline
        {
            let curr_kf = self.keyframes.get_mut(&curr_kf_id)?;
            let new_trans = *(curr_kf.pose?.get_translation()) * inverse_median_depth;
            curr_kf.pose.as_mut().unwrap().set_translation(new_trans);
        }

        // Scale points
        for (_, (mp_id, _)) in &self.keyframes.get(&initial_kf_id)?.mappoint_matches {
            let mp = self.mappoints.get_mut(&mp_id)?;
            mp.position = DVVector3::new((*mp.position) * inverse_median_depth);

            let norm_and_depth = self.mappoints.get(&mp_id)
                .and_then(|mp| {mp.get_norm_and_depth(&self)})?;
            self.mappoints.get_mut(&mp_id)
                .map(|mp| mp.update_norm_and_depth(norm_and_depth));
        }

        match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => {
                todo!("IMU");
            //     pKFcur->mPrevKF = pKFini;
            //     pKFini->mNextKF = pKFcur;
            //     pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            //     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
            },
            _ => {}
        };

        // TODO (mvp): commented this out because I don't think they ever use it??
        // Compute here initial velocity
        // let delta_t = self.keyframes.get(&self.last_kf_id).unwrap().pose * self.keyframes.get(&1).unwrap().pose.inverse();
        // let velocity = false;
        // Eigen::Vector3f phi = deltaT.so3().log(); need to convert to rust
        // let initial_frame_ts = inidata.initial_frame.as_ref().unwrap().timestamp;
        // let curr_frame_ts = inidata.current_frame.as_ref().unwrap().timestamp;
        // let last_frame_ts = inidata.last_frame.as_ref().unwrap().timestamp;
        // let aux = (curr_frame_ts - last_frame_ts).to_std().unwrap().as_secs() / (curr_frame_ts - initial_frame_ts).to_std().unwrap().as_secs();
        // phi *= aux;

        // TODO (multimaps)
        // mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini)

        let curr_kf_pose = self.keyframes.get_mut(&curr_kf_id)?.pose?;
        let curr_kf_timestamp = self.keyframes.get(&curr_kf_id)?.timestamp;

        // self.print_current_map("initialization done".to_string());

        // Update tracking with new info
        let relevant_mappoints = self.mappoints.keys().cloned().collect();
        Some((curr_kf_pose, curr_kf_id, initial_kf_id, relevant_mappoints, curr_kf_timestamp))
    }


    pub fn update_connections(mappoints: &HashMap<Id, MapPoint<FullMapPoint>>, keyframes: &mut HashMap<Id, Frame<FullKeyFrame>>, initial_kf_id: &i32, main_kf_id: &i32) {
        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        let mut kf_counter = HashMap::<Id, i32>::new();
        for (_, (mp_id, _)) in &keyframes.get(main_kf_id).unwrap().mappoint_matches {
            let mp = mappoints.get(&mp_id).unwrap();
            for kf_id in mp.get_observations().keys() {
                if *kf_id != *main_kf_id {
                    *kf_counter.entry(*kf_id).or_insert(0) += 1;
                }
            }
        }

        if kf_counter.is_empty() {
            error!("map::update_connections;kf counter is empty");
            return;
        }

        let (&kf_max, &count_max) = kf_counter.iter().max_by_key(|entry | entry.1).unwrap();
        kf_counter.retain( |&_, count| count > &mut 15 ); //If the counter is greater than threshold add connection

        //In case no keyframe counter is over threshold add the one with maximum counter
        if kf_counter.is_empty() {
            kf_counter.insert(kf_max, count_max);
        }

        for (kf_id, weight) in &kf_counter {
            keyframes.get_mut(&kf_id).unwrap().add_connection(main_kf_id, *weight);
        }
        let parent_kf_id = keyframes.get_mut(main_kf_id).unwrap().insert_all_connections(kf_counter, main_kf_id == initial_kf_id);
        if parent_kf_id.is_some() {
            parent_kf_id.map(|parent_kf| {
                keyframes.get_mut(&parent_kf).unwrap().add_child(*main_kf_id);
            });
        }
    }

    pub fn update_after_ba(&mut self, optimized_poses: optimizer::BundleAdjustmentResult, loop_kf: Id) {
        for (kf_id, pose) in optimized_poses.new_kf_poses {
            if loop_kf == self.initial_kf_id {
                self.keyframes.get_mut(&kf_id).unwrap().pose = Some(pose);
            } else {
                todo!("MVP");
                // Code from ORBSLAM, not sure if we need it
                // pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(),SE3quat.translation()).cast<float>();
                // pKF->mnBAGlobalForKF = nLoopKF;
            }
        }

        for (mp_id, position) in optimized_poses.new_mp_poses {
            if loop_kf == self.initial_kf_id {
                self.mappoints.get_mut(&mp_id).unwrap().position = position;
                let norm_and_depth = self.mappoints.get(&mp_id).unwrap().get_norm_and_depth(&self);
                if norm_and_depth.is_some() {
                    self.mappoints.get_mut(&mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
                }
            } else {
                todo!("MVP");
                // pMP->mPosGBA = vPoint->estimate().cast<float>();
                // pMP->mnBAGlobalForKF = nLoopKF;
            }
        }

    }

    pub fn replace_mappoint(&self, _mp_to_replace: Id, _mp: Id) {
        todo!("LOCAL MAPPING");
        // if(pMP->mnId==this->mnId)
        //     return;

        // int nvisible, nfound;
        // map<KeyFrame*,tuple<int,int>> obs;
        // {
        //     unique_lock<mutex> lock1(mMutexFeatures);
        //     unique_lock<mutex> lock2(mMutexPos);
        //     obs=mObservations;
        //     mObservations.clear();
        //     mbBad=true;
        //     nvisible = mnVisible;
        //     nfound = mnFound;
        //     mpReplaced = pMP;
        // }

        // for(map<KeyFrame*,tuple<int,int>>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
        // {
        //     // Replace measurement in keyframe
        //     KeyFrame* pKF = mit->first;

        //     tuple<int,int> indexes = mit -> second;
        //     int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        //     if(!pMP->IsInKeyFrame(pKF))
        //     {
        //         if(leftIndex != -1){
        //             pKF->ReplaceMapPointMatch(leftIndex, pMP);
        //             pMP->AddObservation(pKF,leftIndex);
        //         }
        //         if(rightIndex != -1){
        //             pKF->ReplaceMapPointMatch(rightIndex, pMP);
        //             pMP->AddObservation(pKF,rightIndex);
        //         }
        //     }
        //     else
        //     {
        //         if(leftIndex != -1){
        //             pKF->EraseMapPointMatch(leftIndex);
        //         }
        //         if(rightIndex != -1){
        //             pKF->EraseMapPointMatch(rightIndex);
        //         }
        //     }
        // }
        // pMP->IncreaseFound(nfound);
        // pMP->IncreaseVisible(nvisible);
        // pMP->ComputeDistinctiveDescriptors();

        // mpMap->EraseMapPoint(this);


        // Update points
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
        // Update connections in covisibility graph
        // mpCurrentKeyFrame->UpdateConnections();
    }
}
