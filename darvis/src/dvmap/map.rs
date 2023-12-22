use std::collections::{HashMap, HashSet};
use log::{info, warn, error, debug};
use dvcore::{matrix::{DVVector3, DVVectorOfPoint3f}, config::{SETTINGS, SYSTEM}, sensor::{Sensor, FrameSensor, ImuSensor}, maplock::ReadWriteMap};
use logging_timer::time;
use crate::{
    dvmap::{keyframe::*, mappoint::*, pose::DVPose},
    modules::optimizer::{self}
};

use super::misc::Timestamp;

pub type Id = i32;
pub type MapItemHashMap<T> = HashMap<Id, T, nohash_hasher::BuildNoHashHasher<Id>>; // Faster performance

#[derive(Debug, Clone)]
pub struct Map {
    pub id: Id,

    pub keyframes: MapItemHashMap<KeyFrame>, // = mspKeyFrames
    pub mappoints: MapItemHashMap<MapPoint<FullMapPoint>>, // = mspMapPoints

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
            initial_kf_id: -1,
            keyframes: MapItemHashMap::default(),
            mappoints: MapItemHashMap::default(),
        }
    }

    pub fn print_current_map(&self, identifier: String) {
        // For debugging
        debug!("PRINT MAP START;{}", identifier);
        for (id, kf) in &self.keyframes {
            print!("PRINT MAP KF;{};{:?}", id, kf.pose);
            print!(";");
            for connection in &kf.connections.get_covisibility_keyframes(i32::MAX) {
                print!("{},", connection);
            };
            print!(";");
            for mp_match in &kf.mappoint_matches.matches {
                if let Some((_, mp_id)) = mp_match {
                    print!("{},", mp_id);
                }
            };
            debug!("");
        }
        for (id, mp) in &self.mappoints {
            print!("PRINT MAP MP;{};{:?}", id, mp.position);
            print!(";");
            for (kf_id, _indexes) in mp.get_observations() {
                print!("{},", kf_id);
            };
            debug!("");
        }
        debug!("PRINT MAP DONE");
    }

    ////* &mut self ... behind map actor */////////////////////////////////////////////////////
    pub fn insert_mappoint_to_map(&mut self, mp: MapPoint<PrelimMapPoint>, observations_to_add: Vec<(Id, u32, usize)>) -> Id {
        self.last_mp_id += 1;
        let new_mp_id = self.last_mp_id;

        let full_mappoint = MapPoint::<FullMapPoint>::new(mp, self.last_mp_id);
        self.mappoints.insert(self.last_mp_id, full_mappoint);

        let mp = self.mappoints.get_mut(&new_mp_id).unwrap();
        for (kf_id, num_keypoints, index) in observations_to_add {
            if self.keyframes.get(&kf_id).is_none() {
                // Stale reference to deleted keyframe
                continue;
            }

            // Add observation for mp->kf
            mp.add_observation(&kf_id, num_keypoints, index as u32);

            // Add observation kf->mp
            self.keyframes.get_mut(&kf_id).map(|kf| {
                kf.mappoint_matches.add_mappoint(index as u32, mp.get_id(), false);
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

    pub fn insert_keyframe_to_map(&mut self, frame: Frame, is_initialization: bool) -> Id {
        let _span = tracy_client::span!("insert_keyframe");
        self.last_kf_id += 1;
        let new_kf_id = self.last_kf_id;
        if self.keyframes.is_empty() {
            self.initial_kf_id = new_kf_id;
            // self.lowest_kf = kf; // TODO (mvp): ORBSLAM3:Map.cc:67, used to sort mspKeyFrames. I think we can ignore?
        }

        let full_keyframe = KeyFrame::new(frame, self.id, new_kf_id);
        let num_keypoints = full_keyframe.features.num_keypoints;
        self.keyframes.insert(new_kf_id, full_keyframe);

        // Update mp connections after insertion
        // Note: This should run during most normal keyframe insertions, but not 
        // when inserting the first two keyframes in initialization, because this
        // already happens when inserting the mappoints in initialization.
        if !is_initialization {
            // TODO (timing) ... really annoying to have this clone but it's the only way we can
            // have a get_mut on keyframes as well as a get on mappoints below. Another option is to
            // split up the for loop into 2, one for the keyframe update and one for the mappoint update.
            // Possible that that is faster than cloning.
            let mp_matches = self.keyframes.get(&new_kf_id).unwrap().mappoint_matches.clone();

            for i in 0..mp_matches.matches.len() {
                if mp_matches.has_mappoint(&(i as u32)) {
                    let mp_id = mp_matches.get_mappoint(&(i as u32));
                    // Add observation for mp->kf
                    if let Some(mp) = self.mappoints.get_mut(&mp_id) {
                        mp.add_observation(&new_kf_id, num_keypoints, i as u32);
                    } else {
                        self.keyframes.get_mut(&new_kf_id).unwrap().mappoint_matches.delete((i as i32, -1));
                        continue;
                        // Mappoint was deleted by local mapping but not deleted here yet 
                        // because the kf was not in the map at the time.
                    }

                    let norm_and_depth;
                    let best_descriptor;
                    let mp_id = mp_matches.get_mappoint(&(i as u32));
                    {
                        let mp = self.mappoints.get(&mp_id).unwrap();
                        norm_and_depth = mp.get_norm_and_depth(&self).unwrap();
                        best_descriptor = mp.compute_distinctive_descriptors(&self).unwrap();
                    }

                    {
                        let mp = self.mappoints.get_mut(&mp_id).unwrap();
                        mp.update_norm_and_depth(norm_and_depth);
                        mp.update_distinctive_descriptors(best_descriptor);
                    }
                }
            }

            // Update Connections
            Map::update_connections(&self.mappoints, &mut self.keyframes, & self.initial_kf_id, &new_kf_id);
        }
        info!("Created keyframe {}", new_kf_id);

        new_kf_id
    }

    pub fn discard_mappoint(&mut self, id: &Id) {
        self.mappoints
            .remove(id)
            .map(|mappoint| {
                let obs = mappoint.get_observations();
                for (kf_id, indexes) in obs {
                    self.keyframes.get_mut(kf_id).unwrap().mappoint_matches.delete(*indexes);
                }
            });
    }

    pub fn discard_keyframe(&mut self, kf_id: Id) {
        let _span = tracy_client::span!("discard_keyframe");
        if kf_id == self.initial_kf_id {
            return;
        }

        let (connections1, matches1, parent1, mut children1);
        {
            // TODO (timing) ... map mutability
            // same as other issue in map.rs, to remove this clone we need to be able to iterate over an immutable ref to matches and children while mutating other keyframes inside the loop
            let kf = self.keyframes.get(&kf_id).unwrap();
            connections1 = kf.connections.get_covisibility_keyframes(i32::MAX);
            matches1 = kf.mappoint_matches.matches.clone();
            parent1 = kf.connections.parent;
            children1 = kf.connections.children.clone();
        }
        for conn_kf in connections1 {
            self.keyframes.get_mut(&conn_kf).unwrap().connections.delete(&kf_id);
        }

        debug!("Discarding keyframe {} with matches: {:?}", kf_id, matches1);

        for mp_match in &matches1 {
            if let Some((mp_id, _)) = mp_match {
                let should_delete_mappoint = self.mappoints.get_mut(&mp_id).unwrap().delete_observation(&kf_id);
                if should_delete_mappoint {
                    self.discard_mappoint(&mp_id);
                }
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

        while !children1.is_empty() {
            let (mut max, mut child_id, mut parent_id) = (-1, -1, -1);

            for child_kf_id in &children1 {
                let child_kf = self.keyframes.get(&child_kf_id).unwrap();
                let connected_kfs = child_kf.connections.get_covisibility_keyframes(i32::MAX);

                // Check if a parent candidate is connected to the keyframe
                for connected_kf_id in &connected_kfs {
                    for parent_candidate_id in &parent_candidates {
                        if connected_kf_id == parent_candidate_id {
                            let weight = child_kf.connections.get_weight(connected_kf_id);
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
                self.keyframes.get_mut(&child_id).unwrap().connections.change_parent(Some(parent_id));
                parent_candidates.insert(parent_id);
                children1.remove(&child_id);
            } else {
                break;
            }
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        for child_id in &children1 {
            self.keyframes.get_mut(&child_id).unwrap().connections.change_parent(parent1);
        }

        if parent1.is_some() {
            self.keyframes.get_mut(&parent1.unwrap()).unwrap().connections.delete_child(kf_id);
        }
        self.keyframes.remove(&kf_id);

        info!("Discard keyframe {}", kf_id);
    }

    pub fn create_initial_map_monocular(
        &mut self, mp_matches: Vec<i32>, p3d: DVVectorOfPoint3f, initial_frame: Frame, current_frame: Frame
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
            initial_frame,
            true
        );
        let curr_kf_id = self.insert_keyframe_to_map(
            current_frame,
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

        // Bundle Adjustment
        let optimized_poses = optimizer::global_bundle_adjustment(self, 20);
        self.update_after_ba(optimized_poses, 0);

        let median_depth = self.keyframes.get_mut(&initial_kf_id)?.compute_scene_median_depth(& self.mappoints, 2);
        let inverse_median_depth = match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => 4.0 / median_depth,
            _ => 1.0 / median_depth
        };

        if median_depth < 0.0 || self.keyframes.get(&curr_kf_id)?.mappoint_matches.tracked_mappoints(&self, 1) < 50 {
            // reset active map
            warn!("map::create_initial_map_monocular;wrong initialization");
            return None;
        }

        // Scale initial baseline
        {
            let curr_kf = self.keyframes.get_mut(&curr_kf_id)?;
            let new_trans = *(curr_kf.pose.get_translation()) * inverse_median_depth;
            curr_kf.pose.set_translation(new_trans);
        }

        // Scale points
        for item in &self.keyframes.get(&initial_kf_id)?.mappoint_matches.matches {
            if let Some((mp_id, _)) = item {
                let mp = self.mappoints.get_mut(&mp_id)?;
                mp.position = DVVector3::new((*mp.position) * inverse_median_depth);

                let norm_and_depth = self.mappoints.get(&mp_id)
                    .and_then(|mp| {mp.get_norm_and_depth(&self)})?;
                self.mappoints.get_mut(&mp_id)
                    .map(|mp| mp.update_norm_and_depth(norm_and_depth));
            }
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

        let curr_kf_pose = self.keyframes.get_mut(&curr_kf_id)?.pose;
        let curr_kf_timestamp = self.keyframes.get(&curr_kf_id)?.timestamp;

        // self.print_current_map("initialization done".to_string());

        // Update tracking with new info
        let relevant_mappoints = self.mappoints.keys().cloned().collect();
        Some((curr_kf_pose, curr_kf_id, initial_kf_id, relevant_mappoints, curr_kf_timestamp))
    }

    pub fn update_connections(mappoints: &MapItemHashMap<MapPoint<FullMapPoint>>, keyframes: &mut MapItemHashMap<KeyFrame>, initial_kf_id: &i32, main_kf_id: &i32) {
        let _span = tracy_client::span!("update_connections");
        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        let mut kf_counter = HashMap::<Id, i32>::new();
        let kf = keyframes.get_mut(main_kf_id).unwrap();
        for item in kf.mappoint_matches.matches.iter() {
            if let Some((mp_id, _)) = item {
                if let Some(mp) = mappoints.get(&mp_id) {
                    for kf_id in mp.get_observations().keys() {
                        if *kf_id != *main_kf_id {
                            *kf_counter.entry(*kf_id).or_insert(0) += 1;
                        }
                    }
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
            keyframes.get_mut(&kf_id).unwrap().connections.add(main_kf_id, *weight);
        }
        let parent_kf_id = keyframes.get_mut(main_kf_id).unwrap().connections.add_all(kf_counter, main_kf_id == initial_kf_id);
        if parent_kf_id.is_some() {
            parent_kf_id.map(|parent_kf| {
                keyframes.get_mut(&parent_kf).unwrap().connections.add_child(*main_kf_id);
            });
        }
    }

    pub fn update_after_ba(&mut self, optimized_poses: optimizer::BundleAdjustmentResult, loop_kf: Id) {
        for (kf_id, pose) in optimized_poses.new_kf_poses {
            if let Some(kf) = self.keyframes.get_mut(&kf_id) {
                if loop_kf == self.initial_kf_id {
                    kf.pose = pose;
                } else {
                    todo!("MVP LOOP CLOSING");
                    // Code from ORBSLAM, not sure if we need it
                    // pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(),SE3quat.translation()).cast<float>();
                    // pKF->mnBAGlobalForKF = nLoopKF;
                }
            } else {
                // Possible that map actor deleted mappoint after local BA has finished but before
                // this message is processed
                continue;
            }
        }

        for (mp_id, position) in optimized_poses.new_mp_poses {
            if loop_kf == self.initial_kf_id {
                match self.mappoints.get_mut(&mp_id) {
                    // Possible that map actor deleted mappoint after local BA has finished but before
                    // this message is processed
                    Some(mp) => {
                        mp.position = position;
                        let norm_and_depth = self.mappoints.get(&mp_id).unwrap().get_norm_and_depth(&self);
                        if norm_and_depth.is_some() {
                            self.mappoints.get_mut(&mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
                        }
                    },
                    None => continue,
                };
            } else {
                todo!("MVP LOOP CLOSING");
                // pMP->mPosGBA = vPoint->estimate().cast<float>();
                // pMP->mnBAGlobalForKF = nLoopKF;
            }
        }

    }

    pub fn replace_mappoint(&self, _mp_to_replace: Id, _mp: Id) {
        todo!("MVP LOCAL MAPPING");
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
        //             pKF->deleteMapPointMatch(leftIndex);
        //         }
        //         if(rightIndex != -1){
        //             pKF->deleteMapPointMatch(rightIndex);
        //         }
        //     }
        // }
        // pMP->IncreaseFound(nfound);
        // pMP->IncreaseVisible(nvisible);
        // pMP->ComputeDistinctiveDescriptors();

        // mpMap->deleteMapPoint(this);


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
