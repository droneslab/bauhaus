use std::{backtrace::Backtrace, collections::{BTreeSet, HashMap, HashSet}};
use log::{info, warn, error, debug};
use rustc_hash::FxHashMap;
use core::{config::{SETTINGS, SYSTEM}, matrix::{DVVector3, DVVectorOfPoint3f}, sensor::{FrameSensor, ImuSensor, Sensor}, system::Timestamp};
use crate::{
    map::{keyframe::*, mappoint::*, pose::Pose},
    modules::optimizer::{self}
};

use super::{frame::Frame};

pub type Id = i32;
// pub type MapItems<T> = HashMap<Id, T>;
// pub type MapItems<T> = HashMap<Id, T, BuildHasherDefault<SeaHasher>>; // faster performance with seahasher
pub type MapItems<T> = FxHashMap<Id, T>;


#[derive(Debug, Clone)]
pub struct Map {
    pub id: Id,

    pub keyframes: MapItems<KeyFrame>, // = mspKeyFrames
    pub mappoints: MapItems<MapPoint>, // = mspMapPoints

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
            keyframes: MapItems::<KeyFrame>::default(),
            mappoints: MapItems::<MapPoint>::default(),
        }
    }

    pub fn print_current_map(&self, identifier: String) {
        // For debugging
        debug!("PRINT MAP START;{}", identifier);
        for (id, kf) in &self.keyframes {
            print!("PRINT MAP KF;{};{:?}", id, kf.pose);
            print!(";");
            for connection in &kf.get_covisibility_keyframes(i32::MAX) {
                print!("{},", connection);
            };
            print!(";");
            for mp_match in kf.get_mp_matches() {
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
    pub fn insert_mappoint_to_map(&mut self, position: DVVector3<f64>, ref_kf_id: Id, origin_map_id: Id, observations_to_add: Vec<(Id, u32, usize)>) -> Id {
        self.last_mp_id += 1;
        let new_mp_id = self.last_mp_id;

        let full_mappoint = MapPoint::new(position, ref_kf_id, origin_map_id, self.last_mp_id);
        self.mappoints.insert(self.last_mp_id, full_mappoint);

        for (kf_id, _num_keypoints, index) in observations_to_add {
            if self.keyframes.get(&kf_id).is_none() {
                // Stale reference to deleted keyframe
                continue;
            }

            self.add_observation(kf_id, new_mp_id, index as u32, false);
        }

        self.update_mappoint(new_mp_id);

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
            let mp_matches = self.keyframes.get(&new_kf_id).unwrap().get_mp_matches().clone();
            // trace!("Create kf {} mp matches: {:?}", new_kf_id, mp_matches);

            for i in 0..mp_matches.len() {
                if mp_matches[i].is_some() {
                    let mp_id = self.keyframes.get(&new_kf_id).unwrap().get_mp_match(&(i as u32));
                    // Add observation for mp->kf
                    if let Some(mp) = self.mappoints.get_mut(&mp_id) {
                        mp.add_observation(&new_kf_id, num_keypoints, i as u32);
                        // trace!("Add observation for mp->kf: {} {} {}", mp_id, new_kf_id, i);
                        // self.add_observation(new_kf_id, mp_id, i as u32, false);
                    } else {
                        self.keyframes.get_mut(&new_kf_id).unwrap().mappoint_matches.delete_at_indices((i as i32, -1));
                        // trace!("Delete observation for kf->mp: {} {} {}", new_kf_id, mp_id, i);
                        continue;
                        // Mappoint was deleted by local mapping but not deleted here yet 
                        // because the kf was not in the map at the time.
                    }

                    self.update_mappoint(mp_id);
                }
            }

            // Update Connections
            self.update_connections(new_kf_id);
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
                    self.keyframes.get_mut(&kf_id).unwrap().mappoint_matches.delete_at_indices(*indexes);

                    // trace!("Delete observation for kf->mp: {} {} {}", kf_id, id, indexes.0);
                }
            });
    }

    pub fn discard_keyframe(&mut self, kf_id: Id) {
        let _span = tracy_client::span!("discard_keyframe");
        if kf_id == self.initial_kf_id {
            return;
        }

        info!("Discard keyframe {}", kf_id);

        let (connections1, matches1, parent1, mut children1);
        {
            // TODO (timing) ... map mutability
            // same as other issue in map.rs, to remove this clone we need to be able to iterate over an immutable ref to matches and children while mutating other keyframes inside the loop
            let kf = self.keyframes.get(&kf_id).unwrap();
            if kf.dont_delete {
                debug!("Loop closing working on keyframe, don't delete.");
                return;
            }
            connections1 = kf.get_covisibility_keyframes(i32::MAX);
            matches1 = kf.get_mp_matches().clone();
            parent1 = kf.parent;
            children1 = kf.children.clone();
        }
        for conn_kf in connections1 {
            self.keyframes.get_mut(&conn_kf).unwrap().delete_connection(kf_id);
        }

        for mp_match in &matches1 {
            if let Some((mp_id, _)) = mp_match {
                let should_delete_mappoint = self.mappoints.get_mut(&mp_id).unwrap().delete_observation(&kf_id);
                if should_delete_mappoint {
                    self.discard_mappoint(&mp_id);
                    // debug!("Discarded mappoint: {}", mp_id);
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
                let connected_kfs = child_kf.get_covisibility_keyframes(i32::MAX);

                // Check if a parent candidate is connected to the keyframe
                for connected_kf_id in &connected_kfs {
                    for parent_candidate_id in &parent_candidates {
                        if connected_kf_id == parent_candidate_id {
                            let weight = child_kf.get_connected_kf_weight(*connected_kf_id);
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
                if child_id != -1 {
                    self.keyframes.get_mut(&child_id).expect(&format!("Could not get child id {} from parent {}", child_id, kf_id)).parent = Some(parent_id);
                    parent_candidates.insert(parent_id);
                    children1.remove(&child_id);
                }
            } else {
                break;
            }
        }

        // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
        for child_id in children1 {
            self.keyframes.get_mut(&child_id).unwrap().parent = parent1;
        }

        if parent1.is_some() {
            self.keyframes.get_mut(&parent1.unwrap()).unwrap().children.remove(&kf_id);
        }
        self.keyframes.remove(&kf_id);
    }

    pub fn create_initial_map_monocular(
        &mut self, mp_matches: Vec<i32>, p3d: DVVectorOfPoint3f, initial_frame: Frame, current_frame: Frame
    ) -> Option<(Pose, i32, i32, BTreeSet<Id>, Timestamp)> {
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
            // This also adds the observations from mappoint -> keyframe and keyframe -> mappoint
            let point = p3d.get(kf1_index).unwrap();
            let world_pos = DVVector3::new_with(point.x as f64, point.y as f64, point.z as f64);
            let _id = self.insert_mappoint_to_map(
                world_pos, 
                curr_kf_id, 
                self.id,
                vec![(initial_kf_id, num_keypoints, kf1_index), (curr_kf_id, num_keypoints, kf2_index as usize)]
            );
        }
        info!("Monocular initialization created new map with {} mappoints", count);

        // Update Connections
        self.update_connections(initial_kf_id);
        self.update_connections(curr_kf_id);

        // Bundle Adjustment
        let optimized_poses = optimizer::global_bundle_adjustment(self, 20);
        self.update_after_ba(optimized_poses, 0);

        let median_depth = self.keyframes.get_mut(&initial_kf_id)?.compute_scene_median_depth(& self.mappoints, 2);
        let inverse_median_depth = match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => 4.0 / median_depth,
            _ => 1.0 / median_depth
        };

        if median_depth < 0.0 || self.keyframes.get(&curr_kf_id)?.get_tracked_mappoints(&self, 1) < 50 {
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
        for item in self.keyframes.get(&initial_kf_id)?.get_mp_matches() {
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

        // for (id, mp) in &self.mappoints {
        //     println!("Mappoint {} pose {:?}", id, mp.position);
        // }

        Some((curr_kf_pose, curr_kf_id, initial_kf_id, relevant_mappoints, curr_kf_timestamp))
    }

    pub fn update_connections(&mut self, main_kf_id: Id) {
        let _span = tracy_client::span!("update_connections");
        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        let mut kf_counter = HashMap::<Id, i32>::new();
        let kf = self.keyframes.get_mut(&main_kf_id).unwrap();
        for item in kf.get_mp_matches() {
            if let Some((mp_id, _)) = item {
                if let Some(mp) = self.mappoints.get(&mp_id) {
                    for kf_id in mp.get_observations().keys() {
                        if *kf_id != main_kf_id {
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
            self.keyframes.get_mut(&kf_id).unwrap().add_connection(main_kf_id, *weight);
            self.keyframes.get_mut(&main_kf_id).unwrap().add_connection(*kf_id, *weight);
        }
        let parent_kf_id = self.keyframes.get_mut(&main_kf_id).unwrap().add_all_connections(kf_counter, main_kf_id == self.initial_kf_id);
        if parent_kf_id.is_some() {
            parent_kf_id.map(|parent_kf| {
                self.keyframes.get_mut(&parent_kf).unwrap().children.insert(main_kf_id);
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

    pub fn add_observation(&mut self, kf_id: Id, mp_id: Id, index: u32, is_outlier: bool) {
        let num_keypoints = self.keyframes.get(&kf_id).expect(&format!("Could not get kf {}", kf_id)).features.num_keypoints;
        self.mappoints.get_mut(&mp_id).unwrap().add_observation(&kf_id, num_keypoints, index);

        let old_mp_match = self.keyframes.get_mut(&kf_id).unwrap().mappoint_matches.add(index, mp_id, is_outlier);

        if let Some(old_mp_id) = old_mp_match {
            warn!("There should not be an old mp match, kf {}, new mp {}, old mp {}", kf_id, mp_id, old_mp_id);
            println!("{:?}", Backtrace::force_capture());

        }
        // trace!("Add observation mp<->kf: {} {}", mp_id, kf_id);
    }

    pub fn delete_observation(&mut self, kf_id: Id, mp_id: Id) {
        self.keyframes.get_mut(&kf_id).unwrap().mappoint_matches.delete_with_id(mp_id);
        self.mappoints.get_mut(&mp_id).unwrap().delete_observation(&kf_id);
        // trace!("Delete observation mp<->kf: {} {}", mp_id, kf_id);
    }

    pub fn replace_mappoint(&mut self, mp_to_replace_id: Id, mp_id: Id) {
        // void MapPoint::Replace(MapPoint* pMP)

        if mp_to_replace_id == mp_id {
            return;
        }

        let (num_found, num_visible, observations);
        {
            let mp_to_replace = self.mappoints.get(&mp_to_replace_id).unwrap();
            num_found = mp_to_replace.get_found();
            num_visible = mp_to_replace.get_visible();
            observations = mp_to_replace.get_observations().clone();
        }

        // This function has to occur before for loop below because the function deletes the kf's observation
        // at the indexes... which we don't want if we already replaced it with the correct new observation.
        // There's probably some way to clean up this code so it isn't so fragile like this but it's not a priority.
        self.discard_mappoint(&mp_to_replace_id);

        for (kf_id, (index_left, index_right)) in observations {
            // Replace measurement in keyframe
            let kf = self.keyframes.get_mut(&kf_id).unwrap();
            let mp = self.mappoints.get(&mp_id).unwrap();

            if !mp.get_observations().contains_key(&kf_id) {
                if index_left != -1 {
                    let num_keypoints = self.keyframes.get(&kf_id).expect(&format!("Could not get kf {}", kf_id)).features.num_keypoints;
                    self.mappoints.get_mut(&mp_id).unwrap().add_observation(&kf_id, num_keypoints, index_left as u32);
                    let old_mp_match = self.keyframes.get_mut(&kf_id).unwrap().mappoint_matches.add(index_left as u32, mp_id, false);
                    if let Some(old_mp_id) = old_mp_match {
                        if old_mp_id != mp_to_replace_id {
                            error!("KF ({})'s old MP match ({}) should be the same as the one that is replaced ({}).", kf_id, old_mp_id, mp_to_replace_id);
                        }
                    }
                }
                if index_right != -1 {
                    // TODO STEREO
                }
            } else {
                kf.mappoint_matches.delete_at_indices((index_left, index_right));
            }
        }

        let mp = self.mappoints.get(&mp_id).unwrap();
        mp.increase_found(num_found);
        mp.increase_visible(num_visible);

        let best_descriptor = self.mappoints.get(&mp_id)
            .and_then(|mp| mp.compute_distinctive_descriptors(&self)).unwrap();
        self.mappoints.get_mut(&mp_id)
            .map(|mp| mp.update_distinctive_descriptors(best_descriptor));
    }


    pub fn update_mappoint(&mut self, mp_id: Id) {
        // These two functions are called together all the time, and our implementation is more complicated
        // because we need to split up the computing (read lock) from the updating (write lock). The whole
        // sequence is put together here so each time you need to do it you can just call this function.

        // Compute distinctive descriptors
        let best_descriptor = self.mappoints.get(&mp_id)
            .and_then(|mp| mp.compute_distinctive_descriptors(&self)).unwrap();
        self.mappoints.get_mut(&mp_id)
            .map(|mp| mp.update_distinctive_descriptors(best_descriptor));

        // Update normal and depth
        let norm_and_depth = self.mappoints.get(&mp_id)
            .and_then(|mp| {mp.get_norm_and_depth(&self)}).unwrap();
        self.mappoints.get_mut(&mp_id)
            .map(|mp| mp.update_norm_and_depth(norm_and_depth));
    }

}
