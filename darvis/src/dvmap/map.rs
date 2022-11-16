use std::collections::{HashMap, HashSet};
use log::{info, warn, error};

use dvcore::{matrix::{DVVector3}, config::{Sensor, GLOBAL_PARAMS, SYSTEM_SETTINGS, FrameSensor, ImuSensor}};
use crate::{
    dvmap::{keyframe::*, mappoint::*, pose::Pose, bow::DVVocabulary},
    modules::{map_initialization::Initialization, optimizer::{BAResult, Optimizer}}
};

use super::bow;

pub type Id = i32;

#[derive(Debug, Clone, Default)]
pub struct Map {
    pub id: Id,

    // IMU
    pub imu_initialized: bool, // isImuInitialized(), set true by local mapper
    pub imu_ba2: bool, // mbIMU_BA2

    // KeyFrames
    keyframes: HashMap<Id, KeyFrame<FullKeyFrame>>, // = mspKeyFrames
    last_kf_id: Id, // = mnMaxKFid
    pub initial_kf_id: Id, // TODO (multimaps): this is the initial kf id that was added to this map, when this map is made. should tie together map vesioning better, maybe in a single struct

    // MapPoints
    mappoints: HashMap<Id, MapPoint<FullMapPoint>>, // = mspMapPoints
    last_mp_id: Id,

    // Utilities from darvis::utils
    // Sofiya: as far as I can tell, optimizer is just needed here because of global optimization, which 
    // no other thread/module/actor will use except the map. Should there even be a link to Optimizer here,
    // or should we make a specific global optimization object?
    optimizer: Optimizer,

    sensor: Sensor,
    // Sofiya: following are in orbslam3, not sure if we need:
    // mvpKeyFrameOrigins: Vec<KeyFrame>
    // mvBackupKeyFrameOriginsId: Vec<: u32>
    // mpFirstRegionKF: KeyFrame* 
    // static const int THUMB_WIDTH
    // static const int THUMB_HEIGHT
    // mpKFinitial: KeyFrame*
    // mpKFlowerID: KeyFrame*
    // referenceMapPoints: Vec<MapPoint>
    // mvpReferenceMapPoints: Vec<MapPoint>
    // mnInitKFid: u32

    // Sofiya: following are in orbslam3, probably don't need
    // mbFail: bool
    // nNextId: : u32
    // mbImuInitialized: bool
    // mnMapChange: bool
    // mnMapChangeNotified: bool
    // mnBigChangeIdx: i32
}

impl Map {
    pub fn new() -> Map {
        let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");

        Map {
            id: 0, // TODO (Multimaps): this should increase when new maps are made
            keyframes: HashMap::new(),
            mappoints: HashMap::new(),
            optimizer: Optimizer::new(),
            sensor,
            ..Default::default()
        }
    }

    pub fn num_keyframes(&self) -> i32 { self.keyframes.len() as i32 }
    pub fn get_keyframe(&self, id: &Id) -> Option<&KeyFrame<FullKeyFrame>> { self.keyframes.get(id) }
    pub fn get_all_keyframes(&self) -> &HashMap<Id, KeyFrame<FullKeyFrame>> { &self.keyframes }
    pub fn get_mappoint(&self, id: &Id) -> Option<&MapPoint<FullMapPoint>> { self.mappoints.get(id) }
    pub fn get_all_mappoints(&self) -> &HashMap<Id, MapPoint<FullMapPoint>> { &self.mappoints }

    ////* &mut self ... behind map actor */////////////////////////////////////////////////////

    pub fn increase_mappoint_found(&mut self, id: &Id, num: &i32) {
        self.mappoints.get_mut(id).unwrap().increase_found(num);
    }

    pub fn discard_mappoint(&mut self, id: &Id) {
        self.mappoints
            .remove(id)
            .map(|mappoint| {
                for kf_id in mappoint.get_observations().keys() {
                    let indexes = mappoint.get_observations().get_observation(kf_id);
                    self.keyframes.get_mut(kf_id).unwrap().erase_mappoint_match(indexes);
                }
            });
        info!("map::discard_mappoint;{}", id);
    }

    pub fn insert_keyframe_to_map(&mut self, mut kf: &KeyFrame<PrelimKeyFrame>) -> Id {
        // Note: I would really like this to consume the keyframe, but this brings up issues
        // with the map actor being able to take ownership of the keyframe message.
        self.last_kf_id += 1;
        let full_keyframe = KeyFrame::<FullKeyFrame>::new(&kf, self.id, self.last_kf_id);
        self.keyframes.insert(self.last_kf_id, full_keyframe);

        if self.keyframes.is_empty() {
            info!("map::insert_keyframe_to_map;initialized new map;first kf:{};map init kf:{}", self.last_kf_id, self.initial_kf_id);
            self.initial_kf_id = self.last_kf_id;
            // self.lowest_kf = kf; // Sofiya: ORBSLAM3:Map.cc:67, used to sort mspKeyFrames. I think we can ignore?
        }

        info!("map::insert_keyframe_to_map;{}", self.last_kf_id);
        self.last_kf_id
    }

    pub fn insert_mappoint_to_map(&mut self, mp: MapPoint<PrelimMapPoint>) -> Id {
        self.last_mp_id += 1;
        let full_mappoint = MapPoint::<FullMapPoint>::new(mp, self.last_mp_id);
        self.mappoints.insert(self.last_mp_id, full_mappoint);
        info!("insert_mappoint_to_map;{}", self.last_mp_id);
        return self.last_mp_id;
    }

    pub fn create_initial_map_monocular(&mut self, inidata: &Initialization) -> Option<(Pose, i32, i32, HashSet<Id>)> {
        // TODO (IMU)
        // if(mSensor == System::IMU_MONOCULAR)
        //     pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);

        // Create KeyFrames
        let initial_kf_id = self.insert_keyframe_to_map(
            &KeyFrame::<PrelimKeyFrame>::new(&inidata.initial_frame.as_ref().unwrap(), &bow::VOCABULARY)
        );
        let curr_kf_id = self.insert_keyframe_to_map(
            &KeyFrame::<PrelimKeyFrame>::new(&inidata.current_frame.as_ref().unwrap(), &bow::VOCABULARY)
        );

        let mut curr_frame = inidata.current_frame.as_ref().unwrap().clone();

        // Sofiya: Removing keyframes, modifying them, then putting them back in the hashmap at the end.
        // I REALLY don't like this, but my only other option for getting two mutable keyframes inside the keyframes
        // hashmap is to wrap it all in a refcell, which I think introduces way more concurrency problems for the other 
        // actors, since they have to access get_keyframes().
        let mut initial_kf = self.keyframes.remove(&initial_kf_id).unwrap();
        let mut curr_kf = self.keyframes.remove(&curr_kf_id).unwrap();

        for (index, index2) in &inidata.mp_matches {
            let point = inidata.p3d.get(*index as usize).unwrap();
            let world_pos = DVVector3::new_with(point.x as f64, point.y as f64, point.z as f64);
            let new_mp_id = self.insert_mappoint_to_map(
                MapPoint::<PrelimMapPoint>::new(world_pos, curr_kf.id(), self.id)
            );

            {
                let new_mp = self.mappoints.get_mut(&new_mp_id).unwrap();

                initial_kf.add_mappoint(&new_mp, *index, false);
                curr_kf.add_mappoint(&new_mp, *index2, false);

                new_mp.add_observation(&initial_kf.id(), initial_kf.features.num_keypoints, *index);
                new_mp.add_observation(&curr_kf.id(), curr_kf.features.num_keypoints, *index2);
            }

            // Sofiya: clean up this workflow
            let norm_and_depth = self.mappoints.get(&new_mp_id)
                .and_then(|mp2| {mp2.get_norm_and_depth(&self)});
            if norm_and_depth.is_some() {
                self.mappoints.get_mut(&new_mp_id).map(|mp2| mp2.update_norm_and_depth(norm_and_depth.unwrap()));
            }

            let best_descriptor = self.mappoints.get(&new_mp_id)
                .and_then(|mp| mp.compute_distinctive_descriptors(&self));
            if best_descriptor.is_some() {
                self.mappoints.get_mut(&new_mp_id).map(|mp| mp.update_distinctive_descriptors(best_descriptor.unwrap()));
            }

            //Fill Current Frame structure
            curr_frame.add_mappoint(*index2, new_mp_id, false);
        }

        // Update Connections
        self.update_connections(&mut initial_kf);
        self.update_connections(&mut curr_kf);

        // Bundle Adjustment
        let optimized_poses = self.optimizer.global_bundle_adjustment(self, 0, 20);
        self.update_after_ba(optimized_poses);

        let median_depth = initial_kf.compute_scene_median_depth(&self, 2);
        let inverse_median_depth = match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => 4.0 / median_depth,
            _ => 1.0 / median_depth
        };

        if median_depth < 0.0 || curr_kf.tracked_mappoints(&self, 1) < 50 {
            // reset active map
            warn!("map::create_initial_map_monocular;wrong initialization");
            return None;
        }

        // Scale initial baseline
        let new_trans = *(curr_kf.pose.get_translation()) * inverse_median_depth;
        let mut new_pose = Pose::default();
        new_pose.set_translation(new_trans[0], new_trans[1], new_trans[2]);
        curr_kf.pose = new_pose;

        // Scale points
        for (_, (mp_id, _)) in &initial_kf.mappoint_matches {
            let mp = self.mappoints.get_mut(&mp_id).unwrap();
            mp.position = DVVector3::new((*mp.position) * inverse_median_depth);

            // Sofiya: is there a better way to do this workflow? 
            // get and update norm_and_depth needs to be separate because get needs the map, and we can't
            // mutate anything in self.mappoints if we also have to pass in self. So both function calls 
            // also can't be in the same chain.
            let norm_and_depth = self.mappoints.get(&mp_id).and_then(|mp2| mp2.get_norm_and_depth(&self));
            if norm_and_depth.is_some() {
                self.mappoints.get_mut(&mp_id).map(|mp2| mp2.update_norm_and_depth(norm_and_depth.unwrap()));
            }
        }

        // TODO (IMU)
        match self.sensor {
            Sensor(FrameSensor::Mono, ImuSensor::Some) => {
            //     pKFcur->mPrevKF = pKFini;
            //     pKFini->mNextKF = pKFcur;
            //     pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            //     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
            },
            _ => {}
        };

        // Sofiya: commented this out because I don't think they ever use it??
        // Compute here initial velocity
        // let delta_t = self.keyframes.get(&self.last_kf_id).unwrap().pose * self.keyframes.get(&1).unwrap().pose.inverse();
        // let velocity = false;
        // Eigen::Vector3f phi = deltaT.so3().log(); need to convert to rust
        // let initial_frame_ts = inidata.initial_frame.as_ref().unwrap().timestamp;
        // let curr_frame_ts = inidata.current_frame.as_ref().unwrap().timestamp;
        // let last_frame_ts = inidata.last_frame.as_ref().unwrap().timestamp;
        // let aux = (curr_frame_ts - last_frame_ts).to_std().unwrap().as_secs() / (curr_frame_ts - initial_frame_ts).to_std().unwrap().as_secs();
        // phi *= aux;

        // Sofiya: Not implemented, but might need this later.
        // mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints); // sets mvpReferenceMapPoints to mvpLocalMapPoints
        // mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose()); // sets mCameraPose = Tcw.inverse()
        // mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini); // I think this is a multi-maps thing
        // initID = pKFcur->mnId; // I think this is a multi-maps thing

        let curr_kf_id = curr_kf.id();
        let ini_kf_id = initial_kf.id();
        let curr_kf_pose = curr_kf.pose;

        // Put back the keyframes we previously took out to modify.
        self.keyframes.insert(initial_kf_id, initial_kf);
        self.keyframes.insert(curr_kf_id, curr_kf);

        // Update tracking with new info
        let relevant_mappoints = self.mappoints.keys().cloned().collect();
        Some((curr_kf_pose, curr_kf_id, ini_kf_id, relevant_mappoints))
    }


    pub fn update_connections(&mut self, main_kf: &mut KeyFrame<FullKeyFrame>) {
        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        let mut kf_counter = HashMap::<Id, i32>::new();
        for (_, (mp_id, _)) in &main_kf.mappoint_matches {
            let mp = self.mappoints.get(&mp_id).unwrap();
            for kf_id in mp.get_observations().keys() {
                *kf_counter.entry(*kf_id).or_insert(0) += 1;
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
            self.keyframes.get_mut(&kf_id).unwrap().add_connection(&main_kf.id(), *weight);
        }
        main_kf.insert_all_connections(kf_counter, self.initial_kf_id == main_kf.id())
            .map(|parent_kf| {
                self.keyframes.get_mut(&parent_kf).unwrap().add_child(main_kf.id());
            }
        );

    }

    pub fn update_after_ba(&mut self, optimized_poses: BAResult) {
        for (kf_id, pose) in optimized_poses.optimized_kf_poses {
            if optimized_poses.loop_kf_is_first_kf {
                self.keyframes.get_mut(&kf_id).unwrap().pose = pose;
            } else {
                // TODO (MVP)
                // pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(),SE3quat.translation()).cast<float>();
                // pKF->mnBAGlobalForKF = nLoopKF;
            }
        }

        for (mp_id, pose) in optimized_poses.optimized_mp_poses {
            if optimized_poses.loop_kf_is_first_kf {
                self.mappoints.get_mut(&mp_id).unwrap().position = pose.get_translation();
                let norm_and_depth = self.mappoints.get(&mp_id).unwrap().get_norm_and_depth(&self);
                if norm_and_depth.is_some() {
                    self.mappoints.get_mut(&mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
                }
            } else {
                // TODO (MVP)
                // pMP->mPosGBA = vPoint->estimate().cast<float>();
                // pMP->mnBAGlobalForKF = nLoopKF;
            }
        }

    }
}
