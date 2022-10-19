use std::collections::HashMap;

use dvcore::{matrix::{DVVector3, DVVocabulary}, global_params::{Sensor, GLOBAL_PARAMS, SYSTEM_SETTINGS}};

use crate::{dvmap::{keyframe::*, mappoint::*, sensor::SensorType}, modules::tracking_backend::Initialization, utils::optimizer::Optimizer};

use super::{pose::Pose, keypoints::KeyPointsData};

pub type Id = i32;

#[derive(Debug, Clone)]
pub struct Map<S: SensorType> {
    pub id: Id,

    pub imu_initialized: bool, // isImuInitialized(), set true by local mapper

    // KeyFrames
    keyframes: HashMap<Id, KeyFrame<S>>, // = mspKeyFrames
    last_kf_id: Id, // = mnMaxKFid
    initial_kf_id: Id, // todo (multimaps): this is the initial kf id that was added to this map, when this map is made. should tie together map vesioning better, maybe in a single struct

    // MapPoints
    mappoints: HashMap<Id, MapPoint<FullMapPoint>>, // = mspMapPoints
    last_mp_id: Id,

    // Utilities from darvis::utils
    // Sofiya: as far as I can tell, optimizer is just needed here because of global optimization, which 
    // no other thread/module/actor will use except the map. Should there even be a link to Optimizer here,
    // or should we make a specific global optimization object?
    optimizer: Optimizer,

    pub vocabulary: DVVocabulary,
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

impl<S: SensorType> Map<S> {
    pub fn new<S2: SensorType>() -> Map<S> {
        Map {
            id: 0, // TODO (Multimaps): this should increase when new maps are made
            keyframes: HashMap::new(),
            last_kf_id: 0,
            mappoints: HashMap::new(),
            last_mp_id: 0,
            initial_kf_id: 0,
            imu_initialized: false,
            optimizer: Optimizer::new(),
            vocabulary: DVVocabulary::load(GLOBAL_PARAMS.get::<String>(SYSTEM_SETTINGS, "vocabulary_file")),
        }
    }

    pub fn num_keyframes(&self) -> i32 { self.keyframes.len() as i32 }
    pub fn get_keyframe(&self, id: &Id) -> Option<&KeyFrame<S>> { self.keyframes.get(id) }
    pub fn get_mappoint(&self, id: &Id) -> Option<&MapPoint<FullMapPoint>> { self.mappoints.get(id) }

    //* &mut self ... behind map actor *//
    pub fn discard_mappoint(&mut self, id: &Id) {
        self.mappoints.remove(id);
        println!("MapPoint removed {}", id);
        // Following is done in ORB SLAM , check if this is need to be done.
        // pMP.mbTrackInView= false;      
        // pMP.last_frame_seen = self.current_frame.unwrap().id;     
    }

    pub fn increase_found(&mut self, id: &Id, n : i32) {
        self.mappoints.get_mut(id).unwrap().increase_found(n);
    }

    pub fn insert_keyframe_to_map(&mut self, mut kf: KeyFrame<S>) -> Id {
        if self.keyframes.is_empty() {
            println!("First KF: {}; Map init KF: {}", kf.id, self.initial_kf_id);
            self.initial_kf_id = kf.id;
            // self.lowest_kf = kf; // Sofiya: ORBSLAM3:Map.cc:67, used to sort mspKeyFrames. I think we can ignore?
        }

        self.last_kf_id += 1;
        kf.id = self.last_kf_id;
        self.keyframes.insert(self.last_kf_id, kf);
        println!("Inserted kf!");
        self.last_kf_id
    }

    pub fn insert_mappoint_to_map(&mut self, mp: MapPoint<PrelimMapPoint>) -> Id {
        self.last_mp_id += 1;
        let full_mappoint = MapPoint::<FullMapPoint>::new(mp, self.last_mp_id);
        self.mappoints.insert(self.last_mp_id, full_mappoint);
        println!("Inserted mappoint");
        return self.last_mp_id;
    }

    pub fn create_initial_map_monocular(&mut self, inidata: &Initialization<S>) {
        // TODO (IMU)
        // if(mSensor == System::IMU_MONOCULAR)
        //     pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);

        // Create KeyFrames
        let initial_kf_id = self.insert_keyframe_to_map(
            KeyFrame::new(&inidata.initial_frame.as_ref().unwrap(), self.id, &self.vocabulary)
        );
        let curr_kf_id = self.insert_keyframe_to_map(
            KeyFrame::new(&inidata.current_frame.as_ref().unwrap(), self.id, &self.vocabulary)
        );

        // Sofiya: I REALLY don't like this, but my only other option for getting two mutable keyframes inside the keyframes
        // hashmap is to wrap it all in a refcell, which I think introduces way more concurrency problems for the other 
        // actors, since they have to access get_keyframes().
        let mut initial_kf = self.keyframes.remove(&initial_kf_id).unwrap();
        let mut curr_kf = self.keyframes.remove(&curr_kf_id).unwrap();

        // TODO 10/17 verify: not sure ini_matches is correctly set to (index,Id) in search_for_initialization, might be flipped?
        for (index, index2) in &inidata.mp_matches {
            let point = inidata.p3d.get(*index as usize).unwrap();
            let world_pos = DVVector3::new_with(point.x as f64, point.y as f64, point.z as f64);
            let new_mp_id = self.insert_mappoint_to_map(
                MapPoint::<PrelimMapPoint>::new(world_pos, curr_kf.id, self.id)
            );
            let new_mp = self.mappoints.get_mut(&new_mp_id).unwrap();

            initial_kf.add_mappoint(&new_mp, *index);
            curr_kf.add_mappoint(&new_mp, *index2);

            new_mp.add_observation(&initial_kf, *index);
            new_mp.add_observation(&curr_kf, *index2);

            new_mp.compute_distinctive_descriptors();
            new_mp.update_normal_and_depth();
        }

        // Update Connections
        initial_kf.update_connections();
        curr_kf.update_connections();

        // Bundle Adjustment
        self.optimizer.global_bundle_adjustment(self.id, 20);

        let median_depth = self.compute_scene_median_depth(&initial_kf, 2);
        let inverse_median_depth = match S::sensor_type() {
            Sensor::ImuMono => 4.0 / median_depth,
            _ => 1.0 / median_depth
        };

        if median_depth < 0.0 || self.tracked_mappoints_for_keyframe(&curr_kf, 1) < 50 {
            // reset active map
            println!("Wrong initialization, resetting");
            return;
        }

        // Scale initial baseline
        let new_trans = curr_kf.pose.get_translation().vec() * inverse_median_depth;
        let mut new_pose = Pose::default();
        new_pose.set_translation(new_trans[0], new_trans[1], new_trans[2]);
        curr_kf.set_pose(new_pose);

        // Scale points
        for (_index, mp_id) in &initial_kf.mappoint_matches {
            let mp = self.mappoints.get_mut(&mp_id).unwrap();
            mp.position = DVVector3::new(mp.position.vec() * inverse_median_depth);
            mp.update_normal_and_depth();
        }

        // TODO (IMU)
        match S::sensor_type() {
            Sensor::ImuMono => {
            //     pKFcur->mPrevKF = pKFini;
            //     pKFini->mNextKF = pKFcur;
            //     pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

            //     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
            },
            _ => {}
        };

        // stuff to return back to tracking?
            // pose = curr_kf.pose();
            // last_keyframe_id = inidata.current_frame.unwrap().id;
            // local_keyframes.push(current_kf.unwrap().id)
            // local_keyframes.push(initial_kf.unwrap().id)
            // local_points = self.mappoints.keys
            // reference_kf = current_kf

            // TODO 10/17 fill 

        // Compute here initial velocity
        // vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

        // Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
        // mbVelocity = false;
        // Eigen::Vector3f phi = deltaT.so3().log();

        // double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
        // phi *= aux;

        // mLastFrame = Frame(mCurrentFrame);

        // mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        // mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        // mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        // mState=OK;

        // initID = pKFcur->mnId;

        // add keyframes to LM
        // mpLocalMapper->InsertKeyFrame(pKFini);
        // mpLocalMapper->InsertKeyFrame(pKFcur);
        // mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;

        // Put back the keyframes we previously took out to modify.
        self.keyframes.insert(initial_kf_id, initial_kf);
        self.keyframes.insert(curr_kf_id, curr_kf);
    }

    // Sofiya: Below code is all related to keyframes but I put it here because they all need to access mappoint data
    // from self.mappoints in the map. I'm not sure if there's a better way to do this.
    pub fn tracked_mappoints_for_keyframe(&self, kf: &KeyFrame<S>, min_observations: u32) -> i32{
        // KeyFrame::TrackedMapPoints(const int &minObs)
        let mut num_points = 0;
        for (_, mp_id) in &kf.mappoint_matches {
            let mappoint = self.mappoints.get(&mp_id).unwrap();
            if min_observations > 0 {
                if mappoint.observations().len() >= (min_observations as usize) {
                    num_points += 1;
                }
            } else {
                    num_points += 1;
            }
        }

        return num_points;
    }
    pub fn compute_scene_median_depth(&self, kf: &KeyFrame<S>, q: i32) -> f64 {
        if kf.keypoints_data.num_keypoints() == 0 {
            return -1.0;
        }

        let mut depths = Vec::new();
        depths.reserve(kf.keypoints_data.num_keypoints() as usize);
        let rot = kf.pose.get_rotation();
        let rcw2 = rot.vec().row(2);
        let zcw = kf.pose.get_translation().vec()[2];

        for (_index, mp_id) in &kf.mappoint_matches {
            let world_pos = self.mappoints.get(mp_id).unwrap().position.vec();
            let z = (rcw2 * world_pos)[0] + zcw; // first part of this term is scalar but still need to get it from Matrix<1,1> to f64
            depths.push(z);
        }

        depths.sort_by(|a, b| a.total_cmp(&b));

        return depths[(depths.len()-1) / q as usize];
    }
}
