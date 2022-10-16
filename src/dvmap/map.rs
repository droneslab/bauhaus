use std::collections::HashMap;

use dvcore::{matrix::DVVector3, global_params::Sensor};

use crate::{dvmap::{keyframe::*, mappoint::*, sensor::SensorType}, modules::tracking_backend::Initialization};

use super::frame::Frame;

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
    mappoints: HashMap<Id, MapPoint>, // = mspMapPoints
    last_mp_id: Id,

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
        }
    }

    pub fn num_keyframes(&self) -> i32 { self.keyframes.len() as i32 }
    pub fn get_keyframe(&self, id: &Id) -> Option<&KeyFrame<S>> { self.keyframes.get(id) }
    pub fn get_mappoint(&self, id: &Id) -> Option<&MapPoint> { self.mappoints.get(id) }

    pub fn tracked_mappoints_for_keyframe(&self, kf_id: Id, min_observations: u32) -> i32{
        // KeyFrame::TrackedMapPoints(const int &minObs)
        let mut num_points = 0;
        for (_, mp_id) in &self.keyframes.get(&kf_id).unwrap().mappoint_matches {
            let mappoint = self.mappoints.get(&mp_id).unwrap();
            if min_observations > 0 {
                if mappoint.observations.len() >= (min_observations as usize) {
                    num_points += 1;
                }
            } else {
                    num_points += 1;
            }
        }

        return num_points;
    }


    //* &mut self ... behind map actor *//
    pub fn discard_mappoint(&mut self, id: &Id) {
        self.mappoints.remove(id);
        println!("MapPoint removed {}", id);
        // Following is done in ORB SLAM , check if this is need to be done.
        // pMP.mbTrackInView= false;      
        // pMP.last_frame_seen = self.current_frame.unwrap().id;     
    }

    pub fn increase_found(&mut self, id: &Id, n : i32)
    {
        self.mappoints.get_mut(id).unwrap().nfound += n;
        println!("MapPoint found increased {}", id);
    }

    pub fn insert_keyframe_to_map(&mut self, kf: KeyFrame<S>) {
        if self.keyframes.is_empty() {
            println!("First KF: {}; Map init KF: {}", kf.id, self.initial_kf_id);
            self.initial_kf_id = kf.id;
            // self.lowest_kf = kf; // Sofiya: ORBSLAM3:Map.cc:67, used to sort mspKeyFrames. I think we can ignore?
        }

        self.last_kf_id += 1;
        self.keyframes.insert(self.last_kf_id, kf);
        println!("Inserted kf!");
    }

    pub fn new_mappoint(&mut self, mp: MapPoint) {
        // TODO IMPORTANT
    }

    pub fn create_initial_map_monocular(&mut self, inidata: &Initialization<S>) {
        todo!("create initial map monocular");
        // // Create KeyFrames
        // let initial_kf = KeyFrame::new_with_id(inidata.ini_frame, self.id, self.last_kf_id);
        // self.last_kf_id += 1;
        // self.insert_keyframe_to_map(initial_kf);
        // let curr_kf = KeyFrame::new_with_id(inidata.curr_frame, self.id, self.last_kf_id);
        // self.last_kf_id += 1;
        // self.insert_keyframe_to_map(curr_kf);

        // // TODO (IMU)
        // // if(mSensor == System::IMU_MONOCULAR)
        // //     pKFini->mpImuPreintegrated = (IMU::Preintegrated*)(NULL);

        // for (index, mp_id) in inidata.mp_matches { // todo: not sure ini_matches is correctly set to (index,Id) in search_for_initialization, might be flipped?
        //     // Create MapPoint.
        //     let point = inidata.p3d.get(index as usize).unwrap();
        //     let world_pos = DVVector3::new_with(point.x, point.y, point.z);
        //     let new_mp = MapPoint::new_with_id(world_pos, curr_kf.id, self.id, self.last_mp_id);
        //     self.last_mp_id += 1;

        //     initial_kf.add_mappoint_match(new_mp.id, pMP,i);
        //     curr_kf.add_mappoint_match(pMP,mvIniMatches[i]);

        //     new_mp.add_observation(pKFini,i);
        //     new_mp.add_observation(pKFcur,mvIniMatches[i]);

        //     new_mp.compute_distinctive_descriptors();
        //     new_mp.update_normal_and_depth();

        //     //Fill Current Frame structure
        //     self.current_frame.add_mappoint_match(index, mp_id);

        //     self.new_mappoint(new_mp);
        // }

        // // Update Connections
        // initial_kf.update_connections();
        // curr_kf.update_connections();

        // let s_mps = initial_kf.mappoint_matches;

        // // Bundle Adjustment
        // self.optimizer.global_bundle_adjustment(self.map_id, 20);

        // let median_depth = initial_kf.ComputeSceneMedianDepth(2);
        // let inverse_median_depth = match S::sensor_type() {
        //     Sensor::ImuMono => 4.0 / median_depth,
        //     _ => 1.0 / median_depth
        // };

        // if median_depth < 0 || self.tracked_mappoints_for_keyframe(curr_kf, 1) < 50 {
        //     // reset active map
        //     println!("Wrong initialization, resetting");
        //     return;
        // }

        // // Scale initial baseline
        // let translation = curr_kf.pose.get_translation();
        // curr_kf.set_pose(translation * inverse_median_depth);

        // // Scale points
        // for (index, mp_id) in initial_kf.mappoint_matches {
        //     // set mp id world position to pMP->GetWorldPos()*invMedianDepth
        //     // pmp->updatenormalanddepth()
        // }

        // match S::sensor_type() {
        //     Sensor::ImuMono => {
        //     //     pKFcur->mPrevKF = pKFini;
        //     //     pKFini->mNextKF = pKFcur;
        //     //     pKFcur->mpImuPreintegrated = mpImuPreintegratedFromLastKF;

        //     //     mpImuPreintegratedFromLastKF = new IMU::Preintegrated(pKFcur->mpImuPreintegrated->GetUpdatedBias(),pKFcur->mImuCalib);
        //     },
        //     _ => {}
        // };

        // // mCurrentFrame.SetPose(pKFcur->GetPose());
        // // mnLastKeyFrameId=mCurrentFrame.mnId;
        // // mpLastKeyFrame = pKFcur;

        // local_keyframes.push(current_kf.id);
        // local_keyframes.push(initial_kf.id);
        // // mvpLocalMapPoints=mpAtlas->GetAllMapPoints();
        // // mpReferenceKF = pKFcur;
        // // mCurrentFrame.mpReferenceKF = pKFcur;

        // // // Compute here initial velocity
        // // vector<KeyFrame*> vKFs = mpAtlas->GetAllKeyFrames();

        // // Sophus::SE3f deltaT = vKFs.back()->GetPose() * vKFs.front()->GetPoseInverse();
        // // mbVelocity = false;
        // // Eigen::Vector3f phi = deltaT.so3().log();

        // // double aux = (mCurrentFrame.mTimeStamp-mLastFrame.mTimeStamp)/(mCurrentFrame.mTimeStamp-mInitialFrame.mTimeStamp);
        // // phi *= aux;

        // // mLastFrame = Frame(mCurrentFrame);

        // // mpAtlas->SetReferenceMapPoints(mvpLocalMapPoints);

        // // mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

        // // mpAtlas->GetCurrentMap()->mvpKeyFrameOrigins.push_back(pKFini);

        // // mState=OK;

        // // initID = pKFcur->mnId;


        // // Insert KFs in the map
        // // mpAtlas->AddKeyFrame(pKFini);
        // // mpAtlas->AddKeyFrame(pKFcur);


        // //Add mappoints to map Map
        // //     mpAtlas->AddMapPoint(pMP);

        // // add keyframes to LM
        // // mpLocalMapper->InsertKeyFrame(pKFini);
        // // mpLocalMapper->InsertKeyFrame(pKFcur);
        // // mpLocalMapper->mFirstTs=pKFcur->mTimeStamp;
    }
}
