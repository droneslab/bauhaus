use std::collections::HashMap;
use serde::{Deserialize, Serialize};
extern crate nalgebra as na;
use crate::matrix::DVVector3;
use super::{map::{Id}, keyframe::KeyFrame, sensor::{SensorType}};

// Note: Implementing typestate for like here: http://cliffle.com/blog/rust-typestate/#a-simple-example-the-living-and-the-dead
// This way we can encode mappoints that have been created but not inserted into the map as a separate type than mappoints that are legit.
// This prevents making the following mistake:
// 1 - create a mappoint with MapPoint::new() with an id = -1.
// 2 - connect the mappoint to a keyframe, which would incorrectly set the ID to be -1.
// 3 - insert the mappoint into the map, which gives it a proper id.
// The correct order is to flip steps 2 and 3, so by the time an observation is set, we KNOW the mappoint Id has already been set too.
// We can make this a compile-time error by adding state types in the following functions:
// Creating a mappoint returns a MapPoint<PrelimMapItem>
// Inserting into the map turns a MapPoint<PrelimMapItem> into a MapPoint<FullMapItem>
// The function in step 2 (to add a mappoint to a keyframe) takes a MapPoint<FullMapItem>

// Mappoint information that is ALWAYS available, regardless of mappoint state.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MapPoint<M: MapPointState> {
    // List of variables set in constructor in orbslam3, don't think we need these all but copying for reference
    //     mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    //     mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    //     mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    //     mpReplaced(static_cast<MapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), 
    pub position: DVVector3<f64>, // Sofiya: why a vector3 instead of a Pose? Same in ORBSLAM3 too but I thought mps had poses...

    // Map connections
    // Sofiya: it would be nice if we could guarantee that the connections are updated/correct
    // rather than duplicating all these connections across all the objects and hoping we remember
    // to update them correctly after a map modification
    origin_map_id: Id,
    ref_kf: Id, // = mnFirstKFid and mpRefKF ... first one is Id, second is a pointer to the KF, but we only keep Ids 

    // Variables in ORBSLAM, DON'T Set these!!
    // mnFirstFrame ... literally never used meaningfully
    // mnLastFrameSeen ... similar to "mnTrackReferenceForFrame" in KeyFrame. redundant and easy to mess up/get out of sync. Search for this globally to see an example of how to avoid using it.
    // mbTrackInView ... only used by tracking to keep track of which mappoints to show. Just keep this data saved in tracking locally

    // This reassures the compiler that the parameter
    // gets used.
    pub map_data: M,
}

// State type options.
#[derive(Clone, Debug)]
pub struct PrelimMapPoint {} // prelimary map item created locally but not inserted into the map yet
// map item has an assigned id but not inserted into the map yet because it still
// needs some fields updated before it should be used
// #[derive(Clone, Debug)]
// pub struct InterimMapItem {
//     id: Id
// }
#[derive(Clone, Debug)]
pub struct FullMapPoint {
    // Full map item inserted into the map with the following additional fields:
    pub id: Id,

    // Observations
    // This is a part of "map connections" but I don't think we can avoid keeping this here.
    observations: HashMap<Id, i32>, // mObservations ; Keyframes observing the point and associated index in keyframe

    // Variables used by merging
    normal_vector: DVVector3<f64>,  // mNormalVector ; Mean viewing direction

    // following two are only set by tracking and only checked by local mapping for mappoint culling, but not sure what the diff is b/w visible and found
    nvisible: i32, //mnvisible
    nfound: i32, //mnfound
}

pub trait MapPointState {}
impl MapPointState for PrelimMapPoint {}
impl MapPointState for FullMapPoint {}

impl MapPoint<PrelimMapPoint> {
    pub fn new(position: DVVector3<f64>, ref_kf: Id, origin_map_id: Id) -> Self {
        Self {
            position,
            origin_map_id,
            ref_kf,
            map_data: PrelimMapPoint{} 
        }
    }
    // Sofiya: Two constructors, this one takes an idxF that I think is the index in observations
    // Need to figure out why sometimes this one is called instead of the other one 
    // MapPoint::MapPoint(const Eigen::Vector3f &Pos, Map* pMap, Frame* pFrame, const int &idxF):
    // mnFirstKFid(-1), mnFirstFrame(pFrame->mnId), nObs(0), mnTrackReferenceForFrame(0), mnLastFrameSeen(0),
    // mnBALocalForKF(0), mnFuseCandidateForKF(0),mnLoopPointForKF(0), mnCorrectedByKF(0),
    // mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(static_cast<KeyFrame*>(NULL)), mnVisible(1),
    // mnFound(1), mbBad(false), mpReplaced(NULL), mpMap(pMap), mnOriginMapId(pMap->GetId())
    // {
    //     SetWorldPos(Pos);

    //     Eigen::Vector3f Ow;
    //     if(pFrame -> Nleft == -1 || idxF < pFrame -> Nleft){
    //         Ow = pFrame->GetCameraCenter();
    //     }
    //     else{
    //         Eigen::Matrix3f Rwl = pFrame->GetRwc();
    //         Eigen::Vector3f tlr = pFrame->GetRelativePoseTlr().translation();
    //         Eigen::Vector3f twl = pFrame->GetOw();

    //         Ow = Rwl * tlr + twl;
    //     }
    //     mNormalVector = mWorldPos - Ow;
    //     mNormalVector = mNormalVector / mNormalVector.norm();

    //     Eigen::Vector3f PC = mWorldPos - Ow;
    //     const float dist = PC.norm();
    //     const int level = (pFrame -> Nleft == -1) ? pFrame->mvKeysUn[idxF].octave
    //                                             : (idxF < pFrame -> Nleft) ? pFrame->mvKeys[idxF].octave
    //                                                                         : pFrame -> mvKeysRight[idxF].octave;
    //     const float levelScaleFactor =  pFrame->mvScaleFactors[level];
    //     const int nLevels = pFrame->mnScaleLevels;

    //     mfMaxDistance = dist*levelScaleFactor;
    //     mfMinDistance = mfMaxDistance/pFrame->mvScaleFactors[nLevels-1];

    //     pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

    //     // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    //     unique_lock<mutex> lock(mpMap->mMutexPointCreation);
    //     mnId=nNextId++;
    // }
}

impl MapPoint<FullMapPoint> {
    pub(super) fn new(prelim_mappoint: MapPoint<PrelimMapPoint>, id: Id) -> Self {
        Self {
            position: prelim_mappoint.position,
            origin_map_id: prelim_mappoint.origin_map_id,
            ref_kf: prelim_mappoint.ref_kf,
            map_data: FullMapPoint{
                id,
                observations: HashMap::new(),
                normal_vector: DVVector3::zeros::<f64>(),
                nvisible: 1,
                nfound: 1,
            },
        }
    }
    pub fn increase_found(&mut self, n: i32) {
        self.map_data.nfound += n;
        println!("MapPoint found increased {}", self.map_data.id);
    }
    pub fn observations(&self) -> &HashMap<Id, i32>{
        return &self.map_data.observations;
    }
    pub fn add_observation<S: SensorType>(&mut self, kf: &KeyFrame<S>, index: u32) {
        todo!("TODO 10/17 fill");
        // tuple<int,int> indexes;

        // if(mObservations.count(pKF)){
        //     indexes = mObservations[pKF];
        // }
        // else{
        //     indexes = tuple<int,int>(-1,-1);
        // }

        // if(pKF -> NLeft != -1 && idx >= pKF -> NLeft){
        //     get<1>(indexes) = idx;
        // }
        // else{
        //     get<0>(indexes) = idx;
        // }

        // mObservations[pKF]=indexes;

        // if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0)
        //     nObs+=2;
        // else
        //     nObs++;
    }
    
    pub fn compute_distinctive_descriptors(&mut self) {
        todo!("TODO 10/17 fill");
        // // Retrieve all observed descriptors
        // vector<cv::Mat> vDescriptors;

        // map<KeyFrame*,tuple<int,int>> observations;

        // {
        //     unique_lock<mutex> lock1(mMutexFeatures);
        //     if(mbBad)
        //         return;
        //     observations=mObservations;
        // }

        // if(observations.empty())
        //     return;

        // vDescriptors.reserve(observations.size());

        // for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        // {
        //     KeyFrame* pKF = mit->first;

        //     if(!pKF->isBad()){
        //         tuple<int,int> indexes = mit -> second;
        //         int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        //         if(leftIndex != -1){
        //             vDescriptors.push_back(pKF->mDescriptors.row(leftIndex));
        //         }
        //         if(rightIndex != -1){
        //             vDescriptors.push_back(pKF->mDescriptors.row(rightIndex));
        //         }
        //     }
        // }

        // if(vDescriptors.empty())
        //     return;

        // // Compute distances between them
        // const size_t N = vDescriptors.size();

        // float Distances[N][N];
        // for(size_t i=0;i<N;i++)
        // {
        //     Distances[i][i]=0;
        //     for(size_t j=i+1;j<N;j++)
        //     {
        //         int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
        //         Distances[i][j]=distij;
        //         Distances[j][i]=distij;
        //     }
        // }

        // // Take the descriptor with least median distance to the rest
        // int BestMedian = INT_MAX;
        // int BestIdx = 0;
        // for(size_t i=0;i<N;i++)
        // {
        //     vector<int> vDists(Distances[i],Distances[i]+N);
        //     sort(vDists.begin(),vDists.end());
        //     int median = vDists[0.5*(N-1)];

        //     if(median<BestMedian)
        //     {
        //         BestMedian = median;
        //         BestIdx = i;
        //     }
        // }

        // {
        //     unique_lock<mutex> lock(mMutexFeatures);
        //     mDescriptor = vDescriptors[BestIdx].clone();
        // }
    }

    pub fn update_normal_and_depth(&mut self) {
        todo!("TODO 10/17 fill");
        // map<KeyFrame*,tuple<int,int>> observations;
        // KeyFrame* pRefKF;
        // Eigen::Vector3f Pos;
        // {
        //     unique_lock<mutex> lock1(mMutexFeatures);
        //     unique_lock<mutex> lock2(mMutexPos);
        //     if(mbBad)
        //         return;
        //     observations = mObservations;
        //     pRefKF = mpRefKF;
        //     Pos = mWorldPos;
        // }

        // if(observations.empty())
        //     return;

        // Eigen::Vector3f normal;
        // normal.setZero();
        // int n=0;
        // for(map<KeyFrame*,tuple<int,int>>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        // {
        //     KeyFrame* pKF = mit->first;

        //     tuple<int,int> indexes = mit -> second;
        //     int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);

        //     if(leftIndex != -1){
        //         Eigen::Vector3f Owi = pKF->GetCameraCenter();
        //         Eigen::Vector3f normali = Pos - Owi;
        //         normal = normal + normali / normali.norm();
        //         n++;
        //     }
        //     if(rightIndex != -1){
        //         Eigen::Vector3f Owi = pKF->GetRightCameraCenter();
        //         Eigen::Vector3f normali = Pos - Owi;
        //         normal = normal + normali / normali.norm();
        //         n++;
        //     }
        // }

        // Eigen::Vector3f PC = Pos - pRefKF->GetCameraCenter();
        // const float dist = PC.norm();

        // tuple<int ,int> indexes = observations[pRefKF];
        // int leftIndex = get<0>(indexes), rightIndex = get<1>(indexes);
        // int level;
        // if(pRefKF -> NLeft == -1){
        //     level = pRefKF->mvKeysUn[leftIndex].octave;
        // }
        // else if(leftIndex != -1){
        //     level = pRefKF -> mvKeys[leftIndex].octave;
        // }
        // else{
        //     level = pRefKF -> mvKeysRight[rightIndex - pRefKF -> NLeft].octave;
        // }

        // //const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
        // const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
        // const int nLevels = pRefKF->mnScaleLevels;

        // {
        //     unique_lock<mutex> lock3(mMutexPos);
        //     mfMaxDistance = dist*levelScaleFactor;
        //     mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        //     mNormalVector = normal/n;
        // }
    }
}