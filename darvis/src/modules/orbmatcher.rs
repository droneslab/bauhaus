use std::collections::HashMap;
use std::convert::TryInto;
use opencv::core::{Point2f, KeyPoint};
use opencv::prelude::*;
use crate::dvmap::keyframe::KeyFrame;
use crate::dvmap::features::Features;
use crate::dvmap::mappoint::FullMapPoint;
use crate::{
    dvmap::{frame::Frame, mappoint::MapPoint, map::Id, map::Map},
    modules::{camera::Camera},
    lockwrap::ReadOnlyWrapper,
};

const  TH_HIGH: i32= 100;
const  TH_LOW: i32 = 50;
const  HISTO_LENGTH: i32 = 30;

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel

pub fn search_for_initialization(
    ratio: f64, check_orientation: bool,
    F1: &Frame, F2: &Frame, prev_matched: &mut Vec<Point2f>, matches12: &mut HashMap<u32, Id>, window_size : i64
) -> i32 {
    todo!("TODO 10/17 BINDINGS search_for_initialization");
    // //ref code : https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/0df83dde1c85c7ab91a0d47de7a29685d046f637/src/ORBmatcher.cc#L648
    // let mut nmatches: i32=0;

    // let mut rotHist = Vec::<Vec<i32>>::new();
    // rotHist.resize(HISTO_LENGTH as usize, vec![0; 500]);

    // const FACTOR: f64 = 1.0 as f64 / HISTO_LENGTH as f64;

    // let mut matched_distance = vec![std::i32::MAX; F2.keypoints_data.num_keypoints.try_into().unwrap()]; 
    // let mut matches21 = vec![-1 as i32; F2.keypoints_data.num_keypoints.try_into().unwrap()];
    // let iend1 = F1.keypoints_data.num_keypoints;
    // for i1 in 0..iend1 as usize{
    //     let kp1: KeyPoint = F1.keypoints_data.keypoints_un().get(i1).unwrap().clone();
    //     let level1 = kp1.octave;
    //     if level1>0 {
    //         continue;
    //     }

    //     let indices2 = F2.get_features_in_area(&(prev_matched.get(i1).unwrap().x as f64), &(prev_matched.get(i1).unwrap().y as f64), &(window_size as f64), &(level1 as i64), &(level1 as i64));
    //     //vector<size_t> vIndices2 = F2.get_features_in_area(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

    //     if indices2.is_empty() {
    //         continue;
    //     }

    //     let d1 = F1.keypoints_data.descriptors().row(i1 as u32).unwrap();

    //     let mut bestDist = i32::MAX;
    //     let mut bestDist2 = i32::MAX;
    //     let mut bestIdx2: i32 = -1;

    //     for i in 0..indices2.len() {
    //         let i2 = i;
    //         let d2 = F2.keypoints_data.descriptors().row(i2 as u32).unwrap();
    //         let dist = descriptor_distance(&d1,&d2);

    //         if matched_distance[i2]<=dist {
    //             continue;
    //         } 

    //         if dist<bestDist {
    //             bestDist2=bestDist;
    //             bestDist=dist;
    //             bestIdx2=i2 as i32;
    //         } else if dist<bestDist2 {
    //             bestDist2=dist;
    //         }
    //     }

    //     if bestDist<=TH_LOW {
    //         if bestDist< bestDist2* (ratio  as i32) {
    //             if matches21[bestIdx2 as usize]>=0 {
    //                 matches12.remove(&(bestIdx2 as u32));
    //                 nmatches-=1;
    //             }
    //             matches12.insert(i1.try_into().unwrap(), bestIdx2);
    //             matches21[bestIdx2 as usize]=i1 as i32;
    //             matched_distance[bestIdx2 as usize]=bestDist;
    //             nmatches+=1;

    //             if check_orientation {
    //                 let mut rot = F1.keypoints_data.keypoints_un().get(i1).unwrap().angle-F2.keypoints_data.keypoints_un().get(bestIdx2 as usize).unwrap().angle;
    //                 if rot<0.0 {
    //                     rot+=360.0;
    //                 }

    //                 let mut bin = f64::round((rot as f64 )*FACTOR) as i32;
    //                 if bin==HISTO_LENGTH {
    //                     bin=0;
    //                 }
    //                 assert!(bin>=0 && bin<HISTO_LENGTH);

    //                 rotHist[bin as usize].push(i1 as i32);
    //             }
    //         }
    //     }

    // }

    // if check_orientation
    // {
    //     let mut ind1: i32 =-1;
    //     let mut ind2: i32 =-1;
    //     let mut ind3: i32 =-1;

    //     compute_three_maxima(&rotHist,HISTO_LENGTH,&mut ind1,&mut ind2,&mut ind3);

    //     for i in 0..HISTO_LENGTH
    //     {
    //         if i==ind1 || i==ind2 || i==ind3
    //         {
    //             continue;
    //         }
                
    //         for j in 0..rotHist[i as usize].len()
    //         {
    //             let idx1: usize = rotHist[i as usize][j] as usize;
    //             if matches12.contains_key(&(idx1 as u32))
    //             {
    //                 matches12.remove(&(idx1 as u32));
    //                 nmatches-=1;
    //             }
    //         }
    //     }

    // }

    // //Update prev matched
    // for i1 in 0..matches12.len()
    // {
    //     if matches12.contains_key(&(i1 as u32))
    //     {
    //         prev_matched[i1]= F2.keypoints_data.keypoints_un().get(*matches12.get(&(i1 as u32)).unwrap() as usize).unwrap().pt.clone();
    //     }
    // }

    // return nmatches;
}

pub fn compute_three_maxima(
    histo : &Vec<Vec<i32>> , L: i32, ind1: &mut i32, ind2: &mut i32, ind3: &mut i32
) {
    let mut max1: i32=0;
    let mut max2: i32=0;
    let mut max3: i32=0;

    for i in 0..L
    {
        let s = histo[i as usize].len() as i32;
        if s>max1 
        {
            max3=max2;
            max2=max1;
            max1=s;
            *ind3=*ind2;
            *ind2=*ind1;
            *ind1=i;
        }
        else if s>max2 
        {
            max3=max2;
            max2=s;
            *ind3=*ind2;
            *ind2=i;
        }
        else if s>max3 
        {
            max3=s;
            *ind3=i;
        }
    }

    if max2< (0.1 * (max1 as f64)) as i32
    {
        *ind2=-1;
        *ind3=-1;
    }
    else if max3< ((0.1*(max1 as f64)) as i32)
    {
        *ind3=-1;
    }
}

pub fn search_by_projection(frame: &Frame, mappoints: Vec<Id>, th: f64, far_points: bool, th_far_points: f64) -> i32 {
    // int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th, const bool bFarPoints, const float thFarPoints)
    todo!("TODO 10/17 BINDINGS search_by_projection");
}

// Project MapPoints tracked in last frame into the current frame and search matches.
// Used to track from previous frame (Tracking)
pub fn search_by_projection_with_threshold (
    current_frame: &Frame, last_frame: &Frame, threshold: i32, is_mono: bool,
    camera: &Camera, map: &ReadOnlyWrapper<Map>
) -> Vec<MapPoint<FullMapPoint>> {
    todo!("TODO 10/17 BINDINGS search_by_projection");
    // int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
    let nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    let mut rotHist = Vec::<Vec<i32>>::new();
    for i in 0..HISTO_LENGTH as usize {
        rotHist.push(vec![]);
        rotHist[i].reserve(500);
    }
    let factor = 1.0 / (HISTO_LENGTH as f32);

    // Note: this code copied from ORBSLAM2, not ORBSLAM3
    // 2 : https://github.com/raulmur/ORB_SLAM2/blob/master/src/ORBmatcher.cc#L1328
    // 3 : https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/ORBmatcher.cc#L1676
    // I think it is doing the same thing, just that orbslam3 uses Sophus SE3 objects
    // and ORBSLAM2 uses matrices
    // But, copying the matrices is more straightforward than figuring out what Sophus::SE3
    // is doing, especially around the matrix multiplications
    let Rcw = current_frame.pose.unwrap().rotation();
    let tcw = current_frame.pose.unwrap().translation();
    let twc = -Rcw.transpose() * (*tcw);

    let Rlw = last_frame.pose.unwrap().rotation();
    let tlw = current_frame.pose.unwrap().translation();
    let tlc = (*Rlw) * twc + (*tlw);

    let forward = tlc[2] >current_frame.stereo_baseline && !is_mono;
    let backward = -tlc[2] > current_frame.stereo_baseline && !is_mono;

    for i in 0..last_frame.features.num_keypoints as u32 {
        if last_frame.mappoint_matches.contains_key(&i) && !last_frame.is_mp_outlier(&i) {
            // Project
            let x3Dc;
            {
                let map_read_lock = map.read();
                let mappoint = map_read_lock.get_mappoint(&last_frame.mappoint_matches.get(&i).unwrap().0);
                let x3Dw = mappoint.unwrap().position;
                x3Dc = (*Rcw) * (*x3Dw) + (*tcw);
            }

            let xc = x3Dc[0];
            let yc = x3Dc[1];
            let invzc = 1.0 / x3Dc[2];
            if invzc < 0.0 {
                continue;
            }

            let u = camera.fx * xc * invzc + camera.cx;
            let v = camera.fy * yc * invzc + camera.cy;
            if u < current_frame.features.image_bounds.min_x || u > current_frame.features.image_bounds.max_x {
                continue;
            }
            if v < current_frame.features.image_bounds.min_y || v > current_frame.features.image_bounds.max_y {
                continue;
            }


            // let last_octave = last_frame.keypoints.get(i as usize).unwrap().octave;
            // let n_last_octave;
            // if last_frame.Nleft == 0 || i < last_frame.Nleft.try_into().unwrap() {
            //     n_last_octave = last_frame.keypoints.get(i as usize).octave
            // } else {
            //     n_last_octave = last_frame.keypoints_right[i - (last_frame.Nleft as i32)].octave;
            // }

            // // Search in a window. Size depends on scale
            // let radius = threshold * current_frame.mvScaleFactors[n_last_octave];

            // let vIndices2;
            // // TODO (Stereo): last argument should be true if stereo
            // if forward {
            //     vIndices2 = current_frame.get_features_in_area(u,v, radius, n_last_octave, false);
            // } else if backward {
            //     vIndices2 = current_frame.get_features_in_area(u,v, radius, 0, n_last_octave, false);
            // } else {
            //     vIndices2 = current_frame.get_features_in_area(u,v, radius, n_last_octave-1, n_last_octave+1, false);
            // }
            // if vIndices2.is_empty() {
            //     continue;
            // }

            // let dMP = pMP.get_descriptor();
            // let best_dist = 256;
            // let best_idx2 = -1;



    //             for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
    //             {
    //                 const size_t i2 = *vit;

    //                 if(current_frame.mappoint_matches[i2])
    //                     if(current_frame.mappoint_matches[i2]->Observations()>0)
    //                         continue;

    //                 if(current_frame.Nleft == -1 && current_frame.mvuRight[i2]>0)
    //                 {
    //                     const float ur = uv(0) - current_frame.mbf*invzc;
    //                     const float er = fabs(ur - current_frame.mvuRight[i2]);
    //                     if(er>radius)
    //                         continue;
    //                 }

    //                 const cv::Mat &d = current_frame.mDescriptors.row(i2);

    //                 const int dist = DescriptorDistance(dMP,d);

    //                 if(dist<bestDist)
    //                 {
    //                     bestDist=dist;
    //                     bestIdx2=i2;
    //                 }
    //             }

    //             if(bestDist<=TH_HIGH)
    //             {
    //                 current_frame.mappoint_matches[bestIdx2]=pMP;
    //                 nmatches++;

    //                 if(mbCheckOrientation)
    //                 {
    //                     cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
    //                                                                 : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
    //                                                                                         : LastFrame.mvKeysRight[i - LastFrame.Nleft];

    //                     cv::KeyPoint kpCF = (current_frame.Nleft == -1) ? current_frame.mvKeysUn[bestIdx2]
    //                                                                    : (bestIdx2 < current_frame.Nleft) ? current_frame.mvKeys[bestIdx2]
    //                                                                                                      : current_frame.mvKeysRight[bestIdx2 - current_frame.Nleft];
    //                     float rot = kpLF.angle-kpCF.angle;
    //                     if(rot<0.0)
    //                         rot+=360.0f;
    //                     int bin = round(rot*factor);
    //                     if(bin==HISTO_LENGTH)
    //                         bin=0;
    //                     assert(bin>=0 && bin<HISTO_LENGTH);
    //                     rotHist[bin].push_back(bestIdx2);
    //                 }
    //             }
    //             if(current_frame.Nleft != -1){
    //                 Eigen::Vector3f x3Dr = current_frame.GetRelativePoseTrl() * x3Dc;
    //                 Eigen::Vector2f uv = current_frame.mpCamera->project(x3Dr);

    //                 int nLastOctave = (LastFrame.Nleft == -1 || i < LastFrame.Nleft) ? LastFrame.mvKeys[i].octave
    //                                      : LastFrame.mvKeysRight[i - LastFrame.Nleft].octave;

    //                 // Search in a window. Size depends on scale
    //                 float radius = th*current_frame.mvScaleFactors[nLastOctave];

    //                 vector<size_t> vIndices2;

    //                 if(bForward)
    //                     vIndices2 = current_frame.get_features_in_area(uv(0),uv(1), radius, nLastOctave, -1,true);
    //                 else if(bBackward)
    //                     vIndices2 = current_frame.get_features_in_area(uv(0),uv(1), radius, 0, nLastOctave, true);
    //                 else
    //                     vIndices2 = current_frame.get_features_in_area(uv(0),uv(1), radius, nLastOctave-1, nLastOctave+1, true);

    //                 const cv::Mat dMP = pMP->GetDescriptor();

    //                 int bestDist = 256;
    //                 int bestIdx2 = -1;

    //                 for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
    //                 {
    //                     const size_t i2 = *vit;
    //                     if(current_frame.mappoint_matches[i2 + current_frame.Nleft])
    //                         if(current_frame.mappoint_matches[i2 + current_frame.Nleft]->Observations()>0)
    //                             continue;

    //                     const cv::Mat &d = current_frame.mDescriptors.row(i2 + current_frame.Nleft);

    //                     const int dist = DescriptorDistance(dMP,d);

    //                     if(dist<bestDist)
    //                     {
    //                         bestDist=dist;
    //                         bestIdx2=i2;
    //                     }
    //                 }

    //                 if(bestDist<=TH_HIGH)
    //                 {
    //                     current_frame.mappoint_matches[bestIdx2 + current_frame.Nleft]=pMP;
    //                     nmatches++;
    //                     if(mbCheckOrientation)
    //                     {
    //                         cv::KeyPoint kpLF = (LastFrame.Nleft == -1) ? LastFrame.mvKeysUn[i]
    //                                                                     : (i < LastFrame.Nleft) ? LastFrame.mvKeys[i]
    //                                                                                             : LastFrame.mvKeysRight[i - LastFrame.Nleft];

    //                         cv::KeyPoint kpCF = current_frame.mvKeysRight[bestIdx2];

    //                         float rot = kpLF.angle-kpCF.angle;
    //                         if(rot<0.0)
    //                             rot+=360.0f;
    //                         int bin = round(rot*factor);
    //                         if(bin==HISTO_LENGTH)
    //                             bin=0;
    //                         assert(bin>=0 && bin<HISTO_LENGTH);
    //                         rotHist[bin].push_back(bestIdx2  + current_frame.Nleft);

            }
    }

    //Apply rotation consistency
    // if(mbCheckOrientation)
    // {
    //     int ind1=-1;
    //     int ind2=-1;
    //     int ind3=-1;

    //     ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

    //     for(int i=0; i<HISTO_LENGTH; i++)
    //     {
    //         if(i!=ind1 && i!=ind2 && i!=ind3)
    //         {
    //             for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
    //             {
    //                 current_frame.mappoint_matches[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
    //                 nmatches--;
    //             }
    //         }
    //     }
    // }

    // return nmatches;
    return Vec::new();
} 

// Sofiya: other searchbyprojection functions we will have to implement later:

// Search matches between Frame keypoints and projected MapPoints. Returns number of matches
// Used to track the local map (Tracking)
// int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);

// Project MapPoints seen in KeyFrame into the Frame and search matches.
// Used in relocalisation (Tracking)
// int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);

// Project MapPoints using a Similarity Transformation and search matches.
// Used in loop detection (Loop Closing)
// int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th, float ratioHamming=1.0);

// Project MapPoints using a Similarity Transformation and search matches.
// Used in Place Recognition (Loop Closing and Merging)
// int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0);

// pub fn search_by_bow_f<S: SensorType>(
//     pKF1 : &KeyFrame<S>, pKF2 : &Frame<S>, vpMatches12: &mut HashMap<u32, Id>
// ) -> i32 {
//     todo!("TODO 10/17 BINDINGS: search_by_bow");
//     //        int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
// }

// pub fn search_by_bow_kf<S: SensorType>(
//     pKF1 : &KeyFrame<S>, pKF2 : &KeyFrame<S>, vpMatches12: &mut Vec<Id>
// ) -> i32 {
    //        int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);

    // todo!("TODO 10/17 BINDINGS: search_by_bow");

    // let vKeysUn1 = &pKF1.keypoints_data.keypoints_un();
    // let vfeature_vec1 = &pKF1.feature_vec.as_ref().unwrap();
    // let vpMapPoints1 = pKF1.mappoint_matches;
    // let Descriptors1 = &pKF1.keypoints_data.descriptors();

    // let vKeysUn2 = &pKF2.keypoints_data.keypoints_un();
    // let vfeature_vec2 = &pKF2.feature_vec.as_ref().unwrap();
    // let  vpMapPoints2 = pKF2.mappoint_matches;
    // let Descriptors2 = &pKF2.keypoints_data.descriptors();

    // let mut vbMatched2 = vec![false; vpMapPoints2.len()];


    // let mut rotHist = Vec::<Vec<i64>>::new();
    // rotHist.resize(HISTO_LENGTH as usize, vec![0; 500]);

    // // vector<int> rotHist[HISTO_LENGTH];
    // // for(int i=0;i<HISTO_LENGTH;i++)
    // //     rotHist[i].reserve(500);
    // const FACTOR: f64 = 1.0 as f64 / HISTO_LENGTH as f64;

    // //const float factor = 1.0f/HISTO_LENGTH;

    // let mut nmatches = 0;
    // //int nmatches = 0;

    // let vecfeat1 = &vfeature_vec1.1;
    // let vecfeat2 = &vfeature_vec2.1;

    // let word_id1 = vecfeat1[0].last().unwrap();
    
    // let bow1 = &vfeature_vec1.0;
    // bow1.0[0];
    
    // let l1_score = vfeature_vec1.0.l1(&vfeature_vec2.0);

    // // DBoW2::FeatureVector::const_iterator f1it = vfeature_vec1.begin();
    // // DBoW2::FeatureVector::const_iterator f2it = vfeature_vec2.begin();
    // // DBoW2::FeatureVector::const_iterator f1end = vfeature_vec1.end();
    // // DBoW2::FeatureVector::const_iterator f2end = vfeature_vec2.end();


    // //f1it->first  :  vecfeat1
    // let f1it = vecfeat1.iter(); //vecfeat1.iter();
    // let f2it = vecfeat1.iter();// vecfeat2.iter();

    // //let curr_f1 = f1it.next().unwrap();
    // //let curr_f2 = f2it.next().unwrap();

    // // while curr_f1 != f1it.last().unwrap()  && curr_f2  != f2it.last().unwrap() 
    // // //while(f1it != f1end && f2it != f2end)
    // // {

    // //     if curr_f1 == curr_f2
    // //     //if(f1it->first == f2it->first)
    // //     {

    // //         for i1 in 0..curr_f1.len()

    // //         for(size_t i1=0, iend1=f1it->second.size(); i1<iend1; i1++)
    // //         {
    // //             const size_t idx1 = f1it->second[i1];
    // //             if(pKF1 -> NLeft != -1 && idx1 >= pKF1 -> mvKeysUn.size()){
    // //                 continue;
    // //             }

    // //             MapPoint* pMP1 = vpMapPoints1[idx1];
    // //             if(!pMP1)
    // //                 continue;
    // //             if(pMP1->isBad())
    // //                 continue;

    // //             const cv::Mat &d1 = Descriptors1.row(idx1);

    // //             int bestDist1=256;
    // //             int bestIdx2 =-1 ;
    // //             int bestDist2=256;

    // //             for(size_t i2=0, iend2=f2it->second.size(); i2<iend2; i2++)
    // //             {
    // //                 const size_t idx2 = f2it->second[i2];

    // //                 if(pKF2 -> NLeft != -1 && idx2 >= pKF2 -> mvKeysUn.size()){
    // //                     continue;
    // //                 }

    // //                 MapPoint* pMP2 = vpMapPoints2[idx2];

    // //                 if(vbMatched2[idx2] || !pMP2)
    // //                     continue;

    // //                 if(pMP2->isBad())
    // //                     continue;

    // //                 const cv::Mat &d2 = Descriptors2.row(idx2);

    // //                 int dist = DescriptorDistance(d1,d2);

    // //                 if(dist<bestDist1)
    // //                 {
    // //                     bestDist2=bestDist1;
    // //                     bestDist1=dist;
    // //                     bestIdx2=idx2;
    // //                 }
    // //                 else if(dist<bestDist2)
    // //                 {
    // //                     bestDist2=dist;
    // //                 }
    // //             }

    // //             if(bestDist1<TH_LOW)
    // //             {
    // //                 if(static_cast<float>(bestDist1)<mfNNratio*static_cast<float>(bestDist2))
    // //                 {
    // //                     vpMatches12[idx1]=vpMapPoints2[bestIdx2];
    // //                     vbMatched2[bestIdx2]=true;

    // //                     if(mbCheckOrientation)
    // //                     {
    // //                         float rot = vKeysUn1[idx1].angle-vKeysUn2[bestIdx2].angle;
    // //                         if(rot<0.0)
    // //                             rot+=360.0f;
    // //                         int bin = round(rot*factor);
    // //                         if(bin==HISTO_LENGTH)
    // //                             bin=0;
    // //                         assert(bin>=0 && bin<HISTO_LENGTH);
    // //                         rotHist[bin].push_back(idx1);
    // //                     }
    // //                     nmatches++;
    // //                 }
    // //             }
    // //         }

    // //         f1it++;
    // //         f2it++;
    // //     }
    // //     else if(f1it->first < f2it->first)
    // //     {
    // //         f1it = vfeature_vec1.lower_bound(f2it->first);
    // //     }
    // //     else
    // //     {
    // //         f2it = vfeature_vec2.lower_bound(f1it->first);
    // //     }
    // // }

    // // if(mbCheckOrientation)
    // // {
    // //     int ind1=-1;
    // //     int ind2=-1;
    // //     int ind3=-1;

    // //     ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

    // //     for(int i=0; i<HISTO_LENGTH; i++)
    // //     {
    // //         if(i==ind1 || i==ind2 || i==ind3)
    // //             continue;
    // //         for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
    // //         {
    // //             vpMatches12[rotHist[i][j]]=static_cast<MapPoint*>(NULL);
    // //             nmatches--;
    // //         }
    // //     }
    // // }

    // return nmatches;

// }

pub fn descriptor_distance(a : &Mat, b: &Mat) -> i32 {
    //[POSSIBLE BUG]: IMPLEMENTATION NOT VARIFIED
    let pa = 0;
    let pb = 0;

    let mut dist : i32 = 0;

    for i in 0..8
    {
        let mut v : u64 = (*a.at::<f64>(pa).unwrap() as i32 ^ *b.at::<f64>(pb).unwrap() as i32) as u64;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += ((((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24) as i32; 

    }

    // const int *pa = a.ptr<int32_t>();
    // const int *pb = b.ptr<int32_t>();

    // int dist=0;

    // for(int i=0; i<8; i++, pa++, pb++)
    // {
    //     unsigned  int v = *pa ^ *pb;
    //     v = v - ((v >> 1) & 0x55555555);
    //     v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
    //     dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    // }

    return dist;
}