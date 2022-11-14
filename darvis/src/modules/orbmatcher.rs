use std::collections::{HashMap, HashSet};
use std::convert::TryInto;
use std::pin::Pin;
use cxx::CxxVector;
use opencv::core::{Point2f, KeyPoint, CV_8U};
use opencv::prelude::*;
use crate::dvmap::keyframe::{KeyFrame, FullKeyFrame};
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
    ini_frame: &Frame,
    curr_frame: &Frame,
    prev_matched: &mut Vec<Point2f>,
    mp_matches: &mut HashMap<u32, u32>,
    window_size: i32
) -> i32 {
    todo!("Fix calling bindings");

    // Sofiya: Should we avoid making a new orb matcher each time? Is this expensive?
    // let mut matcher = dvos3binding::ffi::new_orb_matcher(48, 48, 0.0, 0.0, 600.0, 600.0,0.1,true);

    // let frame1_keypoints = *ini_frame.features.get_all_keypoints();
    // let frame1_keypoints_cxx = frame1_keypoints.into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;
    // let frame1_descriptors = *ini_frame.features.descriptors;
    // let frame1_descriptors_cxx = frame1_descriptors.into_raw() as *const dvos3binding::ffi::DVMat;

    // let frame2_keypoints = **curr_frame.features.get_all_keypoints();
    // let frame2_keypoints_cxx = frame2_keypoints.into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;
    // let frame2_descriptors = *curr_frame.features.descriptors;
    // let frame2_descriptors_cxx = frame2_descriptors.into_raw() as *const dvos3binding::ffi::DVMat;

    //     let mut grid_v1 = dvos3binding::ffi::VectorOfusize{vec: Vec::new()};
    //     let mut grid_vals = Vec::new();
    //     grid_v1.vec.append(&mut grid_vals);
    //     let mut grid_v2 = dvos3binding::ffi::VectorOfVecusize{vec:Vec::new()  };
    //     grid_v2.vec.push(grid_v1);
    //     let mut grid_v3 = dvos3binding::ffi::DVGrid{vec:Vec::new()};
    //     grid_v3.vec.push(grid_v2.clone());
    //     grid_v3.vec.push(grid_v2.clone());


    // // let mut prev_match_cv = opencv::types::VectorOfPoint2f::default();
    // let prev_matchcv = prev_matched.into_raw() as *mut CxxVector<dvos3binding::ffi::DVPoint2f>;

    // let mut matches_cv=  opencv::types::VectorOfi32::default();
    // let matchescv = matches_cv.into_raw() as *mut CxxVector<i32>;

    // unsafe {
    //     return matcher.pin_mut().SearchForInitialization_1(
    //         &*frame1_keypoints_cxx,
    //         &*frame2_keypoints_cxx, 
    //         &*frame1_descriptors_cxx,
    //         &*frame2_descriptors_cxx,
    //         &grid_v3,
    //         Pin::new_unchecked(prev_matchcv.as_mut().unwrap()), 
    //         Pin::new_unchecked(matchescv.as_mut().unwrap()),
    //         window_size
    //     );
    // }
}

pub fn compute_three_maxima(histo : &Vec<Vec<i32>> , L: i32) -> (i32, i32, i32) {
    let (mut max_1, mut max_2, mut max_3) = (0, 0, 0);
    let (mut ind_1, mut ind_2, mut ind_3) = (-1, -1, -1);

    for i in 0..L
    {
        let s = histo[i as usize].len() as i32;
        if s>max_1 
        {
            max_3=max_2;
            max_2=max_1;
            max_1=s;
            ind_3=ind_2;
            ind_2=ind_1;
            ind_1=i;
        }
        else if s>max_2 
        {
            max_3=max_2;
            max_2=s;
            ind_3=ind_2;
            ind_2=i;
        }
        else if s>max_3 
        {
            max_3=s;
            ind_3=i;
        }
    }

    if max_2< (0.1 * (max_1 as f64)) as i32
    {
        ind_2=-1;
        ind_3=-1;
    }
    else if max_3< ((0.1*(max_1 as f64)) as i32)
    {
        ind_3=-1;
    }
    (ind_1, ind_2, ind_3)
}

pub fn search_by_projection(
    frame: &Frame, mappoints: &HashSet<Id>, th: i32, far_points_threshold: f64,
    check_orientation: bool, ratio: f64
) -> i32 {
    // int ORBmatcher::SearchByProjection(Frame &F, const vector<MapPoint*> &vpMapPoints, const float th, const bool bFarPoints, const float thFarPoints)
    let far_points = far_points_threshold != 0.0;

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

pub fn search_by_bow_f(
    kf : &KeyFrame<FullKeyFrame>, frame : &Frame, check_orientation: bool, 
    ratio: f64, map: &ReadOnlyWrapper<Map>
) -> Result<(i32, HashMap<i32, Option<Id>>), Box<dyn std::error::Error>> {
    // int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
    let mut num_matches = 0;
    let mut kf_match_edits: HashMap<i32, Option<Id>> = HashMap::new();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist: Vec<Vec<i32>> = Vec::new();
    for _ in 0..HISTO_LENGTH {
        rot_hist.push(Vec::new());
    }

    fn update_bests(
        dist: i32, index: i32, best_dist: &mut (i32,i32),
        best_index: &mut i32, no_left: bool
    ) {
        if dist < best_dist.0 && no_left {
            best_dist.1 = best_dist.0;
            best_dist.0 = dist;
            *best_index = index;
        } else if dist < best_dist.1 && no_left {
            best_dist.1 = dist;
        }
    }

    fn do_check_orientation(
        kf: &KeyFrame<FullKeyFrame>, frame: &Frame, index_kf: i32,
        best_index: i32, rot_hist: &mut Vec<Vec<i32>>, factor: f32
    ) {
        let kp_kf = kf.features.get_keypoint(index_kf as usize);
        let kp_frame = frame.features.get_keypoint(best_index as usize);
        let mut rot = kp_kf.angle - kp_frame.angle;
        if rot < 0.0 {
            rot += 360.0;
        }
        let mut bin = (rot * factor).round() as i32;
        if bin == HISTO_LENGTH {
            bin = 0;
        }
        assert!(bin >= 0 && bin < HISTO_LENGTH );
        rot_hist[bin as usize].push(best_index);
    }

    for node_id_kf in kf.bow.get_feat_vec_nodes() {
        for node_id_frame in frame.bow.get_feat_vec_nodes() {
            if node_id_kf == node_id_frame {
                let indices_kf = kf.bow.get_feat_from_node(node_id_kf);
                let indices_f = frame.bow.get_feat_from_node(node_id_frame);

                for index_kf in indices_kf {
                    let mut best_dist_left = (256, 256);
                    let mut best_index_left = -1;

                    let mut best_dist_right = (256, 256);
                    let mut best_index_right = -1;

                    let descriptors_kf = kf.features.descriptors.row(index_kf)?;

                    for index_f in &indices_f {
                        if map.read().get_mappoint(&kf.get_mappoint(&index_f)).is_some() {
                            continue;
                        }
                        let descriptors_f = frame.features.descriptors.row(*index_f)?;

                        let dist = descriptor_distance(&descriptors_kf, &descriptors_f);
                        let no_left = frame.features.has_left_kp().map_or(true, |n_left| *index_f < n_left);
                        update_bests(
                            dist, *index_f as i32,
                            &mut best_dist_left, &mut best_index_left,
                            no_left
                        );
                        update_bests(
                            dist, *index_f as i32,
                            &mut best_dist_right, &mut best_index_right,
                            no_left
                        );
                    }

                    if best_dist_left.0 <= TH_LOW {
                        let map = map.read();
                        let mp_id = &kf.get_mappoint(&index_kf);

                        if (best_dist_left.0 as f64) < ratio * (best_dist_left.1 as f64) {
                            kf_match_edits.insert(best_index_left, Some(*mp_id));

                            if check_orientation {
                                do_check_orientation(
                                    kf, frame, index_kf as i32,
                                    best_index_left, &mut rot_hist, factor
                                );
                            };
                            num_matches += 1;
                        }
                        if (best_dist_right.0 as f64) < ratio * (best_dist_right.1 as f64) {
                            kf_match_edits.insert(best_index_right, Some(*mp_id));

                            if check_orientation {
                                do_check_orientation(
                                    kf, frame, index_kf as i32,
                                    best_index_right, &mut rot_hist, factor
                                );
                            };
                            num_matches += 1;
                        }
                    }
                }
            }
        }
    }

    if check_orientation {
        let (ind_1, ind_2, ind_3) = compute_three_maxima(&rot_hist,HISTO_LENGTH);
        for i in 0..HISTO_LENGTH {
            if i == ind_1 || i == ind_2 || i == ind_3 {
                continue;
            }
            for j in 0..rot_hist[i as usize].len() {
                kf_match_edits.insert(rot_hist[i as usize][j], None);
                num_matches -= 1 ;
            }
        }
    }

    return Ok((num_matches, kf_match_edits));
}

pub fn search_by_bow_kf(
    kf_1 : &KeyFrame<FullKeyFrame>, kf_2 : &KeyFrame<FullKeyFrame>, check_orientation: bool, 
    ratio: f64, map: &ReadOnlyWrapper<Map>
) -> Result<(i32, HashMap<i32, Option<Id>>), Box<dyn std::error::Error>> {
    // int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);
    // Sofiya: THis is EXTREMELY similar to the other search_by_bow, the only difference is
    // this does not check left and right matches and sets the mappoint in kf_match_edits
    // a little differently. Can we combine?
    let mut num_matches = 0;
    let mut kf_match_edits: HashMap<i32, Option<Id>> = HashMap::new();

    let keypoints_1 = kf_1.features.get_all_keypoints();
    let keypoints_2 = kf_2.features.get_all_keypoints();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist: Vec<Vec<i32>> = Vec::new();
    for _ in 0..HISTO_LENGTH {
        rot_hist.push(Vec::new());
    }

    for node_id_kf_1 in kf_1.bow.get_feat_vec_nodes() {
        for node_id_kf_2 in kf_2.bow.get_feat_vec_nodes() {
            if node_id_kf_1 == node_id_kf_2 {
                let indices_kf_1 = kf_1.bow.get_feat_from_node(node_id_kf_1);
                let indices_kf_2 = kf_2.bow.get_feat_from_node(node_id_kf_2);

                for index_kf_1 in indices_kf_1 {
                    let mut best_dist = (256, 256);
                    let mut best_index = -1;
                    let descriptors_kf_1 = kf_1.features.descriptors.row(index_kf_1)?;

                    for index_kf_2 in &indices_kf_2 {
                        if kf_2.features.has_left_kp().map_or(false, |n_left| index_kf_2 > &n_left) {
                            continue;
                        }

                        if kf_2.has_mappoint(&index_kf_2) {
                            continue;
                        }

                        let descriptors_kf_2 = kf_2.features.descriptors.row(*index_kf_2)?;
                        let dist = descriptor_distance(&descriptors_kf_1, &descriptors_kf_2);
                        if dist < best_dist.0 {
                            best_dist.1 = best_dist.0;
                            best_dist.0 = dist;
                            best_index = *index_kf_2 as i32;
                        } else if dist < best_dist.1 {
                            best_dist.1 = dist;
                        }
                    }

                    if best_dist.0 <= TH_LOW {
                        let mp_id = &kf_2.get_mappoint(&(best_index as u32));

                        if (best_dist.0 as f64) < ratio * (best_dist.1 as f64) {
                            kf_match_edits.insert(index_kf_1 as i32, Some(*mp_id)); // for Kf_1

                            if check_orientation {
                                let kp_kf = kf_1.features.get_keypoint(index_kf_1 as usize);
                                let kp_frame = kf_1.features.get_keypoint(best_index as usize);
                                let mut rot = kp_kf.angle - kp_frame.angle;
                                if rot < 0.0 {
                                    rot += 360.0;
                                }
                                let mut bin = (rot * factor).round() as i32;
                                if bin == HISTO_LENGTH {
                                    bin = 0;
                                }
                                assert!(bin >= 0 && bin < HISTO_LENGTH );
                                rot_hist[bin as usize].push(index_kf_1 as i32);
                            }
                            num_matches += 1;
                        }
                    }

                }

            }
        }
    }


    if check_orientation {
        let (ind_1, ind_2, ind_3) = compute_three_maxima(&rot_hist,HISTO_LENGTH);
        for i in 0..HISTO_LENGTH {
            if i == ind_1 || i == ind_2 || i == ind_3 {
                continue;
            }
            for j in 0..rot_hist[i as usize].len() {
                kf_match_edits.insert(rot_hist[i as usize][j], None);
                num_matches -= 1 ;
            }
        }
    }

    return Ok((num_matches, kf_match_edits));
}

pub fn descriptor_distance(a : &Mat, b: &Mat) -> i32 {
    opencv::core::norm2(
        a, b, opencv::core::NormTypes::NORM_HAMMING as i32, &Mat::default()
    ).unwrap() as i32
}