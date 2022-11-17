use std::collections::{HashMap, HashSet};
use std::f64::INFINITY;
use dvcore::config::{GLOBAL_PARAMS, Sensor};
use opencv::core::{Point2f, KeyPoint};
use std::convert::{TryInto, TryFrom};
use std::pin::Pin;
use cxx::CxxVector;
use log::info;
use opencv::features2d::BFMatcher;
use opencv::prelude::*;
use crate::actors::tracking_backend::TrackedMapPointData;
use opencv::types::{VectorOfKeyPoint, VectorOfDMatch};
use crate::dvmap::keyframe::{KeyFrame, FullKeyFrame};
use crate::registered_modules::MATCHER;
use crate::{
    dvmap::{frame::Frame, map::Id, map::Map},
    lockwrap::ReadOnlyWrapper,
};

use super::camera::CAMERA;


const  TH_HIGH: i32= 100;
const  TH_LOW: i32 = 50;
const  HISTO_LENGTH: i32 = 30;

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel


fn match_frames(
    ini_frame: &Frame,
    curr_frame: &Frame,
    prev_matched: &mut Vec<Point2f>,
    mp_matches: &mut HashMap<u32, u32>,
) -> i32
{
    let frame1_keypoints = ini_frame.features.get_all_keypoints();
    info!("{}", frame1_keypoints.len());
    let frame1_descriptors = &*ini_frame.features.descriptors; 
    let frame2_keypoints = curr_frame.features.get_all_keypoints();


    let frame2_descriptors = &*curr_frame.features.descriptors;
 


    let mut pt_indx2 = opencv::types::VectorOfi32::new();
    let mut points2 = opencv::types::VectorOfPoint2f::new();
    KeyPoint::convert(&frame2_keypoints, &mut points2, &pt_indx2).unwrap();  
    
    mp_matches.clear();
    //////////////////////////////////////////////

      // BFMatcher to get good matches
      let bfmtch = BFMatcher::create(6 , true).unwrap(); 
      let mut mask = Mat::default(); 
      let mut matches = VectorOfDMatch::new();
      bfmtch.train_match(&frame2_descriptors, &frame1_descriptors, &mut matches, &mut mask).unwrap(); 

      // Sort the matches based on the distance in ascending order
      // Using O(n^2) sort here. Need to make the code use cv sort function
      // by providing custom comparator

      let mut sorted_matches = VectorOfDMatch::new();
      let mut added = vec![false; matches.len()];
      for i in 0.. matches.len() {
          if added[i] == true {
              continue;
          }
          let mut mn = i;
          let mut dist = matches.get(i).unwrap().distance;
          for j in 0..matches.len() {
              let dmatch2 = matches.get(j).unwrap();
              if dist > dmatch2.distance && !added[j] {
                  mn = j;
                  dist = dmatch2.distance;
              }        
          }      
          let dmatch1 = matches.get(mn).unwrap();
          sorted_matches.push(dmatch1);
          added[mn] = true;
      }




      prev_matched.clear();

      for i in 0..sorted_matches.len(){

        let pindex1: i32 = sorted_matches.get(i).unwrap().train_idx.try_into().unwrap();
        let pindex2: i32 = sorted_matches.get(i).unwrap().query_idx.try_into().unwrap();

        mp_matches.insert(pindex1.try_into().unwrap(), pindex2.try_into().unwrap());

        prev_matched.push(points2.get(sorted_matches.get(i).unwrap().query_idx.try_into().unwrap() ).unwrap());

        

      }
    ////////////////////////////////////////////
    let nmatches = sorted_matches.len() as i32;
    info!("nmatches : {}  ......., hashmap: {}", nmatches, mp_matches.len());
    nmatches
}

pub fn search_for_initialization(
    ini_frame: &Frame,
    curr_frame: &Frame,
    prev_matched: &mut Vec<Point2f>,
    mp_matches: &mut HashMap<u32, u32>,
    window_size: i32
) -> i32 {

    // Pranay: for now using BFMatcher to frame matching, as ORBMatcher API seems dependent on ORBExtractor.
    return match_frames(
        ini_frame,
        curr_frame,
        prev_matched,
        mp_matches
    );

    //todo!("Fix calling bindings");

    // Sofiya: Should we avoid making a new orb matcher each time? Is this expensive?
    let mut matcher = dvos3binding::ffi::new_orb_matcher(48, 48, 0.0, 0.0, 600.0, 600.0,0.1,true);

    let frame1_keypoints = ini_frame.features.get_all_keypoints();
    info!("{}", frame1_keypoints.len());

    let frame1_keypoints_cxx = frame1_keypoints.as_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;

    info!("{}", frame1_keypoints.len());
    let frame1_descriptors = ini_frame.features.descriptors.clone();
    let frame1_descriptors_cxx = frame1_descriptors.as_raw() as *const dvos3binding::ffi::DVMat;

    let frame2_keypoints = curr_frame.features.get_all_keypoints();
    let frame2_keypoints_cxx = frame2_keypoints.as_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;



    info!("{}", frame2_keypoints.len());
    let frame2_descriptors = &*curr_frame.features.descriptors;
    let frame2_descriptors_cxx = frame2_descriptors.clone().into_raw() as *const dvos3binding::ffi::DVMat;



    let grid_dv = curr_frame.features.grid.grid.clone();

    let mut grid_v3: dvos3binding::ffi::DVGrid = curr_frame.features.grid.clone().into();


    // let mut grid_v3 = dvos3binding::ffi::DVGrid{vec:Vec::new()};
    // unsafe{
    //     grid_v3.vec = std::mem::transmute(grid_dv);

    // }



    let mut prev_match_cv = opencv::types::VectorOfPoint2f::default();

    //Pranay: try find way to pass match without cloning
    for i in 0..prev_matched.len()
    {
        prev_match_cv.push(prev_matched.get(i).unwrap().clone());
    }
    info!("prev_match_cv: {}, prev_matched : {}", prev_match_cv.len(), prev_matched.len());
    let prev_matchcv = prev_match_cv.into_raw() as *mut CxxVector<dvos3binding::ffi::DVPoint2f>;

    let mut matches_cv=  opencv::types::VectorOfi32::default();
    let matchescv = matches_cv.into_raw() as *mut CxxVector<i32>;

    unsafe {
        let matches = matcher.pin_mut().SearchForInitialization_1(
            &*frame1_keypoints_cxx,
            &*frame2_keypoints_cxx, 
            &*frame1_descriptors_cxx,
            &*frame2_descriptors_cxx,
            &grid_v3,
            Pin::new_unchecked(prev_matchcv.as_mut().unwrap()), 
            Pin::new_unchecked(matchescv.as_mut().unwrap()),
            window_size
        );
        info!("new matches: {}", matches);
        return matches;

    }

    
}

pub fn search_by_projection_mappoints(
    frame: &mut Frame, mappoints: &HashSet<Id>, th: i32,
    check_orientation: bool, ratio: f64,
    track_in_view: &HashMap<Id, TrackedMapPointData>, track_in_view_right: &HashMap<Id, TrackedMapPointData>, 
    map: &ReadOnlyWrapper<Map>
) -> Result<i32, Box<dyn std::error::Error>> {
    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    // int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);
    let fpt = GLOBAL_PARAMS.get::<f64>(MATCHER, "far_points_threshold");
    let far_points_th = if fpt == 0.0 { INFINITY } else { fpt };
    let mut num_matches = 0;

    for mp_id in mappoints {
        let map = map.read();
        let mp = map.get_mappoint(mp_id).unwrap();
        if let Some(mp_data) = track_in_view.get(mp_id) {
            if mp_data.track_depth > far_points_th {
                continue;
            }

            // The size of the window will depend on the viewing direction
            let mut r = radius_by_viewing_cos(mp_data.view_cos);
            if th != 1 { r *= th as f64; }

            let indices = frame.get_features_in_area(
                &mp_data.proj_x,
                &mp_data.proj_y,
                r * (frame.scale_factors[mp_data.predicted_level as usize] as f64),
                mp_data.predicted_level-1,
                mp_data.predicted_level
            );

            if !indices.is_empty() {
                let best_descriptor = mp.get_best_descriptor();

                let mut best_dist = (256, 256);
                let mut best_level = (-1, -1);
                let mut best_idx: i32 = -1;

                // Get best and second matches with near keypoints
                for idx in indices {
                    if let Some((id, _)) = frame.mappoint_matches.get(&idx) {
                        if map.get_mappoint(id).unwrap().get_observations().len() > 0 {
                            continue;
                        }
                    }

                    // TODO (Stereo)
                    // if(F.Nleft == -1 && F.mvuRight[idx]>0)
                    // {
                    //     const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                    //     if(er>r*F.mvScaleFactors[nPredictedLevel])
                    //         continue;
                    // }

                    let descriptors = frame.features.descriptors.row(idx).unwrap();
                    let dist = descriptor_distance(&best_descriptor, &descriptors);

                    if dist < best_dist.0 {
                        best_dist.1 = best_dist.0;
                        best_dist.0 = dist;
                        best_level.1 = best_level.0;
                        best_level.0 = frame.features.get_octave(idx as usize);
                        best_idx = idx as i32;
                    } else if dist < best_dist.1 {
                        best_level.1 = frame.features.get_octave(idx as usize);
                        best_dist.1 = dist;
                    }
                }

                // Apply ratio to second match (only if best and second are in the same scale level)
                if best_dist.0 <= TH_HIGH {
                    if best_level.0 == best_level.1 && (best_dist.0 as f64) > (ratio * best_dist.1 as f64) {
                        continue;
                    }
                    if best_level.0 != best_level.1 || (best_dist.0 as f64) <= (ratio * best_dist.1 as f64) {
                        frame.add_mappoint(best_idx as u32, *mp_id, false);

                        // TODO (Stereo)
                        // if(F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                        //     F.mvpMapPoints[F.mvLeftToRightMatch[bestIdx] + F.Nleft] = pMP;
                        //     nmatches++;
                        //     right++;
                        // }

                        num_matches += 1;
                    }
                }
            }
        }

        // TODO (Stereo) Basically repeated code above with some small changes to which functions they call
        // if(F.Nleft != -1 && pMP->mbTrackInViewR){
        //     const int &nPredictedLevel = pMP->mnTrackScaleLevelR;
        //     if(nPredictedLevel != -1){
        //         float r = RadiusByViewingCos(pMP->mTrackViewCosR);

        //         const vector<size_t> vIndices =
        //                 F.GetFeaturesInArea(pMP->mTrackProjXR,pMP->mTrackProjYR,r*F.mvScaleFactors[nPredictedLevel],nPredictedLevel-1,nPredictedLevel,true);

        //         if(vIndices.empty())
        //             continue;

        //         const cv::Mat MPdescriptor = pMP->GetDescriptor();

        //         int bestDist=256;
        //         int bestLevel= -1;
        //         int bestDist2=256;
        //         int bestLevel2 = -1;
        //         int bestIdx =-1 ;

        //         // Get best and second matches with near keypoints
        //         for(vector<size_t>::const_iterator vit=vIndices.begin(), vend=vIndices.end(); vit!=vend; vit++)
        //         {
        //             const size_t idx = *vit;

        //             if(F.mvpMapPoints[idx + F.Nleft])
        //                 if(F.mvpMapPoints[idx + F.Nleft]->Observations()>0)
        //                     continue;


        //             const cv::Mat &d = F.mDescriptors.row(idx + F.Nleft);

        //             const int dist = DescriptorDistance(MPdescriptor,d);

        //             if(dist<bestDist)
        //             {
        //                 bestDist2=bestDist;
        //                 bestDist=dist;
        //                 bestLevel2 = bestLevel;
        //                 bestLevel = F.mvKeysRight[idx].octave;
        //                 bestIdx=idx;
        //             }
        //             else if(dist<bestDist2)
        //             {
        //                 bestLevel2 = F.mvKeysRight[idx].octave;
        //                 bestDist2=dist;
        //             }
        //         }

        //         // Apply ratio to second match (only if best and second are in the same scale level)
        //         if(bestDist<=TH_HIGH)
        //         {
        //             if(bestLevel==bestLevel2 && bestDist>mfNNratio*bestDist2)
        //                 continue;

        //             if(F.Nleft != -1 && F.mvRightToLeftMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
        //                 F.mvpMapPoints[F.mvRightToLeftMatch[bestIdx]] = pMP;
        //                 nmatches++;
        //                 left++;
        //             }


        //             F.mvpMapPoints[bestIdx + F.Nleft]=pMP;
        //             nmatches++;
        //             right++;
        //         }
        //     }
        // }
    }

    return Ok(num_matches);
}

pub fn search_by_projection_frame (
    current_frame: &mut Frame, last_frame: &Frame, th: i32,
    should_check_orientation: bool, ratio: f64,
    track_in_view: &HashMap<Id, TrackedMapPointData>, track_in_view_right: &HashMap<Id, TrackedMapPointData>,
    map: &ReadOnlyWrapper<Map>, sensor: Sensor
) -> Result<i32, Box<dyn std::error::Error>> {
    // int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)
    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)
    // TODO (Stereo): This function should work for stereo as well as RGBD as long as get_all_keypoints() returns
    // a concatenated list of left keypoints and right keypoints (there is a to do for this in features.rs). BUT
    // double check that the descriptors works correctly. Instead of splitting descriptors into discriptors_left and right,
    // I have all of them in one long vec. I THINK this should be fine, but double check.
    let mut num_matches = 0;
    let mut kf_match_edits: HashMap<i32, Option<Id>> = HashMap::new();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist = construct_rotation_histogram();

    // Note: this code copied from ORBSLAM2, not ORBSLAM3
    // ORBSLAM2 : https://github.com/raulmur/ORB_SLAM2/blob/master/src/ORBmatcher.cc#L1328
    // ORBSLAM3 : https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/ORBmatcher.cc#L1676
    // I think it is doing the same thing, just that orbslam3 uses Sophus SE3 objects
    // and ORBSLAM2 uses matrices
    // But, copying the matrices is more straightforward than figuring out what Sophus::SE3
    // is doing, especially around the matrix multiplications
    let rcw = current_frame.pose.unwrap().get_rotation();
    let tcw = current_frame.pose.unwrap().get_translation();
    let twc = -rcw.transpose() * (*tcw);

    let rlw = last_frame.pose.unwrap().get_rotation();
    let tlw = current_frame.pose.unwrap().get_translation();
    let tlc = (*rlw) * twc + (*tlw);

    let forward = tlc[2] > current_frame.stereo_baseline && !sensor.is_mono();
    let backward = -tlc[2] > current_frame.stereo_baseline && !sensor.is_mono();

    for idx1 in 0..last_frame.features.get_all_keypoints().len() {
        if last_frame.mappoint_matches.contains_key(&(idx1 as u32)) && !last_frame.is_mp_outlier(&(idx1 as u32)) {
            // Project
            let map = map.read();
            let mp_id = &last_frame.mappoint_matches.get(&(idx1 as u32)).unwrap().0;
            let mappoint = map.get_mappoint(mp_id).unwrap();
            let x_3d_w = mappoint.position;
            let x_3d_c = (*rcw) * (*x_3d_w) + (*tcw);

            let xc = x_3d_c[0];
            let yc = x_3d_c[1];
            let invzc = 1.0 / x_3d_c[2];
            if invzc < 0.0 { continue; }

            let u = CAMERA.get_fx() * xc * invzc + CAMERA.get_cx();
            let v = CAMERA.get_fy() * yc * invzc + CAMERA.get_cy();
            if !current_frame.features.image_bounds.check_bounds(u, v) {
                continue;
            }

            let last_octave = last_frame.features.get_octave(idx1 as usize);

            // Search in a window. Size depends on scale
            let radius = ((th as f32) * current_frame.scale_factors[last_octave as usize]) as f64;

            let indices_2 =
                if forward {
                    current_frame.get_features_in_area(&u,&v, radius, last_octave, -1)
                } else if backward {
                    current_frame.get_features_in_area(&u,&v, radius, 0, last_octave)
                } else {
                    current_frame.get_features_in_area(&u,&v, radius, last_octave-1, last_octave+1)
                };
            if indices_2.is_empty() {
                continue;
            }

            let best_descriptor = mappoint.get_best_descriptor();
            let mut best_dist = 256;
            let mut best_idx: i32 = -1;

            for idx2 in indices_2 {
                if let Some((id, _)) = current_frame.mappoint_matches.get(&idx2) {
                    if map.get_mappoint(id).unwrap().get_observations().len() > 0 {
                        continue;
                    }
                }

                // TODO (need?): Not sure what this is doing
                // Nleft == -1 if the left camera has no extracted keypoints which would happen in the
                // non-stereo case. But mvuRight is > 0 ONLY in the stereo case! So is this ever true?
                // if(CurrentFrame.Nleft == -1 && CurrentFrame.mvuRight[i2]>0)
                // {
                //     const float ur = uv(0) - CurrentFrame.mbf*invzc;
                //     const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                //     if(er>radius)
                //         continue;
                // }

                let descriptor = current_frame.features.descriptors.row(idx2)?;
                let dist = descriptor_distance(best_descriptor, &descriptor);
                if dist < best_dist {
                    best_dist = dist;
                    best_idx = idx2 as i32;
                }
            }

            if best_dist <= TH_HIGH {
                current_frame.add_mappoint(best_idx as u32, *mp_id, false);

                if should_check_orientation {
                    check_orientation_1(
                        &last_frame.features.get_keypoint(idx1 as usize),
                        &current_frame.features.get_keypoint(best_idx as usize),
                        &mut rot_hist, factor, best_idx
                    );
                }
                num_matches += 1;
            }
        }
    }

    // Apply rotation consistency
    if should_check_orientation {
        check_orientation_2(&rot_hist, &mut kf_match_edits, &mut num_matches)
    };

    return Ok(num_matches);
} 

pub fn search_by_projection_kf() {
    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    // int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);
}

pub fn search_by_projection_similarity() {
    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection (Loop Closing)
    // int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th, float ratioHamming=1.0);
}

pub fn search_by_projection_full() {
    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in Place Recognition (Loop Closing and Merging)
    // int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0);
}


pub fn search_by_bow_frame(
    kf : &KeyFrame<FullKeyFrame>, frame : &Frame, should_check_orientation: bool, 
    ratio: f64, map: &ReadOnlyWrapper<Map>
) -> Result<(i32, HashMap<i32, Option<Id>>), Box<dyn std::error::Error>> {
    // int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
    let mut num_matches = 0;
    let mut kf_match_edits: HashMap<i32, Option<Id>> = HashMap::new();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist = construct_rotation_histogram();

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

                            if should_check_orientation {
                                check_orientation_1(
                                    &kf.features.get_keypoint(index_kf as usize),
                                    &frame.features.get_keypoint(best_index_left as usize),
                                    &mut rot_hist, factor, best_index_left
                                );
                            };
                            num_matches += 1;
                        }
                        if (best_dist_right.0 as f64) < ratio * (best_dist_right.1 as f64) {
                            kf_match_edits.insert(best_index_right, Some(*mp_id));

                            if should_check_orientation {
                                check_orientation_1(
                                    &kf.features.get_keypoint(index_kf as usize),
                                    &frame.features.get_keypoint(best_index_left as usize),
                                    &mut rot_hist, factor, best_index_left
                                );
                            };
                            num_matches += 1;
                        }
                    }
                }
            }
        }
    }

    if should_check_orientation {
        check_orientation_2(&rot_hist, &mut kf_match_edits, &mut num_matches)
    };

    return Ok((num_matches, kf_match_edits));
}

pub fn search_by_bow_kf(
    kf_1 : &KeyFrame<FullKeyFrame>, kf_2 : &KeyFrame<FullKeyFrame>, should_check_orientation: bool, 
    ratio: f64, map: &ReadOnlyWrapper<Map>
) -> Result<(i32, HashMap<i32, Option<Id>>), Box<dyn std::error::Error>> {
    // int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);
    // Sofiya: THis is EXTREMELY similar to the other search_by_bow, the only difference is
    // this does not check left and right matches and sets the mappoint in kf_match_edits
    // a little differently. Can we combine?
    let mut num_matches = 0;
    let mut kf_match_edits: HashMap<i32, Option<Id>> = HashMap::new();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist = construct_rotation_histogram();

    for node_id_kf_1 in kf_1.bow.get_feat_vec_nodes() {
        for node_id_kf_2 in kf_2.bow.get_feat_vec_nodes() {
            if node_id_kf_1 != node_id_kf_2 {
                continue;
            }
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

                        if should_check_orientation {
                            check_orientation_1(
                                &kf_1.features.get_keypoint(index_kf_1 as usize),
                                &kf_2.features.get_keypoint(best_index as usize),
                                &mut rot_hist, factor, best_index
                            );
                        }
                        num_matches += 1;
                    }
                }
            }
        }
    }

    if should_check_orientation {
        check_orientation_2(&rot_hist, &mut kf_match_edits, &mut num_matches)
    };

    return Ok((num_matches, kf_match_edits));
}

pub fn descriptor_distance(a : &Mat, b: &Mat) -> i32 {
    return opencv::core::norm2(
        a, b, opencv::core::NormTypes::NORM_HAMMING as i32, &Mat::default()
    ).unwrap() as i32;
}

fn check_orientation_1(
    keypoint_1: &KeyPoint, keypoint_2: &KeyPoint,
    rot_hist: &mut Vec<Vec<i32>>, factor: f32,
    best_index: i32
) {
    let mut rot = keypoint_1.angle - keypoint_2.angle;
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

fn check_orientation_2(rot_hist: &Vec<Vec<i32>>, kf_match_edits: &mut HashMap<i32, Option<Id>>, num_matches: &mut i32) {
    let (ind_1, ind_2, ind_3) = compute_three_maxima(&rot_hist,HISTO_LENGTH);
    for i in 0..HISTO_LENGTH {
        if i == ind_1 || i == ind_2 || i == ind_3 {
            continue;
        }
        for j in 0..rot_hist[i as usize].len() {
            kf_match_edits.insert(rot_hist[i as usize][j], None);
            *num_matches -= 1 ;
        }
    }
}

fn radius_by_viewing_cos(view_cos: f64) -> f64 {
    return match view_cos > 0.998 {
        true => 2.5,
        false => 4.0
    };
}

fn construct_rotation_histogram() -> Vec<Vec<i32>> {
    // Rotation Histogram (to check rotation consistency)
    let mut rot_hist: Vec<Vec<i32>> = Vec::new();
    for _ in 0..HISTO_LENGTH {
        rot_hist.push(Vec::new());
    }
    rot_hist
}

fn compute_three_maxima(histo : &Vec<Vec<i32>> , L: i32) -> (i32, i32, i32) {
    let (mut max_1, mut max_2, mut max_3) = (0, 0, 0);
    let (mut ind_1, mut ind_2, mut ind_3) = (-1, -1, -1);

    for i in 0..L {
        let s = histo[i as usize].len() as i32;
        if s > max_1 {
            max_3=max_2;
            max_2=max_1;
            max_1=s;
            ind_3=ind_2;
            ind_2=ind_1;
            ind_1=i;
        } else if s > max_2  {
            max_3=max_2;
            max_2=s;
            ind_3=ind_2;
            ind_2=i;
        } else if s > max_3  {
            max_3=s;
            ind_3=i;
        }
    }

    if max_2 < (0.1 * (max_1 as f64)) as i32 {
        ind_2=-1;
        ind_3=-1;
    }
    else if max_3< ((0.1*(max_1 as f64)) as i32) {
        ind_3=-1;
    }
    (ind_1, ind_2, ind_3)
}