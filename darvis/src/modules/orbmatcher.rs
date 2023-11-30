use std::collections::{HashMap, HashSet};
use std::f64::INFINITY;
use std::sync::RwLockReadGuard;
use dvcore::config::{SETTINGS};
use dvcore::matrix::{DVVectorOfPoint2f, DVVector3};
use dvcore::sensor::{Sensor, FrameSensor};
use log::{debug, warn, trace};
use logging_timer::time;
use opencv::core::{KeyPoint};
use opencv::prelude::*;
use parking_lot::{MappedRwLockReadGuard};
use crate::actors::map_actor::MapWriteMsg;
use crate::actors::tracking_backend::TrackedMapPointData;
use crate::dvmap::keyframe::{Frame, FullKeyFrame};
use crate::modules::optimizer::LEVEL_SIGMA2;
use crate::registered_actors::{MATCHER, FEATURE_DETECTION, CAMERA};
use crate::{
    dvmap::{keyframe::InitialFrame, map::Id, map::Map},
    maplock::ReadOnlyMap,
};

use super::camera::CAMERA_MODULE;
use super::optimizer::INV_LEVEL_SIGMA2;

const  TH_HIGH: i32= 100;
const  TH_LOW: i32 = 50;
const  HISTO_LENGTH: i32 = 30;

lazy_static! {
    // Note: ORBSLAM3 duplicates this var at every frame and keyframe,
    // but I'm pretty sure that it's set once per-system when the ORBExtractor
    // is created and only ever used by Optimizer.
    // In general, I'd like to remove these kinds of variables away from the
    // frame/keyframe/mappoint implementation and into the object that actually
    // directly uses it.
    pub static ref SCALE_FACTORS: Vec<f32> = {
        let scale_factor = SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor");
        let n_levels = SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels");
        let mut scale_factors = vec![1.0];

        for i in 1..n_levels as usize {
            scale_factors.push(scale_factors[i-1] * (scale_factor as f32));
        }
        scale_factors
    };
}

pub fn search_for_initialization(
    f1: &Frame<InitialFrame>,
    f2: &Frame<InitialFrame>,
    vb_prev_matched: &mut DVVectorOfPoint2f,
    window_size: i32,
) -> (i32, Vec<i32>) {
    let nnratio = 0.9;
    let check_ori=true;

    let mut vn_matches12: Vec<i32> = vec![-1; f1.features.get_all_keypoints().len() as usize];
    let factor = 1.0 / HISTO_LENGTH as f32;
    let mut v_matched_distance = vec![std::i32::MAX; f2.features.get_all_keypoints().len() as usize];
    let mut vn_matches21: Vec<i32> = vec![-1; f2.features.get_all_keypoints().len() as usize];
    let mut n_matches = 0;

    let mut rot_hist = construct_rotation_histogram();

    for (i1, kp1) in f1.features.get_all_keypoints().iter().enumerate() {
        let level1 = kp1.octave();
        if level1 > 0 {
            continue;
        }

        // Pranay : could be a bug ?? get_features_in_area
        let v_indices2 = f2.get_features_in_area(
            &(vb_prev_matched.get(i1).unwrap().x as f64),
            &(vb_prev_matched.get(i1).unwrap().y as f64),
            window_size as f64,
            level1,
            level1,
        );

        if v_indices2.is_empty() {
            continue;
        }

        let d1 = f1.features.descriptors.row(i1 as u32).unwrap();
        let (mut best_dist, mut best_dist2, mut best_idx2) : (i32, i32, i32) = (std::i32::MAX, std::i32::MAX, -1);
        for i2 in v_indices2 {
            let d2 = f2.features.descriptors.row(i2).unwrap();
            let dist = descriptor_distance(&d1, &d2);
            if v_matched_distance[i2 as usize] <= dist {
                continue;
            }
            if dist < best_dist {
                best_dist2 = best_dist;
                best_dist = dist;
                best_idx2 = i2 as i32;
            } else if dist < best_dist2 {
                best_dist2 = dist;
            }
        }
        if best_dist <= TH_LOW {
            if (best_dist as f32) < (best_dist2 as f32 *  nnratio) {
                if vn_matches21[best_idx2 as usize] >= 0 {
                    vn_matches12[vn_matches21[best_idx2 as usize] as usize] = -1;
                    n_matches -= 1;
                }
                vn_matches12[i1] = best_idx2;
                vn_matches21[best_idx2 as usize] = i1 as i32;
                v_matched_distance[best_idx2 as usize] = best_dist;
                n_matches+=1;
                if check_ori {
                    check_orientation_1(
                        &f1.features.get_keypoint(i1 as usize).0,
                        &f2.features.get_keypoint(best_idx2 as usize).0,
                        &mut rot_hist, factor, i1 as u32
                    );
                }
            }
        }
    }

    if check_ori {
        let (ind_1, ind_2, ind_3) = compute_three_maxima(&rot_hist,HISTO_LENGTH);
        for i in 0..HISTO_LENGTH {
            if i == ind_1 as i32 || i == ind_2 as i32 || i == ind_3 as i32 {
                continue;
            }
            for j in 0..rot_hist[i as usize].len() {
                let key = rot_hist[i as usize][j];
                if vn_matches12[key as usize] >= 0 {
                    vn_matches12[key as usize] = -1;
                    n_matches -= 1;
                }
            }
        }
    }

    for (i1, match12) in vn_matches12.iter().enumerate() {
        if *match12 >= 0 {
            *vb_prev_matched.get(i1).as_mut().unwrap() = f2.features.get_keypoint(*match12 as usize).0.pt();
        }
    }

    (n_matches, vn_matches12)
}

#[time()]
pub fn search_by_projection(
    frame: &mut Frame<InitialFrame>, mappoints: &HashSet<Id>, th: i32, ratio: f64,
    track_in_view: &HashMap<Id, TrackedMapPointData>, track_in_view_right: &HashMap<Id, TrackedMapPointData>, 
    map: &ReadOnlyMap<Map>, sensor: Sensor
) -> Result<i32, Box<dyn std::error::Error>> {
    // int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);
    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    let fpt = SETTINGS.get::<f64>(MATCHER, "far_points_threshold");
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
                r * (SCALE_FACTORS[mp_data.predicted_level as usize] as f64),
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

                    match sensor.frame() {
                        FrameSensor::Stereo => {
                            todo!("Stereo")
                            // if(F.Nleft == -1 && F.mvuRight[idx]>0)
                            // {
                            //     const float er = fabs(pMP->mTrackProjXR-F.mvuRight[idx]);
                            //     if(er>r*F.mvScaleFactors[nPredictedLevel])
                            //         continue;
                            // }
                        },
                        _ => {}
                    }

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

                        match sensor.frame() {
                            FrameSensor::Stereo => {
                                todo!("Stereo")
                                // if(F.Nleft != -1 && F.mvLeftToRightMatch[bestIdx] != -1){ //Also match with the stereo observation at right camera
                                //     F.mvpMapPoints[F.mvLeftToRightMatch[bestIdx] + F.Nleft] = pMP;
                                //     nmatches++;
                                //     right++;
                                // }
                            },
                            _ => {}
                        }

                        num_matches += 1;
                    }
                }
            }
        }

        if let Some(mp_data) = track_in_view_right.get(mp_id) {
            todo!("Stereo");
            // Basically repeated code above with some small changes to which functions they call
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
            //                 bestLevel = F.mvKeysRight[idx].octave();
            //                 bestIdx=idx;
            //             }
            //             else if(dist<bestDist2)
            //             {
            //                 bestLevel2 = F.mvKeysRight[idx].octave();
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
    }

    return Ok(num_matches);
}

// Project MapPoints tracked in last frame into the current frame and search matches.
// Used to track from previous frame (Tracking)

#[time()]
pub fn search_by_projection_with_threshold (
    current_frame: &mut Frame<InitialFrame>, last_frame: &Frame<InitialFrame>, th: i32,
    should_check_orientation: bool,
    map: &ReadOnlyMap<Map>, sensor: Sensor
) -> Result<i32, Box<dyn std::error::Error>> {
    // int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)

    // TODO STEREO...search_by_projection_with_threshold...
    // This function should work for stereo as well as RGBD as long as get_all_keypoints() returns
    // a concatenated list of left keypoints and right keypoints (there is a to do for this in features.rs). BUT
    // double check that the descriptors works correctly. Instead of splitting descriptors into discriptors_left and right,
    // I have all of them in one long vec. I THINK this should be fine, but double check.
    let mut num_matches = 0;
    let mut matches = HashMap::<u32, Id>::new();

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

    let forward = tlc[2] > CAMERA_MODULE.stereo_baseline && !sensor.is_mono();
    let backward = -tlc[2] > CAMERA_MODULE.stereo_baseline && !sensor.is_mono();

    // debug!("search by projection keypoints {:?}", last_frame.features.get_all_keypoints().len());
    // debug!("search by projection mappoint matches {:?}", last_frame.mappoint_matches);

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

            let u = CAMERA_MODULE.get_fx() * xc * invzc + CAMERA_MODULE.get_cx();
            let v = CAMERA_MODULE.get_fy() * yc * invzc + CAMERA_MODULE.get_cy();
            if !current_frame.features.image_bounds.check_bounds(u, v) {
                continue;
            }

            let last_octave = last_frame.features.get_octave(idx1 as usize);

            // Search in a window. Size depends on scale
            let radius = ((th as f32) * SCALE_FACTORS[last_octave as usize]) as f64;

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

                // TODO Stereo: Unfinished code, not sure what this is doing in search_by_projection_with_threshold
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
                        &last_frame.features.get_keypoint(idx1 as usize).0,
                        &current_frame.features.get_keypoint(best_idx as usize).0,
                        &mut rot_hist, factor, best_idx as u32
                    );
                }
                num_matches += 1;
            }
        }
    }

    // Apply rotation consistency
    if should_check_orientation {
        check_orientation_2(&rot_hist, &mut matches)
    };

    return Ok(num_matches);
} 

// TODO (mvp): other searchbyprojection functions we will have to implement later:

pub fn _search_by_projection_reloc (
    _current_frame: &mut Frame<InitialFrame>, _keyframe: &Frame<FullKeyFrame>,
    _th: i32, _should_check_orientation: bool, _ratio: f64,
    _track_in_view: &HashMap<Id, TrackedMapPointData>, _track_in_view_right: &HashMap<Id, TrackedMapPointData>,
    _map: &ReadOnlyMap<Map>, _sensor: Sensor
) -> Result<i32, Box<dyn std::error::Error>> {
    // int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);
    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    todo!("Relocalization");
}

// Project MapPoints using a Similarity Transformation and search matches.
// Used in loop detection (Loop Closing)
// int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, std::vector<MapPoint*> &vpMatched, int th, float ratioHamming=1.0);

// Project MapPoints using a Similarity Transformation and search matches.
// Used in Place Recognition (Loop Closing and Merging)
// int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0);

#[time()]
pub fn search_by_bow_f(
    kf: &Frame<FullKeyFrame>, frame: &mut Frame<InitialFrame>,
    should_check_orientation: bool, ratio: f64
) -> Result<(u32, Vec<(Id, u32)>), Box<dyn std::error::Error>> {
    // int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
    frame.clear_mappoints();
    let mut matches = HashMap::<u32, Id>::new();

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

    let kf_featvec = kf.bow.as_ref().unwrap().get_feat_vec_nodes();
    let frame_featvec = frame.bow.as_ref().unwrap().get_feat_vec_nodes();
    let mut i = 0;
    let mut j = 0;

    while i < kf_featvec.len() && j < frame_featvec.len() {
        let kf_node_id = kf_featvec[i];
        let frame_node_id = frame_featvec[j];
        if kf_node_id == frame_node_id {
            let kf_indices = kf.bow.as_ref().unwrap().get_feat_from_node(kf_node_id);

            for kf_index in kf_indices {
                if !kf.mappoint_matches.contains_key(&kf_index) {
                    continue
                }
                let mp_id = &kf.get_mappoint(&kf_index);

                let mut best_dist_left = (256, 256);
                let mut best_index_left = -1;
                let mut best_dist_right = (256, 256);
                let mut best_index_right = -1;
                let descriptors_kf = kf.features.descriptors.row(kf_index)?;

                let frame_indices = frame.bow.as_ref().unwrap().get_feat_from_node(frame_node_id);

                for frame_index in &frame_indices {
                    if matches.contains_key(&frame_index) {
                        continue;
                    }

                    let descriptors_f = frame.features.descriptors.row(*frame_index)?;

                    let dist = descriptor_distance(&descriptors_kf, &descriptors_f);
                    let no_left = frame.features.has_left_kp().map_or(true, |n_left| *frame_index < n_left);
                    // TODO (stereo): I'm not sure if below works in the stereo case
                    update_bests(
                        dist, *frame_index as i32,
                        &mut best_dist_left, &mut best_index_left,
                        no_left
                    );
                    update_bests(
                        dist, *frame_index as i32,
                        &mut best_dist_right, &mut best_index_right,
                        no_left
                    );
                }

                if best_dist_left.0 <= TH_LOW {
                    if (best_dist_left.0 as f64) < ratio * (best_dist_left.1 as f64) {
                        matches.insert(best_index_left as u32, *mp_id);

                        if should_check_orientation {
                            check_orientation_1(
                                &kf.features.get_keypoint(kf_index as usize).0,
                                &frame.features.get_keypoint(best_index_left as usize).0,
                                &mut rot_hist, factor, best_index_left as u32
                            );
                        };
                    }
                    if (best_dist_right.0 as f64) < ratio * (best_dist_right.1 as f64) {
                        matches.insert(best_index_right as u32, *mp_id);

                        if should_check_orientation {
                            check_orientation_1(
                                &kf.features.get_keypoint(kf_index as usize).0,
                                &frame.features.get_keypoint(best_index_right as usize).0,
                                &mut rot_hist, factor, best_index_right as u32
                            );
                        };
                    }
                }
            }
            i += 1;
            j += 1;
        } else if kf_node_id < frame_node_id {
            i = lower_bound(&kf_featvec, &frame_featvec, i, j);
        } else {
            j = lower_bound(&frame_featvec, &kf_featvec, j, i);
        }
    }
    let pre = matches.len();
    if should_check_orientation {
        check_orientation_2(&rot_hist, &mut matches)
    };

    let mut increase_found = vec![];
    for (index, mp_id) in &matches {
        frame.add_mappoint(*index, *mp_id, false);
        increase_found.push((*mp_id, 1));
    }

    return Ok((matches.len() as u32, increase_found));
}

pub fn search_by_bow_kf(
    kf_1 : &Frame<FullKeyFrame>, kf_2 : &Frame<FullKeyFrame>, should_check_orientation: bool, 
    ratio: f64
) -> Result<HashMap<u32, Id>, Box<dyn std::error::Error>> {
    // int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);
    let mut matches = HashMap::<u32, Id>::new();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist = construct_rotation_histogram();

    for node_id_kf_1 in kf_1.bow.as_ref().unwrap().get_feat_vec_nodes() {
        for node_id_kf_2 in kf_2.bow.as_ref().unwrap().get_feat_vec_nodes() {
            if node_id_kf_1 == node_id_kf_2 {
                let indices_kf_1 = kf_1.bow.as_ref().unwrap().get_feat_from_node(node_id_kf_1);
                let indices_kf_2 = kf_2.bow.as_ref().unwrap().get_feat_from_node(node_id_kf_2);

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
                            matches.insert(index_kf_1, *mp_id); // for Kf_1

                            if should_check_orientation {
                                check_orientation_1(
                                    &kf_1.features.get_keypoint(index_kf_1 as usize).0,
                                    &kf_2.features.get_keypoint(best_index as usize).0,
                                    &mut rot_hist, factor, index_kf_1
                                );
                            }
                        }
                    }

                }

            }
        }
    }

    if should_check_orientation {
        check_orientation_2(&rot_hist, &mut matches)
    };

    return Ok(matches);
}

pub fn search_for_triangulation(
    kf_1 : &Frame<FullKeyFrame>, kf_2 : &Frame<FullKeyFrame>,
    should_check_orientation: bool, _only_stereo: bool, course: bool,
    sensor: Sensor
) -> Result<HashMap<usize, usize>, Box<dyn std::error::Error>> {
    // Testing local mapping ... this function return slightly different results than ORB-SLAM. Could be because the optimized pose is slightly different but could also be an error. Uncomment the println lines (and same lines in ORB-SLAM3), then compare the strings.
    //int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse)
    // Some math based on ORB-SLAM2 to avoid some confusion with Sophus.
    // Look here: https://github.com/raulmur/ORB_SLAM2/blob/master/src/ORBmatcher.cc#L657

    //Compute epipole in second image
    let cw = *kf_1.get_camera_center(); //Cw
    let r2w = *kf_2.pose.unwrap().get_rotation(); //R2w
    let t2w = *kf_2.pose.unwrap().get_translation(); //t2w
    let c2 = r2w * cw + t2w; //C2

    // let pose1 = kf_1.pose.unwrap(); // T1w
    // let pose2 = kf_2.pose.unwrap(); // T2w
    // let translation_inverse_2 = kf_2.pose.unwrap().inverse(); // Tw2
    // let cw = *kf_1.get_camera_center(); // Cw
    // let temp = *translation_2.get_translation();
    // let c2 = temp.component_mul(&cw); // c2

    let ep = CAMERA_MODULE.project(DVVector3::new(c2));

    let (r12, t12);
    if matches!(sensor.frame(), FrameSensor::Stereo) {
            todo!("Stereo");
            // Sophus::SE3f Tr1w = pKF1->GetRightPose();
            // Sophus::SE3f Twr2 = pKF2->GetRightPoseInverse();
            // Tll = T1w * Tw2;
            // Tlr = T1w * Twr2;
            // Trl = Tr1w * Tw2;
            // Trr = Tr1w * Twr2;
            // Eigen::Matrix3f Rll = Tll.rotationMatrix(), Rlr  = Tlr.rotationMatrix(), Rrl  = Trl.rotationMatrix(), Rrr  = Trr.rotationMatrix();
            // Eigen::Vector3f tll = Tll.translation(), tlr = Tlr.translation(), trl = Trl.translation(), trr = Trr.translation();
    } else {
        let pose12 = kf_1.pose.unwrap() * kf_2.pose.unwrap().inverse(); // T12 ... which is different than t12
        r12 = pose12.get_rotation();
        t12 = pose12.get_translation();
    }

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node

    let mut matches = HashMap::<usize, usize>::new();
    let matched_already = HashMap::<u32, u32>::new();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist = construct_rotation_histogram();

    let kf1_featvec = kf_1.bow.as_ref().unwrap().get_feat_vec_nodes();
    let kf2_featvec = kf_2.bow.as_ref().unwrap().get_feat_vec_nodes();
    let mut i = 0;
    let mut j = 0;

    while i < kf1_featvec.len() && j < kf2_featvec.len() {
        let kf1_node_id = kf1_featvec[i];
        let kf2_node_id = kf2_featvec[j];
        if kf1_node_id == kf2_node_id {
            // println!("index match {}", kf1_node_id);
            let kf1_indices = kf_1.bow.as_ref().unwrap().get_feat_from_node(kf1_node_id);

            for kf1_index in kf1_indices {
                // If there is already a MapPoint skip
                if kf_1.mappoint_matches.contains_key(&kf1_index) {
                    // println!("Already mappoint at {}", kf1_index);
                    continue
                };

                let stereo1 = match sensor.frame() {
                    FrameSensor::Stereo => {
                        todo!("Stereo");
                        // let stereo1 = false; // const bool bStereo1 = (!pKF1->mpCamera2 && pKF1->mvuRight[idx1]>=0);
                        // if only_stereo && !stereo1 {
                        //     continue
                        // }
                        // stereo1
                    },
                    _ => false
                };

                let (kp1, _right1) = kf_1.features.get_keypoint(kf1_index as usize);

                let mut best_dist = TH_LOW;
                let mut best_index = -1;
                let descriptors_kf_1 = kf_1.features.descriptors.row(kf1_index)?;

                let kf2_indices = kf_2.bow.as_ref().unwrap().get_feat_from_node(kf2_node_id);
                for kf2_index in kf2_indices {
                    // If we have already matched or there is a MapPoint skip
                    if kf_2.mappoint_matches.contains_key(&kf2_index) || matched_already.contains_key(&kf2_index) {
                        continue
                    };

                    let stereo2 = match sensor.frame() {
                        FrameSensor::Stereo => {
                            todo!("Stereo");
                            // let stereo2 = false; // const bool bStereo2 = (!pKF2->mpCamera2 &&  pKF2->mvuRight[idx2]>=0);
                            // if only_stereo && !stereo2 {
                            //     continue
                            // }
                            // stereo2
                        },
                        _ => false
                    };

                    let descriptors_kf_2 = kf_2.features.descriptors.row(kf2_index)?;
                    let dist = descriptor_distance(&descriptors_kf_1, &descriptors_kf_2);

                    if dist > TH_LOW || dist > best_dist {
                        // println!("dist is > 50 or better than best dist {} {} {} ", kf2_index, dist, best_dist);
                        continue
                    }

                    // println!("dist {} {} {}", kf1_index, kf2_index, dist);

                    let (kp2, _right2) = kf_2.features.get_keypoint(kf2_index as usize);

                    if !stereo1 && !stereo2 { // && !kf1->mpCamera2 ... TODO (STEREO)
                        let dist_ex = (ep.0 as f32) - kp2.pt().x;
                        let dist_ey = (ep.1 as f32) - kp2.pt().y;
                        if dist_ex * dist_ex + dist_ey * dist_ey < 100.0 * SCALE_FACTORS[kp2.octave() as usize] {
                            // println!("continuing");
                            continue
                        }
                    }

                    // TODO (Stereo)
                    // if(pKF1->mpCamera2 && pKF2->mpCamera2){
                    //     if(bRight1 && bRight2){
                    //         R12 = Rrr;
                    //         t12 = trr;
                    //         T12 = Trr;

                    //         pCamera1 = pKF1->mpCamera2;
                    //         pCamera2 = pKF2->mpCamera2;
                    //     }
                    //     else if(bRight1 && !bRight2){
                    //         R12 = Rrl;
                    //         t12 = trl;
                    //         T12 = Trl;

                    //         pCamera1 = pKF1->mpCamera2;
                    //         pCamera2 = pKF2->mpCamera;
                    //     }
                    //     else if(!bRight1 && bRight2){
                    //         R12 = Rlr;
                    //         t12 = tlr;
                    //         T12 = Tlr;

                    //         pCamera1 = pKF1->mpCamera;
                    //         pCamera2 = pKF2->mpCamera2;
                    //     }
                    //     else{
                    //         R12 = Rll;
                    //         t12 = tll;
                    //         T12 = Tll;

                    //         pCamera1 = pKF1->mpCamera;
                    //         pCamera2 = pKF2->mpCamera;
                    //     }
                    // }

                    let epipolar = CAMERA_MODULE.epipolar_constrain(&kp1, &kp2, &r12, &t12, LEVEL_SIGMA2[kp2.octave() as usize]);

                    if course || epipolar {
                        best_index = kf2_index as i32;
                        best_dist = dist;
                    }
                }
                // println!("Best index and dist {} {}", best_index, best_dist);
                if best_index >= 0 {
                    let (kp2, _) = kf_2.features.get_keypoint(best_index as usize);
                    matches.insert(kf1_index as usize, best_index as usize);
                    if should_check_orientation {
                        check_orientation_1(
                            &kp1,
                            &kp2,
                            &mut rot_hist, factor, kf1_index
                        );

                    }
                }
            }
            i += 1;
            j += 1;
        } else if kf1_node_id < kf2_node_id {
            i = lower_bound(&kf1_featvec, &kf2_featvec, i, j);
            // println!("Move kf1 to {}", i);
        } else {
            j = lower_bound(&kf2_featvec, &kf1_featvec, j, i);
            // println!("Move kf2 to {}", j);
        }
    }

    if should_check_orientation {
        let (ind_1, ind_2, ind_3) = compute_three_maxima(&rot_hist,HISTO_LENGTH);
        for i in 0..HISTO_LENGTH {
            if i == ind_1 || i == ind_2 || i == ind_3 {
                continue;
            }
            for j in 0..rot_hist[i as usize].len() {
                let key = rot_hist[i as usize][j];
                if matches.contains_key(&(key as usize)) {
                    matches.remove(&(key as usize));
                }
            }
        }
    };

    return Ok(matches);
}

pub fn fuse(kf_id: &Id, fuse_candidates: &Vec<Id>, map: &RwLockReadGuard<Map>, th: f32, is_right: bool) -> Vec<MapWriteMsg> {
    // int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th, const bool bRight)
    let keyframe = map.get_keyframe(kf_id).unwrap();

    let (tcw, ow, _camera) = match is_right {
        true => {
            todo!("Stereo");
            // Tcw = pKF->GetRightPose();
            // Ow = pKF->GetRightCameraCenter();
            // pCamera = pKF->mpCamera2;
        },
        false => {
            (keyframe.pose.unwrap().get_translation(), keyframe.get_camera_center(), CAMERA)
        }
    };

    let mut to_fuse = Vec::new();

    for mp_id in fuse_candidates {
        let mappoint = map.get_mappoint(&mp_id).unwrap();
        if mappoint.is_in_keyframe(keyframe.id()) {
            continue;
        }

        let p_3d_w = mappoint.position;
        let p_3d_c = (*tcw).component_mul(&*p_3d_w);

        // Depth must be positive
        if p_3d_c[2] < 0.0 {
            continue;
        }

        let _inv_z = 1.0 / p_3d_c[2]; // Used by stereo below
        let uv = CAMERA_MODULE.project(DVVector3::new(p_3d_c));

        // Point must be inside the image
        if !keyframe.features.is_in_image(uv.0, uv.1) {
            continue;
        }

        let max_distance = mappoint.get_max_distance_invariance();
        let min_distance = mappoint.get_min_distance_invariance();
        let po = *p_3d_w - *ow;
        let dist_3d = po.norm();

        // Depth must be inside the scale pyramid of the image
        if dist_3d < min_distance || dist_3d > max_distance {
            continue;
        }

        // Viewing angle must be less than 60 deg
        let pn = mappoint.get_normal();
        if po.dot(&pn) < 0.5 * dist_3d {
            continue;
        }

        // Search in a radius
        let predicted_level = mappoint.predict_scale(&dist_3d);
        let radius = th * SCALE_FACTORS[predicted_level as usize];
        let indices = keyframe.get_features_in_area(&uv.0, &uv.1, radius as f64, is_right);
        if indices.is_empty() {
            continue;
        }

        // Match to the most similar keypoint in the radius
        let desc_mp = mappoint.get_best_descriptor();

        let mut best_dist = 256;
        let mut best_idx = -1;

        for idx2 in indices {
            let (kp, is_right) = keyframe.features.get_keypoint(idx2 as usize);
            let kp_level = kp.octave();

            if kp_level < predicted_level - 1 || kp_level > predicted_level {
                continue;
            }

            let (kpx, kpy, ex, ey, e2);
            if is_right {
                todo!("Stereo");
                // let ur = uv.0 - CAMERA_MODULE.stereo_baseline_times_fx * inv_z;

                // Check reprojection error in stereo
                // const float &kpx = kp.pt().x;
                // const float &kpy = kp.pt().y;
                // const float &kpr = pKF->mvuRight[idx];
                // const float ex = uv(0)-kpx;
                // const float ey = uv(1)-kpy;
                // const float er = ur-kpr;
                // const float e2 = ex*ex+ey*ey+er*er;

                // if(e2*pKF->mvInvLevelSigma2[kpLevel]>7.8)
                //     continue;
            } else {
                kpx = kp.pt().x as f64;
                kpy = kp.pt().y as f64;
                ex = uv.0 - kpx;
                ey = uv.1 - kpy;
                e2 = ex * ex + ey * ey;
                if e2 * INV_LEVEL_SIGMA2[kp_level as usize] as f64 > 5.99 {
                    continue;
                }
            }

            let desc_kf = keyframe.features.descriptors.row(idx2).unwrap();
            let dist = descriptor_distance(desc_mp, &desc_kf);

            if dist < best_dist {
                best_dist = dist;
                best_idx = idx2 as i32;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if best_dist <= TH_LOW {
            if keyframe.has_mappoint(&(best_idx as u32)) {
                let mappoint_in_kf_id = keyframe.get_mappoint(&(best_idx as u32));
                let mappoint_in_kf = map.get_mappoint(&mappoint_in_kf_id).unwrap();
                if mappoint_in_kf.get_observations().len() > mappoint.get_observations().len() {
                    warn!("Verify that the order of mp_id and mappoint_in_kf_id is right");
                    to_fuse.push(MapWriteMsg::replace_mappoint(*mp_id, mappoint_in_kf_id));
                } else {
                    to_fuse.push(MapWriteMsg::replace_mappoint(mappoint_in_kf_id, *mp_id));
                }
            } else {
                to_fuse.push(MapWriteMsg::add_observation(*kf_id, *mp_id, best_idx as usize));
            }
        }
    }
    return to_fuse;
}

pub fn descriptor_distance(a : &Mat, b: &Mat) -> i32 {
    return opencv::core::norm2(
        a, b, opencv::core::NormTypes::NORM_HAMMING as i32, &Mat::default()
    ).unwrap() as i32;
}

fn check_orientation_1(
    keypoint_1: &KeyPoint, keypoint_2: &KeyPoint,
    rot_hist: &mut Vec<Vec<u32>>, factor: f32,
    idx: u32
) {
    let mut rot = keypoint_1.angle() - keypoint_2.angle();
    if rot < 0.0 {
        rot += 360.0;
    }
    let mut bin = (rot * factor).round() as i32;
    if bin == HISTO_LENGTH {
        bin = 0;
    }
    assert!(bin >= 0 && bin < HISTO_LENGTH );
    rot_hist[bin as usize].push(idx);
}

fn check_orientation_2(rot_hist: &Vec<Vec<u32>>, matches: &mut HashMap<u32, Id>) {
    let (ind_1, ind_2, ind_3) = compute_three_maxima(&rot_hist,HISTO_LENGTH);
    for i in 0..HISTO_LENGTH {
        if i == ind_1 || i == ind_2 || i == ind_3 {
            continue;
        }
        for j in 0..rot_hist[i as usize].len() {
            let key = rot_hist[i as usize][j];
            if matches.contains_key(&key) {
                matches.remove(&key);
            }
        }
    }
}

fn radius_by_viewing_cos(view_cos: f64) -> f64 {
    return match view_cos > 0.998 {
        true => 2.5,
        false => 4.0
    };
}

fn construct_rotation_histogram() -> Vec<Vec<u32>> {
    // Rotation Histogram (to check rotation consistency)
    let mut rot_hist: Vec<Vec<u32>> = Vec::new();
    for _ in 0..HISTO_LENGTH {
        rot_hist.push(Vec::new());
    }
    rot_hist
}

fn compute_three_maxima(histo : &Vec<Vec<u32>> , histo_length: i32) -> (i32, i32, i32) {
    let (mut max_1, mut max_2, mut max_3) = (0, 0, 0);
    let (mut ind_1, mut ind_2, mut ind_3) = (-1, -1, -1);

    for i in 0..histo_length {
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

fn lower_bound(vec1: &Vec<u32>, vec2: &Vec<u32>, i: usize, j: usize) -> usize {
    let mut curr = i;
    while vec1[curr] < vec2[j] {
        curr += 1;
    }
    curr
}