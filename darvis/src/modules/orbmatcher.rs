use std::collections::{BTreeSet, HashMap, HashSet};
use std::f64::INFINITY;
use core::config::SETTINGS;
use core::matrix::{DVMatrix, DVMatrix4, DVVector3, DVVectorOfPoint2f};
use core::sensor::{Sensor, FrameSensor};
use opencv::core::KeyPoint;
use opencv::prelude::*;
use crate::MapLock;
use crate::actors::tracking_backend::TrackedMapPointData;
use crate::map::pose::{DVRotation, DVTranslation, Pose};
use crate::modules::optimizer::LEVEL_SIGMA2;
use crate::registered_actors::{MATCHER, FEATURE_DETECTION, CAMERA};
use crate::map::{map::Id, keyframe::KeyFrame, frame::Frame};

use super::camera::CAMERA_MODULE;
use super::optimizer::INV_LEVEL_SIGMA2;

const  TH_HIGH: i32= 100;
const  TH_LOW: i32 = 50;
const  HISTO_LENGTH: i32 = 30;

lazy_static! {
    // Note: ORBSLAM3 duplicates this var at every frame and keyframe.
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
    f1: &Frame,
    f2: &Frame,
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
        let v_indices2 = f2.features.get_features_in_area(
            &(vb_prev_matched.get(i1).unwrap().x as f64),
            &(vb_prev_matched.get(i1).unwrap().y as f64),
            window_size as f64,
            Some((level1, level1))
        );

        if v_indices2.is_empty() {
            continue;
        }

        let d1 = f1.features.descriptors.row(i1 as u32);
        let (mut best_dist, mut best_dist2, mut best_idx2) : (i32, i32, i32) = (std::i32::MAX, std::i32::MAX, -1);
        for i2 in v_indices2 {
            let d2 = f2.features.descriptors.row(i2);
            let dist = descriptor_distance(&d1,&d2);
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

pub fn search_by_projection(
    frame: &mut Frame, mappoints: &mut BTreeSet<Id>, th: i32, ratio: f64,
    track_in_view: &HashMap<Id, TrackedMapPointData>, track_in_view_right: &HashMap<Id, TrackedMapPointData>, 
    map: &MapLock, sensor: Sensor
) -> Result<i32, Box<dyn std::error::Error>> {
    // int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);
    // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
    // Used to track the local map (Tracking)
    let fpt = SETTINGS.get::<f64>(MATCHER, "far_points_threshold");
    let far_points_th = if fpt == 0.0 { INFINITY } else { fpt };
    let mut num_matches = 0;

    // print!("Search by projection. ");
    let map = map.read();
    let mut local_mps_to_remove = vec![];

    // println!("Search by projection input mappoints: {:?}", mappoints);

    // print!("Search by projection: ");
    for mp_id in &*mappoints {
        let mp = match map.mappoints.get(&mp_id) {
            Some(mp) => mp,
            None => {
                // println!("SKIP delete {}", mp_id);
                // Mappoint has been deleted but tracking's local_mappoints is not updated with this info
                local_mps_to_remove.push(*mp_id);
                continue
            }
        };
        if let Some(mp_data) = track_in_view.get(&mp_id) {
            if mp_data.track_depth > far_points_th {
                // println!("SKIP depth {}", mp_id);
                continue;
            }

            // The size of the window will depend on the viewing direction
            let mut r = radius_by_viewing_cos(mp_data.view_cos);
            if th != 1 { r *= th as f64; }

            let indices = frame.features.get_features_in_area(
                &mp_data.proj_x,
                &mp_data.proj_y,
                r * (SCALE_FACTORS[mp_data.predicted_level as usize] as f64),
                Some((mp_data.predicted_level-1, mp_data.predicted_level))
            );

            // println!("get features in area,{}, {}, {}, {}, {}, {}, {}", mp_id, mp_data.proj_x, mp_data.proj_y, r * (SCALE_FACTORS[mp_data.predicted_level as usize] as f64), mp_data.predicted_level-1, mp_data.predicted_level, indices.len());

            if !indices.is_empty() {
                let mut best_dist = 256;
                let mut best_level = -1;
                let mut best_dist2 = 256;
                let mut best_level2 = -1;
                let mut best_idx: i32 = -1;

                // Get best and second matches with near keypoints
                for idx in indices {
                    if frame.mappoint_matches.has(&idx) {
                        let id = frame.mappoint_matches.get(&idx);
                        if let Some(mp) = map.mappoints.get(&id) {
                            if mp.get_observations().len() > 0 {
                                continue;
                            }
                        } else {
                            frame.mappoint_matches.delete_at_indices((idx as i32, -1)); // TODO (STEREO)
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

                    let descriptors = frame.features.descriptors.row(idx);
                    let dist = descriptor_distance(&mp.best_descriptor, &descriptors);
                    // print!("{} {}, ", idx, dist);

                    if dist < best_dist {
                        best_dist2 = best_dist;
                        best_dist = dist;
                        best_level2 = best_level;
                        best_level = frame.features.get_octave(idx as usize);
                        best_idx = idx as i32;
                    } else if dist < best_dist2 {
                        best_level2 = frame.features.get_octave(idx as usize);
                        best_dist2 = dist;
                    }
                }

                // println!("... {} {}", best_idx, mp_id);

                // Apply ratio to second match (only if best and second are in the same scale level)
                if best_dist <= TH_HIGH {
                    if best_level == best_level2 && (best_dist as f64) > (ratio * best_dist2 as f64) {
                        continue;
                    }
                    if best_level != best_level2 || (best_dist as f64) <= (ratio * best_dist2 as f64) {
                        frame.mappoint_matches.add(best_idx as u32, *mp_id, false);

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
        } else {
            // println!("SKIP no track data {}", mp_id);
        }

        if let Some(_mp_data) = track_in_view_right.get(&mp_id) {
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

    // println!();
    mappoints.retain(|mp_id| !local_mps_to_remove.contains(&mp_id));
    return Ok(num_matches);
}

// Project MapPoints tracked in last frame into the current frame and search matches.
// Used to track from previous frame (Tracking)

pub fn search_by_projection_with_threshold (
    current_frame: &mut Frame, last_frame: &mut Frame, th: i32,
    should_check_orientation: bool,
    map: &MapLock, sensor: Sensor
) -> Result<i32, Box<dyn std::error::Error>> {
    // int ORBmatcher::SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono)

    // TODO (Stereo)...search_by_projection_with_threshold...
    // This function should work for stereo as well as RGBD as long as get_all_keypoints() returns
    // a concatenated list of left keypoints and right keypoints (there is a to do for this in features.rs). BUT
    // double check that the descriptors works correctly. Instead of splitting descriptors into discriptors_left and right,
    // I have all of them in one long vec. I THINK this should be fine, but double check.
    let mut num_matches = 0;

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
    // println!("Search by projection with threshold");
    {
        let map = map.read();
        for idx1 in 0..last_frame.features.get_all_keypoints().len() {
            if last_frame.mappoint_matches.has(&(idx1 as u32)) {
                let mp_id = last_frame.mappoint_matches.get(&(idx1 as u32));
                if last_frame.mappoint_matches.is_outlier(&(idx1 as u32)) { continue; }
                // Project
                let mappoint = match map.mappoints.get(&mp_id) {
                    Some(mp) => mp,
                    None => {
                        // Mappoint has been deleted but not updated in last_frame yet
                        last_frame.mappoint_matches.delete_at_indices((idx1, -1)); // TODO (STEREO)
                        continue;
                    }
                };
                let x_3d_w = mappoint.position;
                let x_3d_c = (*rcw) * (*x_3d_w) + (*tcw);

                let xc = x_3d_c[0];
                let yc = x_3d_c[1];
                let invzc = 1.0 / x_3d_c[2];
                if invzc < 0.0 { continue; }

                let u = CAMERA_MODULE.fx * xc * invzc + CAMERA_MODULE.cx;
                let v = CAMERA_MODULE.fy * yc * invzc + CAMERA_MODULE.cy;
                if !current_frame.features.is_in_image(u, v) {
                    continue;
                }

                let last_octave = last_frame.features.get_octave(idx1 as usize);

                // Search in a window. Size depends on scale
                let radius = ((th as f32) * SCALE_FACTORS[last_octave as usize]) as f64;

                let indices_2 =
                    if forward {
                        current_frame.features.get_features_in_area(&u,&v, radius, Some((last_octave, -1)))
                    } else if backward {
                        current_frame.features.get_features_in_area(&u,&v, radius, Some((0, last_octave)))
                    } else {
                        current_frame.features.get_features_in_area(&u,&v, radius, Some((last_octave-1, last_octave+1)))
                    };
                if indices_2.is_empty() {
                    continue;
                }
                // print!("({},{},{},{}) ", u, v, radius, indices_2.len());

                let mut best_dist = 256;
                let mut best_idx: i32 = -1;

                for idx2 in indices_2 {
                    if current_frame.mappoint_matches.has(&idx2) {
                        let id = current_frame.mappoint_matches.get(&idx2);
                        if let Some(mappoint) = map.mappoints.get(&id) {
                            if mappoint.get_observations().len() > 0 { 
                                continue;
                            }
                        }
                    }

                    // TODO (Stereo): Unfinished code, not sure what this is doing in search_by_projection_with_threshold
                    // Nleft == -1 if the left camera has no extracted keypoints which would happen in the
                    // non-stereo case. But mvuRight is > 0 ONLY in the stereo case! So is this ever true?
                    // if(CurrentFrame.Nleft == -1 && CurrentFrame.mvuRight[i2]>0)
                    // {
                    //     const float ur = uv(0) - CurrentFrame.mbf*invzc;
                    //     const float er = fabs(ur - CurrentFrame.mvuRight[i2]);
                    //     if(er>radius)
                    //         continue;
                    // }

                    let descriptor = current_frame.features.descriptors.row(idx2);
                    let dist = descriptor_distance(&mappoint.best_descriptor, &descriptor);
                    // print!("{} {} {}, ", idx1, idx2, dist);

                    if dist < best_dist {
                        best_dist = dist;
                        best_idx = idx2 as i32;
                    }
                }
                // println!("BEST: {} {}", best_idx, best_dist);

                if best_dist <= TH_HIGH {
                    current_frame.mappoint_matches.add(best_idx as u32, mp_id, false);
                    // matches.insert(best_idx as u32, mp_id);

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
    }
    // println!("Search by projection with threshold initial {}", num_matches);

    // Apply rotation consistency
    if should_check_orientation {
        let (ind_1, ind_2, ind_3) = compute_three_maxima(&rot_hist,HISTO_LENGTH);
        for i in 0..HISTO_LENGTH {
            if i != ind_1 && i != ind_2 && i != ind_3 {
                for j in 0..rot_hist[i as usize].len() {
                    let key = rot_hist[i as usize][j];
                    current_frame.mappoint_matches.delete_at_indices((key as i32, -1));
                    num_matches -= 1;
                }
            }
        }
    };

    return Ok(num_matches);
} 

// TODO (mvp): other searchbyprojection functions we will have to implement later:

pub fn _search_by_projection_reloc (
    _current_frame: &mut Frame, _keyframe: &KeyFrame,
    _th: i32, _should_check_orientation: bool, _ratio: f64,
    _track_in_view: &HashMap<Id, TrackedMapPointData>, _track_in_view_right: &HashMap<Id, TrackedMapPointData>,
    _map: &MapLock, _sensor: Sensor
) -> Result<i32, Box<dyn std::error::Error>> {
    // int SearchByProjection(Frame &CurrentFrame, KeyFrame* pKF, const std::set<MapPoint*> &sAlreadyFound, const float th, const int ORBdist);
    // Project MapPoints seen in KeyFrame into the Frame and search matches.
    // Used in relocalisation (Tracking)
    todo!("Relocalization");
}

pub fn search_by_sim3(map: &MapLock, kf1_id: Id, kf2_id: Id, matches: &mut Vec<Option<Id>>, s12: f64, r12: &DVRotation, t12: &DVTranslation, th: f32) -> i32 {
    // From ORB-SLAM2:
    // int ORBmatcher::SearchBySim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint*> &vpMatches12,
    //                          const float &s12, const cv::Mat &R12, const cv::Mat &t12, const float th)

    let lock = map.read();

    // Camera 1 from world
    let kf1 = lock.keyframes.get(&kf1_id).unwrap();
    let r1w = kf1.pose.get_rotation();
    let t1w = kf1.pose.get_translation();

    // Camera 2 from world
    let kf2 = lock.keyframes.get(&kf2_id).unwrap();
    let r2w = kf2.pose.get_rotation();
    let tw2 = kf2.pose.get_translation();

    //Transformation between cameras
    let s_r_12 = s12 * **r12;
    let s_r_21 = (1.0 / s12) * r12.transpose();
    let t21 = -s_r_21 * **t12;

    let mappoints1 = kf1.get_mp_matches();
    let mappoints2 =  kf2.get_mp_matches();
    let n1 = mappoints1.len();
    let n2 = mappoints2.len();

    let mut already_matched1 = vec![false; n1];
    let mut already_matched2 = vec![false; n2];

    {
        let map = map.read();
        for i in 0..n1 {
            let mp_id = matches[i];
            if let Some(mp) = mp_id {
                already_matched1[i] = true;
                let mp = map.mappoints.get(&mp).unwrap();
                let (left_idx2, _right_idx2) = mp.get_index_in_keyframe(kf2_id);
                if left_idx2 != -1 && left_idx2 < n2 as i32 {
                    already_matched2[left_idx2 as usize] = true;
                }
            }
        }
    }

    let mut match1 = vec![-1; n1];
    let mut match2 = vec![-1; n2];

    // Transform from KF1 to KF2 and search
    for i in 0..n1 {
        let mp_id = match mappoints1[i] {
            Some((mp_id, _)) => mp_id,
            None => continue
        };
        let lock = map.read();
        let mp = match lock.mappoints.get(&mp_id) {
            Some(mp) => mp,
            None => continue
        };

        if already_matched1[i] {
            continue;
        }
        
        let p_3d_w = mp.position;
        let p_3d_c1 = (*r1w) * (*p_3d_w) + (*t1w);
        let p_3d_c2 = s_r_21 * p_3d_c1 + t21;

        // Depth must be positive
        if p_3d_c2[2] < 0.0 {
            continue;
        }

        let invz = 1.0 / p_3d_c2[2];
        let x = p_3d_c2[0] * invz;
        let y = p_3d_c2[1] * invz;
        
        let u = CAMERA_MODULE.fx * x + CAMERA_MODULE.cx;
        let v = CAMERA_MODULE.fy * y + CAMERA_MODULE.cy;

        // Point must be inside the image
        if !kf2.features.is_in_image(u, v) {
            continue;
        }
        let max_distance = mp.get_max_distance_invariance();
        let min_distance = mp.get_min_distance_invariance();
        let dist_3d = p_3d_c2.norm();

        // Depth must be inside the scale invariance region
        if dist_3d < min_distance || dist_3d > max_distance {
            continue;
        }

        // Compute predicted octave
        let n_predicted_level = mp.predict_scale(&dist_3d);

        // Search in a radius
        let radius = th * SCALE_FACTORS[n_predicted_level as usize];

        let indices = kf2.features.get_features_in_area(
            &u, &v, radius as f64, None
        );
        if indices.is_empty() {
            continue;
        }

        // Match to the most similar keypoint in the radius
        let d_mp = &mp.best_descriptor;

        let mut best_dist = std::i32::MAX;
        let mut best_idx = -1;

        for idx in indices {
            let (kp, _) = kf2.features.get_keypoint(idx as usize);
            if kp.octave() < n_predicted_level - 1 || kp.octave() > n_predicted_level {
                continue;
            }
            let descriptors_kf = kf2.features.descriptors.row(idx);
            let dist = descriptor_distance(&d_mp, &descriptors_kf);
            if dist < best_dist {
                best_dist = dist;
                best_idx = idx as i32;
            }
        }

        if best_dist <= TH_HIGH {
            match1[i] = best_idx;
        }
    }

    // Transform from KF2 to KF2 and search
    for i in 0..n2 {
        let lock = map.read();
        let mp_id = match mappoints2[i] {
            Some((mp, _)) => mp,
            None => continue
        };
        if already_matched2[i] {
            continue;
        }
        let mp = match lock.mappoints.get(&mp_id) {
            Some(mp) => mp,
            None => continue
        };

        let p_3d_w = mp.position;
        let p_3d_c2 = (*r2w) * (*p_3d_w) + (*tw2);
        let p_3d_c1 = s_r_12 * p_3d_c2 + **t12;

        // Depth must be positive
        if p_3d_c1[2] < 0.0 {
            continue;
        }
        
        let invz = 1.0 / p_3d_c1[2];
        let x = p_3d_c1[0] * invz;
        let y = p_3d_c1[1] * invz;

        let u = CAMERA_MODULE.fx * x + CAMERA_MODULE.cx;
        let v = CAMERA_MODULE.fy * y + CAMERA_MODULE.cy;

        // Point must be inside the image
        if !kf1.features.is_in_image(u, v) {
            continue;
        }

        let max_distance = mp.get_max_distance_invariance();
        let min_distance = mp.get_min_distance_invariance();
        let dist_3d = p_3d_c1.norm();

        // Depth must be inside the scale pyramid of the image
        if dist_3d < min_distance || dist_3d > max_distance {
            continue;
        }
        
        // Compute predicted octave
        let n_predicted_level = mp.predict_scale(&dist_3d);

        // Search in a radius of 2.5*sigma(ScaleLevel)
        let radius = th * SCALE_FACTORS[n_predicted_level as usize];
        
        let indices = kf1.features.get_features_in_area(
            &u, &v, radius as f64, None
        );
        if indices.is_empty() {
            continue;
        }
    
        // Match to the most similar keypoint in the radius
        let d_mp = &mp.best_descriptor;

        let mut best_dist = std::i32::MAX;
        let mut best_idx = -1;
        for idx in indices {
            let kp = kf1.features.get_keypoint(idx as usize).0;
            if kp.octave() < n_predicted_level - 1 || kp.octave() > n_predicted_level {
                continue;
            }
            let descriptors_kf = kf1.features.descriptors.row(idx);
            let dist = descriptor_distance(&d_mp, &descriptors_kf);
            if dist < best_dist {
                best_dist = dist;
                best_idx = idx as i32;
            }
        }
        if best_dist <= TH_HIGH {
            match2[i] = best_idx;
        }
    }

    // Check agreement
    let mut found = 0;
    for i in 0..n1 {
        let idx2 = match1[i];
        if idx2 >= 0 {
            let idx1 = match2[idx2 as usize];
            if idx1 == i as i32 {
                matches[i] = Some(mappoints2[i].unwrap().0);
                found += 1;
            }
        }
    }
    return found;

}

pub fn search_by_projection_for_loop_detection(
    map: &MapLock, kf_id: &Id,
    scw: &DVMatrix4<f32>, candidate_mps: &Vec<Id>, matched_mappoints: &mut Vec<Option<Id>>, threshold: i32,
    ratio: f64, check_orientation: bool
) -> Result<i32, Box<dyn std::error::Error>>{
    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in loop detection
    // From ORB-SLAM2:
    // int ORBmatcher::SearchByProjection(KeyFrame* pKF, cv::Mat Scw, const vector<MapPoint*> &vpPoints, vector<MapPoint*> &vpMatched, int th)

    // Some of the matrix stuff in here really sucks because we're using the opencv matrixes (DVMatrix) instead of nalgebra
    
    let lock = map.read();

    // Decompose Scw
    let scw_mat: DVMatrix = scw.into();
    let s_rcw = scw_mat.row_range(0,3).col_range(0,3);
    let scw = (s_rcw.row(0).dot(& *s_rcw.row(0))? as f32).sqrt();
    let rcw = s_rcw.divide_by_scalar(scw); // Should be == s_rCw / scw
    let tcw = DVMatrix::new(scw_mat.row_range(0,3).col(3)?).divide_by_scalar(scw); // Should be == scw_mat / scw
    let ow = DVMatrix::new_expr(rcw.clone().t()?).neg() * &tcw;

    // Set of MapPoints already found in the KeyFrame
    let mut already_found = vec![false; matched_mappoints.len()];

    let mut num_matches = 0;

    // For each Candidate MapPoint Project and Match
    for i in 0..candidate_mps.len() {
        let mp_id = candidate_mps[i];
        let mp = match lock.mappoints.get(&mp_id) {
            Some(mp) => mp,
            None => continue
        };
        if already_found[i] {
            continue;
        }

        // Get 3D Coords.
        let p3dw: Mat = (&mp.position).into();

        // Transform into Camera Coords.
        let p3dc = DVMatrix::new_expr_res(rcw.mat() * &p3dw + tcw.mat());

        // Depth must be positive
        if p3dc.at(2) < 0.0 {
            continue;
        }

        // Project into Image
        let invz = 1.0 / p3dc.at(2);
        let x = p3dc.at(0) * invz;
        let y = p3dc.at(1) * invz;

        let u = CAMERA_MODULE.fx * x + CAMERA_MODULE.cx;
        let v = CAMERA_MODULE.fy * y + CAMERA_MODULE.cy;

        // Point must be inside the image
        let kf = lock.keyframes.get(&kf_id).unwrap();
        if !kf.features.is_in_image(u, v) {
            continue;
        }

        // Depth must be inside the scale invariance region of the point
        let max_distance = mp.get_max_distance_invariance();
        let min_distance = mp.get_min_distance_invariance();
        let po = DVMatrix::new_expr((p3dw - ow.mat()).into_result()?);
        let dist = po.norm();

        if dist < min_distance || dist > max_distance {
            continue;
        }

        // Viewing angle must be less than 60 deg
        let pn: Mat = (&mp.normal_vector).into();
        if po.dot(&pn)? < 0.5 * dist {
            continue;
        }

        let n_predicted_level = mp.predict_scale(&dist);

        // Search in a radius
        let radius = threshold as f32 * SCALE_FACTORS[n_predicted_level as usize];

        let indices = kf.features.get_features_in_area(&u, &v, radius as f64, None);

        if indices.is_empty() {
            continue;
        }

        // Match to the most similar keypoint in the radius
        let d_mp = &mp.best_descriptor;

        let mut best_dist = std::i32::MAX;
        let mut best_idx = -1;
        for idx in indices { 
            if matched_mappoints[idx as usize].is_some() {
                continue;
            }
            let kp_level = kf.features.get_octave(idx as usize);
            if kp_level < n_predicted_level - 1 || kp_level > n_predicted_level {
                continue;
            }

            let descriptors_kf = kf.features.descriptors.row(idx);
            let dist = descriptor_distance(&d_mp, &descriptors_kf);
            if dist < best_dist {
                best_dist = dist;
                best_idx = idx as i32;
            }
        }
        if best_dist <= TH_LOW {
            matched_mappoints[best_idx as usize] = Some(mp_id);
            num_matches += 1;
            already_found[i] = true;
        }
    }

    Ok(num_matches)
}

pub fn search_by_projection_for_place_recognition() {
    todo!("LOOP CLOSING");
    // Project MapPoints using a Similarity Transformation and search matches.
    // Used in Place Recognition
    // int SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming=1.0);
}

pub fn search_by_bow_f(
    kf: &KeyFrame, frame: &mut Frame,
    should_check_orientation: bool, ratio: f64
) -> Result<u32, Box<dyn std::error::Error>> {
    // int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
    frame.mappoint_matches.clear();
    let mut matches = HashMap::<u32, Id>::new();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist = construct_rotation_histogram();

    let kf_featvec = kf.bow.as_ref().unwrap().feat_vec.get_all_nodes();
    let frame_featvec = frame.bow.as_ref().unwrap().feat_vec.get_all_nodes();
    let mut i = 0;
    let mut j = 0;


    while i < kf_featvec.len() && j < frame_featvec.len() {
        let kf_node_id = kf_featvec[i];
        let frame_node_id = frame_featvec[j];
        if kf_node_id == frame_node_id {
            let kf_indices_size = kf.bow.as_ref().unwrap().feat_vec.vec_size(kf_node_id);

            for index_kf in 0..kf_indices_size {
                let real_idx_kf = kf.bow.as_ref().unwrap().feat_vec.vec_get(kf_node_id, index_kf);

                if !kf.has_mp_match(&real_idx_kf) {
                    continue
                }
                let mp_id = &kf.get_mp_match(&real_idx_kf);
                // print!("{} ", mp_id);

                let mut best_dist1 = 256;
                let mut best_dist2 = 256;
                let mut best_idx: i32 = -1;
                let descriptors_kf = kf.features.descriptors.row(real_idx_kf);

                let frame_indices_size = frame.bow.as_ref().unwrap().feat_vec.vec_size(frame_node_id);

                for index_frame in 0..frame_indices_size {
                    let real_idx_frame = frame.bow.as_ref().unwrap().feat_vec.vec_get(frame_node_id, index_frame);

                    if matches.contains_key(&real_idx_frame) {
                        continue;
                    }

                    let descriptors_f = frame.features.descriptors.row(real_idx_frame);

                    let dist = descriptor_distance(&descriptors_kf, &descriptors_f);
                    // println!("{} {} {}", real_idx_kf, real_idx_frame, dist);

                    if dist < best_dist1 {
                        best_dist2 = best_dist1;
                        best_dist1 = dist;
                        best_idx = real_idx_frame as i32;
                    } else if dist < best_dist2 {
                        best_dist2 = dist;
                    }

                }

                if best_dist1 <= TH_LOW {
                    if (best_dist1 as f64) < ratio * (best_dist2 as f64) {
                        // println!("Match: {} {}", best_idx, mp_id);
                        matches.insert(best_idx as u32, *mp_id);

                        if should_check_orientation {
                            check_orientation_1(
                                &kf.features.get_keypoint(real_idx_kf as usize).0,
                                &frame.features.get_keypoint(best_idx as usize).0,
                                &mut rot_hist, factor, best_idx as u32
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
    
    if should_check_orientation {
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


    for (index, mp_id) in &matches {
        frame.mappoint_matches.add(*index, *mp_id, false);
    }

    return Ok(matches.len() as u32);
}

pub fn search_by_bow_kf(
    kf_1 : &KeyFrame, kf_2 : &KeyFrame, should_check_orientation: bool, 
    ratio: f64
) -> Result<HashMap<u32, Id>, Box<dyn std::error::Error>> {
    // int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);
    let mut matches = HashMap::<u32, Id>::new();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist = construct_rotation_histogram();

    for node_id_kf_1 in kf_1.bow.as_ref().unwrap().feat_vec.get_all_nodes() {
        for node_id_kf_2 in kf_2.bow.as_ref().unwrap().feat_vec.get_all_nodes() {
            if node_id_kf_1 == node_id_kf_2 {
                let indices_kf_1_size = kf_1.bow.as_ref().unwrap().feat_vec.vec_size(node_id_kf_1);
                let indices_kf_2_size = kf_2.bow.as_ref().unwrap().feat_vec.vec_size(node_id_kf_2);


                for index1 in 0..indices_kf_1_size {
                    let index_kf_1 = kf_1.bow.as_ref().unwrap().feat_vec.vec_get(node_id_kf_1, index1);
                    let mut best_dist = (256, 256);
                    let mut best_index = -1;
                    let descriptors_kf_1 = kf_1.features.descriptors.row(index_kf_1);

                    for index2 in 0..indices_kf_2_size {
                        let index_kf_2 = kf_2.bow.as_ref().unwrap().feat_vec.vec_get(node_id_kf_2, index2);
                        if kf_2.features.has_left_kp().map_or(false, |n_left| index_kf_2 > n_left) {
                            continue;
                        }

                        if kf_2.has_mp_match(&index_kf_2) {
                            continue;
                        }

                        let descriptors_kf_2 = kf_2.features.descriptors.row(index_kf_2);
                        let dist = descriptor_distance(&descriptors_kf_1, &descriptors_kf_2);

                        if dist < best_dist.0 {
                            best_dist.1 = best_dist.0;
                            best_dist.0 = dist;
                            best_index = index_kf_2 as i32;
                        } else if dist < best_dist.1 {
                            best_dist.1 = dist;
                        }
                    }

                    if best_dist.0 <= TH_LOW {
                        let mp_id = &kf_2.get_mp_match(&(best_index as u32));

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
    };

    return Ok(matches);
}

pub fn search_for_triangulation(
    kf_1 : &KeyFrame, kf_2 : &KeyFrame,
    should_check_orientation: bool, _only_stereo: bool, course: bool,
    sensor: Sensor
) -> Result<Vec<(usize, usize)>, Box<dyn std::error::Error>> {
    // Testing local mapping ... this function return slightly different results than ORB-SLAM. Could be because the optimized pose is slightly different but could also be an error. Uncomment the println lines (and same lines in ORB-SLAM3), then compare the strings.
    //int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse)
    // Some math based on ORB-SLAM2 to avoid some confusion with Sophus.
    // Look here: https://github.com/raulmur/ORB_SLAM2/blob/master/src/ORBmatcher.cc#L657

    // let _span = tracy_client::span!("search_for_triangulation");

    //Compute epipole in second image
    let cw = *kf_1.get_camera_center(); //Cw
    let r2w = *kf_2.pose.get_rotation(); //R2w
    let t2w = *kf_2.pose.get_translation(); //t2w
    let c2 = r2w * cw + t2w; //C2

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
        let pose12 = kf_1.pose * kf_2.pose.inverse(); // T12 ... which is different than t12
        r12 = pose12.get_rotation();
        t12 = pose12.get_translation();
    }

    // Find matches between not tracked keypoints
    // Matching speed-up by ORB Vocabulary
    // Compare only ORB that share the same node

    let mut matches = vec![None; kf_1.features.get_all_keypoints().len() as usize];
    // Note: ORBSLAM includes this vec but never inserts in it.
    // If we don't include the vec, we get the same results as ORBSLAM but introduce a bug where multiple
    // mappoints get made for the same index of the keyframe and the connections don't match up:
    // MP 1 -> KF, KF -> MP 2, MP 2 -> KF
    // Not sure how this error doesn't happen in ORBSLAM, since the rest of the code matches.
    // Including the vec gives us different results than ORBSLAM, but I think ours are correct.
    let mut matched_already = HashSet::<u32>::new();

    let factor = 1.0 / (HISTO_LENGTH as f32);
    let mut rot_hist = construct_rotation_histogram();

    let kf1_featvec = kf_1.bow.as_ref().unwrap().feat_vec.get_all_nodes();
    let kf2_featvec = kf_2.bow.as_ref().unwrap().feat_vec.get_all_nodes();
    let mut i = 0;
    let mut j = 0;

    // print!("Search for triangulation.");
    while i < kf1_featvec.len() && j < kf2_featvec.len() {
        let kf1_node_id = kf1_featvec[i];
        let kf2_node_id = kf2_featvec[j];
        if kf1_node_id == kf2_node_id {
            let kf1_indices_size = kf_1.bow.as_ref().unwrap().feat_vec.vec_size(kf1_node_id);

            for i1 in 0..kf1_indices_size {
                let kf1_index = kf_1.bow.as_ref().unwrap().feat_vec.vec_get(kf1_node_id, i1); // = idx1

                // If there is already a MapPoint skip
                if kf_1.has_mp_match(&kf1_index) {
                    continue
                };

                let stereo1 = match sensor.frame() {
                    FrameSensor::Stereo => {
                        todo!("Stereo");
                    },
                    _ => false
                };

                let (kp1, _right1) = kf_1.features.get_keypoint(kf1_index as usize);

                let mut best_dist = TH_LOW;
                let mut best_index = -1;
                let descriptors_kf_1 = kf_1.features.descriptors.row(kf1_index);

                let kf2_indices_size = kf_2.bow.as_ref().unwrap().feat_vec.vec_size(kf2_node_id);

                for i2 in 0..kf2_indices_size {
                    let kf2_index = kf_2.bow.as_ref().unwrap().feat_vec.vec_get(kf2_node_id, i2); // = idx2

                    // If we have already matched or there is a MapPoint skip
                    if kf_2.has_mp_match(&kf2_index) || matched_already.contains(&kf2_index) {
                        continue
                    };

                    let stereo2 = match sensor.frame() {
                        FrameSensor::Stereo => {
                            todo!("Stereo");
                        },
                        _ => false
                    };

                    let descriptors_kf_2 = kf_2.features.descriptors.row(kf2_index);
                    let dist = descriptor_distance(&descriptors_kf_1, &descriptors_kf_2);

                    if dist > TH_LOW || dist > best_dist {
                        continue
                    }

                    let (kp2, _right2) = kf_2.features.get_keypoint(kf2_index as usize);

                    if !stereo1 && !stereo2 { // && !kf1->mpCamera2 ... TODO (STEREO)
                        let dist_ex = (ep.0 as f32) - kp2.pt().x;
                        let dist_ey = (ep.1 as f32) - kp2.pt().y;
                        if dist_ex * dist_ex + dist_ey * dist_ey < 100.0 * SCALE_FACTORS[kp2.octave() as usize] {
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

                    // println!("{} {} {}, ", kf1_index, kf2_index, dist);

                    let epipolar = CAMERA_MODULE.epipolar_constrain(&kp1, &kp2, &r12, &t12, LEVEL_SIGMA2[kp2.octave() as usize]);
                    if course || epipolar {
                        best_index = kf2_index as i32;
                        best_dist = dist;
                    }
                }
                // println!("BEST: {} {} {}", kf1_index, best_index, best_dist);

                if best_index >= 0 {
                    let (kp2, _) = kf_2.features.get_keypoint(best_index as usize);
                    matches[kf1_index as usize] = Some(best_index as usize);
                    matched_already.insert(best_index as u32);
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
        } else {
            j = lower_bound(&kf2_featvec, &kf1_featvec, j, i);
        }
    }

    // println!("DONE");
    // Debugging
    // println!("search for triangulation: {} {} {} {} {}", skipped1, skipped2, skipped3, skipped4, loops);

    
    if should_check_orientation {
        let (ind_1, ind_2, ind_3) = compute_three_maxima(&rot_hist,HISTO_LENGTH);
        for i in 0..HISTO_LENGTH {
            if i == ind_1 || i == ind_2 || i == ind_3 {
                continue;
            }
            for j in 0..rot_hist[i as usize].len() {
                matches[rot_hist[i as usize][j] as usize] = None;
            }
        }
    };

    let mut matched_pairs = vec![];
    for i in 0..matches.len() {
        match matches[i] {
            Some(idx2) => {
                matched_pairs.push((i, idx2));
                // print!("{} {},", i, idx2);
            },
            None => {}
        }
    }
    // println!("DONE");

    return Ok(matched_pairs);
}

pub fn fuse_from_loop_closing(kf_id: &Id, scw: &Pose, mappoints: &Vec<Id>, map: &MapLock, th: i32, nnratio: f32) ->  Vec<Option<Id>> {
    // int ORBmatcher::Fuse(KeyFrame *pKF, Sophus::Sim3f &Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)

    // not sure why this function is different from the other fuse function, could be a really small difference with a lot of copied code, so double-check the code to save some time
    todo!("LOOP CLOSING");
}

pub fn fuse(kf_id: &Id, fuse_candidates: &Vec<Option<(Id, bool)>>, map: &MapLock, th: f32, is_right: bool) {
    // int ORBmatcher::Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th, const bool bRight)
    
    let (tcw, rcw, ow, _camera);
    {
        let lock = map.read();
        let keyframe = lock.keyframes.get(kf_id).unwrap();

        (tcw, rcw, ow, _camera) = match is_right {
            true => {
                todo!("Stereo");
                // Tcw = pKF->GetRightPose();
                // Ow = pKF->GetRightCameraCenter();
                // pCamera = pKF->mpCamera2;
            },
            false => {
                (keyframe.pose.get_translation(), keyframe.pose.get_rotation(), keyframe.get_camera_center(), CAMERA)
            }
        };
    }

    for fuse_cand in fuse_candidates {
        let (mp_id, _) = match fuse_cand {
            Some((mp_id, _)) => (mp_id, true),
            None => {
                continue
            }
        };

        let (mut best_dist, mut best_idx);
        {
            let lock = map.read();
            let mappoint = match lock.mappoints.get(&mp_id) {
                Some(mp) => mp,
                None => {
                    continue
                }
            };
            if mappoint.get_observations().contains_key(kf_id) {
                continue;
            }

            let p_3d_w = mappoint.position;
            let p_3d_c = (*rcw) * (*p_3d_w) + (*tcw);

            // Depth must be positive
            if p_3d_c[2] < 0.0 {
                continue;
            }

            let _inv_z = 1.0 / p_3d_c[2]; // Used by stereo below
            let uv = CAMERA_MODULE.project(DVVector3::new(p_3d_c));

            // Point must be inside the image
            if !lock.keyframes.get(kf_id).unwrap().features.is_in_image(uv.0, uv.1) {
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
            let pn = mappoint.normal_vector;
            if po.dot(&pn) < 0.5 * dist_3d {
                continue;
            }

            // Search in a radius
            let predicted_level = mappoint.predict_scale(&dist_3d);
            let radius = th * SCALE_FACTORS[predicted_level as usize];
            let indices = lock.keyframes.get(kf_id).unwrap().features.get_features_in_area(&uv.0, &uv.1, radius as f64, None);
            if indices.is_empty() {
                continue;
            }

            // Match to the most similar keypoint in the radius
            best_dist = 256;
            best_idx = -1;

            for idx2 in indices {
                let (kp, is_right) = lock.keyframes.get(kf_id).unwrap().features.get_keypoint(idx2 as usize);
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

                let desc_kf = lock.keyframes.get(kf_id).unwrap().features.descriptors.row(idx2);
                let dist = descriptor_distance(&lock.mappoints.get(mp_id).unwrap().best_descriptor, &desc_kf);

                if dist < best_dist {
                    best_dist = dist;
                    best_idx = idx2 as i32;
                }
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if best_dist <= TH_LOW {
            let has_mappoint_match = map.read().keyframes.get(kf_id).unwrap().has_mp_match(&(best_idx as u32));
            if has_mappoint_match {
                let (mp_in_kf_id, mp_in_kf_obs, mp_obs);
                {
                    let lock = map.read();
                    mp_in_kf_id = lock.keyframes.get(kf_id).unwrap().get_mp_match(&(best_idx as u32));
                    mp_in_kf_obs = lock.mappoints.get(&mp_in_kf_id).unwrap().get_observations().len();
                    mp_obs = lock.mappoints.get(mp_id).unwrap().get_observations().len();
                }
                if mp_in_kf_obs > mp_obs {
                    map.write().replace_mappoint(*mp_id, mp_in_kf_id);
                } else {
                    map.write().replace_mappoint(mp_in_kf_id, *mp_id);
                }
            } else {
                map.write().add_observation(*kf_id, *mp_id, best_idx as u32, false);
            }
        }
    }

    // debug!("Fuse statistics.. none: {}, not in map: {}, already in obs: {}, depth negative: {}, point not in image: {}", quit_none, quit_not_in_map, quit_in_obs, quit_depth, quit_point_in_image);
    // debug!("Statistics continued... indices empty: {}, inv sigma: {}, best dist: {}, predicted level: {}, viewing angle: {}, inside scale: {}", quit_indices_empty, quit_inv_sigma, quit_best_dist, quit_predicted_level, quit_viewing_angle, quit_inside_scale);

    // debug!("search_in_neighbors, kf: {}, observations added: {}, mappoints replaced: {}", kf_id, obs_added, mps_replaced);
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
    // TODO (timing) ... this could be sped up
    let mut curr = i;
    while vec1[curr] < vec2[j] {
        curr += 1;
    }
    curr
}


pub fn descriptor_distance_print(desc1: &Mat, desc2: &Mat, print: bool) -> i32 {
    // I know I said don't use BindCVRawPtr if possible
    // In this case it's ok because we are not doing anything with the pointer
    // other than reading and immediately discarding. It would be wasteful
    // to clone here if we wanted to do the safer strategy of contructing
    // a DVMatrix from the Mat.

    // This code is equivalent to:
    // return opencv::core::norm2(
    //     a, b, opencv::core::NormTypes::NORM_HAMMING as i32, &Mat::default()
    // ).unwrap() as i32;
    // But much faster!!

    dvos3binding::ffi::descriptor_distance(
        &dvos3binding::ffi::WrapBindCVRawPtr { 
            raw_ptr: dvos3binding::BindCVRawPtr {
                raw_ptr: desc1.as_raw()
            } 
        },
        &dvos3binding::ffi::WrapBindCVRawPtr { 
            raw_ptr: dvos3binding::BindCVRawPtr {
                raw_ptr: desc2.as_raw()
            } 
        },
        print
    )
}

pub fn descriptor_distance(desc1: &Mat, desc2: &Mat) -> i32 {
    dvos3binding::ffi::descriptor_distance(
        &dvos3binding::ffi::WrapBindCVRawPtr { 
            raw_ptr: dvos3binding::BindCVRawPtr {
                raw_ptr: desc1.as_raw()
            } 
        },
        &dvos3binding::ffi::WrapBindCVRawPtr { 
            raw_ptr: dvos3binding::BindCVRawPtr {
                raw_ptr: desc2.as_raw()
            } 
        },
        false
    )
}
