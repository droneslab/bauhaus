use core::system::Module;
use std::cmp::Ordering;
use std::collections::{BTreeSet, HashMap, HashSet};
use std::f64::INFINITY;
use core::config::SETTINGS;
use core::matrix::{DVMatrix, DVVector3, DVVectorOfPoint2f};
use core::sensor::{Sensor, FrameSensor};
use log::{debug, error};
use opencv::core::KeyPoint;
use opencv::prelude::*;
use crate::MapLock;
use crate::actors::tracking_backend::TrackedMapPointData;
use crate::map::pose::{DVTranslation, Pose, Sim3};
use crate::modules::optimizer::LEVEL_SIGMA2;
use crate::registered_actors::{CAMERA, CAMERA_MODULE, FEATURE_DETECTION, FEATURE_MATCHER};
use crate::map::{map::Id, keyframe::KeyFrame, frame::Frame};
use crate::modules::module::CameraModule;

use super::module::{BoWModule, DescriptorDistanceTrait, FeatureMatchingModule};
use super::optimizer::INV_LEVEL_SIGMA2;


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


// ORBMatcherTrait is a supertrait that implements the following other traits:
pub trait ORBMatcherTrait: FeatureMatchingModule + DescriptorDistanceTrait + std::fmt::Debug + Send + Sync {}

#[derive(Debug)]
pub struct ORBMatcher {
    high_threshold: i32,
    low_threshold: i32,
    histo_length: i32,
}

impl ORBMatcher {
    pub fn new() -> Self {
        ORBMatcher {
            high_threshold: SETTINGS.get::<i32>(FEATURE_MATCHER, "high_threshold"),
            low_threshold: SETTINGS.get::<i32>(FEATURE_MATCHER, "low_threshold"),
            histo_length: SETTINGS.get::<i32>(FEATURE_MATCHER, "histo_length"),
        }
    }

    fn construct_rotation_histogram(&self) -> Vec<Vec<u32>> {
        // Rotation Histogram (to check rotation consistency)
        let mut rot_hist: Vec<Vec<u32>> = Vec::new();
        for _ in 0..self.histo_length {
            rot_hist.push(Vec::new());
        }
        rot_hist
    }

    fn check_orientation_1(
        &self, 
        keypoint_1: &KeyPoint, keypoint_2: &KeyPoint,
        rot_hist: &mut Vec<Vec<u32>>, factor: f32,
        idx: u32
    ) {
        let mut rot = keypoint_1.angle() - keypoint_2.angle();
        if rot < 0.0 {
            rot += 360.0;
        }
        let mut bin = (rot * factor).round() as i32;
        if bin == self.histo_length {
            bin = 0;
        }
        assert!(bin >= 0 && bin < self.histo_length );
        rot_hist[bin as usize].push(idx);
    }


    fn radius_by_viewing_cos(&self, view_cos: f64) -> f64 {
        return match view_cos > 0.998 {
            true => 2.5,
            false => 4.0
        };
    }

    fn compute_three_maxima(&self, histo : &Vec<Vec<u32>>) -> (i32, i32, i32) {
        let (mut max_1, mut max_2, mut max_3) = (0, 0, 0);
        let (mut ind_1, mut ind_2, mut ind_3) = (-1, -1, -1);

        for i in 0..self.histo_length {
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

    fn lower_bound(&self, vec1: &Vec<u32>, vec2: &Vec<u32>, i: usize, j: usize) -> usize {
        vec1
        .binary_search_by(|element| match element.cmp(&vec2[j]) {
            // Since we try to find position of first element,
            // we treat all equal values as greater to move left.
            Ordering::Equal => Ordering::Greater,
            ord => ord,
        }).unwrap_err()
    }
}

impl ORBMatcherTrait for ORBMatcher { }

impl DescriptorDistanceTrait for ORBMatcher {
    fn descriptor_distance(&self, desc1: &Mat, desc2: &Mat) -> i32 {
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
            false
        )
    }
}


impl FeatureMatchingModule for ORBMatcher {
    fn search_for_initialization(
        &self, 
        f1: &Frame,
        f2: &Frame,
        vb_prev_matched: &mut DVVectorOfPoint2f,
        window_size: i32,
    ) -> (i32, Vec<i32>) {
        let nnratio = 0.9;
        let check_ori=true;

        let mut vn_matches12: Vec<i32> = vec![-1; f1.features.get_all_keypoints().len() as usize];
        let factor = 1.0 / self.histo_length as f32;
        let mut v_matched_distance = vec![std::i32::MAX; f2.features.get_all_keypoints().len() as usize];
        let mut vn_matches21: Vec<i32> = vec![-1; f2.features.get_all_keypoints().len() as usize];
        let mut n_matches = 0;

        let mut rot_hist = self.construct_rotation_histogram();

        for (i1, kp1) in f1.features.get_all_keypoints().iter().enumerate() {
            let level1 = kp1.octave();
            if level1 > 0 {
                continue;
            }

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
                let dist = self.descriptor_distance(&d1,&d2);
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
            if best_dist <= self.low_threshold {
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
                        self.check_orientation_1(
                            &f1.features.get_keypoint(i1 as usize).0,
                            &f2.features.get_keypoint(best_idx2 as usize).0,
                            &mut rot_hist, factor, i1 as u32
                        );
                    }
                }
            }
        }

        if check_ori {
            let (ind_1, ind_2, ind_3) = self.compute_three_maxima(&rot_hist);
            for i in 0..self.histo_length {
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

    fn search_by_projection(
        &self, 
        frame: &mut Frame, mappoints: &mut BTreeSet<Id>, th: i32, ratio: f64,
        track_in_view: &HashMap<Id, TrackedMapPointData>, track_in_view_right: &HashMap<Id, TrackedMapPointData>, 
        map: &MapLock, sensor: Sensor
    ) -> Result<(HashMap<Id, i32>, i32), Box<dyn std::error::Error>> {
        // int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th=3, const bool bFarPoints = false, const float thFarPoints = 50.0f);
        // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
        // Used to track the local map (Tracking)
        let fpt = SETTINGS.get::<f64>(FEATURE_MATCHER, "far_points_threshold");
        let far_points_th = if fpt == 0.0 { INFINITY } else { fpt };
        let mut num_matches = 0;

        let mut non_tracked_points = HashMap::new();

        let map = map.read();
        let mut local_mps_to_remove = vec![];

        println!("Inside search by projection, mappoints: {}", mappoints.len());
        let mut indices_empty = 0;
        let mut level = 0;
        let mut level2 = 0;
        let mut track_in_view_c = 0;
        for mp_id in &*mappoints {
            let mp = match map.mappoints.get(&mp_id) {
                Some(mp) => mp,
                None => {
                    // Mappoint has been deleted but tracking's local_mappoints is not updated with this info
                    local_mps_to_remove.push(*mp_id);
                    continue
                }
            };
            if let Some(mp_data) = track_in_view.get(&mp_id) {
                if mp_data.track_depth > far_points_th {
                    continue;
                }

                // The size of the window will depend on the viewing direction
                let mut r = self.radius_by_viewing_cos(mp_data.view_cos);
                if th != 1 { r *= th as f64; }

                // println!("get features in area: {}, {}, {}, {:?}", mp_data.proj_x, mp_data.proj_y, r, (mp_data.predicted_level-1, mp_data.predicted_level));
                let indices = frame.features.get_features_in_area(
                    &mp_data.proj_x,
                    &mp_data.proj_y,
                    r * (SCALE_FACTORS[mp_data.predicted_level as usize] as f64),
                    Some((mp_data.predicted_level-1, mp_data.predicted_level))
                );

                if !indices.is_empty() {
                    let mut best_dist = 256;
                    let mut best_level = -1;
                    let mut best_dist2 = 256;
                    let mut best_level2 = -1;
                    let mut best_idx: i32 = -1;

                    // Get best and second matches with near keypoints
                    for idx in indices {
                        if let Some((mp_id, _is_outlier)) = frame.mappoint_matches.get(idx as usize) {
                            if let Some(mp) = map.mappoints.get(&mp_id) {
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
                        let dist = self.descriptor_distance(&mp.best_descriptor, &descriptors);

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

                    // Apply ratio to second match (only if best and second are in the same scale level)
                    if best_dist <= self.high_threshold {
                        if best_level == best_level2 && (best_dist as f64) > (ratio * best_dist2 as f64) {
                            level += 1;
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
                    } else {
                        level2 += 1;
                    }
                } else {
                    indices_empty += 1;
                }
            } else {
                track_in_view_c += 1;
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
                //         if(bestDist<=self.high_threshold)
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
            println!("Indices empty: {}, level1: {}, level2: {}, track_in_view_c: {}", indices_empty, level, level2, track_in_view_c);

        mappoints.retain(|mp_id| !local_mps_to_remove.contains(&mp_id));
        return Ok((non_tracked_points, num_matches));
    }

    // Project MapPoints tracked in last frame into the current frame and search matches.
    // Used to track from previous frame (Tracking)

    fn search_by_projection_with_threshold (
        &self, 
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

        let factor = 1.0 / (self.histo_length as f32);
        let mut rot_hist = self.construct_rotation_histogram();

        let tcw = current_frame.pose.unwrap();
        let twc = tcw.inverse().get_translation();

        let tlw = last_frame.pose.unwrap();
        let tlc = (*tlw.get_rotation()) * (*twc) + (*tlw.get_translation());

        let forward = tlc[2] > CAMERA_MODULE.stereo_baseline && !sensor.is_mono();
        let backward = -tlc[2] > CAMERA_MODULE.stereo_baseline && !sensor.is_mono();

        {
            let map = map.read();
            for idx1 in 0..last_frame.features.get_all_keypoints().len() {
                if let Some((mp_id, is_outlier)) = last_frame.mappoint_matches.get(idx1 as usize) {
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
                    let x_3d_c = (*tcw.get_rotation()) * (*x_3d_w) + (*tcw.get_translation());

                    let invzc = 1.0 / x_3d_c[2];
                    if invzc < 0.0 { continue; }

                    let (u, v) = CAMERA_MODULE.project(x_3d_c.into());
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

                    let mut best_dist = 256;
                    let mut best_idx: i32 = -1;

                    for idx2 in indices_2 {
                        if let Some((mp_id, is_outlier)) = current_frame.mappoint_matches.get(idx2 as usize) {
                            if let Some(mappoint) = map.mappoints.get(&mp_id) {
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
                        let dist = self.descriptor_distance(&mappoint.best_descriptor, &descriptor);

                        if dist < best_dist {
                            best_dist = dist;
                            best_idx = idx2 as i32;
                        }
                    }

                    if best_dist <= self.high_threshold {
                        current_frame.mappoint_matches.add(best_idx as u32, mp_id, false);
                        // matches.insert(best_idx as u32, mp_id);

                        if should_check_orientation {
                            self.check_orientation_1(
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

        // Apply rotation consistency
        if should_check_orientation {
            let (ind_1, ind_2, ind_3) = self.compute_three_maxima(&rot_hist);

            for i in 0..self.histo_length {
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

    fn _search_by_projection_reloc (
        &self, 
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

    fn search_by_sim3(&self,  map: &MapLock, kf1_id: Id, kf2_id: Id, matches: &mut HashMap<usize, i32>, sim3: &Sim3, th: f32) -> i32 {
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
        let s12 = sim3.scale;
        let r12 = sim3.pose.get_rotation();
        let t12 = sim3.pose.get_translation();
        let s_r_12 = s12 * *r12;
        let s_r_21 = (1.0 / s12) * r12.transpose();
        let t21 = -s_r_21 * *t12;

        let mappoints1 = kf1.get_mp_matches();
        let mappoints2 =  kf2.get_mp_matches();
        let n1 = mappoints1.len();
        let n2 = mappoints2.len();

        let mut already_matched1 = vec![false; n1];
        let mut already_matched2 = vec![false; n2];

        for i in 0..n1 {
            let mp_id = matches.get(&i);
            if let Some(mp) = mp_id {
                already_matched1[i] = true;
                let mp = lock.mappoints.get(&mp).unwrap();
                let (left_idx2, _right_idx2) = mp.get_index_in_keyframe(kf2_id);
                if left_idx2 != -1 && left_idx2 < n2 as i32 {
                    already_matched2[left_idx2 as usize] = true;
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
                let dist = self.descriptor_distance(&d_mp, &descriptors_kf);
                if dist < best_dist {
                    best_dist = dist;
                    best_idx = idx as i32;
                }
            }

            if best_dist <= self.high_threshold {
                match1[i] = best_idx;
            }
        }

        // Transform from KF2 to KF2 and search
        for i in 0..n2 {
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
            let p_3d_c1 = s_r_12 * p_3d_c2 + *t12;

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
                let dist = self.descriptor_distance(&d_mp, &descriptors_kf);
                if dist < best_dist {
                    best_dist = dist;
                    best_idx = idx as i32;
                }
            }
            if best_dist <= self.high_threshold {
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
                    matches.insert(i, mappoints2[idx2 as usize].unwrap().0);
                    found += 1;
                }
            }
        }
        return found;

    }

    fn search_by_projection_for_loop_detection1(
        &self, 
        map: &MapLock, kf_id: &Id, scw: &Sim3, 
        candidates: &Vec<Id>, // vpPoints
        kfs_for_candidates: &Vec<Id>, // vpPointsKFs
        matches: &mut Vec<Option<Id>>, // vpMatched
        threshold: i32, hamming_ratio: f64
    ) -> Result<(i32, Vec<Option<Id>>), Box<dyn std::error::Error>>{
        // int ORBmatcher::SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming)
        // return vpMatchedKF
        // Used in loop detection

        let lock = map.read();

        let tcw: Pose = (*scw).into();
        let rot = tcw.get_rotation();
        let trans = tcw.get_translation();
        let ow = tcw.inverse().get_translation();

        // Set of MapPoints already found in the KeyFrame
        let already_found = matches.into_iter().filter(|mp_id| mp_id.is_some()).map(|mp_id| mp_id.unwrap()).collect::<BTreeSet<Id>>();

        let mut num_matches = 0;

        // For each Candidate MapPoint Project and Match
        let mut n_already_found = 0;
        let mut n_depth = 0;
        let mut n_not_in_image = 0;
        let mut n_wrong_dist = 0;
        let mut n_viewing_angle = 0;
        let mut n_indices = 0;
        let mut n_has_match  = 0;
        let mut n_levels  = 0;
        let mut n_final_dist_too_low = 0;

        let mut matched_kfs = vec![None; matches.len()];

        for i in 0..candidates.len() {
            let mp_id = candidates[i];
            let kf_i = kfs_for_candidates[i];

            // Discard Bad MapPoints and already found
            let mp = match lock.mappoints.get(&mp_id) {
                Some(mp) => mp,
                None => continue
            };
            if already_found.get(&mp_id).is_some() {
                n_already_found += 1;
                continue;
            }

            // Get 3D Coords.
            let p3dw = *mp.position;

            // Transform into Camera Coords.
            let p3dc = *rot * p3dw + *trans;
        
            // Depth must be positive
            if p3dc[2] < 0.0 {
                n_depth += 1;
                continue;
            }

            // Project into Image
            let invz = 1.0 / p3dc[2];
            let x = p3dc[0] * invz;
            let y = p3dc[1] * invz;

            let u = CAMERA_MODULE.fx * x + CAMERA_MODULE.cx;
            let v = CAMERA_MODULE.fy * y + CAMERA_MODULE.cy;

            // Point must be inside the image
            let kf = lock.keyframes.get(&kf_id).unwrap();
            if !kf.features.is_in_image(u, v) {
                n_not_in_image += 1;
                continue;
            }

            // Depth must be inside the scale invariance region of the point
            let max_distance = mp.get_max_distance_invariance();
            let min_distance = mp.get_min_distance_invariance();
            let po = p3dw - *ow;
            let dist = po.norm();

            if dist < min_distance || dist > max_distance {
                n_wrong_dist += 1;
                continue;
            }

            // Viewing angle must be less than 60 deg
            let pn = &mp.normal_vector;
            if po.dot(&pn) < 0.5 * dist {
                n_viewing_angle += 1;
                continue;
            }

            let n_predicted_level = mp.predict_scale(&dist);

            // Search in a radius
            let radius = threshold as f32 * SCALE_FACTORS[n_predicted_level as usize];

            let indices = kf.features.get_features_in_area(&u, &v, radius as f64, None);

            if indices.is_empty() {
                n_indices += 1;
                continue;
            }

            // Match to the most similar keypoint in the radius
            let d_mp = &mp.best_descriptor;

            let mut best_dist = std::i32::MAX;
            let mut best_idx = -1;
            for idx in indices { 
                // if matches.get(&(idx as usize)).is_some() {
                //     n_has_match += 1;
                //     continue;
                // }
                let kp_level = kf.features.get_octave(idx as usize);
                if kp_level < n_predicted_level - 1 || kp_level > n_predicted_level {
                    n_levels += 1;
                    continue;
                }

                let descriptors_kf = kf.features.descriptors.row(idx);
                let dist = self.descriptor_distance(&d_mp, &descriptors_kf);
                if dist < best_dist {
                    best_dist = dist;
                    best_idx = idx as i32;
                }
            }
            if (best_dist as f64) <= (self.low_threshold as f64) * hamming_ratio {
                matches[best_idx as usize] = Some(mp_id);
                matched_kfs[best_idx as usize] = Some(kf_i);
                num_matches += 1;
            } else {
                n_final_dist_too_low += 1;
            }
        }

        debug!("...Search by projection for loop detection: already found {}, depth {}, not in image {}, wrong dist {}, viewing angle {}, indices {}, levels {}, final dist too low {}", n_already_found, n_depth, n_not_in_image, n_wrong_dist, n_viewing_angle, n_indices, n_levels, n_final_dist_too_low);

        Ok((num_matches, matched_kfs))

    }

    fn search_by_projection_for_loop_detection2(
        &self, 
        map: &MapLock, kf_id: &Id, scw: &Sim3, 
        candidates: &Vec<Id>,
        matches: &mut Vec<Option<Id>>,
        threshold: i32, hamming_ratio: f64
    ) -> Result<i32, Box<dyn std::error::Error>>{
        // int ORBmatcher::SearchByProjection(KeyFrame* pKF, Sophus::Sim3<float> &Scw, const std::vector<MapPoint*> &vpPoints, const std::vector<KeyFrame*> &vpPointsKFs, std::vector<MapPoint*> &vpMatched, std::vector<KeyFrame*> &vpMatchedKF, int th, float ratioHamming)
        // Used in loop detection

        let lock = map.read();

        let tcw: Pose = (*scw).into();
        let rot = tcw.get_rotation();
        let trans = tcw.get_translation();
        let ow = tcw.inverse().get_translation();

        // Set of MapPoints already found in the KeyFrame
        let already_found = matches.into_iter().filter(|mp_id| mp_id.is_some()).map(|mp_id| mp_id.unwrap()).collect::<BTreeSet<Id>>();

        let mut num_matches = 0;

        // For each Candidate MapPoint Project and Match

        for i in 0..candidates.len() {
            let mp_id = candidates[i];

            // Discard Bad MapPoints and already found
            let mp = match lock.mappoints.get(&mp_id) {
                Some(mp) => mp,
                None => continue
            };
            if already_found.get(&mp_id).is_some() {
                continue;
            }

            // Get 3D Coords.
            let p3dw = *mp.position;

            // Transform into Camera Coords.
            let p3dc = *rot * p3dw + *trans;

            // Depth must be positive
            if p3dc[2] < 0.0 {
                continue;
            }

            // Project into Image
            let (u, v) = CAMERA_MODULE.project(DVTranslation::new(p3dc));

            // Point must be inside the image
            let kf = lock.keyframes.get(&kf_id).unwrap();
            if !kf.features.is_in_image(u, v) {
                continue;
            }

            // Depth must be inside the scale invariance region of the point
            let max_distance = mp.get_max_distance_invariance();
            let min_distance = mp.get_min_distance_invariance();
            let po = p3dw - *ow;
            let dist = po.norm();

            if dist < min_distance || dist > max_distance {
                continue;
            }

            // Viewing angle must be less than 60 deg
            let pn = &mp.normal_vector;
            if po.dot(&pn) < 0.5 * dist {
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
                if matches[idx as usize].is_some() {
                    continue;
                }
                let kp_level = kf.features.get_octave(idx as usize);
                if kp_level < n_predicted_level - 1 || kp_level > n_predicted_level {
                    continue;
                }

                let descriptors_kf = kf.features.descriptors.row(idx);
                let dist = self.descriptor_distance(&d_mp, &descriptors_kf);
                if dist < best_dist {
                    best_dist = dist;
                    best_idx = idx as i32;
                }
            }
            if (best_dist as f64) <= (self.low_threshold as f64) * hamming_ratio {
                matches[best_idx as usize] = Some(mp_id);
                num_matches += 1;
            }
        }

        // debug!("...Search by projection for loop detection stats:  already found {}, depth {}, not in image {}, wrong dist {}, viewing angle {}, indices {}, has match {}, levels {}, final dist too low {}", n_already_found, n_depth, n_not_in_image, n_wrong_dist, n_viewing_angle, n_indices, n_has_match, n_levels, n_final_dist_too_low);

        Ok(num_matches)

    }

    fn search_by_bow_f(
        &self, 
        kf: &KeyFrame, frame: &mut Frame,
        should_check_orientation: bool, ratio: f64
    ) -> Result<u32, Box<dyn std::error::Error>> {
        // int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint*> &vpMapPointMatches);
        frame.mappoint_matches.clear();
        let mut matches = HashMap::<u32, Id>::new();

        let factor = 1.0 / (self.histo_length as f32);
        let mut rot_hist = self.construct_rotation_histogram();

        let kf_featvec = kf.bow.as_ref().unwrap().get_feat_vec().get_all_nodes();
        let frame_featvec = frame.bow.as_ref().unwrap().get_feat_vec().get_all_nodes();
        let mut i = 0;
        let mut j = 0;

        while i < kf_featvec.len() && j < frame_featvec.len() {
            let kf_node_id = kf_featvec[i];
            let frame_node_id = frame_featvec[j];
            if kf_node_id == frame_node_id {
                let kf_indices_size = kf.bow.as_ref().unwrap().get_feat_vec().vec_size(kf_node_id);

                for index_kf in 0..kf_indices_size {
                    let real_idx_kf = kf.bow.as_ref().unwrap().get_feat_vec().vec_get(kf_node_id, index_kf);

                    if let Some((mp_id, _is_outlier)) = kf.get_mp_match(&real_idx_kf) {
                        let mut best_dist1 = 256;
                        let mut best_dist2 = 256;
                        let mut best_idx: i32 = -1;
                        let descriptors_kf = kf.features.descriptors.row(real_idx_kf);

                        let frame_indices_size = frame.bow.as_ref().unwrap().get_feat_vec().vec_size(frame_node_id);

                        for index_frame in 0..frame_indices_size {
                            let real_idx_frame = frame.bow.as_ref().unwrap().get_feat_vec().vec_get(frame_node_id, index_frame);

                            if matches.contains_key(&real_idx_frame) {
                                continue;
                            }

                            let descriptors_f = frame.features.descriptors.row(real_idx_frame);

                            let dist = self.descriptor_distance(&descriptors_kf, &descriptors_f);

                            if dist < best_dist1 {
                                best_dist2 = best_dist1;
                                best_dist1 = dist;
                                best_idx = real_idx_frame as i32;
                            } else if dist < best_dist2 {
                                best_dist2 = dist;
                            }

                        }

                        if best_dist1 <= self.low_threshold {
                            if (best_dist1 as f64) < ratio * (best_dist2 as f64) {
                                matches.insert(best_idx as u32, mp_id);

                                if should_check_orientation {
                                    self.check_orientation_1(
                                        &kf.features.get_keypoint(real_idx_kf as usize).0,
                                        &frame.features.get_keypoint(best_idx as usize).0,
                                        &mut rot_hist, factor, best_idx as u32
                                    );
                                };
                            }
                        }
                    }
                }
                i += 1;
                j += 1;
            } else if kf_node_id < frame_node_id {
                i = self.lower_bound(&kf_featvec, &frame_featvec, i, j);
            } else {
                j = self.lower_bound(&frame_featvec, &kf_featvec, j, i);
            }
        }
        
        if should_check_orientation {
            let (ind_1, ind_2, ind_3) = self.compute_three_maxima(&rot_hist);
            for i in 0..self.histo_length {
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

    fn search_by_bow_kf(
        &self, 
        kf_1 : &KeyFrame, kf_2 : &KeyFrame, should_check_orientation: bool, 
        ratio: f64
    ) -> Result<HashMap<u32, Id>, Box<dyn std::error::Error>> {
        // int SearchByBoW(KeyFrame *pKF1, KeyFrame* pKF2, std::vector<MapPoint*> &vpMatches12);
        let mut matches = HashMap::<u32, Id>::new(); // vpMatches12
        let mut matched_already_in_kf2 = HashSet::<Id>::new(); // vbMatched2

        let factor = 1.0 / (self.histo_length as f32);
        let mut rot_hist = self.construct_rotation_histogram();

        let kf_1_featvec = kf_1.bow.as_ref().unwrap().get_feat_vec().get_all_nodes();
        let kf_2_featvec = kf_2.bow.as_ref().unwrap().get_feat_vec().get_all_nodes();
        let mut i = 0;
        let mut j = 0;

        while i < kf_1_featvec.len() && j < kf_2_featvec.len() {
            let kf_1_node_id = kf_1_featvec[i];
            let kf_2_node_id = kf_2_featvec[j];
            if kf_1_node_id == kf_2_node_id {
                let kf_1_indices_size = kf_1.bow.as_ref().unwrap().get_feat_vec().vec_size(kf_1_node_id);

                for index_kf1 in 0..kf_1_indices_size {
                    let real_idx_kf1 = kf_1.bow.as_ref().unwrap().get_feat_vec().vec_get(kf_1_node_id, index_kf1);

                    if let Some((mp_id, is_outlier)) = kf_1.get_mp_match(&real_idx_kf1) {
                        let mut best_dist1 = 256;
                        let mut best_dist2 = 256;
                        let mut best_idx2: i32 = -1;
                        let descriptors_kf_1 = kf_1.features.descriptors.row(real_idx_kf1);

                        let kf_2_indices_size = kf_2.bow.as_ref().unwrap().get_feat_vec().vec_size(kf_2_node_id);

                        for index_kf2 in 0..kf_2_indices_size {
                            let real_idx_kf2 = kf_2.bow.as_ref().unwrap().get_feat_vec().vec_get(kf_2_node_id, index_kf2);

                            if matched_already_in_kf2.contains(&(real_idx_kf2 as i32))
                                || kf_2.get_mp_match(&real_idx_kf2).is_none() {
                                continue;
                            }

                            let descriptors_kf_2 = kf_2.features.descriptors.row(real_idx_kf2);

                            let dist = self.descriptor_distance(&descriptors_kf_1, &descriptors_kf_2);

                            if dist < best_dist1 {
                                best_dist2 = best_dist1;
                                best_dist1 = dist;
                                best_idx2 = real_idx_kf2 as i32;
                            } else if dist < best_dist2 {
                                best_dist2 = dist;
                            }
                        }

                        if best_dist1 <= self.low_threshold {
                            if (best_dist1 as f64) < ratio * (best_dist2 as f64) {
                                matches.insert(real_idx_kf1 as u32, kf_2.get_mp_match(&(best_idx2 as u32)).unwrap().0);
                                matched_already_in_kf2.insert(best_idx2 as i32);

                                if should_check_orientation {
                                    self.check_orientation_1(
                                        &kf_1.features.get_keypoint(real_idx_kf1 as usize).0,
                                        &kf_2.features.get_keypoint(best_idx2 as usize).0,
                                        &mut rot_hist, factor, real_idx_kf1 as u32
                                    );
                                };
                            }
                        }
                    }
                }

                i += 1;
                j += 1;
            } else if kf_1_node_id < kf_2_node_id {
                i = self.lower_bound(&kf_1_featvec, &kf_2_featvec, i, j);
            } else {
                j = self.lower_bound(&kf_2_featvec, &kf_1_featvec, j, i);
            }
        }

        if should_check_orientation {
            let (ind_1, ind_2, ind_3) = self.compute_three_maxima(&rot_hist);
            for i in 0..self.histo_length {
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

    fn search_for_triangulation(
        &self, 
        kf_1 : &KeyFrame, kf_2 : &KeyFrame,
        should_check_orientation: bool, _only_stereo: bool, course: bool,
        sensor: Sensor
    ) -> Result<Vec<(usize, usize)>, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("search_for_triangulation");
        // Testing local mapping ... this function return slightly different results than ORB-SLAM. Could be because the optimized pose is slightly different but could also be an error. Uncomment the println lines (and same lines in ORB-SLAM3), then compare the strings.
        //int ORBmatcher::SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo, const bool bCoarse)
        // Some math based on ORB-SLAM2 to avoid some confusion with Sophus.
        // Look here: https://github.com/raulmur/ORB_SLAM2/blob/master/src/ORBmatcher.cc#L657

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

        let factor = 1.0 / (self.histo_length as f32);
        let mut rot_hist = self.construct_rotation_histogram();

        let kf1_featvec = kf_1.bow.as_ref().unwrap().get_feat_vec().get_all_nodes();
        let kf2_featvec = kf_2.bow.as_ref().unwrap().get_feat_vec().get_all_nodes();
        let mut i = 0;
        let mut j = 0;

        let mut total_visited = 0;
        let mut total_outer_loop = 0;
        let mut total_inner_loop = 0;

        while i < kf1_featvec.len() && j < kf2_featvec.len() {
            let kf1_node_id = kf1_featvec[i];
            let kf2_node_id = kf2_featvec[j];
            if kf1_node_id == kf2_node_id {
                total_visited += 1;
                let kf1_indices_size = kf_1.bow.as_ref().unwrap().get_feat_vec().vec_size(kf1_node_id);
                total_outer_loop += kf1_indices_size;

                // let _span = tracy_client::span!("search_for_triangulation_outerloop");

                for i1 in 0..kf1_indices_size {
                    let kf1_index = kf_1.bow.as_ref().unwrap().get_feat_vec().vec_get(kf1_node_id, i1); // = idx1

                    // If there is already a MapPoint skip
                    if let Some((_id, _is_outlier)) = kf_1.get_mp_match(&kf1_index) {
                        continue
                    } else {

                        let stereo1 = match sensor.frame() {
                            FrameSensor::Stereo => {
                                todo!("Stereo");
                            },
                            _ => false
                        };

                        let (kp1, _right1) = kf_1.features.get_keypoint(kf1_index as usize);

                        let mut best_dist = self.low_threshold;
                        let mut best_index = -1;
                        let descriptors_kf_1 = kf_1.features.descriptors.row(kf1_index);

                        let kf2_indices_size = kf_2.bow.as_ref().unwrap().get_feat_vec().vec_size(kf2_node_id);

                        total_inner_loop += kf2_indices_size;
                        // let _span = tracy_client::span!("search_for_triangulation_innerloop");

                        for i2 in 0..kf2_indices_size {
                            let kf2_index = kf_2.bow.as_ref().unwrap().get_feat_vec().vec_get(kf2_node_id, i2); // = idx2

                            // If we have already matched or there is a MapPoint skip
                            if kf_2.get_mp_match(&kf2_index).is_some() || matched_already.contains(&kf2_index) {
                                continue
                            };

                            let stereo2 = match sensor.frame() {
                                FrameSensor::Stereo => {
                                    todo!("Stereo");
                                },
                                _ => false
                            };

                            let descriptors_kf_2 = kf_2.features.descriptors.row(kf2_index);
                            let dist = self.descriptor_distance(&descriptors_kf_1, &descriptors_kf_2);

                            if dist > self.low_threshold || dist > best_dist {
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

                            let epipolar = CAMERA_MODULE.epipolar_constrain(&kp1, &kp2, &r12, &t12, LEVEL_SIGMA2[kp2.octave() as usize]);
                            if course || epipolar {
                                best_index = kf2_index as i32;
                                best_dist = dist;
                            }
                        }

                        if best_index >= 0 {
                            let (kp2, _) = kf_2.features.get_keypoint(best_index as usize);
                            matches[kf1_index as usize] = Some(best_index as usize);
                            matched_already.insert(best_index as u32);
                            if should_check_orientation {
                                self.check_orientation_1(
                                    &kp1,
                                    &kp2,
                                    &mut rot_hist, factor, kf1_index
                                );

                            }
                        }
                    }
                }
                i += 1;
                j += 1;
            } else if kf1_node_id < kf2_node_id {
                i = self.lower_bound(&kf1_featvec, &kf2_featvec, i, j);
            } else {
                j = self.lower_bound(&kf2_featvec, &kf1_featvec, j, i);
            }
        }

        if should_check_orientation {
            let (ind_1, ind_2, ind_3) = self.compute_three_maxima(&rot_hist);
            for i in 0..self.histo_length {
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
                },
                None => {}
            }
        }

        return Ok(matched_pairs);
    }

    fn fuse_from_loop_closing(&self,  kf_id: &Id, scw: &Sim3, mappoints: &Vec<Id>, map: &MapLock, th: i32) ->  Result<Vec<Option<Id>>, Box<dyn std::error::Error>> {
        // int ORBmatcher::Fuse(KeyFrame *pKF, Sophus::Sim3f &Scw, const vector<MapPoint *> &vpPoints, float th, vector<MapPoint *> &vpReplacePoint)

        // Decompose Scw
        let tcw: Pose = (*scw).into();
        let ow = tcw.inverse().get_translation();

        let already_found: HashSet<Id> = map.read().keyframes.get(kf_id).unwrap().get_mp_matches().iter()
            .filter(|item| item.is_some() )
            .map(|item| item.unwrap().0)
            .collect();

        let mut replace_point = vec![None; mappoints.len()];

        let observations_to_add = {
            let mut observations_to_add = vec![];
            let map_lock = map.read();
            let current_kf = map_lock.keyframes.get(kf_id).unwrap();

            // For each candidate MapPoint project and match
            for i in 0..mappoints.len() {
                let mp_id = mappoints[i];
                let mappoint = match map_lock.mappoints.get(&mp_id) {
                    Some(mp) => mp,
                    None => continue
                };

                // Discard already found
                if already_found.get(&mp_id).is_some() {
                    continue;
                }

                // Get 3D Coords.
                let p3dw = mappoint.position;

                // Transform into Camera Coords.
                let p3dc = *tcw.get_rotation() * *p3dw + *tcw.get_translation();

                // Depth must be positive
                if p3dc[2] < 0.0 {
                    continue;
                }

                // Project into Image
                let (u, v) = CAMERA_MODULE.project(DVVector3::new(p3dc));

                // Point must be inside the image
                if !current_kf.features.is_in_image(u, v) {
                    continue;
                }

                // Depth must be inside the scale pyramid of the image
                let max_distance = mappoint.get_max_distance_invariance();
                let min_distance = mappoint.get_min_distance_invariance();
                let po = *p3dw - *ow;
                // let po = DVMatrix::new_expr((p3dw - ow.mat()).into_result()?);
                let dist_3d = po.norm();

                if dist_3d < min_distance || dist_3d > max_distance {
                    continue;
                }

                // Viewing angle must be less than 60 deg
                let pn = mappoint.normal_vector;

                if po.dot(&pn) < 0.5 * dist_3d {
                    continue;
                }

                // Compute predicted scale level
                let predicted_level = mappoint.predict_scale(&dist_3d);

                // Search in a radius
                let radius = th as f32 * SCALE_FACTORS[predicted_level as usize];

                let indices = current_kf.features.get_features_in_area(&u, &v, radius as f64, None);

                if indices.is_empty() {
                    continue;
                }

                // Match to the most similar keypoint in the radius
                let d_mp = &mappoint.best_descriptor;

                let mut best_dist = i32::MAX;
                let mut best_idx = -1;
                for idx in indices {
                    let (kp, _is_right) = current_kf.features.get_keypoint(idx as usize);
                    let kp_level = kp.octave();

                    if kp_level < predicted_level - 1 || kp_level > predicted_level {
                        continue;
                    }

                    let descriptors_kf = current_kf.features.descriptors.row(idx);
                    let dist = self.descriptor_distance(&d_mp, &descriptors_kf);

                    if dist < best_dist {
                        best_dist = dist;
                        best_idx = idx as i32;
                    }
                }

                // If there is already a MapPoint replace otherwise add new measurement
                if best_dist <= self.low_threshold {
                    if let Some((mp_id, _is_outlier)) = current_kf.get_mp_match(&(best_idx as u32)) {
                        replace_point[i] = Some(mp_id);
                    } else {
                        observations_to_add.push((kf_id, mp_id, best_idx));
                    }
                }
            }
            observations_to_add
        };

        let mut map_write_lock = map.write();
        for (kf_id, mp_id, idx) in observations_to_add {
            map_write_lock.add_observation(*kf_id, mp_id, idx as u32, false);
        }

        Ok(replace_point)

    }

    fn fuse(&self,  kf_id: &Id, fuse_candidates: &Vec<Option<(Id, bool)>>, map: &MapLock, th: f32, is_right: bool) {
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
                    let dist = self.descriptor_distance(&lock.mappoints.get(mp_id).unwrap().best_descriptor, &desc_kf);

                    if dist < best_dist {
                        best_dist = dist;
                        best_idx = idx2 as i32;
                    }
                }
            }

            // If there is already a MapPoint replace otherwise add new measurement
            if best_dist <= self.low_threshold {
                let prev_mappoint_match = {
                    let lock = map.read();
                    lock.keyframes.get(kf_id).unwrap().get_mp_match(&(best_idx as u32))
                };

                if let Some((mp_in_kf_id, _is_outlier)) = prev_mappoint_match {
                    let (mp_in_kf_obs, mp_obs) = {
                        let lock = map.read();
                        if lock.mappoints.get(&mp_in_kf_id).is_none() {
                            error!("Can't find mp {} for kf {} at index {}", mp_in_kf_id, kf_id, best_idx);
                        }
                        let mp_in_kf_obs = lock.mappoints.get(&mp_in_kf_id).unwrap().get_observations().len();
                        let mp_obs = lock.mappoints.get(&mp_id).unwrap().get_observations().len();
                        (mp_in_kf_obs, mp_obs)
                    };

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
    }
}