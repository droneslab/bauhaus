extern crate g2o;

use std::collections::{HashMap, HashSet};
use cxx::UniquePtr;
use core::{
    config::{SETTINGS, SYSTEM}, sensor::{Sensor, FrameSensor}, matrix::DVMatrix7x7,
};
use log::{warn, debug, trace};
use nalgebra::Matrix3;
use opencv::prelude::KeyPointTraitConst;
use crate::{
    map::{frame::Frame, keyframe::KeyFrame, pose::{Pose, DVTranslation}, map::{Map, Id}}, registered_actors::{FEATURE_DETECTION, CAMERA}, MapLock
};

lazy_static! {
    static ref TH_HUBER_MONO: f32 = (5.991 as f32).sqrt();
    static ref TH_HUBER_2D: f32 = (5.99 as f32).sqrt();
    static ref TH_HUBER_3D: f32 = (7.815 as f32).sqrt();

    // Note: Does not change, so can have multiple copies of this.
    // ORBSLAM3 duplicates this var at every frame and keyframe,
    // but I'm pretty sure that it's set once per-system when the ORBExtractor
    // is created and only ever used by Optimizer.
    // In general, I'd like to remove these kinds of variables away from the
    // frame/keyframe/mappoint implementation and into the object that actually
    // directly uses it.
    pub static ref INV_LEVEL_SIGMA2: Vec<f32> = {
        let n_levels = SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels");
        let mut inv_level_sigma2 = Vec::new();
        for i in 0..n_levels as usize {
            inv_level_sigma2.push(1.0 / LEVEL_SIGMA2[i]);
        }
        inv_level_sigma2
    };
    pub static ref LEVEL_SIGMA2: Vec<f32> = {
        let scale_factor = SETTINGS.get::<f64>(FEATURE_DETECTION, "scale_factor");
        let n_levels = SETTINGS.get::<i32>(FEATURE_DETECTION, "n_levels");
        let mut scale_factors = vec![1.0];
        let mut level_sigma2 = vec![1.0];

        for i in 1..n_levels as usize {
            scale_factors.push(scale_factors[i-1] * (scale_factor as f32));
            level_sigma2.push(scale_factors[i] * scale_factors[i]);
        }
        level_sigma2
    };
}

// int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
// but bRecInit is always set to false
pub fn pose_inertial_optimization_last_frame(
    _frame: &mut Frame, _map: &MapLock
) {
    todo!("IMU... pose_inertial_optimization_last_frame");
    // int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
    // but bRecInit is always set to false
}

pub fn pose_inertial_optimization_last_keyframe(_frame: &mut Frame) -> i32 {
    todo!("IMU... pose_inertial_optimization_last_keyframe");
    // int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
    // but bRecInit is always set to false
}

pub fn optimize_pose(
    frame: &mut Frame, map: &MapLock
) -> Option<i32> {
    let _span = tracy_client::span!("optimize_pose");
    //int Optimizer::PoseOptimization(Frame *pFrame)
    let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");

    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(2, camera_param);

    optimizer.pin_mut().add_frame_vertex(0, (*frame.pose.as_ref().unwrap()).into(), false);

    let mut initial_correspondences = 0;
    let mut mp_indexes = vec![];

    {
        // Take lock to construct factor graph
        let map_read_lock = map.read();
        for i in 0..frame.features.num_keypoints {
            if !frame.mappoint_matches.has(&i) {
                continue;
            }
            let mp_id = frame.mappoint_matches.get(&i);
            let position = match map_read_lock.mappoints.get(&mp_id) {
                Some(mp) => mp.position,
                None => {
                    frame.mappoint_matches.delete_at_indices((i as i32, -1)); // TODO (STEREO)
                    continue;
                },
            };

            let (keypoint, _) = &frame.features.get_keypoint(i as usize);
            mp_indexes.push(i);

            match sensor.frame() {
                FrameSensor::Stereo => {
                    // Stereo observations
                    todo!("Stereo");
                    // let edge = optimizer.create_edge_stereo(
                    //     keypoint.octave(), keypoint.pt().x, keypoint.pt().y,
                    //    frame.keypoints_data.mv_right.get(mp_id).unwrap(),
                    //     self.inv_level_sigma2[keypoint.octave() as usize]
                    // );

                    // {
                    //     let map_read_lock = map.read();
                    //     let position = &map_read_lock.mappoints.get(mp_id).unwrap().position;
                    //     optimizer.add_edge_stereo(
                    //         i as i32, edge.clone(), (position).into()
                    //     );
                    // }
                },
                _ => {
                    // Mono observations
                    frame.mappoint_matches.set_outlier(i as usize, false);
                    optimizer.pin_mut().add_edge_monocular_unary(
                        true, 0, keypoint.octave(), keypoint.pt().x, keypoint.pt().y,
                        INV_LEVEL_SIGMA2[keypoint.octave() as usize],
                        (position).into(),
                        mp_id,
                        *TH_HUBER_MONO
                    );
                    // TODO (mvp): below code sets edge's camera to the frame's camera but are we sure we need it? If yes this could pose a problem since we do not have a C++ implementation of the camera
                    // edge->pCamera = pFrame->mpCamera;
                }
            };

            initial_correspondences += 1;
        }
    }

    if initial_correspondences < 3 {
        return None;
    }

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    let chi2_mono = vec![5.991,5.991,5.991,5.991];
    let _chi2_stereo = vec![7.815,7.815,7.815, 7.815];
    let iterations = vec![10,10,10,10];

    let mut num_bad = 0;
    for iteration in 0..4 {
        optimizer.pin_mut().set_vertex_estimate(
            0, 
            (*frame.pose.as_ref().unwrap()).into()
        );

        optimizer.pin_mut().optimize(iterations[iteration], false);

        num_bad = 0;
        let mut index = 0;
        for mut edge in optimizer.pin_mut().get_mut_xyz_onlypose_edges().iter_mut() {
            if frame.mappoint_matches.is_outlier(&mp_indexes[index]) {
                edge.inner.pin_mut().compute_error();
            }

            let chi2 = edge.inner.chi2();

            if chi2 > chi2_mono[iteration] {
                frame.mappoint_matches.set_outlier(mp_indexes[index] as usize, true);
                edge.inner.pin_mut().set_level(1);
                num_bad += 1;
            } else {
                frame.mappoint_matches.set_outlier(mp_indexes[index] as usize, false);
                edge.inner.pin_mut().set_level(0);
            }

            if iteration == 2 {
                edge.inner.pin_mut().set_robust_kernel(false);
            }
            index += 1;
        }

        // TODO (rigid body) SLAM with respect to a rigid body...probably don't have to do this rn?
        // vpEdgesMono_FHR comes from "SLAM with respect to a rigid body"
        // which I didn't implement...
        // see add_edge_monocular in rust_helper.cpp

        // for(size_t i=0, iend=vpEdgesMono_FHR.size(); i<iend; i++)
        // {
        //     ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

        //     const size_t idx = vnIndexEdgeRight[i];

        //     if(pFrame->mappoint_outliers[idx])
        //     {
        //         e->computeError();
        //     }

        //     const float chi2 = e->chi2();

        //     if(chi2>chi2_mono[it])
        //     {
        //         pFrame->mappoint_outliers[idx]=true;
        //         e->setLevel(1);
        //         nBad++;
        //     }
        //     else
        //     {
        //         pFrame->mappoint_outliers[idx]=false;
        //         e->setLevel(0);
        //     }

        //     if(it==2)
        //         e->setRobustKernel(0);
        // }

        match sensor.frame() {
            FrameSensor::Stereo => {
                todo!("Stereo");
                // for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
                // {
                //     g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

                //     const size_t idx = vnIndexEdgeStereo[i];

                //     if(pFrame->mappoint_outliers[idx])
                //     {
                //         e->computeError();
                //     }

                //     const float chi2 = e->chi2();

                //     if(chi2>chi2_stereo[it])
                //     {
                //         pFrame->mappoint_outliers[idx]=true;
                //         e->setLevel(1);
                //         nBad++;
                //     }
                //     else
                //     {
                //         e->setLevel(0);
                //         pFrame->mappoint_outliers[idx]=false;
                //     }

                //     if(it==2)
                //         e->setRobustKernel(0);
                // }
            },
            _ => {}
        }

        if optimizer.num_edges() < 10 {
            break;
        }
    }

    // Recover optimized pose
    let pose = optimizer.recover_optimized_frame_pose(0);
    frame.pose = Some(pose.into());

    // debug!("Set outliers in pose optimization: {}", num_bad);

    // Return number of inliers
    return Some(initial_correspondences - num_bad);
}

pub fn global_bundle_adjustment(map: &Map, iterations: i32) -> BundleAdjustmentResult {
    let _span = tracy_client::span!("global_bundle_adjustment");
    // void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");
    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(1, camera_param);

    // Set KeyFrame vertices
    let mut kf_vertex_ids = HashMap::new();
    let mut id_count = 0;
    for (kf_id, kf) in &map.keyframes {
        optimizer.pin_mut().add_frame_vertex(
            id_count,
            (kf.pose).into(),
            *kf_id == map.initial_kf_id
        );
        kf_vertex_ids.insert(*kf_id, id_count);
        id_count += 1;
    }

    let mut mp_vertex_ids = HashMap::new();
    // Set MapPoint vertices

    let mut _edges = Vec::new();
    for (mp_id, mappoint) in &map.mappoints {
        optimizer.pin_mut().add_mappoint_vertex(
            id_count,
            Pose::new(*mappoint.position, Matrix3::identity()).into() // create pose out of translation only
        );
        mp_vertex_ids.insert(*mp_id, id_count);

        let mut n_edges = 0;

        //SET EDGES
        for (kf_id, (left_index, _right_index)) in mappoint.get_observations() {
            if !optimizer.has_vertex(id_count) || !optimizer.has_vertex(*kf_id) {
                continue;
            }

            match sensor.frame() {
                FrameSensor::Stereo => {
                    todo!("Stereo, Optimizer lines 194-226");
                },
                _ => {
                    if *left_index != -1 {
                        n_edges += 1;

                        let (keypoint, _) = map.keyframes.get(kf_id).unwrap().features.get_keypoint(*left_index as usize);
                        _edges.push(
                            optimizer.pin_mut().add_edge_monocular_binary(
                                true, id_count, *kf_vertex_ids.get(kf_id).unwrap(),
                                keypoint.pt().x, keypoint.pt().y,
                                INV_LEVEL_SIGMA2[keypoint.octave() as usize],
                                *TH_HUBER_2D
                            )
                        );
                    }
                }
            };

            match sensor.frame() {
                FrameSensor::Stereo => {
                    todo!("Stereo, optimizer lines 229-261");
                    // if pkf->mpcamera2...
                },
                _ => {}
            }
        }

        if n_edges == 0 {
            warn!("Removed vertex");
            optimizer.pin_mut().remove_vertex(id_count);
            mp_vertex_ids.remove(mp_id);
        }
        id_count += 1;
    }

    // Optimize!
    optimizer.pin_mut().optimize(iterations, false);

    BundleAdjustmentResult::new(optimizer, kf_vertex_ids, mp_vertex_ids, vec![], vec![])
}

pub fn local_bundle_adjustment(
    map: &MapLock, keyframe_id: Id, loop_kf: i32
) {
    // void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges)

    let _span = tracy_client::span!("local_bundle_adjustment");

    // Setup optimizer
    let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");
    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];
    let mut optimizer = g2o::ffi::new_sparse_optimizer(1, camera_param);

    let mut mp_vertex_ids = HashMap::new();
    let mut edges_kf_body = Vec::<Id>::new(); // vpEdgeKFBody
    let mut kf_vertex_ids = HashMap::new();

    // This is for debugging, can delete later
    let mut mps_to_optimize = 0;
    let mut fixed_kfs = 0;
    let mut kfs_to_optimize = 0;
    let mut edges = 0;

    {
        // Construct factor graph with read lock, but don't have to have lock to optimize.
        let lock = map.read();

        let span = tracy_client::span!("local_bundle_adjustment:construct_keyframes");
        // Local KeyFrames: First Breath Search from Current Keyframe
        let keyframe = lock.keyframes.get(&keyframe_id).unwrap();
        let mut local_keyframes = vec![keyframe.id];
        let current_map_id = lock.id;
        let mut local_ba_for_kf = HashMap::new();
        local_ba_for_kf.insert(keyframe.id, keyframe.id);

        for kf_id in keyframe.get_covisibility_keyframes(i32::MAX) {
            let kf = lock.keyframes.get(&kf_id).unwrap();
            local_ba_for_kf.insert(kf_id, keyframe.id);
            if kf.origin_map_id == current_map_id {
                local_keyframes.push(kf_id);
            }
            // println!("local keyframe from covisibility: {}", kf_id);
        }

        // Local MapPoints seen in Local KeyFrames
        let mut num_fixed_kf = 0;
        let mut local_mappoints = Vec::<Id>::new();
        let mut local_ba_for_mp = HashMap::new(); // mappoint::mnBALocalForKF
        for kf_i_id in &local_keyframes {
            let kf_i = lock.keyframes.get(&kf_i_id).unwrap();
            if kf_i.id == lock.initial_kf_id {
                num_fixed_kf += 1;
            }
            for mp_match in kf_i.get_mp_matches() {
                if let Some((mp_id, _)) = mp_match {
                    if let Some(mp) = lock.mappoints.get(&mp_id) {
                        if mp.origin_map_id == current_map_id {
                            let mappoint_optimized_for_curr_kf = match local_ba_for_mp.get(&mp_id) {
                                Some(id) => *id == keyframe_id,
                                None => false
                            };
                            if !mappoint_optimized_for_curr_kf {
                                local_mappoints.push(*mp_id);
                                local_ba_for_mp.insert(mp_id, keyframe_id);
                            }
                        }
                    }
                }
            }
            // println!("Matches added {}, matches skipped {} for kf {}", matches_added,matches_skipped, kf_i_id);
        }

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
        let mut fixed_cameras = Vec::new();
        let mut ba_fixed_for_kf = HashSet::new();
        for mp_id in &local_mappoints {
            let mp = lock.mappoints.get(&mp_id).unwrap();
            for (kf_id, (_left_index, _right_index)) in mp.get_observations() {
                let kf = lock.keyframes.get(&kf_id).unwrap();
                let local_ba = match local_ba_for_kf.get(kf_id) {
                    Some(local_ba) => *local_ba != keyframe_id,
                    None => true
                };
                if local_ba && !ba_fixed_for_kf.contains(kf_id) {
                    ba_fixed_for_kf.insert(kf_id);
                    if kf.origin_map_id == current_map_id {
                        fixed_cameras.push(*kf_id);
                    }
                }
            }
        }
        if fixed_cameras.len() + num_fixed_kf == 0 {
            warn!("LM_LBA: There are 0 fixed KF in the optimizations, LBA aborted");
        }

        drop(span);

        match sensor.is_imu() {
            true => {
                todo!("IMU");
                // Need to put this in the C++ constructor for BridgeSparseOptimizer::BridgeSparseOptimizer
                // if (pMap->IsInertial())
                //     solver->setUserLambdaInit(100.0);
            },
            false => {}
        }

        // TODO (mvp): mbAbortBA 
        // if(pbStopFlag)
        //     optimizer.setForceStopFlag(pbStopFlag);

        let span = tracy_client::span!("local_bundle_adjustment:add_kf_vertices");
        // Set Local KeyFrame vertices
        let mut vertex_id = 1;
        for kf_id in &local_keyframes {
            let kf = lock.keyframes.get(kf_id).unwrap();
            let set_fixed = *kf_id == lock.initial_kf_id;
            optimizer.pin_mut().add_frame_vertex(vertex_id, (kf.pose).into(), set_fixed);
            kf_vertex_ids.insert(*kf_id, vertex_id);
            vertex_id += 1;
            if set_fixed {
                fixed_kfs += 1;
            } else {
                kfs_to_optimize += 1;
            }
        }
        // println!("Local keyframes: {:?}", local_keyframes);

        // Set Fixed KeyFrame vertices
        for kf_id in &fixed_cameras {
            let kf = lock.keyframes.get(kf_id).unwrap();
            optimizer.pin_mut().add_frame_vertex(vertex_id, (kf.pose).into(), true);
            kf_vertex_ids.insert(*kf_id, vertex_id);
            vertex_id += 1;
            fixed_kfs += 1;
        }
        // println!("Fixed keyframes: {:?}", fixed_cameras);

        drop(span);
        let _span = tracy_client::span!("local_bundle_adjustment:add_mp_vertices");

        // Set MapPoint vertices
        for mp_id in &local_mappoints {
            let mp = lock.mappoints.get(mp_id).unwrap();

            optimizer.pin_mut().add_mappoint_vertex(
                vertex_id,
                Pose::new(*mp.position, Matrix3::identity()).into() // create pose out of translation only
            );
            mp_vertex_ids.insert(*mp_id, vertex_id);
            mps_to_optimize += 1;

            // Set edges
            for (kf_id, (left_index, _right_index)) in mp.get_observations() {
                let kf = lock.keyframes.get(kf_id).unwrap();
                if kf.origin_map_id != current_map_id {
                    continue
                }


                match sensor.frame() {
                    FrameSensor::Stereo => {
                        todo!("Stereo");
                        // if *left_index != -1 && kf.features.get_mv_right(*left_index as usize).unwrap() >= 0.0 {
                        //     // Stereo observation
                        //     // This is still the left observation, but because it is stereo it needs to be 

                        //     // const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
                        //     // Eigen::Matrix<double,3,1> obs;
                        //     // const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
                        //     // obs << kpUn.pt().x, kpUn.pt().y, kp_ur;

                        //     // g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                        //     // e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        //     // e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        //     // e->setMeasurement(obs);
                        //     // const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave()];
                        //     // Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                        //     // e->setInformation(Info);

                        //     // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        //     // e->setRobustKernel(rk);
                        //     // rk->setDelta(thHuberStereo);

                        //     // e->fx = pKFi->fx;
                        //     // e->fy = pKFi->fy;
                        //     // e->cx = pKFi->cx;
                        //     // e->cy = pKFi->cy;
                        //     // e->bf = pKFi->mbf;

                        //     // optimizer.addEdge(e);
                        //     // vpEdgesStereo.push_back(e);

                        //     // nEdges++;
                        // } else {
                        //     warn!("Local bundle adjustment, stereo observation... Pretty sure this line shouldn't be hit.");
                        // }

                        // // if(pKFi->mpCamera2){
                        // //     int rightIndex = get<1>(mit->second);

                        // //     if(rightIndex != -1 ){
                        // //         rightIndex -= pKFi->NLeft;

                        // //         Eigen::Matrix<double,2,1> obs;
                        // //         cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                        // //         obs << kp.pt().x, kp.pt().y;

                        // //         ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

                        // //         e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        // //         e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        // //         e->setMeasurement(obs);
                        // //         const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave()];
                        // //         e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                        // //         g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        // //         e->setRobustKernel(rk);
                        // //         rk->setDelta(thHuberMono);

                        // //         Sophus::SE3f Trl = pKFi-> GetRelativePoseTrl();
                        // //         e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

                        // //         e->pCamera = pKFi->mpCamera2;

                        // //         optimizer.addEdge(e);
                        // //         vpEdgesBody.push_back(e);
                        // //         vpEdgeKFBody.push_back(pKFi);
                        // //         vpMapPointEdgeBody.push_back(pMP);

                        // //         nEdges++;
                        // //     }
                        // // }
                    },
                    _ => {
                        // Monocular observation
                        if *left_index != -1 && kf.features.get_mv_right(*left_index as usize).is_none() {
                            let (kp_un, _) = kf.features.get_keypoint(*left_index as usize);
                            let kf_vertex = *kf_vertex_ids.get(&kf.id).unwrap();
                            // debug!("Adding edge {} -> {}", vertex_id, kf_vertex);

                            optimizer.pin_mut().add_edge_monocular_binary(
                                false, vertex_id, kf_vertex,
                                kp_un.pt().x, kp_un.pt().y,
                                INV_LEVEL_SIGMA2[kp_un.octave() as usize],
                                *TH_HUBER_MONO
                            );

                            // TODO (mvp): below code sets edge's camera to the frame's camera
                            // but are we sure we need it?
                            // edge->pCamera = pFrame->mpCamera;
                            edges += 1;
                            edges_kf_body.push(kf.id);
                        } else {
                            warn!("Local bundle adjustment, monocular observation... Pretty sure this line shouldn't be hit.");
                        }
                    }
                }
            }
            vertex_id += 1;
        }
    }
    tracy_client::plot!("LBA: KFs to Optimize", kfs_to_optimize as f64);
    tracy_client::plot!("LBA: Fixed KFs", fixed_kfs as f64);
    tracy_client::plot!("LBA: MPs to Optimize", mps_to_optimize as f64);
    tracy_client::plot!("LBA: Edges", edges as f64);
    trace!("LBA:{},{},{},{}", kfs_to_optimize, fixed_kfs, mps_to_optimize, edges);

    // Optimize
    {
        let _span = tracy_client::span!("local_bundle_adjustment::optimize");
        optimizer.pin_mut().optimize(10, false);
    }

    // Check inlier observations
    let mut mps_to_discard = Vec::new(); // vToErase
    {
        let _span = tracy_client::span!("local_bundle_adjustment::check_inliers");
        let mut i = 0;
        for edge in optimizer.pin_mut().get_mut_xyz_onlypose_edges().iter() {
            if edge.inner.chi2() > 5.991 || !edge.inner.is_depth_positive() {
                mps_to_discard.push((edge.mappoint_id, edges_kf_body[i]));
            }
            i += 1;
        }
    }


    if matches!(sensor.frame(), FrameSensor::Stereo) {
        todo!("Stereo");
        // For the below vectors, will need to make vector in g2o bindings similar to
        // xyz_onlypose_edges and iterate over it like get_mut_xyz_onlypose_edges above

        // for (_mp_id, _edge) in all_edges_body {
            // ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
            // MapPoint* pMP = vpMapPointEdgeBody[i];

            // if(pMP->isBad())
            //     continue;

            // if(e->chi2()>5.991 || !e->isDepthPositive())
            // {
            //     KeyFrame* pKFi = vpEdgeKFBody[i];
            //     vToErase.push_back(make_pair(pKFi,pMP));
            // }
        // }

        // for (_mp_id, _edge) in all_edges_stereo {
            // g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            // MapPoint* pMP = vpMapPointEdgeStereo[i];

            // if(pMP->isBad())
            //     continue;

            // if(e->chi2()>7.815 || !e->isDepthPositive())
            // {
            //     KeyFrame* pKFi = vpEdgeKFStereo[i];
            //     vToErase.push_back(make_pair(pKFi,pMP));
            // }
        // }
    }

    {
        let _span = tracy_client::span!("local_bundle_adjustment::discard");
        for (mp_id, kf_id) in mps_to_discard {
            map.write().keyframes.get_mut(&kf_id).unwrap().delete_mp_match(mp_id);
            map.write().mappoints.get_mut(&mp_id).unwrap().delete_observation(&kf_id);
        }
    }

    // Recover optimized data

    {
        let _span = tracy_client::span!("local_bundle_adjustment::recover");
        for (kf_id, vertex_id) in kf_vertex_ids {
            let pose = optimizer.recover_optimized_frame_pose(vertex_id);
            let mut lock = map.write();
            let initial_kf_id = lock.initial_kf_id;

            if let Some(kf) = lock.keyframes.get_mut(&kf_id) {
                if loop_kf == initial_kf_id {
                    kf.pose = pose.into();
                } else {
                    todo!("MVP LOOP CLOSING");
                    // Code from ORBSLAM, not sure if we need it
                    // pKF->mTcwGBA = Sophus::SE3d(SE3quat.rotation(),SE3quat.translation()).cast<float>();
                    // pKF->mnBAGlobalForKF = nLoopKF;
                }
            } else {
                // Possible that map actor deleted mappoint after local BA has finished but before
                // this message is processed
                continue;
            }
        }

        //Points
        for (mp_id, vertex_id) in mp_vertex_ids {
            let position = optimizer.recover_optimized_mappoint_pose(vertex_id);
            let translation = nalgebra::Translation3::new(
                position.translation[0] as f64,
                position.translation[1] as f64,
                position.translation[2] as f64
            );
            let mut lock = map.write();

            if loop_kf == lock.initial_kf_id {
                match lock.mappoints.get_mut(&mp_id) {
                    // Possible that map actor deleted mappoint after local BA has finished but before
                    // this message is processed
                    Some(mp) => {
                        mp.position = DVTranslation::new(translation.vector);
                        let norm_and_depth = lock.mappoints.get(&mp_id).unwrap().get_norm_and_depth(&lock);
                        if norm_and_depth.is_some() {
                            lock.mappoints.get_mut(&mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
                        }
                    },
                    None => continue,
                };
            } else {
                todo!("MVP LOOP CLOSING");
                // pMP->mPosGBA = vPoint->estimate().cast<float>();
                // pMP->mnBAGlobalForKF = nLoopKF;
            }
        }
    }
}

pub fn optimize_essential_graph_6dof() {
    todo!("STEREO, RGBD. Used by loop closing");
    // void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
    //                                 const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
    //                                 const LoopClosing::KeyFrameAndPose &CorrectedSim3,
    //                                 const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
}

pub fn optimize_essential_graph_4dof() {
    todo!("IMU. Used by Loop closing");
    // void static OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
    //                                 const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
    //                                 const LoopClosing::KeyFrameAndPose &CorrectedSim3,
    //                                 const map<KeyFrame *, set<KeyFrame *> > &LoopConnections);
}
pub fn optimize_essential_graph() {
    todo!("LOOP CLOSING");
    // void static OptimizeEssentialGraph(KeyFrame* pCurKF, vector<KeyFrame*> &vpFixedKFs, vector<KeyFrame*> &vpFixedCorrectedKFs,
    //                                 vector<KeyFrame*> &vpNonFixedKFs, vector<MapPoint*> &vpNonCorrectedMPs);

}

pub fn optimize_sim3(
    map: &MapLock, _kf1: Id, _kf2: Id, _matched_mps: &mut Vec<Id>, _s12: &Pose, _th2: f32, _b_fix_scale: bool
) -> (i32, DVMatrix7x7<f32>) {
    todo!("LOOP CLOSING");
    // int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, g2o::Sim3 &g2oS12, const float th2,
    //                             const bool bFixScale, Eigen::Matrix<double,7,7> &mAcumHessian, const bool bAllPoints)
    // but bAllPoints is always set to true
    // returns vpMatches1, mAcumHessian
}

#[derive(Debug)]
pub struct BundleAdjustmentResult {
    pub new_mp_poses: HashMap::<Id, DVTranslation>,
    pub new_kf_poses: HashMap::<Id, Pose>,
    pub mps_to_discard: Vec<i32>,
    pub kfs_to_discard: Vec<i32>,
}
impl BundleAdjustmentResult {
    pub fn new(
        optimizer: UniquePtr<g2o::ffi::BridgeSparseOptimizer>, kf_vertex_ids: HashMap<i32, i32>, mp_vertex_ids: HashMap<i32, i32>,
        mps_to_discard: Vec<i32>, kfs_to_discard: Vec<i32>
    ) -> Self {
        // Recover optimized data
        // Keyframes
        let mut new_kf_poses = HashMap::<Id, Pose>::new();
        for (kf_id, vertex_id) in kf_vertex_ids {
            let pose = optimizer.recover_optimized_frame_pose(vertex_id);
            new_kf_poses.insert(kf_id, pose.into());
        }

        //Points
        let mut new_mp_poses = HashMap::<Id, DVTranslation>::new();
        for (mp_id, vertex_id) in mp_vertex_ids {
            let position = optimizer.recover_optimized_mappoint_pose(vertex_id);
            let translation = nalgebra::Translation3::new(
                position.translation[0] as f64,
                position.translation[1] as f64,
                position.translation[2] as f64
            );

            new_mp_poses.insert(mp_id, DVTranslation::new(translation.vector));
        }

        BundleAdjustmentResult { 
            new_mp_poses,
            new_kf_poses,
            mps_to_discard,
            kfs_to_discard
        }

    }
}