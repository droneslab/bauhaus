use core::{config::{SETTINGS, SYSTEM}, sensor::{FrameSensor, Sensor}};
use std::collections::{HashMap, HashSet};

use log::{info, warn};
use nalgebra::Matrix3;
use opencv::core::KeyPointTraitConst;

use crate::{actors::loop_closing::GBA_KILL_SWITCH, map::{map::Id, pose::{DVTranslation, Pose}}, modules::optimizer::INV_LEVEL_SIGMA2, registered_actors::CAMERA, MapLock};

use super::module::LocalMapOptimizationModule;

pub struct LocalBundleAdjustment { }
impl LocalMapOptimizationModule for LocalBundleAdjustment {

    fn optimize(
        &self, map: &MapLock, keyframe_id: Id
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
            }

            // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
            let mut fixed_cameras = Vec::new();
            let mut ba_fixed_for_kf = HashSet::new();
            for mp_id in &local_mappoints {
                let mp = lock.mappoints.get(&mp_id).unwrap();
                for (kf_id, (_left_index, _right_index)) in mp.get_observations() {
                    match lock.keyframes.get(&kf_id) {
                        Some(kf) => {
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
                        },
                        None => {} // KF could have been deleted during optimization, ok to ignore
                    };
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

            // TODO (concurrency): mbAbortBA 
            // if(pbStopFlag)
            //     optimizer.setForceStopFlag(pbStopFlag);

            let span = tracy_client::span!("local_bundle_adjustment:add_kf_vertices");
            // Set Local KeyFrame vertices
            let mut max_kf_id = 0;
            for kf_id in &local_keyframes {
                match lock.keyframes.get(&kf_id) {
                    Some(kf) => {
                        let set_fixed = *kf_id == lock.initial_kf_id;
                        optimizer.pin_mut().add_frame_vertex(kf.id, (kf.pose).into(), set_fixed);
                        kf_vertex_ids.insert(*kf_id, kf.id);
                        if kf.id > max_kf_id {
                            max_kf_id = kf.id;
                        }
                        if set_fixed {
                            fixed_kfs += 1;
                        } else {
                            kfs_to_optimize += 1;
                        }
                    },
                    None => {} // KF could have been deleted during optimization, ok to ignore
                };
            }

            // Set Fixed KeyFrame vertices
            for kf_id in &fixed_cameras {
                match lock.keyframes.get(&kf_id) {
                    Some(kf) => {
                        optimizer.pin_mut().add_frame_vertex(kf.id, (kf.pose).into(), true);
                        kf_vertex_ids.insert(*kf_id, kf.id);
                        if kf.id > max_kf_id {
                            max_kf_id = kf.id;
                        }
                        fixed_kfs += 1;
                    },
                    None => {} // KF could have been deleted during optimization, ok to ignore
                };
            }


            drop(span);
            let _span = tracy_client::span!("local_bundle_adjustment:add_mp_vertices");

            // Set MapPoint vertices
            for mp_id in &local_mappoints {
                let mp = lock.mappoints.get(mp_id).unwrap();

                let vertex_id = mp.id + max_kf_id + 1;

                optimizer.pin_mut().add_mappoint_vertex(
                    vertex_id,
                    Pose::new(*mp.position, Matrix3::identity()).into(), // create pose out of translation only
                    false, true
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
                                // debug!("Adding edge {} -> {}", vertex_id, kf_vertex);

                                let th_huber_mono: f32 = (5.991 as f32).sqrt();

                                optimizer.pin_mut().add_edge_monocular_binary(
                                    true, vertex_id, kf.id,
                                    mp.id,
                                    kp_un.pt().x, kp_un.pt().y,
                                    INV_LEVEL_SIGMA2[kp_un.octave() as usize],
                                    th_huber_mono
                                );
                                edges += 1;
                                edges_kf_body.push(kf.id);
                            } else {
                                warn!("Local bundle adjustment, monocular observation... Pretty sure this line shouldn't be hit.");
                            }
                        }
                    }
                }
            }
        }

        tracy_client::plot!("LBA: KFs to Optimize", kfs_to_optimize as f64);
        tracy_client::plot!("LBA: Fixed KFs", fixed_kfs as f64);
        tracy_client::plot!("LBA: MPs to Optimize", mps_to_optimize as f64);
        tracy_client::plot!("LBA: Edges", edges as f64);

        // TODO (concurrency): pbStopFlag
        if GBA_KILL_SWITCH.load(std::sync::atomic::Ordering::SeqCst) {
            info!("LM-LBA: Stop requested. Finishing");
            return;
        }

        // Optimize
        {
            let _span = tracy_client::span!("local_bundle_adjustment::optimize");
            optimizer.pin_mut().optimize(10, false, false);
        }

        // Check inlier observations
        let mut mps_to_discard = Vec::new(); // vToErase
        {
            let _span = tracy_client::span!("local_bundle_adjustment::check_inliers");
            let mut i = 0;
            for edge in optimizer.pin_mut().get_mut_xyz_edges().iter() {
                if edge.inner.chi2() > 5.991 || !edge.inner.is_depth_positive() {
                    mps_to_discard.push((edges_kf_body[i], edge.mappoint_id));
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
            for (kf_id, mp_id) in mps_to_discard {
                if map.read().mappoints.get(&mp_id).is_none() {
                    // Mappoint may have been deleted in call to delete_observation, if not enough matches
                    continue;
                }
                map.write().delete_observation(kf_id, mp_id);
            }
        }

        // Recover optimized data
        {
            let _span = tracy_client::span!("local_bundle_adjustment::recover");
            for (kf_id, vertex_id) in kf_vertex_ids {
                let pose = optimizer.recover_optimized_frame_pose(vertex_id);
                let mut lock = map.write();

                if let Some(kf) = lock.keyframes.get_mut(&kf_id) {
                    kf.pose = pose.into();
                } else {
                    // Possible that map actor deleted mappoint after local BA has finished but before
                    // this message is processed
                    continue;
                }
            }

            //Points
            for (mp_id, vertex_id) in mp_vertex_ids {
                if map.read().mappoints.get(&mp_id).is_none() {
                    // Mappoint could have been deleted in the delete_observation call above.
                    continue;
                }

                let position = optimizer.recover_optimized_mappoint_pose(vertex_id);
                let translation = nalgebra::Translation3::new(
                    position.translation[0] as f64,
                    position.translation[1] as f64,
                    position.translation[2] as f64
                );
                let mut lock = map.write();
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
            }
        }
    }
}