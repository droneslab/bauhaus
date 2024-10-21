use core::{config::{SETTINGS, SYSTEM}, sensor::{FrameSensor, Sensor}};
use std::{cmp::min, collections::{HashMap, HashSet, VecDeque}};

use log::{debug, info, warn};
use nalgebra::Matrix3;
use opencv::core::KeyPointTraitConst;

use crate::{actors::loop_closing::GBA_KILL_SWITCH, map::{map::Id, pose::{DVTranslation, Pose}}, modules::optimizer::INV_LEVEL_SIGMA2, registered_actors::{CAMERA, CAMERA_MODULE}, MapLock};

use super::{imu::{ImuBias, ImuPreIntegrated}, module_definitions::LocalMapOptimizationModule};

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

        let mut optimizer = match sensor.is_imu() {
            true => g2o::ffi::new_sparse_optimizer(1, camera_param, 100.0),
            false => g2o::ffi::new_sparse_optimizer(1, camera_param, 0.0),
        };

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

            let _span = tracy_client::span!("local_bundle_adjustment:construct_factor_graph");
            // Local KeyFrames: First Breath Search from Current Keyframe
            let keyframe = lock.keyframes.get(&keyframe_id).unwrap();
            let mut local_keyframes = vec![keyframe.id];
            let current_map_id = lock.id;
            let mut local_ba_for_kf = HashMap::new();
            local_ba_for_kf.insert(keyframe.id, keyframe.id);

            let covisibility_kfs = keyframe.get_covisibility_keyframes(i32::MAX);
            println!("LBA! Covisibility kfs: {}, {:?}", covisibility_kfs.len(), covisibility_kfs);

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

            // TODO (concurrency): mbAbortBA 
            // if(pbStopFlag)
            //     optimizer.setForceStopFlag(pbStopFlag);

            // Set Local KeyFrame vertices
            let mut max_kf_id = 0;
            for kf_id in &local_keyframes {
                match lock.keyframes.get(&kf_id) {
                    Some(kf) => {
                        let set_fixed = *kf_id == lock.initial_kf_id;
                        optimizer.pin_mut().add_vertex_se3expmap(kf.id, (kf.get_pose()).into(), set_fixed);
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
                        optimizer.pin_mut().add_vertex_se3expmap(kf.id, (kf.get_pose()).into(), true);
                        kf_vertex_ids.insert(*kf_id, kf.id);
                        if kf.id > max_kf_id {
                            max_kf_id = kf.id;
                        }
                        fixed_kfs += 1;
                    },
                    None => {} // KF could have been deleted during optimization, ok to ignore
                };
            }


            // Set MapPoint vertices
            for mp_id in &local_mappoints {
                let mp = lock.mappoints.get(mp_id).unwrap();

                let vertex_id = mp.id + max_kf_id + 1;

                optimizer.pin_mut().add_vertex_sbapointxyz(
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

                                optimizer.pin_mut().add_edge_se3_project_xyz_monocular_binary(
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

        {
            let _span = tracy_client::span!("local_bundle_adjustment::post_process");

            // Check inlier observations
            let mut mps_to_discard = Vec::new(); // vToErase
            let mut i = 0;
            for edge in optimizer.pin_mut().get_mut_xyz_edges().iter() {
                if edge.inner.chi2() > 5.991 || !edge.inner.is_depth_positive() {
                    mps_to_discard.push((edges_kf_body[i], edge.mappoint_id));
                }
                i += 1;
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

            for (kf_id, mp_id) in mps_to_discard {
                if map.read().mappoints.get(&mp_id).is_none() {
                    // Mappoint may have been deleted in call to delete_observation, if not enough matches
                    continue;
                }
                map.write().delete_observation(kf_id, mp_id);
            }

            // Recover optimized data
            for (kf_id, vertex_id) in kf_vertex_ids {
                let pose = optimizer.recover_optimized_frame_pose(vertex_id);
                let mut lock = map.write();

                if let Some(kf) = lock.keyframes.get_mut(&kf_id) {
                    kf.set_pose(pose.into());
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

        map.write().map_change_index += 1;
    }
}

pub fn local_inertial_ba(map: &MapLock, curr_kf_id: Id, large: bool, rec_init: bool, tracked_mappoint_depths: HashMap<Id, f64>, sensor: Sensor) {
    // void Optimizer::LocalInertialBA(KeyFrame *pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge, bool bRecInit)

    let _span = tracy_client::span!("local_bundle_adjustment (inertial)");

    // Map* pCurrentMap = pKF->GetMap(); // TODO (multi-maps) should use keyframe's map instead of general map

    let _span_construct = tracy_client::span!("local_bundle_adjustment:construct_factor_graph (inertial)");

    let mut max_opt = 10;
    let mut opt_it = 10;
    if large {
        max_opt = 25;
        opt_it = 4;
    }
    let n_d = min(map.read().keyframes.len() as i32 - 2, max_opt);
    let max_kf_id = curr_kf_id;

    let neighbor_kfs = map.read().keyframes.get(&curr_kf_id).unwrap().get_covisibility_keyframes(i32::MAX); // vpNeighsKFs

    let mut optimizable_kfs: VecDeque<Id> = VecDeque::new(); // vpOptimizableKFs
    optimizable_kfs.push_front(curr_kf_id); 
    let mut local_ba_for_kf = HashMap::new(); // keyframe::mnBALocalForKF
    local_ba_for_kf.insert(curr_kf_id, curr_kf_id);
    let mut local_mappoints: Vec<Id> = vec![]; // lLocalMapPoints
    let mut local_ba_for_mp: HashMap<Id, Id> = HashMap::new(); // mappoint::mnBALocalForKF

    {
        let lock = map.read();
        for _ in 1..n_d {
            let kf_id = *optimizable_kfs.back().unwrap();
            let kf = lock.keyframes.get(&kf_id).unwrap();
            if kf.prev_kf_id.is_none() {
                break;
            } else {
                optimizable_kfs.push_back(kf.prev_kf_id.unwrap());
                local_ba_for_kf.insert(kf_id, curr_kf_id);
            }
        }

        // Optimizable points seen by temporal optimizable keyframes
        for i in 0..optimizable_kfs.len() {
            let mps = lock.keyframes.get(&optimizable_kfs[i]).unwrap().get_mp_matches();
            for data in mps {
                if let Some((mp_id, _)) = data {
                    if let Some(mp) = lock.mappoints.get(&mp_id) {
                        if local_ba_for_mp.get(mp_id).is_none() || * local_ba_for_mp.get(mp_id).unwrap() != curr_kf_id {
                            local_mappoints.push(*mp_id);
                            local_ba_for_mp.insert(*mp_id, curr_kf_id);
                        }
                    }
                }
            }
        }
    }

    // Fixed Keyframe: First frame previous KF to optimization window)
    let mut fixed_keyframes: Vec<Id> = vec![]; // lFixedKeyFrames
    let mut ba_fixed_for_kf: HashMap<Id, Id> = HashMap::new(); // keyframe::mnBAFixedForKF
    let mut ba_local_for_kf: HashMap<Id, Id> = HashMap::new(); // keyframe::mnBALocalForKF
    {
        let lock = map.read();
        let last_opt_kf = lock.keyframes.get(& optimizable_kfs.back().unwrap()).unwrap();
        if let Some(prev_kf_id) = last_opt_kf.prev_kf_id {
            fixed_keyframes.push(prev_kf_id);
            ba_fixed_for_kf.insert(prev_kf_id, curr_kf_id);
        } else {
            ba_local_for_kf.insert(* optimizable_kfs.back().unwrap(), 0);
            ba_fixed_for_kf.insert(* optimizable_kfs.back().unwrap(), curr_kf_id);
            fixed_keyframes.push(* optimizable_kfs.back().unwrap());
            optimizable_kfs.pop_back();
        }
    }

    // Optimizable visual KFs
    // const int maxCovKF = 0;
    let opt_vis_kfs: Vec<Id> = vec![]; // lpOptVisKFs
    for _ in neighbor_kfs {
        // todo sofiya what is this?
        if opt_vis_kfs.len() >= 0 {
            break;
        }
        todo!("Huh?");
        
        // KeyFrame* pKFi = vpNeighsKFs[i];
        // if(pKFi->mnBALocalForKF == pKF->mnId || pKFi->mnBAFixedForKF == pKF->mnId)
        //     continue;
        // pKFi->mnBALocalForKF = pKF->mnId;
        // if(!pKFi->isBad() && pKFi->GetMap() == pCurrentMap)
        // {
        //     lpOptVisKFs.push_back(pKFi);

        //     vector<MapPoint*> vpMPs = pKFi->GetMapPointMatches();
        //     for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        //     {
        //         MapPoint* pMP = *vit;
        //         if(pMP)
        //             if(!pMP->isBad())
        //                 if(pMP->mnBALocalForKF!=pKF->mnId)
        //                 {
        //                     lLocalMapPoints.push_back(pMP);
        //                     pMP->mnBALocalForKF=pKF->mnId;
        //                 }
        //     }
        // }
    }

    // Fixed KFs which are not covisible optimizable
    {
        let lock = map.read();
        let max_fix_kf = 200;
        for mp_id in &local_mappoints {
            let observations = lock.mappoints.get(&mp_id).unwrap().get_observations();
            for (kf_id, _) in observations {
                let local =  (ba_local_for_kf.get(&kf_id).is_some() && * ba_local_for_kf.get(&kf_id).unwrap() != curr_kf_id) || ba_local_for_kf.get(&kf_id).is_none();
                let fixed =  (ba_fixed_for_kf.get(&kf_id).is_some() && * ba_fixed_for_kf.get(&kf_id).unwrap() != curr_kf_id) || ba_fixed_for_kf.get(&kf_id).is_none();

                if local && fixed {
                    ba_fixed_for_kf.insert(* kf_id, curr_kf_id);
                }
            }

            if fixed_keyframes.len() >= max_fix_kf {
                break;
            }
        }
    }

    let non_fixed = fixed_keyframes.len() == 0;

    // Setup optimizer
    // ... Note... pretty sure camera params aren't necessary for this optimization but throwing them in here anyway just in case
    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = if large {
        // to avoid iterating for finding optimal lambda
        g2o::ffi::new_sparse_optimizer(5, camera_param, 1e-2)
    } else {
        g2o::ffi::new_sparse_optimizer(5, camera_param, 1e10)
    };


    {
        let lock = map.read();
        // Set Local temporal KeyFrame vertices
        for kf_id in &optimizable_kfs {
            let kf = lock.keyframes.get(&kf_id).unwrap();

            super::optimizer::add_vertex_pose_keyframe(&mut optimizer, kf, false, kf.id);

            if kf.imu_data.is_imu_initialized {
                optimizer.pin_mut().add_vertex_velocity(
                    max_kf_id + 3 * kf.id + 1,
                    false,
                    kf.imu_data.velocity.unwrap().into()
                );
                optimizer.pin_mut().add_vertex_gyrobias(
                    max_kf_id + 3 * kf.id + 2,
                    false,
                    kf.imu_data.imu_bias.get_gyro_bias().into()
                );
                optimizer.pin_mut().add_vertex_accbias(
                    max_kf_id + 3 * kf.id + 3,
                    false,
                    kf.imu_data.imu_bias.get_acc_bias().into()
                );
            }
        }

        // Set Local visual KeyFrame vertices
        for kf_id in &opt_vis_kfs {
            let kf = lock.keyframes.get(&kf_id).unwrap();
            super::optimizer::add_vertex_pose_keyframe(&mut optimizer, kf, false, kf.id);
        }

        // Set Fixed KeyFrame vertices
        for kf_id in &fixed_keyframes {
            let kf = lock.keyframes.get(&kf_id).unwrap();
            super::optimizer::add_vertex_pose_keyframe(&mut optimizer, kf, true, kf.id);

            // This should be done only for keyframe just before temporal window
            if kf.imu_data.is_imu_initialized { 
                optimizer.pin_mut().add_vertex_velocity(
                    max_kf_id + 3 * kf.id + 1,
                    true,
                    kf.imu_data.velocity.unwrap().into()
                );
                optimizer.pin_mut().add_vertex_gyrobias(
                    max_kf_id + 3 * kf.id + 2,
                    true,
                    kf.imu_data.imu_bias.get_gyro_bias().into()
                );
                optimizer.pin_mut().add_vertex_accbias(
                    max_kf_id + 3 * kf.id + 3,
                    true,
                    kf.imu_data.imu_bias.get_acc_bias().into()
                );
            }
        }
    }

    // Create intertial constraints
    let mut new_imu_preintegrated_for_kfs: HashMap<Id, ImuPreIntegrated> = HashMap::new();
    {
        let lock = map.read();
        for i in 0..optimizable_kfs.len() {
            let kf_id = optimizable_kfs[i];
            let kf = lock.keyframes.get(&kf_id).unwrap();
            if kf.prev_kf_id.is_none() {
                debug!("No inertial link to previous frame!!");
                continue;
            }
            let prev_kf = lock.keyframes.get(&kf.prev_kf_id.unwrap()).unwrap();

            if kf.imu_data.imu_preintegrated.is_none() ||
                !kf.imu_data.is_imu_initialized ||
                prev_kf.imu_data.imu_preintegrated.is_none() ||
                !prev_kf.imu_data.is_imu_initialized {
                    warn!("Error building inertial edge... for kf {}: {} {} {} {}", kf.id, kf.imu_data.imu_preintegrated.is_none(), !kf.imu_data.is_imu_initialized, prev_kf.imu_data.imu_preintegrated.is_none(), !prev_kf.imu_data.is_imu_initialized);
                    continue;
            }
            let mut imu_preintegrated = kf.imu_data.imu_preintegrated.as_ref().unwrap().clone();
            imu_preintegrated.set_new_bias(prev_kf.imu_data.imu_bias);

            let mut set_robust_kernel = false;
            let mut delta = 0.0;
            let mut set_information = false;
            let mut information_weight = 0.0;
            if i == optimizable_kfs.len() - 1 || rec_init {
                // All inertial residuals are included without robust cost function, but not that one linking the
                // last optimizable keyframe inside of the local window and the first fixed keyframe out. The
                // information matrix for this measurement is also downweighted. This is done to avoid accumulating
                // error due to fixing variables.
                set_robust_kernel = true;
                delta = (16.92 as f32).sqrt() as f32;
                if i == optimizable_kfs.len() - 1 {
                    set_information = true;
                    information_weight = 1e-2;
                }
            }

            // todo sofiya double check that the above sets this correctly:
            // vei[i]->setInformation(vei[i]->information()*1e-2);

            optimizer.pin_mut().add_edge_inertial_gs(
                prev_kf.id,
                max_kf_id + 3 * prev_kf.id + 1,
                max_kf_id + 3 * prev_kf.id + 2,
                max_kf_id + 3 * prev_kf.id + 3,
                kf_id,
                max_kf_id + 3 * kf_id + 1,
                max_kf_id + 3 * kf_id + 2,
                max_kf_id + 3 * kf_id + 3,
                (& imu_preintegrated).into(),
                set_robust_kernel,
                delta,
                set_information,
                information_weight
            );

            optimizer.pin_mut().add_edge_gyro_and_acc(
                max_kf_id + 3 * prev_kf.id + 2,
                max_kf_id + 3 * kf_id + 2,
                max_kf_id + 3 * prev_kf.id + 3, 
                max_kf_id + 3 * kf_id + 3,
                (& imu_preintegrated).into()
            );

            new_imu_preintegrated_for_kfs.insert(kf_id, imu_preintegrated);
        }
    };
    for (kf_id, new_imu_preintegrated) in new_imu_preintegrated_for_kfs {
        let mut lock = map.write();
        lock.keyframes.get_mut(&kf_id).unwrap().imu_data.imu_preintegrated = Some(new_imu_preintegrated);
    }


    let th_huber_mono: f32 = (5.991 as f32).sqrt();
    let th_huber_stereo: f32 = (7.815 as f32).sqrt();
    let chi2_mono2 = 5.991;
    let chi2_stereo2 = 7.815;

    let ini_mp_id = max_kf_id * 5; // iniMPid

    // todo sofiya do we need this?
    // map<int,int> mVisEdges;
    // for(int i=0;i<N;i++)
    // {
    //     KeyFrame* pKFi = vpOptimizableKFs[i];
    //     mVisEdges[pKFi->mnId] = 0;
    // }
    // for(list<KeyFrame*>::iterator lit=lFixedKeyFrames.begin(), lend=lFixedKeyFrames.end(); lit!=lend; lit++)
    // {
    //     mVisEdges[(*lit)->mnId] = 0;
    // }

    // Set MapPoint vertices
    let mut mp_vertex_ids = HashMap::new();
    let mut edges_kf_mono = Vec::<Id>::new(); // vpEdgeKFMono
    {
        // This is for debugging, can delete later
        let mut mps_to_optimize = 0;
        let mut fixed_kfs = 0;
        let mut kfs_to_optimize = 0;
        let mut edges = 0;

        let lock = map.read();
        for mp_id in &local_mappoints {
            let mp = lock.mappoints.get(mp_id).unwrap();

            let vertex_id = mp.id + ini_mp_id + 1;
            optimizer.pin_mut().add_vertex_sbapointxyz(
                vertex_id,
                Pose::new(*mp.position, Matrix3::identity()).into(), // create pose out of translation only
                false, true
            );
            mp_vertex_ids.insert(*mp_id, vertex_id);
            mps_to_optimize += 1;

            // Create visual constraints
            for (kf_id, (left_index, _right_index)) in mp.get_observations() {
                if (local_ba_for_kf.get(&kf_id).is_some() && * local_ba_for_kf.get(&kf_id).unwrap() != curr_kf_id) &&
                    ba_fixed_for_kf.get(&kf_id).is_some() && * ba_fixed_for_kf.get(&kf_id).unwrap() != curr_kf_id
                {
                    continue;
                }
                let kf = lock.keyframes.get(kf_id).unwrap();


                match sensor.frame() {
                    FrameSensor::Stereo => {
                        todo!("Stereo");
                        // kpUn = pKFi->mvKeysUn[leftIndex];
                        // mVisEdges[pKFi->mnId]++;

                        // const float kp_ur = pKFi->mvuRight[leftIndex];
                        // Eigen::Matrix<double,3,1> obs;
                        // obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                        // EdgeStereo* e = new EdgeStereo(0);

                        // e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                        // e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                        // e->setMeasurement(obs);

                        // // Add here uncerteinty
                        // const float unc2 = pKFi->mpCamera->uncertainty2(obs.head(2));

                        // const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                        // e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                        // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        // e->setRobustKernel(rk);
                        // rk->setDelta(thHuberStereo);

                        // optimizer.addEdge(e);
                        // vpEdgesStereo.push_back(e);
                        // vpEdgeKFStereo.push_back(pKFi);
                        // vpMapPointEdgeStereo.push_back(pMP);
                    },
                    _ => {
                        // Monocular left observation
                        if *left_index != -1 && kf.features.get_mv_right(*left_index as usize).is_none() {
                            let (kp_un, _) = kf.features.get_keypoint(*left_index as usize);
                            // debug!("Adding edge {} -> {}", vertex_id, kf_vertex);

                            // Add here uncerteinty
                            let unc2 = CAMERA_MODULE.uncertainty;
                            let inv_sigma2 = INV_LEVEL_SIGMA2[kp_un.octave() as usize] / unc2;


                            optimizer.pin_mut().add_edge_mono_binary(
                                true, vertex_id, kf.id,
                                mp.id,
                                kp_un.pt().x, kp_un.pt().y,
                                inv_sigma2,
                                th_huber_mono
                            );
                            edges += 1;
                            edges_kf_mono.push(kf.id);
                        } else {
                            warn!("TODO monocular right observation");
                            // Monocular right observation
                            // if(pKFi->mpCamera2){
                            //     int rightIndex = get<1>(mit->second);

                            //     if(rightIndex != -1 ){
                            //         rightIndex -= pKFi->NLeft;
                            //         mVisEdges[pKFi->mnId]++;

                            //         Eigen::Matrix<double,2,1> obs;
                            //         cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
                            //         obs << kp.pt.x, kp.pt.y;

                            //         EdgeMono* e = new EdgeMono(1);

                            //         e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                            //         e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                            //         e->setMeasurement(obs);

                            //         // Add here uncerteinty
                            //         const float unc2 = pKFi->mpCamera->uncertainty2(obs);

                            //         const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave]/unc2;
                            //         e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                            //         g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                            //         e->setRobustKernel(rk);
                            //         rk->setDelta(thHuberMono);

                            //         optimizer.addEdge(e);
                            //         vpEdgesMono.push_back(e);
                            //         vpEdgeKFMono.push_back(pKFi);
                            //         vpMapPointEdgeMono.push_back(pMP);
                            //     }
                            // }
                        }
                    }
                }
            }
        }
    }

    // todo sofiya do we need this?
    // for(map<int,int>::iterator mit=mVisEdges.begin(), mend=mVisEdges.end(); mit!=mend; mit++)
    // {
    //     assert(mit->second>=3);
    // }

    drop(_span_construct);

    // Optimize
    {
        let _span = tracy_client::span!("local_bundle_adjustment::optimize (inertial)");
        optimizer.pin_mut().optimize(opt_it, false, true);
    }

    let _span = tracy_client::span!("local_bundle_adjustment::post_process (inertial)");

    // Check inlier observations
    // Mono
    let mut to_erase = Vec::new(); // vToErase
    let mut i = 0;
    for edge in optimizer.pin_mut().get_mut_xyz_edges().iter() {
        // TODO Sofiya edges_kf_mono might not match up with edges from this iterator
        let close = tracked_mappoint_depths.get(&edge.mappoint_id).is_some() && *tracked_mappoint_depths.get(&edge.mappoint_id).unwrap() < 10.0;
        if (edge.inner.chi2() > chi2_mono2 && !close) || 
            (edge.inner.chi2() > 1.5 * chi2_mono2 && close) || 
            !edge.inner.is_depth_positive() 
        {
            to_erase.push((edges_kf_mono[i], edge.mappoint_id));
        }
        i += 1;
    }

    match sensor.frame() {
        FrameSensor::Stereo => {
            todo!("Stereo");
            // for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
            // {
            //     EdgeStereo* e = vpEdgesStereo[i];
            //     MapPoint* pMP = vpMapPointEdgeStereo[i];

            //     if(pMP->isBad())
            //         continue;

            //     if(e->chi2()>chi2Stereo2)
            //     {
            //         KeyFrame* pKFi = vpEdgeKFStereo[i];
            //         vToErase.push_back(make_pair(pKFi,pMP));
            //     }
            // }
        },
        _ => {}
    }

    // TODO Sofiya ... ignoring this for now because there is not an easy way to get err and err_end out of rust_helper
    // Get Map Mutex and erase outliers
    // From ORBSLAM: TODO: Some convergence problems have been detected here
    // if((2*err < err_end || isnan(err) || isnan(err_end)) && !bLarge) //bGN)
    // {
    //     cout << "FAIL LOCAL-INERTIAL BA!!!!" << endl;
    //     return;
    // }

    if !to_erase.is_empty() {
        let mut lock = map.write();
        for (kf_id, mp_id) in to_erase {
            if lock.mappoints.get(&mp_id).is_none() {
                // Mappoint may have been deleted in call to delete_observation, if not enough matches
                continue;
            }
            lock.delete_observation(kf_id, mp_id);
        }
    }

    for kf_id in &fixed_keyframes {
        let pose = optimizer.recover_optimized_frame_pose(*kf_id);
        let mut lock = map.write();
        if let Some(kf) = lock.keyframes.get_mut(kf_id) {
            kf.set_pose(pose.into());
        } else {
            // Possible that map actor deleted mappoint after local BA has finished but before
            // this message is processed
            continue;
        }
    }

    // Recover optimized data
    // Local temporal Keyframes
    for i in 0..optimizable_kfs.len() {
        let kf_id = optimizable_kfs[i];

        let pose = optimizer.recover_optimized_vertex_pose(kf_id);
        map.write().keyframes.get_mut(&kf_id).unwrap().set_pose(pose.into());

        if map.read().keyframes.get(&kf_id).unwrap().imu_data.is_imu_initialized {
            let velocity = optimizer.recover_optimized_vertex_velocity(max_kf_id + 3 * kf_id + 1);
            let estimate = optimizer.recover_optimized_inertial(
                max_kf_id + 3 * kf_id + 2,
                max_kf_id + 3 * kf_id + 3,
                -1,
                -1
            );
            let b = ImuBias {
                bax: estimate.vb[3],
                bay: estimate.vb[4],
                baz: estimate.vb[5],
                bwx: estimate.vb[0],
                bwy: estimate.vb[1],
                bwz: estimate.vb[2],
            };
            map.write().keyframes.get_mut(&kf_id).unwrap().imu_data.imu_bias = b;
        }
    }

    // Local visual KeyFrame
    for i in 0..opt_vis_kfs.len() {
        let kf_id = opt_vis_kfs[i];
        let pose = optimizer.recover_optimized_vertex_pose(kf_id);
        map.write().keyframes.get_mut(&kf_id).unwrap().set_pose(pose.into());
    }

    //Points
    for i in 0..local_mappoints.len() {
        let mp_id = local_mappoints[i];
        let pose = optimizer.recover_optimized_mappoint_pose(mp_id + ini_mp_id + 1);
        map.write().mappoints.get_mut(&mp_id).unwrap().position = pose.translation.into();
        let norm_and_depth = map.read().mappoints.get(&mp_id).unwrap().get_norm_and_depth(&map.read());
        if norm_and_depth.is_some() {
            map.write().mappoints.get_mut(&mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
        }
    }

    map.write().map_change_index += 1;
}