use core::{config::{SETTINGS, SYSTEM}, sensor::{FrameSensor, Sensor}};
use std::collections::{HashMap, HashSet};

use log::{debug, warn};
use nalgebra::Matrix3;
use opencv::core::KeyPointTraitConst;

use crate::{map::{map::Id, pose::{DVTranslation, Pose}, read_only_lock::ReadWriteMap}, modules::optimizer::{INV_LEVEL_SIGMA2, TH_HUBER_2D}, registered_actors::CAMERA};

use super::{imu::{ImuBias, ImuPreIntegrated}, module_definitions::FullMapOptimizationModule, optimizer::{self, TH_HUBER_MONO}};

pub struct GlobalBundleAdjustment { }
impl FullMapOptimizationModule for GlobalBundleAdjustment {
    fn optimize(&self, map: &mut ReadWriteMap, iterations: i32, robust: bool, loop_kf: Id) -> Result<(), Box<dyn std::error::Error>> { // stop_flag: Option<SharedPtr<bool>>
        let _span = tracy_client::span!("global_bundle_adjustment");
        // void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
        let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");
        let fx= SETTINGS.get::<f64>(CAMERA, "fx");
        let fy= SETTINGS.get::<f64>(CAMERA, "fy");
        let cx= SETTINGS.get::<f64>(CAMERA, "cx");
        let cy= SETTINGS.get::<f64>(CAMERA, "cy");
        let camera_param = [fx, fy, cx,cy];

        let mut optimizer = g2o::ffi::new_sparse_optimizer(1, camera_param, 0.0);
        // if enable_stop_flag {
        //     optimizer.enable_stop_flag();
        // }

        let (kf_vertex_ids, mp_vertex_ids) = {
            let lock = map.read()?;

            // Set KeyFrame vertices
            let mut kf_vertex_ids = HashMap::new();
            let mut id_count = 0;
            for (kf_id, kf) in lock.get_keyframes_iter() {
                optimizer.pin_mut().add_vertex_se3expmap(
                    id_count,
                    (kf.get_pose()).into(),
                    *kf_id == lock.initial_kf_id
                );
                kf_vertex_ids.insert(*kf_id, id_count);
                id_count += 1;
                // println!("ADD KF {} with pose {:?}", kf.id, kf.pose);
            }

            let mut mp_vertex_ids = HashMap::new();

            // Set MapPoint vertices
            let mut _edges = Vec::new();
            for (mp_id, mappoint) in &lock.mappoints {
                optimizer.pin_mut().add_vertex_sbapointxyz(
                    id_count,
                    Pose::new(*mappoint.position, Matrix3::identity()).into(), // create pose out of translation only
                    false, true
                );
                mp_vertex_ids.insert(*mp_id, id_count);
                // println!("ADD MAPPOINT {} with pose {:?}", mp_id, mappoint.position);

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

                                let (keypoint, _) = lock.get_keyframe(*kf_id).features.get_keypoint(*left_index as usize);
                                _edges.push(
                                    optimizer.pin_mut().add_edge_se3_project_xyz_monocular_binary(
                                        robust, id_count, *kf_vertex_ids.get(kf_id).unwrap(),
                                        *mp_id,
                                        keypoint.pt().x, keypoint.pt().y,
                                        INV_LEVEL_SIGMA2[keypoint.octave() as usize],
                                        * TH_HUBER_2D
                                    )
                                );
                                // println!("ADD EDGE KF {} <-> MP {}", kf_id, mp_id);
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

            (kf_vertex_ids, mp_vertex_ids)
        };

        // Optimize!
        {
            let _ = tracy_client::span!("global_bundle_adjustment::optimize");
            optimizer.pin_mut().optimize(iterations, false, false);
        }

        {
            let mut lock = map.write()?;
            let initial_kf_id = lock.initial_kf_id;
            for (kf_id, vertex_id) in kf_vertex_ids {
                let kf = lock.get_keyframe_mut(kf_id);
                let pose: Pose = optimizer.recover_optimized_frame_pose(vertex_id).into();
                // println!("GBA: loop kf {}, initial kf {}", loop_kf, initial_kf_id);

                if loop_kf == initial_kf_id {
                    kf.set_pose(pose.into());
                } else {
                    // println!("GBA: Set kf {} pose: {:?}. Old pose: {:?}", kf.id, pose, kf.pose);
                    kf.gba_pose = Some(pose);
                    kf.ba_global_for_kf = loop_kf;
                    // debug!("(baglobal) set in gba, for kf {}: {}", kf.id, loop_kf);
                }
                // // Possible that map actor deleted mappoint after local BA has finished but before
                // // this message is processed
                // println!("GBA: KF {} is deleted?", kf_id);
                // continue;
            }

            for (mp_id, vertex_id) in mp_vertex_ids {
                match lock.mappoints.get_mut(&mp_id) {
                    // Possible that map actor deleted mappoint after local BA has finished but before
                    // this message is processed
                    Some(mp) => {
                        let position = optimizer.recover_optimized_mappoint_pose(vertex_id);
                        let translation = nalgebra::Translation3::new(
                            position.translation[0] as f64,
                            position.translation[1] as f64,
                            position.translation[2] as f64
                        );
                        let pos = DVTranslation::new(translation.vector);

                        if loop_kf == initial_kf_id {
                            mp.position = pos;
                            let norm_and_depth = lock.mappoints.get(&mp_id).unwrap().get_norm_and_depth(&lock);
                            if norm_and_depth.is_some() {
                                lock.mappoints.get_mut(&mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
                            }
                        } else {
                            mp.gba_pose = Some(pos);
                            mp.ba_global_for_kf = loop_kf;
                            // debug!("(baglobal) set in gba, for mp {}: {}", mp.id, loop_kf);
                        }
                    },
                    None => continue,
                };
            }
        }
        Ok(())
    }
}

pub fn full_inertial_ba(
    map: &ReadWriteMap, iterations: i32, fix_local: bool, loop_id: Id, 
    init: bool, prior_g: f64, prior_a: f64, 
    ) -> Result<(), Box<dyn std::error::Error>> {
    // void static FullInertialBA(Map *pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);
    let max_kf_id = * map.read()?.get_keyframes_iter()
        .max_by(|a, b| a.1.id.cmp(&b.1.id))
        .map(|(k, _v)| k).unwrap();

    println!("BEGIN FULL INERTIAL BA... max kf id: {}, fix local: {}, loop id: {}", max_kf_id, fix_local, loop_id);

    // Setup optimizer
    // ... Note... pretty sure camera params aren't necessary for this optimization but throwing them in here anyway just in case
    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(5, camera_param, 1e-5);

    let non_fixed = 0;

    // Set KeyFrame vertices
    let mut inc_kf = 0;
    let mut kf_vertex_fixed = HashMap::new();
    for (kf_id, kf) in map.read()?.get_keyframes_iter() {
        inc_kf = * kf_id;

        let fixed = if fix_local {
            // todo this isn't local? where does this value come from
            // bFixed = (pKFi->mnBALocalForKF>=(maxKFid-1)) || (pKFi->mnBAFixedForKF>=(maxKFid-1));
            // if(!bFixed)
            //     nNonFixed++;
            // VP->setFixed(bFixed);
            true
        } else {
            false
        };

        optimizer::add_vertex_pose_keyframe(&mut optimizer, kf, fixed, kf.id);
        kf_vertex_fixed.insert(*kf_id, fixed);
        println!("FIBA, add keyframe {}", kf.id);

        // println!("add vertex pose {}", kf.id);

        if kf.imu_data.is_imu_initialized {
            optimizer.pin_mut().add_vertex_velocity(
                max_kf_id + 3 * kf.id + 1,
                fixed,
                kf.imu_data.velocity.unwrap().into()
            );
            // println!("add vertex velocity {}", max_kf_id + 3 * kf.id + 1);

            if !init {
                optimizer.pin_mut().add_vertex_gyrobias(
                    max_kf_id + 3 * kf.id + 2,
                    fixed,
                    kf.imu_data.imu_bias.get_gyro_bias().into()
                );
                optimizer.pin_mut().add_vertex_accbias(
                    max_kf_id + 3 * kf.id + 3,
                    fixed,
                    kf.imu_data.imu_bias.get_acc_bias().into()
                );
            }
        } else {
            println!("KF {} imu is not initialized", kf.id);
        }
    }

    if init {
        let lock = map.read()?;
        let kf = lock.get_keyframe(inc_kf);
        optimizer.pin_mut().add_vertex_gyrobias(
            4 * max_kf_id + 2,
            false,
            kf.imu_data.imu_bias.get_gyro_bias().into()
        );
        optimizer.pin_mut().add_vertex_accbias(
            4 * max_kf_id + 3,
            false,
            kf.imu_data.imu_bias.get_acc_bias().into()
        );
        // println!("add vertex gyro and acc {} {}", 4 * max_kf_id + 2, 4 * max_kf_id + 3);
    }

    if fix_local {
        if non_fixed < 3 {
            return Ok(());
        }
    }

    // IMU links
    let mut new_imu_preintegrated_for_kfs: HashMap<Id, ImuPreIntegrated> = HashMap::new();

    for (kf_id, kf) in map.read()?.get_keyframes_iter() {
        if *kf_id > max_kf_id ||
            kf.prev_kf_id.is_none() ||
            kf.prev_kf_id.unwrap() > max_kf_id ||
            kf.imu_data.imu_preintegrated.is_none() ||
            ! map.read()?.get_keyframe(kf.prev_kf_id.unwrap()).imu_data.is_imu_initialized
        {
            continue;
        }

        let mut imu_preintegrated = kf.imu_data.imu_preintegrated.as_ref().unwrap().clone();
        let prev_kf_id = kf.prev_kf_id.unwrap();

        imu_preintegrated.set_new_bias(map.read()?.get_keyframe(prev_kf_id).imu_data.imu_bias);

        let vp1_id = prev_kf_id;
        let vv1_id = max_kf_id + 3 * prev_kf_id + 1;

        let (vg1_id, va1_id, vg2_id, va2_id) = if !init {
            let vg1_id = max_kf_id + 3 * prev_kf_id + 2;
            let va1_id = max_kf_id + 3 * prev_kf_id + 3;
            let vg2_id = max_kf_id + 3 * kf.id + 2;
            let va2_id = max_kf_id + 3 * kf.id + 3;
            (vg1_id, va1_id, vg2_id, va2_id)
        } else {
            let vg1_id = 4 * max_kf_id + 2;
            let va1_id = 4 * max_kf_id + 3;
            (vg1_id, va1_id, -1, -1)
        };

        let vp2_id = kf.id;
        let vv2_id = max_kf_id + 3 * kf.id + 1;

        // println!("(all of them) {} {} {} {} {} {} ", vp1_id, vv1_id, vg1_id, va1_id, vp2_id, vv2_id);
        optimizer.pin_mut().add_edge_inertial(
            vp1_id,
            vv1_id,
            vg1_id,
            va1_id,
            vp2_id,
            vv2_id,
            (& imu_preintegrated).into(),
            true,
            16.92
        );

        if !init {
            optimizer.pin_mut().add_edge_gyro_and_acc(
                vg1_id, vg2_id, va1_id, va2_id,
                (& imu_preintegrated).into()
            );
        }
        new_imu_preintegrated_for_kfs.insert(*kf_id, imu_preintegrated);

    }
    for (id, preintegrated) in new_imu_preintegrated_for_kfs {
        // debug!("SET KF {} IMU PREINTEGRATED", id);
        map.write()?.get_keyframe_mut(id).imu_data.imu_preintegrated = Some(preintegrated);
    }

    if init {
        let vertex_gyro_bias_id = 4 * max_kf_id + 2; // VG
        let vertex_acc_bias_id = 4 * max_kf_id + 3; // VA

        // Add prior to comon biases
        optimizer.pin_mut().add_edge_prior_for_imu(
            vertex_acc_bias_id,
            vertex_gyro_bias_id,
            [0.0, 0.0, 0.0],
            prior_a,
            prior_g
        );
    }


    let ini_mp_id = max_kf_id * 5;
    let mut mps_not_included: HashSet<Id> = HashSet::new(); // vbNotIncludedMP

    let mut _edges = Vec::new();
    {
        let lock = map.read()?;
        for (mp_id, mp) in & lock.mappoints {
            let id = mp_id + ini_mp_id + 1;
            optimizer.pin_mut().add_vertex_sbapointxyz(
                id,
                Pose::new(* mp.position, Matrix3::identity()).into(),
                false, // todo should this be false? not specified 
                true
            );

            let mut all_fixed = true;

            //Set edges
            for (kf_id, (left_index, right_index)) in mp.get_observations() {
                if *kf_id > max_kf_id {
                    continue;
                }

                if *left_index != -1 && *right_index == -1 {
                    let (keypoint, _) = lock.get_keyframe(*kf_id).features.get_keypoint(*left_index as usize);
                    _edges.push(
                        optimizer.pin_mut().add_edge_mono_binary(
                            true, id, * kf_id,
                            *mp_id,
                            keypoint.pt().x, keypoint.pt().y,
                            INV_LEVEL_SIGMA2[keypoint.octave() as usize],
                            * TH_HUBER_MONO
                        )
                    );

                    if all_fixed && ! kf_vertex_fixed.get(kf_id).unwrap() {
                        all_fixed = false;
                    }
                }

                if *right_index != -1 {
                    todo!("Stereo");
                    // kpUn = pKFi->mvKeysUn[leftIndex];
                    // const float kp_ur = pKFi->mvuRight[leftIndex];
                    // Eigen::Matrix<double,3,1> obs;
                    // obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    // EdgeStereo* e = new EdgeStereo(0);

                    // g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    // if(bAllFixed)
                    //     if(!VP->fixed())
                    //         bAllFixed=false;

                    // e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    // e->setVertex(1, VP);
                    // e->setMeasurement(obs);
                    // const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];

                    // e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

                    // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    // e->setRobustKernel(rk);
                    // rk->setDelta(thHuberStereo);

                    // optimizer.addEdge(e);
                }
                {
                    // TODO multiple cameras
                    // if(pKFi->mpCamera2){ // Monocular right observation
                    //     int rightIndex = get<1>(mit->second);

                    //     if(rightIndex != -1 && rightIndex < pKFi->mvKeysRight.size()){
                    //         rightIndex -= pKFi->NLeft;

                    //         Eigen::Matrix<double,2,1> obs;
                    //         kpUn = pKFi->mvKeysRight[rightIndex];
                    //         obs << kpUn.pt.x, kpUn.pt.y;

                    //         EdgeMono *e = new EdgeMono(1);

                    //         g2o::OptimizableGraph::Vertex* VP = dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId));
                    //         if(bAllFixed)
                    //             if(!VP->fixed())
                    //                 bAllFixed=false;

                    //         e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    //         e->setVertex(1, VP);
                    //         e->setMeasurement(obs);
                    //         const float invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    //         e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    //         g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    //         e->setRobustKernel(rk);
                    //         rk->setDelta(thHuberMono);

                    //         optimizer.addEdge(e);
                    //     }
                }
            }

            if all_fixed {
                optimizer.pin_mut().remove_vertex(id);
                mps_not_included.insert(* mp_id);
            }
        }
    }


    optimizer.pin_mut().optimize(iterations, false, false);

    {
        let mut lock = map.write()?;
        // Recover optimized data
        // Keyframes
        for (kf_id, kf) in lock.get_keyframes_iter_mut() {
            if * kf_id > max_kf_id {
                continue;
            }
            let pose: Pose = optimizer.recover_optimized_frame_pose(*kf_id).into();

            if loop_id == 0 {
                kf.set_pose(pose.into());
                debug!("FIBA result, KF {} pose: {:?} ", kf.id, pose);
            } else {
                // println!("GBA: Set kf {} pose: {:?}. Old pose: {:?}", kf.id, pose, kf.pose);
                kf.gba_pose = Some(pose);
                kf.ba_global_for_kf = loop_id;
                // debug!("(baglobal) set in fiba, for kf {}: {}", kf.id, loop_id);
                debug!("FIBA result, KF {} gba pose: {:?} ", kf.id, pose);
            }

            if kf.imu_data.is_imu_initialized {
                let vertex_velocity = optimizer.recover_optimized_vertex_velocity(max_kf_id + 3 * kf.id + 1);

                if loop_id == 0 {
                    debug!("FIBA result, KF {} velocity: {:?} ", kf.id, vertex_velocity);
                    kf.imu_data.velocity = Some(vertex_velocity.into());
                } else {
                    kf.vwb_gba = Some(vertex_velocity.into());
                    debug!("FIBA result, KF {} gba velocity: {:?} ", kf.id, vertex_velocity);
                }

                let (vg_id, va_id) = if !init {
                    (
                        max_kf_id + 3 * kf.id + 2,
                        max_kf_id + 3 * kf.id + 3
                    )
                } else {
                    (
                        4 * max_kf_id + 2,
                        4 * max_kf_id + 3
                    )
                };
                let estimate = optimizer.recover_optimized_inertial(
                    vg_id, va_id, -1, -1
                );

                let vb = estimate.vb;
                let b = ImuBias {
                    bax: vb[3],
                    bay: vb[4],
                    baz: vb[5],
                    bwx: vb[0],
                    bwy: vb[1],
                    bwz: vb[2],
                };

                if loop_id == 0 {
                    kf.imu_data.set_new_bias(b);
                } else {
                    kf.bias_gba = Some(b);
                }
            }
        }
    }

    {
        // This is a really gross way to handle the locks but not sure if there is an easier option
        //Points
        let mp_keys = map.read()?.mappoints.keys().cloned().collect::<Vec<Id>>();
        for mp_id in mp_keys {
            if mps_not_included.contains(&mp_id) {
                continue;
            }
            let position = optimizer.recover_optimized_mappoint_pose(mp_id + ini_mp_id + 1);
            let translation = nalgebra::Translation3::new(
                position.translation[0] as f64,
                position.translation[1] as f64,
                position.translation[2] as f64
            );
            let pos = DVTranslation::new(translation.vector);

            if loop_id == 0 {
                map.write()?.mappoints.get_mut(&mp_id).unwrap().position = pos;
                let norm_and_depth = {
                    let lock = map.read()?;
                    lock.mappoints.get(&mp_id).unwrap().get_norm_and_depth(&lock)
                };
                if norm_and_depth.is_some() {
                    map.write()?.mappoints.get_mut(&mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
                }
            } else {
                let mut lock = map.write()?;
                let mp = lock.mappoints.get_mut(&mp_id).unwrap();
                mp.gba_pose = Some(pos);
                mp.ba_global_for_kf = loop_id;
                println!("FIBA result, mp posgba... {} ... {}: {:?}", mp_id, mp_id + ini_mp_id + 1, pos);
                // debug!("(baglobal) set in fiba, for mp {}: {}", mp.id, loop_id);
            }
        }
    }

    map.write()?.map_change_index += 1;

    return Ok(());
}
