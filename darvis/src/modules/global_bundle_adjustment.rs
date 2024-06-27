use core::{config::{SETTINGS, SYSTEM}, sensor::{FrameSensor, Sensor}};
use std::collections::HashMap;

use log::warn;
use nalgebra::Matrix3;
use opencv::core::KeyPointTraitConst;

use crate::{map::{map::Id, pose::{DVTranslation, Pose}}, modules::optimizer::INV_LEVEL_SIGMA2, registered_actors::CAMERA, MapLock};

use super::module_definitions::FullMapOptimizationModule;

pub struct GlobalBundleAdjustment { }
impl FullMapOptimizationModule for GlobalBundleAdjustment {
    fn optimize(&self, map: &mut MapLock, iterations: i32, robust: bool, loop_kf: Id) { // stop_flag: Option<SharedPtr<bool>>
        let _span = tracy_client::span!("global_bundle_adjustment");
        // void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
        let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");
        let fx= SETTINGS.get::<f64>(CAMERA, "fx");
        let fy= SETTINGS.get::<f64>(CAMERA, "fy");
        let cx= SETTINGS.get::<f64>(CAMERA, "cx");
        let cy= SETTINGS.get::<f64>(CAMERA, "cy");
        let camera_param = [fx, fy, cx,cy];

        let mut optimizer = g2o::ffi::new_sparse_optimizer(1, camera_param);
        // if enable_stop_flag {
        //     optimizer.enable_stop_flag();
        // }

        println!("Loop kf is {}", loop_kf);

        let (kf_vertex_ids, mp_vertex_ids) = {
            let lock = map.read();

            // Set KeyFrame vertices
            let mut kf_vertex_ids = HashMap::new();
            let mut id_count = 0;
            for (kf_id, kf) in &lock.keyframes {
                optimizer.pin_mut().add_frame_vertex(
                    id_count,
                    (kf.pose).into(),
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
                optimizer.pin_mut().add_mappoint_vertex(
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

                                let TH_HUBER_2D: f32 = (5.99 as f32).sqrt();

                                let (keypoint, _) = lock.keyframes.get(kf_id).unwrap().features.get_keypoint(*left_index as usize);
                                _edges.push(
                                    optimizer.pin_mut().add_edge_monocular_binary(
                                        robust, id_count, *kf_vertex_ids.get(kf_id).unwrap(),
                                        *mp_id,
                                        keypoint.pt().x, keypoint.pt().y,
                                        INV_LEVEL_SIGMA2[keypoint.octave() as usize],
                                        TH_HUBER_2D
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
            let mut lock = map.write();
            let initial_kf_id = lock.initial_kf_id;
            for (kf_id, vertex_id) in kf_vertex_ids {
                if let Some(kf) = lock.keyframes.get_mut(&kf_id) {
                    let pose: Pose = optimizer.recover_optimized_frame_pose(vertex_id).into();

                    if loop_kf == initial_kf_id {
                        kf.pose = pose.into();
                    } else {
                        // println!("GBA: Set kf {} pose: {:?}. Old pose: {:?}", kf.id, pose, kf.pose);
                        kf.gba_pose = Some(pose);
                        kf.ba_global_for_kf = loop_kf;
                    }
                } else {
                    // Possible that map actor deleted mappoint after local BA has finished but before
                    // this message is processed
                    println!("GBA: KF {} is deleted?", kf_id);
                    continue;
                }
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
                        }
                    },
                    None => continue,
                };
            }
        }
    }
}