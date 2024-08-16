extern crate g2o;

use std::{cmp::{max, min}, collections::{BTreeMap, HashMap, HashSet}, fs::File, io::{self, BufRead}, path::Path};
use core::{
    config::{SETTINGS, SYSTEM}, sensor::{Sensor, FrameSensor}
};
use log::{debug, error, info, warn};
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};
use opencv::prelude::KeyPointTraitConst;
use crate::{
    actors::loop_closing::{KeyFrameAndPose, GBA_KILL_SWITCH}, map::{frame::Frame, map::Id, pose::{DVTranslation, Pose, Sim3}}, registered_actors::{CAMERA, FEATURE_DETECTION}, MapLock
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
        for i in 0..frame.mappoint_matches.len() {
            if let Some((mp_id, is_outlier)) = frame.mappoint_matches.get(i) {
                let position = match map_read_lock.mappoints.get(&mp_id) {
                    Some(mp) => mp.position,
                    None => {
                        frame.mappoint_matches.delete_at_indices((i as i32, -1)); // TODO (STEREO)
                        continue;
                    },
                };

                let (keypoint, _) = &frame.features.get_keypoint(i as usize);

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
                        mp_indexes.push(i as u32);
                    }
                };

                initial_correspondences += 1;
            }
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

        optimizer.pin_mut().optimize(iterations[iteration], false, false);

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
                edge.inner.pin_mut().set_robust_kernel(true);
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

    debug!("Set outliers in pose optimization: {}. Optimized pose: {:?}", num_bad, frame.pose.as_ref().unwrap());

    // Return number of inliers
    return Some(initial_correspondences - num_bad);
}


pub fn _optimize_essential_graph_6dof() {
    todo!("STEREO, RGBD. Used by loop closing");
    // void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
    //                                 const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
    //                                 const LoopClosing::KeyFrameAndPose &CorrectedSim3,
    //                                 const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
}

pub fn _optimize_essential_graph_4dof() {
    todo!("IMU. Used by Loop closing");
    // void static OptimizeEssentialGraph4DoF(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
    //                                 const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
    //                                 const LoopClosing::KeyFrameAndPose &CorrectedSim3,
    //                                 const map<KeyFrame *, set<KeyFrame *> > &LoopConnections);
}

pub fn optimize_essential_graph(
    map: &MapLock, loop_kf: Id, curr_kf: Id,
    loop_connections: BTreeMap<Id, HashSet<Id>>,
    non_corrected_sim3: &KeyFrameAndPose, corrected_sim3: &KeyFrameAndPose,
    corrected_mp_references: HashMap::<Id, Id>,
    fix_scale: bool
) {
    // void Optimizer::OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
    //                                        const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
    //                                        const LoopClosing::KeyFrameAndPose &CorrectedSim3,
    //                                        const map<KeyFrame *, set<KeyFrame *> > &LoopConnections, const bool &bFixScale)
    let _span = tracy_client::span!("optimize_essential");

    let camera_param = [
        SETTINGS.get::<f64>(CAMERA, "fx"),
        SETTINGS.get::<f64>(CAMERA, "fy"),
        SETTINGS.get::<f64>(CAMERA, "cx"),
        SETTINGS.get::<f64>(CAMERA, "cy")
    ];
    let mut optimizer = g2o::ffi::new_sparse_optimizer(4, camera_param);

    let mut v_scw: HashMap<Id, Sim3> = HashMap::new(); // vScw

    let mut inserted_edges = HashSet::new(); // sInsertedEdges
    {
        let mut edges_per_vertex = HashMap::new(); // for testing

        // Set KeyFrame vertices
        let lock = map.read();
        for (kf_id, kf) in &lock.keyframes {
            let estimate = match corrected_sim3.get(kf_id) {
                Some(sim3) => *sim3,
                None => kf.pose.into()
            };
            v_scw.insert(*kf_id, estimate);
            edges_per_vertex.insert(*kf_id, [0, 0, 0, 0]);

            optimizer.pin_mut().add_vertex_sim3_expmap(
                *kf_id,
                estimate.into(),
                fix_scale,
                *kf_id == lock.initial_kf_id,
                false,
            );
        }

        // Set loop edges
        let min_feat = 100;
        for (kf_i_id, connected_kfs) in &loop_connections {
            // let connected_kfs = loop_connections.get(&kf_i_id).unwrap();
            let kf_i = lock.keyframes.get(&kf_i_id).unwrap();
            let siw = v_scw.get(&kf_i_id).unwrap();
            let swi = siw.inverse();

            for kf_j in connected_kfs {
                if (kf_i.id != curr_kf || *kf_j != loop_kf) && kf_i.get_connected_kf_weight(*kf_j) < min_feat {
                    continue;
                }

                let sjw = v_scw.get(&kf_j).unwrap();
                let sji = *sjw * swi;
                optimizer.pin_mut().add_one_sim3_edge(
                    *kf_i_id,
                    *kf_j,
                    sji.into(),
                );
                inserted_edges.insert((min(kf_i_id, kf_j), max(kf_i_id, kf_j)));
                edges_per_vertex.entry(*kf_i_id).and_modify(|e| e[0] += 1);
            }
        }

        // Set normal edges
        for (kf_i_id, kf_i) in lock.keyframes.iter() {
            let swi = match non_corrected_sim3.get(&kf_i_id) {
                Some(sim3) => sim3.inverse(),
                None => v_scw.get(&kf_i_id).unwrap().inverse()
            };

            // Spanning tree edge
            match kf_i.parent {
                Some(kf_j_id) => {
                    let sjw = match non_corrected_sim3.get(&kf_j_id) {
                        Some(sim3) => sim3.clone(),
                        None => * v_scw.get(&kf_j_id).unwrap()
                    };
                    let sji = sjw * swi;
                    optimizer.pin_mut().add_one_sim3_edge(
                        *kf_i_id,
                        kf_j_id,
                        sji.into(),
                    );
                    edges_per_vertex.entry(*kf_i_id).and_modify(|e| e[1] += 1);
                },
                None => { 
                    if kf_i.id != lock.initial_kf_id {
                        error!("No parent kf for kf {}", kf_i_id);
                    }
                }
            };

            // Loop edges
            let loop_edges = kf_i.get_loop_edges();
            // print!("Loop edges #2:");
            for edge_kf_id in loop_edges {
                if *edge_kf_id < *kf_i_id {
                    let slw = match non_corrected_sim3.get(&edge_kf_id) {
                        Some(sim3) => sim3.clone(),
                        None => * v_scw.get(&edge_kf_id).unwrap()
                    };
                    let sli = slw * swi;
                    optimizer.pin_mut().add_one_sim3_edge(
                        *kf_i_id,
                        *edge_kf_id,
                        sli.into(),
                    );
                    edges_per_vertex.entry(*kf_i_id).and_modify(|e| e[2] += 1);
                }
            }

            // Covisibility graph edges
            let parent_kf = kf_i.parent;
            // print!("Covisible edges:");
            for kf_n in kf_i.get_covisibles_by_weight(100) {
                if parent_kf != Some(kf_n) && !kf_i.children.contains(&kf_n) {
                    if kf_n < *kf_i_id {
                        if inserted_edges.contains(&(min(kf_i_id, &kf_n), max(kf_i_id, &kf_n))) {
                            continue;
                        }

                        let snw = match non_corrected_sim3.get(&kf_n) {
                            Some(sim3) => sim3.clone(),
                            None => * v_scw.get(&kf_n).unwrap()
                        };
                        let sni = snw * swi;

                        optimizer.pin_mut().add_one_sim3_edge(
                            *kf_i_id,
                            kf_n,
                            sni.into(),
                        );
                        edges_per_vertex.entry(*kf_i_id).and_modify(|e| e[3] += 1);
                    }
                }
            }
        }

    }

    // Optimize!
    optimizer.pin_mut().optimize(20, false, true);

    // For debugging ... prints the .g2o file
    // optimizer.save("after_optimize_ess_darvis.g2o\0",curr_kf as i32);

    let mut corrected_swc = HashMap::new();
    {
        // SE3 Pose Recovering. Sim3:[sR t;0 1] -> SE3:[R t/s;0 1]
        let mut lock = map.write();
        for kf in lock.keyframes.values_mut() {
            let corrected_siw: Sim3 = optimizer.recover_optimized_sim3(kf.id).into();
            corrected_swc.insert(kf.id, corrected_siw.inverse());

            let tiw: Pose = corrected_siw.into(); //[R t/s;0 1]

            kf.pose = tiw;
        }
    }

    {
        // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
        let mp_ids = map.read().mappoints.keys().cloned().collect::<Vec<Id>>();

        let mps_corrected_by: HashMap<Id, i32> = HashMap::new(); // mnCorrectedByKF

        for mp_id in mp_ids {
            let idr = if mps_corrected_by.contains_key(&mp_id) && *mps_corrected_by.get(&mp_id).unwrap() == curr_kf {
                *corrected_mp_references.get(&mp_id).unwrap()
            } else {
                map.read().mappoints.get(&mp_id).unwrap().ref_kf_id
            };

            let srw = v_scw.get(&idr).unwrap();
            let corrected_swr = corrected_swc[&idr];

            {
                let mut lock = map.write();
                let mp = lock.mappoints.get_mut(&mp_id).unwrap();
                let p_3d_w = mp.position;
                mp.position = corrected_swr.map(&srw.map(&p_3d_w));
            }

            let norm_and_depth = map.read().mappoints.get(&mp_id).unwrap().get_norm_and_depth(&map.read());
            if norm_and_depth.is_some() {
                map.write().mappoints.get_mut(&mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
            }

        }

    }
}

pub fn optimize_sim3(
    map: &MapLock, kf1_id: Id, kf2_id: Id, matched_mps: &mut Vec<Option<Id>>,
    sim3: &mut Sim3, th2: i32, fix_scale: bool
) -> i32 {
    // From ORBSLAM2:
    // int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, 
    //                             g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
    // but bAllPoints is always set to true
    // returns vpMatches1, mAcumHessian

    let camera_param = [
        SETTINGS.get::<f64>(CAMERA, "fx"),
        SETTINGS.get::<f64>(CAMERA, "fy"),
        SETTINGS.get::<f64>(CAMERA, "cx"),
        SETTINGS.get::<f64>(CAMERA, "cy")
    ];
    let mut optimizer = g2o::ffi::new_sparse_optimizer(3, camera_param);

    // Camera poses
    let (kf1_rot, kf1_trans, kf2_rot, kf2_trans) = { // R1w, t1w, R2w, t2w
        let lock = map.read();
        let kf1 = lock.keyframes.get(&kf1_id).unwrap();
        let kf2 = lock.keyframes.get(&kf2_id).unwrap();
        (
            kf1.pose.get_rotation(),
            kf1.pose.get_translation(),
            kf2.pose.get_rotation(),
            kf2.pose.get_translation()
        )
    };

    // Set Sim3 vertex
    optimizer.pin_mut().add_vertex_sim3_expmap(
        0,
        (*sim3).into(),
        fix_scale,
        false,
        true
    );

    let mut num_correspondences = 0;
    let mut edge_indexes = vec![];
    {
        // Set MapPoint vertices
        let lock = map.read();
        let mappoints1 = { // vpMapPoints1
            let kf1 = lock.keyframes.get(&kf1_id).unwrap();
            kf1.get_mp_matches()
        };

        for i in 0..matched_mps.len() {
            if matched_mps[i].is_none() {
                continue;
            }
            let mp2_id = matched_mps[i].unwrap();

            let mp1 = match mappoints1[i] {
                Some((id, _)) => {
                    match lock.mappoints.get(&id) {
                        Some(mp) => mp,
                        None => continue
                    }
                },
                None => continue
            };
            let mp2 = match lock.mappoints.get(&mp2_id) {
                Some(mp) => mp,
                None => continue
            };

            let id1 = (2 * i + 1) as i32;
            let id2 = (2 * (i + 1)) as i32;
            let (i2_left, _i2_right) = mp2.get_index_in_keyframe(kf2_id);

            if i2_left >= 0 {
                let p_3d_1c = *kf1_rot * *mp1.position + *kf1_trans;
                let translation = DVTranslation::new(p_3d_1c);
                optimizer.pin_mut().add_mappoint_vertex(
                    id1,
                    Pose::new(*translation, Matrix3::identity()).into(), // create pose out of translation only
                    true, false
                );

                let p_3d_2c = *kf2_rot * *mp2.position + *kf2_trans;
                let translation2 = DVTranslation::new(p_3d_2c);
                optimizer.pin_mut().add_mappoint_vertex(
                    id2,
                    Pose::new(*translation2, Matrix3::identity()).into(), // create pose out of translation only
                    true, false
                );
            } else {
                continue;
            }

            num_correspondences += 1;

            // Set edge x1 = S12*X2
            let huber_delta = (th2 as f32).sqrt();
            let kf1 = lock.keyframes.get(&kf1_id).unwrap();
            let (kp1, _) = kf1.features.get_keypoint(i);

            // Set edge x2 = S21*X1
            let kf2 = lock.keyframes.get(&kf2_id).unwrap();
            let (kp2, _) = kf2.features.get_keypoint(i2_left as usize);

            optimizer.pin_mut().add_both_sim_edges(
                id2, kp1.pt().x, kp1.pt().y, INV_LEVEL_SIGMA2[kp1.octave() as usize],
                id1, kp2.pt().x, kp2.pt().y, INV_LEVEL_SIGMA2[kp2.octave() as usize],
                huber_delta
            );

            edge_indexes.push(i);
        }
    }

    // Optimize!

    // optimizer.save("before_optimize_darvis_sim3.g2o\0",0);
    optimizer.pin_mut().optimize(5, false, false);
    // optimizer.save("after_optimize_darvis_sim3.g2o\0",1);

    // Check inliers
    let removed_edge_indexes = optimizer.pin_mut().remove_sim3_edges_with_chi2(th2 as f32);
    let num_bad = removed_edge_indexes.len();
    for i in removed_edge_indexes {
        let index = edge_indexes[i as usize];
        matched_mps[index] = None;
    }

    let more_iterations = match num_bad > 0 {
        true => 10,
        false => 5
    };
    if num_correspondences - num_bad < 10 {
        return 0;
    }

    // Optimize again only with inliers
    optimizer.pin_mut().optimize(more_iterations, false, false);

    let mut n_in = 0;
    let mut i = 0;
    for edge in optimizer.pin_mut().get_mut_sim3_edges() {
        if edge.edge1.is_null() || edge.edge2.is_null() {
            continue;
        }

        if edge.edge1.chi2() > th2 as f64 && edge.edge2.chi2() > th2 as f64 {
            let index = edge_indexes[i as usize];
            matched_mps[index] = None;
        } else {
            n_in += 1;
        }
        i += 1;
    }

    // Recover optimized Sim3
    let optimized_sim3: Sim3 = optimizer.recover_optimized_sim3(0).into();
    *sim3 = optimized_sim3;
    return n_in;
}
