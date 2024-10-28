extern crate g2o;

use std::{cmp::{max, min}, collections::{BTreeMap, HashMap, HashSet}};
use core::{
    config::{SETTINGS, SYSTEM}, matrix::{DVMatrix3, DVMatrixDynamic, DVVector3}, sensor::{FrameSensor, Sensor}
};
use cxx::UniquePtr;
use dvos3binding::ffi::SVDComputeType;
use g2o::ffi::BridgeSparseOptimizer;
use log::{debug, error};
use nalgebra::Matrix3;
use opencv::prelude::KeyPointTraitConst;
use crate::{
    actors::{loop_closing::KeyFrameAndPose, tracking_backend::TrackedMapPointData}, map::{frame::Frame, keyframe::KeyFrame, map::Id, pose::{DVTranslation, Pose, Sim3}, read_only_lock::ReadWriteMap}, registered_actors::{CAMERA, CAMERA_MODULE, FEATURE_DETECTION}
};
use std::ops::AddAssign;

use super::imu::{ConstraintPoseImu, ImuBias, ImuCalib, ImuPreIntegrated};

lazy_static! {
    pub static ref TH_HUBER_MONO: f32 = (5.991 as f32).sqrt();
    pub static ref TH_HUBER_2D: f32 = (5.99 as f32).sqrt();
    pub static ref TH_HUBER_3D: f32 = (7.815 as f32).sqrt();

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

pub fn pose_inertial_optimization_last_frame(
    frame: &mut Frame, previous_frame: &mut Frame, tracked_mappoint_data: & HashMap<Id, TrackedMapPointData>, map: &ReadWriteMap, sensor: &Sensor
) -> Result<i32, Box<dyn std::error::Error> > {
    // int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
    // but bRecInit is always set to false

    debug!("POSE INERTIAL OPTIMIZATION LAST FRAME!");
    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(7, camera_param, 0.0);

    let mut initial_mono_correspondences = 0; //nInitialMonoCorrespondences
    let initial_stereo_correspondences = 0; //nInitialStereoCorrespondences

    // Set Current Frame vertex
    let vp = 0;
    let vv = 1;
    let vg = 2;
    let va = 3;
    add_vertex_pose_frame(&mut optimizer, frame, false, vp);
    optimizer.pin_mut().add_vertex_velocity(
        vv, false, frame.imu_data.get_velocity().into()
    );
    optimizer.pin_mut().add_vertex_gyrobias(
        vg, false, frame.imu_data.imu_bias.get_gyro_bias().into()
    );
    optimizer.pin_mut().add_vertex_accbias(
        va, false, frame.imu_data.imu_bias.get_acc_bias().into()
    );

    // Set MapPoint vertices
    let mut mp_indexes = vec![]; // vnIndexEdgeMono
    for i in 0..frame.mappoint_matches.matches.len() {
        if frame.mappoint_matches.matches[i].is_none() {
            continue;
        }
        let (mp_id, _) = frame.mappoint_matches.matches[i].unwrap();

        let (kp, is_right) = frame.features.get_keypoint(i);
        if !is_right {
            // Left monocular observation
            initial_mono_correspondences += 1;
            frame.mappoint_matches.set_outlier(i, false);

            // Add here uncerteinty
            let unc2 = CAMERA_MODULE.uncertainty;
            let inv_sigma2 = INV_LEVEL_SIGMA2[kp.octave() as usize] / unc2;

            optimizer.pin_mut().add_edge_mono_only_pose(
                true,
                vp,
                mp_id,
                (map.read()?.mappoints.get(&mp_id).unwrap().position).into(),
                kp.pt().x, kp.pt().y,
                inv_sigma2,
                *TH_HUBER_MONO
            );
            mp_indexes.push(i as u32);
        } else {
            todo!("Stereo");
            // nInitialStereoCorrespondences++;
            // pFrame->mvbOutlier[i] = false;

            // kpUn = pFrame->mvKeysUn[i];
            // const float kp_ur = pFrame->mvuRight[i];
            // Eigen::Matrix<double,3,1> obs;
            // obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            // EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

            // e->setVertex(0, VP);
            // e->setMeasurement(obs);

            // // Add here uncerteinty
            // const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

            // const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
            // e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

            // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            // e->setRobustKernel(rk);
            // rk->setDelta(thHuberStereo);

            // optimizer.addEdge(e);

            // vpEdgesStereo.push_back(e);
            // vnIndexEdgeStereo.push_back(i);
        }

        // TODO Right monocular observation... not sure what is the difference between the below code and teh above stereo observation code
        // Right monocular observation
        // if(bRight && i >= Nleft)
        // {
        //     nInitialMonoCorrespondences++;
        //     pFrame->mvbOutlier[i] = false;

        //     kpUn = pFrame->mvKeysRight[i - Nleft];
        //     Eigen::Matrix<double,2,1> obs;
        //     obs << kpUn.pt.x, kpUn.pt.y;

        //     EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

        //     e->setVertex(0,VP);
        //     e->setMeasurement(obs);

        //     // Add here uncerteinty
        //     const float unc2 = pFrame->mpCamera->uncertainty2(obs);

        //     const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
        //     e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        //     g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        //     e->setRobustKernel(rk);
        //     rk->setDelta(thHuberMono);

        //     optimizer.addEdge(e);

        //     vpEdgesMono.push_back(e);
        //     vnIndexEdgeMono.push_back(i);
        // }
    }

    let num_initial_correspondences = initial_mono_correspondences + initial_stereo_correspondences;

    // Set Previous Frame Vertex
    let vpk = 4;
    let vvk = 5;
    let vgk = 6;
    let vak = 7;

    add_vertex_pose_frame(&mut optimizer, previous_frame, false, vpk);
    optimizer.pin_mut().add_vertex_velocity(
        vvk,
        false,
        previous_frame.imu_data.velocity.unwrap().into()
    );
    optimizer.pin_mut().add_vertex_gyrobias(
        vgk,
        false,
        previous_frame.imu_data.imu_bias.get_gyro_bias().into()
    );
    optimizer.pin_mut().add_vertex_accbias(
        vak,
        false,
        previous_frame.imu_data.imu_bias.get_acc_bias().into()
    );

    optimizer.pin_mut().add_edge_inertial(
        vpk, vvk, vgk, vak, vp, vv,
        frame.imu_data.imu_preintegrated.as_ref().unwrap().into(),
        false,
        0.0
    );

    optimizer.pin_mut().add_edge_gyro_and_acc(
        vgk, vg,
        vak, va,
        frame.imu_data.imu_preintegrated.as_ref().unwrap().into(),
    );

    // todo sofiya where does  pFp->mpcpi come from?
    if previous_frame.imu_data.constraint_pose_imu.is_some() {
        let cpi = previous_frame.imu_data.constraint_pose_imu.as_ref().unwrap();
        optimizer.pin_mut().add_edge_prior_pose_imu(
            vpk, vvk, vgk, vak,
            (&cpi.rwb).into(), cpi.twb.into(), cpi.vwb.into(), cpi.bg.into(), cpi.ba.into(), cpi.h.into(),
            true, 5.0
        );
    } else {
        panic!("pFp->mpcpi does not exist!!! Previous Frame: {}", previous_frame.frame_id);
    }

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    let chi2_mono = vec![5.991,5.991,5.991,5.991];
    let _chi2_stereo = vec![7.815,7.815,7.815, 7.815];
    let iterations = vec![10,10,10,10];

    let mut num_bad = 0;
    let mut num_inliers_mono;
    let mut num_inliers_stereo;
    let mut num_inliers = 0;
    for iteration in 0..4 {
        optimizer.pin_mut().optimize(iterations[iteration], false, false);

        num_bad = 0;
        num_inliers_mono = 0;
        num_inliers_stereo = 0;

        let chi2_close = 1.5 * chi2_mono[iteration];

        let mut i = 0;
        for mut edge in optimizer.pin_mut().get_mut_mono_onlypose_edges().iter_mut() {
            let mp_idx = mp_indexes[i];
            let is_close = tracked_mappoint_data.get(&edge.mappoint_id).is_some() && tracked_mappoint_data.get(&edge.mappoint_id).unwrap().track_depth < 10.0;

            if frame.mappoint_matches.is_outlier(&mp_idx) {
                edge.inner.pin_mut().compute_error();
            }

            let chi2 = edge.inner.chi2();

            if (chi2 > chi2_mono[iteration] && !is_close) ||
                (is_close && chi2 > chi2_close) ||
                ! edge.inner.is_depth_positive()
            {
                frame.mappoint_matches.set_outlier(mp_idx as usize, true);
                edge.inner.pin_mut().set_level(1);
                num_bad += 1;
            } else {
                frame.mappoint_matches.set_outlier(mp_idx as usize, false);
                edge.inner.pin_mut().set_level(0);
                num_inliers_mono += 1;
            }

            if iteration == 2 {
                edge.inner.pin_mut().set_robust_kernel(false);
            }
            i += 1;
        }

        match sensor.frame() {
            FrameSensor::Stereo => {
                todo!("Stereo");
                // for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
                // {
                //     EdgeStereoOnlyPose* e = vpEdgesStereo[i];

                //     const size_t idx = vnIndexEdgeStereo[i];

                //     if(pFrame->mvbOutlier[idx])
                //     {
                //         e->computeError();
                //     }

                //     const float chi2 = e->chi2();

                //     if(chi2>chi2Stereo[it])
                //     {
                //         pFrame->mvbOutlier[idx]=true;
                //         e->setLevel(1);
                //         nBadStereo++;
                //     }
                //     else
                //     {
                //         pFrame->mvbOutlier[idx]=false;
                //         e->setLevel(0);
                //         nInliersStereo++;
                //     }

                //     if(it==2)
                //         e->setRobustKernel(0);
                // }
            },
            _ => {}
        }

        num_inliers = num_inliers_mono + num_inliers_stereo;

        if optimizer.num_edges() < 10 {
            break;
        }
    }


    if num_inliers < 30 {
        num_bad = 0;
        let chi2_mono_out = 18.0;
        let _chi2_stereo_out = 24.0;

        let mut i = 0;
        for mut edge in optimizer.pin_mut().get_mut_mono_onlypose_edges().iter_mut() {
            let idx = mp_indexes[i];
            edge.inner.pin_mut().compute_error();
            if edge.inner.chi2() < chi2_mono_out {
                frame.mappoint_matches.set_outlier(idx as usize, false);
            } else {
                num_bad += 1;
            }
            i += 1;
        }

        match sensor.frame() {
            FrameSensor::Stereo => {
                todo!("Stereo");
                // for(size_t i=0, iend=vnIndexEdgeStereo.size(); i<iend; i++)
                // {
                //     const size_t idx = vnIndexEdgeStereo[i];
                //     e2 = vpEdgesStereo[i];
                //     e2->computeError();
                //     if (e2->chi2()<chi2StereoOut)
                //         pFrame->mvbOutlier[idx]=false;
                //     else
                //         nBad++;
                // }
            },
            _ => {}
        }
    }

    // num_inliers = num_inliers_mono + num_inliers_stereo;

    // Recover optimized pose, velocity and biases
    let recovered_pose: Pose = optimizer.recover_optimized_vertex_pose(vp).into();
    let recovered_velocity = optimizer.recover_optimized_vertex_velocity(vv);
    frame.set_imu_pose_velocity(recovered_pose, recovered_velocity.into());

    let recovered_bias_estimate = optimizer.recover_optimized_inertial(
        vg, va, -1, -1
    );
    let recovered_bias = ImuBias {
        bax: recovered_bias_estimate.vb[0],
        bay: recovered_bias_estimate.vb[1],
        baz: recovered_bias_estimate.vb[2],
        bwx: recovered_bias_estimate.vb[3],
        bwy: recovered_bias_estimate.vb[4],
        bwz: recovered_bias_estimate.vb[5],
    };

    // Recover Hessian, marginalize previous frame states and generate new prior for frame
    let mut h = nalgebra::SMatrix::<f64, 30, 30>::zeros();
    let h_i: nalgebra::SMatrix::<f64, 24, 24> = optimizer.get_hessian_from_edge_inertial(0).into();
    h.view_mut((0, 0), (24, 24)).add_assign(&h_i); // Only one edge was created in this case

    // H.block<24,24>(0,0)+= ei->GetHessian(); 
        // ei from:
        // EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

    let hgr: nalgebra::SMatrix::<f64, 6, 6> = optimizer.get_hessian_from_edge_gyro().into();
    // Eigen::Matrix<double,6,6> Hgr = egr->GetHessian();
        // egr from:
        // EdgeGyroRW* egr = new EdgeGyroRW();
    h.view_mut((9,9), (3,3)).add_assign(&hgr.view((0,0), (3,3)));
    h.view_mut((9,24), (3,3)).add_assign(&hgr.view((0,3), (3,3)));
    h.view_mut((24,9), (3,3)).add_assign(&hgr.view((3,0), (3,3)));
    h.view_mut((24,24), (3,3)).add_assign(&hgr.view((3,3), (3,3)));

    let hgr: nalgebra::SMatrix::<f64, 6, 6> = optimizer.get_hessian_from_edge_acc().into();
    // Eigen::Matrix<double,6,6> Har = ear->GetHessian();
        // ear from: 
        // EdgeAccRW* ear = new EdgeAccRW();
    h.view_mut((12,12), (3,3)).add_assign(&hgr.view((0,0), (3,3)));
    h.view_mut((12,27), (3,3)).add_assign(&hgr.view((0,3), (3,3)));
    h.view_mut((27,12), (3,3)).add_assign(&hgr.view((3,0), (3,3)));
    h.view_mut((27,27), (3,3)).add_assign(&hgr.view((3,3), (3,3)));

    let h_ep: nalgebra::SMatrix::<f64, 15, 15> = optimizer.get_hessian_from_edge_prior().into();
    h.view_mut((0,0), (15,15)).add_assign(&h_ep);
    // H.block<15,15>(0,0) += ep->GetHessian();
        // ep from:
        // EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

    let mut i = 0;
    for mut edge in optimizer.pin_mut().get_mut_mono_onlypose_edges().iter_mut() {
        let idx = mp_indexes[i];
        if !frame.mappoint_matches.is_outlier(&idx) {
            h.view_mut((15, 15), (6, 6)).add_assign(&edge.inner.pin_mut().get_hessian().into());
        }
        i += 1;
    }

    match sensor.frame() {
        FrameSensor::Stereo => {
            todo!("Stereo");
            // for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
            // {
            //     EdgeStereoOnlyPose* e = vpEdgesStereo[i];

            //     const size_t idx = vnIndexEdgeStereo[i];

            //     if(!pFrame->mvbOutlier[idx])
            //     {
            //         H.block<6,6>(15,15) += e->GetHessian();
            //         tot_in++;
            //     }
            //     else
            //         tot_out++;
            // }
        },
        _ => {}
    };

    h = marginalize(h, 0, 14);

    let final_h = {
        let h_15x15 = h.view((15, 15), (15, 15)).into_owned();
        let static_view_from_dyn: nalgebra::SMatrixView<f64, 15, 15> = h_15x15.as_view();
        static_view_from_dyn.into_owned()
    };

    frame.imu_data.constraint_pose_imu = Some(ConstraintPoseImu::new(
        recovered_pose,
        recovered_velocity.into(),
        recovered_bias,
        final_h
    )).unwrap();
        // pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,VP->estimate().twb,VV->estimate(),VG->estimate(),VA->estimate(),H.block<15,15>(15,15));
    println!("Add mpcpi for frame {}", frame.frame_id);

    previous_frame.imu_data.constraint_pose_imu = None;

    return Ok(num_initial_correspondences - num_bad);
}

pub fn pose_inertial_optimization_last_keyframe(
    frame: &mut Frame, tracked_mappoint_data: & HashMap<Id, TrackedMapPointData>,
    map: &ReadWriteMap, sensor: &Sensor
) -> Result<i32, Box<dyn std::error::Error> > {
    // int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
    // but bRecInit is always set to false

    debug!("POSE INERTIAL OPTIMIZATION LAST KEYFRAME!");

    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(7, camera_param, 0.0);

    let mut initial_mono_correspondences = 0; //nInitialMonoCorrespondences
    let mut _initial_stereo_correspondences = 0; //nInitialStereoCorrespondences

    // Set Frame vertex
    let vp = 0;
    let vv = 1;
    let vg = 2;
    let va = 3;
    add_vertex_pose_frame(&mut optimizer, frame, false, vp);
    optimizer.pin_mut().add_vertex_velocity(
        vv, false, frame.imu_data.get_velocity().into()
    );
    optimizer.pin_mut().add_vertex_gyrobias(
        vg, false, frame.imu_data.imu_bias.get_gyro_bias().into()
    );
    optimizer.pin_mut().add_vertex_accbias(
        va, false, frame.imu_data.imu_bias.get_acc_bias().into()
    );

    // Set MapPoint vertices
    let mut mp_indexes = vec![]; // vnIndexEdgeMono
    for i in 0..frame.mappoint_matches.matches.len() {
        if frame.mappoint_matches.matches[i].is_none() {
            continue;
        }
        let (mp_id, _) = frame.mappoint_matches.matches[i].unwrap();

        let (kp, is_right) = frame.features.get_keypoint(i);
        if !is_right {
            // Left monocular observation
            initial_mono_correspondences += 1;
            frame.mappoint_matches.set_outlier(i, false);

            // Add here uncerteinty
            let unc2 = CAMERA_MODULE.uncertainty;
            let inv_sigma2 = INV_LEVEL_SIGMA2[kp.octave() as usize] / unc2;

            optimizer.pin_mut().add_edge_mono_only_pose(
                true,
                vp,
                mp_id,
                (map.read()?.mappoints.get(&mp_id).unwrap().position).into(),
                kp.pt().x, kp.pt().y,
                inv_sigma2,
                *TH_HUBER_MONO
            );
            mp_indexes.push(i as u32);
        } else {
            todo!("Stereo");
            // Stereo observation
            // nInitialStereoCorrespondences++;
            // pFrame->mvbOutlier[i] = false;

            // kpUn = pFrame->mvKeysUn[i];
            // const float kp_ur = pFrame->mvuRight[i];
            // Eigen::Matrix<double,3,1> obs;
            // obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

            // EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

            // e->setVertex(0, VP);
            // e->setMeasurement(obs);

            // // Add here uncerteinty
            // const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

            // const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
            // e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

            // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            // e->setRobustKernel(rk);
            // rk->setDelta(thHuberStereo);

            // optimizer.addEdge(e);

            // vpEdgesStereo.push_back(e);
            // vnIndexEdgeStereo.push_back(i);
        }

        // TODO Right monocular observation... not sure what is the difference between the below code and teh above stereo observation code
        // Right monocular observation
        // if(bRight && i >= Nleft)
        // {
        //     nInitialMonoCorrespondences++;
        //     pFrame->mvbOutlier[i] = false;

        //     kpUn = pFrame->mvKeysRight[i - Nleft];
        //     Eigen::Matrix<double,2,1> obs;
        //     obs << kpUn.pt.x, kpUn.pt.y;

        //     EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

        //     e->setVertex(0,VP);
        //     e->setMeasurement(obs);

        //     // Add here uncerteinty
        //     const float unc2 = pFrame->mpCamera->uncertainty2(obs);

        //     const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
        //     e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

        //     g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        //     e->setRobustKernel(rk);
        //     rk->setDelta(thHuberMono);

        //     optimizer.addEdge(e);

        //     vpEdgesMono.push_back(e);
        //     vnIndexEdgeMono.push_back(i);
        // }
    }

    let num_initial_correspondences = initial_mono_correspondences + _initial_stereo_correspondences;

    // Set Previous Frame Vertex
    let vpk = 4;
    let vvk = 5;
    let vgk = 6;
    let vak = 7;
    {
        let lock = map.read()?;
        let previous_keyframe = lock.get_keyframe(frame.imu_data.prev_keyframe.expect("Frame has IMU data but no prev keyframe?"));
        add_vertex_pose_keyframe(&mut optimizer, previous_keyframe, true, vpk);
        optimizer.pin_mut().add_vertex_velocity(
            vvk,
            true,
            previous_keyframe.imu_data.velocity.unwrap().into()
        );
        optimizer.pin_mut().add_vertex_gyrobias(
            vgk,
            true,
            previous_keyframe.imu_data.imu_bias.get_gyro_bias().into()
        );
        optimizer.pin_mut().add_vertex_accbias(
            vak,
            true,
            previous_keyframe.imu_data.imu_bias.get_acc_bias().into()
        );
    }

    optimizer.pin_mut().add_edge_inertial(
        vpk, vvk, vgk, vak, vp, vv,
        frame.imu_data.imu_preintegrated.as_ref().unwrap().into(),
        false,
        0.0
    );

    optimizer.pin_mut().add_edge_gyro_and_acc(
        vgk, vg,
        vak, va,
        frame.imu_data.imu_preintegrated.as_ref().unwrap().into(),
    );

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    let chi2_mono = vec![5.991,5.991,5.991,5.991];
    let _chi2_stereo = vec![7.815,7.815,7.815, 7.815];
    let iterations = vec![10,10,10,10];

    let mut num_bad = 0;
    let mut num_inliers = 0;
    for iteration in 0..4 {
        optimizer.pin_mut().optimize(iterations[iteration], false, false);

        num_bad = 0;
        num_inliers = 0;

        let chi2_close = 1.5 * chi2_mono[iteration];

        // For monocular observations
        let mut i = 0;
        for mut edge in optimizer.pin_mut().get_mut_mono_onlypose_edges().iter_mut() {
            let mp_idx = mp_indexes[i];
            let is_close = tracked_mappoint_data.get(&edge.mappoint_id).is_some() && tracked_mappoint_data.get(&edge.mappoint_id).unwrap().track_depth < 10.0;

            if frame.mappoint_matches.is_outlier(&mp_idx) {
                edge.inner.pin_mut().compute_error();
            }

            let chi2 = edge.inner.chi2();

            if (chi2 > chi2_mono[iteration] && !is_close) ||
                (is_close && chi2 > chi2_close) ||
                ! edge.inner.is_depth_positive()
            {
                frame.mappoint_matches.set_outlier(mp_idx as usize, true);
                edge.inner.pin_mut().set_level(1);
                num_bad += 1;
            } else {
                frame.mappoint_matches.set_outlier(mp_idx as usize, false);
                edge.inner.pin_mut().set_level(0);
            }

            if iteration == 2 {
                edge.inner.pin_mut().set_robust_kernel(false);
            }
            i += 1;
        }

        match sensor.frame() {
            FrameSensor::Stereo => {
                todo!("Stereo");
                // For stereo observations
                // for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
                // {
                //     EdgeStereoOnlyPose* e = vpEdgesStereo[i];

                //     const size_t idx = vnIndexEdgeStereo[i];

                //     if(pFrame->mvbOutlier[idx])
                //     {
                //         e->computeError();
                //     }

                //     const float chi2 = e->chi2();

                //     if(chi2>chi2Stereo[it])
                //     {
                //         pFrame->mvbOutlier[idx]=true;
                //         e->setLevel(1); // not included in next optimization
                //         nBadStereo++;
                //     }
                //     else
                //     {
                //         pFrame->mvbOutlier[idx]=false;
                //         e->setLevel(0);
                //         nInliersStereo++;
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

    // If not too much tracks, recover not too bad points
    if num_inliers < 30 {
        num_bad = 0;
        let chi2_mono_out = 18.0;
        let _chi2_stereo_out = 24.0;

        let mut i = 0;
        for mut edge in optimizer.pin_mut().get_mut_mono_onlypose_edges().iter_mut() {
            let idx = mp_indexes[i];
            edge.inner.pin_mut().compute_error();
            if edge.inner.chi2() < chi2_mono_out {
                frame.mappoint_matches.set_outlier(idx as usize, false);
            } else {
                num_bad += 1;
            }
            i += 1;
        }

        match sensor.frame() {
            FrameSensor::Stereo => {
                todo!("Stereo");
                // for(size_t i=0, iend=vnIndexEdgeStereo.size(); i<iend; i++)
                // {
                //     const size_t idx = vnIndexEdgeStereo[i];
                //     e2 = vpEdgesStereo[i];
                //     e2->computeError();
                //     if (e2->chi2()<chi2StereoOut)
                //         pFrame->mvbOutlier[idx]=false;
                //     else
                //         nBad++;
                // }
            },
            _ => {}
        }
    }

    // Recover optimized pose, velocity and biases
    let recovered_pose: Pose = optimizer.recover_optimized_vertex_pose(vp).into();
    let recovered_velocity = optimizer.recover_optimized_vertex_velocity(vv);
    frame.set_imu_pose_velocity(recovered_pose, recovered_velocity.into());

    let recovered_bias_estimate = optimizer.recover_optimized_inertial(
        vg, va, -1, -1
    );
    let recovered_bias = ImuBias {
        bax: recovered_bias_estimate.vb[0],
        bay: recovered_bias_estimate.vb[1],
        baz: recovered_bias_estimate.vb[2],
        bwx: recovered_bias_estimate.vb[3],
        bwy: recovered_bias_estimate.vb[4],
        bwz: recovered_bias_estimate.vb[5],
    };

    // Recover Hessian, marginalize keyFframe states and generate new prior for frame
    let mut h = nalgebra::SMatrix::<f64, 15, 15>::zeros();
    h.view_mut((0, 0), (9, 9)).add_assign(&optimizer.get_hessian2_from_edge_inertial(0).into());
    h.view_mut((9, 9), (3, 3)).add_assign(&optimizer.get_hessian2_from_edge_gyro().into());
    h.view_mut((12, 12), (3, 3)).add_assign(&optimizer.get_hessian2_from_edge_acc().into());

    let mut i = 0;
    for mut edge in optimizer.pin_mut().get_mut_mono_onlypose_edges().iter_mut() {
        let idx = mp_indexes[i];
        if !frame.mappoint_matches.is_outlier(&idx) {
            h.view_mut((0, 0), (6, 6)).add_assign(&edge.inner.pin_mut().get_hessian().into());
        }
        i += 1;
    }

    match sensor.frame() {
        FrameSensor::Stereo => {
            todo!("Stereo");
            // for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
            // {
            //     EdgeStereoOnlyPose* e = vpEdgesStereo[i];

            //     const size_t idx = vnIndexEdgeStereo[i];

            //     if(!pFrame->mvbOutlier[idx])
            //     {
            //         H.block<6,6>(0,0) += e->GetHessian();
            //         tot_in++;
            //     }
            //     else
            //         tot_out++;
            // }
        },
        _ => {}
    };

    frame.imu_data.constraint_pose_imu = Some(ConstraintPoseImu::new(
        recovered_pose,
        recovered_velocity.into(),
        recovered_bias,
        h
    )).unwrap();
    println!("Add mpcpi for frame {}", frame.frame_id);
        // pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,VP->estimate().twb,VV->estimate(),VG->estimate(),VA->estimate(),H);

    return Ok(num_initial_correspondences - num_bad);
}

pub fn inertial_optimization_initialization(
    map: &ReadWriteMap, rwg: &mut DVMatrix3<f64>, scale: &mut f64, 
    bg: &mut DVVector3<f64>, ba: &mut DVVector3<f64>, is_mono: bool,
    fixed_velocity: bool, prior_g: f64, prior_a: f64,
) -> Result<(), Box<dyn std::error::Error> > {
    // void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel, bool bGauss, float priorG, float priorA)

    let its = 200;
    let max_kf_id = * map.read()?.get_keyframes_iter()
        .max_by(|a, b| a.1.id.cmp(&b.1.id))
        .map(|(k, _v)| k).unwrap();

    // Setup optimizer
    // ... Note... pretty sure camera params aren't necessary for this optimization but throwing them in here anyway just in case
    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    println!("IMU init.... initial rwg: {:?}", rwg);

    let mut optimizer = if prior_g != 0.0 {
        g2o::ffi::new_sparse_optimizer(5, camera_param, 1e3)
    } else {
        g2o::ffi::new_sparse_optimizer(5, camera_param, 0.0)
    };

    {
        let lock = map.read()?;

        // Set KeyFrame vertices (fixed poses and optimizable velocities)
        for (kf_id, kf) in lock.get_keyframes_iter() {
            add_vertex_pose_keyframe(&mut optimizer, kf, true, kf.id);

            let velocity = match kf.imu_data.velocity {
                Some(v) => v,
                None => {
                    panic!("Should have velocity by now!")
                }
            };
            println!("IMU init.... initial KF {} velocity: {:?}", kf.id, kf.imu_data.velocity.unwrap());

            optimizer.pin_mut().add_vertex_velocity(
                max_kf_id + kf_id + 1,
                fixed_velocity,
                velocity.into()
            );
        }

        // Biases
        let first_kf = lock.get_first_keyframe();
        let gyro_bias = first_kf.imu_data.imu_bias.get_gyro_bias();
        let vertex_gyro_bias_id = max_kf_id * 2 + 2;
        optimizer.pin_mut().add_vertex_gyrobias(
            vertex_gyro_bias_id,
            fixed_velocity,
            gyro_bias.into()
        );
        let acc_bias = first_kf.imu_data.imu_bias.get_acc_bias();
        let vertex_acc_bias_id = max_kf_id * 2 + 3;
        optimizer.pin_mut().add_vertex_accbias(
            vertex_acc_bias_id,
            fixed_velocity,
            acc_bias.into()
        );

        // prior acc bias
        optimizer.pin_mut().add_edge_prior_for_imu(
            vertex_acc_bias_id,
            vertex_gyro_bias_id,
            [0.0, 0.0, 0.0],
            prior_a,
            prior_g
        );
        println!("IMU init.... initial gyro bias: {:?}", gyro_bias);
        println!("IMU init.... initial acc bias: {:?}", acc_bias);

        optimizer.pin_mut().add_vertex_gdir(
            max_kf_id * 2 + 4,
            false,
            rwg.into()
        );
        optimizer.pin_mut().add_vertex_scale(
            max_kf_id * 2 + 5,
            !is_mono,  // Fixed for stereo case
            *scale,
        );
        println!("IMU init.... initial scale: {:?}", scale);

    }

    {
        // Graph edges
        // IMU links with gravity and scale
        let mut new_imu_preintegrated_for_kfs: HashMap<Id, ImuPreIntegrated> = HashMap::new();
        for (kf_id, keyframe) in map.read()?.get_keyframes_iter() {
            if *kf_id > max_kf_id ||
                keyframe.prev_kf_id.is_none() ||
                keyframe.imu_data.imu_preintegrated.is_none() ||
                keyframe.prev_kf_id.unwrap() > max_kf_id 
            {
                continue;
            }

            let mut imu_preintegrated = keyframe.imu_data.imu_preintegrated.as_ref().unwrap().clone();
            let prev_kf_id = keyframe.prev_kf_id.unwrap();


            imu_preintegrated.set_new_bias(map.read()?.get_keyframe(prev_kf_id).imu_data.imu_bias);

            optimizer.pin_mut().add_edge_inertial_gs(
                prev_kf_id,
                max_kf_id + prev_kf_id + 1,
                max_kf_id * 2 + 2,
                max_kf_id * 2 + 3,
                * kf_id,
                max_kf_id + kf_id + 1,
                max_kf_id * 2 + 4,
                max_kf_id * 2 + 5,
                (& imu_preintegrated).into(),
                false,
                1.0,
                false,
                0.0
            );
            new_imu_preintegrated_for_kfs.insert(*kf_id, imu_preintegrated);
        }
        for (kf_id, new_imu_preintegrated) in new_imu_preintegrated_for_kfs {
            let mut lock = map.write()?;
            lock.get_keyframe_mut(kf_id).imu_data.imu_preintegrated = Some(new_imu_preintegrated);
        }
    }

    // Compute error for different scales
    optimizer.pin_mut().optimize(its, false, false);

    // Recover optimized data
    // Biases
    let estimate = optimizer.recover_optimized_inertial(
        max_kf_id * 2 + 2,
        max_kf_id * 2 + 3,
        max_kf_id * 2 + 5,
        max_kf_id * 2 + 4,
    );

    let vb = estimate.vb;
    *bg = estimate.bg.into();
    *ba = estimate.ba.into();
    *scale = estimate.scale;
    *rwg = estimate.rwg.into();

    // println!("C++ RWG: {:?}", estimate.rwg);
    // println!("Rust RWG: {:?}", rwg);

    let b = ImuBias {
        bax: vb[0],
        bay: vb[1],
        baz: vb[2],
        bwx: vb[3],
        bwy: vb[4],
        bwz: vb[5],
    };

    println!("IMU init.... RESULT bias: {:?}", b);

    //Keyframes velocities and biases
    let mut lock = map.write()?;
    for (kf_id, kf) in lock.get_keyframes_iter_mut() {
        if *kf_id > max_kf_id {
            continue;
        }

        let velocity = optimizer.recover_optimized_vertex_velocity(max_kf_id + kf_id + 1);
        kf.imu_data.velocity = Some(velocity.into());  // Velocity is scaled after

        if (* kf.imu_data.imu_bias.get_gyro_bias() - ** bg).norm() > 0.01 {
            kf.imu_data.set_new_bias(b);
            if let Some(imu_preintegrated) = kf.imu_data.imu_preintegrated.as_mut() {
                imu_preintegrated.reintegrate();
                println!("(IMU init) reintegrate");
            }
        } else {
            kf.imu_data.set_new_bias(b);
        }
        println!("IMU init.... RESULT KF velocity {} {:?}", kf.id, kf.imu_data.velocity.unwrap());
        println!("IMU init.... RESULT KF bias {} {:?}", kf.id, kf.imu_data.imu_bias);

    }
    Ok(())
}

pub fn inertial_optimization_scale_refinement(map: &ReadWriteMap, rwg: &mut nalgebra::Matrix3<f64>, scale: &mut f64) -> Result<(), Box<dyn std::error::Error> > {
    // void Optimizer::InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale)
    let max_kf_id = * map.read()?.get_keyframes_iter()
        .max_by(|a, b| a.1.id.cmp(&b.1.id))
        .map(|(k, _v)| k).unwrap();

    let its = 10;
    // Setup optimizer
    // ... Note... pretty sure camera params aren't necessary for this optimization but throwing them in here anyway just in case
    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(6, camera_param, 1e3);

    {
        let lock = map.read()?;

        // Set KeyFrame vertices (all variables are fixed)
        for (kf_id, kf) in lock.get_keyframes_iter() {
            add_vertex_pose_keyframe(&mut optimizer, kf, true, kf.id);

            debug!("kf.imu_data.velocity {:?}", kf.imu_data.velocity);
            optimizer.pin_mut().add_vertex_velocity(
                max_kf_id + kf_id + 1,
                true,
                kf.imu_data.velocity.unwrap().into()
            );

            // Vertex of fixed biases
            let first_kf = lock.get_first_keyframe();
            let gyro_bias = first_kf.imu_data.imu_bias.get_gyro_bias();
            optimizer.pin_mut().add_vertex_gyrobias(
                2 * (max_kf_id + 1) + kf_id,
                true,
                gyro_bias.into()
            );

            optimizer.pin_mut().add_vertex_accbias(
                3 * (max_kf_id + 1) + kf_id,
                true,
                first_kf.imu_data.imu_bias.get_acc_bias().into()
            );
        }

        // Gravity and scale
        optimizer.pin_mut().add_vertex_gdir(
            4 * (max_kf_id + 1),
            false,
            (nalgebra::Matrix3::<f64>::identity()).into(),
        );
        optimizer.pin_mut().add_vertex_scale(
            4 * (max_kf_id + 1) + 1,
            false,
            * scale,
        );
    }

    {
        // Graph edges
        // IMU links with gravity and scale
        let mut new_imu_preintegrated_for_kfs: HashMap<Id, ImuPreIntegrated> = HashMap::new();
        for (kf_id, keyframe) in map.read()?.get_keyframes_iter() {
            if *kf_id > max_kf_id ||
                keyframe.prev_kf_id.is_none() ||
                keyframe.imu_data.imu_preintegrated.is_none() ||
                keyframe.prev_kf_id.unwrap() > max_kf_id 
            {
                continue;
            }

            let mut imu_preintegrated = keyframe.imu_data.imu_preintegrated.as_ref().unwrap().clone();
            let prev_kf_id = keyframe.prev_kf_id.unwrap();

            imu_preintegrated.set_new_bias(map.read()?.get_keyframe(prev_kf_id).imu_data.imu_bias);

            optimizer.pin_mut().add_edge_inertial_gs(
                prev_kf_id,
                max_kf_id + 1 + prev_kf_id,
                *kf_id,
                max_kf_id + 1 + prev_kf_id,
                2 * (max_kf_id + 1) + prev_kf_id,
                3 * (max_kf_id + 1) + prev_kf_id,
                4 * (max_kf_id + 1),
                4 * (max_kf_id + 1) + 1,
                (& imu_preintegrated).into(),
                true,
                1.0,
                false,
                0.0
            );
            new_imu_preintegrated_for_kfs.insert(*kf_id, imu_preintegrated);
        }
        for (kf_id, new_imu_preintegrated) in new_imu_preintegrated_for_kfs {
            let mut lock = map.write()?;
            lock.get_keyframe_mut(kf_id).imu_data.imu_preintegrated = Some(new_imu_preintegrated);
        }
    }
    // Compute error for different scales
    optimizer.pin_mut().optimize(its, false, false);

    // Recover optimized data
    let estimate = optimizer.recover_optimized_inertial(
        -1, // this doesn't matter this time
        -1, // this doesn't matter this time
        4 * (max_kf_id + 1) + 1,
        4 * (max_kf_id + 1),
    );

    *scale = estimate.scale;
    *rwg = estimate.rwg.into();

    Ok(())
}

pub fn optimize_pose(
    frame: &mut Frame, map: &ReadWriteMap
) -> Result<Option<i32>, Box<dyn std::error::Error> > {
    //int Optimizer::PoseOptimization(Frame *pFrame)
    let _span = tracy_client::span!("optimize_pose");

    debug!("REGULAR POSE OPTIMIZATION!");

    let sensor: Sensor = SETTINGS.get(SYSTEM, "sensor");
    let fx= SETTINGS.get::<f64>(CAMERA, "fx");
    let fy= SETTINGS.get::<f64>(CAMERA, "fy");
    let cx= SETTINGS.get::<f64>(CAMERA, "cx");
    let cy= SETTINGS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(2, camera_param, 0.0);

    optimizer.pin_mut().add_vertex_se3expmap(0, (*frame.pose.as_ref().unwrap()).into(), false);

    let mut initial_correspondences = 0;
    let mut mp_indexes = vec![];

    {
        // Take lock to construct factor graph
        let map_read_lock = map.read()?;
        for i in 0..frame.mappoint_matches.len() {
            if let Some((mp_id, _is_outlier)) = frame.mappoint_matches.get(i) {
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
                        //     let map_read_lock = map.read()?;
                        //     let position = &map_read_lock.mappoints.get(mp_id).unwrap().position;
                        //     optimizer.add_edge_stereo(
                        //         i as i32, edge.clone(), (position).into()
                        //     );
                        // }
                    },
                    _ => {
                        // Mono observations
                        frame.mappoint_matches.set_outlier(i as usize, false);
                        optimizer.pin_mut().add_edge_se3_project_xyz_monocular_unary(
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
        return Ok(None);
    }

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    let chi2_mono = vec![5.991,5.991,5.991,5.991];
    let _chi2_stereo = vec![7.815,7.815,7.815, 7.815];
    let iterations = vec![10,10,10,10];

    let mut num_bad = 0;
    for iteration in 0..4 {
        optimizer.pin_mut().update_estimate_vertex_se3xpmap(
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
    return Ok(Some(initial_correspondences - num_bad));
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
    map: &ReadWriteMap, loop_kf: Id, curr_kf: Id,
    loop_connections: BTreeMap<Id, HashSet<Id>>,
    non_corrected_sim3: &KeyFrameAndPose, corrected_sim3: &KeyFrameAndPose,
    corrected_mp_references: HashMap::<Id, Id>,
    fix_scale: bool
) -> Result<(), Box<dyn std::error::Error> > {
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
    let mut optimizer = g2o::ffi::new_sparse_optimizer(4, camera_param, 1e-16);

    let mut v_scw: HashMap<Id, Sim3> = HashMap::new(); // vScw

    let mut inserted_edges = HashSet::new(); // sInsertedEdges
    {
        let mut edges_per_vertex = HashMap::new(); // for testing

        // Set KeyFrame vertices
        let lock = map.read()?;
        for (kf_id, kf) in lock.get_keyframes_iter() {
            let estimate = match corrected_sim3.get(kf_id) {
                Some(sim3) => *sim3,
                None => kf.get_pose().into()
            };
            v_scw.insert(*kf_id, estimate);
            edges_per_vertex.insert(*kf_id, [0, 0, 0, 0]);

            optimizer.pin_mut().add_vertex_sim3expmap(
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
            let kf_i = lock.get_keyframe(*kf_i_id);
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
        for (kf_i_id, kf_i) in lock.get_keyframes_iter() {
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
        let mut lock = map.write()?;
        for (kf_id, kf) in lock.get_keyframes_iter_mut() {
            let corrected_siw: Sim3 = optimizer.recover_optimized_sim3(kf.id).into();
            corrected_swc.insert(*kf_id, corrected_siw.inverse());

            let tiw: Pose = corrected_siw.into(); //[R t/s;0 1]

            kf.set_pose(tiw);
        }
    }

    {
        // Correct points. Transform to "non-optimized" reference keyframe pose and transform back with optimized pose
        let mp_ids = map.read()?.mappoints.keys().cloned().collect::<Vec<Id>>();

        let mps_corrected_by: HashMap<Id, i32> = HashMap::new(); // mnCorrectedByKF

        for mp_id in mp_ids {
            let idr = if mps_corrected_by.contains_key(&mp_id) && *mps_corrected_by.get(&mp_id).unwrap() == curr_kf {
                *corrected_mp_references.get(&mp_id).unwrap()
            } else {
                map.read()?.mappoints.get(&mp_id).unwrap().ref_kf_id
            };

            let srw = v_scw.get(&idr).unwrap();
            let corrected_swr = corrected_swc[&idr];

            {
                let mut lock = map.write()?;
                let mp = lock.mappoints.get_mut(&mp_id).unwrap();
                let p_3d_w = mp.position;
                mp.position = corrected_swr.map(&srw.map(&p_3d_w));
            }

            let norm_and_depth = {
                let lock = map.read()?;
                lock.mappoints.get(&mp_id).unwrap().get_norm_and_depth(&lock)
            };
            if norm_and_depth.is_some() {
                map.write()?.mappoints.get_mut(&mp_id).unwrap().update_norm_and_depth(norm_and_depth.unwrap());
            }
        }
    }

    map.write()?.map_change_index += 1;
    Ok(())
}

pub fn optimize_sim3(
    map: &ReadWriteMap, kf1_id: Id, kf2_id: Id, matched_mps: &mut Vec<Option<Id>>,
    sim3: &mut Sim3, th2: i32, fix_scale: bool
) -> Result<i32, Box<dyn std::error::Error> > {
    // From ORBSLAM2:
    // int Optimizer::OptimizeSim3(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches1, 
    //                             g2o::Sim3 &g2oS12, const float th2, const bool bFixScale)
    // but bAllPoints is always set to true
    // returns vpMatches1, mAcumHessian
    let _span = tracy_client::span!("optimize_sim3");

    let camera_param = [
        SETTINGS.get::<f64>(CAMERA, "fx"),
        SETTINGS.get::<f64>(CAMERA, "fy"),
        SETTINGS.get::<f64>(CAMERA, "cx"),
        SETTINGS.get::<f64>(CAMERA, "cy")
    ];
    let mut optimizer = g2o::ffi::new_sparse_optimizer(3, camera_param, 1e16);

    // Camera poses
    let (kf1_rot, kf1_trans, kf2_rot, kf2_trans) = { // R1w, t1w, R2w, t2w
        let lock = map.read()?;
        let kf1 = lock.get_keyframe(kf1_id);
        let kf2 = lock.get_keyframe(kf2_id);
        (
            kf1.get_pose().get_rotation(),
            kf1.get_pose().get_translation(),
            kf2.get_pose().get_rotation(),
            kf2.get_pose().get_translation()
        )
    };

    // Set Sim3 vertex
    optimizer.pin_mut().add_vertex_sim3expmap(
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
        let lock = map.read()?;
        let mappoints1 = { // vpMapPoints1
            let kf1 = lock.get_keyframe(kf1_id);
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
                optimizer.pin_mut().add_vertex_sbapointxyz(
                    id1,
                    Pose::new(*translation, Matrix3::identity()).into(), // create pose out of translation only
                    true, false
                );

                let p_3d_2c = *kf2_rot * *mp2.position + *kf2_trans;
                let translation2 = DVTranslation::new(p_3d_2c);
                optimizer.pin_mut().add_vertex_sbapointxyz(
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
            let kf1 = lock.get_keyframe(kf1_id);
            let (kp1, _) = kf1.features.get_keypoint(i);

            // Set edge x2 = S21*X1
            let kf2 = lock.get_keyframe(kf2_id);
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
        return Ok(0);
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
    return Ok(n_in);
}

pub fn add_vertex_pose_keyframe(optimizer: &mut UniquePtr<BridgeSparseOptimizer>, kf: &KeyFrame, fixed: bool, vertex_id: i32) {
    // Helper function to call add_vertex_pose because the arguments are pretty gross
    // Maybe this should be in g2o crate instead of here? But I'm pretty sure g2o crate doesn't know about keyframes at all

    let imu_calib = ImuCalib::new();

    let rot = kf.get_pose().get_quaternion();
    let imu_rot = nalgebra::geometry::UnitQuaternion::from_rotation_matrix(
        & nalgebra::Rotation3::from_matrix(& *kf.get_imu_rotation())
    ); // lol whateverrr
    let tcb_rot = imu_calib.tcb.get_quaternion();

    optimizer.pin_mut().add_vertex_pose(
        vertex_id,
        fixed,
        1, // TODO (Stereo... num cams shouldn't be 1)
        kf.get_imu_position().into(), // imu position
        [imu_rot.i, imu_rot.j, imu_rot.k, imu_rot.w], // imu rotation
        kf.get_pose().get_translation().into(), // translation
        [rot.i, rot.j, rot.k, rot.w], // rotation
        imu_calib.tcb.translation.into(), //tcb translation
        [tcb_rot.i, tcb_rot.j, tcb_rot.k, tcb_rot.w], // tcb rotation
        imu_calib.tbc.translation.into(), // tbc translation
        CAMERA_MODULE.stereo_baseline as f32
    );
}

pub fn add_vertex_pose_frame(optimizer: &mut UniquePtr<BridgeSparseOptimizer>, frame: &Frame, fixed: bool, vertex_id: i32) {
    // Helper function to call add_vertex_pose because the arguments are pretty gross
    // Maybe this should be in g2o crate instead of here? But I'm pretty sure g2o crate doesn't know about keyframes at all

    let imu_calib = ImuCalib::new();

    let rot = frame.pose.unwrap().get_quaternion();
    let imu_rot = nalgebra::geometry::UnitQuaternion::from_rotation_matrix(
        & nalgebra::Rotation3::from_matrix(& *frame.get_imu_rotation())
    ); // lol whateverrr
    let tcb_rot = imu_calib.tcb.get_quaternion();

    // println!("KeyFrame {} translation: {:?}", kf.id, kf.get_pose().get_translation());
    // println!("KeyFrame {} rotation: {:?}", kf.id, kf.get_pose().get_rotation());
    // println!("KeyFrame {} imu position: {:?}", kf.id, kf.get_imu_position());
    // println!("KeyFrame {} imu rotation: {:?}", kf.id, kf.get_imu_rotation());

    optimizer.pin_mut().add_vertex_pose(
        vertex_id,
        fixed,
        1, // TODO (Stereo... num cams shouldn't be 1)
        frame.get_imu_position().into(), // imu position
        [imu_rot.i, imu_rot.j, imu_rot.k, imu_rot.w], // imu rotation
        frame.pose.unwrap().get_translation().into(), // translation
        [rot.i, rot.j, rot.k, rot.w], // rotation
        imu_calib.tcb.translation.into(), //tcb translation
        [tcb_rot.i, tcb_rot.j, tcb_rot.k, tcb_rot.w], // tcb rotation
        imu_calib.tbc.translation.into(), // tbc translation
        CAMERA_MODULE.stereo_baseline as f32
    );
}

pub fn marginalize(h: nalgebra::SMatrix<f64, 30, 30>, start: usize, end: usize) -> nalgebra::SMatrix<f64, 30, 30> {
    // Sofiya: Tested!!
    // Eigen::MatrixXd Optimizer::Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end)
    // Goal
    // a  | ab | ac       a*  | 0 | ac*
    // ba | b  | bc  -->  0   | 0 | 0
    // ca | cb | c        ca* | 0 | c*

    // Size of block before block to marginalize
    let a = start;
    // Size of block to marginalize
    let b = end - start + 1;
    // Size of block after block to marginalize
    let c = h.ncols() - (end + 1);

    // Reorder as follows:
    // a  | ab | ac       a  | ac | ab
    // ba | b  | bc  -->  ca | c  | cb
    // ca | cb | c        ba | bc | b

    let mut hn: nalgebra::DMatrix<f64> = nalgebra::DMatrix::zeros(h.nrows(), h.ncols());
    if a > 0 {
        hn.view_mut((0, 0), (a, a)).copy_from(&h.view((0, 0), (a, a)));
        hn.view_mut((0, a + c), (a, b)).copy_from(&h.view((0, a), (a, b)));
        hn.view_mut((a + c, 0), (b, a)).copy_from(&h.view((a, 0), (b, a)));
    }
    if a > 0 && c > 0 {
        hn.view_mut((0, a), (a, c)).copy_from(&h.view((0, a + b), (a, c)));
        hn.view_mut((a, 0), (c, a)).copy_from(&h.view((a + b, 0), (c, a)));
    }
    if c > 0 {
        hn.view_mut((a, a), (c, c)).copy_from(&h.view((a + b, a + b), (c, c)));
        hn.view_mut((a, a + c), (c, b)).copy_from(&h.view((a + b, a), (c, b)));
        hn.view_mut((a + c, a), (b, c)).copy_from(&h.view((a, a + b), (b, c)));
    }
    hn.view_mut((a + c, a + c), (b, b)).copy_from(&h.view((a, a), (b, b)));

    // Perform marginalization (Schur complement)
    let hn_vec = DVMatrixDynamic::new(hn.view((a + c, a + c), (b, b)).into_owned());

    let svd_result = dvos3binding::ffi::svd((&hn_vec).into(), SVDComputeType::ThinUThinV);
    let mut singular_values_inv = nalgebra::DVector::from_row_slice(& svd_result.singular_values);
    let v = {
        // Note: All this instead of :
        // let v: DVMatrixDynamic<f64> = (& svd_result.v).into();
        // Because then we are not allowed to dereference v
        // Probably an easy workaround somewhere
        let mat = svd_result.v;
        let rows = mat.len();
        let cols = mat[0].vec.len();
        let mut res = nalgebra::DMatrix::zeros(rows, cols);

        for i in 0..mat.len() {
            let row = & mat[i].vec;
            for j in 0..row.len() {
                let r: usize = i.try_into().unwrap();
                let c: usize = j.try_into().unwrap();
                res[(r, c)] = mat[i].vec[j];
            }
        }
        res
    };
    let u: DVMatrixDynamic<f64> = (& svd_result.u).into();

    for i in 0..b {
        if singular_values_inv[i] > 1e-6 {
            singular_values_inv[i] = 1.0 / singular_values_inv[i];
        } else {
            singular_values_inv[i] = 0.0;
        }
    }

    let inv_hb = v * nalgebra::DMatrix::from_diagonal(& singular_values_inv) * u.transpose();

    let new_mat = {
        let first = hn.view((0, 0), (a + c, a + c));
        let second = hn.view((0, a + c), (a + c, b));
        let third = hn.view((a + c, 0), (b, a + c));
        first - second * inv_hb * third
    };
    hn.view_mut((0, 0), (a + c, a + c)).copy_from(&(new_mat).into_owned());
    hn.view_mut((a + c, a + c), (b, b)).fill(0.0);
    hn.view_mut((0, a + c), (a + c, b)).fill(0.0);
    hn.view_mut((a + c, 0), (b, a + c)).fill(0.0);

    // Inverse reorder
    // a*  | ac* | 0       a*  | 0 | ac*
    // ca* | c*  | 0  -->  0   | 0 | 0
    // 0   | 0   | 0       ca* | 0 | c*

    let mut res: nalgebra::SMatrix<f64, 30, 30> = nalgebra::SMatrix::zeros();
    if a > 0 {
        res.view_mut((0, 0), (a, a)).copy_from(&hn.view((0, 0), (a, a)));
        res.view_mut((0, a), (a, b)).copy_from(&hn.view((0, a + c), (a, b)));
        res.view_mut((a, 0), (b, a)).copy_from(&hn.view((a + c, 0), (b, a)));
    }
    if a > 0 && c > 0 {
        res.view_mut((0, a + b), (a, c)).copy_from(&hn.view((0, a), (a, c)));
        res.view_mut((a + b, 0), (c, a)).copy_from(&hn.view((a, 0), (c, a)));
    }
    if c > 0 {
        res.view_mut((a + b, a + b), (c, c)).copy_from(&hn.view((a, a), (c, c)));
        res.view_mut((a + b, a), (c, b)).copy_from(&hn.view((a, a + c), (c, b)));
        res.view_mut((a, a + b), (b, c)).copy_from(&hn.view((a + c, a), (b, c)));
    }
    res.view_mut((a, a), (b, b)).copy_from(&hn.view((a + c, a + c), (b, b)));

    return res;
}