use core::{config::{SETTINGS, SYSTEM}, matrix::{DVMatrix, DVMatrix3, DVMatrix4, DVVector3}, sensor::{FrameSensor, ImuSensor, Sensor}, system::Sender};
use std::{collections::{HashMap, VecDeque}, fmt::Display, thread, time::Duration};
use log::{debug, warn};
use nalgebra::{Matrix3, SMatrix, Vector3, SVD};
use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use crate::{actors::messages::UpdateFrameIMUMsg, map::{map::{Id, Map}, pose::DVTranslation}, registered_actors::IMU, MapLock};
use crate::modules::global_bundle_adjustment::full_inertial_ba;

use crate::map::{frame::Frame, pose::Pose};

use super::{module_definitions::ImuModule, optimizer};

const GRAVITY_VALUE: f64 = 9.81;

#[derive(Debug, Clone)]
pub struct IMU {
    pub velocity: Option<Pose>,
    pub imu_ba1: bool, // mbIMU_BA1
    pub imu_preintegrated_from_last_kf: ImuPreIntegrated, // mpImuPreintegratedFromLastKF
    last_bias: Option<ImuBias>,
    pub rwg: Matrix3<f64>, // mRwg
    pub scale: f64, // mScale
    pub timestamp_init: f64, // mTinit // used by local mapping

    imu_calib: ImuCalib,

    sensor: Sensor
}
impl ImuModule for IMU {
    fn ready(&self, map: &MapLock) -> bool {
        self.sensor.is_imu() && !self.velocity.is_none() && map.read().imu_initialized
    }

    fn predict_state_last_keyframe(&self, map: &MapLock, current_frame: &mut Frame, last_keyframe_id: Id) -> Option<bool> {
        // bool Tracking::PredictStateIMU()
        // Split into two functions: predict_state_last_keyframe predict_state_last_frame
        let lock = map.read();
        let kf = lock.keyframes.get(&last_keyframe_id)?;
        let twb1 = * kf.get_imu_position();
        let rwb1 = * kf.get_imu_rotation();
        let vwb1 = * kf.imu_data.get_velocity();

        let gz = Vector3::new(0.0, 0.0, -GRAVITY_VALUE);
        let t12 = self.imu_preintegrated_from_last_kf.d_t;

        let rwb2 = normalize_rotation(rwb1 * self.imu_preintegrated_from_last_kf.get_delta_rotation(kf.imu_data.imu_bias)?)?;
        let twb2 = twb1 + vwb1*t12 + (0.5*t12*t12*gz) + (rwb1 * self.imu_preintegrated_from_last_kf.get_delta_position(kf.imu_data.imu_bias));
        let vwb2 = vwb1 + t12 * gz + rwb1 * self.imu_preintegrated_from_last_kf.get_delta_velocity(kf.imu_data.imu_bias);
        current_frame.set_imu_pose_velocity(rwb2, twb2, vwb2);

        debug!("SOFIYA IMU, Optimizer, PredictStateIMU, modify frame imubias, mPredBias");

        current_frame.imu_data.imu_bias = kf.imu_data.imu_bias;
        // Predbias is never used anywhere??
        // mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;
        return Some(true);

    }

    fn predict_state_last_frame(&self, current_frame: &mut Frame, last_frame: &mut Frame) -> Option<bool> {
        // bool Tracking::PredictStateIMU()
        let twb1 = * last_frame.get_imu_position();
        let rwb1 = * last_frame.get_imu_rotation();
        let vwb1 = * last_frame.imu_data.get_velocity();
        let gz = Vector3::new(0.0, 0.0, -GRAVITY_VALUE);
        let t12 = current_frame.imu_data.imu_preintegrated_frame.as_ref()?.d_t;

        let rwb2 = normalize_rotation(rwb1 * current_frame.imu_data.imu_preintegrated_frame.as_ref()?.get_delta_rotation(last_frame.imu_data.imu_bias)?)?;
        let twb2 = twb1 + vwb1*t12 + (0.5*t12*t12*gz) + (rwb1 * self.imu_preintegrated_from_last_kf.get_delta_position(last_frame.imu_data.imu_bias));
        let vwb2 = vwb1 + t12 * gz + rwb1 * self.imu_preintegrated_from_last_kf.get_delta_velocity(last_frame.imu_data.imu_bias);
        current_frame.set_imu_pose_velocity(rwb2, twb2, vwb2);

        debug!("SOFIYA IMU, Optimizer, PredictStateIMU, modify frame imubias, mPredBias");

        current_frame.imu_data.imu_bias = last_frame.imu_data.imu_bias;
        // Predbias is never used anywhere??
        // mCurrentFrame.mPredBias = mCurrentFrame.mImuBias;

        return Some(true);
    }

    fn preintegrate(&mut self, map: &MapLock, measurements: &mut ImuMeasurements, current_frame: &mut Frame, previous_frame: &mut Frame) -> bool{
        // void Tracking::PreintegrateIMU()
        // println!("PREINTEGRATE!");
        let _span = tracy_client::span!("IMU preintegration");

        if measurements.is_empty() {
            warn!("No IMU data!");
            return false;
        }

        let mut imu_from_last_frame = VecDeque::with_capacity(measurements.len()); // mvImuFromLastFrame
        let imu_per = 0.000000000001; // 0.001 in orbslam, adjusted here for different timestamp units

        while !measurements.is_empty() {
            let m = measurements.pop_front().unwrap();

            if m.timestamp < previous_frame.timestamp - imu_per {
                measurements.pop_front();
            } else if m.timestamp < current_frame.timestamp - imu_per {
                imu_from_last_frame.push_back(m);
                measurements.pop_front();
            } else {
                imu_from_last_frame.push_back(m);
                break;
            }
        }

        let n = imu_from_last_frame.len() - 1;
        if n == 0 {
            warn!("Empty IMU measurements vector!");
            return false;
        }

        let mut imu_preintegrated_from_last_frame = ImuPreIntegrated::new(previous_frame.imu_data.imu_bias);

        for i in 0..n {
            let mut tstep = 0.0;
            let mut acc: Vector3<f64> = Vector3::zeros(); // acc
            let mut ang_vel: Vector3<f64> = Vector3::zeros(); // angVel
            if i == 0 && i < (n - 1) {
                let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                let tini = imu_from_last_frame[i].timestamp - previous_frame.timestamp;
                acc = {
                    let mut temp = imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc 
                        - (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc);
                    temp.scale_mut(tini / tab);
                    temp * 0.5
                };
                ang_vel = {
                    let mut temp = imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel 
                        - (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel);
                    temp.scale_mut(tini / tab);
                    temp * 0.5
                };
                tstep = imu_from_last_frame[i + 1].timestamp - previous_frame.timestamp;
            } else if i < (n - 1) {
                acc = (imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc) * 0.5;
                ang_vel = (imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel) * 0.5;
                tstep = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
            } else if i > 0 && i == (n - 1) {
                let tab = imu_from_last_frame[i].timestamp - imu_from_last_frame[i - 1].timestamp;
                let tend = current_frame.timestamp - imu_from_last_frame[i].timestamp;
                acc = {
                    let mut temp = imu_from_last_frame[i].acc + imu_from_last_frame[i - 1].acc 
                        - (imu_from_last_frame[i].acc - imu_from_last_frame[i - 1].acc);
                    temp.scale_mut(tend / tab);
                    temp * 0.5
                };
                ang_vel = {
                    let mut temp = imu_from_last_frame[i].ang_vel + imu_from_last_frame[i - 1].ang_vel
                        - (imu_from_last_frame[i].ang_vel - imu_from_last_frame[i - 1].ang_vel);
                    temp.scale_mut(tend / tab);
                    temp * 0.5
                };
            } else if i == 0 && i == (n - 1) {
                acc = imu_from_last_frame[i].acc;
                ang_vel = imu_from_last_frame[i].ang_vel;
                tstep = current_frame.timestamp - previous_frame.timestamp;
            }
            tstep = tstep * 1e9; 

            self.imu_preintegrated_from_last_kf.integrate_new_measurement(acc, ang_vel, tstep);
            imu_preintegrated_from_last_frame.integrate_new_measurement(acc, ang_vel, tstep);
        }

        current_frame.imu_data.imu_preintegrated = Some(imu_preintegrated_from_last_frame);
        current_frame.imu_data.imu_preintegrated_frame = Some(self.imu_preintegrated_from_last_kf.clone());
        // mCurrentFrame.mpLastKeyFrame = mpLastKeyFrame;

        // println!("IMU PREINTEGRATED FOR FRAME {}... {:?}", current_frame.frame_id, current_frame.imu_data.imu_preintegrated.as_ref().unwrap());

        return true;
    }

    fn initialize(&self, map: &mut MapLock, current_keyframe_id: Id, prior_g: f64, prior_a: f64, fiba: bool, tracking_backend: Option<&Sender>) {
        //void LocalMapping::InitializeIMU(float priorG, float priorA, bool bFIBA)
        let _span = tracy_client::span!("IMU initialization");

        println!("Imu initialization: start!");
        let (min_time, min_kf) = match self.sensor.frame() {
            FrameSensor::Mono => (2.0, 10),
            FrameSensor::Stereo | FrameSensor::Rgbd => (1.0, 10),
        };

        if map.read().keyframes.len() < min_kf {
            debug!("IMU initialization: Early return not enough KFs ({})", map.read().keyframes.len());
            return;
        }

        // Retrieve all keyframe in temporal order
        let keyframes = {
            let mut keyframes: VecDeque<Id> = VecDeque::new();
            let map_lock = map.read();
            let mut kf = map_lock.keyframes.get(&current_keyframe_id).unwrap();
            while kf.prev_kf_id.is_some() {
                keyframes.push_front(kf.id);
                kf = map_lock.keyframes.get(&kf.prev_kf_id.unwrap()).unwrap();
                println!("kf {} has prev_kf {:?}", kf.id, kf.prev_kf_id);
            }
            keyframes
        };  // lpKF
        if keyframes.len() < min_kf {
            debug!("IMU initialization: Early return not enough KFs #2 ({})", keyframes.len());
            return;
        }
        println!("Sofiya initialize imu, current keyframe is {}, first keyframe is {}", current_keyframe_id, keyframes[0]);

        let first_timestamp = map.read().keyframes.get(& keyframes[0]).unwrap().timestamp; // mFirstTs
        let current_timestamp = map.read().keyframes.get(&current_keyframe_id).unwrap().timestamp; // mpCurrentKeyFrame->mTimeStamp
        if (current_timestamp - first_timestamp) * 1e9 < min_time {
            debug!("IMU initialization: Early return timestamps ({})... current keyframe is {}, first keyframe is {}", (current_timestamp - first_timestamp) * 1e9, current_keyframe_id, keyframes[0]);
            return;
        }

        // Note: ignoring this because we can't access the local mapping queue in advance
        // Also, even if we could, items in the queue are not yet placed into the map as keyframes
        // and I don't want to do that here
        // bInitializing = true;
        // while(CheckNewKeyFrames())
        // {
        //     ProcessNewKeyFrame();
        //     vpKF.push_back(mpCurrentKeyFrame);
        //     lpKF.push_back(mpCurrentKeyFrame);
        // }

        let num_kfs = keyframes.len(); // N
        let bias = ImuBias::new();

        // Compute and KF velocities mRwg estimation
        let mut rwg; // mRwg
        let mut mbg: DVVector3<f64> = DVVector3::new_with(0.0, 0.0, 0.0); // mbg
        let mut mba: DVVector3<f64> = DVVector3::new_with(0.0, 0.0, 0.0); // mba
        let mut dir_g: Vector3<f64> = Vector3::zeros(); // dirG

        if !map.read().imu_initialized {
            for kf_id in keyframes.iter() {
                let (velocity, prev_kf_id) = {
                    let lock = map.read();
                    let kf = lock.keyframes.get(kf_id).unwrap();
                    let prev_kf = match lock.keyframes.get(&kf.prev_kf_id.unwrap()) {
                        Some(kf) => kf,
                        None => continue
                    };
                    if !kf.imu_data.is_imu_initialized{
                        // Sofiya.. why do we both have is_imu_initialized (bImu) and the option for imu_preintegrated to be None/NULL?
                        continue;
                    }
                    let imu_preintegrated = match kf.imu_data.imu_preintegrated.as_ref() {
                        Some(imu_preintegrated) => imu_preintegrated,
                        None => continue
                    };

                    dir_g = dir_g - (*prev_kf.get_imu_rotation() * imu_preintegrated.get_updated_delta_velocity());

                    let mut temp = *kf.get_imu_position() - *prev_kf.get_imu_position();
                    temp.scale_mut( (1.0 / imu_preintegrated.d_t) as f64);
                    (
                        DVVector3::new(temp),
                        kf.prev_kf_id.unwrap()
                    )
                };
                let mut lock = map.write();
                lock.keyframes.get_mut(kf_id).unwrap().imu_data.velocity = Some(velocity);
                lock.keyframes.get_mut(& prev_kf_id).unwrap().imu_data.velocity = Some(velocity);
            }

            dir_g = dir_g / dir_g.norm();
            let gi = Vector3::new(0.0, 0.0, -1.0);
            let v = gi.cross(&dir_g);
            let nv = v.norm();
            let cosg = gi.dot(&dir_g);
            let ang = cosg.acos();
            let vzg = v * ang / nv;

            // Sofiya... need to make sure this is correct!!!
            rwg = {
                let quat = nalgebra::Quaternion::new(1.0, vzg.x, vzg.y, vzg.z).exp();
                let unit_quat = nalgebra::UnitQuaternion::from_quaternion(quat);
                DVMatrix3::new(* unit_quat.to_rotation_matrix().matrix())
            };
        } else {
            rwg = DVMatrix3::identity();
            mbg = map.read().keyframes.get(&current_keyframe_id).unwrap().imu_data.imu_bias.get_gyro_bias();
            mba = map.read().keyframes.get(&current_keyframe_id).unwrap().imu_data.imu_bias.get_acc_bias();
        }

        println!("Begin inertial optimization for initialization");

        let mut scale = 1.0; // mScale
        let mut info_inertial = SMatrix::<f64, 9, 9>::zeros(); // infoInertial
        optimizer::inertial_optimization_initialization(
            map,
            &mut rwg,
            &mut scale,
            &mut mbg,
            &mut mba,
            self.sensor.is_mono(),
            &mut info_inertial,
            false,
            false,
            prior_g,
            prior_a,
        );

        if scale < 1e-1 {
            warn!("Scale too small");
            debug!("Early return");
            return;
        }

        // Before this line we are not changing the map
        {
            if (scale - 1.0).abs() > 0.00001 || !self.sensor.is_mono() {
                todo!("RGBD, Stereo");
                // Sophus::SE3f Twg(mRwg.cast<float>().transpose(), Eigen::Vector3f::Zero());
                // mpAtlas->GetCurrentMap()->ApplyScaledRotation(Twg, mScale, true);
                // mpTracker->UpdateFrameIMU(mScale, vpKF[0]->GetImuBias(), mpCurrentKeyFrame);
            }

            // Check if initialization OK
            if !map.read().imu_initialized {
                for i in 0..num_kfs {
                    map.write().keyframes.get_mut(&keyframes[i]).unwrap().imu_data.is_imu_initialized = true;
                    println!("Set imu initialized for kf {}", keyframes[i]);
                }
            } else {
                println!("Map already initialized? #1");
            }
        }

        println!("Here?");


        if let Some(tracking_backend) = tracking_backend {
            let lock = map.read();
            let first_kf = lock.keyframes.get(&keyframes[0]).unwrap();
            tracking_backend.send(Box::new(
                UpdateFrameIMUMsg{
                    scale: 1.0,
                    imu_bias: first_kf.imu_data.imu_bias.clone(),
                    current_kf_id: current_keyframe_id,
                    imu_initialized: false,
                }
            )).unwrap();
            // mpTracker->UpdateFrameIMU(1.0,vpKF[0]->GetImuBias(),mpCurrentKeyFrame);
        }

        if !map.read().imu_initialized {
            let mut lock = map.write();
            lock.imu_initialized = true;
            lock.keyframes.get_mut(&current_keyframe_id).unwrap().imu_data.is_imu_initialized = true;
            println!("Set imu initialized for kf {}", current_keyframe_id);
        } else {
            println!("Map already initialized? #1");
        }

        println!("Begin fiba");

        if fiba {
            if prior_a != 0.0 {
                full_inertial_ba(map, 100, false, current_keyframe_id, true, prior_g, prior_a);
            } else {
                full_inertial_ba(map, 100, false, current_keyframe_id, false, 0.0, 0.0);
            }
        }

        // Note... again ignoring this, because we can't access the local mapping queue
        // Process keyframes in the queue
        // while(CheckNewKeyFrames())
        // {
        //     ProcessNewKeyFrame();
        //     vpKF.push_back(mpCurrentKeyFrame);
        //     lpKF.push_back(mpCurrentKeyFrame);
        // }

        {
            let mut lock = map.write();
            // Correct keyframes starting at map first keyframe
            let mut kfs_to_check = vec![lock.initial_kf_id];
            let mut i = 0;
            let mut tcw_bef_gba: HashMap<Id, Pose> = HashMap::new();
            while i < kfs_to_check.len() {
                let curr_kf_id = kfs_to_check[i];
                let children = lock.keyframes.get(& curr_kf_id).unwrap().children.clone();

                let (curr_kf_pose_inverse, curr_kf_gba_pose) = {
                    let curr_kf = lock.keyframes.get(&curr_kf_id).unwrap();
                    (curr_kf.pose.inverse(), curr_kf.gba_pose.clone())
                };

                for child_id in & children {
                    let child = lock.keyframes.get_mut(child_id).unwrap();
                    if child.ba_global_for_kf != current_keyframe_id {
                        let tchildc = child.pose * curr_kf_pose_inverse;
                        child.gba_pose = Some(tchildc * curr_kf_gba_pose.unwrap());
                        todo!("SOFIYA need to set mVwbGBA");
                        // if child.imu_data.velocity.is_some() {
                        //     let rcor = child.gba_pose.unwrap().inverse() * child.pose;
                        //     child.vwb_gba = Some(rcor * child.imu_data.velocity.unwrap());
                        // }
                        child.ba_global_for_kf = current_keyframe_id;
                        child.bias_gba = Some(child.imu_data.imu_bias.clone());
                        println!("Add pose for child kf {}", child_id);
                    }
                    kfs_to_check.push(*child_id);
                }



                let kf = lock.keyframes.get_mut(&curr_kf_id).unwrap();
                tcw_bef_gba.insert(curr_kf_id, kf.pose);
                kf.pose = kf.gba_pose.unwrap().clone();
                println!("Update kf {} with pose {:?}", curr_kf_id, kf.pose);
                i += 1;

                todo!("SOFIYA need to set mVwbGBA");
                // if(pKF->bImu)
                // {
                //     pKF->mVwbBefGBA = pKF->GetVelocity();
                //     pKF->SetVelocity(pKF->mVwbGBA);
                //     pKF->SetNewBias(pKF->mBiasGBA);
                // } else {
                //     cout << "KF " << pKF->mnId << " not set to inertial!! \n";
                // }

            }

            // Correct MapPoints
            let mps_to_update = {
                let mut mps_to_update = HashMap::new();
                for (id, mp) in &lock.mappoints {
                    if mp.ba_global_for_kf == current_keyframe_id {
                        // If optimized by Global BA, just update
                        mps_to_update.insert(*id, mp.gba_pose.unwrap());
                    } else {
                        // Update according to the correction of its reference keyframe
                        let ref_kf = lock.keyframes.get(&mp.ref_kf_id).unwrap();

                        if ref_kf.ba_global_for_kf != current_keyframe_id {
                            continue;
                        }

                        // Map to non-corrected camera
                        let tcw_bef_gba_for_ref_kf = tcw_bef_gba.get(&mp.ref_kf_id).unwrap();
                        let rcw = tcw_bef_gba_for_ref_kf.get_rotation();
                        let tcw = tcw_bef_gba_for_ref_kf.get_translation();
                        let xc = *rcw * *mp.position + *tcw;


                        // Backproject using corrected camera
                        let twc = ref_kf.pose.inverse();
                        let rwc = twc.get_rotation();
                        let twc = twc.get_translation();

                        mps_to_update.insert(*id, DVTranslation::new(*rwc * xc + *twc));
                    }
                }
                mps_to_update
            };


            for (mp_id, gba_pose) in mps_to_update {
                lock.mappoints.get_mut(&mp_id).unwrap().position = gba_pose;
            }
        }
        println!("IMU INITIALIZATION SUCCESSFUL!!!!!!!!!!!");
        map.write().map_change_index += 1;
    }
}

impl IMU {
    pub fn new() -> Self {
        Self {
            velocity: None,
            imu_ba1: false,
            last_bias: None,
            imu_calib: ImuCalib::new(),
            sensor: SETTINGS.get::<Sensor>(SYSTEM, "sensor"),
            imu_preintegrated_from_last_kf: ImuPreIntegrated::new(ImuBias::new()),
            rwg: Matrix3::identity(),
            scale: 1.0,
            timestamp_init: 0.0,
        }
    }
}


pub type ImuMeasurements =  VecDeque<ImuPoint>;

pub struct ImuPoint {
    pub acc: Vector3<f64>,  // a
    pub ang_vel: Vector3<f64>, // w
    pub timestamp: f64
}
impl Debug for ImuPoint {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ImuPoint")
            .field("acceleration", &self.acc)
            .field("ang_vel", &self.ang_vel)
            .field("timestamp", &self.timestamp)
            .finish()
    }
}

#[derive(Debug, Clone)]
pub struct ImuCalib {
    pub tcb: Pose,
    pub tbc: Pose,
    cov: SMatrix<f64, 6, 6>,
    cov_walk: SMatrix<f64, 6, 6>,
    is_set: bool,
}
impl ImuCalib {
    pub fn new() -> Self{
        let imu_frequency = SETTINGS.get::<f64>(IMU, "frequency");
        let na = SETTINGS.get::<f64>(IMU, "noise_acc");
        let ngw = SETTINGS.get::<f64>(IMU, "gyro_walk");
        let naw = SETTINGS.get::<f64>(IMU, "acc_walk");
        let ng = SETTINGS.get::<f64>(IMU, "noise_gyro");
        let sf = imu_frequency.sqrt();
        let tbc = {
            let tbc = SETTINGS.get::<DVMatrix4<f64>>(IMU, "T_b_c1");
            let rot = Matrix3::new(
                tbc[(0, 0)], tbc[(0, 1)], tbc[(0, 2)],
                tbc[(1, 0)], tbc[(1, 1)], tbc[(1, 2)],
                tbc[(2, 0)], tbc[(2, 1)], tbc[(2, 2)]
            );
            let trans = Vector3::new(tbc[(0, 3)], tbc[(1, 3)], tbc[(2, 3)]);
            Pose::new(trans, rot)
        };

        ImuCalib::new_internal(tbc,ng * sf,na * sf, ngw / sf, naw / sf)
    }

    fn new_internal(tbc: Pose, ng: f64, na: f64, ngw: f64, naw: f64) -> Self {
        // void Calib::Set(const Sophus::SE3<float> &sophTbc, const float &ng, const float &na, const float &ngw, const float &naw) {

        let ng2 = ng * ng;
        let na2 = na * na;
        let ngw2 = ngw * ngw;
        let naw2 = naw * naw;

        let cov: SMatrix<f64, 6, 6> = SMatrix::from_row_slice(&[
            ng2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, ng2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, ng2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, na2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, na2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, na2,
        ]);

        let cov_walk: SMatrix<f64, 6, 6> = SMatrix::from_row_slice(&[
            ngw2, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, ngw2, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, ngw2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, naw2, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, naw2, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, naw2,
        ]);

        Self {
            tcb: tbc.inverse(),
            tbc,
            cov,
            cov_walk,
            is_set: true
        }
    }
}


#[derive(Debug, Clone)]
struct Integrable {
    a: Vector3<f64>, // acceleration
    w: Vector3<f64>, // angular velocity
    t: f64
}

#[derive(Debug, Clone)]
pub struct ImuPreIntegrated {
    d_t: f64, // dT
    c: SMatrix<f64, 15, 15>, // Eigen::Matrix<float,15,15> C;
    info: SMatrix<f64, 15, 15>, // Eigen::Matrix<float,15,15> Info;
    nga_walk: SMatrix<f64, 6, 6>, // Eigen::DiagonalMatrix<float,6> Nga, NgaWalk
    nga: SMatrix<f64, 6, 6>, // Eigen::DiagonalMatrix<float,6> Nga, NgaWalk

    // Values for the original bias (when integration was computed)
    b: ImuBias,

    d_r: Matrix3<f64>, // Eigen::Matrix3f dR;
    d_v: Vector3<f64>, // Eigen::Vector3f dV;
    d_p: Vector3<f64>, // Eigen::Vector3f dP;
    pub jrg: Matrix3<f64>, // Eigen::Matrix3f JRg;
    jvg: Matrix3<f64>, // Eigen::Matrix3f JVg;
    jva: Matrix3<f64>, // Eigen::Matrix3f JVa;
    jpg: Matrix3<f64>, // Eigen::Matrix3f JPg;
    jpa: Matrix3<f64>, // Eigen::Matrix3f JPa;
    avg_a: Vector3<f64>, // Eigen::Vector3f avgA;
    avg_w: Vector3<f64>, // Eigen::Vector3f avgW;

    // Updated bias
    bu: ImuBias,
    // Dif between original and updated bias
    // This is used to compute the updated values of the preintegration
    d_b: SMatrix<f64, 6, 1>, //  Eigen::Matrix<float,6,1> db;

    measurements: Vec<Integrable>, // mvMeasurements
}
impl ImuPreIntegrated {
    pub fn new(bias: ImuBias) -> Self {
        // 2nd argument in orbslam3, ImuCalib, is constant and read from calibraiton file

        let calib = ImuCalib::new();
        Self {
            nga: calib.cov,
            nga_walk: calib.cov_walk,
            b: bias,
            bu: bias,
            measurements: vec![],
            d_t: 0.0,
            c: SMatrix::<f64, 15, 15>::zeros(),
            info: SMatrix::<f64, 15, 15>::zeros(),
            d_r: Matrix3::identity(),
            d_v: Vector3::<f64>::zeros(),
            d_p: Vector3::<f64>::zeros(),
            jrg: Matrix3::<f64>::zeros(),
            jvg: Matrix3::<f64>::zeros(),
            jva: Matrix3::<f64>::zeros(),
            jpg: Matrix3::<f64>::zeros(),
            jpa: Matrix3::<f64>::zeros(),
            avg_a: Vector3::<f64>::zeros(),
            avg_w: Vector3::<f64>::zeros(),
            d_b: SMatrix::<f64, 6, 1>::zeros(),
        }
    }

    pub fn initialize(&mut self) {
        self.d_v = Vector3::zeros();
        self.d_p = Vector3::zeros();
        self.jrg = Matrix3::zeros();
        self.jvg = Matrix3::zeros();
        self.jva = Matrix3::zeros();
        self.jpg = Matrix3::zeros();
        self.jpa = Matrix3::zeros();
        self.c = SMatrix::<f64, 15, 15>::zeros();
        self.info = SMatrix::<f64, 15, 15>::zeros();
        self.d_b = SMatrix::<f64, 6, 1>::zeros();
        self.b = self.bu;
        self.avg_a = Vector3::zeros();
        self.avg_w = Vector3::zeros();
        self.d_t = 0.0;
        self.measurements.clear();
    }

    pub fn reintegrate(&mut self) {
        // Preintegrated::Reintegrate
        self.initialize();
        for i in 0..self.measurements.len() {
            self.integrate_new_measurement(self.measurements[i].a, self.measurements[i].w, self.measurements[i].t);
        }
    }

    pub fn integrate_new_measurement(&mut self, acceleration: Vector3<f64>, ang_vel: Vector3<f64>, dt: f64) {
        // void Preintegrated::IntegrateNewMeasurement(const Eigen::Vector3f &acceleration, const Eigen::Vector3f &angVel, const float &dt)
        self.measurements = vec![Integrable{a: acceleration, w: ang_vel, t: dt}];

        // Position is updated firstly, as it depends on previously computed velocity and rotation.
        // Velocity is updated secondly, as it depends on previously computed rotation.
        // Rotation is the last to be updated.

        //Matrices to compute covariance
        let mut cov_a = SMatrix::<f64, 9, 9>::identity(); // A
        let mut cov_b = SMatrix::<f64, 9, 6>::zeros(); // B

        let acc: Vector3<f64> = acceleration - Vector3::new(self.b.bax, self.b.bay, self.b.baz);
        let acc_w: Vector3<f64> = ang_vel - Vector3::new(self.b.bwx, self.b.bwy, self.b.bwz);

        self.avg_a = (self.d_t * self.avg_a + self.d_r * acc * dt) / (self.d_t + dt);
        self.avg_w = (self.d_t * self.avg_w + acc_w * dt) / (self.d_t + dt);

        // Update delta position dP and velocity dV (rely on no-updated delta rotation)
        self.d_p = self.d_p + self.d_v * dt + 0.5 * self.d_r * acc * dt * dt;
        self.d_v = self.d_v + self.d_r * acc * dt;

        // Compute velocity and position parts of matrices A and B (rely on non-updated delta rotation)
        let wacc = hat(& acc);
        cov_a.fixed_view_mut::<3, 3>(3, 0).copy_from(&(-self.d_r * dt * wacc));
        cov_a.fixed_view_mut::<3, 3>(6, 0).copy_from(&(-0.5 * self.d_r * dt * dt * wacc));
        cov_a.fixed_view_mut::<3, 3>(6, 3).copy_from(
            &SMatrix::from_row_slice(&[
                dt, 0.0, 0.0,
                0.0, dt, 0.0,
                0.0, 0.0, dt,
            ]
        ));
        cov_b.fixed_view_mut::<3, 3>(3, 3).copy_from(&(self.d_r * dt));
        cov_b.fixed_view_mut::<3, 3>(6, 3).copy_from(&(self.d_r.scale(0.5) * dt * dt));

        // Update position and velocity jacobians wrt bias correction
        self.jpa = self.jpa + self.jva * dt - 0.5 * self.d_r * dt * dt;
        self.jpg = self.jpg + self.jvg * dt - 0.5 * self.d_r * dt * dt * wacc * self.jrg;
        self.jva = self.jva - self.d_r * dt;
        self.jvg = self.jvg - self.d_r * dt * wacc * self.jrg;
    
        // Update delta rotation
        let d_ri = IntegratedRotation::new(ang_vel, self.b, dt);

        // println!("input: {:?}", self.d_r * d_ri.delta_r);

        self.d_r = normalize_rotation(self.d_r * d_ri.delta_r).unwrap();

        // println!("self.d_r normalized: {:?}", self.d_r);

        // Compute rotation parts of matrices A and B
        cov_a.fixed_view_mut::<3, 3>(0, 0).copy_from(&d_ri.delta_r.transpose());
        cov_b.fixed_view_mut::<3, 3>(0, 0).copy_from(&(d_ri.right_j * dt));

        // Update covariance
        {
            let temp = cov_a * self.c.fixed_view::<9, 9>(0, 0) * cov_a.transpose() + cov_b * self.nga * cov_b.transpose();
            self.c.fixed_view_mut::<9, 9>(0, 0).copy_from(&(temp));
        }
        {
            let temp = self.c.fixed_view::<6, 6>(9, 9) + self.nga_walk;
            self.c.fixed_view_mut::<6, 6>(9, 9).copy_from(& temp);
        }

        // Update rotation jacobian wrt bias correction
        self.jrg = d_ri.delta_r.transpose() * self.jrg - d_ri.right_j * dt;

        // Total integrated time
        self.d_t += dt;
    }

    pub fn merge_previous(&mut self, prev: &ImuPreIntegrated) {
        // void Preintegrated::MergePrevious(Preintegrated* pPrev)
        let mut bav = ImuBias::new();
        bav.bwx = self.bu.bwx;
        bav.bwy = self.bu.bwy;
        bav.bwz = self.bu.bwz;
        bav.bax = self.bu.bax;
        bav.bay = self.bu.bay;
        bav.baz = self.bu.baz;

        self.initialize();
        for i in 0..prev.measurements.len() {
            self.integrate_new_measurement(prev.measurements[i].a, prev.measurements[i].w, prev.measurements[i].t);
        }
        for i in 0..self.measurements.len() {
            self.integrate_new_measurement(self.measurements[i].a, self.measurements[i].w, self.measurements[i].t);
        }
    }

    pub fn set_new_bias(&mut self, new_bias: ImuBias) {
        // void Preintegrated::SetNewBias(const Bias &bu_)

        self.d_b[0] = new_bias.bwx - self.b.bwx;
        self.d_b[1] = new_bias.bwy - self.b.bwy;
        self.d_b[2] = new_bias.bwz - self.b.bwz;
        self.d_b[3] = new_bias.bax - self.b.bax;
        self.d_b[4] = new_bias.bay - self.b.bay;
        self.d_b[5] = new_bias.baz - self.b.baz;
    }

    pub fn get_delta_bias(&self) {
        // Eigen::Matrix<float,6,1> Preintegrated::GetDeltaBias(), IMU::Bias Preintegrated::GetDeltaBias(const Bias &b_)
        // Not used???
    }

    pub fn get_delta_rotation(&self, b_: ImuBias) -> Option<Matrix3<f64>> {
        // Preintegrated::GetDeltaRotation
        todo!("IMU");
        // let dbg = Vector3::new(b_.bwx - self.b.bwx, b_.bwy - self.b.bwy, b_.bwz - self.b.bwz);
        // normalize_rotation(self.d_r * (self.jrg  * dbg).exp())
        // return NormalizeRotation(dR * Sophus::SO3f::exp(JRg * dbg).matrix());
    }
    pub fn get_delta_velocity(&self, b_: ImuBias) -> Vector3<f64> {
        // Preintegrated::GetDeltaVelocity
        let dbg = Vector3::new(b_.bwx - self.b.bwx, b_.bwy - self.b.bwy, b_.bwz - self.b.bwz);
        let dba = Vector3::new(b_.bax - self.b.bax, b_.bay - self.b.bay, b_.baz - self.b.baz);
        self.d_v + self.jvg * dbg + self.jva * dba
    }
    pub fn get_delta_position(&self, b_: ImuBias) -> Vector3<f64> {
        // Preintegrated::GetDeltaPosition
        let dbg = Vector3::new(b_.bwx - self.b.bwx, b_.bwy - self.b.bwy, b_.bwz - self.b.bwz);
        let dba = Vector3::new(b_.bax - self.b.bax, b_.bay - self.b.bay, b_.baz - self.b.baz);
        self.d_p + self.jpg * dbg + self.jpa * dba
    }
    pub fn get_updated_delta_rotation(&self) -> Option<Matrix3<f64>> {
        // Preintegrated::GetUpdatedDeltaRotation
        todo!("IMU");
        // let part2 = (self.jrg * self.d_b.fixed_rows::<3>(0)).exp();
        // normalize_rotation(self.d_r * part2)

        // return NormalizeRotation(dR * Sophus::SO3f::exp(JRg*db.head(3)).matrix());
    }
    pub fn get_updated_delta_velocity(&self) -> Vector3<f64> {
        // Preintegrated::GetUpdatedDeltaVelocity
        // dV + JVg * db.head(3) + JVa * db.tail(3);
        self.d_v + self.jvg * self.d_b.fixed_rows::<3>(0) + self.jva * self.d_b.fixed_rows::<3>(3)
    }
    pub fn get_updated_delta_position(&self) -> Vector3<f64> {
        // Preintegrated::GetUpdatedDeltaPosition
        // return dP + JPg*db.head(3) + JPa*db.tail(3);
        self.d_p + self.jpg * self.d_b.fixed_rows::<3>(0) + self.jpa * self.d_b.fixed_rows::<3>(3)
    }
    pub fn get_original_delta_rotation(&self) -> Matrix3<f64>{
        // Preintegrated::GetOriginalDeltaRotation
        self.d_r
    }
    pub fn get_original_delta_velocity(&self) -> Vector3<f64> {
        // Preintegrated::GetOriginalDeltaVelocity
        self.d_v
    }
    pub fn get_original_delta_position(&self) -> Vector3<f64> {
        // Preintegrated::GetOriginalDeltaPosition
        self.d_p
    }
    pub fn get_original_bias(&self) -> ImuBias {
        // Preintegrated::GetOriginalBias
        self.b
    }
    pub fn get_updated_bias(&self) -> ImuBias {
        // Preintegrated::GetUpdatedBias
        self.bu
    }
}
impl Into<g2o::ffi::RustImuPreintegrated> for & ImuPreIntegrated {
    fn into(self) -> g2o::ffi::RustImuPreintegrated {
        fn matrix_into_vec(mat: Matrix3<f64>) -> [[f32; 3]; 3] {
            [
                [mat[(0, 0)] as f32, mat[(0, 1)] as f32, mat[(0, 2)] as f32],
                [mat[(1, 0)] as f32, mat[(1, 1)] as f32, mat[(1, 2)] as f32],
                [mat[(2, 0)] as f32, mat[(2, 1)] as f32, mat[(2, 2)] as f32],
            ]
        }
        // Only needs top-left [9,9] block
        // let c: [[f64; 15]; 15] = [
        //     [self.c[(0,0)], self.c[(0,1)], self.c[(0,2)], self.c[(0,3)], self.c[(0,4)], self.c[(0,5)], self.c[(0,6)], self.c[(0,7)], self.c[(0,8)], self.c[(0,9)]],
        //     [self.c[(1,0)], self.c[(1,1)], self.c[(1,2)], self.c[(1,3)], self.c[(1,4)], self.c[(1,5)], self.c[(1,6)], self.c[(1,7)], self.c[(1,8)], self.c[(1,9)]],
        //     [self.c[(2,0)], self.c[(2,1)], self.c[(2,2)], self.c[(2,3)], self.c[(2,4)], self.c[(2,5)], self.c[(2,6)], self.c[(2,7)], self.c[(2,8)], self.c[(2,9)]],
        //     [self.c[(3,0)], self.c[(3,1)], self.c[(3,2)], self.c[(3,3)], self.c[(3,4)], self.c[(3,5)], self.c[(3,6)], self.c[(3,7)], self.c[(3,8)], self.c[(3,9)]],
        //     [self.c[(4,0)], self.c[(4,1)], self.c[(4,2)], self.c[(4,3)], self.c[(4,4)], self.c[(4,5)], self.c[(4,6)], self.c[(4,7)], self.c[(4,8)], self.c[(4,9)]],
        //     [self.c[(5,0)], self.c[(5,1)], self.c[(5,2)], self.c[(5,3)], self.c[(5,4)], self.c[(5,5)], self.c[(5,6)], self.c[(5,7)], self.c[(5,8)], self.c[(5,9)]],
        //     [self.c[(6,0)], self.c[(6,1)], self.c[(6,2)], self.c[(6,3)], self.c[(6,4)], self.c[(6,5)], self.c[(6,6)], self.c[(6,7)], self.c[(6,8)], self.c[(6,9)]],
        //     [self.c[(7,0)], self.c[(7,1)], self.c[(7,2)], self.c[(7,3)], self.c[(7,4)], self.c[(7,5)], self.c[(7,6)], self.c[(7,7)], self.c[(7,8)], self.c[(7,9)]],
        //     [self.c[(8,0)], self.c[(8,1)], self.c[(8,2)], self.c[(8,3)], self.c[(8,4)], self.c[(8,5)], self.c[(8,6)], self.c[(8,7)], self.c[(8,8)], self.c[(8,9)]],
        //     [self.c[(9,0)], self.c[(9,1)], self.c[(9,2)], self.c[(9,3)], self.c[(9,4)], self.c[(9,5)], self.c[(9,6)], self.c[(9,7)], self.c[(9,8)], self.c[(9,9)]],
        // ];
        g2o::ffi::RustImuPreintegrated {
            jrg: matrix_into_vec(self.jrg),
            jvg: matrix_into_vec(self.jvg),
            jpg: matrix_into_vec(self.jpg),
            jva: matrix_into_vec(self.jva),
            jpa: matrix_into_vec(self.jpa),
            db: [self.d_b[0] as f32, self.d_b[1] as f32, self.d_b[2] as f32, self.d_b[3] as f32, self.d_b[4] as f32, self.d_b[5] as f32],
            dv: [self.d_v[0] as f32, self.d_v[1] as f32, self.d_v[2] as f32],
            bias: self.b.into(),
            t: self.d_t,
        }
    }
}


#[derive(Clone, Debug, Copy, Serialize, Deserialize)]
pub struct ImuBias {
    pub bax: f64,
    pub bay: f64,
    pub baz: f64,
    pub bwx: f64,
    pub bwy: f64,
    pub bwz: f64,
}
impl ImuBias {
    pub fn new() -> Self {
        Self {
            bax: 0.0,
            bay: 0.0,
            baz: 0.0,
            bwx: 0.0,
            bwy: 0.0,
            bwz: 0.0,
        }
    }
    pub fn get_gyro_bias(&self) -> DVVector3<f64> {
        DVVector3::new_with(self.bwx, self.bwy, self.bwz)
    }
    pub fn get_acc_bias(&self) -> DVVector3<f64> {
        DVVector3::new_with(self.bax, self.bay, self.baz)
    }
}
impl Display for ImuBias {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}, {}, {}, {}, {}, {}", self.bax, self.bay, self.baz, self.bwx, self.bwy, self.bwz)
    }
}
impl Into<g2o::ffi::RustImuBias> for ImuBias {
    fn into(self) -> g2o::ffi::RustImuBias {
        g2o::ffi::RustImuBias {
            b_acc_x: self.bax as f32,
            b_acc_y: self.bay as f32,
            b_acc_z: self.baz as f32,
            b_ang_vel_x: self.bwx as f32,
            b_ang_vel_y: self.bwy as f32,
            b_ang_vel_z: self.bwz as f32,
        }
    }
}

pub struct IntegratedRotation {
    pub delta_t: f64, // deltaT // integration time
    pub delta_r: Matrix3<f64>, // deltaR
    pub right_j: Matrix3<f64>, // rightJ // right jacobian
}
impl IntegratedRotation {
    pub fn new(ang_vel: nalgebra::Vector3<f64>, imu_bias: ImuBias, time: f64) -> Self {
        let x = (ang_vel[0] - imu_bias.bwx) * time;
        let y = (ang_vel[1] - imu_bias.bwy) * time;
        let z = (ang_vel[2] - imu_bias.bwz) * time;

        let d2 = x*x + y*y + z*z;
        let d = d2.sqrt();

        let v = Vector3::new(x, y, z);
        let w = hat(&v);
        if d < 1e-4 {
            Self {
                delta_t: time,
                delta_r: Matrix3::identity() + w,
                right_j: Matrix3::identity()
            }
        } else {
            Self {
                delta_t: time,
                delta_r: Matrix3::identity() + w * (d.sin() / d) + w * w * ((1.0 - d.cos()) / d2),
                right_j: Matrix3::identity() - w * ((1.0 - d.cos()) / d2) + w * w * ((d - d.sin()) / (d2 * d))
            }
        }
    }
}

pub fn normalize_rotation(mat: Matrix3<f64>) ->  Option<Matrix3<f64>> {
    // Eigen::Matrix3f NormalizeRotation(const Eigen::Matrix3f &R)
    let svd = SVD::new(mat, true, true);
    // println!("u : {:?}", svd.u?);
    // println!("v_t : {:?}", svd.v_t?);
    Some(svd.u? * svd.v_t?.transpose())
}

pub fn hat(vec: & Vector3<f64>) -> Matrix3<f64> {
    Matrix3::from_row_slice(&[
        0.0, -vec[2], vec[1],
        vec[2], 0.0, -vec[0],
        -vec[1], vec[0], 0.0
    ])

    // Should be equal to:
    // Scalar(0), -omega(2),  omega(1),
    //  omega(2), Scalar(0), -omega(0),
    // -omega(1),  omega(0), Scalar(0);
}

#[derive(Debug, Clone)]
pub struct ImuDataFrame {
    // IMU //
    // Preintegrated IMU measurements from previous keyframe
    pub is_imu_initialized: bool, // bImu
    pub imu_bias: ImuBias,
    pub imu_preintegrated: Option<ImuPreIntegrated>,  // mpImuPreintegrated
    pub imu_preintegrated_frame: Option<ImuPreIntegrated>, // mpImuPreintegratedFrame
    pub velocity: Option<DVVector3<f64>>, // mVw
}

impl ImuDataFrame {
    pub fn new() -> Self {
        Self {
            is_imu_initialized: false,
            imu_bias: ImuBias::new(),
            imu_preintegrated: None,
            imu_preintegrated_frame: None,
            velocity: None,
        }
    }
    pub fn set_new_bias(&mut self, bias: ImuBias) {
        // void Frame::SetNewBias(const IMU::Bias &b)
        self.imu_bias = bias;
        if let Some(imu_preintegrated) = &mut self.imu_preintegrated {
            imu_preintegrated.set_new_bias(bias);
        }
    }

    pub fn get_velocity(&self) -> DVVector3<f64> {
        // Eigen::Vector3f KeyFrame::GetVelocity()
        return self.velocity.expect("Velocity should be set by now");
    }
}