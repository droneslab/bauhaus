extern crate g2o;
use log::{warn, info, debug};
use nalgebra::{Isometry3, Vector3, Vector6};
use opencv::core::Mat;
use std::{collections::{BTreeSet, HashMap}, fmt::Debug, thread::sleep, time::Duration};
use core::{
    config::*, matrix::*, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{messages::{FeatureTracksAndIMUMsg, ShutdownMsg, TrajectoryMsg, UpdateFrameIMUMsg, VisTrajectoryMsg}, tracking_frontend_gtsam::{GtsamFrontendTrackingState, TrackedFeatures}},
    map::{frame::Frame, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{image::draw_optical_flow, imu::{ImuBias, ImuCalib}}, registered_actors::{CAMERA, IMU, SHUTDOWN_ACTOR, TRACKING_BACKEND, VISUALIZER}, ImuInitializationData
};
use gtsam::{geometry::{cal3_s2::Cal3S2, point2::Point2, point3::Point3, pose3::Pose3, rot3::Rot3}, imu::imu_bias::ConstantBias, inference::symbol::Symbol, linear::noise_model::{DiagonalNoiseModel, GaussianNoiseModel, IsotropicNoiseModel}, navigation::combined_imu_factor::{CombinedImuFactor, PreintegratedCombinedMeasurements}, nonlinear::{incremental_fixed_lag_smoother::IncrementalFixedLagSmoother, isam2::ISAM2, levenberg_marquardt_optimizer::LevenbergMarquardtOptimizer, levenberg_marquardt_params::LevenbergMarquardtParams, nonlinear_factor_graph::NonlinearFactorGraph, values::Values}, slam::projection_factor::{GenericProjectionFactorPose3Point3Cal3S2, SmartProjectionPoseFactorCal3S2}, sys::{Key, ffi::DoubleVec}};

type GTSAMVector3 = gtsam::base::vector::Vector3; // Also importing a vector3 from nalgebra

pub struct TrackingBackendGTSAM {
    system: System,
    map: ReadWriteMap,

    last_timestamp: Timestamp,

    // For drawing
    last_image: Option<Mat>,
    curr_frame_id: i32,

    // Modules 
    graph_solver: GraphSolver,

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses

    kf_count: i32,
}

impl Actor for TrackingBackendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let optimizer_type = SETTINGS.get::<i32>(TRACKING_BACKEND, "optimizer_type");
        let use_smart_factors = SETTINGS.get::<bool>(TRACKING_BACKEND, "use_smart_factors");

        let mut actor = TrackingBackendGTSAM {
            system,
            graph_solver: GraphSolver::new(optimizer_type, use_smart_factors),
            map,
            trajectory_poses: Vec::new(),
            last_timestamp: 0.0,
            last_image: None,
            curr_frame_id: 0,
            kf_count: 0,
        };
        tracy_client::set_thread_name!("tracking backend gtsam");

        loop {
            let message = actor.system.receive().unwrap();
            if actor.handle_message(message) {
                break;
            }
            actor.map.match_map_version();
        }
    }

}

impl TrackingBackendGTSAM {
    fn handle_message(&mut self, message: MessageBox) -> bool {
        if message.is::<FeatureTracksAndIMUMsg>() {
            if self.system.queue_full() {
                // Abort additional work if there are too many frames in the msg queue.
                info!("Tracking gtsam dropped 1 frame");
                return false;
            }

            let msg = message.downcast::<FeatureTracksAndIMUMsg>().unwrap_or_else(|_| panic!("Could not downcast tracking message!"));
            self.handle_regular_message(*msg).unwrap();
        } else if message.is::<UpdateFrameIMUMsg>() {
            warn!("I think it may be ok to ignore this because the bias is always set to the ref kf, and local mapping should have done that already");

        } else if message.is::<ShutdownMsg>() {
            // Sleep a little to allow other threads to finish
            sleep(Duration::from_millis(100));
            return true;
        } else {
            warn!("Tracking backend received unknown message type!");
        }
        return false;
    }

    fn handle_regular_message(&mut self, msg: FeatureTracksAndIMUMsg) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("track");

        let current_frame = if matches!(self.graph_solver.solver_state, GraphSolverState::NotInitialized) {
            let imu_init = msg.imu_initialization.expect("Msg should have imu initialization data!");

            debug!("Initializing!");
            let mut current_frame = msg.frame;
            self.graph_solver.initialize(
                current_frame.timestamp,
                &imu_init,
            ).expect("Failed to initialize?");

            // Update frame and create new keyframe
            current_frame.set_imu_pose_velocity(
                imu_init.pose,
                *imu_init.velocity
            );
            current_frame.imu_data.set_new_bias(imu_init.bias);
            let _new_kf_id = self.create_new_keyframe(&mut current_frame).expect("Could not create new keyframe");

            let map = self.map.read()?;
            self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
                pose: imu_init.pose,
                mappoint_matches: vec![],
                mappoints_in_tracking: BTreeSet::new(),
                timestamp: current_frame.timestamp,
                map_version: map.version
            }));

            debug!("Initializing done! First keyframe's timestamp: {}", current_frame.timestamp);

            self.kf_count = 0;

            current_frame
        } else {
            // If we have previous frames already, can track normally
            let mut current_frame = msg.frame;
            println!("Current timestamp: {}", current_frame.timestamp);

            // Solve VIO graph. Includes preintegration
            // Option to batch update instead of update each time. Not sure this works correctly
            let should_update = match self.graph_solver.optimizer {
                Optimizer::ISAM2 {..} => SETTINGS.get::<i32>(TRACKING_BACKEND, "isam_update_interval") == self.kf_count,
                Optimizer::IncrementalFixedLagSmoother {..} => true,
                Optimizer::LevenbergMarquadt { } => true,
            };
            let optimization_results = self.graph_solver.solve(
                msg.tracker_status,
                &mut current_frame,
                & msg.preintegration,
                &msg.feature_tracks,
                should_update
            )?;

            let _new_kf_id = self.create_new_keyframe(&mut current_frame).expect("Could not create new keyframe");

            // If using isam, we need to update all keyframes' poses because they are optimized each time
            self.kf_count = if should_update { 0 } else { self.kf_count + 1 };

            current_frame
        };

        self.last_timestamp = current_frame.timestamp;
        self.update_trajectory_in_logs(& current_frame).expect("Could not save trajectory");
        self.curr_frame_id += 1;

        return Ok(())
    }

    fn update_trajectory_in_logs(
        &mut self, current_frame: &Frame
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.trajectory_poses.push(current_frame.pose.unwrap());

        self.system.send(
            SHUTDOWN_ACTOR, 
            Box::new(TrajectoryMsg{
                pose: current_frame.pose.unwrap(),
                ref_kf_id: current_frame.ref_kf_id.unwrap(),
                timestamp: current_frame.timestamp,
                map_version: self.map.read()?.version
            })
        );

        let map = self.map.read()?;
        self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
            pose: current_frame.pose.unwrap(),
            mappoint_matches: vec![],
            mappoints_in_tracking: BTreeSet::new(),
            timestamp: current_frame.timestamp,
            map_version: map.version,
        }));

        Ok(())
    }

    fn create_new_keyframe(&mut self, current_frame: &mut Frame) -> Result<Id, Box<dyn std::error::Error>> {
        // Calling this so global map has this keyframe saved.
        // Global map not used by Kimera pipeline, but needed for visualization.
        let _span = tracy_client::span!("create_new_keyframe");
        let new_frame = Frame::new_clone(& current_frame);
        let new_kf_id = self.map.write()?.insert_keyframe_to_map(new_frame, false);

        current_frame.ref_kf_id = Some(new_kf_id);

        tracy_client::Client::running()
        .expect("message! without a running Client")
        .message("create new keyframe", 2);

        Ok(new_kf_id)
    }
}

enum Optimizer {
    ISAM2 {
        isam2: ISAM2
    },
    IncrementalFixedLagSmoother {
        smoother: IncrementalFixedLagSmoother
    },
    LevenbergMarquadt { }
}


pub struct GraphSolver {
    solver_state: GraphSolverState,
    use_smart_factors: bool,

    optimizer: Optimizer,

    // New graph -- new factors and values that have not been optimized yet
    graph_new: NonlinearFactorGraph,
    values_new: ValuesWrapper,

    // Main graph
    values_all: ValuesWrapper, // All created nodes
    inserted_kfs: Vec<u64>, // Keyframe IDs that have been inserted into the graph

    curr_id: u64, // Current state ID
    last_timestamp: Timestamp,

    // Managing smart factors
    smartfactors: HashMap<i32, (gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2, bool, i32)>, // Smartfactor lookup. feature ID -> (smart factor object, whether it is in the graph or not, number of observations)
    smartfactor_idx_in_isam2: HashMap<i32, u64>, // (smart factor ID in this object) -> (index in isam2)

    // Initialization
    accel_noise_density: f64, // accelerometer_noise_density, sigma_a
    gyro_noise_density: f64, // gyroscope_noise_density, sigma_g
    accel_random_walk: f64, // accelerometer_random_walk, sigma_wa
    gyro_random_walk: f64, // gyroscope_random_walk, sigma_wg
    sampling_frequency: f64, // sqrt(imu frequency)
    initial_position_sigma: f64,
    initial_roll_pitch_sigma: f64,
    initial_yaw_sigma: f64,
    initial_velocity_sigma: f64,
    initial_acc_bias_sigma: f64,
    initial_gyro_bias_sigma: f64,
    imu_integration_sigma: f64,
    init_bias_sigma: f64,
    k: Cal3S2, // Camera intrinsics
    vision_measurement_noise: IsotropicNoiseModel, // Camera measurement noise model
    tbc: Pose, // Transform from camera frame to body (IMU) 
}

impl GraphSolver {
    pub fn new(optimizer_type: i32, use_smart_factors: bool) -> Self {
        let vision_measurement_noise = IsotropicNoiseModel::from_dim_and_sigma(3, 3.0);
        let k = Cal3S2::new(
            SETTINGS.get::<f64>(CAMERA, "fx"),
            SETTINGS.get::<f64>(CAMERA, "fy"),
            0.0,
            SETTINGS.get::<f64>(CAMERA, "cx"),
            SETTINGS.get::<f64>(CAMERA, "cy"),
        );

        let optimizer = match optimizer_type {
            0 => Optimizer::ISAM2 {
                isam2: ISAM2::new(
                    SETTINGS.get::<f64>(TRACKING_BACKEND, "isam_relinearize_threshold"),
                    SETTINGS.get::<i32>(TRACKING_BACKEND, "isam_relinearize_skip"),
                    SETTINGS.get::<bool>(TRACKING_BACKEND, "isam_cache_linearized_factors"),
                    SETTINGS.get::<bool>(TRACKING_BACKEND, "isam_enable_detailed_results"),
                    SETTINGS.get::<f64>(TRACKING_BACKEND, "isam_wildfire_threshold") as f32,
                    SETTINGS.get::<bool>(TRACKING_BACKEND, "isam_find_unused_factor_slots"),
                )
            },
            1 => Optimizer::IncrementalFixedLagSmoother {
                smoother: IncrementalFixedLagSmoother::new(
                    SETTINGS.get::<f64>(TRACKING_BACKEND, "isam_relinearize_threshold"),
                    SETTINGS.get::<i32>(TRACKING_BACKEND, "isam_relinearize_skip"),
                    SETTINGS.get::<bool>(TRACKING_BACKEND, "isam_cache_linearized_factors"),
                    SETTINGS.get::<bool>(TRACKING_BACKEND, "isam_enable_detailed_results"),
                    SETTINGS.get::<f64>(TRACKING_BACKEND, "isam_wildfire_threshold") as f32,
                    SETTINGS.get::<bool>(TRACKING_BACKEND, "isam_find_unused_factor_slots"),
                    SETTINGS.get::<i32>(TRACKING_BACKEND, "isam_nr_states"),
                )
            },
            2 => Optimizer::LevenbergMarquadt {},
            _ => panic!("Unknown optimizer type!"),
        };

        Self {
            optimizer,
            use_smart_factors,
            solver_state: GraphSolverState::NotInitialized,
            graph_new: NonlinearFactorGraph::default(),
            values_new: ValuesWrapper::default(),
            values_all: ValuesWrapper::default(),
            curr_id: 0,
            smartfactors: HashMap::new(),
            smartfactor_idx_in_isam2: HashMap::new(),
            last_timestamp: 0.0,
            inserted_kfs: Vec::new(),

            initial_position_sigma: SETTINGS.get::<f64>(IMU, "initialPositionSigma"),
            initial_roll_pitch_sigma: SETTINGS.get::<f64>(IMU, "initialRollPitchSigma"),
            initial_yaw_sigma: SETTINGS.get::<f64>(IMU, "initialYawSigma"),
            initial_velocity_sigma: SETTINGS.get::<f64>(IMU, "initialVelocitySigma"),
            initial_acc_bias_sigma: SETTINGS.get::<f64>(IMU, "initialAccBiasSigma"),
            initial_gyro_bias_sigma: SETTINGS.get::<f64>(IMU, "initialGyroBiasSigma"),
            accel_noise_density: SETTINGS.get::<f64>(IMU, "noise_acc"),
            gyro_noise_density: SETTINGS.get::<f64>(IMU, "noise_gyro"),
            sampling_frequency: SETTINGS.get::<f64>(IMU, "frequency").sqrt(),
            accel_random_walk: SETTINGS.get::<f64>(IMU, "acc_walk"),
            gyro_random_walk: SETTINGS.get::<f64>(IMU, "gyro_walk"),
            imu_integration_sigma: SETTINGS.get::<f64>(IMU, "imu_integration_sigma"),
            init_bias_sigma: SETTINGS.get::<f64>(IMU, "imu_bias_init_sigma"),
            tbc: ImuCalib::new().tbc,
            k,
            vision_measurement_noise,
        }
    }

    fn initialize(&mut self, timestamp: f64, imu_init: &ImuInitializationData) -> Result<(), Box<dyn std::error::Error>> {
        println!("... Initial timestamp: {}", timestamp);
        println!("... Initial pose: {:?}", imu_init.pose);
        println!("... Initial pose rotation matrix: {:?}", imu_init.pose.get_rotation());
        println!("... Initial velocity: {:?}", imu_init.velocity);
        println!("... Initial bias: {:?}", imu_init.bias);

        self.add_initial_state(imu_init.pose, imu_init.velocity, imu_init.bias, true)?;

        let (_, _updated_bias) = self.optimize(
            vec![],
            vec![],
        )?;

        self.solver_state = GraphSolverState::Ok;
        Ok(())
    }

    fn add_initial_state(
        &mut self,
        prior_pose: Pose,
        init_vel: DVVector3<f64>,
        init_bias: ImuBias,
        add_to_all_values: bool
    ) -> Result<(), Box<dyn std::error::Error>> {
        // VioBackend::initStateAndSetPriors

        // Create prior factor and add it to the graph
        let prior_state = {
            let trans = prior_pose.translation;
            let rot = prior_pose.get_quaternion();
            GtsamState {
                pose: Pose3::from_parts(
                    Point3::new(trans.x, trans.y, trans.z),
                    Rot3::from(rot)
                ),
                velocity: GTSAMVector3::new(init_vel[0], init_vel[1], init_vel[2]),
                bias: ConstantBias::new(
                    &GTSAMVector3::new(init_bias.bax, init_bias.bay, init_bias.baz),
                    &GTSAMVector3::new(init_bias.bwx, init_bias.bwy, init_bias.bwz)
                )
            }
        };

        // Set initial pose uncertainty: constrain mainly position and global yaw.
        // roll and pitch is observable, therefore low variance.
        let mut pose_prior_covariance2 = nalgebra::Matrix3::zeros();
        pose_prior_covariance2[(0, 0)] = self.initial_roll_pitch_sigma * self.initial_roll_pitch_sigma;
        pose_prior_covariance2[(1, 1)] = self.initial_roll_pitch_sigma * self.initial_roll_pitch_sigma;
        pose_prior_covariance2[(2, 2)] = self.initial_yaw_sigma * self.initial_yaw_sigma;
        pose_prior_covariance2[(2, 2)] = self.initial_yaw_sigma * self.initial_yaw_sigma;

        // Rotate initial uncertainty into local frame, where the uncertainty is
        // specified.
        let b_rot_w = prior_pose.get_rotation().transpose();
        pose_prior_covariance2 = b_rot_w * pose_prior_covariance2 * b_rot_w.transpose();

        let mut pose_prior_covariance = nalgebra::Matrix6::zeros();
        pose_prior_covariance[(0, 0)] = pose_prior_covariance2[(0,0)];
        pose_prior_covariance[(0, 1)] = pose_prior_covariance2[(0,1)];
        pose_prior_covariance[(0, 2)] = pose_prior_covariance2[(0,2)];
        pose_prior_covariance[(1, 0)] = pose_prior_covariance2[(1,0)];
        pose_prior_covariance[(1, 1)] = pose_prior_covariance2[(1,1)];
        pose_prior_covariance[(1, 2)] = pose_prior_covariance2[(1,2)];
        pose_prior_covariance[(2, 0)] = pose_prior_covariance2[(2,0)];
        pose_prior_covariance[(2, 1)] = pose_prior_covariance2[(2,1)];
        pose_prior_covariance[(2, 2)] = pose_prior_covariance2[(2,2)];
        pose_prior_covariance[(3, 3)] = self.initial_position_sigma * self.initial_position_sigma;
        pose_prior_covariance[(4, 4)] = self.initial_position_sigma * self.initial_position_sigma;
        pose_prior_covariance[(5, 5)] = self.initial_position_sigma * self.initial_position_sigma;

        // Add pose prior.
        let pose_noise = GaussianNoiseModel::from_covariance(pose_prior_covariance);
        self.graph_new.add_prior_factor_pose3(
            &Symbol::new(b'x', self.curr_id),
            &prior_state.pose, &pose_noise
        );

        // Add initial velocity priors.
        let v_noise = IsotropicNoiseModel::from_dim_and_sigma(3, self.initial_velocity_sigma);
        self.graph_new.add_prior_factor_vector3_isotropicnoisemodel(
            &Symbol::new(b'v', self.curr_id),
            &prior_state.velocity, &v_noise
        );

        // Add initial bias priors:
        let b_noise = DiagonalNoiseModel::from_sigmas(Vector6::new(
            self.initial_acc_bias_sigma,
            self.initial_acc_bias_sigma,
            self.initial_acc_bias_sigma,
            self.initial_gyro_bias_sigma,
            self.initial_gyro_bias_sigma,
            self.initial_gyro_bias_sigma,
        ));
        self.graph_new.add_prior_factor_constant_bias_diagonal(
            &Symbol::new(b'b', self.curr_id),
            &prior_state.bias, &b_noise
        );

        // Add initial state to the graph
        self.values_new.insert_pose3(
            &Symbol::new(b'x', self.curr_id),
            &prior_state.pose
        );
        self.values_new.insert_vector3(
            &Symbol::new(b'v', self.curr_id),
            &prior_state.velocity
        );
        self.values_new.insert_constant_bias(
            &Symbol::new(b'b', self.curr_id),
            &prior_state.bias
        );
        if add_to_all_values {
            self.values_all.insert_pose3(
                &Symbol::new(b'x', self.curr_id),
                &prior_state.pose
            );
            self.values_all.insert_vector3(
                &Symbol::new(b'v', self.curr_id),
                &prior_state.velocity
            );
            self.values_all.insert_constant_bias(
                &Symbol::new(b'b', self.curr_id),
                &prior_state.bias
            );
        }

        Ok(())
    }

    fn solve(&mut self,
        tracker_status: GtsamFrontendTrackingState,
        current_frame: &mut Frame,
        preintegration: & PreintegratedCombinedMeasurements, 
        feature_tracks: & TrackedFeatures,
        should_update: bool, // Batch update instead of optimizing each time. Doesn't work well though.
    ) -> Result<Vec<(u64, Pose, nalgebra::Vector3<f64>, ImuBias)>, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("solve");

        let timestamp = (current_frame.timestamp * 1e9) as i64; // Convert to int just so we can hash it

        // IMU
        self.create_imu_factor(& preintegration);
        let new_state = self.imu_predict_state(& preintegration);

        // Move node count forward in time
        self.curr_id += 1;

        // Add nodes
        self.values_new.insert_pose3(
            &Symbol::new(b'x', self.curr_id),
            &new_state.pose
        );
        self.values_new.insert_vector3(
            &Symbol::new(b'v', self.curr_id),
            &new_state.velocity
        );
        self.values_new.insert_constant_bias(
            &Symbol::new(b'b', self.curr_id),
            &new_state.bias
        );
        self.values_all.insert_pose3(
            &Symbol::new(b'x', self.curr_id),
            &new_state.pose
        );
        self.values_all.insert_vector3(
            &Symbol::new(b'v', self.curr_id),
            &new_state.velocity
        );
        self.values_all.insert_constant_bias(
            &Symbol::new(b'b', self.curr_id),
            &new_state.bias
        );

        // Add features
        let (new_factor_feature_ids, new_affected_keys) = self.process_features(feature_tracks);

        if should_update {
            self.optimize(
                new_factor_feature_ids,
                new_affected_keys
            )?;
        }

        match tracker_status {
            GtsamFrontendTrackingState::LowDisparity => {
                // Vehicle is not moving
                debug!("Tracker has a LOW_DISPARITY status. Add zero velocity and no motion factors.");
                let zero_velocity_noise = {
                    let precision = SETTINGS.get::<f64>(TRACKING_BACKEND, "zero_velocity_precision");
                    DiagonalNoiseModel::from_precisions(
                        Vector3::new(precision, precision, precision)
                    )
                };
                self.graph_new.add_prior_factor_vector3_diagonalnoisemodel(
                    &Symbol::new(b'v', self.curr_id),
                    &GTSAMVector3::new(0.0, 0.0, 0.0),
                    &zero_velocity_noise
                );
                let no_motion_prior_noise = {
                    let rotation_precision = SETTINGS.get::<f64>(TRACKING_BACKEND, "no_motion_rotation_precision");
                    let position_precision = SETTINGS.get::<f64>(TRACKING_BACKEND, "no_motion_position_precision");
                    DiagonalNoiseModel::from_precisions(
                        Vector6::new(
                            rotation_precision, rotation_precision, rotation_precision,
                            position_precision, position_precision, position_precision
                        )
                    )
                };
                self.graph_new.add_between_factor_pose3(
                    &Symbol::new(b'x', self.curr_id - 1),
                    &Symbol::new(b'x', self.curr_id),
                    &Pose3::default(),
                    & no_motion_prior_noise
                );
            },
            _ => {}
        }


        let mut optimization_results = vec![];
        match self.optimizer {
            Optimizer::ISAM2{..} | Optimizer::IncrementalFixedLagSmoother{..} => {
                // If using the smoothers, we need to edit all the prior keyframes with new optimized values
                self.inserted_kfs.push(self.curr_id);

                for state_key in &self.inserted_kfs {
                    let (pose, velocity, bias) = self.values_all.get_results_from_values(*state_key)?;

                    if state_key == &self.curr_id {
                        current_frame.set_imu_pose_velocity(
                            pose,
                            velocity
                        );
                        current_frame.imu_data.set_new_bias(bias);
                        debug!("OPTIMIZED POSE ESTIMATE... {}; {:?}; {:?}; {:?}", timestamp, pose, velocity, current_frame.imu_data.imu_bias);
                        debug!("STATE KEY: {}, {}.", self.curr_id, timestamp);
                    } else {
                        optimization_results.push(
                            (*state_key, ImuCalib::new().tcb * pose.inverse(), velocity, bias)
                        );
                        debug!("OPTIMIZED POSE ESTIMATE... STATE {}; {:?}; {:?}; {:?}", state_key, pose, velocity, current_frame.imu_data.imu_bias);
                    }
                }
            },
            Optimizer::LevenbergMarquadt{} => {
                // If using LM, only set current frame
                let (pose, velocity, bias) = self.values_all.get_results_from_values(self.curr_id)?;
                current_frame.set_imu_pose_velocity(
                    pose,
                    velocity
                );
                current_frame.imu_data.set_new_bias(bias);
                debug!("OPTIMIZED POSE ESTIMATE... {}; {:?}; {:?}; {:?}", timestamp, pose, velocity, current_frame.imu_data.imu_bias);
            }
        }

        self.last_timestamp = current_frame.timestamp;

        Ok(optimization_results)
    }

    fn imu_predict_state(&self, preintegration: & PreintegratedCombinedMeasurements) -> GtsamState {
        let _span = tracy_client::span!("get_predicted_state");
        // This function will get the predicted state based on the IMU measurement

        // Get the current state (t=k)
        let state_k = GtsamState {
            pose: self.values_all.get_pose3(&Symbol::new(b'x', self.curr_id)).unwrap().into(),
            velocity: self.values_all.get_vector3(&Symbol::new(b'v', self.curr_id)).unwrap().into(),
            bias: self.values_all.get_constant_bias(&Symbol::new(b'b', self.curr_id)).unwrap().into()
        };

        // From this we should predict where we will be at the next time (t=K+1)
        let state_k1 = preintegration.predict(
            &gtsam::navigation::navstate::NavState::new(
                &state_k.pose,
                &state_k.velocity
            ),
            &state_k.bias
        );

        let predicted = GtsamState {
            pose: state_k1.get_pose().into(),
            velocity: state_k1.get_velocity().into(),
            bias: state_k.bias
        };

        return predicted;
    }

    fn create_imu_factor(&mut self, preintegration: & PreintegratedCombinedMeasurements) {
        let imu_factor = CombinedImuFactor::new(
            &Symbol::new(b'x', self.curr_id),
            &Symbol::new(b'v', self.curr_id),
            &Symbol::new(b'x', self.curr_id + 1),
            &Symbol::new(b'v', self.curr_id + 1),
            &Symbol::new(b'b', self.curr_id),
            &Symbol::new(b'b', self.curr_id + 1),
            & preintegration
        );
        self.graph_new.add_combined_imu_factor(&imu_factor);
    }

    fn process_features(&mut self, feature_tracks: &TrackedFeatures) -> (Vec<i32>, Vec<DoubleVec>) {
        let _span = tracy_client::span!("process_smart_features");

        let mut id_notset = 0;
        let mut num_new_landmarks = 0;
        let mut num_updated_landmarks = 0;
        let mut num_skipped = 0;

        let mut new_factor_feature_ids: Vec<i32> = vec![];
        let mut new_affected_keys: Vec<DoubleVec> = vec![];

        // println!("NEW TRACKED FEATURES: {:?}", new_tracked_features);
        // println!("New tracked features len? {}", new_tracked_features.len());
        // for fact in &self.smartfactors {
        //     println!("Smart factors! {:?} -> ({}, {})", fact.0, fact.1.1, fact.1.2);
        // }

        for i in 0..feature_tracks.len() {
            let feature_id = feature_tracks.get_feature_id(i as usize);
            if feature_id == -1 {
                id_notset += 1;
                warn!("Id not set for feature with point {:?}?", feature_tracks.get_point(i as usize));
                continue;
            }
            let point = feature_tracks.get_point(i as usize);

            match self.smartfactors.get_mut(&feature_id) {
                Some((smartfactor, is_in_graph, mut num_observations)) => {
                    // Feature has been seen before, but not necessarily in the factor graph yet

                    // Insert measurements to existing factor
                    smartfactor.add(
                        & Point2::new(point.x as f64, point.y as f64),
                        &Symbol::new(b'x', self.curr_id)
                    );
                    num_observations += 1;

                    match self.optimizer {
                        Optimizer::ISAM2{..} => {
                            // Check for is_in_graph b/c it is possible that the smartfactor is created 
                            // from a previous keyframe, but not in the graph yet if we are batching updates.
                            if *is_in_graph {
                                // Update new_affected_keys to show that this smart factor was updated
                                // This is sent to the isam2 update function
                                let index_in_isam2 = self.smartfactor_idx_in_isam2[&feature_id];
                                new_affected_keys.push(DoubleVec{
                                        vec: vec![index_in_isam2 as f64, self.curr_id as f64]
                                });
                            }
                        },
                        _ => {}
                    }

                    // Only add feature if there are more than two observations
                    if num_observations >= 2 && !*is_in_graph {
                        self.graph_new.add_smartfactor(&smartfactor);
                        num_new_landmarks += 1;
                    } else if num_observations >= 2 && *is_in_graph {
                        num_updated_landmarks += 1;
                    } else {
                        num_skipped += 1;
                    }

                    continue;
                },
                None => {
                    // Feature is newly extracted

                    // Create a smart factor for the new feature
                    let mut smartfactor = SmartProjectionPoseFactorCal3S2::new(
                        &self.vision_measurement_noise,
                        &self.k,
                        & self.tbc.into()
                    );
                    smartfactor.add(
                        & gtsam::geometry::point2::Point2::new(point.x as f64, point.y as f64),
                        &Symbol::new(b'x', self.curr_id)
                    );

                    // Insert to lookup, so we can find this factor in the next frames
                    self.smartfactors.insert(feature_id, (smartfactor, false, 1));

                    // Keep track of factors in the order we insert them in, we will need this after the update in isam2
                    new_factor_feature_ids.push(feature_id);
                    num_skipped += 1;
                }
            }
        }
        debug!("Added new features: {}, updated old features: {}, skipped: {}, id not set: {}", num_new_landmarks, num_updated_landmarks, num_skipped, id_notset);

        return (new_factor_feature_ids, new_affected_keys);
    }

    fn optimize(
        &mut self,
        new_factor_feature_ids: Vec<i32>,
        new_affected_keys: Vec<DoubleVec>,
        // timestamps: Vec<DoubleVec>,
    ) -> Result<(Vec<gtsam::sys::Point>, ImuBias), Box<dyn std::error::Error>> {
        // VioBackend::optimize
        let _span = tracy_client::span!("optimize");

        let mut points = vec![];

        match &mut self.optimizer {
            Optimizer::ISAM2{isam2} => {
                // Removed features... currently not using this
                // let mut removed_feature_ids_idx_in_isam2: Vec<u64> = vec![];
                // for id in &removed_feature_ids {
                //     // Remove smartfactor from our list of smartfactors
                //     self.smartfactors.remove(&(*id as i32));
                //     removed_feature_ids_idx_in_isam2.push(self.smartfactor_idx_in_isam2[&(*id as i32)] as u64);
                // }

                // Perform smoothing update
                let isam2result = isam2.update(
                    & self.graph_new,
                    & self.values_new.get_values(),
                    & new_affected_keys,
                    & vec![]);

                self.values_all.update(isam2.calculate_estimate());

                // ISAM2 returns vector of indexes for each smartfactor within isam2. The vector is 
                // sorted by order of smartfactor insertion.
                // Here, link up inserted smartfactor feature ID with isam2 index. We will need these
                // links later to tell isam2 that a previously inserted smartfactor has an update
                for i in 0..new_factor_feature_ids.len() {
                    let smartfactor_id_here = new_factor_feature_ids[i];
                    let smartfactor_id_in_isam2 = isam2result.new_factor_indices[i];
                    self.smartfactor_idx_in_isam2.insert(smartfactor_id_here, smartfactor_id_in_isam2);
                    self.smartfactors.get_mut(&smartfactor_id_here).unwrap().1 = true; // Mark that this smartfactor is now in the graph
                }
                debug!("TEST! After optimize, smartfactor_idx_in_isam2 is: {:?}", self.smartfactor_idx_in_isam2);

                points = isam2result.points;
            },
            Optimizer::IncrementalFixedLagSmoother{smoother} => {
                // Perform smoothing update
                // SOFIYA TODO THIS 11/13

                // Use current timestamp for each new value. This timestamp will be used
                // to determine if the variable should be marginalized.
                // Needs to use DOUBLE because gtsam works with that, but we
                // are actually counting the number of states in the smoother.
                let mut timestamps = Vec::<DoubleVec>::new();
                for key in &self.values_new.keys {
                    timestamps.push(DoubleVec{
                        vec: vec![*key as f64, self.curr_id as f64]
                    });
                }
                
                let result = smoother.update(
                    & self.graph_new,
                    & self.values_new.get_values(),
                    &timestamps,
                    &Vec::<i32>::new()
                );
            },
            Optimizer::LevenbergMarquadt{} => {
                let params = LevenbergMarquardtParams::default();
                let mut optimizer = LevenbergMarquardtOptimizer::new(
                    & self.graph_new,
                    & self.values_new.get_values(),
                    & params
                );
                let result = optimizer.optimize_safely();
                self.values_all.update(result);
                points = vec![]; // Ignoring mappoints for LM
            }
        };

        // Use the optimized bias to reset integration
        let optimized_bias = if self.values_all.exists(&Symbol::new(b'b', self.curr_id)) {
            let constant_bias = self.values_all.get_constant_bias(&Symbol::new(b'b', self.curr_id))?;
            let gyro_bias = constant_bias.gyro_bias().get_raw();
            let accel_bias = constant_bias.accel_bias().get_raw();
            ImuBias::new_with(
                DVVector3::new_with(gyro_bias[0], gyro_bias[1], gyro_bias[2]),
                DVVector3::new_with(accel_bias[0], accel_bias[1], accel_bias[2])
            )
        } else {
            warn!("Bias wasn't optimized?");
            ImuBias::new()
        };

        // Remove the used up factors and nodes
        self.graph_new.resize(0);
        self.values_new.clear();

        // For Levenberg-Marquardt, re-add prior factors for the current state after each optimization step.
        match self.optimizer {
            Optimizer::LevenbergMarquadt{} => {
                let (pose, velocity, bias) = self.values_all.get_results_from_values(self.curr_id)?;
                self.add_initial_state(pose, velocity.into(), bias, false)?;
            },
            _ => {}
        }

        Ok((points, optimized_bias))
    }


}


#[derive(Debug)]
struct GtsamState {
    pub pose: Pose3,
    pub velocity: GTSAMVector3,
    pub bias: ConstantBias,
}

#[derive(Clone)]
enum GraphSolverState {
    NotInitialized,
    Ok
}


struct ValuesWrapper {
    values: Values, // New values that have not been optimized yet. gtsam object
    keys: Vec<Key>, // Keys in values_new, for iteration
}
impl Default for ValuesWrapper {
    fn default() -> Self {
        Self { values: Default::default(), keys: Default::default() }
    }
}
impl ValuesWrapper {
    fn update(&mut self, new_values: Values) {
        // Update values after optimization.
        // Keys can stay as-is because no new keys are added as part of the optimization.
        self.values = new_values;
    }

    fn get_values(&self) -> &Values {
        &self.values
    }

    fn exists(&self, key: &Symbol) -> bool {
        self.values.exists(key)
    }

    fn clear(&mut self) {
        self.values.clear();
        self.keys.clear();
    }

    fn insert_pose3(
        &mut self,
        key: &Symbol,
        pose: &Pose3
    ) {
        self.keys.push(key.key());
        self.values.insert_pose3(key, pose);
    }
    fn insert_vector3(
        &mut self,
        key: &Symbol,
        vector: &GTSAMVector3
    ) {
        self.keys.push(key.key());
        self.values.insert_vector3(&key, &vector);
    }
    fn insert_constant_bias(
        &mut self,
        key: &Symbol,
        bias: &ConstantBias
    ) {
        self.keys.push(key.key());
        self.values.insert_constant_bias(&key, &bias);
    }

    fn get_pose3(
        &self,
        key: &Symbol,
    ) -> Result<gtsam::geometry::pose3::Pose3Ref<'_>, Box<dyn std::error::Error>> {
        self.values.get_pose3(key).ok_or("Could not find pose3".into())
    }
    fn get_vector3(
        &self,
        key: &gtsam::inference::symbol::Symbol,
    ) -> Result<gtsam::base::vector::Vector3Ref<'_>, Box<dyn std::error::Error>> {
        self.values.get_vector3(key).ok_or("Could not find vector3".into())
    }
    fn get_constant_bias(
        &self,
        key: &gtsam::inference::symbol::Symbol,
    ) -> Result<gtsam::imu::imu_bias::ConstantBiasRef<'_>, Box<dyn std::error::Error>> {
        self.values.get_constantbias(key).ok_or("Could not find bias".into())
    }

    fn get_results_from_values(&self, state_key: u64) -> Result<(Pose, nalgebra::Vector3<f64>, ImuBias), Box<dyn std::error::Error>> {
        let pose = {
            let pose: Isometry3<f64> = self.values.get_pose3(&Symbol::new(b'x', state_key)).unwrap().into();
            Pose::new_from_isometry(pose)
        };
        let velocity = {
            let velocity: GTSAMVector3 = self.values.get_vector3(&Symbol::new(b'v', state_key)).unwrap().into();
            let vel_raw = velocity.get_raw();
            Vector3::new(vel_raw[0], vel_raw[1], vel_raw[2])
        };
        let bias = {
            let bias_ref = self.values.get_constantbias(&Symbol::new(b'b', state_key)).unwrap();
            let accel_bias = bias_ref.accel_bias().get_raw();
            let gyro_bias = bias_ref.gyro_bias().get_raw();
            ImuBias {
                bax: accel_bias[0],
                bay: accel_bias[1],
                baz: accel_bias[2],
                bwx: gyro_bias[0],
                bwy: gyro_bias[1],
                bwz: gyro_bias[2]
            }
        };

        Ok((pose, velocity, bias))
    }

}
