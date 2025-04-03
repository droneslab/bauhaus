extern crate g2o;
use log::{warn, info, debug, error};
use nalgebra::{Isometry3, Quaternion, Vector3, Vector6};
use std::{collections::{HashMap, VecDeque}, f64::INFINITY, thread::sleep, time::Duration};
use opencv::{prelude::*, types::{VectorOfKeyPoint, VectorOfMat, VectorOfPoint2f}};
use gtsam::{
    inference::symbol::Symbol, navigation::combined_imu_factor::{CombinedImuFactor, PreintegratedCombinedMeasurements, PreintegrationCombinedParams}, nonlinear::{
        isam2::ISAM2, nonlinear_factor_graph::NonlinearFactorGraph, values::Values
    },
};
use std::fmt::Debug;
use core::{
    config::*, matrix::*, sensor::{FrameSensor, ImuSensor, Sensor}, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{messages::{ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg}, tracking_frontend_gtsam::{TrackedFeatures, TrackedFeaturesIndexMap}},
    map::{features::Features, frame::Frame, keyframe::{KeyFrame, MapPointMatches}, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{geometric_tools, good_features_to_track::GoodFeaturesExtractor, image, imu::{ImuBias, ImuCalib, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{CameraModule, FeatureExtractionModule, ImuModule}, optimizer::{self, LEVEL_SIGMA2}, orbslam_extractor::ORBExtractor, orbslam_matcher::SCALE_FACTORS, relocalization::Relocalization}, registered_actors::{LOCAL_MAPPING, SHUTDOWN_ACTOR, VISUALIZER}, ImuInitializationData};

use super::{messages::{FeatureTracksAndIMUMsg, NewKeyFrameGTSAMMsg, TrajectoryMsg, VisTrajectoryMsg}, tracking_backend::TrackingState};
use crate::registered_actors::IMU;
use std::collections::BTreeSet;

pub struct TrackingBackendGTSAM {
    system: System,
    map: ReadWriteMap,
    // sensor: Sensor,

    // Last kf/frame
    last_kf_id: Id,
    last_kf_pose: Pose,
    last_timestamp: Timestamp,
    last_kf_imu_bias: ImuBias,
    last_feature_tracks: TrackedFeatures,

    // Modules 
    graph_solver: GraphSolver,

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
}

impl Actor for TrackingBackendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        // let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        // let imu = IMU::new();

        let mut actor = TrackingBackendGTSAM {
            system,
            graph_solver: GraphSolver::new(),
            // sensor,
            map,
            trajectory_poses: Vec::new(),
            last_kf_id: -1,
            last_timestamp: 0.0,
            last_kf_pose: Pose::default(),
            last_kf_imu_bias: ImuBias::new(),
            last_feature_tracks: TrackedFeatures::default(),
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

    fn handle_regular_message(&mut self, mut msg: FeatureTracksAndIMUMsg) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("track");

        if self.last_kf_id == -1 {
            // First frame received here... was already put into local mapping map as latest keyframe
            // Need to initialize the factor graph 
            let map = self.map.read()?;
            let kf0 = map.get_keyframe(0);
            let kf1 = map.get_keyframe(1);
            let kf0_pose = kf0.get_pose();
            let kf1_pose = kf1.get_pose();

            let velocity = DVVector3::new(
                (*kf1_pose.get_translation() - *kf0_pose.get_translation()) / (kf1.timestamp - kf0.timestamp)
            );

            println!("From map, first pose is: {:?}", kf0_pose);
            println!("From map, second pose is: {:?}", kf1_pose);
            println!("From map, velocity is: {:?}", velocity);


            // let imu_init = msg.imu_initialization.expect("Msg should have imu initialization data!");
            // println!("Backend initializing graph...");
            // println!("From imu initialization, translation is: {:?}", imu_init.translation);
            // println!("From imu initialization, rotation is: {:?}", imu_init.rotation);
            // println!("From imu initialization, velocity is: {:?}", imu_init.velocity);
            // println!("From imu initialization, acc bias is: {:?}", imu_init.acc_bias);
            // println!("From imu initialization, gyro bias is: {:?}", imu_init.gyro_bias);


            let imu_init = ImuInitializationData {
                translation: kf1_pose.get_translation(),
                rotation: kf1_pose.get_quaternion().vector(),
                velocity: velocity,
                acc_bias: imu_init.acc_bias,
                gyro_bias: imu_init.gyro_bias
            };


            self.graph_solver.initialize(
                (msg.frame.timestamp * 1e9) as i64,
                &imu_init
            )?;
            self.graph_solver.solver_state = GraphSolverState::Ok;
            self.last_timestamp = msg.frame.timestamp;
            self.last_kf_id = 1;
            self.last_kf_pose = msg.frame.pose.unwrap();
            self.last_kf_imu_bias = ImuBias {
                bax: imu_init.acc_bias[0],
                bay: imu_init.acc_bias[1],
                baz: imu_init.acc_bias[2],
                bwx: imu_init.gyro_bias[0],
                bwy: imu_init.gyro_bias[1],
                bwz: imu_init.gyro_bias[2]
            };

        } else {
            // If we have previous frames already, can track normally
            let mut current_frame = msg.frame;
            // current_frame.imu_data.set_new_bias(self.map.read()?.get_keyframe(self.last_kf_id).imu_data.get_imu_bias());

            // Solve VIO graph. Includes preintegration
            let optimized = self.graph_solver.solve(
                &mut current_frame,
                &mut msg.imu_measurements, &msg.feature_tracks,
                self.last_timestamp, self.last_kf_imu_bias, self.last_kf_id
            )?;
            if !optimized {
                warn!("Could not optimize graph");
            }

            println!("Huh? Optimized pose in frame: {:?}", current_frame.pose);

            current_frame.ref_kf_id = Some(self.last_kf_id);
            self.update_trajectory_in_logs(& current_frame).expect("Could not save trajectory");
            self.last_timestamp = current_frame.timestamp;
            self.last_kf_pose = current_frame.pose.unwrap();
            self.last_kf_id += 1;
            self.last_kf_imu_bias = current_frame.imu_data.imu_bias;


            // SOFIYA TURN OFF LOCAL MAPPING
            // println!("TRACKING BACKEND SEND TO LOCAL MAPPING");
            // // KeyFrame created here and inserted into map
            // self.system.send(
            //     LOCAL_MAPPING,
            //     Box::new( NewKeyFrameGTSAMMsg{
            //         tracking_state: TrackingState::Ok,
            //         keyframe: current_frame,
            //         map_version: self.map.read()?.version
            //     } )
            // );
        }

        self.last_feature_tracks = msg.feature_tracks;

        return Ok(());
    }

    fn update_trajectory_in_logs(
        &mut self, current_frame: &Frame,
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
        // SOFIYA TURN OFF LOCAL MAPPING
        // self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
        //     pose: current_frame.pose.unwrap(),
        //     mappoint_matches: vec![],
        //     nontracked_mappoints: HashMap::new(),
        //     mappoints_in_tracking: BTreeSet::new(),
        //     timestamp: current_frame.timestamp,
        //     map_version: map.version
        // }));

        Ok(())
    }
}

pub struct GraphSolver {
    solver_state: GraphSolverState,

    // New graph
    graph_new: NonlinearFactorGraph, // New factors that have not been optimized yet
    values_new: Values, // New values that have not been optimized yet

    // Main graph
    graph_main: NonlinearFactorGraph, // Main non-linear GTSAM graph, all created factors
    values_initial: Values, // All created nodes

    // Misc GTSAM objects
    isam2: ISAM2, // ISAM2 solvers
    preint_gtsam: PreintegratedCombinedMeasurements, // IMU preintegration

    // IMU preintegration parameters
    // Initialization
    // Noise values from dataset sensor
    accel_noise_density: f64, // accelerometer_noise_density, sigma_a
    gyro_noise_density: f64, // gyroscope_noise_density, sigma_g
    accel_random_walk: f64, // accelerometer_random_walk, sigma_wa
    gyro_random_walk: f64, // gyroscope_random_walk, sigma_wg
    // Noise values for initialization
    // sigma_prior_rotation: [f64; 3],
    // sigma_prior_translation: [f64; 3],
    // sigma_velocity: f64,
    // sigma_bias: f64,
    // Misc
    sigma_camera: f64,

    // Kimera
    // Noise values for initialization
    initial_position_sigma: f64,
    initial_roll_pitch_sigma: f64,
    initial_yaw_sigma: f64,
    initial_velocity_sigma: f64,
    initial_acc_bias_sigma: f64,
    initial_gyro_bias_sigma: f64,

    imu_integration_sigma: f64,
    init_bias_sigma: f64,

    // Iterations of the map
    ct_state: u64,
    ct_state_lookup: HashMap<i64, u64>,
    timestamp_lookup: HashMap<u64, i64>,
    measurement_smart_lookup_left: HashMap<i32, gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2>, // Smart lookup for IMU measurements
}

impl GraphSolver {
    pub fn new() -> Self {
        Self {
            graph_new: NonlinearFactorGraph::default(),
            values_new: Values::default(),
            graph_main: NonlinearFactorGraph::default(),
            values_initial: Values::default(),
            isam2: ISAM2::default(),
            preint_gtsam: PreintegratedCombinedMeasurements::default(),
            solver_state: GraphSolverState::NotInitialized,

            // TODO SOFIYA PARAMS
            // sigma_prior_rotation: [0.1, 0.1, 0.1],
            // sigma_prior_translation: [0.3, 0.3, 0.3],
            // sigma_velocity: 0.1,
            // sigma_bias: 0.15,
            sigma_camera: 0.306555403,
            initial_position_sigma: SETTINGS.get::<f64>(IMU, "initialPositionSigma"),
            initial_roll_pitch_sigma: SETTINGS.get::<f64>(IMU, "initialRollPitchSigma"),
            initial_yaw_sigma: SETTINGS.get::<f64>(IMU, "initialYawSigma"),
            initial_velocity_sigma: SETTINGS.get::<f64>(IMU, "initialVelocitySigma"),
            initial_acc_bias_sigma: SETTINGS.get::<f64>(IMU, "initialAccBiasSigma"),
            initial_gyro_bias_sigma: SETTINGS.get::<f64>(IMU, "initialGyroBiasSigma"),
            accel_noise_density: SETTINGS.get::<f64>(IMU, "noise_acc"),
            gyro_noise_density: SETTINGS.get::<f64>(IMU, "noise_gyro"),
            accel_random_walk: SETTINGS.get::<f64>(IMU, "acc_walk"),
            gyro_random_walk: SETTINGS.get::<f64>(IMU, "gyro_walk"),
            imu_integration_sigma: SETTINGS.get::<f64>(IMU, "imu_integration_sigma"),
            init_bias_sigma: SETTINGS.get::<f64>(IMU, "imu_bias_init_sigma"),

            ct_state: 0,
            ct_state_lookup: HashMap::new(),
            timestamp_lookup: HashMap::new(),
            measurement_smart_lookup_left: HashMap::new(),
        }
    }

    fn solve(&mut self,
        current_frame : &mut Frame, 
        imu_measurements : &mut ImuMeasurements, new_tracked_features : &TrackedFeatures,
        last_timestamp: Timestamp, last_frame_imu_bias : ImuBias, last_kf_id : Id
    ) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("solve");

        let timestamp = (current_frame.timestamp * 1e9) as i64; // Convert to int just so we can hash it

        // Return if the node already exists in the graph
        if self.ct_state_lookup.contains_key(&timestamp) {
            warn!("NODE WITH TIMESTAMP {} ALREADY EXISTS", timestamp);
            return Ok(false);
        }

        if matches!(self.solver_state, GraphSolverState::NotInitialized) {
            panic!("SOLVER SHOULD BE INITIALIZED BY NOW!");
        } else {
            self.create_imu_factor(imu_measurements, current_frame, last_timestamp, last_frame_imu_bias, last_kf_id)?;

            // Original models
            let new_state = self.get_predicted_state();

            // Move node count forward in time
            self.ct_state += 1;

            // Append to our node vectors
            self.values_new.insert_pose3(
                &Symbol::new(b'x', self.ct_state),
                &new_state.pose
            );
            self.values_new.insert_vector3(
                &Symbol::new(b'v', self.ct_state),
                &new_state.velocity
            );
            self.values_new.insert_constant_bias(
                &Symbol::new(b'b', self.ct_state),
                &new_state.bias
            );
            self.values_initial.insert_pose3(
                &Symbol::new(b'x', self.ct_state),
                &new_state.pose
            );
            self.values_initial.insert_vector3(
                &Symbol::new(b'v', self.ct_state),
                &new_state.velocity
            );
            self.values_initial.insert_constant_bias(
                &Symbol::new(b'b', self.ct_state),
                &new_state.bias
            );

            // Add ct state to map
            self.ct_state_lookup.insert(timestamp, self.ct_state);
            self.timestamp_lookup.insert(self.ct_state, timestamp);
        }

        self.process_smart_features(new_tracked_features);

        let optimized_pose = self.optimize();

        if optimized_pose {
            // Update frame with optimized values
            // TODO there's probably a better way to clean up all this conversion than this
            let updated_pose: Isometry3<f64> = self.values_initial.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into();
            current_frame.pose = Some(Pose::new_from_isometry(updated_pose));
            let velocity: gtsam::base::vector::Vector3 = self.values_initial.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into();
            let vel_raw = velocity.get_raw();
            current_frame.imu_data.velocity = Some(DVVector3::new_with(vel_raw[0], vel_raw[1], vel_raw[2]));
            let bias_ref = self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap();
            let accel_bias = bias_ref.accel_bias().get_raw();
            let gyro_bias = bias_ref.gyro_bias().get_raw();

            // TODO SOFIYA Should this be set_new_bias?
            current_frame.imu_data.imu_bias = ImuBias {
                bax: accel_bias[0],
                bay: accel_bias[1],
                baz: accel_bias[2],
                bwx: gyro_bias[0],
                bwy: gyro_bias[1],
                bwz: gyro_bias[2]
            };

            debug!("!!!!!! FINAL FRAME POSE: {:?}", current_frame.pose);
        } else {
            error!("Could not optimize pose!");
        }


        Ok(optimized_pose)
    }

    fn initialize(&mut self, timestamp: i64, imu_init: &ImuInitializationData) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("initialize");

        let init_pose = Pose::new_with_quaternion_convert(*imu_init.translation, imu_init.rotation);

        // These two are identical in the beginning, but _from_state_ is used in
        // the optimizer and _from_increments_ is used as a smooth output
        let w_pose_b_lkf_from_state = init_pose;
        let w_pose_b_lkf_from_increments = init_pose;
        let w_vel_b_lkf = gtsam::base::vector::Vector3::new(
            imu_init.velocity[0],
            imu_init.velocity[1],
            imu_init.velocity[2]
        );
        let imu_bias_lkf = gtsam::imu::imu_bias::ConstantBias::new(
            &gtsam::base::vector::Vector3::new(
                imu_init.acc_bias[0],
                imu_init.acc_bias[1],
                imu_init.acc_bias[2]
            ),
            &gtsam::base::vector::Vector3::new(
                imu_init.gyro_bias[0],
                imu_init.gyro_bias[1],
                imu_init.gyro_bias[2]
            )
        );
        let imu_bias_prev_kf = gtsam::imu::imu_bias::ConstantBias::new(
            &gtsam::base::vector::Vector3::new(
                imu_init.acc_bias[0],
                imu_init.acc_bias[1],
                imu_init.acc_bias[2]
            ),
            &gtsam::base::vector::Vector3::new(
                imu_init.gyro_bias[0],
                imu_init.gyro_bias[1],
                imu_init.gyro_bias[2]
            )
        );

        // Set initial covariance for inertial factors
        // W_Pose_Blkf_ set by motion capture to start with
        let b_rot_w = w_pose_b_lkf_from_state.get_rotation().transpose();

        // Set initial pose uncertainty: constrain mainly position and global yaw.
        // roll and pitch is observable, therefore low variance.
        let mut pose_prior_covariance = nalgebra::Matrix6::zeros();
        pose_prior_covariance[(0, 0)] = self.initial_roll_pitch_sigma * self.initial_roll_pitch_sigma;
        pose_prior_covariance[(1, 1)] = self.initial_roll_pitch_sigma * self.initial_roll_pitch_sigma;
        pose_prior_covariance[(2, 2)] = self.initial_yaw_sigma * self.initial_yaw_sigma;
        pose_prior_covariance[(3, 3)] = self.initial_position_sigma * self.initial_position_sigma;
        pose_prior_covariance[(4, 4)] = self.initial_position_sigma * self.initial_position_sigma;
        pose_prior_covariance[(5, 5)] = self.initial_position_sigma * self.initial_position_sigma;


        // Rotate initial uncertainty into local frame, where the uncertainty is
        // specified.

        // todo
        let temp = b_rot_w * pose_prior_covariance[(3, 3)] * b_rot_w.transpose();
        // pose_prior_covariance.topLeftCorner(3, 3) =
        //     B_Rot_W * pose_prior_covariance.topLeftCorner(3, 3) * B_Rot_W.transpose();

        // Add pose prior.
        let pose_noise = gtsam::linear::noise_model::DiagonalNoiseModel::from_sigmas(Vector6::new(
            pose_prior_covariance[(0, 0)],
            pose_prior_covariance[(1, 1)],
            pose_prior_covariance[(2, 2)],
            pose_prior_covariance[(3, 3)],
            pose_prior_covariance[(4, 4)],
            pose_prior_covariance[(5, 5)]
        ));
        // new_imu_prior_and_other_factors_
        //     .emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
        //         gtsam::Symbol(kPoseSymbolChar, frame_id),
        //         W_Pose_B_lkf_from_state_,
        //         noise_init_pose);


        // Add initial velocity priors.
        let v_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(3, self.initial_velocity_sigma);
        // new_imu_prior_and_other_factors_
        //     .emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        //         gtsam::Symbol(kVelocitySymbolChar, frame_id),
        //         W_Vel_B_lkf_,
        //         noise_init_vel_prior);

        // Add initial bias priors:
        let b_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(6, self.initial_acc_bias_sigma);

        // Vector6 prior_biasSigmas;
        // prior_biasSigmas.head<3>().setConstant(backend_params_.initialAccBiasSigma_);
        // prior_biasSigmas.tail<3>().setConstant(backend_params_.initialGyroBiasSigma_);
        // // TODO(Toni): Make this noise model a member constant.
        // gtsam::SharedNoiseModel imu_bias_prior_noise =
        //     gtsam::noiseModel::Diagonal::Sigmas(prior_biasSigmas);
        // if (VLOG_IS_ON(10))
        // {
        //     LOG(INFO) << "Imu bias for Backend prior:";
        //     imu_bias_lkf_.print();
        // }
        // new_imu_prior_and_other_factors_
        //     .emplace_shared<gtsam::PriorFactor<gtsam::imuBias::ConstantBias>>(
        //         gtsam::Symbol(kImuBiasSymbolChar, frame_id),
        //         imu_bias_lkf_,
        //         imu_bias_prior_noise);



        // Create prior factor and add it to the graph
        let prior_state = {
            // Some really gross conversions here...
            let rot_gtsam: gtsam::geometry::rot3::Rot3 = init_pose.get_quaternion().into();
            GtsamState {
                pose: gtsam::geometry::pose3::Pose3::from_parts(
                    gtsam::geometry::point3::Point3::new(init_pose.translation.x, init_pose.translation.y, init_pose.translation.z),
                    rot_gtsam
                ),
                velocity: gtsam::base::vector::Vector3::new(imu_init.velocity[0], imu_init.velocity[1], imu_init.velocity[2]),
                bias: gtsam::imu::imu_bias::ConstantBias::new(
                    &gtsam::base::vector::Vector3::new(imu_init.acc_bias[0], imu_init.acc_bias[1], imu_init.acc_bias[2]),
                    &gtsam::base::vector::Vector3::new(imu_init.gyro_bias[0], imu_init.gyro_bias[1], imu_init.gyro_bias[2]),
                )
            }
        };


        // OLD:

        // // TODO SOFIYA  where are these coming from
        // let pose_noise = gtsam::linear::noise_model::DiagonalNoiseModel::from_sigmas(Vector6::new(
        //     self.sigma_prior_rotation[0], self.sigma_prior_rotation[1], self.sigma_prior_rotation[2],
        //     self.sigma_prior_translation[0], self.sigma_prior_translation[1], self.sigma_prior_translation[2]
        // ));
        // let v_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(3, self.sigma_velocity);
        // let b_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(6, self.sigma_bias);

        self.graph_new.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);
        self.graph_new.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);
        self.graph_new.add_prior_factor_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);
        self.graph_main.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);
        self.graph_main.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);
        self.graph_main.add_prior_factor_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);

        // Add initial state to the graph
        self.values_new.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
        self.values_new.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
        self.values_new.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);
        self.values_initial.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
        self.values_initial.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
        self.values_initial.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);

        // Add ct state to map
        self.ct_state_lookup.insert(timestamp, self.ct_state);
        self.timestamp_lookup.insert(self.ct_state, timestamp);

        // Kimera... Look at function generateCombinedPim and convertVioImuParamsToGtsam
        // Create GTSAM preintegration parameters for use with Foster's version
        let mut params = PreintegrationCombinedParams::makesharedu();  // Z-up navigation frame: gravity points along negative Z-axis !!!
        params.set_gyroscope_covariance(self.gyro_noise_density * self.gyro_noise_density);
        params.set_accelerometer_covariance(self.accel_noise_density * self.accel_noise_density);
        params.set_integration_covariance(self.imu_integration_sigma * self.imu_integration_sigma);
        params.set_bias_acc_omega_int(self.init_bias_sigma);
        params.set_bias_acc_covariance(self.accel_random_walk * self.accel_random_walk);
        params.set_bias_omega_covariance(self.gyro_random_walk * self.gyro_random_walk);


  
        // Actually create the GTSAM preintegration
        self.preint_gtsam = PreintegratedCombinedMeasurements::new(params, &prior_state.bias);

        Ok(())
    }

    fn get_predicted_state(&self) -> GtsamState {
        let _span = tracy_client::span!("get_predicted_state");
        // This function will get the predicted state based on the IMU measurement

        // Get the current state (t=k)
        let state_k = GtsamState {
            pose: self.values_initial.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into(), // W_Pose_B_lkf_from_state_
            velocity: self.values_initial.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into(),
            bias: self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
        };

        debug!("... current state: {:?}", state_k);

        // From this we should predict where we will be at the next time (t=K+1)
        let state_k1 = self.preint_gtsam.predict(
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

        debug!("IMU PREDICTED POSE USING GTSAM: {:?}", predicted.pose);

        return predicted;
    }

    fn create_imu_factor(&mut self, imu_measurements: &mut ImuMeasurements, current_frame: &mut Frame, last_timestamp: Timestamp, last_frame_imu_bias: ImuBias, last_kf_id: Id) -> Result<(), Box<dyn std::error::Error>> {
        // This function will create a discrete IMU factor using the GTSAM preintegrator class
        // This will integrate from the current state time up to the new update time
        let _span = tracy_client::span!("create_imu_factor");

        let mut imu_from_last_frame = VecDeque::with_capacity(imu_measurements.len()); // mvImuFromLastFrame
        let imu_per = 0.001;

        while !imu_measurements.is_empty() {
            if imu_measurements.front().unwrap().timestamp < last_timestamp - imu_per {
                imu_measurements.pop_front();
            } else if imu_measurements.front().unwrap().timestamp < current_frame.timestamp - imu_per {
                let msmt = imu_measurements.pop_front().unwrap();
                imu_from_last_frame.push_back(msmt);
            } else {
                let msmt = imu_measurements.pop_front().unwrap();
                imu_from_last_frame.push_back(msmt);
                break;
            }
        }
        let n = imu_from_last_frame.len() - 1;

        let mut other_imu = IMU::new();
        let mut imu_preintegrated_from_last_frame = ImuPreIntegrated::new(last_frame_imu_bias);

        for i in 0..n {
            let mut tstep = 0.0;
            let mut acc: Vector3<f64> = Vector3::zeros(); // acc
            let mut ang_vel: Vector3<f64> = Vector3::zeros(); // angVel

            if i == 0 && i < (n - 1) {
                let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                let tini = imu_from_last_frame[i].timestamp - last_timestamp;
                acc = (
                    imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
                    (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tini/tab)
                ) * 0.5;
                ang_vel = (
                    imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
                    (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tini/tab)
                ) * 0.5;
                tstep = imu_from_last_frame[i + 1].timestamp - last_timestamp;
            } else if i < (n - 1) {
                acc = (imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc) * 0.5;
                ang_vel = (imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel) * 0.5;
                tstep = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
            } else if i > 0 && i == (n - 1) {
                let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                let tend = imu_from_last_frame[i + 1].timestamp - current_frame.timestamp;
                acc = (
                    imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
                    (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tend/tab)
                ) * 0.5;
                ang_vel = (
                    imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
                    (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tend/tab)
                ) * 0.5;
                tstep = current_frame.timestamp - imu_from_last_frame[i].timestamp;
            } else if i == 0 && i == (n - 1) {
                acc = imu_from_last_frame[i].acc;
                ang_vel = imu_from_last_frame[i].ang_vel;
                tstep = current_frame.timestamp - last_timestamp;
            }
            // tstep = tstep * 1e9;

            self.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);

            // ORBSLAM3 imu preintegrated object
            other_imu.imu_preintegrated_from_last_kf.integrate_new_measurement(acc, ang_vel, tstep);
            imu_preintegrated_from_last_frame.integrate_new_measurement(acc, ang_vel, tstep);
        }

        let imu_factor = CombinedImuFactor::new(
            &Symbol::new(b'x', self.ct_state),
            &Symbol::new(b'v', self.ct_state),
            &Symbol::new(b'x', self.ct_state + 1),
            &Symbol::new(b'v', self.ct_state + 1),
            &Symbol::new(b'b', self.ct_state),
            &Symbol::new(b'b', self.ct_state + 1),
            & self.preint_gtsam
        );
        self.graph_new.add_combined_imu_factor(&imu_factor);
        self.graph_main.add_combined_imu_factor(&imu_factor);

        // ORBSLAM3 imu preintegrated object
        // current_frame.imu_data.imu_preintegrated = Some(other_imu.imu_preintegrated_from_last_kf.clone());
        // current_frame.imu_data.imu_preintegrated_frame = Some(imu_preintegrated_from_last_frame);
        // current_frame.imu_data.prev_keyframe = Some(last_kf_id);
        // other_imu.predict_state_last_keyframe(&map, current_frame, last_kf_id)?;

        Ok(())
    }

    fn process_smart_features(&mut self, new_tracked_features: &TrackedFeatures) {
        let _span = tracy_client::span!("process_smart_features");

        for i in 0..new_tracked_features.len() - 1 {
            let feature_id = new_tracked_features.get_feature_id(i as usize);
            if feature_id == -1 {
                continue;
            }
            let point = new_tracked_features.get_point(i as usize);

            // Check to see if it is already in the graph
            match self.measurement_smart_lookup_left.get_mut(&feature_id) {
                Some(smartfactor) => {
                    // Insert measurements to a smart factor
                    smartfactor.add(
                        & gtsam::geometry::point2::Point2::new(point.x as f64, point.y as f64),
                        &Symbol::new(b'x', self.ct_state)
                    );
                    continue;
                },
                None => {
                    // If we know it is not in the graph
                    // Create a smart factor for the new feature
                    let measurement_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(2, self.sigma_camera);
                    let k = gtsam::geometry::cal3_s2::Cal3S2::default();

                    // Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
                    let sensor_p_body = ImuCalib::new().tbc;

                    let mut smartfactor_left = gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2::new(
                        &measurement_noise,
                        &k,
                        & sensor_p_body.inverse().into()
                    );

                    // Insert measurements to a smart factor
                    smartfactor_left.add(
                        & gtsam::geometry::point2::Point2::new(point.x as f64, point.y as f64),
                        &Symbol::new(b'x', self.ct_state)
                    );

                    // Add smart factor to FORSTER2 model
                    self.graph_new.add_smartfactor(&smartfactor_left);
                    self.graph_main.add_smartfactor(&smartfactor_left);

                    self.measurement_smart_lookup_left.insert(feature_id, smartfactor_left);
                }
            }
        }
    }

    fn optimize(&mut self) -> bool {
        let _span = tracy_client::span!("optimize");

        // Return if not initialized
        if matches!(self.solver_state, GraphSolverState::NotInitialized) && self.ct_state < 2 {
            return false;
        }

        // Perform smoothing update
        self.isam2.update_noresults(& self.graph_new, & self.values_new);
        self.values_initial = self.isam2.calculate_estimate().into();

        // Remove the used up nodes
        self.values_new.clear();

        // Remove the used up factors
        self.graph_new.resize(0);

        self.reset_imu_integration();

        return true;
    }

    fn reset_imu_integration(&mut self) {
        // Use the optimized bias to reset integration
        if self.values_initial.exists(&Symbol::new(b'b', self.ct_state)) {
            self.preint_gtsam.reset_integration_and_set_bias(
                & self.values_initial.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
            );
        }
    }
}


#[derive(Debug)]
struct GtsamState {
    pub pose: gtsam::geometry::pose3::Pose3,
    pub velocity: gtsam::base::vector::Vector3,
    pub bias: gtsam::imu::imu_bias::ConstantBias,
}

#[derive(Clone)]
enum GraphSolverState {
    NotInitialized,
    Ok
}