extern crate g2o;
use log::{warn, info, debug, error};
use nalgebra::{Isometry3, Matrix3, Vector3, Vector6};
use std::{collections::{BTreeSet, HashMap, VecDeque}, fmt::Debug, sync::atomic::Ordering, thread::sleep, time::Duration};
use gtsam::{
    inference::symbol::Symbol, navigation::combined_imu_factor::{CombinedImuFactor, PreintegratedCombinedMeasurements, PreintegrationCombinedParams}, nonlinear::{
        isam2::ISAM2, nonlinear_factor_graph::NonlinearFactorGraph, values::Values
    },
};
use core::{
    config::*, matrix::*, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{local_mapping_gtsam::LOCAL_MAPPING_IDLE, messages::{FeatureTracksAndIMUMsg, ShutdownMsg, TrajectoryMsg, UpdateFrameIMUMsg, VisTrajectoryMsg, VisTrajectoryTrackingMsg}, tracking_frontend_gtsam::TrackedFeatures},
    map::{frame::Frame, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::imu::{ImuBias, ImuCalib, ImuMeasurements}, registered_actors::{CAMERA, IMU, SHUTDOWN_ACTOR, TRACKING_FRONTEND, VISUALIZER}, ImuInitializationData
};

pub struct TrackingBackendGTSAM {
    system: System,
    map: ReadWriteMap,

    // Last kf/frame
    // last_kf_pose: Pose,
    // last_kf_imu_bias: ImuBias,
    // last_feature_tracks: TrackedFeatures,
    last_timestamp: Timestamp,

    // Modules 
    graph_solver: GraphSolver,

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
}

impl Actor for TrackingBackendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let mut actor = TrackingBackendGTSAM {
            system,
            graph_solver: GraphSolver::new(),
            // sensor,
            map,
            trajectory_poses: Vec::new(),
            last_timestamp: 0.0,
            // last_kf_pose: Pose::default(),
            // last_kf_imu_bias: ImuBias::new(),
            // last_feature_tracks: TrackedFeatures::default(),
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

        if matches!(self.graph_solver.solver_state, GraphSolverState::NotInitialized) {
            // First frame received here... was already put into local mapping map as latest keyframe
            // Need to initialize the factor graph 

            // Initialize from gt
            // let imu_init = msg.imu_initialization.expect("Msg should have imu initialization data!");
            // self.graph_solver.initialize_with_data(
            //     msg.frame.timestamp,
            //     &imu_init
            // ).expect("Failed to initialize?");

            // self.graph_solver.solver_state = GraphSolverState::Ok;
            // self.last_timestamp = msg.frame.timestamp;
            // debug!("Initialization timestamp: {:?}", msg.frame.timestamp);

            // Initialize from map initialization
            self.scale_map_from_imu(&mut msg)?;

            let (kf1_timestamp, kf1_scaled_pose, kf1_scaled_velocity) = {
                let map = self.map.read()?;
                let kf1 = map.get_keyframe(1);
                let kf1_pose = kf1.get_pose();
                let kf1_velocity = kf1.imu_data.velocity.unwrap();
                (kf1.timestamp, kf1_pose, kf1_velocity)
            };
            let init_bias = {
                let imu_init = msg.imu_initialization.as_ref().expect("Msg should have imu initialization data!");
                ImuBias {
                    bax: imu_init.acc_bias[0],
                    bay: imu_init.acc_bias[1],
                    baz: imu_init.acc_bias[2],
                    bwx: imu_init.gyro_bias[0],
                    bwy: imu_init.gyro_bias[1],
                    bwz: imu_init.gyro_bias[2]
                }
            };

            // Now that the map is scaled, re-initialize the graph solver
            // Note (frames): Kf1 pose is Tcw, initialize graph solver with tbw
            let pose_for_init = {
                let tcw = kf1_scaled_pose;
                let tbc = ImuCalib::new().tbc;
                let tbw = tbc * tcw;
                tbw
            };
            debug!("Sofiya! Initial pose for graph solver: {:?}", pose_for_init);
            debug!("Sofiya! Initial pose for graph solver original: {:?}", kf1_scaled_pose);
            debug!("Sofiya! Initial velocity for graph solver: {:?}", kf1_scaled_velocity);
            
            self.graph_solver = GraphSolver::new();
            self.graph_solver.initialize(
                kf1_timestamp,
                pose_for_init,
                kf1_scaled_velocity,
                init_bias
            ).expect("Failed to initialize?");

            self.graph_solver.solver_state = GraphSolverState::Ok;
            self.last_timestamp = kf1_timestamp;

        } else {
            // If we have previous frames already, can track normally
            let mut current_frame = msg.frame;

            // Solve VIO graph. Includes preintegration
            let optimized = self.graph_solver.solve(
                &mut current_frame,
                &mut msg.imu_measurements, &msg.feature_tracks
            )?;
            if !optimized {
                warn!("Could not optimize graph");
            }

            self.create_new_keyframe(&mut current_frame).expect("Could not create new keyframe");

            self.update_trajectory_in_logs(& current_frame).expect("Could not save trajectory");
            self.last_timestamp = current_frame.timestamp;

        }

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
        // self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
        //     pose: current_frame.pose.unwrap(),
        //     mappoint_matches: vec![],
        //     mappoints_in_tracking: BTreeSet::new(),
        //     timestamp: current_frame.timestamp,
        //     map_version: map.version
        // }));
        self.system.try_send(VISUALIZER, Box::new(VisTrajectoryTrackingMsg{
            pose: current_frame.pose.unwrap(),
            timestamp: current_frame.timestamp,
            map_version: map.version
        }));


        Ok(())
    }

    fn scale_map_from_imu(&mut self, msg: &mut FeatureTracksAndIMUMsg) -> Result<(), Box<dyn std::error::Error>> {
        let (scale, velocity) = {
            let map = self.map.read()?;
            let kf0 = map.get_keyframe(0);
            let kf1 = map.get_keyframe(1);
            let kf0_pose = kf0.get_pose();
            let kf1_pose = kf1.get_pose();
            let velocity = DVVector3::new(
                (*kf1_pose.get_translation() - *kf0_pose.get_translation()) / (kf1.timestamp - kf0.timestamp)
            );
        //     // Note (frames): Initial kf1 pose here is Tcw

        //     println!("KF 0 pose: {:?}", kf0_pose);
        //     println!("KF 1 pose: {:?}", kf1_pose);
        //     println!("KF 0 timestamp: {:?}", kf0.timestamp);
        //     println!("KF 1 timestamp: {:?}", kf1.timestamp);
        //     println!("Velocity: {:?}", velocity);

        //     // Initialize with all 0s, as if we are the first keyframe
        //     // Use the imu measurements to predict what kf 1's pose should be
        //     let init_bias = {
        //         let imu_init = msg.imu_initialization.as_ref().expect("Msg should have imu initialization data!");
        //         ImuBias {
        //             bax: imu_init.acc_bias[0],
        //             bay: imu_init.acc_bias[1],
        //             baz: imu_init.acc_bias[2],
        //             bwx: imu_init.gyro_bias[0],
        //             bwy: imu_init.gyro_bias[1],
        //             bwz: imu_init.gyro_bias[2]
        //         }
        //     };
        //     self.graph_solver.initialize(
        //         kf0.timestamp,
        //         kf0_pose,
        //         DVVector3::new_with(0.0, 0.0, 0.0),
        //         init_bias
        //     ).expect("Failed to initialize?");
        //     self.graph_solver.preintegrate(
        //         &mut msg.imu_measurements,
        //         kf1.timestamp,
        //         kf0.timestamp
        //     ).expect("Could not preintegrate!");

        //     let predicted_pose = {
        //         // Note (frames): Predicted state should be Tbw or Twb (assuming Tbw)
        //         let p1 = self.graph_solver.predict_state();

        //         // This is all just to convert back into a type we can easily use
        //         let p2: Isometry3<f64> = (&p1.pose).into();
        //         let p3 = Pose::new_from_isometry(p2);

        //         println!("PREDICTED POSITION: {:?}", p3);
        //         println!("PREDICTED POSITION inverse: {:?}", p3.inverse());

        //         // Note (frames): Convert Tbw to Tcw (the regular pose saved in keyframe)
        //         let tcw = ImuCalib::new().tcb * p3; // Tbw -> Tcw

        //         println!("PREDICTED POSITION Tcw: {:?}", tcw);

        //         tcw
        //     };

            // Using predicted translation, calculate scale of the new map
        //     let predicted_distance = Self::distance(&kf0_pose.get_translation(), &predicted_pose.get_translation());
        //     let initialized_distance = Self::distance(&kf0_pose.get_translation(), &kf1_pose.get_translation());
        //     let scale = predicted_distance / initialized_distance;
        //     println!("PREDICTED DISTANCE: {:4}", predicted_distance);
        //     println!("INITIALIZED DISTANCE: {:4}", initialized_distance);
        //     println!("SCALE: {:3}", scale);
            let scale = 5.5914506912231445;
            (scale, velocity)
        };

        {
            let mut map = self.map.write()?;
            map.get_keyframe_mut(1).imu_data.velocity = Some(velocity);
            map.get_keyframe_mut(0).imu_data.velocity = Some(DVVector3::new_with(0.0, 0.0, 0.0));
            let t = Pose::new(
                Vector3::new(0.0, 0.0, 0.0),
                Matrix3::new(
                0.99650955200195312, 0.074449501931667328, 0.037762001156806946,
                0.061686959117650986, -0.35195067524909973, -0.93398362398147583,
                -0.056244250386953354, 0.93305301666259766, -0.35531479120254517
                )
            );
            map.apply_scaled_rotation(&t, scale,true);
        }

        Ok(())
    }

    fn create_new_keyframe(&mut self, current_frame: &mut Frame) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("create_new_keyframe");
        //CreateNewKeyFrame
        // Ref code: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Tracking.cc#L3216
        let new_frame = Frame::new_clone(& current_frame);
        let new_kf_id = self.map.write()?.insert_keyframe_to_map(new_frame, false);

        // //Reset preintegration from last KF (Create new object)
        // self.imu.imu_preintegrated_from_last_kf = ImuPreIntegrated::new(current_frame.imu_data.get_imu_bias());

        current_frame.ref_kf_id = Some(new_kf_id);
        // self.ref_kf_id = Some(new_kf_id);

        tracy_client::Client::running()
        .expect("message! without a running Client")
        .message("create new keyframe", 2);

        println!("Created new keyframe with pose {:?}", current_frame.pose.unwrap());

        // SOFIYA TURN OFF LOCAL MAPPING
        // println!("TRACKING BACKEND SEND TO LOCAL MAPPING");

        // KeyFrame created here and inserted into map
        // self.system.send(
        //     LOCAL_MAPPING,
        //     Box::new( NewKeyFrameGTSAMMsg{
        //         tracking_state: TrackingState::Ok,
        //         keyframe: current_frame,
        //         map_version: self.map.read()?.version
        //     } )
        // );
    

        Ok(())
    }

    fn distance(v1: &Vector3<f64>, v2: &Vector3<f64>) -> f64 {
        let dx = v2.x - v1.x;
        let dy = v2.y - v1.y;
        let dz = v2.z - v1.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }
}

pub struct GraphSolver {
    solver_state: GraphSolverState,

    // New graph
    graph_new: NonlinearFactorGraph, // New factors that have not been optimized yet
    values_new: Values, // New values that have not been optimized yet

    // Main graph
    // graph_main: NonlinearFactorGraph, // Main non-linear GTSAM graph, all created factors
    values_all: Values, // All created nodes

    // Misc GTSAM objects
    isam2: ISAM2, // ISAM2 solvers
    preint_gtsam: PreintegratedCombinedMeasurements, // IMU preintegration

    // Initialization
    accel_noise_density: f64, // accelerometer_noise_density, sigma_a
    gyro_noise_density: f64, // gyroscope_noise_density, sigma_g
    accel_random_walk: f64, // accelerometer_random_walk, sigma_wa
    gyro_random_walk: f64, // gyroscope_random_walk, sigma_wg
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

    last_timestamp: Timestamp
}

impl GraphSolver {
    pub fn new() -> Self {
        Self {
            graph_new: NonlinearFactorGraph::default(),
            values_new: Values::default(),
            // graph_main: NonlinearFactorGraph::default(),
            values_all: Values::default(),
            isam2: ISAM2::default(),
            preint_gtsam: PreintegratedCombinedMeasurements::default(),
            solver_state: GraphSolverState::NotInitialized,

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

            last_timestamp: 0.0
        }
    }

    fn initialize_with_data(&mut self, timestamp: f64, imu_init: &ImuInitializationData) -> Result<(), Box<dyn std::error::Error>> {
        let init_pose = Pose::new_with_quaternion_convert(*imu_init.translation, imu_init.rotation);
        let init_bias = ImuBias::new_with(imu_init.gyro_bias, imu_init.acc_bias);
        self.initialize(timestamp, init_pose, imu_init.velocity, init_bias)
    }

    fn initialize(&mut self, timestamp: f64, init_pose: Pose, init_vel: DVVector3<f64>, init_bias: ImuBias) -> Result<(), Box<dyn std::error::Error>> {
        let timestamp_converted = (timestamp * 1e9) as i64;
        println!("... Initial pose: {:?}", init_pose);
        println!("... Initial velocity: {:?}", init_vel);
        println!("... Initial bias: {:?}", init_bias);

        // Create prior factor and add it to the graph
        let prior_state = {
            let trans = init_pose.translation;
            let rot = init_pose.get_quaternion();
            GtsamState {
                pose: gtsam::geometry::pose3::Pose3::from_parts(
                    gtsam::geometry::point3::Point3::new(trans.x, trans.y, trans.z),
                    gtsam::geometry::rot3::Rot3::from(rot)
                ),
                velocity: gtsam::base::vector::Vector3::new(init_vel[0], init_vel[1], init_vel[2]),
                bias: gtsam::imu::imu_bias::ConstantBias::new(
                    &gtsam::base::vector::Vector3::new(init_bias.bax, init_bias.bay, init_bias.baz),
                    &gtsam::base::vector::Vector3::new(init_bias.bwx, init_bias.bwy, init_bias.bwz)
                )
            }
        };

        // Set initial pose uncertainty: constrain mainly position and global yaw.
        // roll and pitch is observable, therefore low variance.

        let mut pose_prior_covariance2 = nalgebra::Matrix3::zeros();
        pose_prior_covariance2[(0, 0)] = self.initial_roll_pitch_sigma * self.initial_roll_pitch_sigma;
        pose_prior_covariance2[(1, 1)] = self.initial_roll_pitch_sigma * self.initial_roll_pitch_sigma;
        pose_prior_covariance2[(2, 2)] = self.initial_yaw_sigma * self.initial_yaw_sigma;

        // Rotate initial uncertainty into local frame, where the uncertainty is
        // specified.
        let b_rot_w = init_pose.get_rotation().transpose();
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
        let pose_noise = gtsam::linear::noise_model::GaussianNoiseModel::from_covariance(pose_prior_covariance);
        self.graph_new.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);

        let v_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(3, self.initial_velocity_sigma);
        self.graph_new.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);

        let b_noise = gtsam::linear::noise_model::DiagonalNoiseModel::from_sigmas(Vector6::new(
            self.initial_acc_bias_sigma,
            self.initial_acc_bias_sigma,
            self.initial_acc_bias_sigma,
            self.initial_gyro_bias_sigma,
            self.initial_gyro_bias_sigma,
            self.initial_gyro_bias_sigma,
        ));
        self.graph_new.add_prior_factor_constant_bias_diagonal(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);

        // self.graph_main.add_prior_factor_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose, &pose_noise);
        // self.graph_main.add_prior_factor_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);
        // self.graph_main.add_prior_factor_constant_bias_diagonal(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);

        // Add initial state to the graph
        self.values_new.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
        self.values_new.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
        self.values_new.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);
        self.values_all.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
        self.values_all.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
        self.values_all.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);

        // Add ct state to map
        self.ct_state_lookup.insert(timestamp_converted, self.ct_state);
        self.timestamp_lookup.insert(self.ct_state, timestamp_converted);

        // Create GTSAM preintegration parameters for use with Foster's version
        let mut params = PreintegrationCombinedParams::makesharedu(); // Note (frames): Z up
        params.set_gyroscope_covariance(self.gyro_noise_density * self.gyro_noise_density);
        params.set_accelerometer_covariance(self.accel_noise_density * self.accel_noise_density);
        params.set_integration_covariance(self.imu_integration_sigma * self.imu_integration_sigma);
        params.set_bias_acc_omega_int(self.init_bias_sigma);
        params.set_bias_acc_covariance(self.accel_random_walk * self.accel_random_walk);
        params.set_bias_omega_covariance(self.gyro_random_walk * self.gyro_random_walk);

        // Actually create the GTSAM preintegration
        self.preint_gtsam = PreintegratedCombinedMeasurements::new(params, &prior_state.bias);

        self.last_timestamp = timestamp;

        Ok(())
    }

    fn solve(&mut self,
        current_frame : &mut Frame, 
        imu_measurements : &mut ImuMeasurements, new_tracked_features : &TrackedFeatures
    ) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("solve");

        let timestamp = (current_frame.timestamp * 1e9) as i64; // Convert to int just so we can hash it

        // Return if the node already exists in the graph
        if self.ct_state_lookup.contains_key(&timestamp) {
            warn!("NODE WITH TIMESTAMP {} ALREADY EXISTS", timestamp);
            return Ok(false);
        }

        self.preintegrate(imu_measurements, current_frame.timestamp, self.last_timestamp)?;
        self.create_imu_factor();

        // Original models
        let new_state = self.predict_state();

        // Append to our node vectors
        self.values_new.insert_pose3(
            &Symbol::new(b'x', self.ct_state + 1),
            &new_state.pose
        );
        self.values_new.insert_vector3(
            &Symbol::new(b'v', self.ct_state + 1),
            &new_state.velocity
        );
        self.values_new.insert_constant_bias(
            &Symbol::new(b'b', self.ct_state + 1),
            &new_state.bias
        );
        self.values_all.insert_pose3(
            &Symbol::new(b'x', self.ct_state + 1),
            &new_state.pose
        );
        self.values_all.insert_vector3(
            &Symbol::new(b'v', self.ct_state + 1),
            &new_state.velocity
        );
        self.values_all.insert_constant_bias(
            &Symbol::new(b'b', self.ct_state + 1),
            &new_state.bias
        );

        // Add ct state to map
        self.ct_state_lookup.insert(timestamp, self.ct_state + 1);
        self.timestamp_lookup.insert(self.ct_state + 1, timestamp);

        self.process_smart_features(new_tracked_features);

        debug!("IMU POSE ESTIMATE... {}, {:?}, {:?}, {:?}", timestamp, new_state.pose, new_state.velocity, new_state.bias);

        self.optimize();

        // debug!("OPTIMIZATION COVARIANCE: {:?}", self.isam2.get_marginal_covariance(&Symbol::new(b'x', self.ct_state + 1)));

        // Update frame with optimized values
        // Note (frames): Result should be Tbw or Twb, not sure which one
        let updated_pose: Isometry3<f64> = self.values_all.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into();
        let velocity: gtsam::base::vector::Vector3 = self.values_all.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into();
        let vel_raw = velocity.get_raw();

        let reg = Pose::new_from_isometry(updated_pose);
        let inv = Pose::new_from_isometry(updated_pose.inverse());
        println!("Updated pose inverse: {:?}", inv.get_translation());
        println!("Updated pose regular: {:?}", reg.get_translation());

        // Note (frames): Have a look inside this function. It takes Twb and sets the frame pose to Tcw
        current_frame.set_imu_pose_velocity(
            Pose::new_from_isometry(updated_pose.inverse()),
            Vector3::new(vel_raw[0], vel_raw[1], vel_raw[2])
        );


        let bias_ref = self.values_all.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap();
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

        debug!("OPTIMIZED POSE ESTIMATE... {}, {:?}, {:?}, {:?}", timestamp, updated_pose, velocity, current_frame.imu_data.imu_bias);

        self.last_timestamp = current_frame.timestamp;

        // Move node count forward in time
        self.ct_state += 1;

        Ok(true)
    }


    fn predict_state(&self) -> GtsamState {
        let _span = tracy_client::span!("get_predicted_state");
        // This function will get the predicted state based on the IMU measurement

        // Get the current state (t=k)
        let state_k = GtsamState {
            pose: self.values_all.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into(),
            velocity: self.values_all.get_vector3(&Symbol::new(b'v', self.ct_state)).unwrap().into(),
            bias: self.values_all.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
        };

        debug!("Current state used for imu-based prediction: {:?}", state_k);

        // From this we should predict where we will be at the next time (t=K+1)
        let state_k1 = self.preint_gtsam.predict(
            &gtsam::navigation::navstate::NavState::new(
                &state_k.pose,
                &state_k.velocity
            ),
            &state_k.bias
        );

        // debug!("IMU COVARIANCE: {:?}", self.preint_gtsam.get_covariance());

        // Note (frames): state_k1 seems to be Twb or Tbw

        let predicted = GtsamState {
            pose: state_k1.get_pose().into(),
            velocity: state_k1.get_velocity().into(),
            bias: state_k.bias
        };

        return predicted;
    }

    fn preintegrate(&mut self, imu_measurements: &mut ImuMeasurements, current_timestamp: Timestamp, last_timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // This function will create a discrete IMU factor using the GTSAM preintegrator class
        // This will integrate from the current state time up to the new update time
        let _span = tracy_client::span!("create_imu_factor");

        // From ORBSLAM:
        let mut imu_from_last_frame = VecDeque::with_capacity(imu_measurements.len()); // mvImuFromLastFrame
        let imu_per = 0.001;
        while !imu_measurements.is_empty() {
            if imu_measurements.front().unwrap().timestamp < last_timestamp - imu_per {
                imu_measurements.pop_front();
            } else if imu_measurements.front().unwrap().timestamp < current_timestamp - imu_per {
                let msmt = imu_measurements.pop_front().unwrap();
                imu_from_last_frame.push_back(msmt);
            } else {
                let msmt = imu_measurements.pop_front().unwrap();
                imu_from_last_frame.push_back(msmt);
                break;
            }
        }
        let n = imu_from_last_frame.len() - 1;

        // ORBSLAM3 imu preintegrated object. If not doing imu-based optimizations in local mapping and loop closure, don't need this
        // let mut other_imu = IMU::new();
        // let mut imu_preintegrated_from_last_frame = ImuPreIntegrated::new(last_frame_imu_bias);

        // for i in 0..imu_measurements.len()-1 {
        // Commented out code here from ORBSLAM3 as well
        for i in 0..n {
            // let mut tstep = imu_measurements[i + 1].timestamp - imu_measurements[i].timestamp;
            // let mut acc: Vector3<f64> = imu_measurements[i].acc; // acc
            // let mut ang_vel: Vector3<f64> = imu_measurements[i].ang_vel; // angVel
            let mut tstep = 0.0;
            let mut acc: Vector3<f64> = Vector3::zeros(); // acc
            let mut ang_vel: Vector3<f64> = Vector3::zeros(); // angVel

            // orbslam:
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
                let tend = imu_from_last_frame[i + 1].timestamp - current_timestamp;
                acc = (
                    imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
                    (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tend/tab)
                ) * 0.5;
                ang_vel = (
                    imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
                    (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tend/tab)
                ) * 0.5;
                tstep = current_timestamp - imu_from_last_frame[i].timestamp;
            } else if i == 0 && i == (n - 1) {
                acc = imu_from_last_frame[i].acc;
                ang_vel = imu_from_last_frame[i].ang_vel;
                tstep = current_timestamp - last_timestamp;
            }
            // tstep = tstep * 1e2;

            self.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);
            // println!("Tstep: {:?}", tstep);

            // ORBSLAM3 imu preintegrated object
            // other_imu.imu_preintegrated_from_last_kf.integrate_new_measurement(acc, ang_vel, tstep);
            // imu_preintegrated_from_last_frame.integrate_new_measurement(acc, ang_vel, tstep);
        }
        Ok(())
    }

    fn create_imu_factor(&mut self) {
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
        // self.graph_main.add_combined_imu_factor(&imu_factor);

        // ORBSLAM3 imu preintegrated object
        // current_frame.imu_data.imu_preintegrated = Some(other_imu.imu_preintegrated_from_last_kf.clone());
        // current_frame.imu_data.imu_preintegrated_frame = Some(imu_preintegrated_from_last_frame);
        // current_frame.imu_data.prev_keyframe = Some(last_kf_id);
        // other_imu.predict_state_last_keyframe(&map, current_frame, last_kf_id)?;
    }

    fn process_smart_features(&mut self, new_tracked_features: &TrackedFeatures) {
        let _span = tracy_client::span!("process_smart_features");

        println!("New tracked features length: {}", new_tracked_features.len());
        let mut id_notset = 0;
        let mut added_new = 0;
        let mut added_old = 0;

        for i in 0..new_tracked_features.len() - 1 {
            let feature_id = new_tracked_features.get_feature_id(i as usize);
            if feature_id == -1 {
                id_notset += 1;
                continue;
            }
            let point = new_tracked_features.get_point(i as usize);

            // Check to see if it is already in the graph
            // match self.measurement_smart_lookup_left.get_mut(&feature_id) {
            //     Some(smartfactor) => {
            //         // Insert measurements to a smart factor
            //         smartfactor.add(
            //             & gtsam::geometry::point2::Point2::new(point.x as f64, point.y as f64),
            //             &Symbol::new(b'x', self.ct_state + 1)
            //         );
            //         added_old += 1;
            //         continue;
            //     },
            //     None => {
                    // If we know it is not in the graph
                    // Create a smart factor for the new feature
                    let measurement_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(2, self.sigma_camera);
                    let k = gtsam::geometry::cal3_s2::Cal3S2::new(
                        SETTINGS.get::<f64>(CAMERA, "fx"),
                        SETTINGS.get::<f64>(CAMERA, "fy"),
                        0.0,
                        SETTINGS.get::<f64>(CAMERA, "cx"),
                        SETTINGS.get::<f64>(CAMERA, "cy"),
                    );

                    // Note (frames): Transformation from camera frame to imu frame, i.e., pose of imu frame in camera frame
                    let transform = ImuCalib::new().tbc;

                    let mut smartfactor_left = gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2::new(
                        &measurement_noise,
                        &k,
                        & transform.into()
                    );

                    // Insert measurements to a smart factor
                    smartfactor_left.add(
                        & gtsam::geometry::point2::Point2::new(point.x as f64, point.y as f64),
                        &Symbol::new(b'x', self.ct_state + 1)
                    );

                    // Add smart factor to FORSTER2 model
                    self.graph_new.add_smartfactor(&smartfactor_left);
                    // self.graph_main.add_smartfactor(&smartfactor_left);

                    self.measurement_smart_lookup_left.insert(feature_id, smartfactor_left);

                    added_new += 1;
                // }
            // }
        }
        println!("Added new features: {}, added old features: {}, id not set: {}", added_new, added_old, id_notset);
    }

    fn optimize(&mut self) {
        let _span = tracy_client::span!("optimize");

        // Perform smoothing update
        self.isam2.update_noresults(& self.graph_new, & self.values_new);
        self.values_all = self.isam2.calculate_estimate().into();

        self.clean_up();
    }

    fn clean_up(&mut self) {
        // Remove the used up nodes
        self.values_new.clear();

        // Remove the used up factors
        self.graph_new.resize(0);

        // Use the optimized bias to reset integration
        if self.values_all.exists(&Symbol::new(b'b', self.ct_state + 1)) {
            self.preint_gtsam.reset_integration_and_set_bias(
                & self.values_all.get_constantbias(&Symbol::new(b'b', self.ct_state + 1)).unwrap().into()
            );
        } else {
            warn!("Bias wasn't optimized?");
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