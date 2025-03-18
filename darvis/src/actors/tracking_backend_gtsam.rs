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
    actors::messages::{ShutdownMsg, UpdateFrameIMUMsg, VisFeaturesMsg}, map::{features::Features, frame::Frame, keyframe::{KeyFrame, MapPointMatches}, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{geometric_tools, good_features_to_track::GoodFeaturesExtractor, image, imu::{ImuBias, ImuCalib, ImuMeasurements, ImuPreIntegrated, IMU}, map_initialization::MapInitialization, module_definitions::{CameraModule, FeatureExtractionModule, ImuModule}, optimizer::{self, LEVEL_SIGMA2}, orbslam_extractor::ORBExtractor, orbslam_matcher::SCALE_FACTORS, relocalization::Relocalization}, registered_actors::{CAMERA_MODULE, FEATURE_DETECTION, FEATURE_MATCHER, FEATURE_MATCHING_MODULE, LOCAL_MAPPING, SHUTDOWN_ACTOR, TRACKING_BACKEND, VISUALIZER}
};

use super::{messages::{FeatureTracksAndIMUMsg, NewKeyFrameGTSAMMsg, NewKeyFrameMsg, TrajectoryMsg, VisTrajectoryMsg}, tracking_backend::TrackingState};
use crate::registered_actors::IMU;
use std::collections::BTreeSet;

pub struct TrackingBackendGTSAM {
    system: System,
    map: ReadWriteMap,
    sensor: Sensor,

    // Frames
    last_kf_id: Id,
    last_kf_pose: Pose,
    current_kf_id: Id,
    prev_timestamp: Timestamp,

    // Modules 
    imu: IMU,
    graph_solver: GraphSolver,

    // Poses in trajectory
    trajectory_poses: Vec<Pose>, //mlRelativeFramePoses
}

impl Actor for TrackingBackendGTSAM {
    type MapRef = ReadWriteMap;

    fn spawn(system: System, map: Self::MapRef) {
        let sensor = SETTINGS.get::<Sensor>(SYSTEM, "sensor");
        let imu = IMU::new();

        let mut actor = TrackingBackendGTSAM {
            system,
            graph_solver: GraphSolver::new(),
            sensor,
            map,
            imu,
            trajectory_poses: Vec::new(),
            last_kf_id: -1,
            current_kf_id: -1,
            prev_timestamp: 0.0,
            last_kf_pose: Pose::default(),
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
            // If this is the first frame, don't do anything
            // If we are receiving it here, it was already put into the map as the latest keyframe
            self.last_kf_id += 1;
            self.last_kf_pose = msg.frame.pose.unwrap();
        } else {
            // If we have previous frames already, can track normally
            let mut current_frame = msg.frame;
            if self.last_kf_id != -1 {
                let lock = self.map.read()?;
                let ref_kf = lock.get_keyframe(self.last_kf_id);
                current_frame.imu_data.set_new_bias(ref_kf.imu_data.get_imu_bias());
            }

            // Solve VIO graph. Includes preintegration
            let prev_state_uninit = matches!(self.graph_solver.solver_state, GraphSolverState::NotInitialized);
            let optimized = self.graph_solver.solve(&mut current_frame, self.last_kf_id, &mut msg.imu_measurements, & msg.feature_ids, & self.map)?;
            if !optimized {
                warn!("Could not optimize graph");
            }
            let graph_just_initialized = prev_state_uninit && matches!(self.graph_solver.solver_state, GraphSolverState::Ok);

            current_frame.ref_kf_id = Some(self.last_kf_id);
            self.update_trajectory_in_logs(& current_frame).expect("Could not save trajectory");
            self.prev_timestamp = current_frame.timestamp;
            self.last_kf_pose = current_frame.pose.unwrap();
            self.last_kf_id += 1;

            if graph_just_initialized {
                debug!("CURRENT KF ID: {}, LAST KF ID: {}", self.current_kf_id, self.last_kf_id);
                // If graph was just initialized then it was already sent to local mapping through initialization in frontend
            } else {
                debug!("CURRENT KF ID: {}, LAST KF ID: {}", self.current_kf_id, self.last_kf_id);

                println!("TRACKING BACKEND SEND TO LOCAL MAPPING");
                // KeyFrame created here and inserted into map
                // self.system.send(
                //     LOCAL_MAPPING,
                //     Box::new( NewKeyFrameGTSAMMsg{
                //         tracking_state: TrackingState::Ok,
                //         feature_tracks: msg.feature_ids,
                //         keyframe: current_frame,
                //         tracked_mappoint_depths: HashMap::new(), //self.track_in_view.iter().map(|(k, v)| (*k, v.track_depth)).collect(),
                //         map_version: self.map.read()?.version
                //     } )
                // );
            }
        }

        return Ok(());
    }

    fn update_trajectory_in_logs(
        &mut self, current_frame: &Frame,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let map = self.map.read()?;
        let last_kf = map.get_keyframe(self.last_kf_id);

        let relative_pose = current_frame.pose.unwrap() * last_kf.get_pose();

        self.trajectory_poses.push(relative_pose);

        // debug!("Sanity check... curr frame is {}, ref kf is {:?}, last kf is {}", current_frame.frame_id, current_frame.ref_kf_id, last_kf.id);

        self.system.send(
            SHUTDOWN_ACTOR, 
            Box::new(TrajectoryMsg{
                pose: current_frame.pose.unwrap().inverse(),
                ref_kf_id: current_frame.ref_kf_id.unwrap(),
                timestamp: current_frame.timestamp,
                map_version: self.map.read()?.version
            })
        );

        let map = self.map.read()?;
        let bla = last_kf.get_mp_matches().iter().filter_map(|v| match v { 
           Some((id, _is_outlier)) => Some(*id),
           None => None
        });
        self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
            pose: current_frame.pose.unwrap(),
            mappoint_matches: vec![],
            nontracked_mappoints: HashMap::new(),
            mappoints_in_tracking: bla.collect(),
            timestamp: current_frame.timestamp,
            map_version: map.version
        }));

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
    // prior_q_g_to_i: Quaternion<f64>, // prior_qGtoI
    // prior_p_i_in_g: Vector3<f64>, // prior_pIinG
    // prior_v_i_in_g: [f64; 3], // prior_vIinG
    prior_ba: [f64; 3], // prior_ba
    prior_bg: [f64; 3], // prior_bg
    // Noise values from dataset sensor
    accel_noise_density: f64, // accelerometer_noise_density, sigma_a
    gyro_noise_density: f64, // gyroscope_noise_density, sigma_g
    accel_random_walk: f64, // accelerometer_random_walk, sigma_wa
    gyro_random_walk: f64, // gyroscope_random_walk, sigma_wg
    // Noise values for initialization
    sigma_prior_rotation: [f64; 3],
    sigma_prior_translation: [f64; 3],
    sigma_velocity: f64,
    sigma_bias: f64,
    // Misc
    sigma_camera: f64,

    // Iterations of the map
    ct_state: u64,
    ct_state_lookup: HashMap<i64, u64>,
    timestamp_lookup: HashMap<u64, i64>,
    measurement_smart_lookup_left: HashMap<i32, gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2>, // Smart lookup for IMU measurements
}

impl GraphSolver {
    pub fn new() -> Self {
        //           <!-- Rotation and Translation from camera frame to IMU frame -->
        //   <rosparam param="R_C0toI">[0.9999717314190615,    -0.007438121416209933,  0.001100323844221122,
        //                              0.00743200596269379,    0.9999574688631824,    0.005461295826837418,
        //                             -0.0011408988276470173, -0.0054529638303831614, 0.9999844816472552]</rosparam>
        //   <rosparam param="p_IinC0">[-0.03921656415229387,   0.00621263233002485,   0.0012210059575531885]</rosparam>

        //   <!-- Initialization -->
        //   <rosparam param="prior_qGtoI">[0.716147, 0.051158, 0.133778, 0.683096]</rosparam>       
        //   <rosparam param="prior_pIinG">[0.925493, -6.214668, 0.422872]</rosparam>       
        //   <rosparam param="prior_vIinG">[1.128364, -2.280640, 0.213326]</rosparam>       
        //   <rosparam param="prior_ba">[0.00489771688759235973,  0.00800897351104824969, 0.03020588299505782420]</rosparam>       
        //   <rosparam param="prior_bg">[-0.00041196751646696610, 0.00992948457018005999, 0.02188282212555122189]</rosparam>       

        //   <param name="sigma_camera"                 type="double"   value="0.306555403" />
        
        //   <!-- Noise Values for ETH Dataset Sensor -->
        //   <param name="accelerometer_noise_density"  type="double"   value="0.08" />    <!-- sigma_a -->
        //   <param name="gyroscope_noise_density"      type="double"   value="0.004" />   <!-- sigma_g -->
        //   <param name="accelerometer_random_walk"    type="double"   value="0.00004" /> <!-- sigma_wa -->
        //   <param name="gyroscope_random_walk"        type="double"   value="2.0e-6" />  <!-- sigma_wg -->

        //   <!-- Noise Values for Initialization -->
        //   <param name="sigma_prior_rotation"         type="double"   value="0.1" />
        //   <param name="sigma_prior_translation"      type="double"   value="0.3" />
        //   <param name="sigma_velocity"               type="double"   value="0.1" />
        //   <param name="sigma_bias"                   type="double"   value="0.15" />
        //   <param name="sigma_pose_rotation"          type="double"   value="0.1" />
        //   <param name="sigma_pose_translation"       type="double"   value="0.2" />

        Self {
            graph_new: NonlinearFactorGraph::default(),
            values_new: Values::default(),
            graph_main: NonlinearFactorGraph::default(),
            values_initial: Values::default(),
            isam2: ISAM2::default(),
            preint_gtsam: PreintegratedCombinedMeasurements::default(),
            solver_state: GraphSolverState::NotInitialized,

            // TODO SOFIYA PARAMS
            sigma_prior_rotation: [0.1, 0.1, 0.1],
            sigma_prior_translation: [0.3, 0.3, 0.3],
            sigma_velocity: 0.1,
            sigma_bias: 0.15,
            sigma_camera: 0.306555403,
            accel_noise_density: SETTINGS.get::<f64>(IMU, "noise_acc"),
            gyro_noise_density: SETTINGS.get::<f64>(IMU, "noise_gyro"),
            accel_random_walk: SETTINGS.get::<f64>(IMU, "acc_walk"),
            gyro_random_walk: SETTINGS.get::<f64>(IMU, "gyro_walk"),

            // TODO SOFIYA IS THIS RIGHT?
            prior_ba: [1e-3, 1e-3, 1e-3],
            prior_bg: [1e-5, 1e-3, 1e-3],

            ct_state: 0,
            ct_state_lookup: HashMap::new(),
            timestamp_lookup: HashMap::new(),
            measurement_smart_lookup_left: HashMap::new(),
        }
    }

    fn solve(&mut self, current_frame: &mut Frame, last_kf_id: Id, imu_measurements: &mut ImuMeasurements, feature_ids: & Vec<i32>, map: &ReadWriteMap) -> Result<bool, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("solve");

        let timestamp = (current_frame.timestamp * 1e9) as i64; // Convert to int just so we can hash it

        // Return if the node already exists in the graph
        if self.ct_state_lookup.contains_key(&timestamp) {
            warn!("NODE WITH TIMESTAMP {} ALREADY EXISTS", timestamp);
            return Ok(false);
        }

        if matches!(self.solver_state, GraphSolverState::NotInitialized) {
            self.add_initials_and_priors(timestamp, map)?;
            self.solver_state = GraphSolverState::Ok;
        } else {
            self.create_imu_factor(imu_measurements, current_frame, last_kf_id, map)?;

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

        self.process_smart_features(current_frame, feature_ids);

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

    fn add_initials_and_priors(&mut self, timestamp: i64, map: &ReadWriteMap) -> Result<(), Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("add_initials_and_priors");

        // Create prior factor and add it to the graph
        let prior_state = {
            let map = map.read()?;
            let first_kf = map.get_keyframe(1);
            let trans = first_kf.get_pose().translation;
            let rot = first_kf.get_pose().get_quaternion();
            // let vel = first_kf.imu_data.velocity.unwrap();
            let vel = [0.0, 0.0, 0.0];
            GtsamState {
                pose: gtsam::geometry::pose3::Pose3::from_parts(
                    gtsam::geometry::point3::Point3::new(trans.x, trans.y, trans.z),
                    gtsam::geometry::rot3::Rot3::from(rot)
                ),
                velocity: gtsam::base::vector::Vector3::new(vel[0], vel[1], vel[2]),
                bias: gtsam::imu::imu_bias::ConstantBias::new(
                    &gtsam::base::vector::Vector3::new(self.prior_ba[0], self.prior_ba[1], self.prior_ba[2]),
                    &gtsam::base::vector::Vector3::new(self.prior_bg[0], self.prior_bg[1], self.prior_bg[2])
                )
            }
        };

        // TODO SOFIYA  where are these coming from
        let pose_noise = gtsam::linear::noise_model::DiagonalNoiseModel::from_sigmas(Vector6::new(
            self.sigma_prior_rotation[0], self.sigma_prior_rotation[1], self.sigma_prior_rotation[2],
            self.sigma_prior_translation[0], self.sigma_prior_translation[1], self.sigma_prior_translation[2]
        ));
        let v_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(3, self.sigma_velocity);
        let b_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(6, self.sigma_bias);

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

        // Create GTSAM preintegration parameters for use with Foster's version
        let mut params = PreintegrationCombinedParams::makesharedu();  // Z-up navigation frame: gravity points along negative Z-axis !!!
        params.set_params(
            self.accel_noise_density * self.accel_noise_density, // acc white noise in continuous
            self.gyro_noise_density * self.gyro_noise_density, // gyro white noise in continuous
            self.accel_random_walk * self.accel_random_walk, // acc bias in continuous
            self.gyro_random_walk * self.gyro_random_walk, // gyro bias in continuous
            0.1, // error committed in integrating position from velocities
            1e-5 // error in the bias used for preintegration
        );

        // Actually create the GTSAM preintegration
        self.preint_gtsam = PreintegratedCombinedMeasurements::new(params, &prior_state.bias);

        Ok(())
    }

    fn get_predicted_state(&self) -> GtsamState {
        let _span = tracy_client::span!("get_predicted_state");
        // This function will get the predicted state based on the IMU measurement

        // Get the current state (t=k)
        let state_k = GtsamState {
            pose: self.values_initial.get_pose3(&Symbol::new(b'x', self.ct_state)).unwrap().into(),
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

    fn create_imu_factor(&mut self, imu_measurements: &mut ImuMeasurements, current_frame: &mut Frame, last_kf_id: Id, map: &ReadWriteMap) -> Result<(), Box<dyn std::error::Error>> {
        // This function will create a discrete IMU factor using the GTSAM preintegrator class
        // This will integrate from the current state time up to the new update time
        let _span = tracy_client::span!("create_imu_factor");

        let mut imu_from_last_frame = VecDeque::with_capacity(imu_measurements.len()); // mvImuFromLastFrame
        let imu_per = 0.001;
        let prev_ts = {
            let map = map.read()?;
            let previous_kf = map.get_keyframe(last_kf_id);
            previous_kf.timestamp
        };

        while !imu_measurements.is_empty() {
            if imu_measurements.front().unwrap().timestamp < prev_ts - imu_per {
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
        let mut imu_preintegrated_from_last_frame;
        {
            let map = map.read()?;
            let previous_frame = map.get_keyframe(last_kf_id);
            imu_preintegrated_from_last_frame = ImuPreIntegrated::new(previous_frame.imu_data.imu_bias);

            for i in 0..n {
                let mut tstep = 0.0;
                let mut acc: Vector3<f64> = Vector3::zeros(); // acc
                let mut ang_vel: Vector3<f64> = Vector3::zeros(); // angVel

                if i == 0 && i < (n - 1) {
                    let tab = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                    let tini = imu_from_last_frame[i].timestamp - prev_ts;
                    acc = (
                        imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc -
                        (imu_from_last_frame[i + 1].acc - imu_from_last_frame[i].acc) * (tini/tab)
                    ) * 0.5;
                    ang_vel = (
                        imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel -
                        (imu_from_last_frame[i + 1].ang_vel - imu_from_last_frame[i].ang_vel) * (tini/tab)
                    ) * 0.5;
                    tstep = imu_from_last_frame[i + 1].timestamp - prev_ts;
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
                    tstep = current_frame.timestamp - prev_ts;
                }
                // tstep = tstep * 1e9;

                self.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);

                // ORBSLAM3 imu preintegrated object
                other_imu.imu_preintegrated_from_last_kf.integrate_new_measurement(acc, ang_vel, tstep);
                imu_preintegrated_from_last_frame.integrate_new_measurement(acc, ang_vel, tstep);
            }
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
        current_frame.imu_data.imu_preintegrated = Some(other_imu.imu_preintegrated_from_last_kf.clone());
        current_frame.imu_data.imu_preintegrated_frame = Some(imu_preintegrated_from_last_frame);
        current_frame.imu_data.prev_keyframe = Some(last_kf_id);
        println!("In create imu factor, last kf id is: {}", last_kf_id);
        // other_imu.predict_state_last_keyframe(&map, current_frame, last_kf_id)?;

        Ok(())
    }

    fn process_smart_features(&mut self, current_frame: &Frame, feature_ids: & Vec<i32>) {
        let _span = tracy_client::span!("process_smart_features");

        let features = current_frame.features.get_all_keypoints();
        for i in 0..features.len() - 1 {
            let feature_id = feature_ids[i as usize];
            if feature_id == -1 {
                continue;
            }
            let (kp, _is_outlier) = current_frame.features.get_keypoint(i as usize);

            // Check to see if it is already in the graph
            match self.measurement_smart_lookup_left.get_mut(&feature_id) {
                Some(smartfactor) => {
                    // Insert measurements to a smart factor
                    smartfactor.add(
                        & gtsam::geometry::point2::Point2::new(kp.pt().x as f64, kp.pt().y as f64),
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
                        & gtsam::geometry::point2::Point2::new(kp.pt().x as f64, kp.pt().y as f64),
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