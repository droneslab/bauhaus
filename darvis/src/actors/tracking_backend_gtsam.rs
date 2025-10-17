extern crate g2o;
use log::{warn, info, debug};
use nalgebra::{Isometry3, Matrix3, Vector3, Vector6};
use opencv::core::Mat;
use std::{collections::{BTreeSet, HashMap, VecDeque}, fmt::Debug, thread::sleep, time::Duration};
use gtsam::{
    inference::symbol::Symbol, navigation::combined_imu_factor::{CombinedImuFactor, PreintegratedCombinedMeasurements, PreintegrationCombinedParams}, nonlinear::{
        incremental_fixed_lag_smoother::IncrementalFixedLagSmoother, 
        isam2::ISAM2, levenberg_marquardt_optimizer::LevenbergMarquardtOptimizer, levenberg_marquardt_params::LevenbergMarquardtParams, nonlinear_factor_graph::NonlinearFactorGraph, values::Values
    },
};
use core::{
    config::*, matrix::*, system::{Actor, MessageBox, System, Timestamp}
};
use crate::{
    actors::{messages::{FeatureTracksAndIMUMsg, ShutdownMsg, TrajectoryMsg, UpdateFrameIMUMsg, VisTrajectoryMsg}, tracking_frontend_gtsam::{GtsamFrontendTrackingState, TrackedFeatures}},
    map::{frame::Frame, map::Id, pose::Pose, read_only_lock::ReadWriteMap}, modules::{image::draw_optical_flow, imu::{ImuBias, ImuCalib, ImuMeasurements}}, registered_actors::{CAMERA, IMU, SHUTDOWN_ACTOR, TRACKING_BACKEND, VISUALIZER}, ImuInitializationData
};
use num_traits::Pow;
use gtsam::sys::ffi::DoubleVec;

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
        // println!("Imu measurements right now: {:?}", msg.imu_measurements);

        let current_frame = if matches!(self.graph_solver.solver_state, GraphSolverState::NotInitialized) {
            // Initialize from gt
            debug!("Initializing!");
            let mut current_frame = msg.frame;
            let (init_pose, init_velocity, init_bias) = self.graph_solver.initialize(
                current_frame.timestamp,
                &msg.imu_initialization.expect("Msg should have imu initialization data!"),
                msg.feature_tracks,
            ).expect("Failed to initialize?");

            // Update frame and create new keyframe
            current_frame.set_imu_pose_velocity(
                init_pose,
                init_velocity
            );
            current_frame.imu_data.set_new_bias(init_bias);
            let _new_kf_id = self.create_new_keyframe(&mut current_frame).expect("Could not create new keyframe");

            let map = self.map.read()?;
            self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
                pose: init_pose,
                mappoint_matches: vec![],
                mappoints_in_tracking: BTreeSet::new(),
                timestamp: current_frame.timestamp,
                map_version: map.version
            }));

            debug!("Initializing done! First keyframe's timestamp: {}", current_frame.timestamp);


            self.kf_count = 0;

            // Only for visualizing optical flow
            // self.last_image = Some(msg.frame.image.as_ref().unwrap().clone());


            // Initialize from map initialization
                // let imu_init = msg.imu_initialization.expect("Msg should have imu initialization data!"); // Rotation in this is TBW! 
                // let init_bias = ImuBias::new_with(imu_init.gyro_bias, imu_init.acc_bias);

                // // self.scale_map_from_imu(&mut msg)?;

                // // Instead of scale map from imu..............
                // // {
                // //     let current_distance = Self::distance(&self.map.read().unwrap().get_keyframe(0).get_pose().get_translation(), &self.map.read().unwrap().get_keyframe(1).get_pose().get_translation());
                // //     let gt_distance = Self::distance(
                // //         & Vector3::new(4.633082, -1.807218, 0.830638),
                // //         & Vector3::new(4.629793, -1.804637, 0.862069)
                // //     );
                // //     println!("Current distance: {:?}", current_distance);
                // //     println!("GT distance: {:?}", gt_distance);
                // //     println!("Scale: {:?}", current_distance / gt_distance);
                // // }

                // // {
                // //     let mut map = self.map.write()?;
                // //     // Ground truth rotation is Tbw, need to store in keyframe as Tcw
                // //     let tbw = Pose::new_with_quaternion_convert(
                // //         Vector3::new(0.0, 0.0, 0.0),
                // //         imu_init.rotation
                // //     );
                // //     let twc = tbw.inverse() * ImuCalib::new().tbc; // twb * tbc
                // //     let tcw = twc.inverse();

                // //     map.get_keyframe_mut(1).imu_data.velocity = Some(imu_init.velocity);
                // //     map.get_keyframe_mut(0).imu_data.velocity = Some(DVVector3::new_with(0.0, 0.0, 0.0));
                // //     map.apply_scaled_rotation(&tcw, 1.0,false);
                // // }


                // // Now that the map is scaled, re-initialize the graph solver
                // let init_velocity = {
                //     let map = self.map.read()?;
                //     let kf0 = map.get_keyframe(0);
                //     let kf1 = map.get_keyframe(1);
                //     let kf0_pose = kf0.get_pose();
                //     let kf1_pose = kf1.get_pose();

                //     let vel_camera = DVVector3::new(
                //         (*kf1_pose.get_translation() - *kf0_pose.get_translation()) / (kf1.timestamp - kf0.timestamp)
                //     ); // Twc ? Or Tcw ? Assuming Twc

                //     let init_velocity2 = (*vel_camera).transpose() * *ImuCalib::new().tcb.get_rotation(); // Twc * Tcb = Twb
                //     let init_velocity3 = DVVector3::new_with(
                //         init_velocity2[0],
                //         init_velocity2[1],
                //         init_velocity2[2]
                //     );
                //     init_velocity3
                // };

                // let (init_timestamp, init_pose) = {
                //     let mut map = self.map.write()?;
                //     let kf = map.get_keyframe_mut(1);

                //     // Note (frames): Kf1 pose is Tcw, initialize graph solver with Tbw
                //     let tcw = kf.get_pose();
                //     let tbw = ImuCalib::new().tbc * tcw;

                //     kf.imu_data.velocity = Some(init_velocity);
                //     (kf.timestamp, tbw)
                // };


                // self.graph_solver = GraphSolver::new();
                // self.graph_solver.initialize(
                //     init_timestamp,
                //     init_pose,
                //     init_velocity,
                //     init_bias
                // ).expect("Failed to initialize?");
                // self.graph_solver.process_smart_features(&msg.feature_tracks, 0);

                // self.graph_solver.solver_state = GraphSolverState::Ok;
                // self.last_timestamp = msg.frame.timestamp;
                // println!("Initializing done!");

                // msg.frame.pose = Some(init_pose.clone());
                // self.create_new_keyframe(&mut msg.frame).expect("Could not create new keyframe");
                // let map = self.map.read()?;
                // self.system.try_send(VISUALIZER, Box::new(VisTrajectoryMsg{
                //     pose: init_pose,
                //     mappoint_matches: vec![],
                //     mappoints_in_tracking: BTreeSet::new(),
                //     timestamp: msg.frame.timestamp,
                //     map_version: map.version,
                //     debug: msg.imu_measurements.clone(),
                // }));
            current_frame
        } else {
            // If we have previous frames already, can track normally
            let mut current_frame = msg.frame;
            println!("Current timestamp: {}", current_frame.timestamp);

            // Visualizing optical flow
            // if self.last_image.is_some() {
                // draw_optical_flow(
                //     self.last_image.as_ref().unwrap(),
                //     current_frame.image.as_ref().unwrap(),
                //     & msg.last_features.get_points_as_vector_of_point2f(),
                //     & msg.feature_tracks.get_points_as_vector_of_point2f(),
                //     &format!("results/flow/backend{}.png", self.curr_frame_id),
                // ).unwrap();
                // debug!("Backend, tracked features last: {:?}", msg.last_features);
                // debug!("Backend, tracked features now: {:?}", msg.feature_tracks);
            // }

            // Solve VIO graph. Includes preintegration

            // Option to batch update instead of update each time. Not sure this works correctly
            let should_update = match self.graph_solver.optimizer {
                Optimizer::ISAM2 {..} => SETTINGS.get::<i32>(TRACKING_BACKEND, "isam_update_interval") == self.kf_count,
                Optimizer::IncrementalFixedLagSmoother {..} => true,
                Optimizer::LevenbergMarquadt { } => true,
            };
            println!("Updating graph? {}", should_update);
            let optimization_results = self.graph_solver.solve(
                msg.tracker_status,
                &mut current_frame,
                &mut msg.imu_measurements.clone(),
                &msg.feature_tracks,
                msg.removed_feature_ids,
                should_update
            )?;

            let _new_kf_id = self.create_new_keyframe(&mut current_frame).expect("Could not create new keyframe");

            // If using isam, we need to update all keyframes' poses because they are optimized each time
            match self.graph_solver.optimizer {
                Optimizer::ISAM2 {..} => {
                    for (state_key, pose, _velocity, _bias) in optimization_results.iter() {
                        self.map.write()?.get_keyframe_mut(*state_key as i32).set_pose(*pose);
                    }
                },
                _ => {}
            }

            self.kf_count = if should_update { 0 } else { self.kf_count + 1 };

            current_frame

            // sleep(Duration::from_millis(10000000));
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

    fn scale_map_from_imu(&mut self, msg: &mut FeatureTracksAndIMUMsg) -> Result<(), Box<dyn std::error::Error>> {
        let (_scale, velocity) = {
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

        let t = {
            let rot1 = Matrix3::new(
                0.99977350234985352, 0.011315983720123768, -0.018024308606982231,
                0.021248025819659233,  -0.48284673690795898,   0.87544715404510498,
                0.0012035687686875463,  -0.87563186883926392,  -0.48297786712646484,
            );

            let rot2 = Matrix3::new(
                0.99999994039535522,  4.4160465506593027e-08,  0.00025777640985324979,
                4.4160010759242141e-08,     0.99999994039535522, -0.00034262429107911885,
                -0.00025777640985324979,  0.00034262429107911885,     0.99999988079071045,
            );

            let rot3 = Matrix3::new(
                0.99995380640029907,  -3.057897265534848e-05,    0.009609355591237545,
                -2.7023115762858652e-05,     0.99998205900192261,   0.0059941890649497509,
                -0.0096093658357858658,  -0.0059941718354821205,     0.99993586540222168,
            );

            let rot4 = Matrix3::new(
                0.99999964237213135,  -4.904205752609414e-07, -0.00086154910968616605,
                -5.3174642289377516e-07 ,     0.9999992847442627 , -0.0011864284751936793,
                0.00086154905147850513  , 0.0011864284751936793  ,   0.99999892711639404,
            );
            let final_rot = rot1 * rot2 * rot3 * rot4;


            println!("Final rotation: {:?}", final_rot);

            // let final_rot = Matrix3::new(
            //     1.0, -0.0, 0.0,
            //     0.0,  0.0, 1.0,
            //     -0.0, -1.0, 0.0
            // );

            Pose::new(
                Vector3::new(0.0, 0.0, 0.0),
                final_rot
            )
        };

        let scale = 0.54736596345901489 *
            0.99563616514205933 *
            1.0052615404129028 *
            0.99784046411514282;

        {
            let mut map = self.map.write()?;
            map.get_keyframe_mut(1).imu_data.velocity = Some(velocity);
            map.get_keyframe_mut(0).imu_data.velocity = Some(DVVector3::new_with(0.0, 0.0, 0.0));
            map.apply_scaled_rotation(&t, scale,true);
        }

        Ok(())
    }

    fn create_new_keyframe(&mut self, current_frame: &mut Frame) -> Result<Id, Box<dyn std::error::Error>> {
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

        // println!("Created new keyframe with pose {:?}", current_frame.pose.unwrap());

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
    

        Ok(new_kf_id)
    }

    fn distance(v1: &Vector3<f64>, v2: &Vector3<f64>) -> f64 {
        let dx = v2.x - v1.x;
        let dy = v2.y - v1.y;
        let dz = v2.z - v1.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
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

    // New graph
    graph_new: NonlinearFactorGraph, // New factors that have not been optimized yet
    values_new: Values, // New values that have not been optimized yet

    // Main graph
    values_all: Values, // All created nodes
    inserted_kfs: Vec<u64>, // Keyframe IDs that have been inserted into the graph

    // Misc GTSAM objects
    optimizer: Optimizer,
    preint_gtsam: PreintegratedCombinedMeasurements, // IMU preintegration

    // Initialization
    accel_noise_density: f64, // accelerometer_noise_density, sigma_a
    gyro_noise_density: f64, // gyroscope_noise_density, sigma_g
    accel_random_walk: f64, // accelerometer_random_walk, sigma_wa
    gyro_random_walk: f64, // gyroscope_random_walk, sigma_wg
    sampling_frequency: f64, // sqrt(imu frequency)
    // Camera defaults
    k: gtsam::geometry::cal3_s2::Cal3S2, // Camera intrinsics
    vision_measurement_noise: gtsam::linear::noise_model::IsotropicNoiseModel, // Camera measurement noise model
    tbc: Pose, // Transform from camera frame to body (IMU) 

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

    // Managing smart factors
    smartfactors: HashMap<i32, (bool, gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2)>, // Lookup of smart factor pointers by feature ID. Bool contains whether it is in the graph or not (can be added to this list without being in the graph if we are batching updates)
    smartfactor_idx_in_isam2: HashMap<i32, u64>, // (smart factor ID in this object) -> (index in isam2)

    last_timestamp: Timestamp
}

impl GraphSolver {
    pub fn new(optimizer_type: i32, use_smart_factors: bool) -> Self {
        let vision_measurement_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(2, 3.0);
        let k = gtsam::geometry::cal3_s2::Cal3S2::new(
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
                )
            },
            1 => Optimizer::IncrementalFixedLagSmoother {
                smoother: IncrementalFixedLagSmoother::new(
                    0.0,
                    SETTINGS.get::<f64>(TRACKING_BACKEND, "isam_relinearize_threshold"),
                    SETTINGS.get::<i32>(TRACKING_BACKEND, "isam_relinearize_skip"),
                    SETTINGS.get::<bool>(TRACKING_BACKEND, "isam_cache_linearized_factors"),
                    SETTINGS.get::<bool>(TRACKING_BACKEND, "isam_enable_detailed_results"),
                )
            },
            2 => Optimizer::LevenbergMarquadt {},
            _ => panic!("Unknown optimizer type!"),
        };

        Self {
            optimizer,
            use_smart_factors,
            solver_state: GraphSolverState::NotInitialized,

            preint_gtsam: PreintegratedCombinedMeasurements::default(),
            graph_new: NonlinearFactorGraph::default(),
            values_new: Values::default(),
            values_all: Values::default(),

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

            ct_state: 0,
            smartfactors: HashMap::new(),
            smartfactor_idx_in_isam2: HashMap::new(),
            last_timestamp: 0.0,
            inserted_kfs: Vec::new(),
        }
    }

    fn initialize(&mut self, timestamp: f64, imu_init: &ImuInitializationData, feature_tracks: TrackedFeatures) -> Result<(Pose, nalgebra::Vector3<f64>, ImuBias), Box<dyn std::error::Error>> {
        let init_bias = ImuBias::new_with(imu_init.gyro_bias, imu_init.acc_bias);
        let init_pose = Pose::new(
                *imu_init.pose.get_translation(),
                *imu_init.pose.get_rotation()
        );
        println!("... Initial pose: {:?}", init_pose);
        println!("... Initial pose rotation matrix: {:?}", init_pose.get_rotation());
        println!("... Initial velocity: {:?}", imu_init.velocity);
        println!("... Initial bias: {:?}", init_bias);

        // Create preintegraiton object and add all priors to graph
        let prior_state = self.add_initial_state(init_pose, imu_init.velocity, init_bias, true)?;
        self.create_preintegration(prior_state.bias);
        self.last_timestamp = timestamp;

        // Add features to graph
        let (new_factor_feature_ids, new_affected_keys) = self.process_features(&feature_tracks, 0);
        self.optimize(new_factor_feature_ids, new_affected_keys, vec![]);

        self.solver_state = GraphSolverState::Ok;
        let (pose, velocity, bias) = self.get_results_from_values(self.ct_state)?;

        return Ok((pose, velocity, bias));
    }

    fn add_initial_state(&mut self, prior_pose: Pose, init_vel: DVVector3<f64>, init_bias: ImuBias, add_to_all_values: bool) -> Result<GtsamState, Box<dyn std::error::Error>> {
        // Create prior factor and add it to the graph
        let prior_state = {
            let trans = prior_pose.translation;
            let rot = prior_pose.get_quaternion();
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
        pose_prior_covariance2[(2, 2)] = self.initial_yaw_sigma * self.initial_yaw_sigma;

        // Rotate initial uncertainty into local frame, where the uncertainty is
        // specified.
        let b_rot_w = * prior_pose.get_rotation();
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

        // Add initial velocity priors.
        let v_noise = gtsam::linear::noise_model::IsotropicNoiseModel::from_dim_and_sigma(3, self.initial_velocity_sigma);
        self.graph_new.add_prior_factor_vector3_isotropicnoisemodel(&Symbol::new(b'v', self.ct_state), &prior_state.velocity, &v_noise);

        // Add initial bias priors:
        let b_noise = gtsam::linear::noise_model::DiagonalNoiseModel::from_sigmas(Vector6::new(
            self.initial_acc_bias_sigma,
            self.initial_acc_bias_sigma,
            self.initial_acc_bias_sigma,
            self.initial_gyro_bias_sigma,
            self.initial_gyro_bias_sigma,
            self.initial_gyro_bias_sigma,
        ));
        self.graph_new.add_prior_factor_constant_bias_diagonal(&Symbol::new(b'b', self.ct_state), &prior_state.bias, &b_noise);

        // Add initial state to the graph
        self.values_new.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
        self.values_new.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
        self.values_new.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);
        if add_to_all_values {
            self.values_all.insert_pose3(&Symbol::new(b'x', self.ct_state), &prior_state.pose);
            self.values_all.insert_vector3(&Symbol::new(b'v', self.ct_state), &prior_state.velocity);
            self.values_all.insert_constant_bias(&Symbol::new(b'b', self.ct_state), &prior_state.bias);
        }

        Ok(prior_state)
    }

    fn create_preintegration(&mut self, prior_bias: gtsam::imu::imu_bias::ConstantBias) {
        // Create GTSAM preintegration parameters for use with Foster's version
        // let mut params = PreintegrationCombinedParams::new(-9.81, 0.0, 0.0);
        let mut params = PreintegrationCombinedParams::new(0.0, 0.0, -9.81);

        // TODO KIMERA
        // preint_imu_params.gyroscopeCovariance =
        //     std::pow(imu_params.gyro_noise_density_, 2.0) *
        //     Eigen::Matrix3d::Identity();
        // preint_imu_params.accelerometerCovariance =
        //     std::pow(imu_params.acc_noise_density_, 2.0) *
        //     Eigen::Matrix3d::Identity();
        // preint_imu_params.integrationCovariance =
        //     std::pow(imu_params.imu_integration_sigma_, 2.0) *
        //     Eigen::Matrix3d::Identity();
        // preint_imu_params.use2ndOrderCoriolis = false;

        params.set_gyroscope_covariance(f64::pow(self.gyro_noise_density, 2));
        params.set_accelerometer_covariance(f64::pow(self.accel_noise_density, 2));
        // SOFIYA TODO... removed these because they do not seem to be in kimera?
        // params.set_bias_acc_covariance(f64::pow(self.accel_random_walk, 2));
        // params.set_bias_omega_covariance(f64::pow(self.gyro_random_walk, 2));
        // These are not in orbslam but are in kimera:
        params.set_integration_covariance(f64::pow(self.imu_integration_sigma, 2));
        params.set_bias_acc_omega_int(self.init_bias_sigma);

        // Actually create the GTSAM preintegration
        self.preint_gtsam = PreintegratedCombinedMeasurements::new(params, &prior_bias);
    }

    fn solve(&mut self,
        tracker_status: GtsamFrontendTrackingState,
        current_frame : &mut Frame, 
        imu_measurements : &mut ImuMeasurements, new_tracked_features : &TrackedFeatures,
        removed_feature_ids: Vec<u64>,
        should_update: bool,
    ) -> Result<Vec<(u64, Pose, nalgebra::Vector3<f64>, ImuBias)>, Box<dyn std::error::Error>> {
        let _span = tracy_client::span!("solve");

        let timestamp = (current_frame.timestamp * 1e9) as i64; // Convert to int just so we can hash it

        print!("what the heck {}, {}", timestamp, current_frame.timestamp);

        self.preintegrate(imu_measurements, current_frame.timestamp, self.last_timestamp)?;
        self.create_imu_factor();

        // IMU
        let new_state = self.imu_predict_state();
        debug!("IMU POSE ESTIMATE... {}; {:?}; {:?}; {:?}", timestamp, new_state.pose, new_state.velocity, new_state.bias);

        // Move node count forward in time
        self.ct_state += 1;

        // Add nodes
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
        self.values_all.insert_pose3(
            &Symbol::new(b'x', self.ct_state),
            &new_state.pose
        );
        self.values_all.insert_vector3(
            &Symbol::new(b'v', self.ct_state),
            &new_state.velocity
        );
        self.values_all.insert_constant_bias(
            &Symbol::new(b'b', self.ct_state),
            &new_state.bias
        );

        // Add features
        let (new_factor_feature_ids, new_affected_keys) = self.process_features(new_tracked_features, self.ct_state);

        if should_update {
            self.optimize(new_factor_feature_ids, new_affected_keys, removed_feature_ids);
        }

        match tracker_status {
            GtsamFrontendTrackingState::LowDisparity => {
                // Vehicle is not moving
                debug!("Tracker has a LOW_DISPARITY status. Add zero velocity and no motion factors.");
                let zero_velocity_noise = {
                    let precision = SETTINGS.get::<f64>(TRACKING_BACKEND, "zero_velocity_precision");
                    gtsam::linear::noise_model::DiagonalNoiseModel::from_precisions(
                        Vector3::new(precision, precision, precision)
                    )
                };
                self.graph_new.add_prior_factor_vector3_diagonalnoisemodel(
                    &Symbol::new(b'v', self.ct_state),
                    &gtsam::base::vector::Vector3::new(0.0, 0.0, 0.0),
                    &zero_velocity_noise
                );
                let no_motion_prior_noise = {
                    let rotation_precision = SETTINGS.get::<f64>(TRACKING_BACKEND, "no_motion_rotation_precision");
                    let position_precision = SETTINGS.get::<f64>(TRACKING_BACKEND, "no_motion_position_precision");
                    gtsam::linear::noise_model::DiagonalNoiseModel::from_precisions(
                        Vector6::new(
                            rotation_precision, rotation_precision, rotation_precision,
                            position_precision, position_precision, position_precision)
                    )
                };
                self.graph_new.add_between_factor_pose3(
                    &Symbol::new(b'x', self.ct_state - 1),
                    &Symbol::new(b'x', self.ct_state),
                    &gtsam::geometry::pose3::Pose3::default(),
                    & no_motion_prior_noise
                );
            },
            _ => {}
        }


        // match &self.optimizer {
        //     Optimizer::ISAM2{isam2} => {
        //         debug!("OPTIMIZATION COVARIANCE: {:?}", isam2.get_marginal_covariance(&Symbol::new(b'x', self.ct_state - 1)));
        //     },
        //     _ => {}
        // }

        let mut optimization_results = vec![];
        match self.optimizer {
            Optimizer::ISAM2{..} | Optimizer::IncrementalFixedLagSmoother{..} => {
                // If using the smoothers, we need to edit all the prior keyframes with new optimized values
                // Note (frames): Result should be Tbw or Twb, not sure which one.
                self.inserted_kfs.push(self.ct_state);

                for state_key in &self.inserted_kfs {
                    let (pose, velocity, bias) = self.get_results_from_values(*state_key)?;

                    if state_key == &self.ct_state {
                        // Current frame
                        current_frame.set_imu_pose_velocity(
                            pose,
                            velocity
                        );
                        current_frame.imu_data.set_new_bias(bias);
                        debug!("OPTIMIZED POSE ESTIMATE... {}; {:?}; {:?}; {:?}", timestamp, pose, velocity, current_frame.imu_data.imu_bias);
                        debug!("STATE KEY: {}, {}.", self.ct_state, timestamp);
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
                let (pose, velocity, bias) = self.get_results_from_values(self.ct_state)?;
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

    fn imu_predict_state(&self) -> GtsamState {
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

        let predicted = GtsamState {
            pose: state_k1.get_pose().into(),
            velocity: state_k1.get_velocity().into(),
            bias: state_k.bias
        };

        // debug!("IMU COVARIANCE: {:?}", self.preint_gtsam.get_covariance());

        return predicted;
    }

    fn preintegrate(&mut self, imu_measurements: &mut ImuMeasurements, current_timestamp: Timestamp, last_timestamp: Timestamp) -> Result<(), Box<dyn std::error::Error>> {
        // This function will create a discrete IMU factor using the GTSAM preintegrator class
        // This will integrate from the current state time up to the new update time
        let _span = tracy_client::span!("create_imu_factor");

        // let mut total_tstep = 0.0;
        // for i in 1..imu_measurements.len() {
        //     let tstep = imu_measurements[i].timestamp - imu_measurements[i - 1].timestamp;
        //     let acc: Vector3<f64> = imu_measurements[i].acc; // acc
        //     let ang_vel: Vector3<f64> = imu_measurements[i].ang_vel; // angVel
        //     println!("Preintegrating {} measurement:  Acc: {} {} {}, Omega: {} {} {}, dt: {}",
        //         i,
        //         acc.x, acc.y, acc.z, // acc
        //         ang_vel.x, ang_vel.y, ang_vel.z, // angVel
        //         tstep
        //     );

        //     self.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);
        //     total_tstep += tstep;
        // }

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
                // println!("#1, current timestamp: {}, last timestamp: {}", imu_from_last_frame[i + 1].timestamp, last_timestamp);
            } else if i < (n - 1) {
                acc = (imu_from_last_frame[i].acc + imu_from_last_frame[i + 1].acc) * 0.5;
                ang_vel = (imu_from_last_frame[i].ang_vel + imu_from_last_frame[i + 1].ang_vel) * 0.5;
                tstep = imu_from_last_frame[i + 1].timestamp - imu_from_last_frame[i].timestamp;
                // println!("#2, current timestamp: {}, last timestamp: {}", imu_from_last_frame[i + 1].timestamp, imu_from_last_frame[i].timestamp);
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
                // println!("#3, current timestamp: {}, last timestamp: {}", current_timestamp, imu_from_last_frame[i].timestamp);
            } else if i == 0 && i == (n - 1) {
                acc = imu_from_last_frame[i].acc;
                ang_vel = imu_from_last_frame[i].ang_vel;
                tstep = current_timestamp - last_timestamp;
                // println!("#4, current timestamp: {}, last timestamp: {}", current_timestamp, last_timestamp);
            }

            acc = Vector3::<f64>::new(acc[0], acc[1], acc[2]);

            if tstep == 0.0 {
                // IMU messages between frames have one overlapping message, so if we have multiple frames
                // between keyframes then there will be the same message twice in a row. Safe to skip.
                continue;
            }

            self.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);

            // println!("RAW IMU: Acc: {} {} {}, Omega: {} {} {}",
            //     imu_from_last_frame[i].acc.x, imu_from_last_frame[i].acc.y, imu_from_last_frame[i].acc.z,
            //     imu_from_last_frame[i].ang_vel.x, imu_from_last_frame[i].ang_vel.y, imu_from_last_frame[i].ang_vel.z,
            // );
            // println!("Preintegrating {} measurement:  Acc: {} {} {}, Omega: {} {} {}, dt: {}",
            //     i,
            //     acc.x, acc.y, acc.z, // acc
            //     ang_vel.x, ang_vel.y, ang_vel.z, // angVel
            //     tstep
            // );
            // total_tstep += tstep;
        }
        // println!("Sofiya, total tstep: {}", total_tstep);
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
    }

    fn process_features(&mut self, new_tracked_features: &TrackedFeatures, state_id: u64) -> (Vec<i32>, Vec<DoubleVec>) {
        let _span = tracy_client::span!("process_smart_features");

        let mut id_notset = 0;
        let mut added_new = 0;
        let mut added_old = 0;

        let mut new_factor_feature_ids: Vec<i32> = vec![];
        let mut new_affected_keys: Vec<DoubleVec> = vec![];

        println!("NEW TRACKED FEATURES: {:?}", new_tracked_features);

        for i in 0..new_tracked_features.len() {
            let feature_id = new_tracked_features.get_feature_id(i as usize);
            if feature_id == -1 {
                id_notset += 1;
                warn!("Id not set for feature with point {:?}", new_tracked_features.get_point(i as usize));
                continue;
            }
            let point = new_tracked_features.get_point(i as usize);

            if self.use_smart_factors {
                // USING SMART FACTORS! 
                // Check to see if it is already in the graph
                match self.smartfactors.get_mut(&feature_id) {
                    Some((is_in_graph, smartfactor)) => {
                        // Insert measurements to a smart factor
                        smartfactor.add(
                            & gtsam::geometry::point2::Point2::new(point.x as f64, point.y as f64),
                            &Symbol::new(b'x', state_id)
                        );
                        added_old += 1;

                        match self.optimizer {
                            Optimizer::ISAM2{..} => {
                                // Check for is_in_graph b/c it is possible that the smartfactor is created 
                                // from a previous keyframe, but not in the graph yet if we are batching updates.
                                if *is_in_graph {
                                    // Update new_affected_keys to show that this smart factor was updated
                                    // This is sent to the isam2 update function
                                    let index_in_isam2 = self.smartfactor_idx_in_isam2[&feature_id];
                                    new_affected_keys.push(DoubleVec{
                                            vec: vec![index_in_isam2 as f64, state_id as f64]
                                    });
                                }
                            },
                            _ => {}
                        }

                        continue;
                    },
                    None => {
                        // If we know it is not in the graph
                        // Create a smart factor for the new feature
                        let mut smartfactor_left = gtsam::slam::projection_factor::SmartProjectionPoseFactorCal3S2::new(
                            &self.vision_measurement_noise,
                            &self.k,
                            & self.tbc.inverse().into()
                        );

                        smartfactor_left.add(
                            & gtsam::geometry::point2::Point2::new(point.x as f64, point.y as f64),
                            &Symbol::new(b'x', state_id)
                        );

                        // Add smart factor
                        self.graph_new.add_smartfactor(&smartfactor_left);

                        // Insert to smartfactor lookup, so we can find this smartfactor in the next frames
                        self.smartfactors.insert(feature_id, (false, smartfactor_left));
                        // Keep track of smartfactors in the order we insert them in, we will need this after the update in isam2
                        new_factor_feature_ids.push(feature_id);

                        added_new += 1;
                    }
                }
            } else {
                // USING GENERIC FACTORS!
                let factor = gtsam::slam::projection_factor::GenericProjectionFactorPose3Point3Cal3S2::new(
                    & gtsam::geometry::point2::Point2::new(point.x as f64, point.y as f64),
                    &self.vision_measurement_noise,
                    &Symbol::new(b'x', state_id),
                    &Symbol::new(b'l', feature_id.try_into().unwrap()),
                    &self.k,
                    & self.tbc.inverse().into()
                );

                self.graph_new.add_generic_factor(&factor);

            }
        }
        debug!("Added new features: {}, updated old features: {}, id not set: {}", added_new, added_old, id_notset);

        return (new_factor_feature_ids, new_affected_keys);
    }

    fn optimize(&mut self, new_factor_feature_ids: Vec<i32>, new_affected_keys: Vec<DoubleVec>, _removed_feature_ids: Vec<u64>) -> Vec<gtsam::sys::Point> {
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
                let isam2result = isam2.update(& self.graph_new, & self.values_new, & new_affected_keys, & vec![]);

                self.values_all = isam2.calculate_estimate();

                // ISAM2 returns vector of indexes for each smartfactor within isam2. The vector is 
                // sorted by order of smartfactor insertion.
                // Here, link up inserted smartfactor feature ID with isam2 index. We will need these
                // links later to tell isam2 that a previously inserted smartfactor has an update
                for i in 0..new_factor_feature_ids.len() {
                    let smartfactor_id_here = new_factor_feature_ids[i];
                    let smartfactor_id_in_isam2 = isam2result.new_factor_indices[i];
                    self.smartfactor_idx_in_isam2.insert(smartfactor_id_here, smartfactor_id_in_isam2);
                    self.smartfactors.get_mut(&smartfactor_id_here).unwrap().0 = true; // Mark that this smartfactor is now in the graph
                }
                debug!("TEST! After optimize, smartfactor_idx_in_isam2 is: {:?}", self.smartfactor_idx_in_isam2);

                points = isam2result.points;
            },
            Optimizer::IncrementalFixedLagSmoother{smoother} => {
                // Perform smoothing update
                let result = smoother.update(& self.graph_new, & self.values_new);

                // self.values_all = smoother.calculate_estimate();

                // // ISAM2 returns vector of indexes for each smartfactor within isam2. The vector is 
                // // sorted by order of smartfactor insertion.
                // // Here, link up inserted smartfactor feature ID with isam2 index. We will need these
                // // links later to tell isam2 that a previously inserted smartfactor has an update
                // for i in 0..new_factor_feature_ids.len() {
                //     let smartfactor_id_here = new_factor_feature_ids[i];
                //     let smartfactor_id_in_isam2 = isam2result.new_factor_indices[i];
                //     self.smartfactor_idx_in_isam2.insert(smartfactor_id_here, smartfactor_id_in_isam2);
                //     self.smartfactors.get_mut(&smartfactor_id_here).unwrap().0 = true; // Mark that this smartfactor is now in the graph
                // }
                // debug!("TEST! After optimize, smartfactor_idx_in_isam2 is: {:?}", self.smartfactor_idx_in_isam2);

                // points = isam2result.points;

            },
            Optimizer::LevenbergMarquadt{} => {
                let params = LevenbergMarquardtParams::default();
                let mut optimizer = LevenbergMarquardtOptimizer::new(& self.graph_new, & self.values_new, & params);
                let result = optimizer.optimize_safely();
                self.values_all = result;
                points = vec![]; // Ignoring mappoints for LM
            }
        };

        // Use the optimized bias to reset integration
        if self.values_all.exists(&Symbol::new(b'b', self.ct_state)) {
            self.preint_gtsam.reset_integration_and_set_bias(
                & self.values_all.get_constantbias(&Symbol::new(b'b', self.ct_state)).unwrap().into()
            );
        } else {
            warn!("Bias wasn't optimized?");
        }

        // Remove the used up factors and nodes
        self.graph_new.resize(0);
        self.values_new.clear();

        // For Levenberg-Marquardt, re-add prior factors for the current state after each optimization step.
        match self.optimizer {
            Optimizer::LevenbergMarquadt{} => {
                let (pose, velocity, bias) = self.get_results_from_values(self.ct_state).unwrap();
                self.add_initial_state(pose.into(), velocity.into(), bias.into(), false).unwrap();
            },
            _ => {}
        }

        return points;
    }


    fn get_results_from_values(&self, state_key: u64) -> Result<(Pose, nalgebra::Vector3<f64>, ImuBias), Box<dyn std::error::Error>> {
        let pose = {
            let pose: Isometry3<f64> = self.values_all.get_pose3(&Symbol::new(b'x', state_key)).unwrap().into();
            Pose::new_from_isometry(pose)
        };
        let velocity = {
            let velocity: gtsam::base::vector::Vector3 = self.values_all.get_vector3(&Symbol::new(b'v', state_key)).unwrap().into();
            let vel_raw = velocity.get_raw();
            Vector3::new(vel_raw[0], vel_raw[1], vel_raw[2])
        };
        let bias = {
            let bias_ref = self.values_all.get_constantbias(&Symbol::new(b'b', state_key)).unwrap();
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