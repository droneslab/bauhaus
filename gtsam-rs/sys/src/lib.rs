pub use self::ffi::*;

// Common types
pub type Key = u64;

#[cxx::bridge(namespace = "gtsam")]
mod ffi {
    unsafe extern "C++" {
        include!("base/vector.h");

        type Vector3;

        fn default_vector3() -> UniquePtr<Vector3>;

        fn new_vector3(x: f64, y: f64, z: f64) -> UniquePtr<Vector3>;

        fn vector3_to_raw(src: &Vector3, dst: &mut [f64]);
    }


    unsafe extern "C++" {
        include!("geometry/point3.h");

        type Point3;

        fn default_point3() -> UniquePtr<Point3>;

        fn new_point3(x: f64, y: f64, z: f64) -> UniquePtr<Point3>;

        fn point3_to_raw(src: &Point3, dst: &mut [f64]);
    }

    unsafe extern "C++" {
        include!("geometry/point2.h");

        type Point2;

        fn default_point2() -> UniquePtr<Point2>;

        fn new_point2(x: f64, y: f64) -> UniquePtr<Point2>;
    }


    unsafe extern "C++" {
        include!("geometry/pose3.h");

        type Pose3;

        fn default_pose3() -> UniquePtr<Pose3>;

        fn new_pose3(rotation: &Rot3, point: &Point3) -> UniquePtr<Pose3>;

        fn pose3_rotation(pose: &Pose3) -> &Rot3;

        fn pose3_translation(pose: &Pose3) -> &Point3;
    }

    unsafe extern "C++" {
        include!("geometry/rot3.h");

        type Rot3;

        fn default_rot3() -> UniquePtr<Rot3>;

        fn from_rot3_quaternion(w: f64, x: f64, y: f64, z: f64) -> UniquePtr<Rot3>;

        fn rot3_to_raw(src: &Rot3, dst: &mut [f64]);
    }

    unsafe extern "C++" {
        include!("geometry/cal3_s2.h");

        type Cal3_S2;

        fn default_cal3_s2() -> SharedPtr<Cal3_S2>;
    } 

    #[namespace = "gtsam::imuBias"]
    unsafe extern "C++" {
        include!("imu/imu.h");
        type ConstantBias;

        fn default_constantbias() -> UniquePtr<ConstantBias>;

        fn new_constantbias(
            bias_acc: &Vector3,
            bias_gyro: &Vector3,
        ) -> UniquePtr<ConstantBias>;

        fn accel_bias(bias: &ConstantBias) -> &Vector3;
        fn gyro_bias(bias: &ConstantBias) -> &Vector3;
    }

    unsafe extern "C++" {
        include!("imu/imu.h");
        type ImuFactor;
        type PreintegratedImuMeasurements;

        fn default_imufactor() -> UniquePtr<ImuFactor>;
        fn default_preintegratedimumeasurements() -> UniquePtr<PreintegratedImuMeasurements>;

    }


    unsafe extern "C++" {
        include!("inference/symbol.h");

        type Symbol;

        fn default_symbol() -> UniquePtr<Symbol>;

        fn new_symbol(character: u8, index: u64) -> UniquePtr<Symbol>;

        fn key(&self) -> u64;
    }

    unsafe extern "C++" {
        include!("linear/noise_model.h");

        type BaseNoiseModel;
    }

    unsafe extern "C++" {
        include!("linear/noise_model.h");

        type DiagonalNoiseModel;
        type IsotropicNoiseModel;

        fn from_diagonal_noise_model_sigmas(sigmas: &mut [f64]) -> SharedPtr<DiagonalNoiseModel>;

        fn cast_diagonal_noise_model_to_base_noise_model(
            a: &SharedPtr<DiagonalNoiseModel>,
        ) -> SharedPtr<BaseNoiseModel>;


        fn from_isotropic_noise_model_dim_and_sigma(dim: usize, sigma: f64) -> SharedPtr<IsotropicNoiseModel>;

        fn cast_isotropic_noise_model_to_base_noise_model(
            a: &SharedPtr<IsotropicNoiseModel>,
        ) -> SharedPtr<BaseNoiseModel>;
    }

    unsafe extern "C++" {
        include!("nonlinear/levenberg_marquardt_optimizer.h");

        type LevenbergMarquardtOptimizer;

        fn new_levenberg_marquardt_optimizer(
            graph: &NonlinearFactorGraph,
            initial_values: &Values,
            params: &LevenbergMarquardtParams,
        ) -> UniquePtr<LevenbergMarquardtOptimizer>;

        fn optimizeSafely(self: Pin<&mut Self>) -> &Values;
    }

    unsafe extern "C++" {
        include!("nonlinear/levenberg_marquardt_params.h");

        type LevenbergMarquardtParams;

        fn default_levenberg_marquardt_params() -> UniquePtr<LevenbergMarquardtParams>;

        fn levenberg_marquardt_params_set_max_iterations(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: u32,
        );

        fn levenberg_marquardt_params_set_lambda_upper_bound(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: f64,
        );

        fn levenberg_marquardt_params_set_lambda_lower_bound(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: f64,
        );


        fn levenberg_marquardt_params_set_diagonal_damping(
            params: Pin<&mut LevenbergMarquardtParams>,
            flag: bool,
        );


        fn levenberg_marquardt_params_set_relative_error_to_l(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: f64,
        );


        fn levenberg_marquardt_params_set_absolute_error_to_l(
            params: Pin<&mut LevenbergMarquardtParams>,
            value: f64,
        );

    }

    unsafe extern "C++" {
        include!("nonlinear/isam2.h");

        type ISAM2;

        fn default_isam2() -> UniquePtr<ISAM2>;

        fn update_noresults(
            isam2: Pin<&mut ISAM2>,
            graph: &NonlinearFactorGraph,
            initial_values: &Values,
        );

        fn calculate_estimate(isam2: &ISAM2) -> UniquePtr<Values>;
    }

    unsafe extern "C++" {
        include!("nonlinear/nonlinear_factor_graph.h");

        type NonlinearFactorGraph;

        fn default_nonlinear_factor_graph() -> UniquePtr<NonlinearFactorGraph>;

        fn nonlinear_factor_graph_resize(graph: Pin<&mut NonlinearFactorGraph>, size: usize);

        fn nonlinear_factor_graph_add_between_factor_pose3(
            graph: Pin<&mut NonlinearFactorGraph>,
            key1: u64,
            key2: u64,
            measured: &Pose3,
            model: &SharedPtr<BaseNoiseModel>,
        );

        fn nonlinear_factor_graph_add_prior_factor_pose3(
            graph: Pin<&mut NonlinearFactorGraph>,
            key: u64,
            prior: &Pose3,
            model: &SharedPtr<BaseNoiseModel>,
        );

        fn nonlinear_factor_graph_add_prior_factor_constant_bias(
            graph: Pin<&mut NonlinearFactorGraph>,
            key: u64,
            prior: &ConstantBias,
            model: &SharedPtr<BaseNoiseModel>,
        );


        fn nonlinear_factor_graph_add_prior_factor_vector3(
            graph: Pin<&mut NonlinearFactorGraph>,
            key: u64,
            prior: &Vector3,
            model: &SharedPtr<BaseNoiseModel>,
        );

        fn nonlinear_factor_graph_add_combined_imu_factor(
            graph: Pin<&mut NonlinearFactorGraph>,
            factor: &SharedPtr<CombinedImuFactor>,
        );

        fn nonlinear_factor_graph_add_smart_projection_pose_factor(
            graph: Pin<&mut NonlinearFactorGraph>,
            factor: &SmartProjectionPoseFactorCal3_S2,
        );
    }

    unsafe extern "C++" {
        include!("nonlinear/values.h");

        type Values;

        fn default_values() -> UniquePtr<Values>;

        fn clear_values(values: Pin<&mut Values>);

        fn values_at_pose3(values: &Values, key: u64) -> &Pose3;

        fn values_at_vector3(values: &Values, key: u64) -> &Vector3;

        fn values_at_constant_bias(values: &Values, key: u64) -> &ConstantBias;

        fn values_exists(values: &Values, key: u64) -> bool;

        fn values_insert_pose3(values: Pin<&mut Values>, key: u64, value: &Pose3);

        fn values_insert_constant_bias(values: Pin<&mut Values>, key: u64, value: &ConstantBias);

        fn values_insert_vector3(values: Pin<&mut Values>, key: u64, value: &Vector3);

    }

    unsafe extern "C++"{
        include!("navigation/combined_imu_factor.h");

        type CombinedImuFactor;
        fn default_combined_imu_factor() -> SharedPtr<CombinedImuFactor>;
        fn new_combined_imu_factor(
            pose_i: u64,
            vel_i: u64,
            pose_j: u64,
            vel_j: u64,
            bias_i: u64,
            bias_j: u64,
            preintegrated_measurements: &PreintegratedCombinedMeasurements,
        ) -> SharedPtr<CombinedImuFactor>;


        type PreintegrationCombinedParams;
        fn new_preintegrated_combined_params_makesharedu() -> SharedPtr<PreintegrationCombinedParams>;
        fn set_accelerometer_covariance(
            params: &mut SharedPtr<PreintegrationCombinedParams>,
            sigma_a_sq: f64,
        );
        fn set_gyroscope_covariance(
            params: &mut SharedPtr<PreintegrationCombinedParams>,
            sigma_g_sq: f64,
        );
        fn bias_acc_covariance(
            params: &mut SharedPtr<PreintegrationCombinedParams>,
            sigma_wa_sq: f64,
        );
        fn bias_omega_covariance(
            params: &mut SharedPtr<PreintegrationCombinedParams>,
            sigma_wg_sq: f64,
        );
        fn set_integration_covariance(
            params: &mut SharedPtr<PreintegrationCombinedParams>,
            val: f64,
        );
        fn bias_acc_omega_int(
            params: &mut SharedPtr<PreintegrationCombinedParams>,
            val: f64,
        );

        type PreintegratedCombinedMeasurements;
        fn default_preintegrated_combined_measurements() -> UniquePtr<PreintegratedCombinedMeasurements>;
        fn new_preintegrated_combined_measurements(
            params: SharedPtr<PreintegrationCombinedParams>,
            bias: &ConstantBias,
        ) -> UniquePtr<PreintegratedCombinedMeasurements>;

        fn integrateMeasurement(
            preintegrated_measurements: Pin<&mut PreintegratedCombinedMeasurements>,
            measured_acc: &Vector3,
            measured_omega: &Vector3,
            dt: f64,
        );

        fn predict(
            preintegrated_measurements: &PreintegratedCombinedMeasurements,
            navstate: &NavState,
            bias: &ConstantBias,
        ) -> UniquePtr<NavState>;

        fn reset_integration_and_set_bias(
            preintegrated_measurements: Pin<&mut PreintegratedCombinedMeasurements>,
            bias: &ConstantBias,
        );
    }

    unsafe extern "C++" {
        include!("navigation/navstate.h");

        type NavState;

        fn new_navstate(pose: &Pose3, velocity: &Vector3) -> UniquePtr<NavState>;
        fn pose(navstate: &NavState) -> &Pose3;
        fn velocity(navstate: &NavState) -> &Vector3;
    }

    unsafe extern "C++" {
        include!("slam/projection_factor.h");

        type SmartProjectionPoseFactorCal3_S2;

        fn new_smart_projection_pose_factor(
            measurement_noise: &SharedPtr<IsotropicNoiseModel>,
            k: &SharedPtr<Cal3_S2>,
            sensor_p_body: &Pose3,
        ) -> UniquePtr<SmartProjectionPoseFactorCal3_S2>;

        fn add(
            smartfactor: Pin<&mut SmartProjectionPoseFactorCal3_S2>,
            point: &Point2,
            key: u64
        );
    }
}
