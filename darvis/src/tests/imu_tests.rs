
mod sim3solver_tests {
    use core::config::load_config;
    use std::env;

    use nalgebra::Vector3;

    use crate::modules::imu::{ImuBias, ImuPreIntegrated, IntegratedRotation};

    
    #[test]
    fn test_imu_preintegrated() {

        let mut system_config = env::current_dir().unwrap();
        system_config.push("orbslam_config.yaml");
        let mut dataset_config = env::current_dir().unwrap();
        dataset_config.push("config_datasets/EUROC.yaml");
        let _ = load_config(
            &system_config.into_os_string().into_string().unwrap(),
            &dataset_config.into_os_string().into_string().unwrap()
        ).expect("Could not load config");

        let bias = ImuBias::new(); // init with all 0s
        let mut preintegrated = ImuPreIntegrated::new(bias);

        // Nga and NgaWalk taken from dataset configuration. For euroc, should be:
        // Nga = {5.78000027e-06, 5.78000027e-06, 5.78000027e-06, 0.000800000038, 0.000800000038, 0.000800000038}
        // Ngawalk = {1.88044198e-12, 1.88044198e-12, 1.88044198e-12, 4.50000037e-08,  4.50000037e-08, 4.50000037e-08}

        // Start off with:
        // everyhting 0s except nga, ngawalk
        // dR = 1, 0, 0, 0, 1, 0, 0, 0, 1

        let acc = Vector3::new(7.95973063, -0.416782618, -2.41897345);
        let ang_vel = Vector3::new(-0.0987856388, 0.13404128, 0.0335103199);
        let dt = 0.00499987602;
        preintegrated.integrate_new_measurement(acc, ang_vel, dt);

        println!("preintegrated: {:?}", preintegrated);

        // dR
        // assert_eq!(preintegrated.get_original_delta_rotation(), nalgebra::Matrix3::new(0.999999762, 0.000167460079, -0.000670200272, -0.000167651044, 0.999999821, -0.000493822154, 0.000670123554, 0.000493980129, 0.999999702));
    
        // dV
        assert!(preintegrated.get_original_delta_velocity() - nalgebra::Vector3::new(0.0397976674, -0.0020838615, -0.0120945675) < nalgebra::Vector3::new(1e-6, 1e-6, 1e-6));

        // dP
        assert!(preintegrated.get_original_delta_position() - nalgebra::Vector3::new(9.94916991e-05, -5.20952472e-06, -3.02356693e-05) < nalgebra::Vector3::new(1e-6, 1e-6, 1e-6));

        // Other values that I'm not testing for...

        // C = {1.4449282e-10, -3.98521764e-18, 
        //   -9.93819352e-19, 0 <repeats 12 times>, -3.98475194e-18, 1.4449282e-10, 1.35275897e-18, 0 <repeats 12 times>, 
        //   -9.96584617e-19, 1.35143404e-18, 1.44492834e-10, 0 <repeats 15 times>, 1.99990087e-08, 0, 0, 4.99962814e-11, 
        //   0 <repeats 12 times>, 1.99990087e-08, 0, 0, 4.99962814e-11, 0 <repeats 12 times>, 1.99990087e-08, 0, 0, 4.99962814e-11, 0, 
        //   0, 0, 0, 0, 0, 0, 0, 0, 4.99962814e-11, 0, 0, 1.24987612e-13, 0 <repeats 12 times>, 4.99962814e-11, 0, 0, 1.24987612e-13, 
        //   0 <repeats 12 times>, 4.99962814e-11, 0, 0, 1.24987612e-13, 0 <repeats 15 times>, 1.88044198e-12, 0 <repeats 15 times>, 
        //   1.88044198e-12, 0 <repeats 15 times>, 1.88044198e-12, 0 <repeats 15 times>, 4.50000037e-08, 0 <repeats 15 times>, 
        //   4.50000037e-08, 0 <repeats 15 times>, 4.50000037e-08}}}},

        // info = all 0s
        // b = all 0s
        // JRg = -0.00499987556, 4.19134011e-07, -1.67536382e-06, -4.18582374e-07, 
        //   -0.00499987556, -1.23485268e-06, 1.67550172e-06, 1.23466543e-06, -0.00499987556
        // JVg = all 0s
        // JVa = -0.00499987602, 0, 0, 0, -0.00499987602, 0, 0, 0, -0.00499987602
        // JPg = all 0s
        // JPa = -1.24993803e-05, 0, 0, 0, -1.24993803e-05, 0, 0, 0, -1.24993803e-05
        // avgA = {7.95973063, -0.416782647, -2.41897345}
        // avgW = -0.0987856314, 0.13404128, 0.0335103199

    }

    #[test]
    fn test_integrated_rotation() {
        let ir = IntegratedRotation::new(
            Vector3::new(-0.0987856388, 0.13404128, 0.0335103199),
            ImuBias::new(),
            0.00499987602
        );

        assert!(
            ir.delta_r - nalgebra::Matrix3::new(
                0.999999762, 0.000167381921, -0.000670231122,
                -0.000167712948, 0.999999881, -0.000493859698,
                0.000670148351, 0.000493972038, 0.999999642
            ) < nalgebra::Matrix3::new(
                1e-6, 1e-6, 1e-6,
                1e-6, 1e-6, 1e-6,
                1e-6, 1e-6, 1e-6
            )
        );
        assert_eq!(ir.right_j, nalgebra::Matrix3::new(0.99999994, -8.38288834e-05, 0.000335081073, 8.37185507e-05, 
            0.99999994, 0.000246976648, -0.000335108663, -0.00024693922, 0.999999881));
    }

    #[test]
    fn test_imu_utils() {
        // input = 0.999999762;, 0.000167381921, 
        //   -0.000670231122, -0.000167712948, 0.999999881, -0.000493859698, 0.000670148351, 0.000493972038, 
        //   0.999999642


        // matrix u = 0.818077385, 0.575106502, -0.00139272108, -0.407460123, 
        //     0.577891409, -0.707119346, -0.40586406, 0.579045773, 0.707092881

        // matrix v = 0.818174362, 0.574969888, -0.000560401008, -0.406889349, 
        //     0.578308821, -0.707106709, -0.40624091, 0.578764558, 0.707106709


        // let r = nalgebra::Matrix3::new(
        // 0.999999762, 0.000167381921, -0.000670231122,
        // -0.000167712948, 0.999999881, -0.000493859698,
        // 0.000670148351, 0.000493972038, 0.999999642
        // );
        // let res = crate::modules::imu::normalize_rotation(r).unwrap();
        // assert_eq!(res, nalgebra::Matrix3::new(
        //     0.999999762, 0.000167460079, -0.000670200272,
        //     -0.000167651044, 0.999999821, -0.000493822154,
        //     0.000670123554, 0.000493980129, 0.999999702
        // ));

        // let v = nalgebra::Vector3::new(0.0397976674, -0.0020838615, -0.0120945675);
        // let res = crate::modules::imu::hat(& v);
        // assert_eq!(res, nalgebra::Matrix3::new(
        //     0.0, -0.0120945675, 0.0020838615,
        //     0.0120945675, 0.0, -0.0397976674,
        //     -0.0020838615, 0.0397976674, 0.0
        // ));

    }
}