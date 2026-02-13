
#![allow(dead_code)]
#![allow(unused_imports)]
#![allow(unused_variables)]

mod gtsamtests {
    use core::{config::load_config, matrix::BHVector3};
    use std::env;

    use nalgebra::{Matrix3, Vector3};
    use opencv::core::Point2f;
    use opencv::imgcodecs;
    use opencv::types::VectorOfPoint2f;
    use crate::image::draw_optical_flow;

    use crate::modules::image;
    use crate::{ImuInitializationData, actors::{tracking_backend_gtsam::GraphSolver, tracking_frontend_gtsam::{GtsamFrontendTrackingState, TrackedFeatures}}, map::{frame::Frame, pose::Pose}, modules::imu::{ImuBias, ImuMeasurements, ImuPoint, PreintegrationGTSAM}};

    #[test]
    fn test_optimization() {
        load_config_files();

        //* Frame 1 */
        println!("======= FRAME 1! =======");
        // Initialize preintegration
        let mut preintegration = PreintegrationGTSAM::default();
        preintegration.initialize(ImuBias::new());
        preintegration.reset_integration_and_set_bias(& ImuBias::new_with(
            BHVector3::new_with(0.0, 0.0, 0.0),
            BHVector3::new_with(0.0, 0.0, 0.0)
        ));

        // Initialize graph solver
        let init_timestamp: f64 = 1403636582313555456.0;
        let init_imu = ImuInitializationData {
            pose: Pose::new(
                Vector3::new(04.71169, -1.78947, 0.685225),
                Matrix3::new(
                    -0.396892, 0.356007, -0.846014,
                    0.173771, 0.934192, 0.311592,
                    0.901268, -0.0233444, -0.432637
                )
            ),
            velocity: BHVector3::<f64>::new_with(-0.023791, -0.016973, 00.091484),
            bias: ImuBias::new_with(
                BHVector3::new_with(-0.003172, 00.021267, 00.078502),
                BHVector3::new_with(-0.025266, 0000.1367, 00.075598),
            )
        };
        let mut graph_solver = GraphSolver::new(1);
        graph_solver.initialize(
            init_timestamp,
            &init_imu,
        ).expect("Failed to initialize?");


        //* Frame 2 (only has preintegration) */
        println!("======= FRAME 2! =======");

        let mut frame2 = Frame::new_no_features(2, None, 1.0, None).expect("Could not create frame!");
        let imu_msmts1 = get_imu_msmts1();
        preintegrate(imu_msmts1, &mut preintegration);

        let mut feature_tracks1 = get_feature_tracks1();
        feature_tracks1.undistorted_points = feature_tracks1.points.clone();

        let _ = graph_solver.solve(
            GtsamFrontendTrackingState::Ok,
            &mut frame2,
            & preintegration.get_preintegration_clone(),
            & feature_tracks1,
            true
        ).unwrap();

        // let (pose, velocity, bias) = graph_solver.values_all.get_results_from_values(graph_solver.curr_id).unwrap();
        preintegration.reset_integration_and_set_bias(
            & ImuBias::new_with(
                BHVector3::new_with(-0.003172,00.021267, 00.078502),
                BHVector3::new_with(-0.025266, 0000.1367, 00.075598),
            )
        );



        //* Frame 3  (Preintegration and first features) */
        println!("======= FRAME 3! =======");

        let mut frame3 = Frame::new_no_features(3, None, 1.0, None).expect("Could not create frame!");
        let imu_msmts2 = get_imu_msmts2();
        preintegrate(imu_msmts2, &mut preintegration);

        let mut feature_tracks2 = get_feature_tracks2();
        feature_tracks2.undistorted_points = feature_tracks2.points.clone();

        let _ = graph_solver.solve(
            GtsamFrontendTrackingState::Ok,
            &mut frame3,
            & preintegration.get_preintegration_clone(),
            & feature_tracks2,
            true
        ).unwrap();

        let (pose, velocity, bias) = graph_solver.values_all.get_results_from_values(graph_solver.curr_id).unwrap();
        preintegration.reset_integration_and_set_bias(
            & ImuBias::new_with(
                BHVector3::new_with(-0.003172,00.021267, 00.078502),
                BHVector3::new_with(-0.025266, 0000.1367, 00.075598),
            )
        );


        //* Frame 4 (actual full optimization) */
        println!("======= FRAME 4! =======");

        let mut frame4 = Frame::new_no_features(4, None, 1.0, None).expect("Could not create frame!");
        let imu_msmts3 = get_imu_msmts3();
        preintegrate(imu_msmts3, &mut preintegration);

        // SOfiya todo this
        let mut feature_tracks3 = get_feature_tracks3();
        feature_tracks3.undistorted_points = feature_tracks3.points.clone();

        let _ = graph_solver.solve(
            GtsamFrontendTrackingState::Ok,
            &mut frame4,
            & preintegration.get_preintegration_clone(),
            & feature_tracks3,
            true
        ).unwrap();




        // let frame1 = image::read_image_file(&"/home/sofiya/datasets/euroc/MH_01_easy/mav0/cam0/data/1403636582513555456.png".to_string(), imgcodecs::IMREAD_GRAYSCALE);
        // let frame2 = image::read_image_file(&"/home/sofiya/datasets/euroc/MH_01_easy/mav0/cam0/data/1403636582713555456.png".to_string(), imgcodecs::IMREAD_GRAYSCALE);
        // fn sort_tracks(tracks: & TrackedFeatures) -> VectorOfPoint2f {
        //     let mut sorted = VectorOfPoint2f::new();
        //     sorted.reserve(tracks.last_feature_id as usize);
        //     for id in tracks.feature_ids.iter() {
        //         sorted.push(tracks.points[*id as usize]);
        //     }
        //     sorted

        // }
        // let tracks1_sorted = sort_tracks(& feature_tracks1);
        // let tracks2_sorted = sort_tracks(& feature_tracks2);

        // draw_optical_flow(
        //     &frame1,
        //     &frame2,
        //     & tracks1_sorted,
        //     & tracks2_sorted,
        //     &format!("results/flow/test.png"),
        // ).unwrap();



    }

    fn load_config_files() {
        let mut system_config = env::current_dir().unwrap();
        system_config.push("config_systems/gtsam_config.yaml");
        let mut dataset_config = env::current_dir().unwrap();
        dataset_config.push("config_datasets/EUROC.yaml");
        let _ = load_config(
            &system_config.into_os_string().into_string().unwrap(),
            &dataset_config.into_os_string().into_string().unwrap(),
        )
        .expect("Could not load config");
    }
    
    fn preintegrate(imu_msmts: ImuMeasurements, preintegration: &mut PreintegrationGTSAM) {
        // Copying contents of preintegration function so we don't have to worry about that last value
        // or anything about timestamp/timestep units
        for i in 0..imu_msmts.len() {
            let tstep = imu_msmts[i].timestamp;
            let acc: Vector3<f64> = imu_msmts[i].acc; // acc
            let ang_vel: Vector3<f64> = imu_msmts[i].ang_vel; // angVel
            // println!("Preintegrating {} measurement:  Acc: {} {} {}, Omega: {} {} {}, dt: {}",
            //     i,
            //     acc.x, acc.y, acc.z, // acc
            //     ang_vel.x, ang_vel.y, ang_vel.z, // angVel
            //     tstep
            // );

            preintegration.preint_gtsam.integrate_measurement(&acc.into(), &ang_vel.into(), tstep);
        }

    }


    fn get_imu_msmts1() -> ImuMeasurements{
        let mut msmts = ImuMeasurements::new();
        msmts.push_back(ImuPoint { acc: Vector3::new(11.9968, -0.22065, -5.54893), ang_vel: Vector3::new(0.0467748, 0.0453786, 0.111003), timestamp: 0.00499994}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.866, -0.22065, -5.45904), ang_vel: Vector3::new(0.0467748, 0.0656244, 0.117984), timestamp: 0.00500019}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.8088, -0.261511, -5.43452), ang_vel: Vector3::new(0.0418879, 0.0802851, 0.122173), timestamp: 0.00499994}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.7516, -0.236994, -5.42635), ang_vel: Vector3::new(0.037001, 0.0935496, 0.119381), timestamp: 0.00499994}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.6617, -0.318716, -5.37731), ang_vel: Vector3::new(0.0404916, 0.101927, 0.114494), timestamp: 0.00499994}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.6536, -0.384094, -5.45904), ang_vel: Vector3::new(0.0376991, 0.124966, 0.10472), timestamp: 0.00500019}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.5637, -0.335061, -5.41), ang_vel: Vector3::new(0.0376991, 0.134041, 0.102625), timestamp: 0.00499994}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.4166, -0.302372, -5.32011), ang_vel: Vector3::new(0.0397935, 0.152891, 0.0998328), timestamp: 0.00499994}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.3594, -0.294199, -5.27925), ang_vel: Vector3::new(0.0390954, 0.172439, 0.101229), timestamp: 0.00499994}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.2041, -0.310544, -5.22204), ang_vel: Vector3::new(0.0390954, 0.185005, 0.0998328), timestamp: 0.00500019}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(11.1142033, -0.269682875, -5.14849125), ang_vel: Vector3::new(0.0439822972, 0.205948852, 0.0998328332), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.942587, -0.236994042, -5.08311358), ang_vel: Vector3::new(0.0397935069, 0.227590934, 0.0942477796), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.8445205, -0.155271958, -5.08311358), ang_vel: Vector3::new(0.0390953752, 0.245044227, 0.0893608577), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.6892485, -0.0163444167, -5.01773592), ang_vel: Vector3::new(0.0363028484, 0.255516202, 0.0928515162), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.550321, 0.024516625, -4.94418604), ang_vel: Vector3::new(0.0286233997, 0.272969495, 0.0879645943), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.5258043, 0.0980665, -4.903325), ang_vel: Vector3::new(0.0181514242, 0.285535866, 0.0865683309), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.4931155, 0.171616375, -4.83794733), ang_vel: Vector3::new(0.00837758041, 0.293913446, 0.0809832773), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.4113934, 0.122583125, -4.79708629), ang_vel: Vector3::new(-0.0013962634, 0.304385422, 0.0684169067), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.4195656, 0.073549875, -4.74805304), ang_vel: Vector3::new(-0.00977384381, 0.313461134, 0.0600393263), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.3868768, 0.0572054583, -4.64181433), ang_vel: Vector3::new(-0.0160570291, 0.319046187, 0.053756141), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.3214991, 0.0326888333, -4.52740342), ang_vel: Vector3::new(-0.0195476876, 0.327423768, 0.0558505361), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.2561215, -0.114410917, -4.42116471), ang_vel: Vector3::new(-0.0202458193, 0.330216294, 0.0642281165), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.2724659, -0.179788583, -4.35578704), ang_vel: Vector3::new(-0.0118682389, 0.335103216, 0.0788888822), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.1090217, -0.187960792, -4.20868729), ang_vel: Vector3::new(-0.0027925268, 0.337197611, 0.0998328332), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.0518162, -0.187960792, -4.10244858), ang_vel: Vector3::new(0.0041887902, 0.335801348, 0.119380521), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.0599885, -0.14709975, -4.10244858), ang_vel: Vector3::new(0.00907571211, 0.33231069, 0.12705997), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.0599885, -0.073549875, -4.06158754), ang_vel: Vector3::new(0.00977384381, 0.328820031, 0.12705997), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.1090217, -0.0408610417, -4.06975975), ang_vel: Vector3::new(0.00488692191, 0.335103216, 0.113097336), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.158055, 0.0572054583, -4.13513742), ang_vel: Vector3::new(0.000698131701, 0.335801348, 0.10262536), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(10.1008495, 0.155271958, -4.13513742), ang_vel: Vector3::new(-0.0111701072, 0.333008821, 0.0851720675), timestamp: 0.005000192}); //1403636582458555392
        msmts.push_back(ImuPoint { acc: Vector3::new(10.002783, 0.302371708, -4.13513742), ang_vel: Vector3::new(-0.016755160819145562,0.32602750427254074,0.063529984772593598), timestamp: 0.004999936}); // 1403636582463555584
        msmts.push_back(ImuPoint { acc: Vector3::new(9.96192196, 0.212477417, -4.16782625), ang_vel: Vector3::new(-0.0251327412, 0.318348056, 0.0411897703), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.97826638, 0.228821833, -4.20868729), ang_vel: Vector3::new(-0.0404916386, 0.308574212, 0.0237364778), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.92923312, 0.261510667, -4.17599846), ang_vel: Vector3::new(-0.0530580093, 0.2994985, 0.00837758041), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.89654429, 0.171616375, -4.14330962), ang_vel: Vector3::new(-0.0593411946, 0.289026524, -0.0041887902), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.87202767, 0.220649625, -4.11062079), ang_vel: Vector3::new(-0.0649262482, 0.281347075, -0.0188495559), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.83116662, 0.114410917, -4.08610417), ang_vel: Vector3::new(-0.0691150384, 0.272969495, -0.0202458193), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.82299442, -0.0572054583, -4.118793), ang_vel: Vector3::new(-0.074700092, 0.265988178, -0.0237364778), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.82299442, -0.0408610417, -4.11062079), ang_vel: Vector3::new(-0.0781907505, 0.261101256, -0.0202458193), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.71675571, -0.0980665, -4.04524312), ang_vel: Vector3::new(-0.0802851456, 0.258308729, -0.00767944871), timestamp: 0.004999936}); 
        msmts
    }

    fn get_imu_msmts2() -> ImuMeasurements {
        let mut msmts = ImuMeasurements::new();
        msmts.push_back(ImuPoint { acc: Vector3::new(9.56965596, -0.163444167, -3.98803767), ang_vel: Vector3::new(-0.0844739358, 0.256912466, 0.0125663706), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.54513933, -0.302371708, -3.94717662), ang_vel: Vector3::new(-0.0809832773, 0.245742359, 0.0342084533), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.49610608, -0.286027292, -3.93083221), ang_vel: Vector3::new(-0.0719075652, 0.240855437, 0.0614355897), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.34083412, -0.212477417, -3.93083221), ang_vel: Vector3::new(-0.0684169067, 0.235968515, 0.0788888822), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.27545646, -0.114410917, -3.90631558), ang_vel: Vector3::new(-0.0635299848, 0.231081593, 0.0900589894), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.21007879, 0.0653776667, -3.93083221), ang_vel: Vector3::new(-0.0621337214, 0.227590934, 0.0928515162), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.1201845, 0.138927542, -3.91448779), ang_vel: Vector3::new(-0.0719075652, 0.222704013, 0.0872664626), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.03846242, 0.138927542, -3.88179896), ang_vel: Vector3::new(-0.0802851456, 0.217817091, 0.074700092), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.00577358, 0.204305208, -3.81642129), ang_vel: Vector3::new(-0.0844739358, 0.208043247, 0.0670206433), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.97308475, 0.253338458, -3.80007687), ang_vel: Vector3::new(-0.0879645943, 0.198269403, 0.0572467995), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.07115125, 0.24516625, -3.80824908), ang_vel: Vector3::new(-0.0865683309, 0.187099296, 0.0474729557), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.218251, 0.335060542, -3.8245935), ang_vel: Vector3::new(-0.0949459113, 0.174532925, 0.0321140582), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.34900633, 0.392266, -3.80824908), ang_vel: Vector3::new(-0.106116019, 0.16406095, 0.0041887902), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.47158946, 0.408610417, -3.79190467), ang_vel: Vector3::new(-0.117984257, 0.155683369, -0.0230383461), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.55331154, 0.416782625, -3.78373246), ang_vel: Vector3::new(-0.127758101, 0.146607657, -0.0376991118), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.48793387, 0.302371708, -3.74287142), ang_vel: Vector3::new(-0.13962634, 0.14381513, -0.046774824), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.46341725, 0.0898942917, -3.71835479), ang_vel: Vector3::new(-0.137531945, 0.13543755, -0.0439822972), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.47158946, -0.0817220833, -3.77556025), ang_vel: Vector3::new(-0.136833813, 0.132645023, -0.027925268), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.38169517, -0.155271958, -3.79190467), ang_vel: Vector3::new(-0.132645023, 0.12705997, -0.0020943951), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.26728425, -0.179788583, -3.79190467), ang_vel: Vector3::new(-0.119380521, 0.130550628, 0.0342084533), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.18556217, -0.14709975, -3.79190467), ang_vel: Vector3::new(-0.117286126, 0.122173048, 0.0565486678), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.05480683, 0.0163444167, -3.80824908), ang_vel: Vector3::new(-0.113097336, 0.121474916, 0.0837758041), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.99760137, 0.0408610417, -3.84093792), ang_vel: Vector3::new(-0.113795467, 0.120078653, 0.100530965), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.86684604, 0.073549875, -3.84093792), ang_vel: Vector3::new(-0.121474916, 0.118682389, 0.095644043), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.80964058, 0.122583125, -3.80824908), ang_vel: Vector3::new(-0.128456233, 0.117286126, 0.0823795407), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.60533537, 0.122583125, -3.75104362), ang_vel: Vector3::new(-0.129852496, 0.115191731, 0.0753982237), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.61350758, 0.073549875, -3.77556025), ang_vel: Vector3::new(-0.124965574, 0.115889862, 0.0628318531), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.61350758, 0.04903325, -3.75921583), ang_vel: Vector3::new(-0.118682389, 0.117286126, 0.0586430629), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.58899096, 0.0817220833, -3.69383817), ang_vel: Vector3::new(-0.105417887, 0.116587994, 0.0628318531), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.56447433, 0.0, -3.6284605), ang_vel: Vector3::new(-0.0879645943, 0.122871179, 0.0698131701), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.59716317, 0.0653776667, -3.53856621), ang_vel: Vector3::new(-0.0705113018, 0.120078653, 0.0809832773), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.65436862, 0.138927542, -3.47318854), ang_vel: Vector3::new(-0.0579449312, 0.117984257, 0.0893608577), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.7279185, 0.14709975, -3.40781087), ang_vel: Vector3::new(-0.0495673508, 0.118682389, 0.0970403064), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.86684604, 0.212477417, -3.34243321), ang_vel: Vector3::new(-0.0453785606, 0.11100294, 0.095644043), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.10384008, 0.286027292, -3.24436671), ang_vel: Vector3::new(-0.0411897703, 0.111701072, 0.0984365698), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.218251, 0.286027292, -3.17898904), ang_vel: Vector3::new(-0.0432841654, 0.113097336, 0.101927228), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.43072842, 0.24516625, -3.12178358), ang_vel: Vector3::new(-0.0453785606, 0.115191731, 0.10262536), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.56965596, 0.253338458, -3.08092254), ang_vel: Vector3::new(-0.0509636142, 0.117286126, 0.110304809), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.63503363, 0.277855083, -3.03188929), ang_vel: Vector3::new(-0.0516617459, 0.120776784, 0.120776784), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.66772246, 0.277855083, -3.06457812), ang_vel: Vector3::new(-0.0579449312, 0.122871179, 0.13124876), timestamp: 0.004999936}); 
        msmts
    }

    fn get_imu_msmts3() -> ImuMeasurements {
        let mut msmts = ImuMeasurements::new();
        msmts.push_back(ImuPoint { acc: Vector3::new(9.72492792, 0.310543917, -3.11361138), ang_vel: Vector3::new(-0.0663225116, 0.129852496, 0.147305789), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.610517, 0.277855083, -3.17081683), ang_vel: Vector3::new(-0.0740019603, 0.138230077, 0.153588974), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.54513933, 0.277855083, -3.30157217), ang_vel: Vector3::new(-0.081681409, 0.148702052, 0.152192711), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.34900633, 0.261510667, -3.41598308), ang_vel: Vector3::new(-0.0809832773, 0.153588974, 0.157777764), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.06297904, 0.196133, -3.48136075), ang_vel: Vector3::new(-0.0879645943, 0.162664686, 0.162664686), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.81781279, 0.122583125, -3.51404958), ang_vel: Vector3::new(-0.0900589894, 0.16824974, 0.161966555), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.63802421, 0.0898942917, -3.530394), ang_vel: Vector3::new(-0.0935496479, 0.16824974, 0.158475896), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.44189121, 0.130755333, -3.46501633), ang_vel: Vector3::new(-0.0949459113, 0.16824974, 0.152192711), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.36016913, 0.0980665, -3.42415529), ang_vel: Vector3::new(-0.0935496479, 0.16824974, 0.154985238), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.36834133, 0.196133, -3.36694983), ang_vel: Vector3::new(-0.0928515162, 0.17243853, 0.164759081), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.41737458, 0.269682875, -3.30157217), ang_vel: Vector3::new(-0.0963421747, 0.171740398, 0.178023584), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.54812992, 0.196133, -3.22802229), ang_vel: Vector3::new(-0.10262536, 0.177325452, 0.184306769), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.80146837, 0.14709975, -3.18716125), ang_vel: Vector3::new(-0.110304809, 0.193382481, 0.189193691), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.98942917, 0.073549875, -3.21167787), ang_vel: Vector3::new(-0.115191731, 0.204552588, 0.191288086), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.15287333, 0.073549875, -3.22802229), ang_vel: Vector3::new(-0.115191731, 0.219911486, 0.193382481), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.32448971, 0.228821833, -3.22802229), ang_vel: Vector3::new(-0.119380521, 0.230383461, 0.188495559), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.47976167, 0.351404958, -3.30974438), ang_vel: Vector3::new(-0.126361838, 0.237364778, 0.187099296), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.47158946, 0.416782625, -3.30157217), ang_vel: Vector3::new(-0.14381513, 0.235270383, 0.181514242), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.58600037, 0.367749375, -3.30974438), ang_vel: Vector3::new(-0.16406095, 0.232477856, 0.178023584), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.73310012, 0.2941995, -3.41598308), ang_vel: Vector3::new(-0.182212374, 0.235968515, 0.175929189), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.74127233, 0.236994042, -3.44867192), ang_vel: Vector3::new(-0.189193691, 0.2422517, 0.173136662), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.67589467, 0.0326888333, -3.4323275), ang_vel: Vector3::new(-0.19687314, 0.254119939, 0.166155345), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.57782817, 0.0163444167, -3.48136075), ang_vel: Vector3::new(-0.198967535, 0.263893783, 0.16406095), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(9.26728425, -0.00817220833, -3.48136075), ang_vel: Vector3::new(-0.198967535, 0.274365758, 0.165457213), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.9240515, -0.0817220833, -3.44867192), ang_vel: Vector3::new(-0.202458193, 0.290422788, 0.169646003), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.64619642, -0.187960792, -3.54673842), ang_vel: Vector3::new(-0.197571271, 0.297404105, 0.180816111), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.3356525, -0.2941995, -3.61211608), ang_vel: Vector3::new(-0.185004901, 0.302291026, 0.191986218), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.03328079, -0.335060542, -3.60394387), ang_vel: Vector3::new(-0.173136662, 0.30787608, 0.198269403), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(7.87800883, -0.326888333, -3.55491062), ang_vel: Vector3::new(-0.159174028, 0.316951792, 0.200363798), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(7.81263117, -0.220649625, -3.52222179), ang_vel: Vector3::new(-0.154985238, 0.317649924, 0.194778745), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(7.75542571, -0.179788583, -3.44049971), ang_vel: Vector3::new(-0.152890842, 0.322536846, 0.186401164), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(7.76359792, -0.286027292, -3.36694983), ang_vel: Vector3::new(-0.149400184, 0.329518163, 0.181514242), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(7.86166442, -0.335060542, -3.27705554), ang_vel: Vector3::new(-0.148702052, 0.333706953, 0.180816111), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(7.91886987, -0.367749375, -3.19533346), ang_vel: Vector3::new(-0.141720735, 0.339990138, 0.186401164), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(7.92704208, -0.392266, -3.07275033), ang_vel: Vector3::new(-0.129852496, 0.349763982, 0.193382481), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.05779742, -0.253338458, -3.03188929), ang_vel: Vector3::new(-0.115889862, 0.349763982, 0.20525072), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.08231404, -0.0572054583, -2.99920046), ang_vel: Vector3::new(-0.108908545, 0.357443431, 0.2136283), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.22124158, 0.0, -2.97468383), ang_vel: Vector3::new(-0.111701072, 0.366519143, 0.20943951), timestamp: 0.005000192}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.3356525, 0.00817220833, -2.99102825), ang_vel: Vector3::new(-0.115889862, 0.367217275, 0.194080613), timestamp: 0.004999936}); 
        msmts.push_back(ImuPoint { acc: Vector3::new(8.31930808, 0.0408610417, -2.95016721), ang_vel: Vector3::new(-0.118682389, 0.368613538, 0.161966555), timestamp: 0.004999936}); 
        msmts
    }

    fn get_feature_tracks1() -> TrackedFeatures {
        let mut feature_tracks = TrackedFeatures::default();
        // Bearing vectors are all 0 here because they're only used by frontend
        feature_tracks.add_with_id(10, Point2f::new(176.240585, 206.457779), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(11, Point2f::new(164.030502, 314.438721), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(24, Point2f::new(751.0, 315.240906), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(27, Point2f::new(365.366699, 167.864075), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(35, Point2f::new(220.691895, 268.948761), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(37, Point2f::new(304.842712, 233.22728), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(46, Point2f::new(138.282471, 343.100311), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(48, Point2f::new(11.0614567, 379.510956), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(60, Point2f::new(373.910797, 144.029907), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(61, Point2f::new(0.0, 394.339722), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(68, Point2f::new(0.0, 434.423523), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(69, Point2f::new(183.894852, 282.822357), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(77, Point2f::new(131.31459, 323.784882), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(81, Point2f::new(161.216858, 250.833649), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(82, Point2f::new(74.2532272, 357.869324), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(83, Point2f::new(86.0919418, 322.076691), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(84, Point2f::new(29.851469, 393.537537), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(86, Point2f::new(0.0, 356.230225), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(88, Point2f::new(0.0, 220.333084), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(89, Point2f::new(696.346619, 265.150848), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(90, Point2f::new(215.110504, 319.964355), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(92, Point2f::new(22.6360435, 349.853516), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(96, Point2f::new(381.782837, 337.961426), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(97, Point2f::new(129.520981, 292.868591), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(99, Point2f::new(726.015076, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(102, Point2f::new(236.873856, 295.093933), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(104, Point2f::new(592.453796, 187.716949), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(106, Point2f::new(31.5815334, 327.539062), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(107, Point2f::new(181.298172, 304.492218), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(109, Point2f::new(0.0, 335.356415), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(116, Point2f::new(180.031494, 335.096283), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(120, Point2f::new(0.0, 413.478363), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(131, Point2f::new(544.62561, 416.448029), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(135, Point2f::new(114.371262, 366.870422), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(137, Point2f::new(504.335876, 385.600067), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(141, Point2f::new(303.423248, 306.249969), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(142, Point2f::new(209.809479, 299.041718), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(145, Point2f::new(524.029785, 290.30777), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(147, Point2f::new(569.657471, 373.254913), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(148, Point2f::new(739.446228, 465.214325), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(150, Point2f::new(340.270294, 322.970184), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(157, Point2f::new(580.489319, 351.859192), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(158, Point2f::new(639.49292, 458.781006), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(167, Point2f::new(485.249512, 324.74762), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(168, Point2f::new(284.749023, 342.55069), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(173, Point2f::new(573.264465, 314.803406), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(174, Point2f::new(616.537292, 433.558411), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(181, Point2f::new(269.619202, 362.894165), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(184, Point2f::new(580.772339, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(185, Point2f::new(716.832031, 365.263367), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(188, Point2f::new(546.4104, 344.868317), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(189, Point2f::new(697.369751, 470.103943), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(190, Point2f::new(451.200012, 306.689453), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(194, Point2f::new(372.901062, 295.276337), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(200, Point2f::new(673.314087, 333.985443), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(206, Point2f::new(602.958557, 296.062958), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(210, Point2f::new(520.157776, 368.286133), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(217, Point2f::new(227.26944, 364.954834), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(218, Point2f::new(317.66687, 331.367493), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(219, Point2f::new(649.677246, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(220, Point2f::new(279.981018, 447.752258), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(311, Point2f::new(372.985535, 8.94408226), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(312, Point2f::new(751.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(313, Point2f::new(751.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(314, Point2f::new(597.937866, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(315, Point2f::new(320.236145, 16.1383057), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(316, Point2f::new(167.385025, 20.6029873), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(317, Point2f::new(630.387329, 155.990311), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(318, Point2f::new(201.434753, 28.7033691), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(319, Point2f::new(41.8503189, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(320, Point2f::new(42.1622276, 20.9585323), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(321, Point2f::new(179.779266, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(322, Point2f::new(242.91597, 31.1761208), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(323, Point2f::new(132.647491, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(324, Point2f::new(84.0944748, 6.42525291), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(325, Point2f::new(138.024551, 30.0853481), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(326, Point2f::new(13.0815172, 3.94986486), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(327, Point2f::new(751.0, 237.181686), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(328, Point2f::new(365.887726, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(329, Point2f::new(156.082748, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(330, Point2f::new(533.601318, 39.0278969), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(331, Point2f::new(477.608948, 61.2269211), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(332, Point2f::new(656.401306, 34.3765984), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(333, Point2f::new(0.0, 1.03609037), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(334, Point2f::new(369.134125, 100.981636), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(335, Point2f::new(587.924377, 143.329651), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(336, Point2f::new(13.4258137, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(337, Point2f::new(313.965851, 184.793594), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(338, Point2f::new(122.90757, 8.45714474), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(339, Point2f::new(530.613831, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(340, Point2f::new(435.564789, 46.9638138), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(341, Point2f::new(497.677368, 159.944214), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(342, Point2f::new(397.530273, 63.3075714), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(343, Point2f::new(455.173309, 131.187637), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(344, Point2f::new(751.0, 208.239578), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(345, Point2f::new(198.910812, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(346, Point2f::new(429.631073, 136.8853), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(347, Point2f::new(338.957336, 163.56395), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(348, Point2f::new(238.996979, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(349, Point2f::new(575.051453, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(350, Point2f::new(205.143799, 135.332581), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(351, Point2f::new(522.291077, 163.54483), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(352, Point2f::new(307.599335, 67.6146622), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(353, Point2f::new(258.681366, 225.915771), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(354, Point2f::new(614.302979, 25.1697845), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(355, Point2f::new(624.569885, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(356, Point2f::new(226.691986, 227.754639), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(357, Point2f::new(276.267487, 121.691811), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(358, Point2f::new(618.653137, 129.353745), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(359, Point2f::new(610.539734, 173.908203), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(360, Point2f::new(397.634583, 95.2044754), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(361, Point2f::new(746.016663, 56.4414368), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(362, Point2f::new(336.595367, 102.815903), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(363, Point2f::new(256.089722, 189.849411), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(364, Point2f::new(406.127258, 45.9048538), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(365, Point2f::new(460.318787, 93.7548523), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(366, Point2f::new(355.236725, 66.1057587), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(367, Point2f::new(403.647156, 139.050858), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(368, Point2f::new(360.731598, 221.144714), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(369, Point2f::new(241.48288, 130.00209), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(370, Point2f::new(293.115265, 154.877563), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(371, Point2f::new(539.136475, 101.460114), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(372, Point2f::new(432.216248, 92.3166962), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(373, Point2f::new(313.79422, 252.012802), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(374, Point2f::new(197.176682, 214.287216), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(375, Point2f::new(593.018188, 97.8740463), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(376, Point2f::new(472.921844, 99.9139099), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(377, Point2f::new(105.681023, 183.154938), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(378, Point2f::new(275.97467, 238.889908), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(379, Point2f::new(402.189667, 167.882584), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(380, Point2f::new(18.692831, 196.627533), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(381, Point2f::new(155.595352, 195.476593), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(382, Point2f::new(315.647095, 139.243271), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(383, Point2f::new(0.0, 442.838074), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(384, Point2f::new(738.255493, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(385, Point2f::new(71.315506, 157.352768), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(386, Point2f::new(625.806396, 270.538025), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(387, Point2f::new(165.579086, 372.046021), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(388, Point2f::new(328.589172, 210.402771), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(389, Point2f::new(684.767517, 70.6401672), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(390, Point2f::new(303.334839, 133.222015), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(391, Point2f::new(357.327026, 17.2985268), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(392, Point2f::new(62.1724701, 246.7845), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(393, Point2f::new(90.0343094, 244.615524), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(394, Point2f::new(673.847961, 242.757431), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(395, Point2f::new(460.493347, 18.0154171), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(396, Point2f::new(463.028625, 54.0424347), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(397, Point2f::new(751.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(398, Point2f::new(681.309509, 138.524765), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(399, Point2f::new(184.118744, 183.855042), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(400, Point2f::new(0.0, 442.626587), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(401, Point2f::new(155.899506, 172.74736), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(402, Point2f::new(590.831177, 218.85408), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(403, Point2f::new(0.0, 309.871582), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(404, Point2f::new(566.680542, 169.352402), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(405, Point2f::new(38.3943939, 167.186218), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(406, Point2f::new(658.623596, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(407, Point2f::new(71.7935715, 275.866699), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(408, Point2f::new(0.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(409, Point2f::new(374.683899, 188.518387), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(410, Point2f::new(100.471649, 275.348724), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(411, Point2f::new(43.7177467, 231.327942), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(412, Point2f::new(493.399231, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(413, Point2f::new(206.865143, 187.665344), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(414, Point2f::new(164.21022, 148.884018), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(415, Point2f::new(0.0, 220.041748), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(416, Point2f::new(54.6542435, 405.814911), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(417, Point2f::new(660.165527, 279.568512), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(418, Point2f::new(340.00705, 83.9958115), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(419, Point2f::new(184.192917, 363.813599), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(420, Point2f::new(751.0, 266.109253), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(421, Point2f::new(489.132751, 348.070068), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(422, Point2f::new(303.589935, 212.442429), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(423, Point2f::new(559.328369, 252.134995), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(424, Point2f::new(751.0, 278.244385), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(425, Point2f::new(667.71637, 215.09848), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(426, Point2f::new(299.46814, 284.279266), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(427, Point2f::new(101.259445, 392.489563), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(428, Point2f::new(155.568481, 110.889259), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(429, Point2f::new(327.900238, 267.046539), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(430, Point2f::new(0.0, 280.568054), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(431, Point2f::new(24.4731255, 440.113861), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(432, Point2f::new(27.202076, 286.020996), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(433, Point2f::new(640.360962, 244.291656), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(434, Point2f::new(692.843079, 186.851929), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(435, Point2f::new(751.0, 16.1438046), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(436, Point2f::new(741.698425, 255.420349), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(437, Point2f::new(493.684967, 199.035263), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(438, Point2f::new(296.68811, 41.3265724), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(439, Point2f::new(751.0, 93.4901199), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(440, Point2f::new(134.287476, 244.930634), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(441, Point2f::new(751.0, 158.158615), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(442, Point2f::new(154.510529, 279.827057), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(443, Point2f::new(230.865387, 337.620422), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(444, Point2f::new(65.0676422, 314.892151), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(445, Point2f::new(181.477798, 260.292053), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(446, Point2f::new(445.77179, 398.89209), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(447, Point2f::new(751.0, 213.665375), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(448, Point2f::new(0.0, 409.014557), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(449, Point2f::new(317.075806, 290.097687), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(450, Point2f::new(690.93866, 34.6631737), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(451, Point2f::new(333.635773, 422.035919), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(452, Point2f::new(0.0, 466.83255), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(453, Point2f::new(0.0, 34.4694633), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(454, Point2f::new(483.438477, 459.293427), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(455, Point2f::new(725.43512, 102.039627), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(456, Point2f::new(366.48941, 447.143585), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(457, Point2f::new(0.0, 351.538727), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(458, Point2f::new(600.519592, 444.085358), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(459, Point2f::new(259.857483, 308.186951), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(460, Point2f::new(126.41748, 377.988861), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(461, Point2f::new(0.0, 372.882446), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(462, Point2f::new(0.0, 314.256165), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(463, Point2f::new(437.780426, 230.681122), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(464, Point2f::new(280.816162, 301.72644), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(465, Point2f::new(506.774292, 215.091003), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(466, Point2f::new(444.462921, 368.441162), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(467, Point2f::new(99.0363998, 349.032166), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(468, Point2f::new(751.0, 18.4490566), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(469, Point2f::new(246.623459, 267.36795), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(470, Point2f::new(751.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(471, Point2f::new(94.9498138, 297.414917), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(472, Point2f::new(751.0, 326.897339), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(473, Point2f::new(56.9281502, 340.224579), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(474, Point2f::new(751.0, 62.9095535), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(475, Point2f::new(598.565918, 371.374878), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(476, Point2f::new(336.961304, 242.992722), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(477, Point2f::new(751.0, 63.6250496), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(478, Point2f::new(324.699402, 307.414886), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(479, Point2f::new(542.529297, 372.230804), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(480, Point2f::new(0.0, 466.707703), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(481, Point2f::new(304.703156, 446.16275), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(482, Point2f::new(649.158875, 313.227814), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(483, Point2f::new(751.0, 357.549805), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(484, Point2f::new(751.0, 127.988579), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(485, Point2f::new(112.821373, 404.657013), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(486, Point2f::new(564.744812, 433.612244), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(487, Point2f::new(741.921753, 336.221649), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(488, Point2f::new(466.745941, 353.903351), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(489, Point2f::new(746.818542, 375.958282), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(490, Point2f::new(194.389206, 429.262512), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(491, Point2f::new(521.198914, 445.214752), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(492, Point2f::new(751.0, 336.204987), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(493, Point2f::new(525.175049, 261.416779), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(494, Point2f::new(345.954956, 282.070648), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(495, Point2f::new(534.490784, 243.42421), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(496, Point2f::new(655.203247, 353.744263), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(497, Point2f::new(68.844696, 183.784134), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(498, Point2f::new(417.934723, 354.959534), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(499, Point2f::new(626.192749, 363.130737), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(500, Point2f::new(751.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(501, Point2f::new(751.0, 424.947357), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(502, Point2f::new(382.210236, 365.01889), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(503, Point2f::new(390.216858, 426.243073), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(504, Point2f::new(601.805664, 471.337494), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(505, Point2f::new(371.280243, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(506, Point2f::new(398.640747, 452.823639), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(507, Point2f::new(431.490356, 288.302338), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(508, Point2f::new(139.877106, 459.263336), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(509, Point2f::new(470.80899, 380.860718), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(510, Point2f::new(413.998871, 399.918701), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(511, Point2f::new(309.572662, 419.174927), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(512, Point2f::new(533.810486, 310.527985), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(513, Point2f::new(105.689529, 474.840607), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(514, Point2f::new(286.502075, 389.235199), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(515, Point2f::new(751.0, 399.998962), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(516, Point2f::new(495.83194, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(517, Point2f::new(349.541168, 387.600067), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(518, Point2f::new(434.11322, 426.266907), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(519, Point2f::new(308.822205, 357.187195), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(520, Point2f::new(0.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(521, Point2f::new(126.649834, 419.625946), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(522, Point2f::new(0.0, 61.3864822), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(523, Point2f::new(517.571472, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(524, Point2f::new(0.0, 159.826324), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(525, Point2f::new(475.052979, 255.087997), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(526, Point2f::new(621.045898, 388.474579), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(527, Point2f::new(85.3948517, 418.437531), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(528, Point2f::new(440.526947, 246.987762), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(529, Point2f::new(684.53241, 312.049164), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(530, Point2f::new(10.0808306, 60.5040359), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(531, Point2f::new(624.153687, 335.852905), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(532, Point2f::new(178.34552, 380.852264), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(533, Point2f::new(611.085999, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(534, Point2f::new(751.0, 473.530273), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(535, Point2f::new(231.353043, 434.740723), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(536, Point2f::new(212.467499, 416.475281), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(537, Point2f::new(258.122162, 392.281403), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(538, Point2f::new(751.0, 372.961853), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(539, Point2f::new(202.476395, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(540, Point2f::new(339.596588, 349.470459), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(541, Point2f::new(751.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(542, Point2f::new(694.925232, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(543, Point2f::new(39.3294182, 143.689423), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(544, Point2f::new(163.901672, 443.830994), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(545, Point2f::new(305.042114, 390.462189), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(546, Point2f::new(238.99234, 389.599976), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(547, Point2f::new(262.370117, 338.245575), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(548, Point2f::new(751.0, 421.404968), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(549, Point2f::new(751.0, 475.637634), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(550, Point2f::new(434.273987, 322.763184), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(551, Point2f::new(668.390503, 403.830627), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(552, Point2f::new(327.006927, 379.221375), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(553, Point2f::new(429.733765, 379.632782), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(554, Point2f::new(453.830292, 452.555115), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(555, Point2f::new(662.457947, 418.117981), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks
    }


    fn get_feature_tracks2() -> TrackedFeatures {
        let mut feature_tracks = TrackedFeatures::default();

        feature_tracks.add_with_id(11, Point2f::new(164.392288, 417.944672), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(27, Point2f::new(355.989014, 200.530151), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(35, Point2f::new(217.092606, 360.766968), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(37, Point2f::new(297.277252, 297.425476), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(46, Point2f::new(140.292847, 449.497559), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(48, Point2f::new(19.6316605, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(61, Point2f::new(0.0,479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(69, Point2f::new(183.402542, 383.068146), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(77, Point2f::new(133.290573, 431.963135), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(81, Point2f::new(161.668137, 349.924011), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(82, Point2f::new(79.9788742, 471.996613), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(83, Point2f::new(91.4441452, 435.127808), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(84, Point2f::new(37.0765533, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(86, Point2f::new(0.0,479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(88, Point2f::new(0.0,313.183899), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(89, Point2f::new(664.141846, 353.487549), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(90, Point2f::new(212.55658, 417.5625), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(92, Point2f::new(32.1241455, 468.077576), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(96, Point2f::new(368.870209, 454.701294), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(97, Point2f::new(131.77095, 395.114899), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(102, Point2f::new(232.886948, 387.917572), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(104, Point2f::new(576.139221, 229.283234), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(106, Point2f::new(40.4418678, 446.004089), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(107, Point2f::new(180.385773, 405.112701), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(109, Point2f::new(0.0,459.724854), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(116, Point2f::new(179.320633, 436.631531), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(135, Point2f::new(117.679848, 477.042755), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(142, Point2f::new(207.626923, 397.714844), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(147, Point2f::new(540.350342, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(157, Point2f::new(551.586975, 464.859772), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(167, Point2f::new(464.674774, 436.540405), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(168, Point2f::new(278.212921, 463.609741), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(173, Point2f::new(546.712646, 422.049347), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(181, Point2f::new(264.596527, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(185, Point2f::new(675.27594, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(188, Point2f::new(520.554688, 459.192993), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(190, Point2f::new(433.614807, 411.296356), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(200, Point2f::new(637.521301, 443.275543), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(206, Point2f::new(575.40625, 393.644073), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(210, Point2f::new(494.973297, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(217, Point2f::new(225.974564, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(311, Point2f::new(365.474213, 36.629055), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(314, Point2f::new(584.532532, 15.1028929), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(315, Point2f::new(313.457581, 39.7117081), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(316, Point2f::new(162.325256, 46.006794), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(317, Point2f::new(617.658752, 175.554352), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(318, Point2f::new(195.964401, 53.4646339), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(319, Point2f::new(38.1137543, 0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(320, Point2f::new(37.6316986, 47.4455185), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(321, Point2f::new(175.031784, 19.6891766), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(322, Point2f::new(236.933533, 55.4674492), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(323, Point2f::new(128.493729, 9.92650127), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(324, Point2f::new(79.5269699, 32.7663574), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(325, Point2f::new(133.051193, 55.6552086), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(326, Point2f::new(8.77833652, 31.2644882), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(327, Point2f::new(751.0, 284.706635), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(328, Point2f::new(358.43396, 18.0043755), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(329, Point2f::new(151.942154, 0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(330, Point2f::new(522.655273, 60.148243), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(331, Point2f::new(467.59729, 88.4459991), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(332, Point2f::new(641.603455, 57.3277588), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(333, Point2f::new(0.0,28.6983662), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(334, Point2f::new(361.259796, 129.533066), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(335, Point2f::new(576.507629, 161.328613), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(336, Point2f::new(9.73493958, 0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(337, Point2f::new(306.143524, 214.400742), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(338, Point2f::new(118.49791, 34.7847519), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(339, Point2f::new(519.459595, 15.5671482), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(340, Point2f::new(426.164948, 75.8866272), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(342, Point2f::new(389.453918, 93.6352615), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(343, Point2f::new(445.703613, 156.941544), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(345, Point2f::new(193.947098, 14.8391094), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(346, Point2f::new(420.454285, 162.84285), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(348, Point2f::new(233.342712, 20.2303886), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(349, Point2f::new(562.300171, 0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(350, Point2f::new(198.599609, 160.307205), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(352, Point2f::new(300.75061, 101.040565), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(353, Point2f::new(252.041397, 281.928802), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(354, Point2f::new(600.586182, 47.9813385), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(355, Point2f::new(610.459595, 14.6518173), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(357, Point2f::new(269.389374, 154.279404), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(358, Point2f::new(606.681091, 146.901321), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(359, Point2f::new(597.556335, 196.470581), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(360, Point2f::new(389.082153, 123.614113), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(361, Point2f::new(728.009033, 78.7693024), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(362, Point2f::new(329.162872, 129.020477), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(363, Point2f::new(248.56189, 218.82045), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(364, Point2f::new(397.490784, 75.0930328), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(365, Point2f::new(450.671417, 120.689064), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(366, Point2f::new(347.405762, 98.2785187), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(367, Point2f::new(394.83313, 166.799637), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(369, Point2f::new(235.144974, 157.707016), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(370, Point2f::new(285.548981, 184.957199), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(371, Point2f::new(527.88855, 126.913139), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(372, Point2f::new(423.162445, 119.399208), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(375, Point2f::new(580.285278, 122.3255), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(376, Point2f::new(463.236206, 124.552544), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(377, Point2f::new(108.21138, 273.644928), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(378, Point2f::new(269.080048, 301.08197), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(379, Point2f::new(392.703705, 197.803787), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(384, Point2f::new(719.790527, 0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(387, Point2f::new(165.347916, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(389, Point2f::new(669.394165, 91.6940002), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(390, Point2f::new(295.673676, 165.179153), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(392, Point2f::new(66.1867981, 348.479614), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(393, Point2f::new(94.1677551, 344.513123), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(395, Point2f::new(451.196045, 39.868515), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(396, Point2f::new(453.225372, 82.4209366), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(398, Point2f::new(667.120605, 157.190536), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(401, Point2f::new(151.610046, 216.426636), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(406, Point2f::new(642.595093, 0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(407, Point2f::new(76.8512726, 382.142761), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(408, Point2f::new(0.0,0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(410, Point2f::new(103.314606, 376.833252), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(411, Point2f::new(49.0935555, 332.812866), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(412, Point2f::new(483.263947, 0.0820324048), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(414, Point2f::new(157.973709, 174.227127), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(415, Point2f::new(0.0,314.961212), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(416, Point2f::new(61.8188286, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(417, Point2f::new(631.672852, 368.466034), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(418, Point2f::new(332.776733, 111.777039), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(419, Point2f::new(182.528351, 466.413055), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(420, Point2f::new(743.487305, 354.343994), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(421, Point2f::new(467.652527, 466.279785), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(422, Point2f::new(295.650604, 244.594208), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(424, Point2f::new(751.0, 368.025116), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(426, Point2f::new(291.86438, 360.472626), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(427, Point2f::new(104.808701, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(428, Point2f::new(149.229095, 135.8806), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(430, Point2f::new(0.0,391.619019), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(432, Point2f::new(36.1223297, 400.563843), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(433, Point2f::new(615.24707, 324.015167), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(434, Point2f::new(671.73822, 232.634842), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(436, Point2f::new(705.577576, 340.77655), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(438, Point2f::new(289.826843, 73.2332916), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(439, Point2f::new(751.0, 115.010323), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(440, Point2f::new(135.865219, 345.602722), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(441, Point2f::new(740.285522, 176.853546), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(443, Point2f::new(227.078568, 433.168274), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(444, Point2f::new(71.5432053, 428.812927), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(445, Point2f::new(181.059204, 359.089539), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(449, Point2f::new(309.127197, 366.635437), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(450, Point2f::new(674.747375, 57.2461548), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(453, Point2f::new(0.0,61.1830063), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(455, Point2f::new(708.921875, 122.539055), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(459, Point2f::new(253.865112, 399.483551), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(460, Point2f::new(128.737778, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(461, Point2f::new(0.0,479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(462, Point2f::new(0.0,435.687286), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(464, Point2f::new(274.245972, 387.145081), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(467, Point2f::new(103.275291, 460.63974), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(471, Point2f::new(98.6060562, 404.428741), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(472, Point2f::new(751.0, 426.643707), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(473, Point2f::new(63.9890099, 456.949646), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(474, Point2f::new(751.0, 85.7971268), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(478, Point2f::new(315.951904, 417.431), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(479, Point2f::new(515.614624, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(482, Point2f::new(619.940674, 408.257202), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(484, Point2f::new(751.0, 148.217422), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(485, Point2f::new(116.870628, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(487, Point2f::new(700.884277, 443.633636), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(488, Point2f::new(445.495636, 475.54599), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(492, Point2f::new(734.004517, 440.898346), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(497, Point2f::new(73.1892395, 277.835236), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(507, Point2f::new(416.39032, 392.281311), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(512, Point2f::new(509.839142, 415.677185), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(522, Point2f::new(0.0,87.1505051), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(524, Point2f::new(0.0,181.337891), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(527, Point2f::new(90.9277039, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(528, Point2f::new(426.30838, 334.420685), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(529, Point2f::new(649.958557, 414.581055), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(530, Point2f::new(4.61131144, 86.0685654), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(531, Point2f::new(591.910278, 445.36618), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(550, Point2f::new(416.795563, 432.533569), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(556, Point2f::new(751.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(557, Point2f::new(698.972168, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(558, Point2f::new(751.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(559, Point2f::new(751.0, 260.734924), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(560, Point2f::new(37.5504303, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(561, Point2f::new(751.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(562, Point2f::new(369.832184, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(563, Point2f::new(243.009857, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(564, Point2f::new(329.955139, 195.523758), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(565, Point2f::new(461.714874, 179.74852), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(566, Point2f::new(140.057968, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(567, Point2f::new(575.159119, 214.939468), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(568, Point2f::new(214.010742, 460.993988), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(569, Point2f::new(600.314819, 357.574615), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(570, Point2f::new(0.0,228.860977), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(571, Point2f::new(701.473633, 300.39917), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(572, Point2f::new(369.020508, 155.929459), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(573, Point2f::new(573.957092, 291.300873), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(574, Point2f::new(496.608093, 202.165512), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(575, Point2f::new(307.318268, 318.797546), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(576, Point2f::new(85.3046875, 261.486176), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(577, Point2f::new(318.455261, 240.273483), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(578, Point2f::new(645.420227, 327.666321), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(579, Point2f::new(751.0, 47.0139427), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(580, Point2f::new(32.0344849, 312.560913), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(581, Point2f::new(747.507629, 411.988251), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(582, Point2f::new(0.0,0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(583, Point2f::new(368.011505, 354.5867), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(584, Point2f::new(502.912994, 136.24501), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(585, Point2f::new(217.667557, 68.5554581), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(586, Point2f::new(556.551514, 119.428337), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(587, Point2f::new(430.470428, 192.854004), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(588, Point2f::new(404.759583, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(589, Point2f::new(303.335541, 76.0884018), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(590, Point2f::new(395.621521, 219.680435), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(591, Point2f::new(358.094147, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(592, Point2f::new(347.804199, 77.8198853), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(593, Point2f::new(247.249023, 303.250732), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(594, Point2f::new(541.523193, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(595, Point2f::new(0.0,336.080597), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(596, Point2f::new(541.214905, 334.934692), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(597, Point2f::new(0.0,479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(598, Point2f::new(0.0,428.674225), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(599, Point2f::new(751.0, 312.67514), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(600, Point2f::new(751.0, 213.884964), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(601, Point2f::new(328.423279, 154.997498), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(602, Point2f::new(751.0, 384.625458), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(603, Point2f::new(260.289917, 181.21257), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(604, Point2f::new(275.563263, 411.169983), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(605, Point2f::new(552.280212, 227.030228), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(606, Point2f::new(175.619659, 211.162354), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(607, Point2f::new(292.015747, 32.6878128), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(608, Point2f::new(123.570099, 196.190521), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(609, Point2f::new(41.6128197, 244.471237), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(610, Point2f::new(161.125397, 69.7316513), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(611, Point2f::new(278.859619, 205.441986), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(612, Point2f::new(723.184937, 319.974182), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(613, Point2f::new(546.193726, 291.119843), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(614, Point2f::new(150.09288, 317.678192), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(615, Point2f::new(432.393585, 139.667908), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(616, Point2f::new(483.935272, 155.22226), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(617, Point2f::new(199.665726, 216.794037), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(618, Point2f::new(2.8318131, 370.818176), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(619, Point2f::new(645.171143, 261.719604), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(620, Point2f::new(0.0,462.096985), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(621, Point2f::new(344.504913, 374.744385), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(622, Point2f::new(89.0250854, 176.677551), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(623, Point2f::new(617.469421, 252.277267), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(624, Point2f::new(248.606735, 368.433868), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(625, Point2f::new(266.632843, 111.879433), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(626, Point2f::new(157.468399, 372.138245), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(627, Point2f::new(751.0, 40.5510674), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(628, Point2f::new(499.939209, 227.810867), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(629, Point2f::new(10.534194, 254.88588), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(630, Point2f::new(751.0, 464.451965), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(631, Point2f::new(476.398438, 41.1488533), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(632, Point2f::new(628.001343, 156.366119), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(633, Point2f::new(444.638855, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(634, Point2f::new(106.254944, 313.015686), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(635, Point2f::new(318.16925, 337.608246), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(636, Point2f::new(79.3843155, 325.125458), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(637, Point2f::new(751.0, 234.607285), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(638, Point2f::new(524.266724, 84.2468262), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(639, Point2f::new(572.27002, 320.2836), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(640, Point2f::new(488.674408, 297.416046), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(641, Point2f::new(480.079102, 214.367538), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(642, Point2f::new(0.0,479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(643, Point2f::new(496.618042, 274.050323), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(644, Point2f::new(399.699554, 463.796417), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(645, Point2f::new(349.156433, 341.509827), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(646, Point2f::new(107.281555, 68.7305374), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(647, Point2f::new(62.429493, 299.791077), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(648, Point2f::new(751.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(649, Point2f::new(27.5696201, 14.6622639), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(650, Point2f::new(726.403748, 239.676559), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(651, Point2f::new(509.915192, 300.243988), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(652, Point2f::new(369.012085, 177.514145), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(653, Point2f::new(609.265259, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(654, Point2f::new(184.223114, 187.043121), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(655, Point2f::new(149.083298, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(656, Point2f::new(292.251373, 398.490082), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(657, Point2f::new(647.511963, 10.9253464), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(658, Point2f::new(249.236008, 443.837708), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(659, Point2f::new(407.788147, 134.785736), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(660, Point2f::new(296.135254, 136.543991), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(661, Point2f::new(0.0,245.450607), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(662, Point2f::new(118.372803, 164.453964), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(663, Point2f::new(187.727188, 84.8875885), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(664, Point2f::new(245.171661, 107.66465), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(665, Point2f::new(178.416489, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(666, Point2f::new(332.410889, 355.811554), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(667, Point2f::new(287.255005, 224.777298), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(668, Point2f::new(562.347412, 439.054626), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(669, Point2f::new(430.242493, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(670, Point2f::new(751.0, 86.3693771), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(671, Point2f::new(359.270111, 403.716278), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(672, Point2f::new(105.449265, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(673, Point2f::new(657.33905, 123.688835), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(674, Point2f::new(324.916534, 439.385651), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(675, Point2f::new(595.193848, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(676, Point2f::new(572.065735, 418.168365), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(677, Point2f::new(724.197083, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(678, Point2f::new(67.4126205, 137.8806), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(679, Point2f::new(620.910339, 118.742897), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(680, Point2f::new(751.0, 161.484833), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(681, Point2f::new(715.852112, 162.404999), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(682, Point2f::new(226.19577, 223.315475), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(683, Point2f::new(59.6789742, 79.106781), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(684, Point2f::new(305.667358, 264.08316), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(685, Point2f::new(0.0,3.65125084), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(686, Point2f::new(637.924133, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(687, Point2f::new(26.3063622, 162.52655), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(688, Point2f::new(56.570385, 171.721954), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(689, Point2f::new(566.845764, 46.3044014), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(690, Point2f::new(481.566895, 414.73996), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(691, Point2f::new(487.979218, 442.829193), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(692, Point2f::new(528.293518, 436.320892), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(693, Point2f::new(470.414642, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 

        feature_tracks
    }

    fn get_feature_tracks3() -> TrackedFeatures {
        let mut feature_tracks = TrackedFeatures::default();
        feature_tracks.add_with_id(37, Point2f::new(284.757996, 364.69632), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(81, Point2f::new(149.661545, 460.219604), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(89, Point2f::new(635.820862, 436.748322), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(97, Point2f::new(120.157616, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(311, Point2f::new(349.017944, 74.2340927), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(314, Point2f::new(559.847534, 44.5120354), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(315, Point2f::new(297.810028, 73.9034958), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(316, Point2f::new(148.335541, 85.28022), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(317, Point2f::new(598.309753, 198.308746), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(318, Point2f::new(181.855194, 91.271553), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(319, Point2f::new(24.1928101, 22.5145836), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(320, Point2f::new(22.8436699, 90.2855225), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(321, Point2f::new(160.995117, 58.3007126), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(322, Point2f::new(222.482391, 91.7931061), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(323, Point2f::new(114.403786, 50.7081261), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(324, Point2f::new(64.9852829, 74.0040665), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(325, Point2f::new(118.916794, 95.5782623), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(326, Point2f::new(0.0, 75.0242996), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(327, Point2f::new(732.027039, 329.438873), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(328, Point2f::new(341.693512, 56.944046), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(329, Point2f::new(138.077316, 36.4941826), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(330, Point2f::new(502.629761, 87.4776001), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(331, Point2f::new(449.349213, 122.670906), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(332, Point2f::new(616.95575, 84.2268982), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(333, Point2f::new(0.0, 73.7334671), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(334, Point2f::new(346.509338, 166.171463), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(335, Point2f::new(558.229492, 183.420944), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(336, Point2f::new(0.0, 24.9191036), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(337, Point2f::new(292.673035, 253.573807), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(338, Point2f::new(104.451736, 75.9155273), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(339, Point2f::new(498.082062, 44.9928398), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(340, Point2f::new(408.781525, 111.995522), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(342, Point2f::new(373.43454, 131.662201), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(343, Point2f::new(430.229126, 189.135712), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(345, Point2f::new(179.662003, 53.1177979), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(346, Point2f::new(405.152466, 195.537796), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(348, Point2f::new(218.543961, 57.1319389), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(349, Point2f::new(538.325195, 28.6323566), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(350, Point2f::new(185.538773, 197.795364), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(352, Point2f::new(285.62207, 143.896759), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(354, Point2f::new(576.687256, 75.291748), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(355, Point2f::new(585.193359, 44.1467896), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(357, Point2f::new(255.109421, 196.86499), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(358, Point2f::new(587.356567, 168.11348), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(359, Point2f::new(579.547424, 220.015106), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(360, Point2f::new(373.482788, 159.108475), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(361, Point2f::new(698.910583, 102.639824), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(362, Point2f::new(314.861633, 162.054565), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(363, Point2f::new(234.870773, 258.869537), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(364, Point2f::new(380.597565, 111.980949), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(365, Point2f::new(433.999634, 154.180908), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(366, Point2f::new(332.263702, 137.827469), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(367, Point2f::new(380.009644, 200.472214), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(369, Point2f::new(221.636093, 197.041641), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(370, Point2f::new(271.713562, 226.055252), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(371, Point2f::new(509.332672, 156.761337), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(372, Point2f::new(406.894287, 153.43924), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(375, Point2f::new(560.284302, 150.981644), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(376, Point2f::new(446.445282, 155.629074), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(378, Point2f::new(256.378845, 370.906342), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(384, Point2f::new(686.186584, 12.5645494), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(389, Point2f::new(644.237915, 115.342789), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(390, Point2f::new(281.459534, 206.511185), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(392, Point2f::new(53.8959312, 461.657043), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(393, Point2f::new(82.9230652, 456.043945), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(395, Point2f::new(432.753235, 69.4588852), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(396, Point2f::new(435.23822, 117.487808), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(398, Point2f::new(645.550476, 178.245621), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(401, Point2f::new(137.420212, 257.612946), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(406, Point2f::new(613.325012, 6.93614435), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(407, Point2f::new(65.0821991, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(408, Point2f::new(0.0, 42.9775581), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(410, Point2f::new(92.0103989, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(411, Point2f::new(37.3411407, 445.928802), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(412, Point2f::new(462.838226, 30.9549313), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(414, Point2f::new(144.263794, 213.474487), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(415, Point2f::new(0.0, 423.414337), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(417, Point2f::new(605.7677, 454.262512), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(418, Point2f::new(317.981201, 150.764984), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(422, Point2f::new(282.634735, 287.051605), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(424, Point2f::new(746.144348, 447.426025), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(426, Point2f::new(278.837158, 445.004761), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(428, Point2f::new(135.040604, 174.60553), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(430, Point2f::new(0.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(432, Point2f::new(25.528223, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(433, Point2f::new(590.873657, 400.621185), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(438, Point2f::new(274.940033, 110.628525), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(439, Point2f::new(733.358154, 137.355789), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(440, Point2f::new(124.321793, 456.168762), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(441, Point2f::new(715.795044, 196.17514), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(445, Point2f::new(169.845627, 468.226715), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(449, Point2f::new(296.100494, 451.136261), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(450, Point2f::new(647.602905, 83.0784149), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(453, Point2f::new(0.0, 105.301117), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(455, Point2f::new(682.650024, 144.522934), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(472, Point2f::new(751.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(474, Point2f::new(751.0, 109.371307), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(482, Point2f::new(600.207336, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(484, Point2f::new(730.531372, 168.573639), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(522, Point2f::new(0.0, 131.331177), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(524, Point2f::new(0.0, 220.81633), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(528, Point2f::new(408.680511, 424.380188), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(530, Point2f::new(0.0, 128.655228), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(557, Point2f::new(665.770935, 0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(560, Point2f::new(24.1195717, 0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(562, Point2f::new(352.861511, 19.3831806), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(563, Point2f::new(228.193573, 32.0309563), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(565, Point2f::new(446.282715, 211.087494), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(566, Point2f::new(126.34288, 7.78187561), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(569, Point2f::new(576.623108, 442.223785), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(570, Point2f::new(0.0, 269.391998), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(571, Point2f::new(680.607605, 337.702118), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(572, Point2f::new(354.346222, 190.017242), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(574, Point2f::new(479.141541, 246.945953), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(575, Point2f::new(293.633881, 392.391632), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(578, Point2f::new(618.811462, 406.192078), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(580, Point2f::new(20.2675514, 424.062134), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(581, Point2f::new(713.101013, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(582, Point2f::new(0.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(584, Point2f::new(485.387451, 167.090683), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(585, Point2f::new(203.315521, 105.119499), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(586, Point2f::new(537.083191, 148.669296), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(588, Point2f::new(386.477722, 23.8869133), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(589, Point2f::new(288.399109, 118.367279), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(590, Point2f::new(381.427704, 256.07663), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(592, Point2f::new(331.864105, 118.339096), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(593, Point2f::new(233.658096, 374.025452), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(594, Point2f::new(518.11792, 16.6637115), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(596, Point2f::new(520.455933, 417.58371), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(599, Point2f::new(751.0, 357.064362), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(600, Point2f::new(751.0, 230.865555), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(601, Point2f::new(313.730774, 192.417725), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(603, Point2f::new(246.365738, 222.112381), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(607, Point2f::new(277.050201, 67.8770523), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(608, Point2f::new(108.482941, 236.54364), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(610, Point2f::new(147.022354, 107.721123), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(611, Point2f::new(265.57724, 244.877701), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(613, Point2f::new(524.951111, 365.861328), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(615, Point2f::new(416.0354, 172.88559), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(616, Point2f::new(467.384338, 186.576584), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(617, Point2f::new(186.075211, 257.916443), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(618, Point2f::new(0.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(626, Point2f::new(145.795807, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(629, Point2f::new(0.0, 297.364166), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(630, Point2f::new(751.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(632, Point2f::new(607.956299, 178.059052), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(633, Point2f::new(425.444244, 28.7794991), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(634, Point2f::new(94.7196045, 424.663208), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(636, Point2f::new(68.6740036, 436.199188), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(638, Point2f::new(504.471375, 112.095276), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(639, Point2f::new(549.641968, 399.358887), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(640, Point2f::new(469.367035, 375.664246), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(645, Point2f::new(335.859772, 409.092255), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(646, Point2f::new(92.9411392, 108.690941), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(647, Point2f::new(51.8298378, 408.985504), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(649, Point2f::new(13.3227406, 57.8330727), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(651, Point2f::new(489.960693, 379.050934), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(653, Point2f::new(582.038025, 15.0033245), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(654, Point2f::new(169.733139, 225.984024), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(657, Point2f::new(619.558594, 40.1219368), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(659, Point2f::new(392.295288, 169.180359), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(660, Point2f::new(281.642883, 172.577667), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(661, Point2f::new(0.0, 288.452911), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(662, Point2f::new(102.701599, 204.028275), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(663, Point2f::new(173.532211, 121.751694), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(664, Point2f::new(230.729309, 153.478195), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(666, Point2f::new(319.540253, 430.380585), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(667, Point2f::new(274.035522, 265.287079), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(672, Point2f::new(91.5311127, 25.0688858), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(673, Point2f::new(634.383728, 146.270981), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(678, Point2f::new(51.77491, 175.103607), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(679, Point2f::new(599.013916, 141.429993), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(681, Point2f::new(692.232117, 183.354538), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(683, Point2f::new(45.1234894, 120.234718), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(685, Point2f::new(0.0, 49.2685928), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(687, Point2f::new(8.2922287, 201.038879), BHVector3::new_with(0.0, 0.0, 0.0));
        feature_tracks.add_with_id(694, Point2f::new(749.89856, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(695, Point2f::new(751.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(696, Point2f::new(751.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(697, Point2f::new(751.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(698, Point2f::new(654.03241, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(699, Point2f::new(591.055481, 277.982086), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(700, Point2f::new(55.9404144, 319.904297), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(701, Point2f::new(751.0, 306.930054), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(702, Point2f::new(233.933365, 6.15858269), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(703, Point2f::new(108.351242, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(704, Point2f::new(341.265564, 241.840347), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(705, Point2f::new(314.431488, 228.581635), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(706, Point2f::new(524.735107, 282.449158), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(707, Point2f::new(95.7971497, 351.772583), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(708, Point2f::new(626.32843, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(709, Point2f::new(537.415283, 257.96994), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(710, Point2f::new(372.915375, 228.698898), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(711, Point2f::new(558.191101, 238.129807), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(712, Point2f::new(412.655823, 227.562561), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(713, Point2f::new(302.666595, 279.300354), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(714, Point2f::new(492.419067, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(715, Point2f::new(547.006592, 372.185181), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(716, Point2f::new(0.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(717, Point2f::new(242.3172, 351.786774), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(718, Point2f::new(323.262695, 465.491669), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(719, Point2f::new(210.681808, 456.145203), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(720, Point2f::new(57.7856369, 97.3482742), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(721, Point2f::new(479.160278, 271.593933), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(722, Point2f::new(751.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(723, Point2f::new(751.0, 69.4403381), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(724, Point2f::new(710.643066, 434.376892), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(725, Point2f::new(35.1098289, 333.4487), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(726, Point2f::new(12.3161488, 296.023956), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(727, Point2f::new(104.367462, 314.49707), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(728, Point2f::new(198.500381, 374.538208), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(729, Point2f::new(134.591995, 436.88360), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(730, Point2f::new(312.082214, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(731, Point2f::new(675.735229, 421.164673), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(732, Point2f::new(1.28796053, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(733, Point2f::new(366.988129, 437.746155), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(734, Point2f::new(483.172913, 428.361694), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(735, Point2f::new(266.187744, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(736, Point2f::new(751.0, 65.8072433), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(737, Point2f::new(351.155121, 316.912415), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(738, Point2f::new(384.032715, 230.512772), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(739, Point2f::new(636.032593, 362.458191), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(740, Point2f::new(640.629883, 323.260681), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(741, Point2f::new(315.814575, 255.022842), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(742, Point2f::new(132.299179, 361.453217), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(743, Point2f::new(81.7710648, 403.042145), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(744, Point2f::new(350.040771, 456.590485), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(745, Point2f::new(457.258118, 69.9766541), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(746, Point2f::new(71.0068283, 63.089901), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(747, Point2f::new(704.296082, 357.536774), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(748, Point2f::new(560.431091, 275.29834), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(749, Point2f::new(194.589661, 429.375763), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(750, Point2f::new(0.0, 467.641968), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(751, Point2f::new(461.904633, 97.774498), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(752, Point2f::new(0.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(753, Point2f::new(127.421196, 402.077271), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(754, Point2f::new(136.362946, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(755, Point2f::new(587.641296, 313.290222), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(756, Point2f::new(659.215149, 221.145935), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(757, Point2f::new(751.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(758, Point2f::new(316.189148, 10.1973886), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(759, Point2f::new(435.75238, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(760, Point2f::new(0.641676068, 363.675385), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(761, Point2f::new(73.6438828, 217.469971), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(762, Point2f::new(733.778076, 255.425598), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(763, Point2f::new(0.0, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(764, Point2f::new(593.347473, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(765, Point2f::new(591.601562, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(766, Point2f::new(57.3264732, 380.260681), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(767, Point2f::new(704.335876, 259.402588), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(768, Point2f::new(251.311142, 464.145813), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(769, Point2f::new(261.374786, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(770, Point2f::new(374.283905, 72.91008), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(771, Point2f::new(675.81488, 387.33197), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(772, Point2f::new(566.013245, 470.150269), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(773, Point2f::new(324.22995, 308.852234), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(774, Point2f::new(646.8349, 406.036346), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(775, Point2f::new(437.640533, 254.812256), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(776, Point2f::new(467.591431, 344.684937), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(777, Point2f::new(345.051605, 428.691498), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(778, Point2f::new(709.717285, 469.055542), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(779, Point2f::new(707.435669, 226.788361), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(780, Point2f::new(310.273895, 328.010925), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(781, Point2f::new(556.454102, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(782, Point2f::new(306.049744, 105.434273), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(783, Point2f::new(483.737061, 479.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(784, Point2f::new(751.0, 259.574127), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(785, Point2f::new(294.96048, 10.9669352), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(786, Point2f::new(355.710968, 111.563416), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(787, Point2f::new(710.791077, 386.762817), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(788, Point2f::new(82.7277832, 155.060257), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(789, Point2f::new(293.603821, 342.783875), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(790, Point2f::new(192.9646, 123.862091), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(791, Point2f::new(12.7377777, 261.119812), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(792, Point2f::new(498.13382, 134.521667), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(793, Point2f::new(215.351547, 352.545288), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(794, Point2f::new(103.943184, 175.291245), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(795, Point2f::new(409.307098, 85.0694046), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(796, Point2f::new(675.731323, 72.632843), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(797, Point2f::new(711.631042, 405.934113), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(798, Point2f::new(648.964478, 10.6939592), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(799, Point2f::new(230.108765, 128.85141), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(800, Point2f::new(751.0, 108.932091), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(801, Point2f::new(373.057037, 181.454391), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(802, Point2f::new(341.254791, 295.659943), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(803, Point2f::new(58.1145592, 198.12558), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(804, Point2f::new(127.115425, 119.195801), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(805, Point2f::new(171.183823, 4.58899117), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(806, Point2f::new(327.367767, 75.9282303), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(807, Point2f::new(380.6073, 291.072174), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(808, Point2f::new(671.830688, 119.527412), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(809, Point2f::new(207.629776, 266.602417), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(810, Point2f::new(193.065887, 229.224701), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(811, Point2f::new(471.064636, 71.8773499), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(812, Point2f::new(527.5495, 86.3128662), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(813, Point2f::new(410.223785, 52.867218), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(814, Point2f::new(235.753891, 286.944244), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(815, Point2f::new(529.197571, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(816, Point2f::new(738.012512, 87.7694321), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(817, Point2f::new(245.483093, 125.913734), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(818, Point2f::new(533.162537, 325.306976), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(819, Point2f::new(432.417969, 85.467804), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(820, Point2f::new(182.23613, 281.107025), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(821, Point2f::new(500.210968, 360.376617), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(822, Point2f::new(159.97374, 196.999939), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(823, Point2f::new(266.015076, 165.555008), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(824, Point2f::new(0.0, 87.6089935), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(825, Point2f::new(394.541565, 129.638123), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(826, Point2f::new(0.0, 135.086121), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(827, Point2f::new(582.816101, 106.223595), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(828, Point2f::new(0.0, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(829, Point2f::new(354.268768, 144.149704), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(830, Point2f::new(0.0, 172.966278), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(831, Point2f::new(0.0, 12.8887682), BHVector3::new_with(0.0, 0.0, 0.0)); 
        feature_tracks.add_with_id(832, Point2f::new(20.302515, 0.0), BHVector3::new_with(0.0, 0.0, 0.0)); 

        feature_tracks
    }
}