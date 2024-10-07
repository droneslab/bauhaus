// #[cfg(test)]
// mod loopclosing_tests {
//     use core::{config::{self, load_config, SETTINGS}, matrix::{DVMatrix, DVVectorOfKeyPoint}};
//     use std::{env, fs::{self, File}, io::{self, BufRead}, path::Path};

//     use nalgebra::{UnitQuaternion, Vector3};
//     use opencv::core::{MatTraitConst, MatTraitConstManual};

//     use crate::{map::{features::Features, frame::Frame, map::Id, pose::{DVRotation, DVTranslation, Pose, Sim3}}, modules::image, registered_actors::CAMERA};

//         #[test]
//     fn test_sim3_map() {
//         let pos: DVTranslation = DVTranslation::new_with(1.4798, -0.211756, -3.47994);

//         let first = Sim3::new(
//             DVTranslation::new_with(-0.295157, -0.0171382, 5.18072),
//             DVRotation::new(nalgebra::Matrix3::new(
//                 0.999646, 0.00510355, 0.026096,
//                 -0.00473212, 0.999887, -0.0142753,
//                 -0.0261659, 0.0141468, 0.999558
//             )),
//             1.0
//         );

//         let second = Sim3::new(
//             DVTranslation::new_with(0.00185393, 0.0290222,   -1.84645),
//             DVRotation::new(nalgebra::Matrix3::new(
//                 0.999978, -0.00370639, -0.00557443,
//                 0.00376623, 0.999935, 0.0107627,
//                 0.00553418, -0.0107835, 0.999927
//             )),
//             0.484749
//         );

//         let res = second.map(&first.map(&pos));

//         let expected_res = DVTranslation::new_with(0.527145, -0.050572,  -1.03763);

//         println!("Result: {:?}", res);
//         println!("Expected: {:?}", expected_res);
//     }

//     // #[test]
//     fn test_essential_graph_optimization() {
//         let mut system_config = env::current_dir().unwrap();
//         system_config.push("system_config.yaml");
//         let mut dataset_config = env::current_dir().unwrap();
//         dataset_config.push("config_datasets/KITTI00-02.yaml");
//         let _ = load_config(
//             &system_config.into_os_string().into_string().unwrap(),
//             &dataset_config.into_os_string().into_string().unwrap()
//         ).expect("Could not load config");

//         let camera_param = [
//             SETTINGS.get::<f64>(CAMERA, "fx"),
//             SETTINGS.get::<f64>(CAMERA, "fy"),
//             SETTINGS.get::<f64>(CAMERA, "cx"),
//             SETTINGS.get::<f64>(CAMERA, "cy")
//         ];
//         let mut optimizer = g2o::ffi::new_sparse_optimizer(4, camera_param);

//         // The output is wrapped in a Result to allow matching on errors.
//         // Returns an Iterator to the Reader of the lines of the file.
//         fn read_lines<P>(filename: P) -> io::Result<io::Lines<io::BufReader<File>>>
//         where P: AsRef<Path>, {
//             let file = File::open(filename)?;
//             Ok(io::BufReader::new(file).lines())
//         }

//         if let Ok(lines) = read_lines("/home/sofiya/darvis-home/darvis-orbslam3/bla2.txt") {
//             for line in lines.flatten() {
//                 if line.contains("G2O_ESTIMATE VERTEX") {
//                     let split: Vec<&str> = line.split_whitespace().collect();

//                     let kf_id = split[2].parse::<Id>().unwrap();

//                     let estimate = {
//                         let rotation = [
//                             split[6].parse::<f64>().unwrap(),
//                             split[3].parse::<f64>().unwrap(),
//                             split[4].parse::<f64>().unwrap(),
//                             split[5].parse::<f64>().unwrap()
//                         ];
//                         let translation = [
//                             split[7].parse::<f64>().unwrap(),
//                             split[8].parse::<f64>().unwrap(),
//                             split[9].parse::<f64>().unwrap()
//                         ];
//                         let scale = split[10].parse::<f64>().unwrap();

//                         g2o::ffi::RustSim3 { 
//                             translation,
//                             rotation,
//                             scale
//                         }
//                     };

//                     optimizer.pin_mut().add_vertex_sim3expmap(
//                         kf_id,
//                         estimate.into(),
//                         false,
//                         kf_id == 0,
//                         false,
//                     );

//                 } else if line.contains("G2O_ESTIMATE EDGE") {
//                     let split: Vec<&str> = line.split_whitespace().collect();

//                     let kf_i_id = split[2].parse::<Id>().unwrap();
//                     let kf_j_id = split[3].parse::<Id>().unwrap();

//                     let estimate = {
//                         let rotation = [
//                             split[7].parse::<f64>().unwrap(),
//                             split[4].parse::<f64>().unwrap(),
//                             split[5].parse::<f64>().unwrap(),
//                             split[6].parse::<f64>().unwrap()
//                         ];
//                         let translation = [
//                             split[8].parse::<f64>().unwrap(),
//                             split[9].parse::<f64>().unwrap(),
//                             split[10].parse::<f64>().unwrap()
//                         ];
//                         let scale = split[11].parse::<f64>().unwrap();

//                         g2o::ffi::RustSim3 { 
//                             translation,
//                             rotation,
//                             scale
//                         }
//                     };

//                     optimizer.pin_mut().add_one_sim3_edge(
//                         kf_i_id,
//                         kf_j_id,
//                         estimate.into(),
//                     );

//                 }
//             }
//         }



//         optimizer.save("DARVIS_BEFORE_OPTIMIZATION.g2o\0", 0 as i32);

//         optimizer.pin_mut().optimize(20, false, true);

//         optimizer.save("DARVIS_AFTER_OPTIMIZATION.g2o\0",0 as i32);
//     }

//     // #[test]
//     fn test_dbow_words_frame1577() {
//         // Testing the extraction of BoW words from a frame
//         let frame = setup_frame("src/tests/data/kitti00_frame1577.png", 2000);

//         // Compare to orbslam
//         let expected_words = fs::read_to_string("/home/sofiya/darvis-home/darvis/darvis/src/tests/data/kitti00_frame1577_words.txt").unwrap();
//         let result_words = frame.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
//         let mut result_words_format = "".to_owned();
//         for word in result_words {
//             result_words_format.push_str(&format!("{}, ", word));
//         }

//         assert_eq!(result_words_format, expected_words);
//     }

//     // #[test]
//     fn test_descriptors_frame1577() {
//         let frame = setup_frame("src/tests/data/kitti00_frame1577.png", 2000);

//         // Compare to orbslam
//         let expected_descs = fs::read_to_string("/home/sofiya/darvis-home/darvis/darvis/src/tests/data/kitti00_frame1577_descriptors.txt").unwrap();
//         let real_descs = format_descriptors(&frame.features);

//         assert_eq!(real_descs, expected_descs);
//     }

//     // #[test]
//     fn test_dbow_words_frame0() {
//         // Testing the extraction of BoW words from a frame
//         let frame = setup_frame("src/tests/data/kitti00_frame0.png", 10000);

//         // Compare to orbslam
//         let expected_words = fs::read_to_string("/home/sofiya/darvis-home/darvis/darvis/src/tests/data/kitti00_frame0_words.txt").unwrap();
//         let result_words = frame.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
//         let mut result_words_format = "".to_owned();
//         for word in result_words {
//             result_words_format.push_str(&format!("{}, ", word));
//         }

//         assert_eq!(result_words_format, expected_words);
//     }

//     fn setup_frame(image_path: &str, num_features: i32) -> Frame {
//         // Need this to prevent a panic from tracy
//         let _client = tracy_client::Client::start();
//         tracy_client::set_thread_name!("main");

//         let dataset_config = "config_datasets/KITTI00-02.yaml".to_owned();
//         let system_config = "system_config.yaml".to_owned();
//         let _ = load_config(&system_config, &dataset_config).expect("Could not load config");

//         // Read image
//         let image = image::read_image_file(&image_path.to_string());
//         let image_cols = image.cols() as u32;
//         let image_rows = image.rows() as u32;

//         // Extract features
//         let mut orb_extractor = dvos3binding::ffi::new_orb_extractor(
//             num_features,
//             1.2,
//             8,
//             20,
//             7,
//             0,
//             1000
//         );
//         let image_dv: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::new(image)).into();
//         let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
//         let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();
//         orb_extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);

//         // Create frame
//         let mut frame = Frame::new(
//             0, 
//             DVVectorOfKeyPoint::new(keypoints.kp_ptr.kp_ptr),
//             DVMatrix::new(descriptors.mat_ptr.mat_ptr),
//             image_cols,
//             image_rows,
//             0.0
//         ).expect("Could not create frame!");

//         // Compute bow
//         frame.compute_bow();

//         frame
//     }

//     fn format_descriptors(features: &Features) -> String {
//         let mut real_descs = vec![];
//         for index in 0..(*features.descriptors).rows() as i32 {
//             let mut row = vec![];
//             for index2 in 0..features.descriptors.row(index as u32).cols() as i32 {
//                 // CV_8U == u8
//                 row.push(format!("{:?}", (*features.descriptors).at_2d::<u8>(index, index2).unwrap()));
//             }
//             real_descs.push(format!("{}", row.join(", ")));
//         }
//         real_descs.join(";\n")
//     }

// }
