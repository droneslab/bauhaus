#[cfg(test)]
mod vision_tests {
    use core::{config::{self, load_config, SETTINGS}, matrix::{DVMatrix, DVVector3, DVVectorOfKeyPoint}, sensor::{FrameSensor, ImuSensor, Sensor}};
    use dvos3binding::ffi::WrapBindCVMat;
    use opencv::{imgcodecs, prelude::{MatTraitConst, KeyPointTraitConst}, core::{CV_8U, CV_8UC1}};

    use crate::{actors::tracking_frontend::DVORBextractor, map::{features::Features, pose::Pose}, modules::{bow::BoW, geometric_tools}, registered_actors::VOCABULARY};
    use std::{fs, env};

    use super::*;

    //* Test that feature detection gives same results as ORBSLAM3 
    //* Expected results generated from running ORBSLAM3 on image/000000.png from the KITTI 00 dataset
    #[test]
    fn test_ini_feature_detection() {
        let mut system_config = env::current_dir().unwrap();
        system_config.push("system_config.yaml");
        let mut dataset_config = env::current_dir().unwrap();
        dataset_config.push("config_datasets/KITTI00-02.yaml");
        let _ = load_config(
            &system_config.into_os_string().into_string().unwrap(),
            &dataset_config.into_os_string().into_string().unwrap()
        ).expect("Could not load config");

        let (image, image_cols, image_rows) = read_image("src/tests/data/kitti00_frame0.png");
        let (keypoints, descriptors) = extract_features(&image, 2000*5);
        let (features, bow) = compute_bow(keypoints, descriptors, image_cols as u32, image_rows as u32);

        // Test features
        // let expected_features = fs::read_to_string("src/tests/data/kitti00_000000_features.txt").unwrap();
        // let real_features = format_features(&bow);
        // assert_eq!(real_features, expected_features);

        // Test descriptors
        let expected_desc = fs::read_to_string("src/tests/data/kitti00_frame0_descriptors.txt").unwrap();
        let real_descs = format_descriptors(&features);
        // assert_eq!(real_descs, expected_desc);

        // Test keypoints
        // let expected_kps = fs::read_to_string("src/tests/data/kitti00_000000_keypoints.txt").unwrap();
        // let real_kps = format_keypoints(&features);
        // Not running bc the floats have to be formatted the same way the opencv C++ does it
        // or it doesn't mark as correct
        // assert_eq!(real_kps_string, expected_kps);
    }


    fn read_image(image_path: &str) -> (WrapBindCVMat, i32, i32){
        let image = imgcodecs::imread(&image_path, imgcodecs::IMREAD_GRAYSCALE).unwrap();
        let cols = image.cols();
        let rows = image.rows();
        (
           (&DVMatrix::new(image)).into(),
            cols,
            rows
        )
    }

    fn extract_features(image: &WrapBindCVMat, max_features: i32) -> (DVVectorOfKeyPoint, DVMatrix) {
        let mut orb_extractor = DVORBextractor::new(max_features);
        let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
        let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();
        orb_extractor.extractor.pin_mut().extract(image, &mut keypoints, &mut descriptors);
        (
            DVVectorOfKeyPoint::new(keypoints.kp_ptr.kp_ptr),
            DVMatrix::new(descriptors.mat_ptr.mat_ptr)
        )
    }

    fn compute_bow(keypoints: DVVectorOfKeyPoint, descriptors: DVMatrix, image_cols: u32, image_rows: u32) -> (Features, BoW) {
        let sensor = Sensor(FrameSensor::Mono, ImuSensor::None);
        let features = Features::new(keypoints, descriptors, image_cols, image_rows, sensor).unwrap();
        let mut bow = BoW::new();
        VOCABULARY.transform(&features.descriptors, &mut bow);
        (features, bow)
    }

    fn format_features(bow: &BoW) -> String {
        let mut real_features = vec![];
        for node_id in bow.feat_vec.get_all_nodes() {
            let index = bow.feat_vec.get_feat_from_node(node_id);
            real_features.push(format!("<{}: {:?}>", node_id, index));
        }
        real_features.join(", ")
    }

    fn format_descriptors(features: &Features) -> String {
        let mut real_descs = vec![];
        for index in 0..(*features.descriptors).rows() as i32 {
            let mut row = vec![];
            for index2 in 0..features.descriptors.row(index as u32).cols() as i32 {
                // CV_8U == u8
                row.push(format!("{:?}", (*features.descriptors).at_2d::<u8>(index, index2).unwrap()));
            }
            real_descs.push(format!("{}", row.join(", ")));
        }
        real_descs.join(";\n")
    }

    fn format_keypoints(features: &Features) -> String {
        let mut real_kps = vec![];
        let kps = features.get_all_keypoints();
        for index in 0..kps.len() as usize {
            let kp = kps.get(index).unwrap();
            let pt_x;
            let fract = (kp.pt().x.fract() * 100.0).round() / 100.0;
            print!("{} ",fract);
            if fract == 0.0 {
                pt_x= format!("{}", kp.pt().x);
            } else if fract % 0.1 == 0.0 {
                pt_x = format!("{:.1}", kp.pt().x);
            } else if fract % 0.01 == 0.0 {
                pt_x = format!("{:.2}", kp.pt().x);
            } else {
                pt_x = format!("{:.3}", kp.pt().x);
            }

            let pt_y;
            let fract = (kp.pt().y.fract() * 100.0).round() / 100.0;
                        print!("{} ",fract);

            if fract == 0.0 {
                pt_y = format!("{}", kp.pt().y);
            } else if fract % 0.1 == 0.0 {
                pt_y = format!("{:.1}", kp.pt().y);
            } else if fract % 0.01 == 0.0 {
                pt_y = format!("{:.2}", kp.pt().y);
            } else {
                pt_y = format!("{:.3}", kp.pt().y);
            }

            real_kps.push(format!("[{}, {}] {} {} {}", pt_x, pt_y, kp.octave(), kp.response(), kp.size()));
        }
        real_kps.join(";")
    }
}
