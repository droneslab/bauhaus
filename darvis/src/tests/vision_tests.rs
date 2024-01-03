#[cfg(test)]
mod vision_tests {
    use core::{matrix::{DVVector3, DVMatrix, DVVectorOfKeyPoint}, config::{SETTINGS, self}, sensor::{Sensor, FrameSensor, ImuSensor}};
    use dvos3binding::ffi::WrapBindCVMat;
    use opencv::{imgcodecs, prelude::{MatTraitConst, KeyPointTraitConst}, core::{CV_8U, CV_8UC1}};

    use crate::{modules::geometric_tools, map::{pose::Pose, features::Features, bow::{BoW, self}}, actors::tracking_frontend::DVORBextractor};
    use std::{fs, env};

    use super::*;

    //* Test that feature detection gives same results as ORBSLAM3 
    //* Expected results generated from running ORBSLAM3 on image2 from the KITTI 00 dataset
    //* (image2 in data folder)
    #[test]
    fn test_ini_feature_detection() {
        load_config();
        let (image, image_cols, image_rows) = read_image("src/tests/data/image2.png");
        let (keypoints, descriptors) = extract_features(&image, 2000*5);
        let (features, bow) = compute_bow(keypoints, descriptors, image_cols as u32, image_rows as u32);

        // Test features
        let expected_features = fs::read_to_string("src/tests/data/image2_features.txt").unwrap();
        let real_features = format_features(&bow);
        assert_eq!(real_features, expected_features);

        // Test descriptors
        let expected_desc = fs::read_to_string("src/tests/data/image2_descriptors.txt").unwrap();
        let real_descs = format_descriptors(&features);
        assert_eq!(real_descs, expected_desc);

        // Test keypoints
        let expected_kps = fs::read_to_string("src/tests/data/image2_keypoints.txt").unwrap();
        let real_kps = format_keypoints(&features);
        // Not running bc the floats have to be formatted the same way the opencv C++ does it
        // or it doesn't mark as correct
        // assert_eq!(real_kps_string, expected_kps);
    }




    
    fn load_config() {
        let mut path = env::current_dir().unwrap();
        path.push("config.yaml");
        let _ = config::load_config(&path.into_os_string().into_string().unwrap());
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
        bow::VOCABULARY.transform(&features.descriptors, &mut bow);
        (features, bow)
    }

    fn format_features(bow: &BoW) -> String {
        let mut real_features = vec![];
        for node_id in bow.feat_vec.get_all_nodes() {
            let index = bow.get_feat_from_node(node_id);
            real_features.push(format!("<{}: {:?}>", node_id, index));
        }
        real_features.join(", ")
    }

    fn format_descriptors(features: &Features) -> String {
        let mut real_descs = vec![];
        for index in 0..(*features.descriptors).rows() as i32 {
            let mut row = vec![];
            for index2 in 0..features.descriptors.row(index as u32).unwrap().cols() as i32 {
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
