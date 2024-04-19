#[cfg(test)]
mod loopclosing_tests {
    use core::{config::{self, load_config}, matrix::{DVMatrix, DVVectorOfKeyPoint}};
    use std::{env, fs};

    use opencv::core::{MatTraitConst, MatTraitConstManual};

    use crate::{map::{features::Features, frame::Frame}, modules::image};

    #[test]
    fn test_dbow_words_frame1577() {
        // Testing the extraction of BoW words from a frame
        let frame = setup_frame("src/tests/data/kitti00_frame1577.png", 2000);

        // Compare to orbslam
        let expected_words = fs::read_to_string("/home/sofiya/darvis-home/darvis/darvis/src/tests/data/kitti00_frame1577_words.txt").unwrap();
        let result_words = frame.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
        let mut result_words_format = "".to_owned();
        for word in result_words {
            result_words_format.push_str(&format!("{}, ", word));
        }

        assert_eq!(result_words_format, expected_words);
    }

    // #[test]
    fn test_descriptors_frame1577() {
        let frame = setup_frame("src/tests/data/kitti00_frame1577.png", 2000);

        // Compare to orbslam
        let expected_descs = fs::read_to_string("/home/sofiya/darvis-home/darvis/darvis/src/tests/data/kitti00_frame1577_descriptors.txt").unwrap();
        let real_descs = format_descriptors(&frame.features);

        // println!("{}", real_descs);
        assert_eq!(real_descs, expected_descs);
    }

    

    // #[test]
    fn test_dbow_words_frame0() {
        // Testing the extraction of BoW words from a frame
        let frame = setup_frame("src/tests/data/kitti00_frame0.png", 10000);

        // Compare to orbslam
        let expected_words = fs::read_to_string("/home/sofiya/darvis-home/darvis/darvis/src/tests/data/kitti00_frame0_words.txt").unwrap();
        let result_words = frame.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
        let mut result_words_format = "".to_owned();
        for word in result_words {
            result_words_format.push_str(&format!("{}, ", word));
        }

        assert_eq!(result_words_format, expected_words);
    }

    fn setup_frame(image_path: &str, num_features: i32) -> Frame {
        // Need this to prevent a panic from tracy
        let _client = tracy_client::Client::start();
        tracy_client::set_thread_name!("main");

        let dataset_config = "config_datasets/KITTI00-02.yaml".to_owned();
        let system_config = "system_config.yaml".to_owned();
        let _ = load_config(&system_config, &dataset_config).expect("Could not load config");

        // Read image
        let image = image::read_image_file(&image_path.to_string());
        let image_cols = image.cols() as u32;
        let image_rows = image.rows() as u32;

        // Extract features
        let mut orb_extractor = dvos3binding::ffi::new_orb_extractor(
            num_features,
            1.2,
            8,
            20,
            7,
            // 0,
            // 1000
        );
        let image_dv: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::new(image)).into();
        let mut descriptors: dvos3binding::ffi::WrapBindCVMat = (&DVMatrix::default()).into();
        let mut keypoints: dvos3binding::ffi::WrapBindCVKeyPoints = DVVectorOfKeyPoint::empty().into();
        orb_extractor.pin_mut().extract(&image_dv, &mut keypoints, &mut descriptors);

        // Create frame
        let mut frame = Frame::new(
            0, 
            DVVectorOfKeyPoint::new(keypoints.kp_ptr.kp_ptr),
            DVMatrix::new(descriptors.mat_ptr.mat_ptr),
            image_cols,
            image_rows,
            0.0
        ).expect("Could not create frame!");

        // Compute bow
        frame.compute_bow();

        frame
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

}
