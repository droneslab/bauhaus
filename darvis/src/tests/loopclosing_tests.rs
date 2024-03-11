#[cfg(test)]
mod loopclosing_tests {
    use core::{config::{self, load_config}, matrix::{DVMatrix, DVVectorOfKeyPoint}};
    use std::{env, fs};

    use opencv::core::MatTraitConst;

    use crate::{map::frame::Frame, modules::image};

    #[test]
    fn test_dbow_words() {
        // Testing the extraction of BoW words from a frame

        // Need this to prevent a panic from tracy
        let _client = tracy_client::Client::start();
        tracy_client::set_thread_name!("main");

        let dataset_config = "config_datasets/KITTI00-02.yaml".to_owned();
        let system_config = "system_config.yaml".to_owned();
        let _ = load_config(&system_config, &dataset_config).expect("Could not load config");

        // Read image
        let image_path = "src/tests/data/kitti00_frame0.png";
        let image = image::read_image_file(&image_path.to_string());
        let image_cols = image.cols() as u32;
        let image_rows = image.rows() as u32;

        // Extract features
        let mut orb_extractor = dvos3binding::ffi::new_orb_extractor(
            10000, //2000,
            1.2,
            8,
            20,
            7,
            0,
            1000
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


        // Compare to orbslam
        let expected_words = fs::read_to_string("/home/sofiya/darvis-home/darvis/darvis/src/tests/data/kitti00_frame0_words.txt").unwrap();
        let result_words = frame.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
        let mut result_words_format = "".to_owned();
        for word in result_words {
            result_words_format.push_str(&format!("{}, ", word));
        }


        assert_eq!(result_words_format, expected_words);
    }


}
