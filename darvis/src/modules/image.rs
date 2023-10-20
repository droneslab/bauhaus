use opencv::{imgcodecs, prelude::Mat, types::VectorOfKeyPoint, core::Vector};


pub fn read_image_file(path: &String) -> Mat {
    imgcodecs::imread(path, imgcodecs::IMREAD_GRAYSCALE).expect("Could not read image.")
}

pub fn decode_image(image: &Mat) -> Mat {
    imgcodecs::imdecode(image, imgcodecs::IMREAD_GRAYSCALE).expect("Could not decode image.")
}

pub fn write_image_file(path: &String, image: &Mat) {
    let params = Vector::new();
    let _ = imgcodecs::imwrite(path, image, &params).expect("Could not read image.");
}

pub fn write_features(image: &Mat, keypoints: &VectorOfKeyPoint, path: &String) {
    let mut dst_img = Mat::default();
    opencv::features2d::draw_keypoints(
        image,
        keypoints,
        &mut dst_img,
        opencv::core::VecN([0., 255., 0., 255.]),
        opencv::features2d::DrawMatchesFlags::DEFAULT,
    );

    opencv::imgcodecs::imwrite(path, &dst_img, &opencv::core::Vector::default()).unwrap();
}
