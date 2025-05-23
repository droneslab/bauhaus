use opencv::{imgcodecs, imgproc::resize, prelude::Mat, types::VectorOfKeyPoint, core::{Vector, KeyPoint, DMatch}};

pub fn read_image_file(path: &String) -> Mat {
    // let _span = tracy_client::span!("read image");
    let image = imgcodecs::imread(path, imgcodecs::IMREAD_UNCHANGED).expect("Could not read image.");
    image
}

pub fn resize_image(image: & Mat, width: i32, height: i32) -> Result<Mat, opencv::Error> {
    let mut resized_img = Mat::default();
    resize(image, &mut resized_img, opencv::core::Size::new(width, height), 0.0, 0.0, 1)?;
    Ok(resized_img)
}

pub fn _decode_image(image: &Mat) -> Mat {
    imgcodecs::imdecode(image, imgcodecs::IMREAD_GRAYSCALE).expect("Could not decode image.")
}

pub fn _write_image_file(path: &String, image: &Mat) {
    let params = Vector::new();
    let _ = imgcodecs::imwrite(path, image, &params).expect("Could not read image.");
}

pub fn write_features(image: &Mat, keypoints: &VectorOfKeyPoint) -> Result<Mat, opencv::Error> {
    let mut dst_img = Mat::default();
    opencv::features2d::draw_keypoints(
        image,
        keypoints,
        &mut dst_img,
        opencv::core::VecN([0., 255., 0., 255.]),
        opencv::features2d::DrawMatchesFlags::DEFAULT,
    )?;

    Ok(dst_img)

    // opencv::imgcodecs::imwrite(path, &dst_img, &opencv::core::Vector::default()).unwrap();
}

pub fn write_feature_matches(
    image1: &Mat, image2: &Mat, keypoints1: &Vector<KeyPoint>, keypoints2: &Vector<KeyPoint>, matches: &Vector<DMatch>
) -> Result<Mat, opencv::Error> {
    let mut dst_img = Mat::default();

    opencv::features2d::draw_matches(
        image1,
        keypoints1,
        image2,
        keypoints2,
        matches,
        &mut dst_img,
        opencv::core::VecN([0., 255., 0., 255.]),
        opencv::core::VecN([0., 255., 0., 255.]),
        &opencv::core::Vector::default(),
        opencv::features2d::DrawMatchesFlags::DEFAULT,
    )?;

    Ok(dst_img)
}