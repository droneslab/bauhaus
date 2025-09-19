use opencv::{core::{DMatch, KeyPoint, MatExprTraitConst, MatTraitConst, Point2f, RNGTrait, Vector}, imgcodecs, imgproc::resize, prelude::Mat, types::VectorOfKeyPoint};

pub fn read_image_file(path: &String, color: i32) -> Mat {
    // let _span = tracy_client::span!("read image");
    let image = imgcodecs::imread(path, color).expect("Could not read image.");
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


pub fn draw_optical_flow(
    last_image: &Mat, curr_image: & Mat,
    last_points: &Vector<Point2f>, curr_points: &Vector<Point2f>,
    filename: &str,
) -> Result<(), opencv::Error> {
        let mut curr_image_color = {
            let mut dst = Mat::default();
            opencv::imgproc::cvt_color(curr_image, &mut dst,opencv::imgproc::COLOR_GRAY2BGR, 0).unwrap();
            dst
        };
        let last_image_color = {
            let mut dst = Mat::default();
            opencv::imgproc::cvt_color(last_image, &mut dst,opencv::imgproc::COLOR_GRAY2BGR, 0).unwrap();
            dst
        };
        let mut mask = Mat::zeros(last_image_color.rows(), last_image_color.cols(), last_image_color.typ())?.to_mat()?;

        // Create some random colors
        let mut colors: Vec<opencv::core::Scalar_<f64>> = vec![];
        let mut rng: opencv::core::RNG = opencv::core::RNG::new(0).unwrap();
        for _ in 0..last_points.len() {
            let r = rng.uniform(0, 256).unwrap();
            let g = rng.uniform(0, 256).unwrap();
            let b = rng.uniform(0, 256).unwrap();
            colors.push(opencv::core::Scalar_::<f64>::new(r as f64,g as f64,b as f64,0.0));
        }

        // Draw points
        for i in 0..last_points.len() {
            let prev_pt = last_points.get(i)?;
            let curr_pt = curr_points.get(i)?;
            opencv::imgproc::line(
                &mut mask,
                opencv::core::Point::new(curr_pt.x as i32, curr_pt.y as i32),
                opencv::core::Point::new(prev_pt.x as i32, prev_pt.y as i32),
                colors[i],
                2,
                opencv::imgproc::LineTypes::LINE_8 as i32,
                0
            ).unwrap();
            opencv::imgproc::circle(
                &mut curr_image_color,
                opencv::core::Point::new(curr_pt.x as i32, curr_pt.y as i32),
                5,
                colors[i],
                1,
                opencv::imgproc::LineTypes::LINE_8 as i32,
                0
            ).unwrap();
        }

        // Combine images
        let mut dst_img = Mat::default();
        opencv::core::add(& curr_image_color, & mask, &mut dst_img, & opencv::core::Mat::default(), -1).expect("Couldn't add image");
        opencv::imgcodecs::imwrite(filename, &dst_img, &opencv::core::Vector::default()).unwrap();

        Ok(())

}
