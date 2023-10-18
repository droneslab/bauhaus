use dvcore::matrix::{DVMatrix, DVVectorOfKeyPoint};
use opencv::{imgcodecs, prelude::Mat, types::VectorOfKeyPoint};


pub fn read_image_file(path: &String) -> Mat {
    imgcodecs::imread(path, imgcodecs::IMREAD_GRAYSCALE).expect("Could not read image.")
}
