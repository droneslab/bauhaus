extern crate dvos3binding;
use nalgebra::{Isometry3};
use cxx::{let_cxx_string, CxxVector, SharedPtr};

use opencv::core::{CV_8UC2, CV_8UC1};
use opencv::platform_types::size_t;
use opencv::prelude::*;
use opencv::types::VectorOfKeyPoint;

use std::pin::Pin;

fn main() {
    let mut matcher = dvos3binding::ffi::new_orb_matcher(48, 48, 0.0, 0.0, 600.0, 600.0,0.1,true);
    
    let mut kps_cv1 = opencv::types::VectorOfKeyPoint::default();
    let mut kps_cv2 = opencv::types::VectorOfKeyPoint::default();

    let mut matches_cv = opencv::types::VectorOfi32::default();
    
    let mut grid_cv = opencv::types::VectorOfsize_t::default();
    //let mut grid_cv = vec![vec![vec![0 as usize;5];0];0];

    //let mut grid_cv = opencv::core::Vec3b::default();
    
    let mut prevMat_cv = opencv::types::VectorOfPoint2f::default();
    
    let mut grid_vals = Vec::new();
    for i in 1..11
    {
        kps_cv1.push(opencv::core::KeyPoint::new_coords(i as f32 *5.0, 1.0/i as f32 *3.0, 5.0, 1.4, 1.1, 3, 0).unwrap());

        kps_cv2.push(opencv::core::KeyPoint::new_coords(i as f32 *5.0, 1.0/i as f32 *3.0, 5.0, 1.4, 1.1, 3, 0).unwrap());

        matches_cv.push(i);

        grid_cv.push( i as usize);
        grid_vals.push(i as usize);

    }

    //let desc1 = opencv::core::Mat::eye(4,4,CV_8UC1).unwrap().to_mat().unwrap();
    //let desc2 = opencv::core::Mat::eye(4,4,CV_8UC1).unwrap().to_mat().unwrap();
    
    let desc1 =opencv::core::Mat::zeros(10, 32, CV_8UC1).unwrap().to_mat().unwrap();
    let desc2 =opencv::core::Mat::ones(10, 32, CV_8UC1).unwrap().to_mat().unwrap();
    
    unsafe{

        let  kps1cv = kps_cv1.into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;
        let  kps2cv = kps_cv2.into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;


        let  desc1cv = desc1.into_raw() as *const dvos3binding::ffi::DVMat;
        let  desc2cv = desc2.into_raw() as *const dvos3binding::ffi::DVMat;

        
        let mut grid_v1 = dvos3binding::ffi::VectorOfusize{vec: Vec::new()};

        grid_v1.vec.append(&mut grid_vals);
        let mut grid_v2 = dvos3binding::ffi::VectorOfVecusize{vec:Vec::new()  };
        
        grid_v2.vec.push(grid_v1);

        let mut grid_v3 = dvos3binding::ffi::DVGrid{vec:Vec::new()};

        grid_v3.vec.push(grid_v2.clone());
        grid_v3.vec.push(grid_v2.clone());


        let mut prev_match_cv= opencv::types::VectorOfPoint2f::default();
        let  prev_matchcv = prev_match_cv.into_raw() as *mut CxxVector<dvos3binding::ffi::DVPoint2f>;


        let mut matches_cv=  opencv::types::VectorOfi32::default();
        let  matchescv = matches_cv.into_raw() as *mut CxxVector<i32>;

        matcher.pin_mut().SearchForInitialization_1(
            &*kps1cv,
            &*kps2cv,
            &*desc1cv,
            &*desc2cv,
            &grid_v3,
             Pin::new_unchecked(prev_matchcv.as_mut().unwrap()), 
            Pin::new_unchecked(matchescv.as_mut().unwrap()),
            100
            );


        println!("\nDone!!");

    }


}
