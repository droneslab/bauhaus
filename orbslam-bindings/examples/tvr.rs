extern crate dvos3binding;
use nalgebra::{Isometry3};
use cxx::{let_cxx_string, CxxVector};

use opencv::prelude::*;
use opencv::types::VectorOfKeyPoint;

use std::pin::Pin;

fn main() {
    let mut tvr = dvos3binding::ffi::new_two_view_reconstruction(1.0, 1.0, 1.0, 1.0, 1.0, 200);
    
    let mut kps_cv1 = opencv::types::VectorOfKeyPoint::default();
    let mut kps_cv2 = opencv::types::VectorOfKeyPoint::default();

    let mut matches_cv = opencv::types::VectorOfi32::default();

    for i in 1..10
    {
        kps_cv1.push(opencv::core::KeyPoint::new_coords(i as f32 *5.0, 1.0/i as f32 *3.0, 5.0, 1.4, 1.1, 3, 0).unwrap());

        kps_cv2.push(opencv::core::KeyPoint::new_coords(i as f32 *5.0, 1.0/i as f32 *3.0, 5.0, 1.4, 1.1, 3, 0).unwrap());

        matches_cv.push(i);
    }


    // T21: &mut Pose,
    // vP3D: Pin<&mut CxxVector<DVPoint3f>>,
    // vbTriangulated: Pin<&mut CxxVector<DVbool>>


    unsafe{

        let matches_new = matches_cv.into_raw() as *const CxxVector<i32>;

        let mut pose = dvos3binding::ffi::Pose{pose : [[0.0;4];4]};

        let mut vbTriangulated =  Vec::new().as_mut_ptr() as *mut dvos3binding::ffi::VectorOfDVBool; 
        
        let  kps1cv = kps_cv1.into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;
        let  kps2cv = kps_cv2.into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>;

        
        let mut vP3D  = dvos3binding::ffi::VectorOfDVPoint3f{vec:Vec::new() };
        let mut vbTriangulated  = dvos3binding::ffi::VectorOfDVBool{ vec:Vec::new() };

        tvr.pin_mut().Reconstruct_1(&*kps1cv,
             &*kps2cv,
             &*matches_new, 
             &mut pose,
            &mut vP3D,
             &mut vbTriangulated
            );


        //print!("{:?}", &(*vP3D).vec.len());
        println!("DVPoint3f");
        for val in vP3D.vec
        {
            print!("{:?}", val)
        }
        println!("vbTriangulated");
        for val in vbTriangulated.vec
        {
            print!("{:?}", val)
        }
        
        print!("\nPose{:?}", pose.pose)
    }


}
