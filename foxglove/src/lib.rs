// pub type TranslationVector = [f64; 3];
// pub type RotationVector = [f64; 4];

#[cxx::bridge(namespace = "foxglove")]
pub mod ffi {
    // Shared structs with fields visible to both languages.

    struct Pose {
        translation: [f64; 3], // in C++: array<double, 3>,
        rotation: [f64; 4] // in C++: array<double, 4> 
    }

    unsafe extern "C++" {
        // Note: can't use relative path because cargo hates it :(

        // include!("publish.h");
        // // Opaque types which both languages can pass around
        // // but only C++ can see the fields.
        // type BridgeSparseOptimizer;

        // fn new_sparse_optimizer(opt_type: i32, camera_param: [f64;4]) -> UniquePtr<BridgeSparseOptimizer>;


    }
}