extern crate timely;
use timely::dataflow::{InputHandle, ProbeHandle};
// use timely::dataflow::channels::pact::Exchange;
use timely::dataflow::Scope;
use timely::dataflow::operators::*;

use std::hash::{Hash, Hasher};
use std::collections::hash_map::DefaultHasher;
fn my_hash<T>(obj: T) -> u64
where
    T: Hash,
{
    let mut hasher = DefaultHasher::new();
    obj.hash(&mut hasher);
    hasher.finish()
}

// #[allow(unused_imports)]
use opencv::{
    core,
    features2d,
    features2d::{Feature2DTrait, ORB},
    highgui,
    imgproc,
    prelude::*,
    videoio,
    imgcodecs,
    types::{PtrOfORB, VectorOfKeyPoint},
};

pub fn orb_extract(img_paths: Vec<String>) {

    // initializes and runs a timely dataflow.
    timely::execute_from_args(std::env::args().skip(2), move |worker| {

        let index = worker.index();

        // create input and output handles.
        let mut input = InputHandle::new();
        let mut probe = ProbeHandle::new();

        // build a new dataflow.
        worker.dataflow(|scope| {
            input.to_stream(scope)
                .exchange(|x: &String| my_hash(x))
                .inspect(move |x| {
                    // Read image
                    let img = imgcodecs::imread(x, imgcodecs::IMREAD_COLOR).unwrap();
                    let mut orb: PtrOfORB = ORB::default().unwrap();
                    let mut kp = VectorOfKeyPoint::new();
                    let mut des = Mat::default().unwrap();

                    orb.detect_and_compute(&img,&Mat::default().unwrap(), &mut kp, &mut des, false).unwrap();
                    println!("worker {}, processed {}, found {} keypoints",index, x, kp.len());
                })
                .probe_with(&mut probe);
        });

        for i in 0..img_paths.len() {
            if index == 0 {
                input.send(img_paths[i].to_owned());
            }
            input.advance_to(i + 1);
            while probe.less_than(input.time()) {
                worker.step();
            }
        }
    }).unwrap();
}