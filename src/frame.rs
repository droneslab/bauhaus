mod frame {

    mod pose;

    pub struct Frame {
        id: u64,
        timestamp: u64,
        keyPoints: opencv::types::VectorOfKeyPoint,
        descriptors: opencv::core::Mat,
        pose: Pose,
        depthThreshold: u64,
        camParams: opencv::core::Mat
    }
}

// impl Frame {
//     fn setPose(&self, pose: Pose) {
//         self.pose = pose;
//     }
// }
