extern crate g2orust;

use dvcore::{
    lockwrap::ReadOnlyWrapper,
    global_params::{GLOBAL_PARAMS, SYSTEM_SETTINGS},
};
use crate::dvmap::{frame::Frame, pose::Pose, map::{Map, Id}, keypoints::*, sensor::SensorType};

#[derive(Debug, Clone)]
pub struct Optimizer {
    // Note: Does not change, so can have multiple copies of this.
    // ORBSLAM3 duplicates this var at every frame and keyframe,
    // but I'm pretty sure that it's set once per-system when the ORBExtractor
    // is created and only ever used by Optimizer.
    // In general, I'd like to remove these kinds of variables away from the
    // frame/keyframe/mappoint implementation and into the object that actually
    // directly uses it.
    pub inv_level_sigma2: Vec<f32>, // mvInvLevelSigma2
}

impl Optimizer {
    pub fn new() -> Optimizer {
        // See ORBExtractor constructor: https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/ORBextractor.cc#L409
        // and note above about inv_level_sigma2
        let scale_factor= GLOBAL_PARAMS.get::<f64>(SYSTEM_SETTINGS, "scale_factor");
        let max_features = GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "max_features");
        let n_levels = GLOBAL_PARAMS.get::<i32>(SYSTEM_SETTINGS, "n_levels");

        let mut scale_factors = vec![1.0];
        let mut level_sigma2 = vec![1.0];
        for i in 1..n_levels as usize {
            scale_factors.push(scale_factors[i-1] * (scale_factor as f32));
            level_sigma2.push(scale_factors[i] * scale_factors[i]);
        }

        let mut inv_level_sigma2 = Vec::new();
        for i in 1..n_levels as usize {
            inv_level_sigma2.push(1.0 / level_sigma2[i]);
        }

        Optimizer{
            inv_level_sigma2: inv_level_sigma2
        }
    }

    // int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
    // but bRecInit is always set to false
    pub fn pose_inertial_optimization_last_frame<S: SensorType>(&self, frame: &mut Frame<S>) -> i32 {
        todo!("Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame)");
    }

    //int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
    // but bRecInit is always set to false
    pub fn pose_inertial_optimization_last_keyframe<S: SensorType>(&self, frame: &mut Frame<S>) -> i32 {
        todo!("Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame)");
    }

    pub fn optimize_pose<S: SensorType + 'static>(&self, frame: &mut Frame<S>, map: &ReadOnlyWrapper<Map<S>>) -> (i32, Option<Pose>) {
        let optimizer = g2orust::ffi::new_sparse_optimizer();

        // Don't need this but saving here for reference
        // example of how to send a string to C++
        // let_cxx_string!(vertex_name = "VertexSE3Expmap");

        let vertex = optimizer.create_frame_vertex(1, (*frame.pose.as_ref().unwrap()).into());

        if frame.mappoint_matches.len() < 3 {
            return (0, None);
        }

        let mut edges = Vec::new();

        let mut initial_correspondences = 0;
        for i in 0..frame.keypoints_data.num_keypoints() as u32 {
            match frame.mappoint_matches.get(&i) {
                Some(mp_id) => {
                    let keypoint = &frame.keypoints_data.keypoints_un().get(i as usize).unwrap();
                    let edge = optimizer.create_edge_monocular(
                        keypoint.octave, keypoint.pt.x, keypoint.pt.y,
                        self.inv_level_sigma2[keypoint.octave as usize]
                    );
                    // Sofiya: below code sets edge's camera to the frame's camera
                    // but are we sure we need it?
                    // edge->pCamera = pFrame->mpCamera;
                    {
                        let map_read_lock = map.read();
                        let position = &map_read_lock.get_mappoint(mp_id).unwrap().get_position();
                        optimizer.add_edge_monocular(
                            i as i32, edge.clone(), (*position).into()
                        );
                    }

                    edges.push((i, edge));

                    initial_correspondences += 1;
                },
                None => {}
            }
        }

        if initial_correspondences < 3 {
            return (0, None);
        }

        // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
        // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
        let chi2_mono = vec![5.991,5.991,5.991,5.991];
        let _chi2_stereo = vec![7.815,7.815,7.815, 7.815];
        let iterations = vec![10,10,10,10];

        let mut num_bad = 0;
        for iteration in 0..4 {
            // Note: re: vertex.clone() ... 
            // Creates a second shared_ptr holding shared ownership of the same
            // object. There is still only one Object but two SharedPtr<Object>.
            // Both pointers point to the same object on the heap.
            // see https://cxx.rs/binding/sharedptr.html
            optimizer.set_vertex_estimate(
                vertex.clone(), 
                (*frame.pose.as_ref().unwrap()).into()
            );

            optimizer.optimize(iterations[iteration]);

            num_bad = 0;
            for (index, edge) in &edges {
                if frame.mappoint_is_outlier(&index) {
                    edge.compute_error();
                }

                let chi2 = edge.chi2();

                if chi2 > chi2_mono[iteration] {
                    frame.set_outlier(index, true);
                    edge.set_level(1);
                    num_bad += 1;
                } else {
                    frame.set_outlier(index, false);
                    edge.set_level(0);
                }

                if iteration == 2 {
                    edge.set_robust_kernel(false);
                }
            }

            // TODO SLAM with respect to a rigid body...probably don't have to do this rn?
            // vpEdgesMono_FHR comes from "SLAM with respect to a rigid body"
            // which I didn't implement...
            // see add_edge_monocular in rust_helper.cpp

            // for(size_t i=0, iend=vpEdgesMono_FHR.size(); i<iend; i++)
            // {
            //     ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

            //     const size_t idx = vnIndexEdgeRight[i];

            //     if(pFrame->mappoint_outliers[idx])
            //     {
            //         e->computeError();
            //     }

            //     const float chi2 = e->chi2();

            //     if(chi2>chi2_mono[it])
            //     {
            //         pFrame->mappoint_outliers[idx]=true;
            //         e->setLevel(1);
            //         nBad++;
            //     }
            //     else
            //     {
            //         pFrame->mappoint_outliers[idx]=false;
            //         e->setLevel(0);
            //     }

            //     if(it==2)
            //         e->setRobustKernel(0);
            // }

            // TODO (Stereo)
            // for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
            // {
            //     g2o::EdgeStereoSE3ProjectXYZOnlyPose* e = vpEdgesStereo[i];

            //     const size_t idx = vnIndexEdgeStereo[i];

            //     if(pFrame->mappoint_outliers[idx])
            //     {
            //         e->computeError();
            //     }

            //     const float chi2 = e->chi2();

            //     if(chi2>chi2_stereo[it])
            //     {
            //         pFrame->mappoint_outliers[idx]=true;
            //         e->setLevel(1);
            //         nBad++;
            //     }
            //     else
            //     {                
            //         e->setLevel(0);
            //         pFrame->mappoint_outliers[idx]=false;
            //     }

            //     if(it==2)
            //         e->setRobustKernel(0);
            // }

            if optimizer.num_edges() < 10 {
                break;
            }
        }

        // Recover optimized pose
        let pose = optimizer.recover_optimized_pose();

        // Return number of inliers
        return (initial_correspondences - num_bad, Some(pose.into()));
    }

    pub fn global_bundle_adjustment(&self, current_map: Id, iterations: i32) {
        todo!("important: global bundle adjustment");
    }
    // void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
}