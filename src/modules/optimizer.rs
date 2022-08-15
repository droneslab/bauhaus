extern crate g2orust;
use cxx::{let_cxx_string};
use std::convert::TryInto;

use darvis::{
    map::{
        frame::Frame, pose::Pose, map::Map
    },
    lockwrap::ReadOnlyWrapper,
    dvutils::{DVVector3, DVMatrix3, DVVector3_to_array}
};

// Note: can't implement From trait because 
// g2orust::ffi::Pose isn't defined in this crate
fn pose_to_bridgepose(pose: &Pose) -> g2orust::ffi::Pose {
    g2orust::ffi::Pose {
        translation: pose.t_to_array(),
        rotation: pose.r_quaternion_to_array()
    }
}

pub fn optimize_pose(
    frame: &mut Frame, map: &ReadOnlyWrapper<Map>
) -> (Option<Pose>, i32) {
    let optimizer = g2orust::ffi::new_sparse_optimizer();

    // let_cxx_string!(vertex_name = "VertexSE3Expmap"); // Don't need this but saving here for reference

    let vertex = optimizer.create_frame_vertex(
        1,
        pose_to_bridgepose(&(frame.pose.as_ref().unwrap()))
    );

    if frame.mappoint_matches.len() < 3 {
        return (None, 0);
    }

    let mut edges = Vec::new();

    let mut initial_correspondences = 0;
    for i in 0..frame.N {
    // for (index, mp_id) in &frame.mappoint_matches {
        match frame.mappoint_matches.get(&(i as i32)) {
            Some(mp_id) => {
                let keypoint = &frame.key_points_un.get(i).unwrap();
                let edge = optimizer.create_edge_monocular(
                    keypoint.octave, keypoint.pt.x, keypoint.pt.y,
                    frame.mvInvLevelSigma2[keypoint.octave as usize]
                );
                // Sofiya TODO; uncomment this
                // edge->pCamera = pFrame->mpCamera;

                let position;
                {
                    let map_read_lock = map.read();
                    let mappoint = map_read_lock.mappoints.get(mp_id).unwrap();
                    position = mappoint.position;
                }

                optimizer.add_edge_monocular(
                    i as i32, edge.clone(), DVVector3_to_array(position));

                edges.push((i as i32, edge));

                initial_correspondences += 1;
            },
            None => {}
        }
    }

    if initial_correspondences < 3 {
        return (None, 0);
    }

    // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    let chi2Mono = vec![5.991,5.991,5.991,5.991];
    let chi2Stereo = vec![7.815,7.815,7.815, 7.815];
    let iterations = vec![10,10,10,10];

    let mut num_bad = 0;
    for iteration in 0..4 {
        optimizer.set_vertex_estimate(
            vertex.clone(), 
            pose_to_bridgepose(&(frame.pose.as_ref().unwrap()))
        );
        // Note: re: vertex.clone() ... 
        // Creates a second shared_ptr holding shared ownership of the same
        // object. There is still only one Object but two SharedPtr<Object>.
        // Both pointers point to the same object on the heap.
        // see https://cxx.rs/binding/sharedptr.html

        optimizer.optimize(iterations[iteration]);

        num_bad = 0;
        for (index, edge) in &edges {
            if frame.mappoint_outliers[&index] {
                edge.compute_error();
            }

            let chi2 = edge.chi2();

            if chi2 > chi2Mono[iteration] {
                *frame.mappoint_outliers.get_mut(index).unwrap() = true;
                edge.set_level(1);
                num_bad += 1;
            } else {
                *frame.mappoint_outliers.get_mut(index).unwrap() = false;
                edge.set_level(0);
            }

            if iteration == 2 {
                edge.set_robust_kernel(false);
            }
        }

        // Sofiya TODO
        // vpEdgesMono_FHR comes from "SLAM with respect to a rigid body"
        // which I didn't implement...
        // see add_edge_monocular in rust_helper.cpp

        // for(size_t i=0, iend=vpEdgesMono_FHR.size(); i<iend; i++)
        // {
        //     ORB_SLAM3::EdgeSE3ProjectXYZOnlyPoseToBody* e = vpEdgesMono_FHR[i];

        //     const size_t idx = vnIndexEdgeRight[i];

        //     if(pFrame->mvbOutlier[idx])
        //     {
        //         e->computeError();
        //     }

        //     const float chi2 = e->chi2();

        //     if(chi2>chi2Mono[it])
        //     {
        //         pFrame->mvbOutlier[idx]=true;
        //         e->setLevel(1);
        //         nBad++;
        //     }
        //     else
        //     {
        //         pFrame->mvbOutlier[idx]=false;
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

        //     if(pFrame->mvbOutlier[idx])
        //     {
        //         e->computeError();
        //     }

        //     const float chi2 = e->chi2();

        //     if(chi2>chi2Stereo[it])
        //     {
        //         pFrame->mvbOutlier[idx]=true;
        //         e->setLevel(1);
        //         nBad++;
        //     }
        //     else
        //     {                
        //         e->setLevel(0);
        //         pFrame->mvbOutlier[idx]=false;
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

    //     let map_msg = MapWriteMsg::set_pose(frame, pose);
    // map_actor.send_new(map_msg).unwrap();

    // Return pose and number of inliers
    return (Some(Pose::new_from_bridgepose(pose)), initial_correspondences - num_bad);
}