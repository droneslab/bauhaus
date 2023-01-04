extern crate g2o;

use std::collections::{HashMap};
use dvcore::{
    lockwrap::ReadOnlyWrapper,
    config::{GLOBAL_PARAMS, SYSTEM_SETTINGS, Sensor},
};
use log::{info, warn, debug};
use nalgebra::Matrix3;
use crate::{dvmap::{frame::Frame, pose::Pose, map::{Map, Id}, keyframe::{KeyFrame, PrelimKeyFrame, FullKeyFrame}}, registered_modules::{FEATURE_DETECTION, CAMERA}};

lazy_static! {
    pub static ref INV_LEVEL_SIGMA2: Vec<f32> = {
    // Note: Does not change, so can have multiple copies of this.
    // ORBSLAM3 duplicates this var at every frame and keyframe,
    // but I'm pretty sure that it's set once per-system when the ORBExtractor
    // is created and only ever used by Optimizer.
    // In general, I'd like to remove these kinds of variables away from the
    // frame/keyframe/mappoint implementation and into the object that actually
    // directly uses it.
    let scale_factor = GLOBAL_PARAMS.get::<f64>(FEATURE_DETECTION, "scale_factor");
    let max_features = GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "max_features");
    let n_levels = GLOBAL_PARAMS.get::<i32>(FEATURE_DETECTION, "n_levels");
    let mut scale_factors = vec![1.0];
    let mut level_sigma2 = vec![1.0];

    for i in 1..n_levels as usize {
        scale_factors.push(scale_factors[i-1] * (scale_factor as f32));
        level_sigma2.push(scale_factors[i] * scale_factors[i]);
    }

    let mut inv_level_sigma2 = Vec::new();
    for i in 0..n_levels as usize {
        inv_level_sigma2.push(1.0 / level_sigma2[i]);
    }
    inv_level_sigma2
};
}
// int Optimizer::PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit)
// but bRecInit is always set to false
pub fn pose_inertial_optimization_last_frame(frame: &mut Frame, map: &ReadOnlyWrapper<Map>) -> i32 {
    // TODO IMU
    // let mut optimizer = g2o::ffi::new_sparse_optimizer(2);

    // let nInitialMonoCorrespondences = 0;
    // let nInitialStereoCorrespondences = 0;
    // let nInitialCorrespondences = 0;

    // Set Current Frame vertex
    // let vertex = optimizer.pin_mut().add_frame_vertex(1, (*frame.pose.as_ref().unwrap()).into(), false);

    // Set MapPoint vertices
//     const int N = pFrame->N;
//     const int Nleft = pFrame->Nleft;
//     const bool bRight = (Nleft!=-1);

//     vector<EdgeMonoOnlyPose*> vpEdgesMono;
//     vector<EdgeStereoOnlyPose*> vpEdgesStereo;
//     vector<size_t> vnIndexEdgeMono;
//     vector<size_t> vnIndexEdgeStereo;
//     vpEdgesMono.reserve(N);
//     vpEdgesStereo.reserve(N);
//     vnIndexEdgeMono.reserve(N);
//     vnIndexEdgeStereo.reserve(N);

//     const float thHuberMono = sqrt(5.991);
//     const float thHuberStereo = sqrt(7.815);

//     {
//         unique_lock<mutex> lock(MapPoint::mGlobalMutex);

//         for(int i=0; i<N; i++)
//         {
//             MapPoint* pMP = pFrame->mvpMapPoints[i];
//             if(pMP)
//             {
//                 cv::KeyPoint kpUn;
//                 // Left monocular observation
//                 if((!bRight && pFrame->mvuRight[i]<0) || i < Nleft)
//                 {
//                     if(i < Nleft) // pair left-right
//                         kpUn = pFrame->mvKeys[i];
//                     else
//                         kpUn = pFrame->mvKeysUn[i];

//                     nInitialMonoCorrespondences++;
//                     pFrame->mvbOutlier[i] = false;

//                     Eigen::Matrix<double,2,1> obs;
//                     obs << kpUn.pt.x, kpUn.pt.y;

//                     EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),0);

//                     e->setVertex(0,VP);
//                     e->setMeasurement(obs);

//                     // Add here uncerteinty
//                     const float unc2 = pFrame->mpCamera->uncertainty2(obs);

//                     const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
//                     e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

//                     g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                     e->setRobustKernel(rk);
//                     rk->setDelta(thHuberMono);

//                     optimizer.addEdge(e);

//                     vpEdgesMono.push_back(e);
//                     vnIndexEdgeMono.push_back(i);
//                 }
//                 // Stereo observation
//                 else if(!bRight)
//                 {
//                     nInitialStereoCorrespondences++;
//                     pFrame->mvbOutlier[i] = false;

//                     kpUn = pFrame->mvKeysUn[i];
//                     const float kp_ur = pFrame->mvuRight[i];
//                     Eigen::Matrix<double,3,1> obs;
//                     obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

//                     EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

//                     e->setVertex(0, VP);
//                     e->setMeasurement(obs);

//                     // Add here uncerteinty
//                     const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

//                     const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
//                     e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

//                     g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                     e->setRobustKernel(rk);
//                     rk->setDelta(thHuberStereo);

//                     optimizer.addEdge(e);

//                     vpEdgesStereo.push_back(e);
//                     vnIndexEdgeStereo.push_back(i);
//                 }

//                 // Right monocular observation
//                 if(bRight && i >= Nleft)
//                 {
//                     nInitialMonoCorrespondences++;
//                     pFrame->mvbOutlier[i] = false;

//                     kpUn = pFrame->mvKeysRight[i - Nleft];
//                     Eigen::Matrix<double,2,1> obs;
//                     obs << kpUn.pt.x, kpUn.pt.y;

//                     EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

//                     e->setVertex(0,VP);
//                     e->setMeasurement(obs);

//                     // Add here uncerteinty
//                     const float unc2 = pFrame->mpCamera->uncertainty2(obs);

//                     const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
//                     e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

//                     g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
//                     e->setRobustKernel(rk);
//                     rk->setDelta(thHuberMono);

//                     optimizer.addEdge(e);

//                     vpEdgesMono.push_back(e);
//                     vnIndexEdgeMono.push_back(i);
//                 }
//             }
//         }
//     }

//     nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

//     // Set Previous Frame Vertex
//     Frame* pFp = pFrame->mpPrevFrame;

//     VertexPose* VPk = new VertexPose(pFp);
//     VPk->setId(4);
//     VPk->setFixed(false);
//     optimizer.addVertex(VPk);
//     VertexVelocity* VVk = new VertexVelocity(pFp);
//     VVk->setId(5);
//     VVk->setFixed(false);
//     optimizer.addVertex(VVk);
//     VertexGyroBias* VGk = new VertexGyroBias(pFp);
//     VGk->setId(6);
//     VGk->setFixed(false);
//     optimizer.addVertex(VGk);
//     VertexAccBias* VAk = new VertexAccBias(pFp);
//     VAk->setId(7);
//     VAk->setFixed(false);
//     optimizer.addVertex(VAk);

//     EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegratedFrame);

//     ei->setVertex(0, VPk);
//     ei->setVertex(1, VVk);
//     ei->setVertex(2, VGk);
//     ei->setVertex(3, VAk);
//     ei->setVertex(4, VP);
//     ei->setVertex(5, VV);
//     optimizer.addEdge(ei);

//     EdgeGyroRW* egr = new EdgeGyroRW();
//     egr->setVertex(0,VGk);
//     egr->setVertex(1,VG);
//     Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
//     egr->setInformation(InfoG);
//     optimizer.addEdge(egr);

//     EdgeAccRW* ear = new EdgeAccRW();
//     ear->setVertex(0,VAk);
//     ear->setVertex(1,VA);
//     Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
//     ear->setInformation(InfoA);
//     optimizer.addEdge(ear);

//     if (!pFp->mpcpi)
//         Verbose::PrintMess("pFp->mpcpi does not exist!!!\nPrevious Frame " + to_string(pFp->mnId), Verbose::VERBOSITY_NORMAL);

//     EdgePriorPoseImu* ep = new EdgePriorPoseImu(pFp->mpcpi);

//     ep->setVertex(0,VPk);
//     ep->setVertex(1,VVk);
//     ep->setVertex(2,VGk);
//     ep->setVertex(3,VAk);
//     g2o::RobustKernelHuber* rkp = new g2o::RobustKernelHuber;
//     ep->setRobustKernel(rkp);
//     rkp->setDelta(5);
//     optimizer.addEdge(ep);

//     // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
//     // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
//     const float chi2Mono[4]={5.991,5.991,5.991,5.991};
//     const float chi2Stereo[4]={15.6f,9.8f,7.815f,7.815f};
//     const int its[4]={10,10,10,10};

//     int nBad=0;
//     int nBadMono = 0;
//     int nBadStereo = 0;
//     int nInliersMono = 0;
//     int nInliersStereo = 0;
//     int nInliers=0;
//     for(size_t it=0; it<4; it++)
//     {
//         optimizer.initializeOptimization(0);
//         optimizer.optimize(its[it]);

//         nBad=0;
//         nBadMono = 0;
//         nBadStereo = 0;
//         nInliers=0;
//         nInliersMono=0;
//         nInliersStereo=0;
//         float chi2close = 1.5*chi2Mono[it];

//         for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
//         {
//             EdgeMonoOnlyPose* e = vpEdgesMono[i];

//             const size_t idx = vnIndexEdgeMono[i];
//             bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth<10.f;

//             if(pFrame->mvbOutlier[idx])
//             {
//                 e->computeError();
//             }

//             const float chi2 = e->chi2();

//             if((chi2>chi2Mono[it]&&!bClose)||(bClose && chi2>chi2close)||!e->isDepthPositive())
//             {
//                 pFrame->mvbOutlier[idx]=true;
//                 e->setLevel(1);
//                 nBadMono++;
//             }
//             else
//             {
//                 pFrame->mvbOutlier[idx]=false;
//                 e->setLevel(0);
//                 nInliersMono++;
//             }

//             if (it==2)
//                 e->setRobustKernel(0);

//         }

//         for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
//         {
//             EdgeStereoOnlyPose* e = vpEdgesStereo[i];

//             const size_t idx = vnIndexEdgeStereo[i];

//             if(pFrame->mvbOutlier[idx])
//             {
//                 e->computeError();
//             }

//             const float chi2 = e->chi2();

//             if(chi2>chi2Stereo[it])
//             {
//                 pFrame->mvbOutlier[idx]=true;
//                 e->setLevel(1);
//                 nBadStereo++;
//             }
//             else
//             {
//                 pFrame->mvbOutlier[idx]=false;
//                 e->setLevel(0);
//                 nInliersStereo++;
//             }

//             if(it==2)
//                 e->setRobustKernel(0);
//         }

//         nInliers = nInliersMono + nInliersStereo;
//         nBad = nBadMono + nBadStereo;

//         if(optimizer.edges().size()<10)
//         {
//             break;
//         }
//     }


//     if ((nInliers<30) && !bRecInit)
//     {
//         nBad=0;
//         const float chi2MonoOut = 18.f;
//         const float chi2StereoOut = 24.f;
//         EdgeMonoOnlyPose* e1;
//         EdgeStereoOnlyPose* e2;
//         for(size_t i=0, iend=vnIndexEdgeMono.size(); i<iend; i++)
//         {
//             const size_t idx = vnIndexEdgeMono[i];
//             e1 = vpEdgesMono[i];
//             e1->computeError();
//             if (e1->chi2()<chi2MonoOut)
//                 pFrame->mvbOutlier[idx]=false;
//             else
//                 nBad++;

//         }
//         for(size_t i=0, iend=vnIndexEdgeStereo.size(); i<iend; i++)
//         {
//             const size_t idx = vnIndexEdgeStereo[i];
//             e2 = vpEdgesStereo[i];
//             e2->computeError();
//             if (e2->chi2()<chi2StereoOut)
//                 pFrame->mvbOutlier[idx]=false;
//             else
//                 nBad++;
//         }
//     }

//     nInliers = nInliersMono + nInliersStereo;


//     // Recover optimized pose, velocity and biases
//     pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
//     Vector6d b;
//     b << VG->estimate(), VA->estimate();
//     pFrame->mImuBias = IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]);

//     // Recover Hessian, marginalize previous frame states and generate new prior for frame
//     Eigen::Matrix<double,30,30> H;
//     H.setZero();

//     H.block<24,24>(0,0)+= ei->GetHessian();

//     Eigen::Matrix<double,6,6> Hgr = egr->GetHessian();
//     H.block<3,3>(9,9) += Hgr.block<3,3>(0,0);
//     H.block<3,3>(9,24) += Hgr.block<3,3>(0,3);
//     H.block<3,3>(24,9) += Hgr.block<3,3>(3,0);
//     H.block<3,3>(24,24) += Hgr.block<3,3>(3,3);

//     Eigen::Matrix<double,6,6> Har = ear->GetHessian();
//     H.block<3,3>(12,12) += Har.block<3,3>(0,0);
//     H.block<3,3>(12,27) += Har.block<3,3>(0,3);
//     H.block<3,3>(27,12) += Har.block<3,3>(3,0);
//     H.block<3,3>(27,27) += Har.block<3,3>(3,3);

//     H.block<15,15>(0,0) += ep->GetHessian();

//     int tot_in = 0, tot_out = 0;
//     for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
//     {
//         EdgeMonoOnlyPose* e = vpEdgesMono[i];

//         const size_t idx = vnIndexEdgeMono[i];

//         if(!pFrame->mvbOutlier[idx])
//         {
//             H.block<6,6>(15,15) += e->GetHessian();
//             tot_in++;
//         }
//         else
//             tot_out++;
//     }

//     for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
//     {
//         EdgeStereoOnlyPose* e = vpEdgesStereo[i];

//         const size_t idx = vnIndexEdgeStereo[i];

//         if(!pFrame->mvbOutlier[idx])
//         {
//             H.block<6,6>(15,15) += e->GetHessian();
//             tot_in++;
//         }
//         else
//             tot_out++;
//     }

//     H = Marginalize(H,0,14);

//     pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,VP->estimate().twb,VV->estimate(),VG->estimate(),VA->estimate(),H.block<15,15>(15,15));
//     delete pFp->mpcpi;
//     pFp->mpcpi = NULL;

//     return nInitialCorrespondences-nBad;
// }
todo!("TODO (IMU) Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame)");
}

//int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
// but bRecInit is always set to false
pub fn pose_inertial_optimization_last_keyframe(frame: &mut Frame) -> i32 {
    // TODO IMU
    // let mut optimizer = g2o::ffi::new_sparse_optimizer(2);

    // let nInitialMonoCorrespondences = 0;
    // let nInitialStereoCorrespondences = 0;
    // let nInitialCorrespondences = 0;

    // // Set Frame vertex
    // let vertex = optimizer.pin_mut().add_frame_vertex(1, (*frame.pose.as_ref().unwrap()).into(), false);


    // Set MapPoint vertices
    // const int N = pFrame->N;
    // const int Nleft = pFrame->Nleft;
    // const bool bRight = (Nleft!=-1);

    // vector<EdgeMonoOnlyPose*> vpEdgesMono;
    // vector<EdgeStereoOnlyPose*> vpEdgesStereo;
    // vector<size_t> vnIndexEdgeMono;
    // vector<size_t> vnIndexEdgeStereo;
    // vpEdgesMono.reserve(N);
    // vpEdgesStereo.reserve(N);
    // vnIndexEdgeMono.reserve(N);
    // vnIndexEdgeStereo.reserve(N);

    // const float thHuberMono = sqrt(5.991);
    // const float thHuberStereo = sqrt(7.815);

    // {
    //     unique_lock<mutex> lock(MapPoint::mGlobalMutex);

    //     for(int i=0; i<N; i++)
    //     {
    //         MapPoint* pMP = pFrame->mvpMapPoints[i];
    //         if(pMP)
    //         {
    //             cv::KeyPoint kpUn;

    //             // Left monocular observation
    //             if((!bRight && pFrame->mvuRight[i]<0) || i < Nleft)
    //             {
    //                 if(i < Nleft) // pair left-right
    //                     kpUn = pFrame->mvKeys[i];
    //                 else
    //                     kpUn = pFrame->mvKeysUn[i];

    //                 nInitialMonoCorrespondences++;
    //                 pFrame->mvbOutlier[i] = false;

    //                 Eigen::Matrix<double,2,1> obs;
    //                 obs << kpUn.pt.x, kpUn.pt.y;

    //                 EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),0);

    //                 e->setVertex(0,VP);
    //                 e->setMeasurement(obs);

    //                 // Add here uncerteinty
    //                 const float unc2 = pFrame->mpCamera->uncertainty2(obs);

    //                 const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
    //                 e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

    //                 g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    //                 e->setRobustKernel(rk);
    //                 rk->setDelta(thHuberMono);

    //                 optimizer.addEdge(e);

    //                 vpEdgesMono.push_back(e);
    //                 vnIndexEdgeMono.push_back(i);
    //             }
    //             // Stereo observation
    //             else if(!bRight)
    //             {
    //                 nInitialStereoCorrespondences++;
    //                 pFrame->mvbOutlier[i] = false;

    //                 kpUn = pFrame->mvKeysUn[i];
    //                 const float kp_ur = pFrame->mvuRight[i];
    //                 Eigen::Matrix<double,3,1> obs;
    //                 obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

    //                 EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

    //                 e->setVertex(0, VP);
    //                 e->setMeasurement(obs);

    //                 // Add here uncerteinty
    //                 const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

    //                 const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
    //                 e->setInformation(Eigen::Matrix3d::Identity()*invSigma2);

    //                 g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    //                 e->setRobustKernel(rk);
    //                 rk->setDelta(thHuberStereo);

    //                 optimizer.addEdge(e);

    //                 vpEdgesStereo.push_back(e);
    //                 vnIndexEdgeStereo.push_back(i);
    //             }

    //             // Right monocular observation
    //             if(bRight && i >= Nleft)
    //             {
    //                 nInitialMonoCorrespondences++;
    //                 pFrame->mvbOutlier[i] = false;

    //                 kpUn = pFrame->mvKeysRight[i - Nleft];
    //                 Eigen::Matrix<double,2,1> obs;
    //                 obs << kpUn.pt.x, kpUn.pt.y;

    //                 EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

    //                 e->setVertex(0,VP);
    //                 e->setMeasurement(obs);

    //                 // Add here uncerteinty
    //                 const float unc2 = pFrame->mpCamera->uncertainty2(obs);

    //                 const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave]/unc2;
    //                 e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

    //                 g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    //                 e->setRobustKernel(rk);
    //                 rk->setDelta(thHuberMono);

    //                 optimizer.addEdge(e);

    //                 vpEdgesMono.push_back(e);
    //                 vnIndexEdgeMono.push_back(i);
    //             }
    //         }
    //     }
    // }
    // nInitialCorrespondences = nInitialMonoCorrespondences + nInitialStereoCorrespondences;

    // KeyFrame* pKF = pFrame->mpLastKeyFrame;
    // VertexPose* VPk = new VertexPose(pKF);
    // VPk->setId(4);
    // VPk->setFixed(true);
    // optimizer.addVertex(VPk);
    // VertexVelocity* VVk = new VertexVelocity(pKF);
    // VVk->setId(5);
    // VVk->setFixed(true);
    // optimizer.addVertex(VVk);
    // VertexGyroBias* VGk = new VertexGyroBias(pKF);
    // VGk->setId(6);
    // VGk->setFixed(true);
    // optimizer.addVertex(VGk);
    // VertexAccBias* VAk = new VertexAccBias(pKF);
    // VAk->setId(7);
    // VAk->setFixed(true);
    // optimizer.addVertex(VAk);

    // EdgeInertial* ei = new EdgeInertial(pFrame->mpImuPreintegrated);

    // ei->setVertex(0, VPk);
    // ei->setVertex(1, VVk);
    // ei->setVertex(2, VGk);
    // ei->setVertex(3, VAk);
    // ei->setVertex(4, VP);
    // ei->setVertex(5, VV);
    // optimizer.addEdge(ei);

    // EdgeGyroRW* egr = new EdgeGyroRW();
    // egr->setVertex(0,VGk);
    // egr->setVertex(1,VG);
    // Eigen::Matrix3d InfoG = pFrame->mpImuPreintegrated->C.block<3,3>(9,9).cast<double>().inverse();
    // egr->setInformation(InfoG);
    // optimizer.addEdge(egr);

    // EdgeAccRW* ear = new EdgeAccRW();
    // ear->setVertex(0,VAk);
    // ear->setVertex(1,VA);
    // Eigen::Matrix3d InfoA = pFrame->mpImuPreintegrated->C.block<3,3>(12,12).cast<double>().inverse();
    // ear->setInformation(InfoA);
    // optimizer.addEdge(ear);

    // // We perform 4 optimizations, after each optimization we classify observation as inlier/outlier
    // // At the next optimization, outliers are not included, but at the end they can be classified as inliers again.
    // float chi2Mono[4]={12,7.5,5.991,5.991};
    // float chi2Stereo[4]={15.6,9.8,7.815,7.815};

    // int its[4]={10,10,10,10};

    // int nBad = 0;
    // int nBadMono = 0;
    // int nBadStereo = 0;
    // int nInliersMono = 0;
    // int nInliersStereo = 0;
    // int nInliers = 0;
    // for(size_t it=0; it<4; it++)
    // {
    //     optimizer.initializeOptimization(0);
    //     optimizer.optimize(its[it]);

    //     nBad = 0;
    //     nBadMono = 0;
    //     nBadStereo = 0;
    //     nInliers = 0;
    //     nInliersMono = 0;
    //     nInliersStereo = 0;
    //     float chi2close = 1.5*chi2Mono[it];

    //     // For monocular observations
    //     for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
    //     {
    //         EdgeMonoOnlyPose* e = vpEdgesMono[i];

    //         const size_t idx = vnIndexEdgeMono[i];

    //         if(pFrame->mvbOutlier[idx])
    //         {
    //             e->computeError();
    //         }

    //         const float chi2 = e->chi2();
    //         bool bClose = pFrame->mvpMapPoints[idx]->mTrackDepth<10.f;

    //         if((chi2>chi2Mono[it]&&!bClose)||(bClose && chi2>chi2close)||!e->isDepthPositive())
    //         {
    //             pFrame->mvbOutlier[idx]=true;
    //             e->setLevel(1);
    //             nBadMono++;
    //         }
    //         else
    //         {
    //             pFrame->mvbOutlier[idx]=false;
    //             e->setLevel(0);
    //             nInliersMono++;
    //         }

    //         if (it==2)
    //             e->setRobustKernel(0);
    //     }

    //     // For stereo observations
    //     for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
    //     {
    //         EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    //         const size_t idx = vnIndexEdgeStereo[i];

    //         if(pFrame->mvbOutlier[idx])
    //         {
    //             e->computeError();
    //         }

    //         const float chi2 = e->chi2();

    //         if(chi2>chi2Stereo[it])
    //         {
    //             pFrame->mvbOutlier[idx]=true;
    //             e->setLevel(1); // not included in next optimization
    //             nBadStereo++;
    //         }
    //         else
    //         {
    //             pFrame->mvbOutlier[idx]=false;
    //             e->setLevel(0);
    //             nInliersStereo++;
    //         }

    //         if(it==2)
    //             e->setRobustKernel(0);
    //     }

    //     nInliers = nInliersMono + nInliersStereo;
    //     nBad = nBadMono + nBadStereo;

    //     if(optimizer.edges().size()<10)
    //     {
    //         break;
    //     }

    // }

    // // If not too much tracks, recover not too bad points
    // if ((nInliers<30) && !bRecInit)
    // {
    //     nBad=0;
    //     const float chi2MonoOut = 18.f;
    //     const float chi2StereoOut = 24.f;
    //     EdgeMonoOnlyPose* e1;
    //     EdgeStereoOnlyPose* e2;
    //     for(size_t i=0, iend=vnIndexEdgeMono.size(); i<iend; i++)
    //     {
    //         const size_t idx = vnIndexEdgeMono[i];
    //         e1 = vpEdgesMono[i];
    //         e1->computeError();
    //         if (e1->chi2()<chi2MonoOut)
    //             pFrame->mvbOutlier[idx]=false;
    //         else
    //             nBad++;
    //     }
    //     for(size_t i=0, iend=vnIndexEdgeStereo.size(); i<iend; i++)
    //     {
    //         const size_t idx = vnIndexEdgeStereo[i];
    //         e2 = vpEdgesStereo[i];
    //         e2->computeError();
    //         if (e2->chi2()<chi2StereoOut)
    //             pFrame->mvbOutlier[idx]=false;
    //         else
    //             nBad++;
    //     }
    // }

    // // Recover optimized pose, velocity and biases
    // pFrame->SetImuPoseVelocity(VP->estimate().Rwb.cast<float>(), VP->estimate().twb.cast<float>(), VV->estimate().cast<float>());
    // Vector6d b;
    // b << VG->estimate(), VA->estimate();
    // pFrame->mImuBias = IMU::Bias(b[3],b[4],b[5],b[0],b[1],b[2]);

    // // Recover Hessian, marginalize keyFframe states and generate new prior for frame
    // Eigen::Matrix<double,15,15> H;
    // H.setZero();

    // H.block<9,9>(0,0)+= ei->GetHessian2();
    // H.block<3,3>(9,9) += egr->GetHessian2();
    // H.block<3,3>(12,12) += ear->GetHessian2();

    // int tot_in = 0, tot_out = 0;
    // for(size_t i=0, iend=vpEdgesMono.size(); i<iend; i++)
    // {
    //     EdgeMonoOnlyPose* e = vpEdgesMono[i];

    //     const size_t idx = vnIndexEdgeMono[i];

    //     if(!pFrame->mvbOutlier[idx])
    //     {
    //         H.block<6,6>(0,0) += e->GetHessian();
    //         tot_in++;
    //     }
    //     else
    //         tot_out++;
    // }

    // for(size_t i=0, iend=vpEdgesStereo.size(); i<iend; i++)
    // {
    //     EdgeStereoOnlyPose* e = vpEdgesStereo[i];

    //     const size_t idx = vnIndexEdgeStereo[i];

    //     if(!pFrame->mvbOutlier[idx])
    //     {
    //         H.block<6,6>(0,0) += e->GetHessian();
    //         tot_in++;
    //     }
    //     else
    //         tot_out++;
    // }

    // pFrame->mpcpi = new ConstraintPoseImu(VP->estimate().Rwb,VP->estimate().twb,VV->estimate(),VG->estimate(),VA->estimate(),H);

    // return nInitialCorrespondences-nBad;
    todo!("TODO (IMU) Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame)");
}

pub fn optimize_pose(frame: &mut Frame, map: &ReadOnlyWrapper<Map>) -> Option<(i32, Pose)> {
    //int Optimizer::PoseOptimization(Frame *pFrame)
    let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");

    let fx= GLOBAL_PARAMS.get::<f64>(CAMERA, "fx");
    let fy= GLOBAL_PARAMS.get::<f64>(CAMERA, "fy");
    let cx= GLOBAL_PARAMS.get::<f64>(CAMERA, "cx");
    let cy= GLOBAL_PARAMS.get::<f64>(CAMERA, "cy");

    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(1, camera_param);

    // Don't need this but saving here for reference
    // example of how to send a string to C++
    // let_cxx_string!(vertex_name = "VertexSE3Expmap");

    let frame_vertex_id = 1;
    optimizer.pin_mut().add_frame_vertex(frame_vertex_id, (*frame.pose.as_ref().unwrap()).into(), false);

    if frame.mappoint_matches.len() < 3 {
        return None;
    }

    let mut edges = Vec::new();

    let mut initial_correspondences = 0;
    for i in 0..frame.features.num_keypoints as u32 {
        match frame.mappoint_matches.get(&i) {
            Some((mp_id, _)) => {
                let keypoint = &frame.features.get_keypoint(i as usize);

                let edge = match sensor.frame() {
                    crate::FrameSensor::Stereo => {
                        // Stereo observations
                        todo!("TODO (Stereo)");
                        // let edge = optimizer.create_edge_stereo(
                        //     keypoint.octave, keypoint.pt.x, keypoint.pt.y,
                        //    frame.keypoints_data.mv_right.get(mp_id).unwrap(),
                        //     self.inv_level_sigma2[keypoint.octave as usize]
                        // );

                        // {
                        //     let map_read_lock = map.read();
                        //     let position = &map_read_lock.get_mappoint(mp_id).unwrap().position;
                        //     optimizer.add_edge_stereo(
                        //         i as i32, edge.clone(), (position).into()
                        //     );
                        // }
                    },
                    _ => {
                        // Mono observations
                        let map_read_lock = map.read();
                        let position = &map_read_lock.get_mappoint(&mp_id).unwrap().position;
                        let edge = optimizer.pin_mut().add_edge_monocular_unary(
                            false, frame_vertex_id, keypoint.octave, keypoint.pt.x, keypoint.pt.y,
                            INV_LEVEL_SIGMA2[keypoint.octave as usize],
                            (position).into()
                        );
                        // Sofiya: below code sets edge's camera to the frame's camera
                        // but are we sure we need it?
                        // edge->pCamera = pFrame->mpCamera;
                        edge
                    }
                };

                edges.push((i, edge));

                initial_correspondences += 1;
            },
            None => {}
        }
    }

    if initial_correspondences < 3 {
        return None;
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
        optimizer.pin_mut().set_vertex_estimate(
            frame_vertex_id, 
            (*frame.pose.as_ref().unwrap()).into()
        );

        optimizer.pin_mut().optimize(iterations[iteration]);

        num_bad = 0;
        // Sofiya todo; need to uncomment this
        for (index, edge) in &mut edges {
            if frame.is_mp_outlier(&index) {
                edge.pin_mut().compute_error();
            }

            let chi2 = edge.chi2();

            if chi2 > chi2_mono[iteration] {
                frame.set_mp_outlier(&index, true);
                edge.pin_mut().set_level(1);
                num_bad += 1;
            } else {
                frame.set_mp_outlier(&index, false);
                edge.pin_mut().set_level(0);
            }

            if iteration == 2 {
                edge.pin_mut().set_robust_kernel(false);
            }
        }

        // TODO (rigid body) SLAM with respect to a rigid body...probably don't have to do this rn?
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
    let pose = optimizer.recover_optimized_frame_pose(frame_vertex_id);

    // Return number of inliers
    return Some((initial_correspondences - num_bad, pose.into()));
}

pub fn global_bundle_adjustment(map: &Map, loop_kf: i32, iterations: i32) -> BAResult {
    // void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");

    let fx= GLOBAL_PARAMS.get::<f64>(CAMERA, "fx");
    let fy= GLOBAL_PARAMS.get::<f64>(CAMERA, "fy");
    let cx= GLOBAL_PARAMS.get::<f64>(CAMERA, "cx");
    let cy= GLOBAL_PARAMS.get::<f64>(CAMERA, "cy");
    
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(1, camera_param);

    let mut max_kf_id = 0;

    // Set KeyFrame vertices
    let keyframes = map.get_all_keyframes();
    let mut kf_vertex_ids = Vec::new();//HashMap::<Id, SharedPtr<VertexSE3Expmap>>::new();
    for (id, kf) in keyframes {
        optimizer.pin_mut().add_frame_vertex(
            *id,
            (kf.pose).into(),
            *id == map.initial_kf_id
        );
        kf_vertex_ids.push(*id);

        if *id > max_kf_id {
            max_kf_id = *id;
        }
    }

    let mut mappoint_vertices = Vec::new();
    // Set MapPoint vertices
    let mappoints = map.get_all_mappoints();

    for (mp_id, mappoint) in mappoints {
        let mp_vertex_id = mp_id + max_kf_id + 1;
        optimizer.pin_mut().add_mappoint_vertex(
            mp_vertex_id,
            Pose::new(&*mappoint.position, &Matrix3::identity()).into() // create pose out of translation only
        );
        let observations = mappoint.get_observations();

        let mut n_edges = 0;

        //SET EDGES
        for kf_id in observations.keys() {
            if !optimizer.has_vertex(mp_vertex_id) || !optimizer.has_vertex(*kf_id) {
                continue;
            }

            let (left_index, right_index) = observations.get_observation(kf_id);

            match sensor.frame() {
                crate::FrameSensor::Stereo => {
                    todo!("TODO (Stereo), Optimizer lines 194-226");
                },
                _ => {
                    if left_index != -1 {
                        n_edges += 1;

                        let keypoint = map.get_keyframe(kf_id).unwrap().features.get_keypoint(left_index as usize);
                        optimizer.pin_mut().add_edge_monocular_binary(
                            false, mp_vertex_id, *kf_id,
                            keypoint.octave, keypoint.pt.x, keypoint.pt.y,
                            INV_LEVEL_SIGMA2[keypoint.octave as usize]
                        );
                        // Sofiya: below code sets edge's camera to the frame's camera
                        // but are we sure we need it?
                        // edge->pCamera = pFrame->mpCamera;
                    }
                }
            };

            // if(pKF->mpCamera2){
            // TODO (Stereo) Optimizer lines 229-261
            // }
        }

        if n_edges == 0 {
            warn!("Removed vertex");
            optimizer.pin_mut().remove_vertex(mp_vertex_id);
        } else {
            mappoint_vertices.push((mp_vertex_id, mp_id));
        }
    }

    // Optimize!
    optimizer.pin_mut().optimize(iterations);
    info!("optimizer::global_bundle_adjustment;End of the optimization");

    // Recover optimized data
    // Keyframes
    let mut optimized_kf_poses = HashMap::<Id, Pose>::new();
    for id in kf_vertex_ids {
        let pose = optimizer.recover_optimized_frame_pose(id);
        optimized_kf_poses.insert(id, pose.into());
    }

    //Points
    let mut optimized_mp_poses = HashMap::<Id, Pose>::new();
    for (mp_vertex_id, mp_id) in mappoint_vertices {
        let pose = optimizer.recover_optimized_mappoint_pose(mp_vertex_id);
        optimized_mp_poses.insert(*mp_id, pose.into());
    }

    BAResult {
        optimized_mp_poses, optimized_kf_poses,
        loop_kf_is_first_kf: loop_kf == map.initial_kf_id
    }
}

pub fn local_bundle_adjustment(
    map: &Map, keyframe: &KeyFrame<FullKeyFrame>,
    force_stop_flag: bool,
    num_opt_kf: i32,num_fixed_kf: i32, num_mps: i32, num_edges: i32
) -> BAResult {
    // void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges)
    todo!("TODO LOCAL MAPPING, local bundle adjustment");
}


pub struct BAResult {
    pub optimized_mp_poses: HashMap::<Id, Pose>,
    pub optimized_kf_poses: HashMap::<Id, Pose>,
    pub loop_kf_is_first_kf: bool
}