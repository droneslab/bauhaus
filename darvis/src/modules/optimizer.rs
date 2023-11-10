extern crate g2o;

use std::collections::{HashMap, HashSet};
use cxx::UniquePtr;
use dvcore::{
    lockwrap::ReadOnlyWrapper,
    config::{GLOBAL_PARAMS, SYSTEM_SETTINGS}, sensor::{Sensor, FrameSensor},
};
use log::{warn, debug};
use nalgebra::Matrix3;
use opencv::prelude::KeyPointTraitConst;
use crate::{dvmap::{keyframe::InitialFrame, pose::DVPose, map::{Map, Id}, keyframe::{Frame, FullKeyFrame}}, registered_actors::{FEATURE_DETECTION, CAMERA}, actors::map_actor::MapWriteMsg};
use g2o::ffi::EdgeSE3ProjectXYZ;

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
pub fn pose_inertial_optimization_last_frame(_frame: &mut Frame<InitialFrame>, _map: &ReadOnlyWrapper<Map>) -> i32 {
    todo!("IMU... Optimizer::PoseInertialOptimizationLastFrame(&mCurrentFrame)");
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
//                     obs << kpUn.pt().x, kpUn.pt().y;

//                     EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),0);

//                     e->setVertex(0,VP);
//                     e->setMeasurement(obs);

//                     // Add here uncerteinty
//                     const float unc2 = pFrame->mpCamera->uncertainty2(obs);

//                     const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave()]/unc2;
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
//                     obs << kpUn.pt().x, kpUn.pt().y, kp_ur;

//                     EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

//                     e->setVertex(0, VP);
//                     e->setMeasurement(obs);

//                     // Add here uncerteinty
//                     const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

//                     const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave()]/unc2;
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
//                     obs << kpUn.pt().x, kpUn.pt().y;

//                     EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

//                     e->setVertex(0,VP);
//                     e->setMeasurement(obs);

//                     // Add here uncerteinty
//                     const float unc2 = pFrame->mpCamera->uncertainty2(obs);

//                     const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave()]/unc2;
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
}

//int Optimizer::PoseInertialOptimizationLastKeyFrame(Frame *pFrame, bool bRecInit)
// but bRecInit is always set to false
pub fn pose_inertial_optimization_last_keyframe(_frame: &mut Frame<InitialFrame>) -> i32 {
    todo!("IMU... Optimizer::PoseInertialOptimizationLastKeyFrame(&mCurrentFrame)");
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
    //                 obs << kpUn.pt().x, kpUn.pt().y;

    //                 EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),0);

    //                 e->setVertex(0,VP);
    //                 e->setMeasurement(obs);

    //                 // Add here uncerteinty
    //                 const float unc2 = pFrame->mpCamera->uncertainty2(obs);

    //                 const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave()]/unc2;
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
    //                 obs << kpUn.pt().x, kpUn.pt().y, kp_ur;

    //                 EdgeStereoOnlyPose* e = new EdgeStereoOnlyPose(pMP->GetWorldPos());

    //                 e->setVertex(0, VP);
    //                 e->setMeasurement(obs);

    //                 // Add here uncerteinty
    //                 const float unc2 = pFrame->mpCamera->uncertainty2(obs.head(2));

    //                 const float &invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave()]/unc2;
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
    //                 obs << kpUn.pt().x, kpUn.pt().y;

    //                 EdgeMonoOnlyPose* e = new EdgeMonoOnlyPose(pMP->GetWorldPos(),1);

    //                 e->setVertex(0,VP);
    //                 e->setMeasurement(obs);

    //                 // Add here uncerteinty
    //                 const float unc2 = pFrame->mpCamera->uncertainty2(obs);

    //                 const float invSigma2 = pFrame->mvInvLevelSigma2[kpUn.octave()]/unc2;
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
}

pub fn optimize_pose(frame: &mut Frame<InitialFrame>, map: &ReadOnlyWrapper<Map>) -> Option<(i32, DVPose)> {
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

    let mut initial_correspondences = 0;
    let mut mp_indexes = vec![];
    for i in 0..frame.features.num_keypoints as u32 {
        match frame.mappoint_matches.get(&i) {
            Some((mp_id, _)) => {
                let (keypoint, _) = &frame.features.get_keypoint(i as usize);
                mp_indexes.push(i);

                match sensor.frame() {
                    FrameSensor::Stereo => {
                        // Stereo observations
                        todo!("Stereo");
                        // let edge = optimizer.create_edge_stereo(
                        //     keypoint.octave(), keypoint.pt().x, keypoint.pt().y,
                        //    frame.keypoints_data.mv_right.get(mp_id).unwrap(),
                        //     self.inv_level_sigma2[keypoint.octave() as usize]
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
                        optimizer.pin_mut().add_edge_monocular_unary(
                            true, frame_vertex_id, keypoint.octave(), keypoint.pt().x, keypoint.pt().y,
                            INV_LEVEL_SIGMA2[keypoint.octave() as usize],
                            (position).into(),
                            *mp_id
                        );
                        // TODO (mvp): below code sets edge's camera to the frame's camera
                        // but are we sure we need it?
                        // edge->pCamera = pFrame->mpCamera;
                    }
                };

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
        let mut index = 0;
        // SOFIYA TEST
        // for mut edge in optimizer.pin_mut().get_mut_xyz_onlypose_edges().iter_mut() {
        //     if frame.is_mp_outlier(&mp_indexes[index]) {
        //         edge.inner.pin_mut().compute_error();
        //     }

        //     let chi2 = edge.inner.chi2();

        //     if chi2 > chi2_mono[iteration] {
        //         frame.set_mp_outlier(&mp_indexes[index], true);
        //         edge.inner.pin_mut().set_level(1);
        //         num_bad += 1;
        //     } else {
        //         frame.set_mp_outlier(&mp_indexes[index], false);
        //         edge.inner.pin_mut().set_level(0);
        //     }

        //     if iteration == 2 {
        //         edge.inner.pin_mut().set_robust_kernel(false);
        //     }
        //     index += 1;
        // }

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

        match sensor.frame() {
            FrameSensor::Stereo => {
                todo!("Stereo");
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
            },
            _ => {}
        }

        if optimizer.num_edges() < 10 {
            break;
        }
    }
    debug!("Set {} outliers in pose optimization", num_bad);

    // Recover optimized pose
    let pose = optimizer.recover_optimized_frame_pose(frame_vertex_id);

    // Return number of inliers
    return Some((initial_correspondences - num_bad, pose.into()));
}

pub fn global_bundle_adjustment(map: &Map, iterations: i32) -> BAResult {
    // void Optimizer::GlobalBundleAdjustemnt(Map* pMap, int nIterations, bool* pbStopFlag, const unsigned long nLoopKF, const bool bRobust)
    let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");
    let fx= GLOBAL_PARAMS.get::<f64>(CAMERA, "fx");
    let fy= GLOBAL_PARAMS.get::<f64>(CAMERA, "fy");
    let cx= GLOBAL_PARAMS.get::<f64>(CAMERA, "cx");
    let cy= GLOBAL_PARAMS.get::<f64>(CAMERA, "cy");
    let camera_param = [fx, fy, cx,cy];

    let mut optimizer = g2o::ffi::new_sparse_optimizer(1, camera_param);

    // Set KeyFrame vertices
    let keyframes = map.get_all_keyframes();
    let mut kf_vertex_ids = HashMap::new();
    let mut id_count = 0;
    for (id, kf) in keyframes {
        optimizer.pin_mut().add_frame_vertex(
            id_count,
            (kf.pose.unwrap()).into(),
            *id == map.initial_kf_id
        );
        kf_vertex_ids.insert(id, id_count);
        id_count += 1;
    }

    let mut mp_vertex_ids = HashMap::new();
    // Set MapPoint vertices
    let mappoints = map.get_all_mappoints();

    let mut _edges = Vec::new();
    for (mp_id, mappoint) in mappoints {
        optimizer.pin_mut().add_mappoint_vertex(
            id_count,
            DVPose::new(&*mappoint.position, &Matrix3::identity()).into() // create pose out of translation only
        );
        mp_vertex_ids.insert(mp_id, id_count);

        let observations = mappoint.get_observations();
        let mut n_edges = 0;

        //SET EDGES
        for (kf_id, (left_index, _right_index)) in observations {
            if !optimizer.has_vertex(id_count) || !optimizer.has_vertex(*kf_id) {
                continue;
            }

            match sensor.frame() {
                FrameSensor::Stereo => {
                    todo!("Stereo, Optimizer lines 194-226");
                },
                _ => {
                    if *left_index != -1 {
                        n_edges += 1;

                        let (keypoint, _) = map.get_keyframe(kf_id).unwrap().features.get_keypoint(*left_index as usize);
                        _edges.push(
                            optimizer.pin_mut().add_edge_monocular_binary(
                                true, id_count, *kf_vertex_ids.get(kf_id).unwrap(),
                                keypoint.octave(), keypoint.pt().x, keypoint.pt().y,
                                INV_LEVEL_SIGMA2[keypoint.octave() as usize],
                                0
                            )
                        );
                        // TODO (mvp): below code sets edge's camera to the frame's camera
                        // but are we sure we need it?
                        // edge->pCamera = pFrame->mpCamera;
                    }
                }
            };

            match sensor.frame() {
                FrameSensor::Stereo => {
                    todo!("Stereo, optimizer lines 229-261");
                    // if pkf->mpcamera2...
                },
                _ => {}
            }
        }

        if n_edges == 0 {
            warn!("Removed vertex");
            optimizer.pin_mut().remove_vertex(id_count);
        } else {
            mp_vertex_ids.remove(mp_id);
        }
        id_count += 1;
    }

    // Optimize!
    optimizer.pin_mut().optimize(iterations);

    // Recover optimized data
    // Keyframes
    let mut optimized_kf_poses = HashMap::<Id, DVPose>::new();
    for (kf_id, vertex_id) in kf_vertex_ids {
        let pose = optimizer.recover_optimized_frame_pose(vertex_id);
        optimized_kf_poses.insert(*kf_id, pose.into());
    }

    //Points
    let mut optimized_mp_poses = HashMap::<Id, DVPose>::new();
    for (mp_id, vertex_id) in mp_vertex_ids {
        let pose = optimizer.recover_optimized_mappoint_pose(vertex_id);
        optimized_mp_poses.insert(*mp_id, pose.into());
    }

    BAResult { optimized_mp_poses, optimized_kf_poses, }
}

pub fn local_bundle_adjustment(
    map: &Map, keyframe: &Frame<FullKeyFrame>,
) -> Vec<MapWriteMsg> {
    todo!();
    // SOFIYA TEST
    // // void Optimizer::LocalBundleAdjustment(KeyFrame *pKF, bool* pbStopFlag, Map* pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges)

    // // Local KeyFrames: First Breath Search from Current Keyframe
    // let mut local_keyframes = vec![keyframe.id()];
    // let current_map_id = map.id;
    // let mut local_ba_for_kf = HashMap::new();
    // local_ba_for_kf.insert(keyframe.id(), keyframe.id());

    // for kf_id in keyframe.get_connections(i32::MAX) {
    //     let kf = map.get_keyframe(&kf_id).unwrap();
    //     local_ba_for_kf.insert(kf_id, keyframe.id());
    //     if kf.full_kf_info.origin_map_id == current_map_id {
    //         local_keyframes.push(kf_id);
    //     }
    // }

    // // Local MapPoints seen in Local KeyFrames
    // let mut num_fixed_kf = 0;
    // let mut local_mappoints = Vec::<Id>::new();
    // let mut local_ba_for_mp = HashMap::new();

    // for kf_id in &local_keyframes {
    //     let kf = map.get_keyframe(&kf_id).unwrap();
    //     if kf.id() == map.initial_kf_id {
    //         num_fixed_kf += 1;
    //     }
    //     for (_, (mp_id, _)) in &kf.mappoint_matches {
    //         let mp = map.get_mappoint(&mp_id).unwrap();
    //         if mp.origin_map_id == current_map_id {
    //             local_mappoints.push(*mp_id);
    //             local_ba_for_mp.insert(mp_id, keyframe.id());
    //         }
    //     }
    // }

    // // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    // let mut fixed_cameras = Vec::new();
    // let mut ba_fixed_for_kf = HashSet::new();
    // for mp_id in &local_mappoints {
    //     let mp = map.get_mappoint(&mp_id).unwrap();
    //     for (kf_id, (_left_index, _right_index)) in mp.get_observations() {
    //         let kf = map.get_keyframe(&kf_id).unwrap();
    //         let local_ba = match local_ba_for_kf.contains_key(kf_id) {
    //             true => *local_ba_for_kf.get(kf_id).unwrap() != keyframe.id(),
    //             false => true
    //         };
    //         if local_ba && !ba_fixed_for_kf.contains(kf_id) {
    //             ba_fixed_for_kf.insert(kf_id);
    //             if kf.full_kf_info.origin_map_id == current_map_id {
    //                 fixed_cameras.push(*kf_id);
    //             }
    //         }
    //     }
    // }
    // if fixed_cameras.len() + num_fixed_kf == 0 {
    //     warn!("LM_LBA: There are 0 fixed KF in the optimizations, LBA aborted");
    // }

    // // Setup optimizer
    // let sensor: Sensor = GLOBAL_PARAMS.get(SYSTEM_SETTINGS, "sensor");
    // let fx= GLOBAL_PARAMS.get::<f64>(CAMERA, "fx");
    // let fy= GLOBAL_PARAMS.get::<f64>(CAMERA, "fy");
    // let cx= GLOBAL_PARAMS.get::<f64>(CAMERA, "cx");
    // let cy= GLOBAL_PARAMS.get::<f64>(CAMERA, "cy");
    // let camera_param = [fx, fy, cx,cy];
    // let mut optimizer = g2o::ffi::new_sparse_optimizer(1, camera_param);

    // match sensor.is_imu() {
    //     true => {
    //         todo!("IMU");
    //         // Need to put this in the C++ constructor for BridgeSparseOptimizer::BridgeSparseOptimizer
    //         // if (pMap->IsInertial())
    //         //     solver->setUserLambdaInit(100.0);
    //     },
    //     false => {}
    // }

    // // TODO (design): mbAbortBA 
    // // if(pbStopFlag)
    // //     optimizer.setForceStopFlag(pbStopFlag);

    // // Set Local KeyFrame vertices
    // let mut vertex_id = 1;
    // let mut all_kf_vertices = HashMap::new();
    // for kf_id in &local_keyframes {
    //     let kf = map.get_keyframe(kf_id).unwrap();
    //     let set_fixed = *kf_id == map.initial_kf_id;
    //     optimizer.pin_mut().add_frame_vertex(vertex_id, (*kf.pose.as_ref().unwrap()).into(), set_fixed);
    //     all_kf_vertices.insert(*kf_id, vertex_id);
    //     vertex_id += 1;
    // }

    // // Set Fixed KeyFrame vertices
    // for kf_id in &fixed_cameras {
    //     let kf = map.get_keyframe(kf_id).unwrap();
    //     optimizer.pin_mut().add_frame_vertex(vertex_id, (*kf.pose.as_ref().unwrap()).into(), true);
    //     all_kf_vertices.insert(*kf_id, vertex_id);
    //     vertex_id += 1;
    // }

    // // Set MapPoint vertices
    // let mut all_mp_vertices = HashMap::new();
    // let all_edges_stereo = Vec::<(Id, UniquePtr<EdgeSE3ProjectXYZ>)>::new(); //vpEdgesStereo
    // let all_edges_body = Vec::<(Id, UniquePtr<EdgeSE3ProjectXYZ>)>::new(); // vpEdgesBody

    // for mp_id in &local_mappoints {
    //     let mp = map.get_mappoint(mp_id).unwrap();

    //     optimizer.pin_mut().add_mappoint_vertex(
    //         vertex_id,
    //         DVPose::new(&*mp.position, &Matrix3::identity()).into() // create pose out of translation only
    //     );
    //     all_mp_vertices.insert(*mp_id, vertex_id);

    //     let observations = mp.get_observations();
    //     // Set edges
    //     for (kf_id, (left_index, _right_index)) in observations {
    //         let kf = map.get_keyframe(kf_id).unwrap();
    //         if kf.full_kf_info.origin_map_id != current_map_id {
    //             continue
    //         }


    //         match sensor.frame() {
    //             FrameSensor::Stereo => {
    //                 todo!("Stereo");
    //                 // if *left_index != -1 && kf.features.get_mv_right(*left_index as usize).unwrap() >= 0.0 {
    //                 //     // Stereo observation
    //                 //     // This is still the left observation, but because it is stereo it needs to be 

    //                 //     // const cv::KeyPoint &kpUn = pKFi->mvKeysUn[leftIndex];
    //                 //     // Eigen::Matrix<double,3,1> obs;
    //                 //     // const float kp_ur = pKFi->mvuRight[get<0>(mit->second)];
    //                 //     // obs << kpUn.pt().x, kpUn.pt().y, kp_ur;

    //                 //     // g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

    //                 //     // e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
    //                 //     // e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
    //                 //     // e->setMeasurement(obs);
    //                 //     // const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave()];
    //                 //     // Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
    //                 //     // e->setInformation(Info);

    //                 //     // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    //                 //     // e->setRobustKernel(rk);
    //                 //     // rk->setDelta(thHuberStereo);

    //                 //     // e->fx = pKFi->fx;
    //                 //     // e->fy = pKFi->fy;
    //                 //     // e->cx = pKFi->cx;
    //                 //     // e->cy = pKFi->cy;
    //                 //     // e->bf = pKFi->mbf;

    //                 //     // optimizer.addEdge(e);
    //                 //     // vpEdgesStereo.push_back(e);

    //                 //     // nEdges++;
    //                 // } else {
    //                 //     warn!("Local bundle adjustment, stereo observation... Pretty sure this line shouldn't be hit.");
    //                 // }

    //                 // // if(pKFi->mpCamera2){
    //                 // //     int rightIndex = get<1>(mit->second);

    //                 // //     if(rightIndex != -1 ){
    //                 // //         rightIndex -= pKFi->NLeft;

    //                 // //         Eigen::Matrix<double,2,1> obs;
    //                 // //         cv::KeyPoint kp = pKFi->mvKeysRight[rightIndex];
    //                 // //         obs << kp.pt().x, kp.pt().y;

    //                 // //         ORB_SLAM3::EdgeSE3ProjectXYZToBody *e = new ORB_SLAM3::EdgeSE3ProjectXYZToBody();

    //                 // //         e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
    //                 // //         e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
    //                 // //         e->setMeasurement(obs);
    //                 // //         const float &invSigma2 = pKFi->mvInvLevelSigma2[kp.octave()];
    //                 // //         e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

    //                 // //         g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    //                 // //         e->setRobustKernel(rk);
    //                 // //         rk->setDelta(thHuberMono);

    //                 // //         Sophus::SE3f Trl = pKFi-> GetRelativePoseTrl();
    //                 // //         e->mTrl = g2o::SE3Quat(Trl.unit_quaternion().cast<double>(), Trl.translation().cast<double>());

    //                 // //         e->pCamera = pKFi->mpCamera2;

    //                 // //         optimizer.addEdge(e);
    //                 // //         vpEdgesBody.push_back(e);
    //                 // //         vpEdgeKFBody.push_back(pKFi);
    //                 // //         vpMapPointEdgeBody.push_back(pMP);

    //                 // //         nEdges++;
    //                 // //     }
    //                 // // }
    //             },
    //             _ => {
    //                 // Monocular observation
    //                 if *left_index != -1 && kf.features.get_mv_right(*left_index as usize).is_none() {
    //                     let (kp_un, _) = kf.features.get_keypoint(*left_index as usize);
    //                     let kf_vertex = *all_kf_vertices.get(&kf.id()).unwrap();
    //                     // debug!("Adding edge {} -> {}", vertex_id, kf_vertex);

    //                     optimizer.pin_mut().add_edge_monocular_binary(
    //                         false, vertex_id, kf_vertex,
    //                         kp_un.octave(), kp_un.pt().x, kp_un.pt().y,
    //                         INV_LEVEL_SIGMA2[kp_un.octave() as usize],
    //                         1 // set delta of robustkernelhuber to thHuberMono
    //                     );

    //                     // TODO (mvp): below code sets edge's camera to the frame's camera
    //                     // but are we sure we need it?
    //                     // edge->pCamera = pFrame->mpCamera;
    //                 } else {
    //                     warn!("Local bundle adjustment, monocular observation... Pretty sure this line shouldn't be hit.");
    //                 }
    //             }
    //         }
    //     }
    //     vertex_id += 1;
    // }

    // // Optimize
    // optimizer.pin_mut().optimize(10);

    // let mut map_edits = Vec::new();

    // // Check inlier observations
    // let mut to_discard = Vec::new();
    // for edge in optimizer.pin_mut().get_mut_xyz_onlypose_edges().iter() {
    //     if edge.inner.chi2() > 5.991 || !edge.inner.is_depth_positive() {
    //         to_discard.push(edge.mappoint_id);
    //     }
    // }
    // map_edits.push(MapWriteMsg::discard_many_mappoints(&to_discard));

    // for (_mp_id, _edge) in all_edges_body {
    //     todo!("Stereo");
    //     // ORB_SLAM3::EdgeSE3ProjectXYZToBody* e = vpEdgesBody[i];
    //     // MapPoint* pMP = vpMapPointEdgeBody[i];

    //     // if(pMP->isBad())
    //     //     continue;

    //     // if(e->chi2()>5.991 || !e->isDepthPositive())
    //     // {
    //     //     KeyFrame* pKFi = vpEdgeKFBody[i];
    //     //     vToErase.push_back(make_pair(pKFi,pMP));
    //     // }
    // }

    // for (_mp_id, _edge) in all_edges_stereo {
    //     todo!("Stereo");
    //     // g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
    //     // MapPoint* pMP = vpMapPointEdgeStereo[i];

    //     // if(pMP->isBad())
    //     //     continue;

    //     // if(e->chi2()>7.815 || !e->isDepthPositive())
    //     // {
    //     //     KeyFrame* pKFi = vpEdgeKFStereo[i];
    //     //     vToErase.push_back(make_pair(pKFi,pMP));
    //     // }
    // }

    // // Recover optimized data
    // //Keyframes
    // for (kf_id, vertex_id) in all_kf_vertices {
    //     let pose = optimizer.recover_optimized_frame_pose(vertex_id);
    //     map_edits.push(MapWriteMsg::edit_keyframe_pose(kf_id, pose.into()));
    // }

    // //Points
    // for (mp_id, vertex_id) in all_mp_vertices {
    //     let pose = optimizer.recover_optimized_mappoint_pose(vertex_id);
    //     map_edits.push(MapWriteMsg::edit_mappoint_pose(mp_id, pose.into()));

    // }
// 
    // return map_edits;
}


pub struct BAResult {
    pub optimized_mp_poses: HashMap::<Id, DVPose>,
    pub optimized_kf_poses: HashMap::<Id, DVPose>,
}