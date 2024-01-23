use core::matrix::DVMatrix4;
use std::collections::{HashMap, HashSet};

use nalgebra::Matrix4;

use crate::{map::{keyframe::KeyFrame, map::Id, pose::{DVRotation, DVTranslation, Pose}}, modules::camera::CAMERA_MODULE, registered_actors::CAMERA};

pub struct Sim3Solver {
    probability: f32,
    min_inliers: i32,
    max_iterations: i32,

    // KeyFrames and matches
    kf_1_id: Id,
    kf_2_id: Id,

    // std::vector<Eigen::Vector3f> mvX3Dc1;
    // std::vector<Eigen::Vector3f> mvX3Dc2;
    // std::vector<MapPoint*> mvpMapPoints1;
    // std::vector<MapPoint*> mvpMapPoints2;
    // std::vector<MapPoint*> mvpMatches12;
    // std::vector<size_t> mvnIndices1;
    // std::vector<size_t> mvSigmaSquare1;
    // std::vector<size_t> mvSigmaSquare2;
    // std::vector<size_t> mvnMaxError1;
    // std::vector<size_t> mvnMaxError2;

    // int N;
    // int mN1;

    // Current Estimation
    R12_i: DVRotation, // mR12i
    t12_i: DVTranslation, // mt12i
    s12_i: f32, // ms12i
    T12_i: DVMatrix4<f32>, // mT12i
    T21_i: DVMatrix4<f32>, // mT21i
    inliers_i: Vec<bool>, // mvbInliersi
    inliers_count: i32, // mnInliersi

    // // Current Ransac State
    // int mnIterations;
    // std::vector<bool> mvbBestInliers;
    // int mnBestInliers;
    // Eigen::Matrix4f mBestT12;
    // Eigen::Matrix3f mBestRotation;
    // Eigen::Vector3f mBestTranslation;
    // float mBestScale;

    // // Scale is fixed to 1 in the stereo/RGBD case
    // bool mbFixScale;

    // // Indices for random selection
    // std::vector<size_t> mvAllIndices;

    // // Projections
    // std::vector<Eigen::Vector2f> mvP1im1;
    // std::vector<Eigen::Vector2f> mvP2im2;

    // RANSAC probability
    ransac_probability: f32, // mRansacProb

    // RANSAC min inliers
    ransac_min_inliers: i32, // mRansacMinInliers

    // RANSAC max iterations
    ransac_max_iterations: i32, //mRansacMaxIts

    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    thresh: f32, // mTh
    sigma2: f32 // mSigma2
}

impl Sim3Solver {
    pub fn new(
        kf_1: &KeyFrame, kf_2: &KeyFrame, matches: &HashMap<u32, i32>, fix_scale: bool,
        probability: f32, min_inliers: i32, max_iterations:i32
    ) -> Self {

        let keyframe_mp1 = kf_1.get_mp_matches(); // vpKeyFrameMP1

        // mN1 = vpMatched12.size();

        // mvpMapPoints1.reserve(mN1);
        // mvpMapPoints2.reserve(mN1);
        // mvpMatches12 = vpMatched12;
        // mvnIndices1.reserve(mN1);
        // mvX3Dc1.reserve(mN1);
        // mvX3Dc2.reserve(mN1);

        // cv::Mat Rcw1 = pKF1->GetRotation();
        // cv::Mat tcw1 = pKF1->GetTranslation();
        // cv::Mat Rcw2 = pKF2->GetRotation();
        // cv::Mat tcw2 = pKF2->GetTranslation();

        // mvAllIndices.reserve(mN1);

        // size_t idx=0;
        for (index, mp_id) in matches {
            // MapPoint* pMP1 = vpKeyFrameMP1[i1];
            // MapPoint* pMP2 = vpMatched12[i1];

            // if(!pMP1)
            //     continue;

            // if(pMP1->isBad() || pMP2->isBad())
            //     continue;

            // int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
            // int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

            // if(indexKF1<0 || indexKF2<0)
            //     continue;

            // const cv::KeyPoint &kp1 = pKF1->mvKeysUn[indexKF1];
            // const cv::KeyPoint &kp2 = pKF2->mvKeysUn[indexKF2];

            // const float sigmaSquare1 = pKF1->mvLevelSigma2[kp1.octave];
            // const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];

            // mvnMaxError1.push_back(9.210*sigmaSquare1);
            // mvnMaxError2.push_back(9.210*sigmaSquare2);

            // mvpMapPoints1.push_back(pMP1);
            // mvpMapPoints2.push_back(pMP2);
            // mvnIndices1.push_back(i1);

            // cv::Mat X3D1w = pMP1->GetWorldPos();
            // mvX3Dc1.push_back(Rcw1*X3D1w+tcw1);

            // cv::Mat X3D2w = pMP2->GetWorldPos();
            // mvX3Dc2.push_back(Rcw2*X3D2w+tcw2);

            // mvAllIndices.push_back(idx);
            // idx++;
        }

        // mK1 = pKF1->mK;
        // mK2 = pKF2->mK;

        // FromCameraToImage(mvX3Dc1,mvP1im1,mK1);
        // FromCameraToImage(mvX3Dc2,mvP2im2,mK2);


        // N = mvpMapPoints1.size(); // number of correspondences

        // mvbInliersi.resize(N);

        // // Adjust Parameters according to number of correspondences
        // float epsilon = (float)probability/N;

        // // Set RANSAC iterations according to probability, epsilon, and max iterations
        // int nIterations;

        // if(minInliers==N)
        //     nIterations=1;
        // else
        //     nIterations = ceil(log(1-mRansacProb)/log(1-pow(epsilon,3)));

        // maxIterations = max(1,min(nIterations,maxIterations));

        // mnIterations = 0;



        todo!("LOOP CLOSING")
    }

    pub fn iterate(&self, num_iterations: i32) -> (DVMatrix4<f32>, bool, Vec<bool>, i32) {
        // Eigen::Matrix4f Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers, bool &bConverge)
        // Return bool &bNoMore, vector<bool> &vbInliers, bool &bConverge


        // bNoMore = false;
        // vbInliers = vector<bool>(mN1,false);
        // nInliers=0;

        // if(N<mRansacMinInliers)
        // {
        //     bNoMore = true;
        //     return cv::Mat();
        // }

        // vector<size_t> vAvailableIndices;

        // cv::Mat P3Dc1i(3,3,CV_32F);
        // cv::Mat P3Dc2i(3,3,CV_32F);

        // int nCurrentIterations = 0;
        // while(mnIterations<mRansacMaxIts && nCurrentIterations<nIterations)
        // {
        //     nCurrentIterations++;
        //     mnIterations++;

        //     vAvailableIndices = mvAllIndices;

        //     // Get min set of points
        //     for(short i = 0; i < 3; ++i)
        //     {
        //         int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

        //         int idx = vAvailableIndices[randi];

        //         mvX3Dc1[idx].copyTo(P3Dc1i.col(i));
        //         mvX3Dc2[idx].copyTo(P3Dc2i.col(i));

        //         vAvailableIndices[randi] = vAvailableIndices.back();
        //         vAvailableIndices.pop_back();
        //     }

        //     ComputeSim3(P3Dc1i,P3Dc2i);

        //     CheckInliers();

        //     if(mnInliersi>=mnBestInliers)
        //     {
        //         mvbBestInliers = mvbInliersi;
        //         mnBestInliers = mnInliersi;
        //         mBestT12 = mT12i.clone();
        //         mBestRotation = mR12i.clone();
        //         mBestTranslation = mt12i.clone();
        //         mBestScale = ms12i;

        //         if(mnInliersi>mRansacMinInliers)
        //         {
        //             nInliers = mnInliersi;
        //             for(int i=0; i<N; i++)
        //                 if(mvbInliersi[i])
        //                     vbInliers[mvnIndices1[i]] = true;
        //             return mBestT12;
        //         }
        //     }
        // }

        // if(mnIterations>=mRansacMaxIts)
        //     bNoMore=true;

        // return cv::Mat();

        todo!("LOOP CLOSING");
    }

    fn find(&self) ->  (DVMatrix4<f32>, bool, Vec<bool>, i32) {
        // Eigen::Matrix4f Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
        self.iterate(self.max_iterations)
    }

    fn compute_centroid(&self) {
        // void Sim3Solver::ComputeCentroid(Eigen::Matrix3f &P, Eigen::Matrix3f &Pr, Eigen::Vector3f &C)

        // cv::reduce(P,C,1,CV_REDUCE_SUM);
        // C = C/P.cols;

        // for(int i=0; i<P.cols; i++)
        // {
        //     Pr.col(i)=P.col(i)-C;
        // }

    }

    fn compute_sim3(&self) {
        // void Sim3Solver::ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2)

        // Custom implementation of:
        // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

        // Step 1: Centroid and relative coordinates

        // cv::Mat Pr1(P1.size(),P1.type()); // Relative coordinates to centroid (set 1)
        // cv::Mat Pr2(P2.size(),P2.type()); // Relative coordinates to centroid (set 2)
        // cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
        // cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

        // ComputeCentroid(P1,Pr1,O1);
        // ComputeCentroid(P2,Pr2,O2);

        // // Step 2: Compute M matrix

        // cv::Mat M = Pr2*Pr1.t();

        // // Step 3: Compute N matrix

        // double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

        // cv::Mat N(4,4,P1.type());

        // N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
        // N12 = M.at<float>(1,2)-M.at<float>(2,1);
        // N13 = M.at<float>(2,0)-M.at<float>(0,2);
        // N14 = M.at<float>(0,1)-M.at<float>(1,0);
        // N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
        // N23 = M.at<float>(0,1)+M.at<float>(1,0);
        // N24 = M.at<float>(2,0)+M.at<float>(0,2);
        // N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
        // N34 = M.at<float>(1,2)+M.at<float>(2,1);
        // N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

        // N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
        //                             N12, N22, N23, N24,
        //                             N13, N23, N33, N34,
        //                             N14, N24, N34, N44);


        // // Step 4: Eigenvector of the highest eigenvalue

        // cv::Mat eval, evec;

        // cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

        // cv::Mat vec(1,3,evec.type());
        // (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

        // // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        // double ang=atan2(norm(vec),evec.at<float>(0,0));

        // vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half

        // mR12i.create(3,3,P1.type());

        // cv::Rodrigues(vec,mR12i); // computes the rotation matrix from angle-axis

        // // Step 5: Rotate set 2

        // cv::Mat P3 = mR12i*Pr2;

        // // Step 6: Scale

        // if(!mbFixScale)
        // {
        //     double nom = Pr1.dot(P3);
        //     cv::Mat aux_P3(P3.size(),P3.type());
        //     aux_P3=P3;
        //     cv::pow(P3,2,aux_P3);
        //     double den = 0;

        //     for(int i=0; i<aux_P3.rows; i++)
        //     {
        //         for(int j=0; j<aux_P3.cols; j++)
        //         {
        //             den+=aux_P3.at<float>(i,j);
        //         }
        //     }

        //     ms12i = nom/den;
        // }
        // else
        //     ms12i = 1.0f;

        // // Step 7: Translation

        // mt12i.create(1,3,P1.type());
        // mt12i = O1 - ms12i*mR12i*O2;

        // // Step 8: Transformation

        // // Step 8.1 T12
        // mT12i = cv::Mat::eye(4,4,P1.type());

        // cv::Mat sR = ms12i*mR12i;

        // sR.copyTo(mT12i.rowRange(0,3).colRange(0,3));
        // mt12i.copyTo(mT12i.rowRange(0,3).col(3));

        // // Step 8.2 T21

        // mT21i = cv::Mat::eye(4,4,P1.type());

        // cv::Mat sRinv = (1.0/ms12i)*mR12i.t();

        // sRinv.copyTo(mT21i.rowRange(0,3).colRange(0,3));
        // cv::Mat tinv = -sRinv*mt12i;
        // tinv.copyTo(mT21i.rowRange(0,3).col(3));

    }

    fn check_inliers(&self) {
        // void Sim3Solver::CheckInliers()

        // vector<cv::Mat> vP1im2, vP2im1;
        // Project(mvX3Dc2,vP2im1,mT12i,mK1);
        // Project(mvX3Dc1,vP1im2,mT21i,mK2);

        // mnInliersi=0;

        // for(size_t i=0; i<mvP1im1.size(); i++)
        // {
        //     cv::Mat dist1 = mvP1im1[i]-vP2im1[i];
        //     cv::Mat dist2 = vP1im2[i]-mvP2im2[i];

        //     const float err1 = dist1.dot(dist1);
        //     const float err2 = dist2.dot(dist2);

        //     if(err1<mvnMaxError1[i] && err2<mvnMaxError2[i])
        //     {
        //         mvbInliersi[i]=true;
        //         mnInliersi++;
        //     }
        //     else
        //         mvbInliersi[i]=false;
        // }

    }

    fn project(&self, p3d_w: &Vec<Matrix4<f32>>, pose: &Pose) -> Vec<Pose> {
        // void Sim3Solver::Project(const vector<Eigen::Vector3f> &vP3Dw, vector<Eigen::Vector2f> &vP2D, Eigen::Matrix4f Tcw, GeometricCamera* pCamera)
        // cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        // cv::Mat tcw = Tcw.rowRange(0,3).col(3);
        // const float &fx = K.at<float>(0,0);
        // const float &fy = K.at<float>(1,1);
        // const float &cx = K.at<float>(0,2);
        // const float &cy = K.at<float>(1,2);

        let rcw = pose.get_rotation();
        let tcw = pose.get_translation();
        let p2d = vec![0; p3d_w.len()];
        for pose in p3d_w {
            // let p3d_c = rcw * pose + tcw;
            // const float invz = 1/(P3Dc.at<float>(2));
            // const float x = P3Dc.at<float>(0)*invz;
            // const float y = P3Dc.at<float>(1)*invz;

            // vP2D.push_back((cv::Mat_<float>(2,1) << fx*x+cx, fy*y+cy));
        }

        todo!("LOOP CLOSING");
        // return p2d;
    }

    pub fn get_estimates(&self) -> (DVRotation, DVTranslation, f32) {
        return (self.R12_i.clone(), self.t12_i.clone(), self.s12_i);
    }
}