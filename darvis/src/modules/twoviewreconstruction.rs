use std::{ops::DivAssign, f32, collections::HashMap};
use rand::{Rng, SeedableRng, rngs::StdRng};
use nalgebra::{Matrix3x4, Matrix4, Vector4, Matrix3, Vector3};
use opencv::{core::*, prelude::Mat, types::{VectorOfPoint3f, VectorOfPoint2f}};
use cv_convert::{opencv::core::Point3f, TryFromCv};

use crate::{
    dvmap::{
        pose::Pose, map::Id
    },
    matrix::{DVVectorOfKeyPoint},
};


#[derive(Debug, Clone)]
pub struct TwoViewReconstruction {
    pub keypoints1: DVVectorOfKeyPoint,
    pub keypoints2: DVVectorOfKeyPoint,
    matches12: Vec<(usize, usize)>,
    matched1: Vec<bool>,

    sets: Vec<Vec<usize>>, // Ransac sets
    sigma : f32, 
    max_iterations: usize,
    sigma2: f32,
}

impl TwoViewReconstruction {
    pub fn default() -> TwoViewReconstruction {
        TwoViewReconstruction {
            keypoints1: DVVectorOfKeyPoint::empty(),
            keypoints2: DVVectorOfKeyPoint::empty(),
            matches12: Vec::new(),
            matched1: Vec::new(),
            sets: Vec::new(),
            sigma: 1.0,
            max_iterations: 200,
            sigma2: 1.0,
        }
    }

    pub fn reconstruct(
        &mut self, 
        v_keys1: &DVVectorOfKeyPoint, 
        v_keys2: &DVVectorOfKeyPoint,
        matches_hashmap: &HashMap<u32, Id>,
        t21: &mut Pose,
        v_p3_d: &mut opencv::types::VectorOfPoint3f,
        vb_triangulated: &mut Vec<bool>
    ) -> bool {
        self.keypoints1.clear();
        self.keypoints2.clear();

        self.keypoints1 = v_keys1.clone();
        self.keypoints2 = v_keys2.clone();

        // Fill structures with current keypoints and matches with reference frame
        // Reference Frame: 1, Current Frame: 2
        self.matches12.clear();
        self.matches12.reserve(self.keypoints2.len() as usize);
        self.matched1.resize(self.keypoints1.len() as usize, false);

        for i in 0..self.matches12.len() as u32 {
            if matches_hashmap.contains_key(&i) {
                self.matches12.push((i as usize, *matches_hashmap.get(&i).unwrap() as usize));
                self.matched1[i as usize] = true;
            }
        }

        let N = self.matches12.len();

        // Indices for minimum set selection
        let mut v_all_indices = Vec::<usize>::new();
        v_all_indices.reserve(N);

        let mut v_available_indices = Vec::<usize>::new();

        for i in 0..N {
            v_available_indices.push(i);
        }

        // Generate sets of 8 points for each RANSAC iteration
        self.sets = vec![vec![0;8]; self.max_iterations as usize];

        let mut r = StdRng::seed_from_u64(0); // <- Here we set the seed

        // DUtils::Random::SeedRandOnce(0);

        for it in 0..self.max_iterations {
            v_available_indices = v_all_indices.clone();

            // Select a minimum set
            for j in 0..8
            {
                let randi = r.gen_range(0, v_available_indices.len()-1); //int randi = DUtils::Random::RandomInt(0,v_available_indices.size()-1);

                let idx = v_available_indices[randi];

                self.sets[it][j] = idx;

                v_available_indices[randi] = *v_available_indices.last().unwrap();
                v_available_indices.pop();
            }
        }

        // Launch threads to compute in parallel a fundamental matrix and a homography
        let vb_matches_inliers_h = Vec::<bool>::new();
        let vb_matches_inliers_f = Vec::<bool>::new();

        let pt_indx = opencv::types::VectorOfi32::new();
        let mut pn1 = VectorOfPoint2f::new();
        self.keypoints1.convert(&mut pn1, &pt_indx);
        // opencv::core::KeyPoint::convert(self.keypoints1.into(), &mut pn1, &pt_indx).unwrap();

        let mut pn2 = VectorOfPoint2f::new();
        self.keypoints2.convert(&mut pn2, &pt_indx);

        // opencv::core::KeyPoint::convert(self.keypoints2.into(), &mut pn2, &pt_indx).unwrap();

        let mut mask = Mat::default();

        let homography = opencv::calib3d::find_homography(&pn2, 
            &pn1,
            &mut mask,
            opencv::calib3d::RANSAC,
            3.0
        ).unwrap();

        let homography_21 : Matrix3<f32> = nalgebra::Matrix3::<f32>::try_from_cv(&homography).unwrap();
        let homography_12 = homography_21.try_inverse().unwrap();

        //let H12 = H.inv(opencv::core::DECOMP_LU).unwrap().to_mat().unwrap();

        let mut vbMatchesInliers = Vec::new();
        let SH = self.check_homography(&homography_21, &homography_12, &mut vbMatchesInliers, self.sigma);

        let F = opencv::calib3d::find_fundamental_mat(
            &pn2, &pn1, 
            opencv::calib3d::FM_RANSAC, 
            3.0, 0.99,  200, 
            &mut mask
        ).unwrap();

        let F21 = nalgebra::Matrix3::<f32>::try_from_cv(&F).unwrap();

        let SF = self.check_fundamental(&F21, &mut vbMatchesInliers, self.sigma);

        //TODO: Need to implement thread
        //thread threadH(&TwoViewReconstruction::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
        //thread threadF(&TwoViewReconstruction::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

        // // Wait until both threads have finished
        // threadH.join();
        // threadF.join();

        // Compute ratio of scores
        if SH+SF == 0.0  {
            return false;
        } 

        let RH = SH/(SH+SF);

        let minParallax = 1.0;

        let mK = Matrix3::<f32>::default();
        // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
        if RH > 0.50 {
            return self.reconstruct_from_homography(&vb_matches_inliers_h,&homography_21, &mK, t21, v_p3_d, vb_triangulated, minParallax,50);
        } else //if(pF_HF>0.6)
        {
            return self.reconstruct_from_fundamental(&vb_matches_inliers_f,&F21,&mK, t21,v_p3_d, vb_triangulated, minParallax,50);
        }
    }

    //float TwoViewReconstruction::CheckHomography(const Eigen::Matrix3f &H21, const Eigen::Matrix3f &H12, vector<bool> &vbMatchesInliers, float sigma)
    pub fn check_homography(
        &self, 
        H21: &Matrix3<f32>, H12: &Matrix3<f32>, 
        matches_inliers: &mut Vec<bool>, sigma: f32
    ) -> f32 {
        let N = self.matches12.len();

        let h11 = H21[(0,0)];
        let h12 = H21[(0,1)];
        let h13 = H21[(0,2)];
        let h21 = H21[(1,0)];
        let h22 = H21[(1,1)];
        let h23 = H21[(1,2)];
        let h31 = H21[(2,0)];
        let h32 = H21[(2,1)];
        let h33 = H21[(2,2)];

        let h11inv = H12[(0,0)];
        let h12inv = H12[(0,1)];
        let h13inv = H12[(0,2)];
        let h21inv = H12[(1,0)];
        let h22inv = H12[(1,1)];
        let h23inv = H12[(1,2)];
        let h31inv = H12[(2,0)];
        let h32inv = H12[(2,1)];
        let h33inv = H12[(2,2)];

        matches_inliers.resize(N, false);

        let mut score = 0.0f32;
        let th = 5.991;

        let inv_sigma_square = 1.0/(sigma*sigma);

        for i in 0..N {
            let mut bIn = true;

            let kp1 = &self.keypoints1.get(self.matches12[i].0).unwrap();
            let kp2 = &self.keypoints1.get(self.matches12[i].1).unwrap();

            let u1 = kp1.pt.x;
            let v1 = kp1.pt.y;
            let u2 = kp2.pt.x;
            let v2 = kp2.pt.y;

            // Reprojection error in first image
            // x2in1 = H12*x2

            let w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
            let u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
            let v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

            let square_dist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

            let chi_square1 = square_dist1*inv_sigma_square;

            if chi_square1>th {
                bIn = false;
            }  else {
                score += th - chi_square1;
            }

            // Reprojection error in second image
            // x1in2 = H21*x1

            let w1in2inv = 1.0/(h31*u1+h32*v1+h33);
            let u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
            let v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

            let square_dist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

            let chi_square2 = square_dist2*inv_sigma_square;

            if chi_square2>th {
                bIn = false;
            } else {
                score += th - chi_square2;
            }


            if bIn {
                matches_inliers[i]=true;
            } else {
                matches_inliers[i]=false;
            }
        }

        return score;
    }

    //float TwoViewReconstruction::CheckFundamental(const Eigen::Matrix3f &F21, vector<bool> &vbMatchesInliers, float sigma)
    pub fn check_fundamental(
        &self, 
        F21: &Matrix3<f32>, matches_inliers: &mut Vec<bool>, sigma: f32
    ) -> f32 {
        let N = self.matches12.len();

        let f11 = F21[(0,0)];
        let f12 = F21[(0,1)];
        let f13 = F21[(0,2)];
        let f21 = F21[(1,0)];
        let f22 = F21[(1,1)];
        let f23 = F21[(1,2)];
        let f31 = F21[(2,0)];
        let f32 = F21[(2,1)];
        let f33 = F21[(2,2)];

        matches_inliers.resize(N, false);

        let mut score = 0.0f32;

        let th = 3.841f32;
        let th_score = 5.991;

        let inv_sigma_square = 1.0/(sigma*sigma);

        for i in 0..N {
            let mut bIn = true;

            let kp1 = &self.keypoints1.get(self.matches12[i].0).unwrap();
            let kp2 = &self.keypoints1.get(self.matches12[i].1).unwrap();

            let u1 = kp1.pt.x;
            let v1 = kp1.pt.y;
            let u2 = kp2.pt.x;
            let v2 = kp2.pt.y;

            // Reprojection error in second image
            // l2=F21x1=(a2,b2,c2)

            let a2 = f11*u1+f12*v1+f13;
            let b2 = f21*u1+f22*v1+f23;
            let c2 = f31*u1+f32*v1+f33;

            let num2 = a2*u2+b2*v2+c2;

            let square_dist1 = num2*num2/(a2*a2+b2*b2);

            let chi_square1 = square_dist1*inv_sigma_square;

            if chi_square1>th{
                bIn = false;
            } else {
                score += th_score - chi_square1;
            }

            // Reprojection error in second image
            // l1 =x2tF21=(a1,b1,c1)

            let a1 = f11*u2+f21*v2+f31;
            let b1 = f12*u2+f22*v2+f32;
            let c1 = f13*u2+f23*v2+f33;

            let num1 = a1*u1+b1*v1+c1;

            let square_dist2 = num1*num1/(a1*a1+b1*b1);

            let chi_square2 = square_dist2*inv_sigma_square;

            if chi_square2>th {
                bIn = false;
            } else {
                score += th_score - chi_square2;
            }


            if bIn {
                matches_inliers[i]=true;
            } else {
                matches_inliers[i]=false;
            }

        }

        return score;
    }

    pub fn reconstruct_from_homography(
        &self,
        matches_inliers: &Vec<bool>, 
        H21: &Matrix3<f32>, 
        K: &Matrix3<f32>, 
        T21: &mut Pose, 
        vP3D: &mut VectorOfPoint3f, 
        triangulated: &mut Vec<bool>,
        min_parallax: f32 , 
        min_triangulated: i32
    ) -> bool {
        let mut N=0;

        for i in 0..matches_inliers.len() {
            if matches_inliers[i] {
                N+=1;
            }
        }

        // We recover 8 motion hypotheses using the method of Faugeras et al.
        // Motion and structure from motion in a piecewise planar environment.
        // International Journal of Pattern Recognition and Artificial Intelligence, 1988
        let invK = K.try_inverse().unwrap();
        let A = invK * H21 * K;

        let svd = nalgebra::linalg::SVD::new(A, true, true);

        let U = svd.u.unwrap();
        let Vt = svd.v_t.unwrap();
        let w = svd.singular_values;

        let V = Vt.transpose();

        let s = U.determinant()* Vt.determinant();
        let d1 = w[0];
        let d2 = w[1];
        let d3 = w[2];

        if d1/d2<1.00001 || d2/d3<1.00001 {
            return false;
        }

        let mut vr = Vec::<Matrix3<f32>>::new();
        let mut vt = Vec::<Vector3<f32>>::new();
        let mut vn = Vec::<Vector3<f32>>::new();

        vr.reserve(8);
        vt.reserve(8);
        vn.reserve(8);

        let aux1 = f32::sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
        let aux3 = f32::sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
        let x1 = vec![aux1,aux1,-aux1,-aux1];
        let x3 = vec![aux3,-aux3,aux3,-aux3];

        let aux_stheta = f32::sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

        let ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
        let stheta = vec![aux_stheta, -aux_stheta, -aux_stheta, aux_stheta];

        for i in 0..4 {
            let mut Rp = Matrix3::<f32>::zeros(); 
            Rp[(0,0)] = ctheta;
            Rp[(0,2)] = -stheta[i];
            Rp[(1,1)] = 1.0;
            Rp[(2,0)] = stheta[i];
            Rp[(2,2)] = ctheta;

            let R = s*U*Rp*Vt;
            vr.push(R);

            let mut tp = Vector3::<f32>::new(x1[i], 0.0, -x3[i]);
            tp *= d1 - d3;

            let t = U * tp;
            vt.push(t / t.norm());

            let np = Vector3::<f32>::new(x1[i], 0.0, x3[i]);

            let mut n = V * np;
            if n[2] < 0.0 {
                n = -n;
            }

            vn.push(n);
        }

        let aux_sphi = f32::sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

        let cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
        let sphi = vec![aux_sphi, -aux_sphi, -aux_sphi, aux_sphi];

        for i in 0..4 {
            let mut Rp = Matrix3::<f32>::zeros(); 

            Rp[(0,0)] = cphi;
            Rp[(0,2)] = sphi[i];
            Rp[(1,1)] = -1.0;
            Rp[(2,0)] = sphi[i];
            Rp[(2,2)] = -cphi;

            let R = s*U*Rp*Vt;
            vr.push(R);

            let mut tp = Vector3::<f32>::new(x1[i], 0.0, x3[i]);
            tp *= d1+d3;

            let t = U * tp;
            vt.push(t / t.norm());

            let np= Vector3::<f32>::new(x1[i], 0.0, x3[i]);

            let mut n = V * np;
            if n[2] < 0.0 {
                n = -n;
            }
            vn.push(n);
        }

        let mut best_good = 0;
        let mut second_best_good = 0;
        let mut best_solution_idx = -1i64;
        let mut  best_parallax = -1.0;
        let mut best_P3D = VectorOfPoint3f::new();
        let mut best_triangulated = Vec::<bool>::new();

        // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
        // We reconstruct all hypotheses and check in terms of triangulated points and parallax
        for i in 0..8 {
            let mut parallaxi: f32 = 0.0;
            let mut vP3Di = VectorOfPoint3f::new();
            let mut vbTriangulatedi = Vec::<bool>::new();

            let nGood = self.CheckRT(&vr[i],&vt[i],&self.keypoints1 ,&self.keypoints2,&self.matches12,matches_inliers,K,&mut vP3Di, 4.0*self.sigma2, &mut vbTriangulatedi, &mut parallaxi);

            if nGood>best_good {
                second_best_good = best_good;
                best_good = nGood;
                best_solution_idx = i as i64;
                best_parallax = parallaxi;
                best_P3D = vP3Di;
                best_triangulated = vbTriangulatedi;
            } else if nGood>second_best_good  {
                second_best_good = nGood;
            }
        }

        if second_best_good<(0.75*best_good as f32) as i32 && best_parallax>=min_parallax && best_good>min_triangulated && best_good>(0.9*N as f32) as i32  {
            *T21 = Pose::new(
                &nalgebra::convert(vt[best_solution_idx as usize]), &nalgebra::convert(vr[best_solution_idx as usize])
            );
            *triangulated = best_triangulated;
            return true;
        }

        return false;
    }

    pub fn reconstruct_from_fundamental(
        &self, 
        vbMatchesInliers: &Vec<bool>,
        F21: &Matrix3<f32>,
        K: &Matrix3<f32>,
        T21: &mut Pose,
        vP3D: &mut VectorOfPoint3f,
        vbTriangulated: &mut Vec<bool>,
        minParallax: f32,
        minTriangulated: i32
    ) -> bool {
        let mut N=0;

        for i in 0..vbMatchesInliers.len() {
            if vbMatchesInliers[i] {
                N+=1;
            }
        }

        // Compute Essential Matrix from Fundamental Matrix
        let E21 = K.transpose() * F21 * K;

        let (mut R1, mut R2) = (Matrix3::<f32>::identity(), Matrix3::<f32>::identity());
        let mut t = Vector3::<f32>::zeros();

        // Recover the 4 motion hypotheses
        self.DecomposeE(&E21, &mut R1, &mut R2, &mut t);

        let t1 = t;
        let t2 = -t;

        // Reconstruct with the 4 hyphoteses and check
        let (mut vP3D1, mut vP3D2, mut vP3D3, mut vP3D4) = ( VectorOfPoint3f::new(), VectorOfPoint3f::new(), VectorOfPoint3f::new(), VectorOfPoint3f::new());
        let (mut vbTriangulated1, mut vbTriangulated2, mut vbTriangulated3, mut vbTriangulated4) = (Vec::<bool>::new(), Vec::<bool>::new(), Vec::<bool>::new(), Vec::<bool>::new());
        let (mut parallax1, mut parallax2, mut parallax3, mut parallax4) = (0.0, 0.0, 0.0, 0.0);

        let nGood1 = self.CheckRT(&R1,&t1,&self.keypoints1,&self.keypoints2,&self.matches12,vbMatchesInliers,K, &mut vP3D1, 4.0*self.sigma2, &mut vbTriangulated1, &mut parallax1);
        let nGood2 = self.CheckRT(&R2,&t1,&self.keypoints1,&self.keypoints2,&self.matches12,vbMatchesInliers,K, &mut vP3D2, 4.0*self.sigma2, &mut vbTriangulated2, &mut parallax2);
        let nGood3 = self.CheckRT(&R1,&t2,&self.keypoints1,&self.keypoints2,&self.matches12,vbMatchesInliers,K, &mut vP3D3, 4.0*self.sigma2, &mut vbTriangulated3, &mut parallax3);
        let nGood4 = self.CheckRT(&R2,&t2,&self.keypoints1,&self.keypoints2,&self.matches12,vbMatchesInliers,K, &mut vP3D4, 4.0*self.sigma2, &mut vbTriangulated4, &mut parallax4);

        let maxGood = i32::max(nGood1,i32::max(nGood2,i32::max(nGood3,nGood4)));

        let nMinGood = i32::max((0.9*N as f32) as i32,minTriangulated);

        let mut nsimilar = 0;
        if nGood1>(0.7*maxGood as f32) as i32 {
            nsimilar+=1;
        }

        if nGood2>(0.7*maxGood as f32) as i32 {
            nsimilar+=1;
        }
        if nGood3>(0.7*maxGood as f32) as i32 {
            nsimilar+=1;
        }
        if nGood4>(0.7*maxGood as f32) as i32 {
            nsimilar+=1;
        }

        // If there is not a clear winner or not enough triangulated points reject initialization
        if maxGood<nMinGood || nsimilar>1 {
            return false;
        }

        // If best reconstruction has enough parallax initialize
        if maxGood==nGood1 {
            if parallax1>minParallax {
                *vP3D = vP3D1.clone();
                *vbTriangulated = vbTriangulated1;

                *T21 = Pose::new(&nalgebra::convert(t1), &nalgebra::convert(R1)); //Sophus::SE3f(R1, t1);
                return true;
            }
        } else if maxGood==nGood2 && parallax2>minParallax {
            *vP3D = vP3D2.clone();
            *vbTriangulated = vbTriangulated2;

            *T21 = Pose::new(&nalgebra::convert(t1), &nalgebra::convert(R2)); //Sophus::SE3f(R1, t1);
            return true;
        } else if maxGood==nGood3 {
            if parallax3>minParallax {
                *vP3D = vP3D3.clone();
                *vbTriangulated = vbTriangulated3;

                *T21 = Pose::new(&nalgebra::convert(t2), &nalgebra::convert(R1)); //Sophus::SE3f(R1, t1);
                return true;
            }

        } else if maxGood==nGood4 {
            if parallax4>minParallax {
                *vP3D = vP3D4.clone();
                *vbTriangulated = vbTriangulated4;

                *T21 = Pose::new(&nalgebra::convert(t2), &nalgebra::convert(R2)); //Sophus::SE3f(R1, t1);
                return true;
            }
        }

        return false;
    }

    pub fn CheckRT(
        &self, 
        R: &Matrix3<f32>, 
        t: &Vector3<f32>,
        vKeys1: &DVVectorOfKeyPoint,
        v_keys2: &DVVectorOfKeyPoint,
        vMatches12: &Vec<(usize,usize)>, 
        vbMatchesInliers: &Vec<bool>,
        K: &Matrix3<f32>, 
        vP3D_cv: &mut opencv::types::VectorOfPoint3f,
        th2: f32,
        vbGood : &mut Vec<bool>,
        parallax: &mut f32
    ) -> i32 {
        // Calibration parameters
        let fx = K[(0,0)];
        let fy = K[(1,1)];
        let cx = K[(0,2)];
        let cy = K[(1,2)];

        let mut vbGood = vec![false; vKeys1.len() as usize];

        let mut  vP3D = Vec::<Point3f>::new();
        vP3D.reserve(vKeys1.len() as usize);

        let mut vCosParallax = Vec::<f32>::new();
        vCosParallax.reserve(vKeys1.len() as usize);

        // Camera 1 Projection Matrix K[I|0]
        let mut P1 = Matrix3x4::<f32>::zeros();
        //P1.block<3,3>(0,0) = K; // TODO (low priority) try find api to assign 3x3 to 3x4 matrix
        P1[(0,0)] =  K[(0,0)];
        P1[(0,1)] =  K[(0,1)];
        P1[(0,2)] =  K[(0,2)];
        P1[(1,0)] =  K[(1,0)];
        P1[(1,1)] =  K[(1,1)];
        P1[(1,2)] =  K[(1,2)];
        P1[(2,0)] =  K[(2,0)];
        P1[(2,1)] =  K[(2,1)];
        P1[(2,2)] =  K[(2,2)];

        let mut O1= Vector3::<f32>::zeros();//Eigen::Vector3f O1;

        // Camera 2 Projection Matrix K[R|t]
        let mut P2 = Matrix3x4::identity();
        //P2.block<3,3>(0,0) = R;
        P2[(0,0)] =  R[(0,0)];
        P2[(0,1)] =  R[(0,1)];
        P2[(0,2)] =  R[(0,2)];
        P2[(1,0)] =  R[(1,0)];
        P2[(1,1)] =  R[(1,1)];
        P2[(1,2)] =  R[(1,2)];
        P2[(2,0)] =  R[(2,0)];
        P2[(2,1)] =  R[(2,1)];
        P2[(2,2)] =  R[(2,2)];

        //P2.block<3,1>(0,3) = t;
        P2[(0,3)] =  t[(0)];
        P2[(1,3)] =  t[(1)];
        P2[(2,3)] =  t[(2)];
        P2 = K * P2;

        let O2 = -R.transpose() * t;

        let mut nGood=0;

        for i in 0..vMatches12.len() {
            if vbMatchesInliers[i]== false {
                continue;
            }

            let kp1 = vKeys1.get(vMatches12[i].0).unwrap();
            let kp2 = v_keys2.get(vMatches12[i].1).unwrap();
            // const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
            // const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];

            let mut p3dC1 = Vector3::<f32>::zeros();

            let x_p1 = Vector3::<f32>::new(kp1.pt.x, kp1.pt.y, 1.0);
            let x_p2 = Vector3::<f32>::new(kp2.pt.x, kp2.pt.y, 1.0);

            self.Triangulate(&x_p1, &x_p2, &P1, &P2, &mut p3dC1);


            if p3dC1[0].is_infinite() || p3dC1[1].is_infinite() || p3dC1[2].is_infinite() {
                vbGood[vMatches12[i].0]=false;
                continue;
            }

            // Check parallax
            let normal1 = p3dC1 - O1;
            let dist1 = normal1.norm();

            let normal2 = p3dC1 - O2;
            let dist2 = normal2.norm();

            let cosParallax = normal1.dot(&normal2) / (dist1*dist2);

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)            
            if p3dC1[2] <=0.0 && cosParallax< 0.99998 {
                continue;
            }

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            let p3dC2 = R * p3dC1 + t;

            if p3dC2[2] <=0.0 && cosParallax< 0.99998 {
                continue;
            }

            // Check reprojection error in first image
            let invZ1 = 1.0/p3dC1[2];
            let im1x = fx*p3dC1[0]*invZ1+cx;
            let im1y = fy*p3dC1[1]*invZ1+cy;

            let squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

            if squareError1>th2 {
                continue;
            }

            // Check reprojection error in second image
            let invZ2 = 1.0/p3dC2[2];
            let im2x = fx*p3dC2[0]*invZ2+cx;
            let im2y = fy*p3dC2[1]*invZ2+cy;

            let squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

            if squareError2>th2 {
                continue;
            }

            vCosParallax.push(cosParallax);
            vP3D[vMatches12[i].0] = Point3f::new(p3dC1[0], p3dC1[1], p3dC1[2]);
            nGood+=1;

            if cosParallax<0.99998 {
                vbGood[vMatches12[i].0]=true;
            }

        }

        if nGood>0 {
            vCosParallax.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let idx = i32::min(50,(vCosParallax.len()-1) as i32);
            *parallax = f32::acos(vCosParallax[idx as usize])*180.0/CV_PI as f32;
        } else {
            *parallax=0.0;
        }

        return nGood;
    }

    pub fn Triangulate(
        &self,
        x_c1 : &Vector3<f32>,
        x_c2 : &Vector3<f32>,
        Tc1w : &Matrix3x4<f32>,
        Tc2w : &Matrix3x4<f32>,
        x3D: &mut Vector3<f32>
    ) -> bool {
        let mut A = Matrix4::<f32>::identity();

        A[(0,0)] = x_c1[(0)] * Tc1w[(2,0)] - Tc1w[(0,0)];
        A[(0,1)] = x_c1[(0)] * Tc1w[(2,1)] - Tc1w[(0,1)];
        A[(0,2)] = x_c1[(0)] * Tc1w[(2,2)] - Tc1w[(0,2)];
        A[(0,3)] = x_c1[(0)] * Tc1w[(2,3)] - Tc1w[(0,3)];

        A[(1,0)] = x_c1[(1)] * Tc1w[(2,0)] - Tc1w[(1,0)];
        A[(1,1)] = x_c1[(1)] * Tc1w[(2,1)] - Tc1w[(1,1)];
        A[(1,2)] = x_c1[(1)] * Tc1w[(2,2)] - Tc1w[(1,2)];
        A[(1,3)] = x_c1[(1)] * Tc1w[(2,3)] - Tc1w[(1,3)];

        A[(2,0)] = x_c1[(0)] * Tc1w[(2,0)] - Tc1w[(0,0)];
        A[(2,1)] = x_c1[(0)] * Tc1w[(2,1)] - Tc1w[(0,1)];
        A[(2,2)] = x_c1[(0)] * Tc1w[(2,2)] - Tc1w[(0,2)];
        A[(2,3)] = x_c1[(0)] * Tc1w[(2,3)] - Tc1w[(0,3)];

        A[(3,0)] = x_c1[(1)] * Tc1w[(2,0)] - Tc1w[(1,0)];
        A[(3,1)] = x_c1[(1)] * Tc1w[(2,1)] - Tc1w[(1,1)];
        A[(3,2)] = x_c1[(1)] * Tc1w[(2,2)] - Tc1w[(1,2)];
        A[(3,3)] = x_c1[(1)] * Tc1w[(2,3)] - Tc1w[(1,3)];

        // Eigen::Matrix4f A;
        // A.block<1,4>(0,0) = x_c1(0) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(0,0);
        // A.block<1,4>(1,0) = x_c1(1) * Tc1w.block<1,4>(2,0) - Tc1w.block<1,4>(1,0);
        // A.block<1,4>(2,0) = x_c2(0) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(0,0);
        // A.block<1,4>(3,0) = x_c2(1) * Tc2w.block<1,4>(2,0) - Tc2w.block<1,4>(1,0);

        let svd = nalgebra::linalg::SVD::new(A, false, true);

        let v_t_h = svd.v_t.unwrap();
        let x3Dh = Vector4::<f32>::new(v_t_h[(0,3)], v_t_h[(1,3)], v_t_h[(2,3)], v_t_h[(3,3)]);

        // Eigen::JacobiSVD<Eigen::Matrix4f> svd(A, Eigen::ComputeFullV);    
        // Eigen::Vector4f x3Dh = svd.matrixV().col(3);

        if x3Dh[3] ==0.0 {
            return false;
        }

        // Euclidean coordinates
        x3D[0] = x3Dh[0]/x3Dh[3];
        x3D[1] = x3Dh[1]/x3Dh[3];
        x3D[2] = x3Dh[2]/x3Dh[3];

        return true;
    }

    pub fn DecomposeE(&self, E: &Matrix3<f32>, R1: &mut Matrix3<f32>, R2: &mut Matrix3<f32>, t: &mut Vector3<f32>) {
        let svd = nalgebra::linalg::SVD::new(*E, true, true);
        let U = svd.u.unwrap();
        let Vt = svd.v_t.unwrap();

        // Eigen::JacobiSVD<Eigen::Matrix3f> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
        // Eigen::Matrix3f U = svd.matrixU();
        // Eigen::Matrix3f Vt = svd.matrixV().transpose();

        //t = U.col(2);
        t[0]= U[(0,2)];
        t[1]= U[(1,2)];
        t[2]= U[(2,2)];

        t.div_assign(t.norm());
        //t = t.div_assign(rhs) / t.norm();

        let mut W = Matrix3::<f32>::zeros();
        W[(0,1)] = -1.0;
        W[(1,0)] = 1.0;
        W[(2,2)] = 1.0;

        *R1 = U * W * Vt;
        if R1.determinant() < 0.0 {
            R1.neg_mut();
            //*R1 = *-R1;
        }

        *R2 = U * W.transpose() * Vt;
        if R2.determinant() < 0.0 {
            R2.neg_mut();
            //R2 = -R2;
        }
    }
}
