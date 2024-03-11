// Everything in this function uses opencv::Mat objects instead of nalgebra or DVMatrix (which wraps nalgebra) objects

use std::{cmp::{max, min}, collections::HashMap};


use log::error;
use opencv::{core::{no_array, norm, MatExprResult, Range, Scalar, CV_64F, NORM_L2}, hub_prelude::{KeyPointTraitConst, MatExprTraitConst, MatTraitConst}};
use rand::Rng;

use crate::{map::{map::Id, pose::{DVRotation, DVTranslation}}, modules::optimizer::LEVEL_SIGMA2, registered_actors::CAMERA_MODULE, MapLock};
use opencv::prelude::*;

pub struct Sim3Solver {
    x_3d_c1: Vec<Option<Mat>>, // mvX3Dc1
    x_3d_c2: Vec<Option<Mat>>, // mvX3Dc2
    max_error1: Vec<f64>, // mvnMaxError1
    max_error2: Vec<f64>, // mvnMaxError2
    indices1: Vec<u32>,

    num_features: i32, // N, mN1 ... same number

    current_estimation: Sim3Estimation,
    current_ransac_state: RansacState,

    // Indices for random selection
    all_indices: Vec<u32>,

    // Projections
    p1_im1: Vec<Option<(f64, f64)>>, // mvP1im1
    p2_im2: Vec<Option<(f64, f64)>>, // mvP2im2

    // RANSAC
    ransac_min_inliers: i32, // mRansacMinInliers
    ransac_max_its: i32, // mRansacMaxIts

    fix_scale: bool,

    // NOT USED?
    // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
    // thresh: f32, // mTh
    // sigma2: f32 // mSigma2
    // std::vector<MapPoint*> mvpMatches12;
    // std::vector<MapPoint*> mvpMapPoints1;
    // std::vector<MapPoint*> mvpMapPoints2;

    // KeyFrames and matches
    // kf1_id: Id,
    // kf2_id: Id,

}

impl Sim3Solver {
    pub fn new(
        map: &MapLock,
        kf1_id: Id, kf2_id: Id, matches: &HashMap<u32, i32>, fix_scale: bool,
        probability: f32, min_inliers: i32, max_iterations:i32
    ) -> Self {
        let map_lock = map.read();
        let kf1 = map_lock.keyframes.get(&kf1_id).unwrap();
        let kf2 = map_lock.keyframes.get(&kf2_id).unwrap();

        let keyframe_mp1 = kf1.get_mp_matches(); // vpKeyFrameMP1
        let num_features = keyframe_mp1.len() as i32; // N

        let kf1_rot: Mat = (&kf1.pose.get_rotation()).into(); // Rcw1
        let kf1_trans: Mat = (&kf1.pose.get_translation()).into(); // tcw1
        let kf2_rot: Mat = (&kf2.pose.get_rotation()).into(); // Rcw2
        let kf2_trans: Mat = (&kf2.pose.get_translation()).into(); // tcw2

        let mut max_error1 = vec![]; // mvnMaxError1
        let mut max_error2 = vec![]; // mvnMaxError2
        let mut mappoints1 = vec![-1; num_features as usize]; // mvpMapPoints1
        let mut mappoints2 = vec![-1; num_features as usize]; // mvpMapPoints2
        let mut indices1 = vec![0; num_features as usize]; // mvnIndices1 // todo maybe this shouldn't be 0
        let mut x_3d_c1 = vec![None; num_features as usize]; // mvX3Dc1
        let mut x_3d_c2 = vec![None; num_features as usize]; // mvX3Dc2
        let mut all_indices = vec![]; // mvAllIndices

        // Sofiya... sometimes keyframe_mp1 does not have any of the indexes in matches
        // Matches found in search_by_bow_kf
        println!("Matches: {:?}", matches);
        println!("KF matches: {:?}", keyframe_mp1);
        print!("Skipping...");
        for (index, mp2_id) in matches {
            let mp1_id = match keyframe_mp1[*index as usize] {
                Some((mp1_id, _)) => mp1_id,
                None => {
                    print!("s1 {}, ", index);
                    panic!("This should not happen?")
                }
            };

            let (kf1_left_idx, _) = map.read().mappoints.get(&mp1_id).unwrap().get_index_in_keyframe(kf1_id);
            let (kf2_left_idx, _) = map.read().mappoints.get(&mp2_id).unwrap().get_index_in_keyframe(kf2_id);

            if kf1_left_idx < 0 || kf2_left_idx < 0 {
                print!("s2 {}, ", index);
                continue;
            }

            let (kp1, _) = kf1.features.get_keypoint(kf1_left_idx as usize);
            let (kp2, _) = kf2.features.get_keypoint(kf2_left_idx as usize);

            let sigma_square1 = LEVEL_SIGMA2[kp1.octave() as usize];
            let sigma_square2 = LEVEL_SIGMA2[kp2.octave() as usize];

            max_error1.push((9.210 * sigma_square1) as f64);
            max_error2.push((9.210 * sigma_square2) as f64);

            mappoints1.push(mp1_id);
            mappoints2.push(*mp2_id);
            indices1.push(*index);

            let x_3d_1w: Mat = (&map.read().mappoints.get(&mp1_id).unwrap().position).into();
            x_3d_c1.push(Some(res_to_mat(&kf1_rot * x_3d_1w + &kf1_trans).unwrap()));

            let x_3d_2w: Mat = (&map.read().mappoints.get(&mp2_id).unwrap().position).into();
            x_3d_c2[*index as usize] = Some(res_to_mat(&kf2_rot * x_3d_2w + &kf2_trans).unwrap());

            all_indices.push(*index);
        }

        let p1_im1 = from_camera_to_image(&x_3d_c1);
        let p2_im2 = from_camera_to_image(&x_3d_c2);

        let inliers_i = vec![false; num_features as usize]; // mvbInliersi

        // Adjust Parameters according to number of correspondences
        let epsilon = probability / (num_features as f32);

        // Set RANSAC iterations according to probability, epsilon, and max iterations
        let iterations = match min_inliers == num_features as i32 {
            true => 1,
            false => ((1.0 - probability).ln() / (1.0 - epsilon.powf(3.0)).ln()).ceil() as i32
        }; // todo double check that c++ log() is the same as ln() here

        let ransac_max_its = max(1, min(iterations, max_iterations));

        Self {
            ransac_max_its,
            ransac_min_inliers: min_inliers,
            // kf1_id,
            // kf2_id,
            num_features,
            x_3d_c1,
            x_3d_c2,
            current_estimation: Sim3Estimation{
                r_12_i: Mat::default(),
                t_12_i: Mat::default(),
                s_12_i: 1.0,
                T12_i: Mat::default(),
                T21_i: Mat::default(),
                inliers_i,
                inliers_count: 0,
            },
            current_ransac_state: RansacState{
                num_iterations: 0,
                best_inliers: vec![],
                best_inliers_count: 0,
                best_T12: Mat::default(),
                best_rotation: Mat::default(),
                best_translation: Mat::default(),
                best_scale: 1.0,
            },
            all_indices,
            p1_im1,
            p2_im2,
            max_error1,
            max_error2,
            indices1,
            fix_scale
        }
    }

    pub fn iterate(&mut self, num_iterations: i32) -> Result<(bool, Option<(Mat, Vec<bool>, i32)>), opencv::Error> {
        // Eigen::Matrix4f Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers, bool &bConverge)
        // Returns bool &bNoMore, vector<bool> &vbInliers, bool &bConverge

        let mut no_more = false; // bNoMore
        let mut vec_inliers = vec![false; self.num_features as usize]; // vbInliers
        let num_inliers; // nInliers

        if self.num_features < self.ransac_min_inliers {
            no_more = true;
            return Ok((no_more, None));
        }

        let p_3d_c1_i = Mat::new_rows_cols_with_default(3,3, CV_64F, Scalar::all(0.0))?; // P3Dc1i
        let p_3d_c2_i = Mat::new_rows_cols_with_default(3,3, CV_64F, Scalar::all(0.0))?; // P3Dc2i

        let mut current_iterations = 0;
        while self.current_ransac_state.num_iterations < self.ransac_max_its && current_iterations < num_iterations {
            self.current_ransac_state.num_iterations += 1;
            current_iterations += 1;

            // Get min set of points
            for _ in 0..3 {
                let randi = rand::thread_rng().gen_range(0..self.all_indices.len()-1);

                let idx = self.all_indices[randi] as usize;
                match self.x_3d_c1[idx] {
                    None => {
                        error!("x_3d_c1: {:?}", self.x_3d_c1);
                        panic!("x_3d_c1[{}] is None", idx)
                    },
                    Some(_) => {
                        self.x_3d_c1[idx].as_ref().unwrap().copy_to(
                            &mut p_3d_c1_i.row(idx as i32)?
                        )?;
                    }
                };

                match self.x_3d_c2[idx] {
                    None => {
                        error!("x_3d_c1: {:?}", self.x_3d_c1);
                        panic!("x_3d_c2[{}] is None", idx)
                    },
                    Some(_) => {
                        self.x_3d_c2[idx].as_ref().unwrap().copy_to(
                            &mut p_3d_c2_i.row(idx as i32)?
                        )?;
                    }
                };

                self.all_indices[randi] = self.all_indices.pop().unwrap(); // todo is this equivalent to .back() followed by pop_back() ?
            }

            self.compute_sim3(&p_3d_c1_i, &p_3d_c2_i)?; 

            self.check_inliers()?;

            // todo get rid of these clones?
            if self.current_estimation.inliers_count > self.current_ransac_state.best_inliers_count {
                self.current_ransac_state.best_inliers = self.current_estimation.inliers_i.clone();
                self.current_ransac_state.best_inliers_count = self.current_estimation.inliers_count;
                self.current_ransac_state.best_T12 = self.current_estimation.T12_i.clone();
                self.current_ransac_state.best_rotation = self.current_estimation.r_12_i.clone();
                self.current_ransac_state.best_translation = self.current_estimation.t_12_i.clone();
                self.current_ransac_state.best_scale = self.current_estimation.s_12_i;

                if self.current_estimation.inliers_count > self.ransac_min_inliers {
                    num_inliers = self.current_estimation.inliers_count;

                    for i in 0..self.num_features {
                        if self.current_estimation.inliers_i[i as usize] {
                            vec_inliers[self.indices1[i as usize] as usize] = true;
                        }
                    }
                    return Ok((no_more, Some((self.current_ransac_state.best_T12.clone(), vec_inliers, num_inliers))));
                }
            }
        }

        if self.current_ransac_state.num_iterations >= self.ransac_max_its {
            no_more = true;
        }
        return Ok((no_more, None));
    }

    fn find(&mut self) -> Result<(bool, Option<(Mat, Vec<bool>, i32)>), opencv::Error> {
        // Eigen::Matrix4f Sim3Solver::find(vector<bool> &vbInliers12, int &nInliers)
        self.iterate(self.ransac_max_its)
    }

    fn compute_centroid(&self, p: &Mat) -> Result<(opencv::core::Mat, opencv::core::Mat), opencv::Error> {
        // void Sim3Solver::ComputeCentroid(Eigen::Matrix3f &P, Eigen::Matrix3f &Pr, Eigen::Vector3f &C)
        let mut c = Mat::default();
        opencv::core::reduce(&p, &mut c, 0, opencv::core::REDUCE_SUM, CV_64F)?;

        let division = 1.0 / p.cols() as f64;
        c = c.mul(&division, 1.)?.to_mat()?;

        let pr = Mat::default();
        for i in 0..p.cols() {
            let temp = res_to_mat(p.col(i)? - &c)?;
            temp.copy_to(&mut pr.col(i)?)?;
        }

        return Ok((pr, c));
    }

    fn compute_sim3(&mut self, p1: &Mat, p2: &Mat) -> Result<(), opencv::Error> {
        // void Sim3Solver::ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2)

        // Custom implementation of:
        // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

        // Step 1: Centroid and relative coordinates

        // pr1 = Relative coordinates to centroid (set 1)
        // O1 = Centroid of P1
        let (pr1, o1) = self.compute_centroid(&p1)?;
        // pr2 = Relative coordinates to centroid (set 2)
        // O2 = Centroid of P2
        let (pr2, o2) = self.compute_centroid(&p2)?;

        // Step 2: Compute M matrix
        let m_mat = res_to_mat(&pr2 * pr1.t()?)?;

        // Step 3: Compute N matrix
        let n_mat = {
            let n11 = m_mat.at_2d::<f64>(0,0)? + m_mat.at_2d::<f64>(1,1)? + m_mat.at_2d::<f64>(2,2)?;
            let n12 = m_mat.at_2d::<f64>(1,2)? - m_mat.at_2d::<f64>(2,1)?;
            let n13 = m_mat.at_2d::<f64>(2,0)? - m_mat.at_2d::<f64>(0,2)?;
            let n14 = m_mat.at_2d::<f64>(0,1)? - m_mat.at_2d::<f64>(1,0)?;
            let n22 = m_mat.at_2d::<f64>(0,0)? - m_mat.at_2d::<f64>(1,1)? - m_mat.at_2d::<f64>(2,2)?;
            let n23 = m_mat.at_2d::<f64>(0,1)? + m_mat.at_2d::<f64>(1,0)?;
            let n24 = m_mat.at_2d::<f64>(2,0)? + m_mat.at_2d::<f64>(0,2)?;
            let n33 = -m_mat.at_2d::<f64>(0,0)? + m_mat.at_2d::<f64>(1,1)? - m_mat.at_2d::<f64>(2,2)?;
            let n34 = m_mat.at_2d::<f64>(1,2)? + m_mat.at_2d::<f64>(2,1)?;
            let n44 = -m_mat.at_2d::<f64>(0,0)? - m_mat.at_2d::<f64>(1,1)? + m_mat.at_2d::<f64>(2,2)?;

            Mat::from_slice_2d(&[
                [n11, n12, n13, n14],
                [n12, n22, n23, n24],
                [n13, n23, n33, n34],
                [n14, n24, n34, n44],
            ])?
        };

        // Step 4: Eigenvector of the highest eigenvalue

        let mut eval = Mat::default();
        let mut evec = Mat::default();
        opencv::core::eigen(&n_mat, &mut eval, &mut evec)?; //evec[0] is the quaternion of the desired rotation

        let mut vec = Mat::default();
        evec.row(0)?.col_range(& opencv::core::Range::new(1,2)?)?.copy_to(&mut vec)?; //extract imaginary part of the quaternion (sin*axis)

        // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        let ang = norm(&vec, NORM_L2, &Mat::default())?.atan2(* evec.at_2d::<f64>(0,0)?);
        let mut vec = {
            let top = res_to_mat(2.0 * ang * &vec)?;
            let bottom = norm(&vec, NORM_L2, &Mat::default())?;
            res_to_mat(top / bottom)? //Angle-axis representation. quaternion angle is the half
        };

        let mut r_12_i = Mat::default();

        vec = res_to_mat(2.0 * ang * &vec / norm(&vec, NORM_L2, &Mat::default())?)?;
        opencv::calib3d::rodrigues(&vec, &mut r_12_i, &mut no_array()).unwrap(); // computes the rotation matrix from angle-axis

        // Step 5: Rotate set 2
        let p3 = res_to_mat(&r_12_i * pr2)?;

        // Step 6: Scale
        if !self.fix_scale {
            let norm = pr1.dot(&p3)?;
            let mut aux_p3 = p3.clone();
            opencv::core::pow(&p3, 2.0, &mut aux_p3)?;
            let mut den = 0.0;
            for i in 0..aux_p3.rows() {
                for j in 0..aux_p3.cols() {
                    den += aux_p3.at_2d::<f64>(i,j)?;
                }
            }
            self.current_estimation.s_12_i = norm / den;
        } else {
            self.current_estimation.s_12_i = 1.0;
        }

        // Step 7: Translation
        self.current_estimation.t_12_i = {
            let t_12_i = o1 - self.current_estimation.s_12_i * &r_12_i * o2;
            res_to_mat(t_12_i)?
        };

        // Step 8: Transformation

        // Step 8.1 T12
        self.current_estimation.T12_i = Mat::new_rows_cols_with_default(4,4, CV_64F, Scalar::all(0.0)).unwrap();
        let s_r = res_to_mat(self.current_estimation.s_12_i * &r_12_i)?;
        s_r.copy_to(
            &mut self.current_estimation.T12_i
            .row_range(&Range::new(0,3)?)?
            .col_range(&Range::new(0,3)?)?
        )?;
        self.current_estimation.t_12_i.copy_to(
            &mut self.current_estimation.T12_i.
            row_range(&Range::new(0,3)?)?
            .col(0)?
        )?;

        // Step 8.2 T21
        let s_r_inv = res_to_mat((1.0 / self.current_estimation.s_12_i) * r_12_i.t()?)?;

        s_r_inv.copy_to(
            &mut self.current_estimation.T21_i
            .row_range(&Range::new(0,3)?)?
            .col_range(&Range::new(0,3)?)?
        )?;

        let neg_s_r_inv = s_r_inv.mul(&-1., 1.)?.to_mat()?;
        let tinv = res_to_mat(neg_s_r_inv * &self.current_estimation.t_12_i)?;
        tinv.copy_to(
            &mut self.current_estimation.T21_i
            .row_range(&Range::new(0,3)?)?
            .col(3)?
        )?;

        Ok(())
    }

    fn check_inliers(&mut self) -> Result<(), opencv::Error> {
        // void Sim3Solver::CheckInliers()
        let v_p2_im1 = self.project(&self.x_3d_c2, &self.current_estimation.T12_i)?; // vP2im1
        let v_p1_im2 = self.project(&self.x_3d_c1, &self.current_estimation.T21_i)?; // vP1im2

        self.current_estimation.inliers_count = 0;

        for i in 0..self.p1_im1.len() {
            if self.p1_im1[i].is_none() || self.p2_im2[i].is_none() {
                continue;
            }
            let dist1 = Mat::from_slice_2d(&[
                [self.p1_im1[i].unwrap().0 - v_p2_im1[i].0],
                [self.p1_im1[i].unwrap().1 - v_p2_im1[i].1]
            ])?;
            let dist2 = Mat::from_slice_2d(&[
                [v_p1_im2[i].0 - self.p2_im2[i].unwrap().0],
                [v_p1_im2[i].1 - self.p2_im2[i].unwrap().1]
            ])?;

            let err1 = dist1.dot(&dist1)?;
            let err2 = dist2.dot(&dist2)?;

            if err1 < self.max_error1[i] && err2 < self.max_error2[i] {
                self.current_estimation.inliers_i[i] = true;
                self.current_estimation.inliers_count += 1;
            } else {
                self.current_estimation.inliers_i[i] = false;
            }
        }

        Ok(())
    }

    fn project(&self, p3d_w: &Vec<Option<Mat>>, tcw: &Mat) -> Result<Vec<(f64, f64)>, opencv::Error> {
        // void Sim3Solver::Project(const vector<Eigen::Vector3f> &vP3Dw, vector<Eigen::Vector2f> &vP2D, Eigen::Matrix4f Tcw, GeometricCamera* pCamera)

        let rcw = tcw.row_range(& Range::new(0,3)?)?.col_range(& Range::new(0,3)?)?;
        let tcw = tcw.row_range(& Range::new(0,3)?)?.col(3)?;
        let mut p2d = vec![(0.0,0.0); p3d_w.len()];
        for pose_in_vec in p3d_w {
            if pose_in_vec.is_none() {
                continue;
            }
            let p3d_c = res_to_mat(&rcw * pose_in_vec.as_ref().unwrap() + &tcw)?;
            let invz = 1.0 / p3d_c.at::<f64>(2)?;
            let x = p3d_c.at::<f64>(0)? * invz;
            let y = p3d_c.at::<f64>(1)? * invz;

            p2d.push((
                CAMERA_MODULE.fx * x + CAMERA_MODULE.cx,
                CAMERA_MODULE.fy * y + CAMERA_MODULE.cy
            ));
        }

        return Ok(p2d);
    }

    pub fn get_estimates(&mut self) -> (DVRotation, DVTranslation, f64) {
        (
            self.current_estimation.r_12_i.clone().into(),
            (&self.current_estimation.t_12_i.clone()).into(),
            self.current_estimation.s_12_i
        )
    }
}

fn from_camera_to_image(p3dc: &Vec<Option<Mat>>) -> Vec<Option<(f64, f64)>> {
    // void Sim3Solver::FromCameraToImage(const vector<Eigen::Vector3f> &vP3Dc, vector<Eigen::Vector2f> &vP2D, GeometricCamera* pCamera)
    let mut p2d = vec![None; p3dc.len()];
    for i in 0..p3dc.len() {
        if p3dc[i].is_none() {
            continue;
        }
        let pt_2d = CAMERA_MODULE.project(p3dc[i].as_ref().unwrap().into());
        p2d.push(Some(pt_2d));
    }
    return p2d;
}

fn res_to_mat<T: opencv::prelude::MatExprTraitConst>(res: MatExprResult<T>) -> Result<Mat, opencv::Error> {
    // Can't be an into() because both input and output are not defined in this crate
    res.into_result()?.to_mat()
}

struct Sim3Estimation{
    r_12_i: Mat, // mR12i
    t_12_i: Mat, // mt12i
    s_12_i: f64, // ms12i
    T12_i: Mat, // mT12i
    T21_i: Mat, // mT21i
    inliers_i: Vec<bool>, // mvbInliersi
    inliers_count: i32, // mnInliersi
}


struct RansacState {
    // Current Ransac State
    num_iterations: i32, // mnIterations
    best_inliers: Vec<bool>, // mvbBestInliers
    best_inliers_count: i32, // mnBestInliers
    best_T12: Mat, // mBestT12
    best_rotation: Mat, // mBestRotation
    best_translation: Mat, // mBestTranslation
    best_scale: f64, // mBestScale
}