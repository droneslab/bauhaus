// Everything in this function uses opencv::Mat objects instead of nalgebra or DVMatrix (which wraps nalgebra) objects

use std::{cmp::{max, min}, collections::HashMap};


use log::warn;
use opencv::{core::{no_array, norm, MatExprResult, Range, Scalar, CV_64F, NORM_L2}, hub_prelude::{KeyPointTraitConst, MatExprTraitConst, MatTraitConst}};
use rand::Rng;

use crate::{map::{map::Id, pose::Sim3}, modules::optimizer::LEVEL_SIGMA2, registered_actors::CAMERA_MODULE, MapLock};
use opencv::prelude::*;

pub struct Sim3Solver {
    n: usize, // N, total mappoints that could have matches
    n1: usize, // mN1, total mappoints that could have matches
    all_indices: Vec<usize>, // mvAllIndices
    indices1: Vec<u32>, // mvnIndices1

    x_3d_c1: Vec<Mat>, // mvX3Dc1
    x_3d_c2: Vec<Mat>, // mvX3Dc2
    max_error1: Vec<f64>, // mvnMaxError1
    max_error2: Vec<f64>, // mvnMaxError2

    // Projections
    p1_im1: Vec<(f64, f64)>, // mvP1im1
    p2_im2: Vec<(f64, f64)>, // mvP2im2

    current_estimation: Sim3Estimation,
    current_ransac_state: RansacState,

    // RANSAC
    ransac_min_inliers: i32, // mRansacMinInliers
    ransac_max_its: i32, // mRansacMaxIts

    fix_scale: bool,
}

impl Sim3Solver {
    pub fn new(
        map: &MapLock,
        kf1_id: Id, kf2_id: Id, matches: &HashMap<usize, Id>, fix_scale: bool,
        mut keyframe_matched_mp: HashMap<usize, Id>
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let map_lock = map.read();
        let kf1 = map_lock.keyframes.get(&kf1_id).unwrap();
        let kf2 = map_lock.keyframes.get(&kf2_id).unwrap();

        let keyframe_mp1 = kf1.get_mp_matches(); // vpKeyFrameMP1

        let kf1_rot: Mat = (&kf1.pose.get_rotation()).into(); // Rcw1
        let kf1_trans: Mat = (&kf1.pose.get_translation()).into(); // tcw1
        let kf2_rot: Mat = (&kf2.pose.get_rotation()).into(); // Rcw2
        let kf2_trans: Mat = (&kf2.pose.get_translation()).into(); // tcw2

        let mut all_indices = vec![]; // mvAllIndices
        let mut indices1 = vec![]; // mvnIndices1
        // let mut max_error1 = vec![None; total_indices_available]; // mvnMaxError1
        // let mut max_error2 = vec![None; total_indices_available]; // mvnMaxError2
        // let mut x_3d_c1 = vec![None; total_indices_available]; // mvX3Dc1
        // let mut x_3d_c2 = vec![None; total_indices_available]; // mvX3Dc2
        // let mut p1_im1 = vec![None; total_indices_available]; // mvP1im1
        // let mut p2_im2 = vec![None; total_indices_available]; // mvP2im2
        let mut max_error1 = vec![]; // mvnMaxError1
        let mut max_error2 = vec![]; // mvnMaxError2
        let mut x_3d_c1 = vec![]; // mvX3Dc1
        let mut x_3d_c2 = vec![]; // mvX3Dc2
        let mut p1_im1 = vec![]; // mvP1im1
        let mut p2_im2 = vec![]; // mvP2im2

        let mut i = 0;
        // println!("sim3solver NEW: ");
        for (index, mp2_id) in matches {
            let mp1_id = match keyframe_mp1[*index as usize] {
                Some((mp1_id, _)) => mp1_id,
                None => {
                    warn!("KF {} does not have MP match at index {}. This should not happen?", kf1_id, index);
                    continue
                }
            };

            let (kf1_left_idx, _) = map_lock.mappoints.get(&mp1_id).unwrap().get_index_in_keyframe(kf1_id);
            let (kf2_left_idx, _) = map_lock.mappoints.get(&mp2_id).unwrap().get_index_in_keyframe(kf2_id);

            if kf1_left_idx < 0 || kf2_left_idx < 0 {
                continue;
            }

            let (kp1, _) = kf1.features.get_keypoint(kf1_left_idx as usize);
            let (kp2, _) = kf2.features.get_keypoint(kf2_left_idx as usize);

            let sigma_square1 = LEVEL_SIGMA2[kp1.octave() as usize];
            let sigma_square2 = LEVEL_SIGMA2[kp2.octave() as usize];

            // p1_im1 and p2_im2 are constructed in this function in orbslam2:
            // void Sim3Solver::FromCameraToImage(const vector<Eigen::Vector3f> &vP3Dc, vector<Eigen::Vector2f> &vP2D, GeometricCamera* pCamera)

            let x_3d_1w: Mat = (&map_lock.mappoints.get(&mp1_id).unwrap().position).into();
            // println!("x_3d_1w: {:?}, ", x_3d_1w.data_typed::<f64>()?);

            let this_x3dc1 = res_to_mat(&kf1_rot * x_3d_1w + &kf1_trans).unwrap();
            // println!("kf1_rot without into: {:?}", kf1.pose.get_rotation());
            // println!("kf1_rot: {:?}, ", kf1_rot.data_typed::<f64>()?);
            // println!("kf1_trans: {:?}, ", kf1_trans.data_typed::<f64>()?);
            // println!("...this_x3dc1: {:?}, ", this_x3dc1.data_typed::<f64>()?);

            p1_im1.push(CAMERA_MODULE.project((&this_x3dc1).into()));
            x_3d_c1.push(this_x3dc1);

            let x_3d_2w: Mat = (&map_lock.mappoints.get(&mp2_id).unwrap().position).into();
            // println!("x_3d_2w: {:?}, ", x_3d_2w.data_typed::<f64>()?);

            let this_x3dc2 = res_to_mat(&kf2_rot * x_3d_2w + &kf2_trans).unwrap();
            // println!("kf2_rot: {:?}, ", kf2_rot.data_typed::<f64>()?);
            // println!("kf2_trans: {:?}, ", kf2_trans.data_typed::<f64>()?);
            // println!("...this_x3dc2: {:?}, ", this_x3dc2.data_typed::<f64>()?);
            // println!("===");

            p2_im2.push(CAMERA_MODULE.project((&this_x3dc2).into()));
            x_3d_c2.push(this_x3dc2);

            max_error1.push(9.210 * sigma_square1 as f64);
            max_error2.push(9.210 * sigma_square2 as f64);

            indices1.push(*index as u32);
            all_indices.push(i);
            i += 1;
        }

        let mut sim3_solver = Self {
            ransac_max_its: 300,
            ransac_min_inliers: 6,
            n: all_indices.len(),
            n1: keyframe_mp1.len(),
            x_3d_c1,
            x_3d_c2,
            max_error1,
            max_error2,
            p1_im1,
            p2_im2,
            indices1,
            current_estimation: Sim3Estimation{
                r_12_i: Mat::default(),
                t_12_i: Mat::default(),
                s_12_i: 1.0,
                T12_i: Mat::new_rows_cols_with_default(4,4, CV_64F, Scalar::all(0.0))?,
                T21_i: Mat::new_rows_cols_with_default(4,4, CV_64F, Scalar::all(0.0))?,
                inliers_i: vec![false; all_indices.len()], // mvbInliersi,
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
            fix_scale
        };

        sim3_solver.set_ransac_parameters(0.99, 6, 300);

        Ok(sim3_solver)
    }

    pub fn set_ransac_parameters(&mut self, probability: f32, min_inliers: i32, max_iterations: i32) {
        // Adjust Parameters according to number of correspondences
        let epsilon = (min_inliers as f32) / (self.n as f32);

        // Set RANSAC iterations according to probability, epsilon, and max iterations
        let iterations = match min_inliers == self.n as i32 {
            true => 1,
            false => ((1.0 - probability).ln() / (1.0 - epsilon.powf(3.0)).ln()).ceil() as i32
        };

        self.ransac_max_its = max(1, min(iterations, max_iterations));
    }

    pub fn iterate(&mut self, num_iterations: i32) -> Result<(bool, Option<(Vec<bool>, i32)>), opencv::Error> {
        // Eigen::Matrix4f Sim3Solver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers, bool &bConverge)
        // Returns bool &bNoMore, vector<bool> &vbInliers, bool &bConverge

        let mut no_more = false; // bNoMore
        let mut vec_inliers = vec![false; self.n1]; // vbInliers
        let num_inliers; // nInliers

        if (self.n as i32) < self.ransac_min_inliers {
            no_more = true;
            return Ok((no_more, None));
        }

        let p_3d_c1_i = Mat::new_rows_cols_with_default(3,3, CV_64F, Scalar::all(0.0))?; // P3Dc1i
        let p_3d_c2_i = Mat::new_rows_cols_with_default(3,3, CV_64F, Scalar::all(0.0))?; // P3Dc2i

        let mut current_iterations = 0;
        while self.current_ransac_state.num_iterations < self.ransac_max_its && current_iterations < num_iterations {
            self.current_ransac_state.num_iterations += 1;
            current_iterations += 1;

            let mut available_indices: Vec<usize> = self.all_indices.clone();

            // Get min set of points
            for i in 0..3 {
                let randi = rand::thread_rng().gen_range(0..available_indices.len()-1);
                // let randi = dvos3binding::ffi::RandomInt(0, available_indices.len() as i32 - 2) as usize;
                let idx = available_indices[randi] as usize;
                self.x_3d_c1[idx].copy_to(
                    &mut p_3d_c1_i.col(i)?
                )?;

                self.x_3d_c2[idx].copy_to(
                    &mut p_3d_c2_i.col(i)?
                )?;

                available_indices[randi] = available_indices.pop().unwrap();
            }
            
            self.compute_sim3(&p_3d_c1_i, &p_3d_c2_i)?;

            self.check_inliers()?;

            // println!("Sim3 inliers: {}", self.current_estimation.inliers_count);

            if self.current_estimation.inliers_count > self.current_ransac_state.best_inliers_count {
                self.current_ransac_state.best_inliers = self.current_estimation.inliers_i.clone();
                self.current_ransac_state.best_inliers_count = self.current_estimation.inliers_count;
                self.current_ransac_state.best_T12 = self.current_estimation.T12_i.clone();
                self.current_ransac_state.best_rotation = self.current_estimation.r_12_i.clone();
                self.current_ransac_state.best_translation = self.current_estimation.t_12_i.clone();
                self.current_ransac_state.best_scale = self.current_estimation.s_12_i;

                if self.current_estimation.inliers_count > self.ransac_min_inliers {
                    num_inliers = self.current_estimation.inliers_count;

                    for i in 0..self.n {
                        if self.current_estimation.inliers_i[i as usize] {
                            vec_inliers[self.indices1[i] as usize] = true;
                        }
                    }
                    //self.current_ransac_state.best_T12.clone(), 
                    return Ok((no_more, Some((vec_inliers, num_inliers))));
                }
            }
        }

        if self.current_ransac_state.num_iterations >= self.ransac_max_its {
            no_more = true;
        }
        return Ok((no_more, None));
    }

    fn compute_centroid(&self, p: &Mat) -> Result<(opencv::core::Mat, opencv::core::Mat), opencv::Error> {
        // void Sim3Solver::ComputeCentroid(Eigen::Matrix3f &P, Eigen::Matrix3f &Pr, Eigen::Vector3f &C)
        let mut c = Mat::default();
        opencv::core::reduce(&p, &mut c, 1, opencv::core::REDUCE_SUM, CV_64F)?;

        let division = 1.0 / p.cols() as f64;
        c = c.mul(&division, 1.)?.to_mat()?;

        let pr = Mat::new_rows_cols_with_default(p.rows(), p.cols(), CV_64F, Scalar::all(0.0))?;
        for i in 0..p.cols() {
            let temp = res_to_mat(p.col(i)?.clone() - &c)?;
            temp.copy_to(&mut pr.col(i)?)?;
        }

        return Ok((pr, c));
    }

    fn compute_sim3(&mut self, p1: &Mat, p2: &Mat) -> Result<(), opencv::Error> {
        // void Sim3Solver::ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2)

        // Custom implementation of:
        // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

        // Step 1: Centroid and relative coordinates

        // println!("ComputeSim3");
        // println!("P1: {:?}", &p1.data_typed::<f64>()?);
        // println!("P2: {:?}", &p2.data_typed::<f64>()?);

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

        let mut vec = Mat::new_rows_cols_with_default(0, 3, CV_64F, Scalar::all(0.0))?;
        evec.row(0)?.col_range(& opencv::core::Range::new(1,4)?)?.copy_to(&mut vec)?; //extract imaginary part of the quaternion (sin*axis)

        // Rotation angle. sin is the norm of the imaginary part, cos is the real part
        let ang = norm(&vec, NORM_L2, &Mat::default())?.atan2(* evec.at_2d::<f64>(0,0)?);
        vec = {
            let top = res_to_mat(2.0 * ang * &vec)?;
            let bottom = norm(&vec, NORM_L2, &Mat::default())?;
            res_to_mat(top / bottom)? //Angle-axis representation. quaternion angle is the half
        };

        self.current_estimation.r_12_i = Mat::new_rows_cols_with_default(0, 3, CV_64F, Scalar::all(0.0))?;

        vec = res_to_mat(2.0 * ang * &vec / norm(&vec, NORM_L2, &Mat::default())?)?;
        opencv::calib3d::rodrigues(&vec, &mut self.current_estimation.r_12_i, &mut no_array())?; // computes the rotation matrix from angle-axis

        // Step 5: Rotate set 2
        let p3 = res_to_mat(&self.current_estimation.r_12_i * pr2)?;

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
            let t_12_i = o1 - self.current_estimation.s_12_i * &self.current_estimation.r_12_i * o2;
            res_to_mat(t_12_i)?
        };

        // Step 8: Transformation

        // Step 8.1 T12
        self.current_estimation.T12_i = Mat::new_rows_cols_with_default(4,4, CV_64F, Scalar::all(0.0))?;
        let s_r = res_to_mat(self.current_estimation.s_12_i * &self.current_estimation.r_12_i)?;

        s_r.copy_to(
            &mut self.current_estimation.T12_i
            .row_range(&Range::new(0,3)?)?
            .col_range(&Range::new(0,3)?)?
        )?;

        self.current_estimation.t_12_i.copy_to(
            &mut self.current_estimation.T12_i
            .col(0)?
        )?;

        // Step 8.2 T21
        let s_r_inv = res_to_mat((1.0 / self.current_estimation.s_12_i) * self.current_estimation.r_12_i.t()?)?;

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
            let dist1 = Mat::from_slice_2d(&[
                [self.p1_im1[i].0 - v_p2_im1[i].0],
                [self.p1_im1[i].1 - v_p2_im1[i].1]
            ])?;
            let dist2 = Mat::from_slice_2d(&[
                [v_p1_im2[i].0 - self.p2_im2[i].0],
                [v_p1_im2[i].1 - self.p2_im2[i].1]
            ])?;

            let err1 = dist1.dot(&dist1)?;
            let err2 = dist2.dot(&dist2)?;

            // println!("Error: {} < {}, {} < {}", err1, self.max_error1[i], err2, self.max_error2[i]);

            if err1 < self.max_error1[i] && err2 < self.max_error2[i] {
                self.current_estimation.inliers_i[i] = true;
                self.current_estimation.inliers_count += 1;
            } else {
                self.current_estimation.inliers_i[i] = false;
            }
        }

        Ok(())
    }

    fn project(&self, p3d_w: &Vec<Mat>, tcw: &Mat) -> Result<Vec<(f64, f64)>, opencv::Error> {
        // void Sim3Solver::Project(const vector<Eigen::Vector3f> &vP3Dw, vector<Eigen::Vector2f> &vP2D, Eigen::Matrix4f Tcw, GeometricCamera* pCamera)

        let rcw = tcw.row_range(& Range::new(0,3)?)?.col_range(& Range::new(0,3)?)?;
        let tcw = tcw.row_range(& Range::new(0,3)?)?.col(3)?;
        let mut p2d = vec![];
        for pose_in_vec in p3d_w {
            let p3d_c = res_to_mat(&rcw * pose_in_vec + &tcw)?;
            let invz = 1.0 / p3d_c.at::<f64>(2)?;
            let x = p3d_c.at::<f64>(0)? * invz;
            let y = p3d_c.at::<f64>(1)? * invz;

            p2d.push((
                CAMERA_MODULE.fx * x + CAMERA_MODULE.cx,
                CAMERA_MODULE.fy * y + CAMERA_MODULE.cy
            ));
        }

        Ok(p2d)
    }

    pub fn get_estimates(&mut self) -> Sim3 {
        Sim3::new(
            (&self.current_ransac_state.best_translation.clone()).into(),
            self.current_ransac_state.best_rotation.clone().into(),
            self.current_ransac_state.best_scale
        )
    }
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