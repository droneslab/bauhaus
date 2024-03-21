// A lot of functions here are copied over from sim3solver.rs so that we don't have
// to make the full sim3solver object which relies on the map

#[cfg(test)]
mod sim3solver_tests {
    use core::matrix::DVVector3;
    use std::cmp::{max, min};
    use opencv::core::{no_array, norm, Mat, MatExprResult, MatExprTraitConst, MatTraitConst, MatTraitConstManual, Range, Scalar, CV_64F, NORM_L2};

    use crate::map::pose::{DVRotation, DVTranslation, Pose, Sim3};

    #[test]
    fn test_sim3_struct() {
        let sim3 = Sim3 {
            pose: Pose::new(
                nalgebra::Vector3::new(-0.289838, 0.0579359, 1.17659),
                nalgebra::Matrix3::new(
                    0.999724, 0.0119505, 0.0202439,
                    -0.0118632, 0.99992, -0.00443018,
                    -0.0202952, 0.0041888, 0.999785
                )
            ),
            scale: 1.0
        };
        let vector = DVVector3::new(nalgebra::Vector3::new(-0.448621, -0.163819, 0.27061));
        let map = * sim3.map(&vector);

        let expected_map = nalgebra::Vector3::new(-0.734815, -0.101746, 1.45556);
        println!("map: {:?}", map);
        assert_eq!(map, expected_map);
    }

    #[test]
    fn test_set_ransac_param() {
        fn set_ransac_params(min_inliers: i32, num_matches: i32) -> i32 {
            let probability: f32 = 0.99;
            let max_iterations = 300;
            let epsilon = (min_inliers as f32) / (num_matches as f32);
            let iterations = match min_inliers == num_matches as i32 {
                true => 1,
                false => ((1.0 - probability).ln() / (1.0 - epsilon.powf(3.0)).ln()).ceil() as i32
            };
            max(1, min(iterations, max_iterations))
        }

        assert_eq!(set_ransac_params(6, 58), 300);
        assert_eq!(set_ransac_params(20, 58), 110);
    }

    #[test]
    fn test_compute_centroid() {
        let p = Mat::from_slice_2d(&[
            &[-1.9746182, 0.5801363, 0.63902175],
            &[0.15127298, 0.10896853, 0.66591078],
            &[2.7354946, 1.382499, 6.5072355]
        ]).unwrap();
        let (pr, o) = compute_centroid(&p).unwrap();
        let expected_pr = Mat::from_slice_2d(&[
            &[-1.7227981, 0.83195639, 0.89084184],
            &[-0.15744445, -0.1997489, 0.35719335],
            &[-0.80624843, -2.1592441, 2.9654925]
        ]).unwrap();
        let expected_o = Mat::from_slice_2d(&[
            &[-0.25182006],
            &[0.30871743],
            &[3.541743]]).unwrap();

        assert!(compare_mats(&pr, &expected_pr));
        assert!(compare_mats(&o, &expected_o));
    }

    #[test]
    fn test_compute_sim3() {
        let p1 = Mat::from_slice_2d(&[
            &[0.6906426, 1.0464532, -0.18071842],
            &[-0.64418668, -0.029071447, 0.13930871],
            &[6.7210565, 2.4318123, 0.79204011]
        ]).unwrap();
        let p2 = Mat::from_slice_2d(&[
            &[0.29090601, 0.48953232, -0.089658424],
            &[-0.26681289, -0.015115698, 0.067958578],
            &[2.8357472, 1.1394876, 0.39604735]
        ]).unwrap();

        let estimate = compute_sim3(&p1, &p2, false).unwrap();

        let expected_r12i = Mat::from_slice_2d(&[
            &[0.99996179, -0.008439796, 0.0022765258],
            &[0.0084298803, 0.99995512, 0.0043306849],
            &[-0.0023129736, -0.0043113283, 0.99998802]
        ]).unwrap();
        let expected_t12i = Mat::from_slice_2d(&[
            &[-0.050236207],
            &[-0.02472719],
            &[-0.22514816]
        ]).unwrap();
        let expected_s12i = 2.42998;
        let expected_T12i = Mat::from_slice_2d(&[
            &[2.4298859, -0.020508524, 0.0055319089, -0.050236207],
            &[0.020484429, 2.4298697, 0.010523472, -0.02472719],
            &[-0.0056204763, -0.010476436, 2.4299495, -0.22514816],
            &[0.0, 0.0, 0.0, 1.0]
        ]).unwrap();
        let expected_T21i = Mat::from_slice_2d(&[
            &[0.41151053, 0.0034691172, -0.00095184939, 0.020544203],
            &[-0.0034731978, 0.41150779, -0.0017742248, 0.0096014868],
            &[0.00093685015, 0.0017821905, 0.41152135, 0.092744403],
            &[0.0, 0.0, 0.0, 1.0]
        ]).unwrap();


        assert!(compare_mats(&estimate.r_12_i, &expected_r12i));
        assert!(compare_mats(&estimate.t_12_i, &expected_t12i));
        assert!((estimate.s_12_i - expected_s12i).abs() < 0.0001);
        assert!(compare_mats(&estimate.T12_i, &expected_T12i));
        assert!(compare_mats(&estimate.T21_i, &expected_T21i));


    }

    #[test]
    fn test_project() {

    }

    fn compute_centroid(p: &Mat) -> Result<(opencv::core::Mat, opencv::core::Mat), opencv::Error> {
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

    fn compute_sim3(p1: &Mat, p2: &Mat, fix_scale: bool) -> Result<Sim3Estimation, opencv::Error> {
        // void Sim3Solver::ComputeSim3(Eigen::Matrix3f &P1, Eigen::Matrix3f &P2)

        let mut current_estimation = Sim3Estimation {
            r_12_i: Mat::default(),
            t_12_i: Mat::default(),
            s_12_i: 0.0,
            T12_i: Mat::new_rows_cols_with_default(4,4, CV_64F, Scalar::all(0.0))?,
            T21_i: Mat::new_rows_cols_with_default(4,4, CV_64F, Scalar::all(0.0))?,
            inliers_i: vec![],
            inliers_count: 0
        };

        // Custom implementation of:
        // Horn 1987, Closed-form solution of absolute orientataion using unit quaternions

        // Step 1: Centroid and relative coordinates

        // pr1 = Relative coordinates to centroid (set 1)
        // O1 = Centroid of P1
        let (pr1, o1) = compute_centroid(&p1)?;
        // pr2 = Relative coordinates to centroid (set 2)
        // O2 = Centroid of P2
        let (pr2, o2) = compute_centroid(&p2)?;

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

        current_estimation.r_12_i = Mat::new_rows_cols_with_default(0, 3, CV_64F, Scalar::all(0.0))?;

        vec = res_to_mat(2.0 * ang * &vec / norm(&vec, NORM_L2, &Mat::default())?)?;
        opencv::calib3d::rodrigues(&vec, &mut current_estimation.r_12_i, &mut no_array())?; // computes the rotation matrix from angle-axis

        // Step 5: Rotate set 2
        let p3 = res_to_mat(&current_estimation.r_12_i * pr2)?;

        // Step 6: Scale
        if !fix_scale {
            let norm = pr1.dot(&p3)?;
            let mut aux_p3 = p3.clone();
            opencv::core::pow(&p3, 2.0, &mut aux_p3)?;
            let mut den = 0.0;
            for i in 0..aux_p3.rows() {
                for j in 0..aux_p3.cols() {
                    den += aux_p3.at_2d::<f64>(i,j)?;
                }
            }
            current_estimation.s_12_i = norm / den;
        } else {
            current_estimation.s_12_i = 1.0;
        }

        // Step 7: Translation
        current_estimation.t_12_i = {
            let t_12_i = o1 - current_estimation.s_12_i * &current_estimation.r_12_i * o2;
            res_to_mat(t_12_i)?
        };

        // Step 8: Transformation

        // Step 8.1 T12
        current_estimation.T12_i = Mat::new_rows_cols_with_default(4,4, CV_64F, Scalar::all(0.0))?;
        let s_r = res_to_mat(current_estimation.s_12_i * &current_estimation.r_12_i)?;

        s_r.copy_to(
            &mut current_estimation.T12_i
            .row_range(&Range::new(0,3)?)?
            .col_range(&Range::new(0,3)?)?
        )?;

        current_estimation.t_12_i.copy_to(
            &mut current_estimation.T12_i
            .col(0)?
        )?;

        // Step 8.2 T21
        let s_r_inv = res_to_mat((1.0 / current_estimation.s_12_i) * current_estimation.r_12_i.t()?)?;

        s_r_inv.copy_to(
            &mut current_estimation.T21_i
            .row_range(&Range::new(0,3)?)?
            .col_range(&Range::new(0,3)?)?
        )?;

        let neg_s_r_inv = s_r_inv.mul(&-1., 1.)?.to_mat()?;
        let tinv = res_to_mat(neg_s_r_inv * &current_estimation.t_12_i)?;
        tinv.copy_to(
            &mut current_estimation.T21_i
            .row_range(&Range::new(0,3)?)?
            .col(3)?
        )?;
        
        Ok(current_estimation)
    }

    fn res_to_mat<T: opencv::prelude::MatExprTraitConst>(res: MatExprResult<T>) -> Result<Mat, opencv::Error> {
        // Can't be an into() because both input and output are not defined in this crate
        res.into_result()?.to_mat()
    }

    fn compare_mats(m1: &Mat, m2: &Mat) -> bool {
        let mut diff = Mat::default();
        opencv::core::absdiff(m1, m2, &mut diff);
        let mut diff_sum = Mat::default();
        opencv::core::reduce(&diff, &mut diff_sum, 0, opencv::core::REDUCE_SUM, CV_64F).unwrap();
        let diff_sum = diff_sum.at_2d::<f64>(0, 0).unwrap();
        *diff_sum < 0.0001
    }

    #[derive(Debug)]
    struct Sim3Estimation{
        r_12_i: Mat, // mR12i
        t_12_i: Mat, // mt12i
        s_12_i: f64, // ms12i
        T12_i: Mat, // mT12i
        T21_i: Mat, // mT21i
        inliers_i: Vec<bool>, // mvbInliersi
        inliers_count: i32, // mnInliersi
    }


}
