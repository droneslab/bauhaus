#[cfg(test)]
mod tools_tests {
    use dvcore::matrix::DVVector3;

    use crate::{modules::geometric_tools, dvmap::pose::DVPose};

    use super::*;

    #[test]
    fn test_geometric_tools() {
        // Inputs (copied from an example input in ORBSLAM3)
        let translation1 = nalgebra::Vector3::new(
            -0.0293614, 0.0492868, -2.58484
        );
        let rotation1 = nalgebra::Matrix3::new(
        0.997738, -0.00115789, 0.0672201,
        0.00176257, 0.999959, -0.00893691,
        -0.0672069, 0.00903517, 0.997698
        );
        let translation2 = nalgebra::Vector3::new(
            -0.0113896, 0.00524169, -0.809653
        );
        let rotation2 = nalgebra::Matrix3::new(
        0.998599, -0.00157662, 0.0528905,
        0.00139543, 0.999993, 0.00346237,
        -0.0528956, -0.00338372, 0.998594
        );
        let c1 = DVVector3::new_with(-0.345261, -0.0295131, 1.0);
        let c2 = DVVector3::new_with(-0.317161, -0.0189408, 1.0);

        let result = geometric_tools::triangulate(
            c1,
            c2,
            DVPose::new(translation1, rotation1),
            DVPose::new(translation2, rotation2)
        ).unwrap();
        let result_trun = (f64::trunc(result[0]  * 10000.0) / 10000.0, f64::trunc(result[1]  * 10000.0) / 10000.0, f64::trunc(result[2]  * 10000.0) / 10000.0);

        let expected = DVVector3::new_with(-5.2549, -0.2957, 14.6811);
        assert_eq!(
            result_trun,
            (expected[0], expected[1], expected[2])
        );
    }
}
