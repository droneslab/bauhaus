// In ORBSLAM, there are a lot of extra variables that are set to 0 or null
// because they are only used in stereo or IMU cases. This encapsulates 
// "all feature data" into a struct that has different variables depending
// on the type of sensor.

// TODO (design, code organization) ... I don't think it makes the most sense to name this "features" or even to have it separate from a keyframe
// But at the same time it's nice to hide a lot of this logic away from the keyframe. It would be good to
// re-factor this eventually but it isn't high priority.

use core::config::SETTINGS;
use std::fmt::Debug;
use core::sensor::{Sensor, FrameSensor};
use opencv::prelude::{Mat, MatTraitConst, MatTrait, KeyPointTraitConst};
use opencv::types::{VectorOff32};
use opencv::core::{KeyPoint, CV_32F, Scalar, Point2f};
use crate::registered_actors::{CAMERA_MODULE, FEATURES};
use crate::{
    matrix::{DVMatrix, DVVectorOfKeyPoint},
    map::map::Id,
};

use super::keyframe::MapPointMatches;


#[derive(Clone, Debug, Default)]
enum KeyPoints {
    #[default] Empty,
    Mono { 
        keypoints_un: DVVectorOfKeyPoint // Undistorted keypoints actually used by the system. For stereo, this is redundant bc images must be rectified
    },
    Stereo { 
        keypoints_left: DVVectorOfKeyPoint,
        keypoints_left_cutoff: u32, // Nleft, if stereo and index passed in is < keypoints_left_cutoff, need to get keypoint from keypoints_right instead of keypoints
        keypoints_right: DVVectorOfKeyPoint,
        mv_right: Vec<f32>, // mvuRight
        mv_depth: Vec<f32>, //mvDepth
    },
    Rgbd {
        keypoints_un: DVVectorOfKeyPoint, // Undistorted keypoints actually used by the system. For stereo, this is redundant bc images must be rectified
        mv_depth: Vec<f32>, //mvDepth
    }
}

#[derive(Clone, Debug, Default)]
pub struct Features {
    // Common across all sensor types

    pub num_keypoints: u32, // N
    keypoints: KeyPoints,
    pub descriptors: DVMatrix, // mDescriptors

    pub image_width: u32,
    pub image_height: u32,

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    grid_element_width_inv: f64,
    grid_element_height_inv: f64,
    grid: Vec<Vec<Vec<usize>>>, // mGrid

    // Image Bounds
    min_x: f64,//static float mnMinX;
    max_x: f64,//static float mnMaxX;
    min_y: f64,//static float mnMinY;
    max_y: f64,//static float mnMaxY;

    // Settings
    frame_grid_cols: i32, 
    frame_grid_rows: i32,

}


// Function to find minimum of two f32 values
fn min_float(a: f32, b: f32) -> f32 {
    if a < b {
        a
    } else {
        b
    }
}

// Function to find maximum of two f32 values
fn max_float(a: f32, b: f32) -> f32 {
    if a > b {
        a
    } else {
        b
    }
}

impl Features {
    pub fn empty() -> Self{
        Features {
            num_keypoints: 0,
            keypoints: KeyPoints::Empty,
            descriptors: DVMatrix::empty(),
            image_width: 0,
            image_height: 0,
            grid_element_width_inv: 0.0,
            grid_element_height_inv: 0.0,
            grid: vec![],
            min_x: 0.0,
            max_x: 0.0,
            min_y: 0.0,
            max_y: 0.0,
            frame_grid_cols: 0,
            frame_grid_rows: 0
        }
    }

    pub fn new(
        keypoints: DVVectorOfKeyPoint,
        descriptors: DVMatrix,
        im_width: u32, im_height: u32,
        sensor: Sensor
    ) -> Result<Features, Box<dyn std::error::Error>> {
        // Grid
        let frame_grid_cols = SETTINGS.get::<i32>(FEATURES, "frame_grid_cols");
        let frame_grid_rows = SETTINGS.get::<i32>(FEATURES, "frame_grid_rows");

        let mut grid = Vec::new();
        for _ in 0..frame_grid_cols  {
            let mut row = Vec::new();
            for _ in 0..frame_grid_rows {
                row.push(Vec::new());
            }
            grid.push(row);
        }

        // Image Bounds
        let (min_x, max_x, min_y, max_y) = match &CAMERA_MODULE.dist_coef {
            Some(_vec) => {

                let points = vec![
                    Point2f::new(0.0, 0.0),
                    Point2f::new(im_width as f32, 0.0),
                    Point2f::new(0.0, im_height as f32),
                    Point2f::new(im_width as f32, im_height as f32),
                ];

                // keypoints for point2f

                // Reshape points
                let mut mat = Mat::from_slice_2d(&points.iter().map(|p| vec![p.x, p.y]).collect::<Vec<_>>()).unwrap();

                mat = mat.reshape(2, 0).unwrap();

                let mut undistorted_points = mat.clone();
                // Undistort points
                // let mut undistorted_points = Mat::default();
                let dist_coefs = VectorOff32::from_iter((*_vec).clone());
                opencv::calib3d::undistort_points(
                    &mat,
                    &mut undistorted_points,
                    &CAMERA_MODULE.k_matrix.mat(),
                    &dist_coefs,
                    &Mat::eye(3, 3, opencv::core::CV_32F)?,
                    &CAMERA_MODULE.k_matrix.mat(),
                )?;
                let undistorted_points = undistorted_points.reshape(1, 0).unwrap();

                // Get the min and max values
                let mn_min_x = min_float(
                    *undistorted_points.at_2d::<f32>(0, 0).unwrap(),
                    *undistorted_points.at_2d::<f32>(2, 0).unwrap(),
                );
                let mn_max_x = max_float(
                    *undistorted_points.at_2d::<f32>(1, 0).unwrap(),
                    *undistorted_points.at_2d::<f32>(3, 0).unwrap(),
                );
                let mn_min_y = min_float(
                    *undistorted_points.at_2d::<f32>(0, 1).unwrap(),
                    *undistorted_points.at_2d::<f32>(1, 1).unwrap(),
                );
                let mn_max_y = max_float(
                    *undistorted_points.at_2d::<f32>(2, 1).unwrap(),
                    *undistorted_points.at_2d::<f32>(3, 1).unwrap(),
                );

                let mn_max_x_f64 = mn_max_x as f64;
                let mn_max_y_f64 = mn_max_y as f64;
                let mn_min_x_f64 = mn_min_x as f64;
                let mn_min_y_f64 = mn_min_y as f64;

                (mn_min_x_f64, mn_max_x_f64, mn_min_y_f64, mn_max_y_f64)
            },
            None => {
                (0.0 as f64 , im_width as f64, 0.0 as f64, im_height as f64)
            }
        };


        match sensor.frame() {
            FrameSensor::Mono => {
                let keypoints_un = Self::undistort_keypoints(&keypoints)?;
                let num_keypoints = keypoints.len() as u32;
                // assign features to grid
                let grid_element_width_inv =  frame_grid_cols as f64/(max_x - min_x) as f64;
                let grid_element_height_inv = frame_grid_rows as f64/(max_y - min_y) as f64;
                for i in 0..keypoints_un.len() as usize {
                    let kp = &keypoints_un.get(i).unwrap();
                    let pos_x = ((kp.pt().x-(min_x as f32))*grid_element_width_inv as f32).round() as i32;
                    let pos_y = ((kp.pt().y-(min_y as f32))*grid_element_height_inv as f32).round() as i32;

                    let not_in_bounds = pos_x<0 || pos_x>=frame_grid_cols as i32 || pos_y<0 || pos_y>=frame_grid_rows as i32;

                    //Keypoint's coordinates are undistorted, which could cause to go out of the image
                    if not_in_bounds {
                        continue;
                    } else {
                        grid[pos_x as usize][pos_y as usize].push(i);
                    }
                }

                let features = Features {
                    num_keypoints,
                    keypoints: KeyPoints::Mono { keypoints_un },
                    descriptors,
                    grid_element_width_inv,
                    grid_element_height_inv,
                    grid,
                    min_x,
                    max_x,
                    min_y,
                    max_y,
                    frame_grid_cols,
                    frame_grid_rows,
                    image_width: im_width,
                    image_height: im_height
                };

                Ok(features)
            },
            FrameSensor::Rgbd => {
                todo!("RGBD");
                // mv_right and mv_depth size should be same as keypoints, if there isn't a value for an index it should be 0
            },
            FrameSensor::Stereo => {
                todo!("Stereo");
                // mv_right and mv_depth size should be same as keypoints, if there isn't a value for an index it should be 0
            }

        }
    }

    pub fn has_left_kp(&self) -> Option<u32> {
        match &self.keypoints {
            KeyPoints::Stereo{keypoints_left_cutoff, ..} => Some(*keypoints_left_cutoff),
            _ => None
        }
    }

    pub fn get_all_keypoints(&self) -> &DVVectorOfKeyPoint {
        match &self.keypoints {
            KeyPoints::Mono{keypoints_un, ..} | KeyPoints::Rgbd{keypoints_un, ..} => keypoints_un,
            KeyPoints::Stereo{  ..} => { 
                todo!("Stereo, need to concat keypoints_left and keypoints_right
                    but can we do this without copying?")
            },
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn get_keypoint(&self, index: usize) -> (KeyPoint, bool) {
        // equivalent to mvKeysUn[leftIndex] or mvKeysUn[rightIndex]
        // Return keypoint and whether it is in the right frame or not
        match &self.keypoints {
            KeyPoints::Mono{keypoints_un, ..} | KeyPoints::Rgbd{keypoints_un, ..} => (keypoints_un.get(index).unwrap(), false),
            KeyPoints::Stereo{keypoints_left, keypoints_left_cutoff, keypoints_right, ..} => {
                if index < *keypoints_left_cutoff as usize {
                    (keypoints_left.get(index).unwrap(), false)
                } else {
                    (keypoints_right.get(index).unwrap(), true)
                }
            },
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn replace_keypoints_and_descriptors(&mut self, keypoints: opencv::types::VectorOfKeyPoint, descriptors: Mat) {
        match &self.keypoints {
            KeyPoints::Mono{..} => {
                self.num_keypoints = keypoints.len() as u32;
                self.keypoints = KeyPoints::Mono { keypoints_un: DVVectorOfKeyPoint::new(keypoints) };
                self.descriptors = DVMatrix::new(descriptors);
            },
            KeyPoints::Rgbd{ ..} => todo!("RGBD"),
            KeyPoints::Stereo{  ..} => todo!("Stereo"), 
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        };
    }

    pub fn remove_keypoint_and_descriptor(&mut self, index: usize) -> Result<(), Box<dyn std::error::Error>> {
        match self.keypoints {
            KeyPoints::Mono{ref mut keypoints_un, ..} => {
                keypoints_un.remove(index)?;
                self.num_keypoints -= 1;
                Ok(())
            },
            KeyPoints::Rgbd{ ..} => todo!("RGBD"),
            KeyPoints::Stereo{  ..} => todo!("Stereo"), 
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn get_mv_depth(&self, i: usize) -> Option<f32> {
        match &self.keypoints {
            KeyPoints::Mono{..}  => None,
            KeyPoints::Stereo{mv_depth, ..} | KeyPoints::Rgbd{mv_depth, ..} => Some(mv_depth[i]),
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn get_mv_right(&self, i: usize) -> Option<f32> {
        // mvuRight
        match &self.keypoints {
            KeyPoints::Mono{..} | KeyPoints::Rgbd{..}  => None,
            KeyPoints::Stereo{mv_right, ..}  => Some(mv_right[i]),
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn get_octave(&self, index: usize) -> i32 {
        // Note: Use this function if you ever see a code pattern like this
        // bestLevel = (F.Nleft == -1) ? F.mvKeysUn[idx].octave()
        //                     : (idx < F.Nleft) ? F.mvKeys[idx].octave()
        //                                         : F.mvKeysRight[idx - F.Nleft].octave();
        self.get_keypoint(index).0.octave()
    }

    pub fn check_close_tracked_mappoints( &self, th_depth: f32, mappoint_matches: &Vec<Option<(Id, bool)>> ) -> (i32, i32) {
        match &self.keypoints {
            KeyPoints::Mono{..}  => (0,0),
            KeyPoints::Stereo{mv_depth, ..} | KeyPoints::Rgbd{mv_depth, ..} => {
                let (mut tracked_close, mut non_tracked_close) = (0, 0);
                for i in 0..self.num_keypoints as u32 {
                    let depth = mv_depth[i as usize];
                    if depth > 0.0 && depth < th_depth {
                        if let Some((_id, is_outlier)) = mappoint_matches[i as usize] {
                            if !is_outlier {
                                tracked_close += 1;
                            } else {
                                non_tracked_close += 1;
                            }
                        }
                    }
                }
                (tracked_close, non_tracked_close) 
            },
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    fn assign_features_to_grid() {

    }

    fn undistort_keypoints(keypoints: &DVVectorOfKeyPoint) -> Result<DVVectorOfKeyPoint, Box<dyn std::error::Error>> {
        // void Frame::UndistortKeyPoints()
        if let Some(dist_coef) = &CAMERA_MODULE.dist_coef {

            let num_keypoints = keypoints.len();
            // Fill matrix with points
            let mut mat = Mat::new_rows_cols_with_default(num_keypoints,2, CV_32F, Scalar::all(0.0))?;
            for i in 0..num_keypoints {
                *mat.at_2d_mut::<f32>(i, 0)? = keypoints.get(i as usize)?.pt().x;
                *mat.at_2d_mut::<f32>(i, 1)? = keypoints.get(i as usize)?.pt().y;
            }

            // TODO (timing) ... can we do this in place? Then we don't have to construct and return keypoints_un

            // Undistort points
            mat = mat.reshape(2, 0)?;
            let mut undistorted = mat.clone();
            let dist_coefs = VectorOff32::from_iter((*dist_coef).clone());
            opencv::calib3d::undistort_points(
                &mat,
                &mut undistorted,
                &CAMERA_MODULE.k_matrix.mat(),
                &dist_coefs,
                &Mat::eye(3, 3, opencv::core::CV_32F)?,
                &CAMERA_MODULE.k_matrix.mat(),
            )?;

            mat = mat.reshape(1, 0)?;

            // Fill undistorted keypoint vector
            let mut keypoints_un = opencv::types::VectorOfKeyPoint::new();
            for i in 0..num_keypoints {
                let kp = keypoints.get(i as usize)?;
                kp.pt().x = *mat.at_2d::<f32>(i, 0)?;
                kp.pt().y = *mat.at_2d::<f32>(i, 1)?;
                keypoints_un.push(kp);
            }

            Ok(DVVectorOfKeyPoint::new(keypoints_un))
        } else {
            return Ok(keypoints.clone());
        }
    }

    pub fn get_features_in_area(&self, x: &f64, y: &f64, r: f64, levels: Option<(i32, i32)>,) -> Vec<u32> {
        //GetFeaturesInArea
        let mut indices = vec![];

        let grid_element_width_inv = self.grid_element_width_inv;
        let grid_element_height_inv = self.grid_element_height_inv;

        let factor_x = r;
        let factor_y = r;

        let min_cell_x = i64::max(0, ((x-self.min_x-factor_x)*grid_element_width_inv).floor() as i64);
        let max_cell_x = i64::min((self.frame_grid_cols-1) as i64, ((x-self.min_x+factor_x)*grid_element_width_inv).ceil() as i64);
        let min_cell_y = i64::max(0, ((y-self.min_y-factor_y)*grid_element_height_inv).floor() as i64);
        let max_cell_y = i64::min((self.frame_grid_rows-1) as i64, ((y-self.min_y+factor_y)*grid_element_height_inv).ceil() as i64);

        if !self.is_in_image(min_cell_x as f64, min_cell_y as f64) || !self.is_in_image(max_cell_x as f64, max_cell_y as f64) {
            return indices;
        }

        let check_levels = levels.is_some() && (levels.unwrap().0>0 || levels.unwrap().1>=0);

        // println!("Get features in area... min cell x: {}, max cell x: {}, min cell y: {}, max cell y: {}", min_cell_x, max_cell_x, min_cell_y, max_cell_y);
        for ix in min_cell_x..max_cell_x + 1 {
            for iy in min_cell_y..max_cell_y + 1 {
                let v_cell  =&self.grid[ix as usize][iy as usize];
                // TODO (STEREO) 
                //const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];

                if v_cell.is_empty() {
                    continue;
                }

                for j in 0..v_cell.len() {
                    //TODO (Stereo) Need to update this if stereo images are processed
                    let (kp_un, _is_right) = &self.get_keypoint(v_cell[j]);
                    if check_levels {
                        if kp_un.octave() < levels.unwrap().0 as i32 {
                            continue;
                        }
                        if levels.unwrap().1 >= 0 {
                            if kp_un.octave() > levels.unwrap().1 as i32 {
                                continue;
                            }
                        }
                    }

                    let distx = kp_un.pt().x - (*x as f32);
                    let disty = kp_un.pt().y - (*y as f32);

                    if distx.abs()<(factor_x as f32) && disty.abs() < (factor_y as f32)  {
                        indices.push(v_cell[j] as u32);
                    }
                }
            }
        }
        return indices;
    }

    pub fn is_in_image(&self, x: f64, y: f64) -> bool {
        // bool KeyFrame::IsInImage(const float &x, const float &y) const
        return x >= self.min_x && x < self.max_x && y >= self.min_y && y < self.max_y;
    }
}

