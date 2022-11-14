// There are a lot of extra variables that are set to 0 or null
// because they are only used in stereo or IMU cases. This encapsulates 
// "all feature data" into a struct that has different variables depending
// on the type of sensor.
use std::collections::{HashMap};
use std::fmt::Debug;
use dvcore::global_params::FrameSensor;
use dvcore::global_params::{*};
use opencv::prelude::{Mat, MatTraitConst, MatTrait};
use opencv::types::{VectorOff32};
use serde::{Deserialize, Serialize};
use opencv::core::{KeyPoint, CV_32F, Scalar};
use crate::{
    matrix::{DVMatrix, DVVectorOfKeyPoint},
    dvmap::{frame::ImageBounds, map::Id}, modules::camera::Camera,
};

pub const FRAME_GRID_ROWS :usize = 48;
pub const FRAME_GRID_COLS :usize = 64;

#[derive(Clone, Debug, Default)]
enum KeyPoints {
    #[default] Empty,
    Mono{ 
        keypoints_orig: DVVectorOfKeyPoint, // Original for visualization.
        keypoints_un: DVVectorOfKeyPoint // Undistorted keypoints actually used by the system. For stereo, this is redundant bc images must be rectified
    },
    Stereo{
        keypoints_left: DVVectorOfKeyPoint,
        keypoints_left_cutoff: u32, // Nleft, if stereo and index passed in is < keypoints_left_cutoff, need to get keypoint from keypoints_right instead of keypoints
        keypoints_right: DVVectorOfKeyPoint,
        mv_right: HashMap<u32, f32>, // mvuRight
        mv_depth: HashMap<u32, f32>, //mvDepth
    },
    Rgbd{
        keypoints_orig: DVVectorOfKeyPoint, // Original for visualization.
        keypoints_un: DVVectorOfKeyPoint, // Undistorted keypoints actually used by the system. For stereo, this is redundant bc images must be rectified
        mv_depth: HashMap<u32, f32>, //mvDepth
    }
}

#[derive(Clone, Debug, Default)]
pub struct Features {
    // Common across all sensor types:
    pub image_bounds: ImageBounds, // min_x, max_x, min_y, max_y
    pub num_keypoints: u32,
    keypoints: KeyPoints,
    pub descriptors: DVMatrix, // mDescriptors
    grid: Grid, // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
}

impl Features {
    pub fn new(
        keypoints: &DVVectorOfKeyPoint,
        descriptors: &DVMatrix,
        im_width: i32, im_height: i32,
        camera: &Camera,
        sensor: Sensor
    ) -> Result<Features, Box<dyn std::error::Error>> {
        let image_bounds = ImageBounds::new(im_width, im_height, &camera.dist_coef);

        let keypoints_orig = keypoints.clone();
        let mut grid = Grid::default(&image_bounds);

        match sensor.frame() {
            FrameSensor::Mono => {
                let keypoints_un =  Self::undistort_keypoints(keypoints, camera)?;
                let num_keypoints = keypoints_un.len() as u32;

                // assign features to grid
                grid.assign_features(&image_bounds, &keypoints_un);
                Ok::<Features, Box<dyn std::error::Error>>(
                    Features {
                        image_bounds,
                        num_keypoints,
                        keypoints: KeyPoints::Mono { keypoints_orig, keypoints_un },
                        descriptors: descriptors.clone(),
                        grid,
                        ..Default::default()
                    }
                )
            },
            FrameSensor::Rgbd => {
                todo!("todo RGBD");
            },
            FrameSensor::Stereo => {
                todo!("todo stereo");
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
            KeyPoints::Stereo{keypoints_left, keypoints_right, ..} => {
                todo!("TODO Stereo, need to concat keypoints_left and keypoints_right
                    but can we do this without copying?")
            },
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn get_keypoint(&self, index: usize) -> KeyPoint {
        match &self.keypoints {
            KeyPoints::Mono{keypoints_un, ..} | KeyPoints::Rgbd{keypoints_un, ..} => keypoints_un.get(index).unwrap(),
            KeyPoints::Stereo{keypoints_left, keypoints_left_cutoff, keypoints_right, ..} => {
                if index < *keypoints_left_cutoff as usize {
                    keypoints_left.get(index).unwrap()
                } else {
                    keypoints_right.get(index).unwrap()
                }
            },
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn mv_depth_get(&self, i: &u32) -> Option<&f32> {
        match &self.keypoints {
            KeyPoints::Mono{..}  => None,
            KeyPoints::Stereo{mv_depth, ..} | KeyPoints::Rgbd{mv_depth, ..} => mv_depth.get(i),
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn get_octave(&self, index: usize) -> i32 {
        self.get_keypoint(index).octave
    }

    pub fn check_close_tracked_mappoints( &self, th_depth: f32, mappoint_matches: &HashMap::<u32, (Id, bool)> ) -> (i32, i32) {
        match &self.keypoints {
            KeyPoints::Mono{..}  => (0,0),
            KeyPoints::Stereo{mv_depth, ..} | KeyPoints::Rgbd{mv_depth, ..} => {
                let (mut tracked_close, mut non_tracked_close) = (0, 0);
                for i in 0..self.num_keypoints as u32 {
                    if let Some(value) = mv_depth.get(&i) {
                        let depth = *value;
                        if depth > 0.0 && depth < th_depth {
                            let mp = mappoint_matches.get(&i);
                            if mp.is_some() && !mp.unwrap().1 {
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

    fn undistort_keypoints(keypoints: &DVVectorOfKeyPoint, camera: &Camera) -> Result<DVVectorOfKeyPoint, Box<dyn std::error::Error>> {
        // TODO (Potential bugs) not sure I did this right
        if let Some(dist_coef) = &camera.dist_coef {

            let N = keypoints.len();
            // Fill matrix with points
            let mut mat = Mat::new_rows_cols_with_default(N,2, CV_32F, Scalar::all(0.0))?;
            for i in 0..N {
                *mat.at_2d_mut::<f32>(i, 0)? = keypoints.get(i as usize)?.pt.x;
                *mat.at_2d_mut::<f32>(i, 1)? = keypoints.get(i as usize)?.pt.y;
            }

            // Undistort points
            mat = mat.reshape(2, 0)?;
            let mut undistorted = mat.clone();
            let dist_coefs = VectorOff32::from_iter((*camera.dist_coef.as_ref().unwrap()).clone());
            opencv::calib3d::undistort_points(
                &mat,
                &mut undistorted,
                &camera.to_K_matrix()?,
                &dist_coefs,
                &Mat::eye(3, 3, opencv::core::CV_32F)?,
                &camera.to_K_matrix()?,
            )?;

            mat = mat.reshape(1, 0)?;

            // Fill undistorted keypoint vector
            let mut undistorted_kp_vec = opencv::types::VectorOfKeyPoint::new();
            for i in 0..N {
                let mut kp = keypoints.get(i as usize)?;
                kp.pt.x = *mat.at_2d::<f32>(i, 0)?;
                kp.pt.y = *mat.at_2d::<f32>(i, 1)?;
                undistorted_kp_vec.push(kp);
            }

            Ok(DVVectorOfKeyPoint::new(undistorted_kp_vec))
        } else {
            return Ok(keypoints.clone());
        }
    }

    pub fn get_features_in_area(&self, x: &f64, y: &f64, r: &f64, min_level: &i64, max_level: &i64) -> Vec<usize> {
        //GetFeaturesInArea
        let mut indices = Vec::<usize>::new();
        indices.reserve(self.num_keypoints as usize);

        let frame_grid_rows = FRAME_GRID_ROWS as i64;
        let frame_grid_cols = FRAME_GRID_COLS as i64;
        let grid_element_width_inv = self.grid.grid_element_width_inv;
        let grid_element_height_inv = self.grid.grid_element_height_inv;

        let factor_x = *r;
        let factor_y = *r;

        let min_cell_x = i64::max(0, ((x-self.image_bounds.min_x-factor_x)*grid_element_width_inv).floor() as i64);
        let max_cell_x = i64::min(frame_grid_cols-1, ((x-self.image_bounds.min_x+factor_x)*grid_element_width_inv).ceil() as i64);
        let min_cell_y = i64::max(0, ((y-self.image_bounds.min_y-factor_y)*grid_element_height_inv).floor() as i64);
        let max_cell_y = i64::min(frame_grid_rows-1, ((y-self.image_bounds.min_y+factor_y)*grid_element_height_inv).ceil() as i64);

        if min_cell_x >= frame_grid_cols || max_cell_x < 0 || min_cell_y >= frame_grid_rows || max_cell_y < 0 {
            return indices;
        }

        let b_check_levels = *min_level>0 || *max_level>=0;

        for ix in min_cell_x..max_cell_x + 1 {
            for iy in min_cell_y..max_cell_y + 1 {
                let v_cell = Vec::<usize>::new();
                //const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];

                if v_cell.is_empty() {
                    continue;
                }

                for j in 0..v_cell.len() {
                    //TODO (Stereo) Need to update this if stereo images are processed
                    let kp_un = &self.get_keypoint(v_cell[j]);
                    if b_check_levels {
                        if kp_un.octave< *min_level as i32 {
                            continue;
                        }
                        if *max_level>=0 {
                            if kp_un.octave> *max_level as i32 {
                                continue;
                            }
                        }
                    }

                    let distx = kp_un.pt.x- (*x as f32);
                    let disty = kp_un.pt.y- (*y as f32);

                    if distx.abs()<(factor_x as f32) && disty.abs() < (factor_y as f32)  {
                        indices.push(v_cell[j]);
                    }
                }
            }
        }
        return indices;
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
struct Grid {
    pub grid_element_width_inv: f64,
    pub grid_element_height_inv: f64,
    pub grid: Vec<Vec<usize>> // mGrid
}
impl Grid {
    pub fn default(image_bounds: &ImageBounds) -> Self {
        Grid {
            grid_element_width_inv: FRAME_GRID_COLS as f64/(image_bounds.max_x - image_bounds.min_x) as f64,
            grid_element_height_inv: FRAME_GRID_ROWS as f64/(image_bounds.max_y - image_bounds.min_y) as f64,
            grid: Self::initialize_grid()
        }
    }

    fn initialize_grid() -> Vec<Vec<usize>> {
        let mut grid = Vec::new();
        for _ in 0..FRAME_GRID_ROWS {
            let row = vec![0; FRAME_GRID_COLS];
            grid.push(row);
        }
        grid
    }

    pub fn assign_features(&mut self, image_bounds: &ImageBounds, keypoints_un: &DVVectorOfKeyPoint) {
        for i in 0..keypoints_un.len() as usize {
            let keypoint = &keypoints_un.get(i).unwrap();
            if let Some((pos_x, pos_y)) = self.pos_in_grid(&image_bounds, keypoint) { 
                self.grid[pos_y as usize][pos_x as usize] = i;
            }
        }
    }

    fn pos_in_grid(&self, image_bounds: &ImageBounds, kp : &KeyPoint) -> Option<(i32, i32)> {
        let pos_x = (kp.pt.x-(image_bounds.min_x as f32)*self.grid_element_width_inv as f32).round() as i32;
        let pos_y = (kp.pt.y-(image_bounds.min_y as f32)*self.grid_element_height_inv as f32).round() as i32;

        let x_in_bounds = pos_x >= 0 && pos_x < (FRAME_GRID_COLS as i32);
        let y_in_bounds = pos_y >= 0 && pos_y < (FRAME_GRID_ROWS as i32);
        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if x_in_bounds && y_in_bounds {
            return Some((pos_x, pos_y));
        } else{
            return None;
        }
    }
}



// For conversion to/from C++
// impl From<DVVectorOfKeyPoint> for CxxVector<dvos3binding::ffi::DVKeyPoint> {
//     fn from(vec: DVVectorOfKeyPoint) -> Self { 
//         *vec.into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>
//     }
// }
