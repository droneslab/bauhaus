// There are a lot of extra variables that are set to 0 or null
// because they are only used in stereo or IMU cases. This encapsulates 
// "all feature data" into a struct that has different variables depending
// on the type of sensor.
use std::collections::{HashMap};
use std::fmt::Debug;
use dvcore::config::FrameSensor;
use dvcore::config::{*};
use log::debug;
use opencv::prelude::{Mat, MatTraitConst, MatTrait};
use opencv::types::{VectorOff32};
use serde::{Deserialize, Serialize};
use opencv::core::{KeyPoint, CV_32F, Scalar};
use crate::modules::camera::CAMERA_MODULE;
use crate::{
    matrix::{DVMatrix, DVVectorOfKeyPoint},
    dvmap::{map::Id},
};

pub const FRAME_GRID_ROWS :usize = 48;
pub const FRAME_GRID_COLS :usize = 64;

// TODO (eventually) ... I don't think it makes the most sense to name this "features" or even to have it separate from a keyframe
// But at the same time it's nice to hide a lot of this logic away from the keyframe. It would be good to
// re-factor this eventually but it isn't high priority.

#[derive(Clone, Debug, Default)]
enum KeyPoints {
    #[default] Empty,
    Mono { 
        keypoints_orig: DVVectorOfKeyPoint, // Original for visualization.
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
        keypoints_orig: DVVectorOfKeyPoint, // Original for visualization.
        keypoints_un: DVVectorOfKeyPoint, // Undistorted keypoints actually used by the system. For stereo, this is redundant bc images must be rectified
        mv_depth: Vec<f32>, //mvDepth
    }
}

#[derive(Clone, Debug, Default)]
pub struct Features {
    // Common across all sensor types:
    pub num_keypoints: u32,
    keypoints: KeyPoints,
    pub image_bounds: ImageBounds,
    pub descriptors: DVMatrix, // mDescriptors
    pub grid: Grid, // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
}

impl Features {
    pub fn new(
        keypoints: DVVectorOfKeyPoint,
        descriptors: DVMatrix,
        im_width: i32, im_height: i32,
        sensor: Sensor
    ) -> Result<Features, Box<dyn std::error::Error>> {
        let image_bounds = ImageBounds::new(im_width, im_height, &CAMERA_MODULE.dist_coef);
        let mut grid = Grid::default(&image_bounds);
        let keypoints_orig = keypoints.clone();

        match sensor.frame() {
            FrameSensor::Mono => {
                let keypoints_un =  Self::undistort_keypoints(&keypoints_orig)?;
                let num_keypoints = keypoints_un.len() as u32;

                // assign features to grid
                grid.assign_features(&image_bounds, &keypoints_un);
                Ok::<Features, Box<dyn std::error::Error>>(
                    Features {
                        num_keypoints,
                        image_bounds,
                        keypoints: KeyPoints::Mono { keypoints_orig, keypoints_un },
                        descriptors: descriptors.clone(),
                        grid,
                    }
                )
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
            KeyPoints::Stereo{keypoints_left, keypoints_right, ..} => { 
                todo!("Stereo, need to concat keypoints_left and keypoints_right
                    but can we do this without copying?")
            },
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn get_keypoint(&self, index: usize) -> (KeyPoint, bool) {
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

    pub fn get_mv_depth(&self, i: usize) -> Option<f32> {
        match &self.keypoints {
            KeyPoints::Mono{..}  => None,
            KeyPoints::Stereo{mv_depth, ..} | KeyPoints::Rgbd{mv_depth, ..} => Some(mv_depth[i]),
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn get_mv_right(&self, i: usize) -> Option<f32> {
        match &self.keypoints {
            KeyPoints::Mono{..} | KeyPoints::Rgbd{..}  => None,
            KeyPoints::Stereo{mv_right, ..}  => Some(mv_right[i]),
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    pub fn get_octave(&self, index: usize) -> i32 {
        // Note: Use this function if you ever see a code pattern like this
        // bestLevel = (F.Nleft == -1) ? F.mvKeysUn[idx].octave
        //                     : (idx < F.Nleft) ? F.mvKeys[idx].octave
        //                                         : F.mvKeysRight[idx - F.Nleft].octave;
        self.get_keypoint(index).0.octave
    }

    pub fn check_close_tracked_mappoints( &self, th_depth: f32, mappoint_matches: &HashMap::<u32, (Id, bool)> ) -> (i32, i32) {
        match &self.keypoints {
            KeyPoints::Mono{..}  => (0,0),
            KeyPoints::Stereo{mv_depth, ..} | KeyPoints::Rgbd{mv_depth, ..} => {
                let (mut tracked_close, mut non_tracked_close) = (0, 0);
                for i in 0..self.num_keypoints as u32 {
                    let depth = mv_depth[i as usize];
                    if depth > 0.0 && depth < th_depth {
                        let mp = mappoint_matches.get(&i);
                        if mp.is_some() && !mp.unwrap().1 {
                            tracked_close += 1;
                        } else {
                            non_tracked_close += 1;
                        }
                    }
                }
                (tracked_close, non_tracked_close) 
            },
            KeyPoints::Empty => panic!("Keypoints should not be empty")
        }
    }

    fn undistort_keypoints(keypoints: &DVVectorOfKeyPoint) -> Result<DVVectorOfKeyPoint, Box<dyn std::error::Error>> {
        if let Some(dist_coef) = &CAMERA_MODULE.dist_coef {

            let num_keypoints = keypoints.len();
            // Fill matrix with points
            let mut mat = Mat::new_rows_cols_with_default(num_keypoints,2, CV_32F, Scalar::all(0.0))?;
            for i in 0..num_keypoints {
                *mat.at_2d_mut::<f32>(i, 0)? = keypoints.get(i as usize)?.pt.x;
                *mat.at_2d_mut::<f32>(i, 1)? = keypoints.get(i as usize)?.pt.y;
            }

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
            let mut undistorted_kp_vec = opencv::types::VectorOfKeyPoint::new();
            for i in 0..num_keypoints {
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

    pub fn get_features_in_area(&self, x: &f64, y: &f64, r: f64, min_level: i32, max_level: i32, image_bounds: &ImageBounds) -> Vec<u32> {
        //GetFeaturesInArea
        let mut indices = Vec::<u32>::new();
        indices.reserve(self.num_keypoints as usize);

        let frame_grid_rows = FRAME_GRID_ROWS as i64;
        let frame_grid_cols = FRAME_GRID_COLS as i64;
        let grid_element_width_inv = self.grid.grid_element_width_inv;
        let grid_element_height_inv = self.grid.grid_element_height_inv;

        let factor_x = r;
        let factor_y = r;

        let min_cell_x = i64::max(0, ((x-image_bounds.min_x-factor_x)*grid_element_width_inv).floor() as i64);
        let max_cell_x = i64::min(frame_grid_cols-1, ((x-image_bounds.min_x+factor_x)*grid_element_width_inv).ceil() as i64);
        let min_cell_y = i64::max(0, ((y-image_bounds.min_y-factor_y)*grid_element_height_inv).floor() as i64);
        let max_cell_y = i64::min(frame_grid_rows-1, ((y-image_bounds.min_y+factor_y)*grid_element_height_inv).ceil() as i64);

        if !image_bounds.check_bounds(min_cell_x as f64, min_cell_y as f64) || !image_bounds.check_bounds(max_cell_x as f64, max_cell_y as f64) {
            return indices;
        }

        let check_levels = min_level>0 || max_level>=0;

        for ix in min_cell_x..max_cell_x + 1 {
            for iy in min_cell_y..max_cell_y + 1 {
                let v_cell  =&self.grid.grid[ix as usize][iy as usize];
                //const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];

                if v_cell.is_empty() {
                    continue;
                }

                for j in 0..v_cell.len() {
                    //TODO (Stereo) Need to update this if stereo images are processed
                    let (kp_un, is_right) = &self.get_keypoint(v_cell[j]);
                    if check_levels {
                        if kp_un.octave < min_level as i32 {
                            continue;
                        }
                        if max_level >= 0 {
                            if kp_un.octave > max_level as i32 {
                                continue;
                            }
                        }
                    }

                    let distx = kp_un.pt.x - (*x as f32);
                    let disty = kp_un.pt.y - (*y as f32);

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
        return x >= self.image_bounds.min_x && x < self.image_bounds.max_x && y >= self.image_bounds.min_y && y < self.image_bounds.max_y;
    }
}




#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct ImageBounds {
    pub min_x: f64,//static float mnMinX;
    pub max_x: f64,//static float mnMaxX;
    pub min_y: f64,//static float mnMinY;
    pub max_y: f64,//static float mnMaxY;
}

impl ImageBounds {
    pub fn new(im_width: i32, im_height: i32, dist_coef: &Option<Vec<f32>>) -> ImageBounds {
        //ComputeImageBounds
        let min_x = 0.0;
        let mut max_x = 0.0;
        let min_y = 0.0;
        let mut max_y = 0.0;

        match dist_coef {
            Some(vec) => {
                todo!("mid priority: implement code if dist_coef is non-zero");
                // cv::Mat mat(4,2,CV_32F);
                // mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
                // mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
                // mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
                // mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

                // mat=mat.reshape(2);
                // cv::undistortPoints(mat,mat,static_cast<Pinhole*>(mpCamera)->toK(),mDistCoef,cv::Mat(),mK);
                // mat=mat.reshape(1);

                // // Undistort corners
                // mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
                // mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
                // mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
                // mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));
            },
            None => {
                max_x = im_width as f64;
                max_y = im_height as f64;
            }
        }

        ImageBounds{ min_x, max_x, min_y, max_y }
    }

    pub fn check_bounds(&self, x: f64, y: f64) -> bool {
        x >= self.min_x && x < self.max_x && y >= self.min_y && y < self.max_y
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Grid {
    pub grid_element_width_inv: f64,
    pub grid_element_height_inv: f64,
    pub grid: Vec<Vec<Vec<usize>>> // mGrid
}
impl Grid {
    pub fn default(image_bounds: &ImageBounds) -> Self {
        Grid {
            grid_element_width_inv: FRAME_GRID_COLS as f64/(image_bounds.max_x - image_bounds.min_x) as f64,
            grid_element_height_inv: FRAME_GRID_ROWS as f64/(image_bounds.max_y - image_bounds.min_y) as f64,
            grid: Self::initialize_grid()
        }
    }

    pub fn empty() -> Self {
        Grid {
            grid_element_width_inv: 0.0,
            grid_element_height_inv: 0.0,
            grid: Self::initialize_grid()
        }
    }

    fn initialize_grid() -> Vec<Vec<Vec<usize>>> {
        let mut grid = Vec::new();
        for i in 0.. FRAME_GRID_COLS  {
            let mut row = Vec::new(); //vec![];
            for j in 0..FRAME_GRID_ROWS
            {
                row.push(Vec::new());
                
            }
            grid.push(row);
            // let row = vec![Vec::new(); FRAME_GRID_COLS];
            // grid.push(row);
        }
        //println!("Grid row col : {:?}, {:?}", FRAME_GRID_ROWS, FRAME_GRID_COLS);
        grid
    }

    pub fn assign_features(&mut self, image_bounds: &ImageBounds, keypoints_un: &DVVectorOfKeyPoint) {
        for i in 0..keypoints_un.len() as usize {
            let keypoint = &keypoints_un.get(i).unwrap();
            if let Some((pos_x, pos_y)) = self.pos_in_grid(&image_bounds, keypoint) { 
                self.grid[pos_x as usize][pos_y as usize].push(i);
            }
        }
    }

    pub fn pos_in_grid(&self, image_bounds: &ImageBounds, kp : &KeyPoint) -> Option<(i32, i32)> {
        let pos_x = ((kp.pt.x-(image_bounds.min_x as f32))*self.grid_element_width_inv as f32).round() as i32;
        let pos_y = ((kp.pt.y-(image_bounds.min_y as f32))*self.grid_element_height_inv as f32).round() as i32;

        //let x_in_bounds = pos_x >= 0 && pos_x < (FRAME_GRID_COLS as i32);
        //let y_in_bounds = pos_y >= 0 && pos_y < (FRAME_GRID_ROWS as i32);

        let not_in_bounds = pos_x<0 || pos_x>=FRAME_GRID_COLS as i32 || pos_y<0 || pos_y>=FRAME_GRID_ROWS as i32;

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if not_in_bounds
        {
            return None;
        }
        else
        {
            return Some((pos_x, pos_y));
        }
    }
}

// From implementations to make it easier to pass this into opencv functions
impl From<Grid> for dvos3binding::ffi::Grid {
    fn from(dvgrid: Grid) -> dvos3binding::ffi::Grid { 
        let mut grid = dvos3binding::ffi::Grid{vec: Vec::new()};

        for i in 0.. FRAME_GRID_COLS  {
            let mut row = dvos3binding::ffi::VectorOfVecusize{vec: Vec::new()};

            for j in 0..FRAME_GRID_ROWS
            {
                let mut col = dvos3binding::ffi::VectorOfusize{vec: Vec::new()};
                // Bug here ...dvgrid.grid[i][j].len() is 0

                for k in 0..dvgrid.grid[i][j].len()
                {
                    let val = dvgrid.grid[i][j][k];
                    col.vec.push(val);
                }

                row.vec.push(col);
            }
            grid.vec.push(row);

        }


        grid
    }
}



// For conversion to/from C++
// impl From<DVVectorOfKeyPoint> for CxxVector<dvos3binding::ffi::DVKeyPoint> {
//     fn from(vec: DVVectorOfKeyPoint) -> Self { 
//         *vec.into_raw() as *const CxxVector<dvos3binding::ffi::DVKeyPoint>
//     }
// }
