// Sofiya: Don't delete this for now, trying something new out with keypoints

use std::collections::{HashMap};
use std::fmt::Debug;
use std::marker::PhantomData;
use dvcore::global_params::Sensor::*;
use opencv::prelude::{Mat, MatTraitConst, MatTrait};
use opencv::types::{VectorOff32};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use opencv::core::{KeyPoint, CV_32F, Scalar};
use crate::{
    matrix::{DVMatrix, DVVectorOfKeyPoint},
    dvmap::{frame::ImageBounds, map::Id}, utils::camera::Camera,
};

use super::sensor::SensorType;

pub const FRAME_GRID_ROWS :usize = 48;
pub const FRAME_GRID_COLS :usize = 64;

#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct KeyPoints<S> {
    // Common across all sensor types:
    pub num_keypoints: u32,
    keypoints: DVVectorOfKeyPoint, // Vector of keypoints (original for visualization).
    descriptors: DVMatrix, // mDescriptors
    grid: Grid, // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.

    // Mono and RGBD only:
    keypoints_un: Option<DVVectorOfKeyPoint>, // Undistorted keypoints actually used by the system. For stereo, this is redundant bc images must be rectified

    // Stereo only:
    keypoints_left_cutoff: u32, // Nleft, if stereo and index passed in is < keypoints_left_cutoff, need to get keypoint from keypoints_right instead of keypoints
    keypoints_right: Option<DVVectorOfKeyPoint>,
    mv_right: Option<HashMap<u32, f32>>, // mvuRight

    // Stereo and RGBD only:
    mv_depth: Option<HashMap<u32, f32>>, //mvDepth

    sensor: PhantomData<S>,
}

impl<S: SensorType> KeyPoints<S> {
    pub fn new<S2: SensorType>(
        keypoints: &DVVectorOfKeyPoint,
        descriptors: &DVMatrix,
        image_bounds: &ImageBounds,
        camera: &Camera
    ) -> Result<KeyPoints::<S2>, Box<dyn std::error::Error>> {
        match S::sensor_type() {
            Mono | ImuMono => {
                let mut kpdata = KeyPoints::<S2> {
                    num_keypoints: keypoints.len() as u32,
                    keypoints: keypoints.clone(), 
                    keypoints_un: Some(Self::undistort_keypoints(keypoints, camera)?),
                    descriptors: descriptors.clone(),
                    grid: Grid::default(&image_bounds),
                    keypoints_right: None,
                    keypoints_left_cutoff: 0,
                    mv_right: None,
                    mv_depth: None,
                    sensor: PhantomData,
                };
                // assign features to grid
                for i in 0..kpdata.num_keypoints as usize {
                    let kp = &kpdata.keypoints_un.unwrap().get(i).unwrap();
                    let pos_in_grid = kpdata.grid.pos_in_grid(&image_bounds, kp);
                    match pos_in_grid {
                        Some((pos_x, pos_y)) => kpdata.grid.grid[pos_x as usize][pos_y as usize] = i,
                        None => {}
                    }
                }
                Ok::<KeyPoints<S2>, Box<dyn std::error::Error>>(kpdata)
            },
            Rgbd | ImuRgbd => {
                todo!("rgbd");
                let mut kpdata = KeyPoints::<S2> {
                    num_keypoints: keypoints.len() as u32,
                    keypoints: keypoints.clone(), 
                    keypoints_un: Some(Self::undistort_keypoints(keypoints, camera)?),
                    descriptors: descriptors.clone(),
                    grid: Grid::default(&image_bounds),
                    keypoints_right: todo!(),
                    keypoints_left_cutoff: 0,
                    mv_right: todo!(),
                    mv_depth: todo!(),
                    sensor: todo!(),
                };
                // assign features to grid
                // uncomment when this section is done
                // for i in 0..self.num_keypoints() as usize {
                //     let kp = &self.keypoints_un.get(i).unwrap();
                //     let pos_in_grid = self.grid.pos_in_grid(image_bounds, kp);
                //     match pos_in_grid {
                //         Some((pos_x, pos_y)) => self.grid.grid[pos_x as usize][pos_y as usize] = i,
                //         None => {}
                //     }
                // }
                Ok::<KeyPoints<S2>, Box<dyn std::error::Error>>(kpdata)
            },
            Stereo | ImuStereo => {
                todo!("stereo");
                let mut kpdata = KeyPoints::<S2> {
                    num_keypoints: keypoints.len() as u32,
                    keypoints: keypoints.clone(), 
                    keypoints_un: None,
                    descriptors: descriptors.clone(),
                    grid: Grid::default(&image_bounds),
                    keypoints_right: todo!(),
                    keypoints_left_cutoff: 0,
                    mv_right: todo!(),
                    mv_depth: todo!(),
                    sensor: todo!(),
                };
                // assign features to grid
                // for i in 0..self.num_keypoints() as usize {
                //TODO: [Stereo] Need to update this if stereo images are processed
                // let kp = &self.keypoints_un.get(*index as usize).unwrap();
                // const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
                //                                          : (i < Nleft) ? mvKeys[i]
                //                                                          : mvKeysRight[i - Nleft]; 

                // let (mut grid_pos_x, mut grid_pos_y) = (0i64,0i64);
                // if self.grid.pos_in_grid(kp,&mut grid_pos_x,&mut grid_pos_y) {
                    // self.grid.grid[grid_pos_x as usize][grid_pos_y as usize] = *index;

                    //TODO: [Stereo] Need to update this if stereo images are processed
                    // if(Nleft == -1 || i < Nleft)
                    //     mGrid[grid_pos_x][grid_pos_y].push_back(i);
                    // else
                    //     mGridRight[grid_pos_x][grid_pos_y].push_back(i - Nleft);
                // }
                Ok::<KeyPoints<S2>, Box<dyn std::error::Error>>(kpdata)
            }

        }
    }

    pub fn descriptors(&self) -> &DVMatrix {
        &self.descriptors
    }

    pub fn grid(&self) -> &Grid {
        &self.grid
    }

    pub fn keypoints_get(&self, index: usize) -> &KeyPoint {
        match S::sensor_type() {
            Mono | ImuMono => &self.keypoints_un.unwrap().get(index).unwrap() ,
            _ => match index < self.keypoints_left_cutoff as usize {
                true => &self.keypoints.get(index).unwrap(),
                false => &self.keypoints_right.unwrap().get(index).unwrap()
            }
        }
    }

    pub fn mv_depth_get(&self, i: &u32) -> Option<&f32> {
        self.mv_depth.unwrap().get(i)
    }

    pub fn get_octave(&self, index: usize) -> i32 {
        self.keypoints_get(index).octave
    }


    pub fn check_close_tracked_mappoints( &self, th_depth: f32, mappoint_matches: &HashMap::<u32, (Id, bool)> ) -> (i32, i32) {
        match S::sensor_type() {
            Mono | ImuMono => (0,0),
            _ => {
                let (mut tracked_close, mut non_tracked_close) = (0, 0);
                for i in 0..self.num_keypoints() as u32 {
                    match self.mv_depth.unwrap().get(&i) {
                        Some(value) => {
                            let depth = *value;
                            if depth > 0.0 && depth < th_depth {
                                let mp = mappoint_matches.get(&i);
                                if mp.is_some() && !mp.unwrap().1 {
                                    tracked_close += 1;
                                } else {
                                    non_tracked_close += 1;
                                }
                            }
                        },
                        None => {}
                    }
                }
                (tracked_close, non_tracked_close) 
            }
        }
    }

    fn undistort_keypoints(keypoints: &DVVectorOfKeyPoint, camera: &Camera) -> Result<DVVectorOfKeyPoint, Box<dyn std::error::Error>> {
        // Sofiya: I am not sure I did this correctly
        if camera.dist_coef.as_ref().unwrap()[0] == 0.0 {
            return Ok(keypoints.clone());
        }

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
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, Default)]
pub struct Grid {
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

    pub fn empty() -> Self {
        Grid {
            grid_element_width_inv: 0.0,
            grid_element_height_inv: 0.0,
            grid: Self::initialize_grid()
        }
    }

    fn initialize_grid() -> Vec<Vec<usize>> {
        let mut grid = Vec::new();
        for r in 0..FRAME_GRID_ROWS {
            let row = vec![0; FRAME_GRID_COLS];
            grid.push(row);
        }
        grid
    }

    pub fn pos_in_grid(&self, image_bounds: &ImageBounds, kp : &KeyPoint) -> Option<(i32, i32)> {
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


