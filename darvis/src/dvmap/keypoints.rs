// There are a lot of extra variables that are set to 0 or null
// because they are only used in stereo or IMU cases. This encapsulates 
// "all keypoint data" into a struct that has different variables depending
// on the type of sensor, and then implements functions on that struct so we 
// don't have to think about copying the logic of interacting with keypoints
// each time.

use std::collections::{HashMap, HashSet};
use std::fmt::Debug;
use opencv::prelude::{Mat, MatTraitConst, MatTrait};
use opencv::types::{VectorOfPoint2f, VectorOff32};
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use opencv::core::{KeyPoint, Point2f, CV_32F, Scalar};
use crate::{
    matrix::{DVMatrix, DVVectorOfKeyPoint},
    dvmap::{frame::ImageBounds, map::Id}, modules::camera::Camera,
};

pub const FRAME_GRID_ROWS :usize = 48;
pub const FRAME_GRID_COLS :usize = 64;


pub trait KeyPointsData: Serialize + DeserializeOwned + Send + Sync + Clone + Debug + Default {
    // Constructors
    fn new(keypoints: &DVVectorOfKeyPoint, descriptors: &DVMatrix, image_bounds: &ImageBounds, camera: &Camera) -> Result<Self, Box<dyn std::error::Error>>;

    // Getters ... needed because KeyPointsData has no associated data
    // so anything that uses a KeyPointsData object doesn't know
    // about vars in the struct.
    fn grid(&self) -> &Grid;
    fn keypoints_get(&self, index: usize) -> KeyPoint;
    fn descriptors(&self) -> &DVMatrix;
    fn num_keypoints(&self) -> i32;
    fn num_keypoints_left(&self) -> u32;
    fn get_octave(&self, index: &u32) -> i32;
    fn check_close_tracked_mappoints(
        &self,
        th_depth: f32,
        mappoint_matches: &HashMap::<u32, (Id, bool)>,
    ) -> (i32, i32);
}

//* Mono *//
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct KeyPointsMono {
    pub num_keypoints: i32,

    keypoints: DVVectorOfKeyPoint,
    keypoints_un: DVVectorOfKeyPoint,

    descriptors: DVMatrix,

    grid: Grid,

    // Note: empty/not set in the mono case
    // mvuright, mvdepth, mnCloseMPs
    // Nleft, Nright
    // mvLeftToRightMatch, mvRightToLeftMatch
    // mvStereo3Dpoints
    // monoLeft, monoRight
}
impl KeyPointsData for KeyPointsMono {
    fn new(
        keypoints: &DVVectorOfKeyPoint,
        descriptors: &DVMatrix,
        image_bounds: &ImageBounds,
        camera: &Camera
    ) -> Result<Self, Box<dyn std::error::Error>>  {
        let mut kpdata = KeyPointsMono {
            num_keypoints: keypoints.len(),
            keypoints: keypoints.clone(), 
            keypoints_un: undistort_keypoints(keypoints, camera)?,
            descriptors: descriptors.clone(),
            grid: Grid::default(&image_bounds)
        };

        // assign features to grid
        for i in 0..kpdata.num_keypoints() as usize {
            let kp = &kpdata.keypoints_un.get(i).unwrap();
            let pos_in_grid = kpdata.grid.pos_in_grid(&image_bounds, kp);
            match pos_in_grid {
                Some((pos_x, pos_y)) => kpdata.grid.grid[pos_x as usize][pos_y as usize] = i,
                None => {}
            }
        }

        Ok(kpdata)
    }
    fn grid(&self) -> &Grid { & self.grid }
    fn keypoints_get(&self, index: usize) -> KeyPoint {
        self.keypoints_un.get(index).unwrap()
    }
    fn descriptors(&self) -> &DVMatrix { & self.descriptors }
    fn num_keypoints(&self) -> i32 { self.num_keypoints }
    fn num_keypoints_left(&self) -> u32 { 0 }
    fn get_octave(&self, index: &u32) -> i32 { self.keypoints_un.get(*index as usize).unwrap().octave }
    fn check_close_tracked_mappoints(
        &self,
        th_depth: f32,
        mappoint_matches: &HashMap::<u32, (Id, bool)>,
    ) -> (i32, i32) { (0, 0) }
}

//* Stereo *//
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct KeyPointsStereo {
    pub num_keypoints: i32, // N = Nleft + Nright
    pub num_keypoints_left: u32, // Nleft
    pub num_keypoints_right: i32, // Nright

    // Vector of keypoints (original for visualization)
    pub keypoints: DVVectorOfKeyPoint, // mvkeys
    pub keypoints_right: DVVectorOfKeyPoint, // mvkeysright

    pub descriptors: DVMatrix, // mDescriptors

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    pub grid: Grid,

    pub mv_right: HashMap<u32, f32>, // mvuRight
    pub mv_depth: HashMap<u32, f32>, // mvDepth

    // Not set in the stereo case
    // keypoints_un is redundant as images must be rectified
}
impl KeyPointsData for KeyPointsStereo {
    fn new(keypoints: &DVVectorOfKeyPoint, descriptors: &DVMatrix, image_bounds: &ImageBounds, camera: &Camera) -> Result<Self, Box<dyn std::error::Error>> {
        todo!("stereo");
        // assign features to grid
        // uncomment when this section is done
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
        // }
    }
    fn keypoints_get(&self, index: usize) -> KeyPoint {
       match index < self.num_keypoints_left as usize {
            true => self.keypoints.get(index).unwrap(),
            false => self.keypoints_right.get(index).unwrap()
        }
    }
    fn descriptors(&self) -> &DVMatrix {
        todo!("stereo");
    }
    fn num_keypoints(&self) -> i32 {
        todo!("stereo");
    }
    fn num_keypoints_left(&self) -> u32 { self.num_keypoints_left }

    fn grid(&self) -> &Grid { & self.grid }
    fn get_octave(&self, index: &u32) -> i32 { self.keypoints_right.get(*index as usize).unwrap().octave }

    fn check_close_tracked_mappoints(
        &self,
        th_depth: f32,
        mappoint_matches: &HashMap::<u32, (Id, bool)>,
    ) -> (i32, i32) {
        let (mut tracked_close, mut non_tracked_close) = (0, 0);
        for i in 0..self.num_keypoints as u32 {
            match self.mv_depth.get(&i) {
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

//* RGBD *//
#[derive(Serialize, Deserialize, Clone, Debug, Default)]
pub struct KeyPointsRgbd {
    num_keypoints: i32,
    num_keypoints_left: i32,

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    keypoints: DVVectorOfKeyPoint,
    keypoints_right: DVVectorOfKeyPoint,
    keypoints_un: DVVectorOfKeyPoint,

    descriptors: DVMatrix,

    grid: Grid,

    mv_right: HashMap<u32, f32>,
    mv_depth: HashMap<u32, f32>,

    // Note: empty/not set in the RGBD case
    // num_keypoints_left, num_keypoints_right
    // mvLeftToRightMatch, mvRightToLeftMatch
    // mvStereo3Dpoints
    // monoLeft, monoRight
}
impl KeyPointsData for KeyPointsRgbd {
    fn new(keypoints: &DVVectorOfKeyPoint, descriptors: &DVMatrix, image_bounds: &ImageBounds, camera: &Camera) -> Result<Self, Box<dyn std::error::Error>> {
        todo!("rgbd");
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
    }
    fn grid(&self) -> &Grid { & self.grid }
    fn keypoints_get(&self, index: usize) -> KeyPoint {
       match index < self.num_keypoints_left as usize {
            true => self.keypoints.get(index).unwrap(),
            false => self.keypoints_right.get(index).unwrap()
        }
    }
    fn descriptors(&self) -> &DVMatrix {
        todo!("rgbd");
    }
    fn num_keypoints(&self) -> i32 {
        todo!("rgbd");
    }
    fn num_keypoints_left(&self) -> u32 { 0 }
    fn get_octave(&self, index: &u32) -> i32 { self.keypoints_un.get(*index as usize).unwrap().octave }
    fn check_close_tracked_mappoints(
        &self,
        th_depth: f32,
        mappoint_matches: &HashMap::<u32, (Id, bool)>,
    ) -> (i32, i32) {
        let (mut tracked_close, mut non_tracked_close) = (0, 0);
        for i in 0..self.num_keypoints as u32 {
            match self.mv_depth.get(&i) {
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