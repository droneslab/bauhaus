use std::collections::HashMap;
use std::fmt::Debug;
use serde::{Deserialize, Serialize, de::DeserializeOwned};
use opencv::{core::KeyPoint};
use crate::{
    dvutils::{DVMatrix, DVVectorOfKeyPoint},
    map::{frame::ImageBounds, map::Id},
};

pub const FRAME_GRID_ROWS :usize = 48;
pub const FRAME_GRID_COLS :usize = 64;

#[derive(Debug, Clone, Serialize, Deserialize)]
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

// Sofiya note: there are a lot of extra variables that are set to 0 or null
// because they are only used in stereo or IMU cases. This encapsulates 
// "all keypoint data" into a struct that has different variables depending
// on the type of sensor, and then implements functions on that struct so we 
// don't have to think about copying the logic of interacting with keypoints
// each time.
pub trait KeyPointsData: Serialize + DeserializeOwned + Send + Sync + Clone + Debug {
    // Constructors
    fn new(keypoints: &DVVectorOfKeyPoint, descriptors: &DVMatrix, image_bounds: &ImageBounds) -> Self;
    fn empty() -> Self;

    // Getters ... needed because KeyPointsData has no associated data
    // so anything that uses a KeyPointsData object doesn't know
    // about vars in the struct.
    fn grid(&self) -> &Grid;
    fn keypoints_un(&self) -> &DVVectorOfKeyPoint;
    fn descriptors(&self) -> &DVMatrix;
    fn num_keypoints(&self) -> i32;
    fn mv_depth_get(&self, i: &u32) -> Option<&f32>;
    fn check_close_tracked_mappoints(
        &self,
        th_depth: f32,
        mappoint_matches: &HashMap::<u32, Id>,
        mappoint_outliers: &HashMap::<u32, bool>
    ) -> (i32, i32);

    // Modifiers
    fn assign_features_to_grid(&mut self, image_bounds: &ImageBounds, mappoint_matches: &HashMap::<u32, Id>);
}

//* Mono *//
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct KeyPointsMono {
    pub num_keypoints: i32,

    pub keypoints: DVVectorOfKeyPoint,
    pub keypoints_un: DVVectorOfKeyPoint,

    pub descriptors: DVMatrix,

    pub grid: Grid,

    // Note: empty/not set in the mono case
    // mvuright, mvdepth, mnCloseMPs
    // Nleft, Nright
    // mvLeftToRightMatch, mvRightToLeftMatch
    // mvStereo3Dpoints
    // monoLeft, monoRight
}
impl KeyPointsData for KeyPointsMono {
    fn empty() -> Self {
        todo!("empty");
    }
    fn new(
        keypoints: &DVVectorOfKeyPoint,
        descriptors: &DVMatrix,
        image_bounds: &ImageBounds
    ) -> Self {
        KeyPointsMono {
            num_keypoints: keypoints.len(),
            keypoints: keypoints.clone(), 
            keypoints_un: keypoints.clone(), //TODO : need to compute undistorted keypoints
            descriptors: descriptors.clone(),
            grid: Grid::default(&image_bounds)
        }
    }
    fn grid(&self) -> &Grid { & self.grid }
    fn keypoints_un(&self) -> &DVVectorOfKeyPoint { & self.keypoints_un }
    fn descriptors(&self) -> &DVMatrix { & self.descriptors }
    fn num_keypoints(&self) -> i32 { self.num_keypoints }
    fn mv_depth_get(&self, i: &u32) -> Option<&f32> { None }
    fn check_close_tracked_mappoints(
        &self,
        th_depth: f32,
        mappoint_matches: &HashMap::<u32, Id>,
        mappoint_outliers: &HashMap::<u32, bool>
    ) -> (i32, i32) { (0, 0) }

    fn assign_features_to_grid(&mut self, image_bounds: &ImageBounds, mappoint_matches: &HashMap::<u32, Id>) {
        for (index, mp_id) in mappoint_matches {
            let kp = &self.keypoints_un.get(*index as usize).unwrap();
            let pos_in_grid = self.grid.pos_in_grid(image_bounds, kp);
            match pos_in_grid {
                Some((pos_x, pos_y)) => self.grid.grid[pos_x as usize][pos_y as usize] = *index as usize,
                None => {}
            }
        }
    }
}

//* Stereo *//
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct KeyPointsStereo {
    pub num_keypoints: i32, // N = Nleft + Nright
    pub num_keypoints_left: i32, // Nleft
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
    fn new(keypoints: &DVVectorOfKeyPoint, descriptors: &DVMatrix, image_bounds: &ImageBounds) -> Self {
        todo!("empty");
    }
    fn empty() -> Self {
        todo!("empty");
    }
    fn keypoints_un(&self) -> &DVVectorOfKeyPoint {
        todo!("stereo");
    }
    fn descriptors(&self) -> &DVMatrix {
        todo!("stereo");
    }
    fn num_keypoints(&self) -> i32 {
        todo!("stereo");
    }
    fn grid(&self) -> &Grid { & self.grid }
    fn mv_depth_get(&self, i: &u32) -> Option<&f32> { self.mv_depth.get(i) }
    fn check_close_tracked_mappoints(
        &self,
        th_depth: f32,
        mappoint_matches: &HashMap::<u32, Id>,
        mappoint_outliers: &HashMap::<u32, bool>
    ) -> (i32, i32) {
        let (mut tracked_close, mut non_tracked_close) = (0, 0);
        for i in 0..self.num_keypoints as u32 {
            match self.mv_depth.get(&i) {
                Some(value) => {
                    let depth = *value;
                    if depth > 0.0 && depth < th_depth {
                        if mappoint_matches.contains_key(&i) && !mappoint_outliers.contains_key(&i) {
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

    fn assign_features_to_grid(&mut self, image_bounds: &ImageBounds, mappoint_matches: &HashMap::<u32, Id>) {
        todo!("stereo");
        // for (index, mp_id) in mappoint_matches {
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
}

//* RGBD *//
#[derive(Serialize, Deserialize, Clone, Debug)]
pub struct KeyPointsRgbd {
    pub num_keypoints: i32,

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    pub keypoints: DVVectorOfKeyPoint,
    pub keypoints_right: DVVectorOfKeyPoint,
    pub keypoints_un: DVVectorOfKeyPoint,

    pub descriptors: DVMatrix,

    pub grid: Grid,

    pub mv_right: HashMap<u32, f32>,
    pub mv_depth: HashMap<u32, f32>,

    // Note: empty/not set in the RGBD case
    // num_keypoints_left, num_keypoints_right
    // mvLeftToRightMatch, mvRightToLeftMatch
    // mvStereo3Dpoints
    // monoLeft, monoRight
}
impl KeyPointsData for KeyPointsRgbd {
    fn new(keypoints: &DVVectorOfKeyPoint, descriptors: &DVMatrix, image_bounds: &ImageBounds) -> Self {
        todo!("rgbd");
    }
    fn empty() -> Self {
        todo!("rgbd");
    }
    fn grid(&self) -> &Grid { & self.grid }
    fn keypoints_un(&self) -> &DVVectorOfKeyPoint {
        todo!("rgbd");
    }
    fn descriptors(&self) -> &DVMatrix {
        todo!("rgbd");
    }
    fn num_keypoints(&self) -> i32 {
        todo!("rgbd");
    }
    fn mv_depth_get(&self, i: &u32) -> Option<&f32> { self.mv_depth.get(i) }
    fn check_close_tracked_mappoints(
        &self,
        th_depth: f32,
        mappoint_matches: &HashMap::<u32, Id>,
        mappoint_outliers: &HashMap::<u32, bool>
    ) -> (i32, i32) {
        let (mut tracked_close, mut non_tracked_close) = (0, 0);
        for i in 0..self.num_keypoints as u32 {
            match self.mv_depth.get(&i) {
                Some(value) => {
                    let depth = *value;
                    if depth > 0.0 && depth < th_depth {
                        if mappoint_matches.contains_key(&i) && !mappoint_outliers.contains_key(&i) {
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

    fn assign_features_to_grid(&mut self, image_bounds: &ImageBounds, mappoint_matches: &HashMap::<u32, Id>) {
        for (index, mp_id) in mappoint_matches {
            let kp = &self.keypoints_un.get(*index as usize).unwrap();
            let pos_in_grid = self.grid.pos_in_grid(image_bounds, kp);
            match pos_in_grid {
                Some((pos_x, pos_y)) => self.grid.grid[pos_x as usize][pos_y as usize] = *index as usize,
                None => {}
            }
        }
    }
}
