use std::collections::HashMap;
use array2d::Array2D;
use chrono::{DateTime, Utc};
use opencv::{core::KeyPoint,};
use abow::{BoW, Vocabulary, DirectIdx};

use crate::{
    map::{pose::Pose, map::Id}, 
    dvutils::*,
    map::misc::IMUBias
};

const FRAME_GRID_ROWS :i64 = 48;
const FRAME_GRID_COLS :i64 = 64;

#[derive(Debug, Clone)]
pub struct Frame {
    pub id: Id,
    pub pose: Option<Pose>,

    // Image //
    pub width: i32,
    pub height: i32,
    pub timestamp: DateTime<Utc>,
    // Undistorted Image Bounds (computed once).
    pub min_x: f64,//static float mnMinX;
    pub max_x: f64,//static float mnMaxX;
    pub min_y: f64,//static float mnMinY;
    pub max_y: f64,//static float mnMaxY;

    // Keypoints //
    pub keypoints: opencv::types::VectorOfKeyPoint, // == mvkeys
    pub keypoints_right: opencv::types::VectorOfKeyPoint, // == mvkeysright
    pub keypoints_un: opencv::types::VectorOfKeyPoint,
    pub mv_depth: HashMap<i32, f32>, // negative value for monocular points
    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    pub grid_element_width_inv: f64, //static float mfGridElementWidthInv;
    pub grid_element_height_inv: f64,//static float mfGridElementHeightInv;
    pub grid: Array2D<usize>,//std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Descriptors //
    pub descriptors: opencv::core::Mat,

    // Mappoints //
    pub mappoint_matches: HashMap::<i32, Id>, // == mvpmappoints ; i32 is index in array, Id is mappoint Id
    // Flag to identify outlier associations.
    pub mappoint_outliers: HashMap::<i32, bool>, // == mvbOutlier ; i32 is index in array, bool is true if outlier

    // IMU //
    pub imu_bias: Option<IMUBias>,
    pub reference_keyframe_id: Option<Id>,

    // Stereo //
    pub stereo_baseline: f64,

    // Idk where to put this
    // depth_threshold: u64,
    // cam_params: opencv::core::Mat,
    pub N: usize,
    pub Nleft: usize, // TODO (Stereo)
    pub mvInvLevelSigma2: Vec<f32>,
    pub dist_coef: Vec<f64>,//mDistCoef
    pub featvec: Option<(BoW, DirectIdx)>, //pub bow: Option<BoW>,
    pub scale_factors: Vec<f64>,
}

impl Frame {
    pub fn new(id: i32, timestamp: DateTime<Utc>, keypoints_vec: opencv::types::VectorOfKeyPoint, descriptors_vec: opencv::core::Mat, im_width: i32, im_height: i32) -> Self {
        let keypoints = keypoints_vec.cv_vector_of_keypoint();
        let descriptors = descriptors_vec.grayscale_to_cv_mat();

        let mut frame = Frame{
            id: id,
            timestamp: timestamp,
            keypoints: keypoints.clone(),
            keypoints_right: opencv::types::VectorOfKeyPoint::new(), //TODO (Stereo) this needs to be filled with actual keypoints
            descriptors: descriptors,
            pose: None,
            imu_bias: None,
            reference_keyframe_id: None,

            scale_factors: Vec::new(), // == mvScaleFactors
            mappoint_matches: HashMap::new(),
            mappoint_outliers: HashMap::new(),

            // depth_threshold:
            // cam_params: 
            keypoints_un: keypoints.clone(), //TODO : need to compute undistorted keypoints
            mv_depth: HashMap::new(),

            min_x: 0.0,
            max_x: 0.0,
            min_y: 0.0,
            max_y: 0.0,
            dist_coef: vec![0.0; 5],//mDistCoef, setting default to zeros

            // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
            grid_element_width_inv: 0.0,// FRAME_GRID_COLS as f64/(max_x-min_x) as f64, //static float mfGridElementWidthInv;
            //mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
            //mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);
            grid_element_height_inv: 0.0,//static float mfGridElementHeightInv;
            grid: Array2D::filled_with(0, FRAME_GRID_ROWS as usize, FRAME_GRID_COLS as usize),//std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
            width: im_width,
            height: im_height,
            featvec: None,
            mvInvLevelSigma2: vec![], //TODO: should be this: mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

            N: keypoints.len(), //TODO: [Stereo] Need to update this if stereo images are processed
            Nleft: 0, // TODO (STEREO) ... should be 0 if not stereo

            stereo_baseline: 0.0,
        };

        // frame.calculate_pose();
        frame.ComputeImageBounds();

        frame.grid_element_width_inv = FRAME_GRID_COLS as f64/(frame.max_x - frame.min_x) as f64;
        frame.grid_element_height_inv = FRAME_GRID_COLS as f64/(frame.max_y - frame.min_y) as f64;

        frame.assign_features_to_grid();

        frame
    }


    pub fn ComputeImageBounds(&mut self) {
        if self.dist_coef[0]!=0.0 {
            //TODO: implement code if dist_coef is non-zero
            todo!("implement code if dist_coef is non-zero");
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
        } else {
            self.min_x = 0.0;
            self.max_x = self.width as f64;
            self.min_y = 0.0;
            self.max_y = self.height as f64;
        }
    }

    pub fn GetFeaturesInArea(&self, x: &f64, y: &f64, r: &f64, min_level: &i64, max_level: &i64, right: bool) -> Vec<usize> {
        let grid_element_width_inv = FRAME_GRID_COLS as f64/(self.max_x - self.min_x) as f64;
        let grid_element_height_inv = FRAME_GRID_COLS as f64/(self.max_y - self.min_y) as f64;

        let mut indices = Vec::<usize>::new();
        indices.reserve(self.keypoints.len());

        let factorX = *r;
        let factorY = *r;

        let min_cell_x = i64::max(0, ((x-self.min_x-factorX)*grid_element_width_inv).floor() as i64);
        let max_cell_x = i64::min(FRAME_GRID_COLS-1, ((x-self.min_x+factorX)*grid_element_width_inv).ceil() as i64);
        let min_cell_y = i64::max(0, ((y-self.min_y-factorY)*grid_element_height_inv).floor() as i64);
        let max_cell_y = i64::min(FRAME_GRID_ROWS-1, ((y-self.min_y+factorY)*grid_element_height_inv).ceil() as i64);

        if min_cell_x >= FRAME_GRID_COLS || max_cell_x < 0 || min_cell_y >= FRAME_GRID_ROWS || max_cell_y < 0 {
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
                    //TODO: [Stereo] Need to update this if stereo images are processed
                    let kpUn = &self.keypoints_un.get(v_cell[j]).unwrap();
                    //const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[v_cell[j]]
                    //                                         : (!bRight) ? mvKeys[v_cell[j]]
                    //                                                     : mvKeysRight[v_cell[j]];
                    if b_check_levels {
                        if kpUn.octave< *min_level as i32 {
                            continue;
                        }
                        if *max_level>=0 {
                            if kpUn.octave> *max_level as i32 {
                                continue;
                            }
                        }
                    }

                    let distx = kpUn.pt.x- (*x as f32);
                    let disty = kpUn.pt.y- (*y as f32);

                    if distx.abs()<(factorX as f32) && disty.abs() < (factorY as f32)  {
                        indices.push(v_cell[j]);
                    }
                }
            }
        }
        return indices;
    }

    fn assign_features_to_grid(&mut self) {
        // Fill matrix with points
        for (index, mp_id) in &self.mappoint_matches {
            //TODO: [Stereo] Need to update this if stereo images are processed
            let kp = &self.keypoints_un.get(*index as usize).unwrap();
            // const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
            //                                          : (i < Nleft) ? mvKeys[i]
            //                                                          : mvKeysRight[i - Nleft]; 

            let (mut grid_pos_x, mut grid_pos_y) = (0i64,0i64);
            if self.pos_in_grid(kp,&mut grid_pos_x,&mut grid_pos_y) {
                self.grid[(grid_pos_x as usize, grid_pos_y as usize)] = *index as usize;

                //TODO: [Stereo] Need to update this if stereo images are processed
                // if(Nleft == -1 || i < Nleft)
                //     mGrid[grid_pos_x][grid_pos_y].push_back(i);
                // else
                //     mGridRight[grid_pos_x][grid_pos_y].push_back(i - Nleft);
            }
        }
    }

    pub fn pos_in_grid(&self, kp : &KeyPoint, posX: &mut i64, posY : &mut i64) -> bool {
        *posX = (kp.pt.x-(self.min_x as f32)*self.grid_element_width_inv as f32).round() as i64;
        *posY = (kp.pt.y-(self.min_y as f32)*self.grid_element_height_inv as f32).round() as i64;

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if *posX<0 || *posX>=FRAME_GRID_COLS || *posY<0 || *posY>=FRAME_GRID_ROWS {
            return false;
        }

        return true;
    }

    pub fn set_pose(&mut self, Tcw: &Pose) {
        self.pose = Some(Tcw.clone());

        //UpdatePoseMatrices();
        //mbIsSet = true;
        //mbHasPose = true;
    }

    pub fn get_pose(&self) -> Pose {
        self.pose.as_ref().unwrap().clone()
    }

    pub fn compute_BoW(&mut self) {
        //Ref code : https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Frame.cc#L740
        //todo!("Implement : ComputeBoW");
        if self.featvec.is_none() {
            let features = Converter::toDescriptorVector(&self.descriptors);
            let k=10;
            let l =4;
            let voc = Vocabulary::create(&features, k, l);
            self.featvec = Some(voc.transform_with_direct_idx(&features).unwrap());
        }
    }

    pub fn clear_mappoints(&mut self) {
        // TODO Sofiya ... move to map actor
        self.mappoint_matches = HashMap::new();
    }

    // Safe to call these (instead of going through map) because
    // only the tracking thread has these data structures
    // when these functions are called
    #[allow(non_snake_case)]
    pub fn clean_VO_matches(&self) {
        // TODO Sofiya ... move to map actor
        // For each mappoint in mvpmappoints vector, delete it if the num of observations is < 1
    }

    #[allow(non_snake_case)]
    pub fn delete_VO_matches_if_not_outliers(&self) {
        // TODO Sofiya ... move to map actor
        // for(int i=0; i<mCurrentFrame.N;i++)
        // {
        //     if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
        //         mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
        // }
    }

    pub fn get_mappoint_matches(&self) -> &HashMap<i32, Id> {
        &self.mappoint_matches
    }
}