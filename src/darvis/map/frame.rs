use std::collections::HashMap;

use chrono::{DateTime, Utc};

use crate::{
    map::{pose::Pose, map::Id}, 
    dvutils::*
};

use opencv::{
    prelude::*, core::KeyPoint,
};

use super::{misc::IMUBias, mappoint::MapPoint};

unsafe impl Sync for Frame {}

const FRAME_GRID_ROWS :i64 = 48;
const FRAME_GRID_COLS :i64 = 64;

use abow::{BoW, Vocabulary, DirectIdx};

#[derive(Debug, Clone)]
pub struct Frame {
    pub id: Id,
    pub timestamp: DateTime<Utc>,
    pub key_points: opencv::types::VectorOfKeyPoint,
    pub descriptors: opencv::core::Mat,
    pub pose: Option<Pose>,
    pub imu_bias: Option<IMUBias>,
    pub reference_keyframe_id: Option<Id>,
    // depth_threshold: u64,
    // cam_params: opencv::core::Mat,
    pub key_points_un: opencv::types::VectorOfKeyPoint,

    // equivalent to mvpmappoints in orbslam3
    // i32 is index in array, Id is mappoint Id
    pub mappoint_matches: HashMap::<i32, Id>,

    // Flag to identify outlier associations.
    // equivalent to mvbOutlier in orbslam3
    // i32 is index in array, bool is true if outlier
    pub mappoint_outliers: HashMap::<i32, bool>,

    pub N: usize,
    pub mvInvLevelSigma2: Vec<f32>,

    // Undistorted Image Bounds (computed once).
    pub min_x: f64,//static float mnMinX;
    pub max_x: f64,//static float mnMaxX;
    pub min_y: f64,//static float mnMinY;
    pub max_y: f64,//static float mnMaxY;

    pub dist_coef: Vec<f64>,//mDistCoef

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    pub grid_element_width_inv: f64, //static float mfGridElementWidthInv;
    pub grid_element_height_inv: f64,//static float mfGridElementHeightInv;
    pub grid: Vec<Vec<Vec<usize>>>,//std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    pub width: i32,
    pub height: i32,

    
    pub featvec: Option<(BoW, DirectIdx)>, //pub bow: Option<BoW>,

    // Corresponding stereo coordinate and depth for each keypoint.
    pub  mvpMapPoints:  Vec<Id> ,   //std::vector<MapPoint*> mvpMapPoints;

    // MapPoints associated to keypoints, NULL pointer if no association.
    // Flag to identify outlier associations.
    pub mvbOutlier: Vec<bool>,//std::vector<bool> mvbOutlier;
}

impl Frame {
    pub fn new(id: i32, timestamp: DateTime<Utc>, keypoints_vec: opencv::types::VectorOfKeyPoint, descriptors_vec: opencv::core::Mat, im_width: i32, im_height: i32) -> Self {
        let keypoints = keypoints_vec.cv_vector_of_keypoint();
        let descriptors = descriptors_vec.grayscale_to_cv_mat();

        
        let mut frame = Frame{
            id: id,
            timestamp: timestamp,
            key_points: keypoints.clone(),
            descriptors: descriptors,
            pose: None,
            imu_bias: None,
            reference_keyframe_id: None,

            mappoint_matches: HashMap::new(),
            mappoint_outliers: HashMap::new(),

            // depth_threshold:
            // cam_params: 
            key_points_un: keypoints.clone(), //TODO : need to compute undistorted keypoints

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
            grid: Vec::new(),//std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
            width: im_width,
            height: im_height,
            featvec: None,
            mvInvLevelSigma2: vec![], //TODO: should be this: mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();
        };

        // frame.calculate_pose();
        frame.ComputeImageBounds();


        frame.grid_element_width_inv = FRAME_GRID_COLS as f64/(frame.max_x - frame.min_x) as f64;
        frame.grid_element_height_inv = FRAME_GRID_COLS as f64/(frame.max_y - frame.min_y) as f64;

        frame.assign_features_to_grid();


        frame
    }


    pub fn ComputeImageBounds(&mut self) //, im_left: Mat)
    {

        if self.dist_coef[0]!=0.0
        {
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
        }
        else
        {
            self.min_x = 0.0;
            self.max_x = self.width as f64;
            self.min_y = 0.0;
            self.max_y = self.height as f64;
        }
    }



    pub fn GetFeaturesInArea(&self, x: &f64, y: &f64, r: &f64, min_level: &i64, max_level: &i64, right: bool) -> Vec<usize>
    {

        let grid_element_width_inv = FRAME_GRID_COLS as f64/(self.max_x - self.min_x) as f64;

        let grid_element_height_inv = FRAME_GRID_COLS as f64/(self.max_y - self.min_y) as f64;


        let mut indices = Vec::<usize>::new();
        indices.reserve(self.key_points.len());
    
        let factorX = *r;
        let factorY = *r;
        
        let min_cell_x = i64::max(0, ((x-self.min_x-factorX)*grid_element_width_inv).floor() as i64);

        if min_cell_x>=FRAME_GRID_COLS
        {
            return indices;
        }
        
        let max_cell_x = i64::min(FRAME_GRID_COLS-1, ((x-self.min_x+factorX)*grid_element_width_inv).ceil() as i64);

        if max_cell_x<0
        {
            return indices;
        }
    
        let min_cell_y = i64::max(0, ((y-self.min_y-factorY)*grid_element_height_inv).floor() as i64);
        
        if min_cell_y>=FRAME_GRID_ROWS
        {
            return indices;
        }
    
        let max_cell_y = i64::min(FRAME_GRID_ROWS-1, ((y-self.min_y+factorY)*grid_element_height_inv).ceil() as i64);
        if max_cell_y<0
        {
            return indices;
        }
    
        let b_check_levels = *min_level>0 || *max_level>=0;
    
        for  ix in min_cell_x..max_cell_x+1
        {
            for iy in min_cell_y..max_cell_y+1
            {
                //todo!("mGrid implementatiion");
                let v_cell = Vec::<usize>::new();
                //const vector<size_t> vCell = (!bRight) ? mGrid[ix][iy] : mGridRight[ix][iy];

                if v_cell.is_empty()
                {
                    continue;
                }
                    
                for j in 0..v_cell.len()
                {
                
                    //TODO: [Stereo] Need to update this if stereo images are processed
                    let kpUn = &self.key_points_un.get(v_cell[j]).unwrap();
                    //const cv::KeyPoint &kpUn = (Nleft == -1) ? mvKeysUn[v_cell[j]]
                    //                                         : (!bRight) ? mvKeys[v_cell[j]]
                    //                                                     : mvKeysRight[v_cell[j]];
                    if b_check_levels
                    {
                        if kpUn.octave< *min_level as i32
                        {
                            continue;
                        }
                            
                        if *max_level>=0
                        {
                            if kpUn.octave> *max_level as i32
                            {
                                continue;
                            }
                        }

                    }
    
                    let distx = kpUn.pt.x- (*x as f32);
                    let disty = kpUn.pt.y- (*y as f32);
    
                    if distx.abs()<(factorX as f32) && disty.abs() < (factorY as f32) 
                    {
                        indices.push(v_cell[j]);
                    }
                        
                }
            }
        }
    
        return indices;
    }

    //AssignFeaturesToGrid
    pub fn assign_features_to_grid(&mut self)
    {
        // Fill matrix with points
        let n_cells = FRAME_GRID_COLS*FRAME_GRID_ROWS;
        
        self.N = self.key_points.len() ; //TODO: [Stereo] Need to update this if stereo images are processed
        let nReserve = ((0.5 *self.N as f64)/(n_cells as f64)) as usize;
    
        for i in 0..FRAME_GRID_COLS as usize
        {
            for j in 0..FRAME_GRID_ROWS as usize
            {
                self.grid[i][j].reserve(nReserve);
                //TODO: [Stereo] Need to update this if stereo images are processed
                // if(Nleft != -1){
                //     mGridRight[i][j].reserve(nReserve);
                // }                
            }
        }

        for i in 0..self.N
        {    

            //TODO: [Stereo] Need to update this if stereo images are processed
            let kp = &self.key_points_un.get(i).unwrap();
            // const cv::KeyPoint &kp = (Nleft == -1) ? mvKeysUn[i]
            //                                          : (i < Nleft) ? mvKeys[i]
            //                                                          : mvKeysRight[i - Nleft]; 

            let (mut grid_pos_x, mut grid_pos_y) = (0i64,0i64);
            if self.pos_in_grid(kp,&mut grid_pos_x,&mut grid_pos_y)
            {
                self.grid[grid_pos_x as usize][grid_pos_y as usize].push(i);

                //TODO: [Stereo] Need to update this if stereo images are processed
                // if(Nleft == -1 || i < Nleft)
                //     mGrid[grid_pos_x][grid_pos_y].push_back(i);
                // else
                //     mGridRight[grid_pos_x][grid_pos_y].push_back(i - Nleft);
            }
        }
    
    }


    //bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    pub fn pos_in_grid(&self, kp : &KeyPoint, posX: &mut i64, posY : &mut i64) -> bool
    {

        *posX = (kp.pt.x-(self.min_x as f32)*self.grid_element_width_inv as f32).round() as i64;
        *posY = (kp.pt.y-(self.min_y as f32)*self.grid_element_height_inv as f32).round() as i64;

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if *posX<0 || *posX>=FRAME_GRID_COLS || *posY<0 || *posY>=FRAME_GRID_ROWS
        {
            return false;
        }
            
        return true;
    }


    //void Frame::SetPose(const Sophus::SE3<float> &Tcw) 
    pub fn SetPose(&mut self, Tcw: &Pose)
    {
        self.pose = Some(Tcw.clone());
    
        //UpdatePoseMatrices();
        //mbIsSet = true;
        //mbHasPose = true;
    }

<<<<<<< HEAD
    pub fn GetPose(&self) -> Pose
    {
        self.pose.as_ref().unwrap().clone()
    }

    pub fn ComputeBoW(&mut self)
    {

        //Ref code : https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/src/Frame.cc#L740
        //todo!("Implement : ComputeBoW");
        if self.featvec.is_none()
        {
            let features = Converter::toDescriptorVector(&self.descriptors);
            let k=10;
            let l =4;
            let voc = Vocabulary::create(&features, k, l);
            self.featvec = Some(voc.transform_with_direct_idx(&features).unwrap());
            
        }   


=======
    pub fn clear_mappoints(&mut self) {
        self.mappoint_matches = HashMap::new();
>>>>>>> Should be done with track with motion model, new dependency on g2orust bindings crate
    }


    // TODO: These functions moved out of tracking, but need to figure out what should be here and what should go back in tracking
    // Need to compare this current method of tracking to ORBSLAM3, probably the track reference keyframe function?

    // fn calculate_pose(&self, r: &Mat, t: &Mat) -> Pose {
    //     let mut pose = Pose::default();
    //     pose.pos[0] = *t.at::<f64>(0).unwrap();
    //     pose.pos[1] = *t.at::<f64>(1).unwrap();
    //     pose.pos[2] = *t.at::<f64>(2).unwrap();

    //     pose.rot[(0,0)] = *r.at_2d::<f64>(0,0).unwrap();
    //     pose.rot[(0,1)] = *r.at_2d::<f64>(0,1).unwrap();
    //     pose.rot[(0,2)] = *r.at_2d::<f64>(0,2).unwrap();
    //     pose.rot[(1,0)] = *r.at_2d::<f64>(1,0).unwrap();
    //     pose.rot[(1,1)] = *r.at_2d::<f64>(1,1).unwrap();
    //     pose.rot[(1,2)] = *r.at_2d::<f64>(1,2).unwrap();
    //     pose.rot[(2,0)] = *r.at_2d::<f64>(2,0).unwrap();
    //     pose.rot[(2,1)] = *r.at_2d::<f64>(2,1).unwrap();
    //     pose.rot[(2,2)] = *r.at_2d::<f64>(2,2).unwrap();

    //     pose
    // }

    //     fn align(&mut self, _context: Context, msg: &Arc<TrackerMsg>) {
    //     // Convert back to cv structures
    //     let kp1 =  msg.frame.key_points;
    //     let des1 = msg.img1_des.descriptors;

    //     if !self.first_frame {
    //         let (p2f1, p2f2) = self.find_best_match(&des1, &self.prev_des, &kp1, &self.prev_kps);

    //         let (rotation, translation) = self.calculate_transform(&p2f1, &p2f2);
    //         let pose = self.get_pose(&rotation, &translation);

    //         // println!("{}", pose.pos);
    //         // println!("{}", pose.rot);

    //         let vis_id = msg.actor_ids.get(VISUALIZER).unwrap();
    //         vis_id.send_new(VisMsg::new(pose, msg.actor_ids.clone())).unwrap();
    //     } else {
    //         println!("First Image ");
    //         self.first_frame = false;
    //     }
    //     //println!("Tracking done!");
    //     self.prev_kps = kp1;
    //     self.prev_des = des1;
    // }


    // fn find_best_match(&self,
    //     des1 :&Mat,
    //     des2 :&Mat,
    //     kp1: &VectorOfKeyPoint, 
    //     kp2: & VectorOfKeyPoint
    // ) -> (opencv::types::VectorOfPoint2f, opencv::types::VectorOfPoint2f) {
    //     // BFMatcher to get good matches
    //     let bfmtch = BFMatcher::create(6 , true).unwrap(); 
    //     let mut mask = Mat::default(); 
    //     let mut matches = VectorOfDMatch::new();
    //     bfmtch.train_match(&des2, &des1, &mut matches, &mut mask).unwrap(); 

    //     // Sort the matches based on the distance in ascending order
    //     // Using O(n^2) sort here. Need to make the code use cv sort function
    //     // by providing custom comparator

    //     let mut sorted_matches = VectorOfDMatch::new();
    //     let mut added = vec![false; matches.len()];
    //     for i in 0.. matches.len() {
    //         if added[i] == true {
    //             continue;
    //         }
    //         let mut mn = i;
    //         let mut dist = matches.get(i).unwrap().distance;
    //         for j in 0..matches.len() {
    //             let dmatch2 = matches.get(j).unwrap();
    //             if dist > dmatch2.distance && !added[j] {
    //                 mn = j;
    //                 dist = dmatch2.distance;
    //             }
    //         }
    //         let dmatch1 = matches.get(mn).unwrap();
    //         sorted_matches.push(dmatch1);
    //         added[mn] = true;
    //     }

    //     // Point vectors to hold the corresponding matched points 
    //     let mut p2f1 = core::Vector::<core::Point2f>::new(); 
    //     let mut p2f2 = core::Vector::<core::Point2f>::new();

    //     for i in 0..sorted_matches.len(){
    //         p2f1.push(kp1.get(sorted_matches.get(i).unwrap().train_idx.try_into().unwrap()).unwrap().pt);
    //         p2f2.push(kp2.get(sorted_matches.get(i).unwrap().query_idx.try_into().unwrap()).unwrap().pt);

    //     }
    //     (p2f1, p2f2)
    // }

    // fn calculate_transform(
    //     &self,
    //     curr_features: &opencv::types::VectorOfPoint2f,
    //     prev_features: &opencv::types::VectorOfPoint2f,
    // ) -> (Mat, Mat) {
    //     //recovering the pose and the essential matrix
    //     let (mut recover_r, mut recover_t, mut mask) = (Mat::default(), Mat::default(), Mat::default());
    //     let essential_mat = opencv::calib3d::find_essential_mat(
    //         &curr_features,
    //         &prev_features,
    //         718.8560, // self.focal,
    //         core::Point2d::new(607.1928, 185.2157) , //self.pp,
    //         opencv::calib3d::RANSAC,
    //         0.999,
    //         1.0,
    //         &mut mask,
    //     ).unwrap();
    //     opencv::calib3d::recover_pose(
    //         &essential_mat,
    //         &curr_features,
    //         &prev_features,
    //         &mut recover_r,
    //         &mut recover_t,
    //         718.8560, // self.focal,
    //         core::Point2d::new(607.1928, 185.2157) , //self.pp,
    //         &mut mask,
    //     ).unwrap();

    //     (recover_r, recover_t)
    // }

    // fn scale_transform(&self,
    //     scale: f64,
    //     rotation: &Mat,
    //     translation: &Mat,
    //     recover_mat: &Mat,
    //     t: &Mat,
    // ) -> (Mat, Mat) {
    //     //R = recover_mat, R_f= rotation, t_f =translation , t
    //     if scale > 0.1
    //         && (t.at::<f64>(2).unwrap() > t.at::<f64>(0).unwrap())
    //         && (t.at::<f64>(2).unwrap() > t.at::<f64>(1).unwrap())
    //     {
    //         // t_f = t_f + scale*(R_f*t);
    //         let mut rf_cross_t = opencv::core::mul_mat_mat(&rotation, &t).unwrap();
    //         rf_cross_t = opencv::core::mul_matexpr_f64(&rf_cross_t, scale).unwrap();
    //         let t_f = opencv::core::add_mat_matexpr(&translation, &rf_cross_t)
    //             .unwrap()
    //             .to_mat()
    //             .unwrap();

    //         // R_f = R*R_f;
    //         let r_f = opencv::core::mul_mat_mat(&recover_mat, &rotation)
    //             .unwrap()
    //             .to_mat()
    //             .unwrap();

    //         (r_f, t_f)
    //     } else {
    //         println!("scale below 0.1, or incorrect translation");
    //         (rotation.clone(), translation.clone())
    //     }
    // }

    // Safe to call these (instead of going through map) because
    // only the tracking thread has these data structures
    // when these functions are called
    #[allow(non_snake_case)]
    pub fn clean_VO_matches(&self) {
        // TODO
        // For each mappoint in mvpmappoints vector, delete it if the num of observations is < 1
        // First have to find out why frames even HAVE mvpmappoints vectors...
    }

    #[allow(non_snake_case)]
    pub fn delete_VO_matches_if_not_outliers(&self) {
        // TODO
        // for(int i=0; i<mCurrentFrame.N;i++)
        // {
        //     if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
        //         mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
        // }
    }

<<<<<<< HEAD
    pub fn GetMapPointMatches(&self) -> &Vec<Id>
    {
        &self.mvpMapPoints
    }
=======
>>>>>>> Should be done with track with motion model, new dependency on g2orust bindings crate
}