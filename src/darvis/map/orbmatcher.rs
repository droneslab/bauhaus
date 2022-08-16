
use opencv::core::{Point2f, KeyPoint};
use opencv::{
    prelude::*,
};
use crate::{
    map::{
    frame::Frame
    },
};

use super::keyframe::KeyFrame;
use super::map::Id;
use super::mappoint::MapPoint;

unsafe impl Sync for ORBmatcher {}

const  TH_HIGH: i64= 100;
const  TH_LOW: i64 = 50;
const  HISTO_LENGTH: i64 = 30;


#[derive(Debug, Clone)]
pub struct ORBmatcher {
    pub nnratio: f32,//float mfNNratio;
    pub check_orientation: bool,//bool mbCheckOrientation;
}

impl ORBmatcher {
    pub fn new(nnratio: f32, check_orientation: bool) -> Self {
        Self{
            nnratio: nnratio,
            check_orientation: check_orientation,
        }
    }


    // Bit set count operation from
    // http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
    pub fn descriptor_distance(&self, a : &Mat, b: &Mat) -> i64
    {
        //[POSSIBLE BUG]: IMPLEMENTATION NOT VARIFIED
        let pa = 0;
        let pb = 0;

        let mut dist : i64 = 0;

        for i in 0..8
        {
            let mut v : u64 = (*a.at::<f64>(pa).unwrap() as i64 ^ *b.at::<f64>(pb).unwrap() as i64) as u64;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += ((((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24) as i64; 

        }
        
        // const int *pa = a.ptr<int32_t>();
        // const int *pb = b.ptr<int32_t>();

        // int dist=0;

        // for(int i=0; i<8; i++, pa++, pb++)
        // {
        //     unsigned  int v = *pa ^ *pb;
        //     v = v - ((v >> 1) & 0x55555555);
        //     v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        //     dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        // }

        return dist;
    }

    pub fn search_for_initialization(&self, F1: &Frame , F2: &Frame, prev_matched: &mut Vec<Point2f>, matches12: &mut Vec<i64>, window_size : i64 ) -> i64
    {   
        //ref code : https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/0df83dde1c85c7ab91a0d47de7a29685d046f637/src/ORBmatcher.cc#L648
        let mut nmatches: i64=0;
        matches12.resize(F1.key_points_un.len(), -1);


        let mut rotHist = Vec::<Vec<i64>>::new();
        rotHist.resize(HISTO_LENGTH as usize, vec![0; 500]);

        const FACTOR: f64 = 1.0 as f64 / HISTO_LENGTH as f64;

        let mut matched_distance = vec![std::i64::MAX; F2.key_points_un.len()]; 
        let mut matches21 = vec![-1 as i64; F2.key_points_un.len()];  
        let iend1 = F1.key_points_un.len();
        for i1 in 0..iend1
        {

            let kp1: KeyPoint = F1.key_points_un.get(i1).unwrap().clone();//cv::KeyPoint kp1 = F1.mvKeysUn[i1];
            let level1 = kp1.octave;//int level1 = kp1.octave;
            if level1>0
            {
                continue;
            }

            let indices2 = F2.GetFeaturesInArea(&(prev_matched.get(i1).unwrap().x as f64), &(prev_matched.get(i1).unwrap().y as f64), &(window_size as f64), &(level1 as i64), &(level1 as i64), false);
            //vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

            if indices2.is_empty()
            {
                continue;
            }
                

            let mut d1 = F1.descriptors.row(i1 as i32).unwrap();

            let mut bestDist = i64::MAX;
            let mut bestDist2 = i64::MAX;
            let mut bestIdx2: i64 = -1;

            for i in 0..indices2.len()
            {

                let i2 = i;

                let mut d2 = F2.descriptors.row(i2 as i32).unwrap();

                let mut dist = self.descriptor_distance(&d1,&d2);

                if matched_distance[i2]<=dist
                {
                    continue;
                } 

                if dist<bestDist 
                {
                    bestDist2=bestDist;
                    bestDist=dist;
                    bestIdx2=i2 as i64;
                }
                else if dist<bestDist2 
                {
                    bestDist2=dist;
                }
            }

            if bestDist<=TH_LOW 
            {
                if bestDist< bestDist2* (self.nnratio  as i64)
                {
                    if matches21[bestIdx2 as usize]>=0 
                    {
                        matches12[matches21[bestIdx2 as usize] as usize] = -1;
                        nmatches-=1;
                    }
                    matches12[i1]=bestIdx2;
                    matches21[bestIdx2 as usize]=i1 as i64;
                    matched_distance[bestIdx2 as usize]=bestDist;
                    nmatches+=1;

                    if self.check_orientation
                    {
                        let mut rot = F1.key_points_un.get(i1).unwrap().angle-F2.key_points_un.get(bestIdx2 as usize).unwrap().angle;//float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                        if rot<0.0
                        {
                            rot+=360.0;
                        }

                        let mut bin = f64::round((rot as f64 )*FACTOR) as i64;
                        if bin==HISTO_LENGTH 
                        {
                            bin=0;
                        }
                        assert!(bin>=0 && bin<HISTO_LENGTH);

                        //assert(bin>=0 && bin<HISTO_LENGTH);
                        rotHist[bin as usize].push(i1 as i64); //.push_back(i1);
                    }
                }
            }

        }

        if self.check_orientation
        {
            let mut ind1: i64 =-1;
            let mut ind2: i64 =-1;
            let mut ind3: i64 =-1;

            self.compute_three_maxima(&rotHist,HISTO_LENGTH,&mut ind1,&mut ind2,&mut ind3);


            for i in 0..HISTO_LENGTH
            {
                if i==ind1 || i==ind2 || i==ind3
                {
                    continue;
                }
                    
                for j in 0..rotHist[i as usize].len()
                {
                    let idx1: usize = rotHist[i as usize][j] as usize;
                    if matches12[idx1]>=0
                    {
                        matches12[idx1]=-1;
                        nmatches-=1;
                    }
                }
            }

        }

        //Update prev matched
        for i1 in 0..matches12.len()
        {
            if matches12[i1]>=0
            {
                prev_matched[i1]= F2.key_points_un.get(matches12[i1] as usize).unwrap().pt.clone();
            }                
        }

        return nmatches;
    }

    pub fn compute_three_maxima(&self, histo : &Vec<Vec<i64>> , L: i64, ind1: &mut i64, ind2: &mut i64, ind3: &mut i64)
    {
        let mut max1: i64=0;
        let mut max2: i64=0;
        let mut max3: i64=0;

        for i in 0..L
        {
            let s = histo[i as usize].len() as i64;
            if s>max1 
            {
                max3=max2;
                max2=max1;
                max1=s;
                *ind3=*ind2;
                *ind2=*ind1;
                *ind1=i;
            }
            else if s>max2 
            {
                max3=max2;
                max2=s;
                *ind3=*ind2;
                *ind2=i;
            }
            else if s>max3 
            {
                max3=s;
                *ind3=i;
            }
        }

        if max2< (0.1 * (max1 as f64)) as i64
        {
            *ind2=-1;
            *ind3=-1;
        }
        else if max3< ((0.1*(max1 as f64)) as i64)
        {
            *ind3=-1;
        }
    }


    //int ORBmatcher::SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, vector<MapPoint *> &vpMatches12)
    pub fn SearchByBoW(&self, pKF1 : &KeyFrame, pKF2 : &KeyFrame, vpMatches12: &Vec<Id>) -> i32
    {
        todo!("ORBmatcher: SearchByBoW");
        

    }



}