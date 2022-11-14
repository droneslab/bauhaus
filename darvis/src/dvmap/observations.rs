use dvcore::{matrix::DVVector3, global_params::{Sensor, FrameSensor}};
use nalgebra::Vector3;
use serde::{Serialize, de::DeserializeOwned, Deserialize};
use std::{fmt::Debug, collections::{HashMap, hash_map::Keys}};

use super::{keyframe::{KeyFrame, FullKeyFrame}, map::{Id, Map}};

#[derive(Clone, Debug)]
pub struct Observations {
    obs: HashMap<Id, (i32, i32)>,
    sensor: Sensor
}

impl Observations {
    pub fn new(sensor: Sensor) -> Self {
        Self { obs: HashMap::new(), sensor }
    }
    pub fn is_empty(&self) -> bool { 
        self.obs.is_empty() 
    }
    pub fn len(&self) -> usize { 
        self.obs.len()
    }

    pub fn keys(&self) -> Keys<Id, (i32, i32)> {
        self.obs.keys()
    }

    pub fn get_observation(&self, kf_id: &Id) -> (i32, i32) {
        *self.obs.get(kf_id).unwrap()
    }
    pub fn add_observation(&mut self, kf_id: &Id, num_keypoints_left_for_kf: u32, index: u32) {
        let (mut left_index, mut right_index) = match self.obs.get(kf_id) {
            Some((left, right)) => (*left, *right),
            None => (-1, -1)
        };

        match self.sensor.frame() {
            FrameSensor::Stereo => {
                if index >= num_keypoints_left_for_kf {
                    right_index = index as i32;
                } else {
                    left_index = index as i32;
                }
            },
            _ => left_index = index as i32
        }
        // Sofiya: What is this?
            // if(!pKF->mpCamera2 && pKF->mvuRight[idx]>=0)
            //     nObs+=2;
            // else
            //     nObs++;

        self.obs.insert(*kf_id, (left_index, right_index));
    }
    pub fn erase_observation(&mut self, kf_id: &Id) {
        self.obs.remove(kf_id);
    }
    pub fn erase_all_observations(&mut self) {
        self.obs = HashMap::new()
    }

    pub fn get_normal(&self, map: &Map, position: &DVVector3<f64>) -> (i32, nalgebra::Vector3<f64>) { 
        let mut normal = Vector3::<f64>::zeros();
        let mut n = 0;
        let position_opencv = **position;
        for (id, _) in &self.obs {
            let kf = map.get_keyframe(&id).unwrap();
            let mut camera_center = kf.get_camera_center();
            let owi = *camera_center;
            let normali = position_opencv - owi;
            normal = normal + normali / normali.norm();
            n += 1;

            match self.sensor.frame() {
                FrameSensor::Stereo => {
                    camera_center = kf.get_right_camera_center();
                    let owi = *camera_center;
                    let normali = position_opencv - owi;
                    normal = normal + normali / normali.norm();
                    n += 1;
                },
                _ => {}
            }
        }
        (n, normal)
    }

    pub fn compute_descriptors(&self, map: &Map) -> Vec::<opencv::core::Mat> {
        let mut descriptors = Vec::<opencv::core::Mat>::new();
        for (id, (index1, index2)) in &self.obs {
            let kf = map.get_keyframe(&id).unwrap();
            descriptors.push(kf.features.descriptors.row(*index1 as u32).unwrap());
            match self.sensor.frame() {
                FrameSensor::Stereo => descriptors.push(kf.features.descriptors.row(*index2 as u32).unwrap()),
                _ => {}
            }
        }
        descriptors
    }

    pub fn get_level(&self, kf: &KeyFrame<FullKeyFrame>) -> i32 {
        let (left_index, right_index) = self.obs.get(&kf.id()).unwrap();
        // Sofiya: sometimes in orbslam, left index will be -1 even for a stereo
        // camera, if there is no second camera set. I don't know why
        // they would do this though, like then it's not stereo...
        if *left_index != -1 {
            kf.features.get_octave(*left_index as usize)
        } else {
            kf.features.get_octave((right_index - kf.features.num_keypoints as i32) as usize)
        }
    }
}
