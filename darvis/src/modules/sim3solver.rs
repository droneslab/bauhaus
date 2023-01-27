

use std::collections::HashSet;

use dvcore::lockwrap::ReadOnlyWrapper;
use log::{warn, info, debug};
use nalgebra::{Matrix3, Similarity3, MatrixN, SMatrix, Vector3, Matrix4};

use crate::dvmap::{keyframe::{FullKeyFrame, KeyFrame}, map::Map};

use rand::Rng;


#[derive(Clone, Debug)]
pub struct Sim3Solver {
    kf1: KeyFrame<FullKeyFrame>,
    kf2: KeyFrame<FullKeyFrame>,
    matched12: Vec<i32>,
    b_fixed_scale: bool,
    keyframe_matched_mp: Vec<i32>,
    T12: Similarity3<f64>,
    map: ReadOnlyWrapper<Map>,
}

impl Sim3Solver {
    pub fn new(kf1: &KeyFrame<FullKeyFrame>, 
        kf2: &KeyFrame<FullKeyFrame>,
        matched12: &Vec<i32>,
        b_fixed_scale: bool,
        keyframe_matched_mp: &Vec<i32>,
        map: ReadOnlyWrapper<Map>) -> Self {
        Self { 
            kf1: kf1.clone(),
            kf2: kf2.clone(), 
            matched12: matched12.clone(),
            b_fixed_scale: b_fixed_scale , 
            keyframe_matched_mp: keyframe_matched_mp.clone(),
            T12: Similarity3::<f64>::identity(),
            map: map,
        }
    }


    //Pranay : check if the code is correct
    pub fn find(&self) -> Option<Similarity3<f64>> {
        let mut n_initial_correspondences = 0;

        // Indices for minimum set selection
        let mut all_indices = Vec::new();
        let mut available_indices = Vec::new();

        // Set of MapPoints already found in the KeyFrame
        let mut spAlreadyFound = HashSet::new();

        let mut vP3D1 = Vec::new();
        let mut vP3D2 = Vec::new();

        for i in 0..self.matched12.len() {
            let idx1 = self.matched12[i].0;
            let idx2 = self.matched12[i].1;

            let mp1 = self.kf1.get_mappoint(&(idx1 as u32));
            let mp2 = self.kf2.get_mappoint(&(idx2 as u32));

            
            if mp1 ==-1 || mp2 ==-1 {
                continue;
            }

            if self.keyframe_matched_mp[idx2] >= 0 {
                continue;
            }

            if spAlreadyFound.contains(&mp1) {
                continue;
            }

            spAlreadyFound.insert(mp1);
            
            vP3D1.push(self.map.read().get_mappoint(&mp1).unwrap().position);
            vP3D2.push(self.map.read().get_mappoint(&mp2).unwrap().position);

            all_indices.push(i);
        }

        n_initial_correspondences = vP3D1.len();

        if n_initial_correspondences < 3 {
            return None;
        }

        // Select from all correspondences
        let mut best_inliers = Vec::new();
        let mut best_score = 0.0;
        let mut best_T12 = Similarity3::<f64>::identity();
        let mut best_cov12 = SMatrix::<f64,7,7>::zeros();
        let mut best_n_inliers = 0;

        let mut rng = rand::thread_rng();
        rng.gen::<i32>();

        let mut n_iterations = 0;

        while n_iterations < 300 {
            // Select a minimum set
            available_indices.clear();
            available_indices.extend_from_slice(&all_indices);

            let mut selected_indices = Vec::new();

            for _ in 0..3 {
                let pos = rng.gen_range(0, available_indices.len());
                let idx = available_indices[pos];
                selected_indices.push(idx);
                available_indices.swap_remove(pos);
            }

            let mut X1 = Vec::new();
            let mut X2 = Vec::new();

            for i in 0..3 {
                let idx = selected_indices[i];
                X1.push(vP3D1[idx]);
                X2.push(vP3D2[idx]);
            }
        }

        Some(())
    }

    pub fn set_ransac_parameters(
        &mut self, 
        probability: f64,
        min_inliers: i32,
        max_iterations: i32,
    ) {
        self.probability = probability;
        self.min_inliers = min_inliers;
        self.max_iterations = max_iterations;
    }

    pub fn get_estimated_transformation(&self) -> Similarity3<f64> {
        self.T12
    }

    pub fn get_estimated_rotation(&self) -> Matrix3<f64> {
        *self.T12.isometry.rotation.to_rotation_matrix().matrix()
    }

    pub fn get_estimated_translation(&self) -> Vector3<f64> {
        self.T12.isometry.translation.vector
    }

    pub fn get_estimated_scale(&self) -> f64 {
        self.T12.scaling()
    }

    
    pub fn iterate(&self, n_interations: i32, b_no_more: bool, vb_inliers : Vec<bool>, n_inliers: i32, b_coverage : bool ) -> Matrix4<f64>
    {
        todo!("Implement iterate function");
        Matrix4::<f64>::identity()
    }

}