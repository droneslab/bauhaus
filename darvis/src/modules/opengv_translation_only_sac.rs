use core::matrix::{DVMatrix3, DVVector3};

use log::{debug, error, warn};
use nalgebra::{Matrix, Matrix2, Matrix3, MatrixXx3, Rotation3, SVD, Unit, Vector2, Vector3, Vector4};
use rand::Rng;
use crate::map::pose::{DVRotation, DVTranslation, Pose};

#[derive(Clone)]
pub struct TransformationT {
    pub rotation: Matrix3<f64>,
    pub translation: Vector3<f64>,
}
impl Default for TransformationT {
    fn default() -> Self {
        Self {
            rotation: Matrix3::identity(),
            translation: Vector3::zeros(),
        }
    }
}

pub struct CentralRelativeAdapter {
    pub bearing_vectors1: Vec<Vector3<f64>>,
    pub bearing_vectors2: Vec<Vector3<f64>>,

    pub r12: DVRotation,
    pub t12: DVTranslation,
}
impl CentralRelativeAdapter {
    pub fn new(
        bearing_vectors1: Vec<Vector3<f64>>,
        bearing_vectors2: Vec<Vector3<f64>>,
        r12: DVRotation,
        t12: DVTranslation,
    ) -> Self {
        Self {
            bearing_vectors1,
            bearing_vectors2,
            r12,
            t12,
        }
    }
}

pub trait SampleConsensusProblem {
    type Model: Default + Clone;
    type Adaptor;

    fn get_samples(&mut self, iterations: &mut u32, samples: &mut Vec<usize>, max_sample_checks: Option<i32>);
    fn get_indices(&self) -> &Vec<usize>;
    fn compute_model_coefficients(&self, indices: &Vec<usize>, model: &mut Self::Model) -> bool;
    fn select_within_distance(&mut self, model: &Self::Model, threshold: f64) -> Vec<usize>;
    fn count_within_distance(&mut self, model: &Self::Model, threshold: f64) -> i32;
    fn get_selected_distances_to_model(&mut self, model: &Self::Model) -> Vec<f64>;
}

pub struct TranslationOnlySacProblem {
    indices: Vec<usize>,
    shuffled_indices: Vec<usize>,
    adapter: CentralRelativeAdapter,
    ransac_randomize: bool, // Not using this currently but keeping for completeness
    sample_size: usize,
}

impl TranslationOnlySacProblem {
    pub fn new(
        adapter: CentralRelativeAdapter,
        ransac_randomize: bool,
    ) -> Self {
        // Set uniform indices
        let mut indices = vec![];
        for i in 0..adapter.bearing_vectors2.len() {
            indices.push(i);
        }

        Self {
            adapter,
            ransac_randomize,
            shuffled_indices: indices.clone(),
            indices,
            sample_size: 2,
        }
    }

    fn draw_index_sample(&mut self, sample: &mut Vec<usize>) {
        // template<typename M>
        // void
        // opengv::sac::SampleConsensusProblem<M>::drawIndexSample(
        //     std::vector<int> & sample)

        let sample_size = sample.len();
        let index_size = self.shuffled_indices.len();

        let mut rng = rand::thread_rng();
        for i in 0..sample_size {
            // The 1/(RAND_MAX+1.0) trick is when the random numbers are not uniformly
            // distributed and for small modulo elements, that does not matter
            // (and nowadays, random number generators are good)

            let rand: usize = rng.gen_range(0, index_size - i);
            self.shuffled_indices.swap(i, i + rand);
        }

        for i in 0..sample_size {
            sample[i] = self.shuffled_indices[i];
        }
    }

    fn is_sample_good(&self, _sample: &Vec<usize>) -> bool {
        // template<typename M>
        // bool opengv::sac::SampleConsensusProblem<M>::isSampleGood(
        //     const std::vector<int> & sample) const
        // {
        // // Default implementation
        // return true;
        // }

        // SOFIYA TODO why is this just always true?
        return true;
    }

}

impl SampleConsensusProblem for TranslationOnlySacProblem {
    type Model = TransformationT;
    type Adaptor = CentralRelativeAdapter;

    fn get_indices(&self) -> &Vec<usize> {
        & self.indices
    }

    fn get_selected_distances_to_model(&mut self, model: &Self::Model) -> Vec<f64> {
        // Sofiya: Tested, off by one? Could be a threshold thing

        // https://github.com/laurentkneip/opengv/blob/91f4b19c73450833a40e463ad3648aae80b3a7f3/src/sac_problems/relative_pose/TranslationOnlySacProblem.cpp#L51
        // void
        // opengv::sac_problems::
        //     relative_pose::TranslationOnlySacProblem::getSelectedDistancesToModel(
        //     const model_t & model,
        //     const std::vector<int> & indices,
        //     std::vector<double> & scores) const

        let translation = model.translation;
        let rotation = model.rotation;
        self.adapter.t12 = DVVector3::new(translation);
        self.adapter.r12 = DVMatrix3::new(rotation);

        let inverse_solution = {
            let rot_transpose = rotation.transpose();
            Pose::new(
                -rot_transpose*translation,
                rot_transpose
            )
        };
        let mut p_hom = Vector4::new(0.0, 0.0, 0.0, 1.0);

        let mut scores = vec![];
        for i in 0..self.indices.len() {
            let triangulate = triangulate2(&self.adapter, self.indices[i]);
            p_hom[0] = triangulate[0];
            p_hom[1] = triangulate[1];
            p_hom[2] = triangulate[2];

            let mut reprojection1 = Vector3::new(p_hom[0], p_hom[1], p_hom[2]);
            let mut reprojection2 = inverse_solution.get_3x4() * p_hom;
            reprojection1 = reprojection1 / reprojection1.norm();
            reprojection2 = reprojection2 / reprojection2.norm();

            let f1 = self.adapter.bearing_vectors1[self.indices[i]]; // should be [0,1,2,3,etc]
            let f2 = self.adapter.bearing_vectors2[self.indices[i]];

            //bearing-vector based outlier criterium (select threshold accordingly):
                //1-(f1'*f2) = 1-cos(alpha) \in [0:2]
            let reproj_error1 = 1.0 - (f1.transpose() * reprojection1)[0]; // [0] here is just to get the value out of a 1x1 matrix type
            let reproj_error2 = 1.0 - (f2.transpose() * reprojection2)[0]; // [0] here is just to get the value out of a 1x1 matrix type
            scores.push(reproj_error1 + reproj_error2);
        }
        scores
    }

    fn select_within_distance(&mut self, model_coefficients: &Self::Model, threshold: f64) -> Vec<usize> {
        // https://github.com/laurentkneip/opengv/blob/91f4b19c73450833a40e463ad3648aae80b3a7f3/include/opengv/sac/implementation/SampleConsensusProblem.hpp#L168

        // template<typename M>
        // void
        // opengv::sac::SampleConsensusProblem<M>::selectWithinDistance(
        //     const model_t & model_coefficients,
        //     const double threshold,
        //     std::vector<int> &inliers )

        let dist = self.get_selected_distances_to_model(model_coefficients);

        let mut inliers = vec![];
        for i in 0..dist.len() {
            if dist[i] < threshold {
                inliers.push(self.indices[i]);
            }
        }
        inliers
    }

    fn count_within_distance(&mut self, model_coefficients: &Self::Model, threshold: f64) -> i32 {
        // https://github.com/laurentkneip/opengv/blob/91f4b19c73450833a40e463ad3648aae80b3a7f3/include/opengv/sac/implementation/SampleConsensusProblem.hpp#L168

        // template<typename M>
        // int
        // opengv::sac::SampleConsensusProblem<M>::countWithinDistance(
        //     const model_t & model_coefficients, const double threshold)

        let dist = self.get_selected_distances_to_model(model_coefficients);

        let mut count = 0;
        for i in 0..dist.len() {
            if dist[i] < threshold {
                count += 1;
            }
        }
        count
    }

    fn compute_model_coefficients(
        &self,
        indices: &Vec<usize>,
        out_model: &mut Self::Model,
    ) -> bool {
        // Sofiya: Tested

        // https://github.com/laurentkneip/opengv/blob/91f4b19c73450833a40e463ad3648aae80b3a7f3/src/sac_problems/relative_pose/TranslationOnlySacProblem.cpp#L38

        // opengv::sac_problems::
        //     relative_pose::TranslationOnlySacProblem::computeModelCoefficients(
        //     const std::vector<int> &indices,
        //     model_t & outModel) const

        out_model.rotation = *self.adapter.r12;
        out_model.translation = two_pt(&self.adapter, true, indices[0] as usize, indices[1] as usize);

        true
    }

    fn get_samples(&mut self, iterations: &mut u32, samples: &mut Vec<usize>, max_sample_checks: Option<i32>) {
        // https://github.com/laurentkneip/opengv/blob/91f4b19c73450833a40e463ad3648aae80b3a7f3/include/opengv/sac/implementation/SampleConsensusProblem.hpp#L89

        // template<typename M>
        // void
        // opengv::sac::SampleConsensusProblem<M>::getSamples(
        //     int &iterations, std::vector<int> &samples)

        // We're assuming that indices_ have already been set in the constructor
        if self.indices.len() < self.sample_size {
            error!(
                "[sm::SampleConsensusModel::getSamples] Can not select {} unique points out of {}",
                self.sample_size, self.indices.len()
            );
            samples.clear();
            *iterations = u32::MAX;
            return;
        }

        // Get a second point which is different than the first
        *samples = vec![0 as usize; self.sample_size];
        let max_sample_checks = max_sample_checks.unwrap_or(10);

        for i in 0..max_sample_checks {
            self.draw_index_sample(samples);

            // If it's a good sample, stop here
            if self.is_sample_good(& samples) {
                return;
            }
        }

        warn!("Could not select {} sample points in {} iterations", self.sample_size, max_sample_checks);
        samples.clear();
    }

}


pub struct Ransac<M: SampleConsensusProblem> {
    sac_model: M, // the sample-consensus problem we are trying to solve

    // Constants
    threshold: f64, // the threshold for classifying inliers
    max_iterations: u32,
    probability: f64, // the current probability (defines remaining iterations)
    optimize_2d2d_pose_from_inliers: bool, // Not using this currently but keeping for completeness

    // Modified during ransac operation
    iterations: u32, // the current number of iterations
    model: Vec<usize>, // indices for the currently best hypothesis
    pub model_coefficients: M::Model, // the current best model coefficients
    pub inliers: Vec<usize>, // the indices of the samples that have been clasified as inliers
}

impl<M: SampleConsensusProblem> Ransac<M> {
    pub fn new(
        sac_model: M, threshold: f64,
        max_iterations: u32, probability: f64,
        optimize_2d2d_pose_from_inliers: bool,
        inliers: Vec<usize>,
    ) -> Self {
        Self {
            sac_model,
            threshold,
            max_iterations,
            probability,
            optimize_2d2d_pose_from_inliers,
            inliers,
            iterations: 0,
            model: vec![],
            model_coefficients: M::Model::default(),
        }
    }

    pub fn compute_model(&mut self) -> bool {
        // https://github.com/laurentkneip/opengv/blob/91f4b19c73450833a40e463ad3648aae80b3a7f3/include/opengv/sac/implementation/Ransac.hpp#L47

        let mut n_best_inliers_count = -std::i32::MAX;
        let mut k = 1.0;

        let mut selection = vec![];
        let mut model_coefficients = M::Model::default();

        let mut n_inliers_count;
        let mut skipped_count = 0;

        // supress infinite loops by just allowing 10 x maximum allowed iterations for
        // invalid model parameters!
        let max_skip = self.max_iterations * 10;

        // Iterate
        while self.iterations < (k as u32) && skipped_count < max_skip {
            // Get X samples which satisfy the model criteria
            self.sac_model.get_samples(&mut self.iterations, &mut selection, None);

            if selection.is_empty() {
                warn!(
                    "[sm::RandomSampleConsensus::computeModel] No samples could be selected!\n"
                );
                break;
            }

            // Search for inliers in the point cloud for the current plane model M
            let success = self.sac_model.compute_model_coefficients(&selection, &mut model_coefficients);
            if !success {
                skipped_count += 1;
                continue;
            }

            n_inliers_count = self.sac_model.count_within_distance(&model_coefficients, self.threshold);
                //     n_inliers_count = sac_model_->countWithinDistance(
                //         model_coefficients, threshold_ );

            // Better match ?
            if n_inliers_count > n_best_inliers_count {
                n_best_inliers_count = n_inliers_count;

                // Save the current model/inlier/coefficients selection as being the best so far
                self.model = selection.clone();
                self.model_coefficients = model_coefficients.clone();

                // Compute the k parameter (k=log(z)/log(1-w^n))
                let w = (n_best_inliers_count as f64) / (self.sac_model.get_indices().len() as f64);
                    // double w = static_cast<double> (n_best_inliers_count) /
                    //     static_cast<double> (sac_model_->getIndices()->size());
                let mut p_no_outliers = 1.0 - (num_traits::pow(w, selection.len()) as f64);
                p_no_outliers = f64::max(f64::EPSILON, p_no_outliers); // Avoid division by -Inf
                p_no_outliers = f64::min(1.0 - f64::EPSILON, p_no_outliers); // Avoid division by 0.
                k = (1.0 - self.probability).ln() / (p_no_outliers).ln();
            }

            self.iterations += 1;

            // debug!(
            //     "[sm::RandomSampleConsensus::computeModel] Trial {} out of {}: {} inliers (best is: {} so far).\n",
            //     self.iterations, k, n_inliers_count, n_best_inliers_count
            // );

            if self.iterations > self.max_iterations {
                warn!(
                    "[sm::RandomSampleConsensus::computeModel] RANSAC reached the maximum number of trials.\n"
                );
                break;
            }
        }


        debug!(
            "[sm::RandomSampleConsensus::computeModel] Model: {} size, {} inliers.",
            self.model.len(),
            n_best_inliers_count
        );

        if self.model.is_empty() {
            self.inliers.clear();
            return false;
        }

        // Get the set of inliers that correspond to the best model found so far
        self.inliers = self.sac_model.select_within_distance(&self.model_coefficients, self.threshold);
            // sac_model_->selectWithinDistance( model_coefficients_, threshold_, inliers_ );

        return true;
    }

}


pub fn two_pt(
    adapter: &CentralRelativeAdapter,
    unrotate: bool,
    index0: usize,
    index1: usize
) -> Vector3<f64> {
    // Sofiya: Tested

    // opengv::relative_pose::twopt(
    //     const RelativeAdapterBase & adapter,
    //     bool unrotate,
    //     size_t index0,
    //     size_t index1 )

    let f1 = adapter.bearing_vectors1[index0];
    let mut f1prime = adapter.bearing_vectors2[index0];
    let f2 = adapter.bearing_vectors1[index1];
    let mut f2prime = adapter.bearing_vectors2[index1];

    if unrotate {
        let r12 = *adapter.r12;
        f1prime = r12 * f1prime;
        f2prime = r12 * f2prime;
    }

    let normal1 = f1.cross(&f1prime);
    let normal2 = f2.cross(&f2prime);

    let mut translation = normal1.cross(&normal2);
    translation = translation / translation.norm();

    let optical_flow = f1 - f1prime;
    if optical_flow.dot(&translation) < 0.0 {
        translation = -translation;
    }
    translation
}

pub fn triangulate2(
    adapter: &CentralRelativeAdapter,
    index: usize,
) -> Vector3<f64> {
    // Sofiya: Tested, but possibly some floating point differences here

    // https://github.com/laurentkneip/opengv/blob/91f4b19c73450833a40e463ad3648aae80b3a7f3/src/triangulation/methods.cpp#L66

    // opengv::point_t
    // opengv::triangulation::triangulate2(
    //     const relative_pose::RelativeAdapterBase & adapter,
    //     size_t index )

    let t12 = *adapter.t12;
    let r12 = *adapter.r12;
    let f1 = adapter.bearing_vectors1[index];
    let f2 = adapter.bearing_vectors2[index];

    let f2_unrotated = r12 * f2;
    let mut b = Vector2::new(0.0, 0.0);
    b[0] = t12.dot(&f1);
    b[1] = t12.dot(&f2_unrotated);

    let mut a = Matrix2::<f64>::default();
    a[(0,0)] = f1.dot(&f1);
    a[(1,0)] = f1.dot(&f2_unrotated);
    a[(0,1)] = -a[(1,0)];
    a[(1,1)] = -f2_unrotated.dot(&f2_unrotated);
    let lambda = a.try_inverse().unwrap() * b;
    let xm = lambda[0] * f1;
    let xn = t12 + lambda[1] * f2_unrotated;
    let point = {
        let add = xm + xn;
        Vector3::new(add[0] / 2.0, add[1] / 2.0, add[2] / 2.0)
    };
    point
}