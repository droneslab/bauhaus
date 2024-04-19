//*** KEYFRAME DATABASE FROM ORBSLAM2 ***/

use std::collections::{HashSet, HashMap};

use log::debug;

use crate::{map::{keyframe::KeyFrame, map::Id}, registered_actors::VOCABULARY};

use super::map::Map;


#[derive(Debug, Clone)]
pub struct KeyFrameDatabase {
    inverted_file: Vec<Vec<Id>> // mvInvertedFile
}

impl KeyFrameDatabase {
    pub fn new() -> Self {
        let vocab_size = VOCABULARY.size();
        // println!("Vocabulary size: {}", vocab_size);
        KeyFrameDatabase { 
            inverted_file: vec![vec![]; vocab_size]
        }
    }

    pub fn add(&mut self, kf: &KeyFrame) {
        // void KeyFrameDatabase::add(KeyFrame *pKF)
        debug!("Add KF {}", kf.id);
        let word_ids = kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
        // println!("Word ids: {}", word_ids.len());
        // print!("KEYFRAME DATABASE: {}, ", kf.id);
        for word_id in word_ids {
            self.inverted_file[word_id as usize].push(kf.id);
            // print!("{} ", word_id);
        }
        // println!();
    }

    pub fn erase(&mut self, kf: &KeyFrame) {
        // void KeyFrameDatabase::erase

        // Erase elements in the Inverse File for the entry
        let word_ids = kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
        for word_id in word_ids {
            // List of keyframes that share the word
            self.inverted_file[word_id as usize].retain(|&id| id != kf.id);
        }
    }

    pub fn _clear(&mut self) {
        self.inverted_file.clear();
        self.inverted_file.resize(VOCABULARY.size(), Vec::new());
    }

    pub fn detect_n_best_candidates(&self, map: &Map, curr_kf_id: &Id, num_candidates: i32) -> (Vec<Id>, Vec<Id>) {
        // void KeyFrameDatabase::DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates)

        let curr_kf = map.keyframes.get(&curr_kf_id).unwrap();
        let connected_kfs = map.keyframes.get(&curr_kf_id).unwrap().get_connected_keyframes();
        let mut kfs_sharing_words = vec![]; //lKFsSharingWords
        let mut place_recognition_query = HashMap::new(); // mnPlaceRecognitionQuery
        let mut place_recognition_words = HashMap::new(); // mnPlaceRecognitionWords

        // Search all keyframes that share a word with current frame
        for word in curr_kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids() {
            for kf_i_id in &self.inverted_file[word as usize] {
                let different_query = place_recognition_query.get(kf_i_id).is_some() && place_recognition_query[kf_i_id] != curr_kf_id;
                let no_query = place_recognition_query.get(kf_i_id).is_none();

                if different_query || no_query {
                    place_recognition_words.insert(*kf_i_id, 0);
                    if !connected_kfs.contains_key(&kf_i_id) {
                        place_recognition_query.insert(*kf_i_id, curr_kf_id);
                        kfs_sharing_words.push(*kf_i_id);
                    }
                }
                *place_recognition_words.get_mut(kf_i_id).unwrap() += 1;
            }
        }

        debug!("Detect loop candidates... kfs sharing words: {:?}", kfs_sharing_words);
        // debug!("Detect loop candidates... loop words: {:?}", loop_words);

        if kfs_sharing_words.is_empty() {
            return (vec![], vec![]);
        }

        // Only compare against those keyframes that share enough words
        let mut max_common_words = 0;
        let mut max_word_kf_id = -1;
        for kf_id in &kfs_sharing_words {
            match place_recognition_words.get(&kf_id) {
                Some(words) => {
                    if *words > max_common_words {
                        max_common_words = *words;
                        max_word_kf_id = *kf_id;
                    }
                },
                None => continue
            }
        }

        let min_common_words = (max_common_words as f32 * 0.8) as i32;
        let mut score_and_match = vec![];
        let mut n_scores = 0;

        debug!("Detect loop candidates... max common words: {}, min common words: {}, max word kf id: {}", max_common_words, min_common_words, max_word_kf_id);

        // Compute similarity score.
        let mut place_recognition_score = HashMap::new(); // mPlaceRecognitionScore
        for kf_i_id in &kfs_sharing_words {
            let test = *kf_i_id > 75 && *kf_i_id < 80;
            if place_recognition_words[&kf_i_id] > min_common_words  {
                n_scores += 1;
                let kf_i = map.keyframes.get(&kf_i_id).unwrap();
                let si = VOCABULARY.score(curr_kf, kf_i);
                place_recognition_score.insert(kf_i_id, si);
                score_and_match.push((si, kf_i_id));
            }
        }
        // println!();
        // debug!("Detect loop candidates... similarity scores: {:?}", score_and_match);

        if score_and_match.is_empty() {
            return (vec![], vec![]);
        }

        // Lets now accumulate score by covisibility
        let mut acc_score_and_match = vec![]; // lAccScoreAndMatch
        let mut best_acc_score = 0.0;
        for (score, kf_i_id) in score_and_match {
            let kf_i = map.keyframes.get(&kf_i_id).unwrap();
            let neighbor_kfs = kf_i.get_covisibility_keyframes(10);
            let mut best_score = score;
            let mut acc_score = best_score;
            let mut best_kf_id = *kf_i_id;

            for kf_2_id in neighbor_kfs {
                if place_recognition_query.get(&kf_2_id).is_some() && place_recognition_query[&kf_2_id] != curr_kf_id {
                    continue;
                }
                let old_score = *place_recognition_score.get(&kf_2_id).unwrap_or(&0.0);
                acc_score += old_score;
                
                if old_score > best_score {
                    best_kf_id = kf_2_id;
                    best_score = old_score;
                }
            }

            acc_score_and_match.push((acc_score, best_kf_id));
            if acc_score > best_acc_score {
                best_acc_score = acc_score;
            }
        }

        acc_score_and_match.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        // TODO (multimaps): Add in merge candidates
        let mut i = 0;
        let num_candidates = num_candidates as usize;
        let mut loop_candidates = vec![];
        let merge_candidates = vec![];
        let mut already_added_kf = HashSet::new(); // spAlreadyAddedKF
        while i < acc_score_and_match.len() && loop_candidates.len() < num_candidates {
            let (score, kf_id) = acc_score_and_match[i];
            if !already_added_kf.contains(&kf_id) {
                loop_candidates.push(kf_id);
                already_added_kf.insert(kf_id);
            }
        }

        return (merge_candidates, loop_candidates);
    }

    pub fn _detect_relocalization_candidates(&self, _keyframe_id: Id) -> Vec<Id> {
        // std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);

        todo!("Relocalization")
    }
}
