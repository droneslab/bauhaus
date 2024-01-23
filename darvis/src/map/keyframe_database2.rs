use std::{sync::Arc, collections::{HashSet, HashMap}};

use nalgebra::min;
use parking_lot::RwLock;

use crate::{map::map::Id, MapLock};

use super::{keyframe::KeyFrame, bow::VOCABULARY};

#[derive(Debug)]
pub struct KeyFrameDatabase {
    inverted_file: Arc<RwLock<Vec<Vec<Id>>>> // mvInvertedFile
}

impl KeyFrameDatabase {
    pub fn new() -> Self {
        KeyFrameDatabase { 
            inverted_file: Arc::new(RwLock::new(Vec::with_capacity(VOCABULARY.size())))
        }
    }

    pub fn add(&mut self, kf: &KeyFrame) {
        // void KeyFrameDatabase::add(KeyFrame *pKF)
        let word_ids = kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
        for word_id in word_ids {
            let mut inverted_file = self.inverted_file.write();
            inverted_file[word_id as usize].push(kf.id);
        }
    }

    pub fn erase(&mut self, kf: &KeyFrame) {
        // void KeyFrameDatabase::erase

        // Erase elements in the Inverse File for the entry
        let word_ids = kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
        for word_id in word_ids {
            // List of keyframes that share the word
            self.inverted_file.write()[word_id as usize].retain(|&x| x != kf.id);
            // TODO double check this code produces the right output! Not sure what the original is doing
        }
    }

    pub fn clear(&mut self) {
        self.inverted_file.write().clear();
        self.inverted_file.write().resize(VOCABULARY.size(), Vec::new());
    }

    pub fn detect_loop_candidates(&self, map: &MapLock, curr_kf_id: &Id, min_score: f32) -> Vec<Id> {
        let map_read_lock = map.read();
        let inv_file_read_lock = self.inverted_file.read();
        let curr_kf = map_read_lock.keyframes.get(&curr_kf_id).unwrap();
        let connected_kfs = map.read().keyframes.get(&curr_kf_id).unwrap().get_covisibility_keyframes(i32::MAX);
        let mut kfs_sharing_words = vec![]; //lKFsSharingWords
        let mut loop_words = HashMap::new(); // mnLoopWords
        let mut loop_query = HashMap::new(); // mnLoopQuery

        // Search all keyframes that share a word with current keyframes
        // Discard keyframes connected to the query keyframe
        for word in curr_kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids() {
            for kf_i_id in &inv_file_read_lock[word as usize] {
                if (loop_query.get(kf_i_id).is_some() && loop_query[kf_i_id] != curr_kf_id)
                    || loop_query.get(kf_i_id).is_none() {
                    loop_words.insert(*kf_i_id, 0);
                    if !connected_kfs.contains(&kf_i_id) {
                        loop_query.insert(*kf_i_id, curr_kf_id);
                        kfs_sharing_words.push(*kf_i_id);
                    }
                }
                *loop_words.get_mut(kf_i_id).unwrap() += 1;
            }
        }
        if kfs_sharing_words.is_empty() {
            return vec![];
        }

        // Only compare against those keyframes that share enough words
        let mut max_common_words = 0;
        for kf_id in &kfs_sharing_words {
            match loop_words.get(&kf_id) {
                Some(words) => {
                    if *words > max_common_words {
                        max_common_words = *words;
                    }
                },
                None => continue
            }
        }

        let min_common_words = (max_common_words as f32 * 0.8) as i32;
        let mut n_scores = 0;
        let mut score_and_match = vec![];

        // Compute similarity score. Retain the matches whose score is higher than minScore
        let mut loop_score = HashMap::new(); // mLoopScore
        for kf_i_id in &kfs_sharing_words {
            if loop_words[&kf_i_id] > min_common_words {
                n_scores += 1;
                let kf_i = map_read_lock.keyframes.get(&kf_i_id).unwrap();
                let si = VOCABULARY.score(curr_kf, kf_i);
                loop_score.insert(kf_i_id, si);
                if si >= min_score {
                    score_and_match.push((si, kf_i_id));
                }
            }
        }

        if score_and_match.is_empty() {
            return vec![];
        }

        // Lets now accumulate score by covisibility
        let mut acc_score_and_match = vec![];
        let mut best_acc_score = min_score;
        for (score, kf_i_id) in score_and_match {
            let kf_i = map_read_lock.keyframes.get(&kf_i_id).unwrap();
            let neighbor_kfs = kf_i.get_covisibility_keyframes(10);
            let mut best_score = score;
            let mut acc_score = best_score;
            let mut best_kf_id = *kf_i_id;

            for kf_2_id in neighbor_kfs {
                let loop_words = *loop_words.get(&kf_2_id).unwrap();
                let loop_query = *loop_query.get(&kf_2_id).unwrap();
                let loop_score = *loop_score.get(&kf_2_id).unwrap();
                if loop_query == curr_kf_id  && loop_words > min_common_words {
                    acc_score += loop_score;
                    if loop_score > best_score {
                        best_kf_id = kf_2_id;
                        best_score = loop_score;
                    }
                }
            }

            acc_score_and_match.push((acc_score, best_kf_id));
            if acc_score > best_acc_score {
                best_acc_score = acc_score;
            }
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        let mut loop_candidates = vec![];
        let min_score_to_retain = 0.75 * best_acc_score;
        let mut already_added_kf = HashSet::new();
        for (si, kf_i_id) in acc_score_and_match {
            if si > min_score_to_retain {
                if !already_added_kf.contains(&kf_i_id) {
                    loop_candidates.push(kf_i_id);
                    already_added_kf.insert(kf_i_id);
                }
            }
        }
        return loop_candidates;
    }

    pub fn detect_relocalization_candidates(&self, keyframe_id: Id) -> Vec<Id> {
        // std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);

        todo!("Relocalization")
    }
}
