//*** KEYFRAME DATABASE FROM ORBSLAM2 ***/

use std::collections::{HashSet, HashMap};

use log::debug;

use crate::{map::{keyframe::KeyFrame, map::Id}, modules::module_definitions::BoWModule, registered_actors::VOCABULARY_MODULE};
use crate::modules::module_definitions::VocabularyModule;

use super::{frame::Frame, map::Map};


#[derive(Debug, Clone)]
pub struct KeyFrameDatabase {
    inverted_file: Vec<Vec<Id>> // mvInvertedFile
}

impl KeyFrameDatabase {
    pub fn new() -> Self {
        let vocab_size = VOCABULARY_MODULE.size();
        KeyFrameDatabase { 
            inverted_file: vec![vec![]; vocab_size]
        }
    }

    pub fn add(&mut self, kf: &KeyFrame) {
        // void KeyFrameDatabase::add(KeyFrame *pKF)
        let word_ids = kf.bow.as_ref().unwrap().get_bow_vec().get_all_word_ids();
        for word_id in word_ids {
            self.inverted_file[word_id as usize].push(kf.id);
        }
    }

    pub fn erase(&mut self, kf: &KeyFrame) {
        // void KeyFrameDatabase::erase

        // Erase elements in the Inverse File for the entry
        let word_ids = kf.bow.as_ref().unwrap().get_bow_vec().get_all_word_ids();
        for word_id in word_ids {
            // List of keyframes that share the word
            self.inverted_file[word_id as usize].retain(|&id| id != kf.id);
        }
    }

    pub fn clear(&mut self) {
        self.inverted_file.clear();
        self.inverted_file.resize(VOCABULARY_MODULE.size(), Vec::new());
    }

    pub fn detect_n_best_candidates(&self, map: &Map, curr_kf_id: &Id, num_candidates: i32) -> (Vec<Id>, Vec<Id>) {
        // From ORBSLAM3:
        // void KeyFrameDatabase::DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates)

        let curr_kf = map.get_keyframe(*curr_kf_id);
        let connected_kfs = map.get_keyframe(*curr_kf_id).get_connected_keyframes();
        let mut kfs_sharing_words = vec![]; //lKFsSharingWords
        let mut place_recognition_query = HashMap::new(); // mnPlaceRecognitionQuery
        let mut place_recognition_words = HashMap::new(); // mnPlaceRecognitionWords

        // Search all keyframes that share a word with current frame
        for word in curr_kf.bow.as_ref().unwrap().get_bow_vec().get_all_word_ids() {
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

        // debug!("Detect loop candidates... max common words: {}, min common words: {}, max word kf id: {}", max_common_words, min_common_words, max_word_kf_id);

        // Compute similarity score.
        let mut place_recognition_score = HashMap::new(); // mPlaceRecognitionScore
        for kf_i_id in &kfs_sharing_words {
            if place_recognition_words[&kf_i_id] > min_common_words  {
                let kf_i = map.get_keyframe(*kf_i_id);
                let si = VOCABULARY_MODULE.score(curr_kf.bow.as_ref().unwrap(), kf_i.bow.as_ref().unwrap());
                place_recognition_score.insert(kf_i_id, si);
                score_and_match.push((si, kf_i_id));
            }
        }

        if score_and_match.is_empty() {
            return (vec![], vec![]);
        }

        // Lets now accumulate score by covisibility
        let mut acc_score_and_match = vec![]; // lAccScoreAndMatch
        let mut best_acc_score = 0.0;
        for (score, kf_i_id) in score_and_match {
            let kf_i = map.get_keyframe(*kf_i_id);
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
            let (_score, kf_id) = acc_score_and_match[i];
            if !already_added_kf.contains(&kf_id) {
                loop_candidates.push(kf_id);
                already_added_kf.insert(kf_id);
            }
            i+= 1;
        }

        return (merge_candidates, loop_candidates);
    }

    pub fn detect_candidates_above_score(&self, map: &Map, curr_kf_id: &Id, min_score: f32) -> Vec<Id> {
        // From ORBSLAM2:
        // vector<KeyFrame*> KeyFrameDatabase::DetectLoopCandidates(KeyFrame* pKF, float minScore)

        let curr_kf = map.get_keyframe(*curr_kf_id);
        let connected_kfs = map.get_keyframe(*curr_kf_id).get_connected_keyframes();
        let mut kfs_sharing_words = vec![]; //lKFsSharingWords
        let mut loop_words = HashMap::new(); // mnLoopWords
        let mut loop_query = HashMap::new(); // mnLoopQuery

        // debug!("Detect loop candidates... min score: {}", min_score);

        // Search all keyframes that share a word with current keyframes
        // Discard keyframes connected to the query keyframe
        for word in curr_kf.bow.as_ref().unwrap().get_bow_vec().get_all_word_ids() {
            for kf_i_id in &self.inverted_file[word as usize] {
                let different_loop_query = loop_query.get(kf_i_id).is_some() && loop_query[kf_i_id] != curr_kf_id;
                let no_loop_query = loop_query.get(kf_i_id).is_none();

                if different_loop_query || no_loop_query {
                    loop_words.insert(*kf_i_id, 0);
                    if !connected_kfs.contains_key(&kf_i_id) {
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
        let mut max_word_kf_id = -1;
        for kf_id in &kfs_sharing_words {
            match loop_words.get(&kf_id) {
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

        // Compute similarity score. Retain the matches whose score is higher than minScore
        let mut loop_score = HashMap::new(); // mLoopScore
        for kf_i_id in &kfs_sharing_words {
            if loop_words[&kf_i_id] > min_common_words  {
                let kf_i = map.get_keyframe(*kf_i_id);
                let si = VOCABULARY_MODULE.score(curr_kf.bow.as_ref().unwrap(), kf_i.bow.as_ref().unwrap());
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
            let kf_i = map.get_keyframe(*kf_i_id);
            let neighbor_kfs = kf_i.get_covisibility_keyframes(10);
            let mut best_score = score;
            let mut acc_score = best_score;
            let mut best_kf_id = *kf_i_id;

            for kf_2_id in neighbor_kfs {
                let loop_words = loop_words.get(&kf_2_id);
                let loop_query = loop_query.get(&kf_2_id);
                let loop_score = loop_score.get(&kf_2_id);
                if loop_words.is_none() || loop_query.is_none() || loop_score.is_none() {
                    continue;
                }
                let loop_words = *loop_words.unwrap();
                let loop_query = *loop_query.unwrap();
                let loop_score = *loop_score.unwrap();
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

    pub fn detect_relocalization_candidates(&self, map: &Map, frame: &Frame) -> Vec<Id> {
        // std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);
        let mut kfs_sharing_words = vec![]; //lKFsSharingWords

        let mut reloc_queries = HashMap::new(); // mnRelocQuery
        let mut reloc_words = HashMap::new(); // mnRelocWords

        let word_ids = frame.bow.as_ref().unwrap().get_bow_vec().get_all_word_ids();
        for word_id in word_ids {
            let kfs = &self.inverted_file[word_id as usize];

            for kf_i_id in kfs {
                let different_query = reloc_queries.get(kf_i_id).is_some() && reloc_queries[kf_i_id] != frame.frame_id;
                let no_query = reloc_queries.get(kf_i_id).is_none();

                if different_query || no_query {
                    reloc_queries.insert(*kf_i_id, frame.frame_id);
                    reloc_words.insert(*kf_i_id, 0);
                    kfs_sharing_words.push(*kf_i_id);
                }
                *reloc_words.get_mut(kf_i_id).unwrap() += 1;
            }
        }

        if kfs_sharing_words.is_empty() {
            return vec![];
        }

        // Only compare against those keyframes that share enough words
        let mut max_common_words = 0;
        for kf_id in &kfs_sharing_words {
            match reloc_words.get(&kf_id) {
                Some(words) => {
                    if *words > max_common_words {
                        max_common_words = *words;
                    }
                },
                None => continue
            }
        }

        let min_common_words = (max_common_words as f32 * 0.8) as i32;
        let mut score_and_match = vec![];

        // Compute similarity score
        let mut reloc_score = HashMap::new(); // mRelocScore
        for kf_i_id in &kfs_sharing_words {
            if reloc_words[&kf_i_id] > min_common_words  {
                let kf_i = map.get_keyframe(*kf_i_id);
                let si = VOCABULARY_MODULE.score(frame.bow.as_ref().unwrap(), kf_i.bow.as_ref().unwrap());
                reloc_score.insert(kf_i_id, si);
                score_and_match.push((si, kf_i_id));
            }
        }

        if score_and_match.is_empty() {
            return vec![];
        }

        // Lets now accumulate score by covisibility
        let mut acc_score_and_match = vec![];
        let mut best_acc_score = 0.0;
        for (score, kf_i_id) in score_and_match {
            let kf_i = map.get_keyframe(*kf_i_id);
            let neighbor_kfs = kf_i.get_covisibility_keyframes(10);
            let mut best_score = score;
            let mut acc_score = best_score;
            let mut best_kf_id = *kf_i_id;

            for kf_2_id in neighbor_kfs {
                if reloc_queries.get(&kf_2_id).is_some() && reloc_queries[&kf_2_id] != frame.frame_id {
                    continue;
                }
                acc_score += *reloc_score.get(&kf_2_id).unwrap();
                if reloc_score[&kf_2_id] > best_score {
                    best_kf_id = kf_2_id;
                    best_score = reloc_score[&kf_2_id];
                }
            }

            acc_score_and_match.push((acc_score, best_kf_id));
            if acc_score > best_acc_score {
                best_acc_score = acc_score;
            }
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        let mut reloc_candidates = vec![];
        let min_score_to_retain = 0.75 * best_acc_score;
        let mut already_added_kf = HashSet::new();
        for (si, kf_i_id) in acc_score_and_match {
            if si > min_score_to_retain {
                if !already_added_kf.contains(&kf_i_id) {
                    reloc_candidates.push(kf_i_id);
                    already_added_kf.insert(kf_i_id);
                }
            }
        }

        return reloc_candidates;
    }
}
