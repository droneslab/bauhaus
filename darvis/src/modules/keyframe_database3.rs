// Version of KeyFrameDatabase used by ORB-SLAM3
// Not finished

use std::{sync::Arc, collections::{HashSet, HashMap}};

use nalgebra::min;
use parking_lot::RwLock;

use crate::{map::map::Id, MapLock};

use super::{keyframe::KeyFrame, bow::VOCABULARY};

// TODO LOOP CLOSING ... not sure if this should be owned by map or shared with its own mutex
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

    pub fn clear_map(&mut self) {
        // void KeyFrameDatabase::clearMap(Map* pMap)

        todo!("multimaps")
    }

    pub fn detect_candidates(&self, map: &MapLock, curr_kf_id: Id, min_score: f32) -> (Vec<Id>, Vec<Id>) {
        // This function is also very similar to the two in detect_best_candidates, but different enough that it's a pain to combine them
        // void KeyFrameDatabase::DetectCandidates(KeyFrame* pKF, float minScore,vector<KeyFrame*>& vpLoopCand, vector<KeyFrame*>& vpMergeCand)

        let mut loop_candidates = vec![];
        let mut merge_candidates = vec![];

        let map_read_lock = map.read();
        let read_lock = self.inverted_file.read();
        let curr_kf = map_read_lock.keyframes.get(&curr_kf_id).unwrap();
        let connected_kfs = map.read().keyframes.get(&curr_kf_id).unwrap().get_covisibility_keyframes(i32::MAX);

        let mut kfs_sharing_words_loop = vec![]; //lKFsSharingWordsLoop
        let mut loop_words = HashMap::new(); // mnLoopWords
        let mut loop_query = HashMap::new(); // mnLoopQuery
        let mut kfs_sharing_words_merge: Vec<i32> = vec![]; //lKFsSharingWordsMerge
        let mut _merge_words: HashMap<i32, i32> = HashMap::new(); // mnMergeWords
        let mut _merge_query: HashMap<i32, i32> = HashMap::new(); // mnMergeQuery

        // Search all keyframes that share a word with current frame
        // Discard keyframes connected to the query keyframe
        for word in curr_kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids() {
            for kf_i_id in &read_lock[word as usize] {
                let kf_i = map_read_lock.keyframes.get(&kf_i_id).unwrap();
                if kf_i.origin_map_id == curr_kf.origin_map_id { // For consider a loop candidate it a candidate it must be in the same map
                    if (loop_query.get(kf_i_id).is_some() && loop_query[kf_i_id] != curr_kf_id)
                        || loop_query.get(kf_i_id).is_none() {
                        loop_words.insert(*kf_i_id, 0);
                        if !connected_kfs.contains(&kf_i_id) {
                            loop_query.insert(*kf_i_id, curr_kf_id);
                            kfs_sharing_words_loop.push(*kf_i_id);
                        }
                    }
                    *loop_words.get_mut(kf_i_id).unwrap() += 1;
                }
                // TODO (multimaps) .. add the following:
                // else if(!pKFi->GetMap()->IsBad())
                // {
                //     if(pKFi->mnMergeQuery!=pKF->mnId)
                //     {
                //         pKFi->mnMergeWords=0;
                //         if(!spConnectedKeyFrames.count(pKFi))
                //         {
                //             pKFi->mnMergeQuery=pKF->mnId;
                //             lKFsSharingWordsMerge.push_back(pKFi);
                //         }
                //     }
                //     pKFi->mnMergeWords++;
            }
        }
        if kfs_sharing_words_loop.is_empty() && kfs_sharing_words_merge.is_empty() {
            return (loop_candidates, merge_candidates);
        }

        // Only compare against those keyframes that share enough words
        let mut max_common_words = 0;
        for kf_id in &kfs_sharing_words_loop {
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
        for kf_i_id in &kfs_sharing_words_loop {
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

        if !score_and_match.is_empty() {
            let mut acc_score_and_match = vec![];
            let mut best_acc_score = 0.0;
            // Lets now accumulate score by covisibility
            for (score, kf_i_id) in score_and_match {
                let kf_i = map_read_lock.keyframes.get(&kf_i_id).unwrap();
                let neighbor_kfs = kf_i.get_covisibility_keyframes(10);
                let mut best_score = score;
                let mut acc_score = best_score;
                let mut best_kf_id = *kf_i_id;

                for kf_2_id in neighbor_kfs {
                    if loop_query[&kf_2_id] == curr_kf_id  && loop_words[&kf_2_id] > min_common_words {
                        acc_score += loop_score[&kf_2_id];
                        if loop_score[&kf_2_id] > best_score {
                            best_kf_id = kf_2_id;
                            best_score = loop_score[&kf_2_id];
                        }
                    }
                }

                acc_score_and_match.push((acc_score, best_kf_id));
                if acc_score > best_acc_score {
                    best_acc_score = acc_score;
                }
            }

            // Return all those keyframes with a score higher than 0.75*bestScore
            let min_score_to_retain = 0.75 * best_acc_score;
            let mut already_added_kf = HashSet::new();
            for (si, kf_i_id) in acc_score_and_match {
                if si > min_score_to_retain {
                    if !already_added_kf.contains(&kf_i_id) {
                        // TODO (multimaps): add this check back in
                        // if(pKF->GetMap() == pKFi->GetMap())
                        // {
                        //     vpLoopCand.push_back(pKFi);
                        // }
                        // else
                        // {
                        //     vpMergeCand.push_back(pKFi);
                        // }
                        loop_candidates.push(kf_i_id);
                        already_added_kf.insert(kf_i_id);
                    }
                }
            }
        }

        // TODO (multimaps) ... add the same code as above, but for merge candidates.
        // Look for "if(!lKFsSharingWordsMerge.empty())" in KeyFrameDatabase.cc

        // TODO (MVP LOOP CLOSING) ... what is this used for?
        //         for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        // {
        //     list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];

        //     for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        //     {
        //         KeyFrame* pKFi=*lit;
        //         pKFi->mnLoopQuery=-1;
        //         pKFi->mnMergeQuery=-1;
        //     }
        // }

        return (loop_candidates, merge_candidates);
    }

    pub fn detect_best_candidates(&self, map: &MapLock, curr_kf_id: Id, input: DetectBestCandidatesInput) -> (Vec<Id>, Vec<Id>) {
        // Combination of the following functions, which are nearly identical:
        // void KeyFrameDatabase::DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nMinWords);
        // void KeyFrameDatabase::DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates)

        let mut loop_candidates = vec![];
        let mut merge_candidates = vec![];

        let map_read_lock = map.read();
        let read_lock = self.inverted_file.read();
        let curr_kf = map_read_lock.keyframes.get(&curr_kf_id).unwrap();

        // Search all keyframes that share a word with current frame
        let mut kfs_sharing_words = vec![];
        let connected_kfs = map.read().keyframes.get(&curr_kf_id).unwrap().get_covisibility_keyframes(i32::MAX);
        let mut words = HashMap::new(); // mnPlaceRecognitionWords
        let mut query = HashMap::new(); // mnPlaceRecognitionQuery


        for word in curr_kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids() {
            for kf_i_id in &read_lock[word as usize] {
                if (query.get(kf_i_id).is_some() && query[kf_i_id] != curr_kf_id)
                    || query.get(kf_i_id).is_none() {
                    words.insert(*kf_i_id, 0);
                    if !connected_kfs.contains(&kf_i_id) {
                        query.insert(*kf_i_id, curr_kf_id);
                        kfs_sharing_words.push(*kf_i_id);
                    }
                }
                *words.get_mut(kf_i_id).unwrap() += 1;
            }
        }
        if kfs_sharing_words.is_empty() { // todo (multimaps) For MinScore version (DetectCandidates(, add check for kfs_sharing_words_merge.is_empty()
            return (loop_candidates, merge_candidates);
        }

        // Only compare against those keyframes that share enough words
        let mut max_common_words = 0;
        for kf_id in &kfs_sharing_words {
            match words.get(&kf_id) {
                Some(words) => {
                    if *words > max_common_words {
                        max_common_words = *words;
                    }
                },
                None => continue
            }
        }


        let min_common_words = match input {
            DetectBestCandidatesInput::MinWords(min_words) => min((max_common_words as f32 * 0.8) as i32, min_words),
            DetectBestCandidatesInput::NumCandidates(_) => (max_common_words as f32 * 0.8) as i32,
        };
        let mut score_and_match = vec![];
        let mut n_scores = 0;

        // Compute similarity score.
        let mut kf_place_recognition_score = HashMap::new();

        for kf_i_id in kfs_sharing_words {
            if words[&kf_i_id] > min_common_words {
                n_scores += 1;
                let kf_i = map_read_lock.keyframes.get(&kf_i_id).unwrap();
                let si = VOCABULARY.score(curr_kf, kf_i);
                score_and_match.push((si, kf_i_id));
                kf_place_recognition_score.insert(kf_i_id, si);
            }
        }

        if score_and_match.is_empty() {
            return (loop_candidates, merge_candidates);
        }
        
        let mut acc_score_and_match = vec![];
        let mut best_acc_score = 0.0;

        // Lets now accumulate score by covisibility
        for (score, kf_i_id) in score_and_match {
            let kf_i = map_read_lock.keyframes.get(&kf_i_id).unwrap();
            let neighbor_kfs = kf_i.get_covisibility_keyframes(10);
            let mut best_score = score;
            let mut acc_score = best_score;
            let mut best_kf_id = kf_i_id;

            for kf_2_id in neighbor_kfs {
                if query[&kf_2_id] != curr_kf_id {
                    continue;
                }

                acc_score += kf_place_recognition_score[&kf_2_id];
                if kf_place_recognition_score[&kf_2_id] > best_score {
                    best_kf_id = kf_2_id;
                    best_score = kf_place_recognition_score[&kf_2_id];
                }
            }

            acc_score_and_match.push((acc_score, best_kf_id));
            if acc_score > best_acc_score {
                best_acc_score = acc_score;
            }
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        let min_score_to_retain = 0.75 * best_acc_score;
        let mut already_added_kf = HashSet::new();
        let mut loop_candidates = vec![];
        for (si, kf_i_id) in acc_score_and_match {
            match input {
                DetectBestCandidatesInput::MinWords(_) => {
                    if si > min_score_to_retain {
                        let kf_i = map_read_lock.keyframes.get(&kf_i_id).unwrap();
                        if !already_added_kf.contains(&kf_i_id) {
                            // TODO(multimaps): add this check back in
                            // if(pKF->GetMap() == pKFi->GetMap())
                            // {
                            //     vpLoopCand.push_back(pKFi);
                            // }
                            // else
                            // {
                            //     vpMergeCand.push_back(pKFi);
                            // }
                            loop_candidates.push(kf_i_id);
                            already_added_kf.insert(kf_i_id);
                        }
                    }
                },
                DetectBestCandidatesInput::NumCandidates(max_candidates) => {
                    if loop_candidates.len() as i32 >= max_candidates && merge_candidates.len() as i32 >= max_candidates {
                        break;
                    }
                    if !already_added_kf.contains(&kf_i_id) {
                        if (loop_candidates.len() as i32) < max_candidates {
                            loop_candidates.push(kf_i_id);
                        }
                        // TODO (multimaps) add the stuff below:
                        // else if(pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < nNumCandidates && !pKFi->GetMap()->IsBad())
                        // {
                        //     vpMergeCand.push_back(pKFi);
                        // }

                        already_added_kf.insert(kf_i_id);
                    }
                },
            };
        }
        return (loop_candidates, merge_candidates);
    }


    pub fn detect_relocalization_candidates(&self, keyframe_id: Id) -> Vec<Id> {
        // std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);

        todo!("Relocalization")
    }
}


// ORB-SLAM3 Loop Closing
pub enum DetectBestCandidatesInput {
    MinWords(i32), // equivalent to DetectBestCandidates function
    NumCandidates(i32), // = DetectNBestCandidates
}