use std::collections::{HashMap, HashSet};

use super::keyframe::{KeyFrame, FullKeyFrame};
use crate::dvmap::{map::Map};
use super::{bow::{BoW, self}};
use dvcore::{
    lockwrap::ReadOnlyWrapper,
};

pub type WordId = u32;

#[derive(Clone, Debug)]
pub struct KeyFrameDatabase {
    inverted_file: HashMap<WordId, Vec<i32>>,
    map: ReadOnlyWrapper<Map>,
}

impl KeyFrameDatabase {
    pub fn new(map: ReadOnlyWrapper<Map>) -> Self {
        Self { map,inverted_file: HashMap::new() }
    }

    pub fn add(&mut self, keyframe : KeyFrame<FullKeyFrame>)
    {

        let bow_vec = keyframe.bow.unwrap().get_word_vec();
        for bow in bow_vec
        {
            self.inverted_file[&bow].push(keyframe.id());
        }
        
    }
    pub fn erase(&mut self, keyframe : KeyFrame<FullKeyFrame>)
    {

        let bow_vec = keyframe.bow.unwrap().get_word_vec();
        for bow in bow_vec
        {

            let keyframe_ids = self.inverted_file[&bow];

            // Ignore if no such element is found
            if let Some(pos) = keyframe_ids.iter().position(|x| *x == keyframe.id()) {
                keyframe_ids.remove(pos);
            }
        }
        
    }

    pub fn clear(&mut self)
    {
        self.inverted_file.clear();
    }



    //void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates);
    pub fn detect_n_best_candidates(&mut self, keyframe : KeyFrame<FullKeyFrame>, loop_cand : Vec<i32>, merge_cand: Vec<i32>, num_candidates : i32)
    {

        let mut lKFsSharingWords = Vec::new();
        let mut spConnectedKF = Vec::new();

        let mut place_recog_query_kf_map: HashMap<&i32, i32> = HashMap::new();
        let mut place_recog_words_kf_map: HashMap<&i32, i32> = HashMap::new();
        let mut place_recog_score_kf_map: HashMap<&i32, f64> = HashMap::new();
    
        // Search all keyframes that share a word with current frame
        {
            //unique_lock<mutex> lock(mMutex);
    
            //spConnectedKF = pKF->GetConnectedKeyFrames();
            //Pranay : setting 30 for num of connections
            spConnectedKF = keyframe.get_connections(30);
    
            let bow_vec = keyframe.bow.unwrap().get_word_vec();
            for bow in bow_vec
            {
            // for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
            // {

                let keyframe_ids = self.inverted_file[&bow];

                for kfi in keyframe_ids
                {

                    if !place_recog_query_kf_map.contains_key(&kfi)
                    {
                        place_recog_query_kf_map.insert(&kfi, -1);

                    }

                    if place_recog_query_kf_map[&kfi]!=keyframe.id()
                    {
                        place_recog_words_kf_map[&kfi]=0;
                        if !spConnectedKF.contains(&kfi)
                        {
                            place_recog_query_kf_map[&kfi] = keyframe.id();
                            lKFsSharingWords.push(kfi);
                        }
                    }
                    place_recog_words_kf_map[&kfi]+=1;

                }
            
            }
        }

        if lKFsSharingWords.is_empty()
        {
            return;
        }
            
    
        // Only compare against those keyframes that share enough words
        let maxCommonWords=0;

        for kfi in lKFsSharingWords
        {
            if place_recog_words_kf_map[&kfi] > maxCommonWords
            {
                maxCommonWords = place_recog_words_kf_map[&kfi];
            }
        }

    
        let minCommonWords = maxCommonWords as f64 *0.8;
    
        //list<pair<float,KeyFrame*> > lScoreAndMatch;
        let mut lScoreAndMatch = Vec::new();
    
        let mut nscores=0;
        

        // Compute similarity score.
        for kfi in lKFsSharingWords
        {
            if place_recog_words_kf_map[&kfi] as f64 > minCommonWords
            {
                nscores+=1;
                
                let kfi_kf = self.map.read().get_keyframe(&kfi).unwrap();
                let si = bow::VOCABULARY.score(&keyframe.bow.unwrap(), &kfi_kf.bow.unwrap());
                place_recog_score_kf_map[&kfi]=si;
                lScoreAndMatch.push((si,kfi));
            }
        }

        if lScoreAndMatch.is_empty()
        {
            return;
        }

    
        
        let mut lAccScoreAndMatch = Vec::new(); // list<pair<float,KeyFrame*> > lAccScoreAndMatch;
        let bestAccScore : f64 = 0.0; // float bestAccScore = 0;
        

        // // Lets now accumulate score by covisibility
        for (scr, kfi) in lScoreAndMatch
        {
            let kfi_kf = self.map.read().get_keyframe(&kfi).unwrap();
            let neighs_kfi = kfi_kf.get_connections(10); //vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);
    
            let bestScore = scr;
            let accScore = bestScore;
            
            let best_kfi = kfi; //KeyFrame* pBestKF = pKFi;

            for kf2 in neighs_kfi // for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
            {
                if place_recog_query_kf_map[&kf2] != keyframe.id() // if(pKF2->mnPlaceRecognitionQuery!=pKF->mnId)
                {
                    continue;
                }
                    
    
                accScore+= place_recog_score_kf_map[&kf2]; //  pKF2->mPlaceRecognitionScore;
                if place_recog_score_kf_map[&kf2]> bestScore //if(pKF2->mPlaceRecognitionScore>bestScore)
                {
                    best_kfi = kf2; //pBestKF=pKF2;
                    bestScore = place_recog_score_kf_map[&kf2]; //pKF2->mPlaceRecognitionScore;
                }
    
            }
            lAccScoreAndMatch.push((accScore,best_kfi));//lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
            if accScore > bestAccScore //if(accScore>bestAccScore)
            {
                bestAccScore=accScore;
            }    
        }
    
        lAccScoreAndMatch.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap()); // Sorting by score // lAccScoreAndMatch.sort(compFirst);
    
        //loop_cand.reserve(num_candidates as usize); // vpLoopCand.reserve(nNumCandidates);
        //merge_cand.reserve(num_candidates as usize);// vpMergeCand.reserve(nNumCandidates);
        let mut spAlreadyAddedKF: HashSet<&i32> = HashSet::new(); // set<KeyFrame*> spAlreadyAddedKF;

        for (scr, kfi) in lAccScoreAndMatch // while(i < lAccScoreAndMatch.size() && (vpLoopCand.size() < nNumCandidates || vpMergeCand.size() < nNumCandidates))
        {
            if self.map.read().get_keyframe(&kfi).is_none()
            {
                continue;
            }

            if spAlreadyAddedKF.contains(&kfi)
            {
                if loop_cand.len() < num_candidates as usize
                {
                    loop_cand.push(kfi);
                }
                // //Pranay: (TODO) this condition need to be implemented for multi-map systems
                //else if(pKF->GetMap() != pKFi->GetMap() && vpMergeCand.size() < nNumCandidates && !pKFi->GetMap()->IsBad())
                //{
                //    vpMergeCand.push_back(pKFi);
                //}

                spAlreadyAddedKF.insert(&kfi);
            }

            // break if num_candidates is satisfied
            if loop_cand.len() >  num_candidates as usize || merge_cand.len() >  num_candidates as usize 
            {
                break;
            }
        }



    }

}