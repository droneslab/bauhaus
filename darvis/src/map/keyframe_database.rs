use core::maplock::{UnlockedToRead, ReadWriteMap};
use std::sync::Arc;

use parking_lot::RwLock;

use crate::map::map::Id;

use super::{keyframe::KeyFrame, map::Map, bow::VOCABULARY};

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
        let word_ids = kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
        for word_id in word_ids {
            let mut inverted_file = self.inverted_file.write();
            inverted_file[word_id as usize].push(kf.id);
        }
    }

    pub fn erase(&mut self, kf: &KeyFrame) {
        // Erase elements in the Inverse File for the entry
        let word_ids = kf.bow.as_ref().unwrap().bow_vec.get_all_word_ids();
        for word_id in word_ids {
            // List of keyframes that share the word
            self.inverted_file.write()[word_id as usize].retain(|&x| x != kf.id);
            // TODO double check this code produces the right output! Not sure what the original is doing
            // void KeyFrameDatabase::erase
        }
    }

    pub fn clear(&mut self) {
        self.inverted_file.write().clear();
        self.inverted_file.write().resize(VOCABULARY.size(), Vec::new());
    }

    pub fn clear_map(&mut self) {
        todo!("LOOP CLOSING might be just for multimaps")
        // // Erase elements in the Inverse File for the entry
        // for(std::vector<list<KeyFrame*> >::iterator vit=mvInvertedFile.begin(), vend=mvInvertedFile.end(); vit!=vend; vit++)
        // {
        //     // List of keyframes that share the word
        //     list<KeyFrame*> &lKFs =  *vit;

        //     for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend;)
        //     {
        //         KeyFrame* pKFi = *lit;
        //         if(pMap == pKFi->GetMap())
        //         {
        //             lit = lKFs.erase(lit);
        //             // Dont delete the KF because the class Map clean all the KF when it is destroyed
        //         }
        //         else
        //         {
        //             ++lit;
        //         }
        //     }
        // }
    }

    pub fn detect_candidates(&self, keyframe_id: Id, min_score: f32, loop_candidates: Vec<Id>, merge_candidates: Vec<Id>) -> Vec<Id> {
        todo!("LOOP CLOSING")
        // set<KeyFrame*> spConnectedKeyFrames = pKF->GetConnectedKeyFrames();
        // list<KeyFrame*> lKFsSharingWordsLoop,lKFsSharingWordsMerge;

        // // Search all keyframes that share a word with current keyframes
        // // Discard keyframes connected to the query keyframe
        // {
        //     unique_lock<mutex> lock(mMutex);

        //     for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        //     {
        //         list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];

        //         for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        //         {
        //             KeyFrame* pKFi=*lit;
        //             if(pKFi->GetMap()==pKF->GetMap()) // For consider a loop candidate it a candidate it must be in the same map
        //             {
        //                 if(pKFi->mnLoopQuery!=pKF->mnId)
        //                 {
        //                     pKFi->mnLoopWords=0;
        //                     if(!spConnectedKeyFrames.count(pKFi))
        //                     {
        //                         pKFi->mnLoopQuery=pKF->mnId;
        //                         lKFsSharingWordsLoop.push_back(pKFi);
        //                     }
        //                 }
        //                 pKFi->mnLoopWords++;
        //             }
        //             else if(!pKFi->GetMap()->IsBad())
        //             {
        //                 if(pKFi->mnMergeQuery!=pKF->mnId)
        //                 {
        //                     pKFi->mnMergeWords=0;
        //                     if(!spConnectedKeyFrames.count(pKFi))
        //                     {
        //                         pKFi->mnMergeQuery=pKF->mnId;
        //                         lKFsSharingWordsMerge.push_back(pKFi);
        //                     }
        //                 }
        //                 pKFi->mnMergeWords++;
        //             }
        //         }
        //     }
        // }

        // if(lKFsSharingWordsLoop.empty() && lKFsSharingWordsMerge.empty())
        //     return;

        // if(!lKFsSharingWordsLoop.empty())
        // {
        //     list<pair<float,KeyFrame*> > lScoreAndMatch;

        //     // Only compare against those keyframes that share enough words
        //     int maxCommonWords=0;
        //     for(list<KeyFrame*>::iterator lit=lKFsSharingWordsLoop.begin(), lend= lKFsSharingWordsLoop.end(); lit!=lend; lit++)
        //     {
        //         if((*lit)->mnLoopWords>maxCommonWords)
        //             maxCommonWords=(*lit)->mnLoopWords;
        //     }

        //     int minCommonWords = maxCommonWords*0.8f;

        //     int nscores=0;

        //     // Compute similarity score. Retain the matches whose score is higher than minScore
        //     for(list<KeyFrame*>::iterator lit=lKFsSharingWordsLoop.begin(), lend= lKFsSharingWordsLoop.end(); lit!=lend; lit++)
        //     {
        //         KeyFrame* pKFi = *lit;

        //         if(pKFi->mnLoopWords>minCommonWords)
        //         {
        //             nscores++;

        //             float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

        //             pKFi->mLoopScore = si;
        //             if(si>=minScore)
        //                 lScoreAndMatch.push_back(make_pair(si,pKFi));
        //         }
        //     }

        //     if(!lScoreAndMatch.empty())
        //     {
        //         list<pair<float,KeyFrame*> > lAccScoreAndMatch;
        //         float bestAccScore = minScore;

        //         // Lets now accumulate score by covisibility
        //         for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
        //         {
        //             KeyFrame* pKFi = it->second;
        //             vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        //             float bestScore = it->first;
        //             float accScore = it->first;
        //             KeyFrame* pBestKF = pKFi;
        //             for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        //             {
        //                 KeyFrame* pKF2 = *vit;
        //                 if(pKF2->mnLoopQuery==pKF->mnId && pKF2->mnLoopWords>minCommonWords)
        //                 {
        //                     accScore+=pKF2->mLoopScore;
        //                     if(pKF2->mLoopScore>bestScore)
        //                     {
        //                         pBestKF=pKF2;
        //                         bestScore = pKF2->mLoopScore;
        //                     }
        //                 }
        //             }

        //             lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        //             if(accScore>bestAccScore)
        //                 bestAccScore=accScore;
        //         }

        //         // Return all those keyframes with a score higher than 0.75*bestScore
        //         float minScoreToRetain = 0.75f*bestAccScore;

        //         set<KeyFrame*> spAlreadyAddedKF;
        //         vpLoopCand.reserve(lAccScoreAndMatch.size());

        //         for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
        //         {
        //             if(it->first>minScoreToRetain)
        //             {
        //                 KeyFrame* pKFi = it->second;
        //                 if(!spAlreadyAddedKF.count(pKFi))
        //                 {
        //                     vpLoopCand.push_back(pKFi);
        //                     spAlreadyAddedKF.insert(pKFi);
        //                 }
        //             }
        //         }
        //     }

        // }

        // if(!lKFsSharingWordsMerge.empty())
        // {
        //     list<pair<float,KeyFrame*> > lScoreAndMatch;

        //     // Only compare against those keyframes that share enough words
        //     int maxCommonWords=0;
        //     for(list<KeyFrame*>::iterator lit=lKFsSharingWordsMerge.begin(), lend=lKFsSharingWordsMerge.end(); lit!=lend; lit++)
        //     {
        //         if((*lit)->mnMergeWords>maxCommonWords)
        //             maxCommonWords=(*lit)->mnMergeWords;
        //     }

        //     int minCommonWords = maxCommonWords*0.8f;

        //     int nscores=0;

        //     // Compute similarity score. Retain the matches whose score is higher than minScore
        //     for(list<KeyFrame*>::iterator lit=lKFsSharingWordsMerge.begin(), lend=lKFsSharingWordsMerge.end(); lit!=lend; lit++)
        //     {
        //         KeyFrame* pKFi = *lit;

        //         if(pKFi->mnMergeWords>minCommonWords)
        //         {
        //             nscores++;

        //             float si = mpVoc->score(pKF->mBowVec,pKFi->mBowVec);

        //             pKFi->mMergeScore = si;
        //             if(si>=minScore)
        //                 lScoreAndMatch.push_back(make_pair(si,pKFi));
        //         }
        //     }

        //     if(!lScoreAndMatch.empty())
        //     {
        //         list<pair<float,KeyFrame*> > lAccScoreAndMatch;
        //         float bestAccScore = minScore;

        //         // Lets now accumulate score by covisibility
        //         for(list<pair<float,KeyFrame*> >::iterator it=lScoreAndMatch.begin(), itend=lScoreAndMatch.end(); it!=itend; it++)
        //         {
        //             KeyFrame* pKFi = it->second;
        //             vector<KeyFrame*> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

        //             float bestScore = it->first;
        //             float accScore = it->first;
        //             KeyFrame* pBestKF = pKFi;
        //             for(vector<KeyFrame*>::iterator vit=vpNeighs.begin(), vend=vpNeighs.end(); vit!=vend; vit++)
        //             {
        //                 KeyFrame* pKF2 = *vit;
        //                 if(pKF2->mnMergeQuery==pKF->mnId && pKF2->mnMergeWords>minCommonWords)
        //                 {
        //                     accScore+=pKF2->mMergeScore;
        //                     if(pKF2->mMergeScore>bestScore)
        //                     {
        //                         pBestKF=pKF2;
        //                         bestScore = pKF2->mMergeScore;
        //                     }
        //                 }
        //             }

        //             lAccScoreAndMatch.push_back(make_pair(accScore,pBestKF));
        //             if(accScore>bestAccScore)
        //                 bestAccScore=accScore;
        //         }

        //         // Return all those keyframes with a score higher than 0.75*bestScore
        //         float minScoreToRetain = 0.75f*bestAccScore;

        //         set<KeyFrame*> spAlreadyAddedKF;
        //         vpMergeCand.reserve(lAccScoreAndMatch.size());

        //         for(list<pair<float,KeyFrame*> >::iterator it=lAccScoreAndMatch.begin(), itend=lAccScoreAndMatch.end(); it!=itend; it++)
        //         {
        //             if(it->first>minScoreToRetain)
        //             {
        //                 KeyFrame* pKFi = it->second;
        //                 if(!spAlreadyAddedKF.count(pKFi))
        //                 {
        //                     vpMergeCand.push_back(pKFi);
        //                     spAlreadyAddedKF.insert(pKFi);
        //                 }
        //             }
        //         }
        //     }

        // }

        // for(DBoW2::BowVector::const_iterator vit=pKF->mBowVec.begin(), vend=pKF->mBowVec.end(); vit != vend; vit++)
        // {
        //     list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];

        //     for(list<KeyFrame*>::iterator lit=lKFs.begin(), lend= lKFs.end(); lit!=lend; lit++)
        //     {
        //         KeyFrame* pKFi=*lit;
        //         pKFi->mnLoopQuery=-1;
        //         pKFi->mnMergeQuery=-1;
        //     }
        // }


    }

    pub fn detect_best_candidates(&self, keyframe_id: Id) -> Vec<Id> {
        todo!("LOOP CLOSING")
        // void DetectBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nMinWords);
    }

    pub fn detect_n_best_candidates(
        &self, map: &ReadWriteMap<Map>, current_kf_id: Id, num_candidates: usize
    ) -> (Vec<Id>, Vec<Id>) {
        todo!("LOOP CLOSING")
        // void DetectNBestCandidates(KeyFrame *pKF, vector<KeyFrame*> &vpLoopCand, vector<KeyFrame*> &vpMergeCand, int nNumCandidates);
    }

    pub fn detect_relocalization_candidates(&self, keyframe_id: Id) -> Vec<Id> {
        todo!("Relocalization")
        // std::vector<KeyFrame*> DetectRelocalizationCandidates(Frame* F, Map* pMap);
    }
}