import cv2
import numpy as np
from sirius_navigation.theta_semantic_temporal import Observation,warp_with_flow,fuse_road_labels,CausalRoadConsistency


def evidence(cid,n=4,score=.9):
    return [(np.full((8,8),cid,np.uint8),np.full((8,8),score,np.float32),np.ones((8,8),np.float32)) for _ in range(n)]


def test_consistent_neighbors_correct_single_frame_road_sidewalk_swap():
    labels=np.full((8,8),5,np.uint8);scores=np.full((8,8),180,np.uint8)
    fixed,confidence,stats=fuse_road_labels(labels,scores,evidence(6))
    assert np.all(fixed==6) and stats['changed_pixels']==64
    assert confidence.min()>=127


def test_insufficient_conflicting_or_unmatched_evidence_preserves_current():
    labels=np.full((8,8),5,np.uint8);scores=np.full((8,8),180,np.uint8)
    for rows in [evidence(6,2),evidence(6,2)+evidence(5,2),[(ids,s,np.zeros_like(w)) for ids,s,w in evidence(6)]]:
        fixed,_,stats=fuse_road_labels(labels,scores,rows)
        np.testing.assert_array_equal(fixed,labels);assert stats['changed_pixels']==0


def test_grass_tactile_unknown_and_walls_cannot_be_overwritten():
    labels=np.tile(np.array([0,1,2,3,4,5,6,0],np.uint8),(8,1));scores=np.full((8,8),180,np.uint8)
    fixed,_,_=fuse_road_labels(labels,scores,evidence(6))
    protected=np.isin(labels,[0,1,2,3,4]);np.testing.assert_array_equal(fixed[protected],labels[protected])


def test_occlusion_and_wrong_correspondence_are_rejected():
    rng=np.random.default_rng(8);image=rng.integers(0,255,(32,32,3),dtype=np.uint8)
    old=Observation(image.copy(),np.full((32,32),6,np.uint8),np.full((32,32),230,np.uint8),0)
    zero=np.zeros((32,32,2),np.float32)
    _,_,good=warp_with_flow(image,old,zero,zero);assert good[4:-4,4:-4].min()>.99
    wrong=zero.copy();wrong[...,0]=8
    _,_,bad=warp_with_flow(image,old,zero,wrong);assert not bad.any()
    old.image=255-image
    _,_,occluded=warp_with_flow(image,old,zero,zero);assert (occluded==0).mean()>.35


def test_warp_tracks_translation_instead_of_same_image_coordinates():
    rng=np.random.default_rng(9);oldimg=rng.integers(0,255,(32,32,3),dtype=np.uint8)
    current=np.roll(oldimg,3,axis=1);labels=np.zeros((32,32),np.uint8);labels[8:20,8:15]=6
    old=Observation(oldimg,labels,np.full((32,32),230,np.uint8),0)
    backward=np.zeros((32,32,2),np.float32);backward[...,0]=-3;forward=-backward
    ids,_,valid=warp_with_flow(current,old,backward,forward)
    assert ids[12,16]==6 and labels[12,16]==0 and valid[12,16]>.99


def test_history_resets_on_replay_or_gap_and_keeps_raw_masks():
    image=np.zeros((8,8,3),np.uint8);labels=np.full((8,8),5,np.uint8);scores=np.full((8,8),180,np.uint8)
    tracker=CausalRoadConsistency();tracker.apply(image,labels,scores,1)
    tracker.apply(image,labels,scores,4);assert len(tracker.history)==1
    tracker.apply(image,labels,scores,2);assert len(tracker.history)==1
    labels[:]=6;assert np.all(tracker.history[-1].labels==5)


def test_majority_cannot_overwrite_more_confident_current_observation():
    labels=np.full((8,8),6,np.uint8);scores=np.full((8,8),184,np.uint8)
    fixed,_,stats=fuse_road_labels(labels,scores,evidence(5,8,score=.70))
    np.testing.assert_array_equal(fixed,labels);assert stats['changed_pixels']==0
