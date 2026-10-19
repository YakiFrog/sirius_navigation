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


def test_tactile_gap_requires_matching_past_and_future_observations():
    from sirius_navigation.theta_semantic_temporal import stabilize_tactile
    labels=np.full((8,8),6,np.uint8);scores=np.full((8,8),230,np.uint8)
    tactile=(np.full((8,8),4,np.uint8),np.full((8,8),.7,np.float32),np.ones((8,8),np.float32))
    fixed,_,stats=stabilize_tactile(labels,scores,[(*tactile,-.26),(*tactile,.26)])
    assert np.all(fixed==4) and stats['filled_pixels']==64
    fixed,_,_=stabilize_tactile(labels,scores,[(*tactile,-.26),(*tactile,-.52)])
    np.testing.assert_array_equal(fixed,labels)


def test_tactile_does_not_reappear_after_occlusion_or_stale_history():
    from sirius_navigation.theta_semantic_temporal import stabilize_tactile
    labels=np.full((8,8),6,np.uint8);scores=np.full((8,8),230,np.uint8)
    tactile=(np.full((8,8),4,np.uint8),np.full((8,8),.7,np.float32),np.ones((8,8),np.float32))
    for rows in [[(*tactile,-1.),(*tactile,.26)],[(tactile[0],tactile[1],np.zeros((8,8),np.float32),-.26),(*tactile,.26)]]:
        fixed,_,stats=stabilize_tactile(labels,scores,rows);np.testing.assert_array_equal(fixed,labels);assert not stats['filled_pixels']


def test_tactile_preserves_existing_detections_and_grass_even_without_support():
    from sirius_navigation.theta_semantic_temporal import stabilize_tactile
    labels=np.full((8,8),3,np.uint8);labels[2:4]=4;scores=np.full((8,8),230,np.uint8)
    fixed,sc,_=stabilize_tactile(labels,scores,[]);np.testing.assert_array_equal(fixed,labels);np.testing.assert_array_equal(sc,scores)
    tactile=(np.full((8,8),4,np.uint8),np.full((8,8),.7,np.float32),np.ones((8,8),np.float32))
    fixed,_,_=stabilize_tactile(labels,scores,[(*tactile,-.26),(*tactile,.26)]);np.testing.assert_array_equal(fixed,labels)


def test_causal_tactile_requires_two_original_observations_and_expires():
    from sirius_navigation.theta_semantic_temporal import stabilize_tactile
    labels=np.zeros((8,8),np.uint8);scores=np.zeros((8,8),np.uint8)
    tactile=(np.full((8,8),4,np.uint8),np.full((8,8),.7,np.float32),np.ones((8,8),np.float32))
    fixed,_,_=stabilize_tactile(labels,scores,[(*tactile,-.26)],bidirectional=False);assert not fixed.any()
    fixed,_,_=stabilize_tactile(labels,scores,[(*tactile,-.26),(*tactile,-.52)],bidirectional=False);assert np.all(fixed==4)
    fixed,_,_=stabilize_tactile(labels,scores,[(*tactile,-1.),(*tactile,-1.26)],bidirectional=False);assert not fixed.any()


def test_yellow_gap_fill_does_not_expand_gray_pavement_false_detections():
    from sirius_navigation.theta_semantic_temporal import stabilize_tactile,yellow_tactile_appearance
    gray=np.full((8,8,3),128,np.uint8);yellow=gray.copy();yellow[:4]=(20,180,210)
    labels=np.full((8,8),6,np.uint8);scores=np.full((8,8),230,np.uint8)
    tactile=(np.full((8,8),4,np.uint8),np.full((8,8),.8,np.float32),np.ones((8,8),np.float32))
    for image,expected in [(gray,0),(yellow,32)]:
        fixed,_,stats=stabilize_tactile(labels,scores,[(*tactile,-.26),(*tactile,.26)],appearance_mask=yellow_tactile_appearance(image))
        assert stats['filled_pixels']==expected and (fixed==4).sum()==expected


def test_visible_yellow_track_survives_many_detector_misses_then_occlusion():
    from sirius_navigation.theta_semantic_temporal import CausalTactileConsistency
    rng=np.random.default_rng(37);image=np.full((32,32,3),(20,180,210),np.uint8)
    image=np.clip(image.astype(int)+rng.integers(-15,16,(32,32,1)),0,255).astype(np.uint8)
    tracker=CausalTactileConsistency();positive=np.full((32,32),4,np.uint8);unknown=np.zeros_like(positive);score=np.full_like(positive,230)
    tracker.apply(image,positive,score,0.);tracker.apply(image,positive,score,.2)
    filled,_,_=tracker.apply(image,unknown,unknown,.4);assert (filled==4).any()
    for i in range(3,16):
        retained,_,_=tracker.apply(image,unknown,unknown,i*.2)
        assert (retained==4).sum()>500
    assert all(not (x.labels==4).any() for x in tracker.history)
    occluder=np.full_like(image,128)
    cleared,_,_=tracker.apply(occluder,unknown,unknown,3.2);assert not cleared.any()
    # Re-exposing the surface cannot resurrect an occluded track without SAM3.
    cleared,_,_=tracker.apply(image,unknown,unknown,3.4);assert not cleared.any()


def test_gray_tiles_cannot_start_yellow_tactile_tracks_and_grass_is_protected():
    from sirius_navigation.theta_semantic_temporal import CausalTactileConsistency
    image=np.full((32,32,3),128,np.uint8);labels=np.full((32,32),4,np.uint8);scores=np.full_like(labels,250)
    tracker=CausalTactileConsistency();fixed,_,stats=tracker.apply(image,labels,scores,0.)
    assert not fixed.any() and stats['removed_tactile_pixels']==1024
    yellow=np.full_like(image,(20,180,210));labels[:]=3
    fixed,_,_=tracker.apply(yellow,labels,scores,.2);assert np.all(fixed==3)


def test_tactile_track_moves_and_leaves_field_of_view(monkeypatch):
    import sirius_navigation.theta_semantic_temporal as module
    rng=np.random.default_rng(30);image=np.full((32,64,3),(20,180,210),np.uint8)
    image=np.clip(image.astype(int)+rng.integers(-15,16,(32,64,1)),0,255).astype(np.uint8)
    labels=np.zeros((32,64),np.uint8);labels[5:25,40:54]=4;scores=np.full_like(labels,230)
    tracker=module.CausalTactileConsistency();tracker.apply(image,labels,scores,0)
    backward=np.zeros((32,64,2),np.float32);backward[...,0]=-4
    monkeypatch.setattr(module,'align_observation',lambda current,old:module.warp_with_flow(current,old,backward,-backward))
    blank=np.zeros_like(labels)
    for i in range(1,8):
        image=np.roll(image,4,axis=1);fixed,_,_=tracker.apply(image,blank,blank,i*.2)
        if i==1:assert fixed[12,55]==4 and fixed[12,40]==0
    assert not fixed.any()


def test_yellow_track_resets_on_timestamp_reversal_gap_or_shape_change():
    from sirius_navigation.theta_semantic_temporal import CausalTactileConsistency
    image=np.full((32,32,3),(20,180,210),np.uint8);positive=np.full((32,32),4,np.uint8);score=np.full_like(positive,230)
    for stamp,shape in [(0.,(32,32)),(4.,(32,32)),(1.2,(16,16))]:
        tracker=CausalTactileConsistency();tracker.apply(image,positive,score,1.)
        fixed,_,_=tracker.apply(cv2.resize(image,shape),np.zeros(shape,np.uint8),np.zeros(shape,np.uint8),stamp)
        assert not fixed.any()


def test_only_new_fills_drop_tiny_appearance_fragments():
    from sirius_navigation.theta_semantic_temporal import stabilize_tactile
    labels=np.full((8,8),6,np.uint8);labels[0,0]=4;scores=np.full((8,8),230,np.uint8)
    tactile=(np.full((8,8),4,np.uint8),np.full((8,8),.8,np.float32),np.ones((8,8),np.float32))
    appearance=np.zeros((8,8),bool);appearance[3:6,3:6]=True
    fixed,_,stats=stabilize_tactile(labels,scores,[(*tactile,-.26),(*tactile,.26)],appearance_mask=appearance)
    np.testing.assert_array_equal(fixed,labels);assert stats['filled_pixels']==0
