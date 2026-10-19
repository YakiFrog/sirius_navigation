"""Road consistency and visible yellow-tactile tracking from aligned imagery.

Inspired by occlusion-aware temporal segmentation; not a BOFP or SAM3-video
reproduction. Road votes use raw observations; tactile has explicit visual state.
"""
from collections import deque
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class Observation:
    image: np.ndarray
    labels: np.ndarray
    scores: np.ndarray
    stamp: float


def warp_with_flow(current, previous, backward, forward, fb_limit=1.5,
                   photo_limit=30.):
    """Sample previous at current->previous flow; reject non-invertible matches."""
    h,w=current.shape[:2]
    yy,xx=np.mgrid[:h,:w].astype(np.float32)
    u=xx+backward[...,0];v=yy+backward[...,1]
    f=cv2.remap(forward,u,v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
    error=np.linalg.norm(backward+f,axis=2)
    old=cv2.remap(previous.image,u,v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
    gray=cv2.cvtColor(current,cv2.COLOR_BGR2GRAY).astype(np.float32)
    oldgray=cv2.cvtColor(old,cv2.COLOR_BGR2GRAY).astype(np.float32)
    photo=np.abs(gray-oldgray)
    # A local texture check prevents mutually-zero flow on flat, ambiguous regions.
    mean=cv2.blur(gray,(7,7));variance=np.maximum(cv2.blur(gray*gray,(7,7))-mean*mean,0)
    valid=(u>=1)&(u<w-2)&(v>=1)&(v<h-2)&np.isfinite(error)
    valid&=(error<=fb_limit+.03*np.linalg.norm(backward,axis=2))&(photo<=photo_limit)&(variance>=9)
    labels=cv2.remap(previous.labels,u,v,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT)
    scores=cv2.remap(previous.scores,u,v,cv2.INTER_NEAREST,borderMode=cv2.BORDER_CONSTANT).astype(np.float32)/255.
    reliability=np.exp(-.5*(error/fb_limit)**2-.5*(photo/photo_limit)**2)*valid
    return labels,scores,reliability.astype(np.float32)


def align_observation(current,previous):
    if current.shape!=previous.image.shape:raise ValueError('Image shapes differ')
    a=cv2.cvtColor(current,cv2.COLOR_BGR2GRAY);b=cv2.cvtColor(previous.image,cv2.COLOR_BGR2GRAY)
    engine=cv2.DISOpticalFlow_create(cv2.DISOPTICAL_FLOW_PRESET_MEDIUM)
    backward=engine.calc(a,b,None);forward=engine.calc(b,a,None)
    return warp_with_flow(current,previous,backward,forward)


def fuse_road_labels(labels,scores,aligned,score_min=.5,min_support=3,agreement=.75,confidence_gain=.05):
    """Keep current mask unless >=3 agreeing matched observations support a switch.

    aligned entries are (warped IDs, mask confidence [0,1], flow reliability [0,1]).
    agreement is a weighted vote fraction, not calibrated semantic probability.
    """
    labels=np.asarray(labels);scores=np.asarray(scores)
    if labels.shape!=scores.shape:raise ValueError('Label and score shapes differ')
    result=labels.copy();result_scores=scores.copy()
    eligible=np.isin(labels,(5,6))&(scores.astype(np.float32)/255.>=score_min)
    votes=np.zeros((*labels.shape,2),np.float32);counts=np.zeros((*labels.shape,2),np.uint8)
    confidence=np.zeros_like(votes);weight=np.zeros_like(votes)
    base=scores.astype(np.float32)/255.
    for k,cid in enumerate((5,6)):votes[...,k]=1.2*base*(labels==cid)*eligible
    for ids,conf,reliable in aligned:
        valid=(conf>=score_min)&(reliable>=.25)
        for k,cid in enumerate((5,6)):
            take=(ids==cid)&valid
            ww=reliable*take
            votes[...,k]+=conf*ww;counts[...,k]+=take.astype(np.uint8)
            confidence[...,k]+=conf*ww;weight[...,k]+=ww
    winner=np.argmax(votes,axis=2)
    best=np.take_along_axis(votes,winner[...,None],axis=2)[...,0]
    support=np.take_along_axis(counts,winner[...,None],axis=2)[...,0]
    share=best/np.maximum(votes.sum(axis=2),1e-8)
    chosen=winner+5
    mean=confidence/np.maximum(weight,1e-8)
    selected=np.take_along_axis(mean,winner[...,None],axis=2)[...,0]
    change=eligible&(chosen!=labels)&(support>=min_support)&(share>=agreement)&(selected>=base+confidence_gain)
    result[change]=chosen[change]
    result_scores[change]=np.clip(np.rint(selected[change]*255),0,255).astype(np.uint8)
    return result,result_scores,{'changed_pixels':int(change.sum()),'eligible_pixels':int(eligible.sum()),
                               'matched_observations':len(aligned)}


class CausalRoadConsistency:
    def __init__(self,history_size=4,max_gap=1.5,score_min=.5):
        self.history=deque(maxlen=history_size);self.max_gap=max_gap;self.score_min=score_min

    def clear(self):self.history.clear()

    def apply(self,image,labels,scores,stamp):
        if self.history and (stamp<=self.history[-1].stamp or stamp-self.history[-1].stamp>self.max_gap
                             or image.shape!=self.history[-1].image.shape):self.clear()
        aligned=[]
        for old in self.history:
            if stamp-old.stamp<=self.max_gap:
                ids,conf,reliability=align_observation(image,old)
                aligned.append((ids,conf,reliability*np.exp(-abs(stamp-old.stamp)/2.)))
        result=fuse_road_labels(labels,scores,aligned,score_min=self.score_min)
        # Preserve independent raw observations, never feed a corrected result back.
        self.history.append(Observation(image.copy(),labels.copy(),scores.copy(),stamp))
        return result


def stabilize_tactile(labels,scores,aligned,*,bidirectional=True,score_min=.5,
                       max_age=.85,min_support=2,appearance_mask=None,min_fill_area=16):
    """Fill short tactile-mask gaps from repeated, aligned ORIGINAL detections.

    aligned entries: (warped labels, mask score [0,1], reliability [0,1],
    neighbor_time - current_time). Offline requires observations on both sides.
    Existing visible tactile pixels are never removed. IDs1/2/3 are protected.
    """
    result=labels.copy();result_scores=scores.copy();shape=labels.shape
    count=np.zeros(shape,np.uint8);past=np.zeros(shape,bool);future=past.copy()
    weight=np.zeros(shape,np.float32);confidence=np.zeros(shape,np.float32)
    for ids,conf,reliable,dt in aligned:
        if dt==0 or abs(dt)>max_age or (not bidirectional and dt>0):continue
        take=(ids==4)&(conf>=score_min)&(reliable>=.4)
        # Intersect interiors of observations rather than dilating narrow masks.
        take=cv2.erode(take.astype(np.uint8),np.ones((3,3),np.uint8)).astype(bool)
        ww=take*reliable*np.exp(-abs(dt))
        count+=take.astype(np.uint8);weight+=ww;confidence+=ww*conf
        if dt<0:past|=take
        else:future|=take
    mean=confidence/np.maximum(weight,1e-8)
    visible=(labels==4)&(scores.astype(np.float32)/255.>=score_min)
    eligible=np.isin(labels,(0,4,5,6))&~visible
    accepted=eligible&(count>=min_support)&(mean>=.55)&past
    if bidirectional:accepted&=future
    if appearance_mask is not None:
        if appearance_mask.shape!=shape:raise ValueError('Appearance-mask shape differs')
        accepted&=appearance_mask.astype(bool)
    if min_fill_area>1:
        n,components,stats,_=cv2.connectedComponentsWithStats(accepted.astype(np.uint8),8)
        large=np.zeros(n,bool);large[1:]=stats[1:,cv2.CC_STAT_AREA]>=min_fill_area
        accepted=large[components]
    result[accepted]=4
    result_scores[accepted]=np.clip(np.rint(mean[accepted]*255),0,255).astype(np.uint8)
    return result,result_scores,{'filled_pixels':int(accepted.sum()),
        'original_tactile_pixels':int(visible.sum()),'removed_tactile_pixels':0}


def yellow_tactile_appearance(image,min_saturation=100,hue_max=32,close_holes=False):
    """Optional gap-fill gate for yellow tactile surfaces; not a detector.

    Other colors keep raw SAM3 labels. The thresholds are a configurable study
    assumption, not a claim that tactile paving is always yellow.
    """
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    keep=(hsv[...,0]>=15)&(hsv[...,0]<=hue_max)&(hsv[...,1]>=min_saturation)&(hsv[...,2]>=35)
    if close_holes:
        keep=cv2.morphologyEx(keep.astype(np.uint8),cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
    return keep.astype(bool)


def yellow_tactile_seeds(image,labels,scores,score_min=.5,min_area=16):
    """Require SAM3's yellow-tactile class plus component-level yellow evidence.

    Preserve dark ridges within a yellow block instead of clipping each yellow
    pixel. Gray components are unknown, never automatically relabeled sidewalk.
    """
    candidate=(labels==4)&(scores.astype(np.float32)/255.>=score_min)
    yellow=yellow_tactile_appearance(image,45,40)
    n,components,stats,_=cv2.connectedComponentsWithStats(candidate.astype(np.uint8),8)
    area=stats[:,cv2.CC_STAT_AREA];hits=np.bincount(components[yellow],minlength=n)
    valid=(area>=min_area)&(hits/np.maximum(area,1)>=.35);valid[0]=False
    return valid[components]


def merge_tactile_tracks(raw,forward,forward_scores,reverse,reverse_scores):
    """Union independent tactile tracks using scores belonging to class4 only."""
    result=forward.copy();scores=forward_scores.copy()
    take=(reverse==4)&~np.isin(raw,(1,2,3))
    result[take]=4
    scores[take]=np.maximum(np.where(forward[take]==4,forward_scores[take],0),reverse_scores[take])
    return result,scores


class CausalTactileConsistency(CausalRoadConsistency):
    """Track visible yellow tactile surfaces across detector misses.

    Each propagation requires fresh bidirectional-flow, photometric and yellow
    appearance evidence. There is no detector-miss timeout. A track is never
    counted as a fresh detector observation: it retains its detector score.
    Unmatched, occluded and out-of-view pixels are discarded every frame.
    """
    def __init__(self,history_size=4,max_gap=1.5,score_min=.5):
        super().__init__(history_size,max_gap,score_min)
        self.track=None

    def clear(self):
        super().clear();self.track=None

    def apply(self,image,labels,scores,stamp):
        if self.history and (stamp<=self.history[-1].stamp or stamp-self.history[-1].stamp>self.max_gap
                             or image.shape!=self.history[-1].image.shape):self.clear()
        result=labels.copy();result_scores=scores.copy()
        seed=yellow_tactile_seeds(image,labels,scores,self.score_min)
        rejected=(labels==4)&~seed
        result[rejected]=0;result_scores[rejected]=0
        tracked=np.zeros(labels.shape,bool);conf=np.zeros(labels.shape,np.float32)
        if self.track is not None and np.any(self.track.labels==4):
            ids,conf,reliability=align_observation(image,self.track)
            tracked=(ids==4)&(reliability>=.4)&np.isin(labels,(0,4,5,6))
            # Component appearance prevents tracks migrating to gray tile. Local
            # mismatches remain holes; no dilation bridges an occluder or border.
            tracked=yellow_tactile_seeds(image,np.where(tracked,4,0).astype(np.uint8),
                np.rint(conf*255).astype(np.uint8),self.score_min)
        fill=tracked&~seed
        result[fill]=4;result_scores[fill]=np.rint(conf[fill]*255).astype(np.uint8)
        state=np.where(seed|tracked,4,0).astype(np.uint8)
        self.track=Observation(image.copy(),state,np.where(state==4,result_scores,0).astype(np.uint8),stamp)
        self.history.append(Observation(image.copy(),labels.copy(),scores.copy(),stamp))
        return result,result_scores,{'filled_pixels':int(fill.sum()),'original_tactile_pixels':int(seed.sum()),
                                    'removed_tactile_pixels':int(rejected.sum()),'tracked_pixels':int(tracked.sum())}
