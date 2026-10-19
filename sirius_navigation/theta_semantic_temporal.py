"""Conservative road/sidewalk correction from flow-aligned raw SAM3 observations.

Inspired by occlusion-aware temporal segmentation; not a BOFP or SAM3-video
reproduction. Only IDs5/6 may change. No recursive pseudo-label feedback.
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
