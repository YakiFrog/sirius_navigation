"""Sequential SAM3 experiments with exact frame matching and settings restore.

The deployed server treats aspect ratios >1.8 as stereo SBS. Duplicating a wide
mono image as [image|image] transports the entire original as its left image;
SAM3 receives the intended unpadded pixels without restarting that server.
"""
import time
import hashlib
import cv2
import numpy as np
import requests


def mono_transport(frame):
    wide=frame.shape[1]/frame.shape[0]>1.8
    return (np.concatenate([frame,frame],axis=1) if wide else frame),wide


class Sam3ExperimentClient:
    def __init__(self,server='http://localhost:8080',timeout=40.):
        self.server=server.rstrip('/');self.timeout=timeout;self.before=None
        self.http=requests.Session()

    def _get_response(self,path,deadline=None):
        # GET is safe to retry. Never retry an upload: it increments the server's
        # frame version and could make an inference result refer to another input.
        deadline=time.monotonic()+90 if deadline is None else deadline
        for attempt in range(3):
            remaining=deadline-time.monotonic()
            if remaining<=0:raise TimeoutError(f'SAM3 response deadline expired: {path}')
            try:
                response=self.http.get(self.server+path,timeout=(min(5.,remaining),min(30.,remaining)))
                response.raise_for_status()
                return response
            except (requests.Timeout,requests.ConnectionError):
                remaining=deadline-time.monotonic()
                if attempt==2 or remaining<=0:raise
                time.sleep(min(.2*(attempt+1),remaining))

    def get(self,path,deadline=None):
        return self._get_response(path,deadline).json()

    def post(self,path,payload):
        r=self.http.post(self.server+path,json=payload,timeout=10);r.raise_for_status()

    def __enter__(self):
        self.before=self.get('/debug_state')
        self.registry_before=self.get('/class_registry')
        self.thresholds_before=self.get('/class_thresholds')
        self.post('/source_mode',{'mode':'network'})
        return self

    def configure(self,prompt,class_thresholds,threshold=1.,resolution=512):
        self.post('/prompt',{'prompt':prompt})
        self.post('/threshold',{'threshold':threshold})
        self.post('/class_thresholds',{'default':threshold,'classes':class_thresholds})
        current=self.get('/debug_state')['sam3_settings']['resolution']
        if current!=resolution:self.post('/sam3_resolution',{'resolution':resolution})

    def infer(self,frame):
        transport,duplicated=mono_transport(frame)
        before=self.get('/debug_state')['network_frame_version'];target=before+1
        ok,jpeg=cv2.imencode('.jpg',transport,[cv2.IMWRITE_JPEG_QUALITY,90])
        if not ok:raise RuntimeError('JPEG encoding failed')
        self.last_upload=jpeg.tobytes()
        started=time.monotonic()
        response=self.http.post(self.server+'/upload_frame',data=self.last_upload,timeout=10)
        response.raise_for_status()
        while time.monotonic()-started<self.timeout:
            state=self.get('/debug_state',deadline=started+self.timeout);net=state['network_frame_version']
            if net>target:raise RuntimeError('Another publisher changed the SAM3 input')
            if net==target and state['sam3_inference_frame_version']==target:break
            time.sleep(.03)
        else:raise TimeoutError(f'SAM3 did not finish frame {target}')
        maps=[]
        for path in ('/semantic_map.png','/semantic_score.png'):
            r=self._get_response(path)
            if int(r.headers.get('X-Frame-Version',-1))!=target:
                raise RuntimeError('SAM3 returned a stale or mismatched PNG')
            decoded=cv2.imdecode(np.frombuffer(r.content,np.uint8),cv2.IMREAD_GRAYSCALE)
            if decoded is None or decoded.shape!=frame.shape[:2]:
                raise RuntimeError(f'SAM3 image was cropped/resized unexpectedly: {path}')
            maps.append(decoded)
        return *maps,{'network_frame_version':target,'wall_seconds':time.monotonic()-started,
                     'duplicated_sbs_transport':duplicated,'input_shape':list(frame.shape),
                     'uploaded_jpeg_sha256':hashlib.sha256(self.last_upload).hexdigest(),
                     'class_scores':state['class_scores'],'settings':state['sam3_settings']}

    def __exit__(self,exc_type,exc,tb):
        settings=self.before['sam3_settings'];failures=[]
        for path,payload in [('/sam3_resolution',{'resolution':settings['resolution']}),
             ('/prompt',{'prompt':settings['prompt']}),
             ('/threshold',{'threshold':settings['confidence_threshold']}),
             ('/class_thresholds',self.thresholds_before),
             ('/source_mode',{'mode':self.before['mode']})]:
            try:self.post(path,payload)
            except Exception as error:failures.append(str(error))
        after=self.get('/debug_state')
        restored=after['mode']==self.before['mode'] and after['sam3_settings']==settings
        registry_same=self.get('/class_registry')==self.registry_before
        self.restoration={'settings_restored':restored,'registry_unchanged':registry_same,'errors':failures}
        self.http.close()
        if failures or not restored or not registry_same:
            raise RuntimeError('SAM3 settings restoration failed: '+str(self.restoration))
