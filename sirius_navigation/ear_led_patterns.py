"""Temporary ear-only test patterns; existing position/stop control stays authoritative."""
import colorsys
import math

PATTERNS = {'normal','left','right','turning','emergency','solid','blink','alternate','breathe','rainbow','off'}


def make_test(data, now):
    if not isinstance(data,dict) or data.get('pattern') not in PATTERNS|{'auto'}:
        raise ValueError('耳LEDの表示を一覧から選んでください')
    if data['pattern']=='auto':return None
    duration=data.get('duration',10)
    if type(duration) not in (int,float) or not math.isfinite(duration) or not 1<=duration<=30:
        raise ValueError('テスト時間は1〜30秒です')
    color=data.get('color',[0,160,255])
    if not isinstance(color,list) or len(color)!=3 or any(type(v) is not int or not 0<=v<=255 for v in color):
        raise ValueError('RGBは0〜255で指定してください')
    return dict(pattern=data['pattern'],color=color,start=now,until=now+duration)


def output(test, now):
    if not test or now>=test['until']:return None
    elapsed=max(0,now-test['start']);pattern=test['pattern'];color=test['color']
    black='C:0,0,0'
    def rgb(values):return 'C:'+','.join(str(max(0,min(255,round(v)))) for v in values)
    on=int(elapsed/.5)%2==0
    if pattern=='normal':left=right='M:1'
    elif pattern=='off':left=right=black
    elif pattern=='emergency':left=right='C:255,0,0'
    elif pattern=='left':left,right=('C:0,255,0' if on else black),black
    elif pattern=='right':left,right=black,('C:0,255,0' if on else black)
    elif pattern=='turning':left=right='C:255,255,0' if on else black
    elif pattern=='solid':left=right=rgb(color)
    elif pattern=='blink':left=right=rgb(color) if on else black
    elif pattern=='alternate':left,right=(rgb(color),black) if on else (black,rgb(color))
    elif pattern=='breathe':left=right=rgb([v*(.5-.5*math.cos(elapsed*2*math.pi/3)) for v in color])
    else:
        left=rgb([v*255 for v in colorsys.hsv_to_rgb((elapsed/6)%1,1,1)])
        right=rgb([v*255 for v in colorsys.hsv_to_rgb((elapsed/6+.5)%1,1,1)])
    return left,right,'test_'+pattern
