import asyncio
from types import SimpleNamespace
import sys
import threading
import time
from unittest.mock import Mock

import pytest

from sirius_navigation.ear_led_patterns import make_test
from sirius_navigation.sirius_ble_gateway import SiriusBleGateway


@pytest.mark.parametrize('real_stop,expired,expected',[
    (True,False,('C:255,0,0','C:255,0,0')),
    (False,True,('C:0,255,0','C:0,0,0')),
    (False,False,('C:0,160,255','C:0,160,255')),
])
def test_real_stop_priority_and_automatic_restoration(monkeypatch,real_stop,expired,expected):
    monkeypatch.setitem(sys.modules,'bleak',SimpleNamespace(BleakClient=object))
    writes=[];done=threading.Event()
    async def nothing(*args):pass
    async def write(left,right):writes.append((left,right));done.set()
    test=make_test(dict(pattern='solid',duration=5),time.monotonic()-(10 if expired else 0))
    gateway=SimpleNamespace(_stopping=done,_ear_led_test=test,_ear_led_stop=real_stop,
        _ear_led_blinking=False,_ear_led_blink_on=True,ear_led_period=.1,
        _ear_led_left_command='C:0,255,0',_ear_led_right_command='C:0,0,0',
        _ensure_ear_led_connected=nothing,_ear_leds_connected=lambda:True,
        _write_ear_leds=write,_disconnect_ear_leds=nothing,
        _publish_ear_led_status=Mock(),get_logger=Mock(return_value=Mock()))
    asyncio.run(SiriusBleGateway._run_ear_led_client(gateway))
    assert writes==[expected]
