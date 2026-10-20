"""Point conversion must retain the exact raw sensor channels and layout."""
import copy
import struct
import numpy as np
import pytest
from geometry_msgs.msg import Transform, Vector3, Quaternion
from sensor_msgs.msg import PointCloud2, PointField
from sirius_navigation.leveled_lidar_cloud_node import transform_cloud


@pytest.mark.parametrize('bigendian', [False, True])
def test_organized_cloud_keeps_times_rings_padding_and_original_message(bigendian):
    fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
              for name, offset, datatype in [('x',0,7), ('y',4,7), ('z',8,7), ('intensity',12,7),
                                              ('ring',16,4), ('time',20,7)]]
    # Two rows with deliberate nonzero per-point and per-row padding.
    payload = bytearray([0x65]*112)
    prefix = '>' if bigendian else '<'
    for row in range(2):
        for col in range(2):
            offset = row*56+col*24
            struct.pack_into(prefix+'ffffH', payload, offset, 1., 2., 3., 42., row*2+col)
            struct.pack_into(prefix+'f', payload, offset+20, -.04+col*.01)
    msg = PointCloud2(height=2, width=2, fields=fields, is_bigendian=bigendian,
                      point_step=24, row_step=56, data=bytes(payload), is_dense=True)
    msg.header.frame_id = 'raw'; msg.header.stamp.sec = 123
    before = copy.deepcopy(msg)
    transform = Transform(translation=Vector3(x=.5, y=-.1, z=.2),
                          rotation=Quaternion(z=np.sin(np.pi/4), w=np.cos(np.pi/4)))
    result = transform_cloud(msg, transform, 'level')
    assert msg == before
    assert result.header.frame_id == 'level' and result.header.stamp == msg.header.stamp
    assert result.fields == msg.fields and result.row_step == 56 and result.height == 2
    for row in range(2):
        for col in range(2):
            offset = row*56+col*24
            np.testing.assert_allclose(struct.unpack_from(prefix+'fff', result.data, offset), [-1.5,.9,3.2], atol=1e-6)
            assert result.data[offset+12:offset+24] == msg.data[offset+12:offset+24]
        assert result.data[row*56+48:row*56+56] == msg.data[row*56+48:row*56+56]
