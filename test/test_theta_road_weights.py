import numpy as np
from sirius_navigation.theta_road_weights import directional_weights, reliable_observations


def test_weights_use_robot_direction_and_preserve_front_quality():
    x, y = np.array([2., -2., 0., 0.]), np.array([0., 0., 2., -2.])
    np.testing.assert_allclose(directional_weights(x, y, .2), [1., .2, .6, .6])
    np.testing.assert_array_equal(directional_weights(x, y), np.ones(4))
    np.testing.assert_allclose(directional_weights(10*x, 10*y, .2),
                               directional_weights(x, y, .2))


def test_side_transition_is_continuous_and_not_a_hard_seam():
    w = directional_weights(np.array([-.001, 0., .001]), np.ones(3), .2)
    assert np.all(np.diff(w) > 0) and np.ptp(w) < .01


def test_repeated_rear_views_cannot_overwrite_better_observation():
    best = np.zeros((1, 2), np.float32)
    gx, gy = np.array([0]), np.array([0])
    assert reliable_observations(gx, gy, np.array([1.]), best, .5)[0]
    for _ in range(200):
        assert not reliable_observations(gx, gy, np.array([.2]), best, .5)[0]
    # A cell only visible from behind remains observable; a better later view is accepted.
    assert reliable_observations(np.array([1]), gy, np.array([.2]), best, .5)[0]
    assert reliable_observations(gx, gy, np.array([.8]), best, .5)[0]


def test_unknown_weights_do_not_poison_quality_memory():
    best = np.zeros((1, 3), np.float32)
    keep = reliable_observations(np.arange(3), np.zeros(3, int),
                                np.array([np.nan, 0., .1]), best, .5)
    assert keep.tolist() == [False, False, True]
    np.testing.assert_allclose(best, [[0, 0, .1]])


def test_production_fusion_keeps_good_color_through_repeats_and_canvas_growth():
    import rclpy
    from sensor_msgs.msg import PointCloud2, PointField
    from sirius_navigation.theta_indexed_map_node import ThetaIndexedMapNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = ThetaIndexedMapNode()
    try:
        node.min_relative_weight = .5
        node._alloc_canvas(0, 0, 3, 3, .05)
        node._transform_to_map = lambda x, y, z, header: np.column_stack([x, y, z])
        fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                  for i, name in enumerate(('x','y','z','rgb','weight'))]
        def cloud(color, weight):
            values = np.array([[.025,.025,0.,0.,weight]], np.float32)
            values[0,3] = np.array([color],np.uint32).view(np.float32)[0]
            return PointCloud2(height=1,width=1,fields=fields,point_step=20,
                               row_step=20,data=values.tobytes(),is_dense=True)
        node._cloud_callback(cloud(0x204060,1.))
        node._expand_canvas(-.05,-.05,5,5)
        for _ in range(60):
            node._cloud_callback(cloud(0xff0000,.2))
        observed = node.texture_weight > 0
        result = node.texture_sum[observed]/node.texture_weight[observed,None]
        np.testing.assert_allclose(result, [[32.,64.,96.]])
        assert node.best_observation_weight[observed][0] == 1.
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()


def test_production_ground_cloud_changes_weights_without_changing_points():
    import rclpy
    from builtin_interfaces.msg import Time
    from types import SimpleNamespace
    import sensor_msgs_py.point_cloud2 as pc2
    from sirius_navigation.theta_ground_cloud_node import ThetaGroundCloudNode
    started = not rclpy.ok()
    if started:
        rclpy.init()
    node = ThetaGroundCloudNode()
    try:
        node.stride = 16
        sent = []
        node.pub = SimpleNamespace(publish=sent.append)
        node.semantic_pub = None
        image = np.full((node.bev_size,node.bev_size,3),100,np.uint8)
        node.rear_weight = 1.
        node._emit(Time(sec=1),'base',image,None,publish_semantic=False)
        node.rear_weight = .5
        node._emit(Time(sec=1),'base',image,None,publish_semantic=False)
        before,after = [pc2.read_points(m,field_names=('x','y','weight')) for m in sent]
        np.testing.assert_array_equal(before['x'],after['x'])
        np.testing.assert_array_equal(before['y'],after['y'])
        expected = before['weight']*directional_weights(before['x'],before['y'],.5)
        np.testing.assert_allclose(after['weight'],expected,rtol=1e-6)
    finally:
        node.destroy_node()
        if started:
            rclpy.shutdown()
