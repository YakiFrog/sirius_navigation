import numpy as np
from sirius_navigation.theta_road_visibility import visible_ground


def test_obstacle_rejects_projected_ground_behind_it_but_keeps_foreground():
    x = np.array([1., 1.95, 2., 4.])
    keep = visible_ground(x, np.zeros_like(x), 0, .1, [2.], 0.1, 10.)
    assert keep.tolist() == [True, False, False, False]


def test_unknown_beam_does_not_certify_ground_and_clear_beam_has_range_limit():
    angle = np.array([0., .1, .2, .3])
    keep = visible_ground(np.cos(angle), np.sin(angle), 0, .1,
                          [float('nan'), 0., float('inf'), 20.], .1, 10.)
    assert keep.tolist() == [False, False, True, True]
    assert not visible_ground(np.array([11.]), np.array([0.]), 0, .1,
                              [float('inf')], .1, 10.)[0]


def test_partial_fov_and_negative_increment():
    angle = np.array([-.5, 0., .5, 1.5])
    for start, inc in [(-.5, .5), (.5, -.5)]:
        keep = visible_ground(np.cos(angle), np.sin(angle), start, inc,
                              [3., 3., 3.], .1, 10.)
        assert keep.tolist() == [True, True, True, False]


def test_scan_pi_endpoint_rounding_is_not_an_unobserved_wedge():
    keep = visible_ground(np.array([-1.]), np.array([0.]),
                          -3.141592741, .007, np.full(898, 3.), .1, 10.)
    assert keep[0]


def test_old_scan_cannot_be_used_as_current_visibility():
    from types import SimpleNamespace
    from sirius_navigation.theta_ground_cloud_node import ThetaGroundCloudNode
    node = object.__new__(ThetaGroundCloudNode)
    node.strict_scans = {1000000000: object()}
    node.lidar_max_age = .15
    keep = node._strict_visibility(np.array([1.]), np.array([0.]),
                                  SimpleNamespace(sec=2, nanosec=0), 'base')
    assert not keep.any()


def test_mounting_yaw_and_two_observation_times_are_used():
    from types import SimpleNamespace as S
    from builtin_interfaces.msg import Time
    from sirius_navigation.theta_ground_cloud_node import ThetaGroundCloudNode
    node = object.__new__(ThetaGroundCloudNode)
    scan = S(header=S(stamp=Time(sec=1), frame_id='laser'), angle_min=np.pi/2,
             angle_increment=.1, ranges=[2.], range_min=.1, range_max=10.)
    node.strict_scans = {1000000000: scan}
    node.lidar_max_age = .15
    node.lidar_margin = .1
    node.lidar_padding = 0.0
    node.get_parameter = lambda key: S(value='odom')
    calls = []
    def transform(*args):
        calls.append(args)
        return S(transform=S(rotation=S(x=0., y=0., z=2**-.5, w=2**-.5),
                             translation=S(x=0., y=0.)))
    node.visibility_tf = S(lookup_transform_full=transform)
    keep = node._strict_visibility(np.array([1., 3.]), np.zeros(2),
                                  Time(sec=1, nanosec=50000000), 'base')
    assert keep.tolist() == [True, False]
    assert calls[0][0] == 'laser' and calls[0][2] == 'base'
    assert calls[0][1].nanoseconds == 1000000000
    assert calls[0][3].nanoseconds == 1050000000


def test_padding_rejects_silhouette_edges_and_keeps_foreground_and_distant_side():
    angles = np.array([-.2, -.1, 0., .1, .2])
    scan = [np.inf, np.inf, 2., np.inf, np.inf]
    old = visible_ground(3*np.cos(angles), 3*np.sin(angles), -.2, .1, scan, .1, 10.)
    wide = visible_ground(3*np.cos(angles), 3*np.sin(angles), -.2, .1, scan, .1, 10.,
                          obstacle_padding=.25)
    assert old.tolist() == [True, True, False, True, True]
    assert wide.tolist() == [True, False, False, False, True]
    assert visible_ground(np.cos(angles), np.sin(angles), -.2, .1, scan, .1, 10.,
                          obstacle_padding=.25).all()


def test_padding_covers_full_scan_seam_and_ignores_invalid_returns():
    count = 360
    inc = 2*np.pi/count
    scan = np.full(count, np.inf)
    scan[0] = 2.
    angle = np.pi - inc
    assert not visible_ground(np.array([3*np.cos(angle)]), np.array([3*np.sin(angle)]),
                              -np.pi, inc, scan, .1, 10., obstacle_padding=.25)[0]
    assert visible_ground(np.array([1.]), np.array([0.]), -.1, .1,
                          [np.nan, np.inf, 0.], .1, 10., obstacle_padding=.25)[0]


def test_physical_padding_does_not_grow_with_distance_or_scan_resolution():
    for inc in [.01, .005]:
        for distance in [2., 4.]:
            # Lateral offsets are measured at the obstacle, independent of range.
            angles = np.arange(-100, 101)*inc
            scan = np.full(angles.size, np.inf)
            scan[100] = distance
            query = np.array([np.arctan2(.15, distance), np.arctan2(.4, distance)])
            keep = visible_ground(6*np.cos(query), 6*np.sin(query), angles[0], inc,
                                  scan, .1, 10., obstacle_padding=.25)
            assert keep.tolist() == [False, True]
