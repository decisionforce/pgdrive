import numpy as np
from pgdrive.scene_creator.vehicle_module.distance_detector import DetectorMask


def _line_intersect(theta, center, point1, point2, maximum: float = 10000):
    """
    theta: the direction of the laser
    center: the center of the source of laser
    point1: one point of the line intersection
    point2: another poing of the line intersection
    maximum: the return value if no intersection
    """
    x0, y0 = center
    x1, y1 = point1
    x2, y2 = point2
    dx1 = np.cos(theta)
    dy1 = np.sin(theta)
    dx2 = x2 - x1
    dy2 = y2 - y1
    DET = -dx1 * dy2 + dy1 * dx2
    if abs(DET) < 1e-9:
        return maximum
    r = 1.0 / DET * (-dy2 * (x1 - x0) + dx2 * (y1 - y0))
    s = 1.0 / DET * (-dy1 * (x1 - x0) + dx1 * (y1 - y0))
    if 0 - 1e-5 < s < 1 + 1e-5 and r >= 0:
        return r
    return maximum


def _search_angle(point1, point2, num_lasers, start, heading, perceive_distance: float = 50):
    laser_heading = -np.arange(0, num_lasers) * 2 * np.pi / num_lasers + heading
    result = []
    for laser_index in range(num_lasers):
        ret = _line_intersect(theta=laser_heading[laser_index], center=start, point1=point1, point2=point2, maximum=1e8)
        assert 0 <= ret < 1e9
        if ret <= 1e8:
            result.append(ret / 1e8)
        else:
            result.append(1.0)
    return np.asarray(result)


def _test_mask(mask, stick_1_heading_deg, stick_2_heading_deg, max_span, stick1_x, stick2_x):
    stick_1_heading = np.deg2rad(stick_1_heading_deg)
    stick_2_heading = np.deg2rad(stick_2_heading_deg)
    stick_1_heading = stick_1_heading % (2 * np.pi)
    stick_2_heading = stick_2_heading % (2 * np.pi)

    stick1_pos = (stick1_x, 0)
    stick2_pos = (stick2_x, 0)
    mask.update_mask(
        position_dict={
            "stick1": stick1_pos,
            "stick2": stick2_pos
        },
        heading_dict={
            "stick1": stick_1_heading,
            "stick2": stick_2_heading
        }
    )
    mask_1 = mask.get_mask("stick1")
    mask_2 = mask.get_mask("stick2")

    left_of_stick2 = (stick2_pos[0], -max_span / 2)
    right_of_stick2 = (stick2_pos[0], max_span / 2)

    real_mask_1 = _search_angle(
        point1=left_of_stick2,
        point2=right_of_stick2,
        num_lasers=360,
        start=stick1_pos,
        heading=stick_1_heading,
        perceive_distance=100 * max_span
    ) < 1.0
    res = np.stack([real_mask_1, mask_1])
    assert all(mask_1[real_mask_1])  # mask 1 should at least include all True of real mask.
    assert sum(abs(mask_1.astype(int) - real_mask_1.astype(int))) <= 3

    left_of_stick1 = (stick1_pos[0], -max_span / 2)
    right_of_stick1 = (stick1_pos[0], max_span / 2)

    real_mask_2 = _search_angle(
        point1=left_of_stick1,
        point2=right_of_stick1,
        num_lasers=360,
        start=stick2_pos,
        heading=stick_2_heading,
        perceive_distance=100 * max_span
    ) < 1.0
    res2 = np.stack([real_mask_2, mask_2])
    assert all(mask_2[real_mask_2])  # mask 1 should at least include all True of real mask.
    assert sum(abs(mask_2.astype(int) - real_mask_2.astype(int))) <= 3


def test_detector_mask():
    # A infinite long (1e7) stick 2 in front of (0.01m) stick 1.

    pos_xy = [0, -1, 1, -100, 100]
    angles = [0, 0.01, 30, 89, 90, 91, 130, 180, 181, 270, 360, 361, 400]
    angles += [-a for a in angles]

    mask = DetectorMask(num_lasers=360, max_span=1e7)
    _test_mask(mask, -270, 30, 1e7, 0, -1)

    mask = DetectorMask(num_lasers=360, max_span=1)
    _test_mask(mask, -180, -300, 1, 0, -1)
    _test_mask(mask, -361, 270, 1, 0, -100)

    for max_span in [1e7, 1, 0.1]:
        mask = DetectorMask(num_lasers=360, max_span=max_span)
        for pos1_x in pos_xy:
            for pos2_x in pos_xy:
                angles1 = np.random.choice(angles, 10)
                angles2 = np.random.choice(angles, 10)
                for stick_1_heading_deg in angles1:
                    for stick_2_heading_deg in angles2:
                        _test_mask(mask, stick_1_heading_deg, stick_2_heading_deg, max_span, pos1_x, pos2_x)
                print("Finish. ", max_span, pos1_x, pos2_x)


if __name__ == '__main__':
    test_detector_mask()
