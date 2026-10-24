'''
Search pattern generation. Kept separate from the mission node so it can be unit tested
without a drone (see test/test_search_pattern.py)
'''

import math


def lawnmower(north_min, north_max, east_min, east_max, lane_spacing):
    '''
    Classic lawnmower / boustrophedon pattern. Lanes run north-south and we step east
    between them. Returns a list of (north, east) waypoints, every pair is one lane.

        ^ N    _   _
        |     | | | |
        |     | | | |
        |     | |_| |
        +---------------> E
    '''
    if lane_spacing <= 0:
        raise ValueError("lane_spacing has to be > 0")
    if north_max <= north_min or east_max <= east_min:
        raise ValueError("search box is backwards or zero size")

    width = east_max - east_min
    # num_lanes = int(math.floor(width / lane_spacing)) + 1   # floor made gaps BIGGER than lane_spacing
    num_lanes = int(math.ceil(width / lane_spacing - 1e-9)) + 1
    # spread the lanes evenly so the last one lands exactly on east_max
    # (otherwise the last strip of the box was getting skipped)
    if num_lanes > 1:
        step = width / (num_lanes - 1)
    else:
        step = 0.0

    waypoints = []
    for i in range(num_lanes):
        e = east_min + i * step
        if i % 2 == 0:
            waypoints.append((north_min, e))
            waypoints.append((north_max, e))
        else:
            waypoints.append((north_max, e))
            waypoints.append((north_min, e))
    return waypoints


def lane_spacing_for_coverage(altitude, hfov, overlap=0.3):
    '''
    How far apart lanes can be so the camera footprints overlap a bit.
    footprint width on the ground = 2 * alt * tan(hfov / 2)
    '''
    footprint = 2.0 * altitude * math.tan(hfov / 2.0)
    return footprint * (1.0 - overlap)


def path_length(waypoints, start=(0.0, 0.0)):
    total = 0.0
    prev = start
    for wp in waypoints:
        total += math.hypot(wp[0] - prev[0], wp[1] - prev[1])
        prev = wp
    return total
