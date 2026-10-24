import math
import pytest

from spacedrone_mission.search_pattern import lawnmower, lane_spacing_for_coverage, path_length


def test_lawnmower_covers_box_edges():
    wps = lawnmower(0, 20, -9, 9, 6)
    easts = sorted(set(round(e, 6) for _, e in wps))
    # should have lanes on both edges of the box
    assert easts[0] == pytest.approx(-9)
    assert easts[-1] == pytest.approx(9)
    # 18m wide / 6m spacing = 4 lanes
    assert len(easts) == 4


def test_lawnmower_alternates_direction():
    wps = lawnmower(0, 10, 0, 10, 5)
    # lane 1 goes north, lane 2 goes south, ...
    assert wps[0][0] == 0 and wps[1][0] == 10
    assert wps[2][0] == 10 and wps[3][0] == 0


def test_lawnmower_uneven_width():
    # 10 / 4 isnt a whole number, last lane should still be on the edge
    wps = lawnmower(0, 5, 0, 10, 4)
    assert max(e for _, e in wps) == pytest.approx(10)
    # and spacing shouldnt be bigger than what we asked for
    easts = sorted(set(e for _, e in wps))
    for a, b in zip(easts, easts[1:]):
        assert b - a <= 4 + 1e-9


def test_bad_inputs():
    with pytest.raises(ValueError):
        lawnmower(0, 10, 0, 10, 0)
    with pytest.raises(ValueError):
        lawnmower(10, 0, 0, 10, 2)


def test_coverage_spacing():
    # 90 deg fov at 5m -> 10m wide footprint, 30% overlap -> 7m
    assert lane_spacing_for_coverage(5.0, math.pi / 2, 0.3) == pytest.approx(7.0)


def test_path_length():
    assert path_length([(3, 4)]) == pytest.approx(5)
    assert path_length([(0, 10), (10, 10)]) == pytest.approx(20)
