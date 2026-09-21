import math

import pytest

from simulator.navigation.geodetic import EARTH_RADIUS_M, ned_to_geodetic


def test_origin_cardinal_displacements_and_dateline_wrapping():
    assert ned_to_geodetic(0, 0) == pytest.approx((13, 80))
    one_degree = EARTH_RADIUS_M * math.pi / 180
    assert ned_to_geodetic(one_degree, one_degree, 0, 0) == pytest.approx((1, 1))
    assert ned_to_geodetic(-one_degree, -one_degree, 0, 0) == pytest.approx((-1, -1))
    assert ned_to_geodetic(0, one_degree * 0.5, 60, 179.5) == pytest.approx((60, -179.5))


@pytest.mark.parametrize("args", [(0, 0, 90, 0), (0, 0, -90, 0), (0, 0, 0, 181), (float("nan"), 0, 0, 0), (1e9, 0, 0, 0)])
def test_invalid_geographic_domain_is_rejected(args):
    with pytest.raises(ValueError):
        ned_to_geodetic(*args)
