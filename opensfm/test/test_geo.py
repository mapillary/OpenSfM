import numpy as np
from opensfm import geo, pygeo


def test_ecef_lla_consistency():
    lla_before = [46.5274109, 6.5722075, 402.16]
    ecef = geo.ecef_from_lla(lla_before[0], lla_before[1], lla_before[2])
    lla_after = geo.lla_from_ecef(ecef[0], ecef[1], ecef[2])
    assert np.allclose(lla_after, lla_before)


def test_ecef_lla_topocentric_consistency():
    lla_ref = [46.5, 6.5, 400]
    lla_before = [46.5274109, 6.5722075, 402.16]
    enu = geo.topocentric_from_lla(
        lla_before[0], lla_before[1], lla_before[2], lla_ref[0], lla_ref[1], lla_ref[2]
    )
    lla_after = geo.lla_from_topocentric(
        enu[0], enu[1], enu[2], lla_ref[0], lla_ref[1], lla_ref[2]
    )
    assert np.allclose(lla_after, lla_before)


def test_ecef_lla_consistency_pygeo():
    lla_before = [46.5274109, 6.5722075, 402.16]
    ecef = pygeo.ecef_from_lla(lla_before[0], lla_before[1], lla_before[2])
    lla_after = pygeo.lla_from_ecef(ecef[0], ecef[1], ecef[2])
    assert np.allclose(lla_after, lla_before)


def test_ecef_lla_topocentric_consistency_pygeo():
    lla_ref = [46.5, 6.5, 400]
    lla_before = [46.5274109, 6.5722075, 402.16]
    enu = pygeo.topocentric_from_lla(
        lla_before[0], lla_before[1], lla_before[2], lla_ref[0], lla_ref[1], lla_ref[2]
    )
    lla_after = pygeo.lla_from_topocentric(
        enu[0], enu[1], enu[2], lla_ref[0], lla_ref[1], lla_ref[2]
    )
    assert np.allclose(lla_after, lla_before)

def test_eq_geo():
    assert geo.TopocentricConverter(40,30,0) == geo.TopocentricConverter(40,30,0)
    assert geo.TopocentricConverter(40,32,0) != geo.TopocentricConverter(40,30,0)
