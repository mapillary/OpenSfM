import datetime
from pathlib import Path
from unittest import mock

import numpy as np
import pytest
from opensfm import exif, geo, geotag_from_gpx
from PIL import Image
from PIL.ExifTags import Base, IFD


T0 = datetime.datetime(2020, 1, 1, 12, 0, 0)

GPX_CONTENT = """<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="test" xmlns="http://www.topografix.com/GPX/1/1">
  <trk><trkseg>
    <trkpt lat="1.0" lon="2.0"><ele>10.0</ele><time>2020-01-01T12:00:10Z</time></trkpt>
    <trkpt lat="0.0" lon="0.0"><ele>5.0</ele><time>2020-01-01T12:00:00Z</time></trkpt>
    <trkpt lat="2.0" lon="4.0"><time>2020-01-01T12:00:20Z</time></trkpt>
  </trkseg></trk>
</gpx>
"""


def _point(
    seconds: float, lat: float, lon: float, ele: float | None = None
) -> tuple[datetime.datetime, float, float, float | None]:
    return (T0 + datetime.timedelta(seconds=seconds), lat, lon, ele)


def _write_jpeg(path: Path, capture_time: datetime.datetime | None = None) -> None:
    image = Image.fromarray(np.full((32, 48, 3), 128, dtype=np.uint8))
    image_exif = Image.Exif()
    image_exif[Base.ImageDescription] = "description"
    if capture_time is not None:
        image_exif[IFD.Exif] = {
            Base.DateTimeOriginal: capture_time.strftime("%Y:%m:%d %H:%M:%S")
        }
    image.save(path, exif=image_exif)


def _read_geo(path: Path) -> dict[str, float]:
    with open(path, "rb") as f:
        return exif.EXIF(f, lambda: (32, 48)).extract_geo()


def test_add_gps_to_exif(tmp_path: Path) -> None:
    source = tmp_path / "source.jpg"
    target = tmp_path / "target.jpg"
    _write_jpeg(source)

    geotag_from_gpx.add_gps_to_exif(
        source, 52.5162, -13.3777, 123.45, -10.5, updated_filename=target
    )

    geo = _read_geo(target)
    assert np.allclose(geo["latitude"], 52.5162)
    assert np.allclose(geo["longitude"], -13.3777)
    assert np.allclose(geo["altitude"], -10.5)
    assert _read_geo(source) == {}
    with Image.open(target) as image:
        assert image.size == (48, 32)
        assert Base.ImageDescription not in image.getexif()


def test_add_exif_using_timestamp(tmp_path: Path) -> None:
    path = tmp_path / "frame.jpg"
    t0 = datetime.datetime(2020, 1, 1, 12, 0, 0)
    _write_jpeg(path, t0 + datetime.timedelta(seconds=5))
    points: list[tuple[datetime.datetime, float, float, float | None]] = [
        (t0, 40.0, 3.0, 100.0),
        (t0 + datetime.timedelta(seconds=10), 40.001, 3.0, 110.0),
    ]

    geotag_from_gpx.add_exif_using_timestamp(path, points, orientation=6)

    geo = _read_geo(path)
    assert np.allclose(geo["latitude"], 40.0005)
    assert np.allclose(geo["longitude"], 3.0)
    assert np.allclose(geo["altitude"], 105.0)
    with Image.open(path) as image:
        image_exif = image.getexif()
        assert image_exif[Base.Orientation] == 6
        assert image_exif[Base.ImageDescription] == "description"


def test_add_exif_using_timestamp_outside_track(tmp_path: Path) -> None:
    path = tmp_path / "frame.jpg"
    t0 = datetime.datetime(2020, 1, 1, 12, 0, 0)
    _write_jpeg(path)
    points: list[tuple[datetime.datetime, float, float, float | None]] = [
        (t0, 40.0, 3.0, None),
        (t0 + datetime.timedelta(seconds=10), 40.001, 3.0, None),
    ]
    original = path.read_bytes()

    geotag_from_gpx.add_exif_using_timestamp(
        path, points, timestamp=t0 + datetime.timedelta(seconds=20)
    )

    assert path.read_bytes() == original


def test_get_lat_lon_time_sorts_points(tmp_path: Path) -> None:
    gpx_file = tmp_path / "track.gpx"
    gpx_file.write_text(GPX_CONTENT)

    points = geotag_from_gpx.get_lat_lon_time(str(gpx_file), gpx_time="local")

    assert [(p[1], p[2], p[3]) for p in points] == [
        (0.0, 0.0, 5.0),
        (1.0, 2.0, 10.0),
        (2.0, 4.0, None),
    ]
    assert [p[0].second for p in points] == [0, 10, 20]


def test_get_lat_lon_time_utc_converts_to_localtime(tmp_path: Path) -> None:
    gpx_file = tmp_path / "track.gpx"
    gpx_file.write_text(GPX_CONTENT)
    local = geotag_from_gpx.get_lat_lon_time(str(gpx_file), gpx_time="local")

    shift = datetime.timedelta(hours=-7)
    with mock.patch.object(
        geotag_from_gpx, "utc_to_localtime", side_effect=lambda t: t + shift
    ) as to_local:
        utc = geotag_from_gpx.get_lat_lon_time(str(gpx_file))

    assert to_local.call_count == 3
    assert [p[0] for p in utc] == [p[0] + shift for p in local]


def test_utc_to_localtime_applies_offset() -> None:
    with mock.patch.object(geotag_from_gpx, "datetime") as fake_datetime:
        fake_datetime.datetime.utcnow.return_value = datetime.datetime(2020, 1, 1, 15)
        fake_datetime.datetime.now.return_value = datetime.datetime(2020, 1, 1, 10)
        assert geotag_from_gpx.utc_to_localtime(T0) == datetime.datetime(2020, 1, 1, 7)


@pytest.mark.parametrize(
    "start, end, expected",
    [
        ((0.0, 0.0), (1.0, 0.0), 0.0),
        ((0.0, 0.0), (0.0, 1.0), 90.0),
        ((0.0, 0.0), (-1.0, 0.0), 180.0),
        ((0.0, 0.0), (0.0, -1.0), 270.0),
        # Crossing the antimeridian takes the short way round.
        ((0.0, 179.0), (0.0, -179.0), 90.0),
        ((0.0, -179.0), (0.0, 179.0), 270.0),
    ],
)
def test_compute_bearing(
    start: tuple[float, float], end: tuple[float, float], expected: float
) -> None:
    assert geotag_from_gpx.compute_bearing(*start, *end) == pytest.approx(
        expected, abs=1e-9
    )


def test_interpolate_lat_lon_midpoint() -> None:
    points = [_point(0, 0.0, 0.0, 10.0), _point(10, 1.0, 2.0, 20.0)]
    lat, lon, bearing, ele = geotag_from_gpx.interpolate_lat_lon(
        points, T0 + datetime.timedelta(seconds=2.5)
    )
    assert lat == pytest.approx(0.25)
    assert lon == pytest.approx(0.5)
    assert ele == pytest.approx(12.5)
    assert bearing == pytest.approx(geotag_from_gpx.compute_bearing(0.0, 0.0, 1.0, 2.0))


def test_interpolate_lat_lon_at_first_point_and_missing_elevation() -> None:
    points = [_point(0, 0.0, 0.0), _point(10, 1.0, 1.0, 20.0), _point(20, 2.0, 2.0)]
    lat, lon, _, ele = geotag_from_gpx.interpolate_lat_lon(
        points, T0 + datetime.timedelta(seconds=15)
    )
    assert (lat, lon) == pytest.approx((1.5, 1.5))
    assert ele is None


@pytest.mark.parametrize("seconds", [-1, 10, 11])
def test_interpolate_lat_lon_out_of_range(seconds: int) -> None:
    points = [_point(0, 0.0, 0.0), _point(10, 1.0, 1.0)]
    with pytest.raises(ValueError, match="not in scope"):
        geotag_from_gpx.interpolate_lat_lon(
            points, T0 + datetime.timedelta(seconds=seconds)
        )


@pytest.mark.parametrize(
    "value, expected",
    [
        (12.5, (12, 30, 0.0, "N")),
        (-0.75, (0, 45, 0.0, "S")),
        (0.0, (0, 0, 0.0, "")),
        (1.0 + 1 / 3600, (1, 0, 1.0, "N")),
    ],
)
def test_to_deg(value: float, expected: tuple[int, int, float, str]) -> None:
    deg, minutes, seconds, ref = geotag_from_gpx.to_deg(value, ["S", "N"])
    assert (deg, minutes, ref) == (expected[0], expected[1], expected[3])
    assert seconds == pytest.approx(expected[2], abs=1e-5)


def test_gpx_lerp() -> None:
    a = _point(0, 0.0, 10.0, 100.0)
    b = _point(10, 2.0, 20.0, 200.0)
    t, lat, lon, alt = geotag_from_gpx.gpx_lerp(0.25, a, b)
    assert t == T0 + datetime.timedelta(seconds=2.5)
    assert (lat, lon, alt) == pytest.approx((0.5, 12.5, 125.0))

    _, _, _, alt = geotag_from_gpx.gpx_lerp(0.5, a, _point(10, 2.0, 20.0))
    assert alt is None


def test_segment_sphere_intersection() -> None:
    # A at the center, B at distance 4 along x, radius 1 -> 1/4 of the way.
    ratio = geotag_from_gpx.segment_sphere_intersection(
        (0.0, 0.0, 0.0), (4.0, 0.0, 0.0), (0.0, 0.0, 0.0), 1.0
    )
    assert ratio == pytest.approx(0.25)

    # Sphere centered away from A.
    ratio = geotag_from_gpx.segment_sphere_intersection(
        (1.0, 0.0, 0.0), (5.0, 0.0, 0.0), (0.0, 0.0, 0.0), 3.0
    )
    assert ratio == pytest.approx(0.5)


def test_time_next_point_and_distance() -> None:
    last = _point(0, 0.0, 0.0)
    a = _point(2, 0.0, 0.0)
    b = _point(6, 4.0, 4.0)
    nxt = geotag_from_gpx.time_next_point(a, b, last, 4.0)
    assert geotag_from_gpx.time_distance(last, nxt) == pytest.approx(4.0)
    assert (nxt[1], nxt[2]) == pytest.approx((2.0, 2.0))


def test_space_next_point_and_distance() -> None:
    a = _point(0, 0.0, 0.0)
    b = _point(10, 0.0, 0.001)
    nxt = geotag_from_gpx.space_next_point(a, b, a, 50.0)
    assert geotag_from_gpx.space_distance(a, nxt) == pytest.approx(50.0, abs=0.1)
    assert nxt[1] == pytest.approx(0.0)
    assert 0.0 < nxt[2] < 0.001


def test_sample_gpx_by_time() -> None:
    points = [_point(0, 0.0, 0.0), _point(10, 1.0, 1.0)]
    samples = geotag_from_gpx.sample_gpx(points, 0.0, dt=2.5)
    assert [
        geotag_from_gpx.time_distance(points[0], s) for s in samples
    ] == pytest.approx([0.0, 2.5, 5.0, 7.5, 10.0])
    assert samples[2][1] == pytest.approx(0.5)


def test_sample_gpx_by_distance() -> None:
    points = [_point(0, 0.0, 0.0), _point(10, 0.0, 0.001), _point(20, 0.0, 0.002)]
    total = geo.gps_distance(points[0][1:3], points[-1][1:3])
    samples = geotag_from_gpx.sample_gpx(points, 50.0)
    assert len(samples) == int(total // 50.0) + 1
    for s0, s1 in zip(samples, samples[1:]):
        assert geotag_from_gpx.space_distance(s0, s1) == pytest.approx(50.0, abs=0.1)


def test_sample_gpx_zero_spacing_keeps_first_point() -> None:
    points = [_point(0, 0.0, 0.0), _point(10, 0.0, 0.001)]
    assert geotag_from_gpx.sample_gpx(points, 0.0) == [points[0]]
