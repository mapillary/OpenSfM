import datetime
import io
from types import SimpleNamespace
from typing import Any, cast

import numpy as np
import pytest
from exifread.utils import Ratio
from opensfm import config, exif
from opensfm.dataset_base import DataSetBase


def _tag(values: Any) -> SimpleNamespace:
    return SimpleNamespace(values=values)


def _exif(
    tags: dict[str, Any],
    xmp: str = "",
    image_size: tuple[int, int] = (300, 400),
    use_exif_size: bool = True,
) -> exif.EXIF:
    # Not a real image: exifread finds no tags, so the test injects them.
    fileobj = io.BytesIO(b"\x00\x01" + xmp.encode())
    parsed = exif.EXIF(fileobj, lambda: image_size, use_exif_size, name="img.jpg")
    parsed.tags = tags
    return parsed


def _xmp(**attributes: str) -> str:
    attrs = " ".join(f'{k}="{v}"' for k, v in attributes.items())
    return (
        '<x:xmpmeta xmlns:x="adobe:ns:meta/"><rdf:RDF '
        'xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">'
        f"<rdf:Description {attrs}/></rdf:RDF></x:xmpmeta>"
    )


def _data() -> DataSetBase:
    return cast(DataSetBase, SimpleNamespace(config=config.default_config()))


def _epoch(
    year: int, month: int, day: int, hour: int = 0, minute: int = 0, second: int = 0
) -> float:
    moment = datetime.datetime(year, month, day, hour, minute, second)
    return (moment - datetime.datetime(1970, 1, 1)).total_seconds()


def test_eval_frac_and_gps_to_decimal() -> None:
    assert exif.eval_frac(Ratio(3, 4)) == 0.75
    assert exif.eval_frac(Ratio(1, 0)) is None
    dms = [Ratio(10, 1), Ratio(30, 1), Ratio(36, 1)]
    assert exif.gps_to_decimal(dms, "N") == pytest.approx(10.51)
    assert exif.gps_to_decimal(dms, "W") == pytest.approx(-10.51)
    assert exif.gps_to_decimal([Ratio(1, 0), Ratio(0, 1), Ratio(0, 1)], "N") is None


def test_get_tag_as_float() -> None:
    tags = {"A": _tag([Ratio(5, 2), 7]), "B": _tag([Ratio(1, 0)])}
    assert exif.get_tag_as_float(tags, "A") == 2.5
    assert exif.get_tag_as_float(tags, "A", 1) == 7.0
    assert exif.get_tag_as_float(tags, "B") is None
    assert exif.get_tag_as_float(tags, "missing") is None


def test_compute_focal() -> None:
    assert exif.compute_focal(18.0, None, None, None) == (18.0, 0.5)
    assert exif.compute_focal(None, 4.0, 8.0, None) == (18.0, 0.5)
    assert exif.compute_focal(0.0, None, None, None) == (0.0, 0.0)
    # Sensor width looked up from the sensor database by make/model.
    focal_35, ratio = exif.compute_focal(None, 4.0, None, "nikon coolpix p7000")
    sensor_width = exif.sensor_data()["nikon coolpix p7000"]
    assert ratio == pytest.approx(4.0 / sensor_width)
    assert focal_35 == pytest.approx(36.0 * ratio)


def test_sensor_string_and_camera_id() -> None:
    assert exif.sensor_string("Canon", "Canon EOS 5D") == "canon eos 5d"
    assert exif.sensor_string("unknown", "unknown model") == "unknown unknown model"
    camera = exif.camera_id(
        {
            "make": "GoPro",
            "model": "GoPro HERO8",
            "width": 4000.0,
            "height": 3000,
            "projection_type": "perspective",
            "focal_ratio": 0.4567891,
        }
    )
    assert camera == "v2 gopro hero8 4000 3000 perspective 0.4567"


def test_xmp_parsing() -> None:
    xmp_bytes = io.BytesIO(
        b"\xff\xd8junk" + _xmp(**{"GPano:ProjectionType": "equirectangular"}).encode()
    )
    xmp = exif.get_xmp(xmp_bytes)
    assert (
        xmp == [{"@GPano:ProjectionType": "equirectangular"}]
        or xmp[0].get("@GPano:ProjectionType") == "equirectangular"
    )
    assert exif.get_gpano_from_xmp(xmp) == xmp[0]
    assert exif.get_gpano_from_xmp([{"@other": "x"}]) == {}
    assert exif.get_xmp(io.BytesIO(b"no xmp here")) == []
    # Unparseable XMP yields no entries.
    assert exif.get_xmp(io.BytesIO(b"<x:xmpmeta <<< </x:xmpmeta>")) == []
    assert exif.parse_xmp_string("<a>1</a>") == {"a": "1"}
    assert exif.unescape_string("caf\\xe9") == "café"


def test_image_size_sources() -> None:
    tags = {
        "EXIF ExifImageWidth": _tag([640]),
        "EXIF ExifImageLength": _tag([480]),
        "Image ImageWidth": _tag([320]),
        "Image ImageLength": _tag([240]),
    }
    assert _exif(tags).extract_image_size() == (640, 480)
    del tags["EXIF ExifImageWidth"]
    assert _exif(tags).extract_image_size() == (320, 240)
    # Loader returns (height, width).
    assert _exif(tags, use_exif_size=False).extract_image_size() == (400, 300)
    assert _exif({}).extract_image_size() == (400, 300)


def test_make_model_and_orientation() -> None:
    parsed = _exif(
        {
            "EXIF LensMake": _tag(b"LensCo"),
            "Image Make": _tag("BodyCo"),
            "Image Model": _tag(b"\xff\xfe"),
            "Image Orientation": _tag([6]),
        }
    )
    assert parsed.extract_make() == "LensCo"
    assert parsed.extract_model() == "unknown"
    assert parsed.extract_orientation() == 6
    assert _exif({"Image Make": _tag("BodyCo")}).extract_make() == "BodyCo"
    assert _exif({"EXIF LensModel": _tag("Lens")}).extract_model() == "Lens"
    assert _exif({"Image Orientation": _tag([0])}).extract_orientation() == 1
    assert _exif({}).extract_make() == "unknown"


@pytest.mark.parametrize(
    "unit, x_res, y_res, expected",
    [
        (2, Ratio(100, 1), None, 4000 / 100 * 25.4),
        (3, Ratio(1000, 1), None, 4000 / 1000 * 10),
        (4, Ratio(800, 1), None, 4000 / 800),
        (5, Ratio(1, 1), None, 4000 * 0.001),
        (2, Ratio(0, 1), Ratio(200, 1), 4000 / 200 * 25.4),
        (2, Ratio(0, 1), None, None),
        (2, Ratio(1, 0), None, None),
        (9, Ratio(100, 1), None, None),
    ],
)
def test_extract_sensor_width(
    unit: int, x_res: Ratio, y_res: Ratio | None, expected: float | None
) -> None:
    tags = {
        "EXIF FocalPlaneResolutionUnit": _tag([unit]),
        "EXIF FocalPlaneXResolution": _tag([x_res]),
        "EXIF ExifImageWidth": _tag([4000]),
        "EXIF ExifImageLength": _tag([3000]),
    }
    if y_res is not None:
        tags["EXIF FocalPlaneYResolution"] = _tag([y_res])
    width = _exif(tags).extract_sensor_width()
    if expected is None:
        assert width is None
    else:
        assert width == pytest.approx(expected)
    assert _exif({}).extract_sensor_width() is None


def test_extract_focal_uses_sensor_width() -> None:
    tags = {
        "EXIF FocalLength": _tag([Ratio(4, 1)]),
        "EXIF FocalPlaneResolutionUnit": _tag([4]),
        "EXIF FocalPlaneXResolution": _tag([Ratio(500, 1)]),
        "EXIF ExifImageWidth": _tag([4000]),
        "EXIF ExifImageLength": _tag([3000]),
    }
    assert _exif(tags).extract_focal() == pytest.approx((18.0, 0.5))


def test_extract_geo_from_gps_tags() -> None:
    tags = {
        "GPS GPSLatitude": _tag([Ratio(10, 1), Ratio(30, 1), Ratio(0, 1)]),
        "GPS GPSLatitudeRef": _tag("S"),
        "GPS GPSLongitude": _tag([Ratio(20, 1), Ratio(15, 1), Ratio(0, 1)]),
        "GPS GPSLongitudeRef": _tag("W"),
        "GPS GPSAltitude": _tag([Ratio(1234, 10)]),
        "GPS GPSAltitudeRef": _tag([1]),
        "GPS GPSDOP": _tag([Ratio(5, 2)]),
    }
    geo = _exif(tags).extract_geo()
    assert geo == pytest.approx(
        {"latitude": -10.5, "longitude": -20.25, "altitude": -123.4, "dop": 2.5}
    )


def test_extract_geo_defaults_and_altitude_variants() -> None:
    tags = {
        "GPS GPSLatitude": _tag([Ratio(1, 1), Ratio(0, 1), Ratio(0, 1)]),
        "GPS GPSLongitude": _tag([Ratio(2, 1), Ratio(0, 1), Ratio(0, 1)]),
        "GPS GPSAltitude": _tag([50000]),
    }
    geo = _exif(tags).extract_geo()
    # Missing refs default to N/E and altitude is capped.
    assert geo == {"latitude": 1.0, "longitude": 2.0, "altitude": exif.maximum_altitude}
    assert _exif({"GPS GPSAltitude": _tag(["bad"])}).extract_altitude() is None
    assert _exif({}).extract_geo() == {}


def test_extract_geo_from_dji_xmp() -> None:
    xmp = _xmp(
        **{
            "drone-dji:Latitude": "-12.5",
            "drone-dji:Longitude": "+45.25",
            "drone-dji:AbsoluteAltitude": "+101.5",
        }
    )
    parsed = _exif({}, xmp=xmp)
    assert parsed.has_xmp()
    assert parsed.extract_geo() == {
        "latitude": -12.5,
        "longitude": 45.25,
        "altitude": 101.5,
    }


def test_capture_time_from_gps() -> None:
    tags = {
        "GPS GPSDate": _tag("2020:01:02"),
        "GPS GPSTimeStamp": _tag([Ratio(3, 1), Ratio(4, 1), Ratio(55, 10)]),
    }
    assert _exif(tags).extract_capture_time() == _epoch(2020, 1, 2, 3, 4, 5) + 0.5


def test_capture_time_falls_back_to_datetime_with_offset() -> None:
    tags = {
        "GPS GPSDate": _tag("2020:01:02"),
        "GPS GPSTimeStamp": _tag([Ratio(1, 0), Ratio(4, 1), Ratio(5, 1)]),
        "EXIF DateTimeOriginal": _tag("bad date"),
        "EXIF DateTimeDigitized": _tag("2021:05:06 07:08:09"),
        "EXIF SubSecTimeDigitized": _tag("25"),
        "EXIF Tag 0x9012": _tag("+02:00"),
    }
    assert _exif(tags).extract_capture_time() == _epoch(2021, 5, 6, 5, 8, 9) + 0.25


def test_capture_time_invalid_offset_and_missing() -> None:
    tags = {
        "Image DateTime": _tag("2021:05:06 07:08:09"),
        "Image Tag 0x9010": _tag("garbage"),
    }
    assert _exif(tags).extract_capture_time() == _epoch(2021, 5, 6, 7, 8, 9)
    assert _exif(
        {"EXIF DateTimeOriginal": _tag("2000:01:01 00:00:00")}
    ).extract_capture_time() == _epoch(2000, 1, 1)
    assert _exif({}).extract_capture_time() == 0.0


def test_extract_opk_dji_gimbal_matches_camera_tags() -> None:
    geo = {"latitude": 10.0, "longitude": 20.0}
    camera = _xmp(**{"Camera:Yaw": "30", "Camera:Pitch": "5", "Camera:Roll": "2"})
    dji = _xmp(
        **{
            "drone-dji:GimbalYawDegree": "30",
            "drone-dji:GimbalPitchDegree": "-85",
            "drone-dji:GimbalRollDegree": "2",
        }
    )
    opk_camera = _exif({}, xmp=camera).extract_opk(geo)
    opk_dji = _exif({}, xmp=dji).extract_opk(geo)
    assert opk_camera is not None and opk_dji is not None
    for key in ("omega", "phi", "kappa"):
        assert opk_dji[key] == pytest.approx(opk_camera[key])
    # Only the yaw differs from a zero-yaw camera, so only kappa changes much.
    zero_yaw = _xmp(**{"Camera:Yaw": "0", "Camera:Pitch": "5", "Camera:Roll": "2"})
    opk_zero = _exif({}, xmp=zero_yaw).extract_opk(geo)
    assert opk_zero is not None
    assert abs(opk_camera["kappa"] - opk_zero["kappa"]) > 10.0


def test_extract_opk_missing_inputs() -> None:
    geo = {"latitude": 10.0, "longitude": 20.0}
    assert _exif({}).extract_opk(geo) is None
    assert _exif({}, xmp=_xmp(other="1")).extract_opk(geo) is None
    bad = _xmp(**{"Camera:Yaw": "x", "Camera:Pitch": "0", "Camera:Roll": "0"})
    assert _exif({}, xmp=bad).extract_opk(geo) is None
    assert _exif({}, xmp=_xmp(**{"Camera:Yaw": "0"})).extract_opk({}) is None


def test_extract_exif_end_to_end() -> None:
    xmp = _xmp().replace(
        "/></rdf:RDF>",
        "><GPano:ProjectionType>equirectangular</GPano:ProjectionType>"
        "</rdf:Description></rdf:RDF>",
    )
    fileobj = io.BytesIO(b"\x00" + xmp.encode())
    d = exif.extract_exif_from_file(fileobj, lambda: (100, 200), True, name="p.jpg")
    assert d["width"] == 200 and d["height"] == 100
    assert d["projection_type"] == "equirectangular"
    assert d["make"] == "unknown" and d["model"] == "unknown"
    assert d["gps"] == {}
    assert d["capture_time"] == 0.0
    assert "opk" not in d
    assert d["camera"] == "v2 unknown unknown 200 100 equirectangular 0.0"


def test_extract_exif_includes_opk() -> None:
    xmp = _xmp(
        **{
            "drone-dji:Latitude": "+1.0",
            "drone-dji:Longitude": "+2.0",
            "Camera:Yaw": "0",
            "Camera:Pitch": "0",
            "Camera:Roll": "0",
        }
    )
    d = _exif({}, xmp=xmp).extract_exif()
    assert set(d["opk"]) == {"omega", "phi", "kappa"}


def test_hard_coded_calibration() -> None:
    def meta(make: str, model: str, focal_ratio: float = 0.5) -> dict[str, Any]:
        return {"make": make, "model": model, "focal_ratio": focal_ratio}

    hero8 = exif.hard_coded_calibration(meta("GoPro ", "HERO8 Black"))
    assert hero8 is not None and hero8["projection_type"] == "brown"
    assert exif.hard_coded_calibration(meta("gopro", "hero3")) is None
    assert exif.hard_coded_calibration(meta("xiaoyi", "any")) == {
        "focal": 0.5,
        "k1": -0.19,
        "k2": 0.028,
    }
    assert exif.hard_coded_calibration(meta("nobody", "x")) is None


def test_simple_calibrations() -> None:
    assert exif.focal_ratio_calibration({"focal_ratio": 0.7}) == {
        "focal": 0.7,
        "k1": 0.0,
        "k2": 0.0,
        "p1": 0.0,
        "p2": 0.0,
        "k3": 0.0,
    }
    assert exif.focal_ratio_calibration({"focal_ratio": 0}) is None
    xy = exif.focal_xy_calibration({"focal_x": 0.8, "c_x": 0.1})
    assert xy is not None
    assert (xy["focal_x"], xy["focal_y"], xy["c_x"], xy["c_y"]) == (0.8, 0.8, 0.1, 0.0)
    assert exif.focal_xy_calibration({}) is None
    default = exif.default_calibration(_data())
    prior = config.default_config()["default_focal_prior"]
    assert (default["focal"], default["focal_x"], default["k6"]) == (prior, prior, 0.0)


@pytest.mark.parametrize(
    "metadata, expected_keys, projection_type",
    [
        ({"focal_ratio": 0.6}, {"focal"}, "perspective"),
        ({"projection_type": "BROWN", "focal_ratio": 0.6}, {"focal_x"}, "brown"),
        ({"projection_type": "fisheye"}, {"focal", "focal_x"}, "fisheye"),
        (
            {"make": "xiaoyi", "model": "y", "focal_ratio": 0.1},
            {"focal"},
            "perspective",
        ),
    ],
)
def test_calibration_from_metadata(
    metadata: dict[str, Any], expected_keys: set[str], projection_type: str
) -> None:
    full = {"make": "unknown", "model": "unknown", "focal_ratio": 0.0, **metadata}
    calib = exif.calibration_from_metadata(full, _data())
    assert expected_keys <= set(calib)
    assert calib["projection_type"] == projection_type


@pytest.mark.parametrize(
    "projection_type",
    [
        "perspective",
        "brown",
        "fisheye",
        "fisheye_opencv",
        "fisheye62",
        "fisheye624",
        "radial",
        "simple_radial",
        "dual",
        "equirectangular",
        "spherical",
    ],
)
def test_camera_from_exif_metadata(projection_type: str) -> None:
    def calibration(metadata: dict[str, Any], data: DataSetBase) -> dict[str, Any]:
        calib = exif.default_calibration(data)
        calib.update({"projection_type": projection_type, "transition": 0.5})
        calib["focal"] = calib["focal_x"] = 0.8
        calib["focal_y"] = 0.4
        return calib

    metadata = {"camera": "cam", "width": 640.0, "height": 480}
    camera = exif.camera_from_exif_metadata(metadata, _data(), calibration)
    assert camera.id == "cam"
    assert (camera.width, camera.height) == (640, 480)
    expected = "spherical" if projection_type == "equirectangular" else projection_type
    assert camera.projection_type == expected
    if projection_type == "brown":
        assert camera.aspect_ratio == pytest.approx(0.5)


def test_camera_from_exif_metadata_unknown_projection() -> None:
    with pytest.raises(ValueError, match="Unknown projection type"):
        exif.camera_from_exif_metadata(
            {"camera": "c", "width": 1, "height": 1},
            _data(),
            lambda m, d: {"projection_type": "weird"},
        )


def test_camera_from_exif_metadata_default_calibration() -> None:
    camera = exif.camera_from_exif_metadata(
        {
            "camera": "c",
            "width": 10,
            "height": 5,
            "focal_ratio": 0.9,
            "make": "a",
            "model": "b",
        },
        _data(),
    )
    assert camera.projection_type == "perspective"
    assert camera.focal == pytest.approx(0.9)
    assert np.isclose(camera.k1, 0.0)
