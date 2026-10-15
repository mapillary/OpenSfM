import datetime
from pathlib import Path

import numpy as np
from opensfm import exif, geotag_from_gpx
from PIL import Image
from PIL.ExifTags import Base, IFD


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
