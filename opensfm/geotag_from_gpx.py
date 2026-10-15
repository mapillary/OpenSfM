#!/usr/bin/python3


import datetime
import math
import os
import shutil
import sys
import time

import numpy as np
from opensfm import geo
from PIL import Image, TiffImagePlugin
from PIL.ExifTags import Base, GPS, IFD

try:
    import gpxpy
except ImportError:
    print("ERROR: gpxpy module not available")


"""
(source: https://github.com/mapillary/mapillary_tools)


Script for geotagging images using a gpx file from an external GPS.
Intended as a lightweight tool.

!!! This version needs testing, please report issues.!!!

Uses the capture time in EXIF and looks up an interpolated lat, lon, bearing
for each image, and writes the values to the EXIF of the image.

You can supply a time offset in seconds if the GPS clock and camera clocks are not in sync.

Requires gpxpy, e.g. 'pip install gpxpy'
"""


def utc_to_localtime(utc_time: datetime.datetime) -> datetime.datetime:
    utc_offset_timedelta = datetime.datetime.utcnow() - datetime.datetime.now()
    return utc_time - utc_offset_timedelta


def get_lat_lon_time(
    gpx_file: str, gpx_time: str = "utc"
) -> list[tuple[datetime.datetime, float, float, float | None]]:
    """
    Read location and time stamps from a track in a GPX file.

    Returns a list of tuples (time, lat, lon, elevation).

    GPX stores time in UTC, assume your camera used the local
    timezone and convert accordingly.
    """
    with open(gpx_file, "r") as f:
        gpx = gpxpy.parse(f)

    points = []
    for track in gpx.tracks:
        for segment in track.segments:
            for point in segment.points:
                assert point.time is not None, "GPX point without timestamp"
                ptime = (
                    utc_to_localtime(point.time) if gpx_time == "utc" else point.time
                )
                points.append((ptime, point.latitude, point.longitude, point.elevation))

    # sort by time just in case
    points.sort()

    return points


def compute_bearing(
    start_lat: float, start_lon: float, end_lat: float, end_lon: float
) -> float:
    """
    Get the compass bearing from start to end.

    Formula from
    http://www.movable-type.co.uk/scripts/latlong.html
    """
    # make sure everything is in radians
    start_lat = math.radians(start_lat)
    start_lon = math.radians(start_lon)
    end_lat = math.radians(end_lat)
    end_lon = math.radians(end_lon)

    dLong = end_lon - start_lon

    if abs(dLong) > math.pi:
        if dLong > 0.0:
            dLong = -(2.0 * math.pi - dLong)
        else:
            dLong = 2.0 * math.pi + dLong

    y = math.sin(dLong) * math.cos(end_lat)
    x = math.cos(start_lat) * math.sin(end_lat) - math.sin(start_lat) * math.cos(
        end_lat
    ) * math.cos(dLong)
    bearing = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    return bearing


def interpolate_lat_lon(
    points: list[tuple[datetime.datetime, float, float, float | None]],
    t: datetime.datetime,
) -> tuple[float, float, float, float | None]:
    """
    Return interpolated lat, lon and compass bearing for time t.

    Points is a list of tuples (time, lat, lon, elevation), t a datetime object.
    """

    # find the enclosing points in sorted list
    if (t < points[0][0]) or (t >= points[-1][0]):
        raise ValueError("Time t not in scope of gpx file.")

    for i, point in enumerate(points):
        if t < point[0]:
            if i > 0:
                before = points[i - 1]
            else:
                before = points[i]
            after = points[i]
            break

    # time diff
    # pyrefly: ignore [unbound-name]
    dt_before = (t - before[0]).total_seconds()
    # pyrefly: ignore [unbound-name]
    dt_after = (after[0] - t).total_seconds()

    # simple linear interpolation
    lat = (before[1] * dt_after + after[1] * dt_before) / (dt_before + dt_after)
    lon = (before[2] * dt_after + after[2] * dt_before) / (dt_before + dt_after)

    bearing = compute_bearing(before[1], before[2], after[1], after[2])

    if before[3] is not None and after[3] is not None:
        ele = (before[3] * dt_after + after[3] * dt_before) / (dt_before + dt_after)
    else:
        ele = None

    return lat, lon, bearing, ele


def to_deg(value: float, loc: list[str]) -> tuple[int, int, float, str]:
    """
    Convert decimal position to degrees.
    """
    if value < 0:
        loc_value = loc[0]
    elif value > 0:
        loc_value = loc[1]
    else:
        loc_value = ""
    abs_value = abs(value)
    deg = int(abs_value)
    t1 = (abs_value - deg) * 60
    mint = int(t1)
    sec = round((t1 - mint) * 60, 6)
    return (deg, mint, sec, loc_value)


def gpx_lerp(
    alpha: float,
    a: tuple[datetime.datetime, float, float, float | None],
    b: tuple[datetime.datetime, float, float, float | None],
) -> tuple[datetime.datetime, float, float, float | None]:
    """Interpolate gpx point as (1 - alpha) * a + alpha * b"""
    dt = alpha * (b[0] - a[0]).total_seconds()
    t = a[0] + datetime.timedelta(seconds=dt)
    lat = (1 - alpha) * a[1] + alpha * b[1]
    lon = (1 - alpha) * a[2] + alpha * b[2]
    alt = (
        (1 - alpha) * a[3] + alpha * b[3]
        if a[3] is not None and b[3] is not None
        else None
    )
    return t, lat, lon, alt


def segment_sphere_intersection(
    A: tuple[float, float, float],
    B: tuple[float, float, float],
    C: tuple[float, float, float],
    r: float,
) -> float:
    """Intersect the segment AB and the sphere (C,r).

    Assumes A is inside the sphere and B is outside.
    Return the ratio between the length of AI and the length
    of AB, where I is the intersection.
    """
    AB = np.array(B) - np.array(A)
    CA = np.array(A) - np.array(C)
    a = AB.dot(AB)
    b = 2 * AB.dot(CA)
    c = CA.dot(CA) - r**2
    d = max(0, b**2 - 4 * a * c)
    return (-b + np.sqrt(d)) / (2 * a)


def space_next_point(
    a: tuple[datetime.datetime, float, float, float | None],
    b: tuple[datetime.datetime, float, float, float | None],
    last: tuple[datetime.datetime, float, float, float | None],
    dx: float,
) -> tuple[datetime.datetime, float, float, float | None]:
    A = geo.ecef_from_lla(a[1], a[2], 0.0)
    B = geo.ecef_from_lla(b[1], b[2], 0.0)
    C = geo.ecef_from_lla(last[1], last[2], 0.0)
    alpha = segment_sphere_intersection(A, B, C, dx)
    return gpx_lerp(alpha, a, b)


def time_next_point(
    a: tuple[datetime.datetime, float, float, float | None],
    b: tuple[datetime.datetime, float, float, float | None],
    last: tuple[datetime.datetime, float, float, float | None],
    dt: float,
) -> tuple[datetime.datetime, float, float, float | None]:
    da = (a[0] - last[0]).total_seconds()
    db = (b[0] - last[0]).total_seconds()
    alpha = (dt - da) / (db - da)
    return gpx_lerp(alpha, a, b)


def time_distance(
    a: tuple[datetime.datetime, float, float, float | None],
    b: tuple[datetime.datetime, float, float, float | None],
) -> float:
    return (b[0] - a[0]).total_seconds()


def space_distance(
    a: tuple[datetime.datetime, float, float, float | None],
    b: tuple[datetime.datetime, float, float, float | None],
) -> float:
    return geo.gps_distance(a[1:3], b[1:3])


def sample_gpx(
    points: list[tuple[datetime.datetime, float, float, float | None]],
    dx: float,
    dt: float | None = None,
) -> list[tuple[datetime.datetime, float, float, float | None]]:
    if dt is not None:
        dx = float(dt)
        print("Sampling GPX file every {0} seconds".format(dx))
        distance = time_distance
        next_point = time_next_point
    else:
        print("Sampling GPX file every {0} meters".format(dx))
        distance = space_distance
        next_point = space_next_point

    key_points = [points[0]]
    a = points[0]
    for i in range(1, len(points)):
        a, b = points[i - 1], points[i]
        dx_b = distance(key_points[-1], b)
        while dx and dx_b >= dx:
            a = next_point(a, b, key_points[-1], dx)
            key_points.append(a)
            assert np.fabs(dx - distance(key_points[-2], key_points[-1])) < 0.1
            dx_b = distance(key_points[-1], b)
    print("{} points sampled".format(len(key_points)))
    return key_points


EXIF_DATETIME_FORMAT = "%Y:%m:%d %H:%M:%S"


def _dms_rationals(
    deg: tuple[int, int, float, str],
) -> tuple[
    TiffImagePlugin.IFDRational,
    TiffImagePlugin.IFDRational,
    TiffImagePlugin.IFDRational,
]:
    return (
        TiffImagePlugin.IFDRational(deg[0], 1),
        TiffImagePlugin.IFDRational(int(deg[1]), 1),
        TiffImagePlugin.IFDRational(int(deg[2] * 1000000), 1000000),
    )


def _set_gps_exif(
    exif: Image.Exif,
    lat: float,
    lon: float,
    bearing: float,
    bearing_denominator: int,
    elevation: float | None,
) -> None:
    lat_deg = to_deg(lat, ["S", "N"])
    lon_deg = to_deg(lon, ["W", "E"])
    gps: dict[int, object] = {
        GPS.GPSVersionID: b"\x02\x00\x00\x00",
        GPS.GPSLatitudeRef: lat_deg[3],
        GPS.GPSLatitude: _dms_rationals(lat_deg),
        GPS.GPSLongitudeRef: lon_deg[3],
        GPS.GPSLongitude: _dms_rationals(lon_deg),
        GPS.GPSMapDatum: "WGS-84",
        GPS.GPSImgDirectionRef: "T",
        GPS.GPSImgDirection: TiffImagePlugin.IFDRational(
            int(bearing * bearing_denominator), bearing_denominator
        ),
    }
    if elevation is not None:
        gps[GPS.GPSAltitudeRef] = 0 if elevation >= 0 else 1
        gps[GPS.GPSAltitude] = TiffImagePlugin.IFDRational(
            int(abs(elevation) * 100), 100
        )
    exif[IFD.GPSInfo] = gps


def _save_with_exif(
    image: Image.Image, filename: os.PathLike[str] | str, exif: Image.Exif
) -> None:
    image.save(
        filename,
        format=image.format,
        exif=exif,
        icc_profile=image.info.get("icc_profile"),
        quality="keep",
        subsampling="keep",
    )


def add_gps_to_exif(
    filename: os.PathLike[str] | str,
    lat: float,
    lon: float,
    bearing: float,
    elevation: float | None,
    updated_filename: os.PathLike[str] | str | None = None,
    remove_image_description: bool = True,
) -> None:
    """
    Given lat, lon, bearing, elevation, write to EXIF
    """
    if updated_filename is not None:
        shutil.copy2(filename, updated_filename)
        filename = updated_filename

    with Image.open(filename) as image:
        image.load()
        exif = image.getexif()
        _set_gps_exif(exif, lat, lon, bearing, 100, elevation)
        if remove_image_description:
            exif.pop(Base.ImageDescription, None)
        _save_with_exif(image, filename, exif)


def add_exif_using_timestamp(
    filename: os.PathLike[str] | str,
    points: list[tuple[datetime.datetime, float, float, float | None]],
    offset_time: int = 0,
    timestamp: datetime.datetime | None = None,
    orientation: int = 1,
    image_description: str | None = None,
) -> None:
    """
    Find lat, lon and bearing of filename and write to EXIF.
    """
    with Image.open(filename) as image:
        image.load()
        exif = image.getexif()
        exif_ifd = exif.get_ifd(IFD.Exif)
        if timestamp:
            exif_ifd[Base.DateTimeOriginal] = timestamp.strftime(EXIF_DATETIME_FORMAT)
            exif[IFD.Exif] = exif_ifd
            t = timestamp
        else:
            t = datetime.datetime.strptime(
                exif_ifd[Base.DateTimeOriginal], EXIF_DATETIME_FORMAT
            )

        # subtract offset in s between gpx time and exif time
        t = t - datetime.timedelta(seconds=offset_time)

        try:
            lat, lon, bearing, elevation = interpolate_lat_lon(points, t)
            _set_gps_exif(exif, lat, lon, bearing, 1000, elevation)
            exif[Base.Orientation] = orientation
            if image_description is not None:
                exif[Base.ImageDescription] = image_description
            _save_with_exif(image, filename, exif)
        except ValueError as e:
            print("Skipping {0}: {1}".format(filename, e))
            return

    print(
        "Added geodata to: {0} ({1}, {2}, {3}), altitude {4}".format(
            filename, lat, lon, bearing, elevation
        )
    )


if __name__ == "__main__":
    """
    Use from command line as: python geotag_from_gpx.py path gpx_file time_offset

    The time_offset is optional and defaults to 0.
    It is defined as 'exif time' - 'gpx time' in whole seconds,
    so if your camera clock is ahead of the gpx clock by 2s,
    then the offset is 2.
    """

    if len(sys.argv) > 4:
        print("Usage: python geotag_from_gpx.py path gpx_file time_offset")
        raise IOError("Bad input parameters.")
    path: str = sys.argv[1]
    gpx_filename: str = sys.argv[2]

    if len(sys.argv) == 4:
        time_offset = int(sys.argv[3])
    else:
        time_offset = 0

    if path.lower().endswith(".jpg"):
        # single file
        file_list: list[str] = [path]
    else:
        # folder(s)
        file_list: list[str] = []
        for root, _, files in os.walk(path):
            file_list += [
                os.path.join(root, filename)
                for filename in files
                if filename.lower().endswith(".jpg")
            ]

    # start time
    t: float = time.time()

    # read gpx file to get track locations
    gpx = get_lat_lon_time(gpx_filename)

    print(
        "===\nStarting geotagging of {0} images using {1}.\n===".format(
            len(file_list), gpx_filename
        )
    )

    for filepath in file_list:
        add_exif_using_timestamp(filepath, gpx, time_offset)

    print(
        "Done geotagging {0} images in {1} seconds.".format(
            len(file_list), time.time() - t
        )
    )
