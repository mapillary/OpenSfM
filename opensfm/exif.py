import datetime
import logging
from codecs import encode, decode
from typing import Tuple

import exifread
import xmltodict as x2d
from opensfm import pygeometry
from opensfm.dataset import DataSetBase
from opensfm.sensors import sensor_data

logger = logging.getLogger(__name__)

inch_in_mm = 25.4
cm_in_mm = 10
um_in_mm = 0.001
default_projection = "perspective"
maximum_altitude = 1e4


def eval_frac(value):
    try:
        return float(value.num) / float(value.den)
    except ZeroDivisionError:
        return None


def gps_to_decimal(values, reference):
    sign = 1 if reference in "NE" else -1
    degrees = eval_frac(values[0])
    minutes = eval_frac(values[1])
    seconds = eval_frac(values[2])
    if degrees is not None and minutes is not None and seconds is not None:
        return sign * (degrees + minutes / 60 + seconds / 3600)
    return None


def get_tag_as_float(tags, key, index=0):
    if key in tags:
        val = tags[key].values[index]
        if isinstance(val, exifread.utils.Ratio):
            ret_val = eval_frac(val)
            if ret_val is None:
                logger.error(
                    'The rational "{2}" of tag "{0:s}" at index {1:d} c'
                    "aused a division by zero error".format(key, index, val)
                )
            return ret_val
        else:
            return float(val)
    else:
        return None


def compute_focal(focal_35, focal, sensor_width, sensor_string) -> Tuple[float, float]:
    if focal_35 is not None and focal_35 > 0:
        focal_ratio = focal_35 / 36.0  # 35mm film produces 36x24mm pictures.
    else:
        if not sensor_width:
            sensor_width = sensor_data.get(sensor_string, None)
        if sensor_width and focal:
            focal_ratio = focal / sensor_width
            focal_35 = 36.0 * focal_ratio
        else:
            focal_35 = 0.0
            focal_ratio = 0.0
    return focal_35, focal_ratio


def sensor_string(make, model):
    if make != "unknown":
        # remove duplicate 'make' information in 'model'
        model = model.replace(make, "")
    return (make.strip() + " " + model.strip()).strip().lower()


def camera_id(exif):
    return camera_id_(
        exif["make"],
        exif["model"],
        exif["width"],
        exif["height"],
        exif["projection_type"],
        exif["focal_ratio"],
    )


def camera_id_(make, model, width, height, projection_type, focal):
    if make != "unknown":
        # remove duplicate 'make' information in 'model'
        model = model.replace(make, "")
    return " ".join(
        [
            "v2",
            make.strip(),
            model.strip(),
            str(int(width)),
            str(int(height)),
            projection_type,
            str(float(focal))[:6],
        ]
    ).lower()


def extract_exif_from_file(fileobj, image_size_loader, use_exif_size, name=None):
    exif_data = EXIF(fileobj, image_size_loader, use_exif_size, name=name)
    d = exif_data.extract_exif()
    return d


def unescape_string(s):
    return decode(encode(s, "latin-1", "backslashreplace"), "unicode-escape")


def parse_xmp_string(xmp_str):
    for _ in range(2):
        try:
            return x2d.parse(xmp_str)
        except Exception:
            xmp_str = unescape_string(xmp_str)
    return None


def get_xmp(fileobj):
    """Extracts XMP metadata from and image fileobj"""
    img_str = str(fileobj.read())
    xmp_start = img_str.find("<x:xmpmeta")
    xmp_end = img_str.find("</x:xmpmeta")

    if xmp_start < xmp_end:
        xmp_str = img_str[xmp_start : xmp_end + 12]
        xdict = parse_xmp_string(xmp_str)
        if xdict is None:
            return []
        xdict = xdict.get("x:xmpmeta", {})
        xdict = xdict.get("rdf:RDF", {})
        xdict = xdict.get("rdf:Description", {})
        if isinstance(xdict, list):
            return xdict
        else:
            return [xdict]
    else:
        return []


def get_gpano_from_xmp(xmp):
    for i in xmp:
        for k in i:
            if "GPano" in k:
                return i
    return {}


class EXIF:
    def __init__(self, fileobj, image_size_loader, use_exif_size=True, name=None):
        self.image_size_loader = image_size_loader
        self.use_exif_size = use_exif_size
        self.fileobj = fileobj
        self.tags = exifread.process_file(fileobj, details=False)
        fileobj.seek(0)
        self.xmp = get_xmp(fileobj)
        self.fileobj_name = self.fileobj.name if name is None else name

    def extract_image_size(self):
        if (
            self.use_exif_size
            and "EXIF ExifImageWidth" in self.tags
            and "EXIF ExifImageLength" in self.tags
        ):
            width, height = (
                int(self.tags["EXIF ExifImageWidth"].values[0]),
                int(self.tags["EXIF ExifImageLength"].values[0]),
            )
        elif (
            self.use_exif_size
            and "Image ImageWidth" in self.tags
            and "Image ImageLength" in self.tags
        ):
            width, height = (
                int(self.tags["Image ImageWidth"].values[0]),
                int(self.tags["Image ImageLength"].values[0]),
            )
        else:
            height, width = self.image_size_loader()
        return width, height

    def _decode_make_model(self, value):
        """Python 2/3 compatible decoding of make/model field."""
        if hasattr(value, "decode"):
            try:
                return value.decode("utf-8")
            except UnicodeDecodeError:
                return "unknown"
        else:
            return value

    def extract_make(self):
        # Camera make and model
        if "EXIF LensMake" in self.tags:
            make = self.tags["EXIF LensMake"].values
        elif "Image Make" in self.tags:
            make = self.tags["Image Make"].values
        else:
            make = "unknown"
        return self._decode_make_model(make)

    def extract_model(self):
        if "EXIF LensModel" in self.tags:
            model = self.tags["EXIF LensModel"].values
        elif "Image Model" in self.tags:
            model = self.tags["Image Model"].values
        else:
            model = "unknown"
        return self._decode_make_model(model)

    def extract_projection_type(self):
        gpano = get_gpano_from_xmp(self.xmp)
        return gpano.get("GPano:ProjectionType", "perspective")

    def extract_focal(self):
        make, model = self.extract_make(), self.extract_model()
        focal_35, focal_ratio = compute_focal(
            get_tag_as_float(self.tags, "EXIF FocalLengthIn35mmFilm"),
            get_tag_as_float(self.tags, "EXIF FocalLength"),
            self.extract_sensor_width(),
            sensor_string(make, model),
        )
        return focal_35, focal_ratio

    def extract_sensor_width(self):
        """Compute sensor with from width and resolution."""
        if (
            "EXIF FocalPlaneResolutionUnit" not in self.tags
            or "EXIF FocalPlaneXResolution" not in self.tags
        ):
            return None
        resolution_unit = self.tags["EXIF FocalPlaneResolutionUnit"].values[0]
        mm_per_unit = self.get_mm_per_unit(resolution_unit)
        if not mm_per_unit:
            return None
        pixels_per_unit = get_tag_as_float(self.tags, "EXIF FocalPlaneXResolution")
        if pixels_per_unit <= 0:
            pixels_per_unit = get_tag_as_float(self.tags, "EXIF FocalPlaneYResolution")
            if pixels_per_unit <= 0:
                return None
        units_per_pixel = 1 / pixels_per_unit
        width_in_pixels = self.extract_image_size()[0]
        return width_in_pixels * units_per_pixel * mm_per_unit

    def get_mm_per_unit(self, resolution_unit):
        """Length of a resolution unit in millimeters.

        Uses the values from the EXIF specs in
        https://www.sno.phy.queensu.ca/~phil/exiftool/TagNames/EXIF.html

        Args:
            resolution_unit: the resolution unit value given in the EXIF
        """
        if resolution_unit == 2:  # inch
            return inch_in_mm
        elif resolution_unit == 3:  # cm
            return cm_in_mm
        elif resolution_unit == 4:  # mm
            return 1
        elif resolution_unit == 5:  # um
            return um_in_mm
        else:
            logger.warning(
                "Unknown EXIF resolution unit value: {}".format(resolution_unit)
            )
            return None

    def extract_orientation(self):
        orientation = 1
        if "Image Orientation" in self.tags:
            value = self.tags.get("Image Orientation").values[0]
            if type(value) == int:
                orientation = value
        return orientation

    def extract_ref_lon_lat(self):
        if "GPS GPSLatitudeRef" in self.tags:
            reflat = self.tags["GPS GPSLatitudeRef"].values
        else:
            reflat = "N"
        if "GPS GPSLongitudeRef" in self.tags:
            reflon = self.tags["GPS GPSLongitudeRef"].values
        else:
            reflon = "E"
        return reflon, reflat

    def extract_dji_lon_lat(self):
        lon = self.xmp[0]["@drone-dji:Longitude"]
        lat = self.xmp[0]["@drone-dji:Latitude"]
        lon_number = float(lon[1:])
        lat_number = float(lat[1:])
        lon_number = lon_number if lon[0] == "+" else -lon_number
        lat_number = lat_number if lat[0] == "+" else -lat_number
        return lon_number, lat_number

    def extract_dji_altitude(self):
        return float(self.xmp[0]["@drone-dji:AbsoluteAltitude"])

    def has_xmp(self):
        return len(self.xmp) > 0

    def has_dji_latlon(self):
        return (
            self.has_xmp()
            and "@drone-dji:Latitude" in self.xmp[0]
            and "@drone-dji:Longitude" in self.xmp[0]
        )

    def has_dji_altitude(self):
        return self.has_xmp() and "@drone-dji:AbsoluteAltitude" in self.xmp[0]

    def extract_lon_lat(self):
        if self.has_dji_latlon():
            lon, lat = self.extract_dji_lon_lat()
        elif "GPS GPSLatitude" in self.tags:
            reflon, reflat = self.extract_ref_lon_lat()
            lat = gps_to_decimal(self.tags["GPS GPSLatitude"].values, reflat)
            lon = gps_to_decimal(self.tags["GPS GPSLongitude"].values, reflon)
        else:
            lon, lat = None, None
        return lon, lat

    def extract_altitude(self):
        if self.has_dji_altitude():
            altitude = self.extract_dji_altitude()
        elif "GPS GPSAltitude" in self.tags:
            alt_value = self.tags["GPS GPSAltitude"].values[0]
            if isinstance(alt_value, exifread.utils.Ratio):
                altitude = eval_frac(alt_value)
            elif isinstance(alt_value, int):
                altitude = float(alt_value)
            else:
                altitude = None

            # Check if GPSAltitudeRef is equal to 1, which means GPSAltitude should be negative, reference: http://www.exif.org/Exif2-2.PDF#page=53
            if (
                "GPS GPSAltitudeRef" in self.tags
                and self.tags["GPS GPSAltitudeRef"].values[0] == 1
                and altitude is not None
            ):
                altitude = -altitude
        else:
            altitude = None
        return altitude

    def extract_dop(self):
        if "GPS GPSDOP" in self.tags:
            dop = eval_frac(self.tags["GPS GPSDOP"].values[0])
        else:
            dop = None
        return dop

    def extract_geo(self):
        altitude = self.extract_altitude()
        dop = self.extract_dop()
        lon, lat = self.extract_lon_lat()
        d = {}

        if lon is not None and lat is not None:
            d["latitude"] = lat
            d["longitude"] = lon
        if altitude is not None:
            d["altitude"] = min([maximum_altitude, altitude])
        if dop is not None:
            d["dop"] = dop
        return d

    def extract_capture_time(self):
        if (
            "GPS GPSDate" in self.tags
            and "GPS GPSTimeStamp" in self.tags  # Actually GPSDateStamp
        ):
            try:
                hours = int(get_tag_as_float(self.tags, "GPS GPSTimeStamp", 0))
                minutes = int(get_tag_as_float(self.tags, "GPS GPSTimeStamp", 1))
                seconds = get_tag_as_float(self.tags, "GPS GPSTimeStamp", 2)
                gps_timestamp_string = "{0:s} {1:02d}:{2:02d}:{3:02f}".format(
                    self.tags["GPS GPSDate"].values, hours, minutes, seconds
                )
                return (
                    datetime.datetime.strptime(
                        gps_timestamp_string, "%Y:%m:%d %H:%M:%S.%f"
                    )
                    - datetime.datetime(1970, 1, 1)
                ).total_seconds()
            except (TypeError, ValueError):
                logger.info(
                    'The GPS time stamp in image file "{0:s}" is invalid. '
                    "Falling back to DateTime*".format(self.fileobj_name)
                )

        time_strings = [
            ("EXIF DateTimeOriginal", "EXIF SubSecTimeOriginal", "EXIF Tag 0x9011"),
            ("EXIF DateTimeDigitized", "EXIF SubSecTimeDigitized", "EXIF Tag 0x9012"),
            ("Image DateTime", "Image SubSecTime", "Image Tag 0x9010"),
        ]
        for datetime_tag, subsec_tag, offset_tag in time_strings:
            if datetime_tag in self.tags:
                date_time = self.tags[datetime_tag].values
                if subsec_tag in self.tags:
                    subsec_time = self.tags[subsec_tag].values
                else:
                    subsec_time = "0"
                try:
                    s = "{0:s}.{1:s}".format(date_time, subsec_time)
                    d = datetime.datetime.strptime(s, "%Y:%m:%d %H:%M:%S.%f")
                except ValueError:
                    logger.debug(
                        'The "{1:s}" time stamp or "{2:s}" tag is invalid in '
                        'image file "{0:s}"'.format(
                            self.fileobj_name, datetime_tag, subsec_tag
                        )
                    )
                    continue
                # Test for OffsetTimeOriginal | OffsetTimeDigitized | OffsetTime
                if offset_tag in self.tags:
                    offset_time = self.tags[offset_tag].values
                    try:
                        d += datetime.timedelta(
                            hours=-int(offset_time[0:3]), minutes=int(offset_time[4:6])
                        )
                    except (TypeError, ValueError):
                        logger.debug(
                            'The "{0:s}" time zone offset in image file "{1:s}"'
                            " is invalid".format(offset_tag, self.fileobj_name)
                        )
                        logger.debug(
                            'Naively assuming UTC on "{0:s}" in image file '
                            '"{1:s}"'.format(datetime_tag, self.fileobj_name)
                        )
                else:
                    logger.debug(
                        "No GPS time stamp and no time zone offset in image "
                        'file "{0:s}"'.format(self.fileobj_name)
                    )
                    logger.debug(
                        'Naively assuming UTC on "{0:s}" in image file "{1:s}"'.format(
                            datetime_tag, self.fileobj_name
                        )
                    )
                return (d - datetime.datetime(1970, 1, 1)).total_seconds()
        logger.info(
            'Image file "{0:s}" has no valid time stamp'.format(self.fileobj_name)
        )
        return 0.0

    def extract_exif(self):
        width, height = self.extract_image_size()
        projection_type = self.extract_projection_type()
        focal_35, focal_ratio = self.extract_focal()
        make, model = self.extract_make(), self.extract_model()
        orientation = self.extract_orientation()
        geo = self.extract_geo()
        capture_time = self.extract_capture_time()
        d = {
            "make": make,
            "model": model,
            "width": width,
            "height": height,
            "projection_type": projection_type,
            "focal_ratio": focal_ratio,
            "orientation": orientation,
            "capture_time": capture_time,
            "gps": geo,
        }
        d["camera"] = camera_id(d)
        return d


def hard_coded_calibration(exif):
    focal = exif["focal_ratio"]
    fmm35 = int(round(focal * 36.0))
    make = exif["make"].strip().lower()
    model = exif["model"].strip().lower()
    if "gopro" in make:
        if fmm35 == 20:
            # GoPro Hero 3, 7MP medium
            return {"focal": focal, "k1": -0.37, "k2": 0.28}
        elif fmm35 == 15:
            # GoPro Hero 3, 7MP wide
            # "v2 gopro hero3+ black edition 3000 2250 perspective 0.4166"
            return {"focal": 0.466, "k1": -0.195, "k2": 0.030}
        elif fmm35 == 23:
            # GoPro Hero 2, 5MP medium
            return {"focal": focal, "k1": -0.38, "k2": 0.24}
        elif fmm35 == 16:
            # GoPro Hero 2, 5MP wide
            return {"focal": focal, "k1": -0.39, "k2": 0.22}
    elif "bullet5s" in make:
        return {"focal": 0.57, "k1": -0.30, "k2": 0.06}
    elif "garmin" == make:
        if "virb" == model:
            # "v2 garmin virb 4608 3456 perspective 0"
            return {"focal": 0.5, "k1": -0.08, "k2": 0.005}
        elif "virbxe" == model:
            # "v2 garmin virbxe 3477 1950 perspective 0.3888"
            # "v2 garmin virbxe 1600 1200 perspective 0.3888"
            # "v2 garmin virbxe 4000 3000 perspective 0.3888"
            # Calibration when using camera's undistortion
            return {"focal": 0.466, "k1": -0.08, "k2": 0.0}
            # Calibration when not using camera's undistortion
            # return {'focal': 0.466, 'k1': -0.195, 'k2'; 0.030}
    elif "drift" == make:
        if "ghost s" == model:
            return {"focal": 0.47, "k1": -0.22, "k2": 0.03}
    elif "xiaoyi" in make:
        return {"focal": 0.5, "k1": -0.19, "k2": 0.028}
    elif "geo" == make and "frames" == model:
        return {"focal": 0.5, "k1": -0.24, "k2": 0.04}
    elif "sony" == make:
        if "hdr-as200v" == model:
            return {"focal": 0.55, "k1": -0.30, "k2": 0.08}
        elif "hdr-as300" in model:
            return {"focal": 0.3958, "k1": -0.1496, "k2": 0.0201}
    elif "PARROT" == make:
        if "Bebop 2" == model:
            return {"focal": 0.36666666666666666}


def focal_ratio_calibration(exif):
    if exif.get("focal_ratio"):
        return {
            "focal": exif["focal_ratio"],
            "k1": 0.0,
            "k2": 0.0,
            "p1": 0.0,
            "p2": 0.0,
            "k3": 0.0,
        }


def focal_xy_calibration(exif):
    focal = exif.get("focal_x", exif.get("focal_ratio"))
    if focal:
        return {
            "focal_x": focal,
            "focal_y": focal,
            "c_x": exif.get("c_x", 0.0),
            "c_y": exif.get("c_y", 0.0),
            "k1": 0.0,
            "k2": 0.0,
            "p1": 0.0,
            "p2": 0.0,
            "k3": 0.0,
            "k4": 0.0,
            "k5": 0.0,
            "k6": 0.0,
        }


def default_calibration(data: DataSetBase):
    return {
        "focal": data.config["default_focal_prior"],
        "focal_x": data.config["default_focal_prior"],
        "focal_y": data.config["default_focal_prior"],
        "c_x": 0.0,
        "c_y": 0.0,
        "k1": 0.0,
        "k2": 0.0,
        "p1": 0.0,
        "p2": 0.0,
        "k3": 0.0,
        "k4": 0.0,
    }


def calibration_from_metadata(metadata, data: DataSetBase):
    """Finds the best calibration in one of the calibration sources."""
    pt = metadata.get("projection_type", default_projection).lower()
    if (
        pt == "brown"
        or pt == "fisheye_opencv"
        or pt == "radial"
        or pt == "simple_radial"
        or pt == "fisheye62"
    ):
        calib = (
            hard_coded_calibration(metadata)
            or focal_xy_calibration(metadata)
            or default_calibration(data)
        )
    else:
        calib = (
            hard_coded_calibration(metadata)
            or focal_ratio_calibration(metadata)
            or default_calibration(data)
        )
    calib["projection_type"] = pt
    return calib


def camera_from_exif_metadata(
    metadata, data: DataSetBase, calibration_func=calibration_from_metadata
):
    """
    Create a camera object from exif metadata and the calibration
    function that turns metadata into usable calibration parameters.
    """

    calib = calibration_func(metadata, data)
    calib_pt = calib.get("projection_type", default_projection).lower()

    camera = None
    if calib_pt == "perspective":
        camera = pygeometry.Camera.create_perspective(
            calib["focal"], calib["k1"], calib["k2"]
        )
    elif calib_pt == "brown":
        camera = pygeometry.Camera.create_brown(
            calib["focal_x"],
            calib["focal_y"] / calib["focal_x"],
            [calib["c_x"], calib["c_y"]],
            [calib["k1"], calib["k2"], calib["k3"], calib["p1"], calib["p2"]],
        )
    elif calib_pt == "fisheye":
        camera = pygeometry.Camera.create_fisheye(
            calib["focal"], calib["k1"], calib["k2"]
        )
    elif calib_pt == "fisheye_opencv":
        camera = pygeometry.Camera.create_fisheye_opencv(
            calib["focal_x"],
            calib["focal_y"] / calib["focal_x"],
            [calib["c_x"], calib["c_y"]],
            [calib["k1"], calib["k2"], calib["k3"], calib["k4"]],
        )
    elif calib_pt == "fisheye62":
        camera = pygeometry.Camera.create_fisheye62(
            calib["focal_x"],
            calib["focal_y"] / calib["focal_x"],
            [calib["c_x"], calib["c_y"]],
            [
                calib["k1"],
                calib["k2"],
                calib["k3"],
                calib["k4"],
                calib["k5"],
                calib["k6"],
                calib["p1"],
                calib["p2"],
            ],
        )
    elif calib_pt == "radial":
        camera = pygeometry.Camera.create_radial(
            calib["focal_x"],
            calib["focal_y"] / calib["focal_x"],
            [calib["c_x"], calib["c_y"]],
            [calib["k1"], calib["k2"]],
        )
    elif calib_pt == "simple_radial":
        camera = pygeometry.Camera.create_simple_radial(
            calib["focal_x"],
            calib["focal_y"] / calib["focal_x"],
            [calib["c_x"], calib["c_y"]],
            calib["k1"],
        )
    elif calib_pt == "dual":
        camera = pygeometry.Camera.create_dual(
            calib["transition"], calib["focal"], calib["k1"], calib["k2"]
        )
    elif pygeometry.Camera.is_panorama(calib_pt):
        camera = pygeometry.Camera.create_spherical()
    else:
        raise ValueError("Unknown projection type: {}".format(calib_pt))

    camera.id = metadata["camera"]
    camera.width = int(metadata["width"])
    camera.height = int(metadata["height"])
    return camera
