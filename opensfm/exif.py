import datetime
import exifread
import logging
import xmltodict as x2d

from opensfm.sensors import sensor_data
from opensfm import types


logger = logging.getLogger(__name__)


def eval_frac(value):
    return float(value.num) / float(value.den)


def gps_to_decimal(values, reference):
    sign = 1 if reference in 'NE' else -1
    degrees = eval_frac(values[0])
    minutes = eval_frac(values[1])
    seconds = eval_frac(values[2])
    return sign * (degrees + minutes / 60 + seconds / 3600)


def get_float_tag(tags, key):
    if key in tags:
        return float(tags[key].values[0])
    else:
        return None


def get_frac_tag(tags, key):
    if key in tags:
        try:
            return eval_frac(tags[key].values[0])
        except ZeroDivisionError:
            return None
    else:
        return None


def compute_focal(focal_35, focal, sensor_width, sensor_string):
    if focal_35 > 0:
        focal_ratio = focal_35 / 36.0  # 35mm film produces 36x24mm pictures.
    else:
        if not sensor_width:
            sensor_width = sensor_data.get(sensor_string, None)
        if sensor_width and focal:
            focal_ratio = focal / sensor_width
            focal_35 = 36.0 * focal_ratio
        else:
            focal_35 = 0
            focal_ratio = 0
    return focal_35, focal_ratio


def sensor_string(make, model):
    if make != 'unknown':
        # remove duplicate 'make' information in 'model'
        model = model.replace(make, '')
    return (make.strip() + ' ' + model.strip()).lower()


def camera_id(make, model, width, height, projection_type, focal):
    if make != 'unknown':
        # remove duplicate 'make' information in 'model'
        model = model.replace(make, '')
    return ' '.join([
        'v2',
        make.strip(),
        model.strip(),
        str(int(width)),
        str(int(height)),
        projection_type,
        str(focal)[:6],
    ]).lower()


def extract_exif_from_file(fileobj):
    if isinstance(fileobj, (str, unicode)):
        with open(fileobj) as f:
            exif_data = EXIF(f)
    else:
        exif_data = EXIF(fileobj)

    d = exif_data.extract_exif()
    return d


def get_xmp(fileobj):
    '''Extracts XMP metadata from and image fileobj
    '''
    img_str = str(fileobj.read())
    xmp_start = img_str.find('<x:xmpmeta')
    xmp_end = img_str.find('</x:xmpmeta')

    if xmp_start < xmp_end:
        xmp_str = img_str[xmp_start:xmp_end + 12]
        xdict = x2d.parse(xmp_str)
        xdict = xdict.get('x:xmpmeta', {})
        xdict = xdict.get('rdf:RDF', {})
        xdict = xdict.get('rdf:Description', {})
        if isinstance(xdict, list):
            return xdict
        else:
            return [xdict]
    else:
        return []


def get_gpano_from_xmp(xmp):
    for i in xmp:
        for k in i:
            if 'GPano' in k:
                return i
    return {}


class EXIF:

    def __init__(self, fileobj):
        self.tags = exifread.process_file(fileobj, details=False)
        fileobj.seek(0)
        self.xmp = get_xmp(fileobj)

    def extract_image_size(self):
        # Image Width and Image Height
        if ('EXIF ExifImageWidth' in self.tags and
                'EXIF ExifImageLength' in self.tags):
            width, height = (int(self.tags['EXIF ExifImageWidth'].values[0]),
                             int(self.tags['EXIF ExifImageLength'].values[0]))
        else:
            width, height = -1, -1
        return width, height

    def extract_make(self):
        # Camera make and model
        if 'EXIF LensMake' in self.tags:
            make = self.tags['EXIF LensMake'].values
        elif 'Image Make' in self.tags:
            make = self.tags['Image Make'].values
        else:
            make = 'unknown'
        try:
            return make.decode('utf-8')
        except UnicodeDecodeError:
            return 'unknown'

    def extract_model(self):
        if 'EXIF LensModel' in self.tags:
            model = self.tags['EXIF LensModel'].values
        elif 'Image Model' in self.tags:
            model = self.tags['Image Model'].values
        else:
            model = 'unknown'
        try:
            return model.decode('utf-8')
        except UnicodeDecodeError:
            return 'unknown'

    def extract_projection_type(self):
        gpano = get_gpano_from_xmp(self.xmp)
        return gpano.get('GPano:ProjectionType', 'perspective')

    def extract_focal(self):
        make, model = self.extract_make(), self.extract_model()
        focal_35, focal_ratio = compute_focal(
            get_float_tag(self.tags, 'EXIF FocalLengthIn35mmFilm'),
            get_frac_tag(self.tags, 'EXIF FocalLength'),
            get_frac_tag(self.tags, 'EXIF CCD width'),
            sensor_string(make, model))
        return focal_35, focal_ratio

    def extract_orientation(self):
        orientation = 1
        if 'Image Orientation' in self.tags:
            value = self.tags.get('Image Orientation').values[0]
            if type(value) == int:
                orientation = value
        return orientation

    def extract_ref_lon_lat(self):
        if 'GPS GPSLatitudeRef' in self.tags:
            reflat = self.tags['GPS GPSLatitudeRef'].values
        else:
            reflat = 'N'
        if 'GPS GPSLongitudeRef' in self.tags:
            reflon = self.tags['GPS GPSLongitudeRef'].values
        else:
            reflon = 'E'
        return reflon, reflat

    def extract_lon_lat(self):
        if 'GPS GPSLatitude' in self.tags:
            reflon, reflat = self.extract_ref_lon_lat()
            lat = gps_to_decimal(self.tags['GPS GPSLatitude'].values, reflat)
            lon = gps_to_decimal(self.tags['GPS GPSLongitude'].values, reflon)
        else:
            lon, lat = None, None
        return lon, lat

    def extract_altitude(self):
        if 'GPS GPSAltitude' in self.tags:
            altitude = eval_frac(self.tags['GPS GPSAltitude'].values[0])
        else:
            altitude = None
        return altitude

    def extract_dop(self):
        if 'GPS GPSDOP' in self.tags:
            dop = eval_frac(self.tags['GPS GPSDOP'].values[0])
        else:
            dop = None
        return dop

    def extract_geo(self):
        altitude = self.extract_altitude()
        dop = self.extract_dop()
        lon, lat = self.extract_lon_lat()
        d = {}

        if lon is not None and lat is not None:
            d['latitude'] = lat
            d['longitude'] = lon
        if altitude is not None:
            d['altitude'] = altitude
        if dop is not None:
            d['dop'] = dop
        return d

    def extract_capture_time(self):
        time_strings = [('EXIF DateTimeOriginal', 'EXIF SubSecTimeOriginal'),
                        ('EXIF DateTimeDigitized', 'EXIF SubSecTimeDigitized'),
                        ('Image DateTime', 'EXIF SubSecTime')]
        for ts in time_strings:
            if ts[0] in self.tags:
                s = str(self.tags[ts[0]].values)
                try:
                    d = datetime.datetime.strptime(s, '%Y:%m:%d %H:%M:%S')
                except ValueError:
                    continue
                timestamp = (d - datetime.datetime(1970, 1, 1)).total_seconds()   # Assuming d is in UTC
                timestamp += int(str(self.tags.get(ts[1], 0))) / 1000.0;
                return timestamp
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
            'camera': camera_id(make, model, width, height, projection_type, focal_ratio),
            'make': make,
            'model': model,
            'width': width,
            'height': height,
            'projection_type': projection_type,
            'focal_ratio': focal_ratio,
            'orientation': orientation,
            'capture_time': capture_time,
            'gps': geo
        }
        return d


def hard_coded_calibration(exif):
    focal = exif['focal_ratio']
    fmm35 = int(round(focal * 36.0))
    make = exif['make'].strip().lower()
    model = exif['model'].strip().lower()
    if 'gopro' in make:
        if fmm35 == 20:
            # GoPro Hero 3, 7MP medium
            return {'focal': focal, 'k1': -0.37, 'k2': 0.28}
        elif fmm35 == 15:
            # GoPro Hero 3, 7MP wide
            # "v2 gopro hero3+ black edition 3000 2250 perspective 0.4166"
            return {'focal': 0.466, 'k1': -0.195, 'k2': 0.030}
        elif fmm35 == 23:
            # GoPro Hero 2, 5MP medium
            return {'focal': focal, 'k1': -0.38, 'k2': 0.24}
        elif fmm35 == 16:
            # GoPro Hero 2, 5MP wide
            return {'focal': focal, 'k1': -0.39, 'k2': 0.22}
    elif 'bullet5s' in make:
        return {'focal': 0.57, 'k1': -0.30, 'k2': 0.06}
    elif 'garmin' == make:
        if 'virb' == model:
            # "v2 garmin virb 4608 3456 perspective 0"
            return {'focal': 0.5, 'k1': -0.08, 'k2': 0.005}
        elif 'virbxe' == model:
            # "v2 garmin virbxe 3477 1950 perspective 0.3888"
            # "v2 garmin virbxe 1600 1200 perspective 0.3888"
            # "v2 garmin virbxe 4000 3000 perspective 0.3888"
            # Calibration when using camera's undistortion
            return {'focal': 0.466, 'k1': -0.08, 'k2': 0.0}
            # Calibration when not using camera's undistortion
            # return {'focal': 0.466, 'k1': -0.195, 'k2'; 0.030}
    elif 'drift' == make:
        if 'ghost s' == model:
            return {'focal': 0.47, 'k1': -0.22, 'k2': 0.03}
    elif 'xiaoyi' in make:
        return {'focal': 0.5, 'k1': -0.19, 'k2': 0.028}
    elif 'geo' == make and 'frames' == model:
        return {'focal': 0.5, 'k1': -0.24, 'k2': 0.04}
    elif 'sony' == make and 'hdr-as200v' == model:
        return {'focal': 0.55, 'k1': -0.30, 'k2': 0.08}


def focal_ratio_calibration(exif):
    if exif['focal_ratio']:
        return {
            'focal': exif['focal_ratio'],
            'k1': 0.0,
            'k2': 0.0
        }


def default_calibration(data):
    return {
        'focal': data.config['default_focal_prior'],
        'k1': 0.0,
        'k2': 0.0
    }


def camera_from_exif_metadata(metadata, data):
    '''
    Create a camera object from exif metadata
    '''
    pt = metadata.get('projection_type', 'perspective')
    if pt == 'perspective':
        calib = (hard_coded_calibration(metadata)
                 or focal_ratio_calibration(metadata)
                 or default_calibration(data))
        camera = types.PerspectiveCamera()
        camera.id = metadata['camera']
        camera.width = metadata['width']
        camera.height = metadata['height']
        camera.projection_type = metadata.get('projection_type', 'perspective')
        camera.focal = calib['focal']
        camera.k1 = calib['k1']
        camera.k2 = calib['k2']
        camera.focal_prior = calib['focal']
        camera.k1_prior = calib['k1']
        camera.k2_prior = calib['k2']
        return camera
    elif pt in ['equirectangular', 'spherical']:
        camera = types.SphericalCamera()
        camera.id = metadata['camera']
        camera.width = metadata['width']
        camera.height = metadata['height']
        return camera
    else:
        raise ValueError("Unknown projection type: {}".format(pt))
