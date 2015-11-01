class Intrinsics(object):
    """Defines the internal parameters of a camera.

    Attributes:
        fx (real): focal lenght along the X-axis after reconstruction.
        fy (real): focal lenght along the Y-axis after reconstruction.
        fx_prior (real): focal lenght along the X-axis before reconstruction.
        fy_prior (real): focal lenght along the Y-axis before reconstruction.
        cx (int): projection center along the X-axis.
        cy (int): projection center along the Y-axis.
        height (int): image height.
        widht (int): image width.
        k1 (real): first distortion parameter.
        k2 (real): second distortion parameter.

    """

    def __init__(self):
        self.fx = None
        self.fy = None
        self.fx_prior = None
        self.fy_prior = None
        self.cx = None
        self.cy = None
        self.height = None
        self.width = None
        self.k1 = None
        self.k2 = None


class Extrinsics(object):
    """Defines the extrinsics parameters of a camera.

    The extrinsic parameters are defined by a 3x1 rotation vector which
    maps the camera rotation respect to the origin frame (rotation) and
    a 3x1 translation vector which maps the camera translation respect
    to the origin frame (translation).

    Attributes:
        rotation (vector): the rotation vector.
        translation (vector): the rotation vector.

    """

    def __init__(self):
      self.rotation = None
      self.translation = None


class GpsData(object):
    """Defines GPS data from a taken picture.

    Attributes:
        orientation (int): the orientation.
        capture_time (real): the capture time.
        gps_dop (real): the GPS dop.
        gps_position (vector): the GPS position.

    """
    def __init__(self):
      self.orientation = None
      self.capture_time = None
      self.gps_dop = None
      self.gps_position = None


class Camera(object):
    """Defines a physical camera.

    A camera is unique defined for its identification number (id), an
    optional description such brand name (description), the type of
    projection (projection_type) and its internal calibration parameters
    (intrinsics).

    Attributes:
        id (int): identification number.
        description (str): description.
        projection_type (str): projection type.
        intrinsics (Intrinsics): extrinsic parameters.

    """

    def __init__(self, id, projection_type, intrinsics):
        """Defaut constructor

        :param id: The identification number.
        :param description: The camera description.
        :param projection_type: The projection type.
        :param intrinsics: The intrinsic parameters.

        """
        self.id = id
        self.description = None
        self.projection_type = projection_type
        self.intrinsics = intrinsics


class Shot(object):
    """Defines a shot in a reconstructed scene.

    A shot here is refered as a unique view inside the scene containing its
    own identification number (id), the used camera with its refined internal
    parameters (camera), the fully camera pose respect to the scene origin
    frame (extrinsics) and the GPS data obtained in the moment that the picture
    was taken (gps_data).

    Attributes:
        id (int): identification number.
        camera (Camera): camera.
        extrinsics (Extrinsics): extrinsic parameters.
        gps_data (GpsData): GPS data.

    """

    def __init__(self, id, camera, extrinsics, gps_data):
        """Defaut constructor

        :param id: The identification number.
        :param camera: The camera.
        :param extrinsics: The extrinsic parameters.
        :param gps_data: The GPS data.

        """
        self.id = id
        self.camera = camera
        self.extrinsics = extrinsics
        self.gps_data = gps_data


class Point(object):
    """Defines a 3D point.

    Attributes:
        id (int): identification number.
        x  (real): coordinate along the X-axis.
        y  (real): coordinate along the Y-axis.
        z  (real): coordinate along the Z-axis.

    """

    def __init__(self, id, x, y, z):
      self.id = id
      self.x = x
      self.y = y
      self.z = z

    def getCoordinates():
        """Returns the coordinates
        :return: The coordinates in array format
        """
        return [self.x, self.y, self.z]


class Reconstruction(object):
    """Defines the reconstructed scene.

    Attributes:
      cameras (Dict(Camera): List of cameras.
      shots   (Dict(Shot)): List of reconstructed shots.
      points  (Dict(Point)): List of reconstructed points.

    """

    def __init__(self):
      self.cameras = {}
      self.shots = {}
      self.points = {}

    def add_camera(self, camera):
        """Adds a camera in the list

        :param camera: The camera.

        """
        self.cameras[camera.id] = camera

    def get_camera(self, id):
        """Returns a camera by id.

        :return: If exists returns the camera, otherwise None.

        """
        return self.cameras.get(id)

    def add_shot(self, shot):
        """Adds a shot in the list

        :param shot: The shot.

        """
        self.shots[shot.id] = shot

    def get_shot(self, id):
        """Returns a shot by id.

        :return: If exists returns the shot, otherwise None.

        """
        return self.shots.get(id)

    def add_point(self, point):
        """Adds a point in the list

        :param point: The point.

        """
        self.points[point.id] = point

    def get_point(self, id):
        """Returns a point by id.

        :return: If exists returns the point, otherwise None.

        """
        return self.points.get(id)