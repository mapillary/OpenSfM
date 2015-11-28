class Intrinsics(object):
    """Defines the internal parameters of a camera.

    Attributes:
        focal (real): focal lenght after reconstruction.
        focal_prior (real): focal lenght before reconstruction.
        height (int): image height.
        widht (int): image width.
        k1 (real): first distortion parameter.
        k2 (real): second distortion parameter.

    """

    def __init__(self):
        """Defaut constructor

        """
        self.focal = None
        self.focal_prior = None
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

    A camera is unique defined for its identification description (id),
    the type of projection (projection_type) and its internal calibration
    parameters (intrinsics).

    Attributes:
        id (str): camera description.
        projection_type (str): projection type.
        intrinsics (Intrinsics): extrinsic parameters.

    """

    def __init__(self):
        """Defaut constructor

        """
        self.id = None
        self.projection_type = None
        self.intrinsics = None


class Shot(object):
    """Defines a shot in a reconstructed scene.

    A shot here is refered as a unique view inside the scene defined by
    the image filename (id), the used camera with its refined internal
    parameters (camera), the fully camera pose respect to the scene origin
    frame (extrinsics) and the GPS data obtained in the moment that the picture
    was taken (gps_data).

    Attributes:
        id (str): picture filename.
        camera (Camera): camera.
        extrinsics (Extrinsics): extrinsic parameters.
        gps_data (GpsData): GPS data.

    """

    def __init__(self):
        """Defaut constructor

        """
        self.id = None
        self.camera = None
        self.extrinsics = None
        self.gps_data = None


class Point(object):
    """Defines a 3D point.

    Attributes:
        id (int): identification number.
        color (vector(int)): vector containing the RGB values.
        coordinates (vector(real)): vector containing the 3D position.
        reprojection_error (real): the reprojection error.

    """

    def __init__(self):
        """Defaut constructor

        """
        self.id = None
        self.color = None
        self.coordinates = None
        self.reprojection_error = None


class Reconstruction(object):
    """Defines the reconstructed scene.

    Attributes:
      cameras (Dict(Camera): List of cameras.
      shots   (Dict(Shot)): List of reconstructed shots.
      points  (Dict(Point)): List of reconstructed points.

    """

    def __init__(self):
        """Defaut constructor

        """
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