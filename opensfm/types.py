class Intrinsics(object):
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
    def __init__(self):
      self.rotation = None
      self.translation = None


class GpsData(object):
    def __init__(self):
      self.orientation = None
      self.capture_time = None
      self.gps_dop = None
      self.gps_position = None


class Camera(object):
    """Defines a physical camera.

    Attributes:
        id (int): identification number.
        projection_type (str): projection type.

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

    Attributes:
        id (int): identification number.
        camera (Camera): camera.
        extrinsics (Extrinsics): extrinsic parameters
        gps_data (GpsData): GPS data

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
        x  (real): coordinate along X axis.
        y  (real): coordinate along Y axis.
        z  (real): coordinate along Z axis.

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
      camera_types (List(CameraType)): List of camera types.
      shots   (List(Shot)): List of reconstructed shots.
      points  (List(Point)): List of reconstructed points.

    """

    def __init__(self):
      self.cameras = []
      self.shots = []
      self.points = []

    def add_camera(self, camera):
        """Adds a camera in the list

        :param camera: The camera.

        """
        self.cameras.append(camera)

    def get_camera(self, id):
        """Returns a camera by id.

        :return: If exists the camera, otherwise None.

        """
        for item in self.cameras:
            return item if item.id == id else None

    def add_shot(self, shot):
        """Adds a shot in the list

        :param shot: The shot.

        """
        self.shots.append(shot)

    def get_shot(self, id):
        """Returns a shot by id.

        :return: If exists the shot, otherwise None.

        """
        for item in self.shots:
            return item if item.id == id else None

    def add_point(self, point):
        """Adds a point in the list

        :param point: The point.

        """
        self.points.append(point)

    def get_point(self, id):
        """Returns a point by id.

        :return: If exists the point, otherwise None.

        """
        for item in self.points:
            return item if item.id == id else None