class CameraType(object):
    """Defines a camera type.

    Attributes:
        id (int): identification number.
        projection_type (str): projection type.

    """

    def __init__(self, id, projection_type):
        """Defaut constructor

        :param id: The identification number.
        :param projection_type: The projection type.

        """
        self._id = id
        self._projection_type = projection_type

    @property
    def id(self):
        """Returns the identification number
        :return: The identification number.
        """
        return self._id

    @property
    def projection_type(self):
        """Returns the projection type
        :return: The projection type.
        """
        return self._projection_type


class Shot(object):
    """Defines a shot in a reconstructed scene.

    Attributes:
        id (int): identification number.
        camera_type (CameraType): camera type.
        rotation (array): rotation vector
        translation (array): translation vector

    """

    def __init__(self, id, camera_type, rotation, translation):
        """Defaut constructor

        :param id: The identification number.
        :param camera_type: The camera type.
        :param rotation: The rotation vector.
        :param translation: The translation vecto.

        """
        self._id = id
        self._camera_type = camera_type
        self._rotation = rotation
        self._translation = translation

    @property
    def id(self):
        """Returns the identification number
        :return: The identification number.
        """
        return self._id

    @property
    def camera_type(self):
        """Returns the projection type
        :return: The projection type.
        """
        return self._camera_type

    @property
    def rotation(self):
        """Returns the rotation vector
        :return: The rotation vector.
        """
        return self._rotation

    @property
    def translation(self):
        """Returns the translation vector
        :return: The translation vector.
        """
        return self._translation


class Point(object):
    """Defines a 3D point.

    Attributes:
        id (int): identification number.
        x  (real): coordinate along X axis.
        y  (real): coordinate along Y axis.
        z  (real): coordinate along Z axis.

    """

    def __init__(self, id, x, y, z):
      self._id = id
      self._x = x
      self._y = y
      self._z = z

    @property
    def id(self):
        """Returns the identification number
        :return: The identification number.
        """
        return self._id

    def getCoordinates():
        """Returns the coordinates
        :return: The coordinates in array format
        """
        return [self._x, self._y, self._z]


class Reconstruction(object):
    """Defines the reconstructed scene.

    Attributes:
      camera_types (List(CameraType)): List of camera types.
      shots   (List(Shot)): List of reconstructed shots.
      points  (List(Point)): List of reconstructed points.

    """

    def __init__(self):
      self._camera_types = []
      self._shots = []
      self._points = []

    @property
    def camera_types(self):
        """Returns the camera types
        :return: The list with camera types.
        """
        return self._camera_types

    @property
    def shots(self):
        """Returns the shots
        :return: The list with shots.
        """
        return self._shots

    @property
    def points(self):
        """Returns the points
        :return: The list with points.
        """
        return self._points

    def add_camera_type(self, camera_type):
        """Adds a camera type in the list

        :param camera_type: The camera_type.

        """
        self.camera_types.append(camera_type)

    def get_camera_type(self, id):
        """Returns a camera type by id.

        :return: If exists the camera type, otherwise None.

        """
        for item in self.camera_types:
            return item if item.id == id else None

    def add_shot(self, shot):
        """Adds a shot in the list

        :param shot: The shot.

        """
        self._shots.append(shot)

    def get_shot(self, id):
        """Returns a shot by id.

        :return: If exists the shot, otherwise None.

        """
        for item in self._shots:
            return item if item.id == id else None

    def add_point(self, point):
        """Adds a point in the list

        :param point: The point.

        """
        self._points.append(point)

    def get_point(self, id):
        """Returns a point by id.

        :return: If exists the point, otherwise None.

        """
        for item in self._points:
            return item if item.id == id else None