
Camera Coordinate System and Conventions
========================================

Camera
------

The pose of a camera, conceptually, consists of two things:

1. Which direction does it face in, i.e. its local coordinate axes
2. Where is it, i.e. the position of the camera origin

Local coordinate system of camera
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
These online docs say that, from the POV of a camera (a.k.a. a ``Shot``
object):

-  The z-axis points **forward**
-  The y-axis points **down**
-  The x-axis points to the **right**

In the 3D reconstruction viewer, the axes go Red, Green, Blue: x, y, z.

|id-rotation|

The OpenSfM ``Pose`` class contains a ``rotation`` field, representing
the local coordinate system as an **axis-angle vector**.

-  The **direction** of this 3D vector represents the **axis** around
   which to rotate.
-  The **length** of this vector is the **angle** to rotate around said
   axis. It is in radians.

Camera position
~~~~~~~~~~~~~~~~

To get and/or set the actual the actual camera position in world coordinate, one has to use ``get_origin()``/``set_origin()``.


.. |id-rotation| image:: images/id-rotation.png
