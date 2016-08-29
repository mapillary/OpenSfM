.. Notes and doc on dense matching

Dense Matching Notes
====================


Plane of constant depth
-----------------------

A plane of constant depth :math:`d` is defined by :math:`z = d` in camera coordinates.
So it has de following coordinates

.. math::

   \pi_c = (0, 0, -1 / d, 1)


Plane coordinates conversion
----------------------------

The coordinates of a plane in world and camera coordinates are related by

.. math::

   \pi_c = \begin{pmatrix} R & t \\ 0 & 1 \end{pmatrix} \pi_c


Plane-induced homography
------------------------

Given a plane in camera coordinates :math:`\pi_c = (v^T 1)` the homography from image 1 to image 2 is given by

.. math::

   H = K_2 [R_2 R_1^T + (R_2 R_1^T t_1 - t_2) v^T] K_1^{-1}


