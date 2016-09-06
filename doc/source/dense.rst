.. Notes and doc on dense matching

Dense Matching Notes
====================


Backprojection at a given depth
-------------------------------

The backprojection of a pixel :math:`q = (q_x, q_y, 1)^T` at depth :math:`d` in camera coordinates is

.. math::

   X = d K^{-1} q

Backprojection to a plane
-------------------------

The backprojection of a pixel :math:`q = (q_x, q_y, 1)^T` on to the plane :math:`\pi = (v^T, 1)` is

.. math::
   X = \frac{-K^{-1} q}{v^T K^{-1} q}

and has depth

.. math::
   d = \frac{-1}{v^T K^{-1} q}



Plane given point and normal
----------------------------

The plane

.. math::

   \pi = \left( \frac{-n}{n\cdot X}, 1 \right)

Contains the point :math:`X` and has normal :math:`n`


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

   \pi_w = \begin{pmatrix} R & t \\ 0 & 1 \end{pmatrix} \pi_c


Plane-induced homography
------------------------

Given a plane in camera coordinates :math:`\pi_c = (v^T 1)` the homography from image 1 to image 2 is given by

.. math::

   H = K_2 [R_2 R_1^T + (R_2 R_1^T t_1 - t_2) v^T] K_1^{-1}


