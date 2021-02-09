
===========================================
Notes on Multiple Reconstructions Alignment
===========================================


Merging
=======

We have a set of reconstructions.

Let

.. math::
  H_a = \begin{pmatrix} s_a R_a & t_a \\ 0 & 1 \end{pmatrix}

be the similarity transform that maps points in the global merged reference frame to the local reference frame of reconstruction :math:`a`.

Let

.. math::
  P_{ai} = (R_{ai}\ t_{ai})

be the projection matrix of camera :math:`i` in the reconstruction :math:`a`. And let

.. math::
  P_i = (R_i\ t_i)

be the projection matrix of camera :math:`i` in the global reference frame.



The relation between the local and global position of camera :math:`i` is

.. math::
  P_i \propto P_{ai} H_a

and thus

.. math::
  s_a (R_i\ t_i) = (R_{ai}\ t_{ai}) \begin{pmatrix} s_a R_a & t_a \\ 0 & 1 \end{pmatrix}

Solving for the observed values :math:`R_{ai}` and :math:`t_{ai}` gives

.. math::
  R_{ai} &=& R_i R_a^t \\
  t_{ai} &=& s_a t_i - R_i R_a^t t_a
  :label: relative_translation

If we want to get the best absolute projections given the relative ones, we can minimize

.. math::
  \left\|  \log(R_{ai} R_a R_i^t) \right\|^2_{\Sigma_{R_{ai}}} + \left\| t_{ai} - s_a t_i + R_i R_a^t t_a \right\|^2_{\Sigma_{t_{ai}}}

with respect to :math:`\{(R_a\ t_a)\}`.

Alternatively, we can work on the rotation and translation together and minimize

.. math::
  \left\|  \left(\log(R_{ai} R_a R_i^t) ,\  t_{ai} - s_a t_i + R_i R_a^t t_a \right) \right\|^2_{\Sigma_{Rt_{ai}}}



Aligning camera centers instead of translations
-----------------------------------------------

Aligning the translation vectors as done above has the problem that when rotations are not aligned the cameras may end up in different positions even if the translations vectors are the same.  An alternative approach that does not have such problem is to minimize the distance between the optical centers.

Let the optical center of camera :math:`i` be

.. math::
  o_i = -R_i^t t_i

and the optical center of camera :math:`i` in the reconstruction :math:`a` be

.. math::
  o_{ai} = -R_{ai}^t t_{ai}

We want them to align after applying :math:`H_a` so we have

.. math::
  s_a R_a o_i + t_a = o_{ai}

which we can enforce by minimizing

.. math::
  \left\| s_a R_a o_i + t_a - o_{ai} \right\|^2_{\Sigma_{o_{ai}}}

which it can be written in terms of Rs and ts as

.. math::
  \left\| R_{ai}^t t_{ai} - s_a R_a R_i^t t_i + t_a \right\|^2_{\Sigma_{o_{ai}}}




Camera position prior
---------------------

The camera center of camera :math:`i` is :math:`-R_i^t t_i`. If we want it to be close to the GPS position :math:`g_i` we can minimize

.. math::
  \left\| g_i + R_i^t t_i \right\|^2_{\Sigma_{g_i}}

If camera :math:`i` is not part of the optimization parameters we can add the same constraint in terms of :math:`H_a`

.. math::
  \left\| g_i + (R_{ai} R_a)^t (R_{ai} t_a + t_{ai}) / s_a \right\|^2_{\Sigma_{g_i}}


Common point constraint
-----------------------

When a point is present in more than one reconstruction we want the multiple reconstructions of the point to align.  There are several ways to do that.  One is to add a constraint for every pair of reconstructions.

Let :math:`p_{ai}` and :math:`p_{bi}` be point :math:`i` in reconstructions :math:`a` and :math:`b` respectively.

We want to align once mapped to the global reference frame.  That is

.. math::
   s_a^{-1} R_a^t (p_{ai} - t_a) = s_b^{-1} R_b^t (p_{bi} - t_b)

So we can minimize the difference

.. math::
   \|s_a^{-1} R_a^t (p_{ai} - t_a) - s_b^{-1} R_b^t (p_{bi} - t_b)\|_{\Sigma_p}
