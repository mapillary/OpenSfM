
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



Camera position prior
---------------------

The camera center of camera :math:`i` is :math:`-R_i^t t_i`. If we want it to be close to the GPS position :math:`g_i` we can minimize

.. math::
  \left\| g_i + R_i^t t_i \right\|^2_{\Sigma_{g_i}}

If camera :math:`i` is not part of the optimization parameters we can add the same constraint in terms of :math:`H_a`

.. math::
  \left\| g_i + (R_{ai} R_a)^t (R_{ai} t_a + t_{ai}) / s_a \right\|^2_{\Sigma_{g_i}}


