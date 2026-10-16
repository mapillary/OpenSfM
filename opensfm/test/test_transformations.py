import math

import numpy as np
import pytest
from opensfm import transformations as tf


RNG_SEED = 42


def _euler_123() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    return np.array(1.0), np.array(2.0), np.array(3.0)


def _rng() -> np.random.Generator:
    return np.random.default_rng(RNG_SEED)


def _homogeneous_points(rng: np.random.Generator, n: int = 20) -> np.ndarray:
    points = (rng.random((4, n)) - 0.5) * 20
    points[3] = 1.0
    return points


def test_translation_matrix_roundtrip() -> None:
    direction = np.array([1.0, -2.0, 3.5])
    M = tf.translation_matrix(direction)
    assert np.allclose(M[:3, :3], np.identity(3))
    assert np.allclose(M @ [0.0, 0.0, 0.0, 1.0], [1.0, -2.0, 3.5, 1.0])
    assert np.allclose(tf.translation_from_matrix(M), direction)


def test_reflection_matrix_mirrors_across_plane() -> None:
    # Plane x = 2 with normal along x.
    R = tf.reflection_matrix(np.array([2.0, 0.0, 0.0, 1.0]), np.array([3.0, 0, 0]))
    assert np.allclose(R @ [5.0, 1.0, -1.0, 1.0], [-1.0, 1.0, -1.0, 1.0])
    assert np.allclose(R @ [2.0, 7.0, 4.0, 1.0], [2.0, 7.0, 4.0, 1.0])
    assert np.isclose(np.trace(R), 2.0)


def test_reflection_from_matrix_recovers_plane() -> None:
    rng = _rng()
    point = rng.random(3) - 0.5
    normal = rng.random(3) - 0.5
    M0 = tf.reflection_matrix(point, normal)
    p, n = tf.reflection_from_matrix(M0)
    assert tf.is_same_transform(M0, tf.reflection_matrix(p, n))


def test_reflection_from_matrix_rejects_non_reflection() -> None:
    with pytest.raises(ValueError, match="eigenvalue -1"):
        tf.reflection_from_matrix(np.identity(4))


def test_rotation_matrix_about_offset_axis() -> None:
    R = tf.rotation_matrix(math.pi / 2, np.array([0, 0, 1]), np.array([1, 0, 0]))
    assert np.allclose(R @ [0, 0, 0, 1], [1, -1, 0, 1])
    assert np.allclose(np.linalg.det(R[:3, :3]), 1.0)


def test_rotation_matrix_full_turn_is_identity() -> None:
    R = tf.rotation_matrix(2 * math.pi, np.array([0.3, -0.2, 0.9]))
    assert np.allclose(R, np.identity(4))


def test_rotation_matrix_negated_angle_and_axis_is_same() -> None:
    direction = np.array([0.1, 0.4, -0.3])
    point = np.array([0.5, -0.5, 0.2])
    R0 = tf.rotation_matrix(0.7, direction, point)
    R1 = tf.rotation_matrix(-0.7, -direction, point)
    assert tf.is_same_transform(R0, R1)


@pytest.mark.parametrize(
    "direction",
    [
        np.array([0.2, 0.3, 0.9]),  # dominant z component
        np.array([0.3, 0.9, 0.0]),  # z == 0, y != 0
        np.array([1.0, 0.0, 0.0]),  # only x
    ],
)
def test_rotation_from_matrix_roundtrip(direction: np.ndarray) -> None:
    point = np.array([0.1, -0.2, 0.3])
    R0 = tf.rotation_matrix(1.1, direction, point)
    angle, d, p = tf.rotation_from_matrix(R0)
    assert tf.is_same_transform(R0, tf.rotation_matrix(angle, d, p))
    assert np.isclose(abs(angle), 1.1)


def test_rotation_from_matrix_rejects_non_rotation() -> None:
    with pytest.raises(ValueError, match="eigenvalue 1"):
        tf.rotation_from_matrix(np.diag([2.0, 2.0, 2.0, 2.0]))


def test_scale_matrix_uniform_and_origin() -> None:
    points = _homogeneous_points(_rng(), 5)
    S = tf.scale_matrix(np.array(-1.234))
    assert np.allclose((S @ points)[:3], -1.234 * points[:3])

    origin = np.array([1.0, 2.0, 3.0])
    S = tf.scale_matrix(np.array(2.0), origin)
    assert np.allclose(S @ [1.0, 2.0, 3.0, 1.0], [1.0, 2.0, 3.0, 1.0])
    assert np.allclose(S @ [2.0, 2.0, 3.0, 1.0], [3.0, 2.0, 3.0, 1.0])


def test_scale_matrix_directional_only_scales_along_direction() -> None:
    S = tf.scale_matrix(np.array(3.0), None, np.array([0.0, 0.0, 2.0]))
    assert np.allclose(S @ [1.0, 2.0, 1.0, 1.0], [1.0, 2.0, 3.0, 1.0])


@pytest.mark.parametrize("directional", [False, True])
def test_scale_from_matrix_roundtrip(directional: bool) -> None:
    origin = np.array([0.1, 0.2, -0.3])
    direction = np.array([0.3, -0.1, 0.5]) if directional else None
    S0 = tf.scale_matrix(np.array(2.5), origin, direction)
    factor, o, d = tf.scale_from_matrix(S0)
    assert np.isclose(factor, 2.5)
    assert (d is None) == (not directional)
    assert tf.is_same_transform(S0, tf.scale_matrix(factor, o, d))


def test_projection_matrix_orthogonal_on_x_plane() -> None:
    P = tf.projection_matrix(np.array([0, 0, 0]), np.array([1, 0, 0]))
    assert np.allclose(P[1:, 1:], np.identity(4)[1:, 1:])
    assert np.allclose(P @ [5.0, 1.0, 2.0, 1.0], [0.0, 1.0, 2.0, 1.0])


def test_projection_matrix_parallel_projection() -> None:
    P = tf.projection_matrix(
        np.array([3, 0, 0]), np.array([1, 1, 0]), np.array([1, 0, 0])
    )
    v0 = _homogeneous_points(_rng(), 5)
    v1 = P @ v0
    assert np.allclose(v1[1], v0[1])
    assert np.allclose(v1[0], 3 - v1[1])


def test_projection_matrix_pseudo_perspective_composition() -> None:
    point = np.array([0.1, 0.2, 0.3])
    normal = np.array([0.4, -0.2, 0.1])
    persp = np.array([-0.3, 0.2, 0.4])
    P0 = tf.projection_matrix(point, normal)
    P2 = tf.projection_matrix(point, normal, perspective=persp)
    P3 = tf.projection_matrix(point, normal, perspective=persp, pseudo=True)
    assert tf.is_same_transform(P2, P0 @ P3)


_PERSPECTIVE = np.array([-0.3, 0.2, 0.4])


@pytest.mark.parametrize(
    "direction, perspective, pseudo",
    [
        (None, None, False),
        (np.array([0.3, 0.1, -0.4]), None, False),
        (None, _PERSPECTIVE, False),
        (None, _PERSPECTIVE, True),
    ],
)
def test_projection_from_matrix_roundtrip(
    direction: np.ndarray | None, perspective: np.ndarray | None, pseudo: bool
) -> None:
    point = np.array([0.1, 0.2, 0.3])
    normal = np.array([0.4, -0.2, 0.1])
    P0 = tf.projection_matrix(point, normal, direction, perspective, pseudo)
    result = tf.projection_from_matrix(P0, pseudo=pseudo)
    assert tf.is_same_transform(P0, tf.projection_matrix(*result))


def test_clip_matrix_maps_frustum_corners_to_ndc() -> None:
    left, right, bottom, top, near, far = (
        np.array(v) for v in (0.1, 0.5, 0.2, 0.9, 0.3, 1.2)
    )
    M = tf.clip_matrix(left, right, bottom, top, near, far, perspective=False)
    assert np.allclose(M @ [left, bottom, near, 1], [-1, -1, -1, 1])
    assert np.allclose(M @ [right, top, far, 1], [1, 1, 1, 1])

    M = tf.clip_matrix(left, right, bottom, top, near, far, perspective=True)
    v = M @ [left, bottom, near, 1]
    assert np.allclose(v / v[3], [-1, -1, -1, 1])
    v = M @ [right, top, near, 1]
    assert np.allclose(v / v[3], [1, 1, -1, 1])


@pytest.mark.parametrize(
    "frustum, perspective, message",
    [
        ((1.0, 0.0, 0.0, 1.0, 0.1, 1.0), False, "invalid frustum"),
        ((0.0, 1.0, 1.0, 0.0, 0.1, 1.0), False, "invalid frustum"),
        ((0.0, 1.0, 0.0, 1.0, 1.0, 0.1), False, "invalid frustum"),
        ((0.0, 1.0, 0.0, 1.0, 0.0, 1.0), True, "near <= 0"),
    ],
)
def test_clip_matrix_invalid_frustum(
    frustum: tuple[float, ...], perspective: bool, message: str
) -> None:
    with pytest.raises(ValueError, match=message):
        tf.clip_matrix(*(np.array(v) for v in frustum), perspective=perspective)


def test_shear_matrix_preserves_volume_and_plane() -> None:
    direction = np.array([1.0, 0.0, 0.0])
    normal = np.array([0.0, 0.0, 1.0])
    point = np.array([0.0, 0.0, 0.0])
    S = tf.shear_matrix(math.pi / 4, direction, point, normal)
    assert np.isclose(np.linalg.det(S), 1.0)
    # Points on the shear plane do not move; others shift along direction.
    assert np.allclose(S @ [1.0, 2.0, 0.0, 1.0], [1.0, 2.0, 0.0, 1.0])
    assert np.allclose(S @ [0.0, 0.0, 2.0, 1.0], [2.0, 0.0, 2.0, 1.0])


def test_shear_matrix_rejects_non_orthogonal_direction() -> None:
    with pytest.raises(ValueError, match="not orthogonal"):
        tf.shear_matrix(
            0.5, np.array([1.0, 0, 1.0]), np.zeros(3), np.array([0, 0, 1.0])
        )


def test_shear_from_matrix_roundtrip() -> None:
    direction = np.array([1.0, 0.0, 0.0])
    normal = np.array([0.0, 0.0, 1.0])
    S0 = tf.shear_matrix(0.3, direction, np.zeros(3), normal)
    angle, d, p, n = tf.shear_from_matrix(S0)
    assert np.isclose(abs(angle), 0.3)
    assert np.isclose(abs(np.dot(d, direction)), 1.0)
    assert np.isclose(abs(np.dot(n, normal)), 1.0)
    assert np.isclose(p[3], 1.0)
    assert tf.is_same_transform(S0, tf.shear_matrix(angle, d, p, n))


def test_shear_from_matrix_rejects_non_shear() -> None:
    with pytest.raises(ValueError, match="linear independent"):
        tf.shear_from_matrix(np.diag([2.0, 3.0, 4.0, 1.0]))


def test_decompose_matrix_components() -> None:
    _, _, _, trans, _ = tf.decompose_matrix(tf.translation_matrix(np.array([1, 2, 3])))
    assert np.allclose(trans, [1, 2, 3])

    scale, _, _, _, _ = tf.decompose_matrix(tf.scale_matrix(np.array(0.123)))
    assert np.allclose(scale, [0.123] * 3)

    R0 = tf.euler_matrix(1, 2, 3)
    _, _, angles, _, _ = tf.decompose_matrix(R0)
    assert np.allclose(R0, tf.euler_matrix(*angles))


def test_decompose_matrix_negative_determinant_flips_scale() -> None:
    scale, _, _, _, _ = tf.decompose_matrix(np.diag([-2.0, 3.0, 4.0, 1.0]))
    assert np.allclose(scale, [-2.0, -3.0, -4.0])


def test_decompose_compose_roundtrip_with_perspective() -> None:
    scale = np.array([0.4, -0.3, 0.2])
    shear = np.array([0.1, -0.2, 0.3])
    angles = np.array([0.5, -1.0, 2.0])
    trans = np.array([0.3, 0.1, -0.2])
    persp = np.array([0.1, -0.2, 0.3, 0.4])
    M0 = tf.compose_matrix(scale, shear, angles, trans, persp)
    M1 = tf.compose_matrix(*tf.decompose_matrix(M0))
    assert tf.is_same_transform(M0, M1)


def test_decompose_matrix_rejects_degenerate() -> None:
    with pytest.raises(ValueError, match="zero"):
        tf.decompose_matrix(np.zeros((4, 4)))
    with pytest.raises(ValueError, match="singular"):
        tf.decompose_matrix(np.diag([1.0, 0.0, 1.0, 1.0]))


def test_compose_matrix_default_is_identity() -> None:
    assert np.allclose(tf.compose_matrix(), np.identity(4))


def test_orthogonalization_matrix() -> None:
    ortho = tf.orthogonalization_matrix(np.array([10, 10, 10]), np.array([90, 90, 90]))
    assert np.allclose(ortho[:3, :3], np.identity(3) * 10)
    ortho = tf.orthogonalization_matrix(
        np.array([9.8, 12.0, 15.5]), np.array([87.2, 80.7, 69.7])
    )
    assert np.isclose(np.sum(ortho), 43.063229)


def test_affine_matrix_from_points_known_values() -> None:
    v0 = np.array([[0, 1031, 1031, 0], [0, 0, 1600, 1600]])
    v1 = np.array([[675, 826, 826, 677], [55, 52, 281, 277]])
    M = tf.affine_matrix_from_points(v0, v1)
    expected = np.array(
        [
            [0.14549, 0.00062, 675.50008],
            [0.00048, 0.14094, 53.24971],
            [0.0, 0.0, 1.0],
        ]
    )
    assert np.allclose(M, expected, atol=1e-4)


@pytest.mark.parametrize(
    "shear, scale, usesvd",
    [
        (True, True, True),
        (False, True, True),
        (False, True, False),
    ],
)
def test_affine_matrix_from_points_recovers_similarity(
    shear: bool, scale: bool, usesvd: bool
) -> None:
    rng = _rng()
    M = (
        tf.translation_matrix(np.array([0.3, -0.2, 0.1]))
        @ tf.random_rotation_matrix(rng.random(3))
        @ tf.scale_matrix(np.array(0.7))
    )
    v0 = _homogeneous_points(rng, 50)
    v1 = M @ v0
    M_est = tf.affine_matrix_from_points(
        v0[:3], v1[:3], shear=shear, scale=scale, usesvd=usesvd
    )
    assert np.allclose(M_est @ v0, v1)


def test_affine_matrix_from_points_rigid_handles_reflection() -> None:
    # Mirror image point sets force the SVD solution to fix up a det < 0.
    v0 = np.array([[0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    v1 = v0.copy()
    v1[0] *= -1
    M = tf.affine_matrix_from_points(v0, v1, shear=False, scale=False)
    assert np.isclose(np.linalg.det(M[:3, :3]), 1.0)


def test_affine_matrix_from_points_rejects_bad_shapes() -> None:
    with pytest.raises(ValueError, match="wrong shape"):
        tf.affine_matrix_from_points(np.zeros((3, 4)), np.zeros((3, 5)))
    with pytest.raises(ValueError, match="wrong shape"):
        tf.affine_matrix_from_points(np.zeros((3, 2)), np.zeros((3, 2)))


def test_superimposition_matrix() -> None:
    rng = _rng()
    v0 = rng.random((3, 10))
    assert np.allclose(tf.superimposition_matrix(v0, v0), np.identity(4))

    R = tf.random_rotation_matrix(rng.random(3))
    v0 = _homogeneous_points(rng, 30)
    v1 = R @ v0
    assert np.allclose(tf.superimposition_matrix(v0, v1) @ v0, v1)

    M = (
        tf.translation_matrix(np.array([1.0, 2, 3]))
        @ R
        @ tf.scale_matrix(np.array(1.5))
    )
    v1 = M @ v0
    S = tf.superimposition_matrix(v0, v1, scale=True, usesvd=False)
    assert np.allclose(S @ v0, v1)


def test_euler_matrix_known_values() -> None:
    R = tf.euler_matrix(1, 2, 3, "syxz")
    assert np.isclose(np.sum(R[0]), -1.34786452)
    R = tf.euler_matrix(1, 2, 3, (0, 1, 0, 1))
    assert np.isclose(np.sum(R[0]), -0.383436184)


def test_euler_matrix_rejects_unknown_axes_tuple() -> None:
    with pytest.raises(KeyError):
        tf.euler_matrix(1, 2, 3, (9, 9, 9, 9))


@pytest.mark.parametrize("axes", sorted(tf._AXES2TUPLE.keys()))
def test_euler_matrix_from_matrix_roundtrip_all_axes(axes: str) -> None:
    angles = (0.4, -1.1, 2.3)
    R0 = tf.euler_matrix(*angles, axes=axes)
    assert np.allclose(R0[:3, :3] @ R0[:3, :3].T, np.identity(3))
    R1 = tf.euler_matrix(*tf.euler_from_matrix(R0, axes), axes=axes)
    assert np.allclose(R0, R1)
    # The tuple encoding of the axes must give the same matrix.
    assert np.allclose(R0, tf.euler_matrix(*angles, axes=tf._AXES2TUPLE[axes]))


@pytest.mark.parametrize("axes", ["sxyz", "sxyx"])
def test_euler_from_matrix_gimbal_lock(axes: str) -> None:
    # Pitch at pi/2 (or 0 for repeated axes) hits the degenerate branch.
    aj = math.pi / 2 if axes == "sxyz" else 0.0
    R0 = tf.euler_matrix(0.3, aj, 0.0, axes)
    R1 = tf.euler_matrix(*tf.euler_from_matrix(R0, axes), axes=axes)
    assert np.allclose(R0, R1)


def test_euler_from_matrix_accepts_tuple_axes() -> None:
    R0 = tf.euler_matrix(0.1, 0.2, 0.3, "rzyx")
    angles = tf.euler_from_matrix(R0, tf._AXES2TUPLE["rzyx"])
    assert np.allclose(angles, (0.1, 0.2, 0.3))


def test_euler_from_quaternion() -> None:
    angles = tf.euler_from_quaternion(np.array([0.99810947, 0.06146124, 0, 0]))
    assert np.allclose(angles, [0.123, 0, 0])


@pytest.mark.parametrize("axes", sorted(tf._AXES2TUPLE.keys()))
def test_quaternion_from_euler_matches_euler_matrix(axes: str) -> None:
    angles = (0.4, -1.1, 2.3)
    q = tf.quaternion_from_euler(*(np.array(a) for a in angles), axes=axes)
    assert tf.is_same_transform(tf.quaternion_matrix(q), tf.euler_matrix(*angles, axes))


def test_quaternion_from_euler_known_value_and_tuple_axes() -> None:
    q = tf.quaternion_from_euler(*_euler_123(), "ryxz")
    assert np.allclose(q, [0.435953, 0.310622, -0.718287, 0.444435])
    q_tuple = tf.quaternion_from_euler(*_euler_123(), tf._AXES2TUPLE["ryxz"])
    assert np.allclose(q, q_tuple)


def test_quaternion_about_axis() -> None:
    q = tf.quaternion_about_axis(np.array(0.123), np.array([1, 0, 0]))
    assert np.allclose(q, [0.99810947, 0.06146124, 0, 0])
    # Zero axis yields a pure-real quaternion.
    assert np.allclose(
        tf.quaternion_about_axis(np.array(0.0), np.zeros(3)), [1, 0, 0, 0]
    )


def test_quaternion_matrix() -> None:
    M = tf.quaternion_matrix(np.array([0.99810947, 0.06146124, 0, 0]))
    assert np.allclose(M, tf.rotation_matrix(0.123, np.array([1, 0, 0])))
    assert np.allclose(tf.quaternion_matrix(np.array([1, 0, 0, 0])), np.identity(4))
    assert np.allclose(
        tf.quaternion_matrix(np.array([0, 1, 0, 0])), np.diag([1, -1, -1, 1])
    )
    assert np.allclose(tf.quaternion_matrix(np.zeros(4)), np.identity(4))


@pytest.mark.parametrize("isprecise", [False, True])
def test_quaternion_from_matrix_known_values(isprecise: bool) -> None:
    q = tf.quaternion_from_matrix(np.identity(4), isprecise)
    assert np.allclose(q, [1, 0, 0, 0])
    R = tf.rotation_matrix(0.123, np.array([1, 2, 3]))
    q = tf.quaternion_from_matrix(R, isprecise)
    assert np.allclose(q, [0.9981095, 0.0164262, 0.0328524, 0.0492786])


def test_quaternion_from_matrix_imprecise_input() -> None:
    R = np.array(
        [
            [-0.545, 0.797, 0.260, 0],
            [0.733, 0.603, -0.313, 0],
            [-0.407, 0.021, -0.913, 0],
            [0, 0, 0, 1],
        ]
    )
    q = tf.quaternion_from_matrix(R)
    assert np.allclose(q, [0.19069, 0.43736, 0.87485, -0.083611], atol=1e-5)


def test_quaternion_algebra() -> None:
    q = tf.quaternion_multiply(np.array([4, 1, -2, 3]), np.array([8, -5, 6, 7]))
    assert np.allclose(q, [28, -44, -14, 48])

    q0 = tf.random_quaternion(_rng().random(3))
    q1 = tf.quaternion_conjugate(q0)
    assert q1[0] == q0[0] and np.all(q1[1:] == -q0[1:])
    assert np.allclose(
        tf.quaternion_multiply(q0, tf.quaternion_inverse(q0)), [1, 0, 0, 0]
    )

    assert tf.quaternion_real(np.array([3, 0, 1, 2])) == 3.0
    assert np.allclose(tf.quaternion_imag(np.array([3, 0, 1, 2])), [0.0, 1.0, 2.0])


def test_quaternion_slerp() -> None:
    rng = _rng()
    q0 = tf.random_quaternion(rng.random(3))
    q1 = tf.random_quaternion(rng.random(3))
    assert np.allclose(tf.quaternion_slerp(q0, q1, np.array(0.0)), q0)
    assert np.allclose(tf.quaternion_slerp(q0, q1, np.array(1.0)), q1)

    q = tf.quaternion_slerp(q0, q1, np.array(0.5))
    angle = math.acos(np.clip(np.dot(q0, q), -1, 1))
    full = math.acos(abs(np.dot(q0, q1)))
    assert np.isclose(full / angle, 2.0)


def test_quaternion_slerp_identical_and_opposite_inputs() -> None:
    q0 = np.array([1.0, 0.0, 0.0, 0.0])
    assert np.allclose(tf.quaternion_slerp(q0, q0, np.array(0.5)), q0)
    # q and -q are the same rotation: returns q0 unchanged.
    assert np.allclose(tf.quaternion_slerp(q0, -q0, np.array(0.5)), q0)


def test_quaternion_slerp_shortest_path_flips_sign() -> None:
    z = np.array([0, 0, 1.0])
    q0 = tf.quaternion_about_axis(np.array(0.0), z)
    q1 = -tf.quaternion_about_axis(np.array(1.0), z)
    q = tf.quaternion_slerp(q0, q1, np.array(0.5))
    assert np.allclose(q, tf.quaternion_about_axis(np.array(0.5), z))


def test_random_quaternion_and_rotation_are_unit_and_orthonormal() -> None:
    rng = _rng()
    q = tf.random_quaternion(rng.random(3))
    assert np.isclose(tf.vector_norm(q), 1.0)
    assert q.shape == (4,)
    R = tf.random_rotation_matrix(rng.random(3))
    assert np.allclose(R.T @ R, np.identity(4))
    np.random.seed(RNG_SEED)
    assert np.isclose(tf.vector_norm(tf.random_quaternion()), 1.0)


def test_vector_norm() -> None:
    rng = _rng()
    v = rng.random(3)
    assert np.isclose(tf.vector_norm(v), np.linalg.norm(v))
    v = rng.random((6, 5, 3))
    assert np.allclose(tf.vector_norm(v, axis=-1), np.sqrt(np.sum(v * v, axis=2)))
    out = np.empty((6, 3))
    tf.vector_norm(v, axis=1, out=out)
    assert np.allclose(out, np.sqrt(np.sum(v * v, axis=1)))
    assert tf.vector_norm(np.array([])) == 0.0
    assert tf.vector_norm(np.array([1])) == 1.0


def test_unit_vector() -> None:
    rng = _rng()
    v0 = rng.random(3)
    assert np.allclose(tf.unit_vector(v0), v0 / np.linalg.norm(v0))
    v0 = rng.random((5, 4, 3))
    expected = v0 / np.expand_dims(np.sqrt(np.sum(v0 * v0, axis=1)), 1)
    assert np.allclose(tf.unit_vector(v0, axis=1), expected)
    out = np.empty((5, 4, 3))
    tf.unit_vector(v0, axis=1, out=out)
    assert np.allclose(out, expected)
    assert list(tf.unit_vector(np.array([1]))) == [1.0]


def test_random_vector_range() -> None:
    np.random.seed(RNG_SEED)
    v = tf.random_vector(1000)
    assert v.shape == (1000,)
    assert np.all(v >= 0) and np.all(v < 1)


def test_vector_product_and_angle_between_vectors() -> None:
    v = tf.vector_product(np.array([2, 0, 0]), np.array([0, 3, 0]))
    assert np.allclose(v, [0, 0, 6])
    a = tf.angle_between_vectors(np.array([1, -2, 3]), np.array([-1, 2, -3]))
    assert np.isclose(a, math.pi)
    a = tf.angle_between_vectors(
        np.array([1, -2, 3]), np.array([-1, 2, -3]), directed=False
    )
    assert np.isclose(a, 0.0)
    v0 = np.array([[2, 0, 0], [2, 0, 0], [0, 2, 0], [2, 0, 0]])
    v1 = np.array([[0, 3, 0], [0, 0, 3], [0, 0, 3], [3, 3, 3]])
    a = tf.angle_between_vectors(v0, v1, axis=1)
    assert np.allclose(a, [1.5708, 1.5708, 1.5708, 0.95532], atol=1e-4)


def test_inverse_matrix() -> None:
    M = _rng().random((4, 4)) - 0.5
    assert np.allclose(tf.inverse_matrix(M) @ M, np.identity(4))


def test_is_same_transform_normalizes_homogeneous_scale() -> None:
    assert tf.is_same_transform(np.identity(4), 2.0 * np.identity(4))
    assert not tf.is_same_transform(
        np.identity(4), tf.rotation_matrix(0.1, np.array([0, 0, 1.0]))
    )
