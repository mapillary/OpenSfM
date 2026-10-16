from pathlib import Path
from typing import cast
from unittest import mock

import numpy as np
import pytest
from opensfm import dataset, pygeometry, pymap, types
from opensfm.large import tools
from opensfm.large.metadataset import MetaDataSet


def _reconstruction(
    origins: list[list[float]], gps: list[list[float]], prefix: str = ""
) -> types.Reconstruction:
    camera = pygeometry.Camera.create_perspective(1.0, 0.0, 0.0)
    camera.id = "camera"
    rec = types.Reconstruction()
    rec.add_camera(camera)
    for i, (origin, position) in enumerate(zip(origins, gps)):
        pose = pygeometry.Pose()
        pose.set_origin(np.array(origin, dtype=float))
        shot = rec.create_shot(f"{prefix}{i}", camera.id, pose)
        shot.metadata.gps_position.value = np.array(position, dtype=float)
        shot.metadata.gps_accuracy.value = 1.0
    return rec


def _shots(rec: types.Reconstruction) -> dict[str, pymap.Shot]:
    return dict(rec.shots.items())


def _shot_dict(*names: str) -> dict[str, pymap.Shot]:
    rec = _reconstruction([[0, 0, 0]] * len(names), [[0, 0, 0]] * len(names))
    return {name: rec.shots[str(i)] for i, name in enumerate(names)}


def test_kmeans_separates_clusters() -> None:
    rng = np.random.default_rng(0)
    near = rng.normal(0.0, 0.1, size=(20, 2))
    far = rng.normal(10.0, 0.1, size=(20, 2))
    samples = np.vstack([near, far]).astype(np.float32)

    # cv2.kmeans returns (compactness, labels, centers).
    _, labels, centers = cast(
        tuple[float, np.ndarray, np.ndarray], tools.kmeans(samples, 2)
    )

    labels = labels.ravel()
    assert len(set(labels[:20])) == 1
    assert len(set(labels[20:])) == 1
    assert labels[0] != labels[20]
    assert np.allclose(centers[labels[0]], near.mean(0), atol=0.1)
    assert np.allclose(centers[labels[20]], far.mean(0), atol=0.1)


def test_add_cluster_neighbors_includes_nearby_points() -> None:
    # Three points about 11m apart along a meridian.
    positions = np.array([[0.0, 0.0], [0.0001, 0.0], [0.0002, 0.0]])
    labels = np.array([0, 0, 1])
    centers = np.zeros((2, 2))

    clusters = tools.add_cluster_neighbors(positions, labels, centers, 15.0)
    assert [list(map(int, c)) for c in clusters] == [[0, 1, 2], [1, 2]]

    clusters = tools.add_cluster_neighbors(positions, labels, centers, 1.0)
    assert [list(map(int, c)) for c in clusters] == [[0, 1], [2]]


def test_connected_reconstructions() -> None:
    r1 = tools.PartialReconstruction("a", 0)
    r2 = tools.PartialReconstruction("b", 0)
    r3 = tools.PartialReconstruction("c", 1)

    assert (
        len(
            tools.connected_reconstructions(
                {r1: _shot_dict("s1"), r2: _shot_dict("s2")}
            )
        )
        == 0
    )

    edges = tools.connected_reconstructions(
        {r1: _shot_dict("s1", "s2"), r2: _shot_dict("s2"), r3: _shot_dict("s1")}
    )
    assert {frozenset(e) for e in edges} == {
        frozenset((r1, r2)),
        frozenset((r1, r3)),
    }


def test_scale_matrix_positive_definite() -> None:
    covariance = np.array([[4.0, 2.0], [2.0, 3.0]])
    sm = tools.scale_matrix(covariance)
    # sm is the inverse Cholesky factor: sm @ cov @ sm.T == I
    assert np.allclose(sm @ covariance @ sm.T, np.identity(2))
    assert np.allclose(sm, np.linalg.inv(np.linalg.cholesky(covariance)))


def test_scale_matrix_falls_back_to_clipped_diagonal() -> None:
    covariance = np.array([[4.0, 0.0], [0.0, -1.0]])
    sm = tools.scale_matrix(covariance)
    assert np.allclose(sm, np.diag([0.5, 1.0 / np.sqrt(1e-8)]))


def test_invert_similarity_is_inverse() -> None:
    A = pygeometry.Pose(np.array([0.1, -0.2, 0.3])).get_rotation_matrix()
    b = np.array([1.0, 2.0, 3.0])
    s_inv, A_inv, b_inv = tools.invert_similarity(2.0, A, b)
    x = np.array([0.5, -0.5, 4.0])
    y = 2.0 * A.dot(x) + b
    assert s_inv == 0.5
    assert np.allclose(s_inv * A_inv.dot(y) + b_inv, x)


def test_partial_reconstruction_name() -> None:
    key = tools.PartialReconstruction("path/to/submodel", 3)
    assert tools.partial_reconstruction_name(key) == "path/to/submodel_index3"


def test_add_camera_constraints_soft_adds_shared_shots_once() -> None:
    rec_a = _reconstruction([[0, 0, 0], [1, 0, 0]], [[5, 0, 0], [6, 0, 0]])
    rec_b = _reconstruction([[0, 0, 0]], [[6, 0, 0]], prefix="")
    key_a = tools.PartialReconstruction("a", 0)
    key_b = tools.PartialReconstruction("b", 0)
    ra = mock.MagicMock()

    tools.add_camera_constraints_soft(
        ra,
        {key_a: _shots(rec_a), key_b: _shots(rec_b)},
        tools.partial_reconstruction_name,
    )

    assert [c.args[0] for c in ra.add_reconstruction.call_args_list] == [
        "a_index0",
        "b_index0",
    ]
    # Shot "0" appears in both reconstructions but is only added once.
    assert sorted(c.args[0] for c in ra.add_shot.call_args_list) == ["0", "1"]
    ra.add_absolute_position_constraint.assert_any_call("1", 6.0, 0.0, 0.0, 1.0)
    assert ra.add_absolute_position_constraint.call_count == 2
    assert ra.add_relative_motion_constraint.call_count == 3


def test_add_camera_constraints_hard_adds_common_cameras() -> None:
    rec_a = _reconstruction([[0, 0, 0], [1, 0, 0]], [[5, 0, 0], [6, 0, 0]])
    rec_b = _reconstruction([[0, 0, 0]], [[5, 0, 0]])
    key_a = tools.PartialReconstruction("a", 0)
    key_b = tools.PartialReconstruction("b", 0)
    ra = mock.MagicMock()

    tools.add_camera_constraints_hard(
        ra,
        {key_a: _shots(rec_a), key_b: _shots(rec_b)},
        tools.partial_reconstruction_name,
        True,
    )

    assert sorted(c.args[0] for c in ra.add_shot.call_args_list) == [
        "a_index00",
        "a_index01",
        "b_index00",
    ]
    ra.add_relative_absolute_position_constraint.assert_any_call(
        "b_index0", "b_index00", 5.0, 0.0, 0.0, 1.0
    )
    ra.add_common_camera_constraint.assert_called_once()
    args = ra.add_common_camera_constraint.call_args.args
    assert {(args[0], args[1]), (args[2], args[3])} == {
        ("a_index0", "a_index00"),
        ("b_index0", "b_index00"),
    }


def test_add_camera_constraints_hard_without_common_constraint() -> None:
    rec = _reconstruction([[0, 0, 0]], [[5, 0, 0]])
    key_a = tools.PartialReconstruction("a", 0)
    key_b = tools.PartialReconstruction("b", 0)
    ra = mock.MagicMock()
    tools.add_camera_constraints_hard(
        ra, {key_a: _shots(rec), key_b: _shots(rec)}, str, False
    )
    ra.add_common_camera_constraint.assert_not_called()


@pytest.mark.parametrize(
    "constraint_type", ["soft_camera_constraint", "hard_camera_constraint"]
)
def test_align_reconstructions_moves_shots_to_gps(constraint_type: str) -> None:
    origins = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
    gps = [[10.0, 0.0, 0.0], [11.0, 0.0, 0.0], [12.0, 0.0, 0.0], [10.0, 1.0, 0.0]]
    rec = _reconstruction(origins, gps)
    key = tools.PartialReconstruction("a", 0)

    transformations = tools.align_reconstructions(
        {key: _shots(rec)},
        tools.partial_reconstruction_name,
        False,
        constraint_type,
    )

    s, A, b = transformations[key]
    for origin, position in zip(origins, gps):
        assert np.allclose(s * A.dot(origin) + b, position, atol=1e-3)


def test_add_point_constraints_filters_by_resection_result() -> None:
    key_a = tools.PartialReconstruction("a", 0)
    key_b = tools.PartialReconstruction("b", 0)
    key_c = tools.PartialReconstruction("c", 0)
    shots = {key_a: _shot_dict("s1"), key_b: _shot_dict("s1"), key_c: _shot_dict("s2")}

    rec_a = mock.MagicMock()
    rec_a.points = {"p1": mock.MagicMock(coordinates=[1.0, 2.0, 3.0])}
    rec_b = mock.MagicMock()
    rec_b.points = {"q1": mock.MagicMock(coordinates=[4.0, 5.0, 6.0])}
    loaded = {"a": (rec_a, "ga"), "b": (rec_b, "gb")}
    inliers = [("p1", "q1")] * 25
    ra = mock.MagicMock()

    with (
        mock.patch.object(
            tools, "load_reconstruction", side_effect=lambda p, i: (p, loaded[p])
        ),
        mock.patch.object(
            tools.reconstruction,
            "resect_reconstruction",
            return_value=(True, np.identity(4), inliers),
        ) as resect,
    ):
        tools.add_point_constraints(ra, shots, tools.partial_reconstruction_name)

    resect.assert_called_once_with(rec_a, rec_b, "ga", "gb", 0.3, 20)
    assert ra.add_common_point_constraint.call_count == 25
    ra.add_common_point_constraint.assert_called_with(
        "a_index0", 1.0, 2.0, 3.0, "b_index0", 4.0, 5.0, 6.0, 1e-1
    )


@pytest.mark.parametrize(
    "status, scale, num_inliers",
    [(False, 1.0, 25), (True, 2.0, 25), (True, 0.5, 25), (True, 1.0, 5)],
)
def test_add_point_constraints_rejects_bad_alignment(
    status: bool, scale: float, num_inliers: int
) -> None:
    key_a = tools.PartialReconstruction("a", 0)
    key_b = tools.PartialReconstruction("b", 0)
    T = np.identity(4)
    T[:3, :3] *= scale
    ra = mock.MagicMock()
    with (
        mock.patch.object(
            tools,
            "load_reconstruction",
            return_value=("x", (mock.MagicMock(), mock.MagicMock())),
        ),
        mock.patch.object(
            tools.reconstruction,
            "resect_reconstruction",
            return_value=(status, T, [("p", "q")] * num_inliers),
        ),
    ):
        tools.add_point_constraints(
            ra,
            {key_a: _shot_dict("s"), key_b: _shot_dict("s")},
            tools.partial_reconstruction_name,
        )
    ra.add_common_point_constraint.assert_not_called()


def _make_submodel(path: Path, recs: list[types.Reconstruction] | None) -> None:
    path.mkdir(parents=True)
    (path / "image_list.txt").write_text("images/0.jpg\n")
    if recs is not None:
        dataset.DataSet(str(path)).save_reconstruction(recs)


def test_load_reconstruction_shots_and_apply_transformations(tmp_path: Path) -> None:
    meta = MetaDataSet(str(tmp_path))
    rec0 = _reconstruction([[0, 0, 0]], [[0, 0, 0]])
    rec1 = _reconstruction([[1, 0, 0]], [[0, 0, 0]], prefix="x")
    _make_submodel(tmp_path / "submodels" / "submodel_0000", [rec0, rec1])
    _make_submodel(tmp_path / "submodels" / "submodel_0001", None)

    shots = tools.load_reconstruction_shots(meta)

    submodel0 = str(tmp_path / "submodels" / "submodel_0000")
    key0 = tools.PartialReconstruction(submodel0, 0)
    key1 = tools.PartialReconstruction(submodel0, 1)
    assert set(shots) == {key0, key1}
    assert list(shots[key0]) == ["0"]
    assert list(shots[key1]) == ["x0"]

    missing = tools.PartialReconstruction(
        str(tmp_path / "submodels" / "submodel_0001"), 0
    )
    transformations = {
        key0: (1.0, np.identity(3), np.array([10.0, 0.0, 0.0])),
        key1: (2.0, np.identity(3), np.zeros(3)),
        missing: (1.0, np.identity(3), np.zeros(3)),
    }
    tools.apply_transformations(transformations)

    aligned = dataset.DataSet(submodel0).load_reconstruction(
        "reconstruction.aligned.json"
    )
    assert np.allclose(aligned[0].shots["0"].pose.get_origin(), [10.0, 0.0, 0.0])
    assert np.allclose(aligned[1].shots["x0"].pose.get_origin(), [2.0, 0.0, 0.0])
    assert not dataset.DataSet(missing.submodel_path).reconstruction_exists(
        "reconstruction.aligned.json"
    )
