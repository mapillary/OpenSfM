import json
import os
import tempfile
import unittest
from pathlib import Path
from typing import Any
from unittest import mock

import numpy as np
from mapillary.opensfm.annotation_gui_gcp import run_ba
from opensfm import dataset, geo, pygeometry, pymap, types
from opensfm.align import apply_similarity


CAMERA_ID = "cam"
SHOT_X = (-2.0, 0.0, 2.0)
# World coordinates of the ground control points, in front of all cameras.
GCP_POINTS = np.array(
    [
        [-3.0, -2.0, 10.0],
        [3.0, -2.0, 11.0],
        [3.0, 2.0, 9.0],
        [-3.0, 2.0, 12.0],
        [0.0, 0.0, 10.5],
    ]
)
# Similarity (s, A, b) that maps world coordinates into reconstruction B.
SIM_S = 2.0
SIM_A: np.ndarray = pygeometry.Pose(np.array([0.0, 0.0, 0.3])).get_rotation_matrix()
SIM_B = np.array([5.0, -1.0, 2.0])


def _camera() -> pygeometry.Camera:
    camera = pygeometry.Camera.create_perspective(1.0, 0.0, 0.0)
    camera.id = CAMERA_ID
    camera.width = 640
    camera.height = 480
    return camera


def _reconstruction(
    prefix: str, y: float, with_points: bool = True
) -> types.Reconstruction:
    """Three cameras along x looking down +z, at height y (world frame)."""
    rec = types.Reconstruction()
    rec.reference = geo.TopocentricConverter(0.0, 0.0, 0.0)
    rec.add_camera(_camera())
    for i, x in enumerate(SHOT_X):
        pose = pygeometry.Pose()
        pose.set_origin(np.array([x, y, 0.0]))
        shot = rec.create_shot(f"{prefix}{i}", CAMERA_ID, pose)
        shot.metadata.gps_position.value = np.array([x, y, 0.0])
        shot.metadata.gps_accuracy.value = 1.0
    if with_points:
        for j, coords in enumerate(GCP_POINTS):
            point = rec.create_point(f"p{j}", coords)
            point.color = np.array([10.0 * j, 0.0, 0.0])
    return rec


def _gcp(
    gcp_id: str,
    coords: np.ndarray,
    shots: list[pymap.Shot],
    lla: bool = True,
) -> pymap.GroundControlPoint:
    gcp = pymap.GroundControlPoint()
    gcp.id = gcp_id
    if lla:
        lat, lon, alt = geo.lla_from_topocentric(
            coords[0], coords[1], coords[2], 0.0, 0.0, 0.0
        )
        gcp.lla = {"latitude": lat, "longitude": lon, "altitude": alt}
        gcp.has_altitude = True
    for shot in shots:
        obs = pymap.GroundControlPointObservation()
        obs.shot_id = shot.id
        obs.projection = shot.project(coords)
        gcp.add_observation(obs)
    return gcp


def _ordered_shots(rec: types.Reconstruction) -> list[pymap.Shot]:
    return [rec.shots[shot_id] for shot_id in sorted(rec.shots)]


def _gcps(rec: types.Reconstruction) -> list[pymap.GroundControlPoint]:
    shots = _ordered_shots(rec)
    return [_gcp(f"g{i}", p, shots) for i, p in enumerate(GCP_POINTS)]


def _tracks_manager(recs: list[types.Reconstruction]) -> pymap.TracksManager:
    tracks_manager = pymap.TracksManager()
    for rec in recs:
        for shot in rec.shots.values():
            for j, coords in enumerate(GCP_POINTS):
                x, y = shot.project(coords)
                tracks_manager.add_observation(
                    shot.id, f"p{j}", pymap.Observation(x, y, 0.004, 255, 0, 0, j)
                )
    return tracks_manager


class MergeResplitTest(unittest.TestCase):
    def test_merge_prefixes_points_and_keeps_observations(self) -> None:
        rec_a = _reconstruction("a", 0.0)
        rec_b = _reconstruction("b", 1.0)
        tracks_manager = _tracks_manager([rec_a])

        merged = run_ba.merge_reconstructions([rec_a, rec_b], tracks_manager)

        self.assertEqual(len(merged.shots), 6)
        self.assertEqual(len(merged.points), 10)
        self.assertIn("R0_p0", merged.points)
        self.assertIn("R1_p4", merged.points)
        np.testing.assert_allclose(merged.points["R0_p2"].color, [20.0, 0.0, 0.0])
        # Shots from rec A have observations; rec B shots are missing from the
        # tracks manager and are kept without observations.
        self.assertEqual(len(merged.shots["a0"].get_valid_landmarks()), 5)
        self.assertEqual(len(merged.shots["b0"].get_valid_landmarks()), 0)

    def test_resplit_restores_original_split(self) -> None:
        rec_a = _reconstruction("a", 0.0)
        rec_b = _reconstruction("b", 1.0)
        merged = run_ba.merge_reconstructions(
            [rec_a, rec_b], _tracks_manager([rec_a, rec_b])
        )
        merged.points["R1_p0"].coordinates = np.array([1.0, 2.0, 3.0])
        merged.remove_point("R1_p1")

        split = run_ba.resplit_reconstruction(merged, [rec_a, rec_b])

        self.assertEqual(len(split), 2)
        self.assertEqual(sorted(split[0].shots), ["a0", "a1", "a2"])
        self.assertEqual(sorted(split[1].shots), ["b0", "b1", "b2"])
        self.assertEqual(sorted(split[1].points), ["p0", "p2", "p3", "p4"])
        np.testing.assert_allclose(split[1].points["p0"].coordinates, [1, 2, 3])
        self.assertIn(CAMERA_ID, split[0].cameras)


class GcpErrorsTest(unittest.TestCase):
    def test_triangulate_gcps(self) -> None:
        rec = _reconstruction("a", 0.0)
        gcps = _gcps(rec)
        gcps.append(_gcp("single", GCP_POINTS[0], [rec.shots["a0"]]))

        coords = run_ba.triangulate_gcps(gcps, rec)

        for triangulated, expected in zip(coords[:-1], GCP_POINTS):
            np.testing.assert_allclose(triangulated, expected, atol=1e-6)
        self.assertIsNone(coords[-1])

    def test_gcp_geopositional_error(self) -> None:
        rec = _reconstruction("a", 0.0)
        shots = _ordered_shots(rec)
        gcps = [
            _gcp("good", GCP_POINTS[0], shots),
            _gcp("no_lla", GCP_POINTS[1], shots, lla=False),
            _gcp("one_view", GCP_POINTS[2], shots[:1]),
        ]

        errors = run_ba.gcp_geopositional_error(gcps, rec)

        good = errors["good"]
        np.testing.assert_allclose(good["triangulated_xyz"], GCP_POINTS[0], atol=1e-6)
        self.assertAlmostEqual(
            good["error"],
            float(
                np.linalg.norm(
                    np.array(good["expected_xyz"]) - np.array(good["triangulated_xyz"])
                )
            ),
        )
        self.assertIn("error_planar", good)
        for gcp_id in ("no_lla", "one_view"):
            self.assertTrue(np.isnan(errors[gcp_id]["error"]))
            self.assertTrue(np.isnan(errors[gcp_id]["error_planar"]))

    def test_reproject_gcps(self) -> None:
        rec = _reconstruction("a", 0.0)
        shots = _ordered_shots(rec)
        good = _gcp("good", GCP_POINTS[0], shots)
        # An observation in an image outside the reconstruction.
        extra = pymap.GroundControlPointObservation()
        extra.shot_id = "other"
        extra.projection = np.array([0.0, 0.0])
        good.add_observation(extra)
        # One observation shifted by 0.1 to produce a reprojection error.
        wrong = _gcp("wrong", GCP_POINTS[1], shots)
        observations = wrong.observations
        observations[0].projection = observations[0].projection + [0.1, 0.0]
        wrong.observations = observations
        lonely = _gcp("lonely", GCP_POINTS[2], shots[:1])

        output = run_ba.reproject_gcps([good, wrong, lonely], rec, 10)

        self.assertEqual(output["lonely"], {})
        self.assertEqual(sorted(output["good"]), ["a0", "a1", "a2", "other"])
        self.assertEqual(output["good"]["other"]["error"], 0)
        for shot_id in ("a0", "a1", "a2"):
            self.assertAlmostEqual(output["good"][shot_id]["error"], 0.0, places=6)
            np.testing.assert_allclose(
                output["good"][shot_id]["reprojection"],
                rec.shots[shot_id].project(GCP_POINTS[0]),
                atol=1e-6,
            )
        self.assertGreater(output["wrong"]["a0"]["error"], 0.01)

    def test_sorted_errors_wrong_counts_and_std(self) -> None:
        reprojections: dict[str, dict[str, dict[str, Any]]] = {
            "g1": {"s1": {"error": 0.3}, "s2": {"error": 0.01}},
            "g2": {"s1": {"error": 0.4}, "s3": {"error": float("nan")}},
        }

        self.assertEqual(
            run_ba.get_sorted_reprojection_errors(
                {"g1": reprojections["g1"], "g2": {"s1": {"error": 0.4}}}
            ),
            [("g2", "s1", 0.4), ("g1", "s1", 0.3), ("g1", "s2", 0.01)],
        )
        self.assertEqual(
            run_ba.get_number_of_wrong_annotations_per_gcp(reprojections, 0.1),
            {"g1": 1, "g2": 1},
        )
        self.assertAlmostEqual(
            run_ba.compute_gcp_std(reprojections),
            float(np.sqrt((0.3**2 + 0.01**2 + 0.4**2) / 3)),
        )


class AlignmentTest(unittest.TestCase):
    def test_find_alignment_recovers_similarity_and_skips_missing(self) -> None:
        # The returned (s, A, b) maps points0 onto points1, which is how
        # `align` uses it to move reconstruction B into A's frame.
        points0: list[np.ndarray | None] = [*GCP_POINTS, None]
        points1: list[np.ndarray | None] = [
            *(SIM_S * SIM_A.dot(p) + SIM_B for p in GCP_POINTS),
            np.zeros(3),
        ]

        s, A, b = run_ba.find_alignment(points0, points1)

        self.assertAlmostEqual(s, SIM_S)
        np.testing.assert_allclose(A, SIM_A, atol=1e-9)
        np.testing.assert_allclose(b, SIM_B, atol=1e-9)

    def test_find_alignment_without_points_raises(self) -> None:
        with self.assertRaises(ValueError):
            run_ba.find_alignment([None], [None])

    def test_decompose_covariance(self) -> None:
        u, std = run_ba.decompose_covariance(np.diag([9.0, 4.0, 1.0]))
        np.testing.assert_allclose(std, [3.0, 2.0, 1.0])
        np.testing.assert_allclose(np.abs(u), np.identity(3))

    def test_align_3d_annotations_to_reconstruction(self) -> None:
        rec = _reconstruction("a", 0.0)
        gcps = _gcps(rec)
        gcps_3d = {
            "model": [
                ((SIM_S * SIM_A.dot(p) + SIM_B).tolist(), f"g{i}")
                for i, p in enumerate(GCP_POINTS)
            ]
        }

        result = run_ba.align_3d_annotations_to_reconstruction(
            gcps, gcps_3d, "model", rec
        )

        assert result is not None
        reprojections, alignment = result
        self.assertAlmostEqual(alignment["s"], SIM_S)
        np.testing.assert_allclose(alignment["b"], SIM_B, atol=1e-6)
        for coords, gcp_id in gcps_3d["model"]:
            np.testing.assert_allclose(reprojections[gcp_id], coords, atol=1e-6)

    def test_align_3d_annotations_needs_three_triangulated_gcps(self) -> None:
        rec = _reconstruction("a", 0.0)
        gcps = _gcps(rec)[:2]
        self.assertIsNone(
            run_ba.align_3d_annotations_to_reconstruction(
                gcps, {"model": []}, "model", rec
            )
        )


class BundleHelpersTest(unittest.TestCase):
    def test_add_gcp_to_bundle(self) -> None:
        rec = _reconstruction("a", 0.0)
        shots = _ordered_shots(rec)
        triangulated = _gcp("tri", GCP_POINTS[0], shots)
        lla_only = _gcp("lla_only", GCP_POINTS[1], shots[:1])
        unusable = _gcp("unusable", GCP_POINTS[2], shots[:1], lla=False)
        ba = mock.MagicMock()

        run_ba.add_gcp_to_bundle(
            ba, rec.reference, [triangulated, lla_only, unusable], 0.5, rec.shots
        )

        added = {c.args[0]: c.args[1] for c in ba.add_point.call_args_list}
        self.assertEqual(sorted(added), ["gcp-lla_only", "gcp-tri"])
        np.testing.assert_allclose(added["gcp-tri"], GCP_POINTS[0], atol=1e-6)
        np.testing.assert_allclose(added["gcp-lla_only"], GCP_POINTS[1], atol=1e-3)
        # 3 observations for "tri", 1 for "lla_only".
        self.assertEqual(ba.add_point_projection_observation.call_count, 4)
        lla_only_obs = [
            c.args
            for c in ba.add_point_projection_observation.call_args_list
            if c.args[1] == "gcp-lla_only"
        ]
        self.assertEqual(len(lla_only_obs), 1)
        self.assertEqual(lla_only_obs[0][0], "a0")
        self.assertEqual(lla_only_obs[0][3], 0.5)


class ResectTest(unittest.TestCase):
    def test_resect_image_from_gcps(self) -> None:
        rec = _reconstruction("a", 0.0)
        # The target image sees the GCPs from (1, 1, 0) but is not in rec.
        target = _reconstruction("t", 1.0, with_points=False)
        target_shot = target.shots["t1"]
        gcps = [
            _gcp(f"g{i}", p, [*rec.shots.values(), target_shot])
            for i, p in enumerate(GCP_POINTS)
        ]
        dst = types.Reconstruction()
        data = mock.MagicMock()
        metadata = pymap.ShotMeasurements()

        with mock.patch.object(
            run_ba.helpers, "get_image_metadata", return_value=metadata
        ) as get_metadata:
            shot = run_ba.resect_image("t1", _camera(), gcps, rec, data, dst)

        assert shot is not None
        get_metadata.assert_called_once_with(data, "t1")
        self.assertIn("t1", dst.shots)
        self.assertNotIn("t1", rec.shots)
        np.testing.assert_allclose(shot.pose.get_origin(), [0.0, 1.0, 0.0], atol=1e-2)

    def test_resect_image_not_enough_annotations(self) -> None:
        rec = _reconstruction("a", 0.0)
        gcps = _gcps(rec)[:2]
        self.assertIsNone(
            run_ba.resect_image("a0", _camera(), gcps, rec, mock.MagicMock())
        )

    def test_resect_annotated_single_images(self) -> None:
        rec = _reconstruction("a", 0.0)
        target = _reconstruction("t", 1.0, with_points=False)
        gcps = [
            _gcp(f"g{i}", p, [*rec.shots.values(), target.shots["t1"]])
            for i, p in enumerate(GCP_POINTS)
        ]
        data = mock.MagicMock()
        data.images.return_value = ["a0", "a1", "a2", "t1"]
        data.load_exif.return_value = {"camera": CAMERA_ID}

        with mock.patch.object(
            run_ba.helpers,
            "get_image_metadata",
            return_value=pymap.ShotMeasurements(),
        ):
            resected = run_ba.resect_annotated_single_images(
                rec, gcps, {CAMERA_ID: _camera()}, data
            )

        data.load_exif.assert_called_once_with("t1")
        self.assertEqual(list(resected.shots), ["t1"])
        self.assertEqual(list(resected.cameras), [CAMERA_ID])


class AlignEndToEndTest(unittest.TestCase):
    def setUp(self) -> None:
        self._tmp = tempfile.TemporaryDirectory()
        self.path = Path(self._tmp.name)

    def tearDown(self) -> None:
        self._tmp.cleanup()

    def _write_dataset(self, two_reconstructions: bool = True) -> None:
        rec_a = _reconstruction("a", 0.0)
        rec_b = _reconstruction("b", 1.0)
        world_gcps = _gcps(rec_a) if not two_reconstructions else None
        if two_reconstructions:
            shots = [*rec_a.shots.values(), *rec_b.shots.values()]
            world_gcps = [_gcp(f"g{i}", p, shots) for i, p in enumerate(GCP_POINTS)]
            apply_similarity(rec_b, SIM_S, SIM_A, SIM_B)
        assert world_gcps is not None

        recs = [rec_a, rec_b] if two_reconstructions else [rec_a]
        image_names = [s for r in recs for s in r.shots]
        (self.path / "image_list.txt").write_text(
            "".join(f"images/{name}\n" for name in image_names)
        )
        data = dataset.DataSet(str(self.path))
        data.save_reconstruction(recs)
        data.save_camera_models({CAMERA_ID: _camera()})
        data.save_tracks_manager(_tracks_manager(recs))
        for name in image_names:
            data.save_exif(name, {"camera": CAMERA_ID, "width": 640, "height": 480})

        points: list[dict[str, Any]] = []
        for gcp in world_gcps:
            observations: list[dict[str, Any]] = [
                {"shot_id": o.shot_id, "projection": list(o.projection)}
                for o in gcp.observations
            ]
            points.append(
                {"id": gcp.id, "position": gcp.lla, "observations": observations}
            )
        # 3D annotations on an external model, stored as 3-vector projections.
        for i, p in enumerate(GCP_POINTS):
            points[i]["observations"].append(
                {"shot_id": "model.ply", "projection": (2.0 * p).tolist()}
            )
        with open(self.path / "ground_control_points.json", "w") as f:
            json.dump({"points": points}, f)

    def _read_json(self, name: str) -> Any:
        with open(self.path / name) as f:
            return json.load(f)

    def test_missing_files_does_nothing(self) -> None:
        (self.path / "image_list.txt").write_text("images/a0\n")
        run_ba.align(str(self.path), 0, 1, True, False, 0.016, 0.3)
        self.assertEqual(sorted(os.listdir(self.path)), ["image_list.txt"])

    def test_fix_and_load_3d_annotations(self) -> None:
        self._write_dataset()
        data = dataset.DataSet(str(self.path))

        run_ba.fix_3d_annotations_in_gcp_file(data)
        annotations = run_ba.load_3d_annotations_from_gcp_file(data)

        raw = self._read_json("ground_control_points.json")
        model_obs = [
            o for o in raw["points"][0]["observations"] if o["shot_id"] == "model.ply"
        ]
        self.assertEqual(model_obs, [{"shot_id": "model.ply", "point": [-6, -4, 20]}])
        self.assertEqual(list(annotations), ["model.ply"])
        self.assertEqual(annotations["model.ply"][0], ([-6.0, -4.0, 20.0], "g0"))
        self.assertEqual(len(annotations["model.ply"]), 5)

    def test_align_single_reconstruction_only_aligns_3d_models(self) -> None:
        self._write_dataset(two_reconstructions=False)

        run_ba.align(str(self.path), 0, 1, True, 0.016)

        alignment = self._read_json("affine_alignment_0xmodel.ply.json")
        self.assertAlmostEqual(alignment["s"], 2.0, places=5)
        reprojections = self._read_json("gcp_reprojections_3D_0xmodel.ply.json")
        np.testing.assert_allclose(reprojections["g1"], 2.0 * GCP_POINTS[1], atol=1e-4)
        self.assertFalse((self.path / "run_ba_metrics_0x1.json").exists())

    def test_align_rigid_two_reconstructions(self) -> None:
        self._write_dataset()

        run_ba.align(str(self.path), 0, 1, True, False, 0.016, 0.3)

        data = dataset.DataSet(str(self.path))
        rigid = data.load_reconstruction("reconstruction_gcp_rigid_0x1.json")
        # Rec B is moved back into rec A's frame.
        np.testing.assert_allclose(
            rigid[1].shots["b2"].pose.get_origin(), [2.0, 1.0, 0.0], atol=1e-4
        )
        resplit = data.load_reconstruction("reconstruction_gcp_ba_resplit_0x1.json")
        self.assertEqual(sorted(resplit[1].shots), ["b0", "b1", "b2"])

        metrics = self._read_json("run_ba_metrics_0x1.json")
        self.assertEqual(metrics["n_gcp"], 5)
        self.assertEqual(metrics["n_bad_gcp_annotations"], 0)
        self.assertLess(metrics["max_reprojection_error"], 1e-4)
        self.assertEqual(metrics["median_shot_std"], -1)
        self.assertEqual((metrics["rec_a"], metrics["rec_b"]), (0, 1))
        reprojections = self._read_json("gcp_reprojections_0x1.json")
        self.assertEqual(sorted(reprojections), ["g0", "g1", "g2", "g3", "g4"])


class ParseArgsTest(unittest.TestCase):
    def test_parse_args_defaults(self) -> None:
        with mock.patch("sys.argv", ["run_ba", "/data"]):
            args = run_ba.parse_args()
        self.assertEqual(args.path_dataset, "/data")
        self.assertEqual((args.rec_a, args.rec_b), (0, 1))
        self.assertFalse(args.rigid)
        self.assertAlmostEqual(args.px_threshold, 0.016)

    def test_parse_args_rejects_rigid_with_covariance(self) -> None:
        with mock.patch("sys.argv", ["run_ba", "/data", "--rigid", "--covariance"]):
            with self.assertRaises(AssertionError):
                run_ba.parse_args()

    def test_main_calls_align_with_args(self) -> None:
        argv = ["run_ba", "/data", "--rec_a", "2", "--rec_b", "3", "--rigid"]
        with (
            mock.patch("sys.argv", argv),
            mock.patch.object(run_ba, "align", return_value=None) as align,
            mock.patch.object(run_ba.log, "setup"),
        ):
            with self.assertRaises(SystemExit):
                run_ba.main()
        align.assert_called_once_with("/data", 2, 3, True, 0.016)
