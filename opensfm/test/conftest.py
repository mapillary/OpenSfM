from collections import defaultdict
from distutils.version import LooseVersion

import numpy as np
import pytest
from opensfm import multiview, types, geo
from opensfm.synthetic_data import synthetic_examples
from opensfm.synthetic_data import synthetic_scene


def pytest_configure(config):
    use_legacy_numpy_printoptions()


def use_legacy_numpy_printoptions():
    """Ensure numpy use legacy print formant."""
    if LooseVersion(np.__version__).version[:2] > [1, 13]:
        np.set_printoptions(legacy="1.13")


@pytest.fixture(scope="module")
def null_scene() -> types.Reconstruction:
    reconstruction = types.Reconstruction()
    return reconstruction


@pytest.fixture(scope="module")
def scene_synthetic() -> synthetic_scene.SyntheticInputData:
    np.random.seed(42)
    data = synthetic_examples.synthetic_circle_scene()

    maximum_depth = 40
    projection_noise = 1.0
    gps_noise = 5.0

    gcps_count = 10
    gcps_shift = [10.0, 0.0, 100.0]

    reference = geo.TopocentricConverter(47.0, 6.0, 0)
    return synthetic_scene.SyntheticInputData(
        data.get_reconstruction(),
        reference,
        maximum_depth,
        projection_noise,
        gps_noise,
        False,
        gcps_count,
        gcps_shift,
    )


@pytest.fixture(scope="session")
def scene_synthetic_cube():
    np.random.seed(42)
    data = synthetic_examples.synthetic_cube_scene()

    reference = geo.TopocentricConverter(47.0, 6.0, 0)
    reconstruction = data.get_reconstruction()
    input_data = synthetic_scene.SyntheticInputData(
        reconstruction, reference, 40, 0.0, 0.0, False
    )
    return reconstruction, input_data.tracks_manager


@pytest.fixture(scope="module")
def scene_synthetic_rig() -> synthetic_scene.SyntheticInputData:
    np.random.seed(42)
    data = synthetic_examples.synthetic_rig_scene()

    maximum_depth = 40
    projection_noise = 1.0
    gps_noise = 0.1

    reference = geo.TopocentricConverter(47.0, 6.0, 0)
    return synthetic_scene.SyntheticInputData(
        data.get_reconstruction(),
        reference,
        maximum_depth,
        projection_noise,
        gps_noise,
        False,
    )


@pytest.fixture(scope="module")
def pairs_and_poses():
    np.random.seed(42)
    data = synthetic_examples.synthetic_cube_scene()

    reconstruction = data.get_reconstruction()
    reference = geo.TopocentricConverter(0, 0, 0)
    input_data = synthetic_scene.SyntheticInputData(
        reconstruction, reference, 40, 0.0, 0.0, False
    )
    features, tracks_manager = input_data.features, input_data.tracks_manager

    points_keys = list(reconstruction.points.keys())
    pairs, poses = defaultdict(list), defaultdict(list)
    for im1, im2 in tracks_manager.get_all_pairs_connectivity():
        tuples = tracks_manager.get_all_common_observations(im1, im2)
        f1 = [p.point for k, p, _ in tuples if k in points_keys]
        f2 = [p.point for k, _, p in tuples if k in points_keys]
        pairs[im1, im2].append((f1, f2))
        pose1 = reconstruction.shots[im1].pose
        pose2 = reconstruction.shots[im2].pose
        poses[im1, im2] = pose2.relative_to(pose1)

    camera = list(reconstruction.cameras.values())[0]
    return pairs, poses, camera, features, tracks_manager, reconstruction


@pytest.fixture(scope="module")
def pairs_and_their_E(pairs_and_poses):
    pairs, poses, camera, _, _, _ = pairs_and_poses

    pairs = list(sorted(zip(pairs.values(), poses.values()), key=lambda x: -len(x[0])))

    num_pairs = 20
    indices = [np.random.randint(0, len(pairs) - 1) for i in range(num_pairs)]

    ret_pairs = []
    for idx in indices:
        pair = pairs[idx]

        p1 = np.array([x for x, _ in pair[0]])
        p2 = np.array([x for _, x in pair[0]])
        p1 = p1.reshape(-1, p1.shape[-1])
        p2 = p2.reshape(-1, p2.shape[-1])
        f1 = camera.pixel_bearing_many(p1)
        f2 = camera.pixel_bearing_many(p2)

        pose = pair[1]
        R = pose.get_rotation_matrix()
        t_x = multiview.cross_product_matrix(pose.get_origin())
        e = R.dot(t_x)
        e /= np.linalg.norm(e)

        ret_pairs.append((f1, f2, e, pose))
    return ret_pairs


@pytest.fixture(scope="module")
def shots_and_their_points(pairs_and_poses):
    _, _, _, _, tracks_manager, reconstruction = pairs_and_poses

    ret_shots = []
    for shot in reconstruction.shots.values():
        bearings, points = [], []
        for k, obs in tracks_manager.get_shot_observations(shot.id).items():
            if k not in reconstruction.points:
                continue
            p = reconstruction.points[k]
            bearings.append(shot.camera.pixel_bearing(obs.point))
            points.append(p.coordinates)
        ret_shots.append((shot.pose, np.array(bearings), np.array(points)))

    return ret_shots
