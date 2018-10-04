import pytest

from opensfm import reconstruction
from opensfm import io

from opensfm.synthetic_data import synthetic_dataset
from opensfm.synthetic_data import synthetic_scene


@pytest.fixture(scope='module')
def scene():
    generator = synthetic_scene.get_scene_generator('ellipse', 100)
    scene = synthetic_scene.SyntheticScene(generator)
    scene.add_street(10000, 10, 7).perturb_floor([0, 0, 0.1]).\
        perturb_walls([0.2, 0.2, 0.01])

    camera = synthetic_scene.get_camera('perspective', '1', 0.9, -0.1, 0.01)
    scene.add_camera_sequence(camera, 0, 80, 1.5, 3, [0.2, 0.2, 0.01], 0.3)
    return scene


def test_reconstruction_incremental(scene):
    reference = scene.get_reconstruction()
    features, colors, graph = scene.get_tracks_data(40, 1.0)
    dataset = synthetic_dataset.SyntheticDataSet(reference, features,
                                                 colors, graph)

    ply_file = io.reconstruction_to_ply(reference)
    with open("/home/yann/corridor_before.ply", "w") as ply_output:
        ply_output.write(ply_file)

    dataset.config['processes'] = 12
    _, reconstructed_scene = reconstruction.\
        incremental_reconstruction(dataset, graph)

    errors = scene.compare(reconstructed_scene[0])
    ply_file = io.reconstruction_to_ply(reconstructed_scene[0])
    with open("/home/yann/corridor_after.ply", "w") as ply_output:
        ply_output.write(ply_file)
