import json
import os.path
from io import StringIO

import numpy as np
from opensfm import geo, io
from opensfm.test import data_generation


filename = os.path.join(
    data_generation.DATA_PATH, "berlin", "reconstruction_example.json"
)


def test_reconstructions_from_json_consistency():
    with open(filename) as fin:
        obj_before = json.loads(fin.read())
    obj_after = io.reconstructions_to_json(io.reconstructions_from_json(obj_before))

    assert obj_before[0]["cameras"] == obj_after[0]["cameras"]

    # need to explicitly use np.allclose because of numerical
    # differences on 3D coordinates (shot and points)
    for key, shot in obj_before[0]["shots"].items():
        for attr in shot:
            obj1 = obj_before[0]["shots"][key][attr]
            obj2 = obj_after[0]["shots"][key][attr]
            if attr in ["translation", "rotation"]:
                assert np.allclose(np.array(obj1), np.array(obj2))
            else:
                assert obj1 == obj2

    for key, point in obj_before[0]["points"].items():
        for attr in point:
            if attr == "reprojection_error":
                continue
            obj1 = obj_before[0]["points"][key][attr]
            obj2 = obj_after[0]["points"][key][attr]
            if attr == "coordinates":
                assert np.allclose(np.array(obj1), np.array(obj2))
            else:
                assert obj1 == obj2


def test_reconstructions_from_json():
    with open(filename) as fin:
        obj = json.loads(fin.read())

    reconstructions = io.reconstructions_from_json(obj)

    assert len(reconstructions) == 1
    assert len(reconstructions[0].cameras) == 1
    assert len(reconstructions[0].shots) == 3
    assert len(reconstructions[0].points) == 1588
    assert len(reconstructions[0].rig_cameras) == 1
    assert len(reconstructions[0].rig_instances) == 3


def test_reconstruction_to_ply():
    with open(filename) as fin:
        obj = json.loads(fin.read())
    reconstructions = io.reconstructions_from_json(obj)
    ply = io.reconstruction_to_ply(reconstructions[0])
    assert len(ply.splitlines()) > len(reconstructions[0].points)


def test_parse_projection():
    proj = io._parse_projection("WGS84")
    assert proj is None

    proj = io._parse_projection("WGS84 UTM 31N")
    easting, northing = 431760, 4582313.7
    lat, lon = 41.38946, 2.18378
    plon, plat = proj(easting, northing, inverse=True)
    assert np.allclose((lat, lon), (plat, plon))


def test_read_gcp_list():
    text = """WGS84
13.400740745 52.519134104 12.0792090446 2335.0 1416.7 01.jpg
13.400740745 52.519134104 12.0792090446 2639.1 938.0 02.jpg
13.400502446 52.519251158 16.7021233002 766.0 1133.1 01.jpg
    """
    fp = StringIO(text)
    reference = geo.TopocentricConverter(52.51913, 13.4007, 0)
    images = ["01.jpg", "02.jpg"]
    exif = {i: {"width": 3000, "height": 2000} for i in images}

    points = io.read_gcp_list(fp, reference, exif)
    assert len(points) == 2

    a, b = (len(point.observations) for point in points)
    assert min(a, b) == 1
    assert max(a, b) == 2


def test_read_write_ground_control_points():
    text = """
{
  "points": [
    {
      "id": "1",
      "observations": [
        {
          "shot_id": "01.jpg",
          "projection": [0.7153, 0.5787]
        },
        {
          "shot_id": "02.jpg",
          "projection": [0.8085, 0.3831]
        }
      ]
    },
    {
      "id": "2",
      "position": {
        "latitude": 52.519251158,
        "longitude": 13.400502446,
        "altitude": 16.7021233002
      },
      "observations": [
        {
          "shot_id": "01.jpg",
          "projection": [0.2346, 0.4628]
        }
      ]
    }
  ]
}
    """

    def check_points(points):
        assert len(points) == 2
        p1, p2 = points
        if p1.id != "1":
            p1, p2 = p2, p1

        assert p1.coordinates.has_value is False
        assert len(p1.observations) == 2
        assert np.allclose(p2.lla["latitude"], 52.519251158)
        assert np.allclose(p2.lla["longitude"], 13.400502446)
        assert np.allclose(p2.coordinates.value[2], 16.7021233002)
        assert len(p2.observations) == 1

    reference = geo.TopocentricConverter(52.51913, 13.4007, 0)

    # Read json
    fp = StringIO(text)
    points = io.read_ground_control_points(fp, reference)
    check_points(points)

    # Write json and re-read
    fwrite = StringIO()
    io.write_ground_control_points(points, fwrite, reference)
    freread = StringIO(fwrite.getvalue())
    points_reread = io.read_ground_control_points(freread, reference)
    check_points(points_reread)


def test_json_to_and_from_metadata():
    obj = {
        "orientation": 10,
        "capture_time": 1,
        "gps_dop": 2,
        "gps_position": [4, 5, 6],
        "skey": "test",
        "accelerometer": [7, 8, 9],
        "compass": {"angle": 10, "accuracy": 45},
    }
    m = io.json_to_pymap_metadata(obj)
    assert m.orientation.value == 10
    assert m.capture_time.value == 1
    assert m.gps_accuracy.value == 2
    assert np.allclose(m.gps_position.value, [4, 5, 6])
    assert m.sequence_key.value == "test"
    assert np.allclose(m.accelerometer.value, [7, 8, 9])
    assert m.compass_angle.value == 10
    assert m.compass_accuracy.value == 45
    assert obj == io.pymap_metadata_to_json(m)


def test_camera_from_to_vector():
    w, h = 640, 480
    camera_sizes = [
        ("perspective", 3),
        ("brown", 9),
        ("fisheye", 3),
        ("fisheye_opencv", 8),
        ("fisheye62", 12),
        ("radial", 6),
        ("simple_radial", 5),
        ("dual", 4),
        ("spherical", 0),
    ]
    for projection_type, num_params in camera_sizes:
        params = [float(i + 1) for i in range(num_params)]
        camera = io.camera_from_vector("cam1", w, h, projection_type, params)
        params_out = io.camera_to_vector(camera)
        assert params == params_out
