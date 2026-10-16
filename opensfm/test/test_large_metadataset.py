import json
import os
from pathlib import Path

import numpy as np
from opensfm.large.metadataset import MetaDataSet


def _make_dataset(root: Path, images: list[str]) -> None:
    (root / "images").mkdir(parents=True)
    for image in images:
        (root / "images" / image).write_bytes(b"jpeg")


def test_init_uses_default_config_and_creates_submodels_dir(tmp_path: Path) -> None:
    meta = MetaDataSet(str(tmp_path))
    assert meta.config["submodels_relpath"] == "submodels"
    assert (tmp_path / "submodels").is_dir()


def test_init_reads_config_file(tmp_path: Path) -> None:
    (tmp_path / "config.yaml").write_text("submodels_relpath: parts\n")
    meta = MetaDataSet(str(tmp_path))
    assert meta.config["submodels_relpath"] == "parts"
    assert (tmp_path / "parts").is_dir()


def test_image_list_roundtrip(tmp_path: Path) -> None:
    meta = MetaDataSet(str(tmp_path))
    assert not meta.image_list_exists()

    meta.create_image_list([("a.jpg", 1.5, -2.25), ("b.jpg", 0.0, 3.0)])

    assert meta.image_list_exists()
    assert list(meta.images_with_gps()) == [("a.jpg", 1.5, -2.25), ("b.jpg", 0.0, 3.0)]


def test_image_groups(tmp_path: Path) -> None:
    meta = MetaDataSet(str(tmp_path))
    assert not meta.image_groups_exists()
    (tmp_path / "image_groups.txt").write_text("a.jpg group1\nb.jpg group2\n")
    assert meta.image_groups_exists()
    assert list(meta.load_image_groups()) == [["a.jpg", "group1"], ["b.jpg", "group2"]]


def test_clusters_roundtrip(tmp_path: Path) -> None:
    meta = MetaDataSet(str(tmp_path))
    images = np.array([["a.jpg"], ["b.jpg"]])
    positions = np.array([[1.0, 2.0], [3.0, 4.0]])
    labels = np.array([[0], [1]])
    centers = np.array([[1.0, 2.0], [3.0, 4.0]])
    meta.save_clusters(images, positions, labels, centers)

    loaded_images, loaded_positions, loaded_labels, loaded_centers = (
        meta.load_clusters()
    )
    # Images and labels are flattened on load.
    assert loaded_images.tolist() == ["a.jpg", "b.jpg"]
    assert loaded_labels.tolist() == [0, 1]
    assert np.array_equal(loaded_positions, positions)
    assert np.array_equal(loaded_centers, centers)


def test_clusters_with_neighbors_roundtrip(tmp_path: Path) -> None:
    meta = MetaDataSet(str(tmp_path))
    meta.save_clusters_with_neighbors([[0, 1, 2], [2, 3]])
    clusters = meta.load_clusters_with_neighbors()
    assert [list(c) for c in clusters] == [[0, 1, 2], [2, 3]]


def test_save_geojson_files(tmp_path: Path) -> None:
    meta = MetaDataSet(str(tmp_path))
    geojson = {"type": "FeatureCollection", "features": []}
    meta.save_clusters_geojson(geojson)
    meta.save_cluster_with_neighbors_geojson({"type": "Feature"})

    submodels = tmp_path / "submodels"
    assert json.loads((submodels / "clusters.geojson").read_text()) == geojson
    assert json.loads((submodels / "clusters_with_neighbors.geojson").read_text()) == {
        "type": "Feature"
    }


def test_create_submodels_links_images_and_shared_files(tmp_path: Path) -> None:
    _make_dataset(tmp_path, ["a.jpg", "b.jpg", "c.jpg"])
    (tmp_path / "config.yaml").write_text("processes: 1\n")
    (tmp_path / "camera_models.json").write_text("{}")
    (tmp_path / "features").mkdir()
    meta = MetaDataSet(str(tmp_path))

    meta.create_submodels(np.array([["a.jpg", "b.jpg"], ["b.jpg", "c.jpg"]]))

    sub0 = tmp_path / "submodels" / "submodel_0000"
    sub1 = tmp_path / "submodels" / "submodel_0001"
    assert (sub0 / "image_list.txt").read_text() == "images/a.jpg\nimages/b.jpg\n"
    assert (sub1 / "image_list.txt").read_text() == "images/b.jpg\nimages/c.jpg\n"
    assert (sub0 / "images" / "a.jpg").is_symlink()
    assert (sub0 / "images" / "a.jpg").read_bytes() == b"jpeg"
    assert (sub0 / "config.yaml").read_text() == "processes: 1\n"
    assert (sub0 / "reports").is_dir()
    # Shared files are relative symlinks pointing back at the meta dataset.
    assert os.readlink(sub0 / "camera_models.json") == os.path.join(
        "..", "..", "camera_models.json"
    )
    assert (sub0 / "features").is_dir()
    # Missing files are not linked.
    assert not (sub0 / "matches").exists()

    assert meta.get_submodel_paths() == [str(sub0), str(sub1)]

    # Re-running replaces existing links instead of failing.
    meta.create_submodels(np.array([["a.jpg", "b.jpg"]]))
    assert (sub0 / "camera_models.json").is_symlink()


def test_create_symlink_for_nested_path(tmp_path: Path) -> None:
    (tmp_path / "reports").mkdir()
    (tmp_path / "reports" / "features.json").write_text("{}")
    meta = MetaDataSet(str(tmp_path))
    base = tmp_path / "submodels" / "submodel_0000"
    (base / "reports").mkdir(parents=True)

    meta._create_symlink(str(base), os.path.join("reports", "features.json"))

    link = base / "reports" / "features.json"
    assert os.readlink(link) == os.path.join(
        "..", "..", "..", "reports", "features.json"
    )
    assert link.read_text() == "{}"


def test_remove_submodels_keeps_files(tmp_path: Path) -> None:
    meta = MetaDataSet(str(tmp_path))
    (tmp_path / "submodels" / "submodel_0000").mkdir()
    (tmp_path / "submodels" / "submodel_0001").mkdir()
    meta.create_image_list([("a.jpg", 0.0, 0.0)])

    meta.remove_submodels()

    assert meta.get_submodel_paths() == []
    assert meta.image_list_exists()
