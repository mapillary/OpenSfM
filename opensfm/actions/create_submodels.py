import logging
from collections import defaultdict

import numpy as np
from opensfm.dataset import DataSet
from opensfm.large.metadataset import MetaDataSet
from opensfm.large import tools


logger = logging.getLogger(__name__)


def run_dataset(data: DataSet):
    """ Split the dataset into smaller submodels. """

    meta_data = MetaDataSet(data.data_path)

    meta_data.remove_submodels()
    data.invent_reference_lla()
    _create_image_list(data, meta_data)

    if meta_data.image_groups_exists():
        _read_image_groups(meta_data)
    else:
        _cluster_images(meta_data, data.config["submodel_size"])

    _add_cluster_neighbors(meta_data, data.config["submodel_overlap"])
    _save_clusters_geojson(meta_data)
    _save_cluster_neighbors_geojson(meta_data)

    meta_data.create_submodels(meta_data.load_clusters_with_neighbors())


def _create_image_list(data: DataSet, meta_data):
    ills = []
    for image in data.images():
        exif = data.load_exif(image)
        if (
            "gps" not in exif
            or "latitude" not in exif["gps"]
            or "longitude" not in exif["gps"]
        ):
            logger.warning("Skipping {} because of missing GPS".format(image))
            continue

        lat = exif["gps"]["latitude"]
        lon = exif["gps"]["longitude"]
        ills.append((image, lat, lon))

    meta_data.create_image_list(ills)


def _read_image_groups(meta_data: MetaDataSet):
    image_cluster = {}
    cluster_images = defaultdict(list)
    for image, cluster in meta_data.load_image_groups():
        image_cluster[image] = cluster
        cluster_images[cluster].append(image)
    K = len(cluster_images)
    cluster_index = dict(zip(sorted(cluster_images.keys()), range(K)))

    images = []
    positions = []
    labels = []
    centers = np.zeros((K, 2))
    centers_count = np.zeros((K, 1))
    for image, lat, lon in meta_data.images_with_gps():
        images.append(image)
        positions.append([lat, lon])
        cluster = image_cluster[image]
        label = cluster_index[cluster]
        labels.append(label)
        centers[label, 0] += lat
        centers[label, 1] += lon
        centers_count[label] += 1

    images = np.array(images)
    positions = np.array(positions, np.float32)
    labels = np.array(labels)
    centers /= centers_count

    meta_data.save_clusters(images, positions, labels, centers)


def _cluster_images(meta_data: MetaDataSet, cluster_size):
    images = []
    positions = []
    for image, lat, lon in meta_data.images_with_gps():
        images.append(image)
        positions.append([lat, lon])

    positions = np.array(positions, np.float32)
    images = np.array(images).reshape((len(images), 1))

    K = float(images.shape[0]) / cluster_size
    K = int(np.ceil(K))

    labels, centers = tools.kmeans(positions, K)[1:]

    images = images.ravel()
    labels = labels.ravel()

    meta_data.save_clusters(images, positions, labels, centers)


def _add_cluster_neighbors(meta_data: MetaDataSet, max_distance):
    images, positions, labels, centers = meta_data.load_clusters()
    clusters = tools.add_cluster_neighbors(positions, labels, centers, max_distance)

    image_clusters = []
    for cluster in clusters:
        image_clusters.append(list(np.take(images, np.array(cluster))))

    meta_data.save_clusters_with_neighbors(image_clusters)


def _save_cluster_neighbors_geojson(meta_data: MetaDataSet):
    image_coordinates = {}
    for image, lat, lon in meta_data.images_with_gps():
        image_coordinates[image] = [lon, lat]

    features = []
    clusters = meta_data.load_clusters_with_neighbors()
    for cluster_idx, images in enumerate(clusters):
        for image in images:
            features.append(
                {
                    "type": "Feature",
                    "geometry": {
                        "type": "Point",
                        "coordinates": image_coordinates[image],
                    },
                    "properties": {"name": image, "submodel": cluster_idx},
                }
            )
    geojson = {"type": "FeatureCollection", "features": features}
    meta_data.save_cluster_with_neighbors_geojson(geojson)


def _save_clusters_geojson(meta_data: MetaDataSet):
    image_coordinates = {}
    for image, lat, lon in meta_data.images_with_gps():
        image_coordinates[image] = [lon, lat]

    features = []
    images, positions, labels, centers = meta_data.load_clusters()
    for image, label in zip(images, labels):
        features.append(
            {
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": image_coordinates[image]},
                "properties": {"name": image, "submodel": int(label)},  # cluster_idx
            }
        )
    geojson = {"type": "FeatureCollection", "features": features}
    meta_data.save_clusters_geojson(geojson)
