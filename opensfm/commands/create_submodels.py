import cv2
import logging
import numpy as np
import scipy.spatial as spatial

from opensfm import dataset
from opensfm import geo
from opensfm.large import metadataset
from opensfm.large import tools

logger = logging.getLogger(__name__)


class Command:
    name = 'create_submodels'
    help = 'Split the dataset into smaller submodels'

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')
        parser.add_argument('-s', '--cluster-size', type=int, default=50,
                            help='the average cluster size')
        parser.add_argument('-d', '--nghbr-dist', type=int, default=30,
                            help='the max distance in meters to a neighbor for it to be included in the cluster')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        meta_data = metadataset.MetaDataSet(args.dataset)

        self._create_reference_lla(data)
        self._create_image_list(data, meta_data)
        self._cluster_images(meta_data, args.cluster_size)
        self._create_clusters_with_neighbors(meta_data, args.nghbr_dist)
        self._remove_submodel_structure(meta_data)
        self._create_sub_model_structure(meta_data)

    def _create_image_list(self, data, meta_data):
        ills = []
        for image in data.images():
            exif = data.load_exif(image)
            if 'gps' not in exif or \
                'latitude' not in exif['gps'] or \
                'longitude' not in exif['gps']:
                logger.info('Skipping {} because of missing GPS'.format(image))
                continue

            lat = exif['gps']['latitude']
            lon = exif['gps']['longitude']
            ills.append((image, lat, lon))

        meta_data.create_image_list(ills)

    def _cluster_images(self, meta_data, cluster_size):
        images = []
        positions = []
        for image, lat, lon in meta_data.images_with_gps():
            images.append(image)
            positions.append([lat, lon])

        positions = np.array(positions, np.float32)
        images = np.array(images).reshape((len(images), 1))

        K = float(images.shape[0]) / cluster_size
        K = int(np.ceil(K))

        criteria = (cv2.TERM_CRITERIA_MAX_ITER, 100, 1.0)
        flags = cv2.KMEANS_PP_CENTERS

        labels, centers = tools.kmeans(positions, K, criteria, 20, flags)[1:]

        meta_data.save_clusters(images, positions, labels, centers)

    def _create_clusters_with_neighbors(self, meta_data, max_distance):
        images, positions, labels, centers = meta_data.load_clusters()

        reference = np.mean(positions, 0)

        topocentrics = []
        for position in positions:
            x, y, z = geo.topocentric_from_lla(
                position[0],
                position[1],
                0,
                reference[0],
                reference[1],
                0)

            topocentrics.append([x, y])

        topocentrics = np.array(topocentrics)
        topo_tree = spatial.cKDTree(topocentrics)

        clusters = []
        for label in np.arange(centers.shape[0]):
            cluster_indices = np.where(labels.ravel() == label)[0]

            neighbors = []
            for i in cluster_indices:
                neighbors.extend(topo_tree.query_ball_point(topocentrics[i], max_distance))

            cluster = np.union1d(cluster_indices, neighbors)
            clusters.append(list(np.take(images, cluster)))

        meta_data.save_clusters_with_neighbors(clusters)

    def _create_reference_lla(self, data):
        data.invent_reference_lla()

    def _remove_submodel_structure(self, meta_data):
        meta_data.remove_submodels()

    def _create_sub_model_structure(self, meta_data):
        clusters = meta_data.load_clusters_with_neighbors()

        meta_data.create_submodels(clusters)
