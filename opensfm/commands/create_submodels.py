import logging
import numpy as np

from opensfm import dataset
from opensfm.large import metadataset
from opensfm.large import tools

logger = logging.getLogger(__name__)


class Command:
    name = 'create_submodels'
    help = 'Split the dataset into smaller submodels'

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')
        parser.add_argument('-s', '--size', type=int, default=50,
                            help='the average cluster size')
        parser.add_argument('-d', '--dist', type=int, default=30,
                            help='the max distance in meters to a neighbor ' \
                            'for it to be included in the cluster')
        parser.add_argument('-n', '--no-symlinks',
                            action='store_true',
                            help='Do not create any symlinks, ' \
                            'every submodels needs to run complete pipeline')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        meta_data = metadataset.MetaDataSet(args.dataset)

        meta_data.remove_submodels()
        data.invent_reference_lla()

        self._create_image_list(data, meta_data)
        self._cluster_images(meta_data, args.size)
        self._add_cluster_neighbors(meta_data, args.dist)

        meta_data.create_submodels(
            meta_data.load_clusters_with_neighbors(), args.no_symlinks)

    def _create_image_list(self, data, meta_data):
        ills = []
        for image in data.images():
            exif = data.load_exif(image)
            if 'gps' not in exif or \
                'latitude' not in exif['gps'] or \
                'longitude' not in exif['gps']:
                logger.warning('Skipping {} because of missing GPS'.format(image))
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

        labels, centers = tools.kmeans(positions, K)[1:]

        meta_data.save_clusters(images, positions, labels, centers)

    def _add_cluster_neighbors(self, meta_data, max_distance):
        images, positions, labels, centers = meta_data.load_clusters()
        clusters = tools.add_cluster_neighbors(
            positions, labels, centers, max_distance)

        image_clusters = []
        for cluster in clusters:
            image_clusters.append(list(np.take(images, np.array(cluster))))

        meta_data.save_clusters_with_neighbors(image_clusters)
