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

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        meta_data = metadataset.MetaDataSet(args.dataset)

        meta_data.remove_submodels()
        data.invent_reference_lla()

        self._create_image_list(data, meta_data)
        self._cluster_images(meta_data, data.config['submodel_size'])
        self._add_cluster_neighbors(meta_data, data.config['submodel_overlap'])

        meta_data.create_submodels(
            meta_data.load_clusters_with_neighbors(),
            data.config['submodel_use_symlinks'])

    def _create_image_list(self, data, meta_data):
        ills = []
        for image in data.images():
            exif = data.load_exif(image)
            if ('gps' not in exif or
                    'latitude' not in exif['gps'] or
                    'longitude' not in exif['gps']):
                logger.warning(
                    'Skipping {} because of missing GPS'.format(image))
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
