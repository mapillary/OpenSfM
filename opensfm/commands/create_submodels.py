import csv
import cv2
import logging
import numpy as np
import os
import os.path
import scipy.spatial as spatial
from shutil import copyfile

from opensfm import dataset
from opensfm import geo
from opensfm import io

logger = logging.getLogger(__name__)


class Command:
    name = 'create_submodels'
    help = "Split the dataset into smaller submodels"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')
        parser.add_argument('cluster_size', type=int, help='the average cluster size')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        meta_data = MetaDataSet(args.dataset)

        self._create_image_list(data, meta_data)
        self._cluster_images(meta_data, args.cluster_size)
        self._create_clusters_with_neighbors(meta_data)
        self._create_sub_model_structure(meta_data)

    def _create_image_list(self, data, meta_data):
        if not meta_data.image_list_exists():
            ills = []
            for image in data.images():
                exif = data.load_exif(image)
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

        criteria = (cv2.TERM_CRITERIA_MAX_ITER, 20, 1.0)
        flags = cv2.KMEANS_RANDOM_CENTERS

        labels, centers = cv2.kmeans(positions, K, None, criteria, 5, flags)[1:]

        meta_data.save_clusters(images, positions, labels, centers)

    def _create_clusters_with_neighbors(self, meta_data, max_distance=30):
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

    def _create_sub_model_structure(self, meta_data):
        clusters = meta_data.load_clusters_with_neighbors()

        meta_data.create_submodels(clusters)


class MetaDataSet():
    def __init__(self, data_path):
        """
        Create meta dataset instance for large scale reconstruction.

        :param data_path: Path to directory containing meta dataset
        """
        self.data_path = data_path

        self.__submodels_dir_name = 'submodels'
        self.__image_list_file_name = 'image_list_with_gps.csv'
        self.__clusters_file_name = 'clusters.npz'
        self.__clusters_with_neighbors_file_name = 'clusters_with_neighbors.npz'

        io.mkdir_p(self.__submodels_path())

    def __submodels_path(self):
        return os.path.join(self.data_path, self.__submodels_dir_name)

    def __image_list_path(self):
        return os.path.join(self.__submodels_path(), self.__image_list_file_name)

    def __clusters_path(self):
        return os.path.join(self.__submodels_path(), self.__clusters_file_name)

    def __clusters_with_neighbors_path(self):
        return os.path.join(self.__submodels_path(), self.__clusters_with_neighbors_file_name)

    def __create_csv_writer(self, csvfile):
        return csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    def _create_symlink(self, base_path, dir_name):
        images_link_path = os.path.join(base_path, dir_name)
        if os.path.exists(images_link_path):
            os.unlink(images_link_path)

        os.symlink(
            os.path.relpath(os.path.join(self.data_path, dir_name), base_path),
            os.path.join(images_link_path))

    def image_list_exists(self):
        return os.path.isfile(self.__image_list_path())

    def create_image_list(self, ills):
        with open(self.__image_list_path(), 'w') as csvfile:
            w = self.__create_csv_writer(csvfile)

            for image, lat, lon in ills:
                w.writerow([image, lat, lon])

    def images_with_gps(self):
        with open(self.__image_list_path(), 'r') as csvfile:
            image_reader = csv.reader(
                csvfile,
                delimiter=',',
                quotechar='"',
                quoting=csv.QUOTE_MINIMAL)

            for image, lat, lon in image_reader:
                yield image, float(lat), float(lon)

    def save_clusters(self, images, positions, labels, centers):
        filepath = self.__clusters_path()
        np.savez_compressed(
            filepath,
            images=images,
            positions=positions,
            labels=labels,
            centers=centers)

    def load_clusters(self):
        c = np.load(self.__clusters_path())
        return c['images'], c['positions'], c['labels'], c['centers']

    def save_clusters_with_neighbors(self, clusters):
        filepath = self.__clusters_with_neighbors_path()
        np.savez_compressed(
            filepath,
            clusters=clusters)

    def load_clusters_with_neighbors(self):
        c = np.load(self.__clusters_with_neighbors_path())
        return c['clusters']

    def create_submodels(self, clusters):
        for i, cluster in enumerate(clusters):
            # create sub model dir
            submodel_path = os.path.join(self.__submodels_path(), "submodel" + str(i))
            io.mkdir_p(submodel_path)

            # create image list file
            image_list_path = os.path.join(submodel_path, "image_list.txt")
            with open(image_list_path, 'w') as txtfile:
                for image in cluster:
                    txtfile.write("images/{}\n".format(image))

            # copy config.yaml if exists
            config_file_path = os.path.join(self.data_path, 'config.yaml')
            if os.path.exists(config_file_path):
                copyfile(config_file_path, os.path.join(submodel_path, 'config.yaml'))

            # create symlinks to metadata dirs
            self._create_symlink(submodel_path, "images")
            self._create_symlink(submodel_path, "exif")
            self._create_symlink(submodel_path, "features")
            self._create_symlink(submodel_path, "matches")
