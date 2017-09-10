import csv
import numpy as np
import os
import os.path
import shutil

from opensfm import config
from opensfm import io
from opensfm.dataset import DataSet


class MetaDataSet():
    def __init__(self, data_path):
        '''
        Create meta dataset instance for large scale reconstruction.

        :param data_path: Path to directory containing meta dataset
        '''
        self.data_path = data_path

        config_file = os.path.join(self.data_path, 'config.yaml')
        self.config = config.load_config(config_file)

        self._image_list_file_name = 'image_list_with_gps.tsv'
        self._clusters_file_name = 'clusters.npz'
        self._clusters_with_neighbors_file_name = 'clusters_with_neighbors.npz'

        io.mkdir_p(self._submodels_path())

    def _submodels_path(self):
        return os.path.join(self.data_path, self.config['submodels_relpath'])

    def _submodel_path(self, i):
        """Path to submodel i folder."""
        template = self.config['submodel_relpath_template']
        return os.path.join(self.data_path, template % i)

    def _submodel_images_path(self, i):
        """Path to submodel i images folder."""
        template = self.config['submodel_images_relpath_template']
        return os.path.join(self.data_path, template % i)

    def _image_list_path(self):
        return os.path.join(self._submodels_path(), self._image_list_file_name)

    def _clusters_path(self):
        return os.path.join(self._submodels_path(), self._clusters_file_name)

    def _clusters_with_neighbors_path(self):
        return os.path.join(self._submodels_path(), self._clusters_with_neighbors_file_name)

    def _create_csv_writer(self, csvfile):
        return csv.writer(csvfile, delimiter='\t', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    def _create_symlink(self, base_path, dir_name):
        link_path = os.path.join(base_path, dir_name)

        if os.path.islink(link_path):
            os.unlink(link_path)

        os.symlink(
            os.path.relpath(os.path.join(self.data_path, dir_name), base_path),
            os.path.join(link_path))

    def image_list_exists(self):
        return os.path.isfile(self._image_list_path())

    def create_image_list(self, ills):
        with open(self._image_list_path(), 'w') as csvfile:
            w = self._create_csv_writer(csvfile)

            for image, lat, lon in ills:
                w.writerow([image, lat, lon])

    def images_with_gps(self):
        with open(self._image_list_path(), 'r') as csvfile:
            image_reader = csv.reader(
                csvfile,
                delimiter='\t',
                quotechar='"',
                quoting=csv.QUOTE_MINIMAL)

            for image, lat, lon in image_reader:
                yield image, float(lat), float(lon)

    def save_clusters(self, images, positions, labels, centers):
        filepath = self._clusters_path()
        np.savez_compressed(
            filepath,
            images=images,
            positions=positions,
            labels=labels,
            centers=centers)

    def load_clusters(self):
        c = np.load(self._clusters_path())
        return c['images'], c['positions'], c['labels'], c['centers']

    def save_clusters_with_neighbors(self, clusters):
        filepath = self._clusters_with_neighbors_path()
        np.savez_compressed(
            filepath,
            clusters=clusters)

    def load_clusters_with_neighbors(self):
        c = np.load(self._clusters_with_neighbors_path())
        return c['clusters']

    def remove_submodels(self):
        sm = self._submodels_path()
        paths = [os.path.join(sm, o) for o in os.listdir(sm) if os.path.isdir(os.path.join(sm, o))]
        for path in paths:
            shutil.rmtree(path)

    def create_submodels(self, clusters, no_symlinks=False):
        data = DataSet(self.data_path)
        for i, cluster in enumerate(clusters):
            # create sub model dirs
            submodel_path = self._submodel_path(i)
            submodel_images_path = self._submodel_images_path(i)
            io.mkdir_p(submodel_path)
            io.mkdir_p(submodel_images_path)

            # link images and create image list file
            image_list_path = os.path.join(submodel_path, 'image_list.txt')
            with open(image_list_path, 'w') as txtfile:
                for image in cluster:
                    src = data.image_files[image]
                    dst = os.path.join(submodel_images_path, image)
                    if not os.path.isfile(dst):
                        os.symlink(src, dst)
                    dst_relpath = os.path.relpath(dst, submodel_path)
                    txtfile.write(dst_relpath + "\n")

            # copy config.yaml if exists
            config_file_path = os.path.join(self.data_path, 'config.yaml')
            if os.path.exists(config_file_path):
                shutil.copyfile(config_file_path, os.path.join(submodel_path, 'config.yaml'))

            if no_symlinks:
                reference_file_path = os.path.join(self.data_path, 'reference_lla.json')
                if os.path.exists(reference_file_path):
                    shutil.copyfile(reference_file_path, os.path.join(submodel_path, 'reference_lla.json'))
            else:
                # create symlinks to metadata files
                for symlink_path in ['camera_models.json', 'reference_lla.json',
                                     'exif', 'features', 'matches']:
                    self._create_symlink(submodel_path, symlink_path)

    def get_submodel_paths(self):
        submodel_paths = []
        for i in range(999999):
            submodel_path = self._submodel_path(i)
            if os.path.isdir(submodel_path):
                submodel_paths.append(submodel_path)
            else:
                break
        return submodel_paths
