# pyre-strict
import glob
import os
import os.path
import shutil
import sys
from typing import Any, Dict, Generator, List, Tuple

import numpy as np
from numpy.typing import NDArray
from opensfm import config, io
from opensfm.dataset import DataSet


class MetaDataSet:
    def __init__(self, data_path: str) -> None:
        """
        Create meta dataset instance for large scale reconstruction.

        :param data_path: Path to directory containing meta dataset
        """
        self.data_path: str = os.path.abspath(data_path)

        config_file = os.path.join(self.data_path, "config.yaml")
        self.config: Dict[str, Any] = config.load_config(config_file)

        self._image_list_file_name = "image_list_with_gps.tsv"
        self._clusters_file_name = "clusters.npz"
        self._clusters_with_neighbors_file_name = "clusters_with_neighbors.npz"
        self._clusters_with_neighbors_geojson_file_name = (
            "clusters_with_neighbors.geojson"
        )
        self._clusters_geojson_file_name = "clusters.geojson"

        io.mkdir_p(self._submodels_path())

    def _submodels_path(self) -> str:
        return os.path.join(self.data_path, self.config["submodels_relpath"])

    def _submodel_path(self, i: int) -> str:
        """Path to submodel i folder."""
        template = self.config["submodel_relpath_template"]
        return os.path.join(self.data_path, template % i)

    def _submodel_images_path(self, i: int) -> str:
        """Path to submodel i images folder."""
        template = self.config["submodel_images_relpath_template"]
        return os.path.join(self.data_path, template % i)

    def _image_groups_path(self) -> str:
        return os.path.join(self.data_path, "image_groups.txt")

    def _image_list_path(self) -> str:
        return os.path.join(self._submodels_path(), self._image_list_file_name)

    def _clusters_path(self) -> str:
        return os.path.join(self._submodels_path(), self._clusters_file_name)

    def _clusters_with_neighbors_path(self) -> str:
        return os.path.join(
            self._submodels_path(), self._clusters_with_neighbors_file_name
        )

    def _clusters_with_neighbors_geojson_path(self) -> str:
        return os.path.join(
            self._submodels_path(), self._clusters_with_neighbors_geojson_file_name
        )

    def _clusters_geojson_path(self) -> str:
        return os.path.join(self._submodels_path(), self._clusters_geojson_file_name)

    def _create_symlink(self, base_path: str, file_path: str) -> None:
        src = os.path.join(self.data_path, file_path)
        dst = os.path.join(base_path, file_path)

        if not os.path.exists(src):
            return

        # Symlinks on Windows require admin privileges,
        # so we use hard links instead
        if sys.platform == "win32":
            if os.path.isdir(dst):
                shutil.rmtree(dst)
            elif os.path.isfile(dst):
                os.remove(dst)
        else:
            if os.path.islink(dst):
                os.unlink(dst)

        subfolders = len(file_path.split(os.path.sep)) - 1

        if sys.platform == "win32":
            if os.path.isdir(src):
                # Create directory in destination, then make hard links
                # to files
                os.mkdir(dst)

                for f in glob.glob(os.path.join(src, "*")):
                    filename = os.path.basename(f)
                    os.link(f, os.path.join(dst, filename))
            else:
                # Just make hard link
                os.link(src, dst)
        else:
            os.symlink(
                os.path.join(*[".."] * subfolders, os.path.relpath(src, base_path)), dst
            )

    def image_groups_exists(self) -> bool:
        return os.path.isfile(self._image_groups_path())

    def load_image_groups(self) -> Generator[List[str], None, None]:
        with open(self._image_groups_path()) as fin:
            for line in fin:
                yield line.split()

    def image_list_exists(self) -> bool:
        return os.path.isfile(self._image_list_path())

    def create_image_list(self, ills: List[Tuple[str, float, float]]) -> None:
        with io.open_wt(self._image_list_path()) as csvfile:
            for image, lat, lon in ills:
                csvfile.write("{}\t{}\t{}\n".format(image, lat, lon))

    def images_with_gps(self) -> Generator[Tuple[str, float, float], None, None]:
        with io.open_rt(self._image_list_path()) as csvfile:
            for line in csvfile:
                image, lat, lon = line.split("\t")
                yield image, float(lat), float(lon)

    def save_clusters(
        self, images: NDArray, positions: NDArray, labels: NDArray, centers: NDArray
    ) -> None:
        filepath = self._clusters_path()
        np.savez_compressed(
            filepath, images=images, positions=positions, labels=labels, centers=centers
        )

    def load_clusters(self) -> Tuple[NDArray, NDArray, NDArray, NDArray]:
        c = np.load(self._clusters_path())
        images = c["images"].ravel()
        labels = c["labels"].ravel()
        return images, c["positions"], labels, c["centers"]

    def save_clusters_with_neighbors(self, clusters: List[List[int]]) -> None:
        filepath = self._clusters_with_neighbors_path()
        np.savez_compressed(filepath, clusters=np.array(clusters, dtype=object))

    def load_clusters_with_neighbors(self) -> NDArray:
        c = np.load(self._clusters_with_neighbors_path(), allow_pickle=True)
        return c["clusters"]

    def save_cluster_with_neighbors_geojson(self, geojson: Dict[str, Any]) -> None:
        filepath = self._clusters_with_neighbors_geojson_path()
        with io.open_wt(filepath) as fout:
            io.json_dump(geojson, fout)

    def save_clusters_geojson(self, geojson: Dict[str, Any]) -> None:
        filepath = self._clusters_geojson_path()
        with io.open_wt(filepath) as fout:
            io.json_dump(geojson, fout)

    def remove_submodels(self) -> None:
        sm = self._submodels_path()
        paths = [
            os.path.join(sm, o)
            for o in os.listdir(sm)
            if os.path.isdir(os.path.join(sm, o))
        ]
        for path in paths:
            shutil.rmtree(path)

    def create_submodels(self, clusters: NDArray) -> None:
        data = DataSet(self.data_path)
        for i, cluster in enumerate(clusters):
            # create sub model dirs
            submodel_path = self._submodel_path(i)
            submodel_images_path = self._submodel_images_path(i)
            io.mkdir_p(submodel_path)
            io.mkdir_p(submodel_images_path)

            # create image list file
            image_list_path = os.path.join(submodel_path, "image_list.txt")
            with io.open_wt(image_list_path) as txtfile:
                for image in cluster:
                    src = data.image_files[image]
                    dst = os.path.join(submodel_images_path, image)
                    if not os.path.isfile(dst):
                        if sys.platform == "win32":
                            os.link(src, dst)
                        else:
                            os.symlink(os.path.relpath(src, submodel_images_path), dst)
                    dst_relpath = os.path.relpath(dst, submodel_path)
                    txtfile.write(dst_relpath + "\n")

            # copy config.yaml if exists
            config_file_path = os.path.join(self.data_path, "config.yaml")
            if os.path.exists(config_file_path):
                shutil.copyfile(
                    config_file_path, os.path.join(submodel_path, "config.yaml")
                )

            # Create reports folder
            io.mkdir_p(os.path.join(submodel_path, "reports"))

            # create symlinks to additional files
            filepaths = [
                "camera_models.json",
                "reference_lla.json",
                "exif",
                "features",
                "matches",
                "masks",
                "mask_list.txt",
                "segmentations",
                os.path.join("reports", "features"),
                os.path.join("reports", "features.json"),
                os.path.join("reports", "matches.json"),
            ]
            for filepath in filepaths:
                self._create_symlink(submodel_path, filepath)

    def get_submodel_paths(self) -> List[str]:
        submodel_paths = []
        for i in range(999999):
            submodel_path = self._submodel_path(i)
            if os.path.isdir(submodel_path):
                submodel_paths.append(submodel_path)
            else:
                break
        return submodel_paths
