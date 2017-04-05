import csv
import logging
import os.path

from opensfm import dataset
from opensfm.exif import EXIF
from opensfm import io

logger = logging.getLogger(__name__)


class Command:
    name = 'create_submodels'
    help = "Split the dataset into smaller submodels"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        meta_data = MetaDataSet(args.dataset)

        for image in data.images():
            exif_data = EXIF(data.load_image(image))
            lon, lat = exif_data.extract_lon_lat()

            meta_data.add_image_to_list(image, lon, lat)


class MetaDataSet():
    def __init__(self, data_path):
        """
        Create meta dataset instance for large scale reconstruction.

        :param data_path: Path to directory containing meta dataset
        """
        self.data_path = data_path

        self.__submodels_folder_name = 'submodels'

        self.__image_list_file_name = 'image_list_with_gps.csv'

        io.mkdir_p(self.__submodels_path())
        if not os.path.isfile(self.__image_list_path()):
            with open(self.__image_list_path(), 'wb') as f_out:
                w = self.__create_csv_writer(f_out)
                w.writerow(['name', 'lon', 'lat'])

    def __submodels_path(self):
        return os.path.join(self.data_path, self.__submodels_folder_name)

    def __image_list_path(self):
        return os.path.join(self.__submodels_path(), self.__image_list_file_name)

    def __create_csv_writer(self, f):
        return csv.writer(f, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    def add_image_to_list(self, image, lon, lat):
        with open(self.__image_list_path(), 'a') as f_out:
            w = self.__create_csv_writer(f_out)
            w.writerow([image, lon, lat])
