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

        if not meta_data.image_list_exists():
            ills = []
            for image in data.images():
                exif_data = EXIF(data.load_image(image))
                lon, lat = exif_data.extract_lon_lat()
                ills.append((image, lon, lat))

            meta_data.create_image_list(ills)


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

    def __submodels_path(self):
        return os.path.join(self.data_path, self.__submodels_folder_name)

    def __image_list_path(self):
        return os.path.join(self.__submodels_path(), self.__image_list_file_name)

    def __create_csv_writer(self, csvfile):
        return csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)

    def image_list_exists(self):
        return os.path.isfile(self.__image_list_path())

    def create_image_list(self, ills):
        with open(self.__image_list_path(), 'a') as csvfile:
            w = self.__create_csv_writer(csvfile)

            for image, lon, lat in ills:
                w.writerow([image, lon, lat])

    def images_with_gps(self):
        with open(self.__image_list_path(), 'r') as csvfile:
            image_reader = csv.reader(
                csvfile,
                delimiter=',',
                quotechar='"',
                quoting=csv.QUOTE_MINIMAL)

            for image, lon, lat in image_reader:
                yield image, float(lon), float(lat)
