import copy
import logging
import time

from opensfm import dataset
from opensfm import exif


logger = logging.getLogger(__name__)
logging.getLogger("exifread").setLevel(logging.WARNING)


class Command:
    name = 'extract_metadata'
    help = "Extract metadata from images' EXIF tag"

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        start = time.time()
        data = dataset.DataSet(args.dataset)
        # Try not to recreate exif files that already exist
        try:
            camera_models = data.load_camera_models()
            images = data.images_requiring_exif_files()
        except IOError:
            camera_models = {}
            images = data.images()
        for image in images:
            logging.info('Extracting focal lengths for image {}'.format(image))

            # EXIF data in Image
            d = exif.extract_exif_from_file(data.load_image(image))

            # Image Height and Image Width
            if d['width'] <= 0 or not data.config['use_exif_size']:
                d['height'], d['width'] = data.image_as_array(image).shape[:2]

            data.save_exif(image, d)

            if d['camera'] not in camera_models:
                camera = exif.camera_from_exif_metadata(d, data)
                camera_models[d['camera']] = camera

        # Override any camera specified in the camera models overrides file.
        if data.camera_models_overrides_exists():
            overrides = data.load_camera_models_overrides()
            if "all" in overrides:
                for key in camera_models:
                    camera_models[key] = copy.copy(overrides["all"])
                    camera_models[key].id = key
            else:
                for key, value in overrides.items():
                    camera_models[key] = value
        data.save_camera_models(camera_models)

        end = time.time()
        with open(data.profile_log(), 'a') as fout:
            fout.write('focal_from_exif: {0}\n'.format(end - start))
