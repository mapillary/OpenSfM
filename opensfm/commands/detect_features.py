import logging
from timeit import default_timer as timer

import numpy as np

from opensfm import dataset
from opensfm import features
from opensfm import io
from opensfm import log
from opensfm.context import parallel_map

logger = logging.getLogger(__name__)


class Command:
    name = 'detect_features'
    help = 'Compute features for all images'

    def add_arguments(self, parser):
        parser.add_argument('dataset', help='dataset to process')

    def run(self, args):
        data = dataset.DataSet(args.dataset)
        images = data.images()

        arguments = [(image, data) for image in images]

        start = timer()
        processes = data.config['processes']
        parallel_map(detect, arguments, processes)
        end = timer()
        with open(data.profile_log(), 'a') as fout:
            fout.write('detect_features: {0}\n'.format(end - start))

        self.write_report(data, end - start)

    def write_report(self, data, wall_time):
        image_reports = []
        for image in data.images():
            try:
                txt = data.load_report('features/{}.json'.format(image))
                image_reports.append(io.json_loads(txt))
            except IOError:
                logger.warning('No feature report image {}'.format(image))

        report = {
            "wall_time": wall_time,
            "image_reports": image_reports
        }
        data.save_report(io.json_dumps(report), 'features.json')


def detect(args):
    log.setup()

    image, data = args
    logger.info('Extracting {} features for image {}'.format(
        data.feature_type().upper(), image))

    if not data.feature_index_exists(image):
        start = timer()
        mask = data.mask_as_array(image)
        if mask is not None:
            logger.info('Found mask to apply for image {}'.format(image))
        preemptive_max = data.config['preemptive_max']
        p_unsorted, f_unsorted, c_unsorted = features.extract_features(
            data.image_as_array(image), data.config, mask)
        if len(p_unsorted) == 0:
            return

        size = p_unsorted[:, 2]
        order = np.argsort(size)
        p_sorted = p_unsorted[order, :]
        f_sorted = f_unsorted[order, :]
        c_sorted = c_unsorted[order, :]
        p_pre = p_sorted[-preemptive_max:]
        f_pre = f_sorted[-preemptive_max:]
        data.save_features(image, p_sorted, f_sorted, c_sorted)
        data.save_preemptive_features(image, p_pre, f_pre)

        if data.config['matcher_type'] == 'FLANN':
            index = features.build_flann_index(f_sorted, data.config)
            data.save_feature_index(image, index)

        end = timer()
        report = {
            "image": image,
            "num_features": len(p_sorted),
            "wall_time": end - start,
        }
        data.save_report(io.json_dumps(report),
                         'features/{}.json'.format(image))
