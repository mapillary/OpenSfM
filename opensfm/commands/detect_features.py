import logging
from multiprocessing import Pool
import time

import numpy as np

from opensfm import dataset
from opensfm import features

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

        start = time.time()
        processes = data.config.get('processes', 1)
        if processes == 1:
            for arg in arguments:
                detect(arg)
        else:
            p = Pool(processes)
            p.map(detect, arguments)
        end = time.time()
        with open(data.profile_log(), 'a') as fout:
            fout.write('detect_features: {0}\n'.format(end - start))


def detect(args):
    image, data = args
    logger.info('Extracting {} features for image {}'.format(
        data.feature_type().upper(), image))

    if not data.feature_index_exists(image):
        mask = data.mask_as_array(image)
        if mask is not None:
            logger.info('Found mask to apply for image {}'.format(image))
        preemptive_max = data.config.get('preemptive_max', 200)
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

        if data.config.get('matcher_type', 'FLANN') == 'FLANN':
            index = features.build_flann_index(f_sorted, data.config)
            data.save_feature_index(image, index)
