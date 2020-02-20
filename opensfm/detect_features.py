import logging
from timeit import default_timer as timer

import numpy as np

from opensfm import bow
from opensfm import dataset
from opensfm import features
from opensfm import io
from opensfm import log
from opensfm.context import parallel_map

logger = logging.getLogger(__name__)

def run(data):
    images = data.images()
    print(data)
    arguments = [(image, data) for image in images]
    
    print(arguments)

    exit()
    start = timer()
    processes = data.config['processes']
    
    print(processes)
    #detect(arguments[0])
    parallel_map(detect, arguments, processes, 1)
    end = timer()
    # with open(data.profile_log(), 'a') as fout:
    #     fout.write('detect_features: {0}\n'.format(end - start))

    # write_report(data, end - start)


def write_report(data, wall_time):
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
    image, data = args
    log.setup()

    need_words = data.config['matcher_type'] == 'WORDS' or data.config['matching_bow_neighbors'] > 0
    #print("need_words===",need_words)
    #has_words = not need_words or data.words_exist(image)
    #has_features = data.features_exist(image)

    # if has_features and has_words:
    #     logger.info('Skip recomputing {} features for image {}'.format(
    #         data.feature_type().upper(), image))
    #     return
    logger.info('Extracting {} features for image {}'.format(
        data.feature_type().upper(), image))
    start = timer()

    #print(image)## 1.jpg
    p_unmasked, f_unmasked, c_unmasked = features.extract_features(
        data.load_image(image), data.config)
    #print(p_unmasked, f_unmasked, c_unmasked)
    fmask = data.load_features_mask(image, p_unmasked)  

    p_unsorted = p_unmasked[fmask]
    f_unsorted = f_unmasked[fmask]
    c_unsorted = c_unmasked[fmask]

    if len(p_unsorted) == 0:
        logger.warning('No features found in image {}'.format(image))
        return

    size = p_unsorted[:, 2]
    order = np.argsort(size)
    p_sorted = p_unsorted[order, :] # points
    f_sorted = f_unsorted[order, :] # descriptors
    c_sorted = c_unsorted[order, :] # colors
    data.save_features(image, p_sorted, f_sorted, c_sorted)

    # if need_words:
    #     bows = bow.load_bows(data.config)
    #     n_closest = data.config['bow_words_to_match']
    #     closest_words = bows.map_to_words(
    #         f_sorted, n_closest, data.config['bow_matcher_type'])
    #     data.save_words(image, closest_words)
    end = timer()
    report = {
        "image": image,
        "num_features": len(p_sorted),
        "wall_time": end - start,
    }
    data.save_report(io.json_dumps(report), 'features/{}.json'.format(image))
