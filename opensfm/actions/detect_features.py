import logging
from timeit import default_timer as timer

import numpy as np
from opensfm import bow, features, io, log, pygeometry
from opensfm.context import parallel_map


logger = logging.getLogger(__name__)


def run_dataset(data):
    """ Compute features for all images. """

    images = data.images()

    arguments = [(image, data) for image in images]

    start = timer()
    processes = data.config["processes"]
    parallel_map(detect, arguments, processes, 1)
    end = timer()
    write_report(data, end - start)


def write_report(data, wall_time):
    image_reports = []
    for image in data.images():
        try:
            txt = data.load_report("features/{}.json".format(image))
            image_reports.append(io.json_loads(txt))
        except IOError:
            logger.warning("No feature report image {}".format(image))

    report = {"wall_time": wall_time, "image_reports": image_reports}
    data.save_report(io.json_dumps(report), "features.json")


def is_high_res_panorama(data, image_key, image_array):
    """Detect if image is a panorama."""
    exif = data.load_exif(image_key)
    if exif:
        camera = data.load_camera_models()[exif["camera"]]
        w, h = int(exif["width"]), int(exif["height"])
        exif_pano = pygeometry.Camera.is_panorama(camera.projection_type)
    elif image_array is not None:
        h, w = image_array.shape[:2]
        exif_pano = False
    else:
        return False
    return w == 2 * h or exif_pano


def detect(args):
    image, data = args

    log.setup()

    need_words = (
        data.config["matcher_type"] == "WORDS"
        or data.config["matching_bow_neighbors"] > 0
    )
    has_words = not need_words or data.words_exist(image)
    has_features = data.features_exist(image)

    if has_features and has_words:
        logger.info(
            "Skip recomputing {} features for image {}".format(
                data.feature_type().upper(), image
            )
        )
        return

    logger.info(
        "Extracting {} features for image {}".format(data.feature_type().upper(), image)
    )

    start = timer()

    image_array = data.load_image(image)
    p_unmasked, f_unmasked, c_unmasked = features.extract_features(
        image_array, data.config, is_high_res_panorama(data, image, image_array)
    )

    fmask = data.load_features_mask(image, p_unmasked)

    p_unsorted = p_unmasked[fmask]
    f_unsorted = f_unmasked[fmask]
    c_unsorted = c_unmasked[fmask]

    if len(p_unsorted) == 0:
        logger.warning("No features found in image {}".format(image))

    size = p_unsorted[:, 2]
    order = np.argsort(size)
    p_sorted = p_unsorted[order, :]
    f_sorted = f_unsorted[order, :]
    c_sorted = c_unsorted[order, :]
    data.save_features(image, p_sorted, f_sorted, c_sorted)

    if need_words:
        bows = bow.load_bows(data.config)
        n_closest = data.config["bow_words_to_match"]
        closest_words = bows.map_to_words(
            f_sorted, n_closest, data.config["bow_matcher_type"]
        )
        data.save_words(image, closest_words)

    end = timer()
    report = {
        "image": image,
        "num_features": len(p_sorted),
        "wall_time": end - start,
    }
    data.save_report(io.json_dumps(report), "features/{}.json".format(image))
