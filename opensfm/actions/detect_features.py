import itertools
import logging
import math
import queue
import threading
from timeit import default_timer as timer
from typing import Optional, List, Dict, Any, Tuple

import numpy as np
from opensfm import bow, features, io, log, pygeometry, upright
from opensfm.context import parallel_map
from opensfm.dataset import DataSetBase

logger = logging.getLogger(__name__)


def run_dataset(data: DataSetBase):
    """Compute features for all images."""

    start = timer()

    default_queue_size = 10
    max_queue_size = 200
    mem_available = log.memory_available()
    if mem_available:
        expected_mb = mem_available / 2
        expected_images = min(
            max_queue_size, int(expected_mb / average_image_size(data))
        )
        logger.info(f"Capping memory usage to ~ {expected_mb} MB")
    else:
        expected_images = default_queue_size
    logger.info(f"Expecting to process {expected_images} images.")

    process_queue = queue.Queue(expected_images)
    arguments: List[Tuple[str, Any]] = []

    all_images = data.images()
    processes = data.config["processes"]

    if processes == 1:
        for image in all_images:
            counter = Counter()
            read_images(process_queue, data, [image], counter, 1)
            run_detection(process_queue)
            process_queue.get()
    else:
        counter = Counter()
        read_processes = data.config["read_processes"]
        if 1.5 * read_processes >= processes:
            read_processes = max(1, processes // 2)

        chunk_size = math.ceil(len(all_images) / read_processes)
        chunks_count = math.ceil(len(all_images) / chunk_size)
        read_processes = min(read_processes, chunks_count)

        expected: int = len(all_images)
        for i in range(read_processes):
            images_chunk = all_images[i * chunk_size : (i + 1) * chunk_size]
            arguments.append(
                (
                    "producer",
                    (process_queue, data, images_chunk, counter, expected),
                )
            )
        for _ in range(processes):
            arguments.append(("consumer", (process_queue)))
        parallel_map(process, arguments, processes, 1)

    end = timer()
    write_report(data, end - start)


def average_image_size(data: DataSetBase) -> float:
    average_size_mb = 0
    for camera in data.load_camera_models().values():
        average_size_mb += camera.width * camera.height * 4 / 1024 / 1024
    return average_size_mb / len(data.load_camera_models())


def write_report(data: DataSetBase, wall_time: float):
    image_reports = []
    for image in data.images():
        try:
            txt = data.load_report("features/{}.json".format(image))
            image_reports.append(io.json_loads(txt))
        except IOError:
            logger.warning("No feature report image {}".format(image))

    report = {"wall_time": wall_time, "image_reports": image_reports}
    data.save_report(io.json_dumps(report), "features.json")


def is_high_res_panorama(data: DataSetBase, image_key: str, image_array: np.ndarray):
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


class Counter(object):
    """Lock-less counter from https://julien.danjou.info/atomic-lock-free-counters-in-python/
    that relies on the CPython impl. of itertools.count() that is thread-safe. Used, as for
    some reason, joblib doesn't like a good old threading.Lock (everything is stuck)
    """

    def __init__(self):
        self.number_of_read = 0
        self.counter = itertools.count()
        self.read_lock = threading.Lock()

    def increment(self):
        next(self.counter)

    def value(self):
        with self.read_lock:
            value = next(self.counter) - self.number_of_read
            self.number_of_read += 1
        return value


def process(args: Tuple[str, Any]):
    process_type, real_args = args
    if process_type == "producer":
        queue, data, images, counter, expected = real_args
        read_images(queue, data, images, counter, expected)
    if process_type == "consumer":
        queue = real_args
        run_detection(queue)


def read_images(
    queue: queue.Queue,
    data: DataSetBase,
    images: List[str],
    counter: Counter,
    expected: int,
):
    full_queue_timeout = 120
    for image in images:
        logger.info(f"Reading data for image {image} (queue-size={queue.qsize()}")
        image_array = data.load_image(image)
        if data.config["features_bake_segmentation"]:
            segmentation_array = data.load_segmentation(image)
            instances_array = data.load_instances(image)
        else:
            segmentation_array, instances_array = None, None
        args = image, image_array, segmentation_array, instances_array, data
        queue.put(args, block=True, timeout=full_queue_timeout)
        counter.increment()
        if counter.value() == expected:
            logger.info("Finished reading images")
            queue.put(None)


def run_detection(queue: queue.Queue):
    while True:
        args = queue.get()
        if args is None:
            queue.put(None)
            break
        image, image_array, segmentation_array, instances_array, data = args
        detect(image, image_array, segmentation_array, instances_array, data)
        del image_array
        del segmentation_array
        del instances_array


def bake_segmentation(
    points: np.ndarray,
    segmentation: Optional[np.ndarray],
    instances: Optional[np.ndarray],
    exif: Dict[str, Any],
):
    panoptic_data = [None, None]
    for i, p_data in enumerate([segmentation, instances]):
        if p_data is None:
            continue
        new_height, new_width = p_data.shape
        ps = upright.opensfm_to_upright(
            points[:, :2],
            exif["width"],
            exif["height"],
            exif["orientation"],
            new_width=new_width,
            new_height=new_height,
        ).astype(int)
        panoptic_data[i] = p_data[ps[:, 1], ps[:, 0]]
    return panoptic_data


def detect(
    image: str,
    image_array: np.ndarray,
    segmentation_array: Optional[np.ndarray],
    instances_array: Optional[np.ndarray],
    data: DataSetBase,
):
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

    p_unmasked, f_unmasked, c_unmasked = features.extract_features(
        image_array, data.config, is_high_res_panorama(data, image, image_array)
    )

    # Load segmentation and bake it in the data
    if data.config["features_bake_segmentation"]:
        exif = data.load_exif(image)
        s_unsorted, i_unsorted = bake_segmentation(
            p_unmasked, segmentation_array, instances_array, exif
        )
        p_unsorted = p_unmasked
        f_unsorted = f_unmasked
        c_unsorted = c_unmasked
    # Load segmentation, make a mask from it mask and apply it
    else:
        s_unsorted, i_unsorted = None, None
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
    if s_unsorted is not None and i_unsorted is not None:
        semantic_data = features.SemanticData(
            s_unsorted[order], i_unsorted[order], data.segmentation_labels()
        )
    else:
        semantic_data = None
    features_data = features.FeaturesData(p_sorted, f_sorted, c_sorted, semantic_data)
    data.save_features(image, features_data)

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
