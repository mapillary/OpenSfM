# pyre-unsafe
import itertools
import logging
import math
import queue
import threading
from timeit import default_timer as timer
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from opensfm import bow, features, io, log, masking, pygeometry, upright
from opensfm.context import parallel_map
from opensfm.dataset_base import DataSetBase


logger: logging.Logger = logging.getLogger(__name__)


def run_features_processing(data: DataSetBase, images: List[str], force: bool) -> None:
    """Main entry point for running features extraction on a list of images."""
    default_queue_size = 10
    max_queue_size = 200

    mem_available = log.memory_available()
    processes = data.config["processes"]
    if mem_available:
        # Use 90% of available memory
        ratio_use = 0.9
        mem_available *= ratio_use
        logger.info(
            f"Planning to use {mem_available} MB of RAM for both processing queue and parallel processing."
        )

        # 50% for the queue / 50% for parallel processing
        expected_mb = mem_available / 2
        expected_images = min(
            max_queue_size, int(expected_mb / average_image_size(data))
        )
        processing_size = average_processing_size(data)
        logger.info(
            f"Scale-space expected size of a single image : {processing_size} MB"
        )
        processes = min(max(1, int(expected_mb / processing_size)), processes)
    else:
        expected_images = default_queue_size
    logger.info(
        f"Expecting to queue at most {expected_images} images while parallel processing of {processes} images."
    )

    process_queue = queue.Queue(expected_images)
    arguments: List[Tuple[str, Any]] = []

    if processes == 1:
        for image in images:
            counter = Counter()
            read_images(process_queue, data, [image], counter, 1, force)
            run_detection(process_queue)
            process_queue.get()
    else:
        counter = Counter()
        read_processes = data.config["read_processes"]
        if 1.5 * read_processes >= processes:
            read_processes = max(1, processes // 2)

        chunk_size = math.ceil(len(images) / read_processes)
        chunks_count = math.ceil(len(images) / chunk_size)
        read_processes = min(read_processes, chunks_count)

        expected: int = len(images)
        for i in range(read_processes):
            images_chunk = images[i * chunk_size : (i + 1) * chunk_size]
            arguments.append(
                (
                    "producer",
                    (process_queue, data, images_chunk, counter, expected, force),
                )
            )
        for _ in range(processes):
            arguments.append(("consumer", (process_queue)))
        parallel_map(process, arguments, processes, 1)


def average_image_size(data: DataSetBase) -> float:
    average_size_mb = 0
    for camera in data.load_camera_models().values():
        average_size_mb += camera.width * camera.height * 4 / 1024 / 1024
    return average_size_mb / max(1, len(data.load_camera_models()))


def average_processing_size(data: DataSetBase) -> float:
    processing_size = data.config["feature_process_size"]

    min_octave_size = 16  # from covdet.c
    octaveResolution = 3  # from covdet.c
    start_size = processing_size * processing_size * 4 / 1024 / 1024
    last_octave = math.floor(math.log2(processing_size / min_octave_size))

    total_size = 0
    for _ in range(last_octave + 1):
        total_size += start_size * octaveResolution
        start_size /= 2
    return total_size


def is_high_res_panorama(
    data: DataSetBase, image_key: str, image_array: np.ndarray
) -> bool:
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


class Counter:
    """Lock-less counter from https://julien.danjou.info/atomic-lock-free-counters-in-python/
    that relies on the CPython impl. of itertools.count() that is thread-safe. Used, as for
    some reason, joblib doesn't like a good old threading.Lock (everything is stuck)
    """

    def __init__(self) -> None:
        self.number_of_read = 0
        self.counter = itertools.count()
        self.read_lock = threading.Lock()

    def increment(self) -> None:
        next(self.counter)

    def value(self) -> int:
        with self.read_lock:
            value = next(self.counter) - self.number_of_read
            self.number_of_read += 1
        return value


def process(args: Tuple[str, Any]) -> None:
    process_type, real_args = args
    if process_type == "producer":
        queue, data, images, counter, expected, force = real_args
        read_images(queue, data, images, counter, expected, force)
    if process_type == "consumer":
        queue = real_args
        run_detection(queue)


def read_images(
    queue: queue.Queue,
    data: DataSetBase,
    images: List[str],
    counter: Counter,
    expected: int,
    force: bool,
) -> None:
    full_queue_timeout = 600
    for image in images:
        logger.info(f"Reading data for image {image} (queue-size={queue.qsize()})")
        image_array = data.load_image(image)
        if data.config["features_bake_segmentation"]:
            segmentation_array = data.load_segmentation(image)
            instances_array = data.load_instances(image)
        else:
            segmentation_array, instances_array = None, None
        args = image, image_array, segmentation_array, instances_array, data, force
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
        image, image_array, segmentation_array, instances_array, data, force = args
        detect(image, image_array, segmentation_array, instances_array, data, force)
        del image_array
        del segmentation_array
        del instances_array


def bake_segmentation(
    image: np.ndarray,
    points: np.ndarray,
    segmentation: Optional[np.ndarray],
    instances: Optional[np.ndarray],
    exif: Dict[str, Any],
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    exif_height, exif_width, exif_orientation = (
        exif["height"],
        exif["width"],
        exif.get("orientation", 1),
    )
    height, width = image.shape[:2]
    if exif_height != height or exif_width != width:
        logger.error(
            f"Image has inconsistent EXIF dimensions ({exif_width}, {exif_height}) and image dimensions ({width}, {height}). Orientation={exif_orientation}"
        )

    panoptic_data = [None, None]
    for i, p_data in enumerate([segmentation, instances]):
        if p_data is None:
            continue
        new_height, new_width = p_data.shape
        ps = upright.opensfm_to_upright(
            points[:, :2],
            width,
            height,
            exif_orientation,
            new_width=new_width,
            new_height=new_height,
        ).astype(int)
        panoptic_data[i] = p_data[ps[:, 1], ps[:, 0]]
    return tuple(panoptic_data)


def detect(
    image: str,
    image_array: np.ndarray,
    segmentation_array: Optional[np.ndarray],
    instances_array: Optional[np.ndarray],
    data: DataSetBase,
    force: bool = False,
) -> None:
    log.setup()

    need_words = (
        data.config["matcher_type"] == "WORDS"
        or data.config["matching_bow_neighbors"] > 0
    )
    has_words = not need_words or data.words_exist(image)
    has_features = data.features_exist(image)

    if not force and has_features and has_words:
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
            image_array, p_unmasked, segmentation_array, instances_array, exif
        )
        p_unsorted = p_unmasked
        f_unsorted = f_unmasked
        c_unsorted = c_unmasked
    # Load segmentation, make a mask from it mask and apply it
    else:
        s_unsorted, i_unsorted = None, None
        fmask = masking.load_features_mask(data, image, p_unmasked)
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
    if s_unsorted is not None:
        semantic_data = features.SemanticData(
            s_unsorted[order],
            i_unsorted[order] if i_unsorted is not None else None,
            data.segmentation_labels(),
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
