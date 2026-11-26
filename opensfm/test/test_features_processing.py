from typing import Any
from unittest import mock

import numpy as np
from numpy.typing import NDArray
from opensfm import features_processing
from opensfm.actions import extract_metadata
from opensfm.test import data_generation


def _extract_features(
    tmpdir: Any, **config: Any
) -> dict[str, tuple[NDArray, NDArray | None, NDArray]]:
    data = data_generation.create_berlin_test_folder(tmpdir)
    data.config.update({"processes": 2, "read_processes": 1, **config})
    extract_metadata.run_dataset(data)
    features_processing.run_features_processing(data, data.images(), True)
    extracted = {}
    for image in data.images():
        features = data.load_features(image)
        assert features is not None
        extracted[image] = (features.points, features.descriptors, features.colors)
    return extracted


def test_dedicated_readers_extract_the_same_features(tmpdir: Any) -> None:
    shared = _extract_features(tmpdir.mkdir("shared"))
    dedicated = _extract_features(
        tmpdir.mkdir("dedicated"),
        features_dedicated_readers=True,
        read_processes=2,
        features_queue_size=1,
    )

    assert shared.keys() == dedicated.keys()
    for image, arrays in shared.items():
        for expected, actual in zip(arrays, dedicated[image]):
            np.testing.assert_array_equal(expected, actual)


def _schedule(tmpdir: Any, **config: Any) -> tuple[int, int, int, int]:
    """The (threads, readers, extractors, queue size) that
    run_features_processing asks of parallel_map for the Berlin images."""
    data = data_generation.create_berlin_test_folder(tmpdir)
    data.config.update(
        {"processes": 4, "read_processes": 3, "mem_ceiling": 2**16, **config}
    )
    extract_metadata.run_dataset(data)
    with mock.patch.object(features_processing, "parallel_map") as parallel_map:
        features_processing.run_features_processing(data, data.images(), True)
    _, arguments, threads, _ = parallel_map.call_args.args
    readers = [args for kind, args in arguments if kind == "producer"]
    extractors = [args for kind, args in arguments if kind == "consumer"]
    return threads, len(readers), len(extractors), extractors[0].maxsize


def test_readers_take_extractor_threads_by_default(tmpdir: Any) -> None:
    threads, readers, extractors, queue_size = _schedule(tmpdir)

    # 3 readers would take too many of the 4 threads: they are halved.
    assert (threads, readers, extractors) == (4, 2, 4)
    assert queue_size > 2


def test_dedicated_readers_run_beside_every_extractor(tmpdir: Any) -> None:
    *_, shared_queue_size = _schedule(tmpdir.mkdir("shared"))
    threads, readers, extractors, queue_size = _schedule(
        tmpdir.mkdir("dedicated"), features_dedicated_readers=True
    )

    assert (threads, readers, extractors) == (7, 3, 4)
    # The images the 3 readers hold come out of the queue's memory share.
    assert queue_size == shared_queue_size - 3


def test_queue_size_caps_the_queue(tmpdir: Any) -> None:
    *_, queue_size = _schedule(tmpdir, features_queue_size=2)

    assert queue_size == 2
