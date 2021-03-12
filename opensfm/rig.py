"""Tool for handling rigs"""

import logging
import re
from collections import defaultdict
from itertools import combinations

import networkx as nx
import numpy as np
from opensfm import actions, pygeometry, pymap
from opensfm.dataset import DataSet, DataSetBase


logger = logging.getLogger(__name__)


def find_image_rig(image, rig_patterns):
    """Given an image and candidates rig model patterns, return the
    RigID/RigCameraID/Instance Member ID this image belongs to.
    """
    for rig_id, patterns in rig_patterns.items():
        for rig_camera_id, pattern in patterns.items():
            instance_member_id = re.sub(pattern, "", image)
            if instance_member_id == "":
                continue
            if instance_member_id != image:
                return (rig_id, rig_camera_id, instance_member_id)
    return None, None, None


def create_instances_with_patterns(images, rig_patterns):
    """Using the provided patterns, group images that should belong to the same rig instances.
        Incomplete rig instances wrt. the expected size are not considered.

    Returns :
        A dict of list of list of images, each list being an instances being aggregated by Rig ID
    """
    per_pattern = defaultdict(dict)
    for image in images:
        rig_id, rig_camera_id, instance_member_id = find_image_rig(image, rig_patterns)
        if not rig_id:
            continue
        if instance_member_id not in per_pattern[rig_id]:
            per_pattern[rig_id][instance_member_id] = []
        per_pattern[rig_id][instance_member_id].append((image, rig_camera_id))

    complete_instances = defaultdict(list)
    problematic_images = []
    for rig_id, patterns in per_pattern.items():
        for pattern_images in patterns.values():
            expected_size = len(rig_patterns[rig_id])
            if len(pattern_images) != expected_size:
                problematic_images += [im[0] for im in pattern_images]
            else:
                complete_instances[rig_id].append(pattern_images)

    if problematic_images:
        logger.warning(
            (
                "The following images are part of an incomplete rig, thus"
                f"won't be considered of being part of a rig\n {problematic_images}"
            )
        )

    return complete_instances


def create_subset_dataset_from_instances(data: DataSet, instances_per_rig, name):
    """Given a list of images grouped by rigs instances, pick a subset of images
        and create a dataset subset with the provided name from them.

    Returns :
        A DataSet containing a subset of images containing enough rig instances
    """
    subset_images = []
    for instances in instances_per_rig.values():
        instances_sorted = sorted(
            instances, key=lambda x: data.load_exif(x[0][0])["capture_time"]
        )

        subset_size = data.config["rig_calibration_subset_size"]
        middle = len(instances_sorted) / 2
        instances_calibrate = instances_sorted[
            max([0, middle - int(subset_size / 2)]) : min(
                [middle + int(subset_size / 2), len(instances_sorted) - 1]
            )
        ]

        for instance in instances_calibrate:
            subset_images += [x[0] for x in instance]

    return data.subset(name, subset_images)


def compute_relative_pose(rig_id, pose_instances):
    """ Compute a rig model relatives poses given poses grouped by rig instance. """

    # Put all poses instances into some canonical frame taken as the mean of their R|t
    centered_pose_instances = []
    for instance in pose_instances:
        origin_center = np.zeros(3)
        rotation_center = np.zeros(3)
        for shot, _ in instance:
            rotation_center += shot.pose.rotation
            origin_center += shot.pose.get_origin()
        origin_center /= len(instance)
        rotation_center /= len(instance)

        centered_pose_instance = []
        for shot, rig_camera_id in instance:
            instance_pose = pygeometry.Pose(rotation_center)
            instance_pose.set_origin(origin_center)
            instance_pose_camera = shot.pose.relative_to(instance_pose)
            centered_pose_instance.append(
                (
                    instance_pose_camera,
                    rig_camera_id,
                    shot.camera.id,
                )
            )
        centered_pose_instances.append(centered_pose_instance)

    # Average canonical poses per RigCamera ID
    average_origin, average_rotation, count_poses, camera_ids = {}, {}, {}, {}
    for centered_pose_instance in centered_pose_instances:
        for pose, rig_camera_id, camera_id in centered_pose_instance:
            if rig_camera_id not in average_origin:
                average_origin[rig_camera_id] = np.zeros(3)
                average_rotation[rig_camera_id] = np.zeros(3)
                count_poses[rig_camera_id] = 0
            average_origin[rig_camera_id] += pose.get_origin()
            average_rotation[rig_camera_id] += pose.rotation
            camera_ids[rig_camera_id] = camera_id
            count_poses[rig_camera_id] += 1

    # Construct final rig_model results
    rig_model = pymap.RigModel(rig_id)
    for rig_camera_id, count in count_poses.items():
        o = average_origin[rig_camera_id] / count
        r = average_rotation[rig_camera_id] / count
        pose = pygeometry.Pose(r)
        pose.set_origin(o)
        rig_model.add_rig_camera(pymap.RigCamera(pose, rig_camera_id))
    return rig_model


def create_rig_models_from_reconstruction(reconstruction, instances_per_rig):
    """ Computed rig model's, given a reconstruction and rig instances's shots. """
    rig_models = {}
    reconstructions_shots = set(reconstruction.shots)
    for rig_id, instances in instances_per_rig.items():
        pose_groups = []
        for instance in instances:
            if any(
                True if shot_id not in reconstructions_shots else False
                for shot_id, _ in instance
            ):
                continue
            pose_groups.append(
                [
                    (reconstruction.shots[shot_id], rig_camera_id)
                    for shot_id, rig_camera_id in instance
                ]
            )
        rig_models[rig_id] = compute_relative_pose(rig_id, pose_groups)
    return rig_models


def create_rigs_with_pattern(data: DataSet, patterns):
    """Create rig data (`rig_models.json` and `rig_assignments.json`) by performing
    pattern matching to group images belonging to the same instances, followed
    by a bit of ad-hoc SfM to find some initial relative poses.
    """

    # Construct instances assignments for each rig
    instances_per_rig = create_instances_with_patterns(data.images(), patterns)
    for rig_id, instances in instances_per_rig.items():
        logger.info(
            f"Found {len(instances)} rig instances for rig {rig_id} using pattern matching."
        )

    # Create some subset DataSet with enough images from each rig
    subset_data = create_subset_dataset_from_instances(
        data, instances_per_rig, "rig_calibration"
    )

    # # Run a bit of SfM without any rig
    logger.info(f"Running SfM on a subset of {len(subset_data.images())} images.")
    actions.extract_metadata.run_dataset(subset_data)
    actions.detect_features.run_dataset(subset_data)
    actions.match_features.run_dataset(subset_data)
    actions.create_tracks.run_dataset(subset_data)
    actions.reconstruct.run_dataset(subset_data)

    # Compute some relative poses
    rig_models = create_rig_models_from_reconstruction(
        subset_data.load_reconstruction()[0], instances_per_rig
    )

    data.save_rig_models(rig_models)
    data.save_rig_assignments(instances_per_rig)


def same_rig_shot(meta1, meta2):
    """True if shots taken at the same time on a rig."""
    have_gps = (
        "gps" in meta1
        and "gps" in meta2
        and "latitude" in meta1["gps"]
        and "latitude" in meta2["gps"]
    )
    same_gps = (
        have_gps
        and meta1["gps"]["latitude"] == meta2["gps"]["latitude"]
        and meta1["gps"]["longitude"] == meta2["gps"]["longitude"]
    )
    same_time = meta1["capture_time"] == meta2["capture_time"]
    return same_gps and same_time


def detect_rigs(images, data: DataSetBase):
    """Search for rigs in a set of images.

    For each image on a rig, returns rig, rig_camera and rig_pose ids.
    """
    # Build graph of connected images and sequences
    image_graph = nx.Graph()
    sequence_graph = nx.Graph()
    for im1, im2 in combinations(images, 2):
        meta1 = data.load_exif(im1)
        meta2 = data.load_exif(im2)
        if same_rig_shot(meta1, meta2):
            image_graph.add_edge(im1, im2)
            sequence_graph.add_edge(meta1["skey"], meta2["skey"])

    # Build rigs
    # pyre-fixme[16]: Module `nx` has no attribute `connected_components`.
    sequence_cc = nx.connected_components(sequence_graph)
    sequence_rig_info = {}
    for i, cc in enumerate(sequence_cc):
        for j, sequence in enumerate(cc):
            sequence_rig_info[sequence] = {"rig": i, "rig_camera": j}

    # Build rig poses
    # pyre-fixme[16]: Module `nx` has no attribute `connected_components`.
    image_cc = nx.connected_components(image_graph)
    rig_info = {}
    for i, cc in enumerate(image_cc):
        for image in cc:
            meta = data.load_exif(image)
            sr = sequence_rig_info[meta["skey"]]
            rig_info[image] = {
                "rig": sr["rig"],
                "rig_camera": sr["rig_camera"],
                "rig_pose": i,
            }

    return rig_info


def pose_kernel(x, y, rotation_std, translation_std):
    """Gaussian kernel on the diff between two poses."""
    diff = x.relative_to(y)
    dr = sum(diff.rotation ** 2)
    dt = sum(diff.translation ** 2)
    return np.exp(-dr / rotation_std ** 2 - dt / translation_std ** 2)


def pose_mode(poses, rotation_std, translation_std):
    """Find the most popular pose.

    Popular is defined by a Parzen estimatior with the given
    Gaussian kernel standard deviations.
    """
    best_score = 0
    best_pose = None
    for pose in poses:
        score = 0
        for other in poses:
            score += pose_kernel(pose, other, rotation_std, translation_std)
        if score > best_score:
            best_score = score
            best_pose = pose
    return best_pose
