"""Tool for handling rigs"""

import logging
import os
import random
import re
import shutil
from typing import Dict, Tuple, List, Optional, Set, Iterable

import networkx as nx
import numpy as np
import scipy.spatial as spatial
from opensfm import actions, pygeometry, pymap, types
from opensfm.dataset import DataSet


logger = logging.getLogger(__name__)


TRigPatterns = Dict[str, str]
TRigCameraGroup = Set[str]
TRigImage = Tuple[str, str]
TRigInstance = List[TRigImage]

INCOMPLETE_INSTANCE_GROUP = "INCOMPLETE_INSTANCE_GROUP"
INCOMPLETE_INSTANCE_ID = "INCOMPLETE_INSTANCE_ID"


def find_image_rig(
    image: str, rig_patterns: TRigPatterns
) -> Tuple[Optional[str], Optional[str]]:
    """Given an image and candidates rig model patterns, return the
    RigCamera ID/Instance Member ID this image belongs to.
    """
    for rig_camera_id, pattern in rig_patterns.items():
        instance_member_id = re.sub(pattern, "", image)
        if instance_member_id == "":
            continue
        if instance_member_id != image:
            return (rig_camera_id, instance_member_id)
    return None, None


def create_instances_with_patterns(
    images: List[str], rig_patterns: TRigPatterns
) -> Tuple[Dict[str, TRigInstance], List[str]]:
    """Using the provided patterns, group images that should belong to the same rig instances.
    It will also check that a RigCamera belong to exactly one group of RigCameras

    Returns :
        A dict (by instance ID) of list of tuple of (image, rig camera)
    """
    per_instance_id: Dict[str, TRigInstance] = {}
    for image in images:
        rig_camera_id, instance_member_id = find_image_rig(image, rig_patterns)
        if not rig_camera_id and not instance_member_id:
            instance_member_id = INCOMPLETE_INSTANCE_GROUP
            rig_camera_id = INCOMPLETE_INSTANCE_ID
        if instance_member_id not in per_instance_id:
            # pyre-fixme [6]
            per_instance_id[instance_member_id] = []
        # pyre-fixme [6]
        per_instance_id[instance_member_id].append((image, rig_camera_id))

    per_complete_instance_id: Dict[str, TRigInstance] = {}
    single_shots: List[str] = []

    groups_per_camera: Dict[str, TRigCameraGroup] = {}
    for instance_id, cameras in per_instance_id.items():
        if instance_id == INCOMPLETE_INSTANCE_GROUP:
            single_shots += [im for im, _ in cameras]
            continue

        cameras_group = {c for _, c in cameras}
        for _, c in cameras:
            if c in groups_per_camera and cameras_group != groups_per_camera[c]:
                logger.warning(
                    (
                        f"Rig camera {c} already belongs to the rig camera group {groups_per_camera[c]}."
                        f"This rig camera is probably part of an incomplete instance : {cameras_group}"
                    )
                )
            groups_per_camera[c] = cameras_group
        per_complete_instance_id[instance_id] = cameras

    return per_complete_instance_id, single_shots


def group_instances(
    rig_instances: Dict[str, TRigInstance]
) -> Dict[str, List[TRigInstance]]:
    per_rig_camera_group: Dict[str, List[TRigInstance]] = {}
    for cameras in rig_instances.values():
        cameras_group = ", ".join(sorted({c for _, c in cameras}))
        if cameras_group not in per_rig_camera_group:
            per_rig_camera_group[cameras_group] = []
        per_rig_camera_group[cameras_group].append(cameras)
    return per_rig_camera_group


def propose_subset_dataset_from_instances(
    data: DataSet, rig_instances: Dict[str, TRigInstance], name: str
) -> Iterable[Tuple[DataSet, List[List[Tuple[str, str]]]]]:
    """Given a list of images grouped by rigs instances, infitely propose random
        subset of images and create a dataset subset with the provided name from them.

    Returns :
        Yield infinitely DataSet containing a subset of images containing enough rig instances
    """
    per_rig_camera_group = group_instances(rig_instances)

    if not data.reference_lla_exists():
        data.invent_reference_lla()
    reference = data.load_reference()

    instances_to_pick = {}
    for key, instances in per_rig_camera_group.items():
        # build GPS look-up tree
        gpses = []
        for i, instance in enumerate(instances):
            all_gps = []
            for image, _ in instance:
                gps = data.load_exif(image)["gps"]
                all_gps.append(
                    reference.to_topocentric(gps["latitude"], gps["longitude"], 0)
                )
            gpses.append((i, np.average(np.array(all_gps), axis=0)))
        tree = spatial.cKDTree([x[1] for x in gpses])

        # build NN-graph and split by connected components
        nn = 6
        instances_graph = nx.Graph()
        for i, gps in gpses:
            distances, neighbors = tree.query(gps, k=nn)
            for d, n in zip(distances, neighbors):
                if i == n:
                    continue
                instances_graph.add_edge(i, n, weight=d)
        all_components = sorted(
            nx.algorithms.components.connected_components(instances_graph),
            key=len,
            reverse=True,
        )
        logger.info(f"Found {len(all_components)} connected components")

        # keep the biggest one
        biggest_component = all_components[0]
        logger.info(f"Best component has {len(biggest_component)} instances")
        instances_to_pick[key] = biggest_component

    random.seed(42)
    while True:
        total_instances = []
        subset_images = []
        for key, instances in instances_to_pick.items():
            all_instances = per_rig_camera_group[key]

            instances_sorted = sorted(
                [all_instances[i] for i in instances],
                key=lambda x: data.load_exif(x[0][0])["capture_time"],
            )

            subset_size = data.config["rig_calibration_subset_size"]
            random_index = random.randint(0, len(instances_sorted) - 1)
            instances_calibrate = instances_sorted[
                max([0, random_index - int(subset_size / 2)]) : min(
                    [random_index + int(subset_size / 2), len(instances_sorted) - 1]
                )
            ]

            for instance in instances_calibrate:
                subset_images += [x[0] for x in instance]
            total_instances += instances_calibrate

        data.io_handler.rm_if_exist(os.path.join(data.data_path, name))
        yield data.subset(name, subset_images), total_instances


def compute_relative_pose(
    pose_instances: List[List[Tuple[pymap.Shot, str]]],
) -> Dict[str, pymap.RigCamera]:
    """Compute a rig model relatives poses given poses grouped by rig instance."""

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

    # Construct final RigCamera results
    rig_cameras: Dict[str, pymap.RigCamera] = {}
    for rig_camera_id, count in count_poses.items():
        o = average_origin[rig_camera_id] / count
        r = average_rotation[rig_camera_id] / count
        pose = pygeometry.Pose(r)
        pose.set_origin(o)
        rig_cameras[rig_camera_id] = pymap.RigCamera(pose, rig_camera_id)
    return rig_cameras


def create_rig_cameras_from_reconstruction(
    reconstruction: types.Reconstruction, rig_instances: Dict[str, TRigInstance]
) -> Dict[str, pymap.RigCamera]:
    """Compute rig cameras poses, given a reconstruction and rig instances's shots."""
    rig_cameras: Dict[str, pymap.RigCamera] = {}
    reconstructions_shots = set(reconstruction.shots)
    logger.info(f"Computing rig cameras pose using {len(reconstructions_shots)} shots")

    per_rig_camera_group = group_instances(rig_instances)
    logger.info(f"Found {len(per_rig_camera_group)} rig cameras groups")
    for instances in sorted(per_rig_camera_group.values(), key=lambda x: -len(x)):
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
        for rig_camera_id, rig_camera in compute_relative_pose(pose_groups).items():
            if rig_camera_id in rig_cameras:
                logger.warning(
                    f"Ignoring {rig_camera_id} as it was already computed from a bigger set of instances"
                )
            else:
                rig_cameras[rig_camera_id] = rig_camera
    return rig_cameras


def create_rigs_with_pattern(data: DataSet, patterns: TRigPatterns):
    """Create rig data (`rig_cameras.json` and `rig_assignments.json`) by performing
    pattern matching to group images belonging to the same instances, followed
    by a bit of ad-hoc SfM to find some initial relative poses.
    """

    # Construct instances assignments for each rig
    instances_per_rig, single_shots = create_instances_with_patterns(
        data.images(), patterns
    )
    for rig_id, instances in instances_per_rig.items():
        logger.info(
            f"Found {len(instances)} shots for instance {rig_id} using pattern matching."
        )
    logger.info(f"Found {len(single_shots)} single shots using pattern matching.")

    # Create some random subset DataSet with enough images from each rig and run SfM
    count = 0
    max_rounds = data.config["rig_calibration_max_rounds"]
    best_reconstruction = None
    best_rig_cameras = None
    for subset_data, instances in propose_subset_dataset_from_instances(
        data, instances_per_rig, "rig_calibration"
    ):
        if len(subset_data.images()) == 0:
            continue

        if count > max_rounds:
            break
        count += 1

        # Run a bit of SfM without any rig
        logger.info(
            f"Running SfM on a subset of {len(subset_data.images())} images. Round {count}/{max_rounds}"
        )
        actions.extract_metadata.run_dataset(subset_data)
        actions.detect_features.run_dataset(subset_data)
        actions.match_features.run_dataset(subset_data)
        actions.create_tracks.run_dataset(subset_data)
        actions.reconstruct.run_dataset(subset_data)

        reconstruction = subset_data.load_reconstruction()[0]

        # Compute some relative poses
        rig_cameras = create_rig_cameras_from_reconstruction(
            reconstruction, instances_per_rig
        )
        found_cameras = {c for i in instances_per_rig.values() for _, c in i}
        if set(rig_cameras.keys()) != found_cameras:
            logger.error(
                f"Calibrated {len(rig_cameras)} whereas {len(found_cameras)} were requested. Rig creation failed."
            )
            continue

        reconstructed_instances = count_reconstructed_instances(
            instances, reconstruction
        )
        logger.info(
            f"reconstructed {reconstructed_instances} instances over {len(instances)}"
        )
        if (
            reconstructed_instances
            < len(instances) * data.config["rig_calibration_completeness"]
        ):
            continue

        best_reconstruction = reconstruction
        best_rig_cameras = rig_cameras
        break

    if best_reconstruction and best_rig_cameras:
        logger.info(
            f"Found a candidate for rig calibration with {len(best_reconstruction.shots)} shots"
        )
        data.save_rig_cameras(best_rig_cameras)
        data.save_rig_assignments(list(instances_per_rig.values()))
    else:
        logger.error(
            "Could not run any sucessful SfM on images subset for rig calibration"
        )


def count_reconstructed_instances(
    instances: List[List[Tuple[str, str]]], reconstruction: types.Reconstruction
) -> int:
    instances_map = {}
    instances_count = {}
    for i, instance in enumerate(instances):
        instances_count[i] = len(instance)
        for shot_id, _ in instance:
            instances_map[shot_id] = i
    for s in reconstruction.shots:
        instances_count[instances_map[s]] -= 1
    return len(instances) - sum(1 for i in instances_count.values() if i > 0)


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
