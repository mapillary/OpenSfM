import os
from dataclasses import dataclass, asdict
from typing import Any, Dict, IO, Union

import yaml


@dataclass
class OpenSfMConfig:
    ##################################
    # Params for metadata
    ##################################
    use_exif_size: bool = True
    # Treat images from unknown camera models as coming from different cameras
    unknown_camera_models_are_different: bool = False
    default_focal_prior: float = 0.85

    ##################################
    # Params for features
    ##################################
    # Feature type (AKAZE, SURF, SIFT, HAHOG, ORB)
    feature_type: str = "HAHOG"
    # If true, apply square root mapping to features
    feature_root: bool = True
    # If fewer frames are detected, sift_peak_threshold/surf_hessian_threshold is reduced.
    feature_min_frames: int = 4000
    # Same as above but for panorama images
    feature_min_frames_panorama: int = 16000
    # Resize the image if its size is larger than specified. Set to -1 for original size
    feature_process_size: int = 2048
    # Same as above but for panorama images
    feature_process_size_panorama: int = 4096
    feature_use_adaptive_suppression: bool = False
    # Bake segmentation info (class and instance) in the feature data. Thus it is done once for all at extraction time.
    features_bake_segmentation: bool = False

    ##################################
    # Params for SIFT
    ##################################
    # Smaller value -> more features
    sift_peak_threshold: float = 0.1
    # See OpenCV doc
    sift_edge_threshold: int = 10

    ##################################
    # Params for SURF
    ##################################
    # Smaller value -> more features
    surf_hessian_threshold: float = 3000
    # See OpenCV doc
    surf_n_octaves: int = 4
    # See OpenCV doc
    surf_n_octavelayers: int = 2
    # See OpenCV doc
    surf_upright: int = 0

    ##################################
    # Params for AKAZE (See details in lib/src/third_party/akaze/AKAZEConfig.h)
    ##################################
    # Maximum octave evolution of the image 2^sigma (coarsest scale sigma units)
    akaze_omax: int = 4
    # Detector response threshold to accept point
    akaze_dthreshold: float = 0.001
    # Feature type
    akaze_descriptor: str = "MSURF"
    # Size of the descriptor in bits. 0->Full size
    akaze_descriptor_size: int = 0
    # Number of feature channels (1,2,3)
    akaze_descriptor_channels: int = 3
    akaze_kcontrast_percentile: float = 0.7
    akaze_use_isotropic_diffusion: bool = False

    ##################################
    # Params for HAHOG
    ##################################
    hahog_peak_threshold: float = 0.00001
    hahog_edge_threshold: float = 10
    hahog_normalize_to_uchar: bool = True

    ##################################
    # Params for general matching
    ##################################
    # Ratio test for matches
    lowes_ratio: float = 0.8
    # FLANN, BRUTEFORCE, or WORDS
    matcher_type: str = "FLANN"
    # Match symmetrically or one-way
    symmetric_matching: bool = True

    ##################################
    # Params for FLANN matching
    ##################################
    # Algorithm type (KMEANS, KDTREE)
    flann_algorithm: str = "KMEANS"
    # See OpenCV doc
    flann_branching: int = 8
    # See OpenCV doc
    flann_iterations: int = 10
    # See OpenCV doc
    flann_tree: int = 8
    # Smaller -> Faster (but might lose good matches)
    flann_checks: int = 20

    ##################################
    # Params for BoW matching
    ##################################
    bow_file: str = "bow_hahog_root_uchar_10000.npz"
    # Number of words to explore per feature.
    bow_words_to_match: int = 50
    # Number of matching features to check.
    bow_num_checks: int = 20
    # Matcher type to assign words to features
    bow_matcher_type: str = "FLANN"

    ##################################
    # Params for VLAD matching
    ##################################
    vlad_file: str = "bow_hahog_root_uchar_64.npz"

    ##################################
    # Params for guided matching
    ##################################
    # Number of randomized spanning-trees to samples over the tracks-graph
    guided_spanning_trees: int = 5
    # Random ratio higher bound edges are multiplied with
    guided_spanning_trees_random: float = 0.5
    # Threshold for epipolar distance for accepting a match in radians
    guided_matching_threshold: float = 0.006
    # Minimum track length for initial triangulation
    guided_min_length_initial: int = 3
    # Minimum track length for final triangulation
    guided_min_length_final: int = 3
    # Threshold of reprojection for extending a track within a new image (in radians)
    guided_extend_threshold: float = 0.002
    # Number of images considered as neighbors of another one
    guided_extend_image_neighbors: int = 50
    # Maximum number of reprojected neighbors (in the tracks-graph) to check when extending a track within a new image
    guided_extend_feature_neighbors: int = 10

    ##################################
    # Params for matching
    ##################################
    # Maximum gps distance between two images for matching
    matching_gps_distance: float = 150
    # Number of images to match selected by GPS distance. Set to 0 to use no limit (or disable if matching_gps_distance is also 0)
    matching_gps_neighbors: int = 0
    # Number of images to match selected by time taken. Set to 0 to disable
    matching_time_neighbors: int = 0
    # Number of images to match selected by image name. Set to 0 to disable
    matching_order_neighbors: int = 0
    # Number of images to match selected by BoW distance. Set to 0 to disable
    matching_bow_neighbors: int = 0
    # Maximum GPS distance for preempting images before using selection by BoW distance. Set to 0 to disable
    matching_bow_gps_distance: float = 0
    # Number of images (selected by GPS distance) to preempt before using selection by BoW distance. Set to 0 to use no limit (or disable if matching_bow_gps_distance is also 0)
    matching_bow_gps_neighbors: int = 0
    # If True, BoW image selection will use N neighbors from the same camera + N neighbors from any different camera. If False, the selection will take the nearest neighbors from all cameras.
    matching_bow_other_cameras: bool = False
    # Number of images to match selected by VLAD distance. Set to 0 to disable
    matching_vlad_neighbors: int = 0
    # Maximum GPS distance for preempting images before using selection by VLAD distance. Set to 0 to disable
    matching_vlad_gps_distance: float = 0
    # Number of images (selected by GPS distance) to preempt before using selection by VLAD distance. Set to 0 to use no limit (or disable if matching_vlad_gps_distance is also 0)
    matching_vlad_gps_neighbors: int = 0
    # If True, VLAD image selection will use N neighbors from the same camera + N neighbors from any different camera. If False, the selection will take the nearest neighbors from all cameras.
    matching_vlad_other_cameras: bool = False
    # Number of rounds to run when running triangulation-based pair selection
    matching_graph_rounds: int = 0
    # If True, removes static matches using ad-hoc heuristics
    matching_use_filters: bool = False
    # Use segmentation information (if available) to improve matching
    matching_use_segmentation: bool = False

    ##################################
    # Params for geometric estimation
    ##################################
    # Outlier threshold for fundamental matrix estimation as portion of image width
    robust_matching_threshold: float = 0.004
    # Outlier threshold for essential matrix estimation during matching in radians
    robust_matching_calib_threshold: float = 0.004
    # Minimum number of matches to accept matches between two images
    robust_matching_min_match: int = 20
    # Outlier threshold for essential matrix estimation during incremental reconstruction in radians
    five_point_algo_threshold: float = 0.004
    # Minimum number of inliers for considering a two view reconstruction valid
    five_point_algo_min_inliers: int = 20
    # Number of LM iterations to run when refining relative pose during matching
    five_point_refine_match_iterations: int = 10
    # Number of LM iterations to run when refining relative pose during reconstruction
    five_point_refine_rec_iterations: int = 1000
    # Check for Necker reversal ambiguities. Useful for long focal length with long distance capture (aerial manned)
    five_point_reversal_check: bool = False
    # Ratio of triangulated points non-reversed/reversed when checking for Necker reversal ambiguities
    five_point_reversal_ratio: float = 0.95
    # Outlier threshold for accepting a triangulated point in radians
    triangulation_threshold: float = 0.006
    # Minimum angle between views to accept a triangulated point
    triangulation_min_ray_angle: float = 1.0
    # Triangulation type : either considering all rays (FULL), or sing a RANSAC variant (ROBUST)
    triangulation_type: str = "FULL"
    # Number of LM iterations to run when refining a point
    triangulation_refinement_iterations: int = 10
    # Outlier threshold for resection in radians
    resection_threshold: float = 0.004
    # Minimum number of resection inliers to accept it
    resection_min_inliers: int = 10

    ##################################
    # Params for track creation
    ##################################
    # Minimum number of features/images per track
    min_track_length: int = 2

    ##################################
    # Params for bundle adjustment
    ##################################
    # Loss function for the ceres problem (see: http://ceres-solver.org/modeling.html#lossfunction)
    loss_function: str = "SoftLOneLoss"
    # Threshold on the squared residuals.  Usually cost is quadratic for smaller residuals and sub-quadratic above.
    loss_function_threshold: float = 1
    # The standard deviation of the reprojection error
    reprojection_error_sd: float = 0.004
    # The standard deviation of the exif focal length in log-scale
    exif_focal_sd: float = 0.01
    # The standard deviation of the principal point coordinates
    principal_point_sd: float = 0.01
    # The standard deviation of the first radial distortion parameter
    radial_distortion_k1_sd: float = 0.01
    # The standard deviation of the second radial distortion parameter
    radial_distortion_k2_sd: float = 0.01
    # The standard deviation of the third radial distortion parameter
    radial_distortion_k3_sd: float = 0.01
    # The standard deviation of the fourth radial distortion parameter
    radial_distortion_k4_sd: float = 0.01
    # The standard deviation of the first tangential distortion parameter
    tangential_distortion_p1_sd: float = 0.01
    # The standard deviation of the second tangential distortion parameter
    tangential_distortion_p2_sd: float = 0.01
    # The default horizontal standard deviation of the GCPs (in meters)
    gcp_horizontal_sd: float = 0.01
    # The default vertical standard deviation of the GCPs (in meters)
    gcp_vertical_sd: float = 0.1
    # Global weight for GCPs, expressed a ratio of the sum of (# projections) + (# shots) + (# relative motions)
    gcp_global_weight: float = 0.01
    # The standard deviation of the rig translation
    rig_translation_sd: float = 0.1
    # The standard deviation of the rig rotation
    rig_rotation_sd: float = 0.1
    # Type of threshold for filtering outlier : either fixed value (FIXED) or based on actual distribution (AUTO)
    bundle_outlier_filtering_type: str = "FIXED"
    # For AUTO filtering type, projections with larger reprojection than ratio-times-mean, are removed
    bundle_outlier_auto_ratio: float = 3.0
    # For FIXED filtering type, projections with larger reprojection error after bundle adjustment are removed
    bundle_outlier_fixed_threshold: float = 0.006
    # Optimize internal camera parameters during bundle
    optimize_camera_parameters: bool = True
    # Maximum optimizer iterations.
    bundle_max_iterations: int = 100

    # Retriangulate all points from time to time
    retriangulation: bool = True
    # Retriangulate when the number of points grows by this ratio
    retriangulation_ratio: float = 1.2
    # Use analytic derivatives or auto-differentiated ones during bundle adjustment
    bundle_analytic_derivatives: bool = True
    # Bundle after adding 'bundle_interval' cameras
    bundle_interval: int = 999999
    # Bundle when the number of points grows by this ratio
    bundle_new_points_ratio: float = 1.2
    # Max image graph distance for images to be included in local bundle adjustment
    local_bundle_radius: int = 3
    # Minimum number of common points betwenn images to be considered neighbors
    local_bundle_min_common_points: int = 20
    # Max number of shots to optimize during local bundle adjustment
    local_bundle_max_shots: int = 30

    # Save reconstructions at every iteration
    save_partial_reconstructions: bool = False

    ##################################
    # Params for GPS alignment
    ##################################
    # Use or ignore EXIF altitude tag
    use_altitude_tag: bool = True
    # orientation_prior or naive
    align_method: str = "auto"
    # horizontal, vertical or no_roll
    align_orientation_prior: str = "horizontal"
    # Enforce GPS position in bundle adjustment
    bundle_use_gps: bool = True
    # Enforce Ground Control Point position in bundle adjustment
    bundle_use_gcp: bool = True
    # Compensate GPS with a per-camera similarity transform
    bundle_compensate_gps_bias: bool = False

    ##################################
    # Params for rigs
    ##################################
    # Number of rig instances to use when calibration rigs
    rig_calibration_subset_size: int = 15
    # Ratio of reconstructed images needed to consider a reconstruction for rig calibration
    rig_calibration_completeness: float = 0.85
    # Number of SfM tentatives to run until we get a satisfying reconstruction
    rig_calibration_max_rounds: int = 10

    ##################################
    # Params for image undistortion
    ##################################
    # Format in which to save the undistorted images
    undistorted_image_format: str = "jpg"
    # Max width and height of the undistorted image
    undistorted_image_max_size: int = 100000

    ##################################
    # Params for depth estimation
    ##################################
    # Raw depthmap computation algorithm (PATCH_MATCH, BRUTE_FORCE, PATCH_MATCH_SAMPLE)
    depthmap_method: str = "PATCH_MATCH_SAMPLE"
    # Resolution of the depth maps
    depthmap_resolution: int = 640
    # Number of neighboring views
    depthmap_num_neighbors: int = 10
    # Number of neighboring views used for each depthmaps
    depthmap_num_matching_views: int = 6
    # Minimum depth in meters.  Set to 0 to auto-infer from the reconstruction.
    depthmap_min_depth: float = 0
    # Maximum depth in meters.  Set to 0 to auto-infer from the reconstruction.
    depthmap_max_depth: float = 0
    # Number of PatchMatch iterations to run
    depthmap_patchmatch_iterations: int = 3
    # Size of the correlation patch
    depthmap_patch_size: int = 7
    # Patches with lower standard deviation are ignored
    depthmap_min_patch_sd: float = 1.0
    # Minimum correlation score to accept a depth value
    depthmap_min_correlation_score: float = 0.1
    # Threshold to measure depth closeness
    depthmap_same_depth_threshold: float = 0.01
    # Min number of views that should reconstruct a point for it to be valid
    depthmap_min_consistent_views: int = 3
    # Save debug files with partial reconstruction results
    depthmap_save_debug_files: bool = False

    ##################################
    # Params for multi-processing/threading
    ##################################
    # Number of threads to use
    processes: int = 1
    # When processes > 1, number of threads used for reading images
    read_processes: int = 4

    ##################################
    # Params for submodel split and merge
    ##################################
    # Average number of images per submodel
    submodel_size: int = 80
    # Radius of the overlapping region between submodels
    submodel_overlap: float = 30.0
    # Relative path to the submodels directory
    submodels_relpath: str = "submodels"
    # Template to generate the relative path to a submodel directory
    submodel_relpath_template: str = "submodels/submodel_%04d"
    # Template to generate the relative path to a submodel images directory
    submodel_images_relpath_template: str = "submodels/submodel_%04d/images"


def default_config() -> Dict[str, Any]:
    """Return default configuration"""
    return asdict(OpenSfMConfig())


def load_config(filepath) -> Dict[str, Any]:
    """DEPRECATED: = Load config from a config.yaml filepath"""
    if not os.path.isfile(filepath):
        return default_config()

    with open(filepath) as fin:
        return load_config_from_fileobject(fin)


def load_config_from_fileobject(
    f: Union[IO[bytes], IO[str], bytes, str]
) -> Dict[str, Any]:
    """Load config from a config.yaml fileobject"""
    config = default_config()

    new_config = yaml.safe_load(f)
    if new_config:
        for k, v in new_config.items():
            config[k] = v

    return config
