.. Docs on the reporting


Reporting
=========

OpenSfM commands write reports on the work done.  Reports are stored in the ``reports`` folder in json format so that they can be loaded by programatically.  Here is the list of reports produced and the data included.

Feature detection
-----------------
The report on feature detection is stored in the file ``features.json``.  Its structure is as follow::

    {
        "wall_time": {{ total time compting features }},
        "image_reports": [   # For each image
            {
                "wall_time": {{ feature extraction time }},
                "image": {{ image name }},
                "num_features": {{ number of features }}
            },
            ...
        ]
    }

Matching
--------

The report on matching is stored in the file ``matches.json``.  Its structure is as follow::

    {
        "wall_time": {{ total time compting matches }},
        "pairs": {{ list of candidate image pairs }}
        "num_pairs": {{ number of candidate image pairs }},
        "num_pairs_distance": {{ number of pairs selected based on distance }},
        "num_pairs_time": {{ number of pairs selected based on time }},
        "num_pairs_order": {{ number of pairs selected based on order }},
    }

Create tracks
-------------

The report on tracks creation is stored in the file ``tracks.json``.  Its structure is as follow::

    {
        "wall_time": {{ total time computing tracks }}
        "wall_times": {
            "load_features": {{ time loading features }},
            "load_matches": {{ time loading matches }},
            "compute_tracks": {{ time computing tracks }},
        },
        "num_images": {{ number of images with tracks }},
        "num_tracks": {{ number of tracks }},
        "view_graph": {{ number of image tracks for each image pair }}
    }

Reconstruction
--------------

The report on the reconstruction process is stored in the file ``reconstruction.json``.  Its structure is as follow::

    {
        "wall_times": {
            "compute_reconstructions": {{ time computing the reconstruction }},
            "compute_image_pairs": {{ time computing the candidate initial pairs }},
            "load_tracks_graph": {{ time loading tracks }}
        },
        "num_candidate_image_pairs": {{ number of candidate image pairs for initializing reconstructions }},
        "reconstructions": [  # For each reconstruction build
            {
                "bootstrap": {  # Initialization information
                    "memory_usage": {{ memory usage at the end of the process }},
                    "image_pair": {{ initial image pair }},
                    "common_tracks": {{ number of common tracks of the image pair }},
                    "two_view_reconstruction": {
                        "5_point_inliers": {{ number of inliers for the 5-point algorithm }},
                        "plane_based_inliers": {{ number of inliers for the plane based initialization }},
                        "method": {{ method used for initialization "5_point" or "plane_based" }}
                    },
                    "triangulated_points": {{ number of triangulated points }},
                    "decision": {{ either "Success" or the reason for failure }},
                },
                "grow": {  # Incremental growth information
                    "steps": [  # For every growth step
                        {
                            "image": {{ image name }},
                            "resection": {
                                "num_inliers": {{ number of inliers }},
                                "num_common_points": {{ number of reconstructed points visible on the new image }}
                            },
                            "triangulated_points": {{ number of newly triangulated points }},
                            "memory_usage": {{ memory usage after adding the image }},
                            "bundle": {
                                "wall_times": {
                                    "setup": {{ time setting up bundle }},
                                    "run": {{ time running bundle }},
                                    "teardown": {{ time updating the values after bundle }},
                                },
                                "brief_report": {{ Ceres brief report }}
                            },
                        }
                    ]
                }
            }
        ],
        "not_reconstructed_images": {{ images that could not be reconstructed }},
    }
