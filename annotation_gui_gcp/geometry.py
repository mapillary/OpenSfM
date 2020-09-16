import numpy as np
from opensfm import dataset, features


def calc_epipol_line(point_coord, img_pair, path, main_image_idx):
    data = dataset.DataSet(path)
    reconstruction = data.load_reconstruction()
    if not reconstruction[0].get_shot(img_pair[main_image_idx]):
        main_shot = reconstruction[1].get_shot(img_pair[main_image_idx])
        pair_shot = reconstruction[0].get_shot(img_pair[not main_image_idx])
    else :
        main_shot = reconstruction[0].get_shot(img_pair[main_image_idx])
        pair_shot = reconstruction[0].get_shot(img_pair[not main_image_idx])

# CHANGE COMING FROM RECONS 1
    line_pts = [[] for i in range(2)]
    for depth in np.arange(-100, 100, 5):
        point3d = main_shot.back_project(point_coord, depth)
        reprojection = pair_shot.project(point3d)
        line_pts[0].append(reprojection[0])
        line_pts[1].append(reprojection[1])

    return np.transpose(line_pts)

def get_all_track_observations(gcp_database, track_id):
    print(f"Getting all observations of track {track_id}")
    data = dataset.DataSet(gcp_database.path)
    tracks_manager = data.load_tracks_manager()
    track_obs = tracks_manager.get_track_observations(track_id)
    out = {}
    for shot_id, obs in track_obs.items():
        h, w = gcp_database.get_image_size(shot_id)
        point_px = features.denormalized_image_coordinates(np.array([obs.point]), w, h)[0]
        out[shot_id] = point_px
    return out

def get_tracks_visible_in_image(gcp_database, image_key, min_len=5):
    print(f"Getting track observations visible in {image_key}")
    data = dataset.DataSet(gcp_database.path)
    tracks_manager = data.load_tracks_manager()
    reconstruction = data.load_reconstruction()[0]

    out = {}
    for track_id in reconstruction.points:
        track_obs = tracks_manager.get_track_observations(track_id)
        if len(track_obs) < min_len:
            continue
        for shot_id, obs in track_obs.items():
            if shot_id == image_key:
                h, w = gcp_database.get_image_size(shot_id)
                point_px = features.denormalized_image_coordinates(np.array([obs.point]), w, h)[0]
                out[track_id] = point_px, len(track_obs)
    return out
