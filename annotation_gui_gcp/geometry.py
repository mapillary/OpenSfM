from opensfm import dataset


def get_all_track_observations(gcp_database, track_id):
    print(f"Getting all observations of track {track_id}")
    data = dataset.DataSet(gcp_database.path)
    tracks_manager = data.load_tracks_manager()
    track_obs = tracks_manager.get_track_observations(track_id)
    return {shot_id: obs.point for shot_id, obs in track_obs.items()}


def get_tracks_visible_in_image(gcp_database, image_key, min_len=5):
    print(f"Getting track observations visible in {image_key}")
    data = dataset.DataSet(gcp_database.path)
    tracks_manager = data.load_tracks_manager()
    reconstructions = data.load_reconstruction()
    for reconstruction in reconstructions:
        if image_key in reconstruction.shots:
            break

    out = {}
    for track_id in reconstruction.points:
        track_obs = tracks_manager.get_track_observations(track_id)
        if len(track_obs) < min_len:
            continue
        for shot_id, obs in track_obs.items():
            if shot_id == image_key:
                out[track_id] = obs.point, len(track_obs)
    return out
