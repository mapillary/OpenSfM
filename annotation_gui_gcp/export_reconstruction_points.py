import numpy as np

from opensfm.dataset import DataSet
from opensfm.features import denormalized_image_coordinates


def world_points(ds: DataSet):
    # this is taken from export_reconstruction_points.py
    # not in public opensfm?

    output = {}

    tracks_manager = ds.load_tracks_manager()
    for reconstruction in ds.load_reconstruction():
        print(reconstruction.reference)
        for point in reconstruction.points.values():
            x, y, z = point.coordinates
            lat, lon, alt = reconstruction.reference.to_lla(x, y, z)
            images = []
            for shot_id, obs in tracks_manager.get_track_observations(point.id).items():
                if shot_id in reconstruction.shots:
                    shot = reconstruction.shots[shot_id]

                    image_id = shot_id
                    x_px, y_px = obs.point
                    x_px, y_px = denormalized_image_coordinates(
                        np.array([[x_px, y_px]]
                                 ), shot.camera.width, shot.camera.height
                    )[0]
                    images.append(
                        {"image_id": image_id, "x_px": x_px, "y_px": y_px})
            point_id = f"{point.id}"
            output[point_id] = {
                "location": {"lat": lat, "lon": lon, "alt": alt},
                "images": images, "point_id": point_id,
            }
    return output
