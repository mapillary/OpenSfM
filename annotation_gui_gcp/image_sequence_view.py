import tkinter as tk
from typing import Tuple

import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from opensfm import features

from geometry import get_all_track_observations, get_tracks_visible_in_image
from view import View


class ImageSequenceView(View):
    def __init__(self, main_ui, name, sequence_key, image_keys):
        self.group_name = sequence_key
        self.current_image = image_keys[0]
        self.image_keys = image_keys
        self.zoom_window_size_px = 200
        self.image_manager = main_ui.image_manager
        super(ImageSequenceView, self).__init__(main_ui, name)

        # Auto GCP - related stuff
        auto_gcp_button = tk.Button(
            self.toolbox,
            text="Auto GCP",
            command=lambda window=self.window: self.auto_gcp_show_tracks(),
        )
        auto_gcp_button.pack(side="top")

    def get_image(self, new_image):
        return self.image_manager.get_image(new_image)

    def get_candidate_images(self):
        return self.image_keys

    def auto_gcp_show_tracks(self):
        h, w = self.image_manager.get_image_size(self.current_image)
        tracks = get_tracks_visible_in_image(
            self.main_ui.gcp_manager, self.current_image
        )

        # Select some tracks to plot, not all
        # nice_tracks = sorted(tracks, key=lambda tid: -tracks[tid]['length'])
        nice_tracks = tracks.keys()  # all tracks
        tracks = {k: tracks[k] for k in nice_tracks}

        if len(tracks) == 0:
            print("No valid tracks found")
            return

        # Draw track projections
        points = []
        track_lengths = []
        for point, track_length in tracks.values():
            points.append(self.gcp_to_pixel_coordinates(*point))
            track_lengths.append(track_length)

        points = np.array(points)
        norm = matplotlib.colors.Normalize()
        colors = plt.cm.viridis(norm(track_lengths))
        self.tracks_scatter = self.subplot.scatter(
            points[:, 0], points[:, 1], s=10, c=colors, marker="x"
        )
        self.canvas.draw_idle()

        # The next left or right click will be handled differently by setting this:
        self.visible_tracks = tracks

    def auto_gcp_create(self, x, y, add):
        # Find track closest to click location and create a GCP from it
        x, y = self.pixel_to_gcp_coordinates(x, y)
        if add:
            points, track_ids = [], []
            for tid, t in self.visible_tracks.items():
                points.append(t[0])
                track_ids.append(tid)
            dists = np.linalg.norm(np.array(points) - np.array([[x, y]]), axis=1)
            closest_track = track_ids[np.argmin(dists)]
            observations = get_all_track_observations(
                self.main_ui.gcp_manager, closest_track
            )

            new_gcp = self.main_ui.add_gcp()
            print(f"New GCP {new_gcp} with {len(observations)} observations")
            for shot_id, point in observations.items():
                self.main_ui.gcp_manager.add_point_observation(new_gcp, shot_id, point)

        self.visible_tracks = None
        self.tracks_scatter.remove()
        self.canvas.draw()
        self.update_image_list_text()

    def pixel_to_latlon(self, x: float, y: float):
        return None

    def gcp_to_pixel_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """
        Transforms from normalized coordinates to pixels

        The view displays images at a reduced resolution for speed. We use the image
        manager to obtain the reduced coordinates to use for de-normalization.
        """
        h, w = self.image_manager.get_image_size(self.current_image)
        px = features.denormalized_image_coordinates(np.array([[x, y]]), w, h)[0]
        return px.tolist()

    def pixel_to_gcp_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """
        Transforms from pixels to normalized coordinates

        The view displays images at a reduced resolution for speed. We use the image
        manager to obtain the reduced coordinates to use for normalization.
        """
        h, w = self.image_manager.get_image_size(self.current_image)
        coords = features.normalized_image_coordinates(np.array([[x, y]]), w, h)[0]
        return coords.tolist()
