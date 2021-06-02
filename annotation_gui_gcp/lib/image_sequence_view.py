import tkinter as tk

import matplotlib
import numpy as np
from annotation_gui_gcp.lib.geometry import (
    get_all_track_observations,
    get_tracks_visible_in_image,
)
from annotation_gui_gcp.lib.view import View
from matplotlib import pyplot as plt


class ImageSequenceView(View):
    def __init__(self, main_ui, sequence_key, image_keys, show_track_checkbox):
        self.group_name = sequence_key
        self.images_in_list = image_keys
        self.zoom_window_size_px = 200
        self.image_manager = main_ui.image_manager
        super(ImageSequenceView, self).__init__(main_ui, show_track_checkbox)

        # Auto GCP - related stuff
        auto_gcp_button = tk.Button(
            self.toolbox,
            text="Auto GCP",
            command=lambda window=self.window: self.auto_gcp_show_tracks(),
        )
        auto_gcp_button.pack(side="top")

        rotate_button = tk.Button(
            self.toolbox,
            text="Rotate",
            command=lambda window=self.window: self.rotate(),
        )
        rotate_button.pack(side="top")

        self.populate_image_list()
        self.bring_new_image(self.images_in_list[0])
        self.set_title()

    def get_image(self, new_image):
        return self.image_manager.get_image(new_image)

    def get_candidate_images(self):
        return self.images_in_list

    def rotate(self):
        self.rotation = (self.rotation + 1) % 4
        self.bring_new_image(self.current_image, force=True)

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
        self.tracks_scatter = self.ax.scatter(
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

            self.main_ui.add_gcp()
            new_gcp = self.main_ui.curr_point
            print(f"New GCP {new_gcp} with {len(observations)} observations")
            for shot_id, point in observations.items():
                point = point.tolist()
                self.main_ui.gcp_manager.add_point_observation(new_gcp, shot_id, point)

        self.visible_tracks = None
        self.tracks_scatter.remove()
        self.canvas.draw()
        self.update_image_list_text()

    def set_title(self):
        shot = self.current_image
        seq_ix = self.images_in_list.index(shot)
        title = f"{self.group_name} [{seq_ix+1}/{len(self.images_in_list)}]: {shot}"
        self.window.title(title)

    def go_to_image_index(self, idx):
        incoming_image = self.images_in_list[idx]
        incoming_rig_images = self.main_ui.rig_groups.get(
            self.images_in_list[idx], [incoming_image]
        )

        # Update all views in the rig
        for image in incoming_rig_images:
            for v in self.main_ui.sequence_views:
                if image in v.images_in_list:
                    v.bring_new_image(image)
                    break
