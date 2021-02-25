from typing import Tuple

import matplotlib
import numpy as np
from matplotlib import pyplot as plt
from opensfm import features

from oblique_manager import (
    coords_in_rotated_image,
    invert_coords_from_rotated_image,
    ObliqueManager
)
from geometry import get_all_track_observations, get_tracks_visible_in_image
from view import View

import tkinter as tk


class ObliqueView(View):
    def __init__(
        self,
        main_ui,
        oblique_manager,
    ):
        """[summary]

        Args:
            main_ui (GUI.Gui)
            path (str): path containing oblique sfm folder
        """
        self.zoom_window_size_px = 500
        self.name = 'oblique'
        self.image_manager = oblique_manager
        super(ObliqueView, self).__init__(main_ui, False)

        # Auto GCP - related stuff
        auto_gcp_button = tk.Button(
            self.toolbox,
            text="Auto GCP",
            command=lambda window=self.window: self.auto_gcp_show_tracks(),
        )
        auto_gcp_button.pack(side="top")

    def oblique_selection(self, lat, lon):
        self.lat = lat
        self.lon = lon
        self.image_names = self.image_manager.get_candidates(
            self.lat, self.lon)
        self.populate_image_list()
        if self.images_in_list:
            self.bring_new_image(self.images_in_list[0])
        self.set_title()
        return

    def bring_new_image(self, new_image):
        super(ObliqueView, self).bring_new_image(new_image, force=True)
        xlim = self.ax.get_xlim()
        ylim = self.ax.get_ylim()
        x1, y1 = self.image_manager.get_normalized_feature(new_image)
        xx = xlim[0]+x1*(xlim[1]-xlim[0])
        yy = ylim[1]-y1*(ylim[1]-ylim[0])
        artists = self.ax.plot(xx, yy, "rx")
        self.plt_artists.extend(artists)
        self.canvas.draw_idle()

    def get_image(self, new_image):
        return self.image_manager.get_image(new_image)

    def get_candidate_images(self):
        if not self.image_names:
            self.image_names = self.image_manager.get_candidates(
                self.lat, self.lon)
        return self.image_names

    def auto_gcp_show_tracks(self):
        h, w = self.image_manager.get_image_size(self.current_image)
        tracks = get_tracks_visible_in_image(
            self.image_manager, self.current_image
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
        for point, track_length, coord in tracks.values():
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
            points, track_ids, coords = [], [], []
            for tid, t in self.visible_tracks.items():
                points.append(t[0])
                track_ids.append(tid)
                coords.append(t[2])
            dists = np.linalg.norm(
                np.array(points) - np.array([[x, y]]), axis=1)
            closest_track = track_ids[np.argmin(dists)]
            observations = get_all_track_observations(
                self.image_manager, closest_track
            )

            latlonalt = self.visible_tracks[closest_track][2]

            new_gcp = self.main_ui.add_gcp()
            print(f"New GCP {new_gcp} with {len(observations)} observations")
            for shot_id, point in observations.items():
                point = point.tolist()
                self.main_ui.gcp_manager.add_point_observation(
                    new_gcp, shot_id, point, latlonalt, alt=latlonalt[2])

        self.visible_tracks = None
        self.tracks_scatter.remove()
        self.canvas.draw()
        self.update_image_list_text()

        
    def pixel_to_latlon(self, x: float, y: float):
        """
        Todo: From pixels (in the viewing window) to latlon by finding
        nearest feature
        """
        return None

    def gcp_to_pixel_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """
        Transforms from normalized coordinates to pixels
        The view displays images at a reduced resolution for speed. We use the image
        manager to obtain the reduced coordinates to use for de-normalization.
        """
        h, w = self.image_manager.get_image_size(self.current_image)
        px = features.denormalized_image_coordinates(
            np.array([[x, y]]), w, h)[0]
        x = px[0]
        y = px[1]
        return [x, y]

    def pixel_to_gcp_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """
        Transforms from pixels (in the viewing window) to normalized coordinates
        (in the whole geotiff)
        """
        h, w = self.image_manager.get_image_size(self.current_image)
        coords = features.normalized_image_coordinates(
            np.array([[x, y]]), w, h)[0]
        return coords.tolist()

    def set_title(self):
        lat, lon = self.lat, self.lon
        if self.images_in_list:
            t = "Images covering lat:{:.4f}, lon:{:.4f}".format(lat, lon)
            shot = self.current_image
            seq_ix = self.images_in_list.index(shot)
            title = f"{t} [{seq_ix+1}/{len(self.images_in_list)}]: {shot}"
        else:
            title = f"No orthophotos around {lat}, {lon}"
            self.current_image = None
            self.ax.clear()
            self.ax.axis("off")
            self.canvas.draw_idle()

        self.window.title(title)

    def view_pixel_to_source_pixel(self, x: float, y: float) ->  Tuple[float, float]:
        theta = self.image_manager.get_rotation_angle(self.current_image)
        px, py = invert_coords_from_rotated_image((x, y), theta)
        return px, py

    def add_move_or_remove_gcp(self, x, y, add):
        if self.main_ui.curr_point is None:
            return
        reproj = self.main_ui.gcp_manager.gcp_reprojections.get(
            self.main_ui.curr_point)
        if reproj:
            reproj.pop(self.current_image, None)
        self.main_ui.gcp_manager.remove_point_observation(
            self.main_ui.curr_point, self.current_image
        )
        if add:
            self.main_ui.gcp_manager.add_point_observation(
                self.main_ui.curr_point,
                self.current_image,
                self.pixel_to_gcp_coordinates(x, y),
                oblique_coords = self.view_pixel_to_source_pixel(x, y)
            )
            self.zoom_in(x, y)
        else:
            self.zoom_out()
