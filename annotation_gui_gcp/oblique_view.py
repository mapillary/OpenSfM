from typing import Tuple

import numpy as np
from opensfm import features

from oblique_manager import (
    coords_in_rotated_image,
    invert_coords_from_rotated_image,
    ObliqueManager
)
from view import View


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
        # red cross appears in center of image
        artists = self.ax.plot(np.mean(xlim), np.mean(ylim), "rx")
        self.plt_artists.extend(artists)
        self.canvas.draw_idle()

    def get_image(self, new_image):
        return self.image_manager.get_image(new_image)

    def get_candidate_images(self):
        if not self.image_names:
            self.image_names = self.image_manager.get_candidates(
                self.lat, self.lon)
        return self.image_names

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
        x1, y1 = self.image_manager.get_offsets(self.current_image)
        x = px[0] - x1
        y = px[1] - y1
        return [x, y]

    def pixel_to_gcp_coordinates(self, x: float, y: float) -> Tuple[float, float]:
        """
        Transforms from pixels (in the viewing window) to normalized coordinates
        (in the whole geotiff)
        """
        x1, y1 = self.image_manager.get_offsets(self.current_image)
        x += x1
        y += y1
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
        x1, y1 = self.image_manager.get_offsets(self.current_image)
        x += x1
        y += y1
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
