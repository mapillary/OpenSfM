from typing import Dict, Any

from annotation_gui_gcp.lib.views.web_view import WebView


class ImageView(WebView):
    def __init__(
        self,
        main_ui,
        web_app,
        route_prefix,
        image_keys,
        is_geo_reference,
    ):
        super().__init__(main_ui, web_app, route_prefix)
        self.main_ui = main_ui
        self.image_manager = main_ui.image_manager
        self.is_geo_reference = is_geo_reference
        self.image_list = image_keys

        self.app.add_url_rule(
            f"{route_prefix}/image/<int:max_sz>/<path:path>",
            f"{route_prefix}_image",
            view_func=self.get_image,
        )

    def get_candidate_images(self):
        return self.image_list

    def get_image(self, path, max_sz):
        return self.image_manager.get_image(path, max_sz)

    def add_remove_update_point_observation(self, image_id, point_coordinates=None):
        gcp_manager = self.main_ui.gcp_manager
        active_gcp = self.main_ui.curr_point
        if active_gcp is None:
            print("No point selected in the main UI. Doing nothing")
            return

        # Remove the observation for this point if it's already there
        gcp_manager.remove_point_observation(
            active_gcp, image_id, remove_latlon=self.is_geo_reference
        )

        # Add the new observation
        if point_coordinates is not None:
            self.main_ui.gcp_manager.add_point_observation(
                active_gcp,
                image_id,
                point_coordinates,
                latlon=None,
            )

        self.main_ui.populate_gcp_list()

    def display_points(self):
        pass

    def highlight_gcp_reprojection(self, *args, **kwargs):
        pass

    def populate_image_list(self, *args, **kwargs):
        pass

    def sync_to_client(self):
        """
        Sends all the data required to initialize or sync the image view
        """
        # All images assigned to this view
        image_list = self.get_candidate_images()

        # Dict of images -> points
        all_points_this_view = {
            image: self.main_ui.gcp_manager.get_visible_points_coords(image)
            for image in image_list
        }

        data = {
            "points": all_points_this_view,
            "selected_point": self.main_ui.curr_point,
        }

        # for point_id, coords in visible_points_coords.items():
        #     hex_color = distinct_colors[divmod(hash(point_id), 19)[1]]
        #     color = ImageColor.getrgb(hex_color)
        #     data["annotations"][point_id] = {"coordinates": coords, "color": color}

        self.send_sse_message(data)

    def process_client_message(self, data: Dict[str, Any]) -> None:
        command = data["event"]
        if command not in (
            "add_or_update_point_observation",
            "remove_point_observation",
        ):
            raise ValueError(f"Unknown commmand {command}")

        if data["point_id"] != self.main_ui.curr_point:
            print(data["point_id"], self.main_ui.curr_point)
            print("Frontend sending an update for some other point. Ignoring")
            return

        if command == "add_or_update_point_observation":
            self.add_remove_update_point_observation(
                image_id=data["image_id"], point_coordinates=data["xy"]
            )
        else:  # == "remove_point_observation":
            self.add_remove_update_point_observation(
                image_id=data["image_id"], point_coordinates=None
            )
