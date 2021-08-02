import magic
from flask import send_file

from web.web_view import WebView


class ImageView(WebView):
    def __init__(
        self,
        main_ui,
        image_keys,
        is_geo_reference,
        port=5000,
    ):
        super().__init__(main_ui)
        self.main_ui = main_ui
        self.image_manager = main_ui.image_manager
        self.is_geo_reference = is_geo_reference
        self.image_list = image_keys

        self.app.add_url_rule("/image/<key>", view_func=self.get_image)

        self.start(port)

    def get_candidate_images(self):
        return self.image_list

    def get_image(self, key):
        path_image = self.image_manager.image_path(key)
        mimetype = magic.from_file(path_image, mime=True)
        return send_file(path_image, mimetype=mimetype)

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
            lla = (
                self.xyz_to_latlon(*point_coordinates)
                if self.is_geo_reference
                else None
            )
            self.main_ui.gcp_manager.add_point_observation(
                active_gcp,
                image_id,
                point_coordinates,
                lla,
            )

        self.main_ui.populate_gcp_list()

    def display_points(self):
        # Data sent along with the rest of the state in sync_to_client.
        # This extra call might be redundant
        self.sync_to_client()

    def highlight_gcp_reprojection(self, *args, **kwargs):
        # Data sent along with the rest of the state in sync_to_client.
        # This extra call might be redundant
        self.sync_to_client()

    def populate_image_list(self, *args, **kwargs):
        # Data sent along with the rest of the state in sync_to_client.
        # This extra call might be redundant
        self.sync_to_client()

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

    def process_client_message(self, data):
        command = data["command"]

        if data["point_id"] != self.main_ui.curr_point:
            print(data["point_id"], self.main_ui.curr_point)
            print("Frontend sending an update for some other point. Ignoring")
            return

        if command == "add_or_update_point_observation":
            self.add_remove_update_point_observation(
                image_id=data["image_id"], point_coordinates=data["xy"]
            )
        elif command == "remove_point_observation":
            self.add_remove_update_point_observation(
                image_id=data["image_id"], point_coordinates=None
            )
        else:
            raise ValueError
