import json
import logging
from pathlib import Path
from typing import Any, Dict, Tuple

import rasterio
from annotation_gui_gcp.lib.views.web_view import WebView, distinct_colors
from flask import send_file
from PIL import ImageColor

logger: logging.Logger = logging.getLogger(__name__)


def _load_georeference_metadata(path_cad_model) -> Dict[str, Any]:
    path_metadata = path_cad_model.with_suffix(".json")

    if not path_metadata.is_file():
        raise FileNotFoundError(
            f"Did not find expected metadata file for {path_cad_model} in {path_metadata}"
        )
    metadata = json.load(open(path_metadata))
    return metadata


class CADView(WebView):
    def __init__(
        self,
        main_ui,
        web_app,
        route_prefix,
        path_cad_file,
        is_geo_reference=False,
    )-> None:
        super().__init__(main_ui, web_app, route_prefix)

        self.main_ui = main_ui
        self.cad_path = Path(path_cad_file).resolve()
        self.cad_filename = self.cad_path.name

        # Load data required to georeference this model
        self.load_georeference_metadata(path_cad_file)
        self.is_geo_reference = is_geo_reference

        self.app.add_url_rule(
            f"{route_prefix}/model",
            f"{route_prefix}_model",
            view_func=self.get_model,
        )

    def get_model(self) -> Any:
        return send_file(self.cad_path, mimetype="application/octet-stream")

    def process_client_message(self, data: Dict[str, Any]) -> None:
        event = data["event"]
        if event == "add_or_update_point_observation":
            self.add_remove_update_point_observation(point_coordinates=data["xyz"])
        elif event == "remove_point_observation":
            self.add_remove_update_point_observation(None)
        else:
            raise ValueError(f"Unknown event {event}")

    def add_remove_update_point_observation(self, point_coordinates=None)->None:
        gcp_manager = self.main_ui.gcp_manager
        active_gcp = self.main_ui.curr_point
        if active_gcp is None:
            print("No point selected in the main UI. Doing nothing")
            return

        # Remove the observation for this point if it's already there
        gcp_manager.remove_point_observation(
            active_gcp, self.cad_filename, remove_latlon=self.is_geo_reference
        )

        # Add the new observation
        if point_coordinates is not None and self.is_geo_reference is not None:
            lla = self.xyz_to_latlon(*point_coordinates)

            geo = {
                "latitude": lla[0],
                "longitude": lla[1],
                "altitude": lla[2],
                "horizontal_std": 100,
                "vertical_std": None,
            }
            self.main_ui.gcp_manager.add_point_observation(
                active_gcp,
                self.cad_filename,
                point_coordinates,
                precision=100,  # in 3D units
                geo=geo,
            )
            logger.warning(
                f"Saving {geo} on {self.cad_filename} with hardcoded precision values"
            )
        self.main_ui.populate_gcp_list()

    def display_points(self) -> None:
        pass

    def refocus(self, lat, lon)->None:
        x, y, z = self.latlon_to_xyz(lat, lon)
        self.send_sse_message(
            {"x": x, "y": y, "z": z},
            event_type="move_camera",
        )

    def highlight_gcp_reprojection(self, *args, **kwargs)->None:
        pass

    def populate_image_list(self, *args, **kwargs)->None:
        pass

    def latlon_to_xyz(self, lat, lon) -> Tuple[float, float, float]:
        xs, ys, zs = rasterio.warp.transform("EPSG:4326", self.crs, [lon], [lat], [0])
        x = xs[0] * self.scale - self.offset[0]
        y = ys[0] * self.scale - self.offset[1]
        z = zs[0] * self.scale - self.offset[2]
        y, z = z, -y
        return x, y, z

    def xyz_to_latlon(self, x, y, z) -> Tuple[float, float, float]:
        y, z = -z, y

        # Add offset (cm) and transform to m
        x = (x + self.offset[0]) / self.scale
        y = (y + self.offset[1]) / self.scale
        z = (z + self.offset[2]) / self.scale
        lons, lats, alts = rasterio.warp.transform(self.crs, "EPSG:4326", [x], [y], [z])
        return lats[0], lons[0], alts[0]

    def load_georeference_metadata(self, path_cad_model)->None:
        metadata = _load_georeference_metadata(path_cad_model)
        self.scale = metadata["scale"]
        self.crs = metadata["crs"]
        self.offset = metadata["offset"]

    def sync_to_client(self)->None:
        """
        Sends all the data required to initialize or sync the CAD view
        """
        # Points with annotations on this file
        visible_points_coords = self.main_ui.gcp_manager.get_visible_points_coords(
            self.cad_filename
        )

        data = {
            "annotations": {},
            "selected_point": self.main_ui.curr_point,
        }

        for point_id, coords in visible_points_coords.items():
            hex_color = distinct_colors[divmod(hash(point_id), 19)[1]]
            color = ImageColor.getrgb(hex_color)
            data["annotations"][point_id] = {"coordinates": coords[:-1], "precision": coords[-1], "color": color}

        # Add the 3D reprojections of the points
        fn_reprojections = Path(
            f"{self.main_ui.path}/gcp_reprojections_3D_{self.main_ui.ix_a}x{self.cad_filename}.json"
        )
        if fn_reprojections.exists():
            reprojections = json.load(open(fn_reprojections))
            for point_id in data["annotations"]:
                if point_id in reprojections:
                    data["annotations"][point_id]["reprojection"] = reprojections[
                        point_id
                    ]

        self.send_sse_message(data)
