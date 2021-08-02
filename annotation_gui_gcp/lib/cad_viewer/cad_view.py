import json
import os
import tkinter as tk
from pathlib import Path
from queue import Queue
from threading import Thread

import rasterio
from flask import Flask, Response, jsonify, request, send_from_directory
from PIL import ImageColor
from annotation_gui_gcp.lib.view import distinct_colors


def _load_georeference_metadata(path_cad_model):
    path_metadata = path_cad_model.with_suffix(".json")

    if not path_metadata.is_file():
        raise FileNotFoundError(
            f"Did not find expected metadata file for {path_cad_model} in {path_metadata}"
        )
    metadata = json.load(open(path_metadata))
    return metadata


class CadView:
    def __init__(
        self,
        main_ui,
        path_cad_file,
        port=5000,
        is_geo_reference=False,
    ):

        self.main_ui = main_ui
        path_cad_file = Path(path_cad_file).resolve()
        self.cad_filename = path_cad_file.name

        # Create a symlink to the CAD file so that it is reachable
        path_this_file = Path(__file__)
        p_symlink = (
            path_this_file.parent / f"static/resources/cad_models/{self.cad_filename}"
        )
        p_symlink.parent.mkdir(exist_ok=True, parents=True)
        if p_symlink.is_symlink():
            os.remove(p_symlink)

        os.symlink(path_cad_file, p_symlink)

        # Load data required to georeference this model
        self.load_georeference_metadata(path_cad_file)
        self.is_geo_reference = is_geo_reference

        # We use a Queue and a pipe to communicate from the web view to the tk GUI.
        # The Queue contains the information. The pipe is used to wake up the GUI thread.
        q = Queue()
        pipe_read, pipe_write = os.pipe()
        server_thread = Thread(
            target=self.run_server, args=(self.cad_filename, port, q, pipe_write)
        )
        server_thread.start()

        def message_from_client_available(file, mask):
            # This is triggered whenever there is something new in the pipe
            os.read(pipe_read, 1)
            data = q.get()

            # Clear any other messages
            while not q.empty():
                try:
                    q.get(False)
                except Queue.Empty:
                    continue
                q.task_done()

            self.process_message_from_cad_view(data)

        # Use pipes and tk's file handler to wake up the GUI thread
        main_ui.parent.tk.createfilehandler(
            pipe_read, tk.READABLE, message_from_client_available
        )

        # Queue for Flask -> JS sync
        # The GUI thread populates the Queue. The server thread listens to the queue and sends events to the JS client
        self.eventQueue = Queue()
        self.sync_to_client()

    def run_server(self, cad_filename, port, q, pipe_write):
        cad_app = Flask(__name__)

        @cad_app.route("/")
        def send_main_page():
            self.sync_to_client()
            return send_from_directory("templates", "CADAnnotation.html")

        @cad_app.route("/postdata", methods=["POST"])
        def postdata():
            data = request.get_json()
            q.put(data)  # Push the data to the queue
            # Write something (anything) to the pipe to trigger a UI event that reads from the queue
            os.write(pipe_write, b"x")
            return jsonify(success=True)

        # Stream for server -> client updates through Server-Sent Events
        @cad_app.route("/stream")
        def stream():
            def eventStream():
                while True:
                    # time.sleep(1)
                    # self.sync_to_client()
                    msg = self.eventQueue.get()  # blocks until a new message arrives
                    yield msg

            return Response(eventStream(), mimetype="text/event-stream")

        cad_app.run(port=port)
        print("CAD view app finished")

    def process_message_from_cad_view(self, data):
        command = data["command"]
        if command == "add_or_update_point_observation":
            self.add_remove_update_point_observation(point_coordinates=data["xyz"])
        elif command == "remove_point_observation":
            self.add_remove_update_point_observation(None)
        else:
            raise ValueError

        # Update the client with the new data
        self.sync_to_client()

    def add_remove_update_point_observation(self, point_coordinates=None):
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
        if point_coordinates is not None:
            lla = self.xyz_to_latlon(*point_coordinates)
            self.main_ui.gcp_manager.add_point_observation(
                active_gcp,
                self.cad_filename,
                point_coordinates,
                lla if self.is_geo_reference else None,
            )
        self.main_ui.populate_gcp_list()

    def display_points(self):
        # Update the client with the new data
        self.sync_to_client()

    def sync_to_client(self):
        """
        Sends all the data required to initialize the client
        """
        # Points with annotations on this file
        visible_points_coords = self.main_ui.gcp_manager.get_visible_points_coords(
            self.cad_filename
        )

        data = {
            "annotations": {},
            "selected_point": self.main_ui.curr_point,
            "cad_filename": self.cad_filename,
        }

        # Pick a color for each point
        fn_reprojections = Path(
            f"{self.main_ui.path}/gcp_reprojections_3D_{self.main_ui.ix_a}x{self.cad_filename}.json"
        )
        if fn_reprojections.exists():
            reprojections = json.load(open(fn_reprojections))
        else:
            reprojections = {}
        for point_id, coords in visible_points_coords.items():
            hex_color = distinct_colors[divmod(hash(point_id), 19)[1]]
            color = ImageColor.getrgb(hex_color)
            data["annotations"][point_id] = {"coordinates": coords, "color": color}

            reproj = reprojections.get(point_id)
            if reproj:
                data["annotations"][point_id]["reprojection"] = reproj

        self.send_sse_message(data)

    def send_sse_message(self, data, event_type="sync"):
        # Send to the client
        sse_string = f"event: {event_type}\ndata: {json.dumps(data)}\n\n"
        self.eventQueue.put(sse_string)

    def refocus(self, lat, lon):
        x, y, z = self.latlon_to_xyz(lat, lon)
        self.send_sse_message(
            {"x": x, "y": y, "z": z},
            event_type="move_camera",
        )

    def highlight_gcp_reprojection(self, *args, **kwargs):
        pass

    def populate_image_list(self, *args, **kwargs):
        pass

    def latlon_to_xyz(self, lat, lon):
        xs, ys, zs = rasterio.warp.transform("EPSG:4326", self.crs, [lon], [lat], [0])
        x = xs[0] * self.scale - self.offset[0]
        y = ys[0] * self.scale - self.offset[1]
        z = zs[0] * self.scale - self.offset[2]
        y, z = z, -y
        return x, y, z

    def xyz_to_latlon(self, x, y, z):
        y, z = -z, y

        # Add offset (cm) and transform to m
        x = (x + self.offset[0]) / self.scale
        y = (y + self.offset[1]) / self.scale
        z = (z + self.offset[2]) / self.scale
        lons, lats, alts = rasterio.warp.transform(self.crs, "EPSG:4326", [x], [y], [z])
        return lats[0], lons[0], alts[0]

    def load_georeference_metadata(self, path_cad_model):
        metadata = _load_georeference_metadata(path_cad_model)
        self.scale = metadata["scale"]
        self.crs = metadata["crs"]
        self.offset = metadata["offset"]
