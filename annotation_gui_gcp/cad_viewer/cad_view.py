import os
import sys
import tkinter as tk
from queue import Queue
from threading import Thread, get_ident

from flask import Flask, jsonify, request, send_from_directory


class CadView:
    def __init__(
        self,
        main_ui,
        path_cad_model,
    ):

        self.main_ui = main_ui
        self.path_cad_model = path_cad_model

        q = Queue()
        pipe_read, pipe_write = os.pipe()
        server_thread = Thread(
            target=self.run_server, args=(path_cad_model, q, pipe_write)
        )
        server_thread.start()

        def new_data_available(file, mask):
            # This is triggered whenever there is something new in the pipe
            os.read(pipe_read, 1)
            data = q.get()

            # Clear any other messages
            while not q.empty():
                try:
                    q.get(False)
                except Empty:
                    continue
                q.task_done()

            self.process_message_from_cad_view(data)

        # Use pipes and tk's file handler to trigger the UI loop to wake up
        main_ui.master.tk.createfilehandler(pipe_read, tk.READABLE, new_data_available)

    def run_server(self, path_cad_model, q, pipe_write):
        cad_app = Flask(__name__)

        @cad_app.route("/templates/<path:filename>")
        def send_static_resources(filename):
            print(filename)
            return send_from_directory("templates", filename)

        @cad_app.route("/templates/postdata", methods=["POST"])
        def postdata():
            data = request.get_json()
            print(f"Flask app on thread {get_ident()} received data: {data}")
            q.put(data)  # Push the data to the queue
            # Write something (anything) to the pipe to trigger a UI event that reads from the queue
            os.write(pipe_write, b"x")
            return jsonify(success=True)

        cad_app.run()
        print("CAD view app finished")

    def process_message_from_cad_view(self, data):
        print(f"UI Thread {get_ident()} received data {data}")

        command = data["command"]
        if command == "add_or_update_point_observation":
            self.add_remove_update_point_observation(point_coordinates=data["xyz"])
        elif command == "remove_point_observation":
            self.add_remove_update_point_observation(None)
        else:
            raise ValueError

    def add_remove_update_point_observation(self, point_coordinates=None):
        active_gcp = self.main_ui.curr_point
        if active_gcp is None:
            print(f"No point selected in the main UI. Doing nothing")
            return

        # Remove the observation for this point if it's already there
        self.main_ui.gcp_manager.remove_point_observation(
            active_gcp, self.path_cad_model
        )

        # Add the new observation
        if point_coordinates:
            self.main_ui.gcp_manager.add_point_observation(
                active_gcp,
                self.path_cad_model,
                point_coordinates,
            )
            print(
                f"Added {point_coordinates} observation for {active_gcp} in {self.path_cad_model}"
            )

        self.main_ui.populate_gcp_list()
