import abc
import json
import os
import time
import tkinter as tk
import webbrowser
from queue import Queue
from threading import Thread

from flask import Flask, Response, jsonify, render_template, request


class WebView(abc.ABC):
    def __init__(self, main_ui):
        self.main_ui = main_ui
        self.app = Flask(__name__)
        self.eventQueue = None

    @abc.abstractclassmethod
    def sync_to_client(self):
        pass

    @abc.abstractclassmethod
    def process_client_message(self, data):
        pass

    def template_name(self):
        class_name = type(self).__name__
        return class_name

    def run_server(self, port, q, pipe_write):
        app = self.app
        app.config["TEMPLATES_AUTO_RELOAD"] = True

        @app.route("/")
        def send_main_page():
            # self.sync_to_client()
            template = self.template_name()
            return render_template(f"{template}.html", class_name=template)

        @app.route("/postdata", methods=["POST"])
        def postdata():
            data = request.get_json()
            q.put(data)  # Push the data to the queue
            # Write something (anything) to the pipe to trigger a UI event that reads from the queue
            os.write(pipe_write, b"x")
            return jsonify(success=True)

        # Stream for server -> client updates through Server-Sent Events
        @app.route("/stream")
        def stream():
            def eventStream():
                while True:
                    # time.sleep(0.5)
                    msg = self.eventQueue.get()  # blocks until a new message arrives
                    yield msg

            return Response(eventStream(), mimetype="text/event-stream")

        app.run(port=port)
        print(f"{type(self).__name__} app finished")

    def start(self, port):
        # We use a Queue and a pipe to communicate from the web view to the tk GUI.
        # The Queue contains the information. The pipe is used to wake up the GUI thread.
        q = Queue()
        pipe_read, pipe_write = os.pipe()
        server_thread = Thread(target=self.run_server, args=(port, q, pipe_write))
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

            self.process_client_message(data)

        # Use pipes and tk's file handler to wake up the GUI thread
        self.main_ui.parent.tk.createfilehandler(
            pipe_read, tk.READABLE, message_from_client_available
        )

        # Queue for Flask -> JS sync
        # The GUI thread populates the Queue. The server thread listens to the queue and sends events to the JS client
        self.eventQueue = Queue()
        self.sync_to_client()

        # Opening a new window or setting size does not seem possible
        # for webbrowser w/Chrome
        #
        webbrowser.open(f"http://localhost:{port}")

    def send_sse_message(self, data, event_type="sync"):
        # Send to the client
        data["time"] = time.time()
        sse_string = f"event: {event_type}\ndata: {json.dumps(data)}\n\n"
        self.eventQueue.put(sse_string)
