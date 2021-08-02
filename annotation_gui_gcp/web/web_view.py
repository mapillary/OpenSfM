import abc
import json
import time
from queue import Queue
from threading import Thread

from flask import Flask, Response, jsonify, render_template, request

distinct_colors = [
    "#46f0f0",
    "#f032e6",
    "#bcf60c",
    "#fabebe",
    "#008080",
    "#9a6324",
    "#fffac8",
    "#800000",
    "#aaffc3",
    "#808000",
    "#3cb44b",
    "#ffe119",
    "#4363d8",
    "#f58231",
    "#911eb4",
    "#000075",
    "#808080",
    "#ffffff",
    "#000000",
]


class WebView(abc.ABC):
    def __init__(self, main_ui):
        self.main_ui = main_ui
        self.app = Flask(__name__)
        self.eventQueue = None
        self.latlons = main_ui.image_manager.load_latlons()

    @abc.abstractclassmethod
    def sync_to_client(self):
        pass

    @abc.abstractclassmethod
    def process_client_message(self, data):
        pass

    def template_name(self):
        class_name = type(self).__name__
        return class_name

    def run_server(self, port, q):
        app = self.app
        app.config["TEMPLATES_AUTO_RELOAD"] = True

        @app.route("/")
        def send_main_page():
            template = self.template_name()
            self.sync_to_client()
            return render_template(f"{template}.html", class_name=template)

        @app.route("/postdata", methods=["POST"])
        def postdata():
            data = request.get_json()
            self.process_client_message(data)
            self.sync_to_client()
            return jsonify(success=True)

        # Stream for server -> client updates through Server-Sent Events
        @app.route("/stream")
        def stream():
            def eventStream():
                while True:
                    msg = self.eventQueue.get()  # blocks until a new message arrives
                    yield msg

            return Response(eventStream(), mimetype="text/event-stream")

        app.run(port=port)
        print(f"{type(self).__name__} app finished")

    def start(self, port):
        q = Queue()
        server_thread = Thread(target=self.run_server, args=(port, q))
        server_thread.start()

        # Queue for Flask -> JS sync
        # The GUI thread populates the Queue. The server thread listens to the queue and sends events to the JS client
        self.eventQueue = Queue()
        self.sync_to_client()

    def send_sse_message(self, data, event_type="sync"):
        # Send to the client
        data["time"] = time.time()
        sse_string = f"event: {event_type}\ndata: {json.dumps(data)}\n\n"
        self.eventQueue.put(sse_string)
