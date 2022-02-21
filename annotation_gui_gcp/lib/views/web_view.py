import abc
import json
import time
from queue import Queue

from flask import Response, jsonify, render_template, request

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
    def __init__(self, main_ui, web_app, route_prefix):
        self.main_ui = main_ui
        self.app = web_app
        self.eventQueue = Queue()
        self.latlons = main_ui.image_manager.load_latlons()
        self.register_routes(route_prefix)
        self.route_prefix = route_prefix

    @abc.abstractclassmethod
    def sync_to_client(self):
        pass

    @abc.abstractclassmethod
    def process_client_message(self, data):
        pass

    def template_name(self):
        class_name = type(self).__name__
        return class_name

    def register_routes(self, route):
        def send_main_page():
            template = self.template_name()
            return render_template(f"{template}.html", class_name=template)

        self.app.add_url_rule(route, route + "_index", send_main_page)

        def postdata():
            data = request.get_json()

            # Do something with the event received from the client
            if data["event"] != "init":
                self.process_client_message(data)

            # Send a sync event back to the client to reflect the changed state (all views)
            self.main_ui.sync_to_client()
            return jsonify(success=True)

        self.app.add_url_rule(
            route + "/postdata", route + "_postdata", postdata, methods=["POST"]
        )

        # Stream for server -> client updates through Server-Sent Events
        def stream():
            def eventStream():
                while True:
                    msg = self.eventQueue.get()  # blocks until a new message arrives
                    yield msg

            return Response(eventStream(), mimetype="text/event-stream")

        self.app.add_url_rule(route + "/stream", route + "_stream", stream)

    def send_sse_message(self, data, event_type="sync"):
        # Send to the client
        data["time"] = time.time()
        sse_string = f"event: {event_type}\ndata: {json.dumps(data)}\n\n"
        self.eventQueue.put(sse_string)
