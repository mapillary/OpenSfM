from __future__ import annotations

import abc
import json
import time
from collections.abc import Iterator
from queue import Queue
from typing import Any, TYPE_CHECKING

from flask import jsonify, render_template, request, Response

if TYPE_CHECKING:
    from flask import Flask

    from ..GUI import Gui

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
    def __init__(self, main_ui: Gui, web_app: Flask, route_prefix: str) -> None:
        self.main_ui = main_ui
        self.app = web_app
        self.eventQueue: Queue[str] = Queue()
        self.latlons = main_ui.image_manager.load_latlons()
        self.register_routes(route_prefix)
        self.route_prefix = route_prefix

    @abc.abstractclassmethod
    def sync_to_client(self) -> None:
        pass

    @abc.abstractclassmethod
    def process_client_message(self, data: dict[str, Any]) -> None:
        pass

    def template_name(self) -> str:
        class_name = type(self).__name__
        return class_name

    def register_routes(self, route: str) -> None:
        def send_main_page() -> str:
            template = self.template_name()
            return render_template(f"{template}.html", class_name=template)

        self.app.add_url_rule(route, route + "_index", send_main_page)

        def postdata() -> Response:
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
        def stream() -> Response:
            def eventStream() -> Iterator[str]:
                while True:
                    msg = self.eventQueue.get()  # blocks until a new message arrives
                    yield msg

            return Response(eventStream(), mimetype="text/event-stream")

        self.app.add_url_rule(route + "/stream", route + "_stream", stream)

    def send_sse_message(self, data: dict[str, Any], event_type: str = "sync") -> None:
        # Send to the client
        data["time"] = time.time()
        sse_string = f"event: {event_type}\ndata: {json.dumps(data)}\n\n"
        self.eventQueue.put(sse_string)
