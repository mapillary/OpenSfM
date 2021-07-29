
from web.web_view import WebView

class ToolsView(WebView):
    def __init__(
        self,
        main_ui,
        port=4990,
    ):
        super().__init__(main_ui)
        self.start(port)

    def page_source(self):
        return "ToolsView.html"

    def image_filename(self):
        pass

    def process_message(self, data):
        print("DATA : ", data)

        if data['event'] == "sync":
            print("Event: sync to client");

        self.sync_to_client()

