from annotation_gui_gcp.lib.views.web_view import WebView


class ToolsView(WebView):
    def __init__(self, main_ui, web_app):
        super().__init__(main_ui, web_app, "/tools")
        self.main_ui = main_ui

    def process_client_message(self, data):
        # Got some input from the frontend
        if data["event"] == "select_cp":
            self.main_ui.update_active_gcp(data["point_id"])
        elif data["event"] == "delCP":
            self.main_ui.remove_gcp()
        elif data["event"] == "addCP":
            self.main_ui.add_gcp()
        elif data["event"] == "save":
            p_default_gcp = self.main_ui.path + "/ground_control_points.json"
            self.main_ui.save_gcps(p_default_gcp)
        elif data["event"] == "load":
            p_default_gcp = self.main_ui.path + "/ground_control_points.json"
            self.main_ui.load_gcps(p_default_gcp)
        elif data["event"] == "flex":
            self.main_ui.analyze_flex()
        elif data["event"] == "rigid":
            self.main_ui.analyze_rigid()
        elif data["event"] == "full":
            self.main_ui.analyze()

    def sync_to_client(self):
        # Sync state to frontend
        """
        Sends all the data required to initialize or sync the tools view
        """
        data = {
            "points": self.main_ui.gcp_manager.points,
            "selected_point": self.main_ui.curr_point,
        }
        self.send_sse_message(data)
