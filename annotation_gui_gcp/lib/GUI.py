import os
import random
import subprocess
import sys
import time
from collections import defaultdict

import flask
from annotation_gui_gcp.lib.views.cad_view import CADView
from annotation_gui_gcp.lib.views.cp_finder_view import ControlPointFinderView
from annotation_gui_gcp.lib.views.image_view import ImageView
from annotation_gui_gcp.lib.views.tools_view import ToolsView
from opensfm import dataset


class Gui:
    def __init__(
        self,
        app,
        gcp_manager,
        image_manager,
        rig_groups=None,
        cad_paths=(),
    ):
        self.gcp_manager = gcp_manager
        self.image_manager = image_manager
        self.curr_point = None
        self.shot_std = {}
        self.rig_groups = rig_groups if rig_groups else {}
        self.path = self.gcp_manager.path
        self.app = app
        self.ix_a = 0
        self.ix_b = 1

        self.reconstruction_options = self.get_reconstruction_options()
        self.create_ui(cad_paths)

        self.load_gcps()

        self.load_analysis_results(self.ix_a, self.ix_b)

    def get_reconstruction_options(self):
        p_recs = self.path + "/reconstruction.json" if self.path else None
        if p_recs is None or not os.path.exists(p_recs):
            return ["NONE", "NONE"]
        data = dataset.DataSet(self.path)
        recs = data.load_reconstruction()
        options = []
        for ix, rec in enumerate(recs):
            camcount = defaultdict(int)
            for shot in rec.shots.values():
                camcount[shot.camera.id] += 1
            str_repr = f"REC#{ix}: " + ", ".join(
                f"{k}({v})" for k, v in camcount.items()
            )
            options.append(str_repr)
        options.append("None (3d-to-2d)")
        return options

    def sync_to_client(self) -> None:
        for view in self.sequence_views + self.cad_views + [self.tools_view]:
            view.sync_to_client()

    def create_ui(self, cad_paths):
        subpane_routes = []
        has_views_that_need_tracking = len(cad_paths) > 0
        self.tools_view = ToolsView(self, self.app)

        self.sequence_views = []
        for ix, image_keys in enumerate(self.image_manager.seqs.values()):
            v = ImageView(
                self,
                self.app,
                f"/sequence_view_{ix+1}",
                image_keys,
                has_views_that_need_tracking,
            )
            self.sequence_views.append(v)

        cp_view = ControlPointFinderView(self, self.app)
        self.sequence_views.append(cp_view)

        self.cad_views = []
        for ix, cad_path in enumerate(cad_paths):
            v = CADView(self, self.app, f"/cad_view_{ix+1}", cad_path)
            self.cad_views.append(v)

        subpane_routes = [
            v.route_prefix for v in (self.sequence_views + self.cad_views)
        ]

        @self.app.route("/")
        def send_main_page():
            return flask.render_template("mosaic.html", subpane_routes=subpane_routes)

    def analyze_rigid(self):
        self.analyze(rigid=True, covariance=False)

    def analyze_flex(self):
        self.analyze(rigid=False, covariance=False)

    def analyze(self, rigid=False, covariance=True):
        t = time.time() - os.path.getmtime(self.path + "/ground_control_points.json")
        # ix_a = self.reconstruction_options.index(self.rec_a.get())
        # ix_b = self.reconstruction_options.index(self.rec_b.get())
        ix_a = self.ix_a
        ix_b = self.ix_b
        if t > 30:
            print(
                "Please save to ground_control_points.json before running the analysis"
            )
            return

        args = [
            sys.executable,
            os.path.join(os.path.dirname(os.path.dirname(__file__)), "run_ba.py"),
            self.path,
            "--rec_a",
            str(ix_a),
        ]
        if ix_b < len(self.reconstruction_options) - 1:
            args.extend(("--rec_b", str(ix_b)))
        else:
            ix_b = None

        if rigid:
            args.extend(["--rigid"])

        if covariance:
            args.extend(["--covariance"])

        # Call the run_ba script
        subprocess.run(args)

        # Load the results
        self.load_analysis_results(ix_a, ix_b)
        for view in self.sequence_views:
            view.populate_image_list()

        print("Done analyzing")

    def load_analysis_results(self, ix_a, ix_b):
        self.load_shot_std(f"{self.path}/shots_std_{ix_a}x{ix_b}.csv")
        p_gcp_errors = f"{self.path}/gcp_reprojections_{ix_a}x{ix_b}.json"
        self.gcp_manager.load_gcp_reprojections(p_gcp_errors)

    def load_shot_std(self, path):
        self.shot_std = {}
        if os.path.isfile(path):
            with open(path, "r") as f:
                for line in f:
                    shot, std = line[:-1].split(",")
                    self.shot_std[shot] = float(std)

    def load_gcps(self, filename=None):
        self.gcp_manager.load_from_file(filename)
        for view in self.sequence_views + self.cad_views:
            view.display_points()
            view.populate_image_list()
        self.populate_gcp_list()

    def add_gcp(self):
        new_gcp = self.gcp_manager.add_point()
        self.populate_gcp_list()
        self.update_active_gcp(new_gcp)

    def toggle_sticky_zoom(self):
        if self.sticky_zoom.get():
            self.sticky_zoom.set(False)
        else:
            self.sticky_zoom.set(True)

    def populate_gcp_list(self):
        pass

    def remove_gcp(self):
        to_be_removed_point = self.curr_point
        if not to_be_removed_point:
            return
        self.gcp_manager.remove_gcp(to_be_removed_point)
        self.populate_gcp_list()
        self.update_active_gcp(None)

    def update_active_gcp(self, new_active_gcp):
        print("Active GCP is now", new_active_gcp)
        self.curr_point = new_active_gcp
        for view in self.sequence_views + self.cad_views:
            view.display_points()
            if self.curr_point:
                view.highlight_gcp_reprojection(self.curr_point, zoom=False)

    def save_gcps(self, filename=None):
        self.gcp_manager.write_to_file(filename)

    def go_to_current_gcp(self):
        """
        Jumps to the currently selected GCP in all views where it was not visible
        """
        if not self.curr_point:
            return
        shots_gcp_seen = {
            p["shot_id"] for p in self.gcp_manager.points[self.curr_point]
        }
        for view in self.sequence_views:
            shots_gcp_seen_this_view = list(
                shots_gcp_seen.intersection(view.images_in_list)
            )
            if (
                len(shots_gcp_seen_this_view) > 0
                and view.current_image not in shots_gcp_seen
            ):
                target_shot = random.choice(shots_gcp_seen_this_view)
                view.bring_new_image(target_shot)

    def go_to_worst_gcp(self):
        if len(self.gcp_manager.gcp_reprojections) == 0:
            print("No GCP reprojections available. Can't jump to worst GCP")
            return
        worst_gcp = self.gcp_manager.get_worst_gcp()
        if worst_gcp is None:
            return

        self.curr_point = worst_gcp
        self.gcp_list_box.selection_clear(0, "end")
        for ix, gcp_id in enumerate(self.gcp_list_box.get(0, "end")):
            if worst_gcp in gcp_id:
                self.gcp_list_box.selection_set(ix)
                break

        for view in self.sequence_views:
            # Get the shot with worst reprojection error that in this view
            shot_worst_gcp = self.gcp_manager.shot_with_max_gcp_error(
                view.images_in_list, worst_gcp
            )
            if shot_worst_gcp:
                view.bring_new_image(shot_worst_gcp)

    def clear_latlon_sources(self, view):
        # The user has activated the 'Track this' checkbox in some view
        for v in self.sequence_views:
            if v is not view:
                v.is_latlon_source.set(False)

    def refocus_overhead_views(self, lat, lon):
        for view in self.cad_views:
            view.refocus(lat, lon)
