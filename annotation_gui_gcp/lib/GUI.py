import os
import random
import subprocess
import sys
import time
import tkinter as tk
from collections import defaultdict

import matplotlib
from opensfm import dataset

matplotlib.use("TkAgg")

from annotation_gui_gcp.lib.cad_viewer.cad_view import CadView
from annotation_gui_gcp.lib.image_sequence_view import ImageSequenceView
from annotation_gui_gcp.lib.orthophoto_view import OrthoPhotoView

FONT = "TkFixedFont"


class Gui:
    def __init__(
        self,
        parent,
        gcp_manager,
        image_manager,
        rig_groups=None,
        ortho_paths=(),
        cad_paths=(),
    ):
        self.parent = parent
        self.gcp_manager = gcp_manager
        self.image_manager = image_manager
        self.curr_point = None
        self.quick_save_filename = None
        self.shot_std = {}
        self.rig_groups = rig_groups if rig_groups else {}
        self.path = self.gcp_manager.path

        parent.bind_all("q", lambda event: self.go_to_worst_gcp())
        parent.bind_all("z", lambda event: self.toggle_zoom_all_views())
        parent.bind_all("x", lambda event: self.toggle_sticky_zoom())
        parent.bind_all("a", lambda event: self.go_to_current_gcp())
        self.reconstruction_options = self.get_reconstruction_options()
        self.create_ui(ortho_paths, cad_paths)
        parent.lift()

        p_default_gcp = self.path + "/ground_control_points.json"
        if os.path.exists(p_default_gcp):
            self.load_gcps(p_default_gcp)
        self.load_analysis_results(0, 1)

    def get_reconstruction_options(self):
        p_recs = self.path + "/reconstruction.json"
        if not os.path.exists(p_recs):
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

    def create_ui(self, ortho_paths, cad_paths):
        tools_frame = tk.Frame(self.parent)
        tools_frame.pack(side="left", expand=0, fill=tk.Y)
        self.create_tools(tools_frame)
        has_views_that_need_tracking = len(ortho_paths) > 0 or len(cad_paths) > 0
        self.create_sequence_views(show_track_checkbox=has_views_that_need_tracking)
        self.ortho_views = []
        if ortho_paths:
            v = self.sequence_views[0]
            k = v.current_image
            latlon = v.latlons[k]
            self.create_ortho_views(ortho_paths, latlon["lat"], latlon["lon"])

        self.cad_views = [
            CadView(self, cad_path, 5000 + ix) for ix, cad_path in enumerate(cad_paths)
        ]

        self.parent.update_idletasks()
        # self.arrange_ui_onerow()

    def rec_ix_changed(self, *args):
        # Load analysis for the new reconstruction pair if it exists
        self.ix_a = self.reconstruction_options.index(self.rec_a.get())
        self.ix_b = self.reconstruction_options.index(self.rec_b.get())
        if self.ix_b == len(self.reconstruction_options) - 1:
            self.ix_b = None
        print(
            f"Loading analysis results for #{self.ix_a}:{self.rec_a.get()} vs #{self.ix_b}:{self.rec_b.get()}"
        )
        self.load_analysis_results(self.ix_a, self.ix_b)
        for view in self.sequence_views:
            view.populate_image_list()

    def arrange_ui_onerow(self):
        parent = self.parent
        # Arrange views on the screen. All views on one single row
        w = parent.winfo_width()
        h = parent.winfo_height()
        parent.geometry("%dx%d+%d+%d" % (w, h, 0, 0))
        x = parent.winfo_rootx()
        y = parent.winfo_rooty()
        x, y = 0, y + h
        screen_width = parent.winfo_screenwidth()
        screen_height = parent.winfo_screenheight()
        w = screen_width / len(self.sequence_views)
        h = screen_height - y
        for view in self.sequence_views:
            view.window.geometry("%dx%d+%d+%d" % (w, h, x, y))
            x += w

    def create_tools(self, parent):
        width = 15
        gcp_list_frame = tk.Frame(parent)
        gcp_list_frame.pack(side="top", fill=tk.BOTH, expand=1)

        self.gcp_list = tk.StringVar()
        self.gcp_list_box = tk.Listbox(
            gcp_list_frame,
            font=FONT,
            width=width,
            selectmode="browse",
            listvariable=self.gcp_list,
        )
        self.gcp_list_box.pack(side="left", expand=True, fill=tk.BOTH)
        self.gcp_list_box.bind("<<ListboxSelect>>", self.onclick_gcp_list)

        plus_minus_frame = tk.Frame(parent)
        plus_minus_frame.pack(side="top")
        add_button = tk.Button(plus_minus_frame, text="Add GCP", command=self.add_gcp)
        add_button.pack(side="left")
        remove_button = tk.Button(
            plus_minus_frame, text="Remove GCP", command=self.remove_gcp
        )
        remove_button.pack(side="left")

        self.sticky_zoom = tk.BooleanVar(value=False)
        button = tk.Checkbutton(parent, text="Sticky zoom (x)", var=self.sticky_zoom)
        button.pack(side="top")

        self.show_gcp_names = tk.BooleanVar(value=False)
        button = tk.Checkbutton(parent, text="Show GCP names", var=self.show_gcp_names)
        button.pack(side="top")

        txt = tk.Label(parent, text="Analysis")
        txt.pack(side="top")
        analysis_frame = tk.Frame(parent)
        analysis_frame.pack(side="top")

        options = self.reconstruction_options
        self.ix_a = 0
        self.rec_a = tk.StringVar(parent)
        self.rec_a.set(options[0])
        self.rec_a.trace("w", self.rec_ix_changed)
        w = tk.OptionMenu(analysis_frame, self.rec_a, *options[:-1])
        w.pack(side="top", fill=tk.X)
        w.config(width=width)

        self.ix_b = None
        self.rec_b = tk.StringVar(parent)
        self.rec_b.set(options[1])
        self.rec_b.trace("w", self.rec_ix_changed)
        w = tk.OptionMenu(analysis_frame, self.rec_b, *options)
        w.pack(side="top", fill=tk.X)
        w.config(width=width)

        analysis_buttons_frame = tk.Frame(analysis_frame)
        analysis_buttons_frame.pack(side="top")
        button = tk.Button(
            analysis_buttons_frame, text="Rigid", command=self.analyze_rigid
        )
        button.pack(side="left")
        button = tk.Button(
            analysis_buttons_frame, text="Flex", command=self.analyze_flex
        )
        button.pack(side="left")
        button = tk.Button(analysis_buttons_frame, text="Full", command=self.analyze)
        button.pack(side="right")

        io_frame = tk.Frame(parent)
        io_frame.pack(side="top")
        button = tk.Button(io_frame, text="Load", command=self.load_gcps)
        button.pack(side="left")
        button = tk.Button(io_frame, text="Save", command=self.save_gcps)
        button.pack(side="left")
        button = tk.Button(io_frame, text="Save As", command=self.save_gcps_as)
        button.pack(side="left")

    def create_ortho_views(self, ortho_paths, lat, lon):
        for ortho_p in ortho_paths:
            v = OrthoPhotoView(
                self,
                ortho_p,
                init_lat=lat,
                init_lon=lon,
                is_the_geo_reference=ortho_p is ortho_paths[0],
            )
            self.ortho_views.append(v)

    def create_sequence_views(self, show_track_checkbox):
        self.sequence_views = []
        for sequence_key, image_keys in self.image_manager.seqs.items():
            v = ImageSequenceView(self, sequence_key, image_keys, show_track_checkbox)
            self.sequence_views.append(v)

    def analyze_rigid(self):
        self.analyze(rigid=True, covariance=False)

    def analyze_flex(self):
        self.analyze(rigid=False, covariance=False)

    def analyze(self, rigid=False, covariance=True):
        t = time.time() - os.path.getmtime(self.path + "/ground_control_points.json")
        ix_a = self.reconstruction_options.index(self.rec_a.get())
        ix_b = self.reconstruction_options.index(self.rec_b.get())
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
        if filename is None:
            filename = tk.filedialog.askopenfilename(
                title="Open GCP file",
                initialdir=self.path,
                filetypes=(("JSON files", "*.json"), ("all files", "*.*")),
            )
        if filename is None:
            return
        self.quick_save_filename = filename
        self.gcp_manager.load_from_file(filename)
        for view in self.sequence_views + self.ortho_views + self.cad_views:
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

    def toggle_zoom_all_views(self):
        # Zoom in/out on every view, centered on the location of the current GCP
        if self.curr_point is None:
            return
        any_zoomed_in = any(view.zoomed_in for view in self.sequence_views)
        for view in self.sequence_views:
            if any_zoomed_in:
                view.zoom_out()
            else:
                for projection in self.gcp_manager.points[self.curr_point]:
                    if projection["shot_id"] == view.current_image:
                        x, y = projection["projection"]
                        view.zoom_in(x, y)
                        break
            view.canvas.draw_idle()

    def populate_gcp_list(self):
        self.update_gcp_list_text()
        self.update_gcp_list_highlight()
        self.parent.update_idletasks()

    def update_gcp_list_text(self):
        if not self.gcp_manager.points:
            return
        items = []
        n_digits = max(len(str(len(v))) for v in self.gcp_manager.points.values())
        for point_id in sorted(self.gcp_manager.points):
            n_annotations = len(self.gcp_manager.points[point_id])
            txt = "{:0{n}} {}".format(n_annotations, point_id, n=n_digits)
            items.append(txt)
        self.gcp_list.set(items)

    def update_gcp_list_highlight(self):
        if not self.curr_point:
            return
        defaultbg = self.parent.cget("bg")
        for ix, point_id in enumerate(sorted(self.gcp_manager.points)):
            bg = "green" if self.curr_point == point_id else defaultbg
            self.gcp_list_box.itemconfig(ix, bg=bg)

    def remove_gcp(self):
        to_be_removed_point = self.curr_point
        if not to_be_removed_point:
            return
        self.gcp_manager.remove_gcp(to_be_removed_point)
        self.populate_gcp_list()
        self.update_active_gcp(None)

    def update_active_gcp(self, new_active_gcp):
        self.curr_point = new_active_gcp
        for view in self.sequence_views + self.ortho_views + self.cad_views:
            view.display_points()
            if self.curr_point:
                view.highlight_gcp_reprojection(self.curr_point, zoom=False)

        self.update_gcp_list_highlight()

    def onclick_gcp_list(self, event):
        widget = event.widget
        selection = widget.curselection()
        if not selection:
            return
        value = widget.get(int(selection[0]))
        curr_point = value.split(" ")[1] if value != "none" else None
        self.update_active_gcp(curr_point)

    def save_gcps(self):
        if self.quick_save_filename is None:
            self.save_gcps_as()
        else:
            self.gcp_manager.write_to_file(self.quick_save_filename)
            parent = os.path.dirname(self.quick_save_filename)
            dirname = os.path.basename(parent)
            self.gcp_manager.write_to_file(os.path.join(parent, dirname + ".json"))

    def save_gcps_as(self):
        filename = tk.filedialog.asksaveasfilename(
            initialfile="ground_control_points.json",
            title="Save GCP file",
            initialdir=self.path,
            defaultextension=".json",
        )
        if filename is None:
            return
        else:
            self.quick_save_filename = filename
            return self.save_gcps()

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
        for v in self.sequence_views + self.ortho_views:
            if v is not view:
                v.is_latlon_source.set(False)

    def refocus_overhead_views(self, lat, lon):
        for view in self.ortho_views + self.cad_views:
            view.refocus(lat, lon)
