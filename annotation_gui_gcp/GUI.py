import os
import random
import subprocess
import sys
import time
import tkinter as tk

import matplotlib

matplotlib.use("TkAgg")

from image_sequence_view import ImageSequenceView

FONT = "TkFixedFont"


class Gui:
    def __init__(self, master, gcp_manager, image_manager, sequence_groups=()):
        self.master = master
        self.gcp_manager = gcp_manager
        self.image_manager = image_manager
        self.curr_point = None
        self.quick_save_filename = None
        self.shot_std = {}
        self.sequence_groups = sequence_groups
        self.path = self.gcp_manager.path

        master.bind_all("q", lambda event: self.go_to_worst_gcp())
        master.bind_all("z", lambda event: self.toggle_zoom_all_views())
        master.bind_all("x", lambda event: self.toggle_sticky_zoom())
        master.bind_all("a", lambda event: self.go_to_current_gcp())
        master.bind_all("w", lambda event: self.go_to_next_image_all())
        master.bind_all("s", lambda event: self.go_to_prev_image_all())
        self.create_ui()
        master.lift()

        p_default_gcp = self.path + "/ground_control_points.json"
        if os.path.exists(p_default_gcp):
            self.load_gcps(p_default_gcp)
        p_shot_std = self.path + "/shots_std.csv"
        if os.path.exists(p_shot_std):
            self.load_shot_std(p_shot_std)

        for view in self.views:
            view.populate_image_list()
            view.bring_new_image(view.images_in_list[0])

    def create_ui(self):
        tools_frame = tk.Frame(self.master)
        tools_frame.pack(side="left", expand=0, fill=tk.Y)
        self.create_tools(tools_frame)
        self.create_image_views()
        self.master.update_idletasks()
        self.arrange_ui_onerow()

    def arrange_ui_onerow(self):
        master = self.master
        # Arrange views on the screen. All views on one single row
        w = master.winfo_width()
        h = master.winfo_height()
        master.geometry("%dx%d+%d+%d" % (w, h, 0, 0))
        x = master.winfo_rootx()
        y = master.winfo_rooty()
        x, y = 0, y + h
        screen_width = master.winfo_screenwidth()
        screen_height = master.winfo_screenheight()
        w = screen_width / len(self.views)
        h = screen_height - y
        for view in self.views:
            view.window.geometry("%dx%d+%d+%d" % (w, h, x, y))
            x += w

    def create_tools(self, master):
        gcp_list_frame = tk.Frame(master)
        gcp_list_frame.pack(side="top", fill=tk.BOTH, expand=1)

        self.gcp_list = tk.StringVar()
        self.gcp_list_box = tk.Listbox(
            gcp_list_frame,
            font=FONT,
            width=12,
            selectmode="browse",
            listvariable=self.gcp_list,
        )
        self.gcp_list_box.pack(side="left", expand=True, fill=tk.BOTH)
        self.gcp_list_box.bind("<<ListboxSelect>>", self.onclick_gcp_list)

        plus_minus_frame = tk.Frame(master)
        plus_minus_frame.pack(side="top")
        add_button = tk.Button(plus_minus_frame, text="Add GCP", command=self.add_gcp)
        add_button.pack(side="left")
        remove_button = tk.Button(
            plus_minus_frame, text="Remove GCP", command=self.remove_gcp
        )
        remove_button.pack(side="left")

        self.sticky_zoom = tk.BooleanVar(value=True)
        button = tk.Checkbutton(master, text="Sticky zoom (x)", var=self.sticky_zoom)
        button.pack(side="top")

        self.show_gcp_names = tk.BooleanVar(value=False)
        button = tk.Checkbutton(master, text="Show GCP names", var=self.show_gcp_names)
        button.pack(side="top")

        txt = tk.Label(master, text="Analysis")
        txt.pack(side="top")
        analysis_frame = tk.Frame(master)
        analysis_frame.pack(side="top")
        button = tk.Button(
            analysis_frame, text="3d-to-3d", command=self.analyze_3d_to_3d
        )
        button.pack(side="left")
        button = tk.Button(
            analysis_frame, text="3d-to-2d", command=self.analyze_3d_to_2d
        )
        button.pack(side="left")

        io_frame = tk.Frame(master)
        io_frame.pack(side="top")
        button = tk.Button(io_frame, text="Load", command=self.load_gcps)
        button.pack(side="left")
        button = tk.Button(io_frame, text="Save", command=self.save_gcps)
        button.pack(side="left")
        button = tk.Button(io_frame, text="Save As", command=self.save_gcps_as)
        button.pack(side="left")

    def create_image_views(self):
        self.views = []
        for sequence_key, image_keys in self.image_manager.seqs.items():
            v = ImageSequenceView(
                self, f"view {len(self.views)+1}", sequence_key, image_keys
            )
            self.views.append(v)

    def analyze_3d_to_3d(self):
        self.analyze(mode="3d_to_3d")

    def analyze_3d_to_2d(self):
        self.analyze(mode="3d_to_2d")

    def analyze(self, mode):
        # Check that there is a recent ground_control_points.json file
        t = time.time() - os.path.getmtime(self.path + "/ground_control_points.json")
        if t > 30:
            print("Please save before running the analysis")
            return

        # Call the run_ba script
        subprocess.run(
            [
                sys.executable,
                os.path.dirname(__file__) + "/run_ba.py",
                self.path,
                "--mode",
                mode,
            ]
        )
        self.shot_std = {}
        self.load_shot_std(self.path + "/shots_std.csv")
        p_gcp_errors = self.path + "/gcp_reprojections.json"
        self.gcp_manager.load_gcp_reprojections(p_gcp_errors)

        for view in self.views:
            view.populate_image_list()

        print("Done analyzing")

    def load_shot_std(self, path):
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
        for view in self.views:
            view.display_points()
        self.populate_gcp_list()

    def add_gcp(self):
        self.curr_point = self.gcp_manager.add_point()
        self.populate_gcp_list()
        return self.curr_point

    def toggle_sticky_zoom(self):
        if self.sticky_zoom.get():
            self.sticky_zoom.set(False)
        else:
            self.sticky_zoom.set(True)

    def toggle_zoom_all_views(self):
        # Zoom in/out on every view, centered on the location of the current GCP
        if self.curr_point is None:
            return
        any_zoomed_in = any(view.zoomed_in for view in self.views)
        for view in self.views:
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
        self.master.update_idletasks()

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
        defaultbg = self.master.cget("bg")
        for ix, point_id in enumerate(sorted(self.gcp_manager.points)):
            bg = "green" if self.curr_point == point_id else defaultbg
            self.gcp_list_box.itemconfig(ix, bg=bg)

    def remove_gcp(self):
        to_be_removed_point = self.curr_point
        if not to_be_removed_point:
            return
        self.curr_point = None

        self.gcp_manager.remove_gcp(to_be_removed_point)
        for view in self.views:
            view.display_points()

        self.populate_gcp_list()

    def onclick_gcp_list(self, event):
        widget = event.widget
        selection = widget.curselection()
        if not selection:
            return
        value = widget.get(int(selection[0]))
        if value == "none":
            self.curr_point = None
        else:
            self.curr_point = value.split(" ")[1]

        for view in self.views:
            view.display_points()
            if self.curr_point:
                view.highlight_gcp_reprojection(self.curr_point, zoom=False)

        self.update_gcp_list_highlight()

    def save_gcps(self):
        if self.quick_save_filename is None:
            return self.save_gcps_as()
        else:
            return self.gcp_manager.write_to_file(self.quick_save_filename)

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

    def go_to_next_image_all(self):
        for view in self.views:
            view.go_to_adjacent_image(+1, linked=False)

    def go_to_prev_image_all(self):
        for view in self.views:
            view.go_to_adjacent_image(-1, linked=False)

    def go_to_current_gcp(self):
        """
        Jumps to the currently selected GCP in all views where it was not visible
        """
        if not self.curr_point:
            return
        shots_gcp_seen = {
            p["shot_id"] for p in self.gcp_manager.points[self.curr_point]
        }
        for view in self.views:
            shots_gcp_seen_this_view = list(
                shots_gcp_seen.intersection(view.image_keys)
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
        worst_gcp = self.gcp_manager.compute_gcp_errors()[0]
        if worst_gcp is None:
            return

        self.curr_point = worst_gcp
        self.gcp_list_box.selection_clear(0, "end")
        for ix, gcp_id in enumerate(self.gcp_list_box.get(0, "end")):
            if worst_gcp in gcp_id:
                self.gcp_list_box.selection_set(ix)
                break

        for view in self.views:
            # Get the shot with worst reprojection error that in this view
            shot_worst_gcp = self.gcp_manager.shot_with_max_gcp_error(
                view.image_keys, worst_gcp
            )
            if shot_worst_gcp:
                view.bring_new_image(shot_worst_gcp)
