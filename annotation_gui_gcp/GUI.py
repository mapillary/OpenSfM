import matplotlib.patches as mpatches
import tkinter as tk
from tkinter import filedialog
import time
import matplotlib
import os
import subprocess
import sys
import numpy as np
import random

from apps import distinct_colors, read_gcp_file, id_generator

matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
from matplotlib.patches import PathPatch
from matplotlib.text import TextPath
from matplotlib.transforms import IdentityTransform
from matplotlib.offsetbox import AnnotationBbox, AuxTransformBox
from matplotlib import pyplot as plt
from geometry import get_all_track_observations, get_tracks_visible_in_image

from apps import CustomListbox

PREVIOUS_UNICODE = u"\u2193"
NEXT_UNICODE = u"\u2191"
FONT = "TkFixedFont"


class NavigationToolbar(NavigationToolbar2Tk):
    def set_message(self, m):
        pass


class Gui:

    def __init__(self, master, database, n_views, sequence_groups=()):
        self.database = database
        self.point_idx = 0
        self.match_idx = 0
        self.curr_point = None
        self.last_saved_filename = None
        self.shot_std = {}
        self.sequence_groups = sequence_groups

        master.bind_all('q', lambda event: self.go_to_worst_gcp())
        master.bind_all('z', lambda event: self.toggle_zoom_on_key())
        master.bind_all('x', lambda event: self.toggle_sticky_zoom())
        master.bind_all('a', lambda event: self.go_to_current_gcp())
        master.bind_all('w', lambda event: self.go_to_next_image_all())
        master.bind_all('s', lambda event: self.go_to_prev_image_all())
        self.create_ui(master, n_views=n_views)
        master.lift()

        p_default_gcp = self.database.path + '/ground_control_points.json'
        if os.path.exists(p_default_gcp):
            self.load_gcps(p_default_gcp)
        p_shot_std = self.database.path + '/shots_std.csv'
        if os.path.exists(p_shot_std):
            self.load_shot_std(p_shot_std)

        for view in self.views:
            self.populate_sequence_list(view)

    def create_ui(self, master, n_views):
        self.master = master

        tools_frame = tk.Frame(master)
        tools_frame.pack(side='left', expand=0, fill=tk.Y)
        self.create_tools(tools_frame)

        self.views = []
        for view_ix in range(n_views):
            new_view = tk.Toplevel(self.master, name=f"view_ix_{view_ix}")
            new_view.title(f"View {view_ix+1}")
            self.views.append(new_view)

        self.init_image_views()
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
            view.geometry("%dx%d+%d+%d" % (w, h, x, y))
            x += w

    def create_tools(self, master):
        gcp_list_frame = tk.Frame(master)
        gcp_list_frame.pack(side='top', fill=tk.BOTH, expand=1)
        self.gcp_list_box = CustomListbox(gcp_list_frame, font=FONT)
        self.gcp_list_box.pack(side='left', expand=True, fill=tk.Y)
        self.gcp_list_box.bind('<<ListboxSelect>>', self.onclick_gcp_list)

        plus_minus_frame = tk.Frame(master)
        plus_minus_frame.pack(side='top')
        add_button = tk.Button(plus_minus_frame, text="Add GCP", command=self.add_gcp)
        add_button.pack(side='left')
        remove_button = tk.Button(plus_minus_frame, text="Remove GCP", command=self.remove_gcp)
        remove_button.pack(side='left')

        # self.if_show_epipolar = tk.IntVar(value=0)
        # self.check_button = tk.Checkbutton(master, text="Epipolar lines",
        #                                    var=self.if_show_epipolar)
        # self.check_button.pack(side='top')
        self.sticky_zoom = tk.BooleanVar(value=True)
        sticky_zoom_button = tk.Checkbutton(master, text="Sticky zoom (x)", var=self.sticky_zoom)
        sticky_zoom_button.pack(side='top')

        # self.check_button.pack(side='top')

        load_button = tk.Button(master, text="Analyze", command=self.analyze)
        load_button.pack(side="top")

        io_frame = tk.Frame(master)
        io_frame.pack(side="top")
        load_button = tk.Button(io_frame, text="Load", command=self.load_gcps)
        load_button.pack(side="left")
        save_button = tk.Button(io_frame, text="Save", command=self.save_gcps)
        save_button.pack(side="left")
        save_button = tk.Button(io_frame, text="Save As", command=self.save_gcps_as)
        save_button.pack(side="left")

    def clear_artists(self, view):
        for artist in view.plt_artists:
            artist.set_visible(False)
            del artist

    def set_title(self, view):
        shot = view.current_image
        seq_key = view.sequence_key
        seq = self.database.seqs[seq_key]
        seq_ix = seq.index(shot)
        title = "{} [{}/{}]: {}".format(seq_key, seq_ix+1, len(seq), shot)
        view.title(title)

    def populate_sequence_list(self, view):
        image_keys = view.image_keys
        view.sequence_list_box.delete(0, tk.END)
        n_digits = len(str(len(image_keys)))
        defaultbg = view.cget('bg')
        for ix, shot in enumerate(image_keys):
            points = self.database.get_visible_points_coords(shot)
            txt = "{:0{n_digits}} {}".format(ix+1, len(points), n_digits=n_digits)
            shot_std = self.shot_std.get(shot, None)
            if shot_std:
                txt += " {:.2f}".format(shot_std)
            view.sequence_list_box.insert(tk.END, txt)
            # Highlight current frame
            if shot == view.current_image:
                view.sequence_list_box.itemconfig(ix, bg='green')

    def onclick_sequence_list(self, event):
        widget = event.widget
        sel = widget.curselection()
        if not sel:
            return
        view = widget.master.master.master
        self.go_to_image_index(view, int(sel[0]))

    def init_image_views(self, nav_buttons=False):
        for idx, view in enumerate(self.views):
            canvas_frame = tk.Frame(view)
            canvas_frame.pack(side='left', fill=tk.BOTH, expand=1)
            view.canvas_frame = canvas_frame

            if nav_buttons:
                button_frame = tk.Frame(canvas_frame)
                button_frame.pack(side='top')
                prv_btn = tk.Button(button_frame, text=PREVIOUS_UNICODE,
                                    command=lambda _idx=idx: self.go_to_prev_image(self.views[_idx]))
                prv_btn.pack(side='left')
                nxt_btn = tk.Button(button_frame, text=NEXT_UNICODE,
                                    command=lambda _idx=idx: self.go_to_next_image(self.views[_idx]))
                nxt_btn.pack(side='left')

            view.image_keys = list(self.database.seqs.values())[idx]
            view.sequence_key = list(self.database.seqs.keys())[idx]
            view.current_image = view.image_keys[0]

            toolbox = tk.Frame(canvas_frame)
            toolbox.pack(side='left', expand=False, fill=tk.Y)
            auto_gcp_button = tk.Button(toolbox, text="Auto GCP", command=lambda view=view: self.auto_gcp_show_tracks(view))
            auto_gcp_button.pack(side="top")
            view.visible_tracks = None
            view.sequence_list_box = CustomListbox(toolbox, font=FONT, width=12)
            view.sequence_list_box.pack(side='top', expand=True, fill=tk.Y)
            view.sequence_list_box.bind('<<ListboxSelect>>', self.onclick_sequence_list)


            view.figure = Figure()
            view.subplot = view.figure.add_subplot(111)
            view.subplot.imshow(self.database.get_image(view.current_image), aspect='auto')
            view.subplot.axis('scaled')
            view.figure.set_tight_layout(True)
            view.subplot.axis('off')

            view.canvas = FigureCanvasTkAgg(view.figure, canvas_frame)
            view.canvas.draw()
            view.canvas.get_tk_widget().pack(side='top', fill=tk.BOTH, expand=1)
            view.canvas.mpl_connect('button_press_event',
                                       lambda event:
                                       self.on_press(event))
            view.canvas.mpl_connect('scroll_event',
                                       lambda event:
                                       self.on_scroll(event))

            view.zoomed_in = False
            view.last_seen_px = {}
            view.plt_artists = []
            self.set_title(view)


    def auto_gcp_show_tracks(self, view):
        h, w = self.database.get_image_size(view.current_image)
        tracks = get_tracks_visible_in_image(self.database, view.current_image)

        # Select some tracks to plot, not all
        # nice_tracks = sorted(tracks, key=lambda tid: -tracks[tid]['length'])
        nice_tracks = tracks.keys() # all tracks
        tracks = {k:tracks[k] for k in nice_tracks}

        if len(tracks) == 0:
            print("No valid tracks found")
            return

        # Draw track projections
        points = []
        track_lengths = []
        for t in tracks.values():
            points.append(t[0])
            track_lengths.append(t[1])

        points = np.array(points)
        norm = matplotlib.colors.Normalize()
        colors = plt.cm.viridis(norm(track_lengths))
        view.tracks_scatter = view.subplot.scatter(points[:,0], points[:,1], s=10, c=colors, marker='x')
        view.canvas.draw_idle()

        # The next left or right click will be handled differently by setting this:
        view.visible_tracks = tracks


    def auto_gcp_create(self, view, x, y, add):
        # Find track closest to click location and create a GCP from it
        if add:
            points, track_ids = [], []
            for tid, t in view.visible_tracks.items():
                points.append(t[0])
                track_ids.append(tid)
            dists = np.linalg.norm(np.array(points) - np.array([[x,y]]), axis=1)
            closest_track = track_ids[np.argmin(dists)]
            observations = get_all_track_observations(self.database, closest_track)

            new_gcp = self.add_gcp()
            print(f"Created new GCP {new_gcp} from track {closest_track} with {len(observations)} observations")
            for shot_id, point in observations.items():
                self.database.add_point_observation(new_gcp, shot_id, point)

        view.visible_tracks = None
        view.tracks_scatter.remove()
        view.canvas.draw()
        self.populate_sequence_list(view)


    def analyze(self):
        # Check that there is a recent ground_control_points.json file
        t = time.time() - os.path.getmtime(self.database.path + '/ground_control_points.json')
        if t > 30:
            print("Please save before running the analysis")
            return

        # Call the run_ba script
        subprocess.run([
            sys.executable,
            os.path.dirname(__file__) + '/run_ba.py',
            self.database.path
            ]
        )
        self.shot_std = {}
        self.load_shot_std(self.database.path + '/shots_std.csv')
        p_gcp_errors = self.database.path + '/gcp_reprojections.json'
        self.database.load_gcp_reprojections(p_gcp_errors)

        for view in self.views:
            self.populate_sequence_list(view)

        print("Done analyzing")


    def load_shot_std(self, path):
        with open(path, 'r') as f:
            for line in f:
                shot, std = line[:-1].split(',')
                self.shot_std[shot] = float(std)

    def load_gcps(self, filename=None):
        if filename is None:
            filename = filedialog.askopenfilename(
                title="Open GCP file",
                initialdir=self.database.path,
                filetypes=(("JSON files", "*.json"), ("all files", "*.*")),
            )
        if filename is None:
            return
        points = read_gcp_file(filename)
        self.last_saved_filename = filename
        self.database.init_points(points)
        for view in self.views:
            self.display_points(view)
            # if self.if_show_epipolar.get():
            #     self.show_epipolar_lines(view_ix)
        self.populate_gcp_list()

    def display_points(self, view):
        visible_points_coords = self.database.get_visible_points_coords(view.current_image)
        self.clear_artists(view)

        for point_id, coords in visible_points_coords.items():
            color = distinct_colors[divmod(hash(point_id), 19)[1]]
            text_path = TextPath((0, 0), point_id, size=12)
            p1 = PathPatch(text_path, transform=IdentityTransform(), alpha=1, color=color)
            offsetbox2 = AuxTransformBox(IdentityTransform())
            offsetbox2.add_artist(p1)
            artists = [
                AnnotationBbox(offsetbox2, ((coords[0] + 30), (coords[1] + 30)), bboxprops={"alpha":0.05}),
                mpatches.Circle((coords[0], coords[1]), 5, color=color, fill=False),
                mpatches.Circle((coords[0], coords[1]), 0.5, color=color, fill=True),
            ]

            if point_id == self.curr_point:
                artists.extend([
                    mpatches.Circle((coords[0], coords[1]), 10, color=color, fill=False),
                    mpatches.Circle((coords[0], coords[1]), 11, color=color, fill=False),
                    mpatches.Circle((coords[0], coords[1]), 12, color=color, fill=False),
                ])
            for art in artists:
                view.plt_artists.append(art)
                view.subplot.add_artist(art)

        view.figure.canvas.draw()

    def add_gcp(self):
        new_id = id_generator()
        while self.database.point_exists(new_id):
            new_id = id_generator()
        self.database.add_point(new_id)
        self.curr_point = new_id
        self.populate_gcp_list()
        return new_id

    def which_view(self, event):
        for view in self.views:
            if event.canvas == view.canvas:
                return view
        return None

    def on_scroll(self, event):
        view = self.which_view(event)
        if event.xdata is None or event.ydata is None:
            return
        if event.button == 'up':
            self.go_to_next_image(view)
        elif event.button == 'down':
            self.go_to_prev_image(view)

    def zoom_in(self, view, x, y):
        if view.zoomed_in:
            return
        xlim = view.subplot.get_xlim()
        width = max(xlim) - min(xlim)
        window_size = width / 10
        window_size = 100
        view.subplot.set_xlim(x - window_size/2, x + window_size/2)
        view.subplot.set_ylim(y + window_size/2, y - window_size/2)
        view.zoomed_in = True

    def zoom_out(self, view):
        view.subplot.autoscale()
        view.zoomed_in = False

    def toggle_sticky_zoom(self):
        if self.sticky_zoom.get():
            self.sticky_zoom.set(False)
        else:
            self.sticky_zoom.set(True)

    def toggle_zoom_on_key(self):
        # Zoom in/out on every view, centered on the location of the current GCP
        if self.curr_point is None:
            return
        any_zoomed_in = any(view.zoomed_in for view in self.views)
        for view in self.views:
            if any_zoomed_in:
                self.zoom_out(view)
            else:
                for projection in self.database.points[self.curr_point]:
                    if projection["shot_id"] == view.current_image:
                        x, y = projection["projection"]
                        self.zoom_in(view, x, y)
                        break
            view.canvas.draw_idle()


    def add_move_or_remove_gcp(self, view, x, y, leftclick):
        current_image = view.current_image
        if self.curr_point is None:
            return
        self.database.remove_point_observation(self.curr_point, current_image)

        for line in view.subplot.lines:
            line.remove()

        if leftclick: # Left click
            self.database.add_point_observation(self.curr_point, current_image, (x, y))
            self.zoom_in(view, x, y)

        # if self.if_show_epipolar.get():
        #     self.show_epipolar_lines(idx)


    def on_press(self, event):
        view = self.which_view(event)
        x, y = event.xdata, event.ydata
        if None in (x, y):
            return
        if event.button == 2:  # Middle / wheel click:
            if view.zoomed_in:
                self.zoom_out(view)
            else:
                self.zoom_in(view, x, y)
            view.figure.canvas.draw_idle()
        elif event.button in (1, 3):
            # Left click or right click
            view.last_seen_px[self.curr_point] = x, y
            if view.visible_tracks:
                self.auto_gcp_create(view, x, y, event.button == 1)
            elif self.curr_point is not None:
                self.add_move_or_remove_gcp(view, x, y, event.button == 1)
            self.populate_gcp_list()
            self.populate_sequence_list(view)
            self.display_points(view)
        else:
            return


    def populate_gcp_list(self):
        self.gcp_list_box.delete(0, tk.END)
        errors = self.database.compute_gcp_errors()[-1]
        sorted_gcp_ids = sorted(errors, key=lambda k: -errors[k])
        self.gcps = sorted_gcp_ids
        for point_id in self.gcps:
            if point_id in self.database.points:
                n_annotations = len(self.database.points[point_id])
            else:
                n_annotations = 0
            self.gcp_list_box.insert(tk.END, f"{point_id} {n_annotations}")
        self.gcp_list_box.insert(tk.END, "none")
        self.gcp_list_box.selection_clear(0, tk.END)

        # Re-select the currently selected point
        if self.curr_point:
            for ix, gcp_id in enumerate(self.gcp_list_box.get(0, "end")):
                if self.curr_point in gcp_id:
                    self.gcp_list_box.selection_set(ix)
                    break
        self.master.update_idletasks()


    def remove_gcp(self):
        to_be_removed_point = self.curr_point
        if not to_be_removed_point:
            return
        self.curr_point = None

        self.database.remove_gcp(to_be_removed_point)
        for view in self.views:
            self.display_points(view)
            # if self.if_show_epipolar.get():
            #     self.show_epipolar_lines(image_idx)

        self.populate_gcp_list()

    def onclick_gcp_list(self, event):
        widget = event.widget
        selected_row = int(widget.curselection()[0])
        value = widget.get(selected_row)
        if value == 'none':
            self.curr_point = None
        else:
            self.curr_point = value.split(' ')[0]

        for view in self.views:
            self.display_points(view)

    def save_gcps(self):
        if self.last_saved_filename is None:
            return self.save_gcps_as()
        else:
            return self.database.write_to_file(self.last_saved_filename)

    def save_gcps_as(self):
        filename = filedialog.asksaveasfilename(
            initialfile="ground_control_points.json",
            title="Save GCP file",
            initialdir=self.database.path,
            defaultextension=".json",
        )
        if filename is None:
            return
        else:
            self.last_saved_filename = filename
            return self.save_gcps()

    # def show_epipolar_lines(self, main_image_idx):
    #     if len(self.views) > 2:
    #         raise NotImplementedError("Not implemented yet for >2 views")
    #     img1 = self.views[0].current_image
    #     img2 = self.views[1].current_image
    #     img1_size = self.database.get_image_size(img1)
    #     img2_size = self.database.get_image_size(img2)
    #     matched_points = self.database.get_visible_points_coords(self.views[main_image_idx].current_image)
    #     matched_points_coords = convert_tuple_cords_to_list(matched_points)
    #     matched_points_coords = features.normalized_image_coordinates(matched_points_coords, img1_size[1], img1_size[0])
    #     color_idx = 0
    #     for point_idx, point in enumerate(matched_points_coords):
    #         image_pair = [img1, img2]
    #         line = calc_epipol_line(point, image_pair, self.database.path, main_image_idx)
    #         denormalized_lines = features.denormalized_image_coordinates(line, img2_size[1], img2_size[0])
    #         for line_segment in denormalized_lines:
    #             circle = mpatches.Circle((line_segment[0], line_segment[1]), 3,
    #                                      color=distinct_colors[divmod(hash(list(matched_points.keys())[point_idx]), 19)[1]])
    #             self.views[main_image_idx].plt_artists.append(circle)
    #             self.views[not main_image_idx].subplot.add_artist(circle)
    #         color_idx = color_idx + 1
    #     self.views[not main_image_idx].figure.canvas.draw_idle()

    def go_to_image_index(self, view, idx, linked=True):
        views_to_update = {view}
        if linked:
            groups_this_view = [g for g in self.sequence_groups if view.sequence_key in g]
            for g in groups_this_view:
                for v in self.views:
                    if v.sequence_key in g:
                        views_to_update.add(v)
        for v in views_to_update:
            new_image = self.database.bring_image(v.sequence_key, idx)
            self.bring_new_image(new_image, v)

    def go_to_adjacent_image(self, view, offset, linked=True):
        target_ix = self.database.get_image_index(view.current_image, view.sequence_key) + offset
        self.go_to_image_index(view, target_ix, linked)

    def go_to_next_image_all(self):
        for view in self.views:
            self.go_to_adjacent_image(view, +1, linked=False)

    def go_to_prev_image_all(self):
        for view in self.views:
            self.go_to_adjacent_image(view, -1, linked=False)

    def go_to_next_image(self, view):
        self.go_to_adjacent_image(view, +1)

    def go_to_prev_image(self, view):
        self.go_to_adjacent_image(view, -1)

    def highlight_gcp_reprojection(self, view, shot, point_id):
        x, y = 0,0
        for obs in self.database.points[point_id]:
            if obs['shot_id'] == shot:
                x, y = obs['projection']

        x2, y2 = self.database.gcp_reprojections[point_id][shot]['reprojection']
        view.subplot.plot([x, x2], [y, y2], 'r-')
        view.canvas.draw_idle()
        self.zoom_in(view, x, y)

    def go_to_current_gcp(self):
        """
        Jumps to the currently selected GCP in all views where it was not visible
        """
        shots_gcp_seen = set(self.database.gcp_reprojections[self.curr_point].keys())
        for view in self.views:
            shots_gcp_seen_this_view = list(shots_gcp_seen.intersection(view.image_keys))
            if len(shots_gcp_seen_this_view) > 0 and view.current_image not in shots_gcp_seen:
                target_shot = random.choice(shots_gcp_seen_this_view)
                self.bring_new_image(target_shot, view)

    def go_to_worst_gcp(self):
        if len(self.database.gcp_reprojections) == 0:
            print("No GCP reprojections available. Can't jump to worst GCP")
            return
        worst_gcp = self.database.compute_gcp_errors()[0]

        self.curr_point = worst_gcp
        self.gcp_list_box.selection_clear(0, "end")
        for ix, gcp_id in enumerate(self.gcp_list_box.get(0, "end")):
            if worst_gcp in gcp_id:
                self.gcp_list_box.selection_set(ix)
                break

        for view in self.views:
            # Get the shot with worst reprojection error that in this view
            shot_worst_gcp = self.database.shot_with_max_gcp_error(view.image_keys, worst_gcp)
            self.bring_new_image(shot_worst_gcp, view)
            self.highlight_gcp_reprojection(view, shot_worst_gcp, worst_gcp)

    def point_in_view(self, view, point):
        if point is None:
            return None
        for projection in self.database.points[point]:
            if projection["shot_id"] == view.current_image:
                return projection["projection"]
        return None

    def zoom_logic(self, view):
        gcp_visible = self.point_in_view(view, self.curr_point)
        if self.sticky_zoom.get() and self.curr_point in view.last_seen_px:
            # Zoom in to the last seen location of this GCP
            x, y = view.last_seen_px[self.curr_point]
            self.zoom_in(view, x, y)
        else:
            # Show the whole image
            view.subplot.axis("scaled")
            view.figure.set_tight_layout(True)
            view.subplot.axis("off")

    def bring_new_image(self, new_image, view):
        if new_image == view.current_image:
            return
        t0 = time.time()
        view.current_image = new_image
        view.subplot.clear()
        view.subplot.imshow(self.database.get_image(new_image))
        view.subplot.axis('off')
        view.zoomed_in = False
        self.set_title(view)
        # if self.if_show_epipolar.get():
        #     for idx, view in enumerate(self.views):
        #             self.show_epipolar_lines(idx)

        # Update 'last seen' coordinates for all gcps in this view
        for gcp_id in self.database.points:
            gcp_visible = self.point_in_view(view, gcp_id)
            if gcp_visible is not None:
                view.last_seen_px[gcp_id] = gcp_visible

        self.zoom_logic(view)

        self.populate_sequence_list(view)

        self.display_points(view)
        print("Took {:.2f}s to bring_new_image {}".format(time.time()-t0, new_image))
