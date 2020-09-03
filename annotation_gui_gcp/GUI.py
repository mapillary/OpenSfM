from __future__ import print_function
from __future__ import division
from __future__ import absolute_import
from __future__ import unicode_literals

import matplotlib.patches as mpatches
import tkinter as tk
from tkinter import filedialog
import time
import matplotlib
import os

from apps import distinct_colors, read_gcp_file, id_generator, convert_tuple_cords_to_list

matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
from matplotlib.patches import PathPatch
from matplotlib.text import TextPath
from matplotlib.transforms import IdentityTransform
from matplotlib.offsetbox import AnnotationBbox, AuxTransformBox
from epipolarCalculation import calc_epipol_line

from opensfm import features
from apps import CustomListbox

PREVIOUS_UNICODE = u"\u2193"
NEXT_UNICODE = u"\u2191"


class NavigationToolbar(NavigationToolbar2Tk):
    def set_message(self, m):
        pass


class Gui:

    def __init__(self, master, database):
        self.database = database
        self.point_idx = 0
        self.match_idx = 0
        self.curr_point = None
        self.last_saved_filename = None
        self.shot_std = {}

        master.bind('q', lambda event: self.go_to_worst_gcp())
        self.create_ui(master, n_views=5)

        p_default_gcp = self.database.get_path() + '/ground_control_points.json'
        if os.path.exists(p_default_gcp):
            self.load_gcps(p_default_gcp)
        p_shot_std = self.database.get_path() + '/shots_std.csv'
        if os.path.exists(p_shot_std):
            self.load_shot_std(p_shot_std)

    def create_ui(self, master, n_views):
        self.master = master

        tools_frame = tk.Frame(master)
        tools_frame.pack(side='left', expand=0, fill=tk.Y)
        self.create_tools(tools_frame)

        self.views = []
        for view_ix in range(n_views):
            new_window = tk.Toplevel(self.master)
            new_window.title(f"View {view_ix+1}")
            self.views.append(new_window)

        self.init_image_windows()

    def create_tools(self, master):
        gcp_list_frame = tk.Frame(master)
        gcp_list_frame.pack(side='top', fill=tk.BOTH, expand=1)
        self.gcp_list_box = CustomListbox(gcp_list_frame, font=("monospace", 10), width=10)
        self.gcp_list_box.pack(side='left', expand=tk.YES, fill=tk.Y)
        self.gcp_list_box.bind('<<ListboxSelect>>', self.modify_gcp)

        plus_minus_frame = tk.Frame(master)
        plus_minus_frame.pack(side='top')
        self.add_button = tk.Button(plus_minus_frame, text="+", command=self.add_gcp)
        self.add_button.pack(side='left')
        self.remove_button = tk.Button(plus_minus_frame, text="-", command=self.remove_gcp)
        self.remove_button.pack(side='left')

        self.if_show_epipolar = tk.IntVar(value=0)
        self.check_button = tk.Checkbutton(master, text="Epipolar\nlines",
                                           var=self.if_show_epipolar)
        self.check_button.pack(side='top')

        io_frame = tk.Frame(master)
        io_frame.pack(side='top')
        self.load_button = tk.Button(io_frame, text="Load", command=self.load_gcps)
        self.load_button.pack(side='top')
        self.save_button = tk.Button(io_frame, text="Save", command=self.save_gcps)
        self.save_button.pack(side='top')
        self.save_button = tk.Button(io_frame, text="Save As", command=self.save_gcps_as)
        self.save_button.pack(side='top')

    def clear_artists(self, view):
        for artist in view.plt_artists:
            artist.set_visible(False)
            del artist

    def set_title(self, idx):
        shot = self.views[idx].current_image
        seq = self.database.seqs[idx]
        seq_ix = seq.index(shot)

        if shot in self.shot_std:
            shot_std_rank, shot_std = self.shot_std[shot]
            title = "[{}/{}]: {} - #{} (std = {:.2f})".format(seq_ix+1, len(seq), shot, shot_std_rank, shot_std)
        else:
            title = "[{}/{}]: {}".format(seq_ix+1, len(seq), shot)
        self.views[idx].title(title)


    def init_image_windows(self, nav_buttons=False):
        for idx, window in enumerate(self.views):
            nth_viewer_frame = tk.Frame(window)
            nth_viewer_frame.pack(side='left', fill=tk.BOTH, expand=1)

            if nav_buttons:
                button_frame = tk.Frame(nth_viewer_frame)
                button_frame.pack(side='top')
                prv_btn = tk.Button(button_frame, text=PREVIOUS_UNICODE,
                                    command=lambda _idx=idx: self.go_to_previous_image(_idx))
                prv_btn.pack(side='left')
                nxt_btn = tk.Button(button_frame, text=NEXT_UNICODE,
                                    command=lambda _idx=idx: self.go_to_next_image(_idx))
                nxt_btn.pack(side='left')

            window.figure = Figure()
            window.subplot = window.figure.add_subplot(111)
            window.current_image = self.database.get_seqs()[idx][0]  # in init, init the first pair = pairs[0] # TODO update
            window.subplot.imshow(self.database.get_image(window.current_image), aspect='auto')
            window.subplot.axis('scaled')
            window.figure.set_tight_layout(True)
            window.subplot.axis('off')

            window.canvas = FigureCanvasTkAgg(window.figure, nth_viewer_frame)
            window.canvas.draw()
            window.canvas.get_tk_widget().pack(side='top', fill=tk.BOTH, expand=1)
            window.canvas.mpl_connect('button_press_event',
                                       lambda event:
                                       self.on_press(event))
            window.canvas.mpl_connect('scroll_event',
                                       lambda event:
                                       self.on_scroll(event))

            window.zoomed_in = False
            window.plt_artists = []
            self.set_title(idx)

    def load_shot_std(self, path):
        with open(path, 'r') as f:
            for ix, line in enumerate(f):
                shot, std = line[:-1].split(',')
                self.shot_std[shot] = (ix + 1, float(std))

    def load_gcps(self, filename=None):
        if filename is None:
            filename = filedialog.askopenfilename(
                title="Open GCP file",
                initialdir=self.database.get_path(),
                filetypes=(("JSON files", "*.json"), ("all files", "*.*")),
            )
        if filename is None:
            return
        points = read_gcp_file(filename)
        self.last_saved_filename = filename
        self.database.init_points(points)
        for view_ix, view in enumerate(self.views):
            self.display_points(view)
            if self.if_show_epipolar.get():
                self.show_epipolar_lines(view_ix)
        self.repopulate_modify_list()

    def display_points(self, view):
        visible_points_coords = self.database.get_visible_points_coords(view.current_image)
        self.clear_artists(view)

        for point_id, coords in visible_points_coords.items():
            color = distinct_colors[divmod(hash(point_id), 19)[1]]
            text_path = TextPath((0, 0), point_id, size=10)
            p1 = PathPatch(text_path, transform=IdentityTransform(), alpha=1, color=color)
            offsetbox2 = AuxTransformBox(IdentityTransform())
            offsetbox2.add_artist(p1)
            ab = AnnotationBbox(offsetbox2, ((coords[0] + 30), (coords[1] + 30)), bboxprops=dict(alpha=0.05))
            circle = mpatches.Circle((coords[0], coords[1]), 20, color=color, fill=False)

            view.plt_artists.append(ab)
            view.plt_artists.append(circle)

            view.subplot.add_artist(ab)
            view.subplot.add_artist(circle)
        view.figure.canvas.draw()

    def add_gcp(self):
        new_id = id_generator()
        self.database.add_point(new_id)
        self.curr_point = new_id
        self.gcp_list_box.insert(tk.END, new_id)
        self.gcp_list_box.selection_clear(0, tk.END)
        self.gcp_list_box.selection_set(tk.END)

    def which_canvas(self, event):
        for ix, view in enumerate(self.views):
            if event.canvas == view.canvas:
                return ix
        return None

    def on_scroll(self, event):
        idx = self.which_canvas(event)
        if event.xdata is None or event.ydata is None:
            return
        if event.button == 'up':
            self.go_to_next_image(idx)
        elif event.button == 'down':
            self.go_to_previous_image(idx)

    def zoom_in(self, view, x, y):
        xlim = view.subplot.get_xlim()
        width = max(xlim) - min(xlim)
        view.subplot.set_xlim(x - width / 20, x + width / 20)
        view.subplot.set_ylim(y + width / 20, y - width / 20)
        view.zoomed_in = True

    def zoom_out(self, view):
        view.subplot.autoscale()
        view.zoomed_in = False

    def on_press(self, event):
        idx = self.which_canvas(event)
        view = self.views[idx]
        x, y = event.xdata, event.ydata
        current_image = view.current_image
        if None in (x, y):
            return
        if event.button == 2:  # Middle / wheel click:
            if view.zoomed_in:
                self.zoom_out(view)
            else:
                self.zoom_in(view, x, y)
            view.figure.canvas.draw_idle()
        elif self.curr_point is not None and event.button in (1, 3):
            if not self.curr_point:
                return
            self.database.remove_point_observation(self.curr_point, current_image)

            for line in self.views[idx].subplot.lines:
                line.remove()
            if event.button == 1: # Left click
                self.database.add_point_observation(self.curr_point, current_image, (x, y))

            self.display_points(view)
            if self.if_show_epipolar.get():
                self.show_epipolar_lines(idx)
        else:
            return

    def repopulate_modify_list(self):
        self.gcp_list_box.delete(0, tk.END)
        for point_id in self.database.get_points():
            self.gcp_list_box.insert(tk.END, point_id)

    def remove_gcp(self):
        to_be_removed_point = self.curr_point
        if not to_be_removed_point:
            return
        self.curr_point = None

        self.database.remove_gcp(to_be_removed_point)
        for image_idx, view in enumerate(self.views):
            self.display_points(view)
            if self.if_show_epipolar.get():
                self.show_epipolar_lines(image_idx)

        self.repopulate_modify_list()

    def modify_gcp(self, event):
        widget = event.widget
        value = widget.get(int(widget.curselection()[0]))
        self.curr_point = value

    def save_gcps(self):
        if self.last_saved_filename is None:
            return self.save_gcps_as()
        else:
            return self.database.write_to_file(self.last_saved_filename)

    def save_gcps_as(self):
        filename = filedialog.asksaveasfilename(
            initialfile="ground_control_points.json",
            title="Save GCP file",
            initialdir=self.database.get_path(),
            defaultextension=".json",
        )
        if filename is None:
            return
        else:
            self.last_saved_filename = filename
            return self.save_gcps()

    def show_epipolar_lines(self, main_image_idx):
        if len(self.views) > 2:
            raise NotImplementedError("Not implemented yet for >2 views")
        img1 = self.views[0].current_image
        img2 = self.views[1].current_image
        img1_size = self.database.get_image_size(img1)
        img2_size = self.database.get_image_size(img2)
        matched_points = self.database.get_visible_points_coords(self.views[main_image_idx].current_image)
        matched_points_coords = convert_tuple_cords_to_list(matched_points)
        matched_points_coords = features.normalized_image_coordinates(matched_points_coords, img1_size[1], img1_size[0])
        color_idx = 0
        for point_idx, point in enumerate(matched_points_coords):
            image_pair = [img1, img2]
            line = calc_epipol_line(point, image_pair, self.database.get_path(), main_image_idx)
            denormalized_lines = features.denormalized_image_coordinates(line, img2_size[1], img2_size[0])
            for line_segment in denormalized_lines:
                circle = mpatches.Circle((line_segment[0], line_segment[1]), 3,
                                         color=distinct_colors[divmod(hash(list(matched_points.keys())[point_idx]), 19)[1]])
                self.views[main_image_idx].plt_artists.append(circle)
                self.views[not main_image_idx].subplot.add_artist(circle)
            color_idx = color_idx + 1
        self.views[not main_image_idx].figure.canvas.draw_idle()

    def go_to_next_image(self, image_idx):
        current_image = self.views[image_idx].current_image
        new_image = self.database.bring_next_image(current_image, image_idx)
        if current_image != new_image:
            self.bring_new_image(new_image, image_idx)

    def go_to_previous_image(self, image_idx):
        current_image = self.views[image_idx].current_image
        new_image = self.database.bring_previous_image(current_image, image_idx)
        if current_image != new_image:
            self.bring_new_image(new_image, image_idx)

    def highlight_gcp_reprojection(self, image_idx, shot, point_id):
        x, y = 0,0
        for obs in self.database.get_points()[point_id]:
            if obs['shot_id'] == shot:
                x, y = obs['projection']

        x2, y2 = self.database.gcp_reprojections[point_id][shot]['reprojection']
        self.views[image_idx].subplot.plot([x, x2], [y, y2], 'r-')
        self.canvases[image_idx].draw_idle()

    def go_to_worst_gcp(self):
        if len(self.database.gcp_reprojections) == 0:
            print("No GCP reprojections available. Can't jump to worst GCP")
            return
        worst_gcp, shot_worst_gcp, worst_gcp_error = self.database.get_worst_gcp()
        idx_worst_gcp = 0 if shot_worst_gcp in self.database.seqs[0] else 1
        print("Worst GCP observation: {} in shot {}".format(worst_gcp, shot_worst_gcp))

        self.curr_point = worst_gcp
        self.gcp_list_box.selection_clear(0, "end")
        for ix, gcp_id in enumerate(self.gcp_list_box.get(0, "end")):
            if gcp_id == worst_gcp:
                self.gcp_list_box.selection_set(ix)
                break

        if self.views[idx_worst_gcp].current_image != shot_worst_gcp:
            self.bring_new_image(shot_worst_gcp, idx_worst_gcp)

        self.highlight_gcp_reprojection(idx_worst_gcp, shot_worst_gcp, worst_gcp)

    def bring_new_image(self, new_image, image_idx):
        t0 = time.time()
        view = self.views[image_idx]
        view.current_image = new_image
        view.subplot.clear()
        view.subplot.imshow(self.database.get_image(new_image))
        view.subplot.axis('off')
        self.set_title(image_idx)
        view.zoomed_in = False
        view.subplot.axis('scaled')
        view.figure.set_tight_layout(True)
        view.subplot.axis('off')
        self.display_points(view)
        if self.if_show_epipolar.get():
            for idx, view in enumerate(self.views):
                    self.show_epipolar_lines(idx)
        print("Took {:.2f}s to bring_new_image {}".format(time.time()-t0, new_image))
