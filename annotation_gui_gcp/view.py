import colorsys
import tkinter as tk

import matplotlib
import matplotlib.patches as mpatches

matplotlib.use("TkAgg")
from matplotlib import patheffects
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

FONT = "TkFixedFont"

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


def comp_color(color):
    r, g, b = color[1:3], color[3:5], color[5:7]
    r, g, b = [int(c, 16) for c in (r, g, b)]
    h, s, v = colorsys.rgb_to_hsv(r, g, b)
    comp_color = "black" if v > 128 else "white"
    return comp_color


class View:
    def __init__(self, main_ui, name):
        self.main_ui = main_ui
        window = tk.Toplevel(self.main_ui.master, name=name)
        self.window = window

        canvas_frame = tk.Frame(window)
        canvas_frame.pack(side="left", fill=tk.BOTH, expand=1)

        self.toolbox = tk.Frame(canvas_frame)
        self.toolbox.pack(side="left", expand=False, fill=tk.Y)

        self.visible_tracks = None
        self.image_list = tk.StringVar()
        self.image_list_box = tk.Listbox(
            self.toolbox,
            font=FONT,
            width=12,
            selectmode="browse",
            listvariable=self.image_list,
        )
        self.image_list_box.pack(side="bottom", expand=True, fill=tk.Y)
        self.image_list_box.bind("<<ListboxSelect>>", self.onclick_image_list)

        self.figure = Figure()
        self.subplot = self.figure.add_subplot(111)

        self.canvas = FigureCanvasTkAgg(self.figure, canvas_frame)
        self.canvas.get_tk_widget().pack(side="top", fill=tk.BOTH, expand=1)
        self.canvas.mpl_connect(
            "button_press_event", lambda event: self.onclick_image(event)
        )
        self.canvas.mpl_connect("scroll_event", lambda event: self.on_scroll(event))

        self.zoomed_in = False
        self.last_seen_px = {}
        self.plt_artists = []
        self.set_title()

    def onclick_image(self, event):
        x, y = event.xdata, event.ydata
        if None in (x, y):
            return
        if event.button == 2:  # Middle / wheel click:
            if self.zoomed_in:
                self.zoom_out()
            else:
                self.zoom_in(x, y)
            self.figure.canvas.draw_idle()
        elif event.button in (1, 3):
            # Left click or right click
            self.last_seen_px[self.main_ui.curr_point] = x, y
            if self.visible_tracks:
                self.auto_gcp_create(x, y, event.button == 1)
            elif self.main_ui.curr_point is not None:
                self.add_move_or_remove_gcp(x, y, event.button == 1)
            self.main_ui.populate_gcp_list()
            self.update_image_list_text()
            self.display_points()
        else:
            return

    def on_scroll(self, event):
        if event.xdata is None or event.ydata is None:
            return
        if event.button == "up":
            self.go_to_next_image()
        elif event.button == "down":
            self.go_to_prev_image()

    def onclick_image_list(self, event):
        widget = event.widget
        sel = widget.curselection()
        if not sel:
            return
        self.go_to_image_index(int(sel[0]))

    def add_move_or_remove_gcp(self, x, y, add):
        if self.main_ui.curr_point is None:
            return
        reproj = self.main_ui.gcp_manager.gcp_reprojections.get(self.main_ui.curr_point)
        if reproj:
            reproj.pop(self.current_image, None)
        self.main_ui.gcp_manager.remove_point_observation(
            self.main_ui.curr_point, self.current_image
        )
        if add:
            self.main_ui.gcp_manager.add_point_observation(
                self.main_ui.curr_point,
                self.current_image,
                self.pixel_to_gcp_coordinates(x, y),
                latlon=self.pixel_to_latlon(x, y),
            )
            self.zoom_in(x, y)
        else:
            self.zoom_out()

    def zoom_in(self, x, y):
        if self.zoomed_in:
            return
        radius = self.zoom_window_size_px / 2
        self.subplot.set_xlim(x - radius, x + radius)
        self.subplot.set_ylim(y + radius, y - radius)
        self.zoomed_in = True

    def zoom_out(self):
        self.subplot.autoscale()
        self.zoomed_in = False

    def zoom_logic(self):
        point_has_been_seen = self.main_ui.curr_point in self.last_seen_px
        if self.main_ui.sticky_zoom.get() and point_has_been_seen:
            # Zoom in to the last seen location of this GCP
            x, y = self.last_seen_px[self.main_ui.curr_point]
            self.zoom_in(x, y)
        else:
            # Show the whole image
            self.subplot.axis("scaled")
            self.figure.set_tight_layout(True)
            self.subplot.axis("off")

    def point_in_view(self, point):
        if point is None:
            return None
        for projection in self.main_ui.gcp_manager.points[point]:
            if projection["shot_id"] == self.current_image:
                return self.gcp_to_pixel_coordinates(*projection["projection"])
        return None

    def display_points(self):
        visible_points_coords = self.main_ui.gcp_manager.get_visible_points_coords(
            self.current_image
        )
        self.clear_artists()

        for point_id, coords in visible_points_coords.items():
            color = distinct_colors[divmod(hash(point_id), 19)[1]]
            x, y = self.gcp_to_pixel_coordinates(*coords)
            artists = [
                mpatches.Circle((x, y), 5, color=color, fill=False),
            ]

            if self.main_ui.show_gcp_names.get() or point_id == self.main_ui.curr_point:
                text = matplotlib.text.Annotation(
                    point_id,
                    (x, y),
                    xytext=[0, 7],
                    # fontsize=9,
                    textcoords="offset points",
                    ha="center",
                    va="bottom",
                    color=color,
                )
                text.set_path_effects(
                    [
                        patheffects.Stroke(linewidth=3, foreground=comp_color(color)),
                        patheffects.Normal(),
                    ]
                )
                artists.append(text)

            if point_id == self.main_ui.curr_point:
                artists.extend(
                    [
                        mpatches.Circle((x, y), 0.5, color=color, fill=True),
                        mpatches.Circle((x, y), 10, color=color, fill=False),
                        mpatches.Circle((x, y), 11, color=color, fill=False),
                        mpatches.Circle((x, y), 12, color=color, fill=False),
                    ]
                )
            for art in artists:
                self.plt_artists.append(art)
                self.subplot.add_artist(art)

        self.figure.canvas.draw()

    def clear_artists(self):
        for artist in self.plt_artists:
            artist.set_visible(False)
            del artist

    def populate_image_list(self):
        self.update_image_list_text()
        self.update_image_list_highlight()

    def update_image_list_text(self):
        items = []
        self.images_in_list = self.get_candidate_images()
        n_digits = len(str(len(self.images_in_list)))
        for ix, image_name in enumerate(self.images_in_list):
            points = self.main_ui.gcp_manager.get_visible_points_coords(image_name)
            txt = "{:0{n_digits}} {}".format(ix + 1, len(points), n_digits=n_digits)
            shot_std = self.main_ui.shot_std.get(image_name, None)
            if shot_std:
                txt += " {:.2f}".format(shot_std)
            items.append(txt)
        self.image_list.set(items)

    def update_image_list_highlight(self):
        defaultbg = self.window.cget("bg")
        for ix, shot in enumerate(self.image_keys):
            bg = "green" if shot == self.current_image else defaultbg
            self.image_list_box.itemconfig(ix, bg=bg)

    def bring_new_image(self, new_image):
        if new_image == self.current_image:
            return
        self.current_image = new_image
        self.subplot.clear()
        img = self.get_image(new_image)
        self.subplot.imshow(img)
        self.subplot.axis("on")
        self.subplot.axis("scaled")
        self.zoomed_in = False
        self.set_title()

        # Update 'last seen' coordinates for all gcps in this view
        for gcp_id in self.main_ui.gcp_manager.points:
            gcp_visible = self.point_in_view(gcp_id)
            if gcp_visible is not None:
                self.last_seen_px[gcp_id] = gcp_visible

        self.zoom_logic()

        self.update_image_list_highlight()

        self.display_points()

        if self.main_ui.curr_point:
            self.highlight_gcp_reprojection(self.main_ui.curr_point, zoom=False)

    def highlight_gcp_reprojection(self, point_id, zoom=True):
        if point_id not in self.main_ui.gcp_manager.gcp_reprojections:
            return
        shot = self.current_image
        x, y = None, None
        for obs in self.main_ui.gcp_manager.points[point_id]:
            if obs["shot_id"] == shot:
                x, y = obs["projection"]
        if x is None:
            return

        reproj = self.main_ui.gcp_manager.gcp_reprojections[point_id].get(shot)
        if not reproj:
            return
        x2, y2 = reproj["reprojection"]
        artists = self.subplot.plot([x, x2], [y, y2], "r-")
        self.plt_artists.extend(artists)
        if zoom:
            self.zoom_in(x, y)
        self.canvas.draw_idle()

    def go_to_next_image(self):
        self.go_to_adjacent_image(+1)

    def go_to_prev_image(self):
        self.go_to_adjacent_image(-1)

    def go_to_adjacent_image(self, offset, linked=True):
        target_ix = self.images_in_list.index(self.current_image) + offset
        if 0 <= target_ix < len(self.images_in_list):
            self.go_to_image_index(target_ix, linked)

    def go_to_image_index(self, idx, linked=True):
        views_to_update = {self}
        if linked:
            groups_this_view = [
                g for g in self.main_ui.sequence_groups if self.group_name in g
            ]
            for g in groups_this_view:
                for v in self.main_ui.views:
                    if v.group_name in g:
                        views_to_update.add(v)
        for v in views_to_update:
            v.bring_new_image(self.images_in_list[idx])

    def set_title(self):
        shot = self.current_image
        seq_ix = self.image_keys.index(shot)
        title = f"{self.group_name} [{seq_ix+1}/{len(self.image_keys)}]: {shot}"
        self.window.title(title)
