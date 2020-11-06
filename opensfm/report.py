import os
import subprocess
import PIL

from fpdf import FPDF
from opensfm import io


class Report:
    def __init__(self, data):
        self.output_path = os.path.join(data.data_path, "stats")
        self.dataset_name = os.path.basename(data.data_path)

        self.mapi_light_light_green = [210, 245, 226]
        self.mapi_light_green = [5, 203, 99]
        self.mapi_dark_green = [5, 183, 89]
        self.mapi_light_grey = [218, 222, 228]
        self.mapi_dark_grey = [99, 115, 129]
        self.mapi_text_grey = [72, 87, 101]

        self.pdf = FPDF("P", "mm", "A4")
        self.pdf.add_page()

        self.title_size = 20
        self.h1 = 16
        self.h2 = 14
        self.h3 = 12
        self.text = 10
        self.small_text = 8
        self.margin = 10
        self.cell_height = 7
        self.total_size = 190

        self.stats = self._read_stats_file("stats.json")

    def save_report(self, filename):
        self.pdf.output(os.path.join(self.output_path, filename), "F")

    def _make_table(self, columns_names, rows, row_header=False):
        self.pdf.set_font("Helvetica", "", self.h3)
        self.pdf.set_line_width(0.3)

        columns_sizes = [int(self.total_size / len(rows[0]))] * len(rows[0])

        if columns_names:
            self.pdf.set_draw_color(*self.mapi_light_grey)
            self.pdf.set_fill_color(*self.mapi_light_grey)
            for col, size in zip(columns_names, columns_sizes):
                self.pdf.rect(
                    self.pdf.get_x(),
                    self.pdf.get_y(),
                    size,
                    self.cell_height,
                    style="FD",
                )
                self.pdf.set_text_color(*self.mapi_dark_grey)
                self.pdf.cell(size, self.cell_height, col, align="L")
            self.pdf.set_xy(self.margin, self.pdf.get_y() + self.cell_height)

        self.pdf.set_draw_color(*self.mapi_light_grey)
        self.pdf.set_fill_color(*self.mapi_light_light_green)

        for row in rows:
            for i, (col, size) in enumerate(zip(row, columns_sizes)):
                if i == 0 and row_header:
                    self.pdf.set_draw_color(*self.mapi_light_grey)
                    self.pdf.set_fill_color(*self.mapi_light_grey)
                self.pdf.rect(
                    self.pdf.get_x(),
                    self.pdf.get_y(),
                    size,
                    self.cell_height,
                    style="FD",
                )
                self.pdf.set_text_color(*self.mapi_dark_grey)
                if i == 0 and row_header:
                    self.pdf.set_draw_color(*self.mapi_light_grey)
                    self.pdf.set_fill_color(*self.mapi_light_light_green)
                self.pdf.cell(size, self.cell_height, col, align="L")
            self.pdf.set_xy(self.margin, self.pdf.get_y() + self.cell_height)

    def _read_stats_file(self, filename):
        file_path = os.path.join(self.output_path, filename)
        with io.open_rt(file_path) as fin:
            return io.json_load(fin)

    def _make_section(self, title):
        self.pdf.set_font("Helvetica", "B", self.h1)
        self.pdf.set_text_color(*self.mapi_dark_grey)
        self.pdf.cell(0, self.margin, title, align="L")
        self.pdf.set_xy(self.margin, self.pdf.get_y() + 1.5 * self.margin)

    def _make_centered_image(self, image_path, desired_height):
        width, height = PIL.Image.open(image_path).size
        resized_width = width * desired_height / height
        if resized_width > self.total_size:
            resized_width = self.total_size
            desired_height = height * resized_width / width

        self.pdf.image(
            image_path,
            self.pdf.get_x() + self.total_size / 2 - resized_width / 2,
            self.pdf.get_y(),
            h=desired_height,
        )
        self.pdf.set_xy(self.margin, self.pdf.get_y() + desired_height + self.margin)

    def make_title(self):
        # title
        self.pdf.set_font("Helvetica", "B", self.title_size)
        self.pdf.set_text_color(*self.mapi_light_green)
        self.pdf.cell(0, self.margin, "OpenSfM Quality Report", align="C")
        self.pdf.set_xy(self.margin, self.title_size)

        # version number
        version = (
            subprocess.check_output(["git", "describe", "--always"]).strip().decode()
        )
        self.pdf.set_font("Helvetica", "", self.small_text)
        self.pdf.set_text_color(*self.mapi_dark_grey)
        self.pdf.cell(0, self.margin, f"Processed with OpenSfM {version}", align="R")
        self.pdf.set_xy(self.margin, self.pdf.get_y() + 2 * self.margin)

    def make_dataset_summary(self):
        self._make_section("Dataset Summary")

        rows = [
            ["Dataset", self.dataset_name],
            ["Date", self.stats["processing_statistics"]["date"]],
            [
                "Area Covered",
                f"{self.stats['processing_statistics']['area']/1e6:.6f} km²",
            ],
            [
                "Processing Time",
                f"{self.stats['processing_statistics']['steps_times']['Total Time']:.2f} seconds",
            ],
        ]
        self._make_table(None, rows)
        self.pdf.set_xy(self.margin, self.pdf.get_y() + self.margin)

    def make_processing_summary(self):
        self._make_section("Processing Summary")

        rec_shots, init_shots = (
            self.stats["reconstruction_statistics"]["reconstructed_shots_count"],
            self.stats["reconstruction_statistics"]["initial_shots_count"],
        )
        rec_points, init_points = (
            self.stats["reconstruction_statistics"]["reconstructed_points_count"],
            self.stats["reconstruction_statistics"]["initial_points_count"],
        )

        geo_string = []
        if self.stats["reconstruction_statistics"]["has_gps"]:
            geo_string.append("GPS")
        if self.stats["reconstruction_statistics"]["has_gcp"]:
            geo_string.append("GCP")
        geo_string = "and".join(geo_string)

        rows = [
            [
                "Reconstructed Images",
                f"{rec_shots} over {init_shots} shots ({rec_shots/init_shots*100:.1f}%)",
            ],
            [
                "Reconstructed Points",
                f"{rec_points} over {init_points} points ({rec_points/init_points*100:.1f}%)",
            ],
            [
                "Reconstructed Components",
                f"{self.stats['reconstruction_statistics']['components']} component",
            ],
            [
                "Detected Features",
                f"{self.stats['features_statistics']['detected_features']['median']} features",
            ],
            [
                "Reconstructed Features",
                f"{self.stats['features_statistics']['reconstructed_features']['median']} features",
            ],
            ["Geographic Reference", geo_string],
        ]
        if self.stats["reconstruction_statistics"]["has_gps"]:
            rows.append(
                [
                    "GPS error",
                    f"{self.stats['gps_errors']['average_error']:.2f} meters",
                ]
            )
        self._make_table(None, rows)
        self.pdf.set_xy(self.margin, self.pdf.get_y() + self.margin / 2)

        topview_height = 130
        topview_grids = [
            f for f in os.listdir(self.output_path) if f.startswith("topview")
        ]
        self._make_centered_image(
            os.path.join(self.output_path, topview_grids[0]), topview_height
        )

        self.pdf.set_xy(self.margin, self.pdf.get_y() + self.margin)

    def make_processing_time_details(self):
        self._make_section("Processing Time Details")

        columns_names = list(self.stats["processing_statistics"]["steps_times"].keys())
        formatted_floats = []
        for v in self.stats["processing_statistics"]["steps_times"].values():
            formatted_floats.append(f"{v:.2f} sec.")
        rows = [formatted_floats]
        self._make_table(columns_names, rows)
        self.pdf.set_xy(self.margin, self.pdf.get_y() + 2 * self.margin)

    def make_gps_details(self):
        self._make_section("GPS Errors Details")

        columns_names = ["", "Mean", "Sigma", "RMS Error"]
        rows = []
        for comp in ["x", "y", "z"]:
            row = [comp.upper() + " Error (meters)"]
            row.append(f"{self.stats['gps_errors']['mean'][comp]:.2f}")
            row.append(f"{self.stats['gps_errors']['std'][comp]:.2f}")
            row.append(f"{self.stats['gps_errors']['error'][comp]:.2f}")
            rows.append(row)

        rows.append(
            ["Total", "", "", f"{self.stats['gps_errors']['average_error']:.2f}"]
        )
        self._make_table(columns_names, rows)
        self.pdf.set_xy(self.margin, self.pdf.get_y() + self.margin)

    def make_features_details(self):
        self._make_section("Features Details")

        cols = 2
        heatmap_height = 70
        heatmaps = [f for f in os.listdir(self.output_path) if f.startswith("heatmap")]
        if len(heatmaps) == 1:
            self._make_centered_image(
                os.path.join(self.output_path, heatmaps[0]), heatmap_height
            )
        else:
            raise RuntimeError("Please implemted multi-model display")

        columns_names = ["", "Min.", "Max.", "Mean", "Median"]
        rows = []
        for comp in ["detected_features", "reconstructed_features"]:
            row = [comp.replace("_", " ").replace("features", "").capitalize()]
            for t in columns_names[1:]:
                row.append(
                    f"{self.stats['features_statistics'][comp][t.replace('.', '').lower()]:.0f}"
                )
            rows.append(row)
        self._make_table(columns_names, rows)

        self.pdf.set_xy(self.margin, self.pdf.get_y() + self.margin)

    def make_reconstruction_details(self):
        self._make_section("Reconstruction Details")

        rows = [
            [
                "Average reprojection Error",
                f"{self.stats['reconstruction_statistics']['reprojection_error']:.2f} pixels",
            ],
            [
                "Average Track Length",
                f"{self.stats['reconstruction_statistics']['average_track_length']:.2f} images",
            ],
            [
                "Average Track Length (> 2)",
                f"{self.stats['reconstruction_statistics']['average_track_length_over_two']:.2f} images",
            ],
        ]
        self._make_table(None, rows)
        self.pdf.set_xy(self.margin, self.pdf.get_y() + self.margin)

        residual_grid_height = 110
        residual_grids = [
            f for f in os.listdir(self.output_path) if f.startswith("residual")
        ]
        self._make_centered_image(
            os.path.join(self.output_path, residual_grids[0]), residual_grid_height
        )

        self.pdf.set_xy(self.margin, self.pdf.get_y() + self.margin)

    def make_tracks_details(self):
        self._make_section("Tracks Details")
        matchgraph_height = 90
        matchgraph = [
            f for f in os.listdir(self.output_path) if f.startswith("matchgraph")
        ]
        self._make_centered_image(
            os.path.join(self.output_path, matchgraph[0]), matchgraph_height
        )

        histogram = self.stats["reconstruction_statistics"]["histogram_track_length"]
        start_length, end_length = 2, 10
        row_length = ["Length"]
        for length, count in sorted(histogram.items(), key=lambda x: int(x[0])):
            if int(length) < start_length or int(length) > end_length:
                continue
            row_length.append(length)
        row_count = ["Count"]
        for length, count in sorted(histogram.items(), key=lambda x: int(x[0])):
            if int(length) < start_length or int(length) > end_length:
                continue
            row_count.append(f"{count}")

        self._make_table(None, [row_length, row_count])

        self.pdf.set_xy(self.margin, self.pdf.get_y() + self.margin)

    def add_page_break(self):
        self.pdf.add_page("P")

    def generate_report(self):
        self.make_title()
        self.make_dataset_summary()
        self.make_processing_summary()
        self.add_page_break()

        self.make_features_details()
        self.make_reconstruction_details()
        self.add_page_break()

        self.make_tracks_details()
        self.make_gps_details()
        self.make_processing_time_details()
