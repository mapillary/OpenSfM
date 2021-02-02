import json
import os
from collections import OrderedDict


def id_generator():
    i = 0
    while True:
        yield str(i)
        i += 1


class GroundControlPointManager:
    def __init__(self, path):
        self.points = OrderedDict()
        self.latlons = {}
        self.path = path
        self.image_cache = {}
        self.id_generator = id_generator()

        # p_gcp_errors = self.path + "/gcp_reprojections_0x1.json"
        # self.load_gcp_reprojections(p_gcp_errors)

    def load_gcp_reprojections(self, filename):
        if os.path.isfile(filename):
            with open(filename, "r") as f:
                self.gcp_reprojections = json.load(f)
        else:
            self.gcp_reprojections = {}
            print(f"Not found: {filename}")

    def load_from_file(self, file_path):
        self.points = OrderedDict()
        self.latlons = {}
        with open(file_path, "r") as f:
            input_points = json.load(f)["points"]
        for point in input_points:
            self.points[point["id"]] = point["observations"]
            latlon = point.get("position")
            if latlon:
                if "altitude" in latlon:
                    raise NotImplementedError("Not supported: altitude in GCPs")
                self.latlons[point["id"]] = latlon

    def write_to_file(self, filename):
        output_points = []
        for point_id in self.points:
            out_point = {"id": point_id, "observations": self.points[point_id]}
            if out_point["id"] in self.latlons:
                out_point["position"] = self.latlons[point_id]
            output_points.append(out_point)
        with open(filename, "wt") as fp:
            json.dump({"points": output_points}, fp, indent=4, sort_keys=True)

    def get_visible_points_coords(self, main_image):
        visible_points_coords = OrderedDict()
        for point_id, observations in self.points.items():
            for observation in observations:
                if observation["shot_id"] == main_image:
                    visible_points_coords[point_id] = observation["projection"]

        return visible_points_coords

    def point_exists(self, point_id):
        return point_id in self.points

    def add_point(self):
        new_id = next(self.id_generator)
        while self.point_exists(new_id):
            new_id = next(self.id_generator)
        self.points[new_id] = []
        return new_id

    def add_point_observation(self, point_id, shot_id, projection, latlon=None):
        if not self.point_exists(point_id):
            raise ValueError(f"ERROR: trying to modify a non-existing point {point_id}")

        if latlon:
            self.latlons[point_id] = {
                "latitude": latlon[0],
                "longitude": latlon[1],
            }
        self.points[point_id].append(
            {
                "shot_id": shot_id,
                "projection": projection,
            }
        )

    def compute_gcp_errors(self):
        error_avg = {}
        worst_gcp_error = 0
        worst_gcp = None
        shot_worst_gcp = None
        for gcp_id in self.points:
            error_avg[gcp_id] = 0
        for gcp_id in self.gcp_reprojections:
            if gcp_id not in self.points:
                continue
            for shot_id in self.gcp_reprojections[gcp_id]:
                err = self.gcp_reprojections[gcp_id][shot_id]["error"]
                error_avg[gcp_id] += err
                if err > worst_gcp_error:
                    worst_gcp_error = err
                    shot_worst_gcp = shot_id
                    worst_gcp = gcp_id
            error_avg[gcp_id] /= len(self.gcp_reprojections[gcp_id])

        return worst_gcp, shot_worst_gcp, worst_gcp_error, error_avg

    def shot_with_max_gcp_error(self, image_keys, gcp):
        # Return they key with most reprojection error for this GCP
        annotated_images = set(self.gcp_reprojections[gcp]).intersection(
            set(image_keys)
        )
        errors = {k: self.gcp_reprojections[gcp][k]["error"] for k in annotated_images}
        if len(errors) > 0:
            return max(errors, key=lambda k: errors[k])
        else:
            return None

    def remove_gcp(self, point_id):
        if self.point_exists(point_id):
            del self.points[point_id]

    def remove_point_observation(self, point_id, shot_id):
        if not self.point_exists(point_id):
            print("ERROR: trying to modify a non-existing point")
            return
        self.points[point_id] = [
            obs for obs in self.points[point_id] if obs["shot_id"] != shot_id
        ]
        if point_id in self.gcp_reprojections:
            if shot_id in self.gcp_reprojections[point_id]:
                self.gcp_reprojections[point_id][shot_id]["error"] = 0
