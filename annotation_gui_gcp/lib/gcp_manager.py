import json
import os
import typing as t
from collections import OrderedDict
from dataclasses import dataclass


def id_generator():
    i = 0
    while True:
        yield str(i)
        i += 1


@dataclass(frozen=True, eq=True)
class GeodeticMeasurement:
    longitude: float
    latitude: float
    horizontal_std: float  # Standard deviation (m), horizontal.
    measured_at: int  # Unix time
    altitude: t.Optional[float] = None
    vertical_std: t.Optional[float] = None  # Standard deviation (m), vertical.

    def to_dict(self) -> t.Dict[str, t.Any]:
        d = {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "horizontal_std": self.horizontal_std,
            "measured_at": self.measured_at,
        }
        if self.altitude:
            d["altitude"] = self.altitude
        if self.vertical_std:
            d["vertical_std"] = self.vertical_std
        return d

    @classmethod
    def from_dict(cls, d: t.Dict[str, t.Any]):
        return GeodeticMeasurement(**d)


@dataclass(frozen=True, eq=True)
class PointMeasurement:
    image_id: str
    normalized_x: float  # (x-w/2) / max(w,h)
    normalized_y: float  # (y-h/2) / max(w,h)
    normalized_precision: float  # precision as standard deviation in pixels / max(w,h)


@dataclass(frozen=True, eq=True)
class PointMeasurement3D:
    image_id: str
    x: float
    y: float
    z: float
    precision: t.Optional[float]  # precision as standard deviation in (x,y,z) units


class ControlPoint:
    def __init__(self, point_id: str):
        self.id = point_id
        self.observations: t.List[PointMeasurement | PointMeasurement3D] = []
        self.geodetic_measurement: t.Optional[GeodeticMeasurement] = None

    def __repr__(self):
        if self.geodetic_measurement:
            return f"ControlPoint {self.id} @ {self.geodetic_measurement} with {len(self.observations)} image observations"
        else:
            return f"ControlPoint {self.id} with {len(self.observations)} image observations"


def observation_to_json(
    obs: t.Union[PointMeasurement3D, PointMeasurement]
) -> t.Dict[str, t.Any]:
    if isinstance(obs, PointMeasurement):
        return {
            "shot_id": obs.image_id,
            "projection": [obs.normalized_x, obs.normalized_y],
            "precision": obs.normalized_precision,
        }
    elif isinstance(obs, PointMeasurement3D):
        return {
            "shot_id": obs.image_id,
            "point": [obs.x, obs.y, obs.z],
            "precision": obs.precision,
        }
    else:
        raise ValueError(f"Unknown observation {obs}")


def observation_from_json(
    obs: t.Dict[str, t.Any]
) -> t.Union[PointMeasurement3D, PointMeasurement]:
    if "projection" in obs:
        return PointMeasurement(
            image_id=obs["shot_id"],
            normalized_x=obs["projection"][0],
            normalized_y=obs["projection"][1],
            normalized_precision=obs.get(
                "precision", 0.004
            ),  # 3-sigma (99%) confidence of about 8 px in a 640px image.
        )
    elif "point" in obs:
        return PointMeasurement3D(
            image_id=obs["shot_id"],
            x=obs["point"][0],
            y=obs["point"][1],
            z=obs["point"][2],
            precision=obs.get("precision"),
        )
    else:
        raise ValueError(f"Can't interpret {obs}")


class GroundControlPointManager:
    def __init__(self, path: str):
        self.points: t.Dict[str, ControlPoint] = {}
        self.path = path
        self.id_generator = id_generator()
        self.gcp_reprojections = {}

    def load_gcp_reprojections(self, filename: str) -> None:
        if os.path.isfile(filename):
            with open(filename, "r") as f:
                self.gcp_reprojections = json.load(f)
        else:
            self.gcp_reprojections = {}

    def load_from_file(self, file_path: str) -> None:
        if file_path is None:
            file_path = self.path + "/ground_control_points.json"
        self.points.clear()
        with open(file_path, "r") as f:
            input_points = json.load(f)["points"]
        for input_point in input_points:
            point = ControlPoint(input_point["id"])
            for obs in input_point["observations"]:
                point.observations.append(observation_from_json(obs))
            geo = input_point.get("position")
            if geo:
                point.geodetic_measurement = GeodeticMeasurement(
                    longitude=geo["longitude"],
                    latitude=geo["latitude"],
                    measured_at=geo["measured_at"],
                    altitude=geo.get("altitude"),
                    horizontal_std=geo.get("horizontal_std", 100),
                    vertical_std=geo.get("vertical_std"),
                )
            self.points[point.id] = point

    def points_to_json(self) -> t.List[t.Dict[str, t.Any]]:
        output_points = []
        for point in self.points.values():
            out_point: t.Dict[str, t.Any] = {
                "id": point.id,
                "observations": [
                    observation_to_json(obs) for obs in point.observations
                ],
            }
            if point.geodetic_measurement:
                out_point["position"] = point.geodetic_measurement.to_dict()
            output_points.append(out_point)
        return output_points

    def write_to_file(self, file_path):
        if file_path is None:
            file_path = self.path + "/ground_control_points.json"
        output_points = self.points_to_json()
        with open(file_path, "wt") as fp:
            json.dump({"points": output_points}, fp, indent=4, sort_keys=True)

    def get_visible_points_coords(
        self, main_image: str
    ) -> t.OrderedDict[
        str, t.Union[t.Tuple[float, float, float], t.Tuple[float, float, float, float]]
    ]:
        visible_points_coords = OrderedDict()
        for point in self.points.values():
            for obs in point.observations:
                if obs.image_id == main_image:
                    if isinstance(obs, PointMeasurement3D):
                        visible_points_coords[point.id] = (
                            obs.x,
                            obs.y,
                            obs.z,
                            obs.precision,
                        )
                    else:
                        visible_points_coords[point.id] = (
                            obs.normalized_x,
                            obs.normalized_y,
                            obs.normalized_precision,
                        )
        return visible_points_coords

    def point_exists(self, point_id):
        return point_id in self.points

    def add_point(self):
        new_id = next(self.id_generator)
        while self.point_exists(new_id):
            new_id = next(self.id_generator)
        self.points[new_id] = ControlPoint(new_id)
        return new_id

    def add_point_observation(
        self,
        point_id: str,
        shot_id: str,
        projection_or_point: t.Union[
            t.Tuple[float, float], t.Tuple[float, float, float]
        ],
        precision: float,  # certainty radius. normalized pixels for images. 3D units for 3D.
        geo: t.Optional[t.Dict[str, t.Union[float, int]]] = None,
    ):
        point = self.points[point_id]
        assert self.get_observation(point_id, shot_id) is None
        if geo:
            mtime = geo["measured_at"]
            assert isinstance(
                mtime, int
            ), "Measurement time should be integer (unix time)"
            point.geodetic_measurement = GeodeticMeasurement(
                longitude=geo["longitude"],
                latitude=geo["latitude"],
                measured_at=mtime,
                altitude=geo.get("altitude"),
                horizontal_std=geo.get("horizontal_std", 100),
                vertical_std=geo.get("vertical_std"),
            )

        if len(projection_or_point) == 2:
            obs = PointMeasurement(
                image_id=shot_id,
                normalized_x=projection_or_point[0],
                normalized_y=projection_or_point[1],
                normalized_precision=precision,
            )
        else:
            obs = PointMeasurement3D(
                image_id=shot_id,
                x=projection_or_point[0],
                y=projection_or_point[1],
                z=projection_or_point[2],
                precision=precision,
            )
        point.observations.append(obs)

    def get_worst_gcp(self):
        worst_gcp_error = 0
        worst_gcp = None
        for gcp_id in self.gcp_reprojections:
            if gcp_id not in self.points:
                continue
            for shot_id in self.gcp_reprojections[gcp_id]:
                err = self.gcp_reprojections[gcp_id][shot_id]["error"]
                if err > worst_gcp_error:
                    worst_gcp_error = err
                    worst_gcp = gcp_id

        errors_worst_gcp = [
            x["error"] for x in self.gcp_reprojections[worst_gcp].values()
        ]
        n = len(errors_worst_gcp)
        print(f"Worst GCP: {worst_gcp} unconfirmed in {n} images")
        return worst_gcp

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

    def get_observation(
        self, point_id: str, shot_id: str
    ) -> t.Optional[t.Union[PointMeasurement3D, PointMeasurement]]:
        for obs in self.points[point_id].observations:
            if obs.image_id == shot_id:
                return obs

    def remove_point_observation(
        self, point_id: str, shot_id: str, remove_latlon: bool
    ) -> None:
        point = self.points[point_id]
        if remove_latlon:
            point.geodetic_measurement = None
        point.observations = [
            obs for obs in point.observations if obs.image_id != shot_id
        ]
        if point_id in self.gcp_reprojections:
            if shot_id in self.gcp_reprojections[point_id]:
                self.gcp_reprojections[point_id][shot_id]["error"] = 0
