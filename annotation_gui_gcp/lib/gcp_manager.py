# pyre-strict
import json
import os
import typing as t
from collections import OrderedDict
from dataclasses import dataclass
from typing import Generator


def id_generator() -> Generator[str, None, None]:
    i = 0
    while True:
        yield str(i)
        i += 1


class GeodeticDict(t.TypedDict, total=False):
    """TypedDict for geodetic measurement JSON representation."""

    latitude: float
    longitude: float
    horizontal_std: float
    measured_at: int
    altitude: float
    vertical_std: float


class PointObservationDict(t.TypedDict, total=False):
    """TypedDict for 2D point observation JSON representation."""

    shot_id: str
    projection: list[float]
    precision: float


class Point3DObservationDict(t.TypedDict, total=False):
    """TypedDict for 3D point observation JSON representation."""

    shot_id: str
    point: list[float]
    precision: float | None


# Union type for observation dictionaries
ObservationDict = PointObservationDict | Point3DObservationDict


class ControlPointDict(t.TypedDict, total=False):
    """TypedDict for control point JSON representation."""

    id: str
    observations: list[ObservationDict]
    position: GeodeticDict


class GCPFileDict(t.TypedDict):
    """TypedDict for the ground control points JSON file."""

    points: list[ControlPointDict]


class GCPReprojectionError(t.TypedDict):
    """TypedDict for GCP reprojection error data."""

    error: float


@dataclass(frozen=True, eq=True)
class GeodeticMeasurement:
    longitude: float
    latitude: float
    horizontal_std: float  # Standard deviation (m), horizontal.
    measured_at: int  # Unix time
    altitude: float | None = None
    vertical_std: float | None = None  # Standard deviation (m), vertical.

    def to_dict(self) -> GeodeticDict:
        d: GeodeticDict = {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "horizontal_std": self.horizontal_std,
            "measured_at": self.measured_at,
        }
        if self.altitude is not None:
            d["altitude"] = self.altitude
        if self.vertical_std is not None:
            d["vertical_std"] = self.vertical_std
        return d

    @classmethod
    def from_dict(cls, d: GeodeticDict) -> "GeodeticMeasurement":
        return GeodeticMeasurement(
            longitude=d["longitude"],
            latitude=d["latitude"],
            horizontal_std=d["horizontal_std"],
            measured_at=d["measured_at"],
            altitude=d.get("altitude"),
            vertical_std=d.get("vertical_std"),
        )


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
    precision: float | None  # precision as standard deviation in (x,y,z) units


class ControlPoint:
    def __init__(self, point_id: str) -> None:
        self.id = point_id
        self.observations: list[PointMeasurement | PointMeasurement3D] = []
        self.geodetic_measurement: GeodeticMeasurement | None = None

    def __repr__(self) -> str:
        if self.geodetic_measurement:
            return f"ControlPoint {self.id} @ {self.geodetic_measurement} with {len(self.observations)} image observations"
        else:
            return f"ControlPoint {self.id} with {len(self.observations)} image observations"


def observation_to_json(
    obs: PointMeasurement3D | PointMeasurement,
) -> ObservationDict:
    if isinstance(obs, PointMeasurement):
        return PointObservationDict(
            shot_id=obs.image_id,
            projection=[obs.normalized_x, obs.normalized_y],
            precision=obs.normalized_precision,
        )
    elif isinstance(obs, PointMeasurement3D):
        return Point3DObservationDict(
            shot_id=obs.image_id,
            point=[obs.x, obs.y, obs.z],
            precision=obs.precision,
        )
    else:
        raise ValueError(f"Unknown observation {obs}")


def observation_from_json(
    obs: ObservationDict,
) -> PointMeasurement3D | PointMeasurement:
    if "projection" in obs:
        # This is a 2D observation (PointObservationDict)
        obs_2d = t.cast(PointObservationDict, obs)
        projection = obs_2d["projection"]
        return PointMeasurement(
            image_id=obs_2d["shot_id"],
            normalized_x=projection[0],
            normalized_y=projection[1],
            normalized_precision=obs_2d.get(
                "precision", 0.004
            ),  # 3-sigma (99%) confidence of about 8 px in a 640px image.
        )
    elif "point" in obs:
        # This is a 3D observation (Point3DObservationDict)
        obs_3d = t.cast(Point3DObservationDict, obs)
        point = obs_3d["point"]
        return PointMeasurement3D(
            image_id=obs_3d["shot_id"],
            x=point[0],
            y=point[1],
            z=point[2],
            precision=obs_3d.get("precision"),
        )
    else:
        raise ValueError(f"Can't interpret {obs}")


class GroundControlPointManager:
    def __init__(self, path: str) -> None:
        self.points: dict[str, ControlPoint] = {}
        self.path: str = path
        self.id_generator: Generator[str, None, None] = id_generator()
        self.gcp_reprojections: dict[str, dict[str, GCPReprojectionError]] = {}

    def load_gcp_reprojections(self, filename: str) -> None:
        if os.path.isfile(filename):
            with open(filename, "r") as f:
                self.gcp_reprojections = json.load(f)
        else:
            self.gcp_reprojections = {}

    def load_from_file(self, file_path: str | None) -> None:
        if file_path is None:
            file_path = self.path + "/ground_control_points.json"
        self.points.clear()
        with open(file_path, "r") as f:
            gcp_file: GCPFileDict = json.load(f)
        input_points = gcp_file["points"]
        for input_point in input_points:
            point = ControlPoint(input_point["id"])
            observations = input_point.get("observations", [])
            for obs in observations:
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

    def points_to_json(self) -> list[ControlPointDict]:
        output_points: list[ControlPointDict] = []
        for point in self.points.values():
            out_point: ControlPointDict = {
                "id": point.id,
                "observations": [
                    observation_to_json(obs) for obs in point.observations
                ],
            }
            if point.geodetic_measurement:
                out_point["position"] = point.geodetic_measurement.to_dict()
            output_points.append(out_point)
        return output_points

    def write_to_file(self, file_path: str | None) -> None:
        if file_path is None:
            file_path = self.path + "/ground_control_points.json"
        output_points = self.points_to_json()
        with open(file_path, "wt") as fp:
            json.dump({"points": output_points}, fp, indent=4, sort_keys=True)

    def get_visible_points_coords(
        self, main_image: str
    ) -> t.OrderedDict[
        str, tuple[float, float, float] | tuple[float, float, float, float | None]
    ]:
        visible_points_coords: t.OrderedDict[
            str, tuple[float, float, float] | tuple[float, float, float, float | None]
        ] = OrderedDict()
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

    def point_exists(self, point_id: str) -> bool:
        return point_id in self.points

    def add_point(self) -> str:
        new_id = next(self.id_generator)
        while self.point_exists(new_id):
            new_id = next(self.id_generator)
        self.points[new_id] = ControlPoint(new_id)
        return new_id

    def add_point_observation(
        self,
        point_id: str,
        shot_id: str,
        projection_or_point: tuple[float, float] | tuple[float, float, float],
        precision: float,  # certainty radius. normalized pixels for images. 3D units for 3D.
        geo: GeodeticDict | None = None,
    ) -> None:
        point = self.points[point_id]
        assert self.get_observation(point_id, shot_id) is None
        if geo:
            mtime = geo["measured_at"]
            assert isinstance(mtime, int), (
                "Measurement time should be integer (unix time)"
            )
            point.geodetic_measurement = GeodeticMeasurement(
                longitude=geo["longitude"],
                latitude=geo["latitude"],
                measured_at=mtime,
                altitude=geo.get("altitude"),
                horizontal_std=geo.get("horizontal_std", 100),
                vertical_std=geo.get("vertical_std"),
            )

        if len(projection_or_point) == 2:
            obs: PointMeasurement | PointMeasurement3D = PointMeasurement(
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

    def get_worst_gcp(self) -> str | None:
        worst_gcp_error: float = 0
        worst_gcp: str | None = None
        for gcp_id in self.gcp_reprojections:
            if gcp_id not in self.points:
                continue
            for shot_id in self.gcp_reprojections[gcp_id]:
                err = self.gcp_reprojections[gcp_id][shot_id]["error"]
                if err > worst_gcp_error:
                    worst_gcp_error = err
                    worst_gcp = gcp_id

        if worst_gcp is None:
            return None

        errors_worst_gcp = [
            x["error"] for x in self.gcp_reprojections[worst_gcp].values()
        ]
        n = len(errors_worst_gcp)
        print(f"Worst GCP: {worst_gcp} unconfirmed in {n} images")
        return worst_gcp

    def shot_with_max_gcp_error(
        self, image_keys: t.Iterable[str], gcp: str
    ) -> str | None:
        # Return they key with most reprojection error for this GCP
        annotated_images = set(self.gcp_reprojections[gcp]).intersection(
            set(image_keys)
        )
        errors: dict[str, float] = {
            k: self.gcp_reprojections[gcp][k]["error"] for k in annotated_images
        }
        if len(errors) > 0:
            return max(errors, key=lambda k: errors[k])
        else:
            return None

    def remove_gcp(self, point_id: str) -> None:
        if self.point_exists(point_id):
            del self.points[point_id]

    def get_observation(
        self, point_id: str, shot_id: str
    ) -> PointMeasurement3D | PointMeasurement | None:
        for obs in self.points[point_id].observations:
            if obs.image_id == shot_id:
                return obs
        return None

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
