from typing import List, Any
from typing import overload

import numpy

LMedS: Any
MSAC: Any
RANSAC: Any


def ransac_absolute_pose(*args, **kwargs) -> Any:
    ...


def ransac_absolute_pose_known_rotation(*args, **kwargs) -> Any:
    ...


def ransac_essential(*args, **kwargs) -> Any:
    ...


def ransac_line(*args, **kwargs) -> Any:
    ...


def ransac_relative_pose(*args, **kwargs) -> Any:
    ...


def ransac_relative_rotation(*args, **kwargs) -> Any:
    ...


class RansacType:
    LMedS: Any = ...
    MSAC: Any = ...
    RANSAC: Any = ...

    def __init__(self, arg0: int) -> None:
        ...

    @overload
    def __eq__(self, arg0: RansacType) -> bool:
        ...

    @overload
    def __eq__(self, arg0: int) -> bool:
        ...

    @overload
    def __eq__(*args, **kwargs) -> Any:
        ...

    def __getstate__(self) -> tuple:
        ...

    def __hash__(self) -> int:
        ...

    def __int__(self) -> int:
        ...

    @overload
    def __ne__(self, arg0: RansacType) -> bool:
        ...

    @overload
    def __ne__(self, arg0: int) -> bool:
        ...

    @overload
    def __ne__(*args, **kwargs) -> Any:
        ...

    def __setstate__(self, arg0: tuple) -> None:
        ...

    @property
    def __members__(self) -> dict:
        ...


class RobustEstimatorParams:
    def __init__(self) -> None:
        ...

    @property
    def iterations(self) -> int:
        ...

    @iterations.setter
    def iterations(self, val: int) -> None:
        ...

    @property
    def probability(self) -> float:
        ...

    @probability.setter
    def probability(self, val: float) -> None:
        ...

    @property
    def use_iteration_reduction(self) -> bool:
        ...

    @use_iteration_reduction.setter
    def use_iteration_reduction(self, val: bool) -> None:
        ...

    @property
    def use_local_optimization(self) -> bool:
        ...

    @use_local_optimization.setter
    def use_local_optimization(self, val: bool) -> None:
        ...


class ScoreInfoLine:
    def __init__(self) -> None:
        ...

    @property
    def inliers_indices(self) -> List[int]:
        ...

    @inliers_indices.setter
    def inliers_indices(self, val: List[int]) -> None:
        ...

    @property
    def lo_model(self) -> numpy.ndarray:
        ...

    @lo_model.setter
    def lo_model(self, val: numpy.ndarray) -> None:
        ...

    @property
    def model(self) -> numpy.ndarray:
        ...

    @model.setter
    def model(self, val: numpy.ndarray) -> None:
        ...

    @property
    def score(self) -> float:
        ...

    @score.setter
    def score(self, val: float) -> None:
        ...


class ScoreInfoMatrix34d:
    def __init__(self) -> None:
        ...

    @property
    def inliers_indices(self) -> List[int]:
        ...

    @inliers_indices.setter
    def inliers_indices(self, val: List[int]) -> None:
        ...

    @property
    def lo_model(self) -> numpy.ndarray:
        ...

    @lo_model.setter
    def lo_model(self, val: numpy.ndarray) -> None:
        ...

    @property
    def model(self) -> numpy.ndarray:
        ...

    @model.setter
    def model(self, val: numpy.ndarray) -> None:
        ...

    @property
    def score(self) -> float:
        ...

    @score.setter
    def score(self, val: float) -> None:
        ...


class ScoreInfoMatrix3d:
    def __init__(self) -> None:
        ...

    @property
    def inliers_indices(self) -> List[int]:
        ...

    @inliers_indices.setter
    def inliers_indices(self, val: List[int]) -> None:
        ...

    @property
    def lo_model(self) -> numpy.ndarray:
        ...

    @lo_model.setter
    def lo_model(self, val: numpy.ndarray) -> None:
        ...

    @property
    def model(self) -> numpy.ndarray:
        ...

    @model.setter
    def model(self, val: numpy.ndarray) -> None:
        ...

    @property
    def score(self) -> float:
        ...

    @score.setter
    def score(self, val: float) -> None:
        ...


class ScoreInfoVector3d:
    def __init__(self) -> None:
        ...

    @property
    def inliers_indices(self) -> List[int]:
        ...

    @inliers_indices.setter
    def inliers_indices(self, val: List[int]) -> None:
        ...

    @property
    def lo_model(self) -> numpy.ndarray:
        ...

    @lo_model.setter
    def lo_model(self, val: numpy.ndarray) -> None:
        ...

    @property
    def model(self) -> numpy.ndarray:
        ...

    @model.setter
    def model(self, val: numpy.ndarray) -> None:
        ...

    @property
    def score(self) -> float:
        ...

    @score.setter
    def score(self, val: float) -> None:
        ...
