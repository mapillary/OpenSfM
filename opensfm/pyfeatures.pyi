from typing import overload, Any, List

import numpy


def akaze(arg0: numpy.ndarray, arg1: AKAZEOptions) -> List[numpy.ndarray]:
    ...


def hahog(
    image: numpy.ndarray,
    peak_threshold: float = ...,
    edge_threshold: float = ...,
    target_num_features: int = ...,
    use_adaptive_suppression: bool = ...,
) -> List[numpy.ndarray]:
    ...


def match_using_words(
    arg0: numpy.ndarray,
    arg1: numpy.ndarray,
    arg2: numpy.ndarray,
    arg3: numpy.ndarray,
    arg4: float,
    arg5: int,
) -> object:
    ...


class AKAZEOptions:
    def __init__(self) -> None:
        ...

    @property
    def derivative_factor(self) -> float:
        ...

    @derivative_factor.setter
    def derivative_factor(self, val: float) -> None:
        ...

    @property
    def descriptor(self) -> AkazeDescriptorType:
        ...

    @descriptor.setter
    def descriptor(self, val: AkazeDescriptorType) -> None:
        ...

    @property
    def descriptor_channels(self) -> int:
        ...

    @descriptor_channels.setter
    def descriptor_channels(self, val: int) -> None:
        ...

    @property
    def descriptor_pattern_size(self) -> int:
        ...

    @descriptor_pattern_size.setter
    def descriptor_pattern_size(self, val: int) -> None:
        ...

    @property
    def descriptor_size(self) -> int:
        ...

    @descriptor_size.setter
    def descriptor_size(self, val: int) -> None:
        ...

    @property
    def diffusivity(self) -> float:
        ...

    @diffusivity.setter
    def diffusivity(self, val: float) -> None:
        ...

    @property
    def dthreshold(self) -> float:
        ...

    @dthreshold.setter
    def dthreshold(self, val: float) -> None:
        ...

    @property
    def img_height(self) -> int:
        ...

    @img_height.setter
    def img_height(self, val: int) -> None:
        ...

    @property
    def img_width(self) -> int:
        ...

    @img_width.setter
    def img_width(self, val: int) -> None:
        ...

    @property
    def kcontrast(self) -> float:
        ...

    @kcontrast.setter
    def kcontrast(self, val: float) -> None:
        ...

    @property
    def kcontrast_nbins(self) -> int:
        ...

    @kcontrast_nbins.setter
    def kcontrast_nbins(self, val: int) -> None:
        ...

    @property
    def kcontrast_percentile(self) -> float:
        ...

    @kcontrast_percentile.setter
    def kcontrast_percentile(self, val: float) -> None:
        ...

    @property
    def min_dthreshold(self) -> float:
        ...

    @min_dthreshold.setter
    def min_dthreshold(self, val: float) -> None:
        ...

    @property
    def nsublevels(self) -> int:
        ...

    @nsublevels.setter
    def nsublevels(self, val: int) -> None:
        ...

    @property
    def omax(self) -> int:
        ...

    @omax.setter
    def omax(self, val: int) -> None:
        ...

    @property
    def omin(self) -> int:
        ...

    @omin.setter
    def omin(self, val: int) -> None:
        ...

    @property
    def save_keypoints(self) -> bool:
        ...

    @save_keypoints.setter
    def save_keypoints(self, val: bool) -> None:
        ...

    @property
    def save_scale_space(self) -> bool:
        ...

    @save_scale_space.setter
    def save_scale_space(self, val: bool) -> None:
        ...

    @property
    def sderivatives(self) -> float:
        ...

    @sderivatives.setter
    def sderivatives(self, val: float) -> None:
        ...

    @property
    def soffset(self) -> float:
        ...

    @soffset.setter
    def soffset(self, val: float) -> None:
        ...

    @property
    def target_num_features(self) -> int:
        ...

    @target_num_features.setter
    def target_num_features(self, val: int) -> None:
        ...

    @property
    def use_adaptive_suppression(self) -> bool:
        ...

    @use_adaptive_suppression.setter
    def use_adaptive_suppression(self, val: bool) -> None:
        ...

    @property
    def use_isotropic_diffusion(self) -> bool:
        ...

    @use_isotropic_diffusion.setter
    def use_isotropic_diffusion(self, val: bool) -> None:
        ...

    @property
    def verbosity(self) -> bool:
        ...

    @verbosity.setter
    def verbosity(self, val: bool) -> None:
        ...


class AkazeDescriptorType:
    MLDB: Any = ...
    MLDB_UPRIGHT: Any = ...
    MSURF: Any = ...
    MSURF_UPRIGHT: Any = ...
    SURF: Any = ...
    SURF_UPRIGHT: Any = ...

    def __init__(self, arg0: int) -> None:
        ...

    @overload
    def __eq__(self, arg0: AkazeDescriptorType) -> bool:
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
    def __ne__(self, arg0: AkazeDescriptorType) -> bool:
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
