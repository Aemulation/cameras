from abc import abstractmethod
from typing import Callable, Dict, Protocol, Optional, Type

import numpy as np


class CameraProtocol(Protocol):
    def __init__(self, *args, **kwargs) -> None: ...

    def set_framerate(self, framerate: float): ...

    def set_height(self, height: int): ...

    def set_width(self, width: int): ...

    def get_framerate(self) -> float: ...

    def get_height(self) -> int: ...

    def get_width(self) -> int: ...

    def get_frame_size(self) -> int: ...

    def get_max_framerate(self) -> Optional[float]: ...

    def get_max_width(self) -> Optional[int]: ...

    def get_max_height(self) -> Optional[int]: ...

    def open(self): ...

    def close(self): ...

    def start_recording(self): ...

    def stop_recording(self): ...

    def get_lost_frames(self) -> int: ...

    def add_buffer(self, buffer: np.ndarray): ...

    def get_next_buffer(self) -> np.ndarray: ...


class CameraFactoryProtocol(Protocol):
    @abstractmethod
    def __init__(self, **kwargs) -> None: ...

    @abstractmethod
    def create(
        self,
        *args,
        **kwargs,
    ) -> CameraProtocol: ...


class CameraFactoryClassRegistry:
    registry: Dict[str, Type[CameraFactoryProtocol]] = {}

    @classmethod
    def register(cls, camera_factory_name: str) -> Callable:
        def inner_wrapper(
            wrapped_factory: Type[CameraFactoryProtocol],
        ) -> Type[CameraFactoryProtocol]:
            cls.registry[camera_factory_name] = wrapped_factory
            return wrapped_factory

        return inner_wrapper

    @classmethod
    def create(cls, camera_name: str, **kwargs) -> CameraFactoryProtocol:
        try:
            camera_factory = cls.registry[camera_name](**kwargs)
            return camera_factory
        except KeyError:
            raise ValueError(
                f"Camera {camera_name} not found, available camera's are: {','.join(cls.registry.keys())}"
            )
