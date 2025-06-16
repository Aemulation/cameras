from abc import abstractmethod
from typing import Callable, Dict, Protocol, Optional, Type

import numpy as np


class CameraProtocol(Protocol):
    def __init__(self, *args, **kwargs) -> None: ...

    def set_framerate(self, framerate: int): ...

    def set_height(self, height: int): ...

    def set_width(self, width: int): ...

    def get_framerate(self) -> int: ...

    def get_height(self) -> int: ...

    def get_width(self) -> int: ...

    def get_max_framerate(self) -> Optional[int]: ...

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
    @staticmethod
    @abstractmethod
    def create(
        *args,
        **kwargs,
    ) -> CameraProtocol: ...


class CameraFactoryClassRegistry:
    registry: Dict[str, Type[CameraFactoryProtocol]] = {}

    @classmethod
    def register(cls, camera_factory_name: str) -> Callable:
        print(f"[DEBUG] cls: {cls}")
        print(f"[DEBUG] cls.__name__: {cls.__name__}")
        print(f"[DEBUG] cls.__dict__.keys(): {list(cls.__dict__.keys())}")
        print(f"[DEBUG] cls.registry (exists?): {'registry' in cls.__dict__}")
        print(f"[DEBUG] cls.registry: {getattr(cls, 'registry', 'Not Found')}")

        def inner_wrapper(
            wrapped_factory: Type[CameraFactoryProtocol],
        ) -> Type[CameraFactoryProtocol]:
            cls.registry[camera_factory_name] = wrapped_factory
            return wrapped_factory

        return inner_wrapper
