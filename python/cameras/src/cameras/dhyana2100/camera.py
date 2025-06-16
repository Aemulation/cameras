import os
import ctypes
from typing import Any, Optional
import numpy as np

from cameras.camera_protocol import (
    CameraFactoryProtocol,
    CameraProtocol,
    CameraFactoryClassRegistry,
)

CAMERA_NAME = "dhyana2100"


class CameraConfig(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_int),
        ("height", ctypes.c_int),
        ("offsetx", ctypes.c_int),
        ("offsety", ctypes.c_int),
        ("exposure_ms", ctypes.c_float),
        ("framerate", ctypes.c_float),
        ("num_buffer_frames", ctypes.c_int),
        # 1 = True, 0 = False.
        ("fast_binning", ctypes.c_int),
        # 0 is no trigger, 1 hardware trigger, 2 software trigger.
        (
            "input_trigger",
            ctypes.c_int,
        ),
        # Number of frames triggered by a single trigger event.
        (
            "trigger_frames",
            ctypes.c_int,
        ),
        ("bits_per_pixel", ctypes.c_uint),
        # 1 = True, 0 = False.
        ("test_pattern", ctypes.c_int),
        # 1 = True, 0 = False.
        ("full_mode", ctypes.c_int),
        ("analog_gain_mode", ctypes.c_int),
    ]

    def to_dict(self) -> dict[str, Any]:
        return {
            field_name: getattr(self, field_name)
            for field_name, _ in self._fields_  # type: ignore
        }


@CameraFactoryClassRegistry.register(CAMERA_NAME)
class CameraFactory(CameraFactoryProtocol):
    @staticmethod
    def create(
        camera_index: int = 0,
        enable_fan: bool = True,
        enable_TEC: bool = True,
        number_of_copy_threads_per_buffer: int = 8,
        fast_binning: bool = True,
        number_of_frame_buffers: int = 256,
        trigger_frames: int = 1,
        bits_per_pixel: int = 12,
        input_trigger: int = 0,
        *args,
        **kwargs,
    ) -> CameraProtocol:
        return Camera(
            camera_index,
            enable_fan,
            enable_TEC,
            number_of_copy_threads_per_buffer,
            fast_binning,
            number_of_frame_buffers,
            trigger_frames,
            bits_per_pixel,
            input_trigger,
        )


class Camera(CameraProtocol):
    def __init__(
        self,
        camera_index: int,
        enable_fan: bool,
        enable_TEC: bool,
        number_of_copy_threads_per_buffer: int,
        fast_binning: bool,
        number_of_frame_buffers: int,
        trigger_frames: int,
        bits_per_pixel: int,
        input_trigger: int,
    ) -> None:
        valid_bits_per_pixel = (8, 10, 12)
        if bits_per_pixel not in valid_bits_per_pixel:
            raise ValueError(
                f"invalid bits_per_pixel, options are {valid_bits_per_pixel}"
            )

        self.__camera = None
        self.__buffer_shape = None
        self.__enable_fan = enable_fan
        self.__enable_TEC = enable_TEC

        self.__camera_index = camera_index
        self.number_of_copy_threads_per_buffer = number_of_copy_threads_per_buffer

        self.__fast_binning = fast_binning
        self.__number_of_frame_buffers = number_of_frame_buffers
        self.__trigger_frames = trigger_frames
        self.__bits_per_pixel = bits_per_pixel
        self.__input_trigger = input_trigger

        self.__setup_bindings()

    def __setup_bindings(self):
        this_path = os.path.dirname(os.path.abspath(__file__))
        relative_dll_path = (
            "../../../../../cameras/dhyana2100/build/Release/dhyana2100.dll"
        )

        dll_path = os.path.abspath(this_path + "/" + relative_dll_path)
        if not os.path.exists(dll_path):
            raise RuntimeError(f"{dll_path} does not exist")

        current_directory = os.getcwd()
        os.chdir(os.path.dirname(dll_path))
        lib = ctypes.CDLL(dll_path)
        os.chdir(current_directory)

        self.lib = lib

        self.__camera_open = self.lib.camera_open
        self.__camera_open.argtypes = [ctypes.c_int32]
        self.__camera_open.restype = ctypes.c_void_p

        self.__camera_close = self.lib.camera_close
        self.__camera_close.argtypes = [ctypes.c_void_p]
        self.__camera_close.restype = None

        self.__get_config = self.lib.get_config
        self.__get_config.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(CameraConfig),
            ctypes.POINTER(CameraConfig),
            ctypes.POINTER(CameraConfig),
        ]
        self.__get_config.restype = None

        self.__set_config = self.lib.set_config
        self.__set_config.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(CameraConfig),
        ]
        self.__set_config.restype = None

        self.__add_buffer = self.lib.add_buffer
        self.__add_buffer.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
            ctypes.c_size_t,
            ctypes.c_size_t,
        ]
        self.__add_buffer.restype = None

        self.__get_next_buffer = self.lib.get_next_buffer
        self.__get_next_buffer.argtypes = [
            ctypes.c_void_p,
            ctypes.c_void_p,
        ]
        self.__get_next_buffer.restype = None

        self.__start = self.lib.start
        self.__start.argtypes = [
            ctypes.c_void_p,
        ]
        self.__start.restype = None

        self.__stop = self.lib.stop
        self.__stop.argtypes = [
            ctypes.c_void_p,
        ]
        self.__stop.restype = None

        self.__get_lost_frames = self.lib.get_lost_frames
        self.__get_lost_frames.argtypes = [
            ctypes.c_void_p,
        ]
        self.__get_lost_frames.restype = ctypes.c_uint64

        self.__get_temperature = self.lib.get_temperature
        self.__get_temperature.argtypes = [
            ctypes.c_void_p,
            ctypes.POINTER(ctypes.c_float),
            ctypes.POINTER(ctypes.c_float),
        ]
        self.__get_temperature.restype = None

        self.__set_temperature_control = self.lib.set_temperature_control
        self.__set_temperature_control.argtypes = [
            ctypes.c_void_p,
            ctypes.c_bool,
            ctypes.c_bool,
        ]
        self.__set_temperature_control.restype = None

    def set_framerate(self, framerate: int):
        new_config = CameraConfig(
            **(self.__config.to_dict() | {"framerate": framerate})
        )

        return self.set_config(new_config)

    def set_height(self, height: int):
        new_config = CameraConfig(**(self.__config.to_dict() | {"height": height}))

        return self.set_config(new_config)

    def set_width(self, width: int):
        new_config = CameraConfig(**(self.__config.to_dict() | {"width": width}))

        return self.set_config(new_config)

    def get_framerate(self) -> int:
        return self.__config.framerate

    def get_height(self) -> int:
        return self.__config.height

    def get_width(self) -> int:
        return self.__config.width

    def get_max_framerate(self) -> Optional[int]:
        return self.__max_config.framerate

    def get_max_width(self) -> Optional[int]:
        return self.__max_config.framerate

    def get_max_height(self) -> Optional[int]:
        return self.__max_config.framerate

    def open(self):
        if self.__camera is not None:
            raise RuntimeError("Camera already open")

        self.__camera = self.__camera_open(
            ctypes.c_int(self.__camera_index),
            ctypes.c_uint(self.number_of_copy_threads_per_buffer),
        )

        self.update_configs()
        new_config = {
            "full_mode": 1,
            "fast_binning": 1 if self.__fast_binning else 0,
            "trigger_frames": self.__trigger_frames,
            "input_trigger": self.__input_trigger,
            "bits_per_pixel": self.__bits_per_pixel,
            "num_buffer_frames": self.__number_of_frame_buffers,
        }
        self.set_config(CameraConfig(**(self.__config.to_dict() | new_config)))
        self.set_temperature(self.__enable_fan, self.__enable_TEC)

    def close(self):
        if self.__camera is None:
            raise RuntimeError("Camera already closed")

        self.__camera_close(self.__camera)

    def update_configs(self):
        if self.__camera is None:
            raise RuntimeError("Camera is not open")

        config = CameraConfig()
        min_config = CameraConfig()
        max_config = CameraConfig()

        self.__get_config(
            self.__camera,
            ctypes.byref(config),
            ctypes.byref(min_config),
            ctypes.byref(max_config),
        )

        self.__min_config = min_config
        self.__max_config = max_config
        self.__config = config

    def get_config(self) -> CameraConfig:
        return self.__config

    def set_config(self, camera_config: CameraConfig):
        if self.__camera is None:
            raise RuntimeError("Camera is not open.")

        self.__set_config(self.__camera, camera_config)

        self.update_configs()

    def add_buffer(self, buffer: np.ndarray):
        if self.__camera is None:
            raise RuntimeError("Camera is not open")

        if buffer.dtype == np.uint8:
            num_bytes_per_pixel = 1
            buffer_ctype = ctypes.c_uint8
        elif buffer.dtype == np.uint16:
            num_bytes_per_pixel = 2
            buffer_ctype = ctypes.c_uint16
        else:
            raise RuntimeError("Unsupported dtype for camera buffer: {buffer.dtype}")

        if self.__buffer_shape is None:
            self.__buffer_shape = buffer.shape
        elif self.__buffer_shape != buffer.shape:
            raise RuntimeError(
                f"Expected buffer of size ({self.__buffer_shape}), but got buffer of size ({buffer.shape})"
            )

        buffer_pointer = buffer.ctypes.data_as(ctypes.POINTER(buffer_ctype))

        (num_images_per_buffer, image_height, image_width) = buffer.shape
        image_size = ctypes.c_size_t(image_height * image_width * num_bytes_per_pixel)

        self.__add_buffer(
            self.__camera,
            buffer_pointer,
            image_size,
            ctypes.c_size_t(num_images_per_buffer),
        )

    def get_next_buffer(self) -> np.ndarray:
        if self.__camera is None:
            raise RuntimeError("Camera is not open")

        if self.__config.bits_per_pixel <= 8:
            pointer_ctype = ctypes.c_uint8
        elif self.__config.bits_per_pixel <= 16:
            pointer_ctype = ctypes.c_uint16
        else:
            raise RuntimeError(
                "Unsupported number of bits per pixel: {self.config.bits_per_pixel}"
            )

        buffer_pointer = ctypes.POINTER(pointer_ctype)()

        self.__get_next_buffer(self.__camera, ctypes.pointer(buffer_pointer))

        buffer_array = np.ctypeslib.as_array(buffer_pointer, shape=self.__buffer_shape)
        return buffer_array

    def start_recording(self):
        if self.__camera is None:
            raise RuntimeError("Camera is not open")

        self.__start(self.__camera)

    def stop_recording(self):
        if self.__camera is None:
            raise RuntimeError("Camera is not open")

        self.__stop(self.__camera)

    def get_lost_frames(self):
        if self.__camera is None:
            raise RuntimeError("Camera is not open")

        return self.__get_lost_frames(self.__camera)

    def get_temperature(self) -> tuple[float, float]:
        if self.__camera is None:
            raise RuntimeError("Camera is not open")

        device_temp = ctypes.c_float()
        sensor_temp = ctypes.c_float()

        self.__get_temperature(
            self.__camera, ctypes.byref(device_temp), ctypes.byref(sensor_temp)
        )

        return (float(device_temp.value), float(sensor_temp.value))

    def set_temperature(self, enable_fan: bool, enable_TEC: bool):
        if self.__camera is None:
            raise RuntimeError("Camera is not open")

        enable_fan_ctype = ctypes.c_bool(enable_fan)
        enable_TEC_ctype = ctypes.c_bool(enable_TEC)

        self.__set_temperature_control(
            self.__camera, enable_fan_ctype, enable_TEC_ctype
        )
