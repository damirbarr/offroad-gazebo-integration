"""
Shared video-streaming defaults for the Prius Linux Player integration.
"""

from dataclasses import dataclass
from typing import Dict, Tuple


DEFAULT_VIDEO_BITRATE_KBPS = 1500
DEFAULT_VIDEO_DESTINATION_HOST = '127.0.0.1'
DEFAULT_VIDEO_FPS = 15
DEFAULT_VIDEO_HEIGHT = 720
DEFAULT_VIDEO_WIDTH = 1280
DEFAULT_H265_PAYLOAD_TYPE = 96


@dataclass(frozen=True)
class CameraStreamConfig:
    name: str
    topic: str
    port: int


CAMERA_STREAM_CONFIGS: Tuple[CameraStreamConfig, ...] = (
    CameraStreamConfig('main', '/camera/main/image_raw', 1230),
    CameraStreamConfig('left_side', '/camera/left_side/image_raw', 1231),
    CameraStreamConfig('right_side', '/camera/right_side/image_raw', 1232),
    CameraStreamConfig('rear', '/camera/rear/image_raw', 1233),
    CameraStreamConfig('left_mirror', '/camera/left_mirror/image_raw', 1234),
    CameraStreamConfig('right_mirror', '/camera/right_mirror/image_raw', 1235),
)

CAMERA_STREAM_CONFIG_BY_NAME: Dict[str, CameraStreamConfig] = {
    config.name: config for config in CAMERA_STREAM_CONFIGS
}

SUPPORTED_ENCODINGS = {
    'rgb8': ('RGB', 3),
    'bgr8': ('BGR', 3),
    'rgba8': ('RGBA', 4),
    'bgra8': ('BGRA', 4),
}
