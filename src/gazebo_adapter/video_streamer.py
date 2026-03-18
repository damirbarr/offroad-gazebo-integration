#!/usr/bin/env python3
"""
Bridge Prius camera topics to Linux Player as H.265 RTP-over-UDP streams.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np
import gi
import rclpy

gi.require_version('Gst', '1.0')

from gi.repository import Gst  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

try:
    from .video_streaming import (
        CAMERA_STREAM_CONFIGS,
        DEFAULT_H265_PAYLOAD_TYPE,
        DEFAULT_VIDEO_BITRATE_KBPS,
        DEFAULT_VIDEO_DESTINATION_HOST,
        DEFAULT_VIDEO_FPS,
        DEFAULT_VIDEO_HEIGHT,
        DEFAULT_VIDEO_WIDTH,
        SUPPORTED_ENCODINGS,
    )
except ImportError:
    from gazebo_adapter.video_streaming import (
        CAMERA_STREAM_CONFIGS,
        DEFAULT_H265_PAYLOAD_TYPE,
        DEFAULT_VIDEO_BITRATE_KBPS,
        DEFAULT_VIDEO_DESTINATION_HOST,
        DEFAULT_VIDEO_FPS,
        DEFAULT_VIDEO_HEIGHT,
        DEFAULT_VIDEO_WIDTH,
        SUPPORTED_ENCODINGS,
    )


@dataclass
class RuntimeStreamConfig:
    topic: str
    port: int


class VideoPipeline:
    def __init__(
        self,
        camera_name: str,
        destination_host: str,
        port: int,
        bitrate_kbps: int,
        width: int,
        height: int,
        fps: int,
    ):
        self.camera_name = camera_name
        self.destination_host = destination_host
        self.port = port
        self.width = width
        self.height = height
        self.fps = fps
        self.frame_duration_ns = int(Gst.SECOND / fps)
        self.next_pts = 0
        self.expected_encoding: Optional[str] = None

        pipeline_description = (
            'appsrc name=source is-live=true format=time block=false '
            '! videoconvert '
            f'! x265enc bitrate={bitrate_kbps} '
            '! queue max-size-buffers=1 leaky=downstream '
            f'! rtph265pay pt={DEFAULT_H265_PAYLOAD_TYPE} config-interval=1 mtu=1000 '
            f'! udpsink host={destination_host} port={port} sync=false async=false'
        )
        self.pipeline = Gst.parse_launch(pipeline_description)
        self.appsrc = self.pipeline.get_by_name('source')
        self.appsrc.set_property(
            'caps',
            Gst.Caps.from_string(
                f'video/x-raw,format=RGB,width={width},height={height},framerate={fps}/1'
            ),
        )
        self.pipeline.set_state(Gst.State.PLAYING)

    def push_image(self, msg: Image) -> bool:
        encoding_info = SUPPORTED_ENCODINGS.get(msg.encoding.lower())
        if encoding_info is None:
            return False

        gst_format, channels = encoding_info
        if self.expected_encoding != msg.encoding:
            caps = Gst.Caps.from_string(
                f'video/x-raw,format={gst_format},width={msg.width},height={msg.height},framerate={self.fps}/1'
            )
            self.appsrc.set_property('caps', caps)
            self.expected_encoding = msg.encoding

        payload = self._pack_image(msg, channels)
        if payload is None:
            return False

        buffer = Gst.Buffer.new_allocate(None, len(payload), None)
        buffer.fill(0, payload)
        buffer.pts = self.next_pts
        buffer.dts = self.next_pts
        buffer.duration = self.frame_duration_ns
        self.next_pts += self.frame_duration_ns

        result = self.appsrc.emit('push-buffer', buffer)
        return result == Gst.FlowReturn.OK

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.set_state(Gst.State.NULL)

    @staticmethod
    def _pack_image(msg: Image, channels: int) -> Optional[bytes]:
        expected_step = msg.width * channels
        if msg.step == expected_step:
            return bytes(msg.data)

        if msg.step < expected_step:
            return None

        row_view = np.frombuffer(msg.data, dtype=np.uint8)
        try:
            reshaped = row_view.reshape((msg.height, msg.step))
        except ValueError:
            return None
        trimmed = reshaped[:, :expected_step]
        return trimmed.tobytes()


class VideoStreamer(Node):
    def __init__(self):
        super().__init__('video_streamer')
        Gst.init(None)

        self.declare_parameter('video_streaming.enabled', True)
        self.declare_parameter(
            'video_streaming.destination_host',
            DEFAULT_VIDEO_DESTINATION_HOST,
        )
        self.declare_parameter(
            'video_streaming.bitrate_kbps',
            DEFAULT_VIDEO_BITRATE_KBPS,
        )
        self.declare_parameter('video_streaming.width', DEFAULT_VIDEO_WIDTH)
        self.declare_parameter('video_streaming.height', DEFAULT_VIDEO_HEIGHT)
        self.declare_parameter('video_streaming.fps', DEFAULT_VIDEO_FPS)

        self._stream_configs: Dict[str, RuntimeStreamConfig] = {}
        for camera_config in CAMERA_STREAM_CONFIGS:
            topic_parameter = f'video_streaming.cameras.{camera_config.name}.topic'
            port_parameter = f'video_streaming.cameras.{camera_config.name}.port'
            self.declare_parameter(topic_parameter, camera_config.topic)
            self.declare_parameter(port_parameter, camera_config.port)
            topic = self.get_parameter(topic_parameter).get_parameter_value().string_value
            port = self.get_parameter(port_parameter).get_parameter_value().integer_value
            self._stream_configs[camera_config.name] = RuntimeStreamConfig(
                topic=topic,
                port=int(port),
            )

        self._enabled = self._parameter_bool(
            self.get_parameter('video_streaming.enabled').value
        )
        self._destination_host = self.get_parameter(
            'video_streaming.destination_host'
        ).value
        self._bitrate_kbps = int(
            self.get_parameter('video_streaming.bitrate_kbps').value
        )
        self._fps = int(self.get_parameter('video_streaming.fps').value)

        self._pipelines: Dict[str, VideoPipeline] = {}
        self._latest_messages: Dict[str, Optional[Image]] = {
            name: None for name in self._stream_configs
        }
        self._latest_frame_ids: Dict[str, int] = {
            name: 0 for name in self._stream_configs
        }
        self._last_pushed_frame_ids: Dict[str, int] = {
            name: 0 for name in self._stream_configs
        }
        self._subscriptions = []

        if not self._enabled:
            self.get_logger().info('Video streaming disabled by parameter')
            return

        for camera_name, runtime_config in self._stream_configs.items():
            self._pipelines[camera_name] = VideoPipeline(
                camera_name=camera_name,
                destination_host=self._destination_host,
                port=runtime_config.port,
                bitrate_kbps=self._bitrate_kbps,
                width=int(self.get_parameter('video_streaming.width').value),
                height=int(self.get_parameter('video_streaming.height').value),
                fps=self._fps,
            )
            self._subscriptions.append(
                self.create_subscription(
                    Image,
                    runtime_config.topic,
                    self._make_image_callback(camera_name),
                    10,
                )
            )

        self.create_timer(1.0 / self._fps, self._publish_latest_frames)
        self.get_logger().info(
            f'Streaming Prius cameras to Linux Player at {self._destination_host}:1230-1235'
        )

    def destroy_node(self):
        for pipeline in self._pipelines.values():
            pipeline.stop()
        super().destroy_node()

    def _make_image_callback(self, camera_name: str):
        def _callback(msg: Image):
            self._latest_messages[camera_name] = msg
            self._latest_frame_ids[camera_name] += 1

        return _callback

    def _publish_latest_frames(self):
        for camera_name, pipeline in self._pipelines.items():
            latest_frame_id = self._latest_frame_ids[camera_name]
            if latest_frame_id == self._last_pushed_frame_ids[camera_name]:
                continue

            msg = self._latest_messages[camera_name]
            if msg is None:
                continue

            if not pipeline.push_image(msg):
                self.get_logger().warning(
                    f'Failed to push frame for {camera_name} on {self._stream_configs[camera_name].topic}'
                )
                continue

            self._last_pushed_frame_ids[camera_name] = latest_frame_id

    @staticmethod
    def _parameter_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in {'true', '1', 'yes', 'on'}
        return bool(value)


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
