#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Continuous robotics loop: capture RGB from a Mech-Eye camera, stream it to a remote
OpenVLA deployment, and drive an AUBO arm with the predicted Cartesian delta until
the operator stops execution.  All captured frames are recorded to a video file.

The OpenVLA action is expected to contain seven float values:
  - indices [0:3]: translational deltas for the tool center point (meters)
  - indices [3:6]: rotational deltas expressed as axis-angle (radians)
  - index [6]    : gripper command (reserved for future use)

Only the first six values are consumed here; the gripper command is parsed and
reported but not executed.  The script focuses on the end-effector motion.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional, Sequence

import cv2
import numpy as np
from PIL import Image

import pyaubo_sdk
from mecheye.shared import show_error
from mecheye.area_scan_3d_camera import (
    Camera,
    ColorTypeOf2DCamera_Color,
    Frame2D,
)
from mecheye.area_scan_3d_camera_utils import find_and_connect

from openvla_client import OpenVLAClient


# -----------------------------------------------------------------------------
# Mech-Eye camera helper
# -----------------------------------------------------------------------------


class MechEyeRGBSource:
    """Capture 2D RGB frames from a Mech-Eye area-scan camera."""

    def __init__(self, camera: Optional[Camera] = None) -> None:
        self._camera = camera or Camera()
        self._is_connected = False

    def connect(self) -> None:
        """Find and connect to the first available camera."""
        if self._is_connected:
            return
        if not find_and_connect(self._camera):
            raise RuntimeError("No Mech-Eye camera connected.")
        self._is_connected = True

    def disconnect(self) -> None:
        if self._is_connected:
            self._camera.disconnect()
            self._is_connected = False

    def capture_rgb(self) -> np.ndarray:
        """Grab a single RGB frame as an ndarray in RGB order."""
        if not self._is_connected:
            raise RuntimeError("Camera must be connected before capture.")

        frame = Frame2D()
        show_error(self._camera.capture_2d(frame))
        if frame.color_type() != ColorTypeOf2DCamera_Color:
            raise RuntimeError("The connected camera is not configured for color capture.")

        color_map = frame.get_color_image().data()  # OpenCV-style BGR ndarray
        # Ensure we have our own copy before reordering color channels.
        bgr_image = np.array(color_map, copy=True)
        # Convert to RGB for OpenVLA consumption.
        rgb_image = bgr_image[..., ::-1]
        return rgb_image


class VideoRecorder:
    """Append frames to a video file using OpenCV."""

    def __init__(self, output_path: str, fps: float) -> None:
        self._output_path = output_path
        self._fps = fps
        self._writer: Optional[cv2.VideoWriter] = None

    def append(self, frame_bgr: np.ndarray) -> None:
        if frame_bgr.size == 0:
            return
        if self._writer is None:
            height, width = frame_bgr.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self._writer = cv2.VideoWriter(self._output_path, fourcc, self._fps, (width, height))
        self._writer.write(frame_bgr)

    def close(self) -> None:
        if self._writer is not None:
            self._writer.release()
            self._writer = None


# -----------------------------------------------------------------------------
# OpenVLA inference helper
# -----------------------------------------------------------------------------


@dataclass
class OpenVLAInferenceResult:
    action: np.ndarray
    raw_response: object
    request_duration_sec: Optional[float]
    inference_time_sec: Optional[float]
    network_time_sec: Optional[float]


class OpenVLAController:
    """Thin wrapper around OpenVLAClient for direct ndarray requests."""

    def __init__(self, server_url: str, target_size: Sequence[int] = (256, 256)) -> None:
        self._client = OpenVLAClient(server_url)
        self._target_size = tuple(target_size)

    def _resize(self, rgb_image: np.ndarray) -> np.ndarray:
        pil_image = Image.fromarray(rgb_image, mode="RGB")
        resample_attr = getattr(Image, "Resampling", Image)
        resized = pil_image.resize(self._target_size, resample_attr.LANCZOS)
        return np.asarray(resized, dtype=np.uint8)

    def infer(self, rgb_image: np.ndarray, instruction: str) -> OpenVLAInferenceResult:
        """Send a pre-captured RGB image through OpenVLA."""
        processed = self._resize(rgb_image)
        response = self._client.send_request(processed, instruction)
        action = self._client.parse_numpy_result(response)
        if action.shape[0] != 7:
            raise ValueError(f"Expected 7-element action, got shape {action.shape}")
        request_duration = getattr(self._client, "last_request_duration", None)
        inference_time = getattr(self._client, "last_inference_time", None)
        network_time = getattr(self._client, "last_network_overhead", None)
        return OpenVLAInferenceResult(
            action=action,
            raw_response=response,
            request_duration_sec=request_duration,
            inference_time_sec=inference_time,
            network_time_sec=network_time,
        )


# -----------------------------------------------------------------------------
# AUBO motion helper  
# -----------------------------------------------------------------------------


class AuboArmController:
    """Minimal Cartesian jog controller for an AUBO arm."""

    def __init__(
        self,
        robot_ip: str,
        robot_port: int = 30004,
        username: str = "aubo",
        password: str = "123456",
    ) -> None:
        self._ip = robot_ip
        self._port = robot_port
        self._username = username
        self._password = password
        self._client = pyaubo_sdk.RpcClient()
        self._robot_interface: Optional[object] = None

    def __enter__(self) -> "AuboArmController":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    def connect(self) -> None:
        """Connect and log in to the controller."""
        self._client.setRequestTimeout(1000)
        self._client.connect(self._ip, self._port)
        if not self._client.hasConnected():
            raise RuntimeError(f"Failed to connect to AUBO controller at {self._ip}:{self._port}")
        self._client.login(self._username, self._password)
        robot_name = self._client.getRobotNames()[0]
        self._robot_interface = self._client.getRobotInterface(robot_name)

    def disconnect(self) -> None:
        if self._client.hasConnected():
            try:
                self._client.logout()
            finally:
                self._client.disconnect()

    def _ensure_interface(self):
        if self._robot_interface is None:
            raise RuntimeError("Robot is not connected.")
        return self._robot_interface

    def get_tcp_pose(self) -> Sequence[float]:
        """Return the current TCP pose [x, y, z, rx, ry, rz]."""
        robot_interface = self._ensure_interface()
        return robot_interface.getRobotState().getTcpPose()

    def wait_for_motion_completion(self, timeout: float = 5.0) -> None:
        """Block until the current motion queue clears or timeout elapses."""
        robot_interface = self._ensure_interface()
        motion_control = robot_interface.getMotionControl()
        start_ts = time.time()

        exec_id = motion_control.getExecId()
        while exec_id == -1 and (time.time() - start_ts) < timeout:
            time.sleep(0.05)
            exec_id = motion_control.getExecId()

        while motion_control.getExecId() != -1:
            if (time.time() - start_ts) > timeout:
                raise TimeoutError("Timed out waiting for AUBO motion to finish.")
            time.sleep(0.05)

    def move_by_cartesian_delta(
        self,
        delta_action: Sequence[float],
        translational_speed: float,
        translational_acc: float,
    ) -> Sequence[float]:
        """Apply a Cartesian delta to the current TCP pose via moveLine."""
        if len(delta_action) < 6:
            raise ValueError("Cartesian delta must contain at least 6 floats.")
        robot_interface = self._ensure_interface()
        motion_control = robot_interface.getMotionControl()

        current_pose = list(self.get_tcp_pose())
        target_pose = [
            current_pose[0] + delta_action[0],
            current_pose[1] + delta_action[1],
            current_pose[2] + delta_action[2],
            current_pose[3] + delta_action[3],
            current_pose[4] + delta_action[4],
            current_pose[5] + delta_action[5],
        ]

        # Issue a straight-line move to the new pose.
        motion_control.moveLine(
            target_pose,
            translational_speed,
            translational_acc,
            0.0,
            0.0,
        )
        self.wait_for_motion_completion()
        return target_pose


# -----------------------------------------------------------------------------
# Orchestration
# -----------------------------------------------------------------------------


def run_pipeline(args: argparse.Namespace) -> None:
    camera_source = MechEyeRGBSource()
    arm_controller = AuboArmController(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        username=args.robot_user,
        password=args.robot_password,
    )
    vla_controller = OpenVLAController(
        server_url=args.server_url,
        target_size=(args.image_width, args.image_height),
    )
    video_path = args.video_path or f"mecheye_session_{time.strftime('%Y%m%d-%H%M%S')}.mp4"
    recorder = VideoRecorder(output_path=video_path, fps=args.video_fps)

    camera_source.connect()
    try:
        with arm_controller:
            cycle_index = 0
            try:
                while True:
                    cycle_index += 1
                    print(f"\n=== 循环 {cycle_index} 开始 ===")
                    rgb_image = camera_source.capture_rgb()
                    capture_complete_ts = time.time()

                    bgr_frame = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
                    recorder.append(bgr_frame)

                    inference = vla_controller.infer(rgb_image, args.instruction)
                    inference_received_ts = time.time()

                    delta = inference.action[:6]
                    gripper_signal = float(inference.action[6])

                    print(f"OpenVLA action: {inference.action.tolist()}")
                    if args.verbose:
                        print(f"Raw response: {inference.raw_response}")

                    new_pose = arm_controller.move_by_cartesian_delta(
                        delta_action=delta,
                        translational_speed=args.move_speed,
                        translational_acc=args.move_acc,
                    )
                    print(f"Commanded new TCP pose: {new_pose}")
                    print(f"Gripper signal (not executed): {gripper_signal}")

                    total_latency = inference_received_ts - capture_complete_ts
                    print(f"拍照结束到推理结果返回总耗时: {total_latency:.3f} 秒")

                    if inference.request_duration_sec is not None:
                        print(f"OpenVLA请求往返耗时: {inference.request_duration_sec:.3f} 秒")
                    if inference.inference_time_sec is not None:
                        print(f"OpenVLA推理耗时: {inference.inference_time_sec:.3f} 秒")
                    if inference.network_time_sec is not None:
                        print(f"网络传输耗时(估计): {inference.network_time_sec:.3f} 秒")
            except KeyboardInterrupt:
                print("\n检测到手动停止。保存录像并准备退出。")
    finally:
        recorder.close()
        camera_source.disconnect()
        print(f"录像已保存至: {video_path}")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture an image, call OpenVLA, and jog an AUBO arm accordingly.",
    )
    parser.add_argument("--instruction", required=True, help="OpenVLA instruction text.")
    parser.add_argument(
        "--server-url",
        default="http://localhost:8000",
        help="Base URL of the OpenVLA service (default: %(default)s).",
    )
    parser.add_argument("--image-width", type=int, default=256, help="Width sent to OpenVLA.")
    parser.add_argument("--image-height", type=int, default=256, help="Height sent to OpenVLA.")

    parser.add_argument("--robot-ip", default="127.0.0.1", help="AUBO controller IP.")
    parser.add_argument("--robot-port", type=int, default=30004, help="AUBO controller RPC port.")
    parser.add_argument("--robot-user", default="aubo", help="AUBO login user.")
    parser.add_argument("--robot-password", default="123456", help="AUBO login password.")
    parser.add_argument(
        "--move-speed",
        type=float,
        default=0.2,
        help="Linear speed for moveLine in m/s (default: %(default)s).",
    )
    parser.add_argument(
        "--move-acc",
        type=float,
        default=0.2,
        help="Linear acceleration for moveLine in m/s^2 (default: %(default)s).",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Dump the raw OpenVLA response.",
    )
    parser.add_argument(
        "--video-path",
        default=None,
        help="Output path for the recorded camera video (default: autogenerated).",
    )
    parser.add_argument(
        "--video-fps",
        type=float,
        default=5.0,
        help="Frames per second for the recorded video.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        run_pipeline(args)
    except Exception as exc:  # noqa: BLE001
        print(f"[ERROR] {exc}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

# python openvla_aubo_integration.py --instruction "把柠檬抓起来" --server-url http://100.64.51.46:8000 --robot-ip 192.168.100.200
