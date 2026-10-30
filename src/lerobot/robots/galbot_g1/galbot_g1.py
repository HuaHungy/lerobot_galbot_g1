#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Galbot G1 humanoid robot implementation via Galbot SDK.

Based on the official Galbot SDK v1.9.0 Python API reference:
  https://developer.galbot.com/docs/SDK/1.9.0/g1/zh/api_python_reference
  https://developer.galbot.com/docs/SDK/1.9.0/g1/zh/examples_python
  https://developer.galbot.com/docs/SDK/1.9.0/g1/zh/tutorials

Usage:
    from lerobot.robots.galbot_g1 import GalbotG1, GalbotG1Config

    config = GalbotG1Config(
        joint_groups=["left_arm", "right_arm"],
        camera_ids=["HEAD_LEFT_CAMERA", "HEAD_RIGHT_CAMERA"],
    )
    with GalbotG1(config) as robot:
        obs = robot.get_observation()
        robot.send_action(action)
"""

from __future__ import annotations

import logging
import time
from collections import OrderedDict
from functools import cached_property
from typing import TYPE_CHECKING

import numpy as np

from lerobot.types import RobotAction, RobotObservation

from ..robot import Robot
from .config_galbot_g1 import GalbotG1Config

if TYPE_CHECKING:
    from galbot_sdk.g1 import (
        ControlStatus as _ControlStatus,
        GalbotRobot as _GalbotRobot,
        G1JointGroup as _G1JointGroup,
    )

_GALBOT_SDK_AVAILABLE = False
try:
    from galbot_sdk.g1 import (
        ControlStatus as _ControlStatus,
        G1JointGroup as _G1JointGroup,
        GalbotRobot as _GalbotRobot,
        JointCommand as _JointCommand,
        SensorType as _SensorType,
    )

    _GALBOT_SDK_AVAILABLE = True
except ImportError:
    _GalbotRobot = None
    _ControlStatus = None
    _G1JointGroup = None
    _JointCommand = None
    _SensorType = None

logger = logging.getLogger(__name__)

_GROUP_OBS_KEY = {
    "head": "head",
    "left_arm": "left_arm",
    "right_arm": "right_arm",
    "leg": "leg",
}

# Official joint counts per group: https://developer.galbot.com/docs/SDK/1.9.0/g1/zh/examples_python
_KNOWN_JOINT_COUNTS = {
    "head": 2,
    "leg": 5,
    "left_arm": 7,
    "right_arm": 7,
    "left_gripper": 1,
    "right_gripper": 1,
    "left_suction_cup": 1,
    "right_suction_cup": 1,
}

# All-joint order (G1 default): chassis, head, left_arm, right_arm, leg
# The chassis group is not individually controllable but its joints appear in the flat list.
_CHASSIS_JOINT_COUNT = 7


class GalbotG1(Robot):
    """LeRobot interface for Galbot G1 humanoid robot via official Galbot SDK."""

    config_class = GalbotG1Config
    name = "galbot_g1"

    def __init__(self, config: GalbotG1Config):
        super().__init__(config)

        if not _GALBOT_SDK_AVAILABLE:
            raise ImportError(
                "galbot_sdk is required. Install from: "
                "https://github.com/GalaxyGeneralRobotics/GalbotSDK"
            )

        self.config = config
        self.control_dt = config.control_dt

        self._sdk: _GalbotRobot | None = None
        self._connected = False

        # Cached joint info discovered at connect time
        self._joint_names_all: list[str] = []
        self._joint_names_map: dict[str, list[str]] = {}
        # Mapping from group -> slice into flat position array
        self._joint_slices: dict[str, tuple[int, int]] = {}

    # --- Required Properties ---

    @cached_property
    def observation_features(self) -> dict:
        features: dict = OrderedDict()
        for group in self.config.joint_groups:
            features[f"{_GROUP_OBS_KEY.get(group, group)}.q"] = list[float]
        if self.config.enable_gripper:
            features["left_gripper.q"] = float
            features["right_gripper.q"] = float
        if self.config.enable_suction_cup:
            features["left_suction_cup.active"] = bool
            features["right_suction_cup.active"] = bool
        if self.config.enable_imu:
            for axis in ("x", "y", "z"):
                features[f"imu.gyro.{axis}"] = float
                features[f"imu.accel.{axis}"] = float
        for cam_id in self.config.camera_ids + self.config.surround_ids:
            features[f"camera.{cam_id}"] = np.ndarray
        for cam_id in self.config.depth_ids:
            features[f"depth.{cam_id}"] = np.ndarray
        if self.config.enable_lidar:
            features["lidar"] = np.ndarray
        return features

    @cached_property
    def action_features(self) -> dict:
        features: dict = OrderedDict()
        for group in self.config.joint_groups:
            features[f"{_GROUP_OBS_KEY.get(group, group)}.q"] = list[float]
        if self.config.enable_gripper:
            for side in ("left", "right"):
                features[f"{side}_gripper.q"] = float
                features[f"{side}_gripper.speed"] = float
                features[f"{side}_gripper.torque"] = float
        if self.config.enable_suction_cup:
            features["left_suction_cup.activate"] = bool
            features["right_suction_cup.activate"] = bool
        return features

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    # --- Lifecycle ---

    def connect(self, calibrate: bool = True) -> None:
        """Initialize SDK. Matches official get_instance.py example."""
        logger.info("Connecting to Galbot G1...")

        self._sdk = _GalbotRobot()
        # Build sensor set from config (required per official example4_sensor_data_collect.py)
        sensor_set = set()
        for cam_id in self.config.camera_ids:
            s = getattr(_SensorType, cam_id, None)
            if s is not None:
                sensor_set.add(s)
        for cam_id in self.config.surround_ids:
            s = getattr(_SensorType, cam_id, None)
            if s is not None:
                sensor_set.add(s)
        for cam_id in self.config.depth_ids:
            s = getattr(_SensorType, cam_id, None)
            if s is not None:
                sensor_set.add(s)
        if self.config.enable_imu:
            sensor_set.add(_SensorType.TORSO_IMU)
        if self.config.enable_lidar:
            sensor_set.add(_SensorType.BASE_LIDAR)

        if not self._sdk.init(sensor_set):
            raise RuntimeError("GalbotRobot.init() failed.")

        time.sleep(3)  # per official tutorials

        self._discover_joints()

        if not self.config.default_positions:
            self._record_default_positions()

        self._connected = True
        logger.info(
            f"Connected. Groups: {list(self._joint_names_map.keys())}, "
            f"total joints: {len(self._joint_names_all)}"
        )

    def disconnect(self) -> None:
        """Shutdown per official lifecycle."""
        if not self._connected:
            return
        logger.info("Disconnecting Galbot G1...")
        sdk = self._sdk
        if sdk is not None:
            try:
                sdk.request_shutdown()
                sdk.wait_for_shutdown()
                sdk.destroy()
            except Exception as e:
                logger.warning(f"Shutdown error: {e}")
        self._sdk = None
        self._connected = False
        logger.info("Disconnected.")

    # --- Joint Discovery ---

    def _discover_joints(self) -> None:
        """Discover joints and build group -> slice mapping.

        Uses get_joint_positions([], []) for all joints (25 total: 7 chassis + 2 head + 7+7 arms + 2 grippers)
        and get_joint_names(True, []) for names (23 total, excluding chassis).
        """
        sdk = self._sdk
        if sdk is None:
            return

        # Get all active joint names (per official tutorial Example 1)
        try:
            names = sdk.get_joint_names(True, [])
            if names:
                self._joint_names_all = list(names)
                logger.info(f"Active joints ({len(names)}): {names}")
        except Exception as e:
            logger.warning(f"get_joint_names failed: {e}")

        # Fallback: build from known groups
        if not self._joint_names_all:
            for group in self.config.joint_groups:
                count = _KNOWN_JOINT_COUNTS.get(group, 0)
                for i in range(1, count + 1):
                    self._joint_names_all.append(f"{group}_joint{i}")
            logger.warning(f"Using hardcoded joint names: {len(self._joint_names_all)} joints")

        # Map joint names to their groups
        for name in self._joint_names_all:
            for group in list(_KNOWN_JOINT_COUNTS.keys()):
                if name.startswith(f"{group}_joint"):
                    self._joint_names_map.setdefault(group, []).append(name)
                    break

        # Build slice mapping: flat position array → per-group slices
        # G1 default order: chassis(7) + head(2) + left_arm(7) + right_arm(7) + leg(5)
        #                   + left_gripper(1) + right_gripper(1) = 30? No...
        # Actually from testing: 25 positions for 23 named joints (2 extra = chassis-related)
        # We map by matching names in the total flat list
        all_names = self._joint_names_all
        offset = 0
        for group in self.config.joint_groups:
            group_names = self._joint_names_map.get(group, [])
            if group_names:
                start = all_names.index(group_names[0]) if group_names[0] in all_names else offset
                end = start + len(group_names)
                self._joint_slices[group] = (start, end)
                offset = end
                logger.info(f"  {group}: indices [{start}:{end}] -> {group_names}")
            else:
                self._joint_slices[group] = (0, 0)

    # --- Observation ---

    def get_observation(self) -> RobotObservation:
        if not self._connected or self._sdk is None:
            return {}

        obs: RobotObservation = {}

        # Joint positions — per-group (reliable), not flat slice
        for group in self.config.joint_groups:
            try:
                positions = self._sdk.get_joint_positions([group], [])
                if positions and len(positions) > 0:
                    obs[f"{_GROUP_OBS_KEY.get(group, group)}.q"] = list(positions)
            except Exception:
                pass

        # Gripper states
        if self.config.enable_gripper:
            for gripper in ("left_gripper", "right_gripper"):
                try:
                    state = self._sdk.get_gripper_state(gripper)
                    if state is not None:
                        pos = getattr(state, "position", None)
                        if pos is not None:
                            obs[f"{gripper}.q"] = float(pos)
                except Exception:
                    pass

        # Suction cup states
        if self.config.enable_suction_cup:
            for cup in ("left_suction_cup", "right_suction_cup"):
                try:
                    state = self._sdk.get_suction_cup_state(cup)
                    if state is not None:
                        obs[f"{cup}.active"] = bool(getattr(state, "is_active", False))
                except Exception:
                    pass

        # IMU — per official API: get_imu_data(SensorType.TORSO_IMU)
        if self.config.enable_imu:
            try:
                imu = self._sdk.get_imu_data(_SensorType.TORSO_IMU)
                if imu:
                    for key, prefix in (("gyro", "imu.gyro"), ("accel", "imu.accel")):
                        vec = imu.get(key, {})
                        if vec:
                            for axis in ("x", "y", "z"):
                                obs[f"{prefix}.{axis}"] = float(vec.get(axis, 0.0))
            except Exception:
                pass

        # Camera images — per official API: get_rgb_data(SensorType.XXX)
        for cam_id in self.config.camera_ids + self.config.surround_ids:
            try:
                sensor_type = getattr(_SensorType, cam_id, None)
                if sensor_type is None:
                    continue
                rgb = self._sdk.get_rgb_data(sensor_type)
                if rgb:
                    data = rgb.get("data")
                    if data is not None:
                        obs[f"camera.{cam_id}"] = self._decode_image(data)
            except Exception:
                pass

        # Depth images
        for cam_id in self.config.depth_ids:
            try:
                sensor_type = getattr(_SensorType, cam_id, None)
                if sensor_type is None:
                    continue
                depth = self._sdk.get_depth_data(sensor_type)
                if depth:
                    data = depth.get("data")
                    if data is not None:
                        obs[f"depth.{cam_id}"] = np.frombuffer(data, dtype=np.uint16).reshape(
                            (depth["height"], depth["width"])
                        )
            except Exception:
                pass

        # LiDAR
        if self.config.enable_lidar:
            try:
                lidar = self._sdk.get_lidar_data(_SensorType.BASE_LIDAR)
                if lidar and lidar.get("data"):
                    obs["lidar"] = np.frombuffer(lidar["data"], np.uint8)
            except Exception:
                pass

        return obs

    # --- Action ---

    def send_action(self, action: RobotAction) -> RobotAction:
        """Send action via set_joint_commands (high-frequency model inference).

        Per official example:
          https://developer.galbot.com/docs/SDK/1.9.0/g1/zh/examples_python#设置关节指令-(set_joint_commands)
        """
        if not self._connected or self._sdk is None:
            return action

        sdk = self._sdk

        # Joint commands — per official set_joint_commands_example.py
        for group in self.config.joint_groups:
            key = f"{_GROUP_OBS_KEY.get(group, group)}.q"
            if key in action and action[key] is not None:
                positions = action[key]
                if isinstance(positions, np.ndarray):
                    positions = positions.tolist()
                try:
                    cmds = []
                    for p in positions:
                        cmd = _JointCommand()
                        cmd.position = float(p)
                        cmds.append(cmd)
                    status = sdk.set_joint_commands(cmds, [group], [], 0.0)
                    if status != _ControlStatus.SUCCESS:
                        logger.debug(f"set_joint_commands({group}): {status}")
                except Exception as e:
                    logger.debug(f"set_joint_commands({group}) error: {e}")

        # Gripper — per official set_gripper_command.py
        for side in ("left", "right"):
            gripper_name = f"{side}_gripper"
            pos_key = f"{gripper_name}.q"
            if pos_key in action:
                try:
                    width = float(action[pos_key])
                    speed = float(action.get(f"{gripper_name}.speed", 0.05))
                    torque = float(action.get(f"{gripper_name}.torque", 10.0))
                    joint_group = getattr(_G1JointGroup, gripper_name)
                    sdk.set_gripper_command(joint_group, width, speed, torque, False)
                except Exception as e:
                    logger.debug(f"set_gripper_command({gripper_name}) error: {e}")

        # Suction cup — per official set_suction_cup_command.py
        for cup_name in ("left_suction_cup", "right_suction_cup"):
            key = f"{cup_name}.activate"
            if key in action:
                try:
                    activate = bool(action[key])
                    joint_group = getattr(_G1JointGroup, cup_name)
                    sdk.set_suction_cup_command(joint_group, activate)
                except Exception as e:
                    logger.debug(f"set_suction_cup_command({cup_name}) error: {e}")

        return action

    # --- Default positions ---

    def _record_default_positions(self) -> None:
        sdk = self._sdk
        if sdk is None:
            return
        defaults = []
        for group in self.config.joint_groups:
            try:
                positions = sdk.get_joint_positions([group], [])
                if positions:
                    defaults.extend(positions)
            except Exception:
                pass
        if not defaults:
            try:
                defaults = list(sdk.get_joint_positions([], []))
            except Exception:
                pass
        self.config.default_positions = defaults
        if defaults:
            logger.info(f"Default positions: {len(defaults)} joints")

    # --- Reset ---

    def reset(self) -> None:
        """Reset to default positions via set_joint_positions (blocking)."""
        if not self.config.default_positions:
            logger.warning("No default positions, reset skipped.")
            return

        sdk = self._sdk
        if sdk is None:
            return

        try:
            status = sdk.set_joint_positions(
                self.config.default_positions,
                self.config.joint_groups,
                [],
                True,
                0.1,
                20.0,
            )
            if status == _ControlStatus.SUCCESS:
                logger.info("Reset complete.")
            else:
                logger.warning(f"Reset returned {status}")
        except Exception as e:
            logger.warning(f"Reset error: {e}")

    # --- Image decoding ---

    @staticmethod
    def _decode_image(data: bytes) -> np.ndarray:
        """Decode JPEG/PNG to RGB numpy array (H, W, 3)."""
        import cv2

        arr = np.frombuffer(data, np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return arr

    # --- Convenience ---

    def move_to_positions(
        self,
        groups: list[str],
        positions_flat: list[float],
        max_speed: float = 0.1,
        timeout: float = 20.0,
    ) -> _ControlStatus | None:
        """Move robot to absolute joint positions (blocking, safe).

        Uses set_joint_positions per official tutorial Example 1.
        groups + positions_flat are passed directly to SDK.

        Args:
            groups: Joint group names, e.g. ["left_arm", "right_arm"]
            positions_flat: Flat position list matching groups order
            max_speed: Max joint speed in rad/s
            timeout: Blocking timeout in seconds

        Returns:
            ControlStatus or None if not connected
        """
        if not self._connected or self._sdk is None:
            return None
        return self._sdk.set_joint_positions(positions_flat, groups, [], True, max_speed, timeout)

    def move_gripper(
        self,
        gripper_name: str,
        width: float,
        speed: float = 0.05,
        torque: float = 10.0,
    ) -> _ControlStatus | None:
        """Move gripper to target width (blocking).

        Uses set_gripper_command per official example.
        """
        if not self._connected or self._sdk is None:
            return None
        g1_group = getattr(_G1JointGroup, gripper_name, None)
        if g1_group is None:
            raise ValueError(f"Unknown gripper: {gripper_name}")
        return self._sdk.set_gripper_command(g1_group, width, speed, torque, True)

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def get_joint_names(self, group: str | None = None) -> list[str]:
        if group:
            return self._joint_names_map.get(group, [])
        return self._joint_names_all

    def get_joint_group_names(self) -> list[str]:
        return list(self._joint_names_map.keys())

    def get_gripper_state(self, gripper_name: str) -> dict | None:
        if self._sdk is None:
            return None
        try:
            state = self._sdk.get_gripper_state(gripper_name)
            if state is not None:
                return {
                    "position": getattr(state, "position", None),
                    "velocity": getattr(state, "velocity", None),
                    "effort": getattr(state, "effort", None),
                    "is_moving": getattr(state, "is_moving", False),
                }
        except Exception:
            pass
        return None

    def get_device_info(self) -> dict | None:
        if self._sdk is None:
            return None
        try:
            return self._sdk.get_device_information()
        except Exception:
            return None

    def get_battery_info(self) -> dict | None:
        if self._sdk is None:
            return None
        try:
            return self._sdk.get_bms_information()
        except Exception:
            return None

    # --- Context Manager ---

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.disconnect()

    def __del__(self) -> None:
        try:
            if self._connected:
                self.disconnect()
        except Exception:
            pass
