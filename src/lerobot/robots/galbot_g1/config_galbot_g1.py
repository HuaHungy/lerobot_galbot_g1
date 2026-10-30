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

from dataclasses import dataclass, field

from ..config import RobotConfig


@RobotConfig.register_subclass("galbot_g1")
@dataclass
class GalbotG1Config(RobotConfig):
    """Configuration for Galbot G1 humanoid robot via Galbot SDK.

    Attributes:
        joint_groups: List of joint groups to control (e.g. ["left_arm", "right_arm", "head"]).
        camera_ids: List of SensorType camera names to enable for video streaming
            (e.g. ["HEAD_LEFT_CAMERA", "HEAD_RIGHT_CAMERA"]).
        enable_gripper: Whether to include gripper control in action/observation space.
        enable_suction_cup: Whether to include suction cup control in action/observation space.
        enable_leg: Whether to include leg joint control.
        enable_head: Whether to include head joint control.
        enable_imu: Whether to include IMU data in observations.
        control_dt: Control loop timestep in seconds.
        default_positions: Default joint positions for reset.
    """

    # Joint groups to control
    joint_groups: list[str] = field(
        default_factory=lambda: ["left_arm", "right_arm"]
    )

    # Camera SensorType IDs to enable (RGB)
    camera_ids: list[str] = field(
        default_factory=lambda: ["HEAD_LEFT_CAMERA", "HEAD_RIGHT_CAMERA"]
    )

    # Depth camera IDs
    depth_ids: list[str] = field(default_factory=list)

    # Surround camera IDs
    surround_ids: list[str] = field(default_factory=list)

    # End-effector control flags
    enable_gripper: bool = True
    enable_suction_cup: bool = False

    # Body part control flags
    enable_leg: bool = False
    enable_head: bool = False

    # Sensor flags
    enable_imu: bool = True
    enable_lidar: bool = False

    # Control loop timestep in seconds (default 50Hz)
    control_dt: float = 0.02

    # Default joint positions (populated dynamically from robot state on connect)
    default_positions: list[float] = field(default_factory=list)

    # Image format: "jpeg" or "png"
    image_format: str = "jpeg"
