"""G1 全传感器 — 全走 robot 类，保存到 src/sensor/。"""

import os
import time

import cv2
import numpy as np
from lerobot.robots.galbot_g1 import GalbotG1, GalbotG1Config

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "sensor")
os.makedirs(OUTPUT_DIR, exist_ok=True)


def main():
    config = GalbotG1Config(
        joint_groups=["left_arm", "right_arm"],
        camera_ids=[
            "HEAD_LEFT_CAMERA", "HEAD_RIGHT_CAMERA",
            "LEFT_ARM_CAMERA", "RIGHT_ARM_CAMERA",
        ],
        surround_ids=[
            "LEFT_FRONT_SURROUND_CAMERA", "RIGHT_FRONT_SURROUND_CAMERA",
            "LEFT_REAR_SURROUND_CAMERA", "RIGHT_REAR_SURROUND_CAMERA",
        ],
        depth_ids=["LEFT_ARM_DEPTH_CAMERA", "RIGHT_ARM_DEPTH_CAMERA"],
        enable_imu=True,
        enable_lidar=True,
        enable_gripper=True,
    )

    with GalbotG1(config) as robot:
        obs = robot.get_observation()
        saved = 0

        # RGB + 环视
        for key, value in obs.items():
            if key.startswith("camera.") and isinstance(value, np.ndarray) and value.ndim == 3:
                name = key.replace("camera.", "")
                fname = f"{OUTPUT_DIR}/{name}.jpg"
                cv2.imwrite(fname, cv2.cvtColor(value, cv2.COLOR_RGB2BGR))
                saved += 1
                print(f"  ✅ {name}: {value.shape[1]}x{value.shape[0]}")

            elif key.startswith("depth.") and isinstance(value, np.ndarray):
                name = key.replace("depth.", "")
                np.save(f"{OUTPUT_DIR}/{name}.npy", value)
                saved += 1
                print(f"  ✅ {name} depth: {value.shape[1]}x{value.shape[0]}")

            elif key == "lidar":
                np.save(f"{OUTPUT_DIR}/BASE_LIDAR.npy", value)
                saved += 1
                print(f"  ✅ LiDAR: {len(value)} bytes")

        # IMU
        imu_keys = [k for k in obs if "imu" in k]
        if imu_keys:
            print(f"\n  ✅ IMU: { {k: f'{obs[k]:.4f}' for k in imu_keys} }")

        # 关节
        for g in config.joint_groups:
            k = f"{g}.q"
            if k in obs and obs[k]:
                print(f"  ✅ {g}: {len(obs[k])} joints")

        # 夹爪
        for g in ("left_gripper", "right_gripper"):
            k = f"{g}.q"
            if k in obs:
                print(f"  ✅ {g}: {obs[k]}")

        print(f"\n✅ 共保存 {saved} 个文件到 {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
