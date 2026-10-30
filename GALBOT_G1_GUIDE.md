# Galbot G1 接入与测试使用说明

本文按本仓库当前代码说明 Galbot G1 的配置、观测、控制，以及两个实机测试脚本的使用方式。Galbot G1 通过 `galbot_sdk.g1` 接入 LeRobot；它与仓库中的 Unitree G1 是不同的机器人适配模块。

## 1. 相关文件与调用关系

| 文件 | 作用 |
| --- | --- |
| [`src/lerobot/robots/galbot_g1/config_galbot_g1.py`](src/lerobot/robots/galbot_g1/config_galbot_g1.py) | 定义 `GalbotG1Config`，注册配置类型 `galbot_g1` |
| [`src/lerobot/robots/galbot_g1/galbot_g1.py`](src/lerobot/robots/galbot_g1/galbot_g1.py) | 定义 `GalbotG1`，封装 SDK 连接、关节/末端控制和传感器读取 |
| [`src/lerobot/robots/galbot_g1/__init__.py`](src/lerobot/robots/galbot_g1/__init__.py) | 导出 `GalbotG1` 和 `GalbotG1Config` |
| [`src/lerobot/robots/__init__.py`](src/lerobot/robots/__init__.py) | 在机器人包入口导出这两个类 |
| [`src/lerobot/robots/utils.py`](src/lerobot/robots/utils.py) | 支持 `make_robot_from_config(config)` 创建 Galbot G1 实例 |
| [`src/test_galbot_g1_camera.py`](src/test_galbot_g1_camera.py) | 单次读取多路相机、深度、IMU、雷达、关节和夹爪，保存图像及数组 |
| [`src/test_galbot_g1_hardware.py`](src/test_galbot_g1_hardware.py) | 读取当前关节状态，执行指定关节姿态并调整两侧夹爪 |
| [`src/sensor/`](src/sensor/) | 传感器脚本的默认输出目录，也包含当前工作目录已有的采集样例 |

两个测试脚本都调用 `GalbotG1` 的封装接口，不直接操作 SDK。

## 2. 运行环境与安装

本仓库的 `pyproject.toml` 声明 **Python >= 3.12**。需要在能访问机器人且已按机器人要求配置好通信环境的主机上运行，并单独安装与机器人匹配的 Galbot SDK；SDK 不在本项目的 pip 依赖和 extras 中。适配代码注释以 SDK v1.9.0 API 为参考，具体安装包、平台和通信配置请以实际 SDK 环境为准。

在已经安装好 SDK 的 Python 环境中，安装本仓库：

```bash
cd /home/huahungy/galbot_lerobot
python -m pip install -e .
```

`numpy` 和 `opencv-python-headless` 已在项目依赖中；脚本只保存图像，不需要 OpenCV GUI。换到其他机器时，将命令中的项目路径替换为实际克隆位置。

先检查 SDK 和本地适配模块能否导入；以下命令不连接机器人：

```bash
python -c "from galbot_sdk.g1 import GalbotRobot, ControlStatus, G1JointGroup, JointCommand, SensorType; from lerobot.robots.galbot_g1 import GalbotG1, GalbotG1Config; print('imports OK')"
```

未安装 SDK 时，适配模块仍可被导入，但创建 `GalbotG1(config)` 会抛出 `ImportError`。

## 3. 配置项

```python
from lerobot.robots.galbot_g1 import GalbotG1, GalbotG1Config

config = GalbotG1Config(
    joint_groups=["left_arm", "right_arm"],
    camera_ids=["HEAD_LEFT_CAMERA", "HEAD_RIGHT_CAMERA"],
    enable_gripper=True,
    enable_imu=True,
)
```

| 配置项 | 默认值 | 当前实现中的作用 |
| --- | --- | --- |
| `joint_groups` | `["left_arm", "right_arm"]` | 决定读取及下发的关节组，以及默认姿态的拼接顺序 |
| `camera_ids` | `["HEAD_LEFT_CAMERA", "HEAD_RIGHT_CAMERA"]` | RGB 相机的 `SensorType` 枚举名称 |
| `surround_ids` | `[]` | 环视相机枚举名称，使用同一 RGB 读取路径 |
| `depth_ids` | `[]` | 深度相机枚举名称 |
| `enable_gripper` | `True` | 将两侧夹爪纳入观测及动作特征声明 |
| `enable_suction_cup` | `False` | 将两侧吸盘纳入观测及动作特征声明 |
| `enable_imu` | `True` | 订阅并读取 `TORSO_IMU` |
| `enable_lidar` | `False` | 订阅并读取 `BASE_LIDAR` |
| `control_dt` | `0.02` | 保存为控制周期参数，对应名义 50 Hz；不会自动启动或限速控制循环 |
| `default_positions` | `[]` | 为空时在连接后记录当前姿态，供 `reset()` 使用 |
| `enable_head` / `enable_leg` | `False` / `False` | 目前没有参与控制分支；要读写头部、腿部，应将 `"head"`、`"leg"` 加入 `joint_groups` |
| `image_format` | `"jpeg"` | 当前未用于设置 SDK 编码或输出扩展名 |

此外继承 `RobotConfig` 的 `id` 和 `calibration_dir`。相机名称通过 `getattr(SensorType, name, None)` 查找，不存在的名称会被跳过。

## 4. 连接、读取和释放

最小只读示例：

```python
from lerobot.robots.galbot_g1 import GalbotG1, GalbotG1Config

config = GalbotG1Config(
    joint_groups=["left_arm", "right_arm"],
    camera_ids=[],
    enable_gripper=True,
    enable_imu=False,
)

with GalbotG1(config) as robot:
    obs = robot.get_observation()
    print("left_arm:", obs.get("left_arm.q"))
    print("right_arm:", obs.get("right_arm.q"))
    print("left_gripper:", obs.get("left_gripper.q"))
```

`connect()` 创建 SDK 实例，根据配置构造传感器集合，调用 `init(sensor_set)`，等待约 3 秒后发现关节并记录默认姿态。普通连接和读取不调用运动接口。`disconnect()` 依次调用 SDK 的 `request_shutdown()`、`wait_for_shutdown()` 和 `destroy()`。`with` 代码块退出时自动断开；手动连接时应使用 `try/finally` 释放。

可用 `get_joint_names(group)` 查询某组关节名，用 `get_joint_group_names()` 查看发现的组。关节位置实际按组通过 `get_joint_positions([group], [])` 读取，不依赖全量数组的切片；不要根据发现的全量名字直接假设全量位置数组的顺序和长度。

### 观测字典

| 键 | 数据类型 / 含义 |
| --- | --- |
| `left_arm.q`、`right_arm.q` | 关节位置列表，正常情况下各 7 项，单位 rad |
| `head.q`、`leg.q` | 配置对应组后返回，正常情况下分别 2、5 项，单位 rad |
| `left_gripper.q`、`right_gripper.q` | 夹爪位置，脚本按开口宽度 m 使用 |
| `left_suction_cup.active`、`right_suction_cup.active` | 吸盘状态 `bool` |
| `imu.gyro.x/y/z`、`imu.accel.x/y/z` | 三轴陀螺仪及加速度值，直接取自 SDK |
| `camera.<SensorType名称>` | 成功解码时为 RGB 顺序的 `numpy.ndarray`，形状 `(H, W, 3)` |
| `depth.<SensorType名称>` | 按 SDK 宽高重排的 `uint16` 深度数组 `(H, W)`，代码未做距离单位换算 |
| `lidar` | SDK 原始字节转成的一维 `uint8` 数组，尚未解析为点云坐标 |

RGB 使用 OpenCV 解码，再从 BGR 转为 RGB；如果解码失败，当前实现返回一维原始字节数组。各路传感器顺序读取，没有在本层实现时间同步。

`get_observation()` 对多数读取异常直接跳过，因此返回字典可能缺少某些键。未连接时返回 `{}`。读取时应判断键存在、数组形状和关节数，不能把缺失项默认当作零值。

## 5. 动作和便捷接口

| 接口 | 用法与行为 |
| --- | --- |
| `send_action(action)` | 按组将位置转换为 `JointCommand`，调用 `set_joint_commands`；也可下发夹爪、吸盘指令 |
| `move_to_positions(groups, positions_flat, max_speed=0.1, timeout=20.0)` | 阻塞式绝对关节运动，默认最大关节速度 0.1 rad/s、超时 20 秒，返回 SDK 状态 |
| `move_gripper(name, width, speed=0.05, torque=10.0)` | 阻塞式夹爪控制，名称为 `left_gripper` 或 `right_gripper`，宽度按 m 使用，返回 SDK 状态 |
| `reset()` | 阻塞回到 `default_positions`，内部使用速度 0.1、超时 20 秒 |
| `get_gripper_state(name)` | 返回 `position`、`velocity`、`effort`、`is_moving` 字典，失败时返回 `None` |
| `get_device_info()` / `get_battery_info()` | 返回 SDK 设备信息 / 电池信息，失败时返回 `None` |

动作字典的关节键与观测键一致，例如 `left_arm.q` 的值应为 7 个绝对位置；夹爪使用 `left_gripper.q`，可附带 `left_gripper.speed` 和 `left_gripper.torque`；吸盘使用 `left_suction_cup.activate`。右侧键同理。

以下函数展示调用方式，只有被调用时才发送动作；`target` 必须来自已审核的目标姿态：

```python
def send_left_arm_target(robot, target):
    if not robot.is_connected:
        raise RuntimeError("Robot is not connected")
    if len(target) != 7:
        raise ValueError("Expected 7 left-arm joint positions")
    return robot.send_action({"left_arm.q": list(target)})
```

需要连续控制时，由调用方按 `control_dt` 调度循环。当前 `send_action()` 会原样返回输入字典，并将下发失败写到 DEBUG 日志，返回动作不代表执行成功；需要 SDK 返回状态时可使用阻塞便捷接口，并读取实际位置确认结果。

`reset()` 的默认姿态通常是本次连接时的姿态，不是固定零位。显式设置 `default_positions` 时，长度和顺序必须与 `joint_groups` 一致。

## 6. 传感器测试：`src/test_galbot_g1_camera.py`

在项目根目录运行：

```bash
python src/test_galbot_g1_camera.py
```

脚本启用双臂关节、两侧夹爪、IMU、雷达，以及以下相机：

| 分类 | `SensorType` 名称 | 输出 |
| --- | --- | --- |
| 头部 RGB | `HEAD_LEFT_CAMERA`、`HEAD_RIGHT_CAMERA` | 每路一个同名 `.jpg` |
| 手臂 RGB | `LEFT_ARM_CAMERA`、`RIGHT_ARM_CAMERA` | 每路一个同名 `.jpg` |
| 环视 RGB | `LEFT_FRONT_SURROUND_CAMERA`、`RIGHT_FRONT_SURROUND_CAMERA`、`LEFT_REAR_SURROUND_CAMERA`、`RIGHT_REAR_SURROUND_CAMERA` | 每路一个同名 `.jpg` |
| 手臂深度 | `LEFT_ARM_DEPTH_CAMERA`、`RIGHT_ARM_DEPTH_CAMERA` | 每路一个同名 `.npy` |
| 雷达 | `BASE_LIDAR` | `BASE_LIDAR.npy` |

脚本连接后只调用一次 `get_observation()`，将读到的 RGB 从 RGB 转回 BGR 后保存为 JPG，将深度和雷达保存为 NumPy 文件，并在终端打印 IMU、关节数量和夹爪位置。它不发送运动命令，也不是视频录制或连续采集程序。

输出路径根据脚本位置确定，固定为 **`src/sensor/`**，与运行时工作目录无关。数据齐全时通常得到 **8 张 JPG + 2 个深度 NPY + 1 个雷达 NPY，共 11 个文件**；缺少数据时输出更少。再次运行会覆盖同名文件，运行前如需保留样例应先备份。

查看保存的数组：

```python
import numpy as np

depth = np.load("src/sensor/LEFT_ARM_DEPTH_CAMERA.npy", allow_pickle=False)
lidar = np.load("src/sensor/BASE_LIDAR.npy", allow_pickle=False)
print("depth:", depth.shape, depth.dtype)
print("lidar raw bytes:", lidar.shape, lidar.dtype)
```

本次整理时，目录已有的两个深度样例形状为 `(720, 1280)`、类型为 `uint16`，雷达样例为 `(519168,)`、类型为 `uint8`。这些是已有文件的数据，不代表每次采集或其他设备的固定规格。

## 7. 硬件运动测试：`src/test_galbot_g1_hardware.py`

**该脚本会实际移动双臂、头部、腿部和夹爪。运行前必须检查脚本内目标姿态与当前机器人状态，确认活动区域和急停条件。**

```bash
python src/test_galbot_g1_hardware.py
```

脚本没有命令行参数，也没有运行时确认提示；修改目标需要编辑源码中的 `TARGET[...]` 和 `GRIPPER_TARGET`。

### 关节顺序和当前目标

`GROUPS = ["left_arm", "right_arm", "head", "leg"]`。脚本依次拼接各组观测值，预期共 21 个关节，然后用下面的绝对位置覆盖 `TARGET`，单位 rad：

| 组 | `TARGET` 索引 | 当前写入的目标 |
| --- | --- | --- |
| 左臂 | `0`～`6` | `[0.76, -1.1501, -0.6435, -1.4638, -0.1892, -0.0249, -0.1367]` |
| 右臂 | `7`～`13` | `[-0.3352, 1.5606, 0.5287, 1.4717, 0.1220, 0.0176, 0.0157]` |
| 头部 | `14`～`15` | `[0.0, 0.0]` |
| 腿部 | `16`～`20` | `[0.4, 0.8, 0.6, 0.0, 0.0]`，脚本注释顺序为 pitch_down、pitch_middle、pitch_up、yaw、roll |

夹爪目标为：

```python
GRIPPER_TARGET = {"left_gripper": 0.04, "right_gripper": 0.04}
```

即两侧各 0.04 m（4 cm）。这些数值是脚本当前硬编码目标，不是适用于任意初始姿态的通用推荐姿态。

### 执行过程

1. 连接机器人，读取并打印当前 21 个关节的位置。
2. 将上述硬编码目标写入 `TARGET`，比较与当前位置的差值。
3. 若至少一个关节偏差大于 `2e-3` rad，调用 `move_to_positions(GROUPS, TARGET)`，最多尝试 3 次；返回状态名称为 `SUCCESS` 时停止重试。全部偏差不超过该值时跳过关节运动。
4. 分别读取两个夹爪。当前宽度与目标偏差小于 `0.002` m 时跳过，否则调用 `move_gripper()`。读取不到夹爪位置时也会尝试下发目标。
5. 在 `finally` 中断开连接。脚本不会自动调用 `reset()` 回到起始姿态。

脚本保留默认的头部双目相机和 IMU 配置，因此它也会订阅这些传感器。若只测试运动，可将配置显式设为 `camera_ids=[]`、`enable_imu=False`。

运行前要确认四组关节数据分别为 7、7、2、5 项。当前脚本没有完整性校验；数据缺失会导致索引错误或目标对应错误。脚本捕获异常后只打印 traceback，运动失败也可能继续夹爪步骤，所以应查看返回状态并复查实际位置，不能只根据进程退出码判断成功。

## 8. 当前实现边界和排查

| 现象或需求 | 检查方法 / 当前边界 |
| --- | --- |
| `ModuleNotFoundError: lerobot` | 在仓库根目录用当前环境的 `python -m pip install -e .` 安装 |
| `galbot_sdk is required` 或 SDK 导入失败 | 确认 SDK 安装在同一 Python 环境，且所需的五个 SDK 类型均能导入 |
| `GalbotRobot.init() failed` | 检查机器人在线状态、SDK 通信环境及当前订阅的传感器是否可用 |
| 相机文件缺失、观测缺键 | 核对枚举名称和传感器状态；当前脚本只读一帧，可在自定义采集中加入等待或重试 |
| 需要诊断下发失败 | 启用 `logging.basicConfig(level=logging.DEBUG)`，检查控制状态并读取实际关节位置 |
| 深度或雷达后处理 | 深度单位需按 SDK 数据定义解释；雷达 NPY 仅含原始字节，需要另行按 SDK 格式解码 |
| 标定 | `is_calibrated` 固定返回 `True`，`calibrate()`、`configure()` 是空实现，不能代替机器人本身的标定流程 |
| 动作约束 | 适配层没有实现关节限位、碰撞检测或目标长度检查；夹爪/吸盘使能标记也不会阻止显式调用末端控制接口或在 `send_action()` 中传入对应键 |
| 直接运行 LeRobot 录制或策略流程 | 当前特征包含 `list[float]` 和 `np.ndarray` 类型声明，尚未提供逐关节标量及固定图像尺寸的完整数据映射；不能据此假设所有标准 CLI 已直接兼容，接入时需核对数据特征及处理器 |

这两个 `test_*.py` 是手动执行的实机脚本，不是带断言的自动化单元测试。建议先完成只读连接和传感器检查，再审核硬件脚本的目标姿态并进行运动测试。
