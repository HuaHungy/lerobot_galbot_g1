"""G1 测试 — 完全通过 robot 类接口读写。"""

from lerobot.robots.galbot_g1 import GalbotG1, GalbotG1Config


def main():
    # ============================================================
    #  关节目标（绝对值，单位 rad）
    #  TARGET[0]~[6]=左臂  [7]~[13]=右臂  [14]~[15]=头部  [16]~[20]=腿部
    #  夹爪目标（单位 m）
    # ============================================================
    GRIPPER_TARGET = {"left_gripper": 0.04, "right_gripper": 0.04}

    GROUPS = ["left_arm", "right_arm", "head", "leg"]
    LABELS = (
        ["L_j" + str(j) for j in range(1, 8)]
        + ["R_j" + str(j) for j in range(1, 8)]
        + ["H_j" + str(j) for j in range(1, 3)]
        + ["leg_j" + str(j) for j in range(1, 6)]
    )

    config = GalbotG1Config(joint_groups=GROUPS, enable_gripper=True)
    robot = GalbotG1(config)

    try:
        robot.connect()
        print("✅ 连接成功\n")

        # ==== 1. 读取（通过 robot 类） ====
        obs = robot.get_observation()
        cur_flat = []
        for g in GROUPS:
            key = f"{g}.q"
            pos = obs.get(key, [])
            cur_flat.extend(pos)
        TARGET = list(cur_flat)

        print(f"当前位置 ({len(cur_flat)} joints):")
        for i, v in enumerate(cur_flat):
            label = LABELS[i] if i < len(LABELS) else f"[{i}]"
            print(f"  [{i:2d}] {label} = {v:+.4f}")

        # ==== 2. 修改目标 ----
        TARGET[0] = 0.76      # L1
        TARGET[1] = -1.1501   # L2
        TARGET[2] = -0.6435   # L3
        TARGET[3] = -1.4638   # L4
        TARGET[4] = -0.1892   # L5
        TARGET[5] = -0.0249   # L6
        TARGET[6] = -0.1367   # L7
        TARGET[7] = -0.3352   # R1
        TARGET[8] = 1.5606    # R2
        TARGET[9] = 0.5287    # R3
        TARGET[10] = 1.4717   # R4
        TARGET[11] = 0.1220   # R5
        TARGET[12] = 0.0176   # R6
        TARGET[13] = 0.0157   # R7
        TARGET[14] = 0.0      # H1
        TARGET[15] = 0.0      # H2
        TARGET[16] = 0.4      # pitch_down
        TARGET[17] = 0.8      # pitch_middle
        TARGET[18] = 0.6      # pitch_up
        TARGET[19] = 0.0      # yaw
        TARGET[20] = 0.0      # roll

        # ==== 3. 比较 ====
        changed = [i for i, (t, c) in enumerate(zip(TARGET, cur_flat)) if abs(t - c) > 2e-3]
        if not changed:
            print("\n(无变化，跳过)")
        else:
            print(f"\n变动关节: {changed}")
            print(f"目标:     {[f'{TARGET[i]:+.4f}' for i in changed]}")
            print(f"当前:     {[f'{cur_flat[i]:+.4f}' for i in changed]}")

            # ==== 4. 下发（通过 robot 类） ====
            for attempt in range(3):
                status = robot.move_to_positions(GROUPS, TARGET)
                print(f"  attempt {attempt+1}: {status}")
                if status is not None and status.name == "SUCCESS":
                    break

        # ==== 5. 夹爪（通过 robot 类） ====
        print("\n夹爪:")
        for name, tgt_w in GRIPPER_TARGET.items():
            state = robot.get_gripper_state(name)
            cur_w = state["position"] if state else None
            print(f"  {name}: 当前={cur_w}, 目标={tgt_w}")
            if cur_w is not None and abs(tgt_w - cur_w) < 0.002:
                print(f"         偏差小 -> 跳过")
                continue
            s = robot.move_gripper(name, tgt_w)
            print(f"         结果: {s}")

    except Exception as e:
        import traceback
        traceback.print_exc()
    finally:
        robot.disconnect()
        print("\n✅ 断开连接")


if __name__ == "__main__":
    main()
