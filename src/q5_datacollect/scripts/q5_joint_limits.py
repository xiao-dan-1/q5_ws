"""38 维模型契约（关节名 + 手册限位 + 数据集均值姿势）—— 离线侧子集。

权威源：机器人 `~/q5-openpi-body-stage/q5_openpi_interface/q5_openpi_contract.py`
（16KB 全量版，含 build_state_vector / validate_action_chunk / 安全配置）。
本文件只保留离线工具需要的三样东西，名字与顺序**照抄不改一个字**：这些名字就是
`/joint_states` 里的真名，也是 `HybridJointCommand.joint_name` 要填的值。

给离线工具 import 用（放在同目录即可，靠"脚本所在目录在 sys.path 上"）：
    from q5_joint_limits import MODEL_JOINT_NAMES, ARM_JOINT_NAMES, HAND_JOINT_NAMES

顺序见 docs/接口约定/Q5关节顺序与38维映射.md：左臂 0-6 / 右臂 7-13 / 左手 14-25 / 右手 26-37。
硬规则：**按关节名取值，禁止 position[:38]**（/joint_states 的数组顺序是乱的）。
"""

MODEL_JOINT_NAMES = (
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_arm_yaw_joint",
    "left_elbow_pitch_joint", "left_elbow_yaw_joint", "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_arm_yaw_joint",
    "right_elbow_pitch_joint", "right_elbow_yaw_joint", "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
    "left_hand_thumb_bend_joint", "left_hand_thumb_rota_joint1", "left_hand_thumb_rota_joint2",
    "left_hand_index_bend_joint", "left_hand_index_joint1", "left_hand_index_joint2",
    "left_hand_mid_joint1", "left_hand_mid_joint2",
    "left_hand_ring_joint1", "left_hand_ring_joint2",
    "left_hand_pinky_joint1", "left_hand_pinky_joint2",
    "right_hand_thumb_bend_joint", "right_hand_thumb_rota_joint1", "right_hand_thumb_rota_joint2",
    "right_hand_index_bend_joint", "right_hand_index_joint1", "right_hand_index_joint2",
    "right_hand_mid_joint1", "right_hand_mid_joint2",
    "right_hand_ring_joint1", "right_hand_ring_joint2",
    "right_hand_pinky_joint1", "right_hand_pinky_joint2",
)

ARM_JOINT_NAMES = MODEL_JOINT_NAMES[:14]
HAND_JOINT_NAMES = MODEL_JOINT_NAMES[14:]

# 《Q5 Pro 产品使用手册》身体关节限位；手册未给 24 个手关节的硬限位。
MANUAL_ARM_LIMITS = {
    "left_shoulder_pitch_joint": (-2.79, 2.79), "left_shoulder_roll_joint": (-0.24, 1.83),
    "left_arm_yaw_joint": (-2.62, 2.62), "left_elbow_pitch_joint": (-2.27, 0.09),
    "left_elbow_yaw_joint": (-2.62, 2.62), "left_wrist_pitch_joint": (-1.05, 1.05),
    "left_wrist_roll_joint": (-1.05, 1.05),
    "right_shoulder_pitch_joint": (-2.79, 2.79), "right_shoulder_roll_joint": (-1.83, 0.24),
    "right_arm_yaw_joint": (-2.62, 2.62), "right_elbow_pitch_joint": (-2.27, 0.09),
    "right_elbow_yaw_joint": (-2.62, 2.62), "right_wrist_pitch_joint": (-1.05, 1.05),
    "right_wrist_roll_joint": (-1.05, 1.05),
}

# 数据集 71 集首帧均值（手部置 0），来自 docs/数据样例/init_pose_actions.json。
DATASET_MEAN_POSE = (
    -0.1452, 0.2206, -0.1432, -1.9315, 0.0683, -0.5164, -0.1158,
    -0.0009, -0.6920, -0.2266, -1.9125, -1.0476, -0.0949, 0.3931,
) + (0.0,) * 24
