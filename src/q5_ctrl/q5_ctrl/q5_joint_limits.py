"""38 维模型契约（关节名 + 手册限位）。

抄自机器人侧部署的 q5_openpi_contract.py，不改一个字：这些名字就是
/joint_states 里的真名，也是 HybridJointCommand.joint_name 要填的值。
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

# wr1_controller 在 ACTIVE 态只吃**全 20 个身体关节**的消息（手册 §5.2 + 项目记录 + 实测）。
# 只发 14 臂会让未命令的承重关节（踝/膝/髋）在受载时报驱动错误：
# 2026-09-28 实测 —— 只发 14 臂、命令左肘做 0.02 rad 微位移，
# 结果 ankle_joint + right_elbow_pitch 双双 "Drive occured error"，整机进 ERROR。
# 所以发布必须带全这 20 个：目标关节给目标值，其余给**当前反馈值**（保持）。
BODY_EXTRA_JOINT_NAMES = (
    "ankle_joint", "knee_joint", "hip_joint",
    "waist_yaw_joint", "neck_yaw_joint", "neck_pitch_joint",
)
BODY_JOINT_NAMES = ARM_JOINT_NAMES + BODY_EXTRA_JOINT_NAMES

# 《Q5 Pro 产品使用手册 V1.0》第 4 页（2026-10-19 逐值核对）。6 个保持关节的限位。
# 旧值有 5 个与手册不符；实测 ep001 反馈 knee ≈ -0.21、hip ≈ +1.07，会被旧值
# knee [-0.09, 2.09] / hip [-2.09, 0.44] 误判为"反馈异常"并拒绝发布。
MANUAL_BODY_EXTRA_LIMITS = {
    "ankle_joint": (0.00, 1.61),
    "knee_joint": (-2.53, 0.52),
    "hip_joint": (-0.52, 1.57),
    "waist_yaw_joint": (-1.57, 1.57),
    "neck_yaw_joint": (-0.79, 0.79),
    "neck_pitch_joint": (-0.26, 0.70),
}

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

# 手部 24 关节官方**没有**给硬限位（手册缺口，见 docs/XHand/XHand机械限位研究.md）。
# 这里的范围不是"官方限位"，是**合理性闸门**：只用来挡住明显错误的目标
# （漏乘/单位搞错/整块错位），比实测限位宽得多，不冒充硬限位。
# 实测参考：index_bend 正向外展硬限位约 0.16 rad；拇指 rota 需要负值。
MANUAL_HAND_LIMITS = {
    name: (-1.75, 1.75) for name in HAND_JOINT_NAMES
}
# thumb_rota 系列明确需要负值（hand_server 曾在 0 处夹取导致拇指被顶错方向）。
HAND_LIMITS_ARE_ADVISORY = True

# 数据集 71 集首帧均值（手部置 0），来自 docs/数据样例/init_pose_actions.json。
DATASET_MEAN_POSE = (
    -0.1452, 0.2206, -0.1432, -1.9315, 0.0683, -0.5164, -0.1158,
    -0.0009, -0.6920, -0.2266, -1.9125, -1.0476, -0.0949, 0.3931,
) + (0.0,) * 24
