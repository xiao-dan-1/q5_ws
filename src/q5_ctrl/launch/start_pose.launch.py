"""摆到采集起始位姿 = 一条命令，不多不少。

    ros2 launch q5_ctrl start_pose.launch.py

顺序固定：先 activate，再 MPC，最后才是 ABC 那两个节点。

    t=4s   activate  已 ACTIVE 就跳过，否则 /activate_service -> ACTIVE（不激活机器人不动）
    t=6s   mpc       q5_pub_client.py --type mpc --cmd start（/get_pose 与 /servo_poses 都归它）
    t=11s  A. q5_ctrl/vive_static_pose_node        常数 Vive 位姿，让增量恒为 0
           B. vive_dual_arm_teleop/vive_teleop_node
    t=15s  C. ros2 param set ... teleop_enabled true（是"后来发"，不是启动参数）

teleop_base 里的 check / camera_guard / joint_states_relay 没抄进来：
电池诊断、相机、跨机 60Hz 中继都与起始位姿无关，还会把启动拖到 18s。

延时不是随便定的——3~6s 是等 DDS 发现完再发服务调用，11s 是等 MPC start 那句回读跑完，
都照抄 teleop_base 的实测值。即使提前起，vive_teleop_node 也会等 /get_pose 和两路
Vive 位姿就绪后才 _enable_locked()，没就绪只是周期性告警。

**爬升速度要管**：启用后指令不是一步跳到位，而是每拍最多走 hand_lock_max_pos_step_m /
head_max_pos_step_m（旋转同理）。厂商默认 0.003 m / 0.017 rad，乘 90 Hz 得 0.27 m/s 和
1.53 rad/s，摆位姿时看着就是猛冲。下面四个参数把它压到 1/6。

到没到位用眼睛看。本 launch 不会停 MPC，也不会 deactivate；Ctrl-C 只停自己起的三个进程。
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node

Q5_PUB = "/home/developer/teleop_client/q5_pub_client.py"

# 幂等：已经 ACTIVE 就不要再调 /activate_service —— 状态机不允许 ACTIVE -> ACTIVATE，
# 会回 success=False "can not transite from state: ACTIVE to ACTIVATE"（看着像失败，其实没事）
ACT_CMD = """
S=$(timeout 5 ros2 service call /query_xbot_state xbot_common_interfaces/srv/QueryState {} 2>/dev/null \
     | grep -oE "state=[0-9]+" | head -1 | cut -d= -f2)
if [ "$S" = "4" ]; then
  echo "  已在 ACTIVE（state=4），跳过 activate"
else
  echo "  当前 state=${S:-未知}，调 /activate_service"
  ros2 service call /activate_service std_srvs/srv/Trigger {}
fi
"""

# 抄自 q5_bringup/teleop_base.launch.py：启动后显式回读一次，MPC 没起来下游全是空转
MPC_CMD = f"""
/usr/bin/python {Q5_PUB} --type mpc --cmd start 2>&1 | tail -1
sleep 3
echo "  MPC 状态（1=已启动）:"
/usr/bin/python {Q5_PUB} --type mpc --cmd query 2>&1 | tail -1
"""


def generate_launch_description():
    # 爬升速度 = 每拍步长 x control_rate_hz(90)。厂商默认 0.003 m / 0.017 rad 每拍
    # = 0.27 m/s、1.53 rad/s（头 0.8s 转 70°）——2026-10-20 实测现场看着就是"猛"。
    # 这里降到 1/6：0.05 m/s、0.27 rad/s。嫌慢就调大这四个数。
    slow = {
        "hand_lock_max_pos_step_m": 0.0005,
        "hand_lock_max_rot_step_rad": 0.003,
        "head_max_pos_step_m": 0.0005,
        "head_max_rot_step_rad": 0.003,
    }
    plan = [
        (4.0, ExecuteProcess(
            cmd=["bash", "-c", ACT_CMD], name="activate", output="screen")),
        (6.0, ExecuteProcess(
            cmd=["bash", "-c", MPC_CMD], name="mpc", output="screen")),
        (11.0, [
            Node(package="q5_ctrl", executable="vive_static_pose_node", output="screen"),
            # teleop_enabled 不用启动参数给 —— C 步照抄手动流程的 `ros2 param set`。
            # 这是本 launch 与"手动 ABC 正常"那次之间唯一的差别，先把它消掉。
            Node(
                package="vive_dual_arm_teleop",
                executable="vive_teleop_node",
                # 2026-10-20 现场对比用：先跑厂商原速。要降速把这行换回 parameters=[slow]
                # parameters=[slow],
                output="screen",
            ),
        ]),
        # 必须是后来发 param set，不能写成启动参数 teleop_enabled=true：
        # 2026-10-20 实测启动参数那次机器人猛冲，改成这种方式后正常。
        (15.0, ExecuteProcess(
            cmd=["bash", "-c",
                 "ros2 param set /vive_dual_arm_teleop_node teleop_enabled true"],
            name="teleop_on", output="screen")),
    ]

    return LaunchDescription(
        [
            RegisterEventHandler(
                OnShutdown(on_shutdown=[
                    LogInfo(msg="[提醒] start_pose 正在退出，本 launch 不会自动关闭 MPC。"
                                "如已启动 MPC，请在机器人端手动执行："
                                "bash ~/xiaodan/ops/mpc.sh stop；"
                                "再执行 bash ~/xiaodan/ops/mpc.sh status 确认状态为 0。"),
                ])
            ),
            LogInfo(msg="start_pose：activate -> MPC -> vive_static + teleop(locked)"),
        ]
        + [
            TimerAction(period=delay, actions=actions if isinstance(actions, list) else [actions])
            for delay, actions in plan
        ]
    )
