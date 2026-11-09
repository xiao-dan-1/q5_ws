#!/usr/bin/env python3
"""ctrl.launch.py -- 把 Q5 激活到 ACTIVE，让出命令话题给外部 ROS 话题遥控（q5_ctrl 模式）。

只做两件事：
  1. guard：查两条命令话题有没有别的发布者，有则 exit 2 —— 防"两路命令打架"
     （命令被覆盖且无任何报错，这条链路最危险的失效模式）。
  2. 激活：state=ACTIVE(4) 幂等跳过；state=READY(3) 走 ZERO → LIFT → activate。

刻意不起 MPC / 手套桥 / Vive / 相机（它们会抢命令话题）。

低层冷启动（dynamic_launch → ready）**不归本文件管**：机器人要先处于 READY/ACTIVE，
否则提示走 `recover_xiaodan.sh`。

用法：
    ros2 launch q5_bringup ctrl.launch.py
    ros2 launch q5_bringup ctrl.launch.py guard:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

# 环境由调用方负责（.bashrc 已 source ROS + 设 ROS_DOMAIN_ID=211 / rmw_cyclonedds_cpp），
# 本文件不再重复 source。

# 抢占检查：ctrl 模式下"有别的发布者在发命令" = 两路命令互相打架。
# 判据只用 ROS 图（topic info --verbose 的 PUBLISHER 段），不用 pgrep：
# 容器与宿主共享 PID 表，pgrep 会看到陈旧/跨容器的同名进程。
GUARD_CMD = """
bad=0
pub_names() {
  timeout 15 ros2 topic info "$1" --verbose 2>/dev/null | awk '/^Subscription count:/ {exit} /^Node name:/ {n=$3} /^Endpoint type: PUBLISHER/ {if (n != "") print n}' | grep -vxF -e joint_metric_wr1_command_subscriber -e joint_metric_command_subscriber -e simple_trajectory_server 2>/dev/null || true
}
for t in /wr1_controller/commands /hand_controller/commands; do
  o=$(pub_names "$t")
  if [ -n "$o" ]; then
    echo "[guard] 抢占者 $t: $(echo "$o" | paste -sd' ' -)"
    bad=1
  else
    echo "[guard] $t 干净（无其它发布者）"
  fi
done
if [ "$bad" = "1" ]; then
  echo "[guard] 结论: 有别的发布者在写命令话题，两路命令会互相覆盖。先停掉它再启 ctrl。"
  echo "[guard]   参考: /usr/bin/python /home/developer/teleop_client/q5_pub_client.py --type mpc --cmd stop"
  echo "[guard]         pkill -f hand_server"
  exit 2
fi
echo "[guard] 结论: 环境干净，可以激活"
"""

ACTIVATE_CMD = """
ST=$(timeout 8 ros2 topic echo /xbot_state --once --field state 2>/dev/null | head -1 | tr -d '\\r\\n')
case "$ST" in
  4) echo "[ctrl] 已在 ACTIVE(state=4)，跳过激活（幂等）。命令话题已就绪，直接发命令即可。"; exit 0 ;;
  3) echo "[ctrl] state=READY(3)，走 ZERO → LIFT → activate"
     ros2 action send_goal /simple_trajectory xbot_common_interfaces/action/SimpleTrajectory "{traj_type: 0, duration: 4.0}"
     ros2 action send_goal /simple_trajectory xbot_common_interfaces/action/SimpleTrajectory "{traj_type: 2, duration: 4.0}"
     ros2 service call /activate_service std_srvs/srv/Trigger '{}'
     echo "[ctrl] 已下发 ZERO → LIFT → activate" ;;
  *) echo "[ctrl] state='$ST'（非 READY/ACTIVE）—— 本文件只做激活、不做冷启动。" >&2
     echo "[ctrl]    先恢复底层: bash ~/xiaodan/recover_xiaodan.sh" >&2
     exit 3 ;;
esac
"""


def generate_launch_description():
    guard = IfCondition(LaunchConfiguration("guard"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "guard", default_value="true",
            description="激活前检查命令话题有无其它发布者（有则拒绝）"),
        LogInfo(msg="ctrl 模式：只做激活（READY → ZERO → LIFT → activate），刻意不起 MPC/遥操/相机"),
        TimerAction(period=2.0, actions=[
            ExecuteProcess(cmd=["bash", "-c", GUARD_CMD], name="ctrl_guard",
                           output="screen", condition=guard),
        ]),
        TimerAction(period=4.0, actions=[
            ExecuteProcess(cmd=["bash", "-c", ACTIVATE_CMD], name="ctrl_activate",
                           output="screen"),
        ]),
    ])
