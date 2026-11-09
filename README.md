# Q5 ROS 2 工作区

RobotEra Q5 Pro 的 ROS 2 Humble 工作区，包含机器人控制、遥操、数据采集、策略推理和仿真。

## 功能包

| 功能包 | 用途 |
| --- | --- |
| `q5_bringup` | 真机启动编排，提供 Vive、Meta 遥操和推理观测入口 |
| [q5_ctrl](src/q5_ctrl/README.md) | 双臂 14 维、双手 24 维关节控制 |
| [q5_description](src/q5_description/README.md) | URDF、网格、RViz 和 Gazebo |
| `q5_datacollect` | 录制 rosbag 并转换为 LeRobot v2.1 数据集 |
| `q5_policy_runtime` | π0.5 策略推理试运行（dry-run） |
| `q5_hand_bridge`、`q5_zmq_bridge` | 手套 TCP 桥接、RoboDriver 观测与动作桥接 |
| [q5_tools](src/q5_tools/README.md) | 相机检查、MJPEG 服务和关节状态转发 |
| `xbot_sdk_api`、`xhand_control_ros2`、`xhand_control_interfaces` | 机器人 SDK 封装、XHand 驱动和消息接口 |

## 构建

在 Linux 或 WSL 的仓库根目录执行。涉及真机的包需要先加载提供 `xbot_common_interfaces` 的厂商 ROS 2 环境。

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src
source install/setup.bash
```

## 常用入口

```bash
ros2 launch q5_description display.launch.py       # RViz 模型预览
ros2 run q5_datacollect preflight                  # 录制前检查
ros2 launch q5_policy_runtime policy.launch.py     # 策略推理 dry-run
```

连接真机时使用 `ROS_DOMAIN_ID=211` 和 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。
