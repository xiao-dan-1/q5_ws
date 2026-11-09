# q5_tools

机器人侧工具：`camera_guard`、`joint_states_relay`、`mjpeg_server`。

## MJPEG 浏览器图像服务

订阅已有的 ROS Image 话题，将画面通过 HTTP MJPEG 提供给浏览器。
启动前先保证相机节点运行，并使用与相机相同的 ROS 环境
（Q5 当前为 `ROS_DOMAIN_ID=211`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`）。

在 Q5 已配置 ROS 环境的交互终端中运行：

```bash
source /home/developer/xiaodan/q5_ws/install/setup.bash
ros2 run q5_tools mjpeg_server \
  -t /camera/camera/color/image_raw \
  -p 8111 --fps 30 --quality 60
```

浏览器打开启动日志中的地址，例如 `http://192.168.8.100:8111/`。
IP 自动通过 `ip` 命令获取。服务监听所有网卡，按 Ctrl+C 停止。
该命令只转发图像，不启动相机。

`teleop_base.launch.py` 现会在启动后第 12 秒自动启动网页服务，
使用 `/camera/camera/color/image_raw`、端口 8111、30fps、JPEG 质量 60。
`vive_teleop.launch.py` 和 `meta_teleop.launch.py` 都会包含该基础层，
因此正常启动遥操后直接打开日志中的网页地址即可，无需再手动启动 MJPEG。
网页服务随 launch 退出。

参数：

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `-t` / `--topic` | `/q5/head_camera/color/image_raw` | ROS Image 话题 |
| `-p` / `--port` | `8080` | HTTP 端口 |
| `--fps` | `10` | 目标推流帧率，实际新画面帧率受相机限制 |
| `--quality` | `60` | JPEG 质量，1–100 |

依赖：`rclpy`、`sensor_msgs`、`cv_bridge`、`python3-opencv`。

修改源码后构建：

```bash
cd /home/developer/xiaodan/q5_ws
colcon build --packages-select q5_tools
source install/setup.bash
```
