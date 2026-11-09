# q5_description

RobotEra Q5 Pro 的描述包：URDF + 网格 + RViz 显示。和 `q5_ctrl`/`q5_observe` 一样放在
`q5_ws/src/` 下，`colcon build` 后即可用。

## 模型从哪来

模型本体在 Q5work 仓库里重建（`urdf/build_q5_reference.py`），这个包只做打包：

```bash
python3 scripts/import_model.py [Q5work 仓库根目录]
```

它把两份 URDF 复制进 `urdf/`，把网格路径从相对路径写成 `package://q5_description/meshes/...`，
并且**只复制真正被引用到的网格**。默认源目录是 WSL 下的
`/mnt/c/Users/jinbangguo/Desktop/xiaodan/home/06_projects/Q5work/urdf/robotsfan_q5`。

两份 URDF 的区别只有手：`q5_reference_xhand.urdf` 画 XHand 手指，
`q5_reference.urdf` 按官方手册/透明 CAD 图把前臂收在圆法兰上。关节链完全相同
（68 link / 67 joint / 46 可动），所以对 TF、控制器、逆解没有任何影响。

## 用

```bash
colcon build --packages-select q5_description
source install/setup.bash

# 只看模型/手动摆关节
ros2 launch q5_description display.launch.py
ros2 launch q5_description display.launch.py urdf:=q5_reference.urdf rviz:=false

# 只当描述源给别的节点用
ros2 launch q5_description description.launch.py
ros2 run q5_description check_description.py       # 自检：网格齐全、根 link 唯一
ros2 run q5_description check_tf.py                # 自检：TF 树连到手指/头/轮子

# Gazebo classic 里看模型（默认零重力世界，因为没有碰撞几何）
ros2 launch q5_description gazebo.launch.py
ros2 launch q5_description gazebo.launch.py gui:=false        # 无头
ros2 run q5_description capture_image.py --out /tmp/q5.png    # 无头时用世界里的机位相机截图
```

### 为什么默认 `static:=true`

这套 URDF 带官方质量与惯量（底盘 44.9 kg，指节惯量低到 1e-5 量级）。在 Gazebo
classic 的关节求解里这个量级组合会发散：实测模型进世界几帧内所有 link 被拉回原点，
屏幕上只剩一坨底盘。`static:=true` 会先用 `gz sdf -p` 把 URDF 转成 SDF 并插入
`<static>true</static>` 再 spawn，模型不进物理、按 URDF 零位姿渲染。代价是**关节不会动**；
要真的驱动它，需要先补碰撞体、把惯量调到能收敛、再加 ros2_control。
`static:=false` 走原来的动态 spawn 路径，留作对照。

### 卡顿

如果操作发涩，先看有没有 `LIBGL_ALWAYS_SOFTWARE=1` 在环境里（`~/q5_gazebo/start_q5_sim.sh`
会 export 它）。这台机器 WSLg 带 d3d12 GPU 驱动，硬件渲染下 gzclient 约 34% CPU；
强制软件渲染（llvmpipe）实测要 ~400% CPU，手感明显卡。`gazebo.launch.py` 默认会
**清掉**这个变量，需要软件渲染时用 `software_gl:=true`。

## 真的驱动起来（ros2_control）

```bash
ros2 launch q5_description gazebo_control.launch.py     # DART 世界 + ros2_control
```

另开一个终端（先 `source ~/q5_ws/install/setup.bash`）：

```bash
ros2 topic echo /joint_states --once                     # 看关节角

# 位置指令：38 个数，顺序 = config/q5_controllers.yaml 里
# q5_position_controller.joints（左臂 7 → 右臂 7 → 左手 9 → 右手 9 → 踝/膝/髋/腰/颈×2）
ros2 topic pub --rate 30 /q5_position_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0,0,0,0,0,0,0, 0,-1.2,-0.6,-1.4,0,0,0, 0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0, 0,0,0,0.6,0,0.5]}"

# 底盘前后走（rad/s，速度控制器）
ros2 topic pub --rate 10 /q5_wheel_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [2.0, -2.0]}"
```

实测：上面那条位置指令会把右臂抬到头部、肘弯起（对照 `tmp/vis_before.png` / `vis_after.png`）。

### 让它真的往前走

**零重力世界（`q5_control.world`）里走不了**：轮子会空转（实测 3 rad/s 下发后轮子转了
59.6 rad），但底盘位移只有 4e-06 m——没有法向力就没有摩擦，轮子推不动车身。

要前进得用有重力 + 地面摩擦的世界，并且模型要带碰撞体（`build_q5_reference.py` 现在会给
两个驱动轮和底盘裙板加简化碰撞）：

```bash
ros2 launch q5_description gazebo_control.launch.py \
    world:=$HOME/q5_ws/install/share/q5_description/worlds/q5_drive.world

ros2 topic pub --rate 20 /q5_wheel_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [3.0, 3.0]}"      # 两轮同向 = 前进
```

实测（`docs_driving_forward.png`）：落地稳定后 x=-0.0017 m，3 rad/s 走 4 秒 x=1.03 m，
再 4 秒 x=1.34 m。轮径 0.077 m × 3 rad/s = 0.23 m/s，实测 0.26 m/s → **纯滚动不打滑**。
同时踝/膝/髋被位置控制器锁在 0，车身保持站立（轮式倒立摆）。

倒车把两个数取负；原地转把两轮反向。

### 走不直、前轮翘起来（都是预期内的）

**前轮翘头**：把轮速从 0 阶跃到 3 rad/s，反力矩作用在高重心上就把前脚轮顶起来了。
用加速度限幅即可，`scripts/straight_drive.py` 默认 2 rad/s²：

```bash
ros2 run q5_description straight_drive.py --speed 0.25 --seconds 10
```

**走不直**：开环只给左右轮同速，接触/摩擦的左右不对称会让车慢慢转（实测 10 秒偏
0.34 m、转头 0.42 rad）。真机靠编码器 + **IMU** 闭环修正，仿真里同理：
`straight_drive.py` 用差速里程计 + 仿真位姿当 IMU，把航向闭到 0.063 rad、漂移 0.054 m。

注意编码器里程计**看不到打滑**：同一次测试里里程计算 1.63 m、真值只有 1.07 m（滑移
34%）。所以航向必须用 IMU 类传感器，光靠轮子转角会自我感觉良好地跑偏。

### 驱动相关的坑（都踩过）

1. **必须 DART**：`worlds/q5_control.world` 用 `type="dart"`。官方质量/惯量在 ODE 里关节求解发散，模型几帧内塌成一坨。
2. **一次只 spawn 一个带 `ros2_control` 的模型**：gazebo_ros2_control 的 ROS 节点名固定，第二个模型的插件会报
   `Found multiple nodes with same name: /gazebo_ros2_control` 而完全不起作用。
3. **关节名/顺序**：38 个位置关节沿用旧模型 `q5_gazebo/q5_controllers.yaml`（关节名与官方链一致，已逐个核对）。
   6 个拇指关节沿用旧结论先不给控制（惯量 1e-7 量级，容易振荡）。
4. **验证看图像，别只看 `/joint_states`**：环境里若有旧的 `joint_state_publisher`/历史仿真残留，
   `/joint_states` 会连到错的那个发布者，读数恒为 0，但模型其实在动。

`display.launch.py` 默认开关节滑块；没装 `joint_state_publisher_gui` 时会自动退回
无滑块的 `joint_state_publisher`，不会启动失败。

## 边界与已知缺口

- 关节链、关节限位、机身惯量（32 个机身 link + 26 个手部 link）来自官方主机 XML
  `q5_official_host.xml`；**外壳是按手册图与透明 CAD 视图做的近似，不是原厂 CAD**，
  精度只到"同一相机下轮廓 RMS"这个意义（前臂 0.17 px、小腿 0.83 px 等）。
- **没有碰撞几何**：`<collision>` 数量为 0，所以 Gazebo 里不会碰撞，`fit_collision.py`
  那类工作还没做。旧模型的简化碰撞在 Q5work 的 `q5_pro_collision.urdf` 里，不能直接
  套到这套外壳上。
- **没有 ros2_control / transmission 标签**，也没有控制器配置，因此还不能直接
  在仿真里驱动。`~/q5_gazebo/q5_controllers.yaml` 那套是给旧 `q5_pro` 模型写的。
- 手部网格是第三方 XHand 参考资源（39 个 STL 中 17 个各带 1 条非流形边，属原始资源），
  许可见 Q5work 的 `urdf/XHAND_THIRD_PARTY_NOTICE.md`，不要再分发。
- 机器人本体几何 vs 真机的物理偏差没有被验证过：已有的 bag FK 比对只能证明
  "模型 == 厂商上报的位姿"，因为 `/get_pose` 是同一套运动学算出来的。
