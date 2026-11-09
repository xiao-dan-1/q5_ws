# q5_ctrl

WSL/rospc 端的 **Q5 关节命令发布器**。发的是机器人 controller 真正吃的那条消息，
不是自己造的话题。

## 真机接口（2026-09-15 实测，不是从手册抄的）

| 项 | 值 |
|---|---|
| 消息 | `xbot_common_interfaces/msg/HybridJointCommand` |
| 字段 | `header, joint_name[], position[], velocity[], feedforward[], kp[], kd[]` —— 6 个数组**必须等长** |
| 臂 | `/wr1_controller/commands`，14 关节，`kp=85, kd=20, ff=0` |
| 手 | `/hand_controller/commands`，24 关节，`kp=100, kd=0, ff=350` |
| 关节名 | 模型契约名 `left_shoulder_pitch_joint` …，**不带** `follower_` 前缀 / `.pos` 后缀 |
| 位置控制 | 手册说 `velocity/feedforward/kp/kd` 在位置模式下不生效，但项目按已验证模板全量填充 |

## 硬规则（踩过的坑）

1. **按名重排，禁止 `position[:38]`**：`/joint_states` 的数组顺序是乱的（实测开头是
   `left_drv_wheel_joint, left_drv_hang_joint, right_shoulder_pitch_joint …`），
   必须建 name→position 映射再按契约顺序取。
2. **拒绝而非裁剪**：越出手册限位的目标直接报错，不静默夹到限位内。
3. **命令不驱动关节**：机器人必须已 activate 进 ACTIVE，且没有 MPC / Vive 抢同一话题，
   才有执行者。
4. **ACTIVE 态 `wr1_controller` 只吃全关节消息**，单关节消息会被静默忽略；
   READY 态的 `ready_controller` 才吃单关节（话题是 `/ready_controller/commands`）。

## 用法

```bash
source /opt/ros/humble/setup.bash
source ~/q5_ws/install/setup.bash
# 默认 dry-run：只构造+自检+打印，不发布
ros2 run q5_ctrl posture_node --ros-args -p positions:="[38 个数]"
# 真机发布（前提：已 ACTIVE、无 MPC/Vive）
ros2 run q5_ctrl posture_node --ros-args -p positions:="[...]" -p enable_motion:=true
# 只发臂（READY 态或只想动臂）
ros2 run q5_ctrl posture_node --ros-args -p enable_motion:=true -p body_only:=true
```

38 维顺序见 `q5_joint_limits.py`（左臂 0-6 / 右臂 7-13 / 左手 14-25 / 右手 26-37）。

## 消息类型

机器人（有 SDK）直接 `from xbot_common_interfaces.msg import HybridJointCommand`。

**Ubuntu-xd 现在也有真消息**（2026-09-15 补上）：从机器人拷了 src，在
`~/q5_ws/src/xbot_common_interfaces` 用 colcon 编出来，与机器人侧的 `.msg` 逐字节相同、
包名/版本都是 `0.0.0`。所以 `enable_motion:=true` 能正常建发布器，不再被替身挡住。

`xbot_msgs.py` 只是"连消息包都没编"时的兜底——**替身只能 dry-run**，
`enable_motion:=true` 时会明确报错（没有 `_TYPE_SUPPORT`，建不出发布器）。


## 自检

```bash
cd ~/q5_ws/src/q5_ctrl && python3 -m pytest test/ -q
# 13 passed（2026-09-28 实测）
```

覆盖：38 维契约、手部范围闸门全覆盖、手册限位拒绝而非裁剪、非有限值拒绝、
`build_command` 字段等长与增益、话题选择、预检四种判定（无订阅者 / 有冲突发布者 /
干净通过 / 白名单放行），以及**运行期复查**（启动后才出现的抢话题者被发现、
同一冲突不重复刷屏）。共 15 项。

**旧记录（已修）**：2026-09-15 记的挂测 —— 假时钟 `to_msg()` 返回字符串（旧英文
`stamp`），真 `builtin_interfaces/Time` 的 setter 会断言类型。现在假时钟返回**真的**
`Time()`，真消息 / 替身两种环境都过。

## 安全预检（发布前，2026-09-28 加）

真机模式（`enable_motion:=true`）启动时先查环境，**不通过就拒绝启动**，而不是安静地空转：

| 检查 | 判据 | 为什么 |
|---|---|---|
| 命令话题有订阅者 | `count_subscribers(topic) > 0` | READY 态 `/wr1_controller/commands` 只有监视节点、真控制器 unconfigured → 命令发出去没人执行，看着"在跑"其实什么都没发生 |
| 命令话题没有别的发布者 | `get_publishers_info_by_topic` 里除自己与白名单外为空 | MPC / Vive / `hand_server` 抢同一话题 → 两路命令互相打架 |
| `/joint_states` 有发布者 | `count_publishers > 0` | 只是提示，dry-run 下不影响 |

白名单默认放行机器人自带的只读监视节点 `joint_metric_wr1_command_subscriber` /
`joint_metric_command_subscriber`（挂在命令话题上但不是抢控制权）。

| 参数 | 默认 | 说明 |
|---|---|---|
| `check_environment` | `true` | 关掉可只做构造不检查（连不上机器人时用） |
| `force_publish` | `false` | 预检不通过也发（打 WARN；明确知道自己在干什么时才用） |
| `allowed_publishers` | 上面两个监视节点 | 追加自己环境里已知无害的发布者 |
| `env_check_period` | `5.0` | 启动后复查间隔（秒） |

### 运行期复查（2026-09-28 修掉的一个真 bug）

启动闸门只挡得住**启动那一刻**的冲突。原实现是：

```python
if self._check_env and not self._warned_environment and not self._force:   # 只查一次
```

于是"启动之后才加入的抢话题者"完全看不见 —— 而这恰好是实际启动顺序：

```
t=0s    q5_ctrl 起（话题干净，闸门放行）
t=14s   vive_teleop.launch.py 拉起 hand_tcp_hf_bridge
        -> 两者同时发 /hand_controller/commands，无任何告警
```

改成周期性复查（默认 5s），且**只在状态变化时告警**：冲突集合一变就报一次、恢复时报一次，
既不漏也不刷屏。

实测（真实 DDS，两个实例）：

| 场景 | 结果 |
|---|---|
| 无订阅者、无冲突 | 拒绝启动（`rc=1`） |
| 有订阅者、无冲突 | 通过并持续发布（跑满 25s 不退出） |
| 已有另一个发布者在发同一话题 | 拒绝启动（`rc=1`），点名冲突节点 |
| 冲突 + `force_publish:=true` | 打 WARN 后照发 |

启动时的拒绝信息形如：

```text
[INFO] 预检 /wr1_controller/commands: 没有订阅者 —— 命令不会被任何控制器执行（机器人未启动底层 / 未进 ACTIVE，或 DDS 没连上）
[INFO] 预检 /hand_controller/commands: 没有订阅者 —— ...
[INFO] 预检 提示：/joint_states 当前无发布者，收不到关节反馈（dry-run 下不影响）
预检不通过，拒绝发布命令。
[ros2run]: Process exited with failure 1
```

## 构建（必须带 `--merge-install`）

`q5_ws/install/.colcon_install_layout` 是 **`merged`**：install 根目录本身就是每个包的
prefix，所以**不存在** `install/q5_ctrl/` 子目录（`ros2 pkg prefix q5_ctrl` 返回的就是
`install/`），入口也不在 PATH。因此**裸 `colcon build` 会直接报错**：

```text
ERROR:colcon: The install directory 'install' was created with the layout 'merged'.
Please remove the install directory, pick a different one or add the '--merge-install' option.
```

正确写法：

```bash
cd ~/q5_ws
colcon build --merge-install --packages-select q5_ctrl   # 只重编这个包，实测约 2.5s
colcon build --merge-install                             # 全工作区
```

查入口：`ros2 pkg executables q5_ctrl`（实测输出 `q5_ctrl posture_node`）。

## 旧 egg-info 会打坏新入口（2026-10-20 实测，之前"无害"的判断是错的）

每次改版本号重新 build，`install/**/site-packages/` 下就多一份
`q5_ctrl-<版本>-py3.10.egg-info`，colcon **不会删旧的**。几份同名元数据并存时，
`importlib.metadata.distribution('q5-ctrl')` 取哪个**只看目录列举顺序**，不看版本。

后果是入口脚本报 `StopIteration`：

```
$ ros2 run q5_ctrl vive_pose_static
  File ".../lib/q5_ctrl/vive_pose_static", line 25, in importlib_load_entry_point
    return next(matches).load()
StopIteration
```

脚本第 33 行写死的是 `load_entry_point('q5-ctrl==0.2.0', ...)`，没错；错在
`distribution()` 挑中了旧的那份 `entry_points.txt`，里面没有 `vive_pose_static`。
`posture_node` 当时还能跑，只是因为最老那份元数据里恰好有它 —— **不是它没问题**。

处理：留当前版本、删其它，`81_sync_q5_ctrl.sh --build` 已自动做（`_purge_stale_egg`）。
要手工：

```bash
SP=~/q5_ws/install/lib/python3.10/site-packages          # 真机：.../install/q5_ctrl/lib/python3.10/site-packages
rm -rf "$SP"/q5_ctrl-0.*.egg-info                        # 别删当前版本
```
## WSL ↔ 机器人 链路（2026-09-15 实测：**双向可用**）

| 项 | 实测 |
|---|---|
| IP 层 | 同网段（WSL `eth2` = 192.168.8.234/24 ↔ 机器人 192.168.8.100），ping 0.29 ms |
| TCP | WSL→机器人 2222 通；机器人→WSL 的入站 TCP 也通 |
| DDS 发现 | 双向可见：WSL 侧 `--no-daemon` 能列出 32 个节点 / 机器人全部话题（**要显式 `export ROS_DOMAIN_ID=211`——xd 的默认域是 77，见下**）；机器人侧 `ros2 node list` 能看到 WSL 起的 `/q5_posture` |
| 数据面 | WSL 发 `HybridJointCommand` → 机器人 `ros2 topic echo` 收到全部字段；机器人 `/joint_states` → WSL 节点收到 48 关节（38 个契约关节齐全，缺 0） |

**坑：别信 ros2 daemon 的图。**

- `ros2 topic list` 默认走 daemon，图陈旧时只剩 `/parameter_events` `/rosout`，看着像"发现为零"。
  先 `ros2 daemon stop`，再用 `--no-daemon`。
- `ros2 topic echo <话题>` 不给类型时会去问 daemon 要类型，会误报
  `does not appear to be published yet`。一律显式给类型：
  `ros2 topic echo /q5_cmd_type_test xbot_common_interfaces/msg/HybridJointCommand --once`。

**仍成立的限制**：无由头的裸 UDP 反向不通 —— 机器人主动往 WSL 一个从没通信过的新端口发 UDP 收不到
（WSL 是 mirrored 网络模式）。DDS 不受影响（WSL 先发 SPDP，回包进得来），TCP 也不受影响。
以后设计非 DDS 的 UDP 链路要注意这一点。

**旧结论已作废**（本节 2026-09-15 早先写的"WSL 只能靠机器人主动连过来、TCP 全不通、DDS 发现为零、
在 WSL 里只能 dry-run"）：那是 ros2 daemon 缓存造成的误判，不是网络问题。

## 发命令前先看订阅者（READY 态实测）

| 话题 | 谁在订 |
|---|---|
| `/wr1_controller/commands` | 只有监视节点 `joint_metric_wr1_command_subscriber`；真 `wr1_controller` 在 READY 下是 **unconfigured** → **臂命令发出去没人执行** |
| `/hand_controller/commands` | `hand_controller` **active** → **手命令会真的动手** |

所以 READY 态下"发臂"是空操作、"发手"是真动作；要动臂必须先 READY → ZERO → LIFT → ACTIVE。

## 版本与同步（2026-10-20 统一）

仓库里的 `q5_ws/src/q5_ctrl` 是**唯一真源**，版本 `0.2.0` = WSL 一份 + 真机一份的并集：

| 文件 | 来源 | 说明 |
|---|---|---|
| `q5_ctrl/posture_node.py` `q5_joint_limits.py` `xbot_msgs.py` `vive_static_pose_node.py` | WSL | `posture_node` 带 `xbot_common_interfaces` 缺失时的替身回退 |
| `q5_ctrl/keyboard_node.py` `web_node.py` `xbot_keyboard_node.py` | 真机 | 硬依赖厂商 SDK `robot_controller`，只能在真机跑 |
| `q5_ctrl/start_pose_node.py`、`config/start_pose/` | WSL | 采集起始位姿（见下） |
| `test/test_posture_node.py` `test_start_pose_node.py` | WSL | 限位/关节映射/起始位姿纯函数 |
| `q5_contract.py` | — | 已改名为 `q5_joint_limits.py`，不要再建 |

### 起始位姿

**就用 ABC，一条命令：**

```bash
ros2 launch q5_ctrl start_pose.launch.py
```

`launch/start_pose.launch.py` 里没有自研运动代码，全是现成的：

| 时间 | 动作 |
|---|---|
| 4s | 查 `/query_xbot_state`，已 ACTIVE 就跳过；否则 `ros2 service call /activate_service`（不激活机器人不动） |
| 6s | `q5_pub_client.py --type mpc --cmd start` 并回读状态（`/get_pose` 与 `/servo_poses` 都归它） |
| 11s | `q5_ctrl/vive_static_pose_node`（ABC 里的 A） |
| 11s | `vive_dual_arm_teleop/vive_teleop_node`（B），`teleop_enabled` 直接给 `True`（替掉 C） |

`teleop_base` 里的 check / camera_guard / joint_states_relay 没抄进来——电池诊断、相机、
跨机 60Hz 中继都与起始位姿无关，还会把启动拖到 18s。4s/6s 的延时是照抄它的实测值，
等 DDS 发现完再发服务调用。

落点由遥操作节点自己的 `locked_{left,right,head}_pose_xyz_quat` 参数决定，改也是改那儿。
到没到位用眼睛看；本 launch 不停 MPC、不 deactivate，Ctrl-C 只停自己起的三个进程。

`start_pose_node`（下面这段）是另一条路：自己发 `/servo_poses`，带 dry-run 和收敛判据，
不到 1 cm / 0.05 rad 会自动退出。两者终值相同，只是它不需要起厂商节点。

```bash
ros2 run q5_ctrl start_pose_node                                    # dry-run，只打印差值
ros2 run q5_ctrl start_pose_node --ros-args -p enable_motion:=true  # 真动
```

目标在 `config/start_pose.json`，数值与遥操作节点的
`locked_{left,right,head}_pose_xyz_quat` 默认参数一致——改这里等于改那条路的落点。
`/get_pose` 没有发布者就直接退出（code 2），不启停 MPC、不切状态机。

同步（只在仓库改，别在 WSL / 机器人上改）：

```bash
bash openpi_ws/_setup/81_sync_q5_ctrl.sh --check        # 查漏：MISSING / DIFF / EXTRA
bash openpi_ws/_setup/81_sync_q5_ctrl.sh                # 推 WSL + 真机
bash openpi_ws/_setup/81_sync_q5_ctrl.sh nuc --build    # 推完在真机 colcon build
```

`EXTRA` 只报告不删，确认是垃圾后手工删。WSL 是 merge-install，真机是 isolate-install，
colcon 参数不同，脚本里已分好。
