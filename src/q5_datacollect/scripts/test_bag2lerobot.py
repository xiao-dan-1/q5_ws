#!/usr/bin/env python3
"""bag2lerobot 自检：造一个合成 bag（不碰 DDS、不碰真机）跑通全链路。

    source /opt/ros/humble/setup.bash
    source /home/developer/teleop_client/install/setup.bash
    python3 test_bag2lerobot.py

覆盖：按名重排（消息里的关节顺序**故意打乱**，且混入 10 个契约外关节）、
command / echo 两种 action、缺命令时拒绝出帧（不静默退化成回声）、帧数==图数、
图像原样落盘、npz / parquet 产物、q5_joint_limits.py 两份的同步自检。
"""
from __future__ import annotations

import base64
import gc
import importlib.machinery
import importlib.util
import json
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))
# 被 import 的脚本去掉了 .py 后缀（ros2 run 的入口名 = 文件名，见 setup.py），
# import 语句找不到它，只能按路径加载 —— 这也正是它装到 lib/q5_tools/ 之后的形态。
_spec = importlib.util.spec_from_loader(
    "bag2lerobot", importlib.machinery.SourceFileLoader("bag2lerobot", str(_HERE / "bag2lerobot")))
B = importlib.util.module_from_spec(_spec)
sys.modules["bag2lerobot"] = B
_spec.loader.exec_module(B)
from q5_joint_limits import ARM_JOINT_NAMES, HAND_JOINT_NAMES, MODEL_JOINT_NAMES  # noqa: E402

FPS, SECS = 30, 2
N_IMG = FPS * SECS
STATE_LAG = 0.003      # state header 比图像早这么多秒（真机 ~2ms，取 3ms 便于断言）
CMD_LEAD = 0.005       # 命令 header 比图像晚这么多秒（真机相机 header 比到达早 ~28ms）
T0 = 1.0               # 整段往后挪 1s：bag 时间戳不能为负（第一帧 state 要往前 3ms）
# 1x1 白 JPEG —— 只为验证"字节原样落盘"，转换器不解码
JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAQEAYABgAAD/2wBDAAgGBgcGBQgHBwcJCQgKDBQNDAsLDBkSEw8UHRofHh0a"
    "HBwgJC4nICIsIxwcKDcpLDAxNDQ0Hyc5PTgyPC4zNDL/wAALCAABAAEBAREA/8QAFAABAAAAAAAA"
    "AAAAAAAAAAAACf/EABQQAQAAAAAAAAAAAAAAAAAAAAD/2gAIAQEAAD8AKp//2Q==")

# 契约外的 10 个关节（真机 /joint_states 里就有，用来验证"多出来的会被忽略"）
EXTRA = ["left_drv_wheel_joint", "hip_joint", "knee_joint", "ankle_joint", "waist_yaw_joint",
         "neck_pitch_joint", "neck_yaw_joint", "left_drv_hang_joint", "right_drv_wheel_joint",
         "right_drv_hang_joint"]

IDX = {n: i for i, n in enumerate(MODEL_JOINT_NAMES)}


def build_bag(path: Path, with_cmd: bool, jpeg_fn=None, skip=(), late=(),
              cmd_eq_state=()) -> None:
    """jpeg_fn(i) 给第 i 帧的 JPEG 字节；缺省恒用同一张 1×1 图（故意冻帧，测检测器）。

    skip:         这些帧号的 state/命令/图像**整段不写** → 造一个时间空窗
    late:         这些图像帧的**接收时间**推到所有消息之后 → 到达顺序与 stamp 序不符
                  （真机跨机晚到就是这样：header stamp 早、recv 晚）
    cmd_eq_state: 这些组（左臂/右臂/左手/右手）的命令值抄 state → 模拟退化的 follower 副本
    """
    import rosbag2_py
    from builtin_interfaces.msg import Time
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CompressedImage, JointState
    from xbot_common_interfaces.msg import HybridJointCommand

    jpeg_fn = jpeg_fn or (lambda i: JPEG)

    def stamp(t):
        # t 可能为负（第一帧 state 往前挪了 3ms）→ 用 divmod 拆，保证 nanosec ∈ [0, 1e9)
        ns = int(round(t * 1e9))
        return Time(sec=ns // 10**9, nanosec=ns % 10**9)

    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    for name, typ in ((B.STATE_RELAY, "sensor_msgs/msg/JointState"),
                      (B.IMG, "sensor_msgs/msg/CompressedImage"),
                      (B.ARM_CMD, "xbot_common_interfaces/msg/HybridJointCommand"),
                      (B.HAND_CMD, "xbot_common_interfaces/msg/HybridJointCommand")):
        w.create_topic(rosbag2_py.TopicMetadata(
            name=name, type=typ, serialization_format="cdr"))

    order = EXTRA[:5] + list(MODEL_JOINT_NAMES)[::-1] + EXTRA[5:]   # 故意打乱

    def write_state(t):
        js = JointState()
        js.header.stamp = stamp(t)
        js.name = order
        # 契约关节 = 序号×0.01；契约外关节 = 9.0（映射若按下标就会看到 9.0）
        js.position = [IDX[n] * 0.01 if n in IDX else 9.0 for n in order]
        w.write(B.STATE_RELAY, serialize_message(js), int(t * 1e9))

    # 点名要"抄 state"的关节名（按 GROUPS 的切片取）→ 命令值直接用 state 的值 IDX*0.01，
    # 转换后那一组的 38 维切片与 state 逐位相同，正是 dexdagger 那种退化标签的样子
    eq_names = set()
    for _g, _sl in B.GROUPS:
        if _g in set(cmd_eq_state):
            eq_names |= set(MODEL_JOINT_NAMES[_sl])

    def write_cmds(t):
        for topic, names, base in ((B.ARM_CMD, ARM_JOINT_NAMES, 0.5),
                                   (B.HAND_CMD, HAND_JOINT_NAMES, -0.5)):
            rev = list(names)[::-1]                             # 也打乱
            c = HybridJointCommand()
            c.header.stamp = stamp(t)
            c.joint_name = rev
            c.position = [(IDX[n] * 0.01 if n in eq_names else base + names.index(n) * 0.001)
                          for n in rev]
            w.write(topic, serialize_message(c), int(t * 1e9))

    # 预卷：正式帧之前先来几帧 state/命令（真机就是这样：链路先起、相机随后才有图）。
    # 转换器现在是**按 stamp 因果配对**，没有预卷的话第 0 帧图像找不到"stamp ≤ 图像"的命令
    # → 会被判 no_action，少一帧。
    for k in range(3, 0, -1):
        tp = T0 - k / FPS
        write_state(tp - STATE_LAG)
        if with_cmd:
            write_cmds(tp + CMD_LEAD)

    # 时序刻意做成：state 比图像早 3ms；命令比图像**晚** 5ms（真机命令就是比图像"新"）。
    # 因果配对下命令只能配到上一帧那条 → 年龄 = 1/FPS − 5ms；那条"未来"的命令必须不被选中。
    skip_set, late_set = set(skip), set(late)
    img_queue = []
    for i in range(N_IMG):
        if i in skip_set:
            continue                                # 整段不写 → 该处出现时间空窗
        t = T0 + i / FPS
        write_state(t - STATE_LAG)
        if with_cmd:
            write_cmds(t + CMD_LEAD)
        img_queue.append((i, t, jpeg_fn(i)))

    # 「到达顺序」= rosbag2 的**接收时间**：SequentialReader 是按 timestamp 排好返回的，
    # 不按写入次序（实测：只把写入挪到最后、recv 还是早的，读出来照样是 stamp 序）。
    # 真机上相机帧跨机晚到就是这样 —— header stamp 早、recv 时间晚。
    # 所以 late 里的帧不但最后写，recv 也往后推一大截，读出来才"后到却带着早的 stamp"。
    _ord = [k for k in range(len(img_queue)) if img_queue[k][0] not in late_set]
    _ord += [k for k in range(len(img_queue)) if img_queue[k][0] in late_set]
    for k in _ord:
        i, t, data = img_queue[k]
        im = CompressedImage()
        im.header.stamp = stamp(t)
        im.format = "jpeg"
        im.data = data
        recv = t + 10.0 if i in late_set else t + 0.002   # stamp 走 header，recv 只管顺序
        w.write(B.IMG, serialize_message(im), int(recv * 1e9))

    del w
    gc.collect()                                   # 让 rosbag2 收尾写出 metadata.yaml


# 对侧的 q5_ctrl 那份：src/q5_ctrl/q5_ctrl/q5_joint_limits.py（从本文件往上数两级是 src/）
_CONTRACT_OTHER = _HERE.parents[1] / "q5_ctrl" / "q5_ctrl" / "q5_joint_limits.py"
# 至少要有这几个共享常量；名字改了/被删了 → 交集变空，下面的比对会"全过"，所以先钉死
_CONTRACT_SHARED = {"MODEL_JOINT_NAMES", "ARM_JOINT_NAMES", "HAND_JOINT_NAMES",
                    "MANUAL_ARM_LIMITS", "DATASET_MEAN_POSE"}


def check_contract_sync() -> int:
    """两份 q5_joint_limits.py 的共享常量逐字比对 —— 谁只改一边就当场失败。

    两份物理文件是故意的（q5_datacollect / q5_ctrl 各自独立部署），代价就是可能改单边：
    38 个关节名与顺序错一个，采出来的 action 维序就和模型对不上，而且是静默的。
    """
    if not _CONTRACT_OTHER.is_file():
        # 装到 install/lib/q5_datacollect/ 之后就没有对侧目录了 —— 跳过（不是失败）
        print("  [0] 契约同步：找不到对侧 %s，跳过（已装进 lib/ 属正常）" % _CONTRACT_OTHER)
        return 0
    import q5_joint_limits as mine
    spec = importlib.util.spec_from_file_location("_q5_joint_limits_ctrl", str(_CONTRACT_OTHER))
    other = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(other)
    shared = sorted(k for k in vars(mine)
                    if k.isupper() and not k.startswith("_") and hasattr(other, k))
    missing = _CONTRACT_SHARED - set(shared)
    assert not missing, "两份都该有却没对上：%s（改名了？）" % sorted(missing)
    for k in shared:
        a, b = getattr(mine, k), getattr(other, k)
        assert a == b, ("q5_joint_limits.py 两份不一致：%s\n  scripts/ 那份：%r\n  q5_ctrl/ 那份：%r\n"
                        "（改哪份都得同步另一份，否则维序/限位只对一边生效）" % (k, a, b))
    print("  [0] 契约同步：%d 个共享常量与 q5_ctrl 那份逐字一致（%s）✓"
          % (len(shared), " ".join(shared)))
    return 0


def check_dynamic_video_metadata(tmp):
    """真实编码视频：非默认分辨率、单通道、错误输入和跨集一致性。"""
    import cv2

    small_bags = tmp / "small_bags"
    small_bags.mkdir()
    def small_jpeg(i):
        image = np.full((240, 320, 3), i % 256, dtype=np.uint8)
        return cv2.imencode(".jpg", image)[1].tobytes()
    build_bag(small_bags / "ep001", with_cmd=True, jpeg_fn=small_jpeg)
    out = tmp / "small_dataset"
    argv, sys.argv = sys.argv, ["bag2lerobot", str(small_bags), "--task", "small",
                                "-o", str(out), "--jobs", "1"]
    try:
        B.main()
    finally:
        sys.argv = argv
    info = json.loads((out / "meta/info.json").read_text())
    feature = info["features"][B.IMG_KEY]
    assert feature["shape"] == [240, 320, 3], feature
    assert feature["info"]["video.height"] == 240
    assert feature["info"]["video.width"] == 320
    assert feature["info"]["video.channels"] == 3
    assert feature["info"]["video.codec"] == "h264"
    assert feature["info"]["video.pix_fmt"] == "yuv420p"

    # 通道数必须从实际像素格式获得；灰度视频不能被标成 RGB 三通道。
    gray = tmp / "gray.mkv"
    B.subprocess.run(
        ["ffmpeg", "-v", "error", "-nostdin", "-f", "lavfi", "-i",
         "color=size=80x48:rate=24", "-frames:v", "3", "-vf", "format=gray",
         "-c:v", "ffv1", "-threads", "1", str(gray)], check=True,
    )
    actual = B.probe_video_info(gray)
    assert (actual["video.height"], actual["video.width"], actual["video.channels"],
            actual["video.fps"], actual["video.codec"]) == (48, 80, 1, 24., "ffv1"), actual

    def rejects(call, text):
        try:
            call()
        except SystemExit as e:
            assert text in str(e), e
        else:
            raise AssertionError("应拒绝：" + text)
    corrupt = tmp / "corrupt.mp4"
    corrupt.write_bytes(b"not a video")
    rejects(lambda: B.probe_video_info(corrupt), "读取视频元数据失败")

    mixed = tmp / "mixed_video_metadata"
    videos = mixed / "videos" / "chunk-000" / B.IMG_KEY
    videos.mkdir(parents=True)
    source = out / "videos" / "chunk-000" / B.IMG_KEY / "episode_000000.mp4"
    shutil.copy2(source, videos / "episode_000000.mp4")
    large = tmp / "ds_multi/videos/chunk-000" / B.IMG_KEY / "episode_000000.mp4"
    shutil.copy2(large, videos / "episode_000001.mp4")
    eps = [{"episode_index": i, "tasks": ["mixed"], "length": N_IMG} for i in range(2)]
    rejects(lambda: B.write_meta(mixed, "mixed", eps, FPS, []), "各集视频描述不一致")
    assert not (mixed / "meta/info.json").exists()
    rejects(lambda: B.write_meta(mixed, "mixed", eps[:1], 24, []), "实际帧率")
    print("  [15] 动态视频元数据：320x240 完整转换、80x48 单通道/24fps；坏视频/混合尺寸/帧率错配拒绝 ✓")


def main() -> int:
    check_contract_sync()
    tmp = Path(tempfile.mkdtemp(prefix="bag2lr_test_"))
    print("临时目录:", tmp)
    bags = tmp / "bags"
    bags.mkdir(parents=True)
    build_bag(bags / "ep001", with_cmd=True)      # bag 目录由 writer 自己建

    # ---- 1) command 模式：action 取命令话题 ----
    # convert_episode(bag, out, ep, action_src, fps, ...) —— 没有 idx 那一位了（index0 由 main 现算），
    # 老版本是 (bag, out, ep, idx, action, fps)，本文件跟着改过一次（10/14 跑全套时才发现一直在挂）
    out = tmp / "ds_command"
    n, qc = B.convert_episode(bags / "ep001", out, 0, "command", FPS)
    assert n == N_IMG, "帧数 %d != %d" % (n, N_IMG)
    assert qc["no_state"] == 0 and qc["no_action"] == 0, qc
    assert qc["stale_state"] == 0 and qc["stale_action"] == 0, qc
    # 年龄（按 stamp 因果配对）：state 配到同一帧那条 → +3ms；
    # 命令只能配到上一帧那条（本帧那条 stamp 比图像晚 5ms，属于"未来"，不许被选中）
    causal_cmd_age = 1.0 / FPS - CMD_LEAD
    assert abs(qc["age_s_max"] - STATE_LAG) < 1e-6, qc["age_s_max"]
    assert abs(qc["age_a_min"] - causal_cmd_age) < 1e-6, qc["age_a_min"]
    assert abs(qc["age_a_max"] - causal_cmd_age) < 1e-6, qc["age_a_max"]
    assert qc["age_a_future"] == 0, "按 stamp 配对后不该再出现负年龄（配到了未来数据）"

    img_dir = out / "images" / B.IMG_KEY / "episode_000000"
    files = sorted(img_dir.glob("frame_*.jpg"))
    assert len(files) == n, "图数 %d != 帧数 %d" % (len(files), n)
    assert files[0].read_bytes() == JPEG, "图像没有原样落盘"

    d = np.load(out / B.NPZ_DIR / "episode_000000.npz")
    S, A, T = d["state"], d["action"], d["stamp"]        # stamp = 真实图像时刻（质检用）
    # npz 不许落在 data/ 下：HF datasets 的 load_dataset(data_dir=) 会把该目录下
    # 所有文件当 parquet 读，实测报 "Parquet magic bytes not found in footer"
    assert not list((out / B.DATA_DIR).glob("*.npz")), "npz 不能放在 data/ 下"
    want_state = np.arange(len(MODEL_JOINT_NAMES), dtype=np.float32) * 0.01
    for k in (0, n // 2, n - 1):
        assert np.allclose(S[k], want_state), "按名重排错了(帧%d)：%s" % (k, S[k][:5])
    assert np.allclose(A[0][:14], 0.5 + np.arange(14) * 0.001), A[0][:14]
    assert np.allclose(A[0][14:], -0.5 + np.arange(24) * 0.001), A[0][14:]
    assert not np.allclose(A, S), "command 模式下 action 不该等于 state"
    # stamp 是真实图像时刻（+3ms/-5ms 的构造偏移不影响帧间隔）
    assert np.allclose(np.diff(T), 1.0 / FPS, atol=1e-3), "stamp 应为真实图像时刻差"
    print("  [1] command 模式：%d 帧，按名重排/action≠state/图像原样 全部通过" % n)

    # ---- 2) echo 模式：同一份 bag，action 退化成 state ----
    out2 = tmp / "ds_echo"
    n2, qc2 = B.convert_episode(bags / "ep001", out2, 0, "echo", FPS)
    d2 = np.load(out2 / B.NPZ_DIR / "episode_000000.npz")
    assert n2 == n and np.array_equal(d2["action"], d2["state"]), "echo 模式 action 应等于 state"
    assert qc2["age_a_max"] is None, "echo 模式不该有命令年龄（无命令话题）"
    assert not any(qc2["eq"].values()), "echo 模式不该去比 action==state（那本来就应该全等）"
    assert abs(qc2["age_s_max"] - STATE_LAG) < 1e-6, qc2["age_s_max"]
    print("  [2] echo 模式：action == state、命令年龄 n/a ✓（同一份 bag，两种口径）")

    # ---- 3) 没有命令话题 → 拒绝出帧，而不是静默退化成回声 ----
    build_bag(bags / "ep002", with_cmd=False)
    n3, qc3 = B.convert_episode(bags / "ep002", tmp / "ds_nocmd", 0, "command", FPS)
    assert n3 == 0, "缺命令时不该出帧，实际 %d" % n3
    assert qc3["no_action"] == N_IMG, qc3
    print("  [3] 缺命令：0 帧、no_action=%d ✓（拒绝而非静默降级）" % qc3["no_action"])

    # ---- 4) parquet（有 pyarrow 才有）----
    if B.write_parquet(out, 0, 0):        # 第 3 个参数是 index0（本文件只造一集，恒 0）
        import pyarrow.parquet as pq
        tt = pq.read_table(out / B.DATA_DIR / "episode_000000.parquet")
        assert tt.num_rows == n
        assert str(tt.schema.field("action").type) == "fixed_size_list<element: float>[38]"
        assert tt.column_names == ["action", "observation.state", "timestamp", "frame_index",
                                   "episode_index", "index", "task_index"]
        # LeRobot 的 check_timestamps_sync 要求集内严格等间隔（容差 1e-4 s）
        ts = np.asarray(tt.column("timestamp").to_pylist(), dtype=np.float64)
        assert np.abs(np.diff(ts) - 1.0 / FPS).max() < 1e-4, "timestamp 必须等间隔"
        print("  [4] parquet：%d 行，action %s，timestamp 等间隔（误差 %.2e）✓"
              % (tt.num_rows, tt.schema.field("action").type,
                 np.abs(np.diff(ts) - 1.0 / FPS).max()))
    else:
        print("  [4] parquet：本机没有 pyarrow，跳过（用 --pack 在 PC 上补）")

    # ---- 5) 多集 + 视频编码：走 main()（真实工作流 = 一个目录 N 个 bag 一次转完）----
    # 先拦一道 pyarrow：真机没有它，而 main() 那时会 sys.exit("...pyarrow...")，
    # 在这里就是裸 SystemExit + traceback（看着像"测试挂了"，其实是环境缺件）。
    if not B.have_pyarrow():
        print("  [5..8] 本机没有 pyarrow（真机就是这种）：多集/视频/统计这几步跳过，去 PC 上跑")
        shutil.rmtree(tmp, ignore_errors=True)
        print("\nPASS（临时目录已清理；失败时会保留）")
        return 0

    # 视频必须真过 x264：libx264+yuv420p 要求宽高为偶数，1×1 的图编不出来，
    # 所以这一组用 cv2 现造 640×480、每帧不同的 JPEG（顺带覆盖"唯一大小>1"那条分支）。
    try:
        import cv2
    except ImportError:
        print("  [5] 多集/视频：本机没有 cv2，跳过（真机有，转换器也只需要 ffmpeg）")
        shutil.rmtree(tmp, ignore_errors=True)
        print("\nPASS（临时目录已清理；失败时会保留）")
        return 0

    def varied_jpeg(i):
        img = np.full((480, 640, 3), (i * 7) % 256, dtype=np.uint8)
        cv2.putText(img, str(i), (300, 240), cv2.FONT_HERSHEY_SIMPLEX, 3,
                    (255, 255, 255), 4)
        return cv2.imencode(".jpg", img)[1].tobytes()

    multi = tmp / "multi"
    multi.mkdir()
    build_bag(multi / "ep001", with_cmd=True, jpeg_fn=varied_jpeg)
    build_bag(multi / "ep002", with_cmd=True, jpeg_fn=varied_jpeg)
    out5 = tmp / "ds_multi"
    argv = sys.argv
    sys.argv = ["bag2lerobot.py", str(multi), "--task", "multi", "-o", str(out5)]
    try:
        B.main()
    finally:
        sys.argv = argv
    info = json.loads((out5 / "meta" / "info.json").read_text(encoding="utf-8"))
    eps = [json.loads(l) for l in
           (out5 / "meta" / "episodes.jsonl").read_text(encoding="utf-8").splitlines() if l]
    assert info["total_episodes"] == 2, info["total_episodes"]
    assert info["total_frames"] == 2 * N_IMG, info["total_frames"]
    assert info["splits"] == {"train": "0:2"}, info["splits"]
    assert [e["length"] for e in eps] == [N_IMG, N_IMG], eps
    # index0 只从 parquet 读：npz 里**已经没有** index0 了（并行转换时那一集自己算不出真值，
    # 存着只会是假值，见 bag2lerobot 里 npz 那段注释）—— 老版本这个分支读 npz["index0"]，早就错了
    import pyarrow.parquet as pq
    i0 = [int(pq.read_table(out5 / B.DATA_DIR / ("episode_%06d.parquet" % k),
                            columns=["index"])["index"][0].as_py()) for k in (0, 1)]
    assert i0 == [0, N_IMG], "全局 index 偏移必须连续，实际 %s" % i0
    print("  [5] 多集：2 集 %d 帧，index 偏移 %s、splits %s ✓"
          % (info["total_frames"], i0, info["splits"]))

    # ---- 5b) 没 pyarrow 又没 --npz：必须明确报错退出，而不是留个缺 data/ 的半成品 ----
    argv, sys.argv = sys.argv, ["bag2lerobot", str(bags), "--task", "t", "-o",
                                str(tmp / "ds_nopyarrow")]
    real_have, B.have_pyarrow = B.have_pyarrow, (lambda: False)
    try:
        try:
            B.main()
            raise AssertionError("没 pyarrow 时应该报错退出，结果它跑完了")
        except SystemExit as e:
            assert "pyarrow" in str(e), "报错信息没提 pyarrow：%s" % e
        assert not (tmp / "ds_nopyarrow" / "meta" / "info.json").exists(), "不该留下半成品数据集"
        print("  [5b] 没 pyarrow 且没 --npz：当场报错退出、不留半成品 ✓")
    finally:
        B.have_pyarrow = real_have
        sys.argv = argv

    # ---- 6) 视频：必须是 video 模式且帧数严格等于 state 行数（旧 merge 多 2 帧）----
    assert info["features"][B.IMG_KEY]["dtype"] == "video", info["features"][B.IMG_KEY]
    assert info["features"][B.IMG_KEY]["info"]["video.fps"] == float(FPS)
    video_feature = info["features"][B.IMG_KEY]
    assert video_feature["shape"] == [480, 640, 3], video_feature
    assert video_feature["info"]["video.height"] == 480
    assert video_feature["info"]["video.width"] == 640
    assert video_feature["info"]["video.channels"] == 3
    assert info["total_videos"] == 2 and info.get("video_path"), info
    assert "image_path" not in info, "视频模式下不该有 image_path"
    for k in (0, 1):
        v = out5 / "videos" / "chunk-000" / B.IMG_KEY / ("episode_%06d.mp4" % k)
        assert v.is_file(), "缺视频 %s" % v
        got = B.subprocess.run(
            ["ffprobe", "-v", "error", "-count_frames", "-select_streams", "v:0",
             "-show_entries", "stream=nb_read_frames", "-of", "csv=p=0", str(v)],
            capture_output=True, text=True).stdout.strip()
        assert got == str(N_IMG), "ep%d 视频 %s 帧 != %d 行" % (k, got, N_IMG)
    print("  [6] 视频：dtype=video、total_videos=%d，两集 mp4 帧数都 == %d ✓"
          % (info["total_videos"], N_IMG))
    assert not list((out5 / "images").rglob("*.jpg")), "视频模式下原图应已删除"

    # ---- 7) episodes_stats.jsonl：lerobot 的 load_metadata() 必需 ----
    # 缺它 → FileNotFoundError → 转去查 HF Hub → 报误导性的 401 RepositoryNotFoundError
    stats_path = out5 / "meta" / "episodes_stats.jsonl"
    assert stats_path.is_file(), "缺 meta/episodes_stats.jsonl（lerobot 会加载失败）"
    srows = [json.loads(l) for l in stats_path.read_text(encoding="utf-8").splitlines() if l]
    assert len(srows) == 2 and [r["episode_index"] for r in srows] == [0, 1], srows
    feats = set(srows[0]["stats"])
    assert {"action", "observation.state", "timestamp", "frame_index", "episode_index",
            "index", "task_index", B.IMG_KEY} <= feats, feats
    a0 = srows[0]["stats"]["action"]
    assert len(a0["min"]) == 38 and a0["count"] == [N_IMG], (len(a0["min"]), a0["count"])
    print("  [7] episodes_stats.jsonl：%d 行，%d 个 feature，action 38 维、count=%s ✓"
          % (len(srows), len(feats), a0["count"][0]))

    # ---- 8) 冻帧检测器：恒用同一张 JPEG 的那份 → 必须报"唯一大小 1 种" ----
    # （out5 的图已被 main 按设计删掉，只测 ds_command 这份）
    st = B.image_stats(out, 0, n)
    assert st["size_uniq"] == 1 and st["kb_min"] == st["kb_max"], st
    print("  [8] 冻帧检测：每帧同图 → 唯一大小 1 种、min==max ✓"
          "（真机 761 帧变帧实测 495 种）")

    # ---- 9) QC：时间空窗（整段帧被抹掉；平均速率和丢帧率都看不见）----
    gap = tmp / "gap"; gap.mkdir()
    build_bag(gap / "ep001", with_cmd=True, jpeg_fn=varied_jpeg, skip=range(20, 35))
    try:
        B.convert_episode(gap / "ep001", tmp / "ds_gap", 0, "command", FPS)
        raise AssertionError("时间空窗不能被压成等间隔训练样本")
    except SystemExit as e:
        assert "时间断点" in str(e), e
    assert not (tmp / "ds_gap").exists(), "坏集验证失败前不应导出图片或 NPZ"
    print("  [9] 时间空窗：拒绝整集、不导出训练样本 ✓")

    # ---- 10) QC：到达顺序非单调（帧晚到；stamp 没乱，所以出帧和间隔都不受影响）----
    lt = tmp / "late"; lt.mkdir()
    build_bag(lt / "ep001", with_cmd=True, jpeg_fn=varied_jpeg, late=(0,))
    n10, qc10 = B.convert_episode(lt / "ep001", tmp / "ds_late", 0, "command", FPS)
    nd10, back10 = qc10["back"][B.IMG]
    assert n10 == N_IMG, "晚到不是丢帧：出帧数不该变（实得 %d）" % n10
    # 第 0 帧最后到 → 到达序里只有末尾一处倒退（从第 59 帧的 stamp 掉回第 0 帧的），
    # 倒退幅度 = 59 个帧间隔
    assert nd10 == 1 and abs(back10 - (N_IMG - 1) / FPS) < 1e-3, (nd10, back10)
    assert abs(qc10["gaps"][B.IMG][3] - 1.0 / FPS) < 1e-6, "间隔按 stamp 算，不该被到达顺序污染"
    print("  [10] 到达顺序：1 处倒退 %.2fs，但出帧 %d 帧、按 stamp 的间隔无空窗 ✓"
          % (back10, n10))

    # ---- 11) QC：action 是 state 副本（退化的 follower 标签，dexdagger 那种）----
    cp = tmp / "copy"; cp.mkdir()
    build_bag(cp / "ep001", with_cmd=True, jpeg_fn=varied_jpeg, cmd_eq_state=("右手",))
    n11, qc11 = B.convert_episode(cp / "ep001", tmp / "ds_copy", 0, "command", FPS)
    assert qc11["eq"]["右手"] == n11, qc11["eq"]
    assert qc11["eq"]["左臂"] == 0 and qc11["eq"]["右臂"] == 0 and qc11["eq"]["左手"] == 0, qc11["eq"]
    print("  [11] action==state：右手 %d/%d 逐位全等、其余三组 0 ✓（退化的副本会这样）"
          % (qc11["eq"]["右手"], n11))

    # ---- 12) 默认删除 npz 后，原始相机时间仍有持久化记录 ----
    timing = json.loads((out5 / "meta/timing/episode_000000.json").read_text())
    assert len(timing["relative_image_stamps"]) == N_IMG
    assert np.allclose(np.diff(timing["relative_image_stamps"]), 1 / FPS)
    assert not (out5 / B.NPZ_DIR).exists()
    print("  [12] 默认删除 npz 后，meta/timing 保留真实相机时间与 QC ✓")

    # ---- 13) 无效关节消息必须拒绝，不能退回旧消息掩盖坏标签 ----
    from types import SimpleNamespace
    def rejects(call, text):
        try:
            call()
        except SystemExit as e:
            assert text in str(e), e
        else:
            raise AssertionError("应拒绝：" + text)
    for value in (float("nan"), float("inf"), -float("inf"), 1e100):
        msg = SimpleNamespace(name=list(MODEL_JOINT_NAMES), position=[value] + [0.] * 37)
        rejects(lambda: B.vec((1., msg), MODEL_JOINT_NAMES), "非有限")
    msg = SimpleNamespace(name=[MODEL_JOINT_NAMES[0]] * 38, position=[0.] * 38)
    rejects(lambda: B.vec((1., msg), MODEL_JOINT_NAMES), "重复")
    msg = SimpleNamespace(name=list(MODEL_JOINT_NAMES), position=[0.] * 37)
    rejects(lambda: B.vec((1., msg), MODEL_JOINT_NAMES), "长度不一致")
    print("  [13] NaN/Inf/float32 溢出/重名/长度不符均被拒绝 ✓")

    # ---- 14) --pack 不能绕过标签和时间轴检查 ----
    bad_pack = tmp / "pack_invalid"
    (bad_pack / B.NPZ_DIR).mkdir(parents=True)
    arrays = np.zeros((3, B.N), dtype=np.float32)
    pack_path = bad_pack / B.NPZ_DIR / "episode_000000.npz"
    cases = [(np.array([0., 1 / FPS, 1.]), arrays, arrays, "时间断点"),
             (np.array([0., 0., 1 / FPS]), arrays, arrays, "时间断点"),
             (np.arange(3) / FPS, arrays + np.nan, arrays, "非有限"),
             (np.arange(3) / FPS, arrays, arrays + np.inf, "非有限")]
    for stamps, states, actions, text in cases:
        np.savez_compressed(pack_path, state=states, action=actions, stamp=stamps,
                            fps=FPS, episode_index=0)
        rejects(lambda: B.write_parquet(bad_pack, 0, 0), text)
        assert not (bad_pack / B.DATA_DIR).exists()
    # 真实相机的小幅抖动不应误判为缺帧。
    B.validate_episode_data(arrays, arrays, np.array([0., .0334, .0667]), FPS, "jitter")
    print("  [14] --pack 拒绝坏时间轴和非有限标签，正常小幅抖动通过 ✓")

    check_dynamic_video_metadata(tmp)

    # 全过 → 清掉临时目录；任何断言失败都会在此之前抛错，目录保留供排查
    shutil.rmtree(tmp, ignore_errors=True)
    print("\nPASS（临时目录已清理；失败时会保留）")
    return 0

if __name__ == "__main__":
    sys.exit(main())
