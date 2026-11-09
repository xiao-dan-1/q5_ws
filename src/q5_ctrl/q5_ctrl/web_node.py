"""web_node -- 浏览器逐关节控制 Q5，用 RobotEra SDK。

继承 RobotController（rclpy.Node）：
  身体（14 臂 + 6 身体关节）→ 自实现斜坡插值（全 20 关节 + kp=100/kd=2；
      TrajectoryController.move_to_joint_positions 发 48 关节 + 空增益，本机不认）
  手（24 关节）            → RobotController.control_hand_joints（/hand_controller/commands）

HTTP 端点（内嵌 stdlib http.server，后台线程）：
  GET  /          → 网页：44 关节滑杆 + 读取当前 + 发布
  GET  /joints    → JSON：关节分组 + 限位
  GET  /state     → JSON：当前关节角（get_joint_states 缓存）
  POST /publish   → {"joints": {关节名: 值}, "duration": 秒} → 身体走斜坡、手走 control_hand_joints

限位：臂=MANUAL_ARM_LIMITS；身体=MANUAL_BODY_EXTRA_LIMITS（手册第 4 页）；手=±1.75 闸门。

依赖 xbot_sdk_api（RobotController，只在机器人上）。
用法：ros2 run q5_ctrl web_node --ros-args -p port:=11451
浏览器开 http://<IP>:11451/
"""
from __future__ import annotations

import json
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

sys.path.insert(0, "/home/developer/xbot_sdk_api")  # RobotController 在机器人上

import rclpy  # noqa: E402
from robot_controller import RobotController  # noqa: E402
from xbot_common_interfaces.msg import HybridJointCommand  # noqa: E402

from .q5_joint_limits import (  # noqa: E402
    ARM_JOINT_NAMES,
    BODY_EXTRA_JOINT_NAMES,
    BODY_JOINT_NAMES,
    HAND_JOINT_NAMES,
    MANUAL_ARM_LIMITS,
    MANUAL_BODY_EXTRA_LIMITS,
    MANUAL_HAND_LIMITS,
)

# 身体 6 关节（手册限位，见 q5_joint_limits）
BODY_LIMITS = MANUAL_BODY_EXTRA_LIMITS


def _limit(name):
    return (MANUAL_ARM_LIMITS.get(name)
            or BODY_LIMITS.get(name)
            or MANUAL_HAND_LIMITS.get(name)
            or (-1.0, 1.0))


JOINT_GROUPS = [
    ("左臂", list(ARM_JOINT_NAMES[:7])),
    ("右臂", list(ARM_JOINT_NAMES[7:14])),
    ("身体", list(BODY_EXTRA_JOINT_NAMES)),
    ("左手", list(HAND_JOINT_NAMES[:12])),
    ("右手", list(HAND_JOINT_NAMES[12:24])),
]

JOINTS_CONFIG = [
    {"group": g, "joints": [{"name": n, "min": _limit(n)[0], "max": _limit(n)[1]}
                            for n in names]}
    for g, names in JOINT_GROUPS
]

HTML_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Q5 关节控制</title>
<style>
  :root { --accent:#2f6fed; --bg:#f4f5f7; --card:#fff; --muted:#8a8f98; --line:#e6e8ec; }
  * { box-sizing:border-box; }
  body { margin:0; font-family:-apple-system,"PingFang SC","Microsoft YaHei",sans-serif;
         background:var(--bg); color:#1f2328; }
  .toolbar { position:sticky; top:0; z-index:10; background:var(--card);
             border-bottom:1px solid var(--line); padding:10px 20px;
             display:flex; align-items:center; gap:10px; }
  .toolbar h1 { font-size:16px; font-weight:600; margin:0; }
  .toolbar .hint { font-size:12px; color:var(--muted); }
  .toolbar input[type=number] { width:60px; padding:4px 6px; border:1px solid #d0d4da; border-radius:6px; }
  .toolbar .spacer { flex:1; }
  .toolbar button { padding:6px 14px; border:none; border-radius:6px; font-size:13px; cursor:pointer; }
  .toolbar .load { background:#eef2f8; color:var(--accent); }
  .toolbar .pub  { background:var(--accent); color:#fff; }
  #result { font-size:13px; color:#1a9e4b; }
  .wrap { max-width:960px; margin:0 auto; padding:16px; }
  .group { background:var(--card); border-radius:10px; margin-bottom:14px;
           box-shadow:0 1px 3px rgba(0,0,0,.06); overflow:hidden; }
  .group h3 { margin:0; padding:10px 16px; font-size:13px; font-weight:600;
              color:#fff; background:var(--accent); letter-spacing:2px; }
  .joints { padding:4px 16px 8px; }
  .joint-row { display:flex; align-items:center; gap:10px; padding:6px 0; }
  .joint-row label { width:210px; flex:none; font-size:12px;
                     font-family:ui-monospace,SFMono-Regular,monospace; color:#3a4048; }
  .track { position:relative; flex:1; height:22px; }
  .rail { position:absolute; top:50%; transform:translateY(-50%); left:0; right:0;
          height:4px; background:#e0e3e8; border-radius:2px; }
  .fill { position:absolute; top:50%; transform:translateY(-50%); left:0; width:0;
          height:4px; background:var(--accent); border-radius:2px; }
  .ball { position:absolute; top:50%; transform:translate(-50%,-50%); left:0;
          width:14px; height:14px; border-radius:50%; background:var(--accent);
          border:2px solid #fff; box-shadow:0 1px 3px rgba(0,0,0,.3); }
  .num { width:52px; flex:none; text-align:right; font-size:11px;
         font-family:ui-monospace,SFMono-Regular,monospace; }
  .cur { color:var(--muted); }
  .tgt { color:var(--accent); font-weight:600; }
</style>
</head>
<body>
<div class="toolbar">
  <h1>Q5 关节控制</h1>
  <span class="hint">斜坡</span>
  <input id="duration" type="number" value="1.0" step="0.5" min="0.1">
  <span class="hint">s</span>
  <div class="spacer"></div>
  <button class="load" onclick="resetTargets()">目标=当前</button>
  <button class="pub" onclick="publish()">发布</button>
  <span id="result"></span>
</div>
<div class="wrap"><div id="joints"></div></div>
<script>
const targets = {};

async function loadJoints() {
  const groups = await (await fetch("/joints")).json();
  const c = document.getElementById("joints");
  c.innerHTML = "";
  for (const g of groups) {
    const card = document.createElement("div"); card.className = "group";
    const h = document.createElement("h3"); h.textContent = g.group; card.appendChild(h);
    const box = document.createElement("div"); box.className = "joints";
    for (const j of g.joints) box.appendChild(makeJoint(j));
    card.appendChild(box); c.appendChild(card);
  }
  await loadCurrent();
  setInterval(loadCurrent, 100);
}

function makeJoint(j) {
  const row = document.createElement("div"); row.className = "joint-row";
  row.innerHTML = "<label></label><div class='track'><div class='rail'></div><div class='fill'></div><div class='ball'></div></div><span class='num cur'></span><span class='num tgt'></span>";
  const track = row.querySelector(".track"), fill = row.querySelector(".fill"),
        ball = row.querySelector(".ball"), cur = row.querySelector(".cur"),
        tgt = row.querySelector(".tgt");
  row.querySelector("label").textContent = j.name;
  row._name = j.name;
  const pct = v => Math.max(0, Math.min(100, (v - j.min) / (j.max - j.min) * 100));

  row._setCurrent = v => { fill.style.width = pct(v) + "%"; cur.textContent = (+v).toFixed(2); };
  row._initTarget = v => { targets[j.name] = v; ball.style.left = pct(v) + "%"; tgt.textContent = (+v).toFixed(2); };

  let dragging = false;
  const val = e => {
    const r = track.getBoundingClientRect();
    const x = (e.touches ? e.touches[0].clientX : e.clientX) - r.left;
    return j.min + Math.max(0, Math.min(1, x / r.width)) * (j.max - j.min);
  };
  const down = e => { dragging = true; row._initTarget(val(e)); e.preventDefault(); };
  ball.addEventListener("mousedown", down);
  track.addEventListener("mousedown", down);
  window.addEventListener("mousemove", e => { if (dragging) row._initTarget(val(e)); });
  window.addEventListener("mouseup", () => dragging = false);
  ball.addEventListener("touchstart", down);
  track.addEventListener("touchstart", down);
  window.addEventListener("touchmove", e => { if (dragging) row._initTarget(val(e)); });
  window.addEventListener("touchend", () => dragging = false);

  return row;
}

async function loadCurrent() {
  const s = await (await fetch("/state")).json();
  for (const row of document.querySelectorAll(".joint-row")) {
    const v = s[row._name];
    if (v === undefined) continue;
    row._setCurrent(v);
    if (targets[row._name] === undefined) row._initTarget(v);
  }
}

function resetTargets() { for (const k in targets) delete targets[k]; loadCurrent(); }

async function publish() {
  const joints = {};
  for (const name in targets) joints[name] = targets[name];
  const duration = parseFloat(document.getElementById("duration").value) || 1.0;
  const r = await fetch("/publish", {method:"POST", body:JSON.stringify({joints, duration})});
  const res = await r.json();
  document.getElementById("result").textContent = res.ok ? "已发布 ✓" : res.msg;
}

loadJoints();
</script>
</body>
</html>"""


def _local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        s.close()


class WebNode(RobotController):
    def __init__(self):
        super().__init__("web_node")
        self.declare_parameter("port", 11451)
        port = int(self.get_parameter("port").value)
        self._body_pub = self.create_publisher(
            HybridJointCommand, "/wr1_controller/commands", 10)
        HTTPServer.allow_reuse_address = True  # 重启时 socket 若在 TIME_WAIT 也能复用
        self._server = HTTPServer(("0.0.0.0", port), make_handler(self))
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        self.get_logger().info("web_node 已就绪：http://%s:%d/  （/joints /state /publish）"
                               % (_local_ip(), port))

    def state_dict(self):
        js = self.get_joint_states()
        if js is None:
            return {}
        return {n: float(p) for n, p in zip(js.name, js.position)}

    def publish_joints(self, targets, duration=1.0):
        """targets: {关节名: 值}。身体走斜坡、手走 control_hand_joints。返回 (ok, msg)。"""
        if not targets:
            return False, "空目标"
        for n, v in targets.items():
            if v != v or v in (float("inf"), float("-inf")):
                return False, "关节 %s 非有限值" % n

        body = {n: float(v) for n, v in targets.items() if n in BODY_JOINT_NAMES}
        hand_names = [n for n in HAND_JOINT_NAMES if n in targets]
        hand_vals = [float(targets[n]) for n in hand_names]

        if body:
            self._ramp_body(body, duration)
        if hand_names:
            if not self.control_hand_joints(hand_names, hand_vals):
                return False, "手控制失败"
        return True, "已发布 %d 关节（斜坡 %.1fs）" % (len(targets), duration)

    def _ramp_body(self, targets, duration):
        """身体斜坡（复刻 TrajectoryController 的线性插值），发全 20 关节 + kp=100/kd=2。

        TrajectoryController.move_to_joint_positions 发 /joint_states 全 48 关节 + 空增益，
        本机 wr1_controller 不认（零响应）。这里只改消息：20 关节 + 显式增益。
        """
        js = self.get_joint_states()
        if js is None:
            return
        state = {n: p for n, p in zip(js.name, js.position)}
        start = [state.get(n, 0.0) for n in BODY_JOINT_NAMES]
        end = [targets.get(n, start[i]) for i, n in enumerate(BODY_JOINT_NAMES)]

        def _run():
            rate = 50.0
            steps = max(1, int(duration * rate))
            for i in range(steps + 1):
                alpha = i / steps
                msg = HybridJointCommand()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.joint_name = list(BODY_JOINT_NAMES)
                msg.position = [float(s + (e - s) * alpha) for s, e in zip(start, end)]
                msg.velocity = [0.0] * len(BODY_JOINT_NAMES)
                msg.feedforward = [0.0] * len(BODY_JOINT_NAMES)
                msg.kp = [100.0] * len(BODY_JOINT_NAMES)
                msg.kd = [2.0] * len(BODY_JOINT_NAMES)
                self._body_pub.publish(msg)
                time.sleep(duration / steps)

        threading.Thread(target=_run, daemon=True).start()


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, ctype, payload):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _json(self, data):
            self._send(200, "application/json", json.dumps(data).encode())

        def do_GET(self):
            if self.path == "/state":
                self._json(node.state_dict())
            elif self.path == "/joints":
                self._json(JOINTS_CONFIG)
            else:
                self._send(200, "text/html; charset=utf-8", HTML_PAGE.encode())

        def do_POST(self):
            if self.path == "/publish":
                length = int(self.headers.get("Content-Length", 0))
                try:
                    req = json.loads(self.rfile.read(length))
                    ok, msg = node.publish_joints(req.get("joints", {}),
                                                  float(req.get("duration", 1.0)))
                    self._json({"ok": ok, "msg": msg})
                except Exception as exc:
                    self._json({"ok": False, "msg": str(exc)})
            else:
                self._json({"ok": False, "msg": "unknown path"})

        def log_message(self, *args):
            pass

    return Handler


def main():
    rclpy.init()
    node = WebNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node._server.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
