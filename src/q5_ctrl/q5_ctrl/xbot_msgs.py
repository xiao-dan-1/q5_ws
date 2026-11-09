"""WSL 上没有 xbot_common_interfaces 时的替身消息。

7 个字段与真机 ros2 interface show 出来的定义逐项一致，但没有 ROS 类型身份——
万一路径搞错发到机器人上，controller 也不认。这是刻意的：宁可发不出去，不要发错东西。
真机跑必须用真消息（Ubuntu-xd 没装 SDK 就得先装）。
"""


class _Header:
    def __init__(self):
        self.stamp = None
        self.frame_id = ""


class HybridJointCommand:
    def __init__(self):
        self.header = _Header()
        self.joint_name = []
        self.position = []
        self.velocity = []
        self.feedforward = []
        self.kp = []
        self.kd = []
