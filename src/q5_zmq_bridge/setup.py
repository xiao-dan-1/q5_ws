from setuptools import setup

package_name = "q5_zmq_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 两个文件必须装进同一个目录：zmq_server 用 `from msgpack_offline import
        # msgpack`，依赖的正是"脚本所在目录在 sys.path 上"。分开装就没法 import。
        ("lib/" + package_name, [
            "scripts/zmq_server",
            "scripts/msgpack_offline.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaodan",
    maintainer_email="xiaodan@example.com",
    description="Q5 观测/动作 ZMQ 桥：ROS2 <-> RoboDriver",
    license="Proprietary",
)
