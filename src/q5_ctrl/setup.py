from setuptools import find_packages, setup

package_name = "q5_ctrl"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/start_pose.json"]),
        ("share/" + package_name + "/launch", ["launch/start_pose.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaodan",
    maintainer_email="xiaodan@rospc.local",
    description="Q5 关节命令发布器（真机 + WSL/rospc 端）：38 维姿态 -> HybridJointCommand",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # WSL 与真机都跑
            "posture_node = q5_ctrl.posture_node:main",
            "arm_move_node = q5_ctrl.arm_move_node:main",
            "start_pose_node = q5_ctrl.start_pose_node:main",
            "vive_static_pose_node = q5_ctrl.vive_static_pose_node:main",
            # 只能在真机跑（依赖厂商 SDK robot_controller / 硬 import xbot_common_interfaces）
            "keyboard_node = q5_ctrl.keyboard_node:main",
            "xbot_keyboard_node = q5_ctrl.xbot_keyboard_node:main",
            "web_node = q5_ctrl.web_node:main",
        ],
    },
)
