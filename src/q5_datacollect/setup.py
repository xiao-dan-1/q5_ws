from setuptools import setup

package_name = "q5_datacollect"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 都装到 lib/q5_datacollect 同一目录：bag2lerobot 才能 import q5_joint_limits，
        # 且 ros2 run q5_datacollect <文件名> 就是入口（文件名 = 命令名，故去掉 .py）
        ("lib/" + package_name, [
            "scripts/record",           # 录一集（4 话题契约 + 目录约定 + 开录前检查）
            "scripts/convert",          # 转一个任务（路径/任务名/state 全自动，跟 record 对称）
            "scripts/bag2lerobot",      # 底层：bag -> LeRobot v2.1（全控制入口）
            "scripts/preflight",        # 录制前检查（只读，看速率/QoS 的全套）
            "scripts/q5_joint_limits.py",   # 上面两个 import 的 38 维关节契约
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaodan",
    maintainer_email="xiaodan@example.com",
    description="Q5 数采流水线：录制前检查 + bag2lerobot 数据集转换",
    license="Proprietary",
)
