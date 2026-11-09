from setuptools import setup

package_name = "q5_hand_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # 原样安装启动脚本，避免 develop/console_scripts 生成依赖 distribution/pkg_resources
        # 的包装器（沿用 vive_dual_arm_teleop 的惯例）。
        ("lib/" + package_name, ["scripts/hand_bridge"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaodan",
    maintainer_email="xiaodan@example.com",
    description="XHand 桥：TCP（手套）-> /hand_controller/commands",
    license="Proprietary",
)
