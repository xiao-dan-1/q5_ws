from setuptools import setup

package_name = "q5_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("lib/" + package_name, ["scripts/joint_states_relay"]),
        ("lib/" + package_name, ["scripts/camera_guard"]),
        ("lib/" + package_name, ["scripts/mjpeg_server"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaodan",
    maintainer_email="xiaodan@example.com",
    description="Q5 机器人侧常驻工具节点（joint_states 跨机中继、相机守卫、MJPEG 图像服务）",
    license="Proprietary",
)
