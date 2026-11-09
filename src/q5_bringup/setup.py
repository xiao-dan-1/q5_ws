import os
from glob import glob

from setuptools import setup

package_name = "q5_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaodan",
    maintainer_email="xiaodan@example.com",
    description="Q5 启动编排：把 start_xiaodan_mpc.sh 的 7 步流程交给 ros2 launch 托管",
    license="Proprietary",
)
