from setuptools import setup

package_name = "xbot_sdk_api"

setup(
    name=package_name,
    version="0.1.0",
    packages=["robot_controller"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaodan",
    maintainer_email="xiaodan@example.com",
    description="Vendor robot control SDK (robot_controller)",
    license="Proprietary",
)
