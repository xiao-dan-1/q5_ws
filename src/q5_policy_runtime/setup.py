from glob import glob

from setuptools import find_packages, setup

package_name = "q5_policy_runtime"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xiaodan",
    maintainer_email="xiaodan@rospc.local",
    description="Q5 pi0.5 policy inference node (A1 dry-run only)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "policy_node = q5_policy_runtime.policy_node:main",
        ],
    },
)
