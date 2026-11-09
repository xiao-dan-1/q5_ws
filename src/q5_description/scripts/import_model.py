#!/usr/bin/env python3
"""把 Q5work 仓库里重建好的 Q5 模型搬成这个包自己的 urdf/ 和 meshes/。

    python3 scripts/import_model.py [Q5work 仓库根目录]

默认源目录是 WSL 下的 /mnt/c/.../Q5work/urdf/robotsfan_q5。只复制 URDF 真正
引用到的网格，并把网格路径从相对路径改写成 package://q5_description/...，
这样包是自包含的、能直接 colcon build。
"""
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = Path(
    "/mnt/c/Users/jinbangguo/Desktop/xiaodan/home/06_projects/Q5work/urdf/robotsfan_q5"
)
VARIANTS = ("q5_reference.urdf", "q5_reference_xhand.urdf")
PREFIX = "package://q5_description/"


def main():
    source = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else DEFAULT_SOURCE
    if not (source / "q5_reference.urdf").is_file():
        raise SystemExit(f"找不到源模型：{source}")

    (PACKAGE / "urdf").mkdir(exist_ok=True)
    copied, total = set(), 0
    for name in VARIANTS:
        urdf = source / name
        text = urdf.read_text(encoding="utf-8")
        # 去掉 XML 声明：robot_description 是字符串参数，lxml 系的工具
        # （gazebo_ros 的 spawn_entity.py，-topic 和 -file 两条路都会）遇到
        # "Unicode strings with encoding declaration" 会直接报错。
        if text.lstrip().startswith("<?xml"):
            text = text.split("?>", 1)[1].lstrip()
        text = text.replace('filename="meshes/', f'filename="{PREFIX}meshes/')
        (PACKAGE / "urdf" / name).write_text(text, encoding="utf-8")
        for mesh in ET.parse(urdf).getroot().findall(".//mesh"):
            relative = mesh.get("filename")
            target = PACKAGE / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source / relative, target)
            if relative not in copied:
                copied.add(relative)
                total += (source / relative).stat().st_size
        print(f"{name}: {len(urdf.read_text(encoding='utf-8'))} bytes 已改写网格路径")

    print(f"网格 {len(copied)} 个，{total / 1024 / 1024:.2f} MB -> {PACKAGE / 'meshes'}")


if __name__ == "__main__":
    main()
