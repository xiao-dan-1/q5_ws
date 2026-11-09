#!/usr/bin/env python3
"""自检：每个 package:// 网格都在包里、根 link 唯一。

    python3 scripts/check_description.py          # 在源码目录里跑
    ros2 run q5_description check_description.py # 装好之后跑
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def package_root():
    """源码目录用 <pkg>/，装好之后用 install/share/q5_description 加 ament 索引。"""
    here = Path(__file__).resolve()
    if (here.parents[1] / "package.xml").is_file():
        return here.parents[1]
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory("q5_description"))
    except Exception:                                   # 裸拷贝、没 source 环境
        return here.parents[1]


def main():
    package = package_root()
    urdf_dir = package / "urdf"
    files = sorted(urdf_dir.glob("*.urdf"))
    if not files:
        raise SystemExit(f"没有找到 URDF：{urdf_dir}（先跑 scripts/import_model.py）")

    failed = False
    for urdf in files:
        root = ET.parse(urdf).getroot()
        meshes = root.findall(".//mesh")
        missing = []
        for mesh in meshes:
            filename = mesh.get("filename")
            if not filename.startswith("package://q5_description/"):
                missing.append(f"{filename} (未用 package:// 前缀)")
                continue
            path = package / filename[len("package://q5_description/"):]
            if not path.is_file():
                missing.append(filename)
        links = [link.get("name") for link in root.findall("link")]
        children = {j.find("child").get("link") for j in root.findall("joint")}
        roots = [name for name in links if name not in children]
        print(f"{urdf.name}: links={len(links)} joints={len(root.findall('joint'))} "
              f"meshes={len(meshes)} 根 link={roots}")
        if len(roots) != 1 or roots != ["base_link"]:
            print(f"   根 link 不是唯一的 base_link：{roots}")
            failed = True
        if missing:
            failed = True
            print(f"   缺少 {len(missing)} 个网格，例如 {missing[:3]}")
    if failed:
        return 1
    print("OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
