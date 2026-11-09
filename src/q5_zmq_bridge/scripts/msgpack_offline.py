"""离线环境 msgpack 导入：优先系统包，回退 pip 内置 vendor。"""

from __future__ import annotations

import site
import sys
from pathlib import Path


def import_msgpack():
    try:
        import msgpack

        return msgpack
    except ModuleNotFoundError:
        pass

    vendor_roots = [
        Path(path) / "pip" / "_vendor"
        for path in site.getsitepackages() + [site.getusersitepackages()]
    ]
    vendor_roots.extend(
        [
            Path("/usr/lib/python3/dist-packages/pip/_vendor"),
            Path("/usr/local/lib/python3.10/dist-packages/pip/_vendor"),
        ]
    )
    for vendor_root in vendor_roots:
        if (vendor_root / "msgpack").is_dir():
            sys.path.insert(0, str(vendor_root))
            break

    import msgpack

    return msgpack


msgpack = import_msgpack()
