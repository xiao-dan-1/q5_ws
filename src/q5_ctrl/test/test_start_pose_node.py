"""start_pose_node 的纯函数：目标位姿文件、误差计算。"""

import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from q5_ctrl.start_pose_node import SIDES, load_target, pose_error

CONFIG = Path(__file__).resolve().parent.parent / "config" / "start_pose.json"


def _stamped(x, y, z, qx, qy, qz, qw):
    return SimpleNamespace(
        pose=SimpleNamespace(
            position=SimpleNamespace(x=x, y=y, z=z),
            orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
        )
    )


def test_config_has_three_locked_poses():
    frame_id, poses = load_target(CONFIG)
    assert frame_id == "base_link"
    assert set(poses) == set(SIDES)
    assert all(len(pose) == 7 for pose in poses.values())


def test_wrong_length_is_rejected(tmp_path):
    data = json.loads(CONFIG.read_text())
    data["left"] = data["left"][:6]
    path = tmp_path / "pose.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_target(str(path))


def test_unnormalized_quaternion_is_rejected(tmp_path):
    data = json.loads(CONFIG.read_text())
    data["head"] = data["head"][:3] + [0.0, 0.0, 0.0, 2.0]
    path = tmp_path / "pose.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_target(str(path))


def test_identical_pose_has_zero_error():
    target = (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0)
    assert pose_error(target, _stamped(*target)) == (0.0, 0.0)


def test_error_splits_position_and_rotation():
    target = (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0)
    # 位置偏 0.03，姿态绕 z 转 90°（xyzw = 0,0,sin45,cos45）
    distance, angle = pose_error(target, _stamped(0.1, 0.2, 0.33, 0.0, 0.0, 2 ** -0.5, 2 ** -0.5))
    assert distance == pytest.approx(0.03)
    assert angle == pytest.approx(1.5707963, rel=1e-6)
