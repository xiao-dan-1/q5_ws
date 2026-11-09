"""自检：限速插值只做两件事——端点精确、比例不变。不碰 ROS 网络。"""
import pytest

from q5_ctrl.arm_move_node import plan_duration, ramp


def test_ramp_endpoints_exact():
    start = [0.0, -1.0, 0.5]
    target = [1.0, 0.0, -0.5]
    assert ramp(start, target, 0.0) == start
    assert ramp(start, target, 1.0) == target
    # 越界的 alpha 夹住，不会冲过头
    assert ramp(start, target, 2.0) == target
    assert ramp(start, target, -1.0) == start


def test_ramp_preserves_ratio():
    """所有关节共用 alpha：任意时刻位移比例 = 终态比例。

    肘偏航/腕滚这类互相抵消的轴靠这个性质，中途不会被拉开。
    """
    start = [0.0, 0.0]
    target = [1.0, -0.25]
    for alpha in (0.1, 0.25, 0.5, 0.9):
        a, b = ramp(start, target, alpha)
        assert b == pytest.approx(-0.25 * a)
    # 换个量级也一样
    mid = ramp([0.3, -0.2], [0.9, -0.5], 0.4)
    assert (mid[1] - (-0.2)) == pytest.approx(0.4 * (-0.5 - (-0.2)))


def test_plan_duration_uses_max_joint_delta():
    assert plan_duration([0.0, 0.0], [0.3, -1.5], 0.15) == pytest.approx(10.0)
    # 极小移动有下限，不会算成 0 秒
    assert plan_duration([0.0], [0.001], 0.15) == pytest.approx(0.5)
    with pytest.raises(ValueError):
        plan_duration([0.0], [1.0], 0.0)
