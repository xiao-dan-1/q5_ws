"""Pure contract/safety logic for the A1 dry-run build.

No ROS imports here, so the unit tests run without rclpy.
Motion thresholds are reported, not enforced by stopping: this build never publishes.
"""

from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np

from q5_ctrl.q5_joint_limits import (
    ARM_JOINT_NAMES,
    MANUAL_ARM_LIMITS,
    MODEL_JOINT_NAMES,
)

MODEL_ACTION_SHAPE = (10, 38)
ARM_COUNT = len(ARM_JOINT_NAMES)
_EPS = 1e-6


class ContractError(ValueError):
    """The model input or output does not match the fixed contract."""


class SafetyConfigError(ValueError):
    """The safety profile is missing or inconsistent with q5_ctrl."""


def assert_dry_run_only(enable_motion, motion_approved, publish_hands) -> None:
    """This build has no publisher; any motion opt-in is a startup error."""
    if enable_motion or motion_approved or publish_hands:
        raise RuntimeError(
            "This build is dry-run only; motion publishing is disabled.")


def command_hz(execute_steps: int, inference_hz: float, control_hz: float) -> float:
    """Target update rate used by the safety math.

    One target per chunk is refreshed at inference cadence; a multi-step chunk
    would be consumed at control cadence.
    """
    if int(execute_steps) != execute_steps or execute_steps < 1:
        raise ValueError(f"execute_steps must be a positive integer, got {execute_steps!r}")
    hz = inference_hz if execute_steps == 1 else control_hz
    hz = float(hz)
    if not math.isfinite(hz) or hz <= 0.0:
        raise ValueError(f"command rate must be positive, got {hz!r}")
    return hz


def load_safety_config(path: str | Path) -> dict[str, Any]:
    """Load the profile; `joints` is the training-dataset envelope, not a hard limit.

    Two independent position layers, as in the predecessor's interface:
    `MANUAL_ARM_LIMITS` (q5_ctrl) is the mechanical gate for the 14 arms, while a
    `joints` block here describes where the checkpoint's training data lived. The
    envelope may be wider or narrower than the manual limits; a packet must clear
    both. Without a `joints` block the profile keeps its old arms-only behaviour
    and the envelope check is skipped.
    """
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("schema_version") != 1:
        raise SafetyConfigError("safety config schema_version must be 1")
    groups = raw.get("groups")
    if not isinstance(groups, Mapping):
        raise SafetyConfigError("safety config missing groups")
    _check_group(groups.get("arms"), "groups.arms")
    hands_group = groups.get("hands")
    if hands_group is not None:
        _check_group(hands_group, "groups.hands")

    supplied = raw.get("joints")
    if supplied is not None and not isinstance(supplied, Mapping):
        raise SafetyConfigError("safety config joints must be an object")
    joints = dict(supplied or {})
    for name, entry in joints.items():
        if name not in MODEL_JOINT_NAMES:
            raise SafetyConfigError(f"joints.{name} is not a model joint")
        if not isinstance(entry, Mapping):
            raise SafetyConfigError(f"joints.{name} must be an object")
        lower, upper = entry.get("min_position"), entry.get("max_position")
        if not (_is_finite(lower) and _is_finite(upper) and lower < upper):
            raise SafetyConfigError(
                f"joints.{name} needs finite min_position < max_position")
    for name, (lower, upper) in MANUAL_ARM_LIMITS.items():
        joints.setdefault(name, {"min_position": lower, "max_position": upper})

    config = dict(raw)
    config["groups"] = dict(groups)
    config["joints"] = joints
    config["envelope_supplied"] = bool(supplied)
    return config


def missing_motion_safety_fields(
    config: Mapping[str, Any],
    *,
    arms_only: bool = False,
) -> list[str]:
    """List fields that would be required before a motion build could start."""
    missing: list[str] = []
    groups = config.get("groups", {})
    if not isinstance(groups, Mapping):
        return ["groups"]
    required_groups = ("arms",) if arms_only else ("arms", "hands")
    for group_name in required_groups:
        group = groups.get(group_name)
        if not isinstance(group, Mapping):
            missing.append(f"groups.{group_name}")
            continue
        for field in ("position_margin", "max_step", "max_velocity", "max_acceleration"):
            value = group.get(field)
            if not _is_nonnegative(value, positive=field != "position_margin"):
                missing.append(f"groups.{group_name}.{field}")

    joints = config.get("joints", {})
    if not isinstance(joints, Mapping):
        return missing + ["joints"]
    required_joints = ARM_JOINT_NAMES if arms_only else MODEL_JOINT_NAMES
    for joint_name in required_joints:
        limits = joints.get(joint_name)
        if not isinstance(limits, Mapping):
            missing.append(f"joints.{joint_name}")
            continue
        lower = limits.get("min_position")
        upper = limits.get("max_position")
        if not _is_finite(lower):
            missing.append(f"joints.{joint_name}.min_position")
        if not _is_finite(upper):
            missing.append(f"joints.{joint_name}.max_position")
        if _is_finite(lower) and _is_finite(upper) and lower >= upper:
            missing.append(f"joints.{joint_name}.min_position<max_position")
    return missing


def validate_actions(actions: Any) -> np.ndarray:
    """Require a (10, 38) finite float array."""
    actions = np.asarray(actions, dtype=np.float32)
    if actions.shape != MODEL_ACTION_SHAPE:
        raise ContractError(
            f"actions must be {MODEL_ACTION_SHAPE}, got {actions.shape}")
    if not np.isfinite(actions).all():
        raise ContractError("actions contain NaN or Inf")
    return actions


@dataclass(frozen=True)
class ValidationReport:
    """Quantified result of one dry-run action check."""

    first_step_jump: float
    max_step: float
    max_velocity: float
    max_acceleration: float
    would_reject: tuple[str, ...] = ()
    hard_limit_violations: tuple[str, ...] = ()
    envelope_violations: tuple[str, ...] = ()


def validate_action_chunk(
    actions: Any,
    current_arm_positions: Sequence[float],
    *,
    command_hz: float,
    config: Mapping[str, Any],
    execute_steps: int = 1,
) -> ValidationReport:
    """Check a chunk for the dry-run observer.

    Raises ContractError for shape/NaN. Hard-limit, envelope and dynamic
    violations are returned in the report so the caller can keep the run alive
    and mark them. Dynamic limits apply only to the executed prefix, including
    the jump from the latest feedback to the first target. The envelope is
    checked on the target chunk only; the current state is bounded indirectly by
    the step limit, and the model echoes state, so an out-of-domain pose shows up
    here as an out-of-domain target.
    """
    actions = validate_actions(actions)
    if not 1 <= int(execute_steps) <= MODEL_ACTION_SHAPE[0]:
        raise ValueError(f"execute_steps out of range: {execute_steps!r}")
    execute_steps = int(execute_steps)
    hz = float(command_hz)
    if not math.isfinite(hz) or hz <= 0.0:
        raise ValueError(f"command_hz must be positive, got {command_hz!r}")

    current = np.asarray(current_arm_positions, dtype=np.float32)
    if current.shape != (ARM_COUNT,):
        raise ContractError(
            f"current_arm_positions must be ({ARM_COUNT},), got {current.shape}")
    if not np.isfinite(current).all():
        raise ContractError("current_arm_positions contain NaN or Inf")

    arms_group = config["groups"]["arms"]
    hands_group = config["groups"].get("hands")
    joints = config["joints"]

    hard: dict[str, str] = {}
    for index, joint_name in enumerate(ARM_JOINT_NAMES):
        values = actions[:, index]
        actual_min = float(values.min())
        actual_max = float(values.max())
        lower, upper = MANUAL_ARM_LIMITS[joint_name]
        if actual_min < lower or actual_max > upper:
            hard[joint_name] = (
                f"{joint_name} range [{actual_min:.4f}, {actual_max:.4f}] "
                f"outside manual [{lower}, {upper}]")

    envelope: dict[str, str] = {}
    if config.get("envelope_supplied"):
        for index, joint_name in enumerate(MODEL_JOINT_NAMES):
            limits = joints.get(joint_name)
            if limits is None:
                continue
            group = arms_group if index < len(ARM_JOINT_NAMES) else hands_group
            margin = float(group["position_margin"]) if group is not None else 0.0
            values = actions[:, index]
            actual_min = float(values.min())
            actual_max = float(values.max())
            safe_lower = float(limits["min_position"]) + margin
            safe_upper = float(limits["max_position"]) - margin
            if actual_min < safe_lower or actual_max > safe_upper:
                envelope[joint_name] = (
                    f"{joint_name} range [{actual_min:.4f}, {actual_max:.4f}] "
                    f"outside dataset envelope [{safe_lower:.4f}, {safe_upper:.4f}]")

    prefix = actions[:execute_steps, :ARM_COUNT]
    stacked = np.vstack([current.reshape(1, ARM_COUNT), prefix])
    deltas = np.diff(stacked, axis=0)
    velocities = deltas * hz
    accelerations = (
        np.diff(np.vstack([np.zeros((1, ARM_COUNT), dtype=np.float32), velocities]), axis=0)
        * hz
    )

    would_reject: list[str] = []
    limits = (
        ("step", deltas, float(arms_group["max_step"])),
        ("velocity", velocities, float(arms_group["max_velocity"])),
        ("acceleration", accelerations, float(arms_group["max_acceleration"])),
    )
    for index, joint_name in enumerate(ARM_JOINT_NAMES):
        for label, values, limit in limits:
            peak = float(np.max(np.abs(values[:, index])))
            if peak > limit + _EPS:
                would_reject.append(
                    f"{joint_name} {label} {peak:.6f} > {limit:.6f}")

    return ValidationReport(
        first_step_jump=float(np.max(np.abs(deltas[0]))),
        max_step=float(np.max(np.abs(deltas))),
        max_velocity=float(np.max(np.abs(velocities))),
        max_acceleration=float(np.max(np.abs(accelerations))),
        would_reject=tuple(would_reject),
        hard_limit_violations=tuple(hard.values()),
        envelope_violations=tuple(envelope.values()),
    )


def is_stale_response(
    rtt: float,
    observation_age: float,
    *,
    first_success: bool,
    max_round_trip: float,
    max_observation_to_action_age: float,
) -> bool:
    """The first response may take as long as JAX compilation; later ones may not."""
    if not first_success:
        return False
    return rtt > max_round_trip or observation_age > max_observation_to_action_age


class FatalLatch:
    """Process-wide fatal flag. Workers call fail(); the main thread shuts down."""

    def __init__(self) -> None:
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._reason: str | None = None

    def fail(self, reason: str) -> bool:
        with self._lock:
            if self._event.is_set():
                return False
            self._reason = str(reason)
            self._event.set()
            return True

    def is_set(self) -> bool:
        return self._event.is_set()

    @property
    def reason(self) -> str | None:
        with self._lock:
            return self._reason

    def wait(self, timeout: float | None = None) -> bool:
        return self._event.wait(timeout)


class FailureTracker:
    """Independent consecutive counters for transport and contract errors."""

    def __init__(self, max_consecutive: int = 5) -> None:
        if max_consecutive < 1:
            raise ValueError("max_consecutive must be >= 1")
        self.max_consecutive = int(max_consecutive)
        self.protocol_failures = 0
        self.contract_failures = 0

    def record_protocol_error(self) -> bool:
        self.protocol_failures += 1
        return self.protocol_failures >= self.max_consecutive

    def record_contract_error(self) -> bool:
        self.contract_failures += 1
        return self.contract_failures >= self.max_consecutive

    def record_success(self) -> None:
        self.protocol_failures = 0
        self.contract_failures = 0


def exit_code(fatal: bool) -> int:
    """Fatal errors exit 1; run_seconds expiry exits 0."""
    return 1 if fatal else 0


def _check_group(group: Any, label: str) -> None:
    if not isinstance(group, Mapping):
        raise SafetyConfigError(f"safety config missing {label}")
    for field in ("position_margin", "max_step", "max_velocity", "max_acceleration"):
        value = group.get(field)
        if not _is_nonnegative(value, positive=field != "position_margin"):
            raise SafetyConfigError(f"{label}.{field} must be a valid number")


def _is_finite(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _is_nonnegative(value: Any, *, positive: bool) -> bool:
    return _is_finite(value) and (value > 0 if positive else value >= 0)
