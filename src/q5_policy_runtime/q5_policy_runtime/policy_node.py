#!/usr/bin/env python3
"""Q5 pi0.5 policy node -- A1 dry-run only.

Subscribes /joint_states and the head camera (sensor_msgs/CompressedImage), sends
the flat xiaodan observation to the OpenPI server, validates the (10,38) response,
and logs it. There is no command publisher and no motion executor in this build.
"""

from __future__ import annotations

import os
import sys
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState

from .observation import (
    ObservationStore,
    decode_compressed_image,
    stamp_seconds,
    timestamp_skew,
)
from .openpi_client import OpenPiSession, ProtocolError, SessionStopped
from .safety import (
    ARM_COUNT,
    ContractError,
    FailureTracker,
    FatalLatch,
    assert_dry_run_only,
    command_hz,
    exit_code,
    is_stale_response,
    load_safety_config,
    validate_action_chunk,
)

try:
    from xbot_common_interfaces.msg import RobotStatus
except Exception:  # WSL dry-run without the vendor workspace
    RobotStatus = None

# Initial values from the 2026-09-15 interface contract; confirm with
# `ros2 topic info -v` before A1. A BEST_EFFORT subscriber accepts both
# BEST_EFFORT and RELIABLE publishers.
STATE_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
IMAGE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
STATUS_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

ROBOT_STATE_NAMES = {
    0: "INIT", 1: "SELF_TEST", 2: "IDLE", 3: "READY", 4: "ACTIVE",
    5: "SHUTDOWN", 6: "OTA", 7: "E_STOP", -1: "ERROR",
}


class PolicyNode(Node):
    def __init__(self) -> None:
        super().__init__("q5_policy")
        self._declare_parameters()
        # Hard boundary: run before any subscription/worker exists.
        assert_dry_run_only(
            self._enable_motion, self._motion_approved, self._publish_hands)
        self._validate_parameters()

        config_path = self._safety_config_path or os.path.join(
            get_package_share_directory("q5_policy_runtime"),
            "config",
            "safety.first-motion.json",
        )
        self._safety = load_safety_config(config_path)
        self._target_hz = command_hz(
            self._execute_steps, self._inference_hz, self._control_hz)

        self._store = ObservationStore()
        self._session = OpenPiSession(self._policy_host, self._policy_port)
        self._latch = FatalLatch()
        self._tracker = FailureTracker(self._max_consecutive_failures)
        self._stop_event = threading.Event()
        self._last_log: dict[str, float] = {}
        self._log_lock = threading.Lock()
        self._start_monotonic = time.monotonic()
        self._inference_thread: threading.Thread | None = None
        self._shutdown_done = False
        self._destroyed = False
        self._exit_code = 0
        self._stop_reason = "signal"
        self._last_status: int | None = None
        self._velocity_warned = False
        self._metadata_logged = False
        self._first_state_logged = False
        self._first_image_logged = False
        self._state_count = 0
        self._image_count = 0
        self._image_reject_count = 0
        self._last_input_stats = 0.0

        self._protocol_error_total = 0
        self._contract_error_total = 0
        self._would_reject_packets = 0
        self._hard_limit_packets = 0
        self._envelope_packets = 0
        self._inference_count = 0
        self._samples_ms: list[float] = []

        self.create_subscription(
            JointState, self._state_topic, self._on_state, STATE_QOS)
        self.create_subscription(
            CompressedImage, self._image_topic, self._on_image, IMAGE_QOS)
        if RobotStatus is not None:
            self.create_subscription(
                RobotStatus, self._status_topic, self._on_status, STATUS_QOS)
            status_qos = "RELIABLE+TRANSIENT_LOCAL depth 1"
        else:
            status_qos = "disabled (xbot_common_interfaces unavailable)"

        self.get_logger().info(
            f"dry-run build: no command publishers; safety={config_path}")
        self.get_logger().info(
            f"subscriptions: {self._state_topic} RELIABLE depth 5; "
            f"{self._image_topic} BEST_EFFORT depth 1; "
            f"{self._status_topic} {status_qos}")

    def _declare_parameters(self) -> None:
        def param(name, default):
            return self.declare_parameter(name, default).value

        self._policy_host = str(param("policy_host", "127.0.0.1"))
        self._policy_port = int(param("policy_port", 8123))
        self._prompt = str(param("prompt", "pick_orange"))

        self._state_topic = str(param("state_topic", "/joint_states"))
        self._image_topic = str(
            param("image_topic", "/camera/camera/color/image_raw/compressed"))
        self._status_topic = str(param("status_topic", "/xbot_state"))

        self._inference_hz = float(param("inference_hz", 3.0))
        self._control_hz = float(param("control_hz", 30.0))
        self._execute_steps = int(param("execute_steps", 1))

        self._enable_motion = bool(param("enable_motion", False))
        self._motion_approved = bool(param("motion_approved", False))
        self._publish_hands = bool(param("publish_hands", False))
        self._safety_config_path = str(param("safety_config", ""))

        self._state_timeout = float(param("state_timeout", 0.5))
        self._image_timeout = float(param("image_timeout", 0.5))
        self._input_watchdog_timeout = float(param("input_watchdog_timeout", 10.0))

        self._max_round_trip = float(param("max_round_trip", 0.5))
        self._max_observation_to_action_age = float(
            param("max_observation_to_action_age", 0.5))
        self._first_response_timeout = float(param("first_response_timeout", 300.0))
        self._max_consecutive_failures = int(param("max_consecutive_failures", 5))
        self._run_seconds = float(param("run_seconds", 0.0))

    def _validate_parameters(self) -> None:
        if not 1 <= self._execute_steps <= 10:
            raise ValueError(
                f"execute_steps must be in [1, 10], got {self._execute_steps}")
        for name, value in (
            ("inference_hz", self._inference_hz),
            ("control_hz", self._control_hz),
            ("state_timeout", self._state_timeout),
            ("image_timeout", self._image_timeout),
            ("input_watchdog_timeout", self._input_watchdog_timeout),
            ("max_round_trip", self._max_round_trip),
            ("max_observation_to_action_age", self._max_observation_to_action_age),
            ("first_response_timeout", self._first_response_timeout),
        ):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive, got {value}")
        if self._max_consecutive_failures < 1:
            raise ValueError("max_consecutive_failures must be >= 1")
        if self._run_seconds < 0.0 or not np.isfinite(self._run_seconds):
            raise ValueError("run_seconds must be finite and >= 0")

    # --- callbacks -------------------------------------------------------

    def _on_state(self, msg: JointState) -> None:
        try:
            if not self._first_state_logged:
                self._first_state_logged = True
                self.get_logger().info(
                    f"first JointState: {len(msg.name)} joints, "
                    f"{len(msg.velocity)} velocities")
            velocity_ok = self._store.update_state(
                msg.name,
                msg.position,
                msg.velocity,
                stamp_seconds(msg.header.stamp),
            )
            self._state_count += 1
            if not velocity_ok and not self._velocity_warned:
                self._velocity_warned = True
                self.get_logger().warning(
                    "joint velocity missing/mismatched; acceleration checks "
                    "use the v0=0 dry-run placeholder, not measured velocity")
        except ContractError as exc:
            self._log_rate_limited(
                "warning", "state_reject", f"reject JointState: {exc}", 5.0)
        except Exception as exc:  # one bad message must not kill spin
            self._log_rate_limited(
                "warning", "state_exc", f"JointState callback error: {exc}", 5.0)

    def _on_image(self, msg: CompressedImage) -> None:
        try:
            image = decode_compressed_image(msg.data)
            if not self._first_image_logged:
                self._first_image_logged = True
                self.get_logger().info(
                    f"first CompressedImage: {msg.format} "
                    f"{len(msg.data)} bytes -> {image.shape}")
            self._store.update_image(image, stamp_seconds(msg.header.stamp))
            self._image_count += 1
        except Exception as exc:
            self._image_reject_count += 1
            self._log_rate_limited(
                "warning", "image_reject", f"drop image frame: {exc}", 5.0)

    def _on_status(self, msg) -> None:
        state = int(msg.state)
        if state != self._last_status:
            self._last_status = state
            name = ROBOT_STATE_NAMES.get(state, str(state))
            self.get_logger().info(f"/xbot_state: {name} ({state}) {msg.msg}")

    # --- lifecycle -------------------------------------------------------

    def run(self) -> int:
        self._inference_thread = threading.Thread(
            target=self._inference_loop, name="openpi-inference", daemon=True)
        self._inference_thread.start()
        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._latch.is_set():
                    self._exit_code = exit_code(True)
                    self._stop_reason = f"fatal: {self._latch.reason}"
                    break
                if (self._run_seconds > 0.0
                        and time.monotonic() - self._start_monotonic >= self._run_seconds):
                    self._exit_code = exit_code(False)
                    self._stop_reason = "run_seconds"
                    break
        finally:
            self._shutdown()
        return self._exit_code

    def _shutdown(self) -> None:
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._stop_event.set()
        if self._inference_thread is not None:
            self._inference_thread.join(timeout=2.0)
            if self._inference_thread.is_alive():
                self.get_logger().warning(
                    "inference thread did not stop in 2 s; leaving it to exit "
                    "with the process instead of destroying its ROS node")
                self._destroyed = True
                return
        self._session.close()
        self._log_summary()
        self.destroy_node()
        self._destroyed = True

    def destroy_node_if_needed(self) -> None:
        if not self._destroyed:
            self.destroy_node()
            self._destroyed = True

    # --- inference -------------------------------------------------------

    def _inference_loop(self) -> None:
        period = 1.0 / self._inference_hz
        next_start = time.monotonic()
        first_success = False
        while not self._stop_event.is_set() and not self._latch.is_set():
            try:
                now = time.monotonic()
                if now - self._last_input_stats >= 5.0:
                    self._last_input_stats = now
                    self.get_logger().info(
                        f"inputs: state={self._state_count} "
                        f"image={self._image_count} "
                        f"image_rejects={self._image_reject_count}")
                snapshot = self._store.snapshot(
                    now, self._state_timeout, self._image_timeout)
                if snapshot is None:
                    state_age, image_age = self._store.input_ages(now)
                    boot_age = now - self._start_monotonic
                    if not np.isfinite(state_age):
                        state_age = boot_age
                    if not np.isfinite(image_age):
                        image_age = boot_age
                    if max(state_age, image_age) > self._input_watchdog_timeout:
                        self._fail(
                            f"input watchdog: state age {state_age:.1f}s, "
                            f"image age {image_age:.1f}s")
                        return
                    self._stop_event.wait(0.05)
                    continue

                observation = {
                    "state": snapshot.state,
                    "image_head": snapshot.image,
                    "prompt": self._prompt,
                }
                timeout = (self._first_response_timeout
                           if not first_success else self._max_round_trip)
                started = time.monotonic()
                try:
                    actions = self._session.infer(
                        observation, response_timeout=timeout,
                        stop_event=self._stop_event)
                except SessionStopped:
                    return
                except ProtocolError as exc:
                    if self._record_protocol_error(exc):
                        return
                    self._wait_for_next(next_start, period)
                    next_start = self._next_start(next_start, period)
                    continue

                rtt = time.monotonic() - started
                observation_age = started - snapshot.observed_monotonic
                if is_stale_response(
                    rtt,
                    observation_age,
                    first_success=first_success,
                    max_round_trip=self._max_round_trip,
                    max_observation_to_action_age=self._max_observation_to_action_age,
                ):
                    if self._record_protocol_error(
                        f"stale response: rtt={rtt:.3f}s "
                        f"observation_age={observation_age:.3f}s"
                    ):
                        return
                    self._wait_for_next(next_start, period)
                    next_start = self._next_start(next_start, period)
                    continue

                try:
                    report = validate_action_chunk(
                        actions,
                        snapshot.state[:ARM_COUNT],
                        command_hz=self._target_hz,
                        config=self._safety,
                        execute_steps=self._execute_steps,
                    )
                except ContractError as exc:
                    if self._record_contract_error(exc):
                        return
                    self._wait_for_next(next_start, period)
                    next_start = self._next_start(next_start, period)
                    continue

                self._tracker.record_success()
                first_success = True
                self._inference_count += 1
                self._samples_ms.append(rtt * 1000.0)
                if not self._metadata_logged and self._session.metadata is not None:
                    self._metadata_logged = True
                    self.get_logger().info(
                        f"server metadata: {self._session.metadata}")

                if report.would_reject:
                    self._would_reject_packets += 1
                    self._log_rate_limited(
                        "warning", "would_reject",
                        "would_reject_motion: " + "; ".join(report.would_reject),
                        1.0)
                if report.hard_limit_violations:
                    self._hard_limit_packets += 1
                    self._log_rate_limited(
                        "error", "hard_limit",
                        "SEVERE hard_limit_violation: "
                        + "; ".join(report.hard_limit_violations),
                        1.0)
                if report.envelope_violations:
                    self._envelope_packets += 1
                    self._log_rate_limited(
                        "error", "envelope",
                        "envelope_violation (outside training data): "
                        + "; ".join(report.envelope_violations),
                        1.0)
                if not snapshot.velocity_available:
                    self._log_rate_limited(
                        "warning", "velocity_unavailable",
                        "velocity unavailable; dry-run acceleration check assumes "
                        "zero initial velocity and proves nothing about velocity safety",
                        10.0)

                skew = timestamp_skew(
                    snapshot.state_stamp, snapshot.image_stamp)
                if (skew is not None and skew > 0.1) or self._inference_count <= 3 \
                        or self._inference_count % 10 == 0:
                    self.get_logger().info(
                        f"inference #{self._inference_count}: "
                        f"rtt={rtt * 1000.0:.1f}ms "
                        f"obs_age={observation_age * 1000.0:.1f}ms "
                        f"state={snapshot.state.shape} image={snapshot.image.shape} "
                        f"first_step={report.first_step_jump:.4f}rad "
                        f"max_step={report.max_step:.4f} "
                        f"max_vel={report.max_velocity:.4f} "
                        f"max_acc={report.max_acceleration:.4f} "
                        f"would_reject={len(report.would_reject)} "
                        f"hard_limits={len(report.hard_limit_violations)} "
                        f"envelope={len(report.envelope_violations)} "
                        f"skew={skew if skew is not None else 'unset'}")
            except Exception as exc:
                self._fail(f"inference loop crashed: {exc!r}")
                return

            self._wait_for_next(next_start, period)
            next_start = self._next_start(next_start, period)

    def _wait_for_next(self, next_start: float, period: float) -> None:
        delay = next_start + period - time.monotonic()
        if delay > 0.0:
            self._stop_event.wait(delay)

    @staticmethod
    def _next_start(next_start: float, period: float) -> float:
        target = next_start + period
        now = time.monotonic()
        return target if target > now else now

    # --- errors / logging ------------------------------------------------

    def _record_protocol_error(self, exc) -> bool:
        self._protocol_error_total += 1
        fatal = self._tracker.record_protocol_error()
        self._log_rate_limited(
            "warning", "protocol_error",
            f"protocol_error #{self._tracker.protocol_failures}: {exc}", 1.0)
        if fatal:
            self._fail(
                f"{self._tracker.protocol_failures} consecutive protocol errors: {exc}")
        return fatal

    def _record_contract_error(self, exc) -> bool:
        self._contract_error_total += 1
        fatal = self._tracker.record_contract_error()
        self._log_rate_limited(
            "warning", "contract_error",
            f"contract_error #{self._tracker.contract_failures}: {exc}", 1.0)
        if fatal:
            self._fail(
                f"{self._tracker.contract_failures} consecutive contract errors: {exc}")
        return fatal

    def _fail(self, reason: str) -> None:
        if self._latch.fail(reason):
            self.get_logger().error(f"FATAL: {reason}")
        self._stop_event.set()

    def _log_rate_limited(
        self, level: str, key: str, message: str, period: float = 5.0
    ) -> None:
        now = time.monotonic()
        with self._log_lock:
            if now - self._last_log.get(key, 0.0) < period:
                return
            self._last_log[key] = now
        # rclpy caches severity per call site (file, line): calling one line
        # with two different levels raises
        # ValueError("Logger severity cannot be changed between calls.").
        # Give each severity its own call site.
        logger = self.get_logger()
        if level == "warning":
            logger.warning(message)
        else:
            logger.error(message)

    def _log_summary(self) -> None:
        count = self._inference_count
        if self._samples_ms:
            p50 = float(np.percentile(self._samples_ms, 50))
            p95 = float(np.percentile(self._samples_ms, 95))
        else:
            p50 = p95 = float("nan")
        self.get_logger().info(
            f"summary ({self._stop_reason}): inferences={count} "
            f"protocol_errors={self._protocol_error_total} "
            f"contract_errors={self._contract_error_total} "
            f"would_reject_packets={self._would_reject_packets} "
            f"hard_limit_packets={self._hard_limit_packets} "
            f"envelope_packets={self._envelope_packets} "
            f"rtt_p50={p50:.1f}ms rtt_p95={p95:.1f}ms")
        if count == 0:
            self.get_logger().warning("summary: no successful inference")
        if self._hard_limit_packets:
            self.get_logger().error(
                "summary: checkpoint DID NOT pass manual arm limit validation; "
                "do not treat these predictions as safe")
        if self._envelope_packets:
            self.get_logger().error(
                "summary: predictions left the training-data envelope; the pose "
                "is out of domain and the policy will not correct it")


def main(args=None) -> int:
    rclpy.init(args=args)
    node = None
    code = 0
    try:
        node = PolicyNode()
        code = node.run()
    except Exception as exc:
        print(f"q5_policy: startup failed: {exc}", file=sys.stderr)
        code = 1
    finally:
        if node is not None:
            node.destroy_node_if_needed()
        if rclpy.ok():
            rclpy.shutdown()
    return code


if __name__ == "__main__":
    sys.exit(main())
