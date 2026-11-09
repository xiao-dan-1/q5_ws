"""Pure-Python checks for the A1 dry-run runtime. No ROS init, no network."""

import json
import unittest
from pathlib import Path

import numpy as np

from q5_ctrl.q5_joint_limits import MODEL_JOINT_NAMES as MODEL_JOINT_NAMES_TUPLE
from q5_policy_runtime.observation import (
    ObservationStore,
    build_state_vector,
    decode_compressed_image,
    timestamp_skew,
    velocity_available,
)
from q5_policy_runtime.safety import (
    ARM_COUNT,
    ContractError,
    FailureTracker,
    FatalLatch,
    assert_dry_run_only,
    command_hz,
    exit_code,
    is_stale_response,
    load_safety_config,
    missing_motion_safety_fields,
    validate_action_chunk,
)

MODEL_JOINT_NAMES = list(MODEL_JOINT_NAMES_TUPLE)
CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "safety.first-motion.json"


def config():
    return load_safety_config(CONFIG_PATH)


def actions():
    return np.zeros((10, 38), dtype=np.float32)


def current():
    return np.zeros(ARM_COUNT, dtype=np.float32)


class GuardTest(unittest.TestCase):
    def test_dry_run_guard(self):
        assert_dry_run_only(False, False, False)
        for flags in ((True, False, False), (False, True, False), (False, False, True)):
            with self.assertRaises(RuntimeError):
                assert_dry_run_only(*flags)


class ObservationTest(unittest.TestCase):
    def test_state_reorder(self):
        names = list(reversed(MODEL_JOINT_NAMES))
        positions = [float(i) for i in range(38)]
        state = build_state_vector(names, positions)
        np.testing.assert_array_equal(
            state, np.arange(37, -1, -1, dtype=np.float32))
        with self.assertRaises(ContractError):
            build_state_vector(names[:-1], positions[:-1])
        with self.assertRaises(ContractError):
            build_state_vector(names[:-1] + [names[0]], positions)
        with self.assertRaises(ContractError):
            build_state_vector(names, positions[:-1])

    def test_image_decode(self):
        import cv2

        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb[..., 0] = 200  # red
        ok, jpeg = cv2.imencode(".jpg", rgb[..., ::-1])
        self.assertTrue(ok)
        image = decode_compressed_image(jpeg.tobytes())
        self.assertEqual(image.shape, (480, 640, 3))
        self.assertEqual(image.dtype, np.uint8)
        self.assertTrue(image.flags["C_CONTIGUOUS"])
        self.assertGreater(int(image[..., 0].mean()), 150)  # RGB, not BGR

        with self.assertRaises(ContractError):
            decode_compressed_image(b"not a jpeg")

    def test_velocity_flag(self):
        names = list(MODEL_JOINT_NAMES)
        self.assertTrue(velocity_available(names, [0.0] * 38))
        self.assertFalse(velocity_available(names, []))
        self.assertFalse(velocity_available(names, [0.0] * 37))
        self.assertFalse(velocity_available(names[1:], [0.0] * 37))

    def test_store_keeps_previous_state(self):
        store = ObservationStore()
        names = list(MODEL_JOINT_NAMES)
        store.update_state(names, [0.5] * 38, [0.0] * 38, 0.0, now=1.0)
        store.update_image(np.zeros((480, 640, 3), np.uint8), 0.0, now=1.0)
        with self.assertRaises(ContractError):
            store.update_state(names[:-1], [2.0] * 37, [0.0] * 37, 0.0, now=1.1)
        snapshot = store.snapshot(now=1.2, state_timeout=0.5, image_timeout=0.5)
        self.assertIsNotNone(snapshot)
        np.testing.assert_allclose(snapshot.state, 0.5)

    def test_freshness(self):
        store = ObservationStore()
        names = list(MODEL_JOINT_NAMES)
        store.update_state(names, [0.0] * 38, [0.0] * 38, 0.0, now=1.0)
        store.update_image(np.zeros((480, 640, 3), np.uint8), 0.0, now=1.0)
        self.assertIsNotNone(
            store.snapshot(now=1.4, state_timeout=0.5, image_timeout=0.5))
        self.assertIsNone(
            store.snapshot(now=1.6, state_timeout=0.5, image_timeout=0.5))

        store.update_state(names, [0.0] * 38, [0.0] * 38, 0.0, now=2.0)
        self.assertIsNone(
            store.snapshot(now=2.0, state_timeout=0.5, image_timeout=0.5))
        # per-input watchdog: a dead camera must not be masked by flowing state
        state_age, image_age = store.input_ages(now=2.5)
        self.assertAlmostEqual(state_age, 0.5)
        self.assertAlmostEqual(image_age, 1.5)

    def test_input_ages_never_received(self):
        store = ObservationStore()
        state_age, image_age = store.input_ages(now=10.0)
        self.assertEqual(state_age, float("inf"))
        self.assertEqual(image_age, float("inf"))

    def test_skew_is_informational(self):
        self.assertIsNone(timestamp_skew(0.0, 5.0))
        self.assertIsNone(timestamp_skew(5.0, 0.0))
        self.assertAlmostEqual(timestamp_skew(1.0, 1.25), 0.25)


class ValidationTest(unittest.TestCase):
    def test_execute_steps_prefix(self):
        chunk = actions()
        chunk[4, 0] = 0.5
        report = validate_action_chunk(
            chunk, current(), command_hz=3.0, config=config(), execute_steps=2)
        self.assertEqual(report.would_reject, ())
        report = validate_action_chunk(
            chunk, current(), command_hz=3.0, config=config(), execute_steps=5)
        self.assertTrue(report.would_reject)

        first_bad = actions()
        first_bad[0, 0] = 0.5
        report = validate_action_chunk(
            first_bad, current(), command_hz=3.0, config=config(), execute_steps=1)
        self.assertTrue(report.would_reject)

    def test_command_hz(self):
        self.assertEqual(command_hz(1, 3.0, 30.0), 3.0)
        self.assertEqual(command_hz(2, 3.0, 30.0), 30.0)
        with self.assertRaises(ValueError):
            command_hz(0, 3.0, 30.0)
        with self.assertRaises(ValueError):
            command_hz(1, 0.0, 30.0)

    def test_timebase_consistency(self):
        chunk = actions()
        chunk[0, 0] = 0.03
        report = validate_action_chunk(
            chunk, current(), command_hz=3.0, config=config(), execute_steps=1)
        self.assertEqual(report.would_reject, ())

        chunk[0, 0] = 0.04
        report = validate_action_chunk(
            chunk, current(), command_hz=3.0, config=config(), execute_steps=1)
        self.assertTrue(any("step" in item for item in report.would_reject))
        self.assertTrue(any("acceleration" in item for item in report.would_reject))

        chunk[0, 0] = 0.03
        report = validate_action_chunk(
            chunk, current(), command_hz=30.0, config=config(), execute_steps=1)
        self.assertTrue(any("velocity" in item for item in report.would_reject))

    def test_nan_inf(self):
        for bad in (np.nan, np.inf, -np.inf):
            chunk = actions()
            chunk[0, 0] = bad
            with self.assertRaises(ContractError):
                validate_action_chunk(
                    chunk, current(), command_hz=3.0, config=config(),
                    execute_steps=1)

    def test_stale_packet_gate(self):
        self.assertTrue(is_stale_response(
            0.6, 0.1, first_success=True, max_round_trip=0.5,
            max_observation_to_action_age=0.5))
        self.assertTrue(is_stale_response(
            0.1, 0.6, first_success=True, max_round_trip=0.5,
            max_observation_to_action_age=0.5))
        self.assertFalse(is_stale_response(
            10.0, 10.0, first_success=False, max_round_trip=0.5,
            max_observation_to_action_age=0.5))

    def test_would_reject_does_not_count(self):
        chunk = actions()
        chunk[0, 0] = 0.5
        report = validate_action_chunk(
            chunk, current(), command_hz=3.0, config=config(), execute_steps=1)
        self.assertTrue(report.would_reject)
        tracker = FailureTracker(5)
        self.assertEqual(tracker.protocol_failures, 0)
        self.assertEqual(tracker.contract_failures, 0)

    def test_hard_limit_keeps_running(self):
        chunk = actions()
        chunk[0, 0] = 3.5
        report = validate_action_chunk(
            chunk, current(), command_hz=3.0, config=config(), execute_steps=1)
        self.assertTrue(report.hard_limit_violations)
        self.assertIn("left_shoulder_pitch_joint", report.hard_limit_violations[0])

    def test_dataset_envelope_gate(self):
        cfg = config()
        joints = cfg["joints"]
        mid = np.array(
            [(joints[n]["min_position"] + joints[n]["max_position"]) / 2.0
             for n in MODEL_JOINT_NAMES],
            dtype=np.float32,
        )
        chunk = np.tile(mid, (10, 1))
        report = validate_action_chunk(
            chunk, mid[:ARM_COUNT], command_hz=3.0, config=cfg, execute_steps=1)
        self.assertEqual(report.envelope_violations, ())

        # a hand joint 0.01 past the envelope: no manual limit covers hands, so
        # this has to be caught by the envelope layer alone
        hand = 26
        chunk[:, hand] = joints[MODEL_JOINT_NAMES[hand]]["max_position"] + 0.01
        report = validate_action_chunk(
            chunk, mid[:ARM_COUNT], command_hz=3.0, config=cfg, execute_steps=1)
        self.assertTrue(
            any(MODEL_JOINT_NAMES[hand] in item for item in report.envelope_violations))
        self.assertEqual(report.hard_limit_violations, ())

    def test_config_has_envelope(self):
        raw = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
        self.assertEqual(len(raw["joints"]), len(MODEL_JOINT_NAMES))
        cfg = config()
        self.assertEqual(missing_motion_safety_fields(cfg, arms_only=True), [])
        # hand dynamic thresholds are still unapproved, so full motion stays blocked
        missing = missing_motion_safety_fields(cfg, arms_only=False)
        self.assertTrue(any(item.startswith("groups.hands") for item in missing))


class FaultTest(unittest.TestCase):
    def test_fault_latch(self):
        latch = FatalLatch()
        self.assertTrue(latch.fail("first"))
        self.assertFalse(latch.fail("second"))
        self.assertTrue(latch.is_set())
        self.assertEqual(latch.reason, "first")

    def test_protocol_failure_counter(self):
        tracker = FailureTracker(3)
        self.assertFalse(tracker.record_protocol_error())
        self.assertFalse(tracker.record_protocol_error())
        self.assertTrue(tracker.record_protocol_error())
        tracker.record_success()
        self.assertEqual(tracker.protocol_failures, 0)

    def test_contract_failure_counter(self):
        tracker = FailureTracker(3)
        tracker.record_protocol_error()
        tracker.record_protocol_error()
        self.assertFalse(tracker.record_contract_error())
        self.assertFalse(tracker.record_contract_error())
        self.assertTrue(tracker.record_contract_error())
        self.assertEqual(tracker.protocol_failures, 2)
        tracker.record_success()
        self.assertEqual(tracker.contract_failures, 0)

    def test_run_seconds(self):
        self.assertEqual(exit_code(False), 0)
        self.assertEqual(exit_code(True), 1)


if __name__ == "__main__":
    unittest.main()
