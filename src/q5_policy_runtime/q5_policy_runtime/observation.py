"""Observation plumbing: state reorder, image decode, freshness."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass

import numpy as np

from .safety import ContractError, MODEL_JOINT_NAMES

IMAGE_HEIGHT = 480
IMAGE_WIDTH = 640


def build_state_vector(names, positions) -> np.ndarray:
    """Reorder JointState positions into the model's 38-dim order."""
    if len(names) != len(positions):
        raise ContractError(
            f"JointState name/position length mismatch: {len(names)} != {len(positions)}")
    if len(set(names)) != len(names):
        raise ContractError("JointState.name contains duplicate joints")
    by_name = dict(zip(names, positions))
    missing = [name for name in MODEL_JOINT_NAMES if name not in by_name]
    if missing:
        raise ContractError(f"JointState missing model joints: {missing}")
    state = np.asarray([by_name[name] for name in MODEL_JOINT_NAMES], dtype=np.float32)
    if not np.isfinite(state).all():
        raise ContractError("JointState contains NaN or Inf")
    return state


def velocity_available(names, velocities) -> bool:
    """True only when velocity can be matched to every model joint by name."""
    if len(velocities) == 0 or len(velocities) != len(names):
        return False
    present = set(names)
    return all(name in present for name in MODEL_JOINT_NAMES)


def stamp_seconds(stamp) -> float:
    """ROS header stamp as seconds; 0.0 means 'unset' for our purposes."""
    sec = int(getattr(stamp, "sec", 0))
    nanosec = int(getattr(stamp, "nanosec", 0))
    if sec == 0 and nanosec == 0:
        return 0.0
    return sec + nanosec * 1e-9


def timestamp_skew(state_stamp: float, image_stamp: float) -> float | None:
    """Header skew when both stamps exist. Informational this round."""
    if state_stamp <= 0.0 or image_stamp <= 0.0:
        return None
    return abs(state_stamp - image_stamp)


def decode_compressed_image(data) -> np.ndarray:
    """CompressedImage.data (JPEG) -> contiguous RGB uint8 (480,640,3).

    The camera also publishes raw `Image` at ~27 MB/s RELIABLE; a Python
    reader on that stream stalls the writer, so we consume the compressed
    topic instead.
    """
    import cv2

    buffer = np.frombuffer(bytes(data), dtype=np.uint8)
    bgr = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
    if bgr is None:
        raise ContractError("compressed image decode failed")
    if bgr.shape[:2] != (IMAGE_HEIGHT, IMAGE_WIDTH):
        bgr = cv2.resize(
            bgr, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    if rgb.dtype != np.uint8 or rgb.shape != (IMAGE_HEIGHT, IMAGE_WIDTH, 3):
        raise ContractError(
            f"decoded image must be uint8 ({IMAGE_HEIGHT},{IMAGE_WIDTH},3), "
            f"got {rgb.dtype} {rgb.shape}")
    return np.ascontiguousarray(rgb)


@dataclass(frozen=True)
class ObservationSnapshot:
    state: np.ndarray
    image: np.ndarray
    state_monotonic: float
    image_monotonic: float
    state_stamp: float
    image_stamp: float
    velocity_available: bool

    @property
    def observed_monotonic(self) -> float:
        """Age is bounded by the older of the two inputs."""
        return min(self.state_monotonic, self.image_monotonic)


class ObservationStore:
    """Thread-safe latest observation, keyed by local receive time."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state: np.ndarray | None = None
        self._state_t = 0.0
        self._state_stamp = 0.0
        self._velocity_available = False
        self._image: np.ndarray | None = None
        self._image_t = 0.0
        self._image_stamp = 0.0

    def update_state(
        self, names, positions, velocities, stamp: float,
        now: float | None = None,
    ) -> bool:
        state = build_state_vector(names, positions)
        velocity_ok = velocity_available(names, velocities)
        received = time.monotonic() if now is None else now
        with self._lock:
            self._state = state
            self._state_t = received
            self._state_stamp = float(stamp)
            self._velocity_available = velocity_ok
        return velocity_ok

    def update_image(self, image: np.ndarray, stamp: float, now: float | None = None) -> None:
        image = np.asarray(image)
        if image.dtype != np.uint8 or image.shape != (IMAGE_HEIGHT, IMAGE_WIDTH, 3):
            raise ContractError(
                f"image must be uint8 ({IMAGE_HEIGHT},{IMAGE_WIDTH},3), "
                f"got {image.dtype} {image.shape}")
        received = time.monotonic() if now is None else now
        with self._lock:
            self._image = np.ascontiguousarray(image)
            self._image_t = received
            self._image_stamp = float(stamp)

    def input_ages(self, now: float | None = None) -> tuple[float, float]:
        """(state_age, image_age) in seconds; inf when never received."""
        now = time.monotonic() if now is None else now
        with self._lock:
            state_age = math.inf if self._state is None else now - self._state_t
            image_age = math.inf if self._image is None else now - self._image_t
        return state_age, image_age

    def snapshot(
        self,
        now: float | None = None,
        state_timeout: float = 0.5,
        image_timeout: float = 0.5,
    ) -> ObservationSnapshot | None:
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._state is None or self._image is None:
                return None
            if now - self._state_t > state_timeout:
                return None
            if now - self._image_t > image_timeout:
                return None
            return ObservationSnapshot(
                state=self._state,
                image=self._image,
                state_monotonic=self._state_t,
                image_monotonic=self._image_t,
                state_stamp=self._state_stamp,
                image_stamp=self._image_stamp,
                velocity_available=self._velocity_available,
            )
