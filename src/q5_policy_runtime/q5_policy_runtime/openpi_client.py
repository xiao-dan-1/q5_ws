"""Minimal OpenPI websocket client. No ROS imports.

Wire format (post-repack keys):
    {"state": float32 (38,), "image_head": uint8 (H,W,3), "prompt": str}
    -> {"actions": (10,38), ...}

The server sends one metadata frame right after connect; it must be consumed
before the first inference response.
"""

from __future__ import annotations

import functools
import time

import msgpack
import numpy as np
from websockets.sync.client import connect


class ProtocolError(RuntimeError):
    """Transport, handshake, decode, or response-format failure."""


class SessionStopped(RuntimeError):
    """The stop event fired while waiting on the socket."""


def _pack_array(obj):
    if isinstance(obj, (np.ndarray, np.generic)) and obj.dtype.kind in ("V", "O", "c"):
        raise ValueError(f"Unsupported dtype: {obj.dtype}")
    if isinstance(obj, np.ndarray):
        return {
            b"__ndarray__": True,
            b"data": obj.tobytes(),
            b"dtype": obj.dtype.str,
            b"shape": obj.shape,
        }
    if isinstance(obj, np.generic):
        return {
            b"__npgeneric__": True,
            b"data": obj.item(),
            b"dtype": obj.dtype.str,
        }
    return obj


def _unpack_array(obj):
    if b"__ndarray__" in obj:
        return np.ndarray(
            buffer=obj[b"data"],
            dtype=np.dtype(obj[b"dtype"]),
            shape=obj[b"shape"],
        )
    if b"__npgeneric__" in obj:
        return np.dtype(obj[b"dtype"]).type(obj[b"data"])
    return obj


packb = functools.partial(msgpack.packb, default=_pack_array)
unpackb = functools.partial(msgpack.unpackb, object_hook=_unpack_array)


class OpenPiSession:
    """One request/response per infer() call, reconnecting lazily."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8123,
        connect_timeout: float = 5.0,
        metadata_timeout: float = 5.0,
    ) -> None:
        self._uri = f"ws://{host}:{port}"
        self._connect_timeout = float(connect_timeout)
        self._metadata_timeout = float(metadata_timeout)
        self._packer = msgpack.Packer(default=_pack_array)
        self._ws = None
        self.metadata = None

    def close(self) -> None:
        ws = self._ws
        self._ws = None
        if ws is not None:
            try:
                ws.close()
            except Exception:
                pass

    def infer(self, observation, response_timeout: float, stop_event=None) -> np.ndarray:
        self._ensure_connected(stop_event)
        ws = self._ws
        try:
            ws.send(self._packer.pack(observation))
        except Exception as exc:
            self.close()
            raise ProtocolError(f"send failed: {exc}") from exc

        deadline = time.monotonic() + float(response_timeout)
        while True:
            if stop_event is not None and stop_event.is_set():
                raise SessionStopped()
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.close()
                raise ProtocolError(
                    f"no response within {float(response_timeout):.3f}s")
            try:
                raw = ws.recv(timeout=min(0.2, remaining))
                break
            except TimeoutError:
                continue
            except Exception as exc:
                self.close()
                raise ProtocolError(f"recv failed: {exc}") from exc

        if isinstance(raw, str):
            self.close()
            raise ProtocolError(f"server error:\n{raw}")
        try:
            result = unpackb(raw)
        except Exception as exc:
            raise ProtocolError(f"decode failed: {exc}") from exc
        if not isinstance(result, dict) or "actions" not in result:
            keys = sorted(result) if isinstance(result, dict) else type(result).__name__
            raise ProtocolError(f"response missing 'actions': {keys}")
        try:
            return np.asarray(result["actions"], dtype=np.float32)
        except Exception as exc:
            raise ProtocolError(f"actions are not array-like: {exc}") from exc

    def _ensure_connected(self, stop_event) -> None:
        if self._ws is not None:
            return
        try:
            ws = connect(
                self._uri,
                compression=None,
                max_size=None,
                open_timeout=self._connect_timeout,
            )
        except Exception as exc:
            raise ProtocolError(f"connect to {self._uri} failed: {exc}") from exc

        deadline = time.monotonic() + self._metadata_timeout
        try:
            while True:
                if stop_event is not None and stop_event.is_set():
                    raise SessionStopped()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise ProtocolError("timed out waiting for server metadata")
                try:
                    raw = ws.recv(timeout=min(0.2, remaining))
                    break
                except TimeoutError:
                    continue
            self.metadata = unpackb(raw)
        except Exception:
            try:
                ws.close()
            except Exception:
                pass
            raise
        self._ws = ws
