"""ctypes wrapper for libcat_follow_zerocopy (V4L2 fd -> RGA -> RKNN fd).

Board-only: guarded by :func:`runtime_available`.  When the native library is
absent (laptop CI), callers fall back to the NumPy/RKNNLite path.
"""

from __future__ import annotations

import ctypes
import os
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

from cat_follow.logger import get_logger

log = get_logger("vision.zerocopy")

MultiDetection = Tuple[int, int, int, int, float, int]

_MAX_DETECTIONS = 128

_LIB: Optional[ctypes.CDLL] = None
_LIB_PATH: Optional[str] = None
_LOAD_ERROR: Optional[str] = None


class CfZcFrame(ctypes.Structure):
    _fields_ = [
        ("cam_fd", ctypes.c_int),
        ("crop_rgb_fd", ctypes.c_int),
        ("frame_seq", ctypes.c_uint32),
        ("buffer_index", ctypes.c_uint32),
        ("image_size", ctypes.c_uint32),
        ("src_width", ctypes.c_uint32),
        ("src_height", ctypes.c_uint32),
        ("stride", ctypes.c_uint32),
    ]


class CfZcDetection(ctypes.Structure):
    _fields_ = [
        ("x1", ctypes.c_float),
        ("y1", ctypes.c_float),
        ("x2", ctypes.c_float),
        ("y2", ctypes.c_float),
        ("score", ctypes.c_float),
        ("class_id", ctypes.c_int32),
    ]


class CfZcProcessResult(ctypes.Structure):
    _fields_ = [
        ("ok", ctypes.c_int),
        ("rga_ms", ctypes.c_double),
        ("npu_ms", ctypes.c_double),
        ("post_ms", ctypes.c_double),
        ("frame_seq", ctypes.c_uint32),
        ("num_detections", ctypes.c_int),
    ]


def _default_lib_paths() -> list[Path]:
    repo_root = Path(__file__).resolve().parents[2]
    env_path = os.environ.get("CAT_FOLLOW_ZEROCOPY_LIB", "").strip()
    paths: list[Path] = []
    if env_path:
        paths.append(Path(env_path))
    paths.extend(
        [
            Path("/opt/car-x/lib/libcat_follow_zerocopy.so"),
            repo_root / "native" / "zerocopy" / "libcat_follow_zerocopy.so",
        ]
    )
    return paths


def _load_lib() -> Optional[ctypes.CDLL]:
    global _LIB, _LIB_PATH, _LOAD_ERROR
    if _LIB is not None:
        return _LIB
    for candidate in _default_lib_paths():
        if not candidate.is_file():
            continue
        try:
            lib = ctypes.CDLL(str(candidate))
        except OSError as exc:  # pragma: no cover - board only
            _LOAD_ERROR = str(exc)
            continue
        try:
            lib.cf_zc_runtime_available.restype = ctypes.c_int
            lib.cf_zc_last_error.restype = ctypes.c_char_p
            lib.cf_zc_open.restype = ctypes.c_void_p
            lib.cf_zc_open.argtypes = [
                ctypes.c_char_p,
                ctypes.c_char_p,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_int,
                ctypes.c_int,
            ]
            lib.cf_zc_close.argtypes = [ctypes.c_void_p]
            lib.cf_zc_model_load.restype = ctypes.c_int
            lib.cf_zc_model_load.argtypes = [ctypes.c_void_p]
            lib.cf_zc_model_unload.restype = ctypes.c_int
            lib.cf_zc_model_unload.argtypes = [ctypes.c_void_p]
            lib.cf_zc_model_loaded.restype = ctypes.c_int
            lib.cf_zc_model_loaded.argtypes = [ctypes.c_void_p]
            lib.cf_zc_dequeue.restype = ctypes.c_int
            lib.cf_zc_dequeue.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(CfZcFrame),
                ctypes.c_int,
            ]
            lib.cf_zc_requeue.restype = ctypes.c_int
            lib.cf_zc_requeue.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
            lib.cf_zc_copy_camera_nv12.restype = ctypes.c_int
            lib.cf_zc_copy_camera_nv12.argtypes = [
                ctypes.c_void_p,
                ctypes.c_uint32,
                ctypes.c_void_p,
                ctypes.c_size_t,
            ]
            lib.cf_zc_infer_detections.restype = ctypes.c_int
            lib.cf_zc_infer_detections.argtypes = [
                ctypes.c_void_p,
                ctypes.c_uint32,
                ctypes.c_float,
                ctypes.c_int,
                ctypes.POINTER(CfZcDetection),
                ctypes.c_int,
                ctypes.POINTER(CfZcProcessResult),
            ]
            lib.cf_zc_crop_offset.restype = ctypes.c_int
            lib.cf_zc_crop_offset.argtypes = [
                ctypes.c_void_p,
                ctypes.POINTER(ctypes.c_int),
                ctypes.POINTER(ctypes.c_int),
            ]
        except AttributeError as exc:
            _LOAD_ERROR = (
                f"{candidate} is incompatible (missing native model-lifecycle "
                f"symbol: {exc})"
            )
            continue
        _LIB = lib
        _LIB_PATH = str(candidate)
        return lib
    if _LOAD_ERROR is None:
        _LOAD_ERROR = "libcat_follow_zerocopy.so not found"
    return None


def runtime_available() -> bool:
    """True when /dev/rga is accessible and the native .so loads."""
    if not os.path.exists("/dev/rga"):
        return False
    lib = _load_lib()
    if lib is None:
        return False
    try:
        return bool(lib.cf_zc_runtime_available())
    except Exception:  # pragma: no cover
        return False


def last_load_error() -> Optional[str]:
    return _LOAD_ERROR


class ZerocopySession:
    """Thread-safe wrapper around a native CfZcSession."""

    def __init__(
        self,
        handle: ctypes.c_void_p,
        *,
        crop_w: int,
        crop_h: int,
        animal_mode: bool = False,
    ) -> None:
        self._handle = handle
        # The opener owns V4L2 streaming and is the only thread allowed to
        # destroy the combined capture/RGA/RKNN session. Detector/H.264 calls
        # may borrow it concurrently, but cannot idle-close camera resources.
        self._owner_thread_id = threading.get_ident()
        self._lifecycle = threading.Condition()
        # Serializes model load/unload against inference: the detector may
        # idle-unload from its own thread while another caller is mid-invoke,
        # and the native side would then free RKNN state under it.
        self._model_lock = threading.RLock()
        # V4L2 buffers currently dequeued to us. QBUF is exactly-once, so a
        # requeue for an index we do not hold must never reach the driver.
        self._owned_lock = threading.Lock()
        self._owned_buffers: set[int] = set()
        self._closing = False
        self._active_calls = 0
        self._crop_w = crop_w
        self._crop_h = crop_h
        self._animal_mode = bool(animal_mode)
        self.last_perf: dict[str, float] = {}
        self.consecutive_failures = 0
        self.last_error: Optional[str] = None
        offset_x = ctypes.c_int()
        offset_y = ctypes.c_int()
        lib = _load_lib()
        assert lib is not None
        if lib.cf_zc_crop_offset(handle, ctypes.byref(offset_x), ctypes.byref(offset_y)) != 0:
            raise RuntimeError(_last_native_error())
        self.offset_x = int(offset_x.value)
        self.offset_y = int(offset_y.value)

    def _enter_call(self) -> bool:
        with self._lifecycle:
            if self._closing or not self._handle:
                return False
            self._active_calls += 1
            return True

    def _exit_call(self) -> None:
        with self._lifecycle:
            self._active_calls -= 1
            self._lifecycle.notify_all()

    @classmethod
    def open(
        cls,
        *,
        device: str,
        model_path: str,
        src_w: int,
        src_h: int,
        crop_w: int,
        crop_h: int,
        animal_mode: bool = False,
    ) -> Optional["ZerocopySession"]:
        lib = _load_lib()
        if lib is None:
            log.warning("zerocopy library unavailable: %s", _LOAD_ERROR)
            return None
        handle = lib.cf_zc_open(
            device.encode("utf-8"),
            model_path.encode("utf-8"),
            src_w,
            src_h,
            crop_w,
            crop_h,
            -1,
            -1,
        )
        if not handle:
            log.warning("cf_zc_open failed: %s", _last_native_error())
            return None
        try:
            session = cls(
                handle, crop_w=crop_w, crop_h=crop_h, animal_mode=animal_mode
            )
        except BaseException:
            # The native session owns V4L2/RGA/RKNN resources; a partially
            # constructed wrapper would leak them for the process lifetime.
            lib.cf_zc_close(handle)
            raise
        log.info(
            "Zerocopy session opened (device=%s, model=%s, lib=%s)",
            device,
            model_path,
            _LIB_PATH,
        )
        return session

    def close(self) -> None:
        if threading.get_ident() != self._owner_thread_id:
            raise RuntimeError(
                "zerocopy session close must run on its camera owner thread"
            )
        lib = _load_lib()
        if lib is None:
            return
        with self._lifecycle:
            if not self._handle:
                return
            self._closing = True
            while self._active_calls > 0:
                self._lifecycle.wait(timeout=1.0)
            handle = self._handle
            self._handle = None
        with self._owned_lock:
            self._owned_buffers.clear()
        if handle:
            lib.cf_zc_close(handle)

    @property
    def model_loaded(self) -> bool:
        """Whether RKNN/model resources are resident (camera/RGA stay alive)."""
        lib = _load_lib()
        if lib is None or not self._enter_call():
            return False
        try:
            handle = self._handle
            if not handle:
                return False
            state = int(lib.cf_zc_model_loaded(handle))
        finally:
            self._exit_call()
        if state < 0:
            self.last_error = _last_native_error()
            return False
        return state == 1

    def load_model(self) -> bool:
        """Load RKNN resources without touching camera, DMA-BUF, or RGA state."""
        lib = _load_lib()
        if lib is None:
            self.last_error = _LOAD_ERROR or "zerocopy library unavailable"
            return False
        if not self._enter_call():
            self.last_error = "zerocopy session closed"
            return False
        try:
            with self._model_lock:
                handle = self._handle
                ok = bool(handle) and lib.cf_zc_model_load(handle) == 0
        finally:
            self._exit_call()
        if not ok:
            self.last_error = _last_native_error()
        else:
            self.last_error = None
        return ok

    def unload_model(self) -> bool:
        """Release only RKNN/model resources, preserving capture and RGA."""
        lib = _load_lib()
        if lib is None:
            self.last_error = _LOAD_ERROR or "zerocopy library unavailable"
            return False
        if not self._enter_call():
            self.last_error = "zerocopy session closed"
            return False
        try:
            with self._model_lock:
                handle = self._handle
                ok = bool(handle) and lib.cf_zc_model_unload(handle) == 0
        finally:
            self._exit_call()
        if not ok:
            self.last_error = _last_native_error()
        else:
            self.last_error = None
            self.consecutive_failures = 0
        return ok

    def dequeue(self, *, timeout_ms: int = 3000) -> Optional[CfZcFrame]:
        lib = _load_lib()
        if lib is None:
            return None
        wait_start = time.perf_counter()
        if not self._enter_call():
            self.last_error = "zerocopy session closed"
            return None
        lifecycle_wait_ms = (time.perf_counter() - wait_start) * 1000.0
        out = CfZcFrame()
        native_start = time.perf_counter()
        try:
            handle = self._handle
            if not handle:
                self.last_error = "zerocopy session closed"
                return None
            if lib.cf_zc_dequeue(handle, ctypes.byref(out), timeout_ms) != 0:
                self.last_error = _last_native_error()
                return None
            # Register ownership before leaving the lifecycle guard: once
            # ``_exit_call`` runs, a waiting ``close()`` may release the handle
            # while this buffer is still dequeued but untracked.
            with self._owned_lock:
                self._owned_buffers.add(int(out.buffer_index))
        finally:
            self._exit_call()
        native_ms = (time.perf_counter() - native_start) * 1000.0
        self.last_perf = {
            "lifecycle_wait_ms": lifecycle_wait_ms,
            "native_ms": native_ms,
        }
        return out

    def requeue(self, buffer_index: int) -> bool:
        lib = _load_lib()
        if lib is None:
            return False
        index = int(buffer_index)
        # Claim the buffer atomically so two callers cannot both QBUF it.
        with self._owned_lock:
            claimed = index in self._owned_buffers
            self._owned_buffers.discard(index)
        if not claimed:
            # Report success: the buffer is already back with the driver and a
            # second QBUF would corrupt the queue. Loud, but not fatal.
            log.error(
                "Ignoring requeue of V4L2 buffer %d that this session does "
                "not hold (double release?)",
                index,
            )
            return True
        ok = False
        try:
            if not self._enter_call():
                self.last_error = "zerocopy session closed"
                return False
            try:
                handle = self._handle
                if not handle:
                    self.last_error = "zerocopy session closed"
                    return False
                ok = lib.cf_zc_requeue(handle, ctypes.c_uint32(index)) == 0
            finally:
                self._exit_call()
            if not ok:
                self.last_error = _last_native_error()
            return ok
        finally:
            if not ok:
                # Ownership never transferred back, so restore the claim and
                # keep a retry possible.
                with self._owned_lock:
                    self._owned_buffers.add(index)

    def copy_camera_nv12(self, buffer_index: int, dst) -> bool:
        import numpy as np

        lib = _load_lib()
        if lib is None:
            return False
        if not isinstance(dst, np.ndarray) or not dst.flags["WRITEABLE"]:
            raise ValueError("NV12 destination must be a writeable ndarray")
        if not self._enter_call():
            self.last_error = "zerocopy session closed"
            return False
        try:
            handle = self._handle
            if not handle:
                self.last_error = "zerocopy session closed"
                return False
            ok = (
                lib.cf_zc_copy_camera_nv12(
                    handle,
                    ctypes.c_uint32(buffer_index),
                    dst.ctypes.data,
                    dst.nbytes,
                )
                == 0
            )
        finally:
            self._exit_call()
        if not ok:
            self.last_error = _last_native_error()
        return ok

    def infer(
        self, buffer_index: int, score_threshold: float
    ) -> list[MultiDetection]:
        lib = _load_lib()
        if lib is None:
            self.consecutive_failures += 1
            self.last_error = "zerocopy library unavailable"
            return []
        wait_start = time.perf_counter()
        if not self._enter_call():
            self.consecutive_failures += 1
            self.last_error = "zerocopy session closed"
            return []
        lifecycle_wait_ms = (time.perf_counter() - wait_start) * 1000.0
        result = CfZcProcessResult()
        detections: list[MultiDetection] = []
        native_start = time.perf_counter()
        try:
            # Held across the invoke so a concurrent idle-unload cannot free
            # RKNN state while the NPU is reading it.
            with self._model_lock:
                handle = self._handle
                if not handle:
                    self.consecutive_failures += 1
                    self.last_error = "zerocopy session closed"
                    return []
                buf = (CfZcDetection * _MAX_DETECTIONS)()
                if (
                    lib.cf_zc_infer_detections(
                        handle,
                        ctypes.c_uint32(buffer_index),
                        ctypes.c_float(score_threshold),
                        ctypes.c_int(1 if self._animal_mode else 0),
                        buf,
                        _MAX_DETECTIONS,
                        ctypes.byref(result),
                    )
                    != 0
                ):
                    self.consecutive_failures += 1
                    self.last_error = _last_native_error()
                    return []
            for index in range(int(result.num_detections)):
                det = buf[index]
                x1 = int(round(det.x1))
                y1 = int(round(det.y1))
                x2 = int(round(det.x2))
                y2 = int(round(det.y2))
                if x2 > x1 and y2 > y1:
                    detections.append(
                        (x1, y1, x2, y2, float(det.score), int(det.class_id))
                    )
        finally:
            self._exit_call()
        native_ms = (time.perf_counter() - native_start) * 1000.0
        self.last_perf = {
            "pre": float(result.rga_ms),
            "invoke": float(result.npu_ms),
            "post": float(result.post_ms),
            "lifecycle_wait_ms": lifecycle_wait_ms,
            "native_ms": native_ms,
        }
        self.last_perf["total"] = (
            self.last_perf["pre"] + self.last_perf["invoke"] + self.last_perf["post"]
        )
        self.consecutive_failures = 0
        self.last_error = None
        return detections

    def self_test(self) -> None:
        frame = self.dequeue(timeout_ms=5000)
        if frame is None:
            raise RuntimeError(self.last_error or "zerocopy dequeue failed")
        test_error: Optional[BaseException] = None
        try:
            self.infer(frame.buffer_index, score_threshold=0.5)
            if self.consecutive_failures:
                raise RuntimeError(self.last_error or "zerocopy inference failed")
        except BaseException as exc:
            test_error = exc
            raise
        finally:
            if not self.requeue(frame.buffer_index) and test_error is None:
                raise RuntimeError(
                    self.last_error
                    or f"zerocopy self-test requeue failed for {frame.buffer_index}"
                )

    def warmup(self, buffer_index: int, *, score_threshold: float = 0.5) -> None:
        """Reload and run one inference using a caller-owned frame lease."""
        self.infer(buffer_index, score_threshold)
        if self.consecutive_failures:
            raise RuntimeError(self.last_error or "zerocopy warmup inference failed")


def _last_native_error() -> str:
    lib = _load_lib()
    if lib is None:
        return _LOAD_ERROR or "zerocopy library unavailable"
    msg = lib.cf_zc_last_error()
    return msg.decode("utf-8") if msg else "unknown native error"


__all__ = ["ZerocopySession", "runtime_available", "last_load_error", "CfZcFrame"]
