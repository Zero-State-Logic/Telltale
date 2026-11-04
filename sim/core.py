"""ctypes binding to the firmware's C core (build/telltale_core.*).

The evaluation never re-implements the detector in Python: every score,
fault class and alarm comes from the same C sources the ESP32 runs.
Build the library first with tools/build.sh (or `make lib`).
"""
from __future__ import annotations

import ctypes as C
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
NFEAT = 8
NBINS = 32
FEATURES = ("peak", "mean", "energy", "t_peak", "settle", "plateau", "residual", "centroid")
FAULT_NAMES = ("none", "stall", "linkage", "brownout", "friction", "unknown")
STATUS_NAMES = ("learning", "healthy", "anomaly")
UNLOCK, RELOCK = 0, 1
IDLE_NONE, IDLE_LEARNING, IDLE_FORCED, IDLE_POWER_LOSS, IDLE_BLANKED, IDLE_ANCHORING = range(6)

_f = C.c_float
_u16 = C.c_uint16
_u32 = C.c_uint32


class ActConfig(C.Structure):
    _fields_ = [
        ("learn_runs", _u16), ("resid_warmup", _u16), ("feature_mask", _u32),
        ("threshold", _f), ("update_gate", _f), ("ewma_alpha", _f), ("z_clip", _f),
        ("floor_abs", _f * NFEAT), ("floor_rel", _f * NFEAT),
        ("settle_band_abs", _f), ("settle_band_rel", _f), ("settle_run", _u16),
        ("z_strong", _f), ("z_weak", _f),
    ]


class IdleConfig(C.Structure):
    _fields_ = [
        ("k_hi_ma", _f), ("k_lo_ma", _f), ("clip_ma", _f), ("h_hi", _f), ("h_lo", _f),
        ("learn_samples", _u32), ("blank_samples", _u32), ("refractory_samples", _u32),
        ("track_alpha", _f), ("track_gate_ma", _f), ("drift_max", _f),
        ("anchor_samples", _u32), ("anchor_max", _f), ("anchor_timeout", _u32),
    ]


class ActResult(C.Structure):
    _fields_ = [
        ("features", _f * NFEAT), ("z", _f * NFEAT), ("score", _f),
        ("status", C.c_int32), ("fault", C.c_int32), ("updated", C.c_int32),
    ]


class MotionSeg(C.Structure):
    _fields_ = [("target_deg", _f), ("ms", _u32)]


class Motion(C.Structure):
    _fields_ = [("start_deg", _f), ("seg", MotionSeg * 6), ("nseg", _u32), ("dwell_ms", _u32)]


def _library_path() -> Path:
    names = {"win32": "telltale_core.dll", "darwin": "libtelltale_core.dylib"}
    name = names.get(sys.platform, "libtelltale_core.so")
    path = ROOT / "build" / name
    if not path.exists():
        raise FileNotFoundError(
            f"{path} not found. Build the C core first: ./tools/build.sh (or make lib)")
    return path


def _bind(lib):
    vp, fp = C.c_void_p, C.POINTER(_f)
    sigs = {
        "tt_detector_init": (None, [vp]),
        "tt_detector_reset_models": (None, [vp]),
        "tt_detector_size": (_u32, []),
        "tt_result_size": (_u32, []),
        "tt_act_config_size": (_u32, []),
        "tt_idle_config_size": (_u32, []),
        "tt_detector_act_config": (C.POINTER(ActConfig), [vp]),
        "tt_detector_idle_config": (C.POINTER(IdleConfig), [vp]),
        "tt_act_process": (C.c_int, [vp, _u32, fp, _u32, _u32, _f, C.POINTER(ActResult)]),
        "tt_act_learning": (C.c_int, [vp, _u32]),
        "tt_idle_enter": (None, [vp]),
        "tt_idle_feed": (C.c_int, [vp, _f, fp]),
        "tt_idle_feed_block": (_u32, [vp, fp, _u32, C.POINTER(C.c_uint8)]),
        "tt_idle_learned": (C.c_int, [vp]),
        "tt_idle_onset_samples": (_u32, [vp]),
        "tt_motion_unlock": (None, [vp]),
        "tt_motion_relock": (None, [vp]),
        "tt_motion_motion_ms": (_u32, [vp]),
        "tt_motion_total_ms": (_u32, [vp]),
        "tt_motion_sample": (_u32, [vp, _u32, fp, _u32]),
        "tt_hmac_sha256": (None, [C.c_char_p, C.c_size_t, C.c_char_p, C.c_size_t, C.c_char_p]),
        "tt_sha256": (None, [C.c_char_p, C.c_size_t, C.c_char_p]),
        "tt_auth_init": (C.c_int, [vp, C.c_char_p, C.c_size_t]),
        "tt_auth_issue": (C.c_int, [vp, _u32, C.c_char_p, C.c_char_p]),
        "tt_auth_verify": (C.c_int, [vp, _u32, C.c_char_p, C.c_char_p, C.c_char_p, C.c_char_p]),
        "tt_auth_mac_hex": (None, [C.c_char_p, C.c_size_t, C.c_char_p, C.c_char_p, C.c_char_p, C.c_char_p]),
    }
    for name, (restype, argtypes) in sigs.items():
        fn = getattr(lib, name)
        fn.restype = restype
        fn.argtypes = argtypes
    return lib


_LIB = None


def lib():
    global _LIB
    if _LIB is None:
        _LIB = _bind(C.CDLL(str(_library_path())))
        if _LIB.tt_result_size() != C.sizeof(ActResult):
            raise RuntimeError("tt_act_result_t layout mismatch")
        if _LIB.tt_act_config_size() != C.sizeof(ActConfig):
            raise RuntimeError("tt_act_config_t layout mismatch")
        if _LIB.tt_idle_config_size() != C.sizeof(IdleConfig):
            raise RuntimeError("tt_idle_config_t layout mismatch")
    return _LIB


def _fptr(arr: np.ndarray):
    return arr.ctypes.data_as(C.POINTER(_f))


class Detector:
    """One lock's detector state (an opaque tt_detector_t buffer)."""

    def __init__(self, _buf=None):
        L = lib()
        self._size = L.tt_detector_size()
        if _buf is None:
            self._buf = C.create_string_buffer(self._size)
            L.tt_detector_init(self._buf)
        else:
            self._buf = _buf

    def copy(self) -> "Detector":
        buf = C.create_string_buffer(self._size)
        C.memmove(buf, self._buf, self._size)
        return Detector(buf)

    @property
    def act_cfg(self) -> ActConfig:
        return lib().tt_detector_act_config(self._buf).contents

    @property
    def idle_cfg(self) -> IdleConfig:
        return lib().tt_detector_idle_config(self._buf).contents

    def learning(self, kind: int) -> bool:
        return lib().tt_act_learning(self._buf, kind) == 1

    def actuation(self, kind: int, samples_ma: np.ndarray, motion_samples: int,
                  dt_ms: float = 1.0) -> dict:
        x = np.ascontiguousarray(samples_ma, dtype=np.float32)
        res = ActResult()
        rc = lib().tt_act_process(self._buf, kind, _fptr(x), len(x), motion_samples, dt_ms,
                                  C.byref(res))
        if rc != 0:
            raise RuntimeError(f"tt_act_process failed ({rc})")
        return {
            "features": np.array(res.features[:], dtype=np.float64),
            "z": np.array(res.z[:], dtype=np.float64),
            "score": float(res.score),
            "status": STATUS_NAMES[res.status],
            "fault": FAULT_NAMES[res.fault],
            "updated": bool(res.updated),
        }

    def idle_enter(self):
        lib().tt_idle_enter(self._buf)

    def idle_feed(self, sample_ma: float):
        stat = _f(0.0)
        ev = lib().tt_idle_feed(self._buf, float(sample_ma), C.byref(stat))
        return ev, float(stat.value)

    def idle_block(self, samples_ma: np.ndarray) -> np.ndarray:
        x = np.ascontiguousarray(samples_ma, dtype=np.float32)
        events = np.zeros(len(x), dtype=np.uint8)
        lib().tt_idle_feed_block(self._buf, _fptr(x), len(x),
                                 events.ctypes.data_as(C.POINTER(C.c_uint8)))
        return events

    def idle_learned(self) -> bool:
        return lib().tt_idle_learned(self._buf) == 1

    def idle_onset_samples(self) -> int:
        return int(lib().tt_idle_onset_samples(self._buf))


def motion_profile(kind: int, dt_us: int) -> tuple[np.ndarray, int, int]:
    """Frame-latched commanded angle for one actuation, sampled every dt_us.
    Returns (angles, motion_ms, total_ms)."""
    L = lib()
    m = Motion()
    (L.tt_motion_unlock if kind == UNLOCK else L.tt_motion_relock)(C.byref(m))
    motion_ms = L.tt_motion_motion_ms(C.byref(m))
    total_ms = L.tt_motion_total_ms(C.byref(m))
    n = int(total_ms * 1000 // dt_us)
    out = np.zeros(n, dtype=np.float32)
    L.tt_motion_sample(C.byref(m), dt_us, _fptr(out), n)
    return out.astype(np.float64), int(motion_ms), int(total_ms)


def hmac_sha256(key: bytes, msg: bytes) -> bytes:
    out = C.create_string_buffer(32)
    lib().tt_hmac_sha256(key, len(key), msg, len(msg), out)
    return out.raw


def sha256(msg: bytes) -> bytes:
    out = C.create_string_buffer(32)
    lib().tt_sha256(msg, len(msg), out)
    return out.raw
