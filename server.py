"""
KyberLink — PC Host Server (vJoy edition)
===============================================================

What this does
---------------
1. Serves the mobile web app (index.html) over plain HTTP.
2. Runs a WebSocket server receiving the phone's 60Hz JSON stream:
       { "Normalized_Steering": -1.0..1.0, "Throttle_State": bool,
         "Brake_State": bool, "Custom_Keys": {"<key name>": bool, ...} }
3. Feeds a vJoy virtual joystick, which Windows and KyberLink see as a
   real DirectInput controller:
       - Steering     -> vJoy X axis  (absolute position, no mouse involved)
       - Throttle     -> vJoy Y axis  (ramped 0..1)
       - Brake        -> vJoy Z axis  (ramped 0..1)
       - Custom_Keys  -> keyboard keys via pynput (unchanged)
4. Optional force feedback. When the phone is switched to FFB mode (settings),
   the effects the game sends to the vJoy device (constant, ramp, square, sine,
   triangle, sawtooth up/down, spring, damper, inertia, friction) are played in
   software and streamed back to the phone, which turns its on-screen wheel with
   the game's force (no auto-centring) and vibrates. Extra messages:
       phone -> PC:  { "type": "config", "ffb": true|false }
       PC -> phone:  { "type": "ffb", "f": -1..1,      signed steering force
                       "v": 0..1,                      how hard to shake the phone
                       "c": [[kind, centre, k+, k-, sat+, sat-, dead], ...],
                                                        spring/damper/friction/inertia
                       "t": [effect types], "n": count }
                     { "type": "ffb_status", "available": bool, "seen": bool, ... }

A safety watchdog centers the wheel, zeroes the pedals and releases all keys
if the phone disconnects or goes quiet.

Windows only. Requires the vJoy driver (https://github.com/jshafer817/vJoy,
the maintained 2.1.9.x fork) with device 1 enabled and axes X, Y, Z. For FFB
mode also tick "Enable Effects" (and the effects you want) under Force Feedback in
"Configure vJoy".
"""

import asyncio
import ctypes
import functools
import gzip as _gzip_mod
import http.server
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import traceback
from ctypes import wintypes
from pathlib import Path
from typing import NamedTuple

try:
    import orjson
    _loads = orjson.loads
    _dumps = lambda obj: orjson.dumps(obj).decode()
    _dumps_compact = _dumps           # orjson always produces compact output
except ImportError:
    _loads = json.loads
    _dumps = json.dumps
    _dumps_compact = lambda obj: json.dumps(obj, separators=(",", ":"))

try:
    from pynput.keyboard import Controller as KeyboardController, Key
except ImportError:
    print("ERROR: pynput is not installed.")
    print("        Run:  pip install -r requirements.txt")
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("ERROR: websockets is not installed.")
    print("        Run:  pip install -r requirements.txt")
    sys.exit(1)

keyboard = KeyboardController()

_SPECIAL_KEYS = {
    "space": Key.space, "shift": Key.shift, "ctrl": Key.ctrl, "alt": Key.alt,
    "enter": Key.enter, "tab": Key.tab, "esc": Key.esc, "escape": Key.esc,
    "up": Key.up, "down": Key.down, "left": Key.left, "right": Key.right,
    "backspace": Key.backspace, "delete": Key.delete, "capslock": Key.caps_lock,
    **{f"f{n}": getattr(Key, f"f{n}") for n in range(1, 13)},
}


@functools.lru_cache(maxsize=64)
def resolve_key(name: str):
    name = (name or "").strip().lower()
    if not name:
        return None
    if name in _SPECIAL_KEYS:
        return _SPECIAL_KEYS[name]
    return name[0]  # literal character


# =====================================================================
# CONFIGURATION — edit these to taste
# =====================================================================

HTTP_PORT = 8000
WS_PORT = 8765

VJOY_DEVICE_ID = 1          # the vJoy device you enabled in "Configure vJoy"
STEER_AXIS = "X"            # X, Y, Z, RX, RY, RZ
THROTTLE_AXIS = "Y"
BRAKE_AXIS = "Z"
CLUTCH_AXIS = "RX"

# Forza Mode — matches the vjoy.reg GameInput RacingWheel descriptor
# Throttle/Brake/Clutch are axis-inverted: 0=full, max=released
FORZA_MODE_DEFAULT = False       # True = start in Forza mode
FORZA_STEER_AXIS = "X"          # AxisIndex 0, Invert=0
FORZA_THROTTLE_AXIS = "Z"       # AxisIndex 2, Invert=1 → send 1-value
FORZA_BRAKE_AXIS = "RX"         # AxisIndex 3, Invert=1
FORZA_CLUTCH_AXIS = "RY"        # AxisIndex 4, Invert=1
FORZA_HANDBRAKE_AXIS = "RZ"     # AxisIndex 5, Invert=0

OUTPUT_HZ = 120             # how often vJoy is updated (phone sends at 60Hz)

# Steering shaping. The phone already applies your Max Steering Angle, so
# keep these gentle. Set STEER_SMOOTHING = 0 to disable smoothing.
STEER_SMOOTHING = 0.025     # seconds (time constant). Higher = smoother but laggier
STEER_DEADZONE = 0.0        # 0.0-0.1; fraction of travel around center that reads as 0
STEER_EXPO = 1.0            # 1.0 = linear, >1 = finer control near center

# The phone's pedals are on/off buttons, so they're ramped into a smooth
# analog axis. Rates are in "full pedal travel per second".
THROTTLE_RISE, THROTTLE_FALL = 4.0, 8.0
BRAKE_RISE, BRAKE_FALL = 6.0, 12.0
CLUTCH_RISE, CLUTCH_FALL = 4.0, 8.0
THROTTLE_MAX = 1.0
BRAKE_MAX = 1.0
CLUTCH_MAX = 1.0

WATCHDOG_TIMEOUT = 0.75     # seconds of silence before inputs are released
RECENTER_DURATION = 0.25    # seconds for the wheel to return from full lock after a disconnect/pause

START_ENGAGED = True        # False = start paused until you press the hotkey
PAUSE_HOTKEY_VK = 0x78      # F9
PAUSE_HOTKEY_NAME = "F9"

# Force feedback (only used while the phone is in FFB mode)
FFB_HZ = 60                 # how often the effect engine is sampled and sent to the phone
FFB_GAIN = 1.0              # master scale for everything sent to the phone (the phone has its own sliders too)
FFB_INVERT = False          # True = flip steering force direction if the wheel pushes away from center
FFB_CONDITION_GAIN = 1.0    # strength of spring/damper/friction/inertia on the phone's wheel; 1.0 = as the game asks, 0 mutes them
FFB_TEXTURE_GAIN = 4.0      # how strongly quick force changes (kerbs, bumps, slip) shake the phone
FFB_TEXTURE_FLOOR = 0.008   # ...ignoring changes smaller than this, so smooth driving stays quiet
FFB_LOAD_VIBRATION = 0.3    # how much the steady steering force itself shakes the phone (0 = none)
FFB_FORCE_RANGE = None      # None = auto-detect how vJoy scales constant/ramp forces. If FFB feels dead
                            # (or far too strong) set 255 or 10000; the console says which one it detected.
FFB_DEBUG = False           # True = print every FFB packet vJoy delivers (use this if nothing happens)


# =====================================================================
# vJoy interface (ctypes wrapper around vJoyInterface.dll)
# =====================================================================

AXIS_USAGE = {"X": 0x30, "Y": 0x31, "Z": 0x32, "RX": 0x33, "RY": 0x34, "RZ": 0x35}
VJD_STAT_OWN, VJD_STAT_FREE, VJD_STAT_BUSY, VJD_STAT_MISS = 0, 1, 2, 3


class VJoyError(RuntimeError):
    pass


class VJoy:
    def __init__(self, device_id: int):
        self.id = device_id
        self.dll = self._load_dll()
        d = self.dll
        d.vJoyEnabled.restype = wintypes.BOOL
        d.GetVJDStatus.argtypes = [wintypes.UINT]
        d.GetVJDStatus.restype = ctypes.c_int
        d.AcquireVJD.argtypes = [wintypes.UINT]
        d.AcquireVJD.restype = wintypes.BOOL
        d.RelinquishVJD.argtypes = [wintypes.UINT]
        d.RelinquishVJD.restype = None
        d.ResetVJD.argtypes = [wintypes.UINT]
        d.ResetVJD.restype = wintypes.BOOL
        d.GetVJDAxisExist.argtypes = [wintypes.UINT, wintypes.UINT]
        d.GetVJDAxisExist.restype = wintypes.BOOL
        d.GetVJDAxisMax.argtypes = [wintypes.UINT, wintypes.UINT, ctypes.POINTER(wintypes.LONG)]
        d.GetVJDAxisMax.restype = wintypes.BOOL
        d.SetAxis.argtypes = [wintypes.LONG, wintypes.UINT, wintypes.UINT]
        d.SetAxis.restype = wintypes.BOOL
        d.SetBtn.argtypes = [wintypes.BOOL, wintypes.UINT, ctypes.c_ubyte]
        d.SetBtn.restype = wintypes.BOOL

        if not d.vJoyEnabled():
            raise VJoyError("vJoy driver is not installed or not enabled.")
        status = d.GetVJDStatus(self.id)
        if status == VJD_STAT_MISS:
            raise VJoyError(f"vJoy device {self.id} is not configured. Open 'Configure vJoy' and enable it.")
        if status == VJD_STAT_BUSY:
            raise VJoyError(f"vJoy device {self.id} is in use by another program (a feeder?). Close it and retry.")
        if status != VJD_STAT_OWN and not d.AcquireVJD(self.id):
            raise VJoyError(f"Could not acquire vJoy device {self.id}.")
        d.ResetVJD(self.id)

        self.max = {}
        self._last = {}
        self._btn_last = {}

    @staticmethod
    def _load_dll():
        sub = "x64" if struct.calcsize("P") == 8 else "x86"
        candidates = []
        for env in ("ProgramFiles", "ProgramFiles(x86)", "ProgramW6432"):
            base = os.environ.get(env)
            if base:
                candidates.append(Path(base) / "vJoy" / sub / "vJoyInterface.dll")
        candidates.append(Path(__file__).resolve().parent / "vJoyInterface.dll")
        for path in candidates:
            if path.exists():
                try:
                    return ctypes.WinDLL(str(path))
                except OSError:
                    continue
        try:
            return ctypes.WinDLL("vJoyInterface.dll")
        except OSError:
            raise VJoyError("vJoyInterface.dll not found. Install vJoy, or copy the DLL next to server.py.")

    def init_axis(self, name: str) -> None:
        usage = AXIS_USAGE[name]
        if not self.dll.GetVJDAxisExist(self.id, usage):
            raise VJoyError(f"vJoy device {self.id} has no {name} axis. Enable it in 'Configure vJoy'.")
        m = wintypes.LONG(0)
        self.dll.GetVJDAxisMax(self.id, usage, ctypes.byref(m))
        self.max[name] = m.value or 32767

    def set_axis(self, name: str, fraction: float) -> int:
        """fraction 0..1 -> full axis range. Skips redundant writes."""
        fraction = max(0.0, min(1.0, fraction))
        value = int(round(fraction * self.max[name]))
        if self._last.get(name) != value:
            self.dll.SetAxis(value, self.id, AXIS_USAGE[name])
            self._last[name] = value
        return value

    def set_btn(self, button_id: int, pressed: bool) -> None:
        """button_id: 1-based button number. Skips redundant writes."""
        val = bool(pressed)
        if self._btn_last.get(button_id) != val:
            self.dll.SetBtn(val, self.id, button_id)
            self._btn_last[button_id] = val

    def close(self) -> None:
        try:
            self.dll.ResetVJD(self.id)
            self.dll.RelinquishVJD(self.id)
        except Exception:
            pass


vjoy: "VJoy | None" = None


# =====================================================================
# Force feedback (FFB): game -> vJoy -> this program -> phone
# =====================================================================
#
# A game that treats the vJoy device as a force-feedback wheel loads DirectInput
# effects into the driver (constant, ramp, square/sine/triangle/sawtooth, spring,
# damper, inertia, friction) and vJoyInterface.dll hands them to us as HID PID
# packets. FfbEngine plays those effects in software and boils them down to what a
# phone can use, FFB_HZ times a second:
#
#   force  signed torque the game applies to the wheel (constant, ramp, slow waves).
#          The phone turns its on-screen wheel with it, so in a corner the wheel is
#          pulled back towards centre instead of snapping to centre when you let go.
#   conds  the spring / damper / friction / inertia effects. They depend on how the
#          wheel moves, so the phone evaluates them itself (no network lag in them).
#   vib    how hard to shake the phone: fast periodic effects, the "texture" in the
#          force (kerbs, bumps, slip) and a little of the steady load.

# vJoy packet types (FFBPType in vJoy's public.h)
PT_EFFREP, PT_ENVREP, PT_CONDREP, PT_PRIDREP, PT_CONSTREP, PT_RAMPREP = 0x01, 0x02, 0x03, 0x04, 0x05, 0x06
PT_EFOPREP, PT_BLKFRREP, PT_CTRLREP, PT_GAINREP = 0x0A, 0x0B, 0x0C, 0x0D
PACKET_NAMES = {
    0x01: "Set Effect", 0x02: "Set Envelope", 0x03: "Set Condition", 0x04: "Set Periodic",
    0x05: "Set Constant Force", 0x06: "Set Ramp", 0x07: "Custom Force Data", 0x08: "Download Sample",
    0x0A: "Effect Operation", 0x0B: "Block Free", 0x0C: "Device Control", 0x0D: "Device Gain",
    0x0E: "Set Custom Force", 0x11: "Create New Effect", 0x12: "Block Load", 0x13: "Pool Report",
}

# effect types (FFBEType) — the same list as the checkboxes in "Configure vJoy"
ET_CONST, ET_RAMP, ET_SQR, ET_SINE, ET_TRNGL, ET_STUP, ET_STDN, ET_SPRNG, ET_DMPR, ET_INRT, ET_FRCTN = range(1, 12)
PERIODIC_EFFECTS = (ET_SQR, ET_SINE, ET_TRNGL, ET_STUP, ET_STDN)
CONDITION_EFFECTS = (ET_SPRNG, ET_DMPR, ET_INRT, ET_FRCTN)
COND_KIND = {ET_SPRNG: 1, ET_DMPR: 2, ET_FRCTN: 3, ET_INRT: 4}          # what the phone is told
EFFECT_NAMES = {
    ET_CONST: "Constant", ET_RAMP: "Ramp", ET_SQR: "Square", ET_SINE: "Sine", ET_TRNGL: "Triangle",
    ET_STUP: "Sawtooth Up", ET_STDN: "Sawtooth Down", ET_SPRNG: "Spring", ET_DMPR: "Damper",
    ET_INRT: "Inertia", ET_FRCTN: "Friction", 12: "Custom",
}
_MEAN_ABS = {ET_SQR: 1.0, ET_SINE: 2.0 / math.pi, ET_TRNGL: 0.5, ET_STUP: 0.5, ET_STDN: 0.5}
_TEXTURE_TAU = 0.06         # s: changes in the force faster than this count as road "texture"
_RANGE_BLEND_S = 1.0        # s: fade-in when we decide a parameter is in vJoy's raw range (see _Range)

# effect operations (FFBOP) and device controls (FFB_CTRL)
EFF_START, EFF_SOLO, EFF_STOP = 1, 2, 3
CTRL_ENACT, CTRL_DISACT, CTRL_STOPALL, CTRL_DEVRST, CTRL_DEVPAUSE, CTRL_DEVCONT = 1, 2, 3, 4, 5, 6


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _to_i16(val) -> int:
    """Interprets raw word value as signed 16-bit 2's complement (-32768..32767).
    Matches vJoy SDK's TwosCompWord2Int helper for HID FFB data."""
    v = int(val) & 0xFFFF
    return v - 0x10000 if v & 0x8000 else v


def _wave(etype: int, p: float) -> float:
    """One cycle of a periodic effect's waveform, p = 0..1 -> -1..1."""
    if etype == ET_SINE:
        return math.sin(2.0 * math.pi * p)
    if etype == ET_SQR:
        return 1.0 if p < 0.5 else -1.0
    if etype == ET_TRNGL:
        return 1.0 - 4.0 * abs(((p + 0.25) % 1.0) - 0.5)
    if etype == ET_STUP:
        return 2.0 * p - 1.0
    return 1.0 - 2.0 * p            # sawtooth down


def _enveloped(fx: "_Fx", level: float, t: float, dur) -> float:
    """Apply the effect's attack/fade envelope to a (non-negative) magnitude."""
    env = fx.env
    if env is None:
        return level
    attack, fade, attack_ms, fade_ms = env
    if attack_ms > 0 and t < attack_ms:
        level = attack + (level - attack) * (t / attack_ms)
    if dur is not None and fade_ms > 0 and t > dur - fade_ms:
        level += (fade - level) * ((t - (dur - fade_ms)) / fade_ms)
    return max(0.0, level)


class _Fx:
    """One effect block: a single DirectInput effect the game has created."""
    __slots__ = ("etype", "duration", "const", "ramp", "periodic", "cond", "env",
                 "playing", "t0", "loops", "implicit", "updates", "last_update")

    def __init__(self):
        self.etype = 0
        self.duration = None                 # ms; None = runs until stopped
        self.const = 0.0                     # constant force, -1..1
        self.ramp = (0.0, 0.0)               # start, end force (-1..1)
        self.periodic = (0.0, 0.0, 100.0)    # magnitude 0..1, offset -1..1, period ms
        self.cond = None                     # (centre, +coef, -coef, +sat, -sat, deadband) for the steering axis
        self.env = None                      # (attack level, fade level, attack ms, fade ms)
        self.playing = False
        self.t0 = 0.0
        self.loops = None                    # None = repeat until stopped
        self.implicit = False                # first heard of through a parameter report (never saw it set up or started)
        self.updates = 0
        self.last_update = 0.0


class FfbSample(NamedTuple):
    force: float        # signed, -1..1; positive pushes the wheel towards +steering (right)
    vib: float          # 0..1, how hard to shake the phone
    conds: list         # [[kind, centre, k+, k-, sat+, sat-, deadband], ...]  kind: 1 spring 2 damper 3 friction 4 inertia
    types: list         # effect type codes that are playing (for the readout)


class FfbEngine:
    """Software model of the effects a game has loaded into the vJoy device.

    The handler methods are called from vJoy's FFB thread; sample() is called from the
    asyncio loop. Everything is guarded by one lock.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._fx = {}                  # effect block index -> _Fx
        self.gain = 1.0                # device gain (0..1)
        self.actuators = True          # "enable/disable actuators" from the game
        self._paused_at = None
        self.packets = 0
        self.seen = False              # True once the game has sent us any FFB packet
        self.log = None                # optional callable(str): one-line notes on what the game is doing
        self._t_last = 0.0
        self._lp = 0.0                 # low-passed force: the "steady" part
        self._tex = 0.0                # smoothed texture level
        self._idle = True

    # ---- packet handlers (vJoy's thread) ----------------------------------

    def _note(self) -> None:
        if not self.seen:
            self.seen = True
            if self.log:
                self.log("first FFB packet received from the game ✓")
        self.packets += 1

    def _block(self, ebi: int, implicit: bool = False) -> _Fx:
        self._note()
        fx = self._fx.get(ebi)
        if fx is None:
            fx = self._fx[ebi] = _Fx()
            fx.implicit = implicit
        return fx

    @staticmethod
    def _touch(fx: _Fx) -> None:
        fx.updates += 1
        fx.last_update = time.perf_counter()

    def effect_report(self, ebi: int, etype: int, duration_ms: int) -> None:
        with self._lock:
            fx = self._block(ebi)
            fx.implicit = False
            # DirectInput's INFINITE arrives as 0xFFFF (vJoy's field tops out at 0x7FFF)
            fx.duration = None if duration_ms >= 0x7FFF else float(duration_ms)
            if self.log and fx.etype != etype:
                span = "until stopped" if fx.duration is None else f"{duration_ms} ms"
                self.log(f"game loaded effect {ebi}: {EFFECT_NAMES.get(etype, etype)} ({span})")
            fx.etype = etype

    def envelope(self, ebi: int, attack_level: float, fade_level: float, attack_ms: float, fade_ms: float) -> None:
        with self._lock:
            self._block(ebi, implicit=True).env = (attack_level, fade_level, float(attack_ms), float(fade_ms))

    def constant(self, ebi: int, force: float) -> None:
        with self._lock:
            fx = self._block(ebi, implicit=True)
            self._touch(fx)
            fx.const = _clamp(force, -1.0, 1.0)
            fx.etype = fx.etype or ET_CONST
            if not fx.playing:
                fx.playing = True
                fx.t0 = time.perf_counter()
                fx.loops = None
                fx.duration = None
                if self.log:
                    self.log(f"effect {ebi} (Constant) active with force {fx.const:+.2f}")
            self.actuators = True
            self._paused_at = None

    def ramp(self, ebi: int, start: float, end: float) -> None:
        with self._lock:
            fx = self._block(ebi, implicit=True)
            self._touch(fx)
            fx.ramp = (_clamp(start, -1.0, 1.0), _clamp(end, -1.0, 1.0))
            fx.etype = fx.etype or ET_RAMP
            if not fx.playing:
                fx.playing = True
                fx.t0 = time.perf_counter()
            self.actuators = True
            self._paused_at = None

    def periodic(self, ebi: int, magnitude: float, offset: float, period_ms: float) -> None:
        with self._lock:
            fx = self._block(ebi, implicit=True)
            self._touch(fx)
            fx.periodic = (_clamp(magnitude, 0.0, 1.0), _clamp(offset, -1.0, 1.0), max(1.0, float(period_ms)))
            fx.etype = fx.etype or ET_SINE
            if not fx.playing:
                fx.playing = True
                fx.t0 = time.perf_counter()
            self.actuators = True
            self._paused_at = None

    def condition(self, ebi: int, is_y: bool, centre: float, pos_coef: float, neg_coef: float,
                  pos_sat: float, neg_sat: float, deadband: float) -> None:
        with self._lock:
            fx = self._block(ebi, implicit=True)
            self._touch(fx)
            if not is_y:               # only the X axis exists on a steering wheel
                fx.cond = (_clamp(centre, -1.0, 1.0), _clamp(pos_coef, -1.0, 1.0), _clamp(neg_coef, -1.0, 1.0),
                           _clamp(pos_sat, 0.0, 1.0), _clamp(neg_sat, 0.0, 1.0), abs(deadband))
            fx.etype = fx.etype or ET_SPRNG
            if not fx.playing:
                fx.playing = True
                fx.t0 = time.perf_counter()
                fx.loops = None
                fx.duration = None
                if self.log:
                    self.log(f"effect {ebi} (Condition) active: center {centre:+.2f}, coef +{pos_coef:.2f}/-{neg_coef:.2f}")
            self.actuators = True
            self._paused_at = None

    def effect_op(self, ebi: int, op: int, loops: int) -> None:
        now = time.perf_counter()
        with self._lock:
            self._note()
            if op == EFF_STOP:
                fx = self._fx.get(ebi)
                if fx:
                    fx.playing = False
                    fx.implicit = False
            elif op in (EFF_START, EFF_SOLO):
                if op == EFF_SOLO:
                    for other in self._fx.values():
                        other.playing = False
                fx = self._block(ebi)
                fx.implicit = False
                fx.playing = True
                fx.t0 = now
                fx.loops = None if loops >= 255 else max(1, loops)
                self.actuators = True
                self._paused_at = None

    def block_free(self, ebi: int) -> None:
        with self._lock:
            self._note()
            self._fx.pop(ebi, None)

    def device_control(self, ctrl: int) -> None:
        now = time.perf_counter()
        with self._lock:
            self._note()
            if ctrl == CTRL_STOPALL:
                for fx in self._fx.values():
                    fx.playing = False
                    fx.implicit = False
            elif ctrl == CTRL_DEVRST:
                self._fx.clear()
                self.actuators = True
                self._paused_at = None
            elif ctrl == CTRL_ENACT:
                self.actuators = True
            elif ctrl == CTRL_DISACT:
                self.actuators = False
            elif ctrl == CTRL_DEVPAUSE:
                if self._paused_at is None:
                    self._paused_at = now
            elif ctrl == CTRL_DEVCONT:
                if self._paused_at is not None:
                    shift = now - self._paused_at
                    for fx in self._fx.values():
                        fx.t0 += shift
                    self._paused_at = None

    def device_gain(self, gain_0_255: int) -> None:
        with self._lock:
            self._note()
            self.gain = _clamp(gain_0_255 / 255.0, 0.0, 1.0)

    # ---- sampling (asyncio thread) ----------------------------------------

    def sample(self, now: float, enabled_effects: dict = None) -> FfbSample:
        with self._lock:
            dt = _clamp(now - self._t_last, 0.001, 0.1)
            self._t_last = now

            if enabled_effects is None:
                enabled_effects = {}

            def is_vib_on(key):
                k_str = str(key)
                if k_str in enabled_effects:
                    return bool(enabled_effects[k_str])
                if key in enabled_effects:
                    return bool(enabled_effects[key])
                return True

            # If an effect is receiving updates from the game, it is actively playing
            for fx in self._fx.values():
                if not fx.playing and (now - fx.last_update < 0.5) and fx.updates > 0:
                    fx.playing, fx.t0, fx.loops, fx.duration = True, now, None, None
                    if self.log:
                        self.log(f"effect ({EFFECT_NAMES.get(fx.etype, fx.etype)}) auto-started by active parameter stream")

            playing = [fx for fx in self._fx.values() if fx.playing]
            if not playing or not self.actuators or self._paused_at is not None:
                self._idle, self._tex = True, 0.0
                return FfbSample(0.0, 0.0, [], [])

            force = vib = 0.0
            conds, types = [], set()
            g = self.gain
            gc = g * FFB_CONDITION_GAIN
            for fx in playing:
                f, v, cond = self._contribution(fx, now)
                if not fx.playing:                        # a finite effect just ended
                    continue
                force += f

                et = fx.etype
                if abs(f) > 1e-4 or fx.const != 0.0 or fx.ramp != (0.0, 0.0):
                    if et in (ET_CONST, ET_RAMP):
                        types.add(et)
                    elif fx.ramp != (0.0, 0.0):
                        types.add(ET_RAMP)
                    else:
                        types.add(ET_CONST)

                if v > 1e-4:
                    pet = et if et in PERIODIC_EFFECTS else ET_SINE
                    types.add(pet)
                    if is_vib_on(pet):
                        vib += v

                if cond is not None:
                    kind, centre, kp, kn, sp, sn, dead = cond
                    conds.append([kind, centre, kp * gc, kn * gc,
                                  (sp if sp > 0 else 1.0) * gc, (sn if sn > 0 else 1.0) * gc, dead])
                    cet = et if et in CONDITION_EFFECTS else ET_DMPR
                    types.add(cet)
                    if is_vib_on(cet):
                        c_strength = max(abs(kp), abs(kn))
                        if c_strength > 0.05:
                            vib += min(0.3, c_strength * 0.25)

            if not types:                                 # everything that was playing has just ended
                self._idle, self._tex = True, 0.0
                return FfbSample(0.0, 0.0, [], [])
            force = _clamp(force, -1.0, 1.0) * g
            vib = min(1.0, vib) * g

            # Road feel: whatever the force does faster than ~3 Hz (kerbs, bumps, slip, collisions) is texture.
            if self._idle:
                self._lp, self._idle = force, False      # start from the current force so an onset isn't texture
            tex = max(0.0, abs(force - self._lp) - FFB_TEXTURE_FLOOR) * FFB_TEXTURE_GAIN
            self._lp += (force - self._lp) * (1.0 - math.exp(-dt / _TEXTURE_TAU))
            tex = min(1.0, tex)
            self._tex += (tex - self._tex) * (1.0 - math.exp(-dt / (0.02 if tex > self._tex else 0.08)))

            extra_vib = 0.0
            if is_vib_on("texture") and is_vib_on("0"):
                extra_vib += self._tex

            load_vib = FFB_LOAD_VIBRATION * abs(self._lp)
            force_active = [t for t in types if t in (ET_CONST, ET_RAMP)]
            if force_active:
                if any(is_vib_on(t) for t in force_active):
                    extra_vib += load_vib
            elif is_vib_on(ET_CONST):
                extra_vib += load_vib

            vib = min(1.0, vib + extra_vib)
            return FfbSample(force, vib, conds, sorted(types))

    @staticmethod
    def _contribution(fx: _Fx, now: float):
        """(signed force, vibration, condition-or-None) of one playing effect right now."""
        t = max(0.0, (now - fx.t0) * 1000.0)                   # ms since Start
        dur = fx.duration
        is_active = (now - fx.last_update) < 0.5
        if dur is not None and not is_active:
            if dur <= 0 or (fx.loops is not None and t >= dur * fx.loops):
                fx.playing = False
                return 0.0, 0.0, None
            t %= dur                                           # position inside the current loop

        force = 0.0
        vib = 0.0
        cond = None
        et = fx.etype

        # 1. Constant force (either standalone or along with condition/damper)
        if fx.const:
            c = fx.const
            force += math.copysign(_enveloped(fx, abs(c), t, dur), c)

        # 2. Ramp force
        if fx.ramp != (0.0, 0.0):
            a, b = fx.ramp
            v = a + (b - a) * (t / dur) if dur else a
            force += math.copysign(_enveloped(fx, abs(v), t, dur), v)

        # 3. Periodic effects
        if fx.periodic != (0.0, 0.0, 100.0):
            mag, off, period = fx.periodic
            mag = _enveloped(fx, mag, t, dur)
            pet = et if et in PERIODIC_EFFECTS else ET_SINE
            k = _clamp((1000.0 / period - 6.0) / 8.0, 0.0, 1.0)
            force += off + (1.0 - k) * mag * _wave(pet, (t / period) % 1.0)
            vib += k * mag * _MEAN_ABS.get(pet, 0.5)

        # 4. Condition effects (spring, damper, friction, inertia)
        if fx.cond is not None:
            cet = et if et in CONDITION_EFFECTS else ET_DMPR
            cond = [COND_KIND.get(cet, 2), *fx.cond]

        return force, vib, cond


ffb_engine = FfbEngine()


# ---- the vJoyInterface.dll side ------------------------------------------

def _struct(name: str, fields: list):
    """ctypes mirror of one of vJoy's FFB_* structs. The trailing slack means a DLL whose
    struct is a little bigger than ours can never write past the end of our buffer."""
    return type(name, (ctypes.Structure,), {"_fields_": fields + [("_slack", ctypes.c_ubyte * 48)]})


_U8, _U16, _I32, _U32 = ctypes.c_ubyte, ctypes.c_uint16, ctypes.c_int32, ctypes.c_uint32
_EffReport = _struct("_EffReport", [("ebi", _U8), ("etype", _I32), ("duration", _U16), ("trigger_rpt", _U16), ("sample_prd", _U16)])
_EffConst = _struct("_EffConst", [("ebi", _U8), ("magnitude", _I32)])
_EffRamp = _struct("_EffRamp", [("ebi", _U8), ("start", _I32), ("end", _I32)])
_EffPeriod = _struct("_EffPeriod", [("ebi", _U8), ("magnitude", _U32), ("offset", _I32), ("phase", _U32), ("period", _U32)])
_EffCond = _struct("_EffCond", [("ebi", _U8), ("is_y", _I32), ("centre", _I32), ("pos_coef", _I32), ("neg_coef", _I32),
                                ("pos_sat", _U32), ("neg_sat", _U32), ("deadband", _I32)])
_EffEnvlp = _struct("_EffEnvlp", [("ebi", _U8), ("attack_level", _U32), ("fade_level", _U32), ("attack_time", _U32), ("fade_time", _U32)])
_EffOp = _struct("_EffOp", [("ebi", _U8), ("op", _I32), ("loops", _U8)])


class _FfbData(ctypes.Structure):
    """vJoy's FFB_DATA: what the callback receives a pointer to."""
    _pack_ = 1
    _fields_ = [("size", ctypes.c_uint32), ("cmd", ctypes.c_uint32), ("data", ctypes.POINTER(ctypes.c_ubyte))]


_FFB_CALLBACK = ctypes.WINFUNCTYPE(None, ctypes.c_void_p, ctypes.c_void_p)


def _say(text: str) -> None:
    print(f"\n[FFB] {text}")


class _Range:
    """Turns one kind of vJoy parameter into -1..1 (signed) or 0..1 (unsigned).

    Depending on the game and vJoy build, values arrive in:
      * raw HID byte scale (±127 or ±255)
      * per-mille scale (±1000, e.g. 500 = 50% strength)
      * DirectInput scale (±10000)

    This automatically detects which scale the game is using:
      * peak > 1050 -> ±10000 scale
      * peak between raw_limit and 1050 -> ±1000 scale
      * peak <= raw_limit -> raw ±raw_full scale
    """

    def __init__(self, name: str, raw_limit: int, raw_full: float, min_peak: float, min_count: int,
                 override=None, verbose: bool = False):
        self.name, self.raw_limit, self.raw_full = name, raw_limit, float(raw_full)
        self.min_peak, self.min_count, self.verbose = min_peak, min_count, verbose
        self.override = float(override) if override else None     # the user knows best (FFB_FORCE_RANGE)
        self.peak, self.count = 0.0, 0
        self.mode = "scaled"                  # "scaled" = ±10000, "1000" = ±1000, "raw" = ±raw_full
        self._raw_since = 0.0
        self._said_mode = None

    def __call__(self, value: float) -> float:
        if self.override:                     # pinned: full scale is exactly what the user said
            return _clamp(value / self.override, -1.0, 1.0)
        a = abs(value)
        if a:
            self.count += 1
            if a > self.peak:
                self.peak = a
        self._decide()
        if self.mode == "raw":
            full = self.raw_full
        elif self.mode == "1000":
            full = 1000.0
        else:
            full = 10000.0
        return _clamp(value / full, -1.0, 1.0)

    def _decide(self) -> None:
        if self.peak > 1050:
            if self.mode != "scaled":
                self.mode = "scaled"
                _say(f"{self.name}: saw {self.peak:.0f} > 1000 - using the ±10000 scale")
            elif self._said_mode != "scaled":
                _say(f"{self.name}: values reach {self.peak:.0f} - using the documented ±10000 scale")
            self._said_mode = "scaled"
        elif self.count >= 5 and self.peak > self.raw_limit and self.peak <= 1050:
            if self.mode != "1000":
                self.mode = "1000"
                _say(f"{self.name}: {self.count} values peaking at {self.peak:.0f} - auto-detected ±1000 scale (e.g. 500 = 50%)")
            self._said_mode = "1000"
        elif self.count >= self.min_count and self.peak <= self.raw_limit and self.peak >= self.min_peak:
            if self.mode != "raw":
                self.mode = "raw"
                _say(f"{self.name}: {self.count} values peaking at {self.peak:.0f} - using raw ±{self.raw_full:.0f} scale")
            self._said_mode = "raw"

    def weak_hint(self) -> str:
        """Non-empty when the game is clearly sending something but it all rounds to nothing."""
        if not self.override and self.mode == "scaled" and self.count >= 400 and self.peak <= self.raw_limit:
            return (f"The game's {self.name} values are tiny (largest {self.peak:.0f}). If the wheel feels dead, "
                    f"set FFB_FORCE_RANGE = {self.raw_full:.0f} in server.py.")
        return ""


class VJoyFfb:
    """Registers a callback with vJoyInterface.dll and feeds the packets to an FfbEngine.

    The callback runs on a thread owned by the DLL, so it only parses and updates the
    engine's tables; the phone is told about the result from the asyncio loop.
    """

    def __init__(self, vjoy_dev: "VJoy", engine: FfbEngine):
        self.vjoy = vjoy_dev
        self.engine = engine
        self.ok = False
        self.reason = ""        # why FFB can't work (when ok is False)
        self.warning = ""       # a non-fatal warning (when ok is True)
        self._cb = None         # keep a reference or ctypes frees the callback
        self._warned = set()
        self._seen_types = set()
        self._logged_packets = 0
        self._const_logs = 0
        self._cond_logs = 0
        self.packet_counts = {}
        self._const_r = _Range("constant force", 256, 255, 64, 150, FFB_FORCE_RANGE, verbose=True)
        self._ramp_r = _Range("ramp", 128, 127, 32, 20, FFB_FORCE_RANGE)
        self._period_mag, self._period_off = _Range("periodic magnitude", 255, 255, 64, 20), _Range("periodic offset", 128, 127, 32, 20)
        self._cond_signed, self._cond_unsigned = _Range("condition coefficients", 128, 127, 32, 20, FFB_FORCE_RANGE), _Range("condition limits", 255, 255, 64, 20, FFB_FORCE_RANGE)
        self._env_r = _Range("envelope", 255, 255, 64, 10)
        engine.log = _say

    @property
    def hint(self) -> str:
        return " ".join(h for h in (self.warning, self._const_r.weak_hint()) if h)

    def _bind(self, struct_type, *names):
        for name in names:
            fn = getattr(self.vjoy.dll, name, None)
            if fn is not None:
                fn.argtypes = [ctypes.c_void_p, ctypes.POINTER(struct_type)]
                fn.restype = ctypes.c_uint32                    # DWORD: 0 = ERROR_SUCCESS
                return fn
        return None

    def start(self) -> bool:
        d = self.vjoy.dll
        register = getattr(d, "FfbRegisterGenCB", None)
        self._type = self._bind(ctypes.c_int32, "Ffb_h_Type")
        self._report = self._bind(_EffReport, "Ffb_h_Eff_Report")
        self._const = self._bind(_EffConst, "Ffb_h_Eff_Constant", "Ffb_h_Eff_Const")
        self._op = self._bind(_EffOp, "Ffb_h_EffOp", "Ffb_h_Eff_Op")
        self._ramp = self._bind(_EffRamp, "Ffb_h_Eff_Ramp")
        self._period = self._bind(_EffPeriod, "Ffb_h_Eff_Period")
        self._cond = self._bind(_EffCond, "Ffb_h_Eff_Cond")
        self._envlp = self._bind(_EffEnvlp, "Ffb_h_Eff_Envlp")
        self._ebi = self._bind(ctypes.c_uint32, "Ffb_h_EffectBlockIndex", "Ffb_h_EBI")
        self._ctrl = self._bind(ctypes.c_int32, "Ffb_h_DevCtrl")
        self._gain = self._bind(ctypes.c_ubyte, "Ffb_h_DevGain")

        if register is None or None in (self._type, self._report, self._const, self._op):
            self.reason = "vJoyInterface.dll has no force-feedback functions (use vJoy 2.1.9.x from github.com/jshafer817/vJoy)"
            return False
        missing = [n for n, fn in (("ramp", self._ramp), ("periodic", self._period), ("condition", self._cond),
                                   ("envelope", self._envlp), ("block free", self._ebi), ("device control", self._ctrl),
                                   ("device gain", self._gain)) if fn is None]
        if missing:
            _say(f"note: this vJoyInterface.dll can't decode: {', '.join(missing)} packets (those are ignored)")

        is_ffb = getattr(d, "IsDeviceFfb", None)
        if is_ffb is not None:
            try:
                is_ffb.argtypes = [wintypes.UINT]
                is_ffb.restype = wintypes.BOOL
                if not is_ffb(self.vjoy.id):
                    self.warning = f"vJoy says device {self.vjoy.id} has force feedback off - tick 'Enable Effects' in Configure vJoy."
                    _say(f"warning: {self.warning}")
            except Exception:
                pass

        self._cb = _FFB_CALLBACK(self._on_packet)
        register.argtypes = [_FFB_CALLBACK, ctypes.c_void_p]
        register.restype = None
        register(self._cb, None)
        start = getattr(d, "FfbStart", None)                    # older vJoy builds want the queue started explicitly
        if start is not None:
            try:
                start.argtypes = [wintypes.UINT]
                start.restype = wintypes.BOOL
                start(self.vjoy.id)
            except Exception:
                pass
        self.ok = True
        return True

    # ---- callback (runs on vJoy's thread) ---------------------------------

    def _on_packet(self, pkt, _user) -> None:
        if not pkt:
            return
        try:
            self._dispatch(pkt)
        except Exception:
            if "dispatch" not in self._warned:                  # report once, never spam the console
                self._warned.add("dispatch")
                traceback.print_exc()

    @staticmethod
    def _call(fn, pkt, out) -> bool:
        return fn is not None and fn(pkt, ctypes.byref(out)) == 0

    @staticmethod
    def _log(text: str) -> None:
        if FFB_DEBUG:
            _say(text)

    def _dispatch(self, pkt) -> None:
        ptype = ctypes.c_int32(0)
        has_type = self._call(self._type, pkt, ptype)
        t = ptype.value if has_type else 0

        ebi_val = ctypes.c_uint32(0)
        has_ebi = self._ebi is not None and self._call(self._ebi, pkt, ebi_val)
        hdr_ebi = int(ebi_val.value & 0xFF) if has_ebi else 0

        pname = PACKET_NAMES.get(t, f"Type {t}")
        self.packet_counts[pname] = self.packet_counts.get(pname, 0) + 1

        if self._logged_packets < 15 or FFB_DEBUG:
            self._logged_packets += 1
            hdr = ctypes.cast(pkt, ctypes.POINTER(_FfbData)).contents
            n = max(0, min(int(hdr.size) - 8, 16))
            raw = bytes(hdr.data[:n]).hex(" ") if hdr.data and n else ""
            _say(f"Packet #{self.engine.packets + 1}: {pname} (ebi={hdr_ebi}) [{raw}]")
        elif pname not in self._seen_types:
            self._seen_types.add(pname)
            _say(f"New FFB packet type: {pname} (ebi={hdr_ebi})")

        eng = self.engine

        if t == PT_EFFREP or (not has_type and self._report):
            r = _EffReport()
            if self._call(self._report, pkt, r):
                e_id = hdr_ebi or r.ebi
                self._log(f"  effect {e_id}: type {r.etype}, duration {r.duration} ms")
                eng.effect_report(e_id, r.etype, r.duration)
                return

        if t == PT_CONSTREP or (not has_type and self._const):
            r = _EffConst()
            if self._call(self._const, pkt, r):
                e_id = hdr_ebi or r.ebi
                mag = _to_i16(r.magnitude)
                scaled = self._const_r(mag)
                if self._const_logs < 5:
                    self._const_logs += 1
                    _say(f"Constant force packet: ebi={e_id}, raw={r.magnitude}, signed={mag}, scaled={scaled:+.2f}")
                eng.constant(e_id, scaled)
                return

        if t == PT_EFOPREP or (not has_type and self._op):
            r = _EffOp()
            if self._call(self._op, pkt, r):
                e_id = hdr_ebi or r.ebi
                self._log(f"  effect {e_id}: operation {r.op}, loops {r.loops}")
                eng.effect_op(e_id, r.op, r.loops)
                return

        if t == PT_RAMPREP or (not has_type and self._ramp):
            r = _EffRamp()
            if self._call(self._ramp, pkt, r):
                e_id = hdr_ebi or r.ebi
                start = _to_i16(r.start)
                end = _to_i16(r.end)
                self._log(f"  effect {e_id}: ramp {start} -> {end}")
                eng.ramp(e_id, self._ramp_r(start), self._ramp_r(end))
                return

        if t == PT_PRIDREP or (not has_type and self._period):
            r = _EffPeriod()
            if self._call(self._period, pkt, r):
                e_id = hdr_ebi or r.ebi
                off = _to_i16(r.offset)
                self._log(f"  effect {e_id}: periodic magnitude {r.magnitude}, offset {off}, period {r.period} ms")
                eng.periodic(e_id, self._period_mag(r.magnitude), self._period_off(off), r.period)
                return

        if t == PT_CONDREP or (not has_type and self._cond):
            r = _EffCond()
            if self._call(self._cond, pkt, r):
                e_id = hdr_ebi or r.ebi
                centre = _to_i16(r.centre)
                pos_coef = _to_i16(r.pos_coef)
                neg_coef = _to_i16(r.neg_coef)
                s, u = self._cond_signed, self._cond_unsigned
                if self._cond_logs < 5:
                    self._cond_logs += 1
                    _say(f"Condition packet: ebi={e_id}, is_y={r.is_y}, centre={centre}, +coef={pos_coef}, -coef={neg_coef}")
                eng.condition(e_id, bool(r.is_y), s(centre), s(pos_coef), s(neg_coef),
                              u(r.pos_sat), u(r.neg_sat), u(r.deadband))
                return

        if t == PT_ENVREP:
            r = _EffEnvlp()
            if self._call(self._envlp, pkt, r):
                e_id = hdr_ebi or r.ebi
                self._log(f"  effect {e_id}: envelope attack {r.attack_level}/{r.attack_time} ms, fade {r.fade_level}/{r.fade_time} ms")
                u = self._env_r
                eng.envelope(e_id, u(r.attack_level), u(r.fade_level), r.attack_time, r.fade_time)
                return

        if t == PT_BLKFRREP:
            idx = ctypes.c_uint32(0)
            if self._call(self._ebi, pkt, idx):
                e_id = int(idx.value & 0xFF)
                self._log(f"  free effect {e_id}")
                eng.block_free(e_id)
                return

        if t == PT_CTRLREP:
            ctrl = ctypes.c_int32(0)
            if self._call(self._ctrl, pkt, ctrl):
                self._log(f"  device control {ctrl.value}")
                eng.device_control(ctrl.value)
                return

        if t == PT_GAINREP:
            g = ctypes.c_ubyte(0)
            if self._call(self._gain, pkt, g):
                self._log(f"  device gain {g.value}")
                eng.device_gain(g.value)
                return


ffb_bridge: "VJoyFfb | None" = None


# =====================================================================
# Shared controller state
# =====================================================================

class ControllerState:
    def __init__(self):
        self.steering = 0.0         # latest target from the phone (-1..1)
        self.throttle = 0.0           # latest requested pedal state (0..1)
        self.brake = 0.0            # latest requested pedal state (0..1)
        self.clutch = 0.0            # latest requested pedal state (0..1)
        self.custom_key_states = {}
        self.last_update = 0.0
        self.connected_addr = None
        self.injection_enabled = START_ENGAGED
        # what is currently written to vJoy (smoothed), for display
        self.out_steer = 0.0
        self.out_throttle = 0.0
        self.out_brake = 0.0
        self.out_clutch = 0.0
        # force feedback (see the FFB sections)
        self.ws = None                  # the connected phone's websocket
        self.ffb_enabled = False        # the phone is in FFB mode
        self.ffb_effects = {}           # effect_id -> bool (which effects vibrate)
        self.ffb_text = ""              # latest FFB numbers, for the console readout
        # Forza mode
        self.forza_mode = FORZA_MODE_DEFAULT
        self.handbrake = 0.0
        self.forza_buttons = {}   # button_id (1-based) -> bool


controller = ControllerState()


@functools.lru_cache(maxsize=1)
def get_local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def shape_steering(v: float) -> float:
    v = max(-1.0, min(1.0, v))
    sign = 1.0 if v >= 0 else -1.0
    a = abs(v)
    if STEER_DEADZONE > 0:
        a = max(0.0, (a - STEER_DEADZONE) / (1.0 - STEER_DEADZONE))
    if STEER_EXPO != 1.0:
        a = a ** STEER_EXPO
    return sign * a


def approach(cur: float, target: float, rise: float, fall: float, dt: float) -> float:
    if target > cur:
        return min(target, cur + rise * dt)
    return max(target, cur - fall * dt)


# =====================================================================
# Output loop — the only place vJoy is written
# =====================================================================

async def output_loop():
    interval = 1.0 / OUTPUT_HZ
    next_tick = time.perf_counter()
    last = next_tick
    while True:
        next_tick += interval
        delay = next_tick - time.perf_counter()
        if delay > 0:
            await asyncio.sleep(delay)
        else:
            next_tick = time.perf_counter()
        now = time.perf_counter()
        dt = now - last
        last = now
        try:
            live = controller.injection_enabled and controller.connected_addr is not None

            # --- steering ---
            if live:
                target = shape_steering(controller.steering)
                if STEER_SMOOTHING > 0:
                    alpha = 1.0 - math.exp(-dt / STEER_SMOOTHING)
                    controller.out_steer += (target - controller.out_steer) * alpha
                else:
                    controller.out_steer = target
            else:
                rate = 1.0 / RECENTER_DURATION
                controller.out_steer = approach(controller.out_steer, 0.0, rate, rate, dt)

            # --- pedals ---
            thr_target = (THROTTLE_MAX * controller.throttle) if live else 0.0
            brk_target = (BRAKE_MAX * controller.brake) if live else 0.0
            clt_target = (CLUTCH_MAX * controller.clutch) if live else 0.0
            
            controller.out_throttle = approach(controller.out_throttle, thr_target, THROTTLE_RISE, THROTTLE_FALL, dt)
            controller.out_brake = approach(controller.out_brake, brk_target, BRAKE_RISE, BRAKE_FALL, dt)
            controller.out_clutch = approach(controller.out_clutch, clt_target, CLUTCH_RISE, CLUTCH_FALL, dt)

            if controller.forza_mode:
                # Forza mode: steer normal, pedals inverted (0=full, max=released)
                vjoy.set_axis(FORZA_STEER_AXIS, (controller.out_steer + 1.0) / 2.0)
                vjoy.set_axis(FORZA_THROTTLE_AXIS, 1.0 - controller.out_throttle)
                vjoy.set_axis(FORZA_BRAKE_AXIS, 1.0 - controller.out_brake)
                vjoy.set_axis(FORZA_CLUTCH_AXIS, 1.0 - controller.out_clutch)
                if FORZA_HANDBRAKE_AXIS in vjoy.max:
                    vjoy.set_axis(FORZA_HANDBRAKE_AXIS, controller.handbrake)
                # Forza buttons (1-based)
                for btn_id, pressed in controller.forza_buttons.items():
                    try:
                        vjoy.set_btn(btn_id, pressed)
                    except Exception:
                        pass
            else:
                vjoy.set_axis(STEER_AXIS, (controller.out_steer + 1.0) / 2.0)
                vjoy.set_axis(THROTTLE_AXIS, controller.out_throttle)
                vjoy.set_axis(BRAKE_AXIS, controller.out_brake)
                vjoy.set_axis(CLUTCH_AXIS, controller.out_clutch)
        except Exception:
            traceback.print_exc()


# =====================================================================
# FFB streaming — effect engine -> phone
# =====================================================================

def ffb_status_message() -> str:
    ok = ffb_bridge is not None and ffb_bridge.ok
    return _dumps({
        "type": "ffb_status",
        "available": ok,
        "reason": "" if ok else (ffb_bridge.reason if ffb_bridge else "force feedback was not started"),
        "hint": ffb_bridge.hint if ok else "",
        "seen": ffb_engine.seen,        # has the game sent any FFB packet yet?
    })


def _cond_for_phone(c: list) -> list:
    """[kind, centre, k+, k-, sat+, sat-, deadband] with the master gain applied to strengths and limits."""
    kind, centre, kpos, kneg, spos, sneg, dead = c
    g = FFB_GAIN
    return [kind, round(centre, 4), round(kpos * g, 4), round(kneg * g, 4), round(spos * g, 4), round(sneg * g, 4), round(dead, 4)]


async def ffb_loop():
    """While the phone is in FFB mode: sample the effect engine and stream the result to it."""
    period = 1.0 / FFB_HZ
    next_tick = time.perf_counter()
    last_live = False
    last_status = None
    while True:
        next_tick += period
        delay = next_tick - time.perf_counter()
        if delay > 0:
            await asyncio.sleep(delay)
        else:
            next_tick = time.perf_counter()
        try:
            s = ffb_engine.sample(time.perf_counter(), controller.ffb_effects)
            ws = controller.ws
            if ws is None or not controller.ffb_enabled:
                controller.ffb_text = ""
                last_live, last_status = False, None
                continue
            if not controller.injection_enabled:        # paused with the hotkey: the phone isn't driving
                s = FfbSample(0.0, 0.0, [], [])
            sign = -1.0 if FFB_INVERT else 1.0
            force = round(_clamp(s.force * FFB_GAIN * sign, -1.0, 1.0), 4)
            vib = round(min(1.0, s.vib * FFB_GAIN), 4)
            if vib < 0.002:                             # the phone ignores anything this faint; don't keep streaming for it
                vib = 0.0
            cond_str = ""
            if s.conds:
                d_max = max((max(abs(c[2]), abs(c[3])) for c in s.conds if c[0] == 2), default=0.0)
                s_max = max((max(abs(c[2]), abs(c[3])) for c in s.conds if c[0] == 1), default=0.0)
                if d_max > 0.001:
                    cond_str += f" D{d_max*100:2.0f}%"
                if s_max > 0.001:
                    cond_str += f" S{s_max*100:2.0f}%"

            if s.types:
                names = ",".join(EFFECT_NAMES.get(t, str(t)) for t in s.types)
            else:
                n_fx = len(ffb_engine._fx)
                names = f"idle({n_fx}fx)" if n_fx else "idle"
            controller.ffb_text = f"  FFB F{force:+.2f} V{vib*100:2.0f}%{cond_str} {names} [{ffb_engine.packets}pkts]"

            status = (id(ws), ffb_engine.seen, ffb_bridge.hint if ffb_bridge else "")
            if status != last_status:
                last_status = status
                await ws.send(ffb_status_message())
            live = abs(force) > 1e-4 or vib > 1e-4 or bool(s.conds)
            if live or last_live:                       # silent while idle; one final all-zero message tells the phone to stop
                msg = {"type": "ffb", "f": force, "v": vib, "n": len(s.types), "t": s.types,
                       "c": [_cond_for_phone(c) for c in s.conds]}
                await ws.send(_dumps_compact(msg))
                last_live = live
        except websockets.exceptions.ConnectionClosed:
            last_live, last_status = False, None
        except Exception:
            traceback.print_exc()


# =====================================================================
# Keyboard (custom buttons)
# =====================================================================

def apply_custom_keys(custom_keys: dict) -> None:
    states = controller.custom_key_states
    for name, pressed in custom_keys.items():
        pressed = bool(pressed)
        if states.get(name, False) == pressed:
            continue
        resolved = resolve_key(name)
        if resolved is None:
            continue
        (keyboard.press if pressed else keyboard.release)(resolved)
        states[name] = pressed


def release_all_keys() -> None:
    for name, pressed in list(controller.custom_key_states.items()):
        if pressed:
            try:
                keyboard.release(resolve_key(name))
            except Exception:
                pass
        controller.custom_key_states[name] = False


def release_all_forza_buttons() -> None:
    if vjoy:
        for btn_id in list(controller.forza_buttons.keys()):
            try:
                vjoy.set_btn(btn_id, False)
            except Exception:
                pass
    controller.forza_buttons = {}


def toggle_injection() -> None:
    controller.injection_enabled = not controller.injection_enabled
    if controller.injection_enabled:
        print(f"\n[{PAUSE_HOTKEY_NAME}] ENGAGED — phone controls the vJoy device")
    else:
        release_all_keys()
        release_all_forza_buttons()
        print(f"\n[{PAUSE_HOTKEY_NAME}] PAUSED — wheel centered, pedals released")


# =====================================================================
# Global pause/resume hotkey
# =====================================================================

user32 = ctypes.windll.user32
WM_HOTKEY = 0x0312
_HOTKEY_ID = 1


def _hotkey_listener():
    if not user32.RegisterHotKey(None, _HOTKEY_ID, 0, PAUSE_HOTKEY_VK):
        print(f"[!] Could not register the {PAUSE_HOTKEY_NAME} hotkey — another app may be using it.")
        return
    try:
        msg = wintypes.MSG()
        while True:
            ret = user32.GetMessageW(ctypes.byref(msg), None, 0, 0)
            if ret == 0:
                break
            if msg.message == WM_HOTKEY and msg.wParam == _HOTKEY_ID:
                toggle_injection()
            user32.TranslateMessage(ctypes.byref(msg))
            user32.DispatchMessageW(ctypes.byref(msg))
    finally:
        user32.UnregisterHotKey(None, _HOTKEY_ID)


def start_hotkey_listener() -> None:
    threading.Thread(target=_hotkey_listener, daemon=True).start()


# =====================================================================
# Watchdog
# =====================================================================

async def watchdog():
    while True:
        await asyncio.sleep(0.1)
        if controller.connected_addr and (time.perf_counter() - controller.last_update) > WATCHDOG_TIMEOUT:
            print("\n[!] No signal for {:.1f}s — centering wheel and releasing inputs".format(WATCHDOG_TIMEOUT))
            release_all_keys()
            release_all_forza_buttons()
            controller.connected_addr = None


# =====================================================================
# WebSocket handler
# =====================================================================

async def handle_client(websocket):
    addr = websocket.remote_address
    if controller.ws is not None:
        print(f"\n[!] Rejecting connection from {addr} — another phone is already connected")
        await websocket.close(4001, "Another phone is already connected")
        return
    print(f"\n[+] Phone connected: {addr}")
    controller.connected_addr = addr
    controller.ws = websocket
    controller.ffb_enabled = False      # until this phone says otherwise (it sends its mode right after connecting)
    controller.ffb_effects = {}
    _print_skip = 0
    try:
        async for message in websocket:
            try:
                data = _loads(message)
            except (json.JSONDecodeError, ValueError):
                continue
            if not isinstance(data, dict):
                continue

            if "type" in data:              # a control message from the phone, not a telemetry frame
                if data["type"] == "config":
                    wanted_ffb = bool(data.get("ffb", False))
                    if wanted_ffb != controller.ffb_enabled:
                        controller.ffb_enabled = wanted_ffb
                        print(f"\n[FFB] phone switched to {'FORCE FEEDBACK' if wanted_ffb else 'NORMAL'} mode")
                    wanted_forza = bool(data.get("forza", controller.forza_mode))
                    if wanted_forza != controller.forza_mode:
                        controller.forza_mode = wanted_forza
                        print(f"\n[FORZA] Forza mode {'ENABLED' if wanted_forza else 'DISABLED'}")
                    if "ffb_effects" in data and isinstance(data["ffb_effects"], dict):
                        controller.ffb_effects = {str(k): bool(v) for k, v in data["ffb_effects"].items()}
                continue

            steer = float(data.get("Normalized_Steering", 0.0))
            custom_keys = data.get("Custom_Keys", {})
            if not isinstance(custom_keys, dict):
                custom_keys = {}

            controller.steering = steer
            controller.throttle = float(data.get("Throttle_Value", 1.0 if data.get("Throttle_State", False) else 0.0))
            controller.brake = float(data.get("Brake_Value", 1.0 if data.get("Brake_State", False) else 0.0))
            controller.clutch = float(data.get("Clutch_Value", 1.0 if data.get("Clutch_State", False) else 0.0))
            controller.handbrake = float(data.get("Handbrake_Value", 1.0 if data.get("Handbrake_State", False) else 0.0))
            forza_btns = data.get("Forza_Buttons", {})
            if isinstance(forza_btns, dict):
                controller.forza_buttons = {int(k): bool(v) for k, v in forza_btns.items() if str(k).isdigit()}
            controller.connected_addr = addr
            controller.last_update = time.perf_counter()

            if controller.injection_enabled:
                try:
                    apply_custom_keys(custom_keys)
                except Exception:
                    traceback.print_exc()
                status = ""
            else:
                status = f" PAUSED — press {PAUSE_HOTKEY_NAME} to engage"

            if controller.forza_mode:
                active_forza = ",".join(f"B{k}" for k, v in controller.forza_buttons.items() if v) or "-"
                btn_display = f"FORZA [{active_forza}]"
            else:
                active_keys = ",".join(k for k, v in custom_keys.items() if v) or "-"
                btn_display = f"KEYS [{active_keys}]"
            ffb_text = controller.ffb_text if controller.ffb_enabled else ""
            _print_skip += 1
            if _print_skip % 15 == 0:
                print(
                    f"\rSTEER {steer:+.2f} -> {controller.out_steer:+.2f}  "
                    f"THR {controller.out_throttle*100:3.0f}%  BRK {controller.out_brake*100:3.0f}%  CLT {controller.out_clutch*100:3.0f}%  "
                    f"{btn_display}{ffb_text}{status}      ",
                    end="", flush=True,
                )
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print(f"\n[-] Phone disconnected: {addr}")
        release_all_keys()
        release_all_forza_buttons()
        controller.connected_addr = None
        if controller.ws is websocket:
            controller.ws = None
            controller.ffb_enabled = False
            controller.ffb_effects = {}
            # Reset Forza state
            controller.handbrake = 0.0


# =====================================================================
# Static file server — in-memory cache with gzip + Cache-Control
# =====================================================================

class _CachedHTTPHandler(http.server.BaseHTTPRequestHandler):
    """Serves index.html, favicon.ico, wheel.png, and static assets from memory with gzip and Cache-Control."""
    _cache: dict = {}

    @classmethod
    def load_cache(cls, directory: Path):
        _TYPES = {
            '.html': 'text/html; charset=utf-8',
            '.ico': 'image/x-icon',
            '.png': 'image/png',
            '.webp': 'image/webp',
            '.svg': 'image/svg+xml',
        }
        for name in ("index.html", "favicon.ico", "wheel.png"):
            path = directory / name
            if not path.exists():
                continue
            raw = path.read_bytes()
            ct = _TYPES.get(path.suffix, 'application/octet-stream')
            try:
                gz = _gzip_mod.compress(raw, compresslevel=9) if path.suffix in ('.html', '.svg') else None
            except Exception:
                gz = None
            cls._cache[f"/{name}"] = (raw, gz, ct)
            if name == "index.html":
                cls._cache["/"] = (raw, gz, ct)

    def do_GET(self):
        path = self.path.split("?")[0].split("#")[0]
        entry = self._cache.get(path)
        if entry is None:
            # Fallback to loading static asset directly from disk if it exists
            file_path = (Path(__file__).resolve().parent / path.lstrip("/")).resolve()
            if file_path.is_file() and file_path.parent == Path(__file__).resolve().parent:
                _TYPES = {
                    '.html': 'text/html; charset=utf-8',
                    '.ico': 'image/x-icon',
                    '.png': 'image/png',
                    '.webp': 'image/webp',
                    '.svg': 'image/svg+xml',
                }
                raw = file_path.read_bytes()
                ct = _TYPES.get(file_path.suffix, 'application/octet-stream')
                entry = self._cache[path] = (raw, None, ct)
            else:
                self.send_error(404)
                return
        raw, gz, ct = entry
        ae = self.headers.get("Accept-Encoding", "")
        if gz and "gzip" in ae:
            body = gz
            self.send_response(200)
            self.send_header("Content-Encoding", "gzip")
        else:
            body = raw
            self.send_response(200)
        self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "max-age=3600")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        pass  # suppress per-request access logs


def start_http_server(directory: Path) -> http.server.ThreadingHTTPServer:
    _CachedHTTPHandler.load_cache(directory)
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), _CachedHTTPHandler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


# =====================================================================
# Entry point
# =====================================================================

async def main():
    global vjoy, ffb_bridge
    script_dir = Path(__file__).resolve().parent
    if not (script_dir / "index.html").exists():
        print(f"WARNING: index.html not found in {script_dir} — the phone won't have anything to load.")

    try:
        vjoy = VJoy(VJOY_DEVICE_ID)
        for axis in {STEER_AXIS, THROTTLE_AXIS, BRAKE_AXIS, CLUTCH_AXIS,
                     FORZA_THROTTLE_AXIS, FORZA_BRAKE_AXIS, FORZA_CLUTCH_AXIS, FORZA_HANDBRAKE_AXIS}:
            try:
                vjoy.init_axis(axis)
            except VJoyError as e:
                print(f"  [!] Axis {axis}: {e}")
    except VJoyError as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    # Windows timers default to ~15.6ms; ask for 1ms so the 120Hz loop is real.
    try:
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

    # Start with a centered wheel and released pedals.
    vjoy.set_axis(STEER_AXIS, 0.5)
    vjoy.set_axis(THROTTLE_AXIS, 0.0)
    vjoy.set_axis(BRAKE_AXIS, 0.0)
    vjoy.set_axis(CLUTCH_AXIS, 0.0)
    # Also initialise Forza axes to their "released" state (inverted: 1.0 = released for pedals)
    try:
        vjoy.set_axis(FORZA_THROTTLE_AXIS, 1.0)
        vjoy.set_axis(FORZA_BRAKE_AXIS, 1.0)
        vjoy.set_axis(FORZA_CLUTCH_AXIS, 1.0)
        vjoy.set_axis(FORZA_HANDBRAKE_AXIS, 0.0)
    except Exception:
        pass

    # Force feedback is optional: if it can't start, steering and pedals still work.
    ffb_bridge = VJoyFfb(vjoy, ffb_engine)
    try:
        ffb_ready = ffb_bridge.start()
    except Exception as e:
        ffb_bridge.ok, ffb_bridge.reason = False, f"could not start force feedback ({e})"
        ffb_ready = False

    start_http_server(script_dir)
    start_hotkey_listener()
    local_ip = get_local_ip()

    print("=" * 64)
    print(" KyberLink — vJoy server running")
    print("=" * 64)
    print(f" 1. On your phone, open:      http://{local_ip}:{HTTP_PORT}/")
    print(f" 2. In the app's connection field, enter:   {local_ip}   port {WS_PORT}")
    print(f"    (or open  http://{local_ip}:{HTTP_PORT}/?host={local_ip}&port={WS_PORT}  to skip typing it)")
    print(f" vJoy device {VJOY_DEVICE_ID}: steer={STEER_AXIS} throttle={THROTTLE_AXIS} brake={BRAKE_AXIS} clutch={CLUTCH_AXIS}")
    if ffb_ready:
        print(" Force feedback: ready - choose FFB in the phone's settings (gear icon)")
    else:
        print(f" Force feedback: unavailable - {ffb_bridge.reason}")
    print(f" {PAUSE_HOTKEY_NAME} toggles engage/pause.  Ctrl+C to stop.")
    print("=" * 64)

    asyncio.create_task(output_loop())
    asyncio.create_task(watchdog())
    asyncio.create_task(ffb_loop())
    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT, max_size=4096, ping_interval=5, ping_timeout=3):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        release_all_keys()
        release_all_forza_buttons()
        if vjoy:
            vjoy.close()
        try:
            ctypes.windll.winmm.timeEndPeriod(1)
        except Exception:
            pass
