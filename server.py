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

A safety watchdog centers the wheel, zeroes the pedals and releases all keys
if the phone disconnects or goes quiet.

Windows only. Requires the vJoy driver (https://github.com/jshafer817/vJoy,
the maintained 2.1.9.x fork) with device 1 enabled and axes X, Y, Z.
"""

import asyncio
import ctypes
import functools
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
THROTTLE_MAX = 1.0
BRAKE_MAX = 1.0

WATCHDOG_TIMEOUT = 0.75     # seconds of silence before inputs are released
RECENTER_DURATION = 0.25    # seconds for the wheel to return from full lock after a disconnect/pause

START_ENGAGED = True        # False = start paused until you press the hotkey
PAUSE_HOTKEY_VK = 0x78      # F9
PAUSE_HOTKEY_NAME = "F9"


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

    def close(self) -> None:
        try:
            self.dll.ResetVJD(self.id)
            self.dll.RelinquishVJD(self.id)
        except Exception:
            pass


vjoy: "VJoy | None" = None


# =====================================================================
# Shared controller state
# =====================================================================

class ControllerState:
    def __init__(self):
        self.steering = 0.0         # latest target from the phone (-1..1)
        self.throttle = False       # latest requested pedal states
        self.brake = False
        self.custom_key_states = {}
        self.last_update = 0.0
        self.connected_addr = None
        self.injection_enabled = START_ENGAGED
        # what is currently written to vJoy (smoothed), for display
        self.out_steer = 0.0
        self.out_throttle = 0.0
        self.out_brake = 0.0


controller = ControllerState()


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
    last = time.perf_counter()
    while True:
        await asyncio.sleep(1.0 / OUTPUT_HZ)
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
            thr_target = THROTTLE_MAX if (live and controller.throttle) else 0.0
            brk_target = BRAKE_MAX if (live and controller.brake) else 0.0
            controller.out_throttle = approach(controller.out_throttle, thr_target, THROTTLE_RISE, THROTTLE_FALL, dt)
            controller.out_brake = approach(controller.out_brake, brk_target, BRAKE_RISE, BRAKE_FALL, dt)

            vjoy.set_axis(STEER_AXIS, (controller.out_steer + 1.0) / 2.0)
            vjoy.set_axis(THROTTLE_AXIS, controller.out_throttle)
            vjoy.set_axis(BRAKE_AXIS, controller.out_brake)
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


def toggle_injection() -> None:
    controller.injection_enabled = not controller.injection_enabled
    if controller.injection_enabled:
        print(f"\n[{PAUSE_HOTKEY_NAME}] ENGAGED — phone controls the vJoy device")
    else:
        release_all_keys()
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
        if controller.connected_addr and (time.time() - controller.last_update) > WATCHDOG_TIMEOUT:
            print("\n[!] No signal for {:.1f}s — centering wheel and releasing inputs".format(WATCHDOG_TIMEOUT))
            release_all_keys()
            controller.connected_addr = None


# =====================================================================
# WebSocket handler
# =====================================================================

async def handle_client(websocket):
    addr = websocket.remote_address
    print(f"\n[+] Phone connected: {addr}")
    controller.connected_addr = addr
    try:
        async for message in websocket:
            try:
                data = json.loads(message)
            except json.JSONDecodeError:
                continue

            steer = float(data.get("Normalized_Steering", 0.0))
            custom_keys = data.get("Custom_Keys", {})
            if not isinstance(custom_keys, dict):
                custom_keys = {}

            controller.steering = steer
            controller.throttle = bool(data.get("Throttle_State", False))
            controller.brake = bool(data.get("Brake_State", False))
            controller.connected_addr = addr
            controller.last_update = time.time()

            if controller.injection_enabled:
                try:
                    apply_custom_keys(custom_keys)
                except Exception:
                    traceback.print_exc()
                status = ""
            else:
                status = f" PAUSED — press {PAUSE_HOTKEY_NAME} to engage"

            active_keys = ",".join(k for k, v in custom_keys.items() if v) or "-"
            print(
                f"\rSTEER {steer:+.2f} -> {controller.out_steer:+.2f}  "
                f"THR {controller.out_throttle*100:3.0f}%  BRK {controller.out_brake*100:3.0f}%  "
                f"KEYS [{active_keys}]{status}      ",
                end="", flush=True,
            )
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print(f"\n[-] Phone disconnected: {addr}")
        release_all_keys()
        controller.connected_addr = None


# =====================================================================
# Static file server
# =====================================================================

def start_http_server(directory: Path) -> http.server.ThreadingHTTPServer:
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(directory))
    httpd = http.server.ThreadingHTTPServer(("0.0.0.0", HTTP_PORT), handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd


# =====================================================================
# Entry point
# =====================================================================

async def main():
    global vjoy
    script_dir = Path(__file__).resolve().parent
    if not (script_dir / "index.html").exists():
        print(f"WARNING: index.html not found in {script_dir} — the phone won't have anything to load.")

    try:
        vjoy = VJoy(VJOY_DEVICE_ID)
        for axis in {STEER_AXIS, THROTTLE_AXIS, BRAKE_AXIS}:
            vjoy.init_axis(axis)
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

    start_http_server(script_dir)
    start_hotkey_listener()
    local_ip = get_local_ip()

    print("=" * 64)
    print(" KyberLink — vJoy server running")
    print("=" * 64)
    print(f" 1. On your phone, open:      http://{local_ip}:{HTTP_PORT}/")
    print(f" 2. In the app's connection field, enter:   {local_ip}   port {WS_PORT}")
    print(f"    (or open  http://{local_ip}:{HTTP_PORT}/?host={local_ip}&port={WS_PORT}  to skip typing it)")
    print(f" vJoy device {VJOY_DEVICE_ID}: steer={STEER_AXIS} throttle={THROTTLE_AXIS} brake={BRAKE_AXIS}")
    print(f" {PAUSE_HOTKEY_NAME} toggles engage/pause.  Ctrl+C to stop.")
    print("=" * 64)

    asyncio.create_task(output_loop())
    asyncio.create_task(watchdog())
    async with websockets.serve(handle_client, "0.0.0.0", WS_PORT, max_size=None):
        await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        release_all_keys()
        if vjoy:
            vjoy.close()
