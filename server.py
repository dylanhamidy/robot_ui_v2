import atexit
import asyncio
import json
import os
import signal
import subprocess
import time
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path
from typing import Optional

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

ARDUINO_VIDS = {
    0x2341,  # Arduino LLC (Uno, Mega, etc.)
    0x1A86,  # QinHeng CH340/CH341 (clones/Nanos)
    0x0403,  # FTDI FT232 (older boards)
}

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

import sys
# Nuitka --onefile extracts to a temp dir; __file__ points there instead of /app.
# Fall back to the binary's own directory so ui/, plans/, stats/ are found correctly.
BASE = Path(__file__).parent
if not (BASE / "ui").exists():
    BASE = Path(sys.argv[0]).resolve().parent

DRFL_DAEMON_BIN = os.environ.get(
    "DRFL_DAEMON_BIN",
    str(BASE / "lux_drfl_daemon" / "build" / "drfl_daemon"),
)
ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.0.20")
PC_IP = os.environ.get("PC_IP", "192.168.0.50")
PLANS_DIR = BASE / "plans"
STATS_DIR = BASE / "stats"
PLANS_DIR.mkdir(exist_ok=True)
STATS_DIR.mkdir(exist_ok=True)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    scan = asyncio.create_task(_turntable_scan_task())
    watchdog = asyncio.create_task(_turntable_watchdog_task())
    serial_read = asyncio.create_task(_tt_serial_read_task())
    yield
    scan.cancel()
    watchdog.cancel()
    serial_read.cancel()
    try:
        await asyncio.gather(scan, watchdog, serial_read)
    except asyncio.CancelledError:
        pass
    _kill_robot_procs()


app = FastAPI(lifespan=lifespan)

# ── state ──────────────────────────────────────────────────────────────────
_drfl_proc: Optional[subprocess.Popen] = None
_drfl_lock = asyncio.Lock()

# Signalling events for the background stdout reader
_drfl_connect_event: Optional[asyncio.Event] = None
_drfl_connect_success: bool = False
_drfl_done_event: Optional[asyncio.Event] = None
_drfl_done_result: int = 0      # 0 = ok, -1 = error/cancel
_drfl_on_line = None            # async callback(text) per stdout line

_active_plan: Optional[str] = None
_captured_points: list = []
_connected: bool = False
_stop_requested: bool = False
_ws_clients: list[WebSocket] = []
_disconnect_task: Optional[asyncio.Task] = None

# ── turntable state ────────────────────────────────────────────────────────
_turntable = None  # serial.Serial instance when connected
_tt_enabled: bool = False
_tt_direction: str = "CW"
_tt_speed: int = 50  # microseconds between pulses
_tt_pending_port: Optional[str] = None
_tt_rejected_ports: set = set()
_emg_state: Optional[int] = None  # None=unknown, 1=normal, 0=triggered

# Pose capture signalling (capture_pose endpoint)
_pose_event: Optional[asyncio.Event] = None
_pose_result: Optional[list] = None


# ── helpers ────────────────────────────────────────────────────────────────

def _kill_robot_procs():
    """Kill DRFL daemon. Synchronous and idempotent — safe to call from atexit."""
    global _drfl_proc
    if _drfl_proc and _drfl_proc.poll() is None:
        try:
            os.killpg(os.getpgid(_drfl_proc.pid), signal.SIGINT)
            _drfl_proc.wait(timeout=3)
        except (ProcessLookupError, subprocess.TimeoutExpired, OSError):
            pass

def _close_turntable():
    global _turntable, _tt_enabled
    if _turntable is not None:
        try:
            if _turntable.is_open:
                _turntable.write(b"DISABLE\n")
                _turntable.close()
        except Exception:
            pass
    _turntable = None
    _tt_enabled = False

atexit.register(_kill_robot_procs)
atexit.register(_close_turntable)

async def _turntable_scan_task():
    """Scan for Arduino by VID every 2 s; set _tt_pending_port when found."""
    global _tt_pending_port
    while True:
        if serial is not None and not (_turntable and _turntable.is_open):
            try:
                ports = serial.tools.list_ports.comports()
                port_devices = {p.device for p in ports}
                if _tt_pending_port and _tt_pending_port not in port_devices:
                    _tt_pending_port = None
                if _tt_pending_port is None:
                    for p in ports:
                        if p.vid in ARDUINO_VIDS and p.device not in _tt_rejected_ports:
                            _tt_pending_port = p.device
                            break
            except Exception:
                pass
        await asyncio.sleep(2)

async def _turntable_watchdog_task():
    """Detect Arduino unplug by polling in_waiting every 2 s."""
    while True:
        if _turntable and _turntable.is_open:
            try:
                _ = _turntable.in_waiting
            except Exception:
                _close_turntable()
        await asyncio.sleep(2)

async def _handle_emergency():
    global _stop_requested, _tt_enabled, _active_plan
    if _turntable and _turntable.is_open:
        try:
            _turntable.write(b"DISABLE\n")
            _turntable.write(b"LAS:DIS\n")
        except Exception:
            pass
    _tt_enabled = False
    _stop_requested = True
    await _drfl_send({"cmd": "stop"})
    _active_plan = None
    await _broadcast("[EMERGENCY STOP]\n")

async def _debounced_emg_clear():
    await asyncio.sleep(0.2)
    if _emg_state == 1:
        await _broadcast("[EMG_CLEAR]\n")

async def _tt_serial_read_task():
    """Read EMG lines from Arduino; trigger emergency stop on EMG:0."""
    global _emg_state
    loop = asyncio.get_event_loop()
    while True:
        if _turntable and _turntable.is_open:
            try:
                if _turntable.in_waiting > 0:
                    line = await loop.run_in_executor(None, _turntable.readline)
                    text = line.decode(errors="replace").strip()
                    if text.startswith("EMG:"):
                        val = int(text.split(":")[1])
                        prev = _emg_state
                        _emg_state = val
                        await _broadcast(f"[EMG] {val}\n")
                        if val == 0 and prev != 0:
                            await _handle_emergency()
                        elif val == 1 and prev == 0:
                            asyncio.create_task(_debounced_emg_clear())
            except Exception:
                pass
        await asyncio.sleep(0.005)

async def _schedule_safety_shutdown():
    """Grace-period watchdog: if no browser client reconnects within 8 s, stop the robot."""
    await asyncio.sleep(8)
    global _connected, _active_plan
    if _ws_clients:
        return
    _kill_robot_procs()
    _connected = False
    _active_plan = None

def _plan_path(name: str) -> Path:
    return PLANS_DIR / f"{name}.json"

def _stats_path(name: str) -> Path:
    return STATS_DIR / f"{name}.json"

def _coerce_step_floats(step: dict) -> dict:
    """Ensure all numeric fields in a step are stored as the correct Python types."""
    if step.get("type") in ("Turntable", "Laser"):
        if "speed_us" in step:
            step["speed_us"] = int(step["speed_us"])
        if "duration" in step:
            step["duration"] = float(step["duration"])
        if "with_laser" in step:
            step["with_laser"] = bool(step["with_laser"])
        return step
    if step.get("type") == "WeldStraight":
        step["pos_a"] = [float(v) for v in step["pos_a"]]
        step["pos_b"] = [float(v) for v in step["pos_b"]]
        for key in ("vel", "acc"):
            if key in step:
                v = step[key]
                step[key] = [float(x) for x in v] if isinstance(v, list) else float(v)
        for key in ("time", "laser_delay"):
            if key in step:
                step[key] = float(step[key])
        if "with_laser" in step:
            step["with_laser"] = bool(step["with_laser"])
        return step
    if step.get("type") == "MoveC":
        for key in ("pos_start", "pos_via"):
            if key in step and step[key] is not None:
                step[key] = [float(v) for v in step[key]]
        if step.get("pos_end") is not None:
            step["pos_end"] = [float(v) for v in step["pos_end"]]
        for key in ("vel", "acc"):
            if key in step:
                v = step[key]
                step[key] = [float(x) for x in v] if isinstance(v, list) else float(v)
        for key in ("time", "angle1", "angle2"):
            if key in step:
                step[key] = float(step[key])
        if "with_laser" in step:
            step["with_laser"] = bool(step["with_laser"])
        return step
    if step.get("type") == "FreeForm":
        if "blend_radius" in step: step["blend_radius"] = float(step["blend_radius"])
        if "laser_delay" in step: step["laser_delay"] = float(step["laser_delay"])
        if "with_laser" in step: step["with_laser"] = bool(step["with_laser"])
        if "sub_steps" in step:
            for ss in step["sub_steps"]:
                sstype = ss.get("type")
                if sstype == "MoveL" and ss.get("pos") is not None:
                    ss["pos"] = [float(v) for v in ss["pos"]]
                elif sstype == "MoveC":
                    if ss.get("pos_via") is not None: ss["pos_via"] = [float(v) for v in ss["pos_via"]]
                    if ss.get("pos_end") is not None: ss["pos_end"] = [float(v) for v in ss["pos_end"]]
                    ss.pop("pos_start", None)
                for k in ("vel", "acc"):
                    if k in ss: ss[k] = [float(x) for x in ss[k]] if isinstance(ss[k], list) else float(ss[k])
                if "time" in ss: ss["time"] = float(ss["time"])
                if "blend_radius" in ss: ss["blend_radius"] = float(ss["blend_radius"])
        return step
    if "pos" in step:
        step["pos"] = [float(v) for v in step["pos"]]
    for key in ("vel", "acc"):
        if key in step:
            v = step[key]
            step[key] = [float(x) for x in v] if isinstance(v, list) else float(v)
    if "time" in step:
        step["time"] = float(step["time"])
    if "delay" in step:
        step["delay"] = float(step["delay"])
    if "laser_delay" in step:
        step["laser_delay"] = float(step["laser_delay"])
    if "with_laser" in step:
        step["with_laser"] = bool(step["with_laser"])
    return step

def _load_stats(name: str) -> dict:
    p = _stats_path(name)
    if p.exists():
        return json.loads(p.read_text())
    return {"total_runs": 0, "success": 0, "fail": 0, "unknown": 0, "history": []}

def _save_stats(name: str, stats: dict):
    _stats_path(name).write_text(json.dumps(stats, indent=2))

def _record_stat(name: str, result: str):
    stats = _load_stats(name)
    stats["total_runs"] += 1
    stats[result] = stats.get(result, 0) + 1
    stats["history"].append({"timestamp": datetime.now().isoformat(timespec="seconds"), "result": result})
    _save_stats(name, stats)

async def _broadcast(msg: str):
    dead = []
    for ws in _ws_clients:
        try:
            await ws.send_text(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        _ws_clients.remove(ws)

async def _stream_proc(proc: subprocess.Popen, on_line=None):
    loop = asyncio.get_event_loop()
    while True:
        line = await loop.run_in_executor(None, proc.stdout.readline)
        if not line:
            break
        text = line.decode(errors="replace")
        await _broadcast(text)
        if on_line:
            await on_line(text)


# ── DRFL daemon IPC ────────────────────────────────────────────────────────

async def _drfl_send(cmd: dict) -> bool:
    """Write one JSON command to daemon stdin under lock. Returns False if daemon dead."""
    async with _drfl_lock:
        if _drfl_proc is None or _drfl_proc.poll() is not None:
            return False
        try:
            line = json.dumps(cmd) + "\n"
            _drfl_proc.stdin.write(line.encode())
            _drfl_proc.stdin.flush()
            return True
        except OSError:
            return False

async def _drfl_reader_task():
    """Continuously read daemon stdout, broadcast to WS, fire signalling events."""
    global _drfl_connect_event, _drfl_connect_success
    global _drfl_done_event, _drfl_done_result, _drfl_on_line

    loop = asyncio.get_event_loop()
    while _drfl_proc is not None:
        line = await loop.run_in_executor(None, _drfl_proc.stdout.readline)
        if not line:
            break
        text = line.decode(errors="replace")
        await _broadcast(text)

        # Connection phase — signal success or failure so _drfl_start() can unblock
        ev = _drfl_connect_event
        if ev is not None and not ev.is_set():
            if "[CONNECTED]" in text:
                _drfl_connect_success = True
                ev.set()
            elif "[ERROR]" in text:
                _drfl_connect_success = False
                ev.set()

        # Plan phase — signal [DONE] or [ERROR] so _run_robot_segment() can unblock
        dev = _drfl_done_event
        if dev is not None and not dev.is_set():
            if "[DONE]" in text:
                _drfl_done_result = 0
                dev.set()
            elif "[ERROR]" in text:
                _drfl_done_result = -1
                dev.set()

        # Pose capture — unblock capture_pose endpoint
        global _pose_event, _pose_result
        pe = _pose_event
        if pe is not None and not pe.is_set() and "[POSE]" in text:
            try:
                raw = text.split("[POSE]")[1].strip()
                _pose_result = json.loads(raw)
            except Exception:
                _pose_result = None
            pe.set()

        # Per-line callback (parallel turntable mode passes tt_step_callback here)
        cb = _drfl_on_line
        if cb:
            await cb(text)

async def _drfl_start() -> bool:
    """Spawn daemon process, start reader task, wait up to 30 s for [CONNECTED]."""
    global _drfl_proc, _drfl_connect_event, _drfl_connect_success

    _drfl_connect_event = asyncio.Event()
    _drfl_connect_success = False

    _drfl_proc = subprocess.Popen(
        [DRFL_DAEMON_BIN, "--ip", ROBOT_IP, "--port", "12345"],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
    )

    asyncio.get_event_loop().create_task(_drfl_reader_task())

    try:
        await asyncio.wait_for(_drfl_connect_event.wait(), timeout=30.0)
    except asyncio.TimeoutError:
        await _broadcast("[ERROR] DRFL daemon connection timeout\n")
        _drfl_connect_event = None
        return False

    _drfl_connect_event = None
    return _drfl_connect_success


# ── routes ─────────────────────────────────────────────────────────────────

app.mount("/ui", StaticFiles(directory=BASE / "ui"), name="ui")

@app.get("/")
async def index():
    return FileResponse(BASE / "ui" / "index.html")

@app.get("/api/plans")
async def list_plans():
    result = []
    for p in sorted(PLANS_DIR.glob("*.json")):
        plan = json.loads(p.read_text())
        stats = _load_stats(plan["name"])
        result.append({**plan, "stats": stats})
    return result

class PlanBody(BaseModel):
    name: str
    steps: list
    turntable_parallel: Optional[dict] = None
    loop: bool = False

@app.post("/api/plans")
async def create_plan(body: PlanBody):
    p = _plan_path(body.name)
    if p.exists():
        raise HTTPException(400, "Plan already exists")
    steps = [_coerce_step_floats(s) for s in body.steps]
    data = {"name": body.name, "created_at": datetime.now().isoformat(timespec="seconds"), "steps": steps, "loop": body.loop}
    if body.turntable_parallel is not None:
        data["turntable_parallel"] = body.turntable_parallel
    p.write_text(json.dumps(data, indent=2))
    await _broadcast(f"[PLAN_IMPORTED] {data['name']}\n")
    return data

class ImportBody(BaseModel):
    name: str
    steps: list
    created_at: Optional[str] = None

@app.post("/api/plans/import")
async def import_plan(body: ImportBody):
    data = body.model_dump()
    if not data["created_at"]:
        data["created_at"] = datetime.now().isoformat(timespec="seconds")
    data["steps"] = [_coerce_step_floats(s) for s in data["steps"]]
    _plan_path(data["name"]).write_text(json.dumps(data, indent=2))
    await _broadcast(f"[PLAN_IMPORTED] {data['name']}\n")
    return {"ok": True, "name": data["name"]}

@app.get("/api/plans/{name}")
async def get_plan(name: str):
    p = _plan_path(name)
    if not p.exists():
        raise HTTPException(404, "Not found")
    return json.loads(p.read_text())

class UpdateBody(BaseModel):
    steps: list
    turntable_parallel: Optional[dict] = None
    loop: bool = False

@app.put("/api/plans/{name}")
async def update_plan(name: str, body: UpdateBody):
    p = _plan_path(name)
    if not p.exists():
        raise HTTPException(404, "Not found")
    data = json.loads(p.read_text())
    data["steps"] = [_coerce_step_floats(s) for s in body.steps]
    data["loop"] = body.loop
    if body.turntable_parallel is not None:
        data["turntable_parallel"] = body.turntable_parallel
    else:
        data.pop("turntable_parallel", None)
    p.write_text(json.dumps(data, indent=2))
    return data

@app.delete("/api/plans/{name}")
async def delete_plan(name: str):
    p = _plan_path(name)
    if not p.exists():
        raise HTTPException(404, "Not found")
    p.unlink()
    sp = _stats_path(name)
    if sp.exists():
        sp.unlink()
    return {"ok": True}

# ── robot control ──────────────────────────────────────────────────────────

class ConnectBody(BaseModel):
    sudo_password: str
    interface: str = "enp2s0"

@app.post("/api/robot/connect")
async def robot_connect(body: ConnectBody):
    global _connected
    pw = body.sudo_password
    iface = body.interface

    async def run_step(cmd: str, label: str):
        await _broadcast(f"\n[STEP] {label}\n")
        proc = await asyncio.create_subprocess_shell(
            cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.STDOUT
        )
        async for chunk in proc.stdout:
            await _broadcast(chunk.decode(errors="replace"))
        rc = await proc.wait()
        if rc != 0:
            raise RuntimeError(f"{label} failed (exit {rc})")

    daemon_bin = Path(DRFL_DAEMON_BIN)
    daemon_src_dir = BASE / "lux_drfl_daemon"

    def _daemon_needs_build() -> bool:
        if not daemon_bin.exists():
            return True
        src_files = list((daemon_src_dir / "src").glob("**/*.cpp")) + \
                    list((daemon_src_dir / "src").glob("**/*.h")) + \
                    [daemon_src_dir / "CMakeLists.txt"]
        bin_mtime = daemon_bin.stat().st_mtime
        return any(f.exists() and f.stat().st_mtime > bin_mtime for f in src_files)

    try:
        if _daemon_needs_build():
            submodule_marker = daemon_src_dir / "third_party" / "API-DRFL" / "DRFLEx.h"
            if not submodule_marker.exists():
                await run_step(
                    f"git -C {BASE} submodule update --init lux_drfl_daemon/third_party/API-DRFL",
                    "Initialising API-DRFL submodule"
                )
            await run_step(
                f"cmake -B {daemon_src_dir}/build -DDRCF_VERSION=3 {daemon_src_dir}",
                "Configuring DRFL daemon (cmake)"
            )
            await run_step(
                f"cmake --build {daemon_src_dir}/build -j$(nproc)",
                "Building DRFL daemon"
            )

        await run_step(
            f"echo '{pw}' | sudo -S ip addr flush dev {iface} && "
            f"echo '{pw}' | sudo -S ip link set {iface} up && "
            f"echo '{pw}' | sudo -S ip addr add {PC_IP}/24 dev {iface}",
            "Configuring PC IP address"
        )
        await run_step(f"ping -c 4 {ROBOT_IP}", f"Pinging robot at {ROBOT_IP}")

        await _broadcast("\n[STEP] Connecting DRFL daemon...\n")
        ok = await _drfl_start()
        if not ok:
            raise RuntimeError("DRFL daemon failed to connect to robot")

        _connected = True
        return {"ok": True}
    except RuntimeError as e:
        await _broadcast(f"[ERROR] {e}\n")
        return JSONResponse({"ok": False, "error": str(e)}, status_code=500)

class StartBody(BaseModel):
    plan_name: str

@app.post("/api/robot/start")
async def robot_start(body: StartBody):
    global _active_plan
    if _active_plan is not None:
        raise HTTPException(409, "A plan is already running")
    p = _plan_path(body.plan_name)
    if not p.exists():
        raise HTTPException(404, "Plan not found")
    _active_plan = body.plan_name
    asyncio.get_event_loop().create_task(_run_plan_task(body.plan_name, p.resolve()))
    return {"ok": True}

async def _run_plan_task(plan_name: str, plan_path: Path):
    global _active_plan, _stop_requested

    plan_data = json.loads(plan_path.read_text())
    steps = plan_data.get("steps", [])
    active_steps = [s for s in steps if s.get("enabled", True)]
    has_robot_steps = any(s.get("type") not in ("Turntable", "Laser") for s in active_steps)

    tt_parallel = plan_data.get("turntable_parallel")
    is_parallel = tt_parallel is not None
    tt_step_callback = None

    plan_loops = plan_data.get("loop", False)

    if is_parallel:
        # Parallel: daemon handles looping internally when plan_loops=True
        segments = [("robot", active_steps)]
        use_single_pass = not plan_loops
        should_loop = False  # daemon loops internally when loop=True

        with_tt    = [s.get("with_turntable", False) for s in active_steps]
        with_laser = [s.get("with_laser", False)    for s in active_steps]
        laser_delays = [float(s.get("laser_delay", 0.0)) for s in active_steps]

        async def tt_step_callback(text):
            if "[STEP_START]" not in text:
                return
            try:
                idx = int(text.split("[STEP_START]")[1].strip().split()[0])
            except (ValueError, IndexError):
                return
            if not (_turntable and _turntable.is_open):
                return
            try:
                if idx < len(with_tt) and with_tt[idx]:
                    _turntable.write(b"ENABLE\n")
                    await asyncio.sleep(0.05)
                    _turntable.write(f"DIR:{tt_parallel['direction']}\n".encode())
                    await asyncio.sleep(0.05)
                    _turntable.write(f"SPEED:{int(tt_parallel['speed_us'])}\n".encode())
                else:
                    _turntable.write(b"DISABLE\n")
                if idx < len(with_laser):
                    if with_laser[idx]:
                        delay = laser_delays[idx]
                        if delay > 0:
                            await asyncio.sleep(delay)
                        _turntable.write(b"LAS:ENA\n")
                    else:
                        _turntable.write(b"LAS:DIS\n")
            except Exception as e:
                await _broadcast(f"[WARN] Turntable error: {e}\n")
    else:
        # Sequential: split at Turntable/Laser step boundaries
        segments = []
        robot_buf: list = []
        for step in active_steps:
            stype = step.get("type")
            if stype in ("Turntable", "Laser"):
                if robot_buf:
                    segments.append(("robot", list(robot_buf)))
                    robot_buf = []
                segments.append((stype.lower(), step))
            else:
                robot_buf.append(step)
        if robot_buf:
            segments.append(("robot", robot_buf))

        has_tt_or_laser_segs = any(s[0] in ("turntable", "laser") for s in segments)
        use_single_pass = has_tt_or_laser_segs or not plan_loops
        should_loop = plan_loops

        def _laser(enable: bool):
            if _turntable and _turntable.is_open:
                try:
                    _turntable.write(b"LAS:ENA\n" if enable else b"LAS:DIS\n")
                except Exception:
                    pass

    t_start = time.monotonic()
    last_rc = 0

    while True:
        for seg_type, seg_data in segments:
            if _stop_requested:
                break
            if seg_type == "robot":
                if not is_parallel:
                    # Build per-segment laser callback capturing this seg_data
                    async def _laser_cb(text, _seg=seg_data):
                        if "[STEP_START]" not in text:
                            return
                        try:
                            idx = int(text.split("[STEP_START]")[1].strip().split()[0])
                        except (ValueError, IndexError):
                            return
                        if idx < len(_seg):
                            step = _seg[idx]
                            if step.get("with_laser", False):
                                delay = float(step.get("laser_delay", 0.0))
                                if delay > 0:
                                    await asyncio.sleep(delay)
                                _laser(True)
                            else:
                                _laser(False)
                    cb = _laser_cb
                else:
                    cb = tt_step_callback
                last_rc = await _run_robot_segment(
                    seg_data, plan_data, use_single_pass, on_line=cb
                )
                if not is_parallel:
                    _laser(False)
            elif seg_type == "turntable":
                await _run_turntable_segment(seg_data)
            elif seg_type == "laser":
                await _run_laser_segment(seg_data)
        if _stop_requested or not should_loop:
            break

    # Parallel mode: ensure turntable + laser off after stop
    if is_parallel and _turntable and _turntable.is_open:
        try:
            _turntable.write(b"DISABLE\n")
            _turntable.write(b"LAS:DIS\n")
        except Exception:
            pass

    elapsed_total = time.monotonic() - t_start
    await _broadcast(f"[STAT] Finished in {elapsed_total:.1f}s\n")

    if last_rc < 0:
        result = "unknown"
    elif _stop_requested or not plan_loops:
        result = "success"
    else:
        result = "fail"  # loop plan ended without user stop
    _stop_requested = False

    _record_stat(plan_name, result)
    await _broadcast(f"[DONE] Plan '{plan_name}' finished — {result}\n")
    _active_plan = None


async def _run_robot_segment(seg_data: list, plan_data: dict, single_pass: bool, on_line=None) -> int:
    """Send run_plan to daemon; await [DONE] or [ERROR] before returning."""
    global _drfl_done_event, _drfl_done_result, _drfl_on_line

    done_event = asyncio.Event()
    _drfl_done_event = done_event
    _drfl_done_result = 0
    _drfl_on_line = on_line

    ok = await _drfl_send({
        "cmd": "run_plan",
        "plan": {
            "name": plan_data["name"],
            "created_at": plan_data.get("created_at", ""),
            "steps": seg_data,
        },
        "single_pass": single_pass,
        "loop": not single_pass,
    })

    if not ok:
        _drfl_done_event = None
        _drfl_on_line = None
        return -1

    await done_event.wait()
    _drfl_on_line = None
    _drfl_done_event = None
    return _drfl_done_result


async def _run_turntable_segment(seg_data: dict):
    global _stop_requested
    direction = seg_data.get("direction", "CW")
    speed_us = int(seg_data.get("speed_us", 500))
    duration = float(seg_data.get("duration", 1.0))
    with_laser = seg_data.get("with_laser", False)

    if _turntable is None or not _turntable.is_open:
        await _broadcast("[WARN] Turntable not connected — skipping step\n")
        return

    laser_tag = " + Laser" if with_laser else ""
    await _broadcast(f"Turntable{laser_tag}: {direction} · {speed_us} μs · {duration:.1f}s\n")
    try:
        _turntable.write(b"ENABLE\n")
        await asyncio.sleep(0.05)
        _turntable.write(f"DIR:{direction}\n".encode())
        await asyncio.sleep(0.05)
        _turntable.write(f"SPEED:{speed_us}\n".encode())
        if with_laser:
            await asyncio.sleep(0.05)
            _turntable.write(b"LAS:ENA\n")
        elapsed = 0.0
        while elapsed < duration and not _stop_requested:
            await asyncio.sleep(0.1)
            elapsed += 0.1
        _turntable.write(b"DISABLE\n")
        if with_laser:
            await asyncio.sleep(0.05)
            _turntable.write(b"LAS:DIS\n")
    except Exception as e:
        await _broadcast(f"[WARN] Turntable error: {e}\n")


async def _run_laser_segment(seg_data: dict):
    global _stop_requested
    duration = float(seg_data.get("duration", 1.0))

    if _turntable is None or not _turntable.is_open:
        await _broadcast("[WARN] Turntable/laser not connected — skipping laser step\n")
        return

    await _broadcast(f"Laser: {duration:.1f}s\n")
    try:
        _turntable.write(b"LAS:ENA\n")
        elapsed = 0.0
        while elapsed < duration and not _stop_requested:
            await asyncio.sleep(0.1)
            elapsed += 0.1
        _turntable.write(b"LAS:DIS\n")
    except Exception as e:
        await _broadcast(f"[WARN] Laser error: {e}\n")

@app.post("/api/robot/stop")
async def robot_stop():
    global _active_plan, _stop_requested
    if _active_plan is None:
        raise HTTPException(409, "No plan running")
    _stop_requested = True
    # Send stop to daemon (no-op if currently in a turntable segment)
    if _drfl_proc and _drfl_proc.poll() is None:
        await _drfl_send({"cmd": "stop"})
    return {"ok": True}

@app.post("/api/robot/disconnect")
async def robot_disconnect():
    global _drfl_proc, _connected, _active_plan
    if _drfl_proc and _drfl_proc.poll() is None:
        await _drfl_send({"cmd": "close"})
        loop = asyncio.get_event_loop()
        try:
            await asyncio.wait_for(
                loop.run_in_executor(None, lambda: _drfl_proc.wait(timeout=3)),
                timeout=4.0,
            )
        except (asyncio.TimeoutError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(_drfl_proc.pid), signal.SIGINT)
            except (ProcessLookupError, OSError):
                pass
    _drfl_proc = None
    _captured_points.clear()
    _connected = False
    _active_plan = None
    await _broadcast("[DISCONNECTED]\n")
    return {"ok": True}

@app.get("/api/robot/status")
async def robot_status():
    running = _active_plan is not None
    return {"connected": _connected, "running": running, "active_plan": _active_plan}

@app.post("/api/robot/capture_pose")
async def capture_pose():
    """Return current TCP pose (base frame) as [x,y,z,rx,ry,rz] via daemon."""
    global _pose_event, _pose_result
    if not _connected:
        raise HTTPException(409, "Robot not connected")
    _pose_event = asyncio.Event()
    _pose_result = None
    ok = await _drfl_send({"cmd": "capture_pose"})
    if not ok:
        _pose_event = None
        raise HTTPException(503, "Daemon not available")
    try:
        await asyncio.wait_for(_pose_event.wait(), timeout=5.0)
    except asyncio.TimeoutError:
        _pose_event = None
        raise HTTPException(504, "Pose capture timeout")
    _pose_event = None
    if _pose_result is None or len(_pose_result) != 6:
        raise HTTPException(500, "Invalid pose data from daemon")
    return {"pos": _pose_result}

# ── hand-teach ─────────────────────────────────────────────────────────────

@app.post("/api/robot/hand_guide/enable")
async def hand_guide_enable():
    ok = await _drfl_send({"cmd": "enable_hand_guide"})
    return {"ok": ok}

@app.post("/api/robot/hand_guide/disable")
async def hand_guide_disable():
    ok = await _drfl_send({"cmd": "disable_hand_guide"})
    return {"ok": ok}

@app.post("/api/robot/hand_guide/record")
async def hand_guide_record():
    # Daemon POSTs directly to /api/robot/hand_guide/captured; no stdout echo
    ok = await _drfl_send({"cmd": "record_point"})
    return {"ok": ok}

@app.post("/api/robot/hand_guide/clear")
async def hand_guide_clear():
    global _captured_points
    _captured_points.clear()
    return {"ok": True}

@app.get("/api/robot/hand_guide/points")
async def hand_guide_points():
    return {"points": _captured_points}

@app.delete("/api/robot/hand_guide/points")
async def hand_guide_clear_points():
    global _captured_points
    _captured_points.clear()
    return {"ok": True}

@app.post("/api/robot/hand_guide/captured")
async def hand_guide_captured(request: Request):
    """Called by DRFL daemon after each record_point to push step data."""
    point = await request.json()
    _captured_points.append(point)
    await _broadcast(f"[CAPTURE] {json.dumps(point)}\n")
    return {"ok": True, "count": len(_captured_points)}


class HandGuideTypeBody(BaseModel):
    move_type: str

@app.post("/api/robot/hand_guide/type")
async def hand_guide_type(body: HandGuideTypeBody):
    if body.move_type not in ("MoveJ", "MoveL"):
        raise HTTPException(400, "move_type must be MoveJ or MoveL")
    ok = await _drfl_send({"cmd": "set_param", "key": "current_type", "value": body.move_type})
    return {"ok": ok}

class JogBody(BaseModel):
    axis: int
    reference: int = 0
    velocity: float

@app.post("/api/robot/jog")
async def robot_jog(body: JogBody):
    if not _connected:
        raise HTTPException(409, "Not connected")
    if _active_plan is not None:
        raise HTTPException(409, "A plan is running")
    if not (0 <= body.axis <= 11):
        raise HTTPException(400, "axis must be 0-11")
    ok = await _drfl_send({"cmd": "jog", "axis": body.axis,
                           "reference": body.reference, "velocity": body.velocity})
    return {"ok": ok}

@app.post("/api/robot/jog/enable")
async def robot_jog_enable():
    if not _connected:
        raise HTTPException(409, "Not connected")
    if _active_plan is not None:
        raise HTTPException(409, "A plan is running")
    ok = await _drfl_send({"cmd": "enable_jog"})
    return {"ok": ok}

@app.post("/api/robot/jog/disable")
async def robot_jog_disable():
    if not _connected:
        raise HTTPException(409, "Not connected")
    ok = await _drfl_send({"cmd": "disable_jog"})
    return {"ok": ok}

# ── turntable control ──────────────────────────────────────────────────────

class TurntableConnectBody(BaseModel):
    port: str = "/dev/ttyACM0"
    baud: int = 9600

@app.post("/api/turntable/connect")
async def turntable_connect(body: TurntableConnectBody):
    global _turntable
    global _tt_pending_port
    if serial is None:
        raise HTTPException(500, "pyserial not installed — run: pip install pyserial")
    _close_turntable()
    try:
        _turntable = serial.Serial(body.port, body.baud, timeout=1)
        asyncio.create_task(_tt_sync_state())
        return {"ok": True}
    except Exception as e:
        _turntable = None
        if isinstance(e, PermissionError) or getattr(e, "errno", None) == 13:
            _tt_pending_port = body.port
            raise HTTPException(403, "Permission denied — enter sudo password or add user to dialout group")
        raise HTTPException(500, str(e))

@app.post("/api/turntable/disconnect")
async def turntable_disconnect():
    _close_turntable()
    return {"ok": True}

@app.get("/api/turntable/status")
async def turntable_status():
    connected = _turntable is not None and _turntable.is_open
    return {
        "connected": connected,
        "enabled": _tt_enabled,
        "direction": _tt_direction,
        "speed": _tt_speed,
        "pending_port": _tt_pending_port,
        "rejected_ports": sorted(_tt_rejected_ports),
        "emg_state": _emg_state,
    }

class TurntableConfirmBody(BaseModel):
    port: str
    sudo_password: str = ""

@app.post("/api/turntable/confirm")
async def turntable_confirm(body: TurntableConfirmBody):
    global _turntable, _tt_pending_port
    if serial is None:
        raise HTTPException(500, "pyserial not installed")
    if body.port != _tt_pending_port:
        raise HTTPException(400, "Port is not pending confirmation")
    _close_turntable()
    def _is_permission_err(e: Exception) -> bool:
        return isinstance(e, PermissionError) or getattr(e, "errno", None) == 13

    try:
        _turntable = serial.Serial(body.port, 9600, timeout=1)
        _tt_pending_port = None
        asyncio.create_task(_tt_sync_state())
        return {"ok": True}
    except Exception as e:
        _turntable = None
        if not _is_permission_err(e):
            raise HTTPException(500, str(e))
        if not body.sudo_password:
            raise HTTPException(403, "Permission denied — enter sudo password or add user to dialout group")
        result = subprocess.run(
            ["sudo", "-S", "chmod", "a+rw", body.port],
            input=(body.sudo_password + "\n").encode(),
            capture_output=True,
            timeout=10,
        )
        if result.returncode != 0:
            raise HTTPException(500, "chmod failed — wrong sudo password?")
        try:
            _turntable = serial.Serial(body.port, 9600, timeout=1)
            _tt_pending_port = None
            asyncio.create_task(_tt_sync_state())
            return {"ok": True}
        except Exception as e2:
            _turntable = None
            raise HTTPException(500, str(e2))

class TurntableRejectBody(BaseModel):
    port: str

@app.post("/api/turntable/reject")
async def turntable_reject(body: TurntableRejectBody):
    global _tt_pending_port
    _tt_rejected_ports.add(body.port)
    if _tt_pending_port == body.port:
        _tt_pending_port = None
    return {"ok": True}

def _tt_send(cmd: str):
    if _turntable is None or not _turntable.is_open:
        raise HTTPException(409, "Turntable not connected")
    _turntable.write((cmd + "\n").encode())

async def _tt_sync_state():
    await asyncio.sleep(0.3)
    try:
        _tt_send(f"DIR:{_tt_direction}")
        _tt_send(f"SPEED:{_tt_speed}")
    except Exception:
        pass

@app.post("/api/turntable/enable")
async def turntable_enable():
    global _tt_enabled
    _tt_send(f"DIR:{_tt_direction}")
    _tt_send(f"SPEED:{_tt_speed}")
    _tt_send("ENABLE")
    _tt_enabled = True
    return {"ok": True}

@app.post("/api/turntable/disable")
async def turntable_disable():
    global _tt_enabled
    _tt_send("DISABLE")
    _tt_enabled = False
    return {"ok": True}

class TurntableDirectionBody(BaseModel):
    direction: str

@app.post("/api/turntable/direction")
async def turntable_direction(body: TurntableDirectionBody):
    global _tt_direction
    if body.direction not in ("CW", "CCW"):
        raise HTTPException(400, "direction must be CW or CCW")
    _tt_send(f"DIR:{body.direction}")
    _tt_direction = body.direction
    return {"ok": True}

class TurntableSpeedBody(BaseModel):
    delay_us: int

@app.post("/api/turntable/speed")
async def turntable_speed(body: TurntableSpeedBody):
    global _tt_speed
    if body.delay_us < 3:
        raise HTTPException(400, "delay_us must be >= 3")
    _tt_send(f"SPEED:{body.delay_us}")
    _tt_speed = body.delay_us
    return {"ok": True}

# ── WebSocket ──────────────────────────────────────────────────────────────

@app.websocket("/ws/terminal")
async def ws_terminal(ws: WebSocket):
    global _disconnect_task
    await ws.accept()
    # Purge stale connections before adding new one
    dead = []
    for existing in list(_ws_clients):
        try:
            await existing.send_text("")
        except Exception:
            dead.append(existing)
    for d in dead:
        _ws_clients.remove(d)
    _ws_clients.append(ws)
    # Cancel any pending safety-shutdown watchdog — client is back
    if _disconnect_task and not _disconnect_task.done():
        _disconnect_task.cancel()
        _disconnect_task = None
    async def _ping_task():
        while True:
            await asyncio.sleep(30)
            try:
                await ws.send_text("__ping__")
            except Exception:
                break

    ping = asyncio.create_task(_ping_task())
    try:
        while True:
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        ping.cancel()
        if ws in _ws_clients:
            _ws_clients.remove(ws)
        # Start watchdog only when the last client drops
        if not _ws_clients:
            _disconnect_task = asyncio.get_event_loop().create_task(
                _schedule_safety_shutdown()
            )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
