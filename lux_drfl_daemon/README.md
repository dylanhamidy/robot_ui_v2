# lux_drfl_daemon

Persistent C++ process that owns the DRFL robot connection. Replaces ROS2
`move_joint_node` + `pose_capture_node`. One process, one connection.

No `libpoco-dev` needed — Doosan bundles Poco `.so` inside API-DRFL alongside `libDRFL.a`.

## Prerequisites

```bash
sudo apt-get install -y g++ cmake git libcurl4-openssl-dev
```

Supported: Ubuntu 18.04 / 20.04 / 22.04 / 24.04, amd64 + arm64.

## Build

```bash
# From repo root — init submodule first
git submodule update --init third_party/API-DRFL

# Build (auto-detects arch + Ubuntu version)
cmake -B build -DDRCF_VERSION=3 .
cmake --build build -j$(nproc)

# Explicit Ubuntu version (e.g. cross-compile or version mismatch)
cmake -B build -DDRCF_VERSION=3 -DUBUNTU_VERSION=18.04 .
cmake --build build -j$(nproc)
```

API-DRFL submodule provides:
- `include/DRFLEx.h` — DRFL C++ API headers
- `library/Linux/64bits/{amd64|arm64}/{18.04|20.04|22.04|24.04}/libDRFL.a` — prebuilt static lib
- `libPocoFoundation.so` + `libPocoNet.so` — bundled by Doosan, linked directly

## Run

```bash
# Real robot
./build/drfl_daemon --ip 192.168.0.20 --port 12345

# Doosan virtual emulator
./build/drfl_daemon --ip 127.0.0.1 --port 12345
```

Smoke test (emulator running at 127.0.0.1:12345):

```bash
# Terminal 1
./build/drfl_daemon --ip 127.0.0.1 --port 12345

# Terminal 2 — send command via stdin
echo '{"cmd":"run_plan","plan":{"name":"t","steps":[{"type":"MoveJ","pos":[0,0,30,0,0,0],"vel":50,"acc":50,"time":2.0}]},"single_pass":true}' \
  > /proc/$(pgrep drfl_daemon)/fd/0

# Expect: [STEP_START] 0 ... [DONE] complete
```

## IPC protocol

Commands → daemon **stdin** (newline-delimited JSON)  
Output → daemon **stdout** (broadcast to WebSocket by server.py)

### Commands

| cmd | fields | effect |
|---|---|---|
| `run_plan` | `plan:{name,steps[]}`, `single_pass`, `loop` | execute steps in worker thread |
| `stop` | — | `g_cancel=true` + `MoveStop(QUICK)` |
| `record_point` | — | posj+posx → HTTP POST `/api/robot/hand_guide/captured` |
| `clear_plan` | — | clear points buffer |
| `save_plan` | — | HTTP POST `/api/plans/import` |
| `enable_hand_guide` | — | `ROBOT_MODE_MANUAL` + `task_compliance_ctrl` |
| `disable_hand_guide` | — | `release_compliance_ctrl` + `ROBOT_MODE_AUTONOMOUS` |
| `set_param` | `key`, `value` | update `current_type`/`default_vel`/`default_acc`/`default_time`/`plan_name` |
| `close` | — | graceful shutdown |

### Sentinels

| Sentinel | Meaning |
|---|---|
| `[CONNECTED]` | connection + servo-on succeeded |
| `[DISCONNECTED]` | robot dropped connection |
| `[STEP_START] N` | step N about to execute |
| `[DONE] complete` | plan finished normally |
| `[DONE] cancelled` | stopped by `stop` command |
| `[ERROR] <msg>` | error (daemon may exit) |
| `[INFO] <msg>` | informational |

## DRCF version

`-DDRCF_VERSION=2` or `-DDRCF_VERSION=3` (default 3). Passed as compile definition → `#if DRCF_VERSION == 3` in source.
