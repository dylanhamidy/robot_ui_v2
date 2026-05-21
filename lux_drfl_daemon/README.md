# lux_drfl_daemon

Persistent C++ process that owns the DRFL robot connection. Replaces the ROS2
`move_joint_node` + `pose_capture_node` pair. One process, one connection.

## Prerequisites

```bash
# Ubuntu 22.04
sudo apt-get install -y g++ cmake git libpoco-dev libcurl4-openssl-dev

# nlohmann/json is auto-fetched by CMake if not vendored
```

## Build

```bash
# Default: DRCF v3, auto-detect arch
cmake -B build .
cmake --build build -j$(nproc)

# Explicit DRCF version
cmake -B build -DDRCF_VERSION=2 .
cmake --build build -j$(nproc)
```

### API-DRFL submodule

The Doosan DRFL library is a git submodule:

```bash
git submodule update --init third_party/API-DRFL
```

This provides `include/DRFLEx.h` and prebuilt `libDRFL.a` for x86_64 and aarch64.

## Run

```bash
./build/drfl_daemon --ip 192.168.0.20 --port 12345 --drcf 3
```

Smoke test against the Doosan virtual robot emulator (run emulator at 127.0.0.1:12345 first):

```bash
./build/drfl_daemon --ip 127.0.0.1 --port 12345

# In another terminal — paste on stdin:
echo '{"cmd":"run_plan","plan":{"name":"t","steps":[{"type":"MoveJ","pos":[0,0,30,0,0,0],"vel":50,"acc":50,"time":2.0}]},"single_pass":true}' | nc localhost 12345
```

Or interactively:

```bash
./build/drfl_daemon --ip 127.0.0.1 &
echo '{"cmd":"run_plan","plan":{"name":"t","steps":[{"type":"MoveJ","pos":[0,0,30,0,0,0],"vel":50,"acc":50,"time":2.0}]},"single_pass":true}'
# Expect: [STEP_START] 0  ... [DONE] complete
```

## IPC protocol

All commands are newline-delimited JSON written to **stdin**.
All output (sentinels + info) goes to **stdout** (stderr merged to stdout in production).

### Commands (stdin)

| Command | Fields | Effect |
|---|---|---|
| `run_plan` | `plan:{name,steps[]}`, `single_pass:bool`, `loop:bool` | Execute plan steps in worker thread |
| `stop` | — | Set cancel flag, call MoveStop(QUICK) |
| `record_point` | — | Capture current posj+posx, POST to `/api/robot/hand_guide/captured` |
| `clear_plan` | — | Clear captured points buffer |
| `save_plan` | — | POST accumulated steps to `/api/plans/import` |
| `enable_hand_guide` | — | set_robot_mode(MANUAL) + task_compliance_ctrl |
| `disable_hand_guide` | — | release_compliance_ctrl + set_robot_mode(AUTONOMOUS) |
| `set_param` | `key`, `value` | Update `current_type`, `default_vel`, `default_acc`, `default_time`, `plan_name` |
| `close` | — | Graceful shutdown |

### Sentinels (stdout)

| Sentinel | Meaning |
|---|---|
| `[CONNECTED]` | Connection + servo-on succeeded |
| `[DISCONNECTED]` | Robot dropped connection |
| `[STEP_START] N` | Step N about to execute |
| `[DONE] complete` | Plan finished normally |
| `[DONE] cancelled` | Plan stopped by `stop` command |
| `[ERROR] <msg>` | Error — daemon may exit |
| `[INFO] <msg>` | Informational |

## DRCF version

Pass `-DDRCF_VERSION=2` or `-DDRCF_VERSION=3` to cmake. Macro is compiled in via
`target_compile_definitions`. Source file can `#if DRCF_VERSION == 3` for version-specific paths.
Default is 3.
