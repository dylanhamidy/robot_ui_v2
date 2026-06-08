# DRFL Examples Reference

Source: `API-DRFL/example/Linux_64/` (GitHub repo: DoosanRobotics/API-DRFL)

---

## Example Files

| File | Purpose |
|---|---|
| `1_minimal_motion_sample.cpp` | Minimal connect + movej — start here |
| `2_motion_sample.cpp` | More motion types |
| `3_io_sample.cpp` | GPIO / I/O operations |
| `4_minimal_welding_sample.cpp` | Digital welding app integration |
| `5_analog_welding_sample.cpp` | Analog welding |
| `6_realtime_control_sample.cpp` | RT UDP servo loop |
| `7_force_control_sample.cpp` | Force/compliance control |

---

## Example 1: Minimal Motion

Full working connect + movej sequence.

```cpp
#define DRCF_VERSION 2  // or 3 — must match your controller

#include "DRFLEx.h"
using namespace DRAFramework;

CDRFLEx robot;
bool get_control_access = false;
bool is_standby = false;

int main() {
    // 1. Register callbacks BEFORE open_connection
    robot.set_on_monitoring_access_control([](const MONITORING_ACCESS_CONTROL access) {
        if (MONITORING_ACCESS_CONTROL_GRANT == access) get_control_access = true;
        if (MONITORING_ACCESS_CONTROL_LOSS  == access) get_control_access = false;
    });
    robot.set_on_monitoring_state([](const ROBOT_STATE state) {
        is_standby = (STATE_STANDBY == state);
    });

    // 2. Connect
    if (!robot.open_connection("127.0.0.1")) return 1;

    // 3. Setup monitoring version (1 = latest callbacks)
    robot.setup_monitoring_version(1);

    // 4. Get access + servo on (retry loop)
    for (size_t retry = 0; retry < 10; ++retry) {
        if (!get_control_access) {
            robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        if (!is_standby) {
            robot.set_robot_control(CONTROL_SERVO_ON);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }
        break;  // have control + standby
    }

    // 5. Set autonomous mode
    robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);

    // 6. Move
    float pos[6] = {0., 0., 30., 0., 0., 0.};  // joint degrees
    robot.movej(pos, 50, 50);  // vel=50%, acc=50%

    pos[2] = 0;
    robot.movej(pos, 50, 50);

    robot.close_connection();
    return 0;
}
```

**Key takeaways:**
- Callbacks registered before connect
- Retry loop for access + standby (our daemon does same but with `cond_var` blocking)
- `set_robot_mode(ROBOT_MODE_AUTONOMOUS)` required before motion
- `movej(pos, vel, acc)` — vel/acc are percentage of max

---

## Example 4: Digital Welding (condensed)

Shows connect + welding API + `amovel` with `DR_MV_APP_WELD`.

```cpp
// Connect + servo on (same pattern as example 1)

robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
robot.set_robot_system(ROBOT_SYSTEM_VIRTUAL);  // simulation mode

// Configure EtherNet/IP interface (r2m = robot to machine, m2r = machine to robot)
robot.app_weld_set_interface_eip_r2m_process(r2m_process_data);
// ... (many config calls) ...
robot.app_weld_enable_digital(1);

// Set weld conditions
CONFIG_DIGITAL_WELDING_CONDITION weld_cond = {...};
weld_cond._fTargetVel = 3.0f;  // weld travel speed mm/s
robot.app_weld_set_weld_cond_digital(weld_cond);

// Move to weld start then weld with app_type=DR_MV_APP_WELD
float vel[2] = {3, 3}, acc[2] = {70, 70};
robot.amovel(target_pos, vel, acc, 0,
             MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE,
             BLENDING_SPEED_TYPE_DUPLICATE, DR_MV_APP_WELD);
robot.mwait();  // block until weld move done

robot.app_weld_disable_digital(1);
```

**Key takeaways:**
- Welding uses `amovel` + `mwait()` pattern (async move, then wait)
- `DR_MV_APP_WELD` as last arg triggers weld I/O during motion
- Full config requires many `app_weld_set_interface_*` calls

---

## Example 6: Real-Time Control (summary)

Trajectory planning + 1ms servo loop via RT UDP.

**Setup pattern:**
```cpp
// After regular connect + servo on:
robot.connect_rt_control("192.168.137.100", 12347);  // separate UDP port

robot.set_rt_control_output("v1.0", 0.001f, 4);  // version, period=1ms, max loss=4
robot.set_rt_control_input("v1.0", 0.001f, 4);
robot.start_rt_control();

// RT callback (fires at 1ms):
robot.set_on_rt_monitoring_data([](LPRT_OUTPUT_DATA_LIST data) {
    // data->actual_joint_position[6], data->actual_tcp_position[6], etc.
});

// In RT loop (separate real-time thread):
LPRT_OUTPUT_DATA_LIST data = robot.read_data_rt();
// Compute target from trajectory planner
robot.servoj_rt(target_pos, target_vel, target_acc, 0.001f);  // 1ms
```

**Key takeaways:**
- RT control is UDP-based, separate connection from main TCP
- `servoj_rt` / `servol_rt` must be called every period (1ms typical)
- `read_data_rt()` returns full state including jacobian, mass matrix, external forces
- Quintic polynomial trajectory common for smooth servo commands
- **If implementing RT control: ask user to provide full `6_realtime_control_sample.cpp` content** — too complex to reconstruct from memory

---

## Our Daemon Pattern (for reference)

The daemon (`lux_drfl_daemon/src/drfl_daemon.cpp`) uses a simplified version of Example 1's connect pattern:

```
open_connection → ManageAccessControl(FORCE_REQUEST) → wait ACCESS_GRANT (10s)
→ set_robot_control(CONTROL_SERVO_ON) → wait STATE_STANDBY (15s)
→ emit [CONNECTED] to stdout
```

Motion uses synchronous calls — each step blocks via `waitForStandby()` (condition variable on STATE_STANDBY callback), not wall-clock sleep. FreeForm uses `moveb()` with MOVE_POSB array.

---

## Common Patterns

### Check standby before motion
```cpp
while (robot.get_robot_state() != STATE_STANDBY) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
```

### Emergency stop
```cpp
robot.stop(STOP_TYPE_QUICK);  // quick deceleration
// or: robot.stop(STOP_TYPE_QUICK_STO);  // immediate torque cut (STO)
```

### Hand guide (teach mode)
```cpp
robot.set_robot_mode(ROBOT_MODE_MANUAL);
float stiffness[6] = {500,500,500,100,100,100};  // N/m and Nm/rad
robot.task_compliance_ctrl(stiffness, COORDINATE_SYSTEM_TOOL);
// ... hand guide active ...
robot.release_compliance_ctrl();
robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
```

### Jog single axis
```cpp
robot.set_robot_mode(ROBOT_MODE_MANUAL);
robot.jog(JOG_AXIS_TASK_Z, MOVE_REFERENCE_BASE, 10.0f);  // 10% speed
// stop: call jog with velocity=0 or call stop()
```

### Capture TCP pose
```cpp
LPROBOT_TASK_POSE posx = robot.get_current_posx(COORDINATE_SYSTEM_BASE);
float pos[6];
memcpy(pos, posx->_fTargetPos, sizeof(float)*6);
// pos = [x, y, z, rx, ry, rz]
```

---

## Uncertainty Notes

If you need details on:
- Force control (`7_force_control_sample.cpp`) — ask user to paste content
- Analog welding (`5_analog_welding_sample.cpp`) — ask user to paste content
- RT control details beyond the pattern above — ask user to paste `6_realtime_control_sample.cpp`
- Full DRL scripting syntax — consult Doosan online manual
- Specific error codes — check `_get_last_alarm()` and consult manual
