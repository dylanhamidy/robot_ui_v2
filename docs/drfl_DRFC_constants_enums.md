# DRFC.h — Constants & Enums Reference

Source: `lux_drfl_daemon/third_party/API-DRFL/include/DRFC.h`

---

## Size Constants

| Constant | Value | Description |
|---|---|---|
| `NUM_JOINT` | 6 | Number of robot joints |
| `NUMBER_OF_JOINT` | 6 | Alias for NUM_JOINT |
| `NUM_TASK` | 6 | Task-space DOF (x,y,z,rx,ry,rz) |
| `NUMBER_OF_TASK` | 6 | Alias for NUM_TASK |
| `MAX_MOVEB_POINT` | 50 | Max segments in `moveb` array |
| `MAX_SPLINE_POINT` | 100 | Max waypoints in spline arrays |
| `MAX_SYMBOL_SIZE` | 32 | Max chars for symbol/name strings |
| `MAX_STRING_SIZE` | 256 | Max chars for message strings |
| `NUM_DIGITAL` | 16 | Control-box digital I/O channels (v2) |
| `NUM_DIGITAL_V3` | 32 | Control-box digital I/O channels (v3) |
| `NUM_ANALOG` | 2 | Analog I/O channels |
| `NUM_FLANGE_IO` | 6 | Flange digital I/O channels |
| `MAX_MODBUS_TOTAL_REGISTERS` | 100 | Max modbus registers |
| `MAX_MOVEB_POINT` | 50 | Max blended motion segments |

---

## Enums

### ROBOT_STATE

Current operational state of the robot. Callback: `TOnMonitoringStateCB`.

```cpp
typedef enum {
    STATE_INITIALIZING   = 0,   // boot / config loading
    STATE_STANDBY        = 1,   // ready for motion
    STATE_MOVING         = 2,   // executing a move
    STATE_SAFE_OFF       = 3,   // drives disabled (safe-off)
    STATE_TEACHING       = 4,   // direct teaching / manual mode
    STATE_SAFE_STOP      = 5,   // protective stop triggered
    STATE_EMERGENCY_STOP = 6,   // E-stop active
    STATE_HOMMING        = 7,   // homing in progress
    STATE_RECOVERY       = 8,   // recovery mode
    STATE_SAFE_STOP2     = 9,
    STATE_SAFE_OFF2      = 10,
    STATE_NOT_READY      = 15,
    STATE_LAST,
} ROBOT_STATE;
```

**Key rule:** daemon's `waitForStandby()` blocks until `STATE_STANDBY`. Motion commands only work in `STATE_STANDBY` or `STATE_MOVING`.

---

### ROBOT_CONTROL

Commands sent via `set_robot_control()` to transition robot state.

```cpp
typedef enum {
    CONTROL_INIT_CONFIG         = 0,
    CONTROL_ENABLE_OPERATION    = 1,
    CONTROL_RESET_SAFET_STOP    = 2,   // alias: CONTROL_RESET_SAFE_STOP
    CONTROL_RESET_SAFET_OFF     = 3,   // alias: CONTROL_SERVO_ON ← primary use
    CONTROL_SERVO_ON            = 3,   // = CONTROL_RESET_SAFET_OFF
    CONTROL_RECOVERY_SAFE_STOP  = 4,
    CONTROL_RECOVERY_SAFE_OFF   = 5,
    CONTROL_RECOVERY_BACKDRIVE  = 6,
    CONTROL_RESET_RECOVERY      = 7,
    CONTROL_LAST
} ROBOT_CONTROL;
```

**Connect sequence uses:** `set_robot_control(CONTROL_SERVO_ON)` → wait `STATE_STANDBY`.

---

### ROBOT_MODE

```cpp
typedef enum {
    ROBOT_MODE_MANUAL,      // 0 — hand guide, teaching, jog
    ROBOT_MODE_AUTONOMOUS,  // 1 — program execution, API motion
    ROBOT_MODE_RECOVERY,
    ROBOT_MODE_BACKDRIVE,
    ROBOT_MODE_MEASURE,
    ROBOT_MODE_INITIALIZE,
    ROBOT_MODE_LAST,
} ROBOT_MODE;
```

**Rule:** set `ROBOT_MODE_AUTONOMOUS` before any motion API call. Set `ROBOT_MODE_MANUAL` for hand-guide / jog. Our daemon uses both.

---

### ROBOT_SYSTEM

```cpp
typedef enum {
    ROBOT_SYSTEM_REAL    = 0,   // physical robot
    ROBOT_SYSTEM_VIRTUAL = 1,   // simulation / DRL emulation
} ROBOT_SYSTEM;
```

---

### MOVE_MODE

```cpp
typedef enum {
    MOVE_MODE_ABSOLUTE = 0,  // target is absolute position
    MOVE_MODE_RELATIVE = 1,  // target is offset from current
} MOVE_MODE;
```

---

### MOVE_REFERENCE

Coordinate frame for motion target.

```cpp
typedef enum {
    MOVE_REFERENCE_BASE      = 0,    // robot base frame
    MOVE_REFERENCE_TOOL      = 1,    // tool (TCP) frame
    MOVE_REFERENCE_WORLD     = 2,    // world frame
    MOVE_REFERENCE_USER_MIN  = 101,  // user-defined frames 101–200
    MOVE_REFERENCE_USER_MAX  = 200,
} MOVE_REFERENCE;
```

---

### BLENDING_SPEED_TYPE

Controls velocity profile at blend point.

```cpp
typedef enum {
    BLENDING_SPEED_TYPE_DUPLICATE = 0,  // maintain current speed through blend
    BLENDING_SPEED_TYPE_OVERRIDE  = 1,  // adopt next segment's speed at blend
} BLENDING_SPEED_TYPE;
```

---

### STOP_TYPE

```cpp
typedef enum {
    STOP_TYPE_QUICK_STO  = 0,  // immediate torque cutoff (STO)
    STOP_TYPE_QUICK      = 1,  // quick deceleration (used in daemon's stop cmd)
    STOP_TYPE_SLOW       = 2,  // smooth deceleration
    STOP_TYPE_HOLD       = 3,  // pause in place
    STOP_TYPE_EMERGENCY  = 3,  // alias for HOLD
} STOP_TYPE;
```

**Our daemon uses:** `MoveStop(STOP_TYPE_QUICK)` on `stop` command.

---

### MANAGE_ACCESS_CONTROL

Sent to `manage_access_control()` to request/respond to control authority.

```cpp
typedef enum {
    MANAGE_ACCESS_CONTROL_FORCE_REQUEST  = 0,  // forcibly take control
    MANAGE_ACCESS_CONTROL_REQUEST        = 1,  // request politely
    MANAGE_ACCESS_CONTROL_RESPONSE_YES   = 2,  // grant to requester
    MANAGE_ACCESS_CONTROL_RESPONSE_NO    = 3,  // deny requester
} MANAGE_ACCESS_CONTROL;
```

**Connect sequence uses:** `MANAGE_ACCESS_CONTROL_FORCE_REQUEST`.

---

### MONITORING_ACCESS_CONTROL

Received in `TOnMonitoringAccessControlCB` callback.

```cpp
typedef enum {
    MONITORING_ACCESS_CONTROL_REQUEST = 0,  // another client wants control
    MONITORING_ACCESS_CONTROL_DENY    = 1,  // our request was denied
    MONITORING_ACCESS_CONTROL_GRANT   = 2,  // we now have control
    MONITORING_ACCESS_CONTROL_LOSS    = 3,  // we lost control
    MONITORING_ACCESS_CONTROL_LAST
} MONITORING_ACCESS_CONTROL;
```

---

### JOG_AXIS

```cpp
typedef enum {
    JOG_AXIS_JOINT_1 = 0,  // joint 1
    JOG_AXIS_JOINT_2,       // joint 2
    JOG_AXIS_JOINT_3,       // joint 3
    JOG_AXIS_JOINT_4,       // joint 4
    JOG_AXIS_JOINT_5,       // joint 5
    JOG_AXIS_JOINT_6,       // joint 6
    JOG_AXIS_TASK_X,        // TCP +X
    JOG_AXIS_TASK_Y,        // TCP +Y
    JOG_AXIS_TASK_Z,        // TCP +Z
    JOG_AXIS_TASK_RX,       // TCP rotation about X
    JOG_AXIS_TASK_RY,       // TCP rotation about Y
    JOG_AXIS_TASK_RZ,       // TCP rotation about Z
} JOG_AXIS;
```

---

### TASK_AXIS

Used by `move_spiral()`.

```cpp
typedef enum {
    TASK_AXIS_X = 0,
    TASK_AXIS_Y = 1,
    TASK_AXIS_Z = 2,
} TASK_AXIS;
```

---

### COORDINATE_SYSTEM

```cpp
typedef enum {
    COORDINATE_SYSTEM_BASE      = 0,
    COORDINATE_SYSTEM_TOOL      = 1,
    COORDINATE_SYSTEM_WORLD     = 2,
    COORDINATE_SYSTEM_USER_MIN  = 101,  // user frames 101–200
    COORDINATE_SYSTEM_USER_MAX  = 200,
} COORDINATE_SYSTEM;
```

---

### SAFE_STOP_RESET_TYPE

```cpp
typedef enum {
    SAFE_STOP_RESET_TYPE_DEFAULT         = 0,
    SAFE_STOP_RESET_TYPE_PROGRAM_STOP    = 0,  // stop program on reset
    SAFE_STOP_RESET_TYPE_PROGRAM_RESUME  = 1,  // resume program after reset
} SAFE_STOP_RESET_TYPE;
```

---

### SPLINE_VELOCITY_OPTION

Used by `movesx()` / `movesj()`.

```cpp
typedef enum {
    SPLINE_VELOCITY_OPTION_DEFAULT = 0,  // planner chooses
    SPLINE_VELOCITY_OPTION_CONST   = 1,  // constant speed through spline
} SPLINE_VELOCITY_OPTION;
```

---

### DR_MV_APP (Application Type)

Passed as `eAppType` to `movel()`, `moveb()`, etc.

```cpp
typedef enum {
    DR_MV_APP_NONE = 0,  // no application
    DR_MV_APP_WELD = 1,  // welding application (triggers weld I/O)
} DR_MV_APP;
```

---

### MOVE_HOME

```cpp
typedef enum {
    MOVE_HOME_MECHANIC = 0,  // go to mechanical zero
    MOVE_HOME_USER     = 1,  // go to user-defined home
} MOVE_HOME;
```

---

### MOVE_ORIENTATION (movec)

Controls tool orientation behavior along circular arc.

```cpp
typedef enum {
    DR_MV_ORI_TEACH  = 0,  // interpolate orientation as taught
    DR_MV_ORI_FIXED  = 1,  // keep orientation fixed
    DR_MV_ORI_RADIAL = 2,  // radial to arc center
    DR_MV_ORI_INTENT = 3,  // intent-based
} MOVE_ORIENTATION;
```

---

### SINGULARITY_AVOIDANCE

```cpp
typedef enum {
    SINGULARITY_AVOIDANCE_AVOID = 0,  // path detours around singularity
    SINGULARITY_AVOIDANCE_STOP  = 1,  // stop when approaching
    SINGULARITY_AVOIDANCE_VEL   = 2,  // velocity-based handling
} SINGULARITY_AVOIDANCE;
```

---

### SPEED_MODE

```cpp
typedef enum {
    SPEED_NORMAL_MODE  = 0,
    SPEED_REDUCED_MODE = 1,
} MONITORING_SPEED;
typedef MONITORING_SPEED SPEED_MODE;
```

---

### GPIO_CTRLBOX_DIGITAL_INDEX

Indices 0–15 (v2), 0–31 (v3). Used with `set_digital_output()`, `get_digital_input()`.

- `GPIO_CTRLBOX_DIGITAL_INDEX_1 = 0` through `GPIO_CTRLBOX_DIGITAL_INDEX_16 = 15`
- v3 extends to index 32

### GPIO_CTRLBOX_ANALOG_INDEX

- `GPIO_CTRLBOX_ANALOG_INDEX_1 = 0`
- `GPIO_CTRLBOX_ANALOG_INDEX_2 = 1`

### GPIO_TOOL_DIGITAL_INDEX

Flange digital I/O (6 channels on M/H-series):
- `GPIO_TOOL_DIGITAL_INDEX_1 = 0` through `GPIO_TOOL_DIGITAL_INDEX_6`

### GPIO_ANALOG_TYPE

```cpp
// GPIO_ANALOG_TYPE_VOLTAGE / GPIO_ANALOG_TYPE_CURRENT
```

---

### DR_SERVOJ_TYPE

```cpp
typedef enum {
    DR_SERVO_OVERRIDE = 0,  // immediately override current target
    DR_SERVO_QUEUE    = 1,  // queue after current motion
} DR_SERVOJ_TYPE;
```

---

## MOVEB Blend Type (in MOVE_POSB struct)

Field `_iBlendType` of `MOVE_POSB` struct (defined in DRFS.h):

| Value | Meaning |
|---|---|
| 0 | Line blend (MoveL segment into `_fTargetPos[0]`) |
| 1 | Circle blend (MoveC arc via `_fTargetPos[0]`→`_fTargetPos[1]`) |

This is what our FreeForm/moveb implementation uses. Not a named enum — raw `unsigned char`.
