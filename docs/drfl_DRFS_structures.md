# DRFS.h — Struct Reference

Source: `lux_drfl_daemon/third_party/API-DRFL/include/DRFS.h`
(#includes DRFC.h → all enums available)

---

## Critical Motion Structs

### MOVE_POSB — Blend Motion Segment

**Most important struct for FreeForm/moveb.** Each element in the `moveb()` array.

```cpp
typedef struct _MOVE_POSB {
    float         _fTargetPos[2][NUM_TASK];  // [0]=via or line-end, [1]=arc-end
    unsigned char _iBlendType;               // 0=line, 1=circle
    float         _fBlendRad;               // blending radius (mm); 0 on last segment
} MOVE_POSB, *LPMOVE_POSB;
```

**Usage rules:**
- `_iBlendType=0` (line): `_fTargetPos[0]` = endpoint of linear segment. `_fTargetPos[1]` unused.
- `_iBlendType=1` (circle): `_fTargetPos[0]` = via point, `_fTargetPos[1]` = arc endpoint. Start implicit (previous segment's end).
- Last segment: always set `_fBlendRad=0`.
- Max 50 segments (`MAX_MOVEB_POINT`).

**How daemon maps FreeForm sub-steps:**
- `MoveL` sub-step → `_iBlendType=0`, endpoint → `_fTargetPos[0]`
- `MoveC` sub-step → `_iBlendType=1`, `pos_via` → `_fTargetPos[0]`, `pos_end` → `_fTargetPos[1]`

---

### ROBOT_POSE

Generic 6-float position array (joint angles OR task position).

```cpp
typedef struct _ROBOT_POSE {
    float _fPosition[NUM_JOINT];   // [j1,j2,j3,j4,j5,j6] degrees (joint)
                                    // or [x,y,z,rx,ry,rz] mm/deg (task)
} ROBOT_POSE, *LPROBOT_POSE;
```

Returned by: `get_current_posj()`, `get_current_pose()`, `ikin()`, `fkin()`, `trans()`.

---

### ROBOT_TASK_POSE

Task-space pose with solution space.

```cpp
typedef struct _ROBOT_TASK_POSE {
    float         _fTargetPos[NUM_TASK];  // [x,y,z,rx,ry,rz] mm/deg
    unsigned char _iTargetSol;            // solution space 0–7
} ROBOT_TASK_POSE, *LPROBOT_TASK_POSE;
```

Returned by: `get_current_posx()`. Our daemon uses `_fTargetPos` array directly.

---

### ROBOT_VEL / ROBOT_FORCE

```cpp
typedef struct _ROBOT_VEL   { float _fVelocity[NUM_JOINT]; } ROBOT_VEL, *LPROBOT_VEL;
typedef struct _ROBOT_FORCE { float _fForce[NUM_JOINT]; }    ROBOT_FORCE, *LPROBOT_FORCE;
```

---

## Monitoring Data Structs

### ROBOT_MONITORING_JOINT

Per-joint monitoring data (part of `MONITORING_DATA`).

```cpp
typedef struct _ROBOT_MONITORING_JOINT {
    float _fActualPos[NUM_JOINT];   // actual joint positions (INC encoder) [deg]
    float _fActualAbs[NUM_JOINT];   // actual joint positions (ABS encoder) [deg]
    float _fActualVel[NUM_JOINT];   // actual joint velocities [deg/s]
    float _fActualErr[NUM_JOINT];   // joint error
    float _fTargetPos[NUM_JOINT];   // target joint position
    float _fTargetVel[NUM_JOINT];   // target joint velocity
} ROBOT_MONITORING_JOINT, *LPROBOT_MONITORING_JOINT;
```

---

### ROBOT_MONITORING_TASK

Per-task monitoring data.

```cpp
typedef struct _ROBOT_MONITORING_TASK {
    float         _fActualPos[2][NUMBER_OF_JOINT];  // [0]=tool TCP, [1]=flange
    float         _fActualVel[NUMBER_OF_JOINT];
    float         _fActualErr[NUMBER_OF_JOINT];
    float         _fTargetPos[NUMBER_OF_JOINT];
    float         _fTargetVel[NUMBER_OF_JOINT];
    unsigned char _iSolutionSpace;
    float         _fRotationMatrix[3][3];
} ROBOT_MONITORING_TASK, *LPROBOT_MONITORING_TASK;
```

---

### MONITORING_DATA / MONITORING_DATA_EX

Top-level monitoring packet delivered via `TOnMonitoringDataCB` / `TOnMonitoringDataExCB`.

```cpp
typedef struct _MONITORING_DATA {
    MONITORING_CONTROL _tCtrl;   // joint + task + torque data
    MONITORING_MISC    _tMisc;   // IO, brake, button, motor state
} MONITORING_DATA, *LPMONITORING_DATA;

typedef struct _MONITORING_DATA_EX {
    MONITORING_CONTROL_EX _tCtrl;       // + world + user coord data
    MONITORING_MISC       _tMisc;
    // _tMiscEx: force control mode, auto-acc, temperature, isMoving
    // _tModel: singularity, FTS sensor
    // _tFlangeIo: flange serial config
} MONITORING_DATA_EX, *LPMONITORING_DATA_EX;
```

**Use `set_on_monitoring_data_ex` + `setup_monitoring_version(1)` for full data.**

---

### MONITORING_MISC

Miscellaneous state in monitoring packet.

```cpp
typedef struct _MONITORING_MISC {
    double        _dSyncTime;              // inner clock counter
    unsigned char _iActualDI[NUM_FLANGE_IO]; // flange digital input
    unsigned char _iActualDO[NUM_FLANGE_IO]; // flange digital output
    unsigned char _iActualBK[NUM_JOINT];    // brake state per joint
    unsigned int  _iActualBT[NUM_BUTTON];   // robot button state
    float         _fActualMC[NUM_JOINT];    // motor input current
    float         _fActualMT[NUM_JOINT];    // motor temperature
} MONITORING_MISC, *LPMONITORING_MISC;
```

---

### RT_OUTPUT_DATA_LIST

Real-time control output data (1ms cycle). Returned by `read_data_rt()`.

Key fields (selected):
```cpp
typedef struct _RT_OUTPUT_DATA_LIST {
    double  time_stamp;                          // data acquisition timestamp
    float   actual_joint_position[NUMBER_OF_JOINT];  // [deg] INC encoder
    float   actual_joint_position_abs[NUMBER_OF_JOINT]; // [deg] ABS encoder
    float   actual_joint_velocity[NUMBER_OF_JOINT];  // [deg/s]
    float   actual_tcp_position[NUM_TASK];        // [mm, deg] base frame
    float   actual_tcp_velocity[NUMBER_OF_TASK];  // [mm/s, deg/s]
    float   actual_flange_position[NUMBER_OF_TASK];
    float   actual_motor_torque[NUMBER_OF_JOINT]; // [Nm]
    float   actual_joint_torque[NUMBER_OF_JOINT]; // [Nm] estimated
    float   raw_joint_torque[NUMBER_OF_JOINT];    // [Nm] calibrated JTS
    float   external_joint_torque[NUMBER_OF_JOINT]; // [Nm] estimated external
    float   external_tcp_force[NUMBER_OF_TASK];   // [N, Nm] base frame
    float   target_joint_position[NUMBER_OF_JOINT]; // [deg]
    float   target_joint_velocity[NUMBER_OF_JOINT]; // [deg/s]
    float   target_tcp_position[NUMBER_OF_TASK];  // [mm, deg]
    float   jacobian_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    float   gravity_torque[NUMBER_OF_JOINT];
    float   mass_matrix[NUMBER_OF_JOINT][NUMBER_OF_JOINT];
    unsigned short solution_space;
    float   singularity;
    float   operation_speed_rate;                 // 1–100 %
    float   joint_temperature[NUMBER_OF_JOINT];   // celsius
    unsigned short controller_digital_input;      // 16-bit packed
    unsigned short controller_digital_output;
    float   controller_analog_input[2];
    float   controller_analog_output[2];
    unsigned char  flange_digital_input;
    unsigned char  flange_digital_output;
    float   flange_analog_input[4];
    unsigned char  robot_mode;    // ROBOT_MODE enum value
    unsigned char  robot_state;   // ROBOT_STATE enum value
    unsigned short control_mode;  // CONTROL_MODE enum value
    // ... reserved[256]
} RT_OUTPUT_DATA_LIST, *LPRT_OUTPUT_DATA_LIST;
```

---

### SYSTEM_VERSION

```cpp
typedef struct _SYSTEM_VERSION {
    char _szSmartTp[MAX_SYMBOL_SIZE];       // SmartTP version
    char _szController[MAX_SYMBOL_SIZE];    // controller (DRCF) version
    char _szInterpreter[MAX_SYMBOL_SIZE];   // interpreter version
    char _szInverter[MAX_SYMBOL_SIZE];
    char _szSafetyBoard[MAX_SYMBOL_SIZE];
    char _szRobotSerial[MAX_SYMBOL_SIZE];
    char _szRobotModel[MAX_SYMBOL_SIZE];
    char _szJTSBoard[MAX_SYMBOL_SIZE];
    char _szFlangeBoard[MAX_SYMBOL_SIZE];
} SYSTEM_VERSION, *LPSYSTEM_VERSION;
```

---

### LOG_ALARM

```cpp
typedef struct _LOG_ALARM {
    unsigned char _iLevel;           // alarm level
    unsigned char _iGroup;           // alarm group
    unsigned int  _iIndex;           // alarm code
    char          _szParam[3][MAX_STRING_SIZE];  // alarm parameters
} LOG_ALARM, *LPLOG_ALARM;
```

---

### INVERSE_KINEMATIC_RESPONSE

```cpp
typedef struct _INVERSE_KINEMATIC_RESPONSE {
    float _fTargetPos[NUMBER_OF_JOINT];  // joint angles [deg]
    int   _iStatus;                      // 0=success
} INVERSE_KINEMATIC_RESPONSE, *LPINVERSE_KINEMATIC_RESPONSE;
```

---

## Safety Structs (overview)

Not used directly in our daemon but present in header:

- `CONFIG_VIRTUAL_FENCE` — cube/polygon/cylinder workspace fence
- `CONFIG_PROTECTED_ZONE` — up to 10 safety zone objects
- `SAFETY_OBJECT` — sphere/capsule/cube/OBB/polyprism geometry
- `CONFIG_ADD_SAFETY_ZONE` — zone with property (space-limit or local-zone)
- `SAFETY_ZONE_PROPERTY_LOCAL_ZONE` — per-zone overrides for speed/force/collision limits

---

## Monitoring I/O Structs

### MONITORING_CTRLIO_EX

Control-box I/O snapshot (v2):
```cpp
typedef struct _MONITORING_CTRLIO_EX {
    READ_CTRLIO_INPUT_EX  _tInput;   // DI[16], AI[2], switches, safety
    READ_CTRLIO_OUTPUT_EX _tOutput;  // DO[16], AO[2]
    READ_ENCODER_INPUT    _tEncoder; // encoder strobe + count
    unsigned char         _szReserved[24];
} MONITORING_CTRLIO_EX, *LPMONITORING_CTRLIO_EX;
```

---

## Modbus Structs

### WRITE_MODBUS_DATA / MODBUS_DATA

Used with `add_modbus_signal()`. Supports TCP and RTU types.

### MODBUS_DATA_LIST

```cpp
typedef struct _MODBUS_DATA_LIST {
    unsigned short _nCount;
    MODBUS_DATA    _tRegister[MAX_MODBUS_TOTAL_REGISTERS];  // up to 100
} MODBUS_DATA_LIST, *LPMODBUS_DATA_LIST;
```

---

## Weaving/Welding Structs (app_weld_*)

Not used in our daemon but present for welding applications:

- `CONFIG_TRAPEZOID_WEAVING_SETTING` — trapezoid weave pattern
- `CONFIG_ANALOG_WELDING_INTERFACE` — analog welder I/F config
- `CONFIG_DIGITAL_WELDING_CONDITION` — digital welder conditions
- `CONFIG_DIGITAL_WELDING_INTERFACE_*` — EtherNet/IP R2M / M2R mappings

These are passed to `app_weld_*` family of functions.
