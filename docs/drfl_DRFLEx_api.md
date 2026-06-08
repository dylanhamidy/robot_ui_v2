# DRFLEx.h — API Reference

Source: `lux_drfl_daemon/third_party/API-DRFL/include/DRFLEx.h`  
(includes DRFS.h → DRFC.h)

---

## CDRFLEx C++ Class

All robot control is done via `CDRFLEx` class instance (namespace `DRAFramework`).  
C-style `_function()` equivalents exist but `CDRFLEx` methods are preferred.

```cpp
#include "DRFLEx.h"
using namespace DRAFramework;
CDRFLEx robot;
```

Two internal handles:
- `_rbtCtrl` — main TCP connection (motion, config, callbacks)
- `_rbtCtrlUDP` — RT UDP connection (real-time servo loop)

---

## Connection

```cpp
bool open_connection(string strIpAddr = "192.168.137.100", unsigned int usPort = 12345);
bool close_connection();

bool connect_rt_control(string strIpAddr = "192.168.137.100", unsigned int usPort = 12347);
bool disconnect_rt_control();
```

**Our daemon uses:** `open_connection(ip, 12345)`. RT control via separate UDP port 12347.

---

## Setup

```cpp
bool setup_monitoring_version(int iVersion);
// 0 = older callbacks (set_on_monitoring_data, set_on_monitoring_ctrl_io)
// 1 = newer callbacks (set_on_monitoring_data_ex, set_on_monitoring_ctrl_io_ex)
// Always call with 1 in new code.
```

---

## Robot State & Control

```cpp
ROBOT_STATE  get_robot_state();
bool         set_robot_control(ROBOT_CONTROL eControl);
// Key: set_robot_control(CONTROL_SERVO_ON) → starts servo, waits for STATE_STANDBY

ROBOT_MODE   get_robot_mode();
bool         set_robot_mode(ROBOT_MODE eMode);
// Modes: ROBOT_MODE_MANUAL (teach/jog), ROBOT_MODE_AUTONOMOUS (motion API)

ROBOT_SYSTEM get_robot_system();
bool         set_robot_system(ROBOT_SYSTEM eRobotSystem);
// Real robot: ROBOT_SYSTEM_REAL. Simulation: ROBOT_SYSTEM_VIRTUAL.

SPEED_MODE   get_robot_speed_mode();
bool         set_robot_speed_mode(SPEED_MODE eSpeedMode);
// SPEED_NORMAL_MODE or SPEED_REDUCED_MODE

DRL_PROGRAM_STATE get_program_state();

bool set_safe_stop_reset_type(SAFE_STOP_RESET_TYPE eResetType = SAFE_STOP_RESET_TYPE_DEFAULT);

LPLOG_ALARM  get_last_alarm();
bool         get_system_version(LPSYSTEM_VERSION pVersion);
const char*  get_library_version();
```

---

## Access Control

Multiple clients can connect but only one holds control authority.

```cpp
bool manage_access_control(MANAGE_ACCESS_CONTROL eAccessControl = MANAGE_ACCESS_CONTROL_REQUEST);
// MANAGE_ACCESS_CONTROL_FORCE_REQUEST — forcibly claim control (used at startup)
// MANAGE_ACCESS_CONTROL_RESPONSE_YES  — grant control to requester
// MANAGE_ACCESS_CONTROL_RESPONSE_NO   — deny requester
```

Callback fires on state change:
```cpp
void set_on_monitoring_access_control(TOnMonitoringAccessControlCB cb);
// typedef void (*TOnMonitoringAccessControlCB)(const MONITORING_ACCESS_CONTROL);
// Receives: GRANT (we have it), LOSS (we lost it), REQUEST (someone asking), DENY
```

---

## Position Queries

```cpp
LPROBOT_POSE      get_current_posj();   // joint angles [deg] × 6
LPROBOT_TASK_POSE get_current_posx(COORDINATE_SYSTEM eCoodType = COORDINATE_SYSTEM_BASE);
// Returns ROBOT_TASK_POSE with _fTargetPos[6] and _iTargetSol

LPROBOT_POSE get_current_pose(ROBOT_SPACE eSpaceType = ROBOT_SPACE_JOINT);
LPROBOT_VEL  get_current_velj();
LPROBOT_VEL  get_current_velx();
LPROBOT_VEL  get_desired_velx();
LPROBOT_POSE get_desired_posj();
LPROBOT_POSE get_current_tool_flange_posx();  // flange (not TCP) position
unsigned char get_current_solution_space();   // 0–7

LPROBOT_FORCE get_joint_torque();     // joint sensor torque [Nm]
LPROBOT_FORCE get_external_torque();  // estimated external torque [Nm]
LPROBOT_FORCE get_tool_force(COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
```

---

## Kinematics Utilities

```cpp
LPROBOT_POSE fkin(float fSourcePos[NUM_JOINT], COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
// Forward kinematics: joint angles → TCP position

LPROBOT_POSE ikin(float fSourcePos[NUM_TASK], unsigned char iSolutionSpace,
                  COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
// Inverse kinematics: TCP position → joint angles

LPROBOT_POSE trans(float fSourcePos[NUM_TASK], float fOffset[NUM_TASK],
                   COORDINATE_SYSTEM eSourceRef = COORDINATE_SYSTEM_BASE,
                   COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
// Transform position by offset in given frame

unsigned char get_solution_space(float fTargetPos[NUM_JOINT]);
```

---

## Motion Control

### Trajectory Primitives

`movej`, `movel`, `movec`, `moveb`, `movejx`, `movesj`, `movesx`, `move_spiral`, `move_periodic` — full signatures, parameter notes, async variants, and selection guide in **[`drfl_motion_primitives.md`](drfl_motion_primitives.md)**.

**Daemon usage:** `movej` → MoveJ, `movel` → MoveL/WeldStraight approach, `movec` → MoveC arc, `moveb` → FreeForm.

---

### Motion Control

```cpp
bool stop(STOP_TYPE eStopType = STOP_TYPE_QUICK);
// STOP_TYPE_QUICK=1 (quick decel), STOP_TYPE_SLOW=2, STOP_TYPE_HOLD=3

bool move_pause();
bool move_resume();
bool mwait();  // block until current async motion completes (used after amovel in welding)

bool jog(JOG_AXIS eJogAxis, MOVE_REFERENCE eMoveReference, float fVelocity);
// Hold-to-run jog. Axis: JOG_AXIS_JOINT_1..6 or JOG_AXIS_TASK_X..RZ

bool multi_jog(float fTargetPos[NUM_TASK], MOVE_REFERENCE eMoveReference, float fVelocity);

bool move_home(MOVE_HOME eMode = MOVE_HOME_MECHANIC, unsigned char bRun = 1);
```

---

## Environment-Adaptive Motion

```cpp
bool servoj(float fTargetPos[NUM_JOINT], float fLimitVel[NUM_JOINT],
            float fLimitAcc[NUM_JOINT], float fTargetTime,
            DR_SERVOJ_TYPE eTargetMod = DR_SERVO_OVERRIDE);
// Continuously track target joint pos (non-blocking, real-time update loop)

bool servol(float fTargetPos[NUM_TASK], float fLimitVel[2], float fLimitAcc[2], float fTargetTime);
// Same but task-space

bool speedj(float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
bool speedl(float fTargetVel[NUM_TASK], float fTargetAcc[2], float fTargetTime);
```

---

## Real-Time Control (UDP)

Requires separate `connect_rt_control()` call. 1ms control cycle.

```cpp
bool set_rt_control_input(string strVersion, float fPeriod, int nLossCnt);
bool set_rt_control_output(string strVersion, float fPeriod, int nLossCnt);
bool start_rt_control();
bool stop_rt_control();

LPRT_OUTPUT_DATA_LIST read_data_rt();    // read current robot state
bool write_data_rt(float fExternalForceTorque[NUM_JOINT], int iExternalDI, int iExternalDO,
                   float fExternalAnalogInput[6], float fExternalAnalogOutput[6]);

bool servoj_rt(float fTargetPos[NUM_JOINT], float fTargetVel[NUM_JOINT],
               float fTargetAcc[NUM_JOINT], float fTargetTime);
bool servol_rt(float fTargetPos[NUM_TASK], float fTargetVel[NUM_TASK],
               float fTargetAcc[NUM_TASK], float fTargetTime);
bool speedj_rt(float fTargetVel[NUM_JOINT], float fTargetAcc[NUM_JOINT], float fTargetTime);
bool speedl_rt(float fTargetVel[NUM_TASK], float fTargetAcc[NUM_TASK], float fTargetTime);
bool torque_rt(float fMotorTor[NUM_JOINT], float fTargetTime);

void set_on_rt_monitoring_data(TOnRTMonitoringDataCB pCallbackFunc);
```

**If uncertain about RT API** — ask user to look up `6_realtime_control_sample.cpp` in the GitHub examples.

---

## GPIO

### Control Box

```cpp
bool  set_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex, bool bOnOff);
bool  get_digital_input(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex);
bool  get_digital_output(GPIO_CTRLBOX_DIGITAL_INDEX eGpioIndex);
bool  set_analog_output(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, float fValue);
float get_analog_input(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex);
bool  set_mode_analog_input(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType);
bool  set_mode_analog_output(GPIO_CTRLBOX_ANALOG_INDEX eGpioIndex, GPIO_ANALOG_TYPE eAnalogType);
```

### Flange (Tool)

```cpp
bool  set_tool_digital_output(GPIO_TOOL_DIGITAL_INDEX eGpioIndex, bool bOnOff);
bool  get_tool_digital_input(GPIO_TOOL_DIGITAL_INDEX eGpioIndex);
float get_tool_analog_input(int nCh);
bool  set_tool_digital_output_level(int nLv);  // voltage level
bool  set_mode_tool_analog_input(int nCh, GPIO_ANALOG_TYPE eAnalogType);
```

---

## Modbus

```cpp
bool           add_modbus_signal(const char* lpszSymbol, const char* lpszIpAddress,
                                  unsigned short nPort, MODBUS_REGISTER_TYPE eRegType,
                                  unsigned short iRegIndex, unsigned short nRegValue = 0,
                                  unsigned char nSlaveId = 255);
bool           del_modbus_signal(const char* lpszSymbol);
bool           set_modbus_output(const char* lpszSymbol, unsigned short nValue);
unsigned short get_modbus_input(const char* lpszSymbol);
LPMODBUS_DATA_LIST query_modbus_data_list();
```

---

## Force Control / Compliance

```cpp
bool task_compliance_ctrl(float fTargetStiffness[NUM_TASK],
                           COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL,
                           float fTargetTime = 0.f);
// Used in hand-guide enable: enters compliant mode

bool release_compliance_ctrl();
// Used in hand-guide disable: exits compliant mode

bool set_desired_force(float fTargetForce[NUM_TASK], unsigned char iTargetDirection[NUM_TASK],
                        COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL,
                        float fTargetTime = 0.f, FORCE_MODE eForceMode = FORCE_MODE_ABSOLUTE);
bool release_force(float fTargetTime = 0.f);

bool check_force_condition(FORCE_AXIS eForceAxis, float fTargetMin, float fTargetMax,
                            COORDINATE_SYSTEM eForceReference = COORDINATE_SYSTEM_TOOL);
```

---

## Configuration

```cpp
bool        set_tool(const char* lpszSymbol);
bool        add_tool(const char* lpszSymbol, float fWeight, float fCog[3], float fInertia[NUM_TASK]);
const char* get_tool();

bool        set_tcp(const char* lpszSymbol);
bool        add_tcp(const char* lpszSymbol, float fPosition[NUM_TASK]);
const char* get_tcp();

bool set_workpiece_weight(float fWeight = 0.0, float fCog[3] = COG_DEFAULT, ...);
bool set_singularity_handling(SINGULARITY_AVOIDANCE eMode);
bool change_collision_sensitivity(float fSensitivity);
bool change_operation_speed(float fSpeed);  // 1–100%
```

---

## DRL Script Execution

Execute DRL (Doosan Robot Language) Python-like scripts directly:

```cpp
bool drl_start(ROBOT_SYSTEM eRobotSystem, const char* lpszDrlProgram);
bool drl_stop(unsigned char eStopType = 0);
bool drl_pause();
bool drl_resume();
```

---

## Callbacks

Register before `open_connection()`.

```cpp
void set_on_monitoring_state(TOnMonitoringStateCB cb);        // ROBOT_STATE changes
void set_on_monitoring_data(TOnMonitoringDataCB cb);          // v0 monitoring
void set_on_monitoring_data_ex(TOnMonitoringDataExCB cb);     // v1 monitoring (preferred)
void set_on_monitoring_ctrl_io(TOnMonitoringCtrlIOCB cb);     // v0 I/O
void set_on_monitoring_ctrl_io_ex(TOnMonitoringCtrlIOExCB cb);// v1 I/O (preferred)
void set_on_monitoring_access_control(TOnMonitoringAccessControlCB cb);
void set_on_log_alarm(TOnLogAlarmCB cb);
void set_on_tp_popup(TOnTpPopupCB cb);
void set_on_tp_log(TOnTpLogCB cb);
void set_on_homming_completed(TOnHommingCompletedCB cb);
void set_on_tp_initializing_completed(TOnTpInitializingCompletedCB cb);
void set_on_program_stopped(TOnProgramStoppedCB cb);
void set_on_disconnected(TOnDisconnectedCB cb);
void set_on_mastering_need(TOnMasteringNeedCB cb);
void set_on_monitoring_safety_state(TOnMonitoringSafetyStateCB cb);
void set_on_rt_monitoring_data(TOnRTMonitoringDataCB cb);  // RT data (UDP)
```

**Callback constraint:** do minimal work (<50ms) inside any callback. No blocking calls.

---

## Safe Motion Variants

```cpp
bool _Safe_MoveJ(...);   // movej with additional safety checks
bool _Safe_MoveL(...);   // movel with additional safety checks
bool _Safe_MoveJX(...);  // movejx with safety checks
```

---

## Coordinate System Utilities

```cpp
int  set_user_cart_coord1(int iReqId, float fTargetPos[NUM_TASK],
                           COORDINATE_SYSTEM eTargetRef = COORDINATE_SYSTEM_BASE);
bool set_ref_coord(COORDINATE_SYSTEM eTargetCoordSystem);
bool enable_alter_motion(int iCycleTime, PATH_MODE ePathMode,
                          COORDINATE_SYSTEM eTargetRef, float fLimitDpos[2], float fLimitDposPer[2]);
bool alter_motion(float fTargetPos[NUM_TASK]);  // modify path in real-time
bool disable_alter_motion();
```

---

## Welding Application (app_weld_*)

High-level welding API wrapping EtherNet/IP or analog welder interfaces. Not used in our daemon but present.  
**If you need welding integration, consult `4_minimal_welding_sample.cpp` and Doosan manual.**

Functions: `app_weld_enable_digital`, `app_weld_set_weld_cond_digital`, `app_weld_weave_cond_trapezoidal`, etc.
