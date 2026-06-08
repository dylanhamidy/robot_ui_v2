# Doosan DRFL Motion Primitives — API Reference

> Source: `DRFLEx.h`, `DRFC.h`, `DRFS.h` from `lux_drfl_daemon/third_party/API-DRFL`  
> Manual: Doosan Robotics API Manual GL013303 v1.33.3, Sections 3.3.2–3.3.3

---

## Common Types & Constants

| Symbol | Value | Description |
|--------|-------|-------------|
| `NUM_JOINT` | 6 | Joint-space DOF (J1–J6, degrees) |
| `NUM_TASK` | 6 | Task-space DOF (X, Y, Z mm · A, B, C deg) |
| `MAX_MOVEB_POINT` | 50 | Max segments in a `moveb` blend array |
| `MAX_SPLINE_POINT` | 100 | Max waypoints in a spline (`movesj`/`movesx`) |

### `MOVE_MODE`
| Value | Meaning |
|-------|---------|
| `MOVE_MODE_ABSOLUTE` | Target pos is absolute in the reference frame |
| `MOVE_MODE_RELATIVE` | Target pos is an offset from current pos |

### `MOVE_REFERENCE`
| Value | Meaning |
|-------|---------|
| `MOVE_REFERENCE_BASE` | Robot base frame |
| `MOVE_REFERENCE_TOOL` | TCP/tool frame |
| `MOVE_REFERENCE_WORLD` | World frame |
| `MOVE_REFERENCE_USER_MIN … USER_MAX` | User-defined frames |

### `BLENDING_SPEED_TYPE`
| Value | Meaning |
|-------|---------|
| `BLENDING_SPEED_TYPE_DUPLICATE` | Speed profile duplicates at blend boundary (smooth) |
| `BLENDING_SPEED_TYPE_OVERRIDE` | Override speed at blend point |

### `DR_MV_APP`
| Value | Meaning |
|-------|---------|
| `DR_MV_APP_NONE` | No application extension |
| `DR_MV_APP_WELD` | Welding application mode (enables weld I/O sync) |

### `TASK_AXIS`
| Value | Axis |
|-------|------|
| `TASK_AXIS_X` | X axis of reference frame |
| `TASK_AXIS_Y` | Y axis |
| `TASK_AXIS_Z` | Z axis |

### `SPLINE_VELOCITY_OPTION`
| Value | Meaning |
|-------|---------|
| `SPLINE_VELOCITY_OPTION_DEFAULT` | Velocity varies naturally through waypoints |
| `SPLINE_VELOCITY_OPTION_CONST` | Constant velocity through entire spline |

### `MOVEB_BLENDING_TYPE`
| Value | Meaning |
|-------|---------|
| `MOVEB_BLENDING_TYPE_LINE` (0) | Linear segment in the blend array |
| `MOVEB_BLENDING_TYPE_CIRLCE` (1) | Circular segment (arc) in the blend array |

### `MOVE_POSB` struct (used by `moveb`)
```cpp
typedef struct _MOVE_POSB {
    float         _fTargetPos[2][NUM_TASK];  // [0]=via or line endpoint, [1]=arc endpoint (circle only)
    unsigned char _iBlendType;               // 0=line, 1=circle
    float         _fBlendRad;                // blending radius (mm); 0 = stop at this waypoint
} MOVE_POSB;
```

---

## 3.3.2 Motion Primitives — Basic Trajectory

These are **blocking (synchronous)** commands — they block the calling thread until the motion completes (or is stopped). Each has an `a`-prefixed **asynchronous** variant that returns immediately after dispatching the command.

---

### `movej` — Joint Space Move

Moves all joints simultaneously along a joint-interpolated path to a target joint configuration. The TCP traces an arbitrary path in Cartesian space; only joint angles are controlled. Fastest way to move between distant poses without Cartesian constraints.

```cpp
bool movej(
    float fTargetPos[NUM_JOINT],       // target joint angles (deg), J1–J6
    float fTargetVel,                  // joint velocity (deg/s)
    float fTargetAcc,                  // joint acceleration (deg/s²)
    float fTargetTime        = 0.f,    // motion duration (s); 0 = derive from vel/acc
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    float fBlendingRadius    = 0.f,    // blending radius (mm); 0 = stop-and-go
    BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE
);
```

**Per-joint velocity overload:**
```cpp
bool movej(
    float fTargetPos[NUM_JOINT],
    float fTargetVel[NUM_JOINT],       // individual vel per joint (deg/s)
    float fTargetAcc[NUM_JOINT],       // individual acc per joint (deg/s²)
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    float fBlendingRadius    = 0.f,
    BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE
);
```

**Returns:** `true` on success, `false` on error.

**Notes:**
- `fTargetTime > 0` overrides vel/acc — robot scales motion to finish in exactly that duration.
- `fBlendingRadius > 0` enables continuous-path blending into the next move (no stop at target).
- Joint-space: no Cartesian path guarantee, no singularity issues.

**Caution:**
- When the following motion blends with `BLENDING_SPEED_TYPE_DUPLICATE` and `fBlendingRadius > 0`, the preceding motion can terminate **after** the following motion finishes if the preceding motion's remaining time (based on remaining distance, velocity, and acceleration) is greater than the following motion's total time.

**Examples:**
```cpp
// CASE 1 — velocity/acceleration-based
float q0[6] = { 0, 0, 90, 0, 90, 0 };
drfl.movej(q0, 10, 20);
// Moves to q0 at 10 deg/s, 20 deg/s²

// CASE 2 — time-based
float q0r[6] = { 0, 0, 90, 0, 90, 0 };
drfl.movej(q0r, 0, 0, 5);
// Moves to q0r in exactly 5 s (vel/acc ignored)

// CASE 3 — blending with DUPLICATE
float q0b[6] = { 0, 0, 90, 0, 90, 0 };
float q1b[6] = { 90, 0, 90, 0, 90, 0 };
drfl.movej(q0b, 10, 20, 0, MOVE_MODE_ABSOLUTE, 50);
// Start blending into next move when 50 mm from q0b
drfl.movej(q1b, 10, 20, 0, MOVE_MODE_ABSOLUTE, 0, BLENDING_SPEED_TYPE_DUPLICATE);
// Blends seamlessly from preceding move into q1b
```

---

### `amovej` — Async Joint Space Move

Non-blocking variant of `movej`. Returns immediately after dispatching the command. The motion executes in the background.

```cpp
bool amovej(
    float fTargetPos[NUM_JOINT],
    float fTargetVel,
    float fTargetAcc,
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE
);
```

**Per-joint overload** also available (same pattern as `movej`).  
**Note:** No `fBlendingRadius` param — async blending handled by issuing the next async command before the current one ends.

---

### `movel` — Linear Cartesian Move

Moves the TCP in a straight line in Cartesian space to the target pose. Both position and orientation are linearly interpolated. Use when path shape matters (e.g., approaching a workpiece, welding).

```cpp
bool movel(
    float fTargetPos[NUM_TASK],        // target pose [X, Y, Z mm, A, B, C deg]
    float fTargetVel[2],               // [0]=linear vel (mm/s), [1]=angular vel (deg/s)
    float fTargetAcc[2],               // [0]=linear acc (mm/s²), [1]=angular acc (deg/s²)
    float fTargetTime        = 0.f,    // motion duration (s); 0 = derive from vel/acc
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    float fBlendingRadius    = 0.f,    // blending radius (mm) into next move
    BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE,
    DR_MV_APP eAppType       = DR_MV_APP_NONE
);
```

**Returns:** `true` on success, `false` on error.

**Notes:**
- `fTargetVel[0]` controls TCP speed along the line; `[1]` controls rotational speed.
- Singularities possible near certain configurations — prefer `movej` for large joint-space moves.
- `DR_MV_APP_WELD` enables synchronization with Doosan welding I/O during motion.
- If only `fTargetVel[0]` is set (e.g., `{30, 0}`), linear velocity is applied as given; angular velocity is determined proportionally. Same for `fTargetAcc`.
- `fTargetTime > 0` overrides vel/acc entirely.

**Caution:**
- When the following motion blends with `BLENDING_SPEED_TYPE_DUPLICATE` and `fBlendingRadius > 0`, the preceding motion can terminate **after** the following motion finishes if remaining motion time exceeds the following motion's total time.

**Examples:**
```cpp
// CASE 1 — velocity/acceleration-based
float x1[6]   = { 559, 434.5, 651.5, 0, 180, 0 };
float tvel[2] = { 50, 50 };
float tacc[2] = { 100, 100 };
drfl.movel(x1, tvel, tacc);
// Moves to x1 at 50 mm/s, 100 mm/s²

// CASE 2 — time-based
float x1r[6] = { 559, 434.5, 651.5, 0, 180, 0 };
drfl.movel(x1r, 0, 0, 5);
// Moves to x1r in exactly 5 s

// CASE 3 — relative move in tool frame
float x1t[6] = { 559, 434.5, 651.5, 0, 180, 0 };
drfl.movel(x1t, {50,50}, {100,100}, 0, MOVE_MODE_RELATIVE, MOVE_REFERENCE_TOOL);
// Moves x1t offset from current position in tool coordinates

// CASE 4 — blending into next movel
float x1b[6] = { 559, 434.5, 651.5, 0, 180, 0 };
float x2b[6] = { 559, 434.5, 251.5, 0, 180, 0 };
float tvelb[2] = { 50, 50 };
float taccb[2] = { 100, 100 };
drfl.movel(x1b, tvelb, taccb, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 100);
// Begin blending 100 mm before x1b; chain next movel
```

---

### `amovel` — Async Linear Cartesian Move

Non-blocking variant of `movel`.

```cpp
bool amovel(
    float fTargetPos[NUM_TASK],
    float fTargetVel[2],
    float fTargetAcc[2],
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE,
    DR_MV_APP eAppType       = DR_MV_APP_NONE
);
```

---

### `movec` — Circular Arc Move

Moves the TCP along a circular arc defined by three points: **the current TCP position (implicit start)**, a **via point**, and an **end point**. The robot fits a circle through these three poses and moves along the arc.

```cpp
bool movec(
    float fTargetPos[2][NUM_TASK],     // [0]=via pose, [1]=end pose
    float fTargetVel[2],               // [0]=linear vel (mm/s), [1]=angular vel (deg/s)
    float fTargetAcc[2],               // [0]=linear acc (mm/s²), [1]=angular acc (deg/s²)
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    float fTargetAngle1      = 0.f,    // arc angle constraint (deg); 0 = unconstrained
    float fTargetAngle2      = 0.f,    // full rotation angle (0=arc, 360=full circle)
    float fBlendingRadius    = 0.f,
    BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE,
    MOVE_ORIENTATION eOrientation = DR_MV_ORI_TEACH,
    DR_MV_APP eAppType       = DR_MV_APP_NONE
);
```

**Returns:** `true` on success.

**Notes:**
- The starting point is **implicit** — wherever the TCP currently is when the command executes.
- `fTargetAngle2 = 360` → full circle (robot continues past end point back to start).
- `fTargetPos[0]` (via) and `fTargetPos[1]` (end) must not be collinear with the start position — three collinear points cannot define a circle.
- **Inside `FreeForm`/`moveb` blend segments:** the start is implicit (previous segment's end); only `pos_via` and `pos_end` are provided. See `moveb` below.
- If only `fTargetVel[0]` is set (e.g., `{30, 0}`), linear velocity applies; angular velocity is determined proportionally. Same for `fTargetAcc`.
- `fTargetTime > 0` overrides vel/acc entirely.
- `MOVE_MODE_RELATIVE`: `fTargetPos[0]` is relative from the starting point; `fTargetPos[1]` is relative from `fTargetPos[0]`.
- `fTargetAngle1 > 0, fTargetAngle2 == 0`: total rotated angle along circular path = `fTargetAngle1`.
- `fTargetAngle1 > 2, fTargetAngle2 > 2`: `fTargetAngle1` = constant-velocity angle, `fTargetAngle2` = accel/decel angle on each end. Total arc = `fTargetAngle1 + 2 × fTargetAngle2`.

**Caution:**
- When the following motion blends with `BLENDING_SPEED_TYPE_DUPLICATE` and `fBlendingRadius > 0`, the preceding motion can terminate **after** the following motion finishes if remaining motion time exceeds the following motion's total time.

**Examples:**
```cpp
// CASE 1 — arc via x1[0] to x1[1]
float x1[2][6] = { {559,434.5,651.5,0,180,0}, {559,434.5,251.5,0,180,0} };
float tvel[2]  = { 50, 50 };
float tacc[2]  = { 100, 100 };
drfl.movec(x1, tvel, tacc);
// Arc from current pos through x1[0] (via) to x1[1] (end)

// CASE 2 — time-based arc
float x1r[2][6] = { {559,434.5,651.5,0,180,0}, {559,434.5,251.5,0,180,0} };
drfl.movec(x1r, 0, 0, 5);
// Same arc in exactly 5 s

// CASE 3 — two chained arc segments with blending
float x1b[2][6] = { {559,434.5,651.5,0,180,0}, {559,434.5,251.5,0,180,0} };
float x2b[2][6] = { {559,234.5,651.5,0,180,0}, {559,234.5,451.5,0,180,0} };
float tvelb[2] = { 50, 50 };
float taccb[2] = { 100, 100 };
drfl.movec(x1b, tvelb, taccb, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 0, 0, 50);
// First arc with 50 mm blend radius into next
drfl.movec(x2b, tvelb, taccb, 0, MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE, 0, 0, 0, BLENDING_SPEED_TYPE_DUPLICATE);
// Second arc, seamlessly blended from first
```

---

### `amovec` — Async Circular Arc Move

Non-blocking variant of `movec`.

```cpp
bool amovec(
    float fTargetPos[2][NUM_TASK],
    float fTargetVel[2],
    float fTargetAcc[2],
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    float fTargetAngle1      = 0.f,
    float fTargetAngle2      = 0.f,
    BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE,
    MOVE_ORIENTATION eOrientation = DR_MV_ORI_TEACH,
    DR_MV_APP eAppType       = DR_MV_APP_NONE
);
```

---

### `moveb` — Blended Multi-Segment Move (Look-Ahead)

Executes a continuous, look-ahead blended path through an array of segments. Each segment is either a **linear** or **circular** sub-move. The planner computes the entire trajectory upfront and blends transitions with the specified radius — no stops between segments. This is the underlying primitive behind the `FreeForm` plan step.

```cpp
bool moveb(
    MOVE_POSB tTargetPos[MAX_MOVEB_POINT],  // array of blend segments (up to 50)
    unsigned char nPosCount,                 // number of segments in array
    float fTargetVel[2],                     // [0]=linear vel (mm/s), [1]=angular vel (deg/s)
    float fTargetAcc[2],                     // [0]=linear acc (mm/s²), [1]=angular acc (deg/s²)
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    DR_MV_APP eAppType       = DR_MV_APP_NONE
);
```

**`MOVE_POSB` segment structure:**
```cpp
struct MOVE_POSB {
    float         _fTargetPos[2][NUM_TASK]; // [0]=line endpoint OR arc via; [1]=arc end (circle only)
    unsigned char _iBlendType;              // 0=MOVEB_BLENDING_TYPE_LINE, 1=MOVEB_BLENDING_TYPE_CIRLCE
    float         _fBlendRad;              // blending radius (mm); 0 on last segment
};
```

**Returns:** `true` on success.

**Notes:**
- **Linear segment** (`_iBlendType=0`): `_fTargetPos[0]` is the endpoint; `[1]` is ignored.
- **Circular segment** (`_iBlendType=1`): `_fTargetPos[0]` is the via point; `[1]` is the endpoint. The arc start is implicitly the end of the previous segment (or current TCP pos for the first segment).
- Set `_fBlendRad = 0` on the last segment — no blending needed after the final point.
- Velocity/acceleration apply uniformly to the whole compound path; per-segment speed not supported.
- `MAX_MOVEB_POINT = 50`. Exceeding this causes an error.
- If only `fTargetVel[0]` is set (e.g., `{30, 0}`), linear velocity applies; angular velocity is determined proportionally. Same for `fTargetAcc`.
- `fTargetTime > 0` overrides vel/acc for the entire compound path.
- `MOVE_MODE_RELATIVE`: each `MOVE_POSB` pos is defined relative to the preceding pos in the array.

**`FreeForm` plan step maps directly to `moveb`:** each `MoveL` sub-step → linear segment, each `MoveC` sub-step → circular segment with only `pos_via`+`pos_end` (no `pos_start` — it is implicit).

**Cautions:**
- `_fBlendRad = 0` on any non-final segment → user input error (blend radius required between segments).
- Contiguous Line–Line segments with the **same direction** → user input error (duplicate direction).
- Blend condition causing rapid direction change → user input error (prevents sudden acceleration).
- Does **not** support online blending with preceding or following motions outside the `moveb` call.

**Example:**
```cpp
// LINE → CIRCLE → LINE → LINE compound path
MOVE_POSB xb[4];
memset(xb, 0x00, sizeof(xb));
float tvel[2] = { 50, 50 };
float tacc[2] = { 100, 100 };

xb[0]._iBlendType = 0;  xb[0]._fBlendRad = 50;  // line segment
xb[0]._fTargetPos[0] = 559; xb[0]._fTargetPos[1] = 234.5; xb[0]._fTargetPos[2] = 651.5;
xb[0]._fTargetPos[3] = 0;   xb[0]._fTargetPos[4] = 180;   xb[0]._fTargetPos[5] = 0;

xb[1]._iBlendType = 1;  xb[1]._fBlendRad = 50;  // circle segment (via=[0], end=[1])
xb[1]._fTargetPos[0] = 559; xb[1]._fTargetPos[1] = 234.5; xb[1]._fTargetPos[2] = 451.5;
xb[1]._fTargetPos[3] = 0;   xb[1]._fTargetPos[4] = 180;   xb[1]._fTargetPos[5] = 0;

xb[2]._iBlendType = 0;  xb[2]._fBlendRad = 50;  // line segment
xb[2]._fTargetPos[0] = 559; xb[2]._fTargetPos[1] = 434.5; xb[2]._fTargetPos[2] = 451.5;
xb[2]._fTargetPos[3] = 0;   xb[2]._fTargetPos[4] = 180;   xb[2]._fTargetPos[5] = 0;

xb[3]._iBlendType = 0;  xb[3]._fBlendRad = 0;   // last segment — no blend
xb[3]._fTargetPos[0] = 559; xb[3]._fTargetPos[1] = 234.5; xb[3]._fTargetPos[2] = 251.5;
xb[3]._fTargetPos[3] = 0;   xb[3]._fTargetPos[4] = 180;   xb[3]._fTargetPos[5] = 0;

drfl.moveb(xb, 4, tvel, tacc);
// Executes LINE–CIRCLE–LINE–LINE at 50 mm/s, 100 mm/s²
// Blending at 50 mm radius between each segment
```

---

### `amoveb` — Async Blended Multi-Segment Move

Non-blocking variant of `moveb`.

```cpp
bool amoveb(
    MOVE_POSB tTargetPos[MAX_MOVEB_POINT],
    unsigned char nPosCount,
    float fTargetVel[2],
    float fTargetAcc[2],
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    DR_MV_APP eAppType       = DR_MV_APP_NONE
);
```

---

## 3.3.3 Motion Primitives — Advanced / Blending / Special Trajectories

---

### `movejx` — Joint Move to Cartesian Target with IK Solution Space

Like `movej` but accepts a **Cartesian target pose** and a solution space index. The controller runs inverse kinematics internally to compute the joint configuration, then moves joint-space. Useful when you know the desired TCP pose but want joint-space motion dynamics.

```cpp
bool movejx(
    float fTargetPos[NUM_JOINT],       // target Cartesian pose [X,Y,Z mm, A,B,C deg] (not joints!)
    unsigned char iSolutionSpace,      // IK solution space index (0–7, selects elbow/wrist config)
    float fTargetVel,                  // joint velocity (deg/s)
    float fTargetAcc,                  // joint acceleration (deg/s²)
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    float fBlendingRadius    = 0.f,
    BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE
);
```

**Per-joint velocity overload** also available.

**Returns:** `true` on success.

**Notes:**
- `iSolutionSpace` encodes the arm/elbow/wrist configuration as a 3-bit integer (bit 0=elbow, bit 1=wrist, bit 2=flip). Use `get_solution_space(targetPose)` to query the current configuration before calling.
- Unlike `movel`, the TCP does **not** travel in a straight Cartesian line — it follows the joint-interpolated path.
- Use when you need the convenience of specifying a Cartesian goal but want joint-space kinematics.

---

### `amovejx` — Async Joint Move to Cartesian Target

Non-blocking variant of `movejx`.

```cpp
bool amovejx(
    float fTargetPos[NUM_JOINT],
    unsigned char iSolutionSpace,
    float fTargetVel,
    float fTargetAcc,
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    BLENDING_SPEED_TYPE eBlendingType = BLENDING_SPEED_TYPE_DUPLICATE
);
```

---

### `movesj` — Spline Move in Joint Space

Moves the robot through an ordered array of joint-space waypoints using **spline interpolation**. The path is smooth and continuous through all waypoints — no stopping between them. Equivalent to a smooth polynomial fit through joint angles.

```cpp
bool movesj(
    float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT],  // array of joint waypoints (deg)
    unsigned char nPosCount,                          // number of waypoints (≤ 100)
    float fTargetVel,                                 // joint velocity (deg/s)
    float fTargetAcc,                                 // joint acceleration (deg/s²)
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE
);
```

**Returns:** `true` on success.

**Notes:**
- Robot passes **through** each waypoint (interpolating curve, not blending radius).
- No Cartesian path guarantee — waypoints are joint angles.
- Suitable for pre-programmed smooth joint trajectories (e.g., pick-and-place arcs).

---

### `amovesj` — Async Spline Move in Joint Space

Non-blocking variant of `movesj`.

```cpp
bool amovesj(
    float fTargetPos[MAX_SPLINE_POINT][NUM_JOINT],
    unsigned char nPosCount,
    float fTargetVel,
    float fTargetAcc,
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE
);
```

---

### `movesx` — Spline Move in Cartesian Space

Moves the TCP through an array of Cartesian poses using spline interpolation. Produces a smooth curve in task space through all waypoints. More Cartesian-aware than `movesj` — the TCP traces a smooth spatial curve.

```cpp
bool movesx(
    float fTargetPos[MAX_SPLINE_POINT][NUM_TASK],  // array of Cartesian poses [X,Y,Z mm, A,B,C deg]
    unsigned char nPosCount,                         // number of waypoints (≤ 100)
    float fTargetVel[2],                             // [0]=linear vel (mm/s), [1]=angular vel (deg/s)
    float fTargetAcc[2],                             // [0]=linear acc (mm/s²), [1]=angular acc (deg/s²)
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT
);
```

**Returns:** `true` on success.

**Notes:**
- `SPLINE_VELOCITY_OPTION_CONST`: robot maintains constant TCP speed along the spline (good for uniform process coverage like painting or dispensing).
- `SPLINE_VELOCITY_OPTION_DEFAULT`: speed varies naturally through waypoint curvature.
- TCP path is a smooth curve — not a series of line segments.

---

### `amovesx` — Async Spline Move in Cartesian Space

Non-blocking variant of `movesx`.

```cpp
bool amovesx(
    float fTargetPos[MAX_SPLINE_POINT][NUM_TASK],
    unsigned char nPosCount,
    float fTargetVel[2],
    float fTargetAcc[2],
    float fTargetTime        = 0.f,
    MOVE_MODE eMoveMode      = MOVE_MODE_ABSOLUTE,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_BASE,
    SPLINE_VELOCITY_OPTION eVelOpt = SPLINE_VELOCITY_OPTION_DEFAULT
);
```

---

### `move_spiral` — Spiral Trajectory

Generates an outward spiral trajectory centered on the current TCP position, expanding along a specified task-frame axis. Commonly used for **search patterns** (finding a hole, locating a surface feature) or **polishing** applications.

```cpp
bool move_spiral(
    TASK_AXIS eTaskAxis,          // axis of spiral expansion (X, Y, or Z in reference frame)
    float fRevolution,            // number of full revolutions
    float fMaximuRadius,          // maximum radius at end of spiral (mm)
    float fMaximumLength,         // maximum travel along the axis (mm)
    float fTargetVel[2],          // [0]=linear vel (mm/s), [1]=angular vel (deg/s)
    float fTargetAcc[2],          // [0]=linear acc (mm/s²), [1]=angular acc (deg/s²)
    float fTargetTime    = 0.f,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL
);
```

**Returns:** `true` on success.

**Notes:**
- Default reference is `MOVE_REFERENCE_TOOL` — spiral expands in the tool frame. Switch to `MOVE_REFERENCE_BASE` to expand in the world frame.
- `fMaximumLength` controls axial penetration during the spiral (useful for insertion searches).
- Radius grows linearly with revolutions from 0 to `fMaximuRadius`.
- Orientation is held constant throughout the motion.

---

### `amove_spiral` — Async Spiral Trajectory

Non-blocking variant of `move_spiral`.

```cpp
bool amove_spiral(
    TASK_AXIS eTaskAxis,
    float fRevolution,
    float fMaximuRadius,
    float fMaximumLength,
    float fTargetVel[2],
    float fTargetAcc[2],
    float fTargetTime    = 0.f,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL
);
```

---

### `move_periodic` — Periodic Oscillating Motion

Commands the TCP to oscillate periodically along/around each task-space axis with independent amplitude and period per axis. Produces sinusoidal motion superimposed on the current TCP position. Used for **weaving**, **vibration testing**, or **oscillating surface contact**.

```cpp
bool move_periodic(
    float fAmplitude[NUM_TASK],    // oscillation amplitude per axis [X,Y,Z mm, A,B,C deg]
    float fPeriodic[NUM_TASK],     // oscillation period per axis (s); 0 = no oscillation on that axis
    float fAccelTime,              // ramp-up time to reach full amplitude (s)
    unsigned char nRepeat,         // number of complete oscillation cycles; 0 = infinite
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL
);
```

**Returns:** `true` on success.

**Notes:**
- Each axis oscillates independently: `X_t = X_0 + fAmplitude[0] * sin(2π t / fPeriodic[0])`.
- Set `fPeriodic[i] = 0` to disable oscillation on axis `i`.
- `fAccelTime` prevents sudden jumps at motion start — amplitude ramps from 0 to full over this duration.
- `nRepeat = 0` runs indefinitely until a `stop` command.
- Default reference is tool frame — oscillation happens relative to current TCP orientation.

---

### `amove_periodic` — Async Periodic Oscillating Motion

Non-blocking variant of `move_periodic`.

```cpp
bool amove_periodic(
    float fAmplitude[NUM_TASK],
    float fPeriodic[NUM_TASK],
    float fAccelTime,
    unsigned char nRepeat,
    MOVE_REFERENCE eMoveReference = MOVE_REFERENCE_TOOL
);
```

---

## Sync vs Async Summary

| Command | Blocks until done | Blending radius param |
|---------|:-----------------:|:--------------------:|
| `movej` | yes | yes |
| `amovej` | no | no (issue next cmd before end) |
| `movel` | yes | yes |
| `amovel` | no | no |
| `movec` | yes | yes |
| `amovec` | no | no |
| `moveb` | yes | per-segment `_fBlendRad` |
| `amoveb` | no | per-segment `_fBlendRad` |
| `movejx` | yes | yes |
| `amovejx` | no | no |
| `movesj` | yes | n/a (spline) |
| `amovesj` | no | n/a |
| `movesx` | yes | n/a (spline) |
| `amovesx` | no | n/a |
| `move_spiral` | yes | n/a |
| `amove_spiral` | no | n/a |
| `move_periodic` | yes | n/a |
| `amove_periodic` | no | n/a |

## Motion Command Selection Guide

```
Need to move between poses?
├── Don't care about TCP path → movej (fastest, no singularity risk)
├── TCP must travel straight line → movel
├── TCP must follow an arc → movec
├── Smooth path through many points in joint space → movesj
├── Smooth curve through many Cartesian poses → movesx
├── Continuous blended path (line + arc mix) → moveb  [used by FreeForm step]
├── Search pattern / polishing → move_spiral
├── Welding weave / vibration → move_periodic
└── Cartesian target but joint-space dynamics → movejx
```
