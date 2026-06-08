/*
 * lux_drfl_daemon — persistent DRFL C++ process that owns the robot connection.
 * Reads newline-delimited JSON commands from stdin, emits sentinel strings on stdout.
 *
 * Build:  cmake -B build -DDRCF_VERSION=3 . && cmake --build build -j
 * Run:    ./build/drfl_daemon --ip 192.168.0.20 --port 12345 --drcf 3
 *
 * Sentinel vocabulary (stdout):
 *   [CONNECTED]          connection + servo-on succeeded
 *   [DISCONNECTED]       robot dropped connection
 *   [STEP_START] N       step N about to execute
 *   [DONE] <msg>         plan finished or cancelled
 *   [ERROR] <msg>        fatal or command error
 *   [INFO] <msg>         informational
 */

#include <DRFLEx.h>

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <ctime>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <csignal>
#include <curl/curl.h>

using json = nlohmann::json;
using namespace DRAFramework;  // DRFL C++ namespace

// ── Globals ───────────────────────────────────────────────────────────────────

static CDRFLEx g_robot;

static std::atomic<bool> g_shutdown{false};
static std::atomic<bool> g_cancel{false};
static std::atomic<bool> g_connected{false};  // true once [CONNECTED] emitted

// Thread-safe stdout
static std::mutex g_out_mutex;
static void emit(const std::string& line) {
    std::lock_guard<std::mutex> lk(g_out_mutex);
    std::cout << line << std::endl;
    std::cout.flush();
}

// Robot state
static std::mutex              g_state_mx;
static std::condition_variable g_state_cv;
static ROBOT_STATE             g_robot_state = STATE_INITIALIZING;

// Access control
static std::mutex              g_access_mx;
static std::condition_variable g_access_cv;
static MONITORING_ACCESS_CONTROL g_access_ctrl = MONITORING_ACCESS_CONTROL_REQUEST;

// Worker thread for run_plan
static std::thread  g_worker;
static std::mutex   g_worker_mx;

// Persistent parameters (set via set_param command)
static std::mutex   g_param_mx;
static std::string  g_current_type{"MoveJ"};
static float        g_default_vel{30.0f};
static float        g_default_acc{30.0f};
static float        g_default_time{0.0f};

// Flange button monitoring
static std::mutex              g_btn_mx;
static unsigned int            g_prev_bt[NUM_BUTTON] = {};
static std::atomic<bool>       g_hand_guide_active{false};

// Captured points accumulator (hand guide)

// ── Callbacks ─────────────────────────────────────────────────────────────────

static const char* robotStateName(ROBOT_STATE s) {
    switch (s) {
        case STATE_INITIALIZING: return "INITIALIZING";
        case STATE_STANDBY:      return "STANDBY";
        case STATE_MOVING:       return "MOVING";
        case STATE_SAFE_OFF:     return "SAFE_OFF";
        case STATE_TEACHING:     return "TEACHING";
        case STATE_SAFE_STOP:    return "SAFE_STOP";
        case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
        case STATE_HOMMING:      return "HOMMING";
        case STATE_RECOVERY:     return "RECOVERY";
        default:                 return "UNKNOWN";
    }
}

static void onState(const ROBOT_STATE state) {
    {
        std::lock_guard<std::mutex> lk(g_state_mx);
        g_robot_state = state;
    }
    emit(std::string("[STATE] ") + robotStateName(state));
    g_state_cv.notify_all();
}

static void onAccessControl(const MONITORING_ACCESS_CONTROL ctrl) {
    {
        std::lock_guard<std::mutex> lk(g_access_mx);
        g_access_ctrl = ctrl;
    }
    g_access_cv.notify_all();
    if (ctrl == MONITORING_ACCESS_CONTROL_LOSS) {
        if (g_connected.load()) {
            emit("[ERROR] Access control lost");
            g_shutdown = true;
            g_state_cv.notify_all();  // wake waitForStandby
        } else {
            // LOSS during initial handshake = controller notifying us that the
            // previous holder (e.g. teach pendant) just gave up access.
            // GRANT follows immediately — do not abort.
            emit("[INFO] Access control transferring (previous holder released)");
        }
    }
}

static void onLogAlarm(LPLOG_ALARM tLog) {
    std::string msg = "[ALARM] group=" + std::to_string((int)tLog->_iGroup) +
                      " index=" + std::to_string(tLog->_iIndex) +
                      " param=[" + tLog->_szParam[0] + "," +
                      tLog->_szParam[1] + "," + tLog->_szParam[2] + "]";
    emit(msg);
}

static void onTpLog(const char msg[256]) {
    emit(std::string("[TP] ") + msg);
}

static void onSafetyStopType(const unsigned char stop_type) {
    emit("[SAFETY_STOP] type=" + std::to_string((int)stop_type));
}

static void onDisconnected() {
    emit("[DISCONNECTED]");
    g_shutdown = true;
    g_state_cv.notify_all();  // wake waitForStandby
}

// ── libcurl HTTP POST ─────────────────────────────────────────────────────────

static size_t curlNullWrite(char*, size_t size, size_t nmemb, void*) {
    return size * nmemb;
}

static bool httpPost(const std::string& url, const std::string& body) {
    CURL* curl = curl_easy_init();
    if (!curl) return false;

    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, body.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlNullWrite);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);

    CURLcode res = curl_easy_perform(curl);

    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return res == CURLE_OK;
}

// ── Motion helpers ────────────────────────────────────────────────────────────

static bool waitForStandby(int timeout_ms = 30000) {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeout_ms);

    // Wait until motion has dispatched (MOVING) or already completed (STANDBY).
    // If the motion API is synchronous/blocking, the state may already be STANDBY
    // by the time this is called — treat that as "done dispatching" and skip ahead.
    {
        std::unique_lock<std::mutex> lk(g_state_mx);
        g_state_cv.wait_until(lk, deadline, [] {
            return g_robot_state == STATE_MOVING  ||
                   g_robot_state == STATE_STANDBY ||
                   g_cancel.load() || g_shutdown.load();
        });
    }
    if (g_cancel || g_shutdown) return false;

    // Wait for STANDBY (motion complete)
    {
        std::unique_lock<std::mutex> lk(g_state_mx);
        return g_state_cv.wait_until(lk, deadline, [] {
            return g_robot_state == STATE_STANDBY ||
                   g_cancel.load() || g_shutdown.load();
        }) && !g_cancel && !g_shutdown;
    }
}

// stepIdx is passed so MoveC can emit [STEP_START] after approach (not before).
static bool execStep(const json& step, int stepIdx) {
    std::string type = step.value("type", "MoveJ");

    if (type == "MoveC") {
        // Parse positions
        auto parse6 = [](const json& j, float out[6]) {
            for (int i = 0; i < 6; i++) out[i] = j[i].get<float>();
        };
        float pos_start[6], pos_via[6], pos_end[6];
        parse6(step["pos_start"], pos_start);
        parse6(step["pos_via"],   pos_via);
        parse6(step["pos_end"],   pos_end);

        float vel2[2] = {50.f, 50.f}, acc2[2] = {100.f, 100.f};
        if (step.contains("vel")) {
            auto& vv = step["vel"];
            if (vv.is_array()) { vel2[0] = vv[0].get<float>(); vel2[1] = vv[1].get<float>(); }
            else                { vel2[0] = vel2[1] = vv.get<float>(); }
        }
        if (step.contains("acc")) {
            auto& aa = step["acc"];
            if (aa.is_array()) { acc2[0] = aa[0].get<float>(); acc2[1] = aa[1].get<float>(); }
            else                { acc2[0] = acc2[1] = aa.get<float>(); }
        }
        float time_s  = step.value("time", 0.f);
        float angle2  = step.value("angle2", 0.f);

        emit("[INFO] MoveC step " + std::to_string(stepIdx) +
             ": approach A=[" + std::to_string(pos_start[0]) + "," +
             std::to_string(pos_start[1]) + "," + std::to_string(pos_start[2]) + "]"
             " angle2=" + std::to_string(angle2));

        // Phase 1: fast approach to A
        float fv[2] = {100.f, 100.f}, fa[2] = {100.f, 100.f};
        bool approach_ok = g_robot.movel(pos_start, fv, fa, 0.f, MOVE_MODE_ABSOLUTE,
                                         MOVE_REFERENCE_BASE, 0.f, BLENDING_SPEED_TYPE_DUPLICATE);
        emit("[INFO] MoveC approach movel returned " + std::string(approach_ok ? "true" : "false"));
        if (!waitForStandby()) {
            emit("[ERROR] MoveC approach waitForStandby failed (cancel/shutdown)");
            return false;
        }
        emit("[INFO] MoveC approach complete, starting arc");

        // [STEP_START] emitted here — after approach, laser fires at arc start
        emit("[STEP_START] " + std::to_string(stepIdx));

        // Single movec call — fTargetAngle2=360 for full circle, 0 for partial arc
        float arc[2][NUM_TASK];
        for (int i = 0; i < NUM_TASK; i++) arc[0][i] = pos_via[i];
        for (int i = 0; i < NUM_TASK; i++) arc[1][i] = pos_end[i];

        emit("[INFO] MoveC arc: angle2=" + std::to_string(angle2));
        // fTargetAngle1 = total sweep angle (360 = full circle), fTargetAngle2 = accel/decel angle
        bool arc_ok = g_robot.movec(arc, vel2, acc2, time_s,
                                    MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE,
                                    angle2, 0.f);
        emit("[INFO] MoveC movec returned " + std::string(arc_ok ? "true" : "false"));
        if (!waitForStandby()) {
            emit("[ERROR] MoveC arc waitForStandby failed");
            return false;
        }
        emit("[INFO] MoveC step " + std::to_string(stepIdx) + " complete");
        return true;
    }

    if (type == "WeldStraight") {
        auto parse6 = [](const json& j, float out[6]) {
            for (int i = 0; i < 6; i++) out[i] = j[i].get<float>();
        };
        float pos_a[6], pos_b[6];
        parse6(step["pos_a"], pos_a);
        parse6(step["pos_b"], pos_b);

        float fv[2] = {100.f, 100.f}, fa[2] = {100.f, 100.f};
        emit("[INFO] WeldStraight step " + std::to_string(stepIdx) + ": approach A");
        bool approach_ok = g_robot.movel(pos_a, fv, fa, 0.f, MOVE_MODE_ABSOLUTE,
                                         MOVE_REFERENCE_BASE, 0.f, BLENDING_SPEED_TYPE_DUPLICATE);
        emit("[INFO] WeldStraight approach movel returned " + std::string(approach_ok ? "true" : "false"));
        if (!waitForStandby()) {
            emit("[ERROR] WeldStraight approach waitForStandby failed");
            return false;
        }

        // [STEP_START] emitted here — laser fires at weld start, not during fast approach
        emit("[STEP_START] " + std::to_string(stepIdx));

        float vel2[2] = {10.f, 10.f}, acc2[2] = {10.f, 10.f};
        if (step.contains("vel")) {
            auto& vv = step["vel"];
            if (vv.is_array()) { vel2[0] = vv[0].get<float>(); vel2[1] = vv[1].get<float>(); }
            else                { vel2[0] = vel2[1] = vv.get<float>(); }
        }
        if (step.contains("acc")) {
            auto& aa = step["acc"];
            if (aa.is_array()) { acc2[0] = aa[0].get<float>(); acc2[1] = aa[1].get<float>(); }
            else                { acc2[0] = acc2[1] = aa.get<float>(); }
        }
        float time_s = step.value("time", 0.f);
        emit("[INFO] WeldStraight weld: A→B absolute base frame");
        bool weld_ok = g_robot.movel(pos_b, vel2, acc2, time_s, MOVE_MODE_ABSOLUTE,
                                     MOVE_REFERENCE_BASE, 0.f, BLENDING_SPEED_TYPE_DUPLICATE);
        emit("[INFO] WeldStraight weld movel returned " + std::string(weld_ok ? "true" : "false"));
        if (!waitForStandby()) {
            emit("[ERROR] WeldStraight weld waitForStandby failed");
            return false;
        }
        emit("[INFO] WeldStraight step " + std::to_string(stepIdx) + " complete");
        return true;
    }

    if (type == "FreeForm") {
        auto sub_steps = step.value("sub_steps", json::array());
        if (sub_steps.empty()) {
            emit("[INFO] FreeForm step " + std::to_string(stepIdx) + ": no sub-steps, skipped");
            return true;
        }
        if (sub_steps.size() > MAX_MOVEB_POINT) {
            emit("[ERROR] FreeForm: too many sub-steps (max " + std::to_string(MAX_MOVEB_POINT) + ")");
            return false;
        }

        MOVE_POSB posb[MAX_MOVEB_POINT];
        memset(posb, 0, sizeof(posb));

        float vel2[2] = {30.f, 30.f}, acc2[2] = {30.f, 30.f};
        float blend_rad = step.value("blend_radius", 50.f);

        for (size_t si = 0; si < sub_steps.size(); si++) {
            const auto& ss = sub_steps[si];
            std::string sstype = ss.value("type", "MoveL");
            bool is_last = (si == sub_steps.size() - 1);

            // Collect vel/acc from first sub-step for the whole moveb call
            if (si == 0) {
                if (ss.contains("vel")) {
                    const auto& vv = ss["vel"];
                    if (vv.is_array()) { vel2[0] = vv[0].get<float>(); vel2[1] = vv[1].get<float>(); }
                    else               vel2[0] = vel2[1] = vv.get<float>();
                }
                if (ss.contains("acc")) {
                    const auto& aa = ss["acc"];
                    if (aa.is_array()) { acc2[0] = aa[0].get<float>(); acc2[1] = aa[1].get<float>(); }
                    else               acc2[0] = acc2[1] = aa.get<float>();
                }
            }

            if (sstype == "MoveL") {
                auto arr = ss.value("pos", json::array());
                if (arr.size() < 6) {
                    emit("[ERROR] FreeForm sub-step " + std::to_string(si) + " MoveL: missing pos[6]");
                    return false;
                }
                for (int k = 0; k < NUM_TASK; k++) posb[si]._fTargetPos[0][k] = arr[k].get<float>();
                posb[si]._iBlendType = 0;  // line
                if (is_last) {
                    posb[si]._fBlendRad = 0.f;
                } else {
                    float seg_blend = ss.value("blend_radius", blend_rad);
                    posb[si]._fBlendRad = seg_blend < 0.5f ? 0.5f : seg_blend;
                }

            } else if (sstype == "MoveC") {
                if (!ss.contains("pos_via") || ss["pos_via"].is_null() ||
                    !ss.contains("pos_end") || ss["pos_end"].is_null()) {
                    emit("[ERROR] FreeForm sub-step " + std::to_string(si) + " MoveC: missing pos_via/pos_end");
                    return false;
                }
                for (int k = 0; k < NUM_TASK; k++) posb[si]._fTargetPos[0][k] = ss["pos_via"][k].get<float>();
                for (int k = 0; k < NUM_TASK; k++) posb[si]._fTargetPos[1][k] = ss["pos_end"][k].get<float>();
                posb[si]._iBlendType = 1;  // circle
                if (is_last) {
                    posb[si]._fBlendRad = 0.f;
                } else {
                    float seg_blend = ss.value("blend_radius", blend_rad);
                    posb[si]._fBlendRad = seg_blend < 0.5f ? 0.5f : seg_blend;
                }
            } else {
                emit("[ERROR] FreeForm sub-step " + std::to_string(si) + " unknown type: " + sstype);
                return false;
            }
        }

        emit("[STEP_START] " + std::to_string(stepIdx));

        bool ok = g_robot.moveb(posb, (unsigned char)sub_steps.size(), vel2, acc2, 0.f,
                                MOVE_MODE_ABSOLUTE, MOVE_REFERENCE_BASE);
        if (!ok) {
            emit("[ERROR] FreeForm moveb returned false");
            return false;
        }
        if (!waitForStandby(120000)) {
            emit("[ERROR] FreeForm moveb waitForStandby failed");
            return false;
        }

        emit("[INFO] FreeForm step " + std::to_string(stepIdx) + " complete");
        return true;
    }

    auto arr = step.value("pos", json::array());
    if (arr.size() < 6) {
        emit("[ERROR] step missing pos[6]");
        return false;
    }
    float pos[NUM_JOINT];
    for (int i = 0; i < NUM_JOINT; i++) pos[i] = arr[i].get<float>();

    if (type == "MoveJ") {
        float vel, acc, t;
        {
            std::lock_guard<std::mutex> lk(g_param_mx);
            auto vv = step.find("vel");
            vel = (vv != step.end()) ? vv->get<float>() : g_default_vel;
            auto aa = step.find("acc");
            acc = (aa != step.end()) ? aa->get<float>() : g_default_acc;
            auto tt = step.find("time");
            t   = (tt != step.end()) ? tt->get<float>() : g_default_time;
        }
        bool ok = g_robot.movej(pos, vel, acc, t, MOVE_MODE_ABSOLUTE, 0.0f,
                                BLENDING_SPEED_TYPE_DUPLICATE);
        emit("[INFO] movej returned " + std::string(ok ? "true" : "false"));
        if (!ok) return false;
        return waitForStandby();

    } else if (type == "MoveL") {
        float vel2[2], acc2[2], t;
        {
            std::lock_guard<std::mutex> lk(g_param_mx);
            auto vv = step.find("vel");
            if (vv != step.end() && vv->is_array()) {
                vel2[0] = (*vv)[0].get<float>();
                vel2[1] = (*vv)[1].get<float>();
            } else {
                float v = (vv != step.end()) ? vv->get<float>() : g_default_vel;
                vel2[0] = vel2[1] = v;
            }
            auto aa = step.find("acc");
            if (aa != step.end() && aa->is_array()) {
                acc2[0] = (*aa)[0].get<float>();
                acc2[1] = (*aa)[1].get<float>();
            } else {
                float a = (aa != step.end()) ? aa->get<float>() : g_default_acc;
                acc2[0] = acc2[1] = a;
            }
            auto tt = step.find("time");
            t = (tt != step.end()) ? tt->get<float>() : g_default_time;
        }
        g_robot.movel(pos, vel2, acc2, t, MOVE_MODE_ABSOLUTE,
                      MOVE_REFERENCE_BASE, 0.0f,
                      BLENDING_SPEED_TYPE_DUPLICATE);
        return waitForStandby();
    }

    emit("[INFO] Unknown step type: " + type + " — skipped");
    return true;
}

// ── Plan worker ───────────────────────────────────────────────────────────────

static void planWorker(json steps, bool single_pass, bool loop) {
    bool keep_going = true;
    while (keep_going && !g_cancel && !g_shutdown) {
        for (int i = 0; i < static_cast<int>(steps.size()); i++) {
            if (g_cancel || g_shutdown) break;
            // MoveC and WeldStraight emit [STEP_START] themselves after the approach
            // phase so the laser fires at arc/weld start, not during the fast approach.
            std::string stype = steps[i].value("type", "MoveJ");
            if (stype != "MoveC" && stype != "WeldStraight" && stype != "FreeForm")
                emit("[STEP_START] " + std::to_string(i));
            bool ok = execStep(steps[i], i);
            if (!ok && !g_cancel && !g_shutdown) {
                emit("[ERROR] Step " + std::to_string(i) + " failed");
                g_cancel = true;
                break;
            }
            // Per-step delay (extra pause after motion before advancing)
            float delay_s = steps[i].value("delay", 0.f);
            if (delay_s > 0.f && !g_cancel && !g_shutdown) {
                auto ms = static_cast<int>(delay_s * 1000.f);
                std::this_thread::sleep_for(std::chrono::milliseconds(ms));
            }
        }
        keep_going = loop && !single_pass;
    }

    if (g_cancel) {
        g_cancel = false;
        emit("[DONE] cancelled");
    } else {
        emit("[DONE] complete");
    }
}

// ── Command handlers ──────────────────────────────────────────────────────────

static void cmdRunPlan(const json& cmd) {
    std::lock_guard<std::mutex> lk(g_worker_mx);
    if (g_worker.joinable()) {
        emit("[ERROR] plan already running");
        return;
    }

    auto plan = cmd.value("plan", json::object());
    auto steps = plan.value("steps", json::array());
    bool single_pass = cmd.value("single_pass", false);
    bool loop        = cmd.value("loop", !single_pass);

    g_cancel = false;
    if (g_robot.get_robot_mode() != ROBOT_MODE_AUTONOMOUS)
        g_robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    g_worker = std::thread(planWorker, std::move(steps), single_pass, loop);
    g_worker.detach();
}

static void cmdStop() {
    g_cancel = true;
    try {
        g_robot.stop(STOP_TYPE_QUICK);
    } catch (...) {}
}

static void cmdRecordPoint() {
    LPROBOT_POSE      pj = g_robot.get_current_posj();
    LPROBOT_TASK_POSE px = g_robot.get_current_posx(COORDINATE_SYSTEM_BASE);
    if (!pj || !px) {
        emit("[INFO] record_point: could not read position");
        return;
    }

    std::string type;
    float vel, acc, t;
    {
        std::lock_guard<std::mutex> lk(g_param_mx);
        type = g_current_type;
        vel  = g_default_vel;
        acc  = g_default_acc;
        t    = g_default_time;
    }

    json step;
    step["type"] = type;
    if (type == "MoveJ") {
        json pos = json::array();
        for (int i = 0; i < NUM_JOINT; i++) pos.push_back(pj->_fPosition[i]);
        step["pos"] = pos;
        step["vel"] = vel;
        step["acc"] = acc;
        step["time"] = t;
    } else {
        json pos = json::array();
        for (int i = 0; i < NUM_TASK; i++) pos.push_back(px->_fTargetPos[i]);
        step["pos"] = pos;
        step["vel"] = json::array({vel, vel});
        step["acc"] = json::array({acc, acc});
        step["time"] = t;
    }
    step["enabled"] = true;

    httpPost("http://localhost:8000/api/robot/hand_guide/captured", step.dump());
    // Silent — server broadcasts [CAPTURE] from /api/robot/hand_guide/captured
}


static void cmdEnableHandGuide() {
    g_robot.set_robot_mode(ROBOT_MODE_MANUAL);
    float stx[NUM_TASK] = {3000.0f, 3000.0f, 3000.0f, 200.0f, 200.0f, 200.0f};
    g_robot.task_compliance_ctrl(stx, COORDINATE_SYSTEM_TOOL, 0.0f);
    {
        std::lock_guard<std::mutex> lk(g_btn_mx);
        for (int i = 0; i < NUM_BUTTON; i++) g_prev_bt[i] = 0;
    }
    g_hand_guide_active.store(true);
    emit("[INFO] hand guide enabled");
}

static void cmdDisableHandGuide() {
    g_hand_guide_active.store(false);
    g_robot.release_compliance_ctrl();
    g_robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    emit("[INFO] hand guide disabled");
}

static void cmdSetParam(const json& cmd) {
    std::string key   = cmd.value("key", "");
    auto        val   = cmd.find("value");
    if (val == cmd.end()) return;

    std::lock_guard<std::mutex> lk(g_param_mx);
    if (key == "current_type") {
        g_current_type = val->get<std::string>();
    } else if (key == "default_vel") {
        g_default_vel = val->get<float>();
    } else if (key == "default_acc") {
        g_default_acc = val->get<float>();
    } else if (key == "default_time") {
        g_default_time = val->get<float>();
    }
}

static void cmdCapturePose() {
    LPROBOT_TASK_POSE px = g_robot.get_current_posx(COORDINATE_SYSTEM_BASE);
    if (!px) {
        emit("[ERROR] capture_pose: could not read posx");
        return;
    }
    // Emit [POSE] [x,y,z,rx,ry,rz] — server parses and returns to client
    std::string out = "[POSE] [";
    for (int i = 0; i < NUM_TASK; i++) {
        out += std::to_string(px->_fTargetPos[i]);
        if (i < NUM_TASK - 1) out += ",";
    }
    out += "]";
    emit(out);
}

static void cmdClose() {
    g_shutdown = true;
}

static void cmdEnableJog() {
    if (g_robot.get_robot_mode() != ROBOT_MODE_MANUAL)
        g_robot.set_robot_mode(ROBOT_MODE_MANUAL);
    if (g_robot.get_robot_state() == STATE_SAFE_OFF)
        g_robot.set_robot_control(CONTROL_SERVO_ON);
    emit("[JOG_ENABLED]");
}

static void cmdDisableJog() {
    g_robot.jog(JOG_AXIS_JOINT_1, MOVE_REFERENCE_BASE, 0.0f); // ensure stopped
    if (g_robot.get_robot_mode() != ROBOT_MODE_AUTONOMOUS)
        g_robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    emit("[JOG_DISABLED]");
}

static void cmdJog(const json& cmd) {
    int   axis_int = cmd.value("axis", -1);
    int   ref_int  = cmd.value("reference", 0);
    float vel      = cmd.value("velocity", 0.0f);
    if (axis_int < 0 || axis_int > 11) {
        emit("[ERROR] jog: axis must be 0-11");
        return;
    }
    g_robot.jog(static_cast<JOG_AXIS>(axis_int),
                static_cast<MOVE_REFERENCE>(ref_int),
                vel);
}

// ── Dispatch ──────────────────────────────────────────────────────────────────

static void dispatch(const json& cmd) {
    std::string c = cmd.value("cmd", "");
    if      (c == "run_plan")           cmdRunPlan(cmd);
    else if (c == "stop")               cmdStop();
    else if (c == "jog")                cmdJog(cmd);
    else if (c == "enable_jog")         cmdEnableJog();
    else if (c == "disable_jog")        cmdDisableJog();
    else if (c == "record_point")       cmdRecordPoint();
    else if (c == "enable_hand_guide")  cmdEnableHandGuide();
    else if (c == "disable_hand_guide") cmdDisableHandGuide();
    else if (c == "set_param")          cmdSetParam(cmd);
    else if (c == "capture_pose")       cmdCapturePose();
    else if (c == "close")              cmdClose();
    else emit("[ERROR] unknown command: " + c);
}

// ── Signal handling ───────────────────────────────────────────────────────────

static void sigHandler(int) {
    g_cancel   = true;
    g_shutdown = true;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::string ip   = "192.168.0.20";
    int         port = 12345;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--ip")   && i + 1 < argc) ip   = argv[++i];
        if (!strcmp(argv[i], "--port") && i + 1 < argc) port = std::stoi(argv[++i]);
        // --drcf handled via DRCF_VERSION compile definition
    }

    std::signal(SIGINT,  sigHandler);
    std::signal(SIGTERM, sigHandler);

    curl_global_init(CURL_GLOBAL_DEFAULT);

    // Register callbacks
    g_robot.set_on_monitoring_state(onState);
    g_robot.set_on_monitoring_access_control(onAccessControl);
    g_robot.set_on_disconnected(onDisconnected);
    g_robot.set_on_log_alarm(onLogAlarm);
    g_robot.set_on_tp_log(onTpLog);
    g_robot.set_on_monitoring_safety_stop_type(onSafetyStopType);
    g_robot.set_on_monitoring_data([](const LPMONITORING_DATA) {}); // keepalive v0 (unused with version 1)
    g_robot.set_on_monitoring_data_ex([](const LPMONITORING_DATA_EX pData) {
        const unsigned int* bt = pData->_tMisc._iActualBT;
        std::lock_guard<std::mutex> lk(g_btn_mx);
        unsigned int prev1 = g_prev_bt[1];
        for (int i = 0; i < NUM_BUTTON; i++) g_prev_bt[i] = bt[i];

        // bt[1] rising edge during hand guide → signal frontend to record
        if (g_hand_guide_active.load() && prev1 == 0 && bt[1] == 1) {
            emit("[BTN_RECORD]");
        }
    });

    // Connect
    emit("[INFO] connecting to " + ip + ":" + std::to_string(port));
    if (!g_robot.open_connection(ip, port)) {
        emit("[ERROR] open_connection failed");
        curl_global_cleanup();
        return 1;
    }

    g_robot.setup_monitoring_version(1);

    // Request exclusive access
    g_robot.manage_access_control(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

    // Wait for access grant
    {
        std::unique_lock<std::mutex> lk(g_access_mx);
        bool ok = g_access_cv.wait_for(lk, std::chrono::seconds(10), [] {
            return g_access_ctrl == MONITORING_ACCESS_CONTROL_GRANT ||
                   g_shutdown.load();
        });
        if (!ok || g_shutdown) {
            emit(ok ? "[ERROR] access control denied or lost during handshake"
                    : "[ERROR] access control grant timeout");
            g_robot.close_connection();
            curl_global_cleanup();
            return 1;
        }
    }

    // Servo on
    g_robot.set_robot_control(CONTROL_SERVO_ON);

    // Wait for STANDBY
    {
        std::unique_lock<std::mutex> lk(g_state_mx);
        bool ok = g_state_cv.wait_for(lk, std::chrono::seconds(15), [] {
            return g_robot_state == STATE_STANDBY || g_shutdown.load();
        });
        if (!ok || g_shutdown) {
            emit("[ERROR] robot did not reach STANDBY");
            g_robot.close_connection();
            curl_global_cleanup();
            return 1;
        }
    }

    SYSTEM_VERSION sv{};
    if (g_robot.get_system_version(&sv))
        emit(std::string("[INFO] Controller: ") + sv._szController +
             "  Library: " + g_robot.get_library_version());

    g_robot.set_robot_system(ROBOT_SYSTEM_REAL);
    g_robot.set_robot_mode(ROBOT_MODE_AUTONOMOUS);
    g_connected = true;
    emit("[CONNECTED]");

    // Keepalive thread — check_motion() sends on command socket every 60s to prevent
    // controller's ~5min idle timeout from dropping the command channel.
    // get_robot_state() reads local monitoring cache only (no network traffic).
    std::thread keepalive([] {
        while (!g_shutdown) {
            for (int i = 0; i < 60 && !g_shutdown; i++)
                std::this_thread::sleep_for(std::chrono::seconds(1));
            if (!g_shutdown) g_robot.check_motion();
        }
    });
    keepalive.detach();

    // Command loop
    std::string line;
    while (!g_shutdown && std::getline(std::cin, line)) {
        if (line.empty()) continue;
        try {
            auto cmd = json::parse(line);
            dispatch(cmd);
        } catch (const json::exception& e) {
            emit(std::string("[ERROR] JSON parse: ") + e.what());
        }
    }

    // Graceful shutdown
    g_cancel = true;
    g_hand_guide_active.store(false);
    {
        std::lock_guard<std::mutex> lk(g_worker_mx);
        // worker detached — MoveStop already sent by cmdStop() if called
    }
    try {
        g_robot.stop(STOP_TYPE_QUICK);
    } catch (...) {}
    g_robot.close_connection();
    curl_global_cleanup();
    return 0;
}
