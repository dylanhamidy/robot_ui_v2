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

// Captured points accumulator (hand guide)
struct CapturedStep {
    std::string type;
    float posj[NUM_JOINT];
    float posx[NUM_TASK];
    float vel;
    float acc;
    float time_s;
};
static std::mutex              g_points_mx;
static std::vector<CapturedStep> g_points;
static std::string             g_plan_name;  // set by save_plan / set_param

// ── Callbacks ─────────────────────────────────────────────────────────────────

static void onState(const ROBOT_STATE state) {
    {
        std::lock_guard<std::mutex> lk(g_state_mx);
        g_robot_state = state;
    }
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

    // Wait for MOVING first (motion dispatched)
    {
        std::unique_lock<std::mutex> lk(g_state_mx);
        g_state_cv.wait_until(lk, deadline, [] {
            return g_robot_state == STATE_MOVING ||
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

static bool execStep(const json& step) {
    std::string type = step.value("type", "MoveJ");

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
            emit("[STEP_START] " + std::to_string(i));
            bool ok = execStep(steps[i]);
            if (!ok && !g_cancel && !g_shutdown) {
                emit("[ERROR] Step " + std::to_string(i) + " failed");
                g_cancel = true;
                break;
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
    g_worker = std::thread(planWorker, std::move(steps), single_pass, loop);
    g_worker.detach();
}

static void cmdStop() {
    g_cancel = true;
    try {
        g_robot.MoveStop(STOP_TYPE_QUICK);
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

    CapturedStep s;
    s.type  = type;
    s.vel   = vel;
    s.acc   = acc;
    s.time_s = t;
    for (int i = 0; i < NUM_JOINT; i++) s.posj[i] = pj->_fPosition[i];
    for (int i = 0; i < NUM_TASK;  i++) s.posx[i] = px->_fTargetPos[i];

    {
        std::lock_guard<std::mutex> lk(g_points_mx);
        g_points.push_back(s);
    }

    // Build step JSON matching server's expected shape
    json step;
    step["type"] = type;
    if (type == "MoveJ") {
        json pos = json::array();
        for (int i = 0; i < NUM_JOINT; i++) pos.push_back(s.posj[i]);
        step["pos"] = pos;
        step["vel"] = vel;
        step["acc"] = acc;
        step["time"] = t;
    } else {
        json pos = json::array();
        for (int i = 0; i < NUM_TASK; i++) pos.push_back(s.posx[i]);
        step["pos"] = pos;
        step["vel"] = json::array({vel, vel});
        step["acc"] = json::array({acc, acc});
        step["time"] = t;
    }
    step["enabled"] = true;

    httpPost("http://localhost:8000/api/robot/hand_guide/captured", step.dump());
    // Silent — server broadcasts [CAPTURE] from /api/robot/hand_guide/captured
}

static void cmdClearPlan() {
    std::lock_guard<std::mutex> lk(g_points_mx);
    g_points.clear();
}

static void cmdSavePlan() {
    std::vector<CapturedStep> pts;
    {
        std::lock_guard<std::mutex> lk(g_points_mx);
        pts = g_points;
    }

    // Auto-generate name from timestamp
    std::time_t now = std::time(nullptr);
    char tbuf[32];
    std::strftime(tbuf, sizeof(tbuf), "plan_%Y%m%d_%H%M%S", std::localtime(&now));
    std::string name = g_plan_name.empty() ? tbuf : g_plan_name;

    json steps_arr = json::array();
    for (auto& s : pts) {
        json step;
        step["type"] = s.type;
        step["enabled"] = true;
        if (s.type == "MoveJ") {
            json pos = json::array();
            for (int i = 0; i < NUM_JOINT; i++) pos.push_back(s.posj[i]);
            step["pos"] = pos;
            step["vel"] = s.vel;
            step["acc"] = s.acc;
            step["time"] = s.time_s;
        } else {
            json pos = json::array();
            for (int i = 0; i < NUM_TASK; i++) pos.push_back(s.posx[i]);
            step["pos"] = pos;
            step["vel"] = json::array({s.vel, s.vel});
            step["acc"] = json::array({s.acc, s.acc});
            step["time"] = s.time_s;
        }
        steps_arr.push_back(step);
    }

    json plan;
    plan["name"]  = name;
    plan["steps"] = steps_arr;

    bool ok = httpPost("http://localhost:8000/api/plans/import", plan.dump());
    if (ok) {
        emit("[INFO] plan saved: " + name);
    } else {
        emit("[ERROR] save_plan: HTTP POST failed");
    }
}

static void cmdEnableHandGuide() {
    g_robot.set_robot_mode(ROBOT_MODE_MANUAL);
    float stx[NUM_TASK] = {3000.0f, 3000.0f, 3000.0f, 200.0f, 200.0f, 200.0f};
    g_robot.task_compliance_ctrl(stx, COORDINATE_SYSTEM_TOOL, 0.0f);
    emit("[INFO] hand guide enabled");
}

static void cmdDisableHandGuide() {
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
    } else if (key == "plan_name") {
        g_plan_name = val->get<std::string>();
    }
}

static void cmdClose() {
    g_shutdown = true;
}

// ── Dispatch ──────────────────────────────────────────────────────────────────

static void dispatch(const json& cmd) {
    std::string c = cmd.value("cmd", "");
    if      (c == "run_plan")           cmdRunPlan(cmd);
    else if (c == "stop")               cmdStop();
    else if (c == "record_point")       cmdRecordPoint();
    else if (c == "clear_plan")         cmdClearPlan();
    else if (c == "save_plan")          cmdSavePlan();
    else if (c == "enable_hand_guide")  cmdEnableHandGuide();
    else if (c == "disable_hand_guide") cmdDisableHandGuide();
    else if (c == "set_param")          cmdSetParam(cmd);
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

    // Connect
    emit("[INFO] connecting to " + ip + ":" + std::to_string(port));
    if (!g_robot.open_connection(ip, port)) {
        emit("[ERROR] open_connection failed");
        curl_global_cleanup();
        return 1;
    }

    g_robot.setup_monitoring_version(1);

    // Request exclusive access
    g_robot.ManageAccessControl(MANAGE_ACCESS_CONTROL_FORCE_REQUEST);

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
    {
        std::lock_guard<std::mutex> lk(g_worker_mx);
        // worker detached — MoveStop already sent by cmdStop() if called
    }
    try {
        g_robot.MoveStop(STOP_TYPE_QUICK);
    } catch (...) {}
    g_robot.close_connection();
    curl_global_cleanup();
    return 0;
}
