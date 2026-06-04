#!/usr/bin/env python3
"""
Smoke test: connect daemon, do a small MoveJ and back, then disconnect.
Positions from the official Doosan minimal_motion_sample example.

Usage: python3 smoke_test.py [--ip 192.168.0.20] [--port 12345]
"""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

DAEMON = Path(__file__).parent / "lux_drfl_daemon" / "build" / "drfl_daemon"
IP   = "192.168.0.20"
PORT = "12345"

for i, arg in enumerate(sys.argv[1:]):
    if arg == "--ip"   and i + 2 < len(sys.argv): IP   = sys.argv[i + 2]
    if arg == "--port" and i + 2 < len(sys.argv): PORT = sys.argv[i + 2]

PLAN = {
    "cmd": "run_plan",
    "plan": {
        "name": "smoke_test",
        "steps": [
            # Joint 3 up 30 deg — same as the official example
            {"type": "MoveJ", "pos": [0.0, 0.0, 30.0, 0.0, 0.0, 0.0],
             "vel": 30.0, "acc": 30.0, "time": 0.0, "enabled": True},
            # Back to home
            {"type": "MoveJ", "pos": [0.0, 0.0,  0.0, 0.0, 0.0, 0.0],
             "vel": 30.0, "acc": 30.0, "time": 0.0, "enabled": True},
        ],
    },
    "single_pass": True,
    "loop": False,
}

print(f"Starting daemon: {DAEMON} --ip {IP} --port {PORT}")
proc = subprocess.Popen(
    [str(DAEMON), "--ip", IP, "--port", PORT],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid,
)

def send(cmd: dict):
    line = json.dumps(cmd) + "\n"
    proc.stdin.write(line.encode())
    proc.stdin.flush()

connected = False
try:
    while True:
        raw = proc.stdout.readline()
        if not raw:
            print("Daemon exited unexpectedly.")
            break
        text = raw.decode(errors="replace").rstrip()
        print(text)

        if "[CONNECTED]" in text:
            connected = True
            print("\n--- Sending smoke-test plan ---")
            send(PLAN)

        elif "[DONE]" in text and connected:
            print("\n--- Smoke test complete, disconnecting ---")
            send({"cmd": "close"})

        elif "[DISCONNECTED]" in text:
            break

        elif "[ERROR]" in text:
            if not connected:
                print("\nConnection failed — see error above.")
                break
            # post-connection error: daemon will emit [DISCONNECTED] next; just wait for it

except KeyboardInterrupt:
    print("\nInterrupted — stopping robot.")
    send({"cmd": "stop"})
    time.sleep(0.5)
    send({"cmd": "close"})
finally:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=3)
    except Exception:
        pass
