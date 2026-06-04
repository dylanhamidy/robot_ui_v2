#!/usr/bin/env python3
"""
Run any plan JSON file directly through the DRFL daemon (no UI needed).

Usage:
  python3 run_plan.py <plan.json> [--ip 192.168.0.20] [--port 12345]
"""
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

DAEMON = Path(__file__).parent / "lux_drfl_daemon" / "build" / "drfl_daemon"
IP     = "192.168.0.20"
PORT   = "12345"

args = sys.argv[1:]
plan_file = None
i = 0
while i < len(args):
    if args[i] == "--ip"   and i + 1 < len(args): IP   = args[i + 1]; i += 2
    elif args[i] == "--port" and i + 1 < len(args): PORT = args[i + 1]; i += 2
    else: plan_file = args[i]; i += 1

if not plan_file:
    print("Usage: python3 run_plan.py <plan.json> [--ip IP] [--port PORT]")
    sys.exit(1)

plan_path = Path(plan_file)
if not plan_path.exists():
    print(f"File not found: {plan_file}")
    sys.exit(1)

with open(plan_path) as f:
    plan_data = json.load(f)

cmd = {
    "cmd": "run_plan",
    "plan": plan_data,
    "single_pass": True,
    "loop": False,
}

print(f"Plan:   {plan_path.name}  ({len(plan_data.get('steps', []))} steps)")
print(f"Daemon: {DAEMON} --ip {IP} --port {PORT}")
print()

proc = subprocess.Popen(
    [str(DAEMON), "--ip", IP, "--port", PORT],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    preexec_fn=os.setsid,
)

def send(obj: dict):
    proc.stdin.write((json.dumps(obj) + "\n").encode())
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
            print(f"\n--- Sending plan: {plan_path.name} ---")
            send(cmd)

        elif "[DONE]" in text and connected:
            print("\n--- Plan finished, disconnecting ---")
            send({"cmd": "close"})

        elif "[DISCONNECTED]" in text:
            break

        elif "[ERROR]" in text and not connected:
            print("\nConnection failed.")
            break

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
