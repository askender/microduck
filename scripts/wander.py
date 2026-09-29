#!/usr/bin/env python3
"""Wander: keep a simulated duck walking in randomly changing directions, forever.

How it stops is the point: this holds no state and owns nothing. It only keeps
sending `robot.move` intents at 10 Hz — the daemon's deadman expires an intent
500 ms after the last one, so the moment this exits (Ctrl-C, SIGTERM, a dropped
connection) the duck stops and stands. Walking forever is a client that never
shuts up, nothing more.

Each leg picks a fresh random twist inside the walking policy's trained command
domain (microduck_rl velocity env: vx ±0.4, vy ±0.3, vyaw ±1.0), deliberately
below the boundaries: the full range is what training sampled, not what a
direction change every few seconds is smooth in. Direction changes are eased by
robotd's own command slew (`cmd += α(target − cmd)` per tick), not here.

Usage: wander.py [SOCKET] [MIN_HOLD MAX_HOLD]   # hold seconds per direction, default 3 6
"""
import json
import math
import os
import random
import signal
import socket
import sys
import time

SOCK = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/.cache/duck-sim/duck.sock")
MIN_HOLD = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
MAX_HOLD = float(sys.argv[3]) if len(sys.argv) > 3 else 6.0
RATE_HZ = 10.0  # the deadman is 500 ms; 10 Hz is what scripts/duck-sim `drive` sends

# Speed band from a clean measurement (2026-09-27, notifications fixed, no
# other driver): ≤0.2 m/s the duck only creeps and stands; ~0.3 m/s entrains
# a gait reliably; 0.4 m/s walks then collapses. Legs are 0.28–0.35 m/s with
# small yaw — inside the trained domain (vx ±0.4, vy ±0.3, vyaw ±1.0) but in
# the band that actually walks on the published Hub policy.
SPEED_RANGE = (0.28, 0.35)
YAW_RANGE = (-0.3, 0.3)

stop = False


def bye(_sig, _frame):
    global stop
    stop = True


signal.signal(signal.SIGTERM, bye)
signal.signal(signal.SIGINT, bye)

sock = socket.socket(socket.AF_UNIX)
sock.connect(SOCK)
out = sock.makefile("w")

print(f"wander: talking to {SOCK}, a new direction every {MIN_HOLD}-{MAX_HOLD:.0f} s", flush=True)
try:
    while not stop:
        heading = random.uniform(0.0, 2.0 * math.pi)
        speed = random.uniform(*SPEED_RANGE)
        vx, vy = speed * math.cos(heading), speed * math.sin(heading)
        vyaw = random.uniform(*YAW_RANGE)
        hold = random.uniform(MIN_HOLD, MAX_HOLD)
        print(f"wander: vx={vx:+.2f} vy={vy:+.2f} vyaw={vyaw:+.2f} for {hold:.1f} s", flush=True)
        deadline = time.monotonic() + hold
        while not stop and time.monotonic() < deadline:
            out.write(json.dumps({"jsonrpc": "2.0", "method": "robot.move",
                                  "params": {"vx": vx, "vy": vy, "vyaw": vyaw}}) + "\n")
            out.flush()
            time.sleep(1.0 / RATE_HZ)
except BrokenPipeError:
    print("wander: the daemon closed the connection — exiting", flush=True)

print("wander: stopped — the intent expires on its own and the duck stands", flush=True)
