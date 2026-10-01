#!/usr/bin/env python3
"""Live top-down plan view of the duck sim, in a browser.

A mirror, not a second sim: this process polls the body server's `read` op at
~12 Hz, rebuilds the same MJCF model's qpos from the reply (trunk xyz + trunk
quat + the fourteen actuated joints), renders one ORTHOGRAPHIC top-down frame
per poll, and serves them as an MJPEG stream. Open http://localhost:8910/
while the sim runs.

Run this under `mjpython`, not plain python: on macOS the offscreen renderer
only produces correct, deterministic output when a GL sharegroup exists —
plain python's CGL offscreen context drops world geoms and moves objects
between frames. Verified 2026-09-27 with calibration spheres at known world
positions; azimuth 90 is the map convention (+x right, +y up).

Camera modes: the default view is the locked 仙剑一-style oblique, eased
onto the duck (user preference, 2026-10-01); the page buttons or
`?follow=0` / `/stream?follow=0` switch to the origin-pinned top view and
`?follow=1` back. The yellow home disc is a 2D overlay,
so it stays correct in both modes.

Endpoints: `/` serves a small control page — sidebar buttons for every
control that already exists (pause/resume rendering, camera follow on/off,
walk forward 3 s, sit/stand toggle, walk home, the one-shot skills, health),
plus IJKL keyboard drive and a yaw slider; the stream fills the viewport
beside them, no scrolling.
`/stream` is the MJPEG feed (the follow flag lives here); `/toggle` flips
rendering on and off; `/status` reports state as JSON, including the duck's
OWN pose estimate (a 2 Hz `robot.subscribe` thread) beside the ground truth
so the drift is visible; `/ctl/follow|view|drive|sit|home|skill|health` are what
the buttons press — `/ctl/skill?name=` runs a `robot.do` one-shot (roulade,
kick_left, kick_right, ground_pick — whitelisted against the live skill
list, probed once from a bogus name's refusal), and `/ctl/health` shells out
to `robotctl health` with the sim's sockets; `/ctl/keys?vx=&vy=&vyaw=` carries the
held-key state — a thread retransmits it at 10 Hz and goes idle 0.7 s after
updates stop (robotd's 500 ms deadman then stops the duck). `/ctl/head` sends
`robot.head`: four head-joint deltas from the home pose, radians on the wire,
degrees in the query (neck/pitch clamp ±1.1, yaw ±1.4, roll ±0.31 — the
trained sampling ranges; beyond them tracking is untrained). The head intent
has NO deadman — one call holds until the next — and the walking policy tracks
it as part of its observation, so the head moves while walking. Tracking is a
learned posture, not a servo write: yaw is tight (±0.05), pitch/neck loose and
cross-coupled (measured 2026-10-01, /tmp/head_test.py). The key values
are the measured gaits: forward/reverse cruises (see below), J/L spin in
place at vyaw ±2 (~60 deg/s, radius
~0.01 m — the angular channel is dead below |vyaw| ~1.5, measured 2026-09-28,
/tmp/turn_sweep.py); held with I/K they become arcs (I+J walks 0.20 m/s at
R ≈ 0.2 m). A tap is one turning step, ~25-30 deg — the floor however brief
the tap — and Shift+J/L are fine taps at ±1.5 (~10 deg each, occasionally
not entraining at the threshold edge; tap again). Up/Down nudge the cruise
speed in 0.05 steps — the forward cruise (the vx W sends, [0.3, 1.2],
default 0.5) normally, the reverse cruise (|vx| S sends, [0.4, 1.2],
default 0.8) while S is held; key-repeat ramps, and a change lands
mid-drive. Range ends are the measured envelope (2026-09-30,
/tmp/envelope_sweep.py + range_extend2.py + rev_floor.py): commanded
forward 0.1/0.2 do not walk at all (the gait engages only from 0.3); 1.2
is the dependable ceiling (survives repeats, v_eff 0.53-0.56 m/s), 1.3 is
a coin-flip (fell 1/2 standing start, and ramping 1.2 -> 1.3 mid-drive
gave only 0.15 m/s), 1.4 falls (2/2 standing start; ramped 0.42 m/s but
with falls at stop), 1.5 lands face-down (roulade rights it). Reverse
engages at 0.4 (-0.17 m/s, stops clean x2 — 0.2/0.3 stand still); 1.2 is
its best (-0.46 m/s, dyaw -7 deg over 6 s); from 1.5 the gait collapses
to yawing in place and 2.5 falls. Uncompensated, forward speeds arc right —
worst ~16 deg/s around commanded 0.7, near-straight again at 1.2. X toggles
直线 (straight) compensation, default on: a per-speed counter-vyaw added
whenever W is held with no steering input — +0.18/+0.22/+0.44 at
vx 0.3/0.4/0.5 (residual within ±3 deg/s, lateral deviation 2-6% of
distance vs 14-25% uncompensated) and +0.30..+0.10 over 0.6..1.2 (residual
within ±5 deg/s, single-run anchors, /tmp/comp_high.py) — linear between
anchors, so the COMP table covers every 0.05 step in [0.3, 1.2]. Reverse
gets no compensation (uncalibrated). Small vyaw while walking steers
smoothly at ~40 deg/s per 1.0 vyaw — the turn dead zone below |vyaw| ~1.5
applies only to pure-spin commands (measured 2026-09-30,
/tmp/speed_test.py, straight_recipe.py, straight_fine.py,
straight_recheck.py). Raw strafe is not offered:
the v5 velstand policy was trained with the vy command slot hard-zeroed
(protective_fall, exported 2026-09-14), so in-range vy (±0.3) moves nothing
and over-range vy drifts and yaws (/tmp/vy_sweep.py, 2026-09-30); the RL
repo's current develop trains vy ±0.3, so a policy re-exported from it
would strafe — wire <- -> then. U/O is a steering wheel: vyaw ramps up
0.25 per 100 ms while
held (full lock ±2 in 0.8 s) and springs back four times faster on release.
WASD is a screen-relative compass (added 2026-10-01): hold one and the page
turns the duck to face that SCREEN direction (up/left/down/right, remapped
per view — world yaw = camera azimuth +0/-90/180/+90; az 90 top, 45
oblique) and then cruises forward with a proportional heading hold,
re-aiming from ground truth every 150 ms. IJKL stay body-frame.
The slider sets the same vyaw directly and mirrors the wheel. Paused, the mirror
thread idles and the last frame keeps being served — the picture freezes
instead of going black, stamped PAUSED. Same when nobody is watching: the
mirror renders only while a browser is actually connected to the stream, and
that render + JPEG encode is essentially all of this process's CPU.

Deliberately a client of the public `read` op rather than a patch to
body_server: the simulator's loop is untouched, and the only cost is one
offscreen 960x600 render per frame in this process. The yellow home disc shows
up because the scene XML is the same one the sim loaded (pass DUCK_SIM_SCENE
through if it is not in the environment).

    DUCK_SIM_BODY_PORT        body server port        (default 7801)
    DUCK_SIM_SCENE            scene XML the sim uses  (default RL repo scene.xml)
    DUCK_SIM_PLANVIEW_PORT    this HTTP port          (default 8910)
"""
import io
import json
import math
import os
import socket
import subprocess
import threading
import time
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import mujoco
from PIL import Image


def yaw_degrees(q):
    """Trunk yaw in degrees from an IMU quat (w, x, y, z)."""
    return math.degrees(math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                                   1 - 2 * (q[2] * q[2] + q[3] * q[3])))

BODY_PORT = int(os.environ.get("DUCK_SIM_BODY_PORT", "7801"))
SCENE = os.environ.get(
    "DUCK_SIM_SCENE",
    os.path.expanduser(
        "~/Pollen/microduck_rl/src/mjlab_microduck/robot/microduck/scene.xml"
    ),
)
HTTP_PORT = int(os.environ.get("DUCK_SIM_PLANVIEW_PORT", "8910"))
FPS = 12
WIDTH = 960   # wide landscape — matches a browser window better than a square
HEIGHT = 600

# This file lives in the sim's state dir, next to the duck's IPC sockets.
STATE_DIR = os.path.dirname(os.path.abspath(__file__))
DUCK_SOCK = os.path.join(STATE_DIR, "duck-a.sock")
if not os.path.exists(DUCK_SOCK):
    DUCK_SOCK = os.path.join(STATE_DIR, "duck.sock")
# Passed by scripts/duck-sim's planview() — needed to shell out to `home`.
REPO = os.environ.get("DUCK_SIM_REPO", "")

# Offscreen ortho vertical extent in metres per unit of camera distance
# (= 2*tan(45°/2)); verified against the GL-rendered home_disc, whose radius
# is exactly half a 0.40 m checker cell. The on-screen GL window is ~2.23x
# wider at the same distance — do not copy figures across the two.
ORTHO_FACTOR = 0.82843

# Protocol constant, duplicated from body_server.py (which duplicates
# duck_ipc_proto::JOINT_NAMES) — the wire arrays are indexed by this order.
JOINT_NAMES = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)

model = None
data = None
renderer = None
camera = None
trunk_qpos = 0
joint_qpos = []
gl_ready = threading.Event()


def init_gl() -> None:
    """Build the mirror model and renderer — called on the mirror thread.

    GL contexts are thread-affine: everything that touches mujoco rendering
    must happen on ONE thread. Module level only parses arguments; the model,
    renderer and every render live in mirror_loop's thread.
    """
    global model, data, renderer, camera, trunk_qpos, joint_qpos
    model = mujoco.MjModel.from_xml_path(SCENE)
    # The renderer caps at the model's offscreen framebuffer; the stock
    # scenes ship the 640x480 default.
    model.vis.global_.offwidth = WIDTH
    model.vis.global_.offheight = HEIGHT
    data = mujoco.MjData(model)

    trunk_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
    trunk_qpos = model.jnt_qposadr[trunk_joint]

    # Actuator name -> qpos address, in JOINT_NAMES order, skipping the mouth
    # (no actuator — "fifteen joints out here, fourteen in the model").
    for name in JOINT_NAMES:
        act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if act < 0:
            continue
        joint = model.actuator_trnid[act, 0]
        joint_qpos.append((JOINT_NAMES.index(name), model.jnt_qposadr[joint]))

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.fixedcamid = -1
    camera.lookat[:] = [0.0, 0.0, 0.0]
    camera.elevation = -90.0
    camera.orthographic = True
    # Calibrated 2026-09-27 with calibration spheres in a GL window: azimuth 90
    # over the origin is the map convention — world +x renders to the right,
    # world +y up, axes orthogonal, chirality preserved.
    # Scale (corrected 2026-09-28): the OFFSCREEN ortho camera covers
    # 0.828*distance metres vertically (= 2*tan(45°/2)), NOT the 4.9 m the
    # on-screen GL window shows at the same distance — the old figure came
    # from the window and was wrong here, which is why the home-disc overlay
    # used to be drawn at less than half the GL-rendered disc's size.
    # In this MuJoCo (3.10) MjvCamera exposes no fovy; distance IS the zoom.
    camera.azimuth = 90.0
    camera.distance = 2.2

    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    gl_ready.set()

frame_lock = threading.Lock()
latest_jpeg = None
viewers = 0  # connected MJPEG clients; the mirror renders only for these

# Camera modes. The default frame follows the duck (user preference,
# 2026-10-01); `?follow=0` in the stream URL pins the camera to the world
# origin instead (`?follow=1` — or no flag — follows). A per-tab choice
# would be nicer, but one global flag keeps the mirror trivial — the last
# connected viewer wins.
follow_duck = True
lookat_xy = [0.0, 0.0]
FOLLOW_EASE = 0.15  # per frame at FPS — ~0.5 s time constant
# oblique view (default since 2026-10-01, user preference): the diagonal
# (yaw 45°, pitched down ~50° in the 仙剑一 style — steeper than the
# 35.264° true isometric), same orthographic projection, rotation locked
# like the top view. follow_duck still chooses what the lookat tracks.
iso_view = True

# Rendering on/off, global like the camera mode (one renderer, last viewer
# wins). Paused: the mirror loop idles and the last JPEG keeps being served.
paused = False
frame_count = 0
last_pose = None  # (x, y, yaw_deg) from the most recent mirror reading


def duck_rpc(obj, wait=False, timeout=3.0):
    """One JSON-RPC request/notification on the duck's IPC socket."""
    s = socket.socket(socket.AF_UNIX)
    s.settimeout(timeout)
    s.connect(DUCK_SOCK)
    f = s.makefile("rw")
    f.write(json.dumps(obj) + "\n")
    f.flush()
    reply = f.readline() if wait else None
    s.close()
    return reply


def drive_leg():
    """The 'walk forward' button: 3 s at vx 0.3, the dependable slow gait."""
    t0 = time.time()
    while time.time() - t0 < 3.0:
        try:
            duck_rpc({"jsonrpc": "2.0", "method": "robot.move",
                      "params": {"vx": 0.3, "vy": 0.0, "vyaw": 0.0}})
        except OSError as error:
            print(f"planview: drive failed: {error}", flush=True)
            return
        time.sleep(0.1)
    print("planview: drive leg done", flush=True)


home_proc = None


# Held-key drive state. The page reports what is held (IJKL + the yaw
# slider) and a thread here retransmits robot.move at 10 Hz for as long as
# updates stay fresh — robotd's 500 ms deadman drops motion after the last
# command, so updates going quiet IS the stop signal. Values come from the
# page (cruises and steering live there); arcs need some vx.
kbd_lock = threading.Lock()
kbd_state = {"vx": 0.0, "vy": 0.0, "vyaw": 0.0, "ts": 0.0}
KBD_FRESH = 0.7  # s without an update before the emitter goes idle


def kbd_emitter() -> None:
    while True:
        with kbd_lock:
            st = dict(kbd_state)
        if time.time() - st["ts"] < KBD_FRESH and (st["vx"] or st["vy"] or st["vyaw"]):
            try:
                duck_rpc({"jsonrpc": "2.0", "method": "robot.move",
                          "params": {"vx": st["vx"], "vy": st["vy"], "vyaw": st["vyaw"]}})
            except OSError as error:
                print(f"planview: key drive failed: {error}", flush=True)
        time.sleep(0.1)


threading.Thread(target=kbd_emitter, daemon=True).start()

# The duck's own pose estimate, for the status line: a persistent
# robot.subscribe stream at 2 Hz. robotd's contact-based odometry is all the
# robot itself knows; showing it beside the body server's ground truth makes
# the drift visible while driving. Reconnects when robotd does.
odom_lock = threading.Lock()
odom_pose = None  # [x, y, yaw_deg]
brain_policy = None  # robotd's own word for the tick: walk / stand / held
loop_hz = None  # control loop's achieved rate, from the same frames
state_ts = 0.0  # when the last robot.state arrived — its staleness is the display's


def odom_loop() -> None:
    global odom_pose, brain_policy, loop_hz, state_ts
    while True:
        try:
            s = socket.socket(socket.AF_UNIX); s.settimeout(5); s.connect(DUCK_SOCK)
            f = s.makefile("rw")
            f.write(json.dumps({"jsonrpc": "2.0", "id": 9, "method": "robot.subscribe",
                                "params": {"hz": 2}}) + "\n"); f.flush()
            f.readline()  # the ack names policies, not state
            for line in f:
                m = json.loads(line)
                if m.get("method") == "robot.state":
                    st = m["params"]
                    od = st["odom"]
                    with odom_lock:
                        odom_pose = [od["position"][0], od["position"][1],
                                     math.degrees(od["yaw"])]
                        brain_policy = st.get("policy")
                        loop_hz = (st.get("loop") or {}).get("hz")
                        state_ts = time.time()
        except (OSError, ValueError):
            pass
        time.sleep(3.0)  # robotd away or restarted; try again


threading.Thread(target=odom_loop, daemon=True).start()

# The one-shot skills this robot's `robot.do` answers to, probed once: a
# bogus name's refusal lists them. Skill buttons are whitelisted against it.
skills_cache = None


def skill_list():
    global skills_cache
    if skills_cache is None:
        try:
            reply = duck_rpc({"jsonrpc": "2.0", "id": 8, "method": "robot.do",
                              "params": {"skill": "?list"}}, wait=True)
            reason = json.loads(reply)["result"]["reason"]
            at = reason.find("this robot has ")
            if at < 0:
                raise ValueError(reason)
            skills_cache = [n.strip() for n in reason[at + 15:].split(",")]
        except (OSError, ValueError, KeyError) as error:
            print(f"planview: skill probe failed ({error})", flush=True)
            skills_cache = ["sit_toggle", "roulade", "kick_left", "kick_right",
                            "ground_pick"]
    return skills_cache



def start_home():
    """The 'walk home' button: the controller lives in scripts/duck-sim."""
    global home_proc
    if not REPO:
        return False, "DUCK_SIM_REPO not set — restart planview via scripts/duck-sim"
    if home_proc is not None and home_proc.poll() is None:
        return False, "home is already running"
    home_proc = subprocess.Popen([os.path.join(REPO, "scripts", "duck-sim"), "home"],
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return True, "home started"


def ctl_health() -> str:
    """`scripts/duck-sim ctl health`, as text — robotctl with the sim's sockets."""
    if not REPO:
        return "DUCK_SIM_REPO not set — restart planview via scripts/duck-sim"
    stem = os.path.basename(DUCK_SOCK)[:-5]  # duck-a.sock -> duck-a
    try:
        out = subprocess.run(
            [os.path.join(REPO, "target", "debug", "robotctl"),
             "--robot-socket", DUCK_SOCK,
             "--tof-socket", os.path.join(STATE_DIR, stem + "-tof.sock"),
             "--config-socket", os.path.join(STATE_DIR, stem + "-config.sock"),
             "--socket", os.path.join(STATE_DIR, stem + "-updater.sock"),
             "health"], capture_output=True, text=True, timeout=10)
        return out.stdout + out.stderr
    except (OSError, subprocess.TimeoutExpired) as error:
        return f"health failed: {error}"


def render_frame(reading: dict) -> None:
    global latest_jpeg, frame_count, last_pose
    data.qpos[trunk_qpos : trunk_qpos + 3] = reading["trunk"]
    q = reading["imu"]["quat"]  # w, x, y, z — same order as qpos
    data.qpos[trunk_qpos + 3 : trunk_qpos + 7] = q
    positions = reading["positions"]
    for wire_index, adr in joint_qpos:
        data.qpos[adr] = positions[wire_index]
    mujoco.mj_forward(model, data)
    last_pose = (reading["trunk"][0], reading["trunk"][1], yaw_degrees(q))

    target = reading["trunk"][0:2] if follow_duck else (0.0, 0.0)
    lookat_xy[0] += FOLLOW_EASE * (target[0] - lookat_xy[0])
    lookat_xy[1] += FOLLOW_EASE * (target[1] - lookat_xy[1])
    camera.lookat[0] = lookat_xy[0]
    camera.lookat[1] = lookat_xy[1]

    # Both views are rotation-locked fixed poses; only the lookat eases.
    # The oblique view is pitched to -50 (negative-from-above in MuJoCo) —
    # chosen from a 35.264/45/50/55 sweep to match 仙剑一's high diagonal
    # look while keeping the duck reading as three-dimensional.
    camera.azimuth, camera.elevation = (
        (45.0, -50.0) if iso_view else (90.0, -90.0))

    renderer.update_scene(data, camera=camera)
    rgb = renderer.render()
    img = Image.fromarray(rgb)
    # --- Home-disc 2D overlay: TEMPORARILY DISABLED 2026-09-28 — the user
    # --- wants only the world geom (the GL-rendered home_disc) for now.
    # --- Re-enable by uncommenting the block below.
    # # The home marker as a 2D overlay. The macOS offscreen GL renderer
    # # drops world geoms unpredictably over long runs (the floor survives,
    # # the disc does not) — drawing it here guarantees the yellow circle is
    # # always there.
    # # Projection: world +x is screen right, +y is screen up; the offscreen
    # # ortho camera covers ORTHO_FACTOR*distance metres vertically, so a
    # # world point p lands at
    # #   (WIDTH/2 + (px - lx) * s, HEIGHT/2 - (py - ly) * s),  s = WIDTH / extent.
    # extent_m = ORTHO_FACTOR * camera.distance
    # s = WIDTH / extent_m
    # overlay = Image.new("RGBA", img.size, (0, 0, 0, 0))
    # from PIL import ImageDraw
    # draw = ImageDraw.Draw(overlay)
    # r = 0.2 * s
    # cx = WIDTH / 2 - lookat_xy[0] * s
    # cy = HEIGHT / 2 + lookat_xy[1] * s
    # # Alpha 110 over the light checker cells read as "not rendered" — go
    # # nearly opaque and ring it so it survives both checker colours.
    # draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=(255, 200, 0, 190),
    #              outline=(60, 48, 0, 255), width=3)
    # dot = 3
    # draw.ellipse([cx - dot, cy - dot, cx + dot, cy + dot], fill=(60, 48, 0, 255))
    # img = Image.alpha_composite(img.convert("RGBA"), overlay).convert("RGB")
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=82)
    with frame_lock:
        latest_jpeg = buf.getvalue()
    frame_count += 1


def paused_overlay() -> None:
    """Stamp PAUSED on the frozen frame once, when pausing — pure PIL."""
    global latest_jpeg
    with frame_lock:
        jpeg = latest_jpeg
    if jpeg is None:
        banner("paused", "rendering stopped — press resume")
        return
    img = Image.open(io.BytesIO(jpeg)).convert("RGB")
    from PIL import ImageDraw
    draw = ImageDraw.Draw(img)
    draw.rectangle([6, 6, 104, 28], fill=(18, 18, 24))
    draw.text((14, 11), "PAUSED", fill=(255, 216, 0))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=82)
    with frame_lock:
        latest_jpeg = buf.getvalue()


def mirror_loop() -> None:
    """Poll the body server and render, forever.

    When the body server goes away (its 3D window closed, a restart, or the
    heartbeat gate refusing connections), show a clearly-dead DISCONNECTED
    banner instead of a frozen frame that looks alive — and retry, because
    every reason the server disappears is also a reason it comes back: the
    gate reopens on the next heartbeat, and the next read renders again.
    """
    init_gl()
    while True:
        try:
            run_mirror()
            return
        except Exception as error:
            print(f"planview: body server unreachable ({error}) — retrying in 2 s", flush=True)
            banner("sim disconnected — retrying", "the body server refused or went away; "
                   "reconnects on its own")
            time.sleep(2.0)


def banner(line1: str, line2: str) -> None:
    """A black frame with white text; pure PIL, no GL needed."""
    global latest_jpeg
    img = Image.new("RGB", (WIDTH, HEIGHT), (12, 12, 18))
    from PIL import ImageDraw
    draw = ImageDraw.Draw(img)
    draw.text((WIDTH // 2 - 220, HEIGHT // 2 - 20), line1, fill=(255, 220, 80))
    draw.text((WIDTH // 2 - 200, HEIGHT // 2 + 20), line2, fill=(200, 200, 200))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=82)
    with frame_lock:
        latest_jpeg = buf.getvalue()


def run_mirror() -> None:
    body = socket.socket()
    body.settimeout(5)
    body.connect(("127.0.0.1", BODY_PORT))
    wire = body.makefile("rw")
    wire.write(json.dumps({"op": "hello", "protocol": 1, "joints": 15}) + "\n")
    wire.flush()
    wire.readline()
    print(f"planview: mirroring 127.0.0.1:{BODY_PORT}, scene {SCENE}", flush=True)
    n = 0
    while True:
        if not viewers:
            # Rendering is the cost; with nobody watching, idle at a low rate
            # and keep serving the last frame. The body socket stays open —
            # disconnect is detected when a viewer returns and a read runs.
            time.sleep(0.5)
            continue
        if paused:
            # Rendering is the cost; while paused, idle and keep serving the
            # last frame. The body socket stays open — disconnect is detected
            # on resume.
            time.sleep(1.0 / FPS)
            continue
        wire.write('{"op":"read"}\n')
        wire.flush()
        reading = json.loads(wire.readline())
        if n == 0:
            print("planview: first read ok", flush=True)
        render_frame(reading)
        n += 1
        if n <= 3 or n % 60 == 0:
            print(f"planview: rendered {n} frames", flush=True)
        time.sleep(1.0 / FPS)


PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><title>duck plan view</title>
<style>
  html, body {{ margin: 0; height: 100%; background: #111; color: #ccc;
                font: 14px system-ui; overflow: hidden; }}
  body {{ display: flex; }}
  aside {{ width: 200px; min-width: 200px; padding: 10px; display: flex;
          flex-direction: column; gap: 8px; border-right: 1px solid #333;
          box-sizing: border-box; overflow-y: auto; }}
  main {{ flex: 1; min-width: 0; display: flex; align-items: center;
          justify-content: center; }}
  img {{ max-width: 100%; max-height: 100vh; object-fit: contain; }}
  button {{ font: inherit; padding: 6px 0; cursor: pointer; width: 100%; }}
  .grid2 {{ display: grid; grid-template-columns: 1fr 1fr; gap: 6px; }}
  #healthbox {{ display: none; font: 11px/1.5 ui-monospace, monospace;
               color: #9d9; background: #181818; padding: 8px;
               border-radius: 4px; white-space: pre-wrap; max-height: 40vh;
               overflow-y: auto; }}
  .yaw label {{ font-size: 12px; color: #bbb; }}
  .yaw input {{ width: 100%; }}
  .yawv {{ color: #ffd800; }}
  #hint {{ font-size: 12px; color: #888; line-height: 1.7; }}
  #st {{ margin-top: auto; font-size: 12px; color: #999;
         white-space: pre-line; line-height: 1.6; }}
</style></head>
<body>
<aside>
  <button id="pause">…</button>
  <button id="follow">…</button>
  <button id="view">切到：俯视</button>
  <button id="drive">前进 3 秒</button>
  <button id="sit">坐下 / 站起</button>
  <button id="brain">🧠 大脑 on</button>
  <div class="grid2">
    <button data-skill="kick_left">左踢</button>
    <button data-skill="kick_right">右踢</button>
    <button data-skill="roulade">翻滚</button>
    <button data-skill="ground_pick">捡起</button>
  </div>
  <button id="home">回家</button>
  <button id="health">健康检查</button>
  <pre id="healthbox"></pre>
  <div class="yaw">
    <label>转向 vyaw <span class="yawv" id="yawv">0.0</span></label>
    <input id="yaw" type="range" min="-2" max="2" step="0.1" value="0">
  </div>
  <div class="yaw">
    <label>头部（°，0 = 归位姿态；正俯仰 = 低头）</label>
    <label>颈俯仰 <span class="yawv" id="vneck">0°</span></label>
    <input id="hneck" type="range" min="-63" max="63" step="1" value="0">
    <label>头俯仰 <span class="yawv" id="vhp">0°</span></label>
    <input id="hpitch" type="range" min="-63" max="63" step="1" value="0">
    <label>头偏转 <span class="yawv" id="vhy">0°</span></label>
    <input id="hyaw" type="range" min="-80" max="80" step="1" value="0">
    <label>头侧倾 <span class="yawv" id="vhr">0°</span></label>
    <input id="hroll" type="range" min="-18" max="18" step="1" value="0">
    <button id="headhome">头部归位</button>
  </div>
  <div id="hint">键盘（先点一下页面）<br>I 前进 <span class="yawv" id="cruisev">0.50</span> · K 后退 <span class="yawv" id="revv">0.80</span><br>↑/↓ 调速（<span id="cruiserange"></span>；按住 K 调后退）<br>J/L 原地转向（Shift 精细 ~10°）<br>I+J 行进转弯 · U/O 方向盘渐增回正<br>WASD 走向屏幕 上/左/下/右（先转向对准再前进；随俯视/斜视自动换算）<br>X 直线补偿 <span class="yawv" id="strv">开</span>（按速度抵消右偏，转向时自动让位）<br>头部滑杆：0 = 归位姿态，正俯仰 = 低头；策略追踪（走路时也有效），非舵机直写<br>切走或关闭页面：自动停车（先大脑 off，后断链）</div>
  <div id="st"></div>
</aside>
<main><img src="/stream?follow={follow}"></main>
<script>
const $ = id => document.getElementById(id);
async function refresh() {{
  let r;
  try {{ r = await (await fetch('/status')).json(); }}
  catch (e) {{ $('st').textContent = 'server gone'; return; }}
  viewIso = !!r.iso;
  $('pause').textContent = r.paused ? '▶ 继续渲染' : '⏸ 暂停渲染';
  $('follow').textContent = r.follow ? '切到：定点' : '切到：跟随';
  $('view').textContent = r.iso ? '切到：俯视' : '切到：斜视';
  let s = r.paused ? '已暂停（省 CPU）' : '渲染中 ' + r.fps + ' fps';
  s += '\\n相机：' + (r.follow ? '跟随鸭子' : '原点固定')
       + ' · ' + (r.iso ? '斜视' : '俯视');
  if (r.pos) {{
    s += '\\n位置 (' + r.pos[0].toFixed(2) + ', ' + r.pos[1].toFixed(2) + ') m';
    s += '\\n朝向 ' + r.pos[2].toFixed(0) + '°';
  }}
  if (r.odom && r.pos) {{
    s += '\\n里程 (' + r.odom[0].toFixed(2) + ', ' + r.odom[1].toFixed(2) + ') '
         + r.odom[2].toFixed(0) + '°';
    s += '\\n里程误差 ' + Math.hypot(r.odom[0] - r.pos[0],
                                     r.odom[1] - r.pos[1]).toFixed(2) + ' m';
  }}
  if (r.brain != null) {{
    if (r.brain_age > 5) {{
      s += '\\n🧠 无流（板子离线?）';
    }} else {{
      const on = r.brain !== 'held';
      s += '\\n🧠 大脑 ' + (on ? '开 (' + r.brain + ')' : '关 (held)');
      $('brain').textContent = '🧠 大脑 ' + (on ? 'on' : 'off');
    }}
  }}
  if (r.gate && r.gate.ok) {{
    if (!r.gate.armed) s += '\\n心跳 未上膛（页面开着就喂养）';
    else if (r.gate.starved) s += '\\n心跳 断供中 — 板子已停车';
    else s += '\\n心跳 喂养中（' + Math.max(0, Math.round(r.gate.timeout_s - r.gate.age_s)) + 's 后停车）';
  }}
  $('st').textContent = s;
}}
$('pause').onclick = async () => {{ await fetch('/toggle'); refresh(); }};
$('follow').onclick = async () => {{
  const on = $('follow').textContent.includes('跟随');
  await fetch('/ctl/follow?on=' + (on ? '1' : '0')); refresh();
}};
$('view').onclick = async () => {{
  const iso = $('view').textContent.includes('斜视');
  await fetch('/ctl/view?iso=' + (iso ? '1' : '0')); refresh();
}};
$('drive').onclick = async () => {{ await fetch('/ctl/drive'); }};
$('sit').onclick = async () => {{ await fetch('/ctl/sit'); }};
$('brain').onclick = async () => {{
  const turning_on = $('brain').textContent.includes('off');
  const r = await (await fetch('/ctl/brain?on=' + (turning_on ? '1' : '0'))).json();
  if (!r.ok) {{ alert(r.msg || 'robot.enable failed'); return; }}
  $('brain').textContent = '🧠 大脑 ' + (turning_on ? 'on' : 'off');
}};
// The body server parks the board when nobody watches. Beats go only while
// this tab is visible; hiding or closing it first sends a farewell — the
// brain turns off gracefully (the duck walks home, no mid-stride freeze) —
// and the gate then cuts the link on its own schedule. Coming back re-arms
// the gate; the brain stays off until the 🧠 button is pressed.
const beat = () => fetch('/ctl/heartbeat', {{method: 'POST'}}).catch(() => {{}});
const bye = () => navigator.sendBeacon && navigator.sendBeacon('/ctl/bye');
document.addEventListener('visibilitychange', () => {{
  if (document.visibilityState === 'visible') beat(); else bye();
}});
addEventListener('pagehide', bye);
setInterval(() => {{ if (document.visibilityState === 'visible') beat(); }}, 20000);
beat();
document.querySelectorAll('[data-skill]').forEach(b => {{
  b.onclick = async () => {{
    b.disabled = true;
    const r = await (await fetch('/ctl/skill?name=' + b.dataset.skill)).json();
    if (!r.ok) {{ alert(r.msg); b.disabled = false; return; }}
    setTimeout(() => b.disabled = false, 5000);  // one-shots run 1-3 s
  }};
}});
$('health').onclick = async () => {{
  const box = $('healthbox');
  if (box.style.display === 'block') {{ box.style.display = 'none'; return; }}
  box.style.display = 'block';
  box.textContent = '…';
  box.textContent = (await (await fetch('/ctl/health')).json()).text;
}};
$('home').onclick = async () => {{
  const r = await (await fetch('/ctl/home')).json();
  if (r.ok) {{
    $('home').disabled = true;
    setTimeout(() => $('home').disabled = false, 60000);
  }} else {{ alert(r.msg); }}
}};
// IJKL + U/O steering wheel + yaw slider: report what is held; the server
// retransmits at 10 Hz and stops 0.7 s after updates go quiet (robotd's own
// deadman is 500 ms). J/L spin in place (vyaw ±2, ~60 deg/s measured); held
// with I/K they become forward/backward arcs. U/O ramp the shared steer
// value up like a steering wheel and spring it back on release; the slider
// is a hand on the same wheel.
const keys = {{ i: 0, k: 0, j: 0, l: 0, u: 0, o: 0 }};
// WASD compass drive: walk toward SCREEN up/left/down/right — turn to face
// the screen direction first, then cruise forward. IJKL above are body-frame
// (tank controls); these are world-frame, recomputed per view: the camera
// azimuth is 90 deg in the top view (screen up = world +y) and 45 deg in the
// oblique (screen up = the +x+y diagonal), so screen up = az deg, right =
// az-90, down = az+180, left = az+90 in world yaw. viewIso is refreshed from
// /status every second.
const dirKeys = {{ w: 0, a: 0, s: 0, d: 0 }};
let lastDir = null;                             // most recent WASD press
let viewIso = true;
const yaw = $('yaw');
let steer = 0;                                  // the one vyaw source
let shift = false;                              // Shift+J/L = fine taps
let cruise = 0.5;                               // the vx W sends (Up/Down)
let rev = 0.8;                                  // |vx| S sends (Up/Down while S held)
const CRUISE_MIN = 0.3, CRUISE_MAX = 1.2, CRUISE_STEP = 0.05;
const REV_MIN = 0.4, REV_MAX = 1.2;
// 直线 compensation: the counter-vyaw that cancels the v5 policy's
// speed-dependent rightward arc (measured 2026-09-30, /tmp/straight_*.py +
// comp_high.py). Anchors at every second 0.05 step, linear between; covers
// the whole forward cruise range [0.3, 1.2]. Forward only.
let straight = true;                            // X toggles
const COMP = [
  [0.3, 0.18], [0.35, 0.2], [0.4, 0.22], [0.45, 0.33], [0.5, 0.44],
  [0.55, 0.37], [0.6, 0.3], [0.65, 0.35], [0.7, 0.4], [0.75, 0.33],
  [0.8, 0.25], [0.85, 0.3], [0.9, 0.35], [0.95, 0.35], [1.0, 0.35],
  [1.05, 0.28], [1.1, 0.2], [1.15, 0.15], [1.2, 0.1]];
function comp(v) {{
  if (v < COMP[0][0]) return 0;
  for (let i = 0; i < COMP.length - 1; i++) {{
    const a = COMP[i], b = COMP[i + 1];
    if (v <= b[0]) return a[1] + (b[1] - a[1]) * (v - a[0]) / (b[0] - a[0]);
  }}
  return COMP[COMP.length - 1][1];
}}
const STEER_MAX = 2.0, STEER_RATE = 0.25;       // per 100 ms — lock in 0.8 s
const wheeling = () => keys.u || keys.o || Math.abs(steer) > 0.01;
function held_cmd() {{
  let vx = 0;
  if (keys.i) vx += cruise;
  if (keys.k) vx -= rev;
  let vyaw = steer;
  const lock = shift ? 1.5 : 2.0;               // fine lock ~10 deg/tap
  if (!wheeling()) {{
    if (keys.j && !keys.l) vyaw = lock;
    if (keys.l && !keys.j) vyaw = -lock;
  }}
  // Straight compensation only when nothing else steers — J/L, U/O and the
  // slider stay pure manual input.
  if (straight && keys.i && !keys.k && vyaw === 0) vyaw = comp(cruise);
  return {{ vx, vy: 0, vyaw }};
}}
let driving = false;
function send_cmd() {{
  const c = held_cmd();
  driving = (c.vx !== 0 || c.vyaw !== 0);
  fetch('/ctl/keys?vx=' + c.vx + '&vy=' + c.vy + '&vyaw=' + c.vyaw).catch(() => {{}});
}}
document.addEventListener('keydown', e => {{
  const k = e.key.toLowerCase();
  if (k === 'shift') {{ shift = true; send_cmd(); return; }}
  if (k === 'x' && !e.repeat) {{
    straight = !straight;
    $('strv').textContent = straight ? '开' : '关';
    send_cmd();
    return;
  }}
  if (k === 'arrowup' || k === 'arrowdown') {{
    // cruise control: adjust whichever direction is held — the forward cruise
    // (W) normally, the reverse cruise (S) while S alone is held.
    const d = k === 'arrowup' ? CRUISE_STEP : -CRUISE_STEP;
    if (keys.k && !keys.i) {{
      rev = Math.min(REV_MAX, Math.max(REV_MIN, rev + d));
      $('revv').textContent = rev.toFixed(2);
    }} else {{
      cruise = Math.min(CRUISE_MAX, Math.max(CRUISE_MIN, cruise + d));
      $('cruisev').textContent = cruise.toFixed(2);
    }}
    e.preventDefault(); send_cmd();
    return;
  }}
  if (k in dirKeys) {{ dirKeys[k] = 1; lastDir = k; e.preventDefault(); return; }}
  if (k in keys) {{ keys[k] = 1; e.preventDefault(); send_cmd(); }}
}});
document.addEventListener('keyup', e => {{
  const k = e.key.toLowerCase();
  if (k === 'shift') {{ shift = false; send_cmd(); return; }}
  if (k in dirKeys) {{
    dirKeys[k] = 0;
    if (lastDir === k) lastDir = ['w', 'a', 's', 'd'].find(x => dirKeys[x]) || null;
    if (!lastDir) fetch('/ctl/keys?vx=0&vy=0&vyaw=0').catch(() => {{}});
    return;
  }}
  if (k in keys) {{ keys[k] = 0; send_cmd(); }}
}});
setInterval(() => {{                            // the wheel itself
  if (keys.u && !keys.o) steer = Math.min(STEER_MAX, steer + STEER_RATE);
  else if (keys.o && !keys.u) steer = Math.max(-STEER_MAX, steer - STEER_RATE);
  else if (steer > 0) steer = Math.max(0, steer - 4 * STEER_RATE);
  else if (steer < 0) steer = Math.min(0, steer + 4 * STEER_RATE);
  if (keys.u || keys.o || steer !== 0) {{
    yaw.value = steer;                          // slider mirrors the wheel
    $('yawv').textContent = steer.toFixed(1);
    send_cmd();
  }}
}}, 100);
yaw.addEventListener('input', () => {{          // a hand on the wheel directly
  steer = parseFloat(yaw.value);
  $('yawv').textContent = steer.toFixed(1);
  send_cmd();
}});
yaw.addEventListener('change', () => {{         // released: spring back to zero
  steer = 0;
  yaw.value = 0;
  $('yawv').textContent = '0.0';
  send_cmd();
}});

// Head sliders: degrees, 0 = home pose. robot.head has no deadman, so one
// fetch per input event holds; the server clamps to the trained deltas.
const headSliders = ['hneck', 'hpitch', 'hyaw', 'hroll'];
function sendHead() {{
  $('vneck').textContent = $('hneck').value + '°';
  $('vhp').textContent = $('hpitch').value + '°';
  $('vhy').textContent = $('hyaw').value + '°';
  $('vhr').textContent = $('hroll').value + '°';
  fetch('/ctl/head?neck=' + $('hneck').value + '&pitch=' + $('hpitch').value
        + '&yaw=' + $('hyaw').value + '&roll=' + $('hroll').value).catch(() => {{}});
}}
headSliders.forEach(id => $(id).addEventListener('input', sendHead));
$('headhome').onclick = () => {{
  headSliders.forEach(id => $(id).value = 0);
  sendHead();
}};
setInterval(() => {{ if (driving) send_cmd(); }}, 250);  // keep-alive heartbeat
// The WASD compass loop. 150 ms ticks: aim from the freshest ground truth
// (/status pos, updated at render rate), spin toward the screen direction
// past 15 deg of error, then cruise with a proportional heading hold
// (walking small-vyaw gain is ~40 deg/s per 1.0). Holding two of WASD+IJKL
// at once lets the two emitters interleave — last writer wins per tick.
let compassBusy = false;
setInterval(async () => {{
  const az = viewIso ? 45 : 90;
  const offs = {{ w: 0, a: 90, s: 180, d: -90 }};
  const held = lastDir && dirKeys[lastDir] ? lastDir
             : ['w', 'a', 's', 'd'].find(x => dirKeys[x]);
  if (!held || compassBusy) return;
  compassBusy = true;
  try {{
    const r = await (await fetch('/status')).json();
    if (!r.pos) return;                       // no ground truth — cannot aim
    const err = ((az + offs[held] - r.pos[2]) % 360 + 540) % 360 - 180;
    let vx = 0, vyaw = 0;
    if (Math.abs(err) > 15) {{
      vyaw = err > 0 ? 2.0 : -2.0;            // spin toward the direction
    }} else {{
      vx = cruise;                            // walk it, holding heading
      // comp() pre-cancels the gait's rightward arc so err*0.06 is purely
      // corrective (without it the hold fights the arc and holds ~13 deg).
      vyaw = Math.max(-2, Math.min(2, comp(cruise) + err * 0.06));
    }}
    fetch('/ctl/keys?vx=' + vx + '&vy=0&vyaw=' + vyaw).catch(() => {{}});
  }} catch (e) {{}}
  finally {{ compassBusy = false; }}
}}, 150);
$('cruiserange').textContent = '前进 ' + CRUISE_MIN.toFixed(2) + '–' + CRUISE_MAX.toFixed(2)
  + ' / 后退 ' + REV_MIN.toFixed(2) + '–' + REV_MAX.toFixed(2);
refresh();
setInterval(refresh, 1000);
</script>
</body></html>
"""


def follow_from_path(path: str) -> bool:
    # Follow is the default; ?follow=0 opts out (covers both the page URL,
    # whose flag is passed through to the embedded stream, and /stream).
    return "follow=0" not in path


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        global follow_duck, paused, iso_view
        if self.path.startswith("/status"):
            with odom_lock:
                odom = list(odom_pose) if odom_pose is not None else None
                brain, hz = brain_policy, loop_hz
                brain_age = time.time() - state_ts if state_ts else None
            self._json({"paused": paused, "fps": FPS, "frames": frame_count,
                        "follow": follow_duck, "iso": iso_view,
                        "pos": last_pose, "odom": odom,
                        "brain": brain, "brain_age": brain_age, "loop_hz": hz,
                        "gate": self._gate(beat=False), "viewers": viewers})
            return
        if self.path.startswith("/toggle"):
            paused = not paused
            if paused:
                paused_overlay()
            print(f"planview: rendering {'paused' if paused else 'resumed'}", flush=True)
            self._json({"paused": paused})
            return
        if self.path.startswith("/ctl/follow"):
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            on = q.get("on", ["0"])[0] == "1"
            follow_duck = on
            print(f"planview: camera follow {'on' if on else 'off'} (page button)", flush=True)
            self._json({"follow": follow_duck})
            return
        if self.path.startswith("/ctl/view"):
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            iso_view = q.get("iso", ["0"])[0] == "1"
            print(f"planview: view {'isometric' if iso_view else 'top-down'} (page button)", flush=True)
            self._json({"iso": iso_view})
            return
        if self.path.startswith("/ctl/drive"):
            threading.Thread(target=drive_leg, daemon=True).start()
            self._json({"ok": True})
            return
        if self.path.startswith("/ctl/sit"):
            try:
                reply = duck_rpc({"jsonrpc": "2.0", "id": 7, "method": "robot.do",
                                  "params": {"skill": "sit_toggle"}}, wait=True)
            except OSError as error:
                reply = f"error: {error}"
            print(f"planview: sit_toggle -> {reply}", flush=True)
            self._json({"reply": reply})
            return
        if self.path.startswith("/ctl/brain"):
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            on = q.get("on", ["1"])[0] == "1"
            try:
                reply = duck_rpc({"jsonrpc": "2.0", "id": 11, "method": "robot.enable",
                                  "params": {"on": on}}, wait=True)
                ok, msg = True, str(reply)
            except OSError as error:
                ok, msg = False, str(error)
            print(f"planview: brain {'on' if on else 'off'} -> {msg}", flush=True)
            self._json({"ok": ok, "msg": msg})
            return
        if self.path.startswith("/ctl/heartbeat"):
            # GET asks the gate's state; the page's beats are POSTs (below).
            self._json(self._gate(beat=False))
            return
        if self.path.startswith("/ctl/bye"):
            self._bye()
            return
        if self.path.startswith("/ctl/keys"):
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            try:
                vx = float(q.get("vx", ["0"])[0])
                vy = float(q.get("vy", ["0"])[0])
                vyaw = float(q.get("vyaw", ["0"])[0])
            except ValueError:
                vx = vy = vyaw = 0.0
            with kbd_lock:
                kbd_state.update(vx=vx, vy=vy, vyaw=vyaw, ts=time.time())
            self._json({"ok": True})
            return
        if self.path.startswith("/ctl/head"):
            # robot.head carries 4 head-joint deltas from the home pose,
            # radians on the wire, degrees in the query. The clamps are the
            # policy's trained sampling ranges (curriculum finals) — beyond
            # them tracking is untrained, not stronger.
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)

            def delta(name, limit):
                try:
                    deg = float(q.get(name, ["0"])[0])
                except ValueError:
                    deg = 0.0
                return max(-limit, min(limit, math.radians(deg)))

            params = {"neck_pitch": delta("neck", 1.1),
                      "head_pitch": delta("pitch", 1.1),
                      "head_yaw": delta("yaw", 1.4),
                      "head_roll": delta("roll", 0.31)}
            try:
                duck_rpc({"jsonrpc": "2.0", "method": "robot.head", "params": params})
            except OSError as error:
                print(f"planview: head failed: {error}", flush=True)
                self._json({"ok": False})
                return
            self._json({"ok": True})
            return
        if self.path.startswith("/ctl/skill"):
            q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
            name = q.get("name", [""])[0]
            if name not in skill_list():
                self._json({"ok": False, "msg": f"no skill named {name}"})
                return
            try:
                reply = duck_rpc({"jsonrpc": "2.0", "id": 7, "method": "robot.do",
                                  "params": {"skill": name}}, wait=True, timeout=8)
            except OSError as error:
                reply = f"error: {error}"
            print(f"planview: skill {name} -> {reply}", flush=True)
            self._json({"ok": True})
            return
        if self.path.startswith("/ctl/health"):
            self._json({"text": ctl_health()})
            return
        if self.path.startswith("/ctl/home"):
            ok, msg = start_home()
            print(f"planview: home -> {msg}", flush=True)
            self._json({"ok": ok, "msg": msg})
            return
        if self.path.startswith("/stream"):
            want = follow_from_path(self.path)
            if want != follow_duck:
                follow_duck = want
                print(f"planview: camera follow {'on' if want else 'off'} ({self.path})", flush=True)
            self._stream()
            return
        # Anything else serves the page; its own URL's follow flag is passed
        # through to the embedded stream.
        follow = "1" if follow_from_path(self.path) else "0"
        body = PAGE.format(follow=follow).encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        # The page's heartbeat and farewell are POSTs; every other control
        # here is GET.
        if self.path.startswith("/ctl/heartbeat"):
            self._json(self._gate(beat=True))
        elif self.path.startswith("/ctl/bye"):
            self._bye()
        else:
            self.send_error(501)

    def _gate(self, beat: bool):
        # beat=True is the page's 20 s keep-alive; beat=False asks the gate's
        # state. The body server's gate arms on the first beat and hangs the
        # board up when the beats stop — see HEARTBEAT_TIMEOUT_S there.
        try:
            req = urllib.request.Request(
                f"http://127.0.0.1:{BODY_PORT + 2}/heartbeat",
                data=b"" if beat else None, method="POST" if beat else "GET")
            with urllib.request.urlopen(req, timeout=2) as reply:
                body = json.loads(reply.read())
            ok = True
        except OSError as error:
            body, ok = {"error": str(error)}, False
        return {"ok": ok, **body}

    def _bye(self):
        # The page's farewell, sent when the tab hides or closes: turn the
        # brain off gracefully while the link is still up — the duck walks to
        # its home pose — and let the gate cut the link on its own schedule.
        try:
            reply = duck_rpc({"jsonrpc": "2.0", "id": 12, "method": "robot.enable",
                              "params": {"on": False}}, wait=True)
            ok, msg = True, str(reply)
        except OSError as error:
            ok, msg = False, str(error)
        print(f"planview: bye (tab hidden/closed) -> brain off -> {msg}", flush=True)
        self._json({"ok": ok, "msg": msg})

    def _json(self, obj: dict) -> None:
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _stream(self) -> None:
        global viewers
        viewers += 1
        try:
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
            while True:
                with frame_lock:
                    jpeg = latest_jpeg
                if jpeg is not None:
                    self.wfile.write(boundary + jpeg + b"\r\n")
                    self.wfile.flush()
                time.sleep(1.0 / FPS)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            viewers -= 1

    def log_message(self, *args):
        pass


threading.Thread(target=mirror_loop, daemon=True).start()
print(f"planview: http://localhost:{HTTP_PORT}/ — Ctrl-C to stop", flush=True)
ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler).serve_forever()
