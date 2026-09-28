# The duck by the numbers

Part masses, joint inventory, center of mass, and standing height of the
simulated duck — the facts a controller, a policy, or a curious owner keeps
reaching for. All numbers come from the MJCF robot model in the
`microduck_rl` repository (loaded through `scripts/duck-sim`'s scene), and
from the live simulator under policy control. They describe the *model*; a
real duck's hardware is close but not identical.

Reproduce everything below with a MuJoCo `MjModel` and a socket to the body
server's `read` op — the snippets are at the end.

## Mass budget

The duck is **737.2 g** of 16 bodies and 81 geoms (83 in the home scene, which
adds the floor and the yellow home disc). 14 actuators drive 14 of the joints.

| Part (CAD name) | Mass | Notes |
|---|---:|---|
| `trunk_base` | 199.2 g | Shell, main board, battery — 12 geoms |
| `jaw_soft` (head) | 188.8 g | Head shell, jaw, sensors — 19 geoms; the heaviest single part |
| `yaw_roll_motion` | 48.6 g | Head yaw/roll structure |
| `neck` + `neck_pitch` | 42.5 g | Two neck segments |
| Left leg | 129.0 g | thigh 48.2, ankle/foot 30.0, shank 21.6, hip yaw 23.0, hip roll 6.2 |
| Right leg | 129.0 g | Mirror of the left (`hip_l_2` is a CAD-export naming artifact) |

The distribution is the interesting part: **head and neck carry ~38% of the
mass, the trunk 27%, the legs 35%.** A top-heavy bird — when it falls, it
falls on its head, which is exactly why the velstand training prices
`head_impact` separately from trunk and servo-housing impacts.

## Joints and servos: 14, not 15

The wire protocol names **15 joints** — 5 left leg, 4 head/neck
(`neck_pitch`, `head_pitch`, `head_yaw`, `head_roll`), the `mouth`, and 5
right leg. The model, however, has only **14 actuated joints**: the `mouth`
slot on the wire maps to no joint and no actuator at all — the jaw is a soft
passive body (`jaw_soft`). The fifteenth model joint is the trunk's floating
base, which is a degree of freedom, not a servo. Every published policy emits
14 actions for the 14 actuated joints. If you remember fifteen servos, you are
counting the mouth.

## Standing height

From the model's keyframes, measured sole-to-top over each geom's world AABB
(marginally generous — AABBs of rotated meshes overestimate a little):

| Pose | Trunk height | Total |
|---|---:|---:|
| `STAND` | 120 mm | **~276 mm** |
| `SIT` | 70 mm | ~204 mm |
| `INIT` (folded boot pose) | 120 mm | ~267 mm |

The live policy stands with the trunk at 116–120 mm, consistent with the
`STAND` keyframe.

## Center of mass

Measured on the live, policy-controlled duck (see the note on method below),
twice, with the duck facing two different headings — both runs agree to a
millimetre:

- In the trunk frame the COM sits at **(−3, +4, +21) mm** — essentially on
  the trunk axis, 21 mm above the trunk origin.
- That puts it **~138 mm above the ground** (~133 mm above the soles): above
  a 116 mm trunk because the head and neck hang over it.
- The horizontal projection lands **inside the support polygon**, roughly
  midway between the feet (soles ~85 mm apart) and about 14 mm off the
  foot-to-foot line. The walking policy parks the COM there continuously;
  that offset, not zero, is what "standing" costs to maintain.

**Method, and a trap.** A keyframe is a pose, not a state: released without
the policy, the `STAND` keyframe falls within two seconds, and any COM read
in that window measures a fall, not a stance — the first attempt here put the
COM 85 mm outside the feet, which is that error, photographed. The correct
measurement is to rebuild `qpos` from one `read` of the *running* simulator
(trunk xyz, IMU quat, then the fourteen actuated joints by name) and call
`mj_forward`, which is what the numbers above used.

## Reproducing

```python
import json, socket
import mujoco

# the model, from the scene the sim actually loaded
m = mujoco.MjModel.from_xml_path(scene_xml)
d = mujoco.MjData(m)

# one live reading: trunk xyz, imu quat (w,x,y,z), 14 actuated joints
s = socket.create_connection(("127.0.0.1", 7801))
f = s.makefile("rw")
f.write(json.dumps({"op": "hello", "protocol": 1, "joints": 15}) + "\n"); f.flush(); f.readline()
f.write('{"op":"read"}\n'); f.flush()
r = json.loads(f.readline())

tj = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
adr = m.jnt_qposadr[tj]
d.qpos[adr:adr+3] = r["trunk"]
d.qpos[adr+3:adr+7] = r["imu"]["quat"]
# the 15-name wire order, duplicated from body_server / duck_ipc_proto
JOINT_NAMES = ("left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
               "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
               "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle")
for wire, name in enumerate(JOINT_NAMES):
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    if j >= 0:                                     # "mouth" has no joint: skip
        d.qpos[m.jnt_qposadr[j]] = r["positions"][wire]
mujoco.mj_forward(m, d)

com = d.subtree_com[0]          # world-frame center of mass
mass = m.body_subtreemass[0]    # 0.7372 kg
```

Per-part masses are `m.body_mass[i]` per body; the standing span is the max
and min world z over every duck geom's rotated `geom_aabb` corners (skip
`geom_bodyid == 0`, or the infinite floor swallows the answer).
