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

The duck is **737.2 g** across 15 bodies and 81 geoms (the loaded scene adds
the world body and the floor — 16 bodies, 82 geoms; the home scene's yellow
disc makes 83). 14 actuators drive 14 of the 15 joints; the fifteenth is the
trunk's floating base. Counting tags in the MJCF instead of loading it
overcounts: `<default>` classes declare another 8 joints and 7 actuators that
never instantiate.

Per-body mass, mesh-envelope volume, and the density between them. The last
column is measured, not inferred — the export stamps the XL330 mesh at each
servo position; [Where the servos are](#where-the-servos-are) explains.

| Body (CAD name) | Mass | Volume | Density | Servos housed |
|---|---:|---:|---:|---|
| `trunk_base` | 199.2 g | 181 cm³ | 1.10 | 2 — the hip-yaw pair |
| `jaw_soft` (head shell) | 188.8 g | 211 cm³ | 0.89 | 2 — `head_roll` + `mouth`; carries the `head_roll` joint |
| `yaw_roll_motion` | 48.6 g | 21 cm³ | 2.29 | 1 — `head_yaw`; carries the `head_yaw` joint |
| `upper_leg_left` / `_right` | 48.2 g | 37 cm³ | 1.31 | 2 each — hip pitch + knee |
| `neck` | 36.8 g | 32 cm³ | 1.15 | 2 — `neck_pitch` + `head_pitch` |
| `ankle_left` / `_right` | 30.0 g | 33 cm³ | 0.92 | 0 — foot shell |
| `yaw2roll` / `bearing_roll` | 23.0 g | 20 cm³ | 1.17 | 1 each — hip roll |
| `leg` / `leg_2` (shanks) | 21.6 g | 23 cm³ | 0.96 | 1 each — ankle |
| `hip_l` / `hip_l_2` | 6.2 g | 10 cm³ | 0.65 | 0 — hollow seat |
| `neck_pitch` | 5.7 g | 4.6 cm³ | 1.24 | 0 — small coupling piece |
| **Total** | **737.2 g** | **692 cm³** | **1.06** | **15** |

(`hip_l_2` and friends are a CAD-export naming artifact, not a sixth limb
part; each leg is 129.0 g.)

The distribution is the interesting part: **head and neck carry ~38% of the
mass, the trunk 27%, the legs 35%.** A top-heavy bird — when it falls, it
falls on its head, which is exactly why the velstand training prices
`head_impact` separately from trunk and servo-housing impacts.

## Where the servos are

No body in the table above *is* a servo, but the CAD export stamps the
actual XL330 mesh at every servo position — 15 of them, each folded into the
mounting link — so placement is **measured, not inferred**: count the
`xl330`-mesh geoms per body. Every joint's servo sits within 18 mm of that
joint's anchor; the fifteenth (the mouth, which has no joint) sits at the
front of the head shell, where a mouth goes.

| Servo ID | Drives | Mounted in |
|---|---|---|
| 20 / 10 | left / right hip yaw | `trunk_base` |
| 21 / 11 | left / right hip roll | `yaw2roll` / `bearing_roll` |
| 22 / 12 | left / right hip pitch | `upper_leg_left` / `_right` |
| 23 / 13 | left / right knee | `upper_leg_left` / `_right` |
| 24 / 14 | left / right ankle | `leg` / `leg_2` (the shanks) |
| 30 | `neck_pitch` | `neck` |
| 31 | `head_pitch` | `neck` |
| 32 | `head_yaw` | `yaw_roll_motion` |
| 33 | `head_roll` | `jaw_soft` |
| 34 | `mouth` | `jaw_soft` |

The pattern is the natural one: each link houses the servos that drive its
children's joints — the trunk hosts the hip-yaw pair, the upper leg the hip
pitch and the knee, the shank the ankle, the neck the two pitch servos, the
head shell the roll servo and the mouth.

Mass: the servos are *in* the numbers. The export folds each servo's mass
into its mounting link — no servo bodies — at close to the real thing. The
`xl330` mesh is true to size (15.7 cm³ envelope against the real
20 × 34 × 25 mm box), a real XL330-M077-T weighs **18 g**, and at 18 g per
servo the bodies close almost exactly: the neck is 36.8 g around two servos
plus a collar, the shank 21.6 g around one plus its shell. Against the
**~800 g** the RL repository's README quotes for the built bird
(`robot.servos="xl330"` names the model), the sim's **737.2 g** leaves a gap
of a few percent — cables, fasteners, rounding — the same order as the ±5%
mass randomization training applies. The sim is not quietly carrying a
lighter duck than the hardware. The sim drives 14 of the 15; the mouth is
present as geometry only.

## Joints and servos: 14, not 15

The wire protocol names **15 joints** — 5 left leg, 4 head/neck
(`neck_pitch`, `head_pitch`, `head_yaw`, `head_roll`), the `mouth`, and 5
right leg. The model, however, has only **14 actuated joints**: the `mouth`
slot maps to no joint and no actuator at all — there is no jaw in the model
(`jaw_soft`, despite the name, is the head's front shell and carries the
`head_roll` servo joint). The fifteenth model joint is the trunk's floating
base, which is a degree of freedom, not a servo. Every published policy emits
14 actions for the 14 actuated joints. If you remember fifteen servos, you are
counting the mouth.

The 15 wire slots against the real Dynamixel IDs:

| Wire slot | Joint | Servo ID | In the sim |
|---|---|---:|---|
| 0–4 | left hip yaw / roll / pitch, knee, ankle | 20–24 | yes |
| 5 | `neck_pitch` | 30 | yes |
| 6 | `head_pitch` | 31 | yes |
| 7 | `head_yaw` | 32 | yes |
| 8 | `head_roll` | 33 | yes |
| 9 | `mouth` | 34 | **no** — no joint to map to |
| 10–14 | right hip yaw / roll / pitch, knee, ankle | 10–14 | yes |

`body_server` resolves each wire name against the model by name and silently
drops the one that misses — "fifteen joints out here, fourteen in the model".
So a real duck's mouth servo (ID 34, travel roughly −5°..+30°) has no
simulated counterpart and `robot.mouth` is a no-op in the sim — though the
servo itself is in the model as geometry, bolted into the head shell (see
[Where the servos are](#where-the-servos-are)).

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

# mesh-envelope volume per body. This MuJoCo build exposes no mesh_vol, so
# integrate each mesh over its boundary (divergence theorem). mesh_vert and
# mesh_face are global arrays — slice them per mesh, and dedupe by mesh id:
# meshes are shared between geoms (the foot pads, for instance, one mesh
# across 15 geoms).
import numpy as np

meshvol = {}
for g in range(m.ngeom):
    if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_dataid[g] < 0:
        continue
    mid = int(m.geom_dataid[g])
    if mid in meshvol:
        continue
    va, vn = int(m.mesh_vertadr[mid]), int(m.mesh_vertnum[mid])
    fa, fn = int(m.mesh_faceadr[mid]), int(m.mesh_facenum[mid])
    v = m.mesh_vert[va:va + vn].astype(float)
    f = m.mesh_face[fa:fa + fn].reshape(-1, 3)
    a, b, c = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
    meshvol[mid] = abs(np.einsum("ij,ij->i", a, np.cross(b, c)).sum() / 6.0)

bodyvol = {}
for g in range(m.ngeom):
    if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or m.geom_dataid[g] < 0:
        continue
    bid = int(m.geom_bodyid[g])
    if bid:
        bodyvol[bid] = bodyvol.get(bid, 0.0) + meshvol[int(m.geom_dataid[g])]

for bid, vol in bodyvol.items():    # grams, cm3, g/cm3
    print(m.body(bid).name, m.body_mass[bid] * 1e3, vol * 1e6,
          m.body_mass[bid] / vol)

# servo placement: the export stamps the xl330 mesh at every servo position,
# folded into the mounting link — count them per body
from collections import Counter
print(Counter(m.body(int(m.geom_bodyid[g])).name for g in range(m.ngeom)
              if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH
              and m.geom_dataid[g] >= 0
              and m.mesh(int(m.geom_dataid[g])).name == "xl330"))
```

Per-part masses are `m.body_mass[i]` per body; the standing span is the max
and min world z over every duck geom's rotated `geom_aabb` corners (skip
`geom_bodyid == 0`, or the infinite floor swallows the answer). Read the
densities as a rough check, not a signal: hollow shells read low even with
their servos counted — the placement list above is the ground truth for
where the servos are.
