# 数字里的鸭子

> **翻译页。** 本页是 [`body-spec.md`](body-spec.md) 的中文翻译，仅供阅读；机制所有权归英文页——两处不一致时以英文页为准，要修的是翻译。代码块与英文版完全一致。

仿真鸭的零件质量、关节清单、质心和站高——控制器、策略、好奇的主人都会反复来查的数字。全部数字来自 `microduck_rl` 仓库的 MJCF 机器人模型（经 `scripts/duck-sim` 的场景加载），以及策略控制下的实时仿真器。它们描述的是**模型**；真鸭的硬件接近但不完全相同。

以下所有内容都可以用一个 MuJoCo `MjModel` 加一个连到 body server `read` op 的 socket 复现——代码片段在文末。

## 质量预算

整只鸭 **737.2 g**，分布在 15 个 body、81 个 geom 上（加载后的场景另加 world body 和地面——16 个 body、82 个 geom；home 场景的黄色圆盘凑到 83）。15 个关节中 14 个由执行器驱动；第十五个是躯干的浮动基座。直接数 MJCF 里的标签会数多：`<default>` 类声明里另有 8 个关节和 7 个执行器，它们从不实例化。

每个 body 的质量、网格包络体积，以及两者相除的密度。最后一列是实测而非推断——CAD 导出把 XL330 网格印在每一个舵机位置上，[舵机都在哪](#舵机都在哪)一节解释。

| 零件（CAD 名） | 质量 | 体积 | 密度 | 所含舵机 |
|---|---:|---:|---:|---|
| `trunk_base` | 199.2 g | 181 cm³ | 1.10 | 2 — 髋偏航一对 |
| `jaw_soft`（头部前壳） | 188.8 g | 211 cm³ | 0.89 | 2 — `head_roll` + `mouth`；带 `head_roll` 关节 |
| `yaw_roll_motion` | 48.6 g | 21 cm³ | 2.29 | 1 — `head_yaw`；带 `head_yaw` 关节 |
| `upper_leg_left` / `_right` | 48.2 g | 37 cm³ | 1.31 | 各 2 — 髋俯仰 + 膝 |
| `neck` | 36.8 g | 32 cm³ | 1.15 | 2 — `neck_pitch` + `head_pitch` |
| `ankle_left` / `_right` | 30.0 g | 33 cm³ | 0.92 | 0 — 脚壳 |
| `yaw2roll` / `bearing_roll` | 23.0 g | 20 cm³ | 1.17 | 各 1 — 髋侧摆 |
| `leg` / `leg_2`（小腿） | 21.6 g | 23 cm³ | 0.96 | 各 1 — 踝 |
| `hip_l` / `hip_l_2` | 6.2 g | 10 cm³ | 0.65 | 0 — 空心括座 |
| `neck_pitch` | 5.7 g | 4.6 cm³ | 1.24 | 0 — 小连接件 |
| **合计** | **737.2 g** | **692 cm³** | **1.06** | **15** |

（`hip_l_2` 们只是 CAD 导出的命名产物，不是第六条腿的零件；每条腿 129.0 g。）

trunk 的分解——推断而非实测，因为惯量是合并值——大致是 **2 舵机（36 g）+ 电池（~119 g）+ 壳体/主板/轴承（~45 g）**。电池在 CAD 里是真实零件：`trunk_base` 内一个 52 cm³ 的 `np_f970` 网格，按锂聚合物电包密度折 ~119 g，恰好是扣除舵机后 trunk 缺的那块。仿真没有带上的是电池的**电学侧**——电压和温度在那里是常数（见 [仿真页：哪些地方是/不是孪生](../design/simulation.md#7-what-it-is-and-is-not-a-twin-of)）。

分布才是有意思的部分：**头颈占质量的 ~38%，躯干 27%，腿 35%。** 一只头重的鸟——摔倒时头先着地，这正是 velstand 训练把 `head_impact` 与躯干、舵机座冲击分开计价的原因。

## 舵机都在哪

上表没有任何一个 body *是*舵机，但 CAD 导出把真实的 XL330 网格印在了每一个舵机位置——共 15 个，各自折进安装所在的连杆——所以位置是**实测的，不是推断的**：按 body 数一数 `xl330` 网格的 geom 即可。每个关节的舵机都在该关节锚点 18 mm 以内；第十五个（嘴，没有关节）位于头部前壳前端——嘴该在的地方。

| 舵机 ID | 驱动 | 装在 |
|---|---|---|
| 20 / 10 | 左 / 右 髋偏航 | `trunk_base` |
| 21 / 11 | 左 / 右 髋侧摆 | `yaw2roll` / `bearing_roll` |
| 22 / 12 | 左 / 右 髋俯仰 | `upper_leg_left` / `_right` |
| 23 / 13 | 左 / 右 膝 | `upper_leg_left` / `_right` |
| 24 / 14 | 左 / 右 踝 | `leg` / `leg_2`（小腿） |
| 30 | `neck_pitch` | `neck` |
| 31 | `head_pitch` | `neck` |
| 32 | `head_yaw` | `yaw_roll_motion` |
| 33 | `head_roll` | `jaw_soft` |
| 34 | `mouth` | `jaw_soft` |

规律很自然：每根连杆安装着驱动其子关节的舵机——躯干装髋偏航一对，大腿装髋俯仰和膝，小腿装踝，颈部装两个俯仰舵机，头部前壳装横滚舵机和嘴。

质量方面：**舵机质量在数子里**。导出把每只舵机的质量折进安装它的连杆——没有独立的舵机 body——且接近实重。`xl330` 网格尺寸属实（15.7 cm³ 包络，对照实物 20 × 34 × 25 mm 的外盒），一只真的 XL330-M077-T 重 **18 g**；按每只 18 g 算，各 body 几乎精确闭合：`neck` 是 36.8 g = 两只舵机 + 一个领圈，小腿 21.6 g = 一只 + 壳。对照 RL 仓库 README 给整机记的 **~800 g**（`robot.servos="xl330"` 标明了舵机型号），仿真的 **737.2 g** 只差百分之几——线缆、紧固件、圆整——与训练所加的 ±5% 质量随机化同量级。仿真并没有偷偷养一只比硬件轻的鸭。仿真驱动 15 只中的 14 只；嘴只以几何体的形式存在。

## 关节与舵机：14，不是 15

线序协议命名 **15 个关节**——左腿 5 个，头颈 4 个（`neck_pitch`、`head_pitch`、`head_yaw`、`head_roll`），`mouth`，右腿 5 个。但模型只有 **14 个受驱动关节**：`mouth` 槽位不映射到任何关节、任何执行器——模型里没有下巴（`jaw_soft` 名字虽如此，其实是头部前壳，带的是 `head_roll` 舵机关节）。模型的第十五个关节是躯干浮动基座，那是自由度，不是舵机。所有已发布策略对 14 个受驱动关节输出 14 个动作。如果你记得是十五只舵机，那是把嘴数进去了。

15 个线序槽位对真实 Dynamixel ID：

| 线序槽位 | 关节 | 舵机 ID | 在仿真中 |
|---|---|---:|---|
| 0–4 | 左髋偏航/侧摆/俯仰、膝、踝 | 20–24 | 有 |
| 5 | `neck_pitch` | 30 | 有 |
| 6 | `head_pitch` | 31 | 有 |
| 7 | `head_yaw` | 32 | 有 |
| 8 | `head_roll` | 33 | 有 |
| 9 | `mouth` | 34 | **无** — 没有关节可映射 |
| 10–14 | 右髋偏航/侧摆/俯仰、膝、踝 | 10–14 | 有 |

`body_server` 按名字把每个线序名对到模型，对不上的那个被静默丢弃——"外面十五个关节，模型里十四个"。所以真鸭的嘴部舵机（ID 34，行程约 −5°..+30°）在仿真中没有对应物，`robot.mouth` 在仿真中是空操作——不过舵机本身以几何体的形式在模型里，锁在头部前壳上（见[舵机都在哪](#舵机都在哪)）。

## 站高

由模型关键帧测得，从脚底到头顶量每个 geom 的世界系 AABB（略偏宽松——旋转过的网格 AABB 会略微高估）：

| 姿态 | 躯干高度 | 总高 |
|---|---:|---:|
| `STAND` | 120 mm | **~276 mm** |
| `SIT` | 70 mm | ~204 mm |
| `INIT`（开机折叠姿态） | 120 mm | ~267 mm |

实机策略站立时躯干 116–120 mm，与 `STAND` 关键帧一致。

## 质心

在实时、策略控制的鸭身上测的（方法见下），测了两次、朝向不同——两次在毫米级一致：

- 躯干系下质心在 **(−3, +4, +21) mm**——基本在躯干轴上，高于躯干原点 21 mm。
- 即**离地 ~138 mm**（高出脚底 ~133 mm）：比 116 mm 的躯干高，因为头颈悬在躯干上方。
- 水平投影落在**支撑多边形内**，大致在两脚中间（脚底相距 ~85 mm），偏离两脚连线约 14 mm。行走策略让质心一直停在那里；维持"站立"要付的正是这个偏置，而不是零。

**方法与陷阱。** 关键帧是姿态，不是状态：没有策略时放开，`STAND` 关键帧两秒内就倒，这段时间里读到的任何质心都在量一次摔倒而不是站姿——本文第一次尝试就把质心量到了脚外 85 mm，拍下来的正是这个错误。正确的做法是从*运行中*仿真器的一次 `read` 重建 `qpos`（躯干 xyz、IMU 四元数、再按名字填十四个受驱动关节），然后调 `mj_forward`——上面的数字就是这么来的。

## 复现

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

各零件质量是 `m.body_mass[i]`；站姿总高是所有鸭 geom 旋转后 `geom_aabb` 角点的世界 z 最大最小值之差（跳过 `geom_bodyid == 0`，不然无穷大的地面会吞掉答案）。密度只当粗校验读，别当信号：空心壳即使算上舵机，密度读数也偏低——舵机在哪，以上面的位置清单为准。
