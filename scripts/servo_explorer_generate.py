#!/usr/bin/env python3
"""Regenerate scripts/servo_explorer_data.js — the kinematic facts the
servo explorer page draws.

Reads the walking MJCF straight out of the RL repo (the same robot the
body_server mirrors) with mujoco, and emits the body tree, the 14 hinges
in WIRE order (duck_ipc_proto::JOINT_NAMES, the order every positional
array on the sim link uses), the drawable geoms, and any keyframes as
pose presets. Pure data, no physics: the page does its own forward
kinematics in JavaScript, so it runs from a file:// double-click with no
Python anywhere near it.

Run:
    ~/Pollen/microduck_rl/.venv/bin/python scripts/servo_explorer_generate.py
"""
import json
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
MJCF = Path.home() / "Pollen/microduck_rl/src/mjlab_microduck/robot/microduck/robot_walk.xml"
OUT = HERE / "servo_explorer_data.js"

# The wire order — duplicated from `body_server.py`, which duplicates it from
# `duck_ipc_proto::JOINT_NAMES`. The mouth (index 9) has no actuator in the
# model; every positional array on the wire still carries its slot.
JOINT_NAMES = (
    "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
    "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
    "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
)

CN = {
    "left_hip_yaw": "左髋偏航", "left_hip_roll": "左髋侧滚", "left_hip_pitch": "左髋俯仰",
    "left_knee": "左膝", "left_ankle": "左踝",
    "neck_pitch": "颈俯仰", "head_pitch": "头俯仰", "head_yaw": "头偏航", "head_roll": "头侧滚",
    "mouth": "嘴（线上占位，无执行器）",
    "right_hip_yaw": "右髋偏航", "right_hip_roll": "右髋侧滚", "right_hip_pitch": "右髋俯仰",
    "right_knee": "右膝", "right_ankle": "右踝",
}


def geom_boxes(model, bid):
    """Drawable volumes for a body's geoms.

    The shells are meshes: vertices live in the raw asset frame, and the
    compiler's mesh_pos/mesh_quat carry them into the geom frame. Each mesh
    becomes the AABB of its transformed vertices (an oriented box in geom
    frame); plain primitives pass through with their own pose.
    """
    out = []
    for g in range(model.body_geomadr[bid], model.body_geomadr[bid] + model.body_geomnum[bid]):
        gtype = int(model.geom_type[g])
        if gtype == mujoco.mjtGeom.mjGEOM_MESH:
            mid = model.geom_dataid[g]
            v = model.mesh_vert[model.mesh_vertadr[mid]:model.mesh_vertadr[mid] + model.mesh_vertnum[mid]]
            rot = np.zeros(9)
            mujoco.mju_quat2Mat(rot, model.mesh_quat[mid])
            rot = rot.reshape(3, 3)
            vg = v @ rot.T + model.mesh_pos[mid]
            lo, hi = vg.min(axis=0), vg.max(axis=0)
            center = (hi + lo) / 2
            groot = np.zeros(9)
            mujoco.mju_quat2Mat(groot, model.geom_quat[g])
            groot = groot.reshape(3, 3)
            # compose geom frame ∘ box (the AABB is axis-aligned in the geom
            # frame) into one pose in the body frame, so the page needs no
            # extra nesting
            out.append({
                "kind": "box",
                "size": [float(v) for v in (hi - lo) / 2],
                "pos": [float(v) for v in model.geom_pos[g] + groot @ center],
                "quat": [float(v) for v in model.geom_quat[g]],
            })
            continue
        kind = {2: "sphere", 3: "capsule", 5: "capsule", 6: "box"}.get(gtype)
        if kind is None:
            continue
        out.append({
            "kind": kind,
            "size": [float(v) for v in model.geom_size[g]],
            "pos": [float(v) for v in model.geom_pos[g]],
            "quat": [float(v) for v in model.geom_quat[g]],
        })
    return out


def collect(model, bid, parent_name, bodies, order):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
    entry = {
        "name": name,
        "parent": parent_name,
        "pos": [float(v) for v in model.body_pos[bid]],
        "quat": [float(v) for v in model.body_quat[bid]],
        "joints": [],
        "geoms": [],
    }
    for j in range(model.body_jntadr[bid], model.body_jntadr[bid] + model.body_jntnum[bid]):
        if model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        jname = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        entry["joints"].append({
            "name": jname,
            "cn": CN.get(jname, jname),
            "axis": [float(v) for v in model.jnt_axis[j]],
            "pos": [float(v) for v in model.jnt_pos[j]],
            "range": [float(v) for v in model.jnt_range[j]],
        })
    entry["geoms"] = geom_boxes(model, bid)
    bodies[name] = entry
    order.append(name)
    for cid in range(model.nbody):
        if model.body_parentid[cid] == bid and cid != bid:
            collect(model, cid, name, bodies, order)


def main():
    model = mujoco.MjModel.from_xml_path(str(MJCF))

    trunk = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "trunk_base_freejoint")
    bodies, order = {}, []
    collect(model, model.jnt_bodyid[trunk], None, bodies, order)

    # every hinge, in wire order, with its body and slot
    by_name = {j["name"]: (bname, j) for bname, e in bodies.items() for j in e["joints"]}
    servos = []
    for slot, jname in enumerate(JOINT_NAMES):
        if jname == "mouth":
            continue
        bname, joint = by_name[jname]
        servos.append({"slot": slot, "joint": jname, "cn": CN[jname],
                       "body": bname, "range": joint["range"]})
    missing = [n for n in JOINT_NAMES if n != "mouth" and n not in by_name]

    # robot_walk.xml carries no keyframes; the INIT/STAND/SIT poses live in
    # the scene file — read the presets from there (same 21-qpos layout)
    presets = {}
    key_model = mujoco.MjModel.from_xml_path(str(MJCF).replace("robot_walk.xml", "scene_allcollisions.xml"))
    for k in range(key_model.nkey):
        kname = mujoco.mj_id2name(key_model, mujoco.mjtObj.mjOBJ_KEY, k)
        qpos = key_model.key_qpos[k]
        values = []
        for jname in JOINT_NAMES:
            if jname == "mouth":
                values.append(0.0)
                continue
            jid = mujoco.mj_name2id(key_model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            values.append(float(qpos[key_model.jnt_qposadr[jid]]))
        presets[kname] = values

    payload = {
        "mjcf": str(MJCF),
        "joint_names": list(JOINT_NAMES),
        "mouth_slot": JOINT_NAMES.index("mouth"),
        "servos": servos,
        "missing_joints": missing,
        "bodies": [bodies[n] for n in order],
        "presets": presets,
    }
    if missing:
        raise SystemExit(f"wire joints unresolved: {missing}")

    OUT.write_text(
        "// Generated by scripts/servo_explorer_generate.py — do not hand-edit;\n"
        "// re-run the generator after the MJCF changes.\n"
        "window.SERVO_DATA = " + json.dumps(payload, ensure_ascii=False, indent=1) + ";\n",
        encoding="utf-8")
    kinds = {}
    for b in payload["bodies"]:
        for g in b["geoms"]:
            kinds[g["kind"]] = kinds.get(g["kind"], 0) + 1
    print(f"bodies: {len(payload['bodies'])}, geoms: {kinds}, "
          f"servos: {len(payload['servos'])}, presets: {list(presets)}")


if __name__ == "__main__":
    main()
