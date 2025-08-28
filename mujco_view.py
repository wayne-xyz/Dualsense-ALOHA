import argparse
import os
import sys
import time

try:
    import tkinter as tk
    from tkinter import filedialog
    _HAS_TK = True
except Exception:
    _HAS_TK = False

import mujoco
import mujoco.viewer


def pick_xml_file() -> str:
    if not _HAS_TK:
        return ""
    root = tk.Tk()
    root.withdraw()
    path = filedialog.askopenfilename(
        title="Select MuJoCo XML file",
        filetypes=[("MuJoCo XML", "*.xml"), ("All files", "*.*")],
    )
    root.update()
    root.destroy()
    return path or ""


def load_model(xml_path: str) -> tuple[mujoco.MjModel, mujoco.MjData]:
    if not os.path.isfile(xml_path):
        raise FileNotFoundError(f"XML file not found: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    return model, data


def print_model_info(model: mujoco.MjModel):
    print(f"Model info:")
    print(f"  nq (qpos): {model.nq}")
    print(f"  nv (qvel): {model.nv}")
    print(f"  nu (actuators): {model.nu}")
    print(f"  nbody: {model.nbody}")
    print(f"  nsite: {model.nsite}")
    print(f"  njnt: {model.njnt}")
    print(f"  Joints:")
    for i in range(model.njnt):
        j = model.joint(i)
        name = j.name
        # Fetch joint type from model arrays to avoid numpy ndarray issues
        joint_type = int(model.jnt_type[i])
        qposadr = model.jnt_qposadr[i]
        dofadr = model.jnt_dofadr[i]
        # Joint type as string
        joint_type_str = {
            mujoco.mjtJoint.mjJNT_FREE: "free",
            mujoco.mjtJoint.mjJNT_BALL: "ball",
            mujoco.mjtJoint.mjJNT_SLIDE: "slide",
            mujoco.mjtJoint.mjJNT_HINGE: "hinge"
        }.get(joint_type, str(joint_type))
        # Number of dof for this joint
        if joint_type == mujoco.mjtJoint.mjJNT_FREE:
            dof = 6
        elif joint_type == mujoco.mjtJoint.mjJNT_BALL:
            dof = 3
        else:
            dof = 1
        print(f"    [{i}] name: '{name}', type: {joint_type_str}, qposadr: {qposadr}, dofadr: {dofadr}, dof: {dof}")
    print(f"  Actuators:")
    for i in range(model.nu):
        a = model.actuator(i)
        trntype = int(a.trntype)
        # Map transform type to readable string
        trntype_str = {
            mujoco.mjtTrn.mjTRN_JOINT: "joint",
            mujoco.mjtTrn.mjTRN_JOINTINPARENT: "joint_in_parent",
            mujoco.mjtTrn.mjTRN_TENDON: "tendon",
            mujoco.mjtTrn.mjTRN_SITE: "site",
            mujoco.mjtTrn.mjTRN_BODY: "body",
        }.get(trntype, str(trntype))
        # Resolve the primary target object name from trnid depending on trntype
        obj_name = ""
        obj_type = None
        if trntype in (mujoco.mjtTrn.mjTRN_JOINT, mujoco.mjtTrn.mjTRN_JOINTINPARENT):
            obj_type = mujoco.mjtObj.mjOBJ_JOINT
        elif trntype == mujoco.mjtTrn.mjTRN_TENDON:
            obj_type = mujoco.mjtObj.mjOBJ_TENDON
        elif trntype == mujoco.mjtTrn.mjTRN_SITE:
            obj_type = mujoco.mjtObj.mjOBJ_SITE
        elif trntype == mujoco.mjtTrn.mjTRN_BODY:
            obj_type = mujoco.mjtObj.mjOBJ_BODY
        if obj_type is not None:
            trnid = model.actuator_trnid[i]
            obj_id = int(trnid[0])
            if obj_id >= 0:
                obj_name = mujoco.mj_id2name(model, obj_type, obj_id) or ""
        print(f"    [{i}] name: '{a.name}', target: '{obj_name}', trntype: {trntype_str}, gear: {a.gear}")


def run_viewer(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    try:
        with mujoco.viewer.launch_passive(model=model, data=data, show_left_ui=True, show_right_ui=False) as viewer:
            mujoco.mjv_defaultFreeCamera(model, viewer.cam)
            last_time = time.time()
            while viewer.is_running():
                # Step at real-time if possible
                now = time.time()
                dt = max(model.opt.timestep, now - last_time)
                last_time = now
                mujoco.mj_step(model, data)
                viewer.sync()
    except KeyboardInterrupt:
        pass


def main():
    parser = argparse.ArgumentParser(description="Quick MuJoCo XML Viewer")
    parser.add_argument("--xml", type=str, default="", help="Path to MuJoCo XML file")
    args = parser.parse_args()

    xml_path = args.xml.strip()
    if not xml_path:
        xml_path = pick_xml_file()
    if not xml_path:
        print("No XML selected. Exiting.")
        sys.exit(0)

    try:
        model, data = load_model(xml_path)
    except Exception as e:
        print(f"Failed to load XML '{xml_path}': {e}")
        sys.exit(1)

    print(f"Loaded: {xml_path}")
    print_model_info(model)
    run_viewer(model, data)


if __name__ == "__main__":
    # print mujoco version
    print(f"Mujoco version: {mujoco.__version__}")
    main()
