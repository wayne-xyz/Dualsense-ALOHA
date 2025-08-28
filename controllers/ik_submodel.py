import os
import tempfile
from typing import Literal, Tuple, List, Dict

import mujoco as mj
from dm_control import mjcf


Side = Literal["left", "right"]


def _remove_other_arm_elements(model: mjcf.RootElement, keep_side: Side) -> None:
    """Remove the opposite arm body and its actuators from the parsed MJCF model.

    This function mutates the provided MJCF tree to keep only the requested arm chain.
    """
    other = "right" if keep_side == "left" else "left"

    # Remove the opposite arm root body
    other_root = mjcf.find(model, 'body', name=f"{other}_base_link")
    if other_root is not None:
        other_root.remove()

    # Remove actuators for the opposite arm
    actuators = list(mjcf.find_all(model, 'actuator'))
    for act in actuators:
        name = act.name or (act.joint and act.joint.name) or ""
        if name.startswith(f"{other}_"):
            act.remove()


def _collect_joint_names_in_order(m: mj.MjModel) -> List[str]:
    names: List[str] = []
    for i in range(m.njnt):
        j = m.joint(i)
        names.append(j.name)
    return names


def build_arm_submodel(
    main_model: mj.MjModel,
    scene_xml_path: str,
    side: Side,
) -> Tuple[mj.MjModel, List[int], List[int], List[str]]:
    """Create an IK-only submodel for a single arm by trimming the ALOHA scene XML.

    Returns:
      - sub_model: the trimmed MuJoCo model
      - main_qpos_indices: indices in the main model qpos that correspond to sub_model's joint order
      - main_qvel_indices: indices in the main model qvel that correspond to sub_model's joint order
      - sub_joint_names: joint names in sub_model order (for velocity-limit maps)
    """
    if not os.path.exists(scene_xml_path):
        raise FileNotFoundError(scene_xml_path)

    # First try the MuJoCo 3.2.0+ mjSpec workflow for robust asset handling
    try:
        spec = mj.MjSpec.from_file(scene_xml_path)

        def iter_bodies_recursive(body):
            yield body
            for child in getattr(body, 'bodies', []):
                yield from iter_bodies_recursive(child)

        # Identify and remove the opposite arm subtree if present
        other = "right" if side == "left" else "left"
        # Attempt to locate the root body of the opposite arm
        world = getattr(spec, 'worldbody', None)
        other_root = None
        if world is not None:
            for b in iter_bodies_recursive(world):
                if getattr(b, 'name', '') == f"{other}_base_link":
                    other_root = b
                    break
        # Remove the opposite subtree by detaching it from its parent if API allows
        if other_root is not None and hasattr(other_root, 'parent') and hasattr(other_root.parent, 'bodies'):
            try:
                other_root.parent.bodies.remove(other_root)
            except Exception:
                pass

        # Also prune actuators belonging to the other side, if accessible via list
        if hasattr(spec, 'actuators'):
            try:
                for act in list(spec.actuators):
                    name = getattr(act, 'name', '') or ''
                    if name.startswith(f"{other}_"):
                        spec.actuators.remove(act)
            except Exception:
                pass

        # Compile the spec to a MuJoCo model
        sub_model = mj.MjModel.from_spec(spec)
    except Exception:
        # Fallback to dm_control MJCF editing, then save to temp file within the XML dir to resolve relative assets
        with open(scene_xml_path, 'r', encoding='utf-8') as f:
            xml_text = f.read()

        mjcf_model = mjcf.from_xml_string(xml_text)
        _remove_other_arm_elements(mjcf_model, keep_side=side)
        trimmed_xml = mjcf_model.to_xml_string()
        base_dir = os.path.dirname(scene_xml_path)
        tmp_path = None
        with tempfile.NamedTemporaryFile('w', suffix='.xml', dir=base_dir, delete=False, encoding='utf-8') as tmp:
            tmp.write(trimmed_xml)
            tmp_path = tmp.name
        try:
            sub_model = mj.MjModel.from_xml_path(tmp_path)
        finally:
            try:
                if tmp_path and os.path.exists(tmp_path):
                    os.remove(tmp_path)
            except Exception:
                pass

    # Build joint order and index mappings
    sub_joint_names: List[str] = _collect_joint_names_in_order(sub_model)
    main_qpos_indices: List[int] = []
    main_qvel_indices: List[int] = []
    for jname in sub_joint_names:
        # Map by exact joint name found in both models (try with and without model prefix)
        try:
            jid_main = main_model.joint(jname).id
        except Exception:
            try:
                jid_main = main_model.joint(f"aloha_scene/{jname}").id
            except Exception as e:
                raise KeyError(f"Failed to resolve joint '{jname}' in main model") from e
        main_qpos_indices.append(int(main_model.jnt_qposadr[jid_main]))
        main_qvel_indices.append(int(main_model.jnt_dofadr[jid_main]))

    return sub_model, main_qpos_indices, main_qvel_indices, sub_joint_names


