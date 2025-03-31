"""ALOHA Robot Configuration."""

# Source From the Nintendo ALOHA: https://github.com/AlmondGod/Nintendo-Aloha/blob/master/bigym/robots/configs/aloha.py
import numpy as np
from mojo.elements.consts import JointType

from bigym.action_modes import PelvisDof
from bigym.const import ASSETS_PATH, HandSide
from bigym.robots.config import (
    ArmConfig,
    FloatingBaseConfig,
    GripperConfig,
    RobotConfig,
)
from bigym.robots.robot import Robot
from bigym.utils.dof import Dof

ALOHA_ACTUATORS = {
    "left_waist": True,
    "left_shoulder": True,
    "left_elbow": True,
    "left_forearm_roll": True,
    "left_wrist_angle": True,
    "left_wrist_rotate": True,
    "left_left_finger": True,
    "left_right_finger": True,
    "right_waist": True,
    "right_shoulder": True,
    "right_elbow": True,
    "right_forearm_roll": True,
    "right_wrist_angle": True,
    "right_wrist_rotate": True,
    "right_left_finger": True,
    "right_right_finger": True,
}

ALOHA_LEFT_ARM = ArmConfig(
    site="left_gripper",
    links=[
        "left_shoulder_link",
        "left_upper_arm_link",
        "left_upper_forearm_link",
        "left_lower_forearm_link",
        "left_wrist_link",
        "left_gripper_link",
    ],
)

ALOHA_RIGHT_ARM = ArmConfig(
    site="right_gripper",
    links=[
        "right_shoulder_link",
        "right_upper_arm_link",
        "right_upper_forearm_link",
        "right_lower_forearm_link",
        "right_wrist_link",
        "right_gripper_link",
    ],
)
STIFFNESS = 1e4
# Define a minimal FloatingBaseConfig for the fixed-base robot
ALOHA_FLOATING_BASE = FloatingBaseConfig(
    dofs={},  
    delta_range_position=(0, 0),  # No movement allowed
    delta_range_rotation=(0, 0),  # No rotation allowed
    offset_position=np.array([0,0,0]),  # Adjusted based on base position in XML
)

ALOHA_GRIPPER = GripperConfig(
    model=ASSETS_PATH / "aloha/aloha_gripper.xml",
    body="gripper_base",
    pinch_site="finger_left_site",
    actuators=["gripper_actuator"],
    pad_bodies=["finger_left", "finger_right"],
    range=np.array([0.002, 0.037]),
)

ALOHA = RobotConfig(
    model=ASSETS_PATH / "aloha/scene.xml",  # Update this path if necessary
    delta_range=(-0.1, 0.1),
    position_kp=300,
    pelvis_body="center",  # Use the left base link as the "pelvis" for a fixed-base robot
    floating_base=ALOHA_FLOATING_BASE,  # Include the minimal FloatingBaseConfig
    gripper=ALOHA_GRIPPER,
    arms={HandSide.LEFT: ALOHA_LEFT_ARM, HandSide.RIGHT: ALOHA_RIGHT_ARM},
    actuators=ALOHA_ACTUATORS,
    cameras=["teleoperator_pov","collaborator_pov", "wrist_cam_left", "wrist_cam_right", "overhead_cam"],
    namespaces_to_remove=["light"],
)


from bigym.action_modes import AlohaPositionActionMode

class AlohaRobot(Robot):
    """ALOHA Robot."""

    @property
    def config(self) -> RobotConfig:
        """Get robot config."""
        return ALOHA

    def get_action_mode(self):
        return AlohaPositionActionMode(
            floating_base=False,
            absolute=False,
            control_all_joints=True
        )