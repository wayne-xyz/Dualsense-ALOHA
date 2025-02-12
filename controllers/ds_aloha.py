import pydualsense
import numpy as np
import numpy as np
import mujoco
import mujoco.viewer
import mink
import h5py
import time
from bigym.envs.dishwasher import DishwasherClose
from bigym.action_modes import AlohaPositionActionMode
from bigym.utils.observation_config import ObservationConfig, CameraConfig
from bigym.robots.configs.aloha import AlohaRobot
from mink import SO3
from reduced_configuration import ReducedConfiguration
from loop_rate_limiters import RateLimiter

# 
_JOINT_NAMES = [
    "waist",
    "shoulder",
    "elbow",
    "forearm_roll",
    "wrist_angle",
    "wrist_rotate",
]

_VELOCITY_LIMITS = {k: np.pi for k in _JOINT_NAMES}


class DSAlohaMocapControl:
    def __init__(self):
        # initialize ds controller
        self.ds_controller=pydualsense.pydualsense()
        self.ds_controller.init()

        # initialize env with action mode and observation setting 
        # this env is based on the nintend-aloha , an example of task in bigym- DishawasherClose
        self.env = DishwasherClose(
            action_mode=AlohaPositionActionMode(floating_base=False, absolute=False, control_all_joints=True),
            observation_config=ObservationConfig(
                cameras=[
                    CameraConfig(name="wrist_cam_left", rgb=True, depth=False, resolution=(480, 640)),
                    CameraConfig(name="wrist_cam_right", rgb=True, depth=False, resolution=(480, 640)),
                    CameraConfig(name="overhead_cam", rgb=True, depth=False, resolution=(480, 640)),
                    CameraConfig(name="teleoperator_pov", rgb=True, depth=False, resolution=(480, 640)),
                ],
            ),
            render_mode="human",
            robot_cls=AlohaRobot
        )

        self.env.reset()
        self.model = self.env.unwrapped._mojo.model
        self.data = self.env.unwrapped._mojo.data



        