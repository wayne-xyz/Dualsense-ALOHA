# from https://github.com/AlmondGod/Nintendo-Aloha/blob/master/envs/aloha_box.py
"""An example of using BiGym with pixels for the ALOHA Robot."""
from bigym.action_modes import AlohaPositionActionMode
from bigym.envs.pick_and_place import StoreBox, PickBox
from bigym.utils.observation_config import ObservationConfig, CameraConfig
from bigym.robots.configs.aloha import AlohaRobot  
import numpy as np

import os
os.environ['MUJOCO_GL'] = 'glfw' 

print("Running 1000 steps with visualization...")
env = PickBox(
    action_mode=AlohaPositionActionMode(floating_base=False, absolute=False, control_all_joints=True),
    observation_config=ObservationConfig(
        cameras=[
            CameraConfig(name="wrist_cam_left", rgb=True, depth=False, resolution=(128, 128)),
            CameraConfig(name="wrist_cam_right", rgb=True, depth=False, resolution=(128, 128)),
            CameraConfig(name="overhead_cam", rgb=True, depth=False, resolution=(1280, 720)),
        ],
    ),
    render_mode="human",
    robot_cls=AlohaRobot
)

action = env.action_space.sample()

action = (np.ones_like(action) / 10 ) - 0.01
action[-1] = 0.01
action[-2] = 0.01

env.reset()

# env.reset_pose() there is no reset_pose in aloha env
for i in range(1000):
    obs, reward, terminated, truncated, info = env.step(action)
    env.render()

    if i < 2 or i % 200 == 0:
        print("iteration ", i, " joint position:", env.unwrapped._robot.qpos)
    
    if terminated or truncated:
        env.reset()

env.close()