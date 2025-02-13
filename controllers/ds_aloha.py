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
import os


# Define joint names and their corresponding velocity limits.
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

    # initialize the control( ds controller and env, from the nintendo aloha)
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

        # Retrieve the Mujoco model and simulation data.
        self.model = self.env.unwrapped._mojo.model
        self.data = self.env.unwrapped._mojo.data
        
        # Initialize camera renderers
        self.camera_renderers = {}
        for camera_config in self.env.observation_config.cameras:
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_config.name)
            renderer = mujoco.Renderer(self.model, camera_config.resolution[0], camera_config.resolution[1])
            self.camera_renderers[camera_config.name] = (renderer, camera_id)        

        # Set initial target positions for left and right end-effectors.
        self.target_l = np.array([-0.4, 0.5, 1.1])
        self.target_r = np.array([0.4, 0.5, 1.1])

        # Initialize orientations as SO3 objects.
        self.rot_l = SO3.identity()
        self.rot_r = SO3.from_matrix(-np.eye(3))

        # Apply initial rotation offsets to orient the end-effectors correctly.
        self.update_rotation('z', np.pi/2, 'left')
        self.update_rotation('z', -np.pi/2, 'right')
        self.update_rotation('y', np.pi/2, 'left')
        self.update_rotation('y', np.pi/2, 'right')

        self.targets_updated = False  # Flag to signal that target poses have changed.

        # Define limits for target positions.
        self.x_min, self.x_max = -0.6, 0.6
        self.y_min, self.y_max = -0.6, 0.6
        self.z_min, self.z_max = 0.78, 1.6

        # Retrieve actuator IDs for left and right grippers.
        self.left_gripper_actuator_id = self.model.actuator("aloha_scene/aloha_gripper_left/gripper_actuator").id
        self.right_gripper_actuator_id = self.model.actuator("aloha_scene/aloha_gripper_right/gripper_actuator").id

        # Set initial gripper positions.
        self.left_gripper_pos = 0.037
        self.right_gripper_pos = 0.037

        self.num_timesteps = 0  # Counter for data recording timesteps.
        
        # Calibrate the JoyCon sensors.
        # TODO: Calibrate the JoyCon sensors, and initialize the hdf5 storage
        # self.calibrate()
        # self.initialize_hdf5_storage()

        # Initialize the action vector (14-dimensional: 7 for left, 7 for right).
        self.action = np.zeros(14)

        # Prepare joint names and velocity limits for both arms.
        self.left_joint_names = []
        self.right_joint_names = []
        self.velocity_limits = {}
        for n in _JOINT_NAMES:
            name_left = f"aloha_scene/left_{n}"
            name_right = f"aloha_scene/right_{n}"
            self.left_joint_names.append(name_left)
            self.right_joint_names.append(name_right)
            self.velocity_limits[name_left] = _VELOCITY_LIMITS[n]
            self.velocity_limits[name_right] = _VELOCITY_LIMITS[n]

        model=self.model
        data=self.data

        # Get indices for joint degrees of freedom, actuator IDs, and positions/velocities for left arm.
        # TODO: add the reduced configuration for the left and right arms
        self.left_dof_ids = np.array([model.joint(name).id for name in self.left_joint_names])
        self.left_actuator_ids = np.array([model.actuator(name).id for name in self.left_joint_names])
        self.left_relevant_qpos_indices = np.array([model.jnt_qposadr[model.joint(name).id] for name in self.left_joint_names])
        self.left_relevant_qvel_indices = np.array([model.jnt_dofadr[model.joint(name).id] for name in self.left_joint_names])
        
        # Create a reduced configuration for the left arm for inverse kinematics.
        self.left_configuration = ReducedConfiguration(model, data, self.left_relevant_qpos_indices, self.left_relevant_qvel_indices)
        
        # Similarly, set up for the right arm.
        self.right_dof_ids = np.array([model.joint(name).id for name in self.right_joint_names])
        self.right_actuator_ids = np.array([model.actuator(name).id for name in self.right_joint_names])
        self.right_relevant_qpos_indices = np.array([model.jnt_qposadr[model.joint(name).id] for name in self.right_joint_names])
        self.right_relevant_qvel_indices = np.array([model.jnt_dofadr[model.joint(name).id] for name in self.right_joint_names])
        self.right_configuration = ReducedConfiguration(model, data, self.right_relevant_qpos_indices, self.right_relevant_qvel_indices)


    def initialize_hdf5_storage(self):
        """
        Set up the HDF5 storage structure for recording simulation data.
        Each episode's data is stored in a separate file in the 'data' directory.
        """
        self.dataset_dir = 'data'
        self.data_dict = {
            '/observations/qpos': [],
            '/observations/qvel': [],
            '/action': [],
        }
        # Set up storage for images from all cameras.
        self.camera_names = ['wrist_cam_left', 'wrist_cam_right', 'overhead_cam', 'teleoperator_pov']
        for cam_name in self.camera_names:
            self.data_dict[f'/observations/images/{cam_name}'] = []

    def store_data(self):
        """
        Record the current simulation data (joint positions, velocities, actions, and camera images)
        into the data dictionary.
        """
        self.data_dict['/observations/qpos'].append(self.get_qpos())
        self.data_dict['/observations/qvel'].append(self.get_qvel())
        self.data_dict['/action'].append(self.get_action())
        for cam_name in self.camera_names:
            self.data_dict[f'/observations/images/{cam_name}'].append(self.get_obs(cam_name))
        self.num_timesteps += 1

    def get_qpos(self):
        """
        Retrieve the current joint positions (qpos) for both arms and concatenate with gripper positions.
        """
        self.l_qpos = self.data.qpos[self.left_relevant_qpos_indices]
        self.r_qpos = self.data.qpos[self.right_relevant_qpos_indices]
        qpos = np.concatenate((self.l_qpos, [self.left_gripper_pos], self.r_qpos, [self.right_gripper_pos]), axis=0)
        return qpos
    
    def get_qvel(self):
        """
        Retrieve the current joint velocities (qvel) for both arms.
        Additionally, compute gripper velocities based on action inputs.
        """
        left_gripper_vel = 1 if self.action[6] > 0 else -1 if self.action[6] < 0 else 0
        right_gripper_vel = 1 if self.action[13] > 0 else -1 if self.action[13] < 0 else 0
        self.l_qvel = self.data.qvel[self.left_relevant_qvel_indices]
        self.r_qvel = self.data.qvel[self.right_relevant_qvel_indices]
        qvel = np.concatenate((self.l_qvel, [left_gripper_vel], self.r_qvel, [right_gripper_vel]), axis=0)
        return qvel
    
    def get_action(self):
        """
        Return a copy of the current action command.
        """
        return self.action.copy()
    
    def get_obs(self, cam_name):
        """
        Render and return the image from the specified camera.
        """
        renderer, cam_id = self.camera_renderers[cam_name]
        renderer.update_scene(self.data, cam_id)
        img = renderer.render()
        return img
    
    def final_save(self):
        """
        Save the collected simulation data into an HDF5 file.
        Each episode is stored as a separate file with the episode index in the filename.
        """
        episode_idx = 16  # Note: Update this index for new episodes.
        t0 = time.time()
        dataset_path = os.path.join(self.dataset_dir, f'episode_{episode_idx}')
        with h5py.File(dataset_path + '.hdf5', 'w', rdcc_nbytes=1024 ** 2 * 2) as root:
            root.attrs['sim'] = True
            obs = root.create_group('observations')
            image = obs.create_group('images')
            # Create datasets for each camera image stream.
            for cam_name in self.camera_names:
                _ = image.create_dataset(cam_name, (self.num_timesteps, 480, 640, 3), dtype='uint8',
                                         chunks=(1, 480, 640, 3))
            # Create datasets for qpos, qvel, and action.
            qpos = obs.create_dataset('qpos', (self.num_timesteps, 14))
            qvel = obs.create_dataset('qvel', (self.num_timesteps, 14))
            action = root.create_dataset('action', (self.num_timesteps, 14))
            # Save each piece of data into its respective dataset.
            for name, array in self.data_dict.items():
                print(f"shape of {name}: {np.array(array).shape}")
                print(f"root[name]: {root[name]}")
                root[name][...] = array[0:self.num_timesteps + 1]
            print(f'Saving: {time.time() - t0:.1f} secs\n')


    