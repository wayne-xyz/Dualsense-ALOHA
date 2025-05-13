import numpy as np
# Override numpy's random.randint to use int64 and return Python int
original_randint = np.random.randint
def new_randint(*args, **kwargs):
    if 'dtype' not in kwargs:
        kwargs['dtype'] = np.int64
    return int(original_randint(*args, **kwargs))  # Convert to Python int
np.random.randint = new_randint

import pydualsense
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
from mujoco import MjModel, MjData
import logging

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
    def __init__(self, save_data=False):
        # initialize ds controller
        self.ds_controller = pydualsense.pydualsense()
        self.controller_connected = False
        self.save_data = save_data  # Flag to control whether to save data

        try:
            self.ds_controller.init()
            if self.ds_controller.conType==pydualsense.enums.ConnectionType.BT:
                print("DualSense controller connected via Bluetooth")
            else:
                print("DualSense controller connected via USB")
            self.controller_connected = True
        except Exception as e:
            print(f"Failed to connect to DualSense controller: {e}")
            self.controller_connected = False

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
        self.model = self.env.unwrapped._mojo.model # Mjmodel type
        self.data = self.env.unwrapped._mojo.data # Mjdata type
        
        # Initialize camera renderers
        self.camera_renderers = {}
        for camera_config in self.env.observation_config.cameras:
            camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_config.name)
            renderer = mujoco.Renderer(self.model, camera_config.resolution[0], camera_config.resolution[1])
            self.camera_renderers[camera_config.name] = (renderer, camera_id)        

        # Set initial target positions for left and right end-effectors. target is the aloha arms
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
        
        # Calibrate the dualsense controller sensors.
        if self.controller_connected:
            self.calibrate()

        # Only initialize HDF5 storage if data saving is enabled
        if self.save_data:
            print("Save mode is on, data will be saved to hdf5 file")
            self.initialize_hdf5_storage()
        else:
            print("Save mode is off, data will not be saved to hdf5 file")

        # Initialize the action vector (14-dimensional: 7 for left, 7 for right).
        self.action = np.zeros(14)

        # Prepare joint names and velocity limits for both arms. Create the name lists and velocity limits for the left and right arms
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
        into the data dictionary. Only stores if save_data is enabled.
        """
        if not self.save_data:
            return
            
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
        Only saves if save_data is enabled and there is data to save.
        """
        if not self.save_data or self.num_timesteps == 0:
            return
            
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

    # change the original code to compatible with the dualsense controller,pydualsense
    def calibrate(self):
        """
        Calibrate the dualsense controllers by averaging a number of sensor readings
        ( acc.x, acc.y, acc.z, gyro.pitch, gyro.yaw, gyro.roll, left-stick, right-stick)
        This offset is then used to adjust raw inputs.
        """

        num_samples=100
        right_samples=[]
        left_samples=[]
        for _ in range(num_samples):
            state=self.ds_controller.state
            acc=state.accelerometer
            gyro=state.gyro
            left_samples.append([acc.X, acc.Y, acc.Z, gyro.Pitch, gyro.Yaw, gyro.Roll, state.LX, state.LY])
            right_samples.append([acc.X, acc.Y, acc.Z, gyro.Pitch, gyro.Yaw, gyro.Roll, state.RX, state.RY])
            time.sleep(0.005)

        self.left_calibrated_offset=np.mean(left_samples, axis=0)
        self.right_calibrated_offset=np.mean(right_samples, axis=0)

    def so3_to_matrix(self, so3_rotation: SO3) -> np.ndarray:
        """
        Convert an SO3 rotation object to a 3x3 rotation matrix.
        :param so3_rotation: SO3 rotation object
        :return: 3x3 rotation matrix
        """
        return so3_rotation.as_matrix()    

    def matrix_to_so3(self, rotation_matrix: np.ndarray) -> SO3:
        """
        Convert a 3x3 rotation matrix to an SO3 rotation object.
        :param rotation_matrix: 3x3 rotation matrix
        :return: SO3 rotation object
        """
        return SO3.from_matrix(rotation_matrix)

    def apply_rotation(self, current_rotation: SO3, rotation_change: np.ndarray) -> SO3:
        """
        Apply an incremental rotation (given as a 3-vector in the Lie algebra) to the current rotation.
        :param current_rotation: SO3 rotation object
        :param rotation_change: 3-vector in the Lie algebra
        :return: SO3 rotation object
        """
        rotation_matrix = self.so3_to_matrix(current_rotation)
        change_matrix = SO3.exp(rotation_change).as_matrix()
        new_rotation_matrix = change_matrix @ rotation_matrix
        return self.matrix_to_so3(new_rotation_matrix)

    def update_rotation(self, axis: str, angle: float, side: str):
        """
        Update the rotation of the specified side ('left' or 'right') by applying a rotation about the given axis.
        :param axis: 'x', 'y', or 'z'
        :param angle: rotation angle in radians
        :param side: 'left' or 'right'
        """

        rotation_change = np.zeros(3)
        if axis == 'x':
            rotation_change[0] = angle
        elif axis == 'y':
            rotation_change[1] = angle
        elif axis == 'z':
            rotation_change[2] = angle

        if side == 'left':
            self.rot_l = self.apply_rotation(self.rot_l, rotation_change)
        else:
            self.rot_r = self.apply_rotation(self.rot_r, rotation_change)
        self.targets_updated = True  # Mark that the target orientation has been updated.

    def ds_controller_l_update(self):
        """
        Update the action vector based on the state of the left DS controller.
        Map the ds controller to the action vector, and update the action vector. action0-6 are for the left arm, action7-13 are for the right arm
        The action vector is 14-dimensional: 7 for left, 7 for right.        
        """
        state=self.ds_controller.state
        rotation=state.gyro
        button_lower=state.DpadLeft # ds controller : True or False, set the target as a nagtive increment, (-0.03), 
        button_higher=state.DpadRight # ds controller : True or False, set the target as a positive increment, (0.03)
        joystick_lx=state.LX
        joystick_ly=state.LY
        button_up=state.DpadUp # ds controller : True or False, set the target as a positive increment, (0.037)
        button_down=state.DpadDown # ds controller : True or False, set the target as a nagtive increment, (0)

        self.action[0]=(joystick_lx-self.left_calibrated_offset[6])* 0.0001
        self.action[1]=(-joystick_ly-self.left_calibrated_offset[7])* 0.0001
        # Dualsense controller raw data range: -32768 to 32767 for gyro ( pydualsense), not set range for index, but set the velocity
        self.action[2]=-0.03 if button_down else 0.03 if button_up else 0
        self.action[3]=(rotation.Pitch-self.left_calibrated_offset[3])*0.000005 
        self.action[4]=(rotation.Roll-self.left_calibrated_offset[5])*0.000005
        self.action[5]=(rotation.Yaw-self.left_calibrated_offset[4])*0.000005
        #self.action[6]=0.037 if button_up else 0.002 if button_down else 0  # gripper position pending

        self.target_l[0]+=self.action[0]
        self.target_l[1]+=self.action[1]
        self.target_l[2]+=self.action[2]
        
        # set the target position within the safe range
        self.target_l = np.clip(self.target_l, [self.x_min, self.y_min, self.z_min], [self.x_max, self.y_max, self.z_max])
        
        # update the rotation of the left arm
        if state.L1:
            self.update_rotation('x', self.action[3], 'left')
            self.update_rotation('y', self.action[4], 'left')
            self.update_rotation('z', self.action[5], 'left')

        #gripper position
        self.left_gripper_pos=self.action[6]
        self.targets_updated=True

    def ds_controller_r_update(self):
        """
        Update the action vector based on the state of the right DS controller.
        Map the ds controller to the action vector, and update the action vector. action7-13 are for the right arm
        The action vector is 14-dimensional: 7 for left, 7 for right.        
        """
        state=self.ds_controller.state
        rotation= state.gyro
        button_lower=state.square
        button_higher=state.circle
        joystick_rx=state.RX
        joystick_ry=state.RY
        button_up=state.triangle
        button_down=state.cross


        self.action[7]=(joystick_rx-self.right_calibrated_offset[6])* 0.0001
        self.action[8]=(-joystick_ry-self.right_calibrated_offset[7])* 0.0001

        self.action[9]=-0.03 if button_down else 0.03 if button_up else 0
        self.action[10]=(rotation.Pitch-self.left_calibrated_offset[3])*0.000005 
        self.action[11]=(rotation.Roll-self.left_calibrated_offset[5])*0.000005
        self.action[12]=(rotation.Yaw-self.left_calibrated_offset[4])*0.000005
        #self.action[13]=0.037 if button_up else 0.002 if button_down else 0

        self.target_r[0]+=self.action[7]
        self.target_r[1]+=self.action[8]
        self.target_r[2]+=self.action[9]

        self.target_r = np.clip(self.target_r, [self.x_min, self.y_min, self.z_min], [self.x_max, self.y_max, self.z_max])
        
        # rotation
        if state.R1: 
            self.update_rotation('x', self.action[10], 'right')
            self.update_rotation('y', self.action[11], 'right')
            self.update_rotation('z', self.action[12], 'right')

        # gripper position
        self.right_gripper_pos=self.action[13]
        self.targets_updated=True


    # update the gripper function with the adapative trigger
    def control_gripper(self, left_gripper_pos, right_gripper_pos):
        """
        Update the gripper positions based on the state of the DS controllers.
        """
        left_gripper_position=np.clip(left_gripper_pos, 0.02, 0.037)
        right_gripper_position=np.clip(right_gripper_pos, 0.02, 0.037)
        self.data.ctrl[self.left_gripper_actuator_id]=left_gripper_position # mujoco's data ctrl is the control signal , when update, send command to simulate
        self.data.ctrl[self.right_gripper_actuator_id]=right_gripper_position


    
    def add_target_sites(self):
        """
        Retrieve site IDs for target markers in the simulation and update their positions.
        """
        self.target_site_id_l=self.model.site('aloha_scene/target').id # target: left arm
        self.target_site_id_r=self.model.site('aloha_scene/target2').id # taget2: right arm
        self.update_target_sites(self.target_l, self.target_r, self.rot_l, self.rot_r) # target represent the desired position , 

    
    
    def update_target_sites(self,target_l,target_r,rot_l,rot_r):
        """
        Update the simulation target sites with the current target positions and orientations.
        :param target_l: desired position of left arm
        :param target_r: desired position of right arm
        :param rot_l: desired orientation of left arm
        :param rot_r: desired orientation of right arm
        """
        self.data.site_xpos[self.target_site_id_l] = target_l # current, runtime world position 
        self.model.site_pos[self.target_site_id_l] = target_l # mujoco's default. nominal position
        self.data.site_xpos[self.target_site_id_r] = target_r
        self.model.site_pos[self.target_site_id_r] = target_r

        rot_l_matrix_flat = rot_l.as_matrix().flatten()
        rot_r_matrix_flat = rot_r.as_matrix().flatten()

        self.data.site_xmat[self.target_site_id_l] = rot_l_matrix_flat
        self.data.site_xmat[self.target_site_id_r] = rot_r_matrix_flat      
    
    def run(self):

        model=self.model
        data=self.data
 
        # Define tasks for left and right end-effectors(both position and orientation)
        # cost parameter: minimize errors in position and orientation, damping factor
        l_ee_task = mink.FrameTask(
                frame_name="aloha_scene/left_gripper",
                frame_type="site",
                position_cost=1.0,
                orientation_cost=1.0,
                lm_damping=1.0,
            )

        r_ee_task = mink.FrameTask(
                frame_name="aloha_scene/right_gripper",
                frame_type="site",
                position_cost=1.0,
                orientation_cost=1.0,
                lm_damping=1.0,
            )

        # Define the collision avoidance limit
        collision_avoidance_limit = mink.CollisionAvoidanceLimit(
            model=model,
            geom_pairs=[],
            minimum_distance_from_collisions=0.1,
            collision_detection_distance=0.1,
        )
        limits = [
            mink.VelocityLimit(model, self.velocity_limits),
            collision_avoidance_limit,
        ]

        solver = "osqp" # solver for the inverse kinematics solver
        max_iters = 20 # maximum number of iterations for the inverse kinematics solver

        # Launch the viewer with the specified UI settings 
        try:
            with mujoco.viewer.launch_passive(
                model=model, data=data, show_left_ui=True, show_right_ui=False
            ) as viewer:
                mujoco.mjv_defaultFreeCamera(model, viewer.cam)

                self.add_target_sites()
                mujoco.mj_forward(model, data)

                l_target_pose = mink.SE3.from_rotation_and_translation(self.rot_l, self.target_l)
                r_target_pose = mink.SE3.from_rotation_and_translation(self.rot_r, self.target_r)

                l_ee_task.set_target(l_target_pose)
                r_ee_task.set_target(r_target_pose)

                sim_rate = RateLimiter(frequency=200.0) 

                # original plan: data recording should be 50hz, 
                # loop is currently 200hz, thus record every 4th loop

                # changing to 5hz since data collection makes the sim too slow
                data_recording_interval = 40
                iters = 0

                # main loop for the simulation 
                while viewer.is_running():
                    self.ds_controller_l_update()
                    self.ds_controller_r_update()

                    self.control_gripper(self.left_gripper_pos, self.right_gripper_pos)
                    if self.targets_updated:
                        l_target_pose = mink.SE3.from_rotation_and_translation(self.rot_l, self.target_l)
                        l_ee_task.set_target(l_target_pose)
                        r_target_pose = mink.SE3.from_rotation_and_translation(self.rot_r, self.target_r)
                        r_ee_task.set_target(r_target_pose)

                        self.update_target_sites(self.target_l, self.target_r, self.rot_l, self.rot_r)
                        self.targets_updated = False

                    for _ in range(max_iters):
                        left_vel = mink.solve_ik(
                            self.left_configuration,
                            [l_ee_task],
                            sim_rate.dt,
                            solver,
                            limits=limits,
                            damping=1e-1,
                        )

                        right_vel = mink.solve_ik(
                            self.right_configuration,
                            [r_ee_task],
                            sim_rate.dt,
                            solver,
                            limits=limits,
                            damping=1e-1,
                        )

                        self.left_configuration.integrate_inplace(left_vel, sim_rate.dt)
                        self.right_configuration.integrate_inplace(right_vel, sim_rate.dt)

                        data.qpos[self.left_relevant_qpos_indices] = self.left_configuration.q
                        data.qpos[self.right_relevant_qpos_indices] = self.right_configuration.q

                        data.qvel[self.left_relevant_qvel_indices] = self.left_configuration.dq
                        data.qvel[self.right_relevant_qvel_indices] = self.right_configuration.dq

                        data.ctrl[self.left_actuator_ids] = self.left_configuration.q
                        data.ctrl[self.right_actuator_ids] = self.right_configuration.q

                        self.control_gripper(self.left_gripper_pos, self.right_gripper_pos)

                        mujoco.mj_step(model, data)

                        iters += 1
                        if iters == data_recording_interval:
                            self.store_data()
                            iters = 0

                        viewer.sync()
                        sim_rate.sleep()

                    if self.num_timesteps == 100:
                        sim_rate.sleep()
                        break
                    
        except KeyboardInterrupt:
            pass
        finally:
            # #print small subset of image data
            # print(f"shape of wrist_cam_left: {np.array(self.data_dict['/observations/images/wrist_cam_left']).shape}")
            # print(self.data_dict['/observations/images/wrist_cam_left'][0] - self.data_dict['/observations/images/wrist_cam_left'][-1])
            # print(f"sum: {np.sum(self.data_dict['/observations/images/wrist_cam_left'][0] - self.data_dict['/observations/images/wrist_cam_left'][-1])}")
            if self.controller_connected:
                self.ds_controller.close()
            self.close()


    def close(self):
        # Only save data if save_data is enabled and there is data to save
        if self.save_data and self.num_timesteps > 0:
            self.final_save()
        for renderer, _ in self.camera_renderers.values():
            renderer.close()


if __name__ == "__main__":
    try: 
        print("Starting DSAlohaMocapControl")
        control = DSAlohaMocapControl()
        print("control start running")
        control.run()    
    except KeyboardInterrupt:
        control.close()
