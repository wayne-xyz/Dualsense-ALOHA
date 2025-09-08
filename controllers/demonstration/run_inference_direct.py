#!/usr/bin/env python3
"""
ACT Model Inference Script for ALOHA Robot
Runs trained ACT policy to perform imitation learning inference
"""

import numpy as np
import torch
import pickle
import time
import mujoco
import mujoco.viewer
import mink
from einops import rearrange
import cv2
import threading
import sys
import os

# Add demonstration folder to path for imports
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from policy import ACTPolicy
from constants import DT
from bigym.envs.dishwasher import DishwasherClose
from bigym.action_modes import AlohaPositionActionMode
from bigym.utils.observation_config import ObservationConfig, CameraConfig
from bigym.robots.configs.aloha import AlohaRobot
from mink import SO3
from mink.configuration import Configuration
from loop_rate_limiters import RateLimiter

# Add parent directory to path for ds_aloha imports
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from ds_aloha import _JOINT_NAMES, _VELOCITY_LIMITS

# ===== INFERENCE CONFIGURATION PARAMETERS =====
SIM_RATE = 50.0                    # Simulation frequency (Hz) - both model inference and arm movement frq
INFERENCE_STEP = 2000               # Total inference steps to run
IK_MAX_ITERS = 20                   # Maximum IK solver iterations per step

ENABLE_IMAGE_MONITORING = False       # Enable/disable image monitoring windows


# 700step Success
# Fail , washer move out of reach range 
# 300 steps , success






class ACTInference:
    def __init__(self, ckpt_path, dataset_stats_path=None):
        """
        Initialize ACT inference system
        
        Args:
            ckpt_path: Path to trained model checkpoint
            dataset_stats_path: Path to dataset statistics pickle file
        """
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        print(f"Using device: {self.device}")
        
        # Load model configuration (based on actual trained model)
        self.policy_config = {
            'lr': 5e-5,
            'num_queries': 15,  # chunk_size from training (actual value from checkpoint)
            'kl_weight': 50,
            'hidden_dim': 512,
            'dim_feedforward': 3200,
            'lr_backbone': 1e-5,
            'backbone': 'resnet18',
            'enc_layers': 4,
            'dec_layers': 7,
            'nheads': 8,
            'camera_names': ['wrist_cam_left', 'wrist_cam_right', 'overhead_cam', 'teleoperator_pov'],
        }


        
        # Load trained model
        self.policy = ACTPolicy(self.policy_config)
        checkpoint = torch.load(ckpt_path, map_location=self.device)
        self.policy.load_state_dict(checkpoint)
        self.policy.to(self.device)
        self.policy.eval()
        print(f"Loaded model from: {ckpt_path}")
        
        # Load dataset statistics for normalization
        if dataset_stats_path and os.path.exists(dataset_stats_path):
            with open(dataset_stats_path, 'rb') as f:
                stats = pickle.load(f)
            self.qpos_mean = torch.from_numpy(stats['qpos_mean']).float().to(self.device)
            self.qpos_std = torch.from_numpy(stats['qpos_std']).float().to(self.device)
            self.action_mean = torch.from_numpy(stats['action_mean']).float().to(self.device)
            self.action_std = torch.from_numpy(stats['action_std']).float().to(self.device)
            print("Loaded dataset statistics for normalization")
        else:
            # Use identity normalization if stats not available
            print("Warning: Using identity normalization - may affect performance")
            self.qpos_mean = torch.zeros(14).to(self.device)
            self.qpos_std = torch.ones(14).to(self.device)
            self.action_mean = torch.zeros(14).to(self.device)
            self.action_std = torch.ones(14).to(self.device)
        
        # Initialize environment (same setup as ds_aloha.py)
        self.setup_environment()
        
        # Direct inference - no temporal aggregation buffers needed
        self.num_loop_iters = 0
        
        # Image monitoring system
        self.image_monitoring = ENABLE_IMAGE_MONITORING
        self.monitor_windows_created = False
        self.latest_images = {}  # Store latest images for monitoring
        
        # Get current end-effector positions from the scene instead of hardcoded values
        # Forward kinematics to get current positions
        mujoco.mj_forward(self.model, self.data)
        
        # Get current positions of the gripper sites
        left_gripper_site_id = self.model.site("aloha_scene/left_gripper").id
        right_gripper_site_id = self.model.site("aloha_scene/right_gripper").id
        
        self.target_l = self.data.site_xpos[left_gripper_site_id].copy()
        self.target_r = self.data.site_xpos[right_gripper_site_id].copy()
        
        # Initialize orientations as SO3 objects
        self.rot_l = SO3.identity()
        self.rot_r = SO3.from_matrix(-np.eye(3))
        
        # Apply initial rotation offsets to orient the end-effectors correctly
        self.update_rotation('z', np.pi/2, 'left')
        self.update_rotation('z', -np.pi/2, 'right')
        self.update_rotation('y', np.pi/2, 'left')
        self.update_rotation('y', np.pi/2, 'right')
        
        self.targets_updated = False
        
        # Define limits for target positions
        self.x_min, self.x_max = -0.6, 0.6
        self.y_min, self.y_max = -0.6, 0.6
        self.z_min, self.z_max = 0.78, 1.6
        

    #     # solve the z-axis discrete , original action[2] and action[9] are +-0.03,0 , but the model is tiny value inference 
    #     self.Z_STEP = 0.03
    #     self.Z_THRESH = 0.0001  
    #     self.Z_COOLDOWN = 1      # in policy queries
    #     self._z_cool = [0, 0]    # [left, right]

    # def _discretize_z(self, action):
    #     for arm_idx, a_i in enumerate([2, 9]):  # z indices
    #         if self._z_cool[arm_idx] > 0:
    #             self._z_cool[arm_idx] -= 1
    #             action[a_i] = 0.0
    #             continue
    #         v = action[a_i]
    #         if abs(v) >= self.Z_THRESH:
    #             action[a_i] = np.sign(v) * self.Z_STEP
    #             self._z_cool[arm_idx] = self.Z_COOLDOWN
    #         else:
    #             action[a_i] = 0.0
    #     return action
        self.Z_GAIN = 18.0
        self._z_acc = [0.0, 0.0]          # [left, right]
        self.Z_ACC_THRESH = 0.45 * 0.03    # emit slightly before full step

    def _accumulate_z(self, action):
        for arm_idx, a_i in enumerate([2, 9]):
            self._z_acc[arm_idx] += action[a_i]
            if abs(self._z_acc[arm_idx]) >= self.Z_ACC_THRESH:
                pulse = np.sign(self._z_acc[arm_idx]) * 0.03
                action[a_i] = pulse
                self._z_acc[arm_idx] -= pulse
            else:
                action[a_i] = 0.0
        return action







    def setup_environment(self):
        """Setup BiGym environment identical to ds_aloha.py"""
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
        
        # Get Mujoco model and data
        self.model = self.env.unwrapped._mojo.model
        self.data = self.env.unwrapped._mojo.data
        
        # Set up joint control (same as ds_aloha.py)
        self.setup_joint_control()
        
        # Set up inverse kinematics system
        self.setup_inverse_kinematics()
        self.setup_ik_system()
        
    def setup_joint_control(self):
        """Setup joint control parameters identical to ds_aloha.py"""
        # Prepare joint names and indices
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
        
        # Get joint indices
        self.left_dof_ids = np.array([self.model.joint(name).id for name in self.left_joint_names])
        self.left_actuator_ids = np.array([self.model.actuator(name).id for name in self.left_joint_names])
        self.left_relevant_qpos_indices = np.array([self.model.jnt_qposadr[self.model.joint(name).id] for name in self.left_joint_names])
        self.left_relevant_qvel_indices = np.array([self.model.jnt_dofadr[self.model.joint(name).id] for name in self.left_joint_names])
        
        self.right_dof_ids = np.array([self.model.joint(name).id for name in self.right_joint_names])
        self.right_actuator_ids = np.array([self.model.actuator(name).id for name in self.right_joint_names])
        self.right_relevant_qpos_indices = np.array([self.model.jnt_qposadr[self.model.joint(name).id] for name in self.right_joint_names])
        self.right_relevant_qvel_indices = np.array([self.model.jnt_dofadr[self.model.joint(name).id] for name in self.right_joint_names])
        
        # Gripper actuators
        self.left_gripper_actuator_id = self.model.actuator("aloha_scene/aloha_gripper_left/gripper_actuator").id
        self.right_gripper_actuator_id = self.model.actuator("aloha_scene/aloha_gripper_right/gripper_actuator").id
        
    def setup_inverse_kinematics(self):
        """Setup inverse kinematics solver identical to ds_aloha.py"""
        # Create configurations for IK
        self.left_configuration = Configuration(model=self.model, q=self.data.qpos.copy())
        self.right_configuration = Configuration(model=self.model, q=self.data.qpos.copy())
        
        # Build velocity limits for IK
        self.full_velocity_limits = {}
        # for j in range(self.model.njnt):
        #     jtype = self.model.jnt_type[j]
        #     if jtype == mujoco.mjtJoint.mjJNT_HINGE or jtype == mujoco.mjtJoint.mjJNT_SLIDE:
        #         jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
        #         self.full_velocity_limits[jname] = 1e-6
        
        for n in _JOINT_NAMES:
            self.full_velocity_limits[f"aloha_scene/left_{n}"] = _VELOCITY_LIMITS[n]
            self.full_velocity_limits[f"aloha_scene/right_{n}"] = _VELOCITY_LIMITS[n]
        
        # IK tasks will be created in setup_ik_system()
        
        # Define limits
        collision_avoidance_limit = mink.CollisionAvoidanceLimit(
            model=self.model,
            geom_pairs=[],
            minimum_distance_from_collisions=0.1,
            collision_detection_distance=0.1,
        )
        
        self.limits = [
            mink.VelocityLimit(self.model, self.full_velocity_limits),
            collision_avoidance_limit,
        ]
        
        # IK solver parameters
        self.solver = "osqp"
        self.max_iters = IK_MAX_ITERS
        
    def setup_ik_system(self):
        """Setup IK tasks and solver configuration (called once in __init__)"""
        # Define IK tasks for end-effectors
        self.l_ee_task = mink.FrameTask(
            frame_name="aloha_scene/left_gripper",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        
        self.r_ee_task = mink.FrameTask(
            frame_name="aloha_scene/right_gripper", 
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        
        print("IK system initialized with tasks for both arms")
        
    def update_ik_targets(self):
        """Update IK target poses based on current target_l, target_r, rot_l, rot_r"""
        l_target_pose = mink.SE3.from_rotation_and_translation(self.rot_l, self.target_l)
        r_target_pose = mink.SE3.from_rotation_and_translation(self.rot_r, self.target_r)
        self.l_ee_task.set_target(l_target_pose)
        self.r_ee_task.set_target(r_target_pose)
        self.update_target_sites(self.target_l, self.target_r, self.rot_l, self.rot_r)
        
    def solve_ik_step(self, dt):
        """Solve IK for one timestep and update robot state"""
        for _ in range(self.max_iters):
            # Sync current robot state to both configurations
            self.left_configuration.update(self.data.qpos.copy())
            self.right_configuration.update(self.data.qpos.copy())
            
            # Solve IK for each arm independently
            left_vel = mink.solve_ik(
                self.left_configuration,
                [self.l_ee_task],
                dt,
                self.solver,
                limits=self.limits,
                damping=1e-1,
            )
            
            right_vel = mink.solve_ik(
                self.right_configuration,
                [self.r_ee_task],
                dt,
                self.solver,
                limits=self.limits,
                damping=1e-1,
            )
            
            # Apply only relevant joint velocities
            masked_left_vel = np.zeros(self.model.nv)
            masked_right_vel = np.zeros(self.model.nv)
            masked_left_vel[self.left_relevant_qvel_indices] = left_vel[self.left_relevant_qvel_indices]
            masked_right_vel[self.right_relevant_qvel_indices] = right_vel[self.right_relevant_qvel_indices]
            
            # Integrate each arm's solution independently
            self.left_configuration.integrate_inplace(masked_left_vel, dt)
            self.right_configuration.integrate_inplace(masked_right_vel, dt)
            
            # Update robot state
            self.data.qpos[self.left_relevant_qpos_indices] = self.left_configuration.q[self.left_relevant_qpos_indices]
            self.data.qpos[self.right_relevant_qpos_indices] = self.right_configuration.q[self.right_relevant_qpos_indices]
            
            self.data.qvel[self.left_relevant_qvel_indices] = masked_left_vel[self.left_relevant_qvel_indices]
            self.data.qvel[self.right_relevant_qvel_indices] = masked_right_vel[self.right_relevant_qvel_indices]
            
            # Drive actuators to current joint positions
            self.data.ctrl[self.left_actuator_ids] = self.data.qpos[self.left_relevant_qpos_indices]
            self.data.ctrl[self.right_actuator_ids] = self.data.qpos[self.right_relevant_qpos_indices]

                        # refresh grippers each iteration (keeps them clipped and consistent)
            self.data.ctrl[self.left_gripper_actuator_id]  = np.clip(self.data.ctrl[self.left_gripper_actuator_id],  0.02, 0.037)
            self.data.ctrl[self.right_gripper_actuator_id] = np.clip(self.data.ctrl[self.right_gripper_actuator_id], 0.02, 0.037)
                        # Step simulation
            mujoco.mj_step(self.model, self.data)
        
    def so3_to_matrix(self, so3_rotation: SO3) -> np.ndarray:
        """Convert an SO3 rotation object to a 3x3 rotation matrix."""
        return so3_rotation.as_matrix()    

    def matrix_to_so3(self, rotation_matrix: np.ndarray) -> SO3:
        """Convert a 3x3 rotation matrix to an SO3 rotation object."""
        return SO3.from_matrix(rotation_matrix)

    def apply_rotation(self, current_rotation: SO3, rotation_change: np.ndarray) -> SO3:
        """Apply an incremental rotation to the current rotation."""
        rotation_matrix = self.so3_to_matrix(current_rotation)
        change_matrix = SO3.exp(rotation_change).as_matrix()
        new_rotation_matrix = change_matrix @ rotation_matrix
        return self.matrix_to_so3(new_rotation_matrix)

    def update_rotation(self, axis: str, angle: float, side: str):
        """Update rotation by applying rotation about the given axis."""
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
        self.targets_updated = True
        
    def add_target_sites(self):
        """Add target sites for visualization."""
        self.target_site_id_l = self.model.site('aloha_scene/target').id
        self.target_site_id_r = self.model.site('aloha_scene/target2').id
        self.update_target_sites(self.target_l, self.target_r, self.rot_l, self.rot_r)
        
    def update_target_sites(self, target_l, target_r, rot_l, rot_r):
        """Update target site positions and orientations."""
        self.data.site_xpos[self.target_site_id_l] = target_l
        self.model.site_pos[self.target_site_id_l] = target_l
        self.data.site_xpos[self.target_site_id_r] = target_r
        self.model.site_pos[self.target_site_id_r] = target_r

        rot_l_matrix_flat = rot_l.as_matrix().flatten()
        rot_r_matrix_flat = rot_r.as_matrix().flatten()

        self.data.site_xmat[self.target_site_id_l] = rot_l_matrix_flat
        self.data.site_xmat[self.target_site_id_r] = rot_r_matrix_flat
        
    def get_qpos(self):
        """Get current joint positions for both arms and concatenate with gripper positions."""
        l_qpos = self.data.qpos[self.left_relevant_qpos_indices]
        r_qpos = self.data.qpos[self.right_relevant_qpos_indices]
        
        # Get current gripper positions from actuator controls
        left_gripper_pos = self.data.ctrl[self.left_gripper_actuator_id] 
        right_gripper_pos = self.data.ctrl[self.right_gripper_actuator_id]
        
        # Combine into 14-dim qpos vector
        qpos = np.concatenate((l_qpos, [left_gripper_pos], r_qpos, [right_gripper_pos]), axis=0)
        return qpos
        
    def get_obs(self, cam_name):
        """Get camera image using BiGym's observation system."""
        # Get full observation from BiGym
        obs = self.env.get_observation()
        
        # BiGym stores camera images with 'rgb_' prefix directly in observation
        rgb_key = f'rgb_{cam_name}'
        
        if rgb_key in obs:
            return obs[rgb_key]
        else:
            print(f"Camera '{cam_name}' (looking for '{rgb_key}') not found in observation")
            print(f"Available keys: {list(obs.keys())}")
            return None
        
    def get_observation(self):
        """Get current observation (qpos and images) from environment"""
        # Get robot state
        l_qpos = self.data.qpos[self.left_relevant_qpos_indices] 
        r_qpos = self.data.qpos[self.right_relevant_qpos_indices]
        
        # Get gripper positions (estimated from actuator controls)
        left_gripper_pos = self.data.ctrl[self.left_gripper_actuator_id] 
        right_gripper_pos = self.data.ctrl[self.right_gripper_actuator_id]
        
        # Combine into 14-dim qpos vector
        qpos = np.concatenate((l_qpos, [left_gripper_pos], r_qpos, [right_gripper_pos]), axis=0)
        
        # Get images from all cameras
        obs = self.env.get_observation()
        curr_images = []
        for cam_name in self.policy_config['camera_names']:
            rgb_key = f'rgb_{cam_name}'
            if rgb_key in obs:
                img = obs[rgb_key]
                
                # BiGym returns images in (C, H, W) format, convert to (H, W, C) to match training data
                if img.shape[0] == 3:  # (C, H, W) format from BiGym
                    img = rearrange(img, 'c h w -> h w c')  # Convert to (H, W, C)
                
                # Ensure correct shape: (H, W, C) = (480, 640, 3)
                if img.shape != (480, 640, 3):
                    print(f"Error: Image shape {img.shape} doesn't match expected (480, 640, 3)")
                
                # Keep in (H, W, C) format to match training data
                curr_images.append(img)
            else:
                print(f"Warning: Camera {cam_name} not found")
                curr_images.append(np.zeros((480, 640, 3), dtype=np.uint8))
        
        # Stack images and convert to torch tensor
        curr_image = np.stack(curr_images, axis=0)  # Shape: (4, 480, 640, 3) = (cameras, H, W, C)
        curr_image = torch.from_numpy(curr_image / 255.0).float().to(self.device)
        
        # Convert to (batch, cameras, C, H, W) format expected by model
        curr_image = curr_image.permute(0, 3, 1, 2)  # (4, 480, 640, 3) -> (4, 3, 480, 640)
        curr_image = curr_image.unsqueeze(0)  # (4, 3, 480, 640) -> (1, 4, 3, 480, 640)
        
        return qpos, curr_image
    
    def preprocess_observation(self, qpos):
        """Preprocess qpos observation using dataset statistics"""
        qpos_tensor = torch.from_numpy(qpos).float().to(self.device).unsqueeze(0)
        qpos_normalized = (qpos_tensor - self.qpos_mean) / self.qpos_std
        return qpos_normalized
    
    def postprocess_action(self, action_tensor):
        """Postprocess action from model output"""
        action_denormalized = action_tensor * self.action_std + self.action_mean
        return action_denormalized.squeeze(0).cpu().numpy()
    
    def select_action(self, policy, stats):
        """Select action using model (matching second author's approach)"""
        with torch.no_grad():
            try:
                # Preprocess inputs (simplified - matching second author)
                curr_image = self.get_image_for_policy()
                qpos_numpy = np.array(self.get_qpos())
                
                # Convert to tensor (no normalization like second author)
                qpos = torch.from_numpy(qpos_numpy).float().unsqueeze(0).to(self.device)
                
                # Get action from policy
                actions = policy(qpos, curr_image)
                if actions is None:
                    print("[DEBUG] Policy returned None for actions.")
                return actions
            except Exception as e:
                print(f"[ERROR] Policy inference failed: {e}")
                print(f"  qpos_numpy shape: {qpos_numpy.shape if 'qpos_numpy' in locals() else 'N/A'}")
                print(f"  curr_image shape: {curr_image.shape if 'curr_image' in locals() else 'N/A'}")
                import traceback
                traceback.print_exc()
                return None
    
    def get_image_for_policy(self):
        """Get images in format expected by policy"""
        curr_images = []
        for cam_name in self.policy_config['camera_names']:
            img = self.get_obs(cam_name)
            if img is not None:
                # Convert from BiGym format (H, W, C) to policy format (C, H, W)
                if img.shape[0] == 3:  # Already (C, H, W)
                    curr_image = img
                else:  # (H, W, C) -> (C, H, W)
                    curr_image = np.transpose(img, (2, 0, 1))
                curr_images.append(curr_image)
            else:
                curr_images.append(np.zeros((3, 480, 640), dtype=np.uint8))
        
        curr_image = np.stack(curr_images, axis=0)
        curr_image = torch.from_numpy(curr_image / 255.0).float().unsqueeze(0).to(self.device)
        
        # Update image monitoring if enabled
        if self.image_monitoring:
            self.update_image_monitoring(curr_images)
        
        return curr_image
    
    def create_monitoring_windows(self):
        """Create single OpenCV window with 2x2 grid for monitoring camera images"""
        if not self.image_monitoring:
            return
            
        # Create single window for all camera views
        self.monitor_window_name = "Camera Monitor - All Views"
        cv2.namedWindow(self.monitor_window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.monitor_window_name, 640, 480)  # 2x2 grid of 320x240 images
        
        self.monitor_windows_created = True
        print("Created unified image monitoring window")
    
    def update_image_monitoring(self, images_list):
        """Update single monitoring window with 2x2 grid of current images"""
        if not self.image_monitoring or not self.monitor_windows_created:
            return
        
        # Process all images and create composite
        processed_images = []
        
        for i, cam_name in enumerate(self.policy_config['camera_names']):
            if i < len(images_list):
                img = images_list[i]
                
                # Convert from policy format (C, H, W) to display format (H, W, C)
                if len(img.shape) == 3 and img.shape[0] == 3:
                    display_img = np.transpose(img, (1, 2, 0))
                else:
                    display_img = img
                
                # Convert to uint8 if needed
                if display_img.dtype == np.float32 or display_img.dtype == np.float64:
                    display_img = (display_img * 255).astype(np.uint8)
                
                # Convert RGB to BGR for OpenCV display
                if len(display_img.shape) == 3:
                    display_img = cv2.cvtColor(display_img, cv2.COLOR_RGB2BGR)
                
                # Resize to standard monitoring size
                display_img = cv2.resize(display_img, (320, 240))
                
                # Add camera name label
                cv2.putText(display_img, cam_name, (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                processed_images.append(display_img)
            else:
                # Create black placeholder if camera missing
                placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
                cv2.putText(placeholder, f"No {cam_name}", (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                processed_images.append(placeholder)
        
        # Ensure we have exactly 4 images (pad with black if needed)
        while len(processed_images) < 4:
            placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
            cv2.putText(placeholder, "Empty", (120, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
            processed_images.append(placeholder)
        
        # Create 2x2 grid composite image
        # Top row: cameras 0 and 1
        top_row = np.hstack([processed_images[0], processed_images[1]])
        # Bottom row: cameras 2 and 3  
        bottom_row = np.hstack([processed_images[2], processed_images[3]])
        # Combine rows
        composite_image = np.vstack([top_row, bottom_row])
        
        # Display composite image
        cv2.imshow(self.monitor_window_name, composite_image)
        
        # Non-blocking waitKey to refresh window
        cv2.waitKey(1)
    
    def close_monitoring_windows(self):
        """Close all monitoring windows"""
        if self.image_monitoring and self.monitor_windows_created:
            cv2.destroyAllWindows()
            print("Closed image monitoring windows")
    
    def is_dishwasher_closed(self) -> bool:
        """
        Check if the dishwasher is closed (door closed and trays pushed in).
        
        Returns:
            True if dishwasher is fully closed, False otherwise
        """
        try:
            # Get current state: [door, bottom_tray, middle_tray]
            # 0 = closed/pushed in, 1 = open/pulled out
            dishwasher_state = self.env.unwrapped.dishwasher.get_state()
            
            # Success condition: all joints should be at 0 (tolerance of 0.05)
            TOLERANCE = 0.05
            is_closed = np.allclose(dishwasher_state, 0, atol=TOLERANCE)
            
            return is_closed
        except Exception as e:
            print(f"Error checking dishwasher state: {e}")
            return False
    
    def get_dishwasher_state_details(self) -> dict:
        """
        Get detailed dishwasher state for debugging.
        
        Returns:
            Dictionary with door, bottom_tray, middle_tray states and closure status
        """
        try:
            state = self.env.unwrapped.dishwasher.get_state()
            return {
                'door': state[0],           # 0 = closed, 1 = open
                'bottom_tray': state[1],    # 0 = pushed in, 1 = pulled out  
                'middle_tray': state[2],    # 0 = pushed in, 1 = pulled out
                'is_closed': self.is_dishwasher_closed(),
                'raw_state': state
            }
        except Exception as e:
            print(f"Error getting dishwasher state: {e}")
            return {'error': str(e)}
    
    def execute_action(self, action, dt):
        """Execute predicted action using delta movement accumulation + IK (matching data collection)"""
        # Split action into components (matching ds_aloha.py action format)
        # Left arm: action[0:3] = translation, action[3:6] = rotation, action[6] = gripper
        # Right arm: action[7:10] = translation, action[10:13] = rotation, action[13] = gripper
        
        # Update target positions with DELTA movements (matching data collection scaling)
        # X,Y: continuous joystick movements (* 0.0003 in data collection)
        # Z: discrete button presses (±0.03 in data collection) 
        action[2] = self.Z_GAIN * action[2]
        action[9] = self.Z_GAIN * action[9]

        self.target_l[0] += action[0]  # += means accumulating (key difference!)
        self.target_l[1] += action[1]
        self.target_l[2] += action[2]

        self.target_l = np.clip(self.target_l, [self.x_min, self.y_min, self.z_min], [self.x_max, self.y_max, self.z_max])
        
        self.target_r[0] += action[7]
        self.target_r[1] += action[8] 
        self.target_r[2] += action[9]
        self.target_r = np.clip(self.target_r, [self.x_min, self.y_min, self.z_min], [self.x_max, self.y_max, self.z_max])
        
        # Skip rotation updates - model was trained with mostly static rotations
        # (rotation was conditional on L1/R1 buttons in data collection, so model learned rotation ≈ 0)
        # self.update_rotation('x', action[3], 'left')
        # self.update_rotation('y', action[4], 'left') 
        # self.update_rotation('z', action[5], 'left')
        # self.update_rotation('x', action[10], 'right')
        # self.update_rotation('y', action[11], 'right')
        # self.update_rotation('z', action[12], 'right')
        
        # Update gripper positions
        left_gripper_clipped = np.clip(action[6], 0.02, 0.037)
        right_gripper_clipped = np.clip(action[13], 0.02, 0.037)
        self.data.ctrl[self.left_gripper_actuator_id] = left_gripper_clipped
        self.data.ctrl[self.right_gripper_actuator_id] = right_gripper_clipped
        
        self.targets_updated = True
    
    def run_inference(self, max_timesteps=1000):
        """Run inference loop with IK solving (matching data collection)"""
        print("Starting inference...")
        
        try:
            with mujoco.viewer.launch_passive(
                model=self.model, data=self.data, show_left_ui=True, show_right_ui=False
            ) as viewer:
                mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)
                
                # Initialize simulation and IK targets
                self.add_target_sites()
                mujoco.mj_forward(self.model, self.data)
                
                # Set initial target poses using class IK tasks
                self.update_ik_targets()
                print("Initial IK targets set for both arms")
                
                sim_rate = RateLimiter(frequency=SIM_RATE)
                
                # Create image monitoring windows
                self.create_monitoring_windows()
                
                # Direct inference setup - query policy every step
                post_process = lambda a: a * self.action_std.cpu().numpy() + self.action_mean.cpu().numpy()
                
                print("=== DIRECT INFERENCE MODE ===")
                print(f"Running at {SIM_RATE}Hz with direct policy inference every step")
                print(f"No temporal aggregation - immediate action execution\n")
                
                step = 0
                action = None  # Will be set in first loop iteration
                
                while viewer.is_running() and step < max_timesteps:
                    self.num_loop_iters += 1
                    
                    # Direct policy inference every step
                    next_actions = self.select_action(self.policy, None)
                    raw_action = next_actions.squeeze(0).detach().cpu().numpy()
                    actions = post_process(raw_action)
                    
                    # Use first action from the chunk directly
                    action = actions[0]
                    
                    # Debug output every 50 steps
                    if step % 50 == 0:
                        print(f"\n[Step {step}] Direct inference:")
                        print(f"  Policy returned {len(actions)} actions (chunk size)")
                        print(f"  Using first action: pos_L={action[:3]}, pos_R={action[7:10]}")
                        print(f"  Z-values before discretization: L={action[2]:.6f}, R={action[9]:.6f}")
                    
                    # Execute action directly (updates target poses)
                    self.execute_action(action, sim_rate.dt)
                    if step % 50 == 0:
                        print(f"  Z-values after discretization: L={action[2]:.6f}, R={action[9]:.6f}")

                    # Update IK target poses if they changed
                    if self.targets_updated:
                        self.update_ik_targets()
                        self.targets_updated = False
                    
                    # Solve IK for both arms using helper method
                    self.solve_ik_step(sim_rate.dt)
                    
                    viewer.sync()
                    sim_rate.sleep()
                    
                    step += 1
                    
                    # Action is updated every step with direct inference
                    
        except KeyboardInterrupt:
            print("\nInference interrupted by user")
        finally:
            # Close monitoring windows
            self.close_monitoring_windows()
            
            # Ensure step is defined even if exception occurs early
            if 'step' in locals():
                print(f"Inference completed. Ran for {step} steps.")
            else:
                print("Inference completed. Error occurred during initialization.")


def main():
    """Main function to run ACT inference"""
    # Paths
    ckpt_path = "policy_best_3.ckpt"
    dataset_stats_path = "dataset_stats_3.pkl"  # Will be created when you run training
    
    # Check if checkpoint exists
    if not os.path.exists(ckpt_path):
        print(f"Error: Checkpoint file not found: {ckpt_path}")
        return
    
    # Initialize inference system
    print("Initializing ACT inference system...")
    inference = ACTInference(ckpt_path, dataset_stats_path)
    
    # Run inference - longer episode for more observation
    inference.run_inference(max_timesteps=INFERENCE_STEP)  # Run for 500 steps (~25 seconds at 20Hz)


if __name__ == "__main__":
    main()