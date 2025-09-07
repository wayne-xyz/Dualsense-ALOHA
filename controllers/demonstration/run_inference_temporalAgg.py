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
import csv

# Import image monitoring utilities
from image_monitor import ImageMonitor

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
SIM_RATE = 100.0                    # Simulation frequency (Hz) - matching original author
INFERENCE_STEP = 1000               # Total inference steps to run
POLICY_QUERY_INTERVAL = 3         # Query policy every N steps
IK_MAX_ITERS = 20                   # Maximum IK solver iterations per step
TEMPORAL_AGG_M = 0.1                # Temporal aggregation parameter m (smaller = faster new observation incorporation)
TEMPORAL_BUFFER_SIZE = 100           # Fixed length for single dynamic temporal aggregation buffer  
TEMPORAL_WINDOW_SIZE = 15            # Number of recent predictions to use for temporal aggregation
ENABLE_IMAGE_MONITORING = False      # Enable/disable image monitoring windows

Z_GAIN = 5



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
        
        # Per-timestep buckets for temporal aggregation across queries
        # Key: timestep index (int), Value: list of action predictions for that timestep (ordered oldest->newest)
        self.action_buckets = {}
        self.num_loop_iters = 0
        
        # Exponential weighting parameters (matching second author)
        self.temporal_agg_m = TEMPORAL_AGG_M  # m parameter from paper
        
        # Image monitoring system
        self.image_monitoring = ENABLE_IMAGE_MONITORING
        self.image_monitor = ImageMonitor(
            camera_names=self.policy_config['camera_names'],
            enabled=self.image_monitoring
        )
        
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
        





        # ===== Inference data logging (CSV) setup =====
        # Prepare output directory and file paths but delay opening files until run_inference
        self._prepare_logging_paths()
        self._last_executed_action = None
        # Track previous targets to compute effective deltas for debugging
        self._prev_target_l = self.target_l.copy()
        self._prev_target_r = self.target_r.copy()

    def _prepare_logging_paths(self):
        """Prepare infer-data directory and CSV file paths with parameterized names."""
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.infer_dir = os.path.join(base_dir, 'infer-data')
        os.makedirs(self.infer_dir, exist_ok=True)

        timestamp = time.strftime('%Y%m%d_%H%M%S')
        filename_suffix = f"{timestamp}_pqi{POLICY_QUERY_INTERVAL}_zg{Z_GAIN}_tw{TEMPORAL_WINDOW_SIZE}"

        self.policy_csv_path = os.path.join(self.infer_dir, f"policy_preds_{filename_suffix}.csv")
        self.exec_csv_path = os.path.join(self.infer_dir, f"executed_actions_{filename_suffix}.csv")

        self._csv_initialized = False

    def _init_csv_writers(self):
        """Open CSV files and write headers."""
        if self._csv_initialized:
            return
        self._policy_csv_file = open(self.policy_csv_path, 'w', newline='')
        self._exec_csv_file = open(self.exec_csv_path, 'w', newline='')
        self._policy_writer = csv.writer(self._policy_csv_file)
        self._exec_writer = csv.writer(self._exec_csv_file)

        policy_header = ['timestamp', 'step', 'loop_iter', 'pred_index'] + [f'a{i}' for i in range(14)]
        exec_header = ['timestamp', 'step', 'loop_iter'] + [f'a{i}' for i in range(14)]
        self._policy_writer.writerow(policy_header)
        self._exec_writer.writerow(exec_header)
        self._csv_initialized = True

    def _close_csv_writers(self):
        """Close CSV files if they were opened."""
        if getattr(self, '_csv_initialized', False):
            try:
                self._policy_csv_file.flush()
                self._exec_csv_file.flush()
            except Exception:
                pass
            try:
                self._policy_csv_file.close()
            except Exception:
                pass
            try:
                self._exec_csv_file.close()
            except Exception:
                pass
            self._csv_initialized = False

    def _log_policy_predictions(self, actions: np.ndarray, step: int, loop_iter: int):
        """Log each policy-predicted action (post-processed) as a row in the policy CSV."""
        if not getattr(self, '_csv_initialized', False):
            return
        tstr = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        # actions expected shape: (num_preds, 14)
        try:
            for i, a in enumerate(actions):
                row = [tstr, step, loop_iter, i] + [float(x) for x in a.tolist()]
                self._policy_writer.writerow(row)
            self._policy_csv_file.flush()
        except Exception:
            # Fail safe: do not crash inference due to logging
            pass

    def _log_executed_action(self, action: np.ndarray, step: int, loop_iter: int):
        """Log the actually executed action (after scaling/clipping) for each sim step."""
        if not getattr(self, '_csv_initialized', False):
            return
        tstr = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
        try:
            row = [tstr, step, loop_iter] + [float(x) for x in action.tolist()]
            self._exec_writer.writerow(row)
            self._exec_csv_file.flush()
        except Exception:
            pass


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
        for j in range(self.model.njnt):
            jtype = self.model.jnt_type[j]
            if jtype == mujoco.mjtJoint.mjJNT_HINGE or jtype == mujoco.mjtJoint.mjJNT_SLIDE:
                jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, j)
                self.full_velocity_limits[jname] = 1e-6
        
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
        """Create monitoring windows using ImageMonitor"""
        self.image_monitor.create_monitoring_windows()
    
    def update_image_monitoring(self, images_list):
        """Update monitoring windows using ImageMonitor"""
        self.image_monitor.update_image_monitoring(images_list)
    
    def close_monitoring_windows(self):
        """Close monitoring windows using ImageMonitor"""
        self.image_monitor.close_monitoring_windows()
    
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
                'raw_state': state,
                'completion_percentage': max(0, 100 * (3 - np.sum(state)) / 3)  # 0-100% completion
            }
        except Exception as e:
            print(f"Error getting dishwasher state: {e}")
            return {'error': str(e)}
    
    def check_task_completion(self, step: int) -> bool:
        """
        Check if dishwasher closing task is completed and print progress.
        
        Args:
            step: Current step number for logging
            
        Returns:
            True if task is completed, False otherwise
        """
        state_details = self.get_dishwasher_state_details()
        
        if 'error' in state_details:
            return False
            
        is_completed = state_details['is_closed']
        
        # Log progress every 50 steps or when completed
        if step % 50 == 0 or is_completed:
            print(f"\n=== TASK PROGRESS [Step {step}] ===")
            print(f"Door: {state_details['door']:.3f} (target: 0.0)")
            print(f"Bottom Tray: {state_details['bottom_tray']:.3f} (target: 0.0)") 
            print(f"Middle Tray: {state_details['middle_tray']:.3f} (target: 0.0)")
            print(f"Completion: {state_details['completion_percentage']:.1f}%")
            print(f"Task Status: {'✅ COMPLETED' if is_completed else '🔄 IN PROGRESS'}")
            print("=" * 40)
            
        return is_completed
    
    def get_recent_predictions_for_aggregation(self):
        """
        Get recent predictions for temporal aggregation using sliding window.
        Returns only the most recent TEMPORAL_WINDOW_SIZE predictions.
        """
        if len(self.action_buffer) == 0:
            return []
        
        # Get only the most recent predictions (sliding window)
        all_predictions = list(self.action_buffer)  # oldest -> newest
        window_predictions = all_predictions[-TEMPORAL_WINDOW_SIZE:]  # last N predictions
        
        return window_predictions
    
    def execute_action(self, action, dt):
        """Execute predicted action using delta movement accumulation + IK (matching data collection)"""
        # Split action into components (matching ds_aloha.py action format)
        # Left arm: action[0:3] = translation, action[3:6] = rotation, action[6] = gripper
        # Right arm: action[7:10] = translation, action[10:13] = rotation, action[13] = gripper
        
        # Update target positions with DELTA movements (matching data collection scaling)
        # X,Y: continuous joystick movements (* 0.0003 in data collection)
        # Z: discrete button presses (±0.03 in data collection) 


        action[2] = Z_GAIN * action[2]
        action[9] = Z_GAIN * action[9]

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
        
        # Store the actually executed action values (after Z scaling and gripper clipping)
        executed_action = action.copy()
        executed_action[6] = left_gripper_clipped
        executed_action[13] = right_gripper_clipped
        self._last_executed_action = executed_action

        self.targets_updated = True
    
    def run_inference(self, max_timesteps=1000):
        """
        Run inference loop with IK solving (matching data collection)
        
        Returns:
            dict: Inference results with success status, completion step, and statistics
        """
        print("Starting inference...")
        
        # Initialize result tracking
        start_time = time.time()
        success = False
        completion_step = None

        # Initialize CSV writers
        self._init_csv_writers()
        
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
                
                # Simple setup
                post_process = lambda a: a * self.action_std.cpu().numpy() + self.action_mean.cpu().numpy()
                
                # Initial policy query and seed buckets for steps [0 .. chunk-1]
                print("=== INITIAL POLICY QUERY ===")
                next_actions = self.select_action(self.policy, None)
                raw_action = next_actions.squeeze(0).detach().cpu().numpy()
                actions = post_process(raw_action)

                # Log initial policy predictions
                self._log_policy_predictions(actions, step=0, loop_iter=self.num_loop_iters)

                # Place predictions into per-timestep buckets
                for i, a_pred in enumerate(actions):
                    bucket_ts = 0 + i
                    if bucket_ts not in self.action_buckets:
                        self.action_buckets[bucket_ts] = []
                    self.action_buckets[bucket_ts].append(a_pred)

                # Aggregate for current timestep 0
                def aggregate_bucket(pred_list):
                    if len(pred_list) == 1:
                        return pred_list[0]
                    # Use last up to TEMPORAL_WINDOW_SIZE predictions with exponential weighting (newest higher)
                    preds = pred_list[-TEMPORAL_WINDOW_SIZE:]
                    w_i = lambda i: np.exp(self.temporal_agg_m * i)
                    weights = np.array([w_i(i) for i in range(len(preds))])
                    weights = weights / np.sum(weights)
                    return np.sum([w * p for w, p in zip(weights, preds)], axis=0)

                action = aggregate_bucket(self.action_buckets.get(0, [actions[0]]))

                print(f"Initial policy query returned {len(actions)} actions (chunk size: {len(actions)})")
                print(f"Timing: {SIM_RATE}Hz sim, policy every {POLICY_QUERY_INTERVAL} steps ({POLICY_QUERY_INTERVAL/SIM_RATE*1000:.0f}ms)\n")
                
                # Flag to track when temporal aggregation starts
                temporal_agg_active = False
                
                step = 0
                
                while viewer.is_running() and step < max_timesteps:
                    self.num_loop_iters += 1
                    
                    # Query policy every N steps and append to per-timestep buckets
                    if (self.num_loop_iters % POLICY_QUERY_INTERVAL == 0):
                        print(f"\n--- POLICY QUERY at loop {self.num_loop_iters} ---")
                        
                        next_actions = self.select_action(self.policy, None)
                        raw_action = next_actions.squeeze(0).detach().cpu().numpy()
                        actions = post_process(raw_action)
                        
                        print(f"New policy query returned {len(actions)} actions")

                        # Log policy predictions for this query
                        self._log_policy_predictions(actions, step=step, loop_iter=self.num_loop_iters)
                        
                        if not temporal_agg_active:
                            print("🔄 ACTIVATING TEMPORAL AGGREGATION")
                            temporal_agg_active = True

                        # Place predictions into per-timestep buckets for [step .. step+len(actions)-1]
                        for i, a_pred in enumerate(actions):
                            bucket_ts = step + i
                            if bucket_ts not in self.action_buckets:
                                self.action_buckets[bucket_ts] = []
                            self.action_buckets[bucket_ts].append(a_pred)
                        # One-line debug: bucket count and weights (newest->oldest)
                        current_bucket_count = len(self.action_buckets.get(step, []))
                        weights_len = min(current_bucket_count, TEMPORAL_WINDOW_SIZE)
                        if weights_len > 0:
                            w = np.array([np.exp(self.temporal_agg_m * i) for i in range(weights_len)])
                            w = w / np.sum(w)
                            w_newest_first = w[::-1]
                            w_str = ','.join(f'{x:.3f}' for x in w_newest_first)
                            print(f"ts={step} bucket={current_bucket_count} weights(new->old)=[{w_str}]")
                        else:
                            print(f"ts={step} bucket={current_bucket_count} weights(new->old)=[]")
                        print("--- END POLICY QUERY ---")
                    
                    # Compute action for current timestep from its bucket
                    bucket = self.action_buckets.get(step, [])
                    if len(bucket) == 0:
                        # No predictions yet for this step; keep previous action
                        pass
                    elif len(bucket) == 1:
                        action = bucket[0]
                    else:
                        preds = bucket[-TEMPORAL_WINDOW_SIZE:]
                        w_i = lambda i: np.exp(self.temporal_agg_m * i)
                        weights = np.array([w_i(i) for i in range(len(preds))])
                        weights = weights / np.sum(weights)
                        action = np.sum([w * p for w, p in zip(weights, preds)], axis=0)

                    # Execute action (updates target poses)
                    self.execute_action(action, sim_rate.dt)

                    # Log the actually executed action after scaling/clipping in execute_action
                    executed = self._last_executed_action if self._last_executed_action is not None else action
                    self._log_executed_action(executed, step=step, loop_iter=self.num_loop_iters)
                    
                    # Update IK target poses if they changed
                    if self.targets_updated:
                        self.update_ik_targets()
                        self.targets_updated = False
                    
                    # Solve IK for both arms using helper method
                    self.solve_ik_step(sim_rate.dt)
                    
                    # Check task completion automatically
                    if self.check_task_completion(step):
                        print(f"\n🎉 SUCCESS! Dishwasher closing task completed at step {step}")
                        print("Model performance verified - task successful!")
                        success = True
                        completion_step = step
                        time.sleep(5)
                        break
                    
                    viewer.sync()
                    sim_rate.sleep()
                    
                    # Compute effective target deltas for debugging
                    eff_delta_l = self.target_l - self._prev_target_l
                    eff_delta_r = self.target_r - self._prev_target_r

                    # Print status 
                    if step % 50 == 0:
                        action_str = f"{action[0]:.4f}, {action[1]:.4f}, {action[2]:.4f}"
                        target_l_str = ', '.join(f'{x:.3f}' for x in self.target_l[:3])
                        target_r_str = ', '.join(f'{x:.3f}' for x in self.target_r[:3])
                        delta_l_str = ', '.join(f'{x:.5f}' for x in eff_delta_l[:3])
                        delta_r_str = ', '.join(f'{x:.5f}' for x in eff_delta_r[:3])

                        # Show temporal aggregation status (bucket-based)
                        if temporal_agg_active:
                            bucket_size = len(self.action_buckets.get(step, []))
                            window_size = min(bucket_size, TEMPORAL_WINDOW_SIZE)
                            temporal_status = f"Temporal agg: ON, bucket(ts={step}) size: {bucket_size}, window: {window_size}"
                        else:
                            temporal_status = "Temporal agg: OFF (using initial action)"

                        # Clip flags: whether targets are at bounds
                        l_clip = [self.target_l[0] == self.x_min or self.target_l[0] == self.x_max,
                                  self.target_l[1] == self.y_min or self.target_l[1] == self.y_max,
                                  self.target_l[2] == self.z_min or self.target_l[2] == self.z_max]
                        r_clip = [self.target_r[0] == self.x_min or self.target_r[0] == self.x_max,
                                  self.target_r[1] == self.y_min or self.target_r[1] == self.y_max,
                                  self.target_r[2] == self.z_min or self.target_r[2] == self.z_max]

                        print(f"Step {step}: Action=[{action_str}]")
                        print(f"  Target_L=[{target_l_str}]  ΔL=[{delta_l_str}]  clipL={l_clip}")
                        print(f"  Target_R=[{target_r_str}]  ΔR=[{delta_r_str}]  clipR={r_clip}")
                        print(f"  {temporal_status}, Loop: {self.num_loop_iters}")
                    
                    step += 1
                    # Update previous targets after each iteration
                    self._prev_target_l = self.target_l.copy()
                    self._prev_target_r = self.target_r.copy()
                    
                    # Keep using the same action until next policy query
                    # (action is updated only during periodic policy queries)
                    
        except KeyboardInterrupt:
            print("\nInference interrupted by user")
        finally:
            # Close monitoring windows
            self.close_monitoring_windows()
            # Close CSV writers
            self._close_csv_writers()
            
            # Calculate results
            end_time = time.time()
            duration = end_time - start_time
            
            # Ensure step is defined even if exception occurs early
            if 'step' in locals():
                total_steps = step
                print(f"Inference completed. Ran for {step} steps.")
            else:
                total_steps = 0
                print("Inference completed. Error occurred during initialization.")
        
        # Return comprehensive results
        results = {
            'success': success,
            'completion_step': completion_step,
            'duration': duration,
            'total_steps': total_steps,
            'final_dishwasher_state': self.get_dishwasher_state_details() if hasattr(self, 'get_dishwasher_state_details') else None,
            'completion_rate': completion_step / total_steps if completion_step and total_steps > 0 else None
        }
        
        return results


def main():
    """Main function to run ACT inference"""
    # Paths
    ckpt_path = "policy_best.ckpt"
    dataset_stats_path = "dataset_stats.pkl"  # Will be created when you run training
    
    # Check if checkpoint exists
    if not os.path.exists(ckpt_path):
        print(f"Error: Checkpoint file not found: {ckpt_path}")
        return
    
    # Initialize inference system
    print("Initializing ACT inference system...")
    inference = ACTInference(ckpt_path, dataset_stats_path)
    
    # Run inference - longer episode for more observation
    results = inference.run_inference(max_timesteps=INFERENCE_STEP)  # Run for 500 steps (~25 seconds at 20Hz)
    
    # Print results summary
    print(f"\n=== INFERENCE RESULTS ===")
    print(f"Success: {results['success']}")
    print(f"Duration: {results['duration']:.2f}s")
    print(f"Total steps: {results['total_steps']}")
    if results['completion_step']:
        print(f"Completed at step: {results['completion_step']}")
        print(f"Completion rate: {results['completion_rate']:.1%}")
    print(f"=========================")


if __name__ == "__main__":
    main()