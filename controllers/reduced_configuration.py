import numpy as np
from typing import Optional
import mujoco
from mink.configuration import Configuration 
from mink.limits import Limit, Constraint

# original file from https://github.com/AlmondGod/Nintendo-Aloha/blob/master/control/reduced_configuration.py 
class ReducedConfiguration(Configuration):
    def __init__(self, model, data, relevant_qpos_indices, relevant_qvel_indices):
        super().__init__(model, data)
        self.relevant_qpos_indices = relevant_qpos_indices
        self.relevant_qvel_indices = relevant_qvel_indices

    @property
    def q(self) -> np.ndarray:
        """Return the relevant qpos entries."""
        return self.data.qpos[self.relevant_qpos_indices].copy()

    @q.setter
    def q(self, value: np.ndarray):
        self.data.qpos[self.relevant_qpos_indices] = value

    @property
    def dq(self) -> np.ndarray:
        """Return the relevant qvel entries."""
        return self.data.qvel[self.relevant_qvel_indices].copy()

    @dq.setter
    def dq(self, value: np.ndarray):
        self.data.qvel[self.relevant_qvel_indices] = value

    def update(self, q: Optional[np.ndarray] = None) -> None:
        if q is not None:
            self.q = q  
        super().update()

    def get_frame_jacobian(self, frame_name: str, frame_type: str) -> np.ndarray:
        full_jacobian = super().get_frame_jacobian(frame_name, frame_type)

        reduced_jacobian = full_jacobian[:, self.relevant_qvel_indices]
        return reduced_jacobian

    def integrate_inplace(self, velocity: np.ndarray, dt: float) -> None:

        full_velocity = np.zeros(self.model.nv)
        full_velocity[self.relevant_qvel_indices] = velocity
        super().integrate_inplace(full_velocity, dt)

    @property
    def nv(self) -> int:
        return len(self.relevant_qvel_indices)

    def check_limits(self, tol: float = 1e-6, safety_break: bool = True) -> None:
        """Check that the current configuration is within bounds for relevant joints."""
        for idx, jnt in enumerate(self.relevant_joints):
            jnt_type = self.model.jnt_type[jnt]
            if (
                jnt_type == mujoco.mjtJoint.mjJNT_FREE
                or not self.model.jnt_limited[jnt]
            ):
                continue
            qval = self.q[idx]
            qmin = self.model.jnt_range[jnt, 0]
            qmax = self.model.jnt_range[jnt, 1]
            if qval < qmin - tol or qval > qmax + tol:
                if safety_break:
                    raise exceptions.NotWithinConfigurationLimits(
                        joint_id=jnt,
                        value=qval,
                        lower=qmin,
                        upper=qmax,
                        model=self.model,
                    )
                else:
                    print(
                        f"Value {qval:.2f} at index {idx} is outside of its limits: "
                        f"[{qmin:.2f}, {qmax:.2f}]"
                    )

    @property
    def relevant_joints(self) -> np.ndarray:
        """Get the joint IDs corresponding to the relevant qpos indices."""

        joint_ids = []
        for qpos_idx in self.relevant_qpos_indices:
            jnt = np.where(self.model.jnt_qposadr == qpos_idx)[0][0]
            joint_ids.append(jnt)
        return np.array(joint_ids)

class ReducedConfigurationLimit(Limit):
    def __init__(self, model, relevant_qpos_indices, qpos_min=None, qpos_max=None):
        self.model = model
        self.relevant_qpos_indices = relevant_qpos_indices
        nq_reduced = len(relevant_qpos_indices)
        
        joint_indices = []
        for qpos_idx in relevant_qpos_indices:
            jnt = np.where(model.jnt_qposadr == qpos_idx)[0]
            if len(jnt) == 0:
                raise ValueError(f"No joint found for qpos index {qpos_idx}")
            joint_indices.append(jnt[0])
        joint_indices = np.array(joint_indices)
        
        if qpos_min is None:
            qpos_min = model.jnt_range[joint_indices, 0]
        if qpos_max is None:
            qpos_max = model.jnt_range[joint_indices, 1]
        self.qpos_min = qpos_min
        self.qpos_max = qpos_max
        self.nq_reduced = nq_reduced
    
    def compute_qp_inequalities(self, configuration: Configuration, dt: float) -> Constraint:
        qpos0 = configuration.q
        dq = configuration.dq
        qpos1 = qpos0 + dq * dt
        lower = (self.qpos_min - qpos0) / dt
        upper = (self.qpos_max - qpos0) / dt
        G = np.vstack([-np.eye(self.nq_reduced), np.eye(self.nq_reduced)])
        h = np.hstack([-lower, upper])
        return Constraint(G=G, h=h)

class ReducedVelocityLimit(Limit):
    def __init__(self, model, relevant_qvel_indices, velocity_limits):
        self.model = model
        self.relevant_qvel_indices = relevant_qvel_indices
        print(f"velocity_limits: {velocity_limits}")
        self.velocity_limits = {
            velocity_limits[i] for i in relevant_qvel_indices
        }
        self.nv_reduced = len(relevant_qvel_indices)
    
    def compute_qp_inequalities(self, configuration: Configuration, dt: float) -> Constraint:
        lower = -self.velocity_limits
        upper = self.velocity_limits
        G = np.vstack([-np.eye(self.nv_reduced), np.eye(self.nv_reduced)])
        h = np.hstack([-lower, upper])
        return Constraint(G=G, h=h)