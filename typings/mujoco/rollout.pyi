import mujoco
from numpy import typing as npt

def rollout(model: mujoco.MjModel, data: mujoco.MjData, initial_state: npt.ArrayLike, control: npt.ArrayLike | None = None, *, control_spec: int = ..., skip_checks: bool = False, nroll: int | None = None, nstep: int | None = None, initial_warmstart: npt.ArrayLike | None = None, state: npt.ArrayLike | None = None, sensordata: npt.ArrayLike | None = None): ...
