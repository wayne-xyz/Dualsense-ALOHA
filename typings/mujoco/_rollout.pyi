import mujoco._structs
import numpy

def rollout(model: mujoco._structs.MjModel, data: mujoco._structs.MjData, nroll: int, nstep: int, control_spec: int, state0: numpy.ndarray[numpy.float64], warmstart0: numpy.ndarray[numpy.float64] | None = ..., control: numpy.ndarray[numpy.float64] | None = ..., state: numpy.ndarray[numpy.float64] | None = ..., sensordata: numpy.ndarray[numpy.float64] | None = ...) -> None: ...
