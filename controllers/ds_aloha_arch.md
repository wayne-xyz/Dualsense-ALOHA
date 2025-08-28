### DualSense ALOHA Control Architecture

This document summarizes how `controllers/ds_aloha.py` ties together the controller input, BiGym environment, robot config, IK loop, and MuJoCo XML assets.

#### High-level module relationships

```mermaid
graph TD
  subgraph "Controller"
    A["controllers/ds_aloha.py\nDSAlohaMocapControl"]
    A1["pydualsense input"]
    A2["SO3 target pose + gripper cmds"]
  end

  subgraph "Env"
    B["bigym/envs/dishwasher.py\nDishwasherClose"]
    B0["BiGymEnv"]
    B1["ObservationConfig/CameraConfig"]
  end

  subgraph "Core"
    C["bigym/bigym_env.py\nBiGymEnv"]
    C1["Mojo loads WORLD_MODEL (world.xml)"]
    C2["Robot(AlohaRobot)"]
    C3["Preset(dishwasher.yaml)"]
    C4["BiGymRenderer + mujoco.Renderer"]
  end

  subgraph "Robot"
    D["bigym/robots/configs/aloha.py\nAlohaRobot -> Robot"]
    D1["RobotConfig.model = envs/xmls/aloha/scene.xml"]
    D2["GripperConfig.model = envs/xmls/aloha/aloha_gripper.xml"]
    D3["Arms, cameras, actuators"]
    D4["bigym/robots/robot.py\nRobot._on_loaded"]
    D5["bigym/robots/gripper.py\nGripper"]
  end

  subgraph "Assets(XML)"
    E["envs/xmls/world.xml"]
    E1["envs/xmls/aloha/scene.xml"]
    E2["envs/xmls/aloha/aloha_gripper.xml"]
    E3["envs/xmls/props/dishwasher/dishwasher.xml"]
  end

  subgraph "Presets"
    P["envs/presets/dishwasher.yaml"]
    P1["bigym/envs/props/preset.py\nPreset loader"]
    P2["bigym/envs/props/dishwasher.py\nDishwasher Prop"]
  end

  subgraph "IK"
    K["controllers/reduced_configuration.py"]
    K1["mink.solve_ik + FrameTask"]
  end

  A1 --> A
  A -->|"sets targets"| A2
  A2 -->|"update loop"| K
  A2 -->|"gripper ctrl"| D5

  A -->|"creates"| B
  B -->|"inherits"| B0

  C -.->|"exposes"| C1
  C -.->|"instantiates"| C2
  C -.->|"loads"| C3
  C -.->|"renders"| C4

  B0 -->|"__init__"| C
  C1 -->|"loads"| E
  C2 -->|"RobotConfig.model"| D1 --> E1
  D2 --> E2
  C3 -->|"yaml"| P --> P1 --> P2 --> E3

  D4 -->|"on_loaded: remove namespaces, configure cameras, wrists, pelvis"| D
  D5 -->|"attach gripper at wrist site, cache actuators"| D

  K -->|"reduced qpos/qvel indices"| C1
  K1 -->|"compute velocities"| C1

  C4 -->|"human/rgb/depth"| A
```

#### Loading sequence and data flow

- "World" base model
  - **File**: `bigym/envs/xmls/world.xml` (constant `WORLD_MODEL` in `bigym/const.py`)
  - **Loaded by**: `Mojo` inside `bigym/bigym_env.BiGymEnv.__init__`
  - Provides the global floor, lighting, and camera `external`.

- Robot (ALOHA)
  - **Config class**: `bigym/robots/configs/aloha.py` → `AlohaRobot`
  - **Scene XML**: `bigym/envs/xmls/aloha/scene.xml` (assigned via `RobotConfig.model`)
  - **Gripper XML**: `bigym/envs/xmls/aloha/aloha_gripper.xml` (via `GripperConfig`)
  - **Hook**: `bigym/robots/robot.Robot._on_loaded` configures cameras, wrist sites, pelvis, and removes namespaces.

- Task environment (DishwasherClose)
  - **Env class**: `bigym/envs/dishwasher.py` → `DishwasherClose` (inherits `BiGymEnv`)
  - **Preset**: `bigym/envs/presets/dishwasher.yaml`
  - **Prop class**: `bigym/envs/props/dishwasher.py` → loads `envs/xmls/props/dishwasher/dishwasher.xml` 
  - **Preset loader**: `bigym/envs/props/preset.Preset` parses YAML and instantiates prop graph.

- Observations and rendering
  - `ObservationConfig` + `CameraConfig` specify which robot/world cameras to render.
  - `BiGymRenderer` and per-resolution `mujoco.Renderer` instances handle human and offscreen rendering.

- Controller and IK
  - `controllers/ds_aloha.py` reads DualSense input via `pydualsense`, calibrates, and updates target positions/orientations (`SO3`) and gripper positions.
  - Creates Mink tasks (`mink.FrameTask`) and uses `ReducedConfiguration` to limit IK to relevant joint indices.
  - Runs `mink.solve_ik` to compute joint velocities, integrates them, writes into `data.qpos/qvel` and `data.ctrl` for both arms and grippers.

#### Files loaded at runtime (non-exhaustive)

- Core/world:
  - `bigym/envs/xmls/world.xml`
- Robot and tools:
  - `bigym/envs/xmls/aloha/scene.xml`
  - `bigym/envs/xmls/aloha/aloha_gripper.xml`
- Props and task assets:
  - `bigym/envs/presets/dishwasher.yaml`
  - `bigym/envs/xmls/props/dishwasher/dishwasher.xml`
- Cameras (referenced in scene XML):
  - `wrist_cam_left`, `wrist_cam_right`, `overhead_cam`, `teleoperator_pov`, `collaborator_pov`

#### How control flows during a sim step (ds_aloha)

1. Read DualSense state → map to target deltas and orientation deltas; update `target_l/target_r`, `rot_l/rot_r`, gripper targets.
2. Update Mink `FrameTask` targets from `SE3(rot, pos)`.
3. Solve IK (`mink.solve_ik`) for left/right reduced configurations; integrate and write to `data.qpos/qvel` and `data.ctrl`.
4. Step MuJoCo; render via viewer; optionally store HDF5 data (qpos/qvel/action/images).

### IK submodel path (no ReducedConfiguration)

An alternative path replaces `ReducedConfiguration` with per-arm IK submodels to avoid modifying Mink. Each IK submodel is a trimmed version of `envs/xmls/aloha/scene.xml` that contains only one arm and its actuators. Mink then operates on these compact models and we synchronize states with the main model.

```mermaid
graph TD
  subgraph "Main Sim"
    M0["BiGymEnv + Mojo (world.xml)"]
    M1["Aloha scene (scene.xml)"]
    M2["Full state: qpos/qvel/ctrl"]
  end

  subgraph "IK Submodels"
    S0["controllers/ik_submodel.py\nBuild per-arm submodel by trimming scene.xml"]
    S1["Left arm submodel\n(joints, left_gripper site)"]
    S2["Right arm submodel\n(joints, right_gripper site)"]
    C1["Mink Configuration (left)"]
    C2["Mink Configuration (right)"]
  end

  T["FrameTask targets (SE3 from ds_aloha)"]

  T --> C1
  T --> C2
  S0 --> S1
  S0 --> S2
  S1 --> C1
  S2 --> C2

  M2 <-->|"sync qpos/qvel slices"| C1
  M2 <-->|"sync qpos/qvel slices"| C2

  C1 -->|"solve_ik + integrate"| M2
  C2 -->|"solve_ik + integrate"| M2

  M2 -->|"set ctrl"| M1
```

Key differences vs ReducedConfiguration:
- **No Mink changes**: Mink runs on stand-alone IK models with only the needed DOFs.
- **Explicit mapping**: We maintain lists mapping submodel joints to main `qpos/qvel` indices.
- **Same rendering/logging**: All rendering and recording continue to use the main model.

Files added/used in this path:
- `controllers/ik_submodel.py`: builds per-arm IK submodels and index mappings.
- `controllers/ds_aloha.py`: feature flag `use_ik_submodels=True` to enable this path.

How to enable:

```python
control = DSAlohaMocapControl(use_ik_submodels=True)
```

Per-step flow when submodels are enabled:
- Copy current main `qpos` slices into each submodel.
- Set `FrameTask` targets on submodels and run `mink.solve_ik`.
- Integrate submodels; copy back `q/qvel` to the main model indices.
- Drive main actuators toward updated joint positions; step MuJoCo; render/log as usual.