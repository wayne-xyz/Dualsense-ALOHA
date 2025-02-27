# Dualsense-ALOHA
Dualsense™ Controller Teleoperation and ACT autonomy on ALOHA for Bigym benchmark tasks 

- Insipred by the [Nitendo-Aloha](https://github.com/AlmondGod/Nintendo-Aloha)
- Reproduce the implementation and concept of Nintendo-Aloha and replace the controller to be compatible with Dualsense™ Controller
- Add the unique feature of Dualsense Controller, haptic feedback and adaptive triggers
- Original Readme of [bigym](https://github.com/chernyadev/bigym)


## Read Guide 
### Basic Concept : 

#### Bigym
- **What It Is:**  
  Bigym is a benchmark and simulation environment for bimanual robotic manipulation tasks.  It provides standardized tasks (e.g., reaching, stacking, object manipulation) that simulate realistic robotic operations.
- **Role in the Repo:**  
  Bigym serves as the foundation for the tasks. The repository extends Bigym by adding new, context-specific tasks designed for dual-arm (bimanual) control.

#### ALOHA(Simulated)
- **What It Is:**  
  ALOHA (A Low-cost Open-source Hardware System for Bimanual Teleoperation) is a framework that enables affordable bimanual robot control using consumer-grade hardware.

- **Role in this Repo:**  
  We build upon ALOHA's architecture to enable teleoperation using a Dualsense™ controller, allowing for intuitive bimanual robot control and demonstration data collection for training autonomous behaviors.

#### Mujoco
- **What It Is:**  
  Mujoco is a high-performance physics engine designed for simulating complex robot dynamics.
- **Role in the Repo:**  
  It provides the simulation backbone, allowing the Bigym/ALOHA tasks to run in a realistic environment.

#### ACT (Action-Chunking Transformer)
- **What It Is:**  
  ACT is a transformer-based machine learning model tailored for processing sequential data.
- **How It Works:**  
  - Breaks down long sequences of actions into manageable "chunks."
  - Uses attention mechanisms to capture temporal dependencies in demonstration data.
- **Role in the Repo:**  
  The model learns from teleoperated demonstration data (state–action sequences) to eventually control the robot autonomously.

#### Nintendo-Aloha

what Nintendo-Aloha does :  
- 1 Extending the Bigym for Bimanual ALOHA Tasks
    - 1.1 Implementing new tasks for bimanual manipulation
    - 1.2 Integrating ALOHA hardware for teleoperation
    - 1.3 Collecting demonstration data via Nintendo Switch controllers
- 2 Teleoperation with Nintendo Switch Joycons
    - 2.1 `teleop_joycon.py` : read the controller's data and send it to the simulation environment
    - 2.2 Input mapping strategy 


#### Dualsense-ALOHA
what Dualsense-Aloha does : 
- 1 Extending the Bigym for Bimanual ALOHA Tasks(Same as Nintendo-Aloha)
- 2 Teleoperation with Dualsense Controller
    - 2.1 `teleop_dualsense.py` : read the controller's data and send it to the simulation environment
    - 2.2 Input mapping strategy 
    - 2.3 add the unique feature of Dualsense Controller, haptic feedback and adaptive triggers


## Dualsense Controller's Unique feature


## Install

`pip install .`

## Dualsense controller python library support 

`pip install pydualsense` 

###  setup error :

```
FileNotFoundError: [Errno 2] No such file or directory: 'cmake'
.....
ERROR: Failed building wheel for dm-tree
.....
ERROR: Failed to build installable wheels for some pyproject.toml based projects (dm-tree)
```


The error occurs because one of the dependencies (dm-tree) requires CMake to be built, but CMake is not installed on your system. 

#### Mac device solution :  
```
brew install cmake
```

#### Windows device solution : 
```
choco install cmake
```




