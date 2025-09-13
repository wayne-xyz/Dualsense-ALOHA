<img width="1282" height="404" alt="Group 7" src="https://github.com/user-attachments/assets/02a92b52-0249-4eab-9ca0-349b0f661da9" />


# Dualsense-ALOHA
Dualsense™ Controller Teleoperation and ACT autonomy IL on ALOHA for one Bigym benchmark task

**Outcomes:**
1. The ACT model successfully completed the task.
2. Implemented a multi-buffer inference system for improved movement stability, and applied an inference result scaler to address action attenuation—ensuring that the movement amplitude was sufficient to avoid object penetration in this scenario 100% avoided.
3. Integrated MuJoCo Warp in headless mode, achieving a 10x increase in step speed.


**Intro:**
- Insipred by the [Nitendo-Aloha](https://github.com/AlmondGod/Nintendo-Aloha)
- Reproduce the implementation and concept of Nintendo-Aloha and replace the controller to be compatible with Dualsense™ Controller
- Explore ACT IL, Mujoco, Warp
- Original Readme of [bigym](https://github.com/chernyadev/bigym)








## Data collection Teleoprate based on Dualsense controller: 
Teleoperation, refer to the [ds_aloha.py](controllers/demonstration/ds_aloha.py) script,based on [pydualsense](https://github.com/flok/pydualsense) library.

![Dishwasher Close Task GIF](controllers/demo/closerdishwasher2.gif)





https://github.com/user-attachments/assets/cee4a915-c5a2-44ed-bc1a-9dd3fbc2a980





## Inference :
[Model training repository](https://github.com/wayne-xyz/act-bigym-aloha-dualsense)

Model inference entry point: [controllers/demonstration/run_inference_temporalAgg.py](controllers/demonstration/run_inference_temporalAgg.py)












## MuJoCo Warp Accelerator
<img width="2471" height="1573" alt="Warp Compare" src="https://github.com/user-attachments/assets/a78020a2-0b3c-456a-8cba-c6f9d2f13ab1" />


## Reflections

### 1. Action Attenuation
After ACT model training, action outputs are often weaker than intended, leading to less accurate positioning (though movement direction is usually correct). This is likely due to most collected actions being near zero. To compensate, we apply a scaling factor to the inferred actions so the robot arm moves sufficiently. Comparing action distributions between the dataset and policy outputs is recommended.

<img width="3160" height="3389" alt="Compare Data Collection and Inference" src="https://github.com/user-attachments/assets/281e4a24-c122-4201-a021-91cdd9eb92a7" />




https://github.com/user-attachments/assets/175b3d9d-7c23-4e3f-8d4d-24566844e96b







### 2. Discrete Action Handling
Binary controller actions (e.g., action2, action9) can become diluted during training. To address this, we use MixUp interpolation to convert discrete actions into smoother, continuous values, improving model learning and inference.

### 3. MuJoCo Warp Suitability
MuJoCo Warp is optimized for fast, headless multi-world simulations and is not ideal for single-world, visualized policy inference.

### 4. Others
Model training throug colab by A100 about 2s per it.  vs my RTX3060 17s per it. 
