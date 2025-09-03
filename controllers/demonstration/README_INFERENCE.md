# ACT Model Inference for ALOHA

This directory contains the inference code to run your trained ACT model on the ALOHA robot.

## Files Overview

- **`run_inference.py`** - Main inference script
- **`generate_stats.py`** - Generate dataset statistics for normalization
- **`policy.py`** - ACT policy class definition
- **`utils.py`** - Data loading utilities
- **`constants.py`** - Configuration constants
- **`detr/`** - DETR transformer architecture
- **`policy_epoch_3000_seed_0.ckpt`** - Your trained model checkpoint

## How to Run Inference

### Step 1: Generate Dataset Statistics (First time only)

```bash
cd controllers/demonstration
python generate_stats.py
```

This will create `dataset_stats.pkl` needed for proper normalization.

### Step 2: Run Inference

```bash
python run_inference.py
```

## What the Inference Script Does

1. **Loads the trained ACT model** from the checkpoint file
2. **Sets up the same BiGym environment** used during data collection
3. **Runs the inference loop**:
   - Gets current robot state (joint positions + camera images)
   - Feeds observations to the ACT model
   - Gets action predictions (with action chunking)
   - Executes actions on the robot via inverse kinematics
   - Repeats at 50Hz control frequency

## Key Features

- **Action Chunking**: Uses the same chunking strategy as training
- **Proper Normalization**: Normalizes observations using dataset statistics
- **BiGym Integration**: Uses identical environment setup as data collection
- **Real-time Control**: Runs at 50Hz for smooth robot control

## Troubleshooting

### If you get import errors:
- Make sure you're in the `controllers/demonstration` directory
- Check that all required files were copied correctly

### If the robot behavior looks wrong:
- Ensure `dataset_stats.pkl` was generated from your training data
- Check that the model checkpoint matches your training configuration
- Verify camera feeds are working correctly

### If performance is poor:
- The model might need more training data
- Check if normalization statistics are correct
- Verify the action space matches between training and inference

## Model Configuration

The inference script uses the same configuration as training:
- **Backbone**: ResNet18
- **Transformer**: 4 encoder layers, 7 decoder layers
- **Action Chunking**: 100 time steps
- **Control Frequency**: 50Hz
- **Cameras**: 4 cameras (wrist_left, wrist_right, overhead, teleoperator_pov)

## Notes

- The inference runs in simulation using the same BiGym environment
- Action execution uses position control for joint actuators
- Gripper positions are clipped to safe ranges (0.02-0.037)
- Press Ctrl+C to stop inference safely