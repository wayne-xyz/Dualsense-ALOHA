# ACT Inference System - Comprehensive Guide

This document provides detailed information about the improved ACT (Action Chunking with Transformers) inference system for ALOHA robot dishwasher closing tasks.

## Overview

The inference system has been significantly enhanced with multiple improvements including temporal aggregation, automatic task completion detection, modular architecture, and comprehensive evaluation capabilities.

## File Structure

### Core Inference Files

- **`run_inference_temporalAgg.py`** - Main inference script with temporal aggregation
- **`run_inference_direct.py`** - Direct inference without temporal aggregation  
- **`evaluate_inference.py`** - Multi-trial evaluation system
- **`image_monitor.py`** - Modular image monitoring utilities

### Key Features

## 1. Temporal Aggregation System

### Enhanced Buffer Management
- **Single Dynamic Buffer**: Replaced complex nested buffer system with simple `deque(maxlen=100)`
- **Sliding Window**: Uses only recent 15 predictions for aggregation (configurable via `TEMPORAL_WINDOW_SIZE`)
- **Exponential Weighting**: Newer predictions receive higher weights using `w_i = exp(m * i)`

### Configuration Parameters
```python
TEMPORAL_BUFFER_SIZE = 100      # Total buffer capacity
TEMPORAL_WINDOW_SIZE = 15       # Recent predictions window for aggregation  
TEMPORAL_AGG_M = 0.1           # Exponential weighting parameter
```

### Temporal Aggregation Process
1. **Initial Phase**: Direct action execution without buffering
2. **Activation**: Temporal aggregation starts after first periodic policy query
3. **Buffering**: All action chunks stored in single deque buffer
4. **Aggregation**: Weighted average of recent predictions using sliding window
5. **Execution**: Aggregated actions executed with delta movement accumulation

## 2. Task Completion Detection

### Automatic Progress Monitoring
- **Real-time State Tracking**: Monitors dishwasher door, bottom_tray, middle_tray positions
- **Success Detection**: Automatically detects task completion with tolerance-based checking
- **Progress Logging**: Regular status updates every 50 steps

### Dishwasher State Functions
```python
is_dishwasher_closed()           # Boolean success check
get_dishwasher_state_details()   # Detailed state information
check_task_completion(step)      # Progress monitoring with logging
```

### Success Criteria
- Door position ≈ 0 (closed)
- Bottom tray ≈ 0 (pushed in)
- Middle tray ≈ 0 (pushed in)
- Tolerance: ±0.05 for all joints

## 3. Inverse Kinematics (IK) System

### Dual-Arm IK Architecture
- **Independent Solving**: Each arm's IK solved separately using dedicated configurations
- **Scene-Based Initialization**: Initial positions taken from current scene state (no hardcoded values)
- **Delta Movement Accumulation**: Actions applied as incremental movements to target poses

### IK Configuration
```python
IK_MAX_ITERS = 40               # Maximum IK iterations per step
SIM_RATE = 200.0               # Simulation frequency (Hz)
POLICY_QUERY_INTERVAL = 5       # Policy queries every N steps
```

## 4. Modular Architecture

### Image Monitoring (`image_monitor.py`)
- **Separated Concerns**: Image monitoring extracted to dedicated module
- **2x2 Grid Display**: Real-time camera feed visualization
- **Configurable**: Enable/disable via `ENABLE_IMAGE_MONITORING` flag

### ImageMonitor Class
```python
monitor = ImageMonitor(camera_names, enabled=True)
monitor.create_monitoring_windows()
monitor.update_image_monitoring(images)
monitor.close_monitoring_windows()
```

## 5. Action Execution System

### Enhanced Action Processing
- **Full Chunk Utilization**: Executes all actions in each chunk sequentially (not just first action)
- **Z-Axis Scaling**: Special scaling for vertical movements (`Z_GAIN = 4`)
- **Gripper Control**: Proper gripper position clamping (0.02 - 0.037 range)
- **Workspace Limits**: Position clamping within defined workspace boundaries

### Action Format (14-dimensional)
- Left arm: [translation(3), rotation(3), gripper(1)]
- Right arm: [translation(3), rotation(3), gripper(1)]

## 6. Evaluation System

### Multi-Trial Evaluation (`evaluate_inference.py`)
- **10-Trial Testing**: Comprehensive performance assessment
- **Success Rate Tracking**: Statistical analysis of completion rates
- **Detailed Metrics**: Duration, completion steps, failure analysis
- **Results Logging**: Timestamped result files for record keeping

### Evaluation Metrics
- Success rate percentage
- Average completion time
- Average completion steps  
- Fastest/slowest completion analysis
- Failed trial diagnostics

### Usage
```bash
cd controllers/demonstration
python evaluate_inference.py
```

## 7. Return Value System

### Comprehensive Results
The `run_inference()` function now returns detailed results:

```python
results = {
    'success': bool,                    # Task completion status
    'completion_step': int,             # Step when task completed  
    'duration': float,                  # Total inference time (seconds)
    'total_steps': int,                 # Total steps executed
    'final_dishwasher_state': dict,     # Final state details
    'completion_rate': float            # Percentage completion rate
}
```

## Usage Instructions

### Single Inference Run
```bash
cd controllers/demonstration
python run_inference_temporalAgg.py    # With temporal aggregation
python run_inference_direct.py         # Direct inference
```

### Multi-Trial Evaluation
```bash
python evaluate_inference.py           # 10-trial evaluation with statistics
```

### Configuration Options

#### Core Parameters
```python
SIM_RATE = 200.0                       # Simulation frequency
INFERENCE_STEP = 1000                  # Maximum steps per trial
POLICY_QUERY_INTERVAL = 5              # Policy query frequency
```

#### Temporal Aggregation
```python
TEMPORAL_BUFFER_SIZE = 100             # Buffer capacity
TEMPORAL_WINDOW_SIZE = 15              # Aggregation window size
TEMPORAL_AGG_M = 0.1                   # Exponential weighting factor
```

#### Monitoring
```python
ENABLE_IMAGE_MONITORING = False        # Enable real-time image display
```

## Performance Improvements

### Key Enhancements
1. **Buffer Overflow Fix**: Eliminated "beyond buffer range" warnings
2. **Temporal Window Optimization**: Focus on recent predictions for better responsiveness
3. **Automatic Success Detection**: No manual intervention required
4. **Modular Code Structure**: Reduced main file from 800+ to manageable size
5. **Comprehensive Statistics**: Detailed performance tracking and analysis

### Expected Performance
- **Success Rate**: Varies based on model quality and task complexity
- **Completion Time**: Typically 15-30 seconds for successful trials
- **Completion Steps**: Usually 200-600 steps depending on initial scene state

## Troubleshooting

### Common Issues
1. **Checkpoint Not Found**: Ensure `policy_best.ckpt` exists in working directory
2. **Image Monitoring Errors**: Disable via `ENABLE_IMAGE_MONITORING = False`
3. **IK Convergence**: Adjust `IK_MAX_ITERS` if convergence issues occur
4. **Buffer Overflow**: Should not occur with new deque implementation

### Debug Information
- Real-time temporal aggregation status
- Task completion progress logging
- Detailed error messages with stack traces
- Buffer state information

## Architecture Comparison

### Temporal Aggregation vs Direct Inference
| Feature | Temporal Aggregation | Direct Inference |
|---------|---------------------|-----------------|
| **Smoothness** | High (weighted averaging) | Lower (single predictions) |
| **Responsiveness** | Balanced (recent focus) | High (immediate) |
| **Robustness** | High (noise reduction) | Lower (sensitive to outliers) |
| **Complexity** | Higher (buffer management) | Lower (direct execution) |
| **Performance** | Better for complex tasks | Good for simple tasks |

## Future Improvements

### Potential Enhancements
1. **Adaptive Window Size**: Dynamic temporal window based on task phase
2. **Multi-Task Support**: Extend beyond dishwasher closing
3. **Real-time Visualization**: Enhanced monitoring with 3D pose display
4. **Performance Optimization**: GPU acceleration for temporal aggregation
5. **Advanced IK**: Obstacle avoidance and joint limit optimization

---

This inference system represents a significant advancement in ACT model deployment, providing robust, configurable, and well-monitored robotic task execution with comprehensive evaluation capabilities.