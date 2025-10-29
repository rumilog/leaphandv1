# Quaternion-to-Angle Conversion System for LEAP Hand Teleoperation

## Overview

This system converts Quest 2 VR hand tracking quaternions into joint angles for the LEAP hand robot. It implements the swing-twist decomposition method for accurate 2-DOF joint control and provides a complete teleoperation pipeline.

## System Architecture

```
Quest 2 VR → Unity (HandController.cs) → server_env.py → LEAP Hand
```

### Data Flow:
1. **Quest 2 VR**: Provides hand tracking data with bone quaternions
2. **Unity HandController.cs**: Converts quaternions to joint angles using swing-twist decomposition
3. **server_env.py**: Receives angles and controls the LEAP hand robot
4. **LEAP Hand**: Executes the commanded joint positions

## Key Features

### 1. Quaternion Mathematics
- **Normalization**: Ensures unit quaternions for stable calculations
- **Inverse**: Computes quaternion conjugates for relative rotations
- **Multiplication**: Hamilton product for quaternion composition
- **Axis-Angle**: Extracts rotation axis and angle from quaternions

### 2. Swing-Twist Decomposition
- **TwistOnly**: Projects rotation onto a specific axis
- **RemoveTwist**: Removes twist component to isolate swing
- **TwistAngleAroundAxis**: Computes signed angle around an axis

### 3. Joint Mapping
The system maps VR hand bones to LEAP hand joints:

| Finger | VR Bones | LEAP Joints | Description |
|--------|----------|-------------|-------------|
| Index | 5-8 | 0-3 | MCP Side, MCP Forward, PIP, DIP |
| Middle | 9-12 | 4-7 | MCP Side, MCP Forward, PIP, DIP |
| Ring | 13-16 | 8-11 | MCP Side, MCP Forward, PIP, DIP |
| Thumb | 1-4 | 12-15 | MCP Side, MCP Forward, PIP, DIP |

### 4. Joint Types
- **MCP (Metacarpophalangeal)**: 2-DOF joint (abduction + flexion)
- **PIP (Proximal Interphalangeal)**: 1-DOF hinge joint
- **DIP (Distal Interphalangeal)**: 1-DOF hinge joint

## Usage Instructions

### 1. Calibration
Before using the system, you must calibrate it in a neutral pose:

1. **Start the Unity application** with Quest 2 VR
2. **Hold your hand flat** (neutral pose) with fingers extended
3. **Press the B button** on the Quest 2 controller to calibrate
4. **Wait for "Calibration completed!"** message

### 2. Control
- **Press A button**: Toggle hand tracking on/off
- **Press B button**: Calibrate neutral pose
- **Move your hand**: The LEAP hand will follow your movements

### 3. Server Configuration
In `server_env.py`, configure:

```python
# LEAP Hand configuration
LEAP_HAND_PORT = '/dev/ttyUSB0'  # Adjust to your port
USE_LEAP_HAND = True  # Enable LEAP hand control
```

## Technical Details

### Joint Limits (Degrees)
```csharp
Index/Middle/Ring: [-60, 128, 108, 117]  // MCP Side, MCP Forward, PIP, DIP
Thumb:            [-20, 140, 90, 77]     // MCP Side, MCP Forward, PIP, DIP
```

### Angle Conversion
1. **VR Space → LEAP Space**: Add 180° offset (`+ π` radians)
2. **Safety Clipping**: Apply joint limits before sending to robot
3. **Coordinate System**: Convert from VR tracking space to LEAP hand space

### Data Format
- **Old Format**: 5 values (one per finger)
- **New Format**: 16 values (4 joints per finger)
- **Order**: [DIP, PIP, MCP_flex, MCP_abd] for each finger

## Implementation Details

### HandController.cs Changes
- Added quaternion utility functions
- Implemented swing-twist decomposition
- Created calibration system
- Added joint limit safety checks
- Modified finger tracking to use new system

### server_env.py Changes
- Added LEAP hand API integration
- Implemented dual-format support (old/new)
- Added proper error handling
- Included safety clipping

## Troubleshooting

### Common Issues

1. **"Calibration not done"**
   - Press B button while in neutral pose
   - Ensure hand tracking is enabled

2. **"LEAP Hand API not available"**
   - Check Python path configuration
   - Verify LEAP_Hand_API directory exists

3. **"Failed to initialize LEAP hand"**
   - Check USB port configuration
   - Verify LEAP hand is connected
   - Check permissions on serial port

4. **Erratic finger movements**
   - Recalibrate in neutral pose
   - Check joint limits
   - Verify bone tracking quality

### Debug Information
The system provides detailed logging:
- Calibration status
- Joint angle values
- LEAP hand commands
- Error messages

## Future Improvements

1. **Adaptive Calibration**: Automatic calibration based on hand pose
2. **Filtering**: Low-pass filtering to reduce jitter
3. **Grasp Recognition**: Automatic grasp type detection
4. **Haptic Feedback**: Force feedback from LEAP hand
5. **Multi-Hand Support**: Support for both hands simultaneously

## References

- LEAP Hand API Documentation
- Oculus Hand Tracking SDK
- Quaternion Mathematics
- Swing-Twist Decomposition Theory
