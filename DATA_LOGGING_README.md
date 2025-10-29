# Hand Tracking Data Logging

This system now automatically saves all hand tracking data to timestamped log files for analysis.

## Features

- **Automatic Logging**: All hand tracking data is saved to `hand_tracking_data_YYYYMMDD_HHMMSS.txt`
- **Real-time Display**: Data is still shown in the terminal as before
- **Structured Format**: Each joint angle is logged with timestamp, finger, joint name, and angle value
- **Safe Interruption**: Press Ctrl+C to stop and save data properly

## Usage

### 1. Run the Server with Data Logging
```bash
cd /home/rumi/Desktop/leap
python server_env.py
```

The server will:
- Display data in the terminal as usual
- Automatically create a log file (e.g., `hand_tracking_data_20241201_143022.txt`)
- Save all joint angles with timestamps

### 2. Stop the Server
- Press `Ctrl+C` to stop the server
- Data will be automatically saved to the log file
- You'll see a confirmation message with the filename

### 3. Analyze the Data
Use the provided analysis script:

```bash
python analyze_hand_data.py hand_tracking_data_20241201_143022.txt
```

This will:
- Parse the log file
- Display summary statistics
- Generate plots showing joint angles over time
- Create distribution plots

## Log File Format

```
HAND TRACKING DATA LOG
============================================================
Started: 2024-12-01 14:30:22
Format: [timestamp] [finger] [joint] [angle_degrees]
============================================================

[14:30:22.123] HAND_DATA_START
[14:30:22.123] Index DIP 0.00
[14:30:22.123] Index PIP 11.50
[14:30:22.123] Index MCP_Flex 10.44
[14:30:22.123] Index MCP_Abd 10.45
...
[14:30:22.123] HAND_DATA_END

[14:30:22.128] HAND_DATA_START
[14:30:22.128] Index DIP 0.00
...
```

## Analysis Output

The analysis script generates:
- **Summary Statistics**: Min, max, average, and standard deviation for each joint
- **Joint Angle Plots**: Time series plots for each finger's joints
- **Distribution Plots**: Histogram showing the distribution of all joint angles

## Example Analysis

```bash
$ python analyze_hand_data.py hand_tracking_data_20241201_143022.txt

Analyzing hand tracking data from: hand_tracking_data_20241201_143022.txt
Found 1200 data samples

============================================================
HAND TRACKING DATA SUMMARY
============================================================

Index Finger:
       DIP: Min=  0.0° Max= 45.2° Avg= 12.3° Std= 8.7°
       PIP: Min=  8.1° Max= 35.6° Avg= 18.9° Std= 6.2°
  MCP_Flex: Min=  5.2° Max= 42.1° Avg= 22.4° Std= 9.8°
   MCP_Abd: Min= -5.1° Max= 20.0° Avg= 12.7° Std= 7.3°

...

✓ Joint angle plots saved to hand_tracking_data_20241201_143022_joints.png
✓ Angle distribution plot saved to hand_tracking_data_20241201_143022_distribution.png
✓ Analysis complete! Check the generated PNG files.
```

## Files Created

- `hand_tracking_data_YYYYMMDD_HHMMSS.txt` - Raw log data
- `hand_tracking_data_YYYYMMDD_HHMMSS_joints.png` - Joint angle plots
- `hand_tracking_data_YYYYMMDD_HHMMSS_distribution.png` - Angle distribution plot

## Requirements

For analysis, you'll need:
```bash
pip install matplotlib numpy
```

The server itself only needs the existing dependencies.
