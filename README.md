# LEAP Hand Teleoperation Setup

This project enables real-time teleoperation of the LEAP Hand using hand tracking data from Meta Quest 2.

## Prerequisites

1. **LEAP Hand Hardware**
   - LEAP Hand powered on
   - USB-to-serial connection (typically `/dev/ttyUSB0` on Linux)

2. **Quest 2 Setup**
   - Quest 2 powered on and connected to the same Wi-Fi network as your PC
   - Unity app with `HandController.cs` running on Quest 2

3. **Network Configuration**
   - PC and Quest 2 must be on the same network
   - Update IP addresses in both `server_env.py` and `realtime_teleop_pip_dip.py` to match your network

## Running the Teleoperation Program

### Step 1: Kill Any Existing Processes

**Always kill existing processes first to avoid conflicts:**

```bash
cd /home/rumi/Desktop/leap
pkill -f realtime_teleop_pip_dip.py
pkill -f server_env.py
```

### Step 2: Start the UDP Server

Start the server in the first terminal (receives data from Quest 2):

```bash
cd /home/rumi/Desktop/leap
python3 server_env.py
```

This will:
- Listen for UDP packets from Quest 2
- Display PIP/DIP joint angles in real-time
- Log data to timestamped files (see `DATA_LOGGING_README.md`)

### Step 3: Start the Teleoperation Controller

In a **new terminal**, start the teleoperation script:

```bash
cd /home/rumi/Desktop/leap
python3 realtime_teleop_pip_dip.py
```

This will:
- Connect to the LEAP Hand
- Receive PIP/DIP data from the server
- Control LEAP Hand fingers to mirror your hand movements

### Step 4: Test

Place your hand in view of the Quest 2. The LEAP Hand should now mirror your finger movements (PIP and DIP joints only).

## Running in Background (Optional)

To run both processes in the background:

```bash
cd /home/rumi/Desktop/leap

# Kill existing processes first
pkill -f realtime_teleop_pip_dip.py
pkill -f server_env.py

# Start server in background
nohup python3 server_env.py > server_env.out 2>&1 &

# Start teleop in background
nohup python3 realtime_teleop_pip_dip.py > teleop.out 2>&1 &
```

### Viewing Background Logs

To see output from background processes:

```bash
# Server logs
tail -f /home/rumi/Desktop/leap/server_env.out

# Teleop logs
tail -f /home/rumi/Desktop/leap/teleop.out
```

## Stopping the Program

To stop all processes:

```bash
pkill -f realtime_teleop_pip_dip.py
pkill -f server_env.py
```

Or press `Ctrl+C` if running in foreground terminals.

## Troubleshooting

### LEAP Hand Not Moving

1. **Check Quest 2 is running**: Ensure the Unity app is active on Quest 2
2. **Verify network**: Check IP addresses in `server_env.py` and `realtime_teleop_pip_dip.py` match your network
3. **Check USB connection**: Verify LEAP Hand is connected (usually `/dev/ttyUSB0`)
4. **Check logs**: Review output files or terminal output for error messages

### Inverted Movement

If the LEAP hand moves in the opposite direction:
- The angle mapping in `realtime_teleop_pip_dip.py` can be adjusted
- Current mapping: `3.14 + angle` (positive closes, negative opens)
- If inverted, change to: `3.14 - angle`

### No Data Received

1. Verify Quest 2 Unity app is sending UDP packets
2. Check firewall settings aren't blocking UDP ports 8000/8001
3. Verify IP addresses are correct in both scripts

## Files Overview

- **`HandController.cs`**: Unity script running on Quest 2 that captures hand tracking and sends PIP/DIP angles
- **`server_env.py`**: Python server that receives UDP data from Quest 2 and logs it
- **`realtime_teleop_pip_dip.py`**: Python script that receives data and controls the LEAP Hand
- **`UdpComms.py`**: UDP communication library

## Current Configuration

- **Joints Controlled**: PIP and DIP only (MCP joints stay at neutral `3.14` radians)
- **Update Rate**: ~100 Hz (10ms delay)
- **Data Format**: 10 values (PIP and DIP for 5 fingers: Thumb, Index, Middle, Ring, Pinky)
- **Angle Units**: Radians (converted from degrees on Quest 2)

## See Also

- `DATA_LOGGING_README.md` - Information about data logging and analysis

