#!/usr/bin/env python3
"""
Command-line Pose Controller
Usage: python pose_command.py "peace sign"
"""
import sys
import numpy as np
from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu

def execute_pose(pose_name):
    """Execute a single pose by name"""
    # Pose library
    poses = {
        "open": np.zeros(16),
        "fist": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
        "peace sign": np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
        "thumbs up": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0, 0, 0]),
        "point": np.array([0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
        "ok sign": np.array([0, 0.8, 0.8, 0.8, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0.8, 0.8, 0.8]),
        "half fist": np.array([0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8]),
        "pinch": np.array([0, 0.5, 0.5, 0.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0.5, 0.5, 0.5]),
        "spread": np.array([0.3, 0, 0, 0, -0.3, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0]),
    }
    
    pose_name = pose_name.lower().strip()
    
    if pose_name not in poses:
        print(f"❌ Unknown pose: '{pose_name}'")
        print("Available poses:", list(poses.keys()))
        return False
    
    # Initialize hand
    motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
    dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
    dxl_client.connect()
    
    # Setup control mode
    dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
    dxl_client.set_torque_enabled(motors, True)
    dxl_client.sync_write(motors, np.ones(len(motors)) * 600, 84, 2)
    dxl_client.sync_write([0,4,8], np.ones(3) * 450, 84, 2)
    dxl_client.sync_write(motors, np.ones(len(motors)) * 0, 82, 2)
    dxl_client.sync_write(motors, np.ones(len(motors)) * 200, 80, 2)
    dxl_client.sync_write([0,4,8], np.ones(3) * 150, 80, 2)
    dxl_client.sync_write(motors, np.ones(len(motors)) * 350, 102, 2)
    
    # Get pose
    pose = poses[pose_name]
    print(f"🎭 Executing pose: {pose_name}")
    print(f"   Pose array: {pose}")
    
    # Convert to LEAP convention
    leap_pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
    
    # Handle MCP Side joints specially
    leap_pose[0] = 3.14159 + pose[0]   # Index MCP Side
    leap_pose[4] = 3.14159 + pose[4]   # Middle MCP Side  
    leap_pose[8] = 3.14159 + pose[8]   # Ring MCP Side
    
    print(f"   LEAP array: {leap_pose}")
    
    # Move hand
    dxl_client.write_desired_pos(motors, leap_pose)
    
    print(f"✅ Hand moved to {pose_name}")
    return True

def main():
    """Main function"""
    if len(sys.argv) != 2:
        print("Usage: python pose_command.py \"pose_name\"")
        print("Example: python pose_command.py \"peace sign\"")
        print("Available poses: open, fist, peace sign, thumbs up, point, ok sign, half fist, pinch, spread")
        sys.exit(1)
    
    pose_name = sys.argv[1]
    execute_pose(pose_name)

if __name__ == "__main__":
    main()




