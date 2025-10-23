#!/usr/bin/env python3
"""
Simple NumPy Array Test for LEAP Hand
Easy way to test numpy array control
"""
import numpy as np
import time
from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu

def test_numpy_poses():
    """Test various numpy array poses"""
    print("🤖 Testing NumPy Array Control")
    
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
    
    # Test poses (Allegro convention)
    poses = {
        "Open Hand": np.zeros(16),
        "Half Fist": np.array([0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8]),
        "Fist": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
        "Index Point": np.array([0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
        "Peace Sign": np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
        "Thumbs Up": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0, 0, 0]),
        "MCP Side Test": np.array([0.3, 0, 0, 0, -0.3, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0])
    }
    
    for name, pose in poses.items():
        print(f"\n🎭 {name}")
        print(f"   Pose: {pose}")
        
        # Convert to LEAP convention
        leap_pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        
        # Handle MCP Side joints specially
        leap_pose[0] = 3.14159 + pose[0]   # Index MCP Side
        leap_pose[4] = 3.14159 + pose[4]   # Middle MCP Side  
        leap_pose[8] = 3.14159 + pose[8]   # Ring MCP Side
        
        print(f"   LEAP: {leap_pose}")
        
        # Move hand
        dxl_client.write_desired_pos(motors, leap_pose)
        time.sleep(2)
        
        # Read actual position
        actual = dxl_client.read_pos()
        print(f"   Actual: {actual}")
    
    print("\n✅ Test complete!")

if __name__ == "__main__":
    test_numpy_poses()
