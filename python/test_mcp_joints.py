#!/usr/bin/env python3
"""
Test MCP Side Joint Movement
Test the MCP Side joints (0, 4, 8, 12) specifically to see if they move
"""
import numpy as np
import time
from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu

def test_mcp_joints():
    """Test MCP Side joint movement"""
    print("🔍 Testing MCP Side Joint Movement")
    print("=" * 50)
    
    # Initialize hand
    motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
    kP = 600
    kI = 0
    kD = 200
    curr_lim = 350
    
    try:
        dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
        dxl_client.connect()
    except Exception as e:
        print(f"Error connecting: {e}")
        return
    
    # Setup control mode
    dxl_client.sync_write(motors, np.ones(len(motors))*5, 11, 1)
    dxl_client.set_torque_enabled(motors, True)
    dxl_client.sync_write(motors, np.ones(len(motors)) * kP, 84, 2)
    dxl_client.sync_write([0,4,8], np.ones(3) * (kP * 0.75), 84, 2)  # MCP Side joints get 75% P gain
    dxl_client.sync_write(motors, np.ones(len(motors)) * kI, 82, 2)
    dxl_client.sync_write(motors, np.ones(len(motors)) * kD, 80, 2)
    dxl_client.sync_write([0,4,8], np.ones(3) * (kD * 0.75), 80, 2)  # MCP Side joints get 75% D gain
    dxl_client.sync_write(motors, np.ones(len(motors)) * curr_lim, 102, 2)
    
    # Start with open hand
    open_pose = np.zeros(16)
    leap_pose = lhu.allegro_to_LEAPhand(open_pose)
    dxl_client.write_desired_pos(motors, leap_pose)
    time.sleep(2)
    
    print("Starting with open hand...")
    print(f"Open pose (Allegro): {open_pose}")
    print(f"Open pose (LEAP): {leap_pose}")
    print()
    
    # Test each MCP Side joint individually
    mcp_joints = [0, 4, 8, 12]  # MCP Side joints
    finger_names = ["Index", "Middle", "Ring", "Thumb"]
    
    for i, joint_idx in enumerate(mcp_joints):
        print(f"Testing {finger_names[i]} MCP Side (joint {joint_idx})...")
        
        # Test positive movement (should move away from center)
        test_pose = np.zeros(16)
        test_pose[joint_idx] = 0.3  # Small positive value
        leap_test = lhu.allegro_to_LEAPhand(test_pose, zeros=False)
        
        # Handle MCP Side joints specially
        if joint_idx in [0, 4, 8]:  # MCP Side joints
            leap_test[joint_idx] = 3.14159 + test_pose[joint_idx]
        
        print(f"  Moving to +0.3 rad (Allegro) = {leap_test[joint_idx]:.3f} rad (LEAP)")
        dxl_client.write_desired_pos(motors, leap_test)
        time.sleep(2)
        
        # Read actual position
        actual_pos = dxl_client.read_pos()
        print(f"  Actual position: {actual_pos[joint_idx]:.3f} rad (LEAP)")
        print(f"  Difference: {abs(leap_test[joint_idx] - actual_pos[joint_idx]):.3f} rad")
        
        # Test negative movement
        test_pose[joint_idx] = -0.3  # Small negative value
        leap_test = lhu.allegro_to_LEAPhand(test_pose, zeros=False)
        
        # Handle MCP Side joints specially
        if joint_idx in [0, 4, 8]:  # MCP Side joints
            leap_test[joint_idx] = 3.14159 + test_pose[joint_idx]
        
        print(f"  Moving to -0.3 rad (Allegro) = {leap_test[joint_idx]:.3f} rad (LEAP)")
        dxl_client.write_desired_pos(motors, leap_test)
        time.sleep(2)
        
        # Read actual position
        actual_pos = dxl_client.read_pos()
        print(f"  Actual position: {actual_pos[joint_idx]:.3f} rad (LEAP)")
        print(f"  Difference: {abs(leap_test[joint_idx] - actual_pos[joint_idx]):.3f} rad")
        
        # Return to center
        test_pose[joint_idx] = 0.0
        leap_test = lhu.allegro_to_LEAPhand(test_pose, zeros=False)
        
        # Handle MCP Side joints specially
        if joint_idx in [0, 4, 8]:  # MCP Side joints
            leap_test[joint_idx] = 3.14159 + test_pose[joint_idx]
        dxl_client.write_desired_pos(motors, leap_test)
        time.sleep(1)
        
        print()
    
    # Final status
    print("Final hand status:")
    final_pos = dxl_client.read_pos()
    for i, joint_idx in enumerate(mcp_joints):
        print(f"  {finger_names[i]} MCP Side: {final_pos[joint_idx]:.3f} rad (LEAP)")
    
    print("\n✅ MCP Side joint test complete!")

if __name__ == "__main__":
    test_mcp_joints()
