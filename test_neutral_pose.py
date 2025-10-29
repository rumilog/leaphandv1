#!/usr/bin/env python3
"""
Test Neutral Pose for LEAP Hand

Simple script to set the LEAP hand to a neutral "stop sign" pose
"""

import numpy as np
import time
import sys

# Add the LEAP hand API to the path
sys.path.append('/home/rumi/Desktop/leap/LEAP_Hand_API/python')

try:
    from leap_hand_utils.dynamixel_client import DynamixelClient
    import leap_hand_utils.leap_hand_utils as lhu
    LEAP_AVAILABLE = True
    print("✓ LEAP Hand API loaded successfully")
except ImportError as e:
    print(f"⚠ LEAP Hand API not available: {e}")
    LEAP_AVAILABLE = False

def test_neutral_pose():
    """Test setting LEAP hand to neutral pose"""
    
    if not LEAP_AVAILABLE:
        print("✗ LEAP Hand API not available")
        return
    
    try:
        print("🔌 Connecting to LEAP hand...")
        motors = list(range(16))
        dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
        dxl_client.connect()
        print("✓ Connected to LEAP hand")
        
        # Configure LEAP hand
        print("⚙️ Configuring LEAP hand...")
        dxl_client.sync_write(motors, np.ones(16)*5, 11, 1)
        dxl_client.set_torque_enabled(motors, True)
        dxl_client.sync_write(motors, np.ones(16) * 600, 84, 2)  # P-gain
        dxl_client.sync_write(motors, np.ones(16) * 0, 82, 2)    # I-gain
        dxl_client.sync_write(motors, np.ones(16) * 200, 80, 2)  # D-gain
        dxl_client.sync_write(motors, np.ones(16) * 400, 102, 2) # Current limit
        
        # Set neutral "stop sign" pose
        print("🖐️ Setting neutral 'stop sign' pose...")
        neutral_pose = np.array([
            # Index finger (joints 0-3): MCP_Abd, MCP_Flex, PIP, DIP
            3.14, 3.14, 3.14, 3.14,  # Index: straight, no abduction
            # Middle finger (joints 4-7): MCP_Abd, MCP_Flex, PIP, DIP  
            3.14, 3.14, 3.14, 3.14,  # Middle: straight, no abduction
            # Ring finger (joints 8-11): MCP_Abd, MCP_Flex, PIP, DIP
            3.14, 3.14, 3.14, 3.14,  # Ring: straight, no abduction
            # Thumb (joints 12-15): MCP_Abd, MCP_Flex, PIP, DIP
            3.14, 3.14, 3.14, 3.14   # Thumb: straight, no abduction
        ])
        
        # Send neutral pose
        dxl_client.write_desired_pos(motors, neutral_pose)
        print("✓ Neutral pose sent - fingers should be straight")
        
        # Wait for movement
        print("⏳ Waiting for movement to complete...")
        time.sleep(3)
        
        # Read current position
        current_pos = dxl_client.read_pos()
        print(f"📊 Current position: {current_pos}")
        
        print("✅ Neutral pose test complete!")
        
    except Exception as e:
        print(f"✗ Error: {e}")

if __name__ == "__main__":
    test_neutral_pose()
