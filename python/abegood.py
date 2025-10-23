#!/usr/bin/env python3
"""
Simple NumPy Array Test for LEAP Hand
Easy way to test numpy array control
"""
import numpy as np
import time
# from leap_hand_utils.dynamixel_client import DynamixelClient
# import leap_hand_utils.leap_hand_utils as lhu
from main import LeapNode

def test_numpy_poses():
    """Test various numpy array poses"""
    print("🤖 Testing NumPy Array Control")
    
    # initialize the hand
    leap_node = LeapNode()
    
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
        
        leap_node.set_allegro(pose)
        time.sleep(2)
        
        # Read actual position
        actual = leap_node.read_pos()
        print(f"   Actual: {actual}")
    
    print("\n✅ Test complete!")

if __name__ == "__main__":
    test_numpy_poses()
