#!/usr/bin/env python3
"""
Test LEAP Hand with Quest 2 Hand Tracking Data

This script extracts 3 different hand positions from the logged Quest 2 data
and sends them to the LEAP hand to test if the angles are usable.
"""

import numpy as np
import time
import sys
import os

# Add the LEAP hand API to the path
sys.path.append('/home/rumi/Desktop/leap/LEAP_Hand_API/python')

try:
    from leap_hand_utils.dynamixel_client import DynamixelClient
    import leap_hand_utils.leap_hand_utils as lhu
    LEAP_AVAILABLE = True
except ImportError as e:
    print(f"⚠ LEAP Hand API not available: {e}")
    print("This script will run in simulation mode")
    LEAP_AVAILABLE = False

def parse_hand_data(filename):
    """Parse the logged hand tracking data and extract 3 different poses"""
    
    poses = []
    current_pose = {}
    in_data_section = False
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    for line in lines:
        line = line.strip()
        
        if 'HAND_DATA_START' in line:
            in_data_section = True
            current_pose = {}
            continue
            
        if 'HAND_DATA_END' in line:
            in_data_section = False
            if len(current_pose) == 16:  # All 16 joints captured
                poses.append(current_pose.copy())
            continue
            
        if in_data_section and line.startswith('['):
            # Parse joint data: [timestamp] Finger Joint angle
            import re
            joint_match = re.match(r'\[.*?\] (\w+) (\w+) ([\d.-]+)', line)
            if joint_match:
                finger = joint_match.group(1)
                joint = joint_match.group(2)
                angle = float(joint_match.group(3))
                
                # Convert to LEAP hand joint index
                joint_idx = get_leap_joint_index(finger, joint)
                if joint_idx is not None:
                    current_pose[joint_idx] = angle
    
    print(f"✓ Parsed {len(poses)} complete hand poses from log file")
    return poses

def get_leap_joint_index(finger, joint):
    """Convert finger/joint names to LEAP hand joint indices"""
    
    # LEAP hand joint mapping:
    # Index (0-3), Middle(4-7), Ring(8-11), Thumb(12-15)
    # Each finger: [MCP_Side, MCP_Forward, PIP, DIP]
    
    finger_map = {
        'Index': 0,
        'Middle': 4, 
        'Ring': 8,
        'Thumb': 12
    }
    
    joint_map = {
        'MCP_Abd': 0,    # MCP_Side (abduction)
        'MCP_Flex': 1,   # MCP_Forward (flexion)
        'PIP': 2,        # PIP
        'DIP': 3         # DIP
    }
    
    if finger in finger_map and joint in joint_map:
        return finger_map[finger] + joint_map[joint]
    return None

def convert_quest_to_leap(quest_angles):
    """Convert Quest 2 angles (degrees) to LEAP hand format (radians)"""
    
    # Convert degrees to radians
    leap_angles = np.zeros(16)
    
    for joint_idx, angle_deg in quest_angles.items():
        if joint_idx < 16:
            # Convert degrees to radians
            angle_rad = np.radians(angle_deg)
            
            # Apply LEAP hand conversion (add π offset)
            leap_angles[joint_idx] = angle_rad + np.pi
            
            # Special handling for MCP abduction (side joints)
            if joint_idx in [0, 4, 8]:  # MCP_Side joints
                # Scale abduction to reasonable range for LEAP hand
                leap_angles[joint_idx] = np.pi + (angle_rad * 0.3)  # Scale down abduction
    
    # Apply safety clipping
    leap_angles = lhu.angle_safety_clip(leap_angles)
    
    return leap_angles

def test_leap_hand_connection():
    """Test if LEAP hand is connected and working"""
    
    if not LEAP_AVAILABLE:
        print("⚠ LEAP Hand API not available - running in simulation mode")
        return None
    
    try:
        # Try to connect to LEAP hand
        motors = list(range(16))
        dxl_client = DynamixelClient(motors, '/dev/ttyUSB0', 4000000)
        dxl_client.connect()
        print("✓ Connected to LEAP hand on /dev/ttyUSB0")
        return dxl_client
    except Exception as e:
        print(f"✗ Failed to connect to LEAP hand: {e}")
        return None

def send_pose_to_leap(dxl_client, pose_radians, pose_name):
    """Send a pose to the LEAP hand"""
    
    if dxl_client is None:
        print(f"📋 {pose_name} (Simulation): {pose_radians}")
        return
    
    try:
        # Send pose to LEAP hand
        dxl_client.write_desired_pos(list(range(16)), pose_radians)
        print(f"✓ Sent {pose_name} to LEAP hand")
        
        # Read back current position
        current_pos = dxl_client.read_pos()
        print(f"📊 Current LEAP hand position: {current_pos}")
        
    except Exception as e:
        print(f"✗ Error sending pose to LEAP hand: {e}")

def main():
    print("=" * 80)
    print("LEAP HAND TEST - Quest 2 Hand Tracking Data")
    print("=" * 80)
    
    # Parse the logged data
    log_file = "hand_tracking_data_20251010_130037.txt"
    if not os.path.exists(log_file):
        print(f"✗ Log file not found: {log_file}")
        return
    
    poses = parse_hand_data(log_file)
    if len(poses) < 3:
        print(f"✗ Not enough poses found: {len(poses)}")
        return
    
    # Select 3 different poses (start, middle, end)
    test_poses = [
        (poses[0], "Opening Pose (Start)"),
        (poses[len(poses)//2], "Middle Pose"), 
        (poses[-1], "Closing Pose (End)")
    ]
    
    print(f"\n📊 Selected {len(test_poses)} test poses from {len(poses)} total poses")
    
    # Test LEAP hand connection
    dxl_client = test_leap_hand_connection()
    
    if dxl_client:
        # Configure LEAP hand
        motors = list(range(16))
        dxl_client.sync_write(motors, np.ones(16)*5, 11, 1)
        dxl_client.set_torque_enabled(motors, True)
        dxl_client.sync_write(motors, np.ones(16) * 600, 84, 2)  # P-gain
        dxl_client.sync_write(motors, np.ones(16) * 0, 82, 2)    # I-gain  
        dxl_client.sync_write(motors, np.ones(16) * 200, 80, 2)  # D-gain
        dxl_client.sync_write(motors, np.ones(16) * 350, 102, 2) # Current limit
        print("✓ LEAP hand configured")
    
    # Test each pose
    for i, (quest_pose, pose_name) in enumerate(test_poses):
        print(f"\n{'='*60}")
        print(f"TESTING POSE {i+1}: {pose_name}")
        print(f"{'='*60}")
        
        # Show original Quest 2 angles
        print("📱 Quest 2 angles (degrees):")
        for joint_idx in sorted(quest_pose.keys()):
            finger = ["Index", "Middle", "Ring", "Thumb"][joint_idx // 4]
            joint = ["MCP_Abd", "MCP_Flex", "PIP", "DIP"][joint_idx % 4]
            print(f"  {finger} {joint}: {quest_pose[joint_idx]:.2f}°")
        
        # Convert to LEAP hand format
        leap_pose = convert_quest_to_leap(quest_pose)
        
        print(f"\n🤖 LEAP hand angles (radians):")
        for i, angle in enumerate(leap_pose):
            finger = ["Index", "Middle", "Ring", "Thumb"][i // 4]
            joint = ["MCP_Abd", "MCP_Flex", "PIP", "DIP"][i % 4]
            print(f"  {finger} {joint}: {angle:.3f} rad ({np.degrees(angle):.1f}°)")
        
        # Send to LEAP hand
        send_pose_to_leap(dxl_client, leap_pose, pose_name)
        
        # Wait between poses
        if i < len(test_poses) - 1:
            print(f"\n⏳ Waiting 3 seconds before next pose...")
            time.sleep(3)
    
    print(f"\n{'='*80}")
    print("TEST COMPLETE")
    print("="*80)
    
    if dxl_client:
        print("✓ All poses sent to LEAP hand successfully!")
        print("📊 Check the physical LEAP hand to see if it moved as expected")
    else:
        print("📋 Simulation complete - check the angle conversions above")
        print("💡 If the angles look reasonable, try connecting to a real LEAP hand")

if __name__ == "__main__":
    main()
