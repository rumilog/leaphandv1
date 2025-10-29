#!/usr/bin/env python3
"""
Run LEAP Hand with Quest 2 Hand Tracking Poses

This script cycles through different hand poses from the logged Quest 2 data
and sends them to the LEAP hand to test the complete system.
"""

import numpy as np
import time
import sys
import os
import signal

# Add the LEAP hand API to the path
sys.path.append('/home/rumi/Desktop/leap/LEAP_Hand_API/python')

try:
    from leap_hand_utils.dynamixel_client import DynamixelClient
    import leap_hand_utils.leap_hand_utils as lhu
    LEAP_AVAILABLE = True
    print("✓ LEAP Hand API loaded successfully")
except ImportError as e:
    print(f"⚠ LEAP Hand API not available: {e}")
    print("This script will run in simulation mode")
    LEAP_AVAILABLE = False

class LeapHandController:
    def __init__(self):
        self.dxl_client = None
        self.motors = list(range(16))
        self.is_connected = False
        
        if LEAP_AVAILABLE:
            self.connect_to_leap_hand()
        else:
            print("📋 Running in simulation mode - no LEAP hand connection")
    
    def connect_to_leap_hand(self):
        """Connect to the LEAP hand"""
        try:
            # Try different ports
            ports_to_try = ['/dev/ttyUSB0', '/dev/ttyUSB1', 'COM13']
            
            for port in ports_to_try:
                try:
                    print(f"🔌 Trying to connect to LEAP hand on {port}...")
                    self.dxl_client = DynamixelClient(self.motors, port, 4000000)
                    self.dxl_client.connect()
                    print(f"✓ Connected to LEAP hand on {port}")
                    self.is_connected = True
                    self.configure_leap_hand()
                    return
                except Exception as e:
                    print(f"✗ Failed to connect on {port}: {e}")
                    continue
            
            print("✗ Could not connect to LEAP hand on any port")
            self.dxl_client = None
            
        except Exception as e:
            print(f"✗ LEAP hand connection failed: {e}")
            self.dxl_client = None
    
    def configure_leap_hand(self):
        """Configure LEAP hand parameters"""
        if not self.is_connected:
            return
            
        try:
            print("⚙️ Configuring LEAP hand...")
            
            # Enable position-current control mode
            self.dxl_client.sync_write(self.motors, np.ones(16)*5, 11, 1)
            self.dxl_client.set_torque_enabled(self.motors, True)
            
            # Set PID parameters
            self.dxl_client.sync_write(self.motors, np.ones(16) * 600, 84, 2)  # P-gain (stiffness)
            self.dxl_client.sync_write([0,4,8], np.ones(3) * 450, 84, 2)      # Lower P-gain for side joints
            self.dxl_client.sync_write(self.motors, np.ones(16) * 0, 82, 2)   # I-gain
            self.dxl_client.sync_write(self.motors, np.ones(16) * 200, 80, 2)  # D-gain (damping)
            self.dxl_client.sync_write([0,4,8], np.ones(3) * 150, 80, 2)      # Lower D-gain for side joints
            
            # Set current limit (350 for lite, 550 for full hand)
            self.dxl_client.sync_write(self.motors, np.ones(16) * 350, 102, 2)
            
            print("✓ LEAP hand configured successfully")
            
        except Exception as e:
            print(f"✗ Error configuring LEAP hand: {e}")
    
    def send_pose(self, pose_radians, pose_name="Unknown"):
        """Send a pose to the LEAP hand"""
        if not self.is_connected:
            print(f"📋 {pose_name} (Simulation): {pose_radians}")
            return True
        
        try:
            # Apply safety clipping
            safe_pose = lhu.angle_safety_clip(pose_radians)
            
            # Send pose to LEAP hand
            self.dxl_client.write_desired_pos(self.motors, safe_pose)
            print(f"✓ Sent {pose_name} to LEAP hand")
            
            # Read back current position for verification
            current_pos = self.dxl_client.read_pos()
            print(f"📊 Current position: {current_pos[:4]}... (showing first 4 joints)")
            
            return True
            
        except Exception as e:
            print(f"✗ Error sending pose to LEAP hand: {e}")
            return False
    
    def read_position(self):
        """Read current LEAP hand position"""
        if not self.is_connected:
            return None
        try:
            return self.dxl_client.read_pos()
        except Exception as e:
            print(f"✗ Error reading position: {e}")
            return None

def parse_hand_data(filename):
    """Parse the logged hand tracking data and extract poses"""
    
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
    
    return poses

def get_leap_joint_index(finger, joint):
    """Convert finger/joint names to LEAP hand joint indices"""
    
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
    
    leap_angles = np.zeros(16)
    
    for joint_idx, angle_deg in quest_angles.items():
        if joint_idx < 16:
            # Convert degrees to radians
            angle_rad = np.radians(angle_deg)
            
            # Apply LEAP hand conversion (add π offset)
            leap_angles[joint_idx] = angle_rad + np.pi
            
            # Special handling for MCP abduction (side joints) - scale down
            if joint_idx in [0, 4, 8]:  # MCP_Side joints
                leap_angles[joint_idx] = np.pi + (angle_rad * 0.2)  # Scale down abduction
    
    return leap_angles

def print_pose_info(quest_pose, leap_pose, pose_name):
    """Print detailed information about a pose"""
    print(f"\n{'='*60}")
    print(f"POSE: {pose_name}")
    print(f"{'='*60}")
    
    print("📱 Quest 2 angles (degrees):")
    for joint_idx in sorted(quest_pose.keys()):
        finger = ["Index", "Middle", "Ring", "Thumb"][joint_idx // 4]
        joint = ["MCP_Abd", "MCP_Flex", "PIP", "DIP"][joint_idx % 4]
        print(f"  {finger} {joint}: {quest_pose[joint_idx]:6.2f}°")
    
    print(f"\n🤖 LEAP hand angles (radians):")
    for i, angle in enumerate(leap_pose):
        finger = ["Index", "Middle", "Ring", "Thumb"][i // 4]
        joint = ["MCP_Abd", "MCP_Flex", "PIP", "DIP"][i % 4]
        print(f"  {finger} {joint}: {angle:6.3f} rad ({np.degrees(angle):6.1f}°)")

def main():
    print("=" * 80)
    print("LEAP HAND POSE RUNNER - Quest 2 Hand Tracking Data")
    print("=" * 80)
    
    # Parse the logged data
    log_file = "hand_tracking_data_20251010_130037.txt"
    if not os.path.exists(log_file):
        print(f"✗ Log file not found: {log_file}")
        return
    
    print(f"📂 Loading poses from {log_file}...")
    poses = parse_hand_data(log_file)
    print(f"✓ Loaded {len(poses)} poses from log file")
    
    if len(poses) < 5:
        print(f"✗ Not enough poses found: {len(poses)}")
        return
    
    # Initialize LEAP hand controller
    controller = LeapHandController()
    
    # Select test poses (start, 25%, 50%, 75%, end)
    test_indices = [0, len(poses)//4, len(poses)//2, 3*len(poses)//4, len(poses)-1]
    test_poses = [(poses[i], f"Pose {j+1}") for j, i in enumerate(test_indices)]
    
    print(f"\n🎯 Selected {len(test_poses)} test poses")
    
    # Set up signal handler for graceful shutdown
    def signal_handler(sig, frame):
        print(f"\n\n🛑 Interrupted! Stopping LEAP hand...")
        if controller.is_connected:
            # Move to neutral position
            neutral_pose = np.ones(16) * np.pi  # All joints at π (flat position)
            controller.send_pose(neutral_pose, "Neutral (Shutdown)")
        print("✓ LEAP hand stopped safely")
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Run through each pose
        for i, (quest_pose, pose_name) in enumerate(test_poses):
            print(f"\n{'='*80}")
            print(f"RUNNING POSE {i+1}/{len(test_poses)}: {pose_name}")
            print(f"{'='*80}")
            
            # Convert to LEAP hand format
            leap_pose = convert_quest_to_leap(quest_pose)
            
            # Print detailed pose information
            print_pose_info(quest_pose, leap_pose, pose_name)
            
            # Send pose to LEAP hand
            success = controller.send_pose(leap_pose, pose_name)
            
            if not success:
                print("✗ Failed to send pose to LEAP hand")
                continue
            
            # Wait between poses
            if i < len(test_poses) - 1:
                print(f"\n⏳ Holding pose for 4 seconds...")
                time.sleep(4)
        
        print(f"\n{'='*80}")
        print("🎉 ALL POSES COMPLETED SUCCESSFULLY!")
        print("="*80)
        
        if controller.is_connected:
            print("✓ All poses sent to LEAP hand")
            print("📊 Check the physical LEAP hand to verify the movements")
        else:
            print("📋 Simulation completed - check the angle conversions above")
            print("💡 If the angles look reasonable, try connecting to a real LEAP hand")
        
        # Return to neutral position
        print(f"\n🔄 Returning to neutral position...")
        neutral_pose = np.ones(16) * np.pi  # All joints at π (flat position)
        controller.send_pose(neutral_pose, "Neutral (Final)")
        
    except KeyboardInterrupt:
        print(f"\n\n🛑 Interrupted by user")
    except Exception as e:
        print(f"\n✗ Error during execution: {e}")
    finally:
        if controller.is_connected:
            print("🔄 Returning to neutral position...")
            neutral_pose = np.ones(16) * np.pi
            controller.send_pose(neutral_pose, "Neutral (Cleanup)")

if __name__ == "__main__":
    main()
