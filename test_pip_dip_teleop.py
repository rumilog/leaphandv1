#!/usr/bin/env python3
"""
Test PIP/DIP Teleoperation

Receives 10 PIP/DIP values from Quest 2 via the server
and controls only PIP and DIP joints on the LEAP hand.
"""

import numpy as np
import time
import sys

sys.path.append('/home/rumi/Desktop/leap/LEAP_Hand_API/python')
from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu
import UdpComms as U

def main():
    print("=" * 60)
    print("PIP/DIP TELEOPERATION TEST")
    print("=" * 60)
    
    # Initialize LEAP hand
    print("\n🔌 Connecting to LEAP hand...")
    motors = list(range(16))
    
    try:
        leap_hand = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
        leap_hand.connect()
        print("✓ Connected to LEAP hand")
        
        # Configure for position control
        leap_hand.sync_write(motors, np.ones(16)*5, 11, 1)
        leap_hand.set_torque_enabled(motors, True)
        leap_hand.sync_write(motors, np.ones(16) * 600, 84, 2)
        leap_hand.sync_write(motors, np.ones(16) * 0, 82, 2)
        leap_hand.sync_write(motors, np.ones(16) * 200, 80, 2)
        leap_hand.sync_write(motors, np.ones(16) * 400, 102, 2)
        
        # Set initial pose
        initial_pose = np.ones(16) * 3.14
        leap_hand.write_desired_pos(motors, initial_pose)
        print("✓ LEAP hand initialized")
        
        # Initialize UDP
        print("\n📡 Setting up UDP...")
        sock = U.UdpComms(udpIP="172.26.71.187", sendIP="172.26.109.148", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)
        print("✓ UDP ready")
        
        print("\n🚀 Starting teleoperation...")
        print("🛑 Press Ctrl+C to stop\n")
        
        frame_count = 0
        
        while True:
            data = sock.ReadReceivedData()
            
            if data is not None:
                try:
                    # Parse the data - format: inventory\nunknown_objects\nhand_pose\ngripper_message
                    parts = data.split('\n')
                    
                    if len(parts) >= 4:
                        gripper_message = parts[3].rstrip('\t')
                        pip_dip_values = np.array([float(x) for x in gripper_message.split('\t')])
                        
                        if len(pip_dip_values) == 10:
                            # Create LEAP pose: only set PIP and DIP, keep MCP neutral
                            leap_pose = np.ones(16) * 3.14  # All neutral
                            
                            # Apply PIP/DIP to LEAP joints
                            # Format: [PIP_Thumb, DIP_Thumb, PIP_Index, DIP_Index, PIP_Middle, DIP_Middle, PIP_Ring, DIP_Ring, PIP_Pinky, DIP_Pinky]
                            # Index finger (joints 2, 3): PIP, DIP
                            leap_pose[2] = 3.14 + pip_dip_values[2]  # Index PIP (not inverted)
                            leap_pose[3] = 3.14 + pip_dip_values[3]  # Index DIP (not inverted)
                            
                            # Middle finger (joints 6, 7): PIP, DIP
                            leap_pose[6] = 3.14 + pip_dip_values[4]  # Middle PIP
                            leap_pose[7] = 3.14 + pip_dip_values[5]  # Middle DIP
                            
                            # Ring finger (joints 10, 11): PIP, DIP
                            leap_pose[10] = 3.14 + pip_dip_values[6]  # Ring PIP
                            leap_pose[11] = 3.14 + pip_dip_values[7]  # Ring DIP
                            
                            # Thumb (joints 14, 15): PIP, DIP
                            leap_pose[14] = 3.14 + pip_dip_values[0]  # Thumb PIP
                            leap_pose[15] = 3.14 + pip_dip_values[1]  # Thumb DIP
                            
                            # Send to LEAP hand
                            safe_pose = lhu.angle_safety_clip(leap_pose)
                            leap_hand.write_desired_pos(motors, safe_pose)
                            
                            frame_count += 1
                            if frame_count % 50 == 0:
                                print(f"📊 Frame {frame_count} - Index PIP: {np.degrees(pip_dip_values[2]):.1f}°, DIP: {np.degrees(pip_dip_values[3]):.1f}°")
                
                except Exception as e:
                    print(f"⚠ Error: {e}")
            
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Stopping teleoperation...")
        
        # Return to neutral
        neutral_pose = np.ones(16) * 3.14
        leap_hand.write_desired_pos(motors, neutral_pose)
        time.sleep(1)
        
        print("✓ Done")

if __name__ == "__main__":
    main()
