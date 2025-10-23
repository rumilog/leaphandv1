#!/usr/bin/env python3
"""
Interactive LEAP Hand Control
Control individual fingers with keyboard input
"""
import numpy as np
import time
import sys
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu

class InteractiveLeapHand:
    def __init__(self):
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350  # Set to 550 for full hand
        self.motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        
        # Connect to hand
        self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB0', 4000000)
        self.dxl_client.connect()
        
        # Setup control mode
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)
        
        # Current pose (in Allegro convention)
        self.current_pose = np.zeros(16)
        self.dxl_client.write_desired_pos(self.motors, lhu.allegro_to_LEAPhand(self.current_pose))
        
        print("🎮 Interactive LEAP Hand Control Ready!")
        print_instructions()
    
    def update_hand(self):
        """Update hand with current pose"""
        leap_pose = lhu.allegro_to_LEAPhand(self.current_pose)
        self.dxl_client.write_desired_pos(self.motors, leap_pose)
    
    def move_finger(self, finger, joint, direction):
        """Move a specific finger joint"""
        joint_idx = finger * 4 + joint
        if 0 <= joint_idx < 16:
            # Move in 0.1 radian steps
            step = 0.1 * direction
            self.current_pose[joint_idx] = np.clip(self.current_pose[joint_idx] + step, -2.0, 2.0)
            self.update_hand()
            print(f"Finger {finger}, Joint {joint}: {self.current_pose[joint_idx]:.2f} rad")
    
    def reset_pose(self):
        """Reset to open position"""
        self.current_pose = np.zeros(16)
        self.update_hand()
        print("🔄 Reset to open position")
    
    def show_status(self):
        """Show current joint positions"""
        positions = self.dxl_client.read_pos()
        print("\n📊 Current Joint Positions:")
        finger_names = ["Index", "Middle", "Ring", "Thumb"]
        joint_names = ["MCP Side", "MCP Forward", "PIP", "DIP"]
        
        for i in range(4):
            print(f"\n{finger_names[i]} Finger:")
            for j in range(4):
                joint_idx = i * 4 + j
                print(f"  {joint_names[j]}: {positions[joint_idx]:.3f} rad")

def print_instructions():
    print("\n🎯 Controls:")
    print("  Index Finger:    1,2,3,4 (MCP Side, MCP Forward, PIP, DIP)")
    print("  Middle Finger:   5,6,7,8")
    print("  Ring Finger:     9,0,q,w") 
    print("  Thumb:           e,r,t,y")
    print("  + to close, - to open")
    print("  r to reset, s for status, q to quit")
    print("\nExample: '1+' closes Index MCP Side, '1-' opens it")

def main():
    try:
        hand = InteractiveLeapHand()
        
        while True:
            command = input("\nEnter command: ").strip().lower()
            
            if command == 'q':
                print("👋 Goodbye!")
                break
            elif command == 'r':
                hand.reset_pose()
            elif command == 's':
                hand.show_status()
            elif len(command) == 2 and command[1] in '+-':
                finger_joint = command[0]
                direction = 1 if command[1] == '+' else -1
                
                # Map keys to finger/joint
                finger_map = {
                    '1': (0, 0), '2': (0, 1), '3': (0, 2), '4': (0, 3),  # Index
                    '5': (1, 0), '6': (1, 1), '7': (1, 2), '8': (1, 3),  # Middle
                    '9': (2, 0), '0': (2, 1), 'q': (2, 2), 'w': (2, 3),  # Ring
                    'e': (3, 0), 'r': (3, 1), 't': (3, 2), 'y': (3, 3)   # Thumb
                }
                
                if finger_joint in finger_map:
                    finger, joint = finger_map[finger_joint]
                    hand.move_finger(finger, joint, direction)
                else:
                    print("❌ Invalid finger/joint key")
            else:
                print("❌ Invalid command. Type 'h' for help or 'q' to quit")
                
    except KeyboardInterrupt:
        print("\n👋 Stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()


