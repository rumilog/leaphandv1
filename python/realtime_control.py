#!/usr/bin/env python3
"""
Real-time LEAP Hand Control
Single keystroke control without pressing Enter
"""
import numpy as np
import time
import sys
import tty
import termios
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu

class RealtimeLeapHand:
    def __init__(self, port='/dev/ttyUSB0'):
        """
        Initialize real-time LEAP Hand controller
        
        Args:
            port (str): Serial port for the hand
        """
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350  # Set to 550 for full hand
        self.motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        
        # Connect to hand
        self.dxl_client = DynamixelClient(self.motors, port, 4000000)
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
        
        # Step size for joint movement
        self.step_size = 0.1
        
        print("🎮 Real-time LEAP Hand Control Ready!")
        print_controls()
    
    def update_hand(self):
        """Update hand with current pose"""
        # For MCP Side joints, we need to handle them specially
        # because allegro_to_LEAPhand forces them to 3.14
        leap_pose = lhu.allegro_to_LEAPhand(self.current_pose, zeros=False)
        
        # Manually set MCP Side joints to allow movement
        leap_pose[0] = 3.14159 + self.current_pose[0]   # Index MCP Side
        leap_pose[4] = 3.14159 + self.current_pose[4]   # Middle MCP Side  
        leap_pose[8] = 3.14159 + self.current_pose[8]   # Ring MCP Side
        # Thumb MCP Side (12) is handled normally by the conversion
        
        self.dxl_client.write_desired_pos(self.motors, leap_pose)
    
    def move_joint(self, joint_idx, direction):
        """Move a specific joint"""
        if 0 <= joint_idx < 16:
            # Different behavior for MCP Side joints (0, 4, 8, 12)
            joint_in_finger = joint_idx % 4
            if joint_in_finger == 0:  # MCP Side joints
                # MCP Side joints have special behavior - they move around 0 in Allegro
                # but get converted to 3.14 in LEAP hand convention
                step = self.step_size * 0.5 * direction  # Smaller steps for MCP Side
                min_val, max_val = -0.5, 0.5  # Small range around 0 for MCP Side
            else:  # Other joints
                step = self.step_size * direction
                min_val, max_val = -2.0, 2.0  # Full range for other joints
            
            # Update position
            self.current_pose[joint_idx] = np.clip(
                self.current_pose[joint_idx] + step, min_val, max_val
            )
            self.update_hand()
            
            # Get finger and joint names
            finger_idx = joint_idx // 4
            finger_names = ["Index", "Middle", "Ring", "Thumb"]
            joint_names = ["MCP Side", "MCP Forward", "PIP", "DIP"]
            
            print(f"\r{finger_names[finger_idx]} {joint_names[joint_in_finger]}: {self.current_pose[joint_idx]:.2f} rad", end="", flush=True)
    
    def reset_hand(self):
        """Reset to open position"""
        self.current_pose = np.zeros(16)
        self.update_hand()
        print("\r🔄 Reset to open position", end="", flush=True)
    
    def show_status(self):
        """Show current joint positions"""
        positions = self.dxl_client.read_pos()
        print("\n\n📊 Current Joint Positions:")
        finger_names = ["Index", "Middle", "Ring", "Thumb"]
        joint_names = ["MCP Side", "MCP Forward", "PIP", "DIP"]
        
        for i in range(4):
            print(f"\n{finger_names[i]} Finger:")
            for j in range(4):
                joint_idx = i * 4 + j
                print(f"  {joint_names[j]}: {positions[joint_idx]:.3f} rad")
        print()
    
    def debug_joint(self, joint_idx):
        """Debug a specific joint movement"""
        if 0 <= joint_idx < 16:
            finger_idx = joint_idx // 4
            joint_in_finger = joint_idx % 4
            finger_names = ["Index", "Middle", "Ring", "Thumb"]
            joint_names = ["MCP Side", "MCP Forward", "PIP", "DIP"]
            
            print(f"\n🔍 Debugging {finger_names[finger_idx]} {joint_names[joint_in_finger]} (joint {joint_idx}):")
            print(f"  Current Allegro pose: {self.current_pose[joint_idx]:.3f} rad")
            
            # Convert to LEAP and show
            leap_pose = lhu.allegro_to_LEAPhand(self.current_pose)
            print(f"  Current LEAP pose: {leap_pose[joint_idx]:.3f} rad")
            
            # Read actual position
            actual_pos = self.dxl_client.read_pos()
            print(f"  Actual motor position: {actual_pos[joint_idx]:.3f} rad")
            print(f"  Difference: {abs(leap_pose[joint_idx] - actual_pos[joint_idx]):.3f} rad")

def get_char():
    """Get a single character without pressing Enter"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def print_controls():
    """Print control instructions"""
    print("\n🎯 Real-time Controls (single keystroke):")
    print("  Index Finger:    1,2,3,4 (open) | !,@,#,$ (close)")
    print("  Middle Finger:   5,6,7,8 (open) | %,^,&,* (close)")
    print("  Ring Finger:     9,0,a,z (open) | (,),A,Z (close)") 
    print("  Thumb:           d,f,g,h (open) | D,F,G,H (close)")
    print("  Special:         r=reset, s=status, x=debug, q=quit")
    print("\nExample: Press '1' to open Index MCP Side, press '!' to close it")
    print("         Press '5' to open Middle MCP Side, press '%' to close it")
    print("\nPress any key to start...")
    get_char()

def main():
    """Main control loop"""
    try:
        # Initialize hand
        hand = RealtimeLeapHand()
        
        # Control mapping: key -> (joint_index, direction)
        # Separate keys for open and close
        open_keys = {
            '1': 0, '2': 1, '3': 2, '4': 3,      # Index open
            '5': 4, '6': 5, '7': 6, '8': 7,      # Middle open
            '9': 8, '0': 9, 'a': 10, 'z': 11,    # Ring open
            'd': 12, 'f': 13, 'g': 14, 'h': 15   # Thumb open
        }
        
        close_keys = {
            '!': 0, '@': 1, '#': 2, '$': 3,      # Index close
            '%': 4, '^': 5, '&': 6, '*': 7,      # Middle close
            '(': 8, ')': 9, 'A': 10, 'Z': 11,    # Ring close
            'D': 12, 'F': 13, 'G': 14, 'H': 15   # Thumb close
        }
        
        print("\n🎮 Real-time control active! Press keys to control joints...")
        
        while True:
            # Get single character
            key = get_char()
            
            if key == 'q':
                print("\n👋 Goodbye!")
                break
            elif key == 'r':
                hand.reset_hand()
            elif key == 's':
                hand.show_status()
            elif key == 'x':
                # Debug MCP Side joints
                print("\n🔍 Debugging MCP Side joints (1, 5, 9, d):")
                hand.debug_joint(0)   # Index MCP Side
                hand.debug_joint(4)   # Middle MCP Side  
                hand.debug_joint(8)   # Ring MCP Side
                hand.debug_joint(12)  # Thumb MCP Side
            elif key in open_keys:
                joint_idx = open_keys[key]
                hand.move_joint(joint_idx, 1)  # Open
            elif key in close_keys:
                joint_idx = close_keys[key]
                hand.move_joint(joint_idx, -1)  # Close
            else:
                # Invalid key - just continue
                continue
                
    except KeyboardInterrupt:
        print("\n👋 Stopped by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")

if __name__ == "__main__":
    main()
