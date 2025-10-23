#!/usr/bin/env python3
"""
Simple LEAP Hand Control
Move hand to specific joint positions using numpy arrays
"""
import numpy as np
import time
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu

class SimpleLeapHand:
    def __init__(self, port='/dev/ttyUSB0'):
        """
        Initialize LEAP Hand controller
        
        Args:
            port (str): Serial port for the hand (default: '/dev/ttyUSB0')
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
        
        print("🤖 Simple LEAP Hand Controller Ready!")
        print("📋 Joint order: Index[0-3], Middle[4-7], Ring[8-11], Thumb[12-15]")
        print("📋 Each finger: [MCP Side, MCP Forward, PIP, DIP]")
    
    def move_to_positions(self, joint_positions, coordinate_system='allegro'):
        """
        Move hand to specific joint positions
        
        Args:
            joint_positions (np.array): Array of 16 joint positions
            coordinate_system (str): 'allegro', 'leap', or 'simulation'
                - 'allegro': 0 = open, positive = close (default)
                - 'leap': 3.14 = open, positive = close  
                - 'simulation': -1 to 1 range
        
        Returns:
            bool: True if successful, False if error
        """
        try:
            joint_positions = np.array(joint_positions)
            
            # Validate input
            if len(joint_positions) != 16:
                print(f"❌ Error: Expected 16 joint positions, got {len(joint_positions)}")
                return False
            
            # Convert to LEAP hand coordinates
            if coordinate_system == 'allegro':
                leap_positions = lhu.allegro_to_LEAPhand(joint_positions)
            elif coordinate_system == 'leap':
                leap_positions = joint_positions
            elif coordinate_system == 'simulation':
                leap_positions = lhu.sim_ones_to_LEAPhand(joint_positions)
            else:
                print(f"❌ Error: Unknown coordinate system '{coordinate_system}'")
                print("Valid options: 'allegro', 'leap', 'simulation'")
                return False
            
            # Apply safety clipping
            leap_positions = lhu.angle_safety_clip(leap_positions)
            
            # Move hand
            self.dxl_client.write_desired_pos(self.motors, leap_positions)
            
            print(f"✅ Moved hand to positions using {coordinate_system} coordinates")
            return True
            
        except Exception as e:
            print(f"❌ Error moving hand: {e}")
            return False
    
    def get_current_positions(self, coordinate_system='allegro'):
        """
        Get current joint positions
        
        Args:
            coordinate_system (str): 'allegro', 'leap', or 'simulation'
        
        Returns:
            np.array: Current joint positions in specified coordinate system
        """
        try:
            leap_positions = self.dxl_client.read_pos()
            
            if coordinate_system == 'allegro':
                return lhu.LEAPhand_to_allegro(leap_positions)
            elif coordinate_system == 'leap':
                return leap_positions
            elif coordinate_system == 'simulation':
                return lhu.LEAPhand_to_sim_ones(leap_positions)
            else:
                print(f"❌ Error: Unknown coordinate system '{coordinate_system}'")
                return None
                
        except Exception as e:
            print(f"❌ Error reading positions: {e}")
            return None
    
    def open_hand(self):
        """Open hand (all fingers extended)"""
        open_positions = np.zeros(16)
        return self.move_to_positions(open_positions, 'allegro')
    
    def close_hand(self):
        """Close hand (all fingers bent)"""
        closed_positions = np.array([
            0, 1.5, 1.5, 1.5,  # Index
            0, 1.5, 1.5, 1.5,  # Middle
            0, 1.5, 1.5, 1.5,  # Ring
            0, 1.5, 1.5, 1.5   # Thumb
        ])
        return self.move_to_positions(closed_positions, 'allegro')

def main():
    """Example usage"""
    try:
        # Initialize hand
        hand = SimpleLeapHand()
        
        # Example 1: Open hand
        print("\n🎯 Example 1: Opening hand")
        hand.open_hand()
        time.sleep(2)
        
        # Example 2: Custom positions using Allegro coordinates
        print("\n🎯 Example 2: Custom positions (Allegro coordinates)")
        custom_positions = np.array([
            0, 0.5, 0.5, 0.5,  # Index - slightly bent
            0, 1.0, 1.0, 1.0,  # Middle - more bent
            0, 0.3, 0.3, 0.3,  # Ring - slightly bent
            0, 0.8, 0.8, 0.8   # Thumb - moderately bent
        ])
        hand.move_to_positions(custom_positions, 'allegro')
        time.sleep(2)
        
        # Example 3: Using simulation coordinates (-1 to 1)
        print("\n🎯 Example 3: Using simulation coordinates (-1 to 1)")
        sim_positions = np.array([
            -0.5, 0.5, 0.5, 0.5,   # Index
            -0.8, 0.8, 0.8, 0.8,   # Middle
            -0.3, 0.3, 0.3, 0.3,   # Ring
            -0.6, 0.6, 0.6, 0.6    # Thumb
        ])
        hand.move_to_positions(sim_positions, 'simulation')
        time.sleep(2)
        
        # Example 4: Get current positions
        print("\n🎯 Example 4: Reading current positions")
        current = hand.get_current_positions('allegro')
        print(f"Current positions: {current}")
        
        # Close hand
        print("\n🎯 Closing hand")
        hand.close_hand()
        
        print("\n✅ Demo complete!")
        
    except KeyboardInterrupt:
        print("\n👋 Demo stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
