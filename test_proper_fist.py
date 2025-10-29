#!/usr/bin/env python3
"""
Test Proper Fist Angles on LEAP Hand
This script tests the LEAP hand with the correct angles for a proper fist.
"""

import numpy as np
import time
import sys
sys.path.append('/home/rumi/Desktop/leap/LEAP_Hand_API/python')
from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu

class ProperFistTester:
    def __init__(self, port='/dev/ttyUSB1'):
        """Initialize LEAP hand controller for testing proper fist angles"""
        
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.motors = list(range(16))
        
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
        
        print("🤖 Proper Fist Tester Ready!")
    
    def create_proper_fist_angles(self):
        """Create the correct angles for a proper fist"""
        
        # Proper fist angles (degrees) based on human anatomy
        # Format: [MCP_Abd, MCP_Flex, PIP, DIP] for each finger
        proper_fist_degrees = np.array([
            # Index finger
            0,      # MCP_Abd: 0° (no abduction)
            85,     # MCP_Flex: 85° (strong flexion)
            105,    # PIP: 105° (strong flexion)
            75,     # DIP: 75° (strong flexion)
            
            # Middle finger
            0,      # MCP_Abd: 0° (no abduction)
            90,     # MCP_Flex: 90° (strong flexion)
            105,    # PIP: 105° (strong flexion)
            75,     # DIP: 75° (strong flexion)
            
            # Ring finger
            0,      # MCP_Abd: 0° (no abduction)
            90,     # MCP_Flex: 90° (strong flexion)
            105,    # PIP: 105° (strong flexion)
            75,     # DIP: 75° (strong flexion)
            
            # Thumb
            45,     # MCP_Abd: 45° (toward palm)
            55,     # MCP_Flex: 55° (flexion)
            0,      # PIP: 0° (thumb doesn't have PIP)
            0       # DIP: 0° (thumb doesn't have DIP)
        ])
        
        return proper_fist_degrees
    
    def test_proper_fist(self):
        """Test the LEAP hand with proper fist angles"""
        
        print("\n🎯 Testing Proper Fist Angles")
        print("="*50)
        
        # Get proper fist angles
        fist_degrees = self.create_proper_fist_angles()
        
        # Convert to radians
        fist_radians = np.deg2rad(fist_degrees)
        
        # Convert to LEAP hand format (add π offset)
        leap_angles = fist_radians + np.pi
        
        # Apply safety clipping
        leap_angles = lhu.angle_safety_clip(leap_angles)
        
        print("Proper Fist Angles (degrees):")
        finger_names = ['Index', 'Middle', 'Ring', 'Thumb']
        joint_names = ['MCP_Abd', 'MCP_Flex', 'PIP', 'DIP']
        
        for i, finger in enumerate(finger_names):
            print(f"  {finger}: ", end="")
            for j, joint in enumerate(joint_names):
                joint_idx = i * 4 + j
                print(f"{joint}={fist_degrees[joint_idx]:.1f}° ", end="")
            print()
        
        print(f"\nConverted to LEAP angles (radians):")
        for i, finger in enumerate(finger_names):
            print(f"  {finger}: ", end="")
            for j, joint in enumerate(joint_names):
                joint_idx = i * 4 + j
                print(f"{joint}={leap_angles[joint_idx]:.3f} ", end="")
            print()
        
        # Move hand to proper fist position
        try:
            self.dxl_client.write_desired_pos(self.motors, leap_angles)
            print(f"\n✅ Successfully moved to proper fist position")
            return True
        except Exception as e:
            print(f"❌ Error moving to proper fist: {e}")
            return False
    
    def open_hand(self):
        """Open hand to neutral position"""
        open_positions = np.zeros(16)
        leap_positions = lhu.allegro_to_LEAPhand(open_positions)
        self.dxl_client.write_desired_pos(self.motors, leap_positions)
        print("🤖 Hand opened to neutral position")

def main():
    """Main testing function"""
    
    print("🤖 Proper Fist Tester for LEAP Hand")
    print("="*60)
    
    try:
        # Initialize LEAP hand
        print("\n🤖 Initializing LEAP hand...")
        tester = ProperFistTester()
        
        # Test proper fist
        print("\n🔄 Testing proper fist angles...")
        success = tester.test_proper_fist()
        
        if success:
            print(f"\n⏱️  Holding proper fist for 5 seconds...")
            time.sleep(5)
            
            # Return to open position
            print("\n🔄 Returning to open position...")
            tester.open_hand()
            time.sleep(2)
            
            print("\n✅ Proper fist test completed!")
            print("📊 Check if the LEAP hand now makes a proper fist")
            print("   - All fingers should be strongly flexed")
            print("   - No unwanted tilting (MCP abduction)")
            print("   - Fingers should be close to palm")
        else:
            print(f"⚠️  Failed to create proper fist")
        
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()
