#!/usr/bin/env python3
"""
LEAP Hand Pose Examples
Try different hand poses and movements
"""
import numpy as np
import time
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu

class LeapHandController:
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
        
        print("🤖 LEAP Hand Controller Ready!")
    
    def set_pose(self, pose_name, pose):
        """Set hand to a specific pose"""
        print(f"🎯 Setting pose: {pose_name}")
        self.dxl_client.write_desired_pos(self.motors, pose)
        time.sleep(2)  # Wait for movement
    
    def demo_poses(self):
        """Demonstrate different hand poses"""
        
        # 1. Open Hand (all fingers extended)
        open_pose = lhu.allegro_to_LEAPhand(np.zeros(16))
        self.set_pose("Open Hand", open_pose)
        
        # 2. Fist (all fingers closed)
        fist_pose = lhu.allegro_to_LEAPhand(np.array([
            0, 1.5, 1.5, 1.5,  # Index
            0, 1.5, 1.5, 1.5,  # Middle  
            0, 1.5, 1.5, 1.5,  # Ring
            0, 1.5, 1.5, 1.5   # Thumb
        ]))
        self.set_pose("Fist", fist_pose)
        
        # 3. Peace Sign (Index and Middle extended)
        peace_pose = lhu.allegro_to_LEAPhand(np.array([
            0, 0, 0, 0,         # Index (extended)
            0, 0, 0, 0,         # Middle (extended)
            0, 1.5, 1.5, 1.5,  # Ring (closed)
            0, 1.5, 1.5, 1.5   # Thumb (closed)
        ]))
        self.set_pose("Peace Sign", peace_pose)
        
        # 4. Thumbs Up
        thumbs_up = lhu.allegro_to_LEAPhand(np.array([
            0, 1.5, 1.5, 1.5,  # Index (closed)
            0, 1.5, 1.5, 1.5,  # Middle (closed)
            0, 1.5, 1.5, 1.5,  # Ring (closed)
            0, 0, 0, 0         # Thumb (extended)
        ]))
        self.set_pose("Thumbs Up", thumbs_up)
        
        # 5. Pinch (Index and Thumb)
        pinch_pose = lhu.allegro_to_LEAPhand(np.array([
            0, 0.5, 0.5, 0.5,  # Index (slightly bent)
            0, 1.5, 1.5, 1.5,  # Middle (closed)
            0, 1.5, 1.5, 1.5,  # Ring (closed)
            0, 0.5, 0.5, 0.5   # Thumb (slightly bent)
        ]))
        self.set_pose("Pinch", pinch_pose)
        
        # 6. Back to Open
        self.set_pose("Open Hand", open_pose)
        
        print("🎉 Demo complete!")

def main():
    try:
        controller = LeapHandController()
        controller.demo_poses()
        
        # Keep reading positions for a bit
        print("\n📊 Reading current positions...")
        for i in range(10):
            positions = controller.dxl_client.read_pos()
            print(f"Position {i+1}: {positions}")
            time.sleep(0.5)
            
    except KeyboardInterrupt:
        print("\n👋 Demo stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()


