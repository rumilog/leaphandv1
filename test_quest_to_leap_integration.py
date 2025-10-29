#!/usr/bin/env python3
"""
Test Quest 2 to LEAP Hand Integration
Real-time hand tracking from Quest 2 controlling LEAP hand
"""
import sys
import os
import numpy as np
import time
import UdpComms as U

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

# Quest 2 connection settings (same as server_env.py)
THIS_IP = "172.26.71.187"
OCULUS_IP = "172.26.27.252"

class QuestToLeapController:
    def __init__(self):
        """Initialize Quest 2 to LEAP hand controller"""
        
        # Setup UDP communication with Quest 2
        self.sock = U.UdpComms(udpIP=THIS_IP, sendIP=OCULUS_IP, portTX=8000, portRX=8001, enableRX=True, suppressWarnings=False)
        print("✓ UDP communication setup with Quest 2")
        
        # Setup LEAP hand if available
        self.dxl_client = None
        if LEAP_AVAILABLE:
            try:
                motors = list(range(16))
                self.dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
                self.dxl_client.connect()
                print("✓ Connected to LEAP hand on /dev/ttyUSB1")
                
                # Configure LEAP hand
                self.dxl_client.sync_write(motors, np.ones(16)*5, 11, 1)
                self.dxl_client.set_torque_enabled(motors, True)
                self.dxl_client.sync_write(motors, np.ones(16) * 600, 84, 2)  # P-gain
                self.dxl_client.sync_write(motors, np.ones(16) * 0, 82, 2)    # I-gain  
                self.dxl_client.sync_write(motors, np.ones(16) * 200, 80, 2)  # D-gain
                self.dxl_client.sync_write(motors, np.ones(16) * 350, 102, 2) # Current limit
                print("✓ LEAP hand configured")
                
            except Exception as e:
                print(f"⚠ Failed to connect to LEAP hand: {e}")
                self.dxl_client = None
        
        self.motors = list(range(16))
        self.data_count = 0
        
    def convert_quest_to_leap(self, quest_finger_angles):
        """Convert Quest 2 finger angles to LEAP hand format"""
        
        # Convert degrees to radians
        quest_radians = np.radians(quest_finger_angles)
        
        # Convert to LEAP hand coordinates (add π offset)
        leap_angles = quest_radians + np.pi
        
        # Apply safety clipping
        if LEAP_AVAILABLE:
            leap_angles = lhu.angle_safety_clip(leap_angles)
        
        return leap_angles
    
    def send_to_leap_hand(self, leap_angles):
        """Send angles to LEAP hand"""
        
        if self.dxl_client is None:
            print(f"📋 Simulation: LEAP angles = {leap_angles}")
            return
        
        try:
            self.dxl_client.write_desired_pos(self.motors, leap_angles)
        except Exception as e:
            print(f"❌ Error sending to LEAP hand: {e}")
    
    def process_quest_data(self, data):
        """Process Quest 2 hand tracking data"""
        
        if data is None:
            return
        
        try:
            # Parse Quest 2 data (same format as server_env.py)
            inventory, unknown_objects, hand_pose, gripper_message = data.split('\n')
            gripper_message = gripper_message[:-1]  # remove unneeded tab
            
            # Parse hand position and rotation
            position_message, rotation_message = hand_pose.split('\t')
            position = np.array(position_message[1:-1].split(', ')).astype(np.float64)
            position = np.array([position[2], -position[0], position[1]])
            rotation = np.array(rotation_message[1:-1].split(', ')).astype(np.float64)
            rotation = np.array([-rotation[2], rotation[0], -rotation[1], rotation[3]])
            
            if gripper_message == "":
                return
            
            # Parse finger angles
            gripper_values = np.array(gripper_message.split('\t')).astype(np.float64)
            
            if len(gripper_values) == 16:
                # New format: 16 values (4 joints per finger)
                self.data_count += 1
                
                print(f"\n📊 Quest 2 Data #{self.data_count}")
                print(f"Position: {position}")
                print(f"Rotation: {rotation}")
                print(f"Finger angles (degrees): {gripper_values}")
                
                # Convert to LEAP hand format
                leap_angles = self.convert_quest_to_leap(gripper_values)
                print(f"LEAP angles (degrees): {np.degrees(leap_angles)}")
                
                # Send to LEAP hand
                self.send_to_leap_hand(leap_angles)
                
                # Show finger breakdown
                finger_names = ["Index", "Middle", "Ring", "Thumb"]
                joint_names = ["MCP_Abd", "MCP_Flex", "PIP", "DIP"]
                
                print("Finger joint breakdown:")
                for f in range(4):
                    finger_start = f * 4
                    print(f"  {finger_names[f]}: ", end="")
                    for j in range(4):
                        joint_idx = finger_start + j
                        quest_angle = gripper_values[joint_idx]
                        leap_angle = np.degrees(leap_angles[joint_idx])
                        print(f"{joint_names[j]}={quest_angle:.1f}°→{leap_angle:.1f}° ", end="")
                    print()
                
            else:
                print(f"⚠ Unexpected data format: {len(gripper_values)} values")
                
        except Exception as e:
            print(f"❌ Error processing Quest 2 data: {e}")
    
    def run(self, duration=30):
        """Run Quest 2 to LEAP hand control for specified duration"""
        
        print(f"\n🎯 Starting Quest 2 to LEAP hand control for {duration} seconds")
        print("🕐 Move your hand in front of the Quest 2 to see the LEAP hand respond!")
        print("🛑 Press Ctrl+C to stop early")
        
        start_time = time.time()
        
        try:
            while time.time() - start_time < duration:
                # Read data from Quest 2
                data = self.sock.ReadReceivedData()
                
                if data is not None:
                    self.process_quest_data(data)
                
                time.sleep(0.1)  # 10Hz update rate
                
        except KeyboardInterrupt:
            print("\n🛑 Stopped by user")
        
        print(f"\n✅ Control session complete! Processed {self.data_count} hand poses")

def main():
    print("=" * 80)
    print("QUEST 2 TO LEAP HAND INTEGRATION TEST")
    print("=" * 80)
    
    # Create controller
    controller = QuestToLeapController()
    
    if not LEAP_AVAILABLE:
        print("⚠ LEAP Hand API not available - running in simulation mode")
        print("📋 Quest 2 data will be processed and converted, but not sent to physical hand")
    
    # Run integration test
    controller.run(duration=30)  # Run for 30 seconds
    
    print("\n" + "=" * 80)
    print("INTEGRATION TEST COMPLETE")
    print("=" * 80)

if __name__ == "__main__":
    main()
