#!/usr/bin/env python3
"""
Quest 2 to LEAP Hand Teleoperation - PIP & DIP Joints Only
Receives hand tracking data from Quest 2 and controls LEAP hand
"""
import numpy as np
import time
import sys
import signal
from leap_hand_utils.dynamixel_client import *
import leap_hand_utils.leap_hand_utils as lhu
import UdpComms as U

# Configuration
THIS_IP = "172.26.71.187"  # Your PC's IP
OCULUS_IP = "172.26.109.148"  # Quest 2's IP
LEAP_PORT = '/dev/ttyUSB0'  # LEAP hand serial port
UPDATE_RATE = 30  # Hz - how often to update the hand

class LeapHandTeleop:
    def __init__(self, port=LEAP_PORT):
        """
        Initialize LEAP Hand controller for teleoperation
        
        Args:
            port (str): Serial port for the hand
        """
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        
        # Connect to hand
        print("🔌 Connecting to LEAP hand...")
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
        
        # Initialize hand to open position
        self.current_positions = np.zeros(16)
        leap_positions = lhu.allegro_to_LEAPhand(self.current_positions)
        self.dxl_client.write_desired_pos(self.motors, leap_positions)
        
        print("✅ LEAP Hand connected and initialized")
        print("📋 Joint order: Index[0-3], Middle[4-7], Ring[8-11], Thumb[12-15]")
        print("📋 Each finger: [MCP Side, MCP Forward, PIP, DIP]")
        
    def update_pip_dip_joints(self, quest_angles):
        """
        Update only PIP and DIP joints based on Quest 2 tracking data
        
        Args:
            quest_angles: 10 values from Quest 2 [Thumb PIP, Thumb DIP, Index PIP, Index DIP, 
                                                     Middle PIP, Middle DIP, Ring PIP, Ring DIP, 
                                                     Pinky PIP, Pinky DIP] in radians
        
        Returns:
            bool: True if successful
        """
        try:
            # quest_angles are in radians from Quest 2 (PIP and DIP for 5 fingers)
            # Quest order: [Thumb, Index, Middle, Ring, Pinky] with [PIP, DIP] for each
            # Quest indices: [0,1, 2,3, 4,5, 6,7, 8,9]
            
            # Get current positions (in Allegro convention)
            current_leap = self.dxl_client.read_pos()
            current_allegro = lhu.LEAPhand_to_allegro(current_leap, zeros=False)
            
            # LEAP hand finger mapping (Allegro convention):
            # Index: joints 0-3 [MCP Side, MCP Forward, PIP, DIP]
            # Middle: joints 4-7 [MCP Side, MCP Forward, PIP, DIP]
            # Ring: joints 8-11 [MCP Side, MCP Forward, PIP, DIP]
            # Thumb: joints 12-15 [MCP Side, MCP Forward, PIP, DIP]
            
            # Map Quest angles to LEAP hand PIP/DIP joints
            # We keep MCP joints at their current position
            
            # Index finger - Quest indices [2=Index PIP, 3=Index DIP] -> LEAP joints [2=PIP, 3=DIP]
            current_allegro[2] = quest_angles[2]  # Index PIP
            current_allegro[3] = quest_angles[3]  # Index DIP
            
            # Middle finger - Quest indices [4=Middle PIP, 5=Middle DIP] -> LEAP joints [6=PIP, 7=DIP]
            current_allegro[6] = quest_angles[4]  # Middle PIP
            current_allegro[7] = quest_angles[5]  # Middle DIP
            
            # Ring finger - Quest indices [6=Ring PIP, 7=Ring DIP] -> LEAP joints [10=PIP, 11=DIP]
            current_allegro[10] = quest_angles[6]  # Ring PIP
            current_allegro[11] = quest_angles[7]  # Ring DIP
            
            # Thumb - Quest indices [0=Thumb PIP, 1=Thumb DIP] -> LEAP joints [14=PIP, 15=DIP]
            current_allegro[14] = quest_angles[0]  # Thumb PIP
            current_allegro[15] = quest_angles[1]  # Thumb DIP
            
            # Convert to LEAP coordinates and move hand
            leap_positions = lhu.allegro_to_LEAPhand(current_allegro, zeros=False)
            leap_positions = lhu.angle_safety_clip(leap_positions)
            self.dxl_client.write_desired_pos(self.motors, leap_positions)
            
            return True
            
        except Exception as e:
            print(f"❌ Error updating joints: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def disconnect(self):
        """Disconnect from LEAP hand"""
        try:
            # Set hand to safe position before disconnecting
            print("\n🔄 Resetting hand to safe position...")
            open_positions = np.zeros(16)
            leap_positions = lhu.allegro_to_LEAPhand(open_positions)
            self.dxl_client.write_desired_pos(self.motors, leap_positions)
            time.sleep(0.5)
            
            # Disconnect
            print("🔌 Disconnecting from LEAP hand...")
            self.dxl_client.set_torque_enabled(self.motors, False)
            self.dxl_client.disconnect()
            print("✅ Disconnected successfully")
        except Exception as e:
            print(f"❌ Error during disconnect: {e}")

def main():
    """Main teleoperation loop"""
    print("="*60)
    print("QUEST 2 TO LEAP HAND TELEOPERATION")
    print("PIP & DIP Joints Control")
    print("="*60)
    
    # Create UDP socket to receive data
    print(f"\n📡 Setting up UDP connection...")
    print(f"   Listening on: {THIS_IP}:8001")
    print(f"   Receiving from: {OCULUS_IP}:8000")
    sock = U.UdpComms(udpIP=THIS_IP, sendIP=OCULUS_IP, portTX=8000, portRX=8001, 
                       enableRX=True, suppressWarnings=False)
    
    # Initialize LEAP hand
    hand = LeapHandTeleop()
    
    # Variables for tracking
    update_time = time.time()
    frame_count = 0
    
    def signal_handler(sig, frame):
        print("\n\n⚠️  Stopping teleoperation...")
        hand.disconnect()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    print("\n🎮 Starting teleoperation...")
    print("💡 Move your hand to control the LEAP hand")
    print("⚠️  Press Ctrl+C to stop")
    print("="*60)
    
    try:
        while True:
            # Check if it's time to update (control update rate)
            current_time = time.time()
            if current_time >= update_time:
                update_time = current_time + (1.0 / UPDATE_RATE)
                
                # Receive data from Quest 2
                data = sock.ReadReceivedData()
                
                if data is not None:
                    try:
                        # Parse the UDP data (same format as server_env.py)
                        parts = data.split('\n')
                        
                        if len(parts) >= 4:
                            inventory, unknown_objects, hand_pose, gripper_message = parts[0], parts[1], parts[2], parts[3]
                            
                            # Process gripper data (contains finger angles)
                            if gripper_message:
                                gripper_message = gripper_message[:-1]  # remove trailing tab
                                gripper_values = np.array(gripper_message.split('\t')).astype(np.float64)
                                
                                # Check if we have 10 values (PIP and DIP for 5 fingers)
                                if len(gripper_values) == 10:
                                    # Update LEAP hand with PIP and DIP angles
                                    success = hand.update_pip_dip_joints(gripper_values)
                                    
                                    if success:
                                        frame_count += 1
                                        if frame_count % 30 == 0:  # Print every 30 frames
                                            print(f"✓ Updated LEAP hand (frame {frame_count})")
                                    
                                else:
                                    print(f"⚠️  Expected 10 values (PIP/DIP for 5 fingers), got {len(gripper_values)}")
                    
                    except Exception as e:
                        print(f"❌ Error processing data: {e}")
                
                # Small sleep to avoid busy waiting
                time.sleep(0.001)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Teleoperation stopped by user")
        hand.disconnect()
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        hand.disconnect()

if __name__ == "__main__":
    main()
