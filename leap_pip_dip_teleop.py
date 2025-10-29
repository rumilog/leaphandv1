#!/usr/bin/env python3
"""
LEAP Hand PIP & DIP Teleoperation from Quest 2

This script listens to the same UDP stream as server_env.py and controls
only the PIP and DIP joints of the LEAP hand based on Quest 2 tracking data.
"""

import numpy as np
import time
import sys
import signal
from datetime import datetime

# Add the LEAP hand API to the path
import sys
sys.path.append('LEAP_Hand_API/python')

from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu
import UdpComms as U

# Configuration
THIS_IP = "172.26.71.187"  # Your PC's IP
OCULUS_IP = "172.26.109.148"  # Quest 2's IP
LEAP_PORT = '/dev/ttyUSB0'  # LEAP hand serial port

class LeapPipDipTeleop:
    def __init__(self, port=LEAP_PORT):
        """Initialize LEAP Hand controller for PIP/DIP teleoperation"""
        self.port = port
        self.dxl_client = None
        self.motors = list(range(16))
        self.running = False
        
        # LEAP hand configuration
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        
        # Initialize LEAP hand
        self.init_leap_hand()
        
        # Initialize UDP communication
        self.init_udp()
        
        # Set up signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def init_leap_hand(self):
        """Initialize LEAP hand connection"""
        try:
            print("🔌 Connecting to LEAP hand...")
            
            # Create client
            self.dxl_client = DynamixelClient(self.motors, self.port, 4000000)
            self.dxl_client.connect()
            
            # Configure LEAP hand
            print("⚙️ Configuring LEAP hand...")
            self.dxl_client.sync_write(self.motors, np.ones(len(self.motors))*5, 11, 1)
            self.dxl_client.set_torque_enabled(self.motors, True)
            self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
            self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2)
            self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
            self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
            self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2)
            self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)
            
            # Initialize to open position
            open_positions = np.zeros(16)
            leap_positions = lhu.allegro_to_LEAPhand(open_positions)
            self.dxl_client.write_desired_pos(self.motors, leap_positions)
            
            print("✅ LEAP Hand connected and configured")
            print("📋 Joint order: Index[0-3], Middle[4-7], Ring[8-11], Thumb[12-15]")
            print("📋 Each finger: [MCP Side, MCP Forward, PIP, DIP]")
            
        except Exception as e:
            print(f"❌ Failed to connect to LEAP hand: {e}")
            print(f"⚠️  Continuing in simulation mode...")
            self.dxl_client = None
    
    def init_udp(self):
        """Initialize UDP communication"""
        try:
            print("📡 Setting up UDP communication...")
            print(f"   Listening on: {THIS_IP}:8001")
            print(f"   Receiving from: {OCULUS_IP}:8000")
            
            # Create UDP socket (same settings as server_env.py)
            self.sock = U.UdpComms(udpIP=THIS_IP, sendIP=OCULUS_IP, portTX=8000, portRX=8001, 
                                   enableRX=True, suppressWarnings=False)
            
            print("✅ UDP communication ready")
            
        except Exception as e:
            print(f"❌ UDP setup failed: {e}")
            self.sock = None
    
    def update_pip_dip_joints(self, quest_angles_rad):
        """
        Update only PIP and DIP joints based on Quest 2 tracking data
        
        Args:
            quest_angles_rad: 10 values in radians [Thumb PIP, Thumb DIP, 
                                                   Index PIP, Index DIP,
                                                   Middle PIP, Middle DIP, 
                                                   Ring PIP, Ring DIP,
                                                   Pinky PIP, Pinky DIP]
        """
        if not self.dxl_client:
            return False
        
        try:
            # Get current LEAP hand positions (in Allegro convention)
            current_leap = self.dxl_client.read_pos()
            current_allegro = lhu.LEAPhand_to_allegro(current_leap, zeros=False)
            
            # LEAP hand joint indices:
            # Index: 0=MCP_Side, 1=MCP_Flex, 2=PIP, 3=DIP
            # Middle: 4=MCP_Side, 5=MCP_Flex, 6=PIP, 7=DIP
            # Ring: 8=MCP_Side, 9=MCP_Flex, 10=PIP, 11=DIP
            # Thumb: 12=MCP_Side, 13=MCP_Flex, 14=PIP, 15=DIP
            
            # Quest order: [Thumb PIP, Thumb DIP, Index PIP, Index DIP,
            #              Middle PIP, Middle DIP, Ring PIP, Ring DIP, Pinky PIP, Pinky DIP]
            #              [0         , 1         , 2        , 3       ,
            #               4         , 5         , 6        , 7       , 8         , 9]
            
            # Update only PIP and DIP joints, keep MCP joints unchanged
            
            # Thumb (Quest indices 0,1) -> LEAP joints 14,15
            current_allegro[14] = quest_angles_rad[0]  # Thumb PIP
            current_allegro[15] = quest_angles_rad[1]  # Thumb DIP
            
            # Index (Quest indices 2,3) -> LEAP joints 2,3
            current_allegro[2] = quest_angles_rad[2]  # Index PIP
            current_allegro[3] = quest_angles_rad[3]  # Index DIP
            
            # Middle (Quest indices 4,5) -> LEAP joints 6,7
            current_allegro[6] = quest_angles_rad[4]  # Middle PIP
            current_allegro[7] = quest_angles_rad[5]  # Middle DIP
            
            # Ring (Quest indices 6,7) -> LEAP joints 10,11
            current_allegro[10] = quest_angles_rad[6]  # Ring PIP
            current_allegro[11] = quest_angles_rad[7]  # Ring DIP
            
            # Note: Pinky is not mapped (LEAP hand has 4 fingers: Index, Middle, Ring, Thumb)
            
            # Convert to LEAP coordinates and apply safety limits
            leap_positions = lhu.allegro_to_LEAPhand(current_allegro, zeros=False)
            leap_positions = lhu.angle_safety_clip(leap_positions)
            
            # Send to LEAP hand
            self.dxl_client.write_desired_pos(self.motors, leap_positions)
            
            return True
            
        except Exception as e:
            print(f"❌ Error updating joints: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def process_udp_data(self, data):
        """Process incoming UDP data from Quest 2"""
        try:
            if data is None:
                return False
            
            # Parse data - same format as server_env.py
            parts = data.split('\n')
            
            if len(parts) >= 4:
                inventory, unknown_objects, hand_pose, gripper_message = parts[0], parts[1], parts[2], parts[3]
                
                # Remove trailing tab from gripper message
                if gripper_message:
                    gripper_message = gripper_message[:-1] if gripper_message.endswith('\t') else gripper_message
                    
                    # Parse finger angles
                    gripper_values = np.array(gripper_message.split('\t')).astype(np.float64)
                    
                    # Check if we have 10 values (PIP and DIP for 5 fingers)
                    if len(gripper_values) == 10:
                        # Update LEAP hand with PIP and DIP angles
                        success = self.update_pip_dip_joints(gripper_values)
                        return success
                    else:
                        if len(gripper_values) != 0:  # Don't print for empty messages
                            print(f"⚠️  Expected 10 values (PIP/DIP for 5 fingers), got {len(gripper_values)}")
                        return False
                else:
                    return False
            else:
                return False
                
        except Exception as e:
            print(f"❌ Error processing UDP data: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def run_teleoperation(self):
        """Main teleoperation loop"""
        print("\n" + "="*60)
        print("🚀 LEAP HAND PIP & DIP TELEOPERATION")
        print("="*60)
        print("💡 Move your hand to control the LEAP hand")
        print("🖐️  Only PIP and DIP joints will be controlled")
        print("⚠️  Press Ctrl+C to stop")
        print("="*60 + "\n")
        
        self.running = True
        frame_count = 0
        last_print_time = time.time()
        
        try:
            while self.running:
                # Receive data from Quest 2
                data = self.sock.ReadReceivedData()
                
                if data is not None:
                    success = self.process_udp_data(data)
                    
                    if success:
                        frame_count += 1
                        
                        # Print status every second
                        current_time = time.time()
                        if current_time - last_print_time >= 1.0:
                            fps = frame_count / (current_time - last_print_time)
                            print(f"✓ Teleoperating @ {fps:.1f} Hz (total frames: {frame_count})")
                            frame_count = 0
                            last_print_time = current_time
                
                # Small sleep to avoid busy waiting
                time.sleep(0.001)
                
        except KeyboardInterrupt:
            print("\n\n⚠️  Teleoperation stopped by user")
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\n🔄 Cleaning up...")
        
        if self.dxl_client:
            try:
                # Return to open position
                print("🖐️  Resetting LEAP hand to open position...")
                open_positions = np.zeros(16)
                leap_positions = lhu.allegro_to_LEAPhand(open_positions)
                self.dxl_client.write_desired_pos(self.motors, leap_positions)
                time.sleep(0.5)
                
                # Disconnect
                self.dxl_client.set_torque_enabled(self.motors, False)
                self.dxl_client.disconnect()
                
                print("✅ LEAP hand safely disconnected")
                
            except Exception as e:
                print(f"⚠️  Error during cleanup: {e}")
        
        self.running = False
        print("✓ Teleoperation stopped")
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\n\n🛑 Interrupt received! Stopping teleoperation...")
        self.running = False

def main():
    try:
        teleop = LeapPipDipTeleop()
        
        if not teleop.sock:
            print("❌ UDP communication not available")
            return
        
        if not teleop.dxl_client:
            print("⚠️  LEAP hand not connected - running in simulation mode")
        
        # Start teleoperation
        teleop.run_teleoperation()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()

