#!/usr/bin/env python3
"""
Real-Time Quest 2 to LEAP Hand Teleoperation - PIP/DIP ONLY

This script receives live PIP/DIP hand tracking data from Quest 2 via UDP
and sends it directly to the LEAP hand for real-time teleoperation.
Only PIP and DIP joints are controlled; MCP joints remain in neutral position.
"""

import numpy as np
import time
import sys
import os
import signal
import threading
from datetime import datetime

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

import UdpComms as U

class RealTimeTeleopPipDip:
    def __init__(self):
        self.leap_hand = None
        self.sock = None
        self.running = False
        self.last_pose = None
        self.pose_count = 0
        
        # Initialize LEAP hand
        if LEAP_AVAILABLE:
            self.init_leap_hand()
        
        # Initialize UDP communication
        self.init_udp()
        
        # Set up signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def init_leap_hand(self):
        """Initialize LEAP hand connection"""
        try:
            print("🔌 Connecting to LEAP hand...")
            motors = list(range(16))
            
            # Try different ports
            ports_to_try = ['/dev/ttyUSB0', '/dev/ttyUSB1', 'COM13']
            
            for port in ports_to_try:
                try:
                    self.leap_hand = DynamixelClient(motors, port, 4000000)
                    self.leap_hand.connect()
                    print(f"✓ Connected to LEAP hand on {port}")
                    self.configure_leap_hand()
                    return
                except Exception as e:
                    print(f"✗ Failed to connect on {port}: {e}")
                    continue
            
            print("✗ Could not connect to LEAP hand on any port")
            self.leap_hand = None
            
        except Exception as e:
            print(f"✗ LEAP hand initialization failed: {e}")
            self.leap_hand = None
    
    def configure_leap_hand(self):
        """Configure LEAP hand for teleoperation"""
        if not self.leap_hand:
            return
            
        try:
            print("⚙️ Configuring LEAP hand for teleoperation...")
            motors = list(range(16))
            
            # Enable position-current control mode
            self.leap_hand.sync_write(motors, np.ones(16)*5, 11, 1)
            self.leap_hand.set_torque_enabled(motors, True)
            
            # Set PID parameters for responsive teleoperation
            self.leap_hand.sync_write(motors, np.ones(16) * 600, 84, 2)  # P-gain
            self.leap_hand.sync_write(motors, np.ones(16) * 0, 82, 2)     # I-gain
            self.leap_hand.sync_write(motors, np.ones(16) * 200, 80, 2)   # D-gain
            
            # Set current limit
            self.leap_hand.sync_write(motors, np.ones(16) * 400, 102, 2)
            
            # Set to neutral pose
            self.set_neutral_pose()
            
            print("✓ LEAP hand configured for teleoperation")
            
        except Exception as e:
            print(f"✗ Error configuring LEAP hand: {e}")
    
    def set_neutral_pose(self):
        """Set LEAP hand to neutral pose with fingers extended"""
        if not self.leap_hand:
            return
            
        try:
            print("🖐️ Setting neutral pose...")
            
            # Neutral pose: MCP joints straight (π), PIP/DIP will be set dynamically
            neutral_pose = np.array([
                # Index finger (joints 0-3): MCP_Abd, MCP_Flex, PIP, DIP
                3.14, 3.14, 3.14, 3.14,
                # Middle finger (joints 4-7): MCP_Abd, MCP_Flex, PIP, DIP
                3.14, 3.14, 3.14, 3.14,
                # Ring finger (joints 8-11): MCP_Abd, MCP_Flex, PIP, DIP
                3.14, 3.14, 3.14, 3.14,
                # Thumb (joints 12-15): MCP_Abd, MCP_Flex, PIP, DIP
                3.14, 3.14, 3.14, 3.14
            ])
            
            # Send neutral pose
            self.leap_hand.write_desired_pos(list(range(16)), neutral_pose)
            print("✓ Neutral pose set")
            
            # Wait for movement
            time.sleep(2)
            
        except Exception as e:
            print(f"✗ Error setting neutral pose: {e}")
    
    def init_udp(self):
        """Initialize UDP communication with Quest 2"""
        try:
            print("📡 Setting up UDP communication...")
            # Use the correct IP addresses
            self.sock = U.UdpComms(udpIP="172.26.71.187", sendIP="172.26.109.148", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)
            print("✓ UDP communication ready")
        except Exception as e:
            print(f"✗ UDP setup failed: {e}")
            self.sock = None
    
    def convert_pip_dip_to_leap(self, pip_dip_values):
        """
        Convert PIP/DIP values from Quest 2 to LEAP hand pose
        
        Expected format: 10 values [PIP_Thumb, DIP_Thumb, PIP_Index, DIP_Index, 
                                    PIP_Middle, DIP_Middle, PIP_Ring, DIP_Ring, 
                                    PIP_Pinky, DIP_Pinky]
        Each value is in radians from Quest 2
        """
        leap_pose = np.array([
            # Index finger (joints 0-3): MCP_Abd, MCP_Flex, PIP, DIP
            3.14, 3.14, 0.0, 0.0,
            # Middle finger (joints 4-7): MCP_Abd, MCP_Flex, PIP, DIP  
            3.14, 3.14, 0.0, 0.0,
            # Ring finger (joints 8-11): MCP_Abd, MCP_Flex, PIP, DIP
            3.14, 3.14, 0.0, 0.0,
            # Thumb (joints 12-15): MCP_Abd, MCP_Flex, PIP, DIP
            3.14, 3.14, 0.0, 0.0
        ])
        
        # LEAP hand joint mapping for PIP and DIP:
        # Index: joints 2, 3
        # Middle: joints 6, 7
        # Ring: joints 10, 11
        # Thumb: joints 14, 15
        
        if len(pip_dip_values) == 10:
            # Thumb PIP and DIP (positions 0, 1 in pip_dip_values)
            leap_pose[14] = 3.14 + pip_dip_values[0]  # Thumb PIP
            leap_pose[15] = 3.14 + pip_dip_values[1]  # Thumb DIP
            
            # Index PIP and DIP (positions 2, 3 in pip_dip_values)
            leap_pose[2] = 3.14 + pip_dip_values[2]   # Index PIP
            leap_pose[3] = 3.14 + pip_dip_values[3]   # Index DIP
            
            # Middle PIP and DIP (positions 4, 5 in pip_dip_values)
            leap_pose[6] = 3.14 + pip_dip_values[4]   # Middle PIP
            leap_pose[7] = 3.14 + pip_dip_values[5]   # Middle DIP
            
            # Ring PIP and DIP (positions 6, 7 in pip_dip_values)
            leap_pose[10] = 3.14 + pip_dip_values[6]  # Ring PIP
            leap_pose[11] = 3.14 + pip_dip_values[7]  # Ring DIP
            
            # Pinky PIP and DIP (positions 8, 9 in pip_dip_values)
            # Note: LEAP hand only has 16 joints (4 per finger x 4 fingers)
            # Pinky is not mapped in the original LEAP hand
            # We could map it to a spare joint or skip it
        
        return leap_pose
    
    def process_hand_data(self, data):
        """Process incoming PIP/DIP hand tracking data"""
        try:
            if isinstance(data, str):
                # The server_env.py sends data in a specific format
                # Look for the hand tracking data part
                parts = data.split()
                
                # Find PIP/DIP values (10 values total)
                pip_dip_values = None
                
                # Try to extract the 10 values
                if len(parts) >= 10:
                    try:
                        # Convert last 10 parts to floats
                        pip_dip_values = [float(parts[i]) for i in range(-10, 0)]
                    except (ValueError, IndexError):
                        pass
                
                if pip_dip_values is None:
                    return False
                
                # Convert to LEAP hand pose
                leap_pose = self.convert_pip_dip_to_leap(pip_dip_values)
                
                # Send to LEAP hand
                success = self.send_pose_to_leap(leap_pose, f"Frame {self.pose_count}")
                
                if success:
                    self.pose_count += 1
                    if self.pose_count % 30 == 0:  # Print status every 30 frames
                        print(f"📊 Processed {self.pose_count} frames - PIP: {pip_dip_values[2]:.2f}° DIP: {pip_dip_values[3]:.2f}° (Index)")
                
                return True
                
        except Exception as e:
            print(f"✗ Error processing hand data: {e}")
            return False
    
    def send_pose_to_leap(self, leap_pose, pose_name="Teleop"):
        """Send pose to LEAP hand"""
        if not self.leap_hand:
            print(f"📋 {pose_name} (Simulation): PIP={leap_pose[2]:.2f} DIP={leap_pose[3]:.2f}")
            return True
        
        try:
            # Apply safety clipping
            safe_pose = lhu.angle_safety_clip(leap_pose)
            
            # Send pose to LEAP hand
            self.leap_hand.write_desired_pos(list(range(16)), safe_pose)
            
            return True
            
        except Exception as e:
            print(f"✗ Error sending pose to LEAP hand: {e}")
            return False
    
    def run_teleoperation(self):
        """Main teleoperation loop"""
        print("🚀 Starting PIP/DIP teleoperation...")
        print("📱 Make sure your Quest 2 is running and sending PIP/DIP data!")
        print("🛑 Press Ctrl+C to stop")
        print("="*60)
        
        self.running = True
        
        try:
            while self.running:
                # Receive data from Quest 2
                data = self.sock.ReadReceivedData()
                
                if data is not None:
                    # Process the hand tracking data
                    self.process_hand_data(data)
                
                # Small delay
                time.sleep(0.01)  # 100Hz update rate
                
        except KeyboardInterrupt:
            print(f"\n🛑 Teleoperation stopped by user")
        except Exception as e:
            print(f"\n✗ Error during teleoperation: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\n🔄 Cleaning up...")
        
        if self.leap_hand:
            try:
                # Return to neutral position
                self.set_neutral_pose()
                print("✓ LEAP hand returned to neutral position")
            except Exception as e:
                print(f"⚠ Error returning to neutral: {e}")
        
        self.running = False
        print("✓ Teleoperation stopped")
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print(f"\n🛑 Interrupt received! Stopping teleoperation...")
        self.running = False

def main():
    print("=" * 80)
    print("REAL-TIME PIP/DIP TELEOPERATION")
    print("=" * 80)
    
    # Create teleoperation instance
    teleop = RealTimeTeleopPipDip()
    
    # Check if everything is ready
    if not teleop.sock:
        print("✗ UDP communication not available")
        return
    
    if not teleop.leap_hand and LEAP_AVAILABLE:
        print("⚠ LEAP hand not connected - running in simulation mode")
    
    # Start teleoperation
    teleop.run_teleoperation()

if __name__ == "__main__":
    main()
