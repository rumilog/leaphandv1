#!/usr/bin/env python3
"""
Real-Time Quest 2 to LEAP Hand Teleoperation (Fixed Signal Handling)

This script receives live hand tracking data from Quest 2 via UDP
and sends it directly to the LEAP hand for real-time teleoperation.
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

class RealTimeTeleopFixed:
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
        signal.signal(signal.SIGTERM, self.signal_handler)
    
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
            self.leap_hand.sync_write([0,4,8], np.ones(3) * 450, 84, 2)  # Lower P-gain for side joints
            self.leap_hand.sync_write(motors, np.ones(16) * 0, 82, 2)    # I-gain
            self.leap_hand.sync_write(motors, np.ones(16) * 200, 80, 2)  # D-gain
            self.leap_hand.sync_write([0,4,8], np.ones(3) * 150, 80, 2)  # Lower D-gain for side joints
            
            # Set current limit
            self.leap_hand.sync_write(motors, np.ones(16) * 400, 102, 2)
            
            # Set to neutral "stop sign" pose
            self.set_neutral_pose()
            
            print("✓ LEAP hand configured for teleoperation")
            
        except Exception as e:
            print(f"✗ Error configuring LEAP hand: {e}")
    
    def set_neutral_pose(self):
        """Set LEAP hand to neutral 'stop sign' pose"""
        if not self.leap_hand:
            return
            
        try:
            print("🖐️ Setting neutral 'stop sign' pose...")
            
            # Neutral pose: fingers straight, thumb out
            neutral_pose = np.array([
                # Index finger (joints 0-3): MCP_Abd, MCP_Flex, PIP, DIP
                3.14, 3.14, 3.14, 3.14,  # Index: straight, no abduction
                # Middle finger (joints 4-7): MCP_Abd, MCP_Flex, PIP, DIP  
                3.14, 3.14, 3.14, 3.14,  # Middle: straight, no abduction
                # Ring finger (joints 8-11): MCP_Abd, MCP_Flex, PIP, DIP
                3.14, 3.14, 3.14, 3.14,  # Ring: straight, no abduction
                # Thumb (joints 12-15): MCP_Abd, MCP_Flex, PIP, DIP
                3.14, 3.14, 3.14, 3.14   # Thumb: straight, no abduction
            ])
            
            # Send neutral pose
            self.leap_hand.write_desired_pos(list(range(16)), neutral_pose)
            print("✓ Neutral pose set - fingers should be straight")
            
            # Wait for movement to complete
            time.sleep(2)
            
        except Exception as e:
            print(f"✗ Error setting neutral pose: {e}")
    
    def init_udp(self):
        """Initialize UDP communication with Quest 2"""
        try:
            print("📡 Setting up UDP communication...")
            # Use the same IP addresses as server_env.py
            self.sock = U.UdpComms(udpIP="172.26.71.187", sendIP="172.26.27.252", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)
            print("✓ UDP communication ready")
        except Exception as e:
            print(f"✗ UDP setup failed: {e}")
            self.sock = None
    
    def convert_quest_to_leap(self, quest_angles):
        """Convert Quest 2 angles (degrees) to LEAP hand format (radians)"""
        leap_angles = np.zeros(16)
        
        for joint_idx, angle_deg in quest_angles.items():
            if joint_idx < 16:
                # Convert degrees to radians
                angle_rad = np.radians(angle_deg)
                
                # Apply LEAP hand conversion (add π offset)
                leap_angles[joint_idx] = angle_rad + np.pi
                
                # Special handling for MCP abduction (side joints)
                if joint_idx in [0, 4, 8]:  # MCP_Side joints
                    leap_angles[joint_idx] = np.pi + angle_rad
        
        return leap_angles
    
    def send_pose_to_leap(self, leap_pose, pose_name="Teleop"):
        """Send pose to LEAP hand"""
        if not self.leap_hand:
            print(f"📋 {pose_name} (Simulation): {leap_pose[:4]}...")
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
    
    def process_hand_data(self, data):
        """Process incoming hand tracking data"""
        try:
            # Parse the data - Quest 2 sends full message format
            if isinstance(data, str):
                # Split the data and extract the last 16 numbers (joint angles)
                parts = data.split()
                if len(parts) >= 16:
                    # Get the last 16 parts (joint angles)
                    angle_strings = parts[-16:]
                    try:
                        # Convert to float array
                        angles = [float(angle) for angle in angle_strings]
                        
                        # Convert to dictionary format
                        quest_pose = {}
                        for i, angle in enumerate(angles):
                            quest_pose[i] = angle
                        
                        # Convert to LEAP hand format
                        leap_pose = self.convert_quest_to_leap(quest_pose)
                        
                        # Send to LEAP hand
                        success = self.send_pose_to_leap(leap_pose, f"Frame {self.pose_count}")
                        
                        if success:
                            self.pose_count += 1
                            if self.pose_count % 30 == 0:  # Print status every 30 frames
                                print(f"📊 Processed {self.pose_count} frames")
                        
                        return True
                    except ValueError:
                        print(f"⚠ Could not parse angles from: {angle_strings}")
                        return False
                else:
                    print(f"⚠ Not enough data parts: {len(parts)} (expected at least 16)")
                    return False
            else:
                print(f"⚠ Invalid data type: {type(data)}")
                return False
                
        except Exception as e:
            print(f"✗ Error processing hand data: {e}")
            return False
    
    def run_teleoperation(self):
        """Main teleoperation loop with improved signal handling"""
        print("🚀 Starting real-time teleoperation...")
        print("📱 Make sure your Quest 2 is running and sending data!")
        print("🛑 Press Ctrl+C to stop")
        print("="*60)
        
        self.running = True
        start_time = time.time()
        
        try:
            while self.running:
                # Check if we should stop
                if not self.running:
                    break
                
                # Receive data from Quest 2 with timeout
                try:
                    data = self.sock.ReadReceivedData()
                    
                    if data is not None:
                        # Process the hand tracking data
                        self.process_hand_data(data)
                except Exception as e:
                    if self.running:  # Only print error if we're still running
                        print(f"⚠ UDP error: {e}")
                
                # Small delay to prevent overwhelming the system
                time.sleep(0.01)  # 100Hz update rate
                
        except KeyboardInterrupt:
            print(f"\n🛑 Teleoperation stopped by user (KeyboardInterrupt)")
        except Exception as e:
            print(f"\n✗ Error during teleoperation: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\n🔄 Cleaning up...")
        
        if self.leap_hand:
            try:
                # Return to neutral "stop sign" position
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
        # Force exit if needed
        if sig == signal.SIGINT:
            print("🛑 Forcing exit...")
            sys.exit(0)

def main():
    print("=" * 80)
    print("REAL-TIME QUEST 2 TO LEAP HAND TELEOPERATION (FIXED)")
    print("=" * 80)
    
    # Create teleoperation instance
    teleop = RealTimeTeleopFixed()
    
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
