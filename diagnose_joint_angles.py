#!/usr/bin/env python3
"""
Diagnostic script to analyze joint angle extraction issues in real-time.
This will help identify why MCP abduction is incorrect and PIP/DIP flexion is insufficient.
"""

import socket
import numpy as np
import time
import sys
import signal
from collections import deque

# Add the LEAP Hand API to the path
sys.path.append('/home/rumi/Desktop/leap/LEAP_Hand_API/python')

class JointAngleDiagnostic:
    def __init__(self):
        self.udp_socket = None
        self.running = False
        self.angle_history = deque(maxlen=100)  # Keep last 100 readings
        self.setup_signal_handler()
        
    def setup_signal_handler(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        print("\n🛑 Stopping diagnostic...")
        self.running = False
        if self.udp_socket:
            self.udp_socket.close()
        sys.exit(0)
        
    def init_udp(self):
        """Initialize UDP connection to Quest 2"""
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.bind(('0.0.0.0', 8001))
            self.udp_socket.settimeout(1.0)  # 1 second timeout
            print("✓ UDP socket initialized on port 8001")
            return True
        except Exception as e:
            print(f"❌ Failed to initialize UDP: {e}")
            return False
            
    def analyze_joint_angles(self, quest_angles):
        """Analyze the Quest 2 joint angles for issues"""
        if len(quest_angles) != 16:
            return
            
        # Extract finger data
        fingers = {
            'Thumb': quest_angles[0:4],
            'Index': quest_angles[4:8], 
            'Middle': quest_angles[8:12],
            'Ring': quest_angles[12:16]
        }
        
        # Analyze each finger
        for finger_name, angles in fingers.items():
            mcp_abd, mcp_flex, pip_flex, dip_flex = angles
            
            # Check for MCP abduction issues (should be close to 0 for a fist)
            if abs(mcp_abd) > 5:  # More than 5 degrees abduction
                print(f"⚠️  {finger_name} MCP Abduction: {mcp_abd:.1f}° (should be ~0° for fist)")
            
            # Check for insufficient PIP/DIP flexion (should be high for a fist)
            if pip_flex < 60:  # Less than 60 degrees PIP flexion
                print(f"⚠️  {finger_name} PIP Flexion: {pip_flex:.1f}° (should be >60° for fist)")
            if dip_flex < 60:  # Less than 60 degrees DIP flexion
                print(f"⚠️  {finger_name} DIP Flexion: {dip_flex:.1f}° (should be >60° for fist)")
                
    def run_diagnostic(self):
        """Run the diagnostic for 30 seconds"""
        if not self.init_udp():
            return
            
        print("🔍 Starting joint angle diagnostic...")
        print("📋 Instructions:")
        print("   1. Make a FIST with your hand")
        print("   2. Hold it steady for 10 seconds")
        print("   3. Then make an OPEN hand")
        print("   4. Hold it steady for 10 seconds")
        print("   5. Repeat fist/open a few times")
        print("   6. Press Ctrl+C to stop")
        print("\n" + "="*60)
        
        self.running = True
        start_time = time.time()
        last_analysis_time = 0
        
        while self.running and (time.time() - start_time) < 30:
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                message = data.decode('utf-8').strip()
                
                if message.startswith("HAND_DATA:"):
                    # Parse the hand data
                    data_part = message[10:]  # Remove "HAND_DATA:" prefix
                    try:
                        quest_angles = [float(x) for x in data_part.split(',')]
                        
                        # Store in history
                        self.angle_history.append(quest_angles)
                        
                        # Analyze every 2 seconds
                        current_time = time.time()
                        if current_time - last_analysis_time >= 2.0:
                            print(f"\n📊 Analysis at {current_time - start_time:.1f}s:")
                            self.analyze_joint_angles(quest_angles)
                            last_analysis_time = current_time
                            
                    except ValueError as e:
                        print(f"❌ Error parsing hand data: {e}")
                        
            except socket.timeout:
                continue
            except Exception as e:
                print(f"❌ Error receiving data: {e}")
                break
                
        print("\n📈 Diagnostic Summary:")
        if self.angle_history:
            print(f"   • Received {len(self.angle_history)} angle readings")
            print(f"   • Time range: {time.time() - start_time:.1f} seconds")
            
            # Calculate statistics
            all_angles = np.array(list(self.angle_history))
            print(f"   • Average MCP Abduction: {np.mean(all_angles[:, [0,4,8,12]]):.1f}°")
            print(f"   • Average PIP Flexion: {np.mean(all_angles[:, [2,6,10,14]]):.1f}°")
            print(f"   • Average DIP Flexion: {np.mean(all_angles[:, [3,7,11,15]]):.1f}°")
        else:
            print("   • No data received - check Quest 2 connection")
            
        print("\n✅ Diagnostic completed!")

def main():
    print("🔍 Joint Angle Diagnostic Tool")
    print("="*50)
    
    diagnostic = JointAngleDiagnostic()
    diagnostic.run_diagnostic()

if __name__ == "__main__":
    main()
