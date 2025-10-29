#!/usr/bin/env python3
"""
Investigate Accuracy Issues in Quest 2 to LEAP Hand Pipeline

This script tests both potential sources of inaccuracy:
1. Quaternion to joint angle conversion (Quest 2 side)
2. Joint angle to LEAP hand conversion (Python side)
"""

import numpy as np
import time
import sys
import os
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

class AccuracyInvestigator:
    def __init__(self):
        self.sock = None
        self.leap_hand = None
        self.data_log = []
        self.quest_angles_log = []
        self.leap_angles_log = []
        
        # Initialize connections
        self.init_udp()
        if LEAP_AVAILABLE:
            self.init_leap_hand()
    
    def init_udp(self):
        """Initialize UDP communication with Quest 2"""
        try:
            print("📡 Setting up UDP communication...")
            self.sock = U.UdpComms(udpIP="172.26.71.187", sendIP="172.26.27.252", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)
            print("✓ UDP communication ready")
        except Exception as e:
            print(f"✗ UDP setup failed: {e}")
            self.sock = None
    
    def init_leap_hand(self):
        """Initialize LEAP hand connection"""
        try:
            print("🔌 Connecting to LEAP hand...")
            motors = list(range(16))
            self.leap_hand = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
            self.leap_hand.connect()
            print("✓ Connected to LEAP hand")
            
            # Configure LEAP hand
            self.leap_hand.sync_write(motors, np.ones(16)*5, 11, 1)
            self.leap_hand.set_torque_enabled(motors, True)
            self.leap_hand.sync_write(motors, np.ones(16) * 600, 84, 2)
            self.leap_hand.sync_write(motors, np.ones(16) * 0, 82, 2)
            self.leap_hand.sync_write(motors, np.ones(16) * 200, 80, 2)
            self.leap_hand.sync_write(motors, np.ones(16) * 400, 102, 2)
            print("✓ LEAP hand configured")
            
        except Exception as e:
            print(f"✗ LEAP hand initialization failed: {e}")
            self.leap_hand = None
    
    def analyze_quest_angles(self, quest_angles):
        """Analyze Quest 2 joint angles for issues"""
        print("\n🔍 QUEST 2 ANGLE ANALYSIS:")
        print("=" * 50)
        
        # Check for unrealistic angles
        unrealistic_count = 0
        for i, angle in enumerate(quest_angles):
            if abs(angle) > 180:  # Unrealistic joint angle
                unrealistic_count += 1
                print(f"⚠ Joint {i}: {angle:.2f}° (unrealistic)")
        
        if unrealistic_count == 0:
            print("✓ All angles within realistic range")
        else:
            print(f"⚠ {unrealistic_count} unrealistic angles detected")
        
        # Check for sudden jumps
        if len(self.quest_angles_log) > 0:
            prev_angles = self.quest_angles_log[-1]
            jumps = []
            for i, (curr, prev) in enumerate(zip(quest_angles, prev_angles)):
                jump = abs(curr - prev)
                if jump > 30:  # Sudden jump > 30 degrees
                    jumps.append((i, jump, prev, curr))
            
            if jumps:
                print(f"⚠ {len(jumps)} sudden angle jumps detected:")
                for joint, jump, prev, curr in jumps:
                    print(f"  Joint {joint}: {prev:.1f}° → {curr:.1f}° (jump: {jump:.1f}°)")
            else:
                print("✓ No sudden angle jumps")
        
        # Check for zero or constant angles
        zero_count = sum(1 for angle in quest_angles if abs(angle) < 1.0)
        if zero_count > 8:  # More than half the joints are near zero
            print(f"⚠ {zero_count} joints near zero - possible tracking issues")
        
        # Store for comparison
        self.quest_angles_log.append(quest_angles.copy())
        if len(self.quest_angles_log) > 10:
            self.quest_angles_log.pop(0)
    
    def analyze_leap_conversion(self, quest_angles, leap_angles):
        """Analyze the conversion from Quest 2 to LEAP hand format"""
        print("\n🔍 LEAP HAND CONVERSION ANALYSIS:")
        print("=" * 50)
        
        # Check conversion formula
        print("Conversion formula: leap_angle = π + quest_angle_radians")
        
        # Check for proper range
        leap_degrees = np.degrees(leap_angles)
        out_of_range = []
        for i, angle_deg in enumerate(leap_degrees):
            if angle_deg < 0 or angle_deg > 360:
                out_of_range.append((i, angle_deg))
        
        if out_of_range:
            print(f"⚠ {len(out_of_range)} angles out of range:")
            for joint, angle in out_of_range:
                print(f"  Joint {joint}: {angle:.1f}°")
        else:
            print("✓ All LEAP angles within valid range")
        
        # Check for safety clipping
        if LEAP_AVAILABLE:
            safe_angles = lhu.angle_safety_clip(leap_angles)
            clipped_count = sum(1 for orig, safe in zip(leap_angles, safe_angles) if abs(orig - safe) > 0.01)
            if clipped_count > 0:
                print(f"⚠ {clipped_count} angles were safety-clipped")
            else:
                print("✓ No safety clipping needed")
        
        # Store for comparison
        self.leap_angles_log.append(leap_angles.copy())
        if len(self.leap_angles_log) > 10:
            self.leap_angles_log.pop(0)
    
    def test_specific_poses(self):
        """Test specific hand poses to check accuracy"""
        print("\n🎯 TESTING SPECIFIC POSES:")
        print("=" * 50)
        
        # Test poses: open, closed, individual fingers
        test_poses = {
            "Open Hand": np.zeros(16),
            "Closed Hand": np.array([0, 1.5, 1.5, 1.5] * 4),
            "Index Only": np.array([0, 1.0, 1.0, 1.0] + [0, 0, 0, 0] * 3),
            "Thumb Only": np.array([0, 0, 0, 0] * 3 + [0, 1.0, 1.0, 1.0])
        }
        
        for pose_name, allegro_pose in test_poses.items():
            print(f"\nTesting: {pose_name}")
            
            # Convert to LEAP hand format
            leap_pose = lhu.allegro_to_LEAPhand(allegro_pose)
            leap_pose = lhu.angle_safety_clip(leap_pose)
            
            print(f"  Allegro pose: {allegro_pose[:4]}...")
            print(f"  LEAP pose: {np.degrees(leap_pose[:4])}°...")
            
            # Send to LEAP hand if available
            if self.leap_hand:
                try:
                    self.leap_hand.write_desired_pos(list(range(16)), leap_pose)
                    print(f"  ✓ Sent to LEAP hand")
                    time.sleep(1)  # Wait for movement
                except Exception as e:
                    print(f"  ✗ Error sending to LEAP hand: {e}")
            else:
                print(f"  📋 Simulation mode")
    
    def monitor_live_data(self, duration=30):
        """Monitor live data to identify issues"""
        print(f"\n📊 MONITORING LIVE DATA FOR {duration} SECONDS:")
        print("=" * 50)
        print("🕐 Move your hand in different poses to test accuracy")
        print("🛑 Press Ctrl+C to stop early")
        
        start_time = time.time()
        frame_count = 0
        
        try:
            while time.time() - start_time < duration:
                data = self.sock.ReadReceivedData()
                
                if data is not None:
                    frame_count += 1
                    
                    # Parse Quest 2 data
                    parts = data.split()
                    if len(parts) >= 16:
                        try:
                            quest_angles = np.array([float(angle) for angle in parts[-16:]])
                            
                            # Analyze Quest 2 angles
                            self.analyze_quest_angles(quest_angles)
                            
                            # Convert to LEAP hand format
                            quest_radians = np.radians(quest_angles)
                            leap_angles = quest_radians + np.pi
                            
                            # Analyze conversion
                            self.analyze_leap_conversion(quest_angles, leap_angles)
                            
                            # Print summary every 10 frames
                            if frame_count % 10 == 0:
                                print(f"\n📊 Frame {frame_count} Summary:")
                                print(f"  Quest angles range: {np.min(quest_angles):.1f}° to {np.max(quest_angles):.1f}°")
                                print(f"  LEAP angles range: {np.min(np.degrees(leap_angles)):.1f}° to {np.max(np.degrees(leap_angles)):.1f}°")
                                
                        except ValueError as e:
                            print(f"⚠ Could not parse angles: {e}")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print(f"\n🛑 Monitoring stopped by user")
        
        print(f"\n📊 MONITORING COMPLETE:")
        print(f"  Frames processed: {frame_count}")
        print(f"  Average frame rate: {frame_count/duration:.1f} Hz")
    
    def generate_report(self):
        """Generate a comprehensive accuracy report"""
        print("\n📋 ACCURACY INVESTIGATION REPORT:")
        print("=" * 60)
        
        if len(self.quest_angles_log) > 0:
            quest_data = np.array(self.quest_angles_log)
            print(f"\n🔍 QUEST 2 ANGLE STATISTICS:")
            print(f"  Mean angle range: {np.mean(np.min(quest_data, axis=1)):.1f}° to {np.mean(np.max(quest_data, axis=1)):.1f}°")
            print(f"  Standard deviation: {np.mean(np.std(quest_data, axis=1)):.1f}°")
            print(f"  Unrealistic angles: {np.sum(np.abs(quest_data) > 180)}")
        
        if len(self.leap_angles_log) > 0:
            leap_data = np.array(self.leap_angles_log)
            leap_degrees = np.degrees(leap_data)
            print(f"\n🔍 LEAP HAND ANGLE STATISTICS:")
            print(f"  Mean angle range: {np.mean(np.min(leap_degrees, axis=1)):.1f}° to {np.mean(np.max(leap_degrees, axis=1)):.1f}°")
            print(f"  Standard deviation: {np.mean(np.std(leap_degrees, axis=1)):.1f}°")
        
        print(f"\n🎯 RECOMMENDATIONS:")
        print("1. Check Quest 2 hand tracking calibration")
        print("2. Verify joint angle extraction in HandController.cs")
        print("3. Test with known hand poses")
        print("4. Consider improving quaternion-to-angle conversion")

def main():
    print("=" * 80)
    print("QUEST 2 TO LEAP HAND ACCURACY INVESTIGATION")
    print("=" * 80)
    
    investigator = AccuracyInvestigator()
    
    if not investigator.sock:
        print("✗ UDP communication not available")
        return
    
    # Test specific poses first
    investigator.test_specific_poses()
    
    # Monitor live data
    investigator.monitor_live_data(duration=20)
    
    # Generate report
    investigator.generate_report()

if __name__ == "__main__":
    main()
