#!/usr/bin/env python3
"""
Test LEAP Hand Movement with Quest 2 Data
"""
import sys
import os
import numpy as np
import time

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

def test_leap_hand_basic_movement():
    """Test basic LEAP hand movement"""
    
    if not LEAP_AVAILABLE:
        print("⚠ LEAP Hand API not available - running in simulation mode")
        return
    
    try:
        # Connect to LEAP hand on the correct port
        motors = list(range(16))
        dxl_client = DynamixelClient(motors, '/dev/ttyUSB1', 4000000)
        dxl_client.connect()
        print("✓ Connected to LEAP hand on /dev/ttyUSB1")
        
        # Configure hand
        dxl_client.sync_write(motors, np.ones(16)*5, 11, 1)
        dxl_client.set_torque_enabled(motors, True)
        dxl_client.sync_write(motors, np.ones(16) * 600, 84, 2)  # P-gain
        dxl_client.sync_write(motors, np.ones(16) * 0, 82, 2)    # I-gain  
        dxl_client.sync_write(motors, np.ones(16) * 200, 80, 2)  # D-gain
        dxl_client.sync_write(motors, np.ones(16) * 350, 102, 2) # Current limit
        print("✓ LEAP hand configured")
        
        # Test 1: Read current positions
        print("\n📊 Current hand positions:")
        current_pos = dxl_client.read_pos()
        print(f"Positions: {current_pos}")
        
        # Test 2: Open hand (all zeros in allegro coordinates)
        print("\n🤖 Testing: Open hand")
        open_positions = np.zeros(16)
        leap_positions = lhu.allegro_to_LEAPhand(open_positions)
        leap_positions = lhu.angle_safety_clip(leap_positions)
        dxl_client.write_desired_pos(motors, leap_positions)
        time.sleep(2)
        
        # Test 3: Close hand (all fingers bent)
        print("🤖 Testing: Close hand")
        close_positions = np.array([
            0, 1.5, 1.5, 1.5,  # Index
            0, 1.5, 1.5, 1.5,  # Middle
            0, 1.5, 1.5, 1.5,  # Ring
            0, 1.5, 1.5, 1.5   # Thumb
        ])
        leap_positions = lhu.allegro_to_LEAPhand(close_positions)
        leap_positions = lhu.angle_safety_clip(leap_positions)
        dxl_client.write_desired_pos(motors, leap_positions)
        time.sleep(2)
        
        # Test 4: Individual finger movement
        print("🤖 Testing: Individual finger movement")
        finger_positions = np.zeros(16)
        
        # Move index finger only
        finger_positions[0:4] = [0, 1.0, 1.0, 1.0]
        leap_positions = lhu.allegro_to_LEAPhand(finger_positions)
        leap_positions = lhu.angle_safety_clip(leap_positions)
        dxl_client.write_desired_pos(motors, leap_positions)
        time.sleep(1)
        
        # Move middle finger only
        finger_positions = np.zeros(16)
        finger_positions[4:8] = [0, 1.0, 1.0, 1.0]
        leap_positions = lhu.allegro_to_LEAPhand(finger_positions)
        leap_positions = lhu.angle_safety_clip(leap_positions)
        dxl_client.write_desired_pos(motors, leap_positions)
        time.sleep(1)
        
        # Test 5: Return to open position
        print("🤖 Testing: Return to open position")
        open_positions = np.zeros(16)
        leap_positions = lhu.allegro_to_LEAPhand(open_positions)
        leap_positions = lhu.angle_safety_clip(leap_positions)
        dxl_client.write_desired_pos(motors, leap_positions)
        time.sleep(2)
        
        print("✅ LEAP hand movement test completed successfully!")
        
    except Exception as e:
        print(f"❌ Error testing LEAP hand: {e}")

def test_quest_to_leap_conversion():
    """Test converting Quest 2 data to LEAP hand format"""
    
    # Sample Quest 2 data (from the connection test)
    quest_hand_position = np.array([0.19816, 0.03011, -0.18670])
    quest_hand_rotation = np.array([-0.15361, 0.25086, 0.95113, 0.09394])
    quest_finger_angles = np.array([
        32.03361, 21.68852, 72.34869, 20.00000,  # Index
        20.93213, 30.10728, 5.06608, 7.12455,    # Middle
        34.40340, 25.12161, 21.73530, -20.00000, # Ring
        49.06717, 26.40187, 55.05826, 57.07927   # Thumb
    ])
    
    print("\n📱 Quest 2 Hand Tracking Data:")
    print(f"Position: {quest_hand_position}")
    print(f"Rotation: {quest_hand_rotation}")
    print(f"Finger angles (degrees): {quest_finger_angles}")
    
    # Convert Quest 2 angles to LEAP hand format
    print("\n🔄 Converting Quest 2 data to LEAP hand format...")
    
    # Convert degrees to radians
    quest_radians = np.radians(quest_finger_angles)
    
    # Convert to LEAP hand coordinates (add π offset)
    leap_angles = quest_radians + np.pi
    
    # Apply safety clipping
    if LEAP_AVAILABLE:
        leap_angles = lhu.angle_safety_clip(leap_angles)
    
    print(f"LEAP hand angles (radians): {leap_angles}")
    print(f"LEAP hand angles (degrees): {np.degrees(leap_angles)}")
    
    return leap_angles

def main():
    print("=" * 80)
    print("LEAP HAND MOVEMENT TEST")
    print("=" * 80)
    
    # Test Quest 2 to LEAP conversion
    leap_angles = test_quest_to_leap_conversion()
    
    # Test LEAP hand movement
    if LEAP_AVAILABLE:
        test_leap_hand_basic_movement()
    else:
        print("\n⚠ LEAP Hand API not available - skipping movement test")
    
    print("\n" + "=" * 80)
    print("TEST COMPLETE")
    print("=" * 80)

if __name__ == "__main__":
    main()
