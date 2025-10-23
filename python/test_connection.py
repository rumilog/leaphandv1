#!/usr/bin/env python3
"""
Simple script to test LEAP Hand connection
"""
import os
import glob
from leap_hand_utils.dynamixel_client import DynamixelClient
import numpy as np

def find_serial_ports():
    """Find available serial ports"""
    ports = []
    
    # Check common Linux USB serial ports
    for pattern in ['/dev/ttyUSB*', '/dev/ttyACM*']:
        ports.extend(glob.glob(pattern))
    
    # Check by-id directory if it exists
    if os.path.exists('/dev/serial/by-id'):
        ports.extend(glob.glob('/dev/serial/by-id/*'))
    
    return ports

def test_connection(port):
    """Test connection to a specific port"""
    try:
        print(f"Testing connection to {port}...")
        motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        dxl_client = DynamixelClient(motors, port, 4000000)
        dxl_client.connect()
        
        # Try to read position
        positions = dxl_client.read_pos()
        print(f"✅ SUCCESS! Connected to LEAP Hand on {port}")
        print(f"Current positions: {positions}")
        return True
        
    except Exception as e:
        print(f"❌ Failed to connect to {port}: {e}")
        return False

def main():
    print("🔍 Looking for LEAP Hand...")
    print("Make sure your LEAP Hand is:")
    print("1. Connected to 5V power (motors should be lit)")
    print("2. Connected via Micro-USB cable")
    print("3. Not being used by Dynamixel Wizard")
    print()
    
    # Find available ports
    ports = find_serial_ports()
    
    if not ports:
        print("❌ No serial ports found!")
        print("Try:")
        print("- Check USB connection")
        print("- Check if hand is powered on")
        print("- Try a different USB port")
        return
    
    print(f"Found {len(ports)} serial port(s): {ports}")
    print()
    
    # Test each port
    for port in ports:
        if test_connection(port):
            print(f"\n🎉 LEAP Hand found on {port}!")
            print("You can now use this port in main.py")
            return
    
    print("\n❌ Could not connect to LEAP Hand on any port")
    print("Troubleshooting:")
    print("1. Ensure hand is powered on (motors lit)")
    print("2. Check USB cable connection")
    print("3. Try different USB port")
    print("4. Check if Dynamixel Wizard is running (close it)")
    print("5. Try: sudo chmod 666 /dev/ttyUSB*")

if __name__ == "__main__":
    main()



