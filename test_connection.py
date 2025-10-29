#!/usr/bin/env python3
"""
Test Quest 2 Connection

Simple script to test if we're receiving data from Quest 2
"""

import UdpComms as U
import time

def test_quest_connection():
    print("🔍 Testing Quest 2 connection...")
    
    try:
        # Use the same IP addresses as server_env.py
        sock = U.UdpComms(udpIP="172.26.71.187", sendIP="172.26.27.252", portTX=8000, portRX=8001, enableRX=True, suppressWarnings=True)
        print("✓ UDP communication setup successful")
        
        print("📡 Waiting for data from Quest 2...")
        print("🕐 Make sure your Quest 2 is running and sending data!")
        print("🛑 Press Ctrl+C to stop")
        
        data_count = 0
        start_time = time.time()
        
        while True:
            data = sock.ReadReceivedData()
            
            if data is not None:
                data_count += 1
                print(f"📊 Received data #{data_count}: {data}")
                
                if data_count >= 5:
                    print("✅ Connection successful! Quest 2 is sending data.")
                    break
            else:
                elapsed = time.time() - start_time
                if elapsed > 10:
                    print("⚠️ No data received after 10 seconds")
                    print("🔧 Check that Quest 2 is running and connected")
                    break
                
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print(f"\n🛑 Test stopped by user")
    except Exception as e:
        print(f"✗ Error: {e}")

if __name__ == "__main__":
    test_quest_connection()
