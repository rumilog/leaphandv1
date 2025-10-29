#!/usr/bin/env python3
"""
Simple Quest 2 Connection Test
Tests if Quest 2 is sending hand tracking data via UDP.
"""

import socket
import time
import signal
import sys

class QuestConnectionTest:
    def __init__(self):
        self.udp_socket = None
        self.running = False
        self.setup_signal_handler()
        
    def setup_signal_handler(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        
    def signal_handler(self, sig, frame):
        print("\n🛑 Stopping Quest connection test...")
        self.running = False
        if self.udp_socket:
            self.udp_socket.close()
        sys.exit(0)
        
    def init_udp(self):
        """Initialize UDP connection to Quest 2"""
        try:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.bind(('0.0.0.0', 5005))
            self.udp_socket.settimeout(1.0)  # 1 second timeout
            print("✓ UDP socket initialized on port 5005")
            return True
        except Exception as e:
            print(f"❌ Failed to initialize UDP: {e}")
            return False
            
    def test_quest_connection(self):
        """Test Quest 2 connection for 30 seconds"""
        if not self.init_udp():
            return
            
        print("🔍 Testing Quest 2 Connection...")
        print("📋 Instructions:")
        print("   1. Make sure Quest 2 is running and sending data")
        print("   2. Move your hand in front of the Quest 2")
        print("   3. Press Ctrl+C to stop")
        print("\n" + "="*60)
        
        self.running = True
        start_time = time.time()
        message_count = 0
        
        while self.running and (time.time() - start_time) < 30:
            try:
                data, addr = self.udp_socket.recvfrom(1024)
                message = data.decode('utf-8').strip()
                message_count += 1
                
                print(f"📨 Message {message_count} from {addr}: {message[:100]}...")
                
                if message.startswith("HAND_DATA:"):
                    # Parse the hand data
                    data_part = message[10:]  # Remove "HAND_DATA:" prefix
                    try:
                        quest_angles = [float(x) for x in data_part.split(',')]
                        print(f"   ✓ Parsed {len(quest_angles)} joint angles")
                        
                        # Show first few angles
                        if len(quest_angles) >= 4:
                            print(f"   📊 Sample angles: {quest_angles[:4]}")
                            
                    except ValueError as e:
                        print(f"   ❌ Error parsing hand data: {e}")
                        
            except socket.timeout:
                print("⏳ Waiting for Quest 2 data...")
                continue
            except Exception as e:
                print(f"❌ Error receiving data: {e}")
                break
                
        print(f"\n📈 Connection Test Summary:")
        print(f"   • Received {message_count} messages")
        print(f"   • Time range: {time.time() - start_time:.1f} seconds")
        
        if message_count > 0:
            print("   ✅ Quest 2 connection successful!")
        else:
            print("   ❌ No data received - check Quest 2 connection")
            
        print("\n✅ Connection test completed!")

def main():
    print("🔍 Quest 2 Connection Test")
    print("="*50)
    
    test = QuestConnectionTest()
    test.test_quest_connection()

if __name__ == "__main__":
    main()
