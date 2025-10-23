#!/usr/bin/env python3
"""
Test NumPy Array Control for LEAP Hand
Demonstrates how to control the hand using numpy arrays in different conventions
"""
import numpy as np
import time
from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu

class NumpyHandTester:
    def __init__(self):
        """Initialize the hand controller"""
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        
        try:
            self.dxl_client = DynamixelClient(self.motors, '/dev/ttyUSB0', 4000000)
            self.dxl_client.connect()
        except Exception as e:
            print(f"Error connecting: {e}")
            raise
        
        # Setup control mode
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)
        
        print("🤖 LEAP Hand NumPy Controller Ready!")
    
    def move_to_allegro_pose(self, allegro_pose):
        """Move hand to Allegro convention pose"""
        print(f"📐 Moving to Allegro pose: {allegro_pose}")
        leap_pose = lhu.allegro_to_LEAPhand(allegro_pose, zeros=False)
        
        # Handle MCP Side joints specially
        leap_pose[0] = 3.14159 + allegro_pose[0]   # Index MCP Side
        leap_pose[4] = 3.14159 + allegro_pose[4]   # Middle MCP Side  
        leap_pose[8] = 3.14159 + allegro_pose[8]   # Ring MCP Side
        
        self.dxl_client.write_desired_pos(self.motors, leap_pose)
        time.sleep(2)
        
        # Read actual position
        actual_pos = self.dxl_client.read_pos()
        print(f"✅ Actual position: {actual_pos}")
        return actual_pos
    
    def move_to_leap_pose(self, leap_pose):
        """Move hand to LEAP convention pose"""
        print(f"📐 Moving to LEAP pose: {leap_pose}")
        self.dxl_client.write_desired_pos(self.motors, leap_pose)
        time.sleep(2)
        
        # Read actual position
        actual_pos = self.dxl_client.read_pos()
        print(f"✅ Actual position: {actual_pos}")
        return actual_pos
    
    def move_to_simulation_pose(self, sim_pose):
        """Move hand to simulation convention pose (-1 to 1 range)"""
        print(f"📐 Moving to simulation pose: {sim_pose}")
        leap_pose = lhu.sim_ones_to_LEAPhand(sim_pose)
        self.dxl_client.write_desired_pos(self.motors, leap_pose)
        time.sleep(2)
        
        # Read actual position
        actual_pos = self.dxl_client.read_pos()
        print(f"✅ Actual position: {actual_pos}")
        return actual_pos
    
    def get_current_pose(self, convention='allegro'):
        """Get current hand pose in specified convention"""
        raw_pos = self.dxl_client.read_pos()
        if convention == 'allegro':
            return lhu.LEAPhand_to_allegro(raw_pos, zeros=False)
        elif convention == 'leap':
            return raw_pos
        elif convention == 'simulation':
            return lhu.LEAPhand_to_sim_ones(raw_pos)
        else:
            raise ValueError("Invalid convention. Choose 'allegro', 'leap', or 'simulation'")

def test_allegro_convention():
    """Test Allegro convention control"""
    print("\n" + "="*60)
    print("🧪 TESTING ALLEGRO CONVENTION")
    print("="*60)
    
    tester = NumpyHandTester()
    
    # Test 1: Open hand
    print("\n1️⃣ Open Hand (all zeros)")
    open_pose = np.zeros(16)
    tester.move_to_allegro_pose(open_pose)
    
    # Test 2: Half-closed fist
    print("\n2️⃣ Half-Closed Fist")
    half_fist = np.array([
        0, 0.8, 0.8, 0.8,  # Index
        0, 0.8, 0.8, 0.8,  # Middle
        0, 0.8, 0.8, 0.8,  # Ring
        0, 0.8, 0.8, 0.8   # Thumb
    ])
    tester.move_to_allegro_pose(half_fist)
    
    # Test 3: MCP Side movements
    print("\n3️⃣ MCP Side Movements")
    mcp_side_test = np.zeros(16)
    mcp_side_test[0] = 0.3   # Index MCP Side
    mcp_side_test[4] = -0.3  # Middle MCP Side
    mcp_side_test[8] = 0.2   # Ring MCP Side
    tester.move_to_allegro_pose(mcp_side_test)
    
    # Test 4: Custom pose
    print("\n4️⃣ Custom Pose")
    custom_pose = np.array([
        0.1, 0.5, 0.3, 0.2,  # Index
        0.0, 0.7, 0.4, 0.3,  # Middle
        -0.1, 0.6, 0.5, 0.4, # Ring
        0.2, 0.4, 0.2, 0.1   # Thumb
    ])
    tester.move_to_allegro_pose(custom_pose)
    
    # Return to open
    print("\n🔄 Returning to open position")
    tester.move_to_allegro_pose(np.zeros(16))

def test_leap_convention():
    """Test LEAP convention control"""
    print("\n" + "="*60)
    print("🧪 TESTING LEAP CONVENTION")
    print("="*60)
    
    tester = NumpyHandTester()
    
    # Test 1: Open hand (3.14 for most joints)
    print("\n1️⃣ Open Hand")
    open_pose = np.full(16, 3.14159)
    tester.move_to_leap_pose(open_pose)
    
    # Test 2: Closed fist
    print("\n2️⃣ Closed Fist")
    closed_pose = np.array([
        3.14, 4.64, 4.64, 4.64,  # Index (3.14 + 1.5)
        3.14, 4.64, 4.64, 4.64,  # Middle
        3.14, 4.64, 4.64, 4.64,  # Ring
        3.14, 4.64, 4.64, 4.64   # Thumb
    ])
    tester.move_to_leap_pose(closed_pose)
    
    # Test 3: MCP Side movements
    print("\n3️⃣ MCP Side Movements")
    mcp_side_test = np.full(16, 3.14159)
    mcp_side_test[0] = 3.44   # Index MCP Side (+0.3)
    mcp_side_test[4] = 2.84   # Middle MCP Side (-0.3)
    mcp_side_test[8] = 3.34   # Ring MCP Side (+0.2)
    tester.move_to_leap_pose(mcp_side_test)
    
    # Return to open
    print("\n🔄 Returning to open position")
    tester.move_to_leap_pose(np.full(16, 3.14159))

def test_simulation_convention():
    """Test simulation convention control (-1 to 1 range)"""
    print("\n" + "="*60)
    print("🧪 TESTING SIMULATION CONVENTION")
    print("="*60)
    
    tester = NumpyHandTester()
    
    # Test 1: Open hand (all zeros in sim convention)
    print("\n1️⃣ Open Hand")
    open_pose = np.zeros(16)
    tester.move_to_simulation_pose(open_pose)
    
    # Test 2: Half-closed fist
    print("\n2️⃣ Half-Closed Fist")
    half_fist = np.array([
        0, 0.5, 0.5, 0.5,  # Index
        0, 0.5, 0.5, 0.5,  # Middle
        0, 0.5, 0.5, 0.5,  # Ring
        0, 0.5, 0.5, 0.5   # Thumb
    ])
    tester.move_to_simulation_pose(half_fist)
    
    # Test 3: MCP Side movements
    print("\n3️⃣ MCP Side Movements")
    mcp_side_test = np.zeros(16)
    mcp_side_test[0] = 0.3   # Index MCP Side
    mcp_side_test[4] = -0.3  # Middle MCP Side
    mcp_side_test[8] = 0.2   # Ring MCP Side
    tester.move_to_simulation_pose(mcp_side_test)
    
    # Return to open
    print("\n🔄 Returning to open position")
    tester.move_to_simulation_pose(np.zeros(16))

def test_dynamic_control():
    """Test dynamic control with numpy arrays"""
    print("\n" + "="*60)
    print("🧪 TESTING DYNAMIC CONTROL")
    print("="*60)
    
    tester = NumpyHandTester()
    
    # Start with open hand
    tester.move_to_allegro_pose(np.zeros(16))
    
    # Create a sequence of poses
    poses = [
        ("Open", np.zeros(16)),
        ("Index Point", np.array([0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5])),
        ("Peace Sign", np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5])),
        ("Fist", np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5])),
        ("Thumbs Up", np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0, 0, 0])),
        ("Open", np.zeros(16))
    ]
    
    for name, pose in poses:
        print(f"\n🎭 {name}")
        tester.move_to_allegro_pose(pose)
        time.sleep(1)

def interactive_test():
    """Interactive test where you can input numpy arrays"""
    print("\n" + "="*60)
    print("🧪 INTERACTIVE NUMPY TEST")
    print("="*60)
    
    tester = NumpyHandTester()
    
    print("Enter numpy arrays to control the hand!")
    print("Format: [0, 0.5, 1.0, 0.2, ...] (16 values)")
    print("Type 'quit' to exit")
    print("Type 'status' to see current pose")
    print("Type 'open' to return to open position")
    
    while True:
        try:
            user_input = input("\nEnter pose (or command): ").strip()
            
            if user_input.lower() == 'quit':
                break
            elif user_input.lower() == 'status':
                current_allegro = tester.get_current_pose('allegro')
                current_leap = tester.get_current_pose('leap')
                print(f"Current Allegro pose: {current_allegro}")
                print(f"Current LEAP pose: {current_leap}")
            elif user_input.lower() == 'open':
                tester.move_to_allegro_pose(np.zeros(16))
            else:
                # Parse numpy array
                pose_str = user_input.replace('[', '').replace(']', '')
                pose_values = [float(x.strip()) for x in pose_str.split(',')]
                
                if len(pose_values) != 16:
                    print("❌ Error: Must provide exactly 16 values")
                    continue
                
                pose = np.array(pose_values)
                tester.move_to_allegro_pose(pose)
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"❌ Error: {e}")
    
    print("\n👋 Goodbye!")

def main():
    """Main test function"""
    print("🤖 LEAP Hand NumPy Array Control Test")
    print("This script demonstrates how to control the hand using numpy arrays")
    
    try:
        # Run all tests
        test_allegro_convention()
        test_leap_convention()
        test_simulation_convention()
        test_dynamic_control()
        
        # Interactive test
        interactive_test()
        
    except Exception as e:
        print(f"❌ Error: {e}")
        print("Make sure the LEAP Hand is connected and powered on!")

if __name__ == "__main__":
    main()
