#!/usr/bin/env python3
"""
Text-to-Pose LEAP Hand Controller
Convert text commands to hand poses using numpy arrays
"""
import numpy as np
import time
from leap_hand_utils.dynamixel_client import DynamixelClient
import leap_hand_utils.leap_hand_utils as lhu

class TextToPoseController:
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
        
        # Define pose library
        self.pose_library = self._create_pose_library()
        
        print("🤖 Text-to-Pose LEAP Hand Controller Ready!")
        print("Type 'help' to see available poses, 'quit' to exit")
    
    def _create_pose_library(self):
        """Create a library of hand poses"""
        return {
            # Basic poses
            "open": np.zeros(16),
            "open hand": np.zeros(16),
            "close": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            "closed": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            "fist": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            "closed fist": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            
            # Gestures
            "peace sign": np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            "peace": np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            "victory": np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            "v sign": np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            
            "thumbs up": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0, 0, 0]),
            "thumb up": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0, 0, 0]),
            "good": np.array([0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0, 0, 0]),
            
            "point": np.array([0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            "pointing": np.array([0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            "index point": np.array([0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            
            "ok sign": np.array([0, 0.8, 0.8, 0.8, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0.8, 0.8, 0.8]),
            "ok": np.array([0, 0.8, 0.8, 0.8, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0.8, 0.8, 0.8]),
            "okay": np.array([0, 0.8, 0.8, 0.8, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0.8, 0.8, 0.8]),
            
            "rock on": np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 0, 0, 0]),
            "rock": np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 0, 0, 0]),
            
            "middle finger": np.array([0, 1.5, 1.5, 1.5, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            "flip off": np.array([0, 1.5, 1.5, 1.5, 0, 0, 0, 0, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5]),
            
            # Partial poses
            "half fist": np.array([0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8]),
            "half closed": np.array([0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8, 0, 0.8, 0.8, 0.8]),
            
            "pinch": np.array([0, 0.5, 0.5, 0.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0.5, 0.5, 0.5]),
            "pinch grip": np.array([0, 0.5, 0.5, 0.5, 0, 1.5, 1.5, 1.5, 0, 1.5, 1.5, 1.5, 0, 0.5, 0.5, 0.5]),
            
            # MCP Side movements
            "spread": np.array([0.3, 0, 0, 0, -0.3, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0]),
            "spread fingers": np.array([0.3, 0, 0, 0, -0.3, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0]),
            "abduct": np.array([0.3, 0, 0, 0, -0.3, 0, 0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0]),
            
            "close together": np.array([-0.3, 0, 0, 0, 0.3, 0, 0, 0, -0.2, 0, 0, 0, 0, 0, 0, 0]),
            "adduct": np.array([-0.3, 0, 0, 0, 0.3, 0, 0, 0, -0.2, 0, 0, 0, 0, 0, 0, 0]),
        }
    
    def move_to_pose(self, pose_name):
        """Move hand to a specific pose by name"""
        pose_name = pose_name.lower().strip()
        
        if pose_name not in self.pose_library:
            print(f"❌ Unknown pose: '{pose_name}'")
            print("Type 'help' to see available poses")
            return False
        
        pose = self.pose_library[pose_name]
        print(f"🎭 Executing pose: {pose_name}")
        print(f"   Pose array: {pose}")
        
        # Convert to LEAP convention
        leap_pose = lhu.allegro_to_LEAPhand(pose, zeros=False)
        
        # Handle MCP Side joints specially
        leap_pose[0] = 3.14159 + pose[0]   # Index MCP Side
        leap_pose[4] = 3.14159 + pose[4]   # Middle MCP Side  
        leap_pose[8] = 3.14159 + pose[8]   # Ring MCP Side
        
        print(f"   LEAP array: {leap_pose}")
        
        # Move hand
        self.dxl_client.write_desired_pos(self.motors, leap_pose)
        time.sleep(2)
        
        # Read actual position
        actual = self.dxl_client.read_pos()
        print(f"✅ Hand moved to {pose_name}")
        return True
    
    def show_help(self):
        """Show available poses"""
        print("\n🎭 Available Hand Poses:")
        print("=" * 50)
        
        categories = {
            "Basic Poses": ["open", "close", "fist", "half fist"],
            "Gestures": ["peace sign", "thumbs up", "point", "ok sign", "rock on"],
            "Grips": ["pinch", "spread", "close together"],
            "Other": ["middle finger", "flip off"]
        }
        
        for category, poses in categories.items():
            print(f"\n{category}:")
            for pose in poses:
                print(f"  • {pose}")
        
        print(f"\nTotal: {len(self.pose_library)} poses available")
        print("\nUsage: Type any pose name to execute it")
        print("Examples: 'peace sign', 'thumbs up', 'open', 'fist'")
    
    def get_current_pose(self):
        """Get current hand pose"""
        actual = self.dxl_client.read_pos()
        allegro = lhu.LEAPhand_to_allegro(actual, zeros=False)
        print(f"📊 Current pose (Allegro): {allegro}")
        print(f"📊 Current pose (LEAP): {actual}")
        return allegro
    
    def run_interactive(self):
        """Run interactive text-to-pose controller"""
        print("\n🎮 Interactive Text-to-Pose Controller")
        print("Type pose names to control the hand!")
        print("Commands: 'help', 'status', 'quit'")
        print("-" * 50)
        
        while True:
            try:
                user_input = input("\nEnter pose: ").strip()
                
                if user_input.lower() == 'quit':
                    print("👋 Goodbye!")
                    break
                elif user_input.lower() == 'help':
                    self.show_help()
                elif user_input.lower() == 'status':
                    self.get_current_pose()
                elif user_input.lower() == 'list':
                    print("Available poses:", list(self.pose_library.keys()))
                else:
                    self.move_to_pose(user_input)
                    
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

def main():
    """Main function"""
    try:
        controller = TextToPoseController()
        controller.run_interactive()
    except Exception as e:
        print(f"❌ Failed to initialize: {e}")
        print("Make sure the LEAP Hand is connected and powered on!")

if __name__ == "__main__":
    main()




