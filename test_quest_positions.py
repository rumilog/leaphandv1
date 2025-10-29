#!/usr/bin/env python3
"""
Test Quest Hand Positions on LEAP Hand

This script extracts 3 distinct hand positions from Quest tracking data,
converts them to LEAP hand coordinates, and tests them on the physical hand.
"""

import numpy as np
import time
import re
from LEAP_Hand_API.python.leap_hand_utils.dynamixel_client import *
import LEAP_Hand_API.python.leap_hand_utils.leap_hand_utils as lhu

def parse_quest_data(filename):
    """Parse Quest hand tracking data and extract distinct positions"""
    
    positions = []
    current_position = {}
    in_data_section = False
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    for line in lines:
        line = line.strip()
        
        # Check for data start
        if 'HAND_DATA_START' in line:
            in_data_section = True
            current_position = {}
            continue
            
        # Check for data end
        if 'HAND_DATA_END' in line:
            in_data_section = False
            if len(current_position) == 16:  # All 16 joints captured
                positions.append(current_position.copy())
            continue
            
        # Parse joint data
        if in_data_section and line.startswith('['):
            joint_match = re.match(r'\[.*?\] (\w+) (\w+) ([\d.-]+)', line)
            if joint_match:
                finger = joint_match.group(1)
                joint = joint_match.group(2)
                angle = float(joint_match.group(3))
                
                # Map Quest joints to our 16-joint array
                joint_key = f"{finger}_{joint}"
                current_position[joint_key] = angle
    
    print(f"Parsed {len(positions)} complete hand positions from Quest data")
    return positions

def quest_to_leap_mapping(quest_position):
    """Convert Quest hand position to LEAP hand format"""
    
    # Quest joint mapping to LEAP hand 16-joint array:
    # Index[0-3], Middle[4-7], Ring[8-11], Thumb[12-15]
    # Each finger: [MCP Side, MCP Forward, PIP, DIP]
    
    leap_position = np.zeros(16)
    
    # Index finger (joints 0-3)
    leap_position[0] = quest_position.get('Index_MCP_Abd', 0) * np.pi / 180  # MCP Side
    leap_position[1] = quest_position.get('Index_MCP_Flex', 0) * np.pi / 180  # MCP Forward  
    leap_position[2] = quest_position.get('Index_PIP', 0) * np.pi / 180  # PIP
    leap_position[3] = quest_position.get('Index_DIP', 0) * np.pi / 180  # DIP
    
    # Middle finger (joints 4-7)
    leap_position[4] = quest_position.get('Middle_MCP_Abd', 0) * np.pi / 180  # MCP Side
    leap_position[5] = quest_position.get('Middle_MCP_Flex', 0) * np.pi / 180  # MCP Forward
    leap_position[6] = quest_position.get('Middle_PIP', 0) * np.pi / 180  # PIP
    leap_position[7] = quest_position.get('Middle_DIP', 0) * np.pi / 180  # DIP
    
    # Ring finger (joints 8-11)
    leap_position[8] = quest_position.get('Ring_MCP_Abd', 0) * np.pi / 180  # MCP Side
    leap_position[9] = quest_position.get('Ring_MCP_Flex', 0) * np.pi / 180  # MCP Forward
    leap_position[10] = quest_position.get('Ring_PIP', 0) * np.pi / 180  # PIP
    leap_position[11] = quest_position.get('Ring_DIP', 0) * np.pi / 180  # DIP
    
    # Thumb (joints 12-15)
    leap_position[12] = quest_position.get('Thumb_MCP_Abd', 0) * np.pi / 180  # MCP Side
    leap_position[13] = quest_position.get('Thumb_MCP_Flex', 0) * np.pi / 180  # MCP Forward
    leap_position[14] = quest_position.get('Thumb_PIP', 0) * np.pi / 180  # PIP
    leap_position[15] = quest_position.get('Thumb_DIP', 0) * np.pi / 180  # DIP
    
    return leap_position

def select_diverse_positions(positions, num_positions=3):
    """Select diverse hand positions from the data"""
    
    if len(positions) < num_positions:
        print(f"Warning: Only {len(positions)} positions available, using all")
        return positions[:num_positions]
    
    # Convert to numpy arrays for analysis
    leap_positions = []
    for pos in positions:
        leap_pos = quest_to_leap_mapping(pos)
        leap_positions.append(leap_pos)
    
    leap_positions = np.array(leap_positions)
    
    # Calculate diversity based on joint angle differences
    selected_indices = [0]  # Start with first position
    remaining_indices = list(range(1, len(leap_positions)))
    
    while len(selected_indices) < num_positions and remaining_indices:
        max_min_distance = -1
        best_candidate = remaining_indices[0]
        
        for candidate in remaining_indices:
            # Calculate minimum distance to already selected positions
            min_distance = float('inf')
            for selected in selected_indices:
                distance = np.linalg.norm(leap_positions[candidate] - leap_positions[selected])
                min_distance = min(min_distance, distance)
            
            if min_distance > max_min_distance:
                max_min_distance = min_distance
                best_candidate = candidate
        
        selected_indices.append(best_candidate)
        remaining_indices.remove(best_candidate)
    
    return [positions[i] for i in selected_indices]

class QuestLeapTester:
    def __init__(self, port='/dev/ttyUSB0'):
        """Initialize LEAP hand controller for testing Quest positions"""
        
        self.kP = 600
        self.kI = 0
        self.kD = 200
        self.curr_lim = 350
        self.motors = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15]
        
        # Connect to hand
        self.dxl_client = DynamixelClient(self.motors, port, 4000000)
        self.dxl_client.connect()
        
        # Setup control mode
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors))*5, 11, 1)
        self.dxl_client.set_torque_enabled(self.motors, True)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kP, 84, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kP * 0.75), 84, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kI, 82, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.kD, 80, 2)
        self.dxl_client.sync_write([0,4,8], np.ones(3) * (self.kD * 0.75), 80, 2)
        self.dxl_client.sync_write(self.motors, np.ones(len(self.motors)) * self.curr_lim, 102, 2)
        
        print("🤖 Quest-LEAP Hand Tester Ready!")
    
    def test_quest_position(self, quest_position, position_name="Unknown"):
        """Test a Quest position on the LEAP hand"""
        
        print(f"\n🎯 Testing Quest Position: {position_name}")
        print("-" * 50)
        
        # Convert Quest position to LEAP coordinates
        leap_position = quest_to_leap_mapping(quest_position)
        
        # Convert to Allegro coordinates for LEAP hand
        allegro_position = lhu.LEAPhand_to_allegro(leap_position, zeros=False)
        
        # Apply safety clipping
        allegro_position = lhu.angle_safety_clip(lhu.allegro_to_LEAPhand(allegro_position))
        allegro_position = lhu.LEAPhand_to_allegro(allegro_position)
        
        print(f"Quest angles (degrees):")
        for finger in ['Index', 'Middle', 'Ring', 'Thumb']:
            print(f"  {finger}: DIP={quest_position.get(f'{finger}_DIP', 0):.1f}° "
                  f"PIP={quest_position.get(f'{finger}_PIP', 0):.1f}° "
                  f"MCP_Flex={quest_position.get(f'{finger}_MCP_Flex', 0):.1f}° "
                  f"MCP_Abd={quest_position.get(f'{finger}_MCP_Abd', 0):.1f}°")
        
        print(f"\nConverted to Allegro coordinates (radians):")
        finger_names = ['Index', 'Middle', 'Ring', 'Thumb']
        joint_names = ['MCP_Side', 'MCP_Forward', 'PIP', 'DIP']
        
        for i, finger in enumerate(finger_names):
            print(f"  {finger}: ", end="")
            for j, joint in enumerate(joint_names):
                joint_idx = i * 4 + j
                print(f"{joint}={allegro_position[joint_idx]:.3f} ", end="")
            print()
        
        # Move hand to position
        try:
            leap_final = lhu.allegro_to_LEAPhand(allegro_position)
            self.dxl_client.write_desired_pos(self.motors, leap_final)
            print(f"✅ Successfully moved to {position_name}")
            return True
        except Exception as e:
            print(f"❌ Error moving to {position_name}: {e}")
            return False
    
    def open_hand(self):
        """Open hand to neutral position"""
        open_positions = np.zeros(16)
        leap_positions = lhu.allegro_to_LEAPhand(open_positions)
        self.dxl_client.write_desired_pos(self.motors, leap_positions)
        print("🤖 Hand opened to neutral position")

def main():
    """Main testing function"""
    
    print("🎮 Quest Hand Position Tester for LEAP Hand")
    print("=" * 60)
    
    # Parse Quest data
    filename = "hand_tracking_data_20251010_130037.txt"
    print(f"📁 Parsing Quest data from: {filename}")
    
    try:
        positions = parse_quest_data(filename)
        print(f"✅ Parsed {len(positions)} positions from Quest data")
        
        # Select 3 diverse positions
        selected_positions = select_diverse_positions(positions, 3)
        print(f"🎯 Selected {len(selected_positions)} diverse positions for testing")
        
        # Initialize LEAP hand
        print("\n🤖 Initializing LEAP hand...")
        tester = QuestLeapTester()
        
        # Test each position
        for i, position in enumerate(selected_positions):
            position_name = f"Quest_Position_{i+1}"
            
            # Open hand first
            print(f"\n🔄 Opening hand before position {i+1}...")
            tester.open_hand()
            time.sleep(2)
            
            # Test the Quest position
            success = tester.test_quest_position(position, position_name)
            
            if success:
                print(f"⏱️  Holding position {i+1} for 5 seconds...")
                time.sleep(5)
            else:
                print(f"⚠️  Skipping position {i+1} due to error")
        
        # Return to open position
        print("\n🔄 Returning to open position...")
        tester.open_hand()
        
        print("\n✅ Quest position testing complete!")
        print("📊 Check if the LEAP hand movements match your Quest hand movements")
        
    except FileNotFoundError:
        print(f"❌ Error: Quest data file '{filename}' not found")
    except Exception as e:
        print(f"❌ Error: {e}")

if __name__ == "__main__":
    main()

