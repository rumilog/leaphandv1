#!/usr/bin/env python3
"""
Hand Tracking Data Analysis Tool

This script analyzes the hand tracking data to see if it follows expected patterns
for different hand poses (stop sign, pointing, peace sign, fist, etc.)
"""

import re
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

def parse_log_file(filename):
    """Parse the hand tracking log file and extract joint angles"""
    
    data = {
        'timestamps': [],
        'joints': {
            'Index': {'DIP': [], 'PIP': [], 'MCP_Flex': [], 'MCP_Abd': []},
            'Middle': {'DIP': [], 'PIP': [], 'MCP_Flex': [], 'MCP_Abd': []},
            'Ring': {'DIP': [], 'PIP': [], 'MCP_Flex': [], 'MCP_Abd': []},
            'Thumb': {'DIP': [], 'PIP': [], 'MCP_Flex': [], 'MCP_Abd': []}
        }
    }
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    current_timestamp = None
    in_data_section = False
    data_points = 0
    
    for i, line in enumerate(lines):
        line = line.strip()
        
        # Skip empty lines and headers
        if not line or line.startswith('HAND TRACKING DATA LOG') or line.startswith('=') or line.startswith('Format:') or line.startswith('Started:'):
            continue
            
        # Debug: show first few lines
        if i < 20:
            print(f"Line {i}: {line}")
            
        # Check for data start
        if 'HAND_DATA_START' in line:
            in_data_section = True
            print(f"Debug: Found data start at line {i}")
            continue
            
        # Check for data end
        if 'HAND_DATA_END' in line:
            in_data_section = False
            print(f"Debug: Found data end at line {i}")
            continue
            
        # Parse timestamp and joint data
        if in_data_section and line.startswith('['):
            # Extract timestamp
            timestamp_match = re.match(r'\[(\d{2}:\d{2}:\d{2}\.\d{3})\]', line)
            if timestamp_match:
                current_timestamp = timestamp_match.group(1)
                if current_timestamp not in data['timestamps']:  # Avoid duplicates
                    data['timestamps'].append(current_timestamp)
            
            # Parse joint data - handle the format: [timestamp] Finger Joint angle
            joint_match = re.match(r'\[.*?\] (\w+) (\w+) ([\d.-]+)', line)
            if joint_match:
                finger = joint_match.group(1)
                joint = joint_match.group(2)
                angle = float(joint_match.group(3))
                
                if finger in data['joints'] and joint in data['joints'][finger]:
                    data['joints'][finger][joint].append(angle)
                    data_points += 1
                    if data_points <= 5:  # Show first few parsed values
                        print(f"Debug: Parsed {finger} {joint} = {angle}")
            else:
                if i < 20:  # Show why parsing failed for first few lines
                    print(f"Debug: Failed to parse line {i}: {line}")
    
    print(f"Debug: Parsed {data_points} joint angle values")
    print(f"Debug: Found {len(data['timestamps'])} unique timestamps")
    
    return data

def analyze_pose_patterns(data):
    """Analyze the data to identify different hand poses"""
    
    print("=" * 80)
    print("HAND POSE ANALYSIS")
    print("=" * 80)
    
    # Convert to numpy arrays for easier analysis
    for finger in data['joints']:
        for joint in data['joints'][finger]:
            data['joints'][finger][joint] = np.array(data['joints'][finger][joint])
    
    # Calculate statistics for each joint
    print("\nJOINT STATISTICS:")
    print("-" * 50)
    
    for finger in ['Index', 'Middle', 'Ring', 'Thumb']:
        print(f"\n{finger} Finger:")
        for joint in ['DIP', 'PIP', 'MCP_Flex', 'MCP_Abd']:
            angles = data['joints'][finger][joint]
            if len(angles) > 0:
                print(f"  {joint:>8}: Min={angles.min():6.2f}° Max={angles.max():6.2f}° Avg={angles.mean():6.2f}° Range={angles.max()-angles.min():6.2f}°")
    
    # Analyze pose patterns
    print("\n" + "=" * 80)
    print("POSE PATTERN ANALYSIS")
    print("=" * 80)
    
    # Look for "Stop Sign" pose (low flexion, high abduction)
    print("\n1. STOP SIGN ANALYSIS (Expected: Low flexion, High abduction):")
    print("-" * 60)
    
    # Find periods with low overall flexion
    total_flexion = (data['joints']['Index']['DIP'] + data['joints']['Index']['PIP'] + 
                    data['joints']['Middle']['DIP'] + data['joints']['Middle']['PIP'] +
                    data['joints']['Ring']['DIP'] + data['joints']['Ring']['PIP'] +
                    data['joints']['Thumb']['DIP'] + data['joints']['Thumb']['PIP']) / 8
    
    low_flexion_threshold = np.percentile(total_flexion, 20)  # Bottom 20%
    high_abduction_threshold = np.percentile(np.abs(data['joints']['Index']['MCP_Abd']), 80)  # Top 20%
    
    stop_sign_indices = np.where((total_flexion < low_flexion_threshold) & 
                                (np.abs(data['joints']['Index']['MCP_Abd']) > high_abduction_threshold))[0]
    
    if len(stop_sign_indices) > 0:
        print(f"✓ Found {len(stop_sign_indices)} potential 'Stop Sign' poses")
        print(f"  - Low flexion threshold: {low_flexion_threshold:.2f}°")
        print(f"  - High abduction threshold: {high_abduction_threshold:.2f}°")
        
        # Show sample stop sign data
        sample_idx = stop_sign_indices[0]
        print(f"  - Sample at index {sample_idx}:")
        for finger in ['Index', 'Middle', 'Ring', 'Thumb']:
            print(f"    {finger}: DIP={data['joints'][finger]['DIP'][sample_idx]:.1f}° PIP={data['joints'][finger]['PIP'][sample_idx]:.1f}° MCP_Flex={data['joints'][finger]['MCP_Flex'][sample_idx]:.1f}° MCP_Abd={data['joints'][finger]['MCP_Abd'][sample_idx]:.1f}°")
    else:
        print("✗ No clear 'Stop Sign' poses detected")
    
    # Look for "Fist" pose (high flexion, low abduction)
    print("\n2. FIST ANALYSIS (Expected: High flexion, Low abduction):")
    print("-" * 60)
    
    high_flexion_threshold = np.percentile(total_flexion, 80)  # Top 20%
    low_abduction_threshold = np.percentile(np.abs(data['joints']['Index']['MCP_Abd']), 20)  # Bottom 20%
    
    fist_indices = np.where((total_flexion > high_flexion_threshold) & 
                           (np.abs(data['joints']['Index']['MCP_Abd']) < low_abduction_threshold))[0]
    
    if len(fist_indices) > 0:
        print(f"✓ Found {len(fist_indices)} potential 'Fist' poses")
        print(f"  - High flexion threshold: {high_flexion_threshold:.2f}°")
        print(f"  - Low abduction threshold: {low_abduction_threshold:.2f}°")
        
        # Show sample fist data
        sample_idx = fist_indices[0]
        print(f"  - Sample at index {sample_idx}:")
        for finger in ['Index', 'Middle', 'Ring', 'Thumb']:
            print(f"    {finger}: DIP={data['joints'][finger]['DIP'][sample_idx]:.1f}° PIP={data['joints'][finger]['PIP'][sample_idx]:.1f}° MCP_Flex={data['joints'][finger]['MCP_Flex'][sample_idx]:.1f}° MCP_Abd={data['joints'][finger]['MCP_Abd'][sample_idx]:.1f}°")
    else:
        print("✗ No clear 'Fist' poses detected")
    
    # Look for "Pointing" pose (Index extended, others flexed)
    print("\n3. POINTING ANALYSIS (Expected: Index straight, others flexed):")
    print("-" * 60)
    
    # Index should be straighter than others
    index_straightness = (data['joints']['Index']['DIP'] + data['joints']['Index']['PIP']) / 2
    other_flexion = (data['joints']['Middle']['DIP'] + data['joints']['Middle']['PIP'] +
                    data['joints']['Ring']['DIP'] + data['joints']['Ring']['PIP']) / 4
    
    pointing_indices = np.where((index_straightness < 20) & (other_flexion > 30))[0]
    
    if len(pointing_indices) > 0:
        print(f"✓ Found {len(pointing_indices)} potential 'Pointing' poses")
        print(f"  - Index straightness threshold: <20°")
        print(f"  - Other fingers flexion threshold: >30°")
        
        # Show sample pointing data
        sample_idx = pointing_indices[0]
        print(f"  - Sample at index {sample_idx}:")
        for finger in ['Index', 'Middle', 'Ring', 'Thumb']:
            print(f"    {finger}: DIP={data['joints'][finger]['DIP'][sample_idx]:.1f}° PIP={data['joints'][finger]['PIP'][sample_idx]:.1f}° MCP_Flex={data['joints'][finger]['MCP_Flex'][sample_idx]:.1f}° MCP_Abd={data['joints'][finger]['MCP_Abd'][sample_idx]:.1f}°")
    else:
        print("✗ No clear 'Pointing' poses detected")
    
    # Overall movement analysis
    print("\n4. OVERALL MOVEMENT ANALYSIS:")
    print("-" * 60)
    
    # Calculate movement ranges
    total_range = 0
    active_joints = 0
    
    for finger in ['Index', 'Middle', 'Ring', 'Thumb']:
        for joint in ['DIP', 'PIP', 'MCP_Flex', 'MCP_Abd']:
            angles = data['joints'][finger][joint]
            if len(angles) > 0:
                joint_range = angles.max() - angles.min()
                total_range += joint_range
                if joint_range > 5:  # Consider joint "active" if range > 5°
                    active_joints += 1
                    print(f"  {finger} {joint}: Range = {joint_range:.1f}° (Active)")
                else:
                    print(f"  {finger} {joint}: Range = {joint_range:.1f}° (Static)")
    
    print(f"\n  Total active joints: {active_joints}/16")
    print(f"  Average joint range: {total_range/16:.1f}°")
    
    # Check for realistic biomechanical patterns
    print("\n5. BIOMECHANICAL VALIDATION:")
    print("-" * 60)
    
    issues = []
    
    # Check for unrealistic angles
    for finger in ['Index', 'Middle', 'Ring', 'Thumb']:
        for joint in ['DIP', 'PIP', 'MCP_Flex', 'MCP_Abd']:
            angles = data['joints'][finger][joint]
            if len(angles) > 0:
                if angles.max() > 90:
                    issues.append(f"{finger} {joint} exceeds 90° (max: {angles.max():.1f}°)")
                if angles.min() < -30:
                    issues.append(f"{finger} {joint} below -30° (min: {angles.min():.1f}°)")
    
    if issues:
        print("⚠ Issues found:")
        for issue in issues[:10]:  # Show first 10 issues
            print(f"  - {issue}")
        if len(issues) > 10:
            print(f"  ... and {len(issues)-10} more issues")
    else:
        print("✓ All angles within realistic ranges")
    
    return data

def main():
    filename = "hand_tracking_data_20251010_130037.txt"
    
    print("Analyzing hand tracking data...")
    print(f"File: {filename}")
    
    try:
        data = parse_log_file(filename)
        print(f"✓ Successfully parsed {len(data['timestamps'])} data points")
        
        data = analyze_pose_patterns(data)
        
        print("\n" + "=" * 80)
        print("ANALYSIS COMPLETE")
        print("=" * 80)
        
    except FileNotFoundError:
        print(f"✗ Error: File '{filename}' not found")
    except Exception as e:
        print(f"✗ Error: {e}")

if __name__ == "__main__":
    main()