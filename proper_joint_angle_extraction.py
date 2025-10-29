#!/usr/bin/env python3
"""
Proper Joint Angle Extraction from Hand Tracking Data

This demonstrates the correct way to extract joint angles from quaternions
using anatomical understanding rather than arbitrary axis testing.
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

class ProperHandJointExtractor:
    """
    Proper joint angle extraction using anatomical understanding
    """
    
    def __init__(self):
        # Define anatomical joint axes for each finger
        # These should come from the hand model's coordinate frames
        self.joint_axes = {
            'Index': {
                'MCP_Abd': np.array([0, 1, 0]),      # Abduction/adduction axis
                'MCP_Flex': np.array([1, 0, 0]),     # Flexion/extension axis
                'PIP_Flex': np.array([1, 0, 0]),     # PIP flexion axis
                'DIP_Flex': np.array([1, 0, 0])      # DIP flexion axis
            },
            'Middle': {
                'MCP_Abd': np.array([0, 1, 0]),
                'MCP_Flex': np.array([1, 0, 0]),
                'PIP_Flex': np.array([1, 0, 0]),
                'DIP_Flex': np.array([1, 0, 0])
            },
            'Ring': {
                'MCP_Abd': np.array([0, 1, 0]),
                'MCP_Flex': np.array([1, 0, 0]),
                'PIP_Flex': np.array([1, 0, 0]),
                'DIP_Flex': np.array([1, 0, 0])
            },
            'Thumb': {
                'MCP_Abd': np.array([0, 1, 0]),
                'MCP_Flex': np.array([1, 0, 0]),
                'PIP_Flex': np.array([1, 0, 0]),
                'DIP_Flex': np.array([1, 0, 0])
            }
        }
        
        # Joint limits in degrees (anatomical limits)
        self.joint_limits = {
            'Index': {'MCP_Abd': 30, 'MCP_Flex': 90, 'PIP_Flex': 100, 'DIP_Flex': 80},
            'Middle': {'MCP_Abd': 30, 'MCP_Flex': 90, 'PIP_Flex': 100, 'DIP_Flex': 80},
            'Ring': {'MCP_Abd': 30, 'MCP_Flex': 90, 'PIP_Flex': 100, 'DIP_Flex': 80},
            'Thumb': {'MCP_Abd': 45, 'MCP_Flex': 60, 'PIP_Flex': 80, 'DIP_Flex': 70}
        }
    
    def extract_joint_angles_proper(self, bone_rotations):
        """
        Proper joint angle extraction using anatomical understanding
        
        Args:
            bone_rotations: List of quaternions for each bone in the finger
            
        Returns:
            dict: Joint angles in degrees
        """
        if len(bone_rotations) < 4:
            return None
            
        # Extract relative rotations between consecutive bones
        # This is the key - we need the RELATIVE rotation between bones
        q_base_to_prox = bone_rotations[1] * bone_rotations[0].inv()
        q_prox_to_mid = bone_rotations[2] * bone_rotations[1].inv()
        q_mid_to_dist = bone_rotations[3] * bone_rotations[2].inv()
        
        # Extract joint angles using proper anatomical axes
        joint_angles = {}
        
        # MCP Joint (2-DOF: abduction + flexion)
        mcp_abd_angle = self.extract_angle_around_axis(q_base_to_prox, 'MCP_Abd')
        mcp_flex_angle = self.extract_angle_around_axis(q_base_to_prox, 'MCP_Flex')
        
        # PIP Joint (1-DOF: flexion)
        pip_flex_angle = self.extract_angle_around_axis(q_prox_to_mid, 'PIP_Flex')
        
        # DIP Joint (1-DOF: flexion)
        dip_flex_angle = self.extract_angle_around_axis(q_mid_to_dist, 'DIP_Flex')
        
        return {
            'MCP_Abd': mcp_abd_angle,
            'MCP_Flex': mcp_flex_angle,
            'PIP_Flex': pip_flex_angle,
            'DIP_Flex': dip_flex_angle
        }
    
    def extract_angle_around_axis(self, quaternion, joint_type):
        """
        Extract rotation angle around a specific anatomical axis
        
        This is the proper way to get joint angles from quaternions
        """
        # Convert quaternion to rotation matrix
        rotation_matrix = quaternion.as_matrix()
        
        # Get the anatomical axis for this joint type
        axis = self.get_anatomical_axis(joint_type)
        
        # Project the rotation onto the joint axis
        # This gives us the rotation angle around the anatomical axis
        angle = self.angle_around_axis(rotation_matrix, axis)
        
        return np.degrees(angle)
    
    def get_anatomical_axis(self, joint_type):
        """Get the anatomical axis for a joint type"""
        # This should come from the hand model's coordinate frames
        axis_map = {
            'MCP_Abd': np.array([0, 1, 0]),    # Abduction axis
            'MCP_Flex': np.array([1, 0, 0]),   # Flexion axis
            'PIP_Flex': np.array([1, 0, 0]),   # PIP flexion axis
            'DIP_Flex': np.array([1, 0, 0])    # DIP flexion axis
        }
        return axis_map.get(joint_type, np.array([1, 0, 0]))
    
    def angle_around_axis(self, rotation_matrix, axis):
        """
        Extract the rotation angle around a specific axis
        
        This is the mathematically correct way to get joint angles
        """
        # Normalize the axis
        axis = axis / np.linalg.norm(axis)
        
        # Extract the rotation angle around the axis
        # This uses the Rodrigues' rotation formula
        trace = np.trace(rotation_matrix)
        angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))
        
        # Determine the sign of the rotation
        cross_product = np.cross(axis, rotation_matrix @ axis)
        if np.dot(cross_product, axis) < 0:
            angle = -angle
            
        return angle
    
    def apply_joint_limits(self, joint_angles, finger_name):
        """Apply anatomical joint limits"""
        limits = self.joint_limits[finger_name]
        
        for joint, angle in joint_angles.items():
            if joint in limits:
                joint_angles[joint] = np.clip(angle, -limits[joint], limits[joint])
        
        return joint_angles

def demonstrate_proper_extraction():
    """
    Demonstrate the proper way to extract joint angles
    """
    print("=" * 80)
    print("PROPER JOINT ANGLE EXTRACTION")
    print("=" * 80)
    
    extractor = ProperHandJointExtractor()
    
    # Simulate some bone rotations (in practice, these come from hand tracking)
    # These would be the actual quaternions from the hand tracking system
    bone_rotations = [
        R.from_quat([0, 0, 0, 1]),      # Base bone
        R.from_quat([0.1, 0, 0, 0.995]), # Proximal bone (MCP joint)
        R.from_quat([0.2, 0, 0, 0.98]),  # Middle bone (PIP joint)
        R.from_quat([0.3, 0, 0, 0.96])   # Distal bone (DIP joint)
    ]
    
    # Extract joint angles properly
    joint_angles = extractor.extract_joint_angles_proper(bone_rotations)
    
    if joint_angles:
        print("✓ Properly extracted joint angles:")
        for joint, angle in joint_angles.items():
            print(f"  {joint}: {angle:.2f}°")
        
        # Apply joint limits
        limited_angles = extractor.apply_joint_limits(joint_angles, 'Index')
        print("\n✓ After applying joint limits:")
        for joint, angle in limited_angles.items():
            print(f"  {joint}: {angle:.2f}°")
    else:
        print("✗ Failed to extract joint angles")

if __name__ == "__main__":
    demonstrate_proper_extraction()
