using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class HandController : MonoBehaviour
{
    private bool use_robot_hand = false;
    private string finger_angles_message = "";
    private bool allowMove;
    
    // OVRSkeleton reference for getting bone positions
    private OVRSkeleton rightHandSkeleton;
    
    void Start()
    {
        // Find the right hand skeleton component
        GameObject rightHand = GameObject.Find("OVRHandPrefab_Right");
        if (rightHand != null)
        {
            rightHandSkeleton = rightHand.GetComponent<OVRSkeleton>();
            if (rightHandSkeleton == null)
            {
                Debug.LogError("❌ Could not find OVRSkeleton component on right hand!");
            }
            else
            {
                Debug.Log("✓ Found OVRSkeleton for right hand");
            }
        }
        else
        {
            Debug.LogError("❌ Could not find OVRHandPrefab_Right object!");
        }
        
        allowMove = true;
    }

    public string GetFingerGoalMessage()
    {
        if (use_robot_hand)
        {
            return finger_angles_message;
        }
        else
        {
            return "0.00000\t"; // Default value
        }
    }

    // Calculate angle between three points (forming a joint)
    // Returns angle in degrees
    float CalculateJointAngle(Vector3 point1, Vector3 joint, Vector3 point3)
    {
        // Vectors from joint to point1 and point3
        Vector3 vector1 = point1 - joint;
        Vector3 vector2 = point3 - joint;
        
        // Normalize vectors
        vector1.Normalize();
        vector2.Normalize();
        
        // Calculate angle using dot product
        float dot = Vector3.Dot(vector1, vector2);
        dot = Mathf.Clamp(dot, -1f, 1f); // Clamp to avoid numerical errors
        
        // Angle in radians, then convert to degrees
        float angleRad = Mathf.Acos(dot);
        float angleDeg = angleRad * Mathf.Rad2Deg;
        
        // The angle we want is the BEND angle (180 - angle between vectors)
        // When finger is straight, vectors point in opposite directions (180°)
        // When finger is bent, angle decreases
        return 180f - angleDeg;
    }

    // Calculate signed angle for abduction (side-to-side movement)
    float CalculateAbductionAngle(Vector3 fingerBase, Vector3 fingerTip, Vector3 handForward, Vector3 handUp)
    {
        // Project finger direction onto the plane perpendicular to hand forward
        Vector3 fingerDir = (fingerTip - fingerBase).normalized;
        Vector3 projectedDir = Vector3.ProjectOnPlane(fingerDir, handForward);
        
        // Calculate angle from hand up direction
        float angle = Vector3.SignedAngle(handUp, projectedDir, handForward);
        
        return angle;
    }

    float[] ProcessHandFrameFromPositions()
    {
        float[] leapAngles = new float[16]; //  16 joints total for LEAP hand
        
        if (rightHandSkeleton == null || rightHandSkeleton.Bones == null)
        {
            Debug.LogWarning("⚠ Skeleton or bones not available");
            return leapAngles;
        }

        var bones = rightHandSkeleton.Bones;
        
        // OVRSkeleton bone indices:
        // Wrist = 0
        // Index: 5(metacarpal), 6(proximal), 7(intermediate), 8(distal)
        // Middle: 9(metacarpal), 10(proximal), 11(intermediate), 12(distal)
        // Ring: 13(metacarpal), 14(proximal), 15(intermediate), 16(distal)
        // Thumb: 1(metacarpal), 2(proximal), 3(intermediate), 4(distal)
        
        // Get wrist position for reference
        Vector3 wristPos = bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.position;
        Vector3 wristForward = bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.forward;
        Vector3 wristUp = bones[(int)OVRSkeleton.BoneId.Hand_WristRoot].Transform.up;
        
        int leapIndex = 0;
        
        // Process Index finger
        leapIndex = ProcessFinger(bones, OVRSkeleton.BoneId.Hand_Index1, OVRSkeleton.BoneId.Hand_Index2, 
                                   OVRSkeleton.BoneId.Hand_Index3, OVRSkeleton.BoneId.Hand_IndexTip,
                                   wristPos, wristForward, wristUp, leapAngles, leapIndex, "Index");
        
        // Process Middle finger
        leapIndex = ProcessFinger(bones, OVRSkeleton.BoneId.Hand_Middle1, OVRSkeleton.BoneId.Hand_Middle2,
                                   OVRSkeleton.BoneId.Hand_Middle3, OVRSkeleton.BoneId.Hand_MiddleTip,
                                   wristPos, wristForward, wristUp, leapAngles, leapIndex, "Middle");
        
        // Process Ring finger
        leapIndex = ProcessFinger(bones, OVRSkeleton.BoneId.Hand_Ring1, OVRSkeleton.BoneId.Hand_Ring2,
                                   OVRSkeleton.BoneId.Hand_Ring3, OVRSkeleton.BoneId.Hand_RingTip,
                                   wristPos, wristForward, wristUp, leapAngles, leapIndex, "Ring");
        
        // Process Thumb
        leapIndex = ProcessFinger(bones, OVRSkeleton.BoneId.Hand_Thumb1, OVRSkeleton.BoneId.Hand_Thumb2,
                                   OVRSkeleton.BoneId.Hand_Thumb3, OVRSkeleton.BoneId.Hand_ThumbTip,
                                   wristPos, wristForward, wristUp, leapAngles, leapIndex, "Thumb");
        
        return leapAngles;
    }

    int ProcessFinger(IList<OVRBone> bones, OVRSkeleton.BoneId base_id, OVRSkeleton.BoneId prox_id,
                      OVRSkeleton.BoneId mid_id, OVRSkeleton.BoneId dist_id,
                      Vector3 wristPos, Vector3 wristForward, Vector3 wristUp,
                      float[] leapAngles, int startIndex, string fingerName)
    {
        try
        {
            // Get bone positions
            Vector3 basePos = bones[(int)base_id].Transform.position;
            Vector3 proxPos = bones[(int)prox_id].Transform.position;
            Vector3 midPos = bones[(int)mid_id].Transform.position;
            Vector3 distPos = bones[(int)dist_id].Transform.position;
            
            // Calculate MCP abduction (side-to-side)
            float mcpAbd = CalculateAbductionAngle(wristPos, proxPos, wristForward, wristUp);
            
            // Calculate MCP flexion (bend at knuckle)
            float mcpFlex = CalculateJointAngle(basePos, proxPos, midPos);
            
            // Calculate PIP flexion
            float pipFlex = CalculateJointAngle(proxPos, midPos, distPos);
            
            // Calculate DIP flexion
            // For DIP, we use the tip position as the "next" bone
            Vector3 tipDir = (distPos - midPos).normalized;
            Vector3 extendedTip = distPos + tipDir * 0.02f; // Extend slightly for calculation
            float dipFlex = CalculateJointAngle(midPos, distPos, extendedTip);
            
            // Apply limits (LEAP hand limits)
            mcpAbd = Mathf.Clamp(mcpAbd, -30f, 30f);
            mcpFlex = Mathf.Clamp(mcpFlex, 0f, 90f);
            pipFlex = Mathf.Clamp(pipFlex, 0f, 110f);
            dipFlex = Mathf.Clamp(dipFlex, 0f, 80f);
            
            // Store in leap angles array
            leapAngles[startIndex] = mcpAbd;
            leapAngles[startIndex + 1] = mcpFlex;
            leapAngles[startIndex + 2] = pipFlex;
            leapAngles[startIndex + 3] = dipFlex;
            
            Debug.Log("✓ " + fingerName + " angles: MCP_Abd=" + mcpAbd.ToString("F1") + "°, MCP_Flex=" + mcpFlex.ToString("F1") + "°, PIP=" + pipFlex.ToString("F1") + "°, DIP=" + dipFlex.ToString("F1") + "°");
            
            return startIndex + 4;
        }
        catch (Exception e)
        {
            Debug.LogError("❌ Error processing " + fingerName + ": " + e.Message);
            return startIndex + 4;
        }
    }

    void Update()
    {
        if (allowMove)
        {
            // Check if hand tracking is enabled
            if (OVRPlugin.GetHandTrackingEnabled())
            {
                // Check if skeleton is initialized
                if (rightHandSkeleton != null && rightHandSkeleton.IsInitialized && rightHandSkeleton.IsDataValid)
                {
                    use_robot_hand = true;
                    
                    // Calculate angles from bone positions
                    float[] leapAngles = ProcessHandFrameFromPositions();
                    finger_angles_message = "";
                    
                    // Convert to string format for transmission
                    for (int i = 0; i < leapAngles.Length; i++)
                    {
                        finger_angles_message += leapAngles[i].ToString("F5") + '\t';
                    }
                }
                else
                {
                    use_robot_hand = false;
                    if (rightHandSkeleton == null)
                    {
                        Debug.LogWarning("⚠ Right hand skeleton is null");
                    }
                    else if (!rightHandSkeleton.IsInitialized)
                    {
                        Debug.LogWarning("⚠ Right hand skeleton not initialized yet");
                    }
                    else if (!rightHandSkeleton.IsDataValid)
                    {
                        Debug.LogWarning("⚠ Right hand skeleton data not valid");
                    }
                }
            }
            else
            {
                use_robot_hand = false;
            }
        }
    }
}

