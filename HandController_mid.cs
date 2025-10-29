using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;

public class HandController : MonoBehaviour
{
    private bool use_robot_hand = false;
    private string finger_angles_message = "";
    private bool allowMove;
    
    // New quaternion-to-angle conversion system
    private bool calibration_done = false;
    private Dictionary<string, Quaternion> neutral_pose_rotations = new Dictionary<string, Quaternion>();
    private Dictionary<string, Vector3> joint_axes = new Dictionary<string, Vector3>();
    private Dictionary<string, float[]> joint_limits = new Dictionary<string, float[]>();
    private Rigidbody hand;
    private Transform fingerL;
    private Transform fingerR;
    private Transform fingerL_goal;
    private Transform fingerR_goal;
    private float finger_goal = 0;
    public float speed = 0.1f;
    private List<string> unknown_objects = new List<string>();
    public Vector3 virtual_walls_max = new Vector3(0.25f, 0.9f, 0.25f);
    public Vector3 virtual_walls_min = new Vector3(-0.25f, 0f, -0.25f);
    public Vector3 camera_position = new Vector3(0f, 0.35f, -0.5f);
    public Vector3 RotationCorrectionEuler = new Vector3(200f, 180f, 0f);
    private Quaternion rotation_correction;
    private Quaternion hand_rot_correction = Quaternion.Euler(160, 90, 0);
    private Vector3 hand_offset = new Vector3(0f, 0.1034f, 0f);
    private Vector3 hand_detect_offset = new Vector3(0f, 0.07f, 0.05f);
    private Vector3 goal_position = new Vector3(0f, 0.48f, -0.24f);
    private Vector3 goal_position_new;
    private Quaternion goal_rotation = Quaternion.Euler(180, 180, 0);
    private Quaternion goal_rotation_new;
    private Vector3[] bounding_points_hand = { new Vector3 { x = -0.1f, y = -0.1125f,  z = -0.025f}, //A
                                               new Vector3 { x = 0.1f,  y = -0.1125f,  z = -0.025f},  //B
                                               new Vector3 { x = 0.1f,  y = -0.1125f,  z = 0.025f}, //C
                                               new Vector3 { x = -0.1f, y = -0.1125f,  z = 0.025f}, //D
                                               new Vector3 { x = -0.1f, y = -0.0375f,  z = -0.02f},   //E
                                               new Vector3 { x = 0.1f,  y = -0.0375f,  z = -0.02f},  //F
                                               new Vector3 { x = 0.1f,  y = -0.0375f,  z = 0.02f},  //G
                                               new Vector3 { x = -0.1f, y = -0.0375f,  z = 0.02f}};  //H

    private Vector3[] bounding_points_finger = {new Vector3 { x = -1f, y = 0.01f, z = -0.01f}, //I
                                                new Vector3 { x = 1f,  y = 0.01f, z = -0.01f},  //J
                                                new Vector3 { x = 1f,  y = 0.01f, z = 0.01f}, //K
                                                new Vector3 { x = -1f, y = 0.01f, z = 0.01f}};  //L

    private Color controller_color = new Color(1f, 1f, 1f, 0.2f);

    // start is called before the first frame update
    void Start()
    {
        hand = GameObject.Find("python_hand").GetComponent<Rigidbody>();
        fingerR = GameObject.Find("fingerR").GetComponent<Transform>();
        fingerL = GameObject.Find("fingerL").GetComponent<Transform>();
        fingerL_goal = GameObject.Find("fingerL_goal").GetComponent<Transform>();
        fingerR_goal = GameObject.Find("fingerR_goal").GetComponent<Transform>();
        allowMove = false;
        GameObject.Find("node5").GetComponent<Renderer>().material.SetColor("_Color", Color.red);
        rotation_correction = Quaternion.Euler(RotationCorrectionEuler);
        
        // Initialize joint limits (in degrees)
        InitializeJointLimits();
}

    // Initialize joint limits for LEAP hand (in degrees)
    void InitializeJointLimits()
    {
        // LEAP hand joint limits based on leap_hand_utils.py
        // Index, Middle, Ring fingers: [MCP Side, MCP Forward, PIP, DIP]
        // Thumb: [MCP Side, MCP Forward, PIP, DIP]
        
        joint_limits["Index"] = new float[] { 20f, 90f, 90f, 90f };    // MCP Side, MCP Forward, PIP, DIP
        joint_limits["Middle"] = new float[] { 20f, 90f, 90f, 90f };
        joint_limits["Ring"] = new float[] { 20f, 90f, 90f, 90f };
        joint_limits["Thumb"] = new float[] { 60f, 90f, 90f, 90f };
    }

    // Quaternion utility functions
    Quaternion NormalizeQuat(Quaternion q)
    {
        float magnitude = Mathf.Sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (magnitude > 0.0001f)
        {
            return new Quaternion(q.x / magnitude, q.y / magnitude, q.z / magnitude, q.w / magnitude);
        }
        return Quaternion.identity;
    }

    Quaternion InverseQuat(Quaternion q)
    {
        return new Quaternion(-q.x, -q.y, -q.z, q.w);
    }

    Quaternion MulQuat(Quaternion a, Quaternion b)
    {
        return new Quaternion(
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        );
    }

    Vector3 AxisAngleFromQuat(Quaternion q, out float angle)
    {
        q = NormalizeQuat(q);
        angle = 2 * Mathf.Acos(Mathf.Clamp(Mathf.Abs(q.w), 0f, 1f));
        
        if (angle < 0.001f)
        {
            angle = 0f;
            return Vector3.up; // Default axis
        }
        
        float denom = Mathf.Sin(angle / 2);
        Vector3 axis = new Vector3(q.x / denom, q.y / denom, q.z / denom);
        return axis.normalized;
    }

    // Swing-twist decomposition functions
    Quaternion TwistOnly(Quaternion q_rel, Vector3 axisLocal)
    {
        Vector3 v = new Vector3(q_rel.x, q_rel.y, q_rel.z);
        Vector3 proj = Vector3.Dot(v, axisLocal) * axisLocal;
        return NormalizeQuat(new Quaternion(proj.x, proj.y, proj.z, q_rel.w));
    }

    float TwistAngleAroundAxis(Quaternion q_rel, Vector3 axisLocal)
    {
        Quaternion twist = TwistOnly(q_rel, axisLocal);
        float angle = 2 * Mathf.Acos(Mathf.Clamp(Mathf.Abs(twist.w), 0f, 1f));
        Vector3 twistAxis = new Vector3(twist.x, twist.y, twist.z);
        float sign = Mathf.Sign(Vector3.Dot(twistAxis, axisLocal));
        return angle * sign;
    }

    Quaternion RemoveTwist(Quaternion q_rel, Vector3 axisLocal)
    {
        return MulQuat(q_rel, InverseQuat(TwistOnly(q_rel, axisLocal)));
    }

    // Calibration function - call this when hand is in neutral pose
    void CalibrateNeutralPose(OVRPlugin.HandState handState)
    {
        if (handState.BoneRotations.Length < 19) return;

        // Store neutral pose rotations for each bone
        for (int i = 0; i < handState.BoneRotations.Length; i++)
        {
            string boneName = GetBoneName(i);
            if (boneName != "")
            {
                Quaternion boneRot = new Quaternion(
                    handState.BoneRotations[i].x,
                    handState.BoneRotations[i].y,
                    handState.BoneRotations[i].z,
                    handState.BoneRotations[i].w
                );
                neutral_pose_rotations[boneName] = boneRot;
            }
        }

        // Define joint axes in neutral pose
        DefineJointAxes();
        calibration_done = true;
        Debug.Log("Calibration completed!");
    }

    string GetBoneName(int boneIndex)
    {
        // Oculus hand tracking bone indices
        switch (boneIndex)
        {
            case 0: return "Wrist";
            case 1: return "Thumb_0"; case 2: return "Thumb_1"; case 3: return "Thumb_2"; case 4: return "Thumb_3";
            case 5: return "Index_0"; case 6: return "Index_1"; case 7: return "Index_2"; case 8: return "Index_3";
            case 9: return "Middle_0"; case 10: return "Middle_1"; case 11: return "Middle_2"; case 12: return "Middle_3";
            case 13: return "Ring_0"; case 14: return "Ring_1"; case 15: return "Ring_2"; case 16: return "Ring_3";
            case 17: return "Pinky_0"; case 18: return "Pinky_1"; case 19: return "Pinky_2"; case 20: return "Pinky_3";
            default: return "";
        }
    }

    void DefineJointAxes()
    {
        // Define joint axes in neutral pose
        // These axes are defined in the parent bone's local coordinate system
        
        // For each finger, define the axes for MCP, PIP, DIP joints
        string[] fingers = { "Index", "Middle", "Ring", "Pinky", "Thumb" };
        
        foreach (string finger in fingers)
        {
            // MCP axes (2-DOF joint)
            joint_axes[finger + "_MCP_ABD"] = Vector3.right;   // Abduction/adduction axis in parent-local
            joint_axes[finger + "_MCP_FLEX"] = Vector3.forward; // Flexion/extension axis in parent-local
            
            // PIP and DIP axes (1-DOF hinge joints)
            // Use parent-local forward for flexion/extension (curling)
            joint_axes[finger + "_PIP_FLEX"] = Vector3.forward;
            joint_axes[finger + "_DIP_FLEX"] = Vector3.forward;
        }
    }

    // Simplified angle extraction pipeline (no calibration needed)
    float[] ProcessHandFrameSimple(OVRPlugin.HandState handState)
    {
        float[] leapAngles = new float[16]; // 16 joints total for LEAP hand
        int leapIndex = 0;

        // Process each finger: Index, Middle, Ring, Thumb
        string[] fingers = { "Index", "Middle", "Ring", "Thumb" };
        int[] fingerBoneOffsets = { 5, 9, 13, 1 }; // Starting bone indices for each finger

        for (int f = 0; f < fingers.Length; f++)
        {
            string finger = fingers[f];
            int baseBone = fingerBoneOffsets[f];
            
            // Get bone rotations
            Quaternion baseRot = GetBoneRotation(handState, baseBone);
            Quaternion proxRot = GetBoneRotation(handState, baseBone + 1);
            Quaternion midRot = GetBoneRotation(handState, baseBone + 2);
            Quaternion distRot = GetBoneRotation(handState, baseBone + 3);

            // Simple angle extraction using relative rotations
            Quaternion q_MCP = MulQuat(InverseQuat(baseRot), proxRot);
            Quaternion q_PIP = MulQuat(InverseQuat(proxRot), midRot);
            Quaternion q_DIP = MulQuat(InverseQuat(midRot), distRot);

            // Extract angles using proper quaternion decomposition
            float mcpAbdRad, mcpFlexRad, pipFlexRad, dipFlexRad;
            
            // MCP angles (2-DOF) - use swing-twist decomposition for proper separation
            Vector3 abductionAxis = Vector3.up; // Y-axis for side-to-side movement
            Vector3 flexionAxis = Vector3.right; // X-axis for forward-backward movement
            
            // For MCP abduction, try multiple axes to find the best one
            Vector3[] mcpAbdAxes = { Vector3.up, Vector3.forward, -Vector3.up, -Vector3.forward, Vector3.right, -Vector3.right };
            float maxMCPAbdScore = 0f;
            Vector3 bestMCPAbdAxis = abductionAxis;
            
            foreach (Vector3 axis in mcpAbdAxes)
            {
                float angle = TwistAngleAroundAxis(q_MCP, axis);
                float absAngle = Mathf.Abs(angle);
                
                // Score this axis (prefer reasonable ranges)
                float score = 0f;
                if (absAngle > 0.1f && absAngle < 1.5f) // Reasonable range (0.1° to 86°)
                {
                    score = absAngle;
                    // Bonus for both positive and negative angles (abduction can go both ways)
                    if (Mathf.Abs(angle) > 0.2f) score *= 1.1f;
                }
                
                if (score > maxMCPAbdScore)
                {
                    maxMCPAbdScore = score;
                    bestMCPAbdAxis = axis;
                }
            }
            
            // Use the best axis for MCP abduction
            mcpAbdRad = TwistAngleAroundAxis(q_MCP, bestMCPAbdAxis);
            
            // For MCP flexion, try multiple axes to find the best one
            Vector3[] mcpFlexAxes = { Vector3.right, Vector3.forward, -Vector3.right, -Vector3.forward, Vector3.up, -Vector3.up };
            float maxMCPFlexScore = 0f;
            Vector3 bestMCPFlexAxis = flexionAxis;
            
            foreach (Vector3 axis in mcpFlexAxes)
            {
                // Remove abduction first, then test flexion axis
                Quaternion noAbd = RemoveTwist(q_MCP, abductionAxis);
                float angle = TwistAngleAroundAxis(noAbd, axis);
                float absAngle = Mathf.Abs(angle);
                
                // Score this axis (prefer positive angles and reasonable ranges)
                float score = 0f;
                if (absAngle > 0.1f && absAngle < 2.0f) // Reasonable range (0.1° to 115°)
                {
                    score = absAngle;
                    if (angle > 0) score *= 1.2f; // Bonus for positive angles
                }
                
                if (score > maxMCPFlexScore)
                {
                    maxMCPFlexScore = score;
                    bestMCPFlexAxis = axis;
                }
            }
            
            // Use the best axis for MCP flexion
            Quaternion noAbdBest = RemoveTwist(q_MCP, abductionAxis);
            mcpFlexRad = TwistAngleAroundAxis(noAbdBest, bestMCPFlexAxis);
            
            // PIP and DIP angles (1-DOF) - try multiple axes to find the right one
            // Try different axes for better DIP detection
            Vector3[] testAxes = { Vector3.right, Vector3.up, Vector3.forward, -Vector3.right, -Vector3.up, -Vector3.forward };
            
            // Smart axis selection for PIP - look for realistic joint movement
            Vector3 bestPIPAxis = flexionAxis;
            float bestPIPScore = -1f;
            foreach (Vector3 axis in testAxes)
            {
                float angle = TwistAngleAroundAxis(q_PIP, axis);
                float absAngle = Mathf.Abs(angle);
                
                // Score based on: 1) reasonable angle range, 2) consistent with joint limits
                float score = 0f;
                if (absAngle > 0.1f && absAngle < 2.0f) // Reasonable range (0.1° to 115°)
                {
                    score = absAngle;
                    // Bonus for positive angles (flexion is typically positive)
                    if (angle > 0) score *= 1.2f;
                }
                
                if (score > bestPIPScore)
                {
                    bestPIPScore = score;
                    bestPIPAxis = axis;
                }
            }
            
            // Smart axis selection for DIP - similar logic but different scoring
            Vector3 bestDIPAxis = flexionAxis;
            float bestDIPScore = -1f;
            foreach (Vector3 axis in testAxes)
            {
                float angle = TwistAngleAroundAxis(q_DIP, axis);
                float absAngle = Mathf.Abs(angle);
                
                // Score based on: 1) reasonable angle range, 2) consistent with joint limits
                float score = 0f;
                if (absAngle > 0.1f && absAngle < 2.0f) // Reasonable range (0.1° to 115°)
                {
                    score = absAngle;
                    // Bonus for positive angles (flexion is typically positive)
                    if (angle > 0) score *= 1.2f;
                }
                
                if (score > bestDIPScore)
                {
                    bestDIPScore = score;
                    bestDIPAxis = axis;
                }
            }
            
            // Use the best axes found, with fallback to default if no good axis found
            if (bestPIPScore > 0.05f) // Only use if we found a reasonable axis
                pipFlexRad = TwistAngleAroundAxis(q_PIP, bestPIPAxis);
            else
                pipFlexRad = 0f; // No good axis found, set to 0
                
            if (bestDIPScore > 0.05f) // Only use if we found a reasonable axis
                dipFlexRad = TwistAngleAroundAxis(q_DIP, bestDIPAxis);
            else
                dipFlexRad = 0f; // No good axis found, set to 0

            // Convert to degrees and clamp to limits
            float[] limits = joint_limits[finger];
            
            // Convert to degrees
            float mcpAbdDegRaw = mcpAbdRad * Mathf.Rad2Deg;
            float mcpFlexDegRaw = mcpFlexRad * Mathf.Rad2Deg;
            float pipFlexDegRaw = pipFlexRad * Mathf.Rad2Deg;
            float dipFlexDegRaw = dipFlexRad * Mathf.Rad2Deg;
            
            // Clamp to limits
            float mcpAbdDeg = Mathf.Clamp(mcpAbdDegRaw, -limits[0], limits[0]);
            float mcpFlexDeg = Mathf.Clamp(mcpFlexDegRaw, 0, limits[1]);
            float pipFlexDeg = Mathf.Clamp(pipFlexDegRaw, 0, limits[2]);
            float dipFlexDeg = Mathf.Clamp(dipFlexDegRaw, 0, limits[3]);
            
            // Debug output for first finger only
            if (finger == "Index")
            {
                Debug.Log($"Raw angles - MCP_Abd: {mcpAbdDegRaw:F2}°, MCP_Flex: {mcpFlexDegRaw:F2}°, PIP: {pipFlexDegRaw:F2}°, DIP: {dipFlexDegRaw:F2}°");
                Debug.Log($"Best axes - MCP_Abd: {bestMCPAbdAxis} (score: {maxMCPAbdScore:F3}), MCP_Flex: {bestMCPFlexAxis} (score: {maxMCPFlexScore:F3}), PIP: {bestPIPAxis} (score: {bestPIPScore:F3}), DIP: {bestDIPAxis} (score: {bestDIPScore:F3})");
            }

            // Store in LEAP order: [DIP, PIP, MCP_flex, MCP_abd]
            leapAngles[leapIndex++] = dipFlexDeg;
            leapAngles[leapIndex++] = pipFlexDeg;
            leapAngles[leapIndex++] = mcpFlexDeg;
            leapAngles[leapIndex++] = mcpAbdDeg;
        }

        return leapAngles;
    }

    // Main angle extraction pipeline
    float[] ProcessHandFrame(OVRPlugin.HandState handState)
    {
        if (!calibration_done || handState.BoneRotations.Length < 19)
        {
            return new float[16]; // Return zeros if not calibrated or insufficient data
        }

        float[] leapAngles = new float[16]; // 16 joints total for LEAP hand
        int leapIndex = 0;

        // Process each finger: Index, Middle, Ring, Thumb (skip Pinky for now as LEAP has 4 fingers)
        string[] fingers = { "Index", "Middle", "Ring", "Thumb" };
        int[] fingerBoneOffsets = { 5, 9, 13, 1 }; // Starting bone indices for each finger

        for (int f = 0; f < fingers.Length; f++)
        {
            string finger = fingers[f];
            int baseBone = fingerBoneOffsets[f];
            
            // Get bone rotations
            Quaternion baseRot = GetBoneRotation(handState, baseBone);
            Quaternion proxRot = GetBoneRotation(handState, baseBone + 1);
            Quaternion midRot = GetBoneRotation(handState, baseBone + 2);
            Quaternion distRot = GetBoneRotation(handState, baseBone + 3);

            // Step 1: Compute relative quaternions
            Quaternion q_MCP = MulQuat(InverseQuat(baseRot), proxRot);
            Quaternion q_PIP = MulQuat(InverseQuat(proxRot), midRot);
            Quaternion q_DIP = MulQuat(InverseQuat(midRot), distRot);

            // Step 2: Extract angles
            // MCP is 2-DOF (abduction + flexion)
            Vector3 mcpAbdAxis = joint_axes[finger + "_MCP_ABD"];
            Vector3 mcpFlexAxis = joint_axes[finger + "_MCP_FLEX"];
            
            float mcpAbdRad = TwistAngleAroundAxis(q_MCP, mcpAbdAxis);
            Quaternion noAbd = RemoveTwist(q_MCP, mcpAbdAxis);
            float mcpFlexRad = TwistAngleAroundAxis(noAbd, mcpFlexAxis);

            // PIP and DIP are 1-DOF hinge joints
            Vector3 pipAxis = joint_axes[finger + "_PIP_FLEX"];
            Vector3 dipAxis = joint_axes[finger + "_DIP_FLEX"];
            
            float pipFlexRad = TwistAngleAroundAxis(q_PIP, pipAxis);
            float dipFlexRad = TwistAngleAroundAxis(q_DIP, dipAxis);

            // Convert to degrees and clamp to limits
            float[] limits = joint_limits[finger];
            
            float mcpAbdDeg = Mathf.Clamp(mcpAbdRad * Mathf.Rad2Deg, -limits[0], limits[0]);
            float mcpFlexDeg = Mathf.Clamp(mcpFlexRad * Mathf.Rad2Deg, 0, limits[1]);
            float pipFlexDeg = Mathf.Clamp(pipFlexRad * Mathf.Rad2Deg, 0, limits[2]);
            float dipFlexDeg = Mathf.Clamp(dipFlexRad * Mathf.Rad2Deg, 0, limits[3]);

            // Store in LEAP order: [DIP, PIP, MCP_flex, MCP_abd]
            leapAngles[leapIndex++] = dipFlexDeg;
            leapAngles[leapIndex++] = pipFlexDeg;
            leapAngles[leapIndex++] = mcpFlexDeg;
            leapAngles[leapIndex++] = mcpAbdDeg;
        }

        return leapAngles;
    }

    Quaternion GetBoneRotation(OVRPlugin.HandState handState, int boneIndex)
    {
        if (boneIndex >= 0 && boneIndex < handState.BoneRotations.Length)
        {
            return new Quaternion(
                handState.BoneRotations[boneIndex].x,
                handState.BoneRotations[boneIndex].y,
                handState.BoneRotations[boneIndex].z,
                handState.BoneRotations[boneIndex].w
            );
        }
        return Quaternion.identity;
    }

    public string GetFingerGoalMessage()
    {
        if (use_robot_hand)
        {
            return finger_angles_message;
        }
        else
        {
            return finger_goal.ToString("F5") + '\t'; //add tab to make everything consistent.
    }
        }

    public Vector3 GetGoalPosition()
    {
        return goal_position;
    }

    public Quaternion GetGoalRotation()
    {
        return goal_rotation;
    }

    public void MoveHand(string command_txt)
    {
        // Convert the movehand command text into a list of floats
        string[] split_input = command_txt.Split('\t');
        List<float> pose_float = new List<float>(3);
        List<float> rot_float = new List<float>(3);
        float finger_float = float.Parse(split_input[3]);
        foreach (string item in split_input[1].Split(','))
        {
            pose_float.Add(float.Parse(item));
        }
        foreach (string item in split_input[2].Split(','))
        {
            rot_float.Add(float.Parse(item));
        }
        //Move the hand and two fingers to their respective positions(from the command text).
        hand.position = new Vector3(pose_float[0], pose_float[1], pose_float[2]);
        hand.rotation = new Quaternion(rot_float[0], rot_float[1], rot_float[2], rot_float[3]);
        fingerR.localPosition = new Vector3(finger_float / 2, -0.0454f, 0f);
        fingerL.localPosition = new Vector3(-finger_float / 2, -0.0454f, 0f);

    }

    // Move the hand based on the recorded goal positions. This bypasses the python side.
    public void MoveHandSelf()
    {
        hand.position = goal_position;
        hand.rotation = goal_rotation;
    }

    // update is called once per frame
    void Update()
    {
        //Use button one (the A button) to toggle allow Move.
        //When allowMove == False, the controller position stops being tracked, allow the user to freely move
        //their hand. To show this, the hand turns red when allowMove is false.
        if (OVRInput.GetDown(OVRInput.Button.One))
        {
            if (allowMove)
            {
                allowMove = false;
                GameObject.Find("node5").GetComponent<Renderer>().material.SetColor("_Color", Color.red);
            }
            else
            {
                allowMove = true;
                GameObject.Find("node5").GetComponent<Renderer>().material.SetColor("_Color", Color.white);
            }
        }

        // Use button two (the B button) to calibrate neutral pose
        if (OVRInput.GetDown(OVRInput.Button.Two))
        {
            OVRPlugin.HandState handState = default(OVRPlugin.HandState);
            if (OVRPlugin.GetHandTrackingEnabled() && OVRPlugin.GetHandState(OVRPlugin.Step.Render, OVRPlugin.Hand.HandRight, ref handState))
            {
                CalibrateNeutralPose(handState);
            }
        }

        // Update the goal position to send over the socket
        if (allowMove)
        {
            Vector3 controller_position;
            Quaternion controller_rotation;
            controller_rotation = OVRInput.GetLocalControllerRotation(OVRInput.Controller.RTouch);
            controller_position = OVRInput.GetLocalControllerPosition(OVRInput.Controller.RTouch);

            if (OVRPlugin.GetHandTrackingEnabled())
            {
                goal_rotation_new = controller_rotation * hand_rot_correction;
                goal_position_new = controller_position + camera_position + goal_rotation * hand_detect_offset;
            }
            else
            {
                goal_rotation_new = controller_rotation * rotation_correction;
                goal_position_new = controller_position + camera_position + goal_rotation * hand_offset;
            }

            OVRPlugin.HandState handState = default(OVRPlugin.HandState);

            if (OVRPlugin.GetHandTrackingEnabled() && OVRPlugin.GetHandState(OVRPlugin.Step.Render, OVRPlugin.Hand.HandRight, ref handState) && handState.BoneRotations.Length > 18)
            {
                use_robot_hand = true;
                
                // Use simplified angle extraction (working version)
                float[] leapAngles = ProcessHandFrameSimple(handState);
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
            }

            //Comment out to lock the hand to pointing down
            //goal_rotation = Quaternion.Euler(180, 180, 0);

            // keep the hand position within the virtual walls
            bool virtual_wall_collision = false;
            Debug.Log(hand.rotation * (new Vector3(0.10f, -0.1125f, -0.025f)) + hand.position);

            Vector3 finger_scaling = new Vector3(0.015f + Mathf.Max(finger_goal, fingerR.localPosition[0]), 1f, 1f);
            List<Vector3> bounding_points = new List<Vector3>();
            bounding_points.AddRange(bounding_points_hand);
            foreach (Vector3 finger_point in bounding_points_finger)
            {
                bounding_points.Add(Vector3.Scale(finger_point, finger_scaling));
            }
            foreach (Vector3 displacement in bounding_points)
            {
                Vector3 point = goal_rotation_new * displacement + goal_position_new;
                for (int i = 0; i < 3; i++)
                {
                    if (point[i] > virtual_walls_max[i] || point[i] < virtual_walls_min[i])
                    {
                        virtual_wall_collision = true;
                    }
                }
            }
            // if in collision with the virtual walls, vibrate the controllers.
            if (virtual_wall_collision)
            {
                OVRInput.SetControllerVibration(1f, 10f, OVRInput.Controller.RTouch);
            }
            else
            {
                OVRInput.SetControllerVibration(0, 0, OVRInput.Controller.RTouch);
                goal_position = goal_position_new;
                goal_rotation = goal_rotation_new;
            }

            controller_color.a = Mathf.Clamp((goal_position_new - hand.position).magnitude / 0.15f, 0f, 1f);
            GameObject.Find("r_rainier_mesh").GetComponent<Renderer>().material.SetColor("_Color", controller_color);

            // Move the goal finger position based on the thumbstick input
            Vector2 ThumbstickInput = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick);
            finger_goal += speed * ThumbstickInput[1] * Time.deltaTime;
            finger_goal = Mathf.Clamp(finger_goal, 0f, 0.04f);
            fingerL_goal.localPosition = new Vector3(-finger_goal, -0.0454f, 0f);
            fingerR_goal.localPosition = new Vector3(finger_goal, -0.0454f, 0f);
        } 
        else
        {
            controller_color.a = 1f;
            GameObject.Find("r_rainier_mesh").GetComponent<Renderer>().material.SetColor("_Color", controller_color);
        }
    }
}
