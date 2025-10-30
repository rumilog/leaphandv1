import numpy as np
import time
import datetime
import os
import signal
import sys

import UdpComms as U

# PandaGym underwent a major update durring development of this project. 
# If you are using an older version of PandaGym, set OLD_PANDAGYM to True.
OLD_PANDAGYM = False

# Try to import PandaGym, but continue without it if not available
try:
    if OLD_PANDAGYM:
        from panda_gym.envs.panda_tasks.panda_stack import PandaStackEnv
    else:
        from panda_gym.envs.panda_tasks import PandaStackEnv
    PANDAGYM_AVAILABLE = True
    print("PandaGym loaded successfully")
except ImportError as e:
    PANDAGYM_AVAILABLE = False
    print(f"PandaGym not available: {e}")
    print("Continuing without PandaGym simulation...")

THIS_IP = "172.26.71.187" # IP adress of this machine
OCULUS_IP = "172.26.112.211" # IP adress of the Oculus Headset
RESET_AT_TASK_COMPLETION = False # Resets the enviroemnt when the task (stack) is completed.


def new_object_message(new_object_list, game_objects):
    message = ""
    for new_object in new_object_list:
        message += '_newItem\t' + game_objects[new_object]['type'] + '\t' + new_object + '\t' \
        + game_objects[new_object]["size"] + '\t' + game_objects[new_object]["color"] + '\n'
    return message

 
def object_message(object_name):
    if PANDAGYM_AVAILABLE:
        pos = env.sim.get_base_position(object_name)
        vel = env.sim.get_base_velocity(object_name)
        rot = env.sim.get_base_orientation(object_name)
        avel = env.sim.get_base_angular_velocity(object_name)
        return object_name + '\t' + str(-pos[1]) + ',' + str(pos[2]) + ',' + str(pos[0]) + '\t' \
        + str(-vel[1]) + ',' + str(vel[2]) + ',' + str(vel[0]) + '\t' \
        + str(rot[1]) + ',' + str(-rot[2]) + ',' + str(-rot[0]) + ',' + str(rot[3]) + '\t' \
        + str(avel[1]) + ',' + str(-avel[2]) + ',' + str(-avel[0])
    else:
        # Return dummy data when PandaGym is not available
        return object_name + '\t0,0,0\t0,0,0\t0,0,0,1\t0,0,0'


def generate_hand_message(hand_position, hand_rotation, finger_width):
    return '_hand\t' + str(-hand_position[1]) + ',' + str(hand_position[2]) + ',' + str(hand_position[0]) + '\t' \
     + str(hand_rotation[1]) + ',' + str(-hand_rotation[2]) + ',' + str(-hand_rotation[0]) + ',' + str(hand_rotation[3]) + '\t' + str(finger_width)

# Create UDP socket to use for sending (and receiving) Use THIS machines IP for udp.
sock = U.UdpComms(udpIP=THIS_IP, sendIP = OCULUS_IP, portTX=8000, portRX=8001, enableRX=True, suppressWarnings=False)

# Initialize environment if PandaGym is available
env = None
if PANDAGYM_AVAILABLE:
    if OLD_PANDAGYM:
        env = PandaStackEnv(render = True, control_type="joints") # task enviroment
    else:
        env = PandaStackEnv(render_mode = "human", control_type="joints") # task enviroment
    env.reset()
    position = env.robot.get_ee_position()
    gripper = env.robot.get_fingers_width()
else:
    # Use dummy values when PandaGym is not available
    position = np.array([0.0, 0.0, 0.0])
    gripper = 0.0

target_location = np.zeros(3)
target_location[:2] = np.random.uniform(-0.15, 0.15, 2)

if PANDAGYM_AVAILABLE:
    env.sim.set_base_pose("object1", np.array([0.1, 0.1, 0.02]), np.array([0.0, 0.0, 0.0, 1.0]))
    env.sim.set_base_pose("target1", np.array([0, 0, 0.02]) + target_location, np.array([0.0, 0.0, 0.0, 1.0]))
    env.sim.set_base_pose("object2", np.array([0.1, -0.1, 0.02]), np.array([0.0, 0.0, 0.0, 1.0]))
    env.sim.set_base_pose("target2", np.array([0, 0, 0.06]) + target_location, np.array([0.0, 0.0, 0.0, 1.0]))

game_objects = {"object1":{'type':'block', "size":"0.04,0.04,0.04", "color":"3,25,250,1"},
                "object2":{'type':'block', "size":"0.04,0.04,0.04", "color":"3,250,21,1"},
                "target1":{'type':'block', "size":"0.04,0.04,0.04", "color":"3,25,250,0.3"},
                "target2":{'type':'block', "size":"0.04,0.04,0.04", "color":"3,250,21,0.3"}}

new_object_list = []
inventory_list = []
message_index = 1
DELAY = 0.005 # Set communication delay with the oculus headset. Means that the oculus will communicate at a max of 200HZ
ENV_STEP_PERIOD = 0.04 
send_update_time = time.time() - DELAY
sim_update_time = time.time() - ENV_STEP_PERIOD

rotation = np.array([1, 0, 0, 0])
goal_joints = np.empty(9)
if PANDAGYM_AVAILABLE:
    for i in range(9):
        goal_joints[i] = env.robot.get_joint_angle(i)
else:
    goal_joints = np.zeros(9)

# Create data logging file
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
log_filename = f"hand_tracking_data_{timestamp}.txt"
log_file = open(log_filename, 'w')

print("="*60)
print("HAND TRACKING SERVER STARTED")
print("="*60)
if PANDAGYM_AVAILABLE:
    print("✓ PandaGym simulation enabled")
else:
    print("⚠ PandaGym simulation disabled - hand tracking only")
print(f"✓ UDP communication: {THIS_IP} <-> {OCULUS_IP}")
print(f"✓ Data logging to: {log_filename}")
print("✓ Ready to receive hand tracking data")
print("="*60)

# Write header to log file
log_file.write("HAND TRACKING DATA LOG\n")
log_file.write("="*60 + "\n")
log_file.write(f"Started: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
log_file.write("Format: [timestamp] [finger] [joint] [angle_degrees]\n")
log_file.write("="*60 + "\n\n")

# Signal handler to properly close log file on interruption
def signal_handler(sig, frame):
    print(f"\n\nInterrupted! Saving data to {log_filename}...")
    log_file.write(f"\nInterrupted: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    log_file.close()
    print(f"✓ Data saved to: {log_filename}")
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

while(True):
    if time.time() > send_update_time + DELAY:
        send_update_time += DELAY

        # Send an update to Unity
        if PANDAGYM_AVAILABLE:
            hand_position = env.robot.get_ee_position()
            hand_rot = rotation
            finger_width = env.robot.get_fingers_width()
        else:
            hand_position = position
            hand_rot = rotation
            finger_width = gripper
        send_string = str(message_index) + '\n'
        message_index += 1
        
        if PANDAGYM_AVAILABLE:
            for item in inventory_list:
                if not (item in game_objects.keys()):
                    send_string += "_deleteItem" + '\t' + item + '\n'
            for item in game_objects.keys():
                if not(item in inventory_list) and not(item in new_object_list):
                    new_object_list.append(item)
            if len(new_object_list) != 0:
                send_string += new_object_message(new_object_list, game_objects)
            for game_object in game_objects:
                send_string += object_message(game_object) + '\n'
        send_string += generate_hand_message(hand_position, hand_rot, finger_width)
        sock.SendData(send_string) # Send this string to other application

        data = sock.ReadReceivedData() # read data

        if data != None: # if NEW data has been received since last ReadReceivedData function call
            inventory, unknown_objects, hand_pose, gripper_message = data.split('\n')
            gripper_message = gripper_message[:-1] # remove unneeded tab
            inventory_list = inventory.split('\t')[1:]
            new_object_list = unknown_objects.split('\t')[1:]

            object_index = 0

            position_message, rotation_message = hand_pose.split('\t')
            position = np.array(position_message[1:-1].split(', ')).astype(np.float64)
            position = np.array([position[2], -position[0], position[1]])
            rotation = np.array(rotation_message[1:-1].split(', ')).astype(np.float64)
            rotation = np.array([-rotation[2], rotation[0], -rotation[1], rotation[3]])
            if gripper_message == "":
                continue

            # Handle old (5 values), pip/dip (10 values), and new (16 values) finger angle formats
            gripper_values = np.array(gripper_message.split('\t')).astype(np.float64)
            
            if len(gripper_values) == 5:
                # Old format: 5 values (one per finger)
                gripper = gripper_values[0]
                print(f"Old format - Gripper: {gripper:.3f}")
            elif len(gripper_values) == 10:
                # PIP/DIP format: 10 values (2 joints per finger: PIP, DIP for 5 fingers)
                finger_names = ["Thumb", "Index", "Middle", "Ring", "Pinky"]
                joint_names = ["PIP", "DIP"]
                
                # Get current timestamp
                current_time = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                
                print("\n" + "="*60)
                print("HAND TRACKING DATA - PIP & DIP Joints")
                print("="*60)
                
                # Log data to file
                log_file.write(f"[{current_time}] HAND_DATA_START\n")
                
                for f in range(5):  # 5 fingers
                    finger_start = f * 2
                    print(f"\n{finger_names[f]} Finger:")
                    for j in range(2):  # 2 joints per finger (PIP, DIP)
                        joint_idx = finger_start + j
                        angle_rad = gripper_values[joint_idx]
                        angle_deg = np.degrees(angle_rad)
                        print(f"  {joint_names[j]:>4}: {angle_deg:>7.2f}°")
                        # Log each joint angle
                        log_file.write(f"[{current_time}] {finger_names[f]} {joint_names[j]} {angle_deg:.2f}\n")
                
                # Summary statistics
                angle_deg_values = np.degrees(gripper_values)
                print(f"\nSummary:")
                print(f"  Total joints: {len(gripper_values)}")
                print(f"  Min angle: {np.min(angle_deg_values):.2f}°")
                print(f"  Max angle: {np.max(angle_deg_values):.2f}°")
                print(f"  Avg angle: {np.mean(angle_deg_values):.2f}°")
                print("="*60)
                
                # Log summary to file
                log_file.write(f"[{current_time}] SUMMARY Total:{len(gripper_values)} Min:{np.min(angle_deg_values):.2f} Max:{np.max(angle_deg_values):.2f} Avg:{np.mean(angle_deg_values):.2f}\n")
                log_file.write(f"[{current_time}] HAND_DATA_END\n\n")
                log_file.flush()  # Ensure data is written immediately
                
                # For backward compatibility with panda robot, use first value as gripper
                gripper = gripper_values[0] / 90.0  # Scale down for gripper
            elif len(gripper_values) == 16:
                # New format: 16 values (4 joints per finger: DIP, PIP, MCP_flex, MCP_abd)
                # Display detailed finger joint information
                finger_names = ["Index", "Middle", "Ring", "Thumb"]
                joint_names = ["DIP", "PIP", "MCP_Flex", "MCP_Abd"]
                
                # Get current timestamp
                current_time = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                
                print("\n" + "="*60)
                print("HAND TRACKING DATA - Individual Finger Joints")
                print("="*60)
                
                # Log data to file
                log_file.write(f"[{current_time}] HAND_DATA_START\n")
                
                for f in range(4):  # 4 fingers
                    finger_start = f * 4
                    print(f"\n{finger_names[f]} Finger:")
                    for j in range(4):  # 4 joints per finger
                        joint_idx = finger_start + j
                        angle = gripper_values[joint_idx]
                        print(f"  {joint_names[j]:>8}: {angle:>7.2f}°")
                        # Log each joint angle
                        log_file.write(f"[{current_time}] {finger_names[f]} {joint_names[j]} {angle:.2f}\n")
                
                # Summary statistics
                print(f"\nSummary:")
                print(f"  Total joints: {len(gripper_values)}")
                print(f"  Min angle: {np.min(gripper_values):.2f}°")
                print(f"  Max angle: {np.max(gripper_values):.2f}°")
                print(f"  Avg angle: {np.mean(gripper_values):.2f}°")
                print("="*60)
                
                # Log summary to file
                log_file.write(f"[{current_time}] SUMMARY Total:{len(gripper_values)} Min:{np.min(gripper_values):.2f} Max:{np.max(gripper_values):.2f} Avg:{np.mean(gripper_values):.2f}\n")
                log_file.write(f"[{current_time}] HAND_DATA_END\n\n")
                log_file.flush()  # Ensure data is written immediately
                
                # For backward compatibility with panda robot, use first value as gripper
                gripper = gripper_values[0] / 90.0  # Scale down for gripper
            elif len(gripper_values) == 48:
                # Raw quaternions format: 48 values (4 fingers × 3 bones × 4 components)
                finger_names = ["Index", "Middle", "Ring", "Thumb"]
                bone_names = ["Base", "Mid", "Tip"]

                current_time = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-3]
                print("\n" + "="*60)
                print("HAND TRACKING DATA - Raw Bone Quaternions (local space)")
                print("="*60)
                log_file.write(f"[{current_time}] HAND_DATA_START_QUATS\n")

                idx = 0
                for f in range(4):
                    print(f"\n{finger_names[f]} Finger:")
                    for b in range(3):
                        qx, qy, qz, qw = gripper_values[idx:idx+4]
                        idx += 4
                        print(f"  {bone_names[b]:>4}: x={qx: .6f}, y={qy: .6f}, z={qz: .6f}, w={qw: .6f}")
                        log_file.write(f"[{current_time}] {finger_names[f]} {bone_names[b]} q {qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}\n")

                print("\nSummary:")
                print(f"  Total quaternion components: {len(gripper_values)}")
                print("="*60)
                log_file.write(f"[{current_time}] SUMMARY_QUATS Total:{len(gripper_values)}\n")
                log_file.write(f"[{current_time}] HAND_DATA_END_QUATS\n\n")
                log_file.flush()

                # Use tip base-x as a proxy for gripper scaling to keep downstream stable
                gripper = max(min(abs(gripper_values[3]), 1.0), 0.0)
            else:
                # Fallback for unknown format
                gripper = gripper_values[0] if len(gripper_values) > 0 else 0
                print(f"Unknown format ({len(gripper_values)} values) - Using first value: {gripper:.3f}")

            # Update the position of the simulated panda robot acording to the recieved message
            if PANDAGYM_AVAILABLE:
                goal_joints[:7] = env.robot.inverse_kinematics(11, position, rotation)[:7]    
                goal_joints[7:] = gripper*2
                # env.robot.set_joint_angles(goal_joints)

    # Update the panda-gym simulation
    if PANDAGYM_AVAILABLE and time.time() > sim_update_time + ENV_STEP_PERIOD: # each env step takes 40ms, so step env every 0.04 seconds
        sim_update_time += ENV_STEP_PERIOD
        joint_angles = np.empty(9)
        for i in range(7):
            joint_angles[i] = env.robot.get_joint_angle(i)
        
        joint_angles[7:] = env.robot.get_fingers_width()
            

        error = goal_joints - joint_angles

        if OLD_PANDAGYM:
            obs, reward, done, info = env.step(10*error[:8])
        else:
            obs, reward, done, _, info = env.step(10*error[:8])

        print(joint_angles)

        # Reset the enviroment if the task is completed
        if RESET_AT_TASK_COMPLETION:
            distance1 = np.linalg.norm(env.sim.get_base_position("object1") - env.sim.get_base_position("target1"))
            distance2 = np.linalg.norm(env.sim.get_base_position("object2") - env.sim.get_base_position("target2"))
            if distance2 < 0.05 and distance1 < 0.05:
                obs = env.reset()


# Close log file
log_file.write(f"\nEnded: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
log_file.close()
print(f"\n✓ Data saved to: {log_filename}")

if PANDAGYM_AVAILABLE:
    env.close() # close the sim
