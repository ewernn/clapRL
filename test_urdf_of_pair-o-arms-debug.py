import pybullet as p
import pybullet_data
import time

# Function to draw coordinate axes
def draw_axes(robot_id, link_index, length=0.1, lifetime=0):
    # X-axis in red
    p.addUserDebugLine([0, 0, 0], [length, 0, 0], [1, 0, 0], parentObjectUniqueId=robot_id, parentLinkIndex=link_index, lifeTime=lifetime)
    # Y-axis in green
    p.addUserDebugLine([0, 0, 0], [0, length, 0], [0, 1, 0], parentObjectUniqueId=robot_id, parentLinkIndex=link_index, lifeTime=lifetime)
    # Z-axis in blue
    p.addUserDebugLine([0, 0, 0], [0, 0, length], [0, 0, 1], parentObjectUniqueId=robot_id, parentLinkIndex=link_index, lifeTime=lifetime)

# Start PyBullet in GUI mode to visualize the robot:
p.connect(p.GUI)

# Optionally set the gravity:
p.setGravity(0, 0, -9.81)

# Load the URDF. Replace the path with your URDF file name:
robot = p.loadURDF("/Users/ewern/Desktop/code/PyBullet/pair_or_arms.urdf", useFixedBase=True)

# Find the number of joints to identify the wrists
num_joints = p.getNumJoints(robot)
print(f"num_joints: {num_joints}")
left_wrist_index = -1
right_wrist_index = -1

# Assume the wrists are the last two joints, change if different
left_wrist_index = num_joints - 2
right_wrist_index = num_joints - 1

# Draw axes for the left and right wrist
for i in range(num_joints):
    draw_axes(robot, i)
#draw_axes(robot, right_wrist_index)

# Run the simulation for a short period:
for _ in range(240):  # Simulate for 240 timesteps (~4 seconds at 60Hz)
    p.stepSimulation()
    time.sleep(1./60.)  # Real-time simulation

# Disconnect from the simulation
p.disconnect()
