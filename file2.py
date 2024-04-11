'''
tried to make the arms and hands without URDF lol. 

output when I ran it:

Version = 4.1 Metal - 88
Vendor = Apple
Renderer = Apple M1 Pro
b3Printf: Selected demo: Physics Server
startThreads creating 1 threads.
starting thread 0
started thread 0 
MotionThreadFunc thread started
Traceback (most recent call last):
  File "/Users/ewern/Desktop/code/PyBullet/file2.py", line 45, in <module>
    right_elbow_joint_info = p.getJointInfo(right_lower_arm_id, right_elbow_joint_index)
pybullet.error: GetJointInfo failed.
numActiveThreads = 0
stopping threads
Thread with taskId 0 exiting
Thread TERMINATED
destroy semaphore
semaphore destroyed
destroy main semaphore
main semaphore destroyed
'''

import pybullet as p
import pybullet_data
import time

# Set up the Pybullet environment
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Define the arm and hand parameters
upper_arm_length = 0.3
lower_arm_length = 0.25
palm_size = [0.1, 0.1, 0.05]  # Width, Length, Height

# Create the ground plane
plane_id = p.loadURDF("plane.urdf")

# Create the right arm and hand
right_shoulder_pos = [0, 0.2, 0]
right_elbow_pos = [0, 0.2, -upper_arm_length]
right_wrist_pos = [0, 0.2, -upper_arm_length - lower_arm_length]
right_palm_pos = [0, 0.2, -upper_arm_length - lower_arm_length - palm_size[2] / 2]

right_shoulder_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.05), basePosition=right_shoulder_pos)
right_upper_arm_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, upper_arm_length / 2]), baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, upper_arm_length / 2]), basePosition=right_elbow_pos)
right_lower_arm_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, lower_arm_length / 2]), baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, lower_arm_length / 2]), basePosition=right_wrist_pos)
right_palm_id = p.createMultiBody(baseMass=0.5, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=palm_size), baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=palm_size), basePosition=right_palm_pos)

# Create the left arm and hand
left_shoulder_pos = [0, -0.2, 0]
left_elbow_pos = [0, -0.2, -upper_arm_length]
left_wrist_pos = [0, -0.2, -upper_arm_length - lower_arm_length]
left_palm_pos = [0, -0.2, -upper_arm_length - lower_arm_length - palm_size[2] / 2]

left_shoulder_id = p.createMultiBody(baseMass=0, baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.05), basePosition=left_shoulder_pos)
left_upper_arm_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, upper_arm_length / 2]), baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, upper_arm_length / 2]), basePosition=left_elbow_pos)
left_lower_arm_id = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, lower_arm_length / 2]), baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, lower_arm_length / 2]), basePosition=left_wrist_pos)
left_palm_id = p.createMultiBody(baseMass=0.5, baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=palm_size), baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=palm_size), basePosition=left_palm_pos)

# Create joints for the right arm
right_elbow_pos = [0, 0.2, -upper_arm_length]
right_elbow_orient = p.getQuaternionFromEuler([0, 0, 0])
right_elbow_joint = p.createConstraint(parentBodyUniqueId=right_upper_arm_id, parentLinkIndex=-1, childBodyUniqueId=right_lower_arm_id, childLinkIndex=-1, jointType=p.JOINT_POINT2POINT, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, -upper_arm_length / 2], childFramePosition=[0, 0, lower_arm_length / 2])
right_elbow_joint_index = p.getNumJoints(right_lower_arm_id) - 1
right_elbow_joint_info = p.getJointInfo(right_lower_arm_id, right_elbow_joint_index)
right_elbow_joint_axis = right_elbow_joint_info[13]
p.setJointMotorControlMultiDof(right_lower_arm_id, right_elbow_joint_index, p.POSITION_CONTROL, targetPosition=right_elbow_pos, targetOrientation=right_elbow_orient, jointAxis=right_elbow_joint_axis)

right_wrist_pos = [0, 0.2, -upper_arm_length - lower_arm_length]
right_wrist_orient = p.getQuaternionFromEuler([0, 0, 0])
right_wrist_joint = p.createConstraint(parentBodyUniqueId=right_lower_arm_id, parentLinkIndex=-1, childBodyUniqueId=right_palm_id, childLinkIndex=-1, jointType=p.JOINT_POINT2POINT, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, -lower_arm_length / 2], childFramePosition=[0, 0, palm_size[2] / 2])
right_wrist_joint_index = p.getNumJoints(right_palm_id) - 1
right_wrist_joint_info = p.getJointInfo(right_palm_id, right_wrist_joint_index)
right_wrist_joint_axis = right_wrist_joint_info[13]
p.setJointMotorControlMultiDof(right_palm_id, right_wrist_joint_index, p.POSITION_CONTROL, targetPosition=right_wrist_pos, targetOrientation=right_wrist_orient, jointAxis=right_wrist_joint_axis)

# Create joints for the left arm
left_elbow_pos = [0, -0.2, -upper_arm_length]
left_elbow_orient = p.getQuaternionFromEuler([0, 0, 0])
left_elbow_joint = p.createConstraint(parentBodyUniqueId=left_upper_arm_id, parentLinkIndex=-1, childBodyUniqueId=left_lower_arm_id, childLinkIndex=-1, jointType=p.JOINT_POINT2POINT, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, -upper_arm_length / 2], childFramePosition=[0, 0, lower_arm_length / 2])
left_elbow_joint_index = p.getNumJoints(left_lower_arm_id) - 1
left_elbow_joint_info = p.getJointInfo(left_lower_arm_id, left_elbow_joint_index)
left_elbow_joint_axis = left_elbow_joint_info[13]
p.setJointMotorControlMultiDof(left_lower_arm_id, left_elbow_joint_index, p.POSITION_CONTROL, targetPosition=left_elbow_pos, targetOrientation=left_elbow_orient, jointAxis=left_elbow_joint_axis)

left_wrist_pos = [0, -0.2, -upper_arm_length - lower_arm_length]
left_wrist_orient = p.getQuaternionFromEuler([0, 0, 0])
left_wrist_joint = p.createConstraint(parentBodyUniqueId=left_lower_arm_id, parentLinkIndex=-1, childBodyUniqueId=left_palm_id, childLinkIndex=-1, jointType=p.JOINT_POINT2POINT, jointAxis=[0, 0, 0], parentFramePosition=[0, 0, -lower_arm_length / 2], childFramePosition=[0, 0, palm_size[2] / 2])
left_wrist_joint_index = p.getNumJoints(left_palm_id) - 1
left_wrist_joint_info = p.getJointInfo(left_palm_id, left_wrist_joint_index)
left_wrist_joint_axis = left_wrist_joint_info[13]
p.setJointMotorControlMultiDof(left_palm_id, left_wrist_joint_index, p.POSITION_CONTROL, targetPosition=left_wrist_pos, targetOrientation=left_wrist_orient, jointAxis=left_wrist_joint_axis)

# Simulate the environment
while True:
    p.stepSimulation()
    time.sleep(1/100)  # Slow down the simulation for viewing
