import pybullet as p
import pybullet_data
import time

# Start PyBullet in GUI mode to visualize the robot:
p.connect(p.GUI)

# Optionally set the gravity:
p.setGravity(0, 0, -9.81)

# Load the URDF. Replace 'articulated_arms_robot.urdf' with your URDF file name:
robot = p.loadURDF("/Users/ewern/Desktop/code/PyBullet/pair_or_arms.urdf", useFixedBase=True)

# Run the simulation for a short period:
for _ in range(240):  # Simulate for 240 timesteps (~4 seconds at 60Hz)
    p.stepSimulation()
    time.sleep(1./60.)  # Real-time simulation
