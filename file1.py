import pybullet as p
import pybullet_data
import time

# Start PyBullet in GUI mode
physicsClient = p.connect(p.GUI)

# Load the basic environment
p.setGravity(0, 0, -10)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # To load models from the PyBullet data directory

# Load a plane and a humanoid robot
planeId = p.loadURDF("plane.urdf")
humanoidId = p.loadURDF("humanoid/humanoid.urdf", basePosition=[0, 0, 1])

# Simulate for a short period of time
for _ in range(10000):
    p.stepSimulation()
    time.sleep(1/10000)  # Slow down the simulation for viewing

# Disconnect from the session
p.disconnect()
