import numpy as np
import pybullet as pb
import pybullet_data

# Make an instance of a physic client
phyClient = pb.connect(pb.GUI)

# Set/unset real time simulation
pb.setGravity(0, 0, -9.81)
pb.setRealTimeSimulation(0)

# Charge objects
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
floor = pb.loadURDF('plane.urdf')
table = pb.loadURDF('table/table.urdf')
robot = pb.loadURDF('../urdf/scara.urdf.xml', basePosition=[0.0, 0.0, 0.62], useFixedBase=1)

# End program
input('Press Enter to stop...')
pb.disconnect()

