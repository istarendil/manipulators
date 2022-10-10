import numpy as np
import pybullet as pb
import pybullet_data
from solver.solver import Solver

# Move the robot
def move(robot, joint_positions):
    pb.setJointMotorControlArray(robot, 
                                 range(pb.getNumJoints(robot)),
                                 pb.POSITION_CONTROL,
                                 targetPositions = joint_positions,
                                 forces = [100,100,100,100,100,100])
    
    # Execute the simulation
    input('Press Enter to continue...')
    for _ in range(1000):
        pb.stepSimulation()

    return


# Make an instance of a physic client
client = pb.connect(pb.GUI)

# Set/unset real time simulation
pb.setGravity(0, 0, -9.81)
pb.setRealTimeSimulation(0)

# Charge objects
pb.setAdditionalSearchPath(pybullet_data.getDataPath())
floor = pb.loadURDF('plane.urdf')
table = pb.loadURDF('table/table.urdf')
cube = pb.loadURDF('cube.urdf', basePosition = [0.3, 0.3, 0.64], globalScaling = 0.03)
robot = pb.loadURDF('urdf/scara.urdf.xml', 
                    basePosition=[0.0, 0.0, 0.62],
                    useFixedBase = 1,
                    flags = pb.URDF_MERGE_FIXED_LINKS)

# Inverse kinematics
scara = Solver(0.2, 0.3, 0.25, 0.06)

# Movement sequence
th1, th2, d3, th4 = scara.solve_ik(0.3, 0.3, 0.14, 0.0)
move(robot, [th1, th2, d3, th4, 0.03, 0.03])

th1, th2, d3, th4 = scara.solve_ik(0.3, 0.3, 0.02, 0.0)
move(robot, [th1, th2, d3, th4, 0.03, 0.03])

move(robot, [th1, th2, d3, th4, 0.015, 0.015])

th1, th2, d3, th4 = scara.solve_ik(0.3, 0.3, 0.14, 0.0)
move(robot, [th1, th2, d3, th4, 0.015, 0.015])

th1, th2, d3, th4 = scara.solve_ik(0.25, 0.25, 0.02, 0.0)
move(robot, [th1, th2, d3, th4, 0.015, 0.015])

th1, th2, d3, th4 = scara.solve_ik(0.25, 0.25, 0.02, 0.0)
move(robot, [th1, th2, d3, th4, 0.03, 0.03])

th1, th2, d3, th4 = scara.solve_ik(0.25, 0.25, 0.14, 0.0)
move(robot, [th1, th2, d3, th4, 0.03, 0.03])


# End program
input('Press Enter to stop...')
pb.disconnect()
