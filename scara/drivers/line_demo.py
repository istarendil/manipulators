import pybullet as pb
import pybullet_data
from solver.solver import Solver
from planner.planner import Planner

import time

class Scara(Solver, Planner):
    def __init__(self, robot, dt, d1, a1, a2, d4):
        super().__init__(d1, a1, a2, d4)

        self._robot = robot
        self._dt = dt
        self.gripper_state = False
        self.joint_state = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def set_joints(self, joint_state):
        pb.setJointMotorControlArray(self._robot,
                                 range(pb.getNumJoints(self._robot)),
                                 pb.POSITION_CONTROL,
                                 targetPositions = self.joint_state,
                                 forces = [100, 100, 100, 100, 100, 100])
        self.joint_state = joint_state

    def move(self, joint_target):
        self.set_joints(joint_target + [0.015, 0.015])
        time.sleep(self._dt)
        pb.stepSimulation()



if __name__ == '__main__':
    # Make an instance of a physic client
    client = pb.connect(pb.GUI)

    # Simulation parameters
    pb.setGravity(0, 0, -9.81)
    pb.setRealTimeSimulation(0)
    dt = 0.01

    #Environment 
    pb.setAdditionalSearchPath(pybullet_data.getDataPath())
    floor = pb.loadURDF('plane.urdf')
    table = pb.loadURDF('table/table.urdf')
    cube = pb.loadURDF('cube.urdf', basePosition = [0.3, 0.3, 0.64], globalScaling = 0.03)
    robot_urdf = pb.loadURDF('../urdf/scara.urdf.xml',
                        basePosition=[0.0, 0.0, 0.62],
                        useFixedBase = 1,
                        flags = pb.URDF_MERGE_FIXED_LINKS)

    # Instanciating a Scara robot
    scara = Scara(robot_urdf, dt, 0.2, 0.3, 0.25, 0.06)

    # Trajectory
    Vx, Vy, Vz, Va = scara.plan_line(4, dt, (0.50, 0.0, 0.14, 0.0), (0.3, 0.3, 0.02, 45.0))  


    input('Press Enter to continue...')
    for x, y, z, a in zip(Vx, Vy, Vz, Va):
        joint_state = list(scara.solve_ik(x, y, z, a))
        scara.move(joint_state)

    # End program
    input('Press Enter to stop...')
    pb.disconnect()
