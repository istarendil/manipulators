import pybullet as pb
import pybullet_data
from solver.solver import Solver

class Scara(Solver):
    def __init__(self, robot, d1, a1, a2, d4):
        super().__init__(d1, a1, a2, d4)

        self._robot = robot
        self.gripper_state = False

    def set_joints(self, pose):
        q1, q2, q3, q4 = self.solve_ik(pose[0], pose[1], pose[2], pose[3])
        pb.setJointMotorControlArray(self._robot, 
                                 range(pb.getNumJoints(self._robot)),
                                 pb.POSITION_CONTROL,
                                 targetPositions = [q1, q2, q3, q4, pose[4], pose[4]],
                                 forces = [100, 100, 100, 100, 100, 100])

    def move(self):
        input('Press Enter to continue...')
        for _ in range(1000):
            pb.stepSimulation()



if __name__ == '__main__':
    # Make an instance of a physic client
    client = pb.connect(pb.GUI)

    # Simulation parameters
    pb.setGravity(0, 0, -9.81)
    pb.setRealTimeSimulation(0)

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
    scara = Scara(robot_urdf, 0.2, 0.3, 0.25, 0.06)

    # Movement sequence
    poses = ((0.3, 0.3, 0.14, 0.0, 0.03),
             (0.3, 0.3, 0.02, 0.0, 0.03),
             (0.3, 0.3, 0.02, 0.0, 0.015),
             (0.3, 0.3, 0.14, 0.0, 0.015),
             (0.25, 0.25, 0.02, 0.0, 0.015),
             (0.25, 0.25, 0.02, 0.0, 0.03),
             (0.25, 0.25, 0.02, 0.0, 0.03))

    for i, pose in enumerate(poses):
        scara.set_joints(pose)
        scara.move()

# End program
input('Press Enter to stop...')
pb.disconnect()
