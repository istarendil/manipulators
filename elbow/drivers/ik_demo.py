import pybullet as pb
import pybullet_data
from solver.solver import Solver
from numpy import pi

class Elbow(Solver):
    def __init__(self, robot, d1, a2, d4, d6):
        super().__init__(d1, a2, d4, d6)
        self._robot = robot

    def set_joints(self, position, orientation):
        q = self.solve_ik(position, orientation)
        pb.setJointMotorControlArray(self._robot, 
                                 range(pb.getNumJoints(self._robot)),
                                 pb.POSITION_CONTROL,
                                 targetPositions = q)

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
    cube = pb.loadURDF('cube.urdf', basePosition = [0.15, 0.0, 0.775], globalScaling = 0.3)
    robot_urdf = pb.loadURDF('../urdf/elbow.urdf.xml', 
                        basePosition = [-0.4, 0.0, 0.625],
                        useFixedBase = 1)
    
    # Instanciating a Scara robot
    elbow = Elbow(robot_urdf, 0.17, 0.25, 0.23, 0.16)

    # Movement sequence (xyz, rpy)
    poses =  (((0.4, -0.15, 0.3), (-pi/4, 3*pi/4, 0.0)),
             ((0.4, 0.15, 0.3), (pi/4, 3*pi/4, 0.0)))

    for pose in poses:
        elbow.set_joints(pose[0], pose[1])
        elbow.move()

# End program
input('Press Enter to stop...')
pb.disconnect()
