import numpy as np

class Solver:
    def __init__(self, d1, a1, a2, d4):
        self.d1 = d1
        self.a1 = a1
        self.a2 = a2
        self.d4 = d4
        
    # Inverse kinematics solver  
    def solve_ik(self, Ox, Oy, Oz, alpha):
        d3 = self.d1 - self.d4 - Oz
        
        D = (Ox**2 + Oy**2 - self.a1**2 - self.a2**2)/(2*self.a1*self.a2)
        th2 = np.arctan2(np.sqrt(1 - D**2), D)
        
        th1 = np.arctan2(Oy, Ox) - np.arctan2(self.a2*np.sin(th2), self.a1 + self.a2*np.cos(th2))
        
        th4 = th1 + th2 - alpha
        
        return th1, th2, d3, th4
