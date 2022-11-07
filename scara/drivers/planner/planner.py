import numpy as np


class Planner:
    def __init__(self):
        pass

    def plan_j3(self, tf, dt, q0, qf, v0, vf):
        q0 = np.array(q0)
        qf = np.array(qf)
        v0 = np.array(v0)
        vf = np.array(vf)
        
        # Time vector
        t = np.arange(0, tf+dt, dt)

        # Polynomial coefficients
        C0 = q0
        C1 = v0
        C2 = (3*(qf - q0)/tf**2) - ((vf + 2*v0)/tf)
        C3 = -(2*(qf -q0)/tf**3) + ((vf + v0)/tf**2)

        # Resulting trajectories
        q = np.dot(np.array([t**0, t, t**2, t**3]).transpose(), np.array([C0, C1, C2, C3]))
        v = np.dot(np.array([t**0, t, t**2]).transpose(), np.array([C1, 2*C2, 3*C3]))
        a = np.dot(np.array([t**0, t]).transpose(), np.array([2*C2, 6*C3]))

        return a, v, q

    def plan_line(self, tf, dt, o0, of, r0, rf):
        dt = int(tf/dt)

        # Auxiliar vector coefficients
        v = of - o0
        
        # Coordinates
        a = np.linspace(r0, rf, dt)
        x = np.linspace(o0[0], of[0], dt)

        t = (x - o0[0])/ v[0]
        
        y = o0[1] + v[1]*t
        z = o0[2] + v[2]*t
                        
        return x, y, z, a

