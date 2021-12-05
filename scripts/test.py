from openmanipulator_custom import manipulatorHanlder
import numpy as np
from scipy.optimize import fsolve
import modern_robotics as mr

l1 = 0.077
l21 = 0.128
l22 = 0.024
l3 = 0.124
l4 = 0.126

mh = manipulatorHanlder()

q0 = [0.0, 0.5, 0.05, 0.06]
q1 = [0.0, 0.0, 0.0, 0.0]

print("Rotation z function test")
print(mh.ROTZ(0))
print("")

print("Angle to rotation and translation function test")
R0 = mh.angle_to_rotation(q0[0], q0[1], q0[2], q0[3])
p0 = mh.angle_to_pose(q0[0], q0[1], q0[2], q0[3])
R1 = mh.angle_to_rotation(q1[0], q1[1], q1[2], q1[3])
p1 = mh.angle_to_pose(q1[0], q1[1], q1[2], q1[3])
print(R0)
print(p0)
print("")

print("Rotation matrix inverse kinematics test")
R = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]], dtype=float)

#q1, q2, q3 ,q4 = fsolve(mh.rotation_error, q0, args=[R])

Slist = np.array([[0, 0,  1,  0, 0,      0],
                  [1, 0,  0,  0, l1,     0],
                  [1, 0,  0,  0, l1+l21, l22],
                  [1, 0,  0,  0, l1+l21, l22+l3]]).T

print(Slist)

# home position
M = mr.RpToTrans(R0, p0)
T = mr.RpToTrans(R1, p1)
thetalist0 = np.array(q0)
eomg = 0.000000001
ev = 0.000000001

(qlist, success) = mr.IKinSpace(Slist, M, T, thetalist0, eomg, ev)

print("ik result")
print(success)
R_new = mh.angle_to_rotation(qlist[0], qlist[1], qlist[2], qlist[3])
p_new = mh.angle_to_pose(qlist[0], qlist[1], qlist[2], qlist[3])
T_new = mr.RpToTrans(R_new, p_new)

print("desried")
print(T)

print("init guess")
print(M)
print(np.array(q0))
print("soln")
print(T_new)
print(qlist[0], qlist[1], qlist[2], qlist[3])