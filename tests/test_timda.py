import roboticstoolbox as rtb
from spatialmath import SE3

robot = rtb.models.DH.TimdaSingle7()
print(robot)

Te = robot.fkine(robot.q)
print(Te)

Tep = SE3.Trans(0.6, -0.3, 0.1)
sol = robot.ik_LM(Tep)
print(sol)

q_pickup = sol[0]
print(robot.fkine(q_pickup))

qt = rtb.jtraj(robot.q, q_pickup, 50)
robot.plot(qt.q, backend='pyplot', movie='timda_single7.gif')