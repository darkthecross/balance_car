from controller import Controller
import pybullet as p
import time

p.connect(p.GUI)
p.setGravity(0, 0, -10)

p.loadURDF("data/plane.urdf", [0, 0, 0], useFixedBase=True)
rbt = p.loadURDF("data/balance_car.urdf", [0, 0, 2])

maxForce = 50

c = Controller()

f = 0

#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "log.mp4")

while (p.isConnected() and f < 2400):
  pos, quat = p.getBasePositionAndOrientation(rbt)
  lv, av = p.getBaseVelocity(rbt)
  v1, v2 = c.calcWheelVelsAngle(pos, quat, lv, av)
  p.setJointMotorControl2(bodyUniqueId=rbt, 
                          jointIndex=0, 
                          controlMode=p.VELOCITY_CONTROL,
                          targetVelocity = -v1,
                          force = maxForce)
  p.setJointMotorControl2(bodyUniqueId=rbt, 
                          jointIndex=1, 
                          controlMode=p.VELOCITY_CONTROL,
                          targetVelocity = v2,
                          force = maxForce)
  p.stepSimulation()
  time.sleep(1. / 240.)
  f = f + 1

#p.stopStateLogging()