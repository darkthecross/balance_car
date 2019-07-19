from controller import calcWheelVels
import pybullet as p
import time

p.connect(p.GUI)
p.setGravity(0, 0, -10)

p.loadURDF("data/plane.urdf", [0, 0, 0], useFixedBase=True)
rbt = p.loadURDF("data/balance_car.urdf", [0, 0, 2])

maxForce = 50

while (p.isConnected()):
  pos, quat = p.getBasePositionAndOrientation(rbt)
  lv, av = p.getBaseVelocity(rbt)

  v1, v2 = calcWheelVels(pos, quat, lv, av)

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