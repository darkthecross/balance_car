import pybullet as p
import numpy as np
import math

class Controller:
    def __init__(self):
        self.lastP = 0
        self.ts = 1./240.
        self.kp = 1
        self.kd = 0
        self.ki = 1
        self.inti = 0
        self.mul = 50
    
    def calcAngle(self, v1, v2):
        return math.acos(np.dot(v1[:3], v2[:3]))

    # PI control for angle. May move.
    def calcWheelVelsAngle(self, pos, quat, lv, av):
        # Calculate the Z axis of the car body.
        rotMat = p.getMatrixFromQuaternion(quat)
        hMat = np.eye(4, dtype=np.float32)
        hMat[:3, :3] = np.array(rotMat).reshape((3, 3))
        zVec = np.array([0, 0, 1, 1])
        rotatedZ = np.matmul(hMat, zVec)
        # Calculate the angle of Z axis.
        theta = self.calcAngle(zVec, rotatedZ)
        yVec = np.array([0, 1, 0, 1])
        rotatedY = np.matmul(hMat, yVec)
        cross = np.cross(zVec[:3], rotatedZ[:3])
        yTheta = self.calcAngle(cross, rotatedY)
        if yTheta > math.pi / 2:
            theta = -theta
        print(theta)
        # PID
        dTheta = (theta - self.lastP) / self.ts
        self.lastP = theta
        self.inti = self.inti + theta * self.ts
        v = self.kp * theta + self.kd * dTheta + self.ki * self.inti
        res = self.mul * v
        print(res)
        return res, res