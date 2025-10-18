from vpython import *
import numpy as np
import math
from time import sleep

# Replace with your file path
filename = "DATA (4).txt"

scene.range = 5
scene.forward = vector(-1,-1,-1)
scene.width = 600
scene.height = 600

xarrow = arrow(length=2, shaftwidth=0.1, color=color.red,   axis=vector(1,0,0))
yarrow = arrow(length=2, shaftwidth=0.1, color=color.green, axis=vector(0,1,0))
zarrow = arrow(length=2, shaftwidth=0.1, color=color.blue,  axis=vector(0,0,1))

frontArrow = arrow(length=4, shaftwidth=0.1, color=color.purple, axis=vector(1,0,0))
upArrow    = arrow(length=2, shaftwidth=0.1, color=color.magenta, axis=vector(0,1,0))
sideArrow  = arrow(length=2, shaftwidth=0.1, color=color.orange,  axis=vector(0,0,1))

bBoard = box(length=6, width=2, height=0.2, opacity=0.8, pos=vector(0,0,0))
bn     = box(length=1, width=0.75, height=0.1, pos=vector(1,0.1+0.1/2, 0), color=color.blue)
nano   = box(length=1.75, width=0.6, height=0.1, pos=vector(-2,0.1+0.1/2,0), color=color.green)
myObj  = compound([bBoard, bn, nano])

def normalize_quat(q0,q1,q2,q3):
    n = math.sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
    if n == 0:
        return 1,0,0,0
    return q0/n, q1/n, q2/n, q3/n

# Read the file line-by-line
with open(filename, 'r') as file:
    lines = file.readlines()

for line in lines:
    try:
        splitPacket = line.strip().split(',')
        if len(splitPacket) < 10:
            continue

        # Extract quaternions
        dt = float(splitPacket[0])
        q0 = float(splitPacket[5])
        q1 = float(splitPacket[6])
        q2 = float(splitPacket[7])
        q3 = float(splitPacket[8])
        q0,q1,q2,q3 = normalize_quat(q0,q1,q2,q3)

        roll  = -math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2))
        pitch = math.asin(2*(q0*q2-q3*q1))
        yaw   = -math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3))

        rate(30)

        k = vector(math.cos(yaw)*math.cos(pitch),
                   math.sin(pitch),
                   math.sin(yaw)*math.cos(pitch))
        
        y = vector(0,1,0)
        s = cross(k,y)
        v = cross(s,k)
        vrot = v*math.cos(roll)+cross(k,v)*math.sin(roll)

        frontArrow.axis = k * 4
        sideArrow.axis  = cross(k, vrot) * 2
        upArrow.axis    = vrot * 2

        myObj.axis = k
        myObj.up   = vrot

        sleep(dt)  # simulate real-time playback

    except Exception as e:
        print("Error:", e)
        continue
