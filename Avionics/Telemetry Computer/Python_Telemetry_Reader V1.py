from vpython import *
import numpy as np
from time import *
import math
import serial
ad = serial.Serial('com3', 115200)
sleep(1)



scene.range = 5
toRad = 2*np.pi/360
toDeg = 1/toRad
scene.forward = vector(-1,-1,-1)

scene.width = 600
scene.height = 600

xarrow = arrow(length=2, shaftwidth=0.1, color=color.red, axis=vector(1,0,0))
yarrow = arrow(length=2, shaftwidth=0.1, color=color.green, axis=vector(0,1,0))
zarrow = arrow(length=2, shaftwidth=0.1, color=color.blue, axis=vector(0,0,1))

frontArrow = arrow(length=4, shaftwidth=0.1, color=color.purple, axis=vector(1,0,0))
upArrow = arrow(length=1, shaftwidth=0.1, color=color.magenta, axis=vector(0,1,0))
sideArrow = arrow(length=1, shaftwidth=0.1, color=color.orange, axis=vector(0,0,1))

bBoard = box(length=6, width=2, height=0.2, opacity=0.8, pos=vector(0,0,0))
bn = box(length=1, width=0.75, height=0.1, pos=vector(1,0.1+0.1/2, 0), color=color.blue)
nano = box(length=1.75, width=0.6, height=0.1, pos=vector(-2,0.1+0.1/2,0), color=color.green)

myObj = compound([bBoard, bn, nano])

while True:
    try:
        
        while ad.inWaiting()==0:
            pass
        dataPacket = ad.readline()
        dataPacket = str(dataPacket, 'utf-8')
        print(dataPacket)
        splitPacket = dataPacket.split(',')
        q0 = float(splitPacket[0])
        q1 = float(splitPacket[1])
        q2 = float(splitPacket[2])
        q3 = float(splitPacket[3])


        roll = -math.atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
        pitch = math.asin(2*(q0*q2-q3*q1))
        yaw = -math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
        rate(20)
            
        k = vector(cos(yaw)*cos(pitch), sin(pitch), sin(yaw)*cos(pitch))
        y = vector(0,1,0)
        s = cross(k,y)
        v = cross(s,k)
        vrot = v*cos(roll)+cross(k,v)*sin(roll)

        frontArrow.axis = k
        sideArrow.axis = cross(k, vrot)
        upArrow.axis = vrot

        myObj.axis = k
        myObj.up = vrot
            
        sideArrow.length = 2
        upArrow.length = 2
        frontArrow.length = 4
        
    except:
        pass

                



    
    


