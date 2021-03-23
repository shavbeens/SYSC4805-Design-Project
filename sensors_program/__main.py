import b0RemoteApi
import time
import threading
import win32api
import math
import numpy as np
import sys

from os import system
clear = lambda: system('cls')


maxProximityDistance = 0.15

norminalv = 1
forwardFastV = 1
reverseFastV = -1
forwardSlowV = 0.2
reverseSlowV = -0.2

stopv = -0.05

robotAngle = 0
robotPostn = [0, 0, 0]

fieldWidth = 2.0 #meters along x
fieldLength = 4.0 #meters along y

x_p = 90
x_n = 270
y_p = 180
y_n = 360
null_dir = -1

axis_above = 1
axis_on = 0
axis_below = 2

#takes 'a' a position, and 'ref' another location value,
#it checks if 'a' is below, above or within a tolerable distance(on)
#from 'ref'
def axisClass(a, ref):
    global axis_on, axis_below, axis_above
    tol_dist = fieldWidth * 0.025 # or 2.5 percent of field width
    if (a > (ref + tol_dist) ):
        return axis_above
    if (a < (ref - tol_dist) ):
        return axis_below
    return axis_on
    

def atStopPoint(x, y, sp): #sp: stop position 
    s_x = sp[0]
    s_y = sp[1]
    if ( (axisClass(x, s_x) == axis_on) and ( axisClass(y, s_y) == axis_on) ):
        return True
    return False

#checks if angle 'a' is within the tolerance angle(2) of 'pos'
#i.e. if  (pos - 2) < a < (pos + 2) is true
def atAngle(a, pos):
    tol = 2 #angle error tolerance: 2 degrees
    if (a <= tol):  #a little something to handle the 360 to 0 ish
        a = a + 360
    if (a < (pos + tol) and a > (pos - tol)):
        return True
    return False
        

stopPosition = [-0.52, -2.65, -0.39] #x, y, z
stopAngle = y_p

pFrontBtm = 0
pFrontLft = 1
pFrontRht = 2
pFrontTop = 3
pLeft = 4
pRear = 5
pRight = 6

def sendWheelData():
    global leftMtrSpd, rightMtrSpd
    retCode = client.simxSetJointTargetVelocity(leftMotorHndl, leftMtrSpd, client.simxDefaultPublisher())
    retCode = client.simxSetJointTargetVelocity(rightMotorHndl, rightMtrSpd, client.simxDefaultPublisher())

def forwardSlow():
    print("forward movement wheels") 
    global leftMtrSpd, rightMtrSpd
    leftMtrSpd = forwardSlowV
    rightMtrSpd = forwardSlowV
    sendWheelData()    
    
def stopWheels():
    print("stopping wheels") 
    global leftMtrSpd, rightMtrSpd
    leftMtrSpd = stopv
    rightMtrSpd = stopv
    sendWheelData()
        
def diffWheelTurnRight():
    print("differential wheel turn right") 
    global leftMtrSpd, rightMtrSpd    
    leftMtrSpd = forwardSlowV 
    rightMtrSpd = reverseSlowV
    sendWheelData()
    
def diffWheelTurnLeft():
    print("differential wheel turn left") 
    global leftMtrSpd, rightMtrSpd    
    leftMtrSpd =  reverseSlowV
    rightMtrSpd = forwardSlowV
    sendWheelData() 
    

with b0RemoteApi.RemoteApiClient('b0RemoteApi_CoppeliaSim-addOn','b0RemoteApiAddOn',60) as client:
    
    """ display on coppelia """
    client.simxAddStatusbarMessage('Sensor Program Started',client.simxDefaultPublisher())
    
    visionHndl = [-1, -1, -1]
    retCode, visionHndl[0] = client.simxGetObjectHandle('Vision_sensor_L',client.simxServiceCall())
    retCode, visionHndl[1] = client.simxGetObjectHandle('Vision_sensor_M',client.simxServiceCall())
    retCode, visionHndl[2] = client.simxGetObjectHandle('Vision_sensor_R',client.simxServiceCall())

    proximityHndl = [-1, -1, -1, -1, -1, -1, -1]
    retCode, proximityHndl[pFrontBtm] = client.simxGetObjectHandle('Proximity_sensor_front_bottom',client.simxServiceCall())
    retCode, proximityHndl[pFrontLft] = client.simxGetObjectHandle('Proximity_sensor_front_left',client.simxServiceCall())
    retCode, proximityHndl[pFrontRht] = client.simxGetObjectHandle('Proximity_sensor_front_right',client.simxServiceCall())
    retCode, proximityHndl[pFrontTop] = client.simxGetObjectHandle('Proximity_sensor_front_top',client.simxServiceCall())
    retCode, proximityHndl[pLeft] = client.simxGetObjectHandle('Proximity_sensor_left',client.simxServiceCall())
    retCode, proximityHndl[pRear] = client.simxGetObjectHandle('Proximity_sensor_rear',client.simxServiceCall())
    retCode, proximityHndl[pRight] = client.simxGetObjectHandle('Proximity_sensor_right',client.simxServiceCall())

    retCode, leftMotorHndl = client.simxGetObjectHandle('LeftMotor',client.simxServiceCall())
    retCode, rightMotorHndl= client.simxGetObjectHandle('RightMotor',client.simxServiceCall())
    retCode, chuteCapMotorHndl = client.simxGetObjectHandle('ChuteMotor',client.simxServiceCall())
    retCode, chuteMotorHndl= client.simxGetObjectHandle('ChuteCapMotor',client.simxServiceCall())
    retCode, implrMotorHndl = client.simxGetObjectHandle('ImpellerMotor',client.simxServiceCall())
    retCode, augerMotorHndl= client.simxGetObjectHandle('AugerMotor',client.simxServiceCall())

    retCode, robotBodyHndl= client.simxGetObjectHandle('RobotBody',client.simxServiceCall())
    retCode, stopDummyHndl= client.simxGetObjectHandle('Stop',client.simxServiceCall())
    
    leftMtrSpd = 0
    rightMtrSpd = 0
    chuteCapMtrSpd = 0
    chuteMtrSpd = 0
    implrMtrSpd = 0
    augerMtrSpd = 0


    print("Vision Handler ret: {}, vHandler: {}".format(retCode, visionHndl))
    print("Proximity Handler ret: {}, pHandler: {}".format(retCode, visionHndl))
    print("PRESS key 's' to start robot and 'x' to stop ")

    keySprev = 0;
    keyXprev = 0;
    ROBOT_STATE = 'undef'
    
    while(1):


        """ MOVEMENT OPERATIOS: get Orientation(angle) and Position """
        retList = client.simxGetObjectOrientation(robotBodyHndl, -1, client.simxServiceCall())
        ret1 = retList[0]
        if (ret1):
            robotAngle = math.degrees(retList[1][2]) + 180
            print("Angle: \t{0:.1f}, at {1}?: {2} \t".format(robotAngle, x_p, atAngle(robotAngle, x_p)))

        
        retList = client.simxGetObjectPosition(robotBodyHndl, -1, client.simxServiceCall())
        #print("ROBOT RET: {}\t".format(retList))
        ret2 = retList[0]
        if (ret2):
            #robotPostn = retList[1]
            r_x = retList[1][0] # robot x position
            r_y = retList[1][1] # robot y position
            print("Pos: \tx {0:.4f} y {1:.4f} \t".format(r_x, r_y))
            
        retList = client.simxGetObjectPosition(stopDummyHndl, -1, client.simxServiceCall())
        ret3 = retList[0]
        if (ret3):
            stopPosition = retList[1]
            print("Stop: \tx {0:.4f}, y {1:.4f}".format(stopPosition[0],stopPosition[1]))
            print("At stop? \t{} ".format(atStopPoint(r_x, r_y, stopPosition)))

        #if (ret1 and ret2 and ret3):

        """ USER INPUT """
        c_start = 0x53  #character for start
        c_stop = 0x58   #character for stop 
        keyS = win32api.GetKeyState(c_start)
        keyX = win32api.GetKeyState(c_stop)
        if (keyS < 0 and keySprev >= 0):
            print("------------------ s pressed")
            ROBOT_STATE = 'start'
        elif (keyX < 0 and keyXprev >= 0):
            print("------------------ x pressed")
            ROBOT_STATE = 'stop'
            
        keySprev = keyS
        keyXprev = keyX


        """ STATE MACHINE """
        if (ROBOT_STATE == 'start'):
            #start motors
            forwardSlow()

            """ SENSOR OPERATIONS: """
            visionVal = [0,0,0];
            for i in range(0,3,1):
                retList = client.simxReadVisionSensor(visionHndl[i], client.simxServiceCall())
                retCode = retList[0]
                detectionState = retList[1]
                #print("vision sensor 1 retList: {}".format(retList))
                print("vision sensor[{}] detect: {} ".format(i, detectionState))
                if detectionState > -1:
                    Data = retList[2]
                    #print("vision sensor 2 ret: {}, dstate: {}, d[11]: {}".format(retCode, detectionState, Data[11]))
                    #if sensor detect snow
                    print("vision sensor [{}] data: {}".format(i,Data[11]))
                    if Data[11] < 0.7: #0: black, 0.99: white
                        visionVal[i] = 1
                    else:
                        visionVal[i] = 0 #if white

            #if both all sensors are white auger motor can be turned on
            #if the sensor goes black the motor can be turned off
            if visionVal[0] == 0 or visionVal[1] == 0 or visionVal[2] == 0:
                print("SNOW DETECTED {}, snow extraction sequence started ".format(visionVal))
                stopWheels()

            j = maxProximityDistance +1
            proxVal = [j,j,j,j,j,j,j]

            for i in range(0, len(proxVal)):
                #index-> 0: err code, 1: detection state (> 0 means there's value ), 2: detected distance
                detect = client.simxCheckProximitySensor(proximityHndl[i], "sim.handle_all", client.simxServiceCall()) 
                if (detect[1]>0):
                    proxVal[i] = detect[2]
                    print("There's Proximity data for {}, data: {}".format(i, proxVal[i]));
                else :
                    print("No Proximity data for {}".format(i));        
  
  
            k = maxProximityDistance
            if ((proxVal[pFrontBtm] < k) or (proxVal[pFrontLft] < k) or (proxVal[pFrontRht] < j) or (proxVal[pFrontTop] < k)):
                if (proxVal[pRight] < k) :
                    #turn left
                    diffWheelTurnLeft()
                    
                elif (proxVal[pLeft] < k):
                    #turn right
                    diffWheelTurnRight()
                #stop robot
                pass
                
            
        elif (ROBOT_STATE == 'stop'):
            #if robot is not in stop position
            if ( not atStopPoint(r_x, r_y, stopPosition)):
                #move to x first
                p = axisClass(r_x, stopPosition[0])
                if (p == axis_above):
                    pass
                if (p == axis_below):
                    pass
                if (p == axis_on  ):
                    pass                
                
                #move to y
                
            #if at stop postion but robot is not facing stop angle
            elif (not atAngle(robotAngle, stopAngle)):
                diffWheelTurnRight();
            
            else :
                stopWheels()
                
        else :
            #if in an unknown state 
            stopWheels()

        print("Robot state: {0}\t left motor spd: {1} \t right: {2} ".format(ROBOT_STATE, leftMtrSpd, rightMtrSpd))
            
            
        # retCode = client.simxSetJointTargetVelocity(leftMotorHndl, leftMtrSpd, client.simxDefaultPublisher())
        # retCode = client.simxSetJointTargetVelocity(rightMotorHndl, rightMtrSpd, client.simxDefaultPublisher())

        retCode = client.simxSetJointTargetVelocity(chuteCapMotorHndl, chuteCapMtrSpd, client.simxDefaultPublisher())
        retCode = client.simxSetJointTargetVelocity(chuteMotorHndl, chuteMtrSpd, client.simxDefaultPublisher())
        retCode = client.simxSetJointTargetVelocity(implrMotorHndl, implrMtrSpd, client.simxDefaultPublisher())
        retCode = client.simxSetJointTargetVelocity(augerMotorHndl, augerMtrSpd, client.simxDefaultPublisher())

        
        clear()
