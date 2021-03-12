import b0RemoteApi
import time
import threading
import win32api

norminalv = 1
slowDownv = 0.2
stopv = -0.05



with b0RemoteApi.RemoteApiClient('b0RemoteApi_CoppeliaSim-addOn','b0RemoteApiAddOn',60) as client:
    
    """ display on coppelia """
    client.simxAddStatusbarMessage('Sensor Program Started',client.simxDefaultPublisher())
    
    visionHndl = [-1, -1, -1]
    retCode, visionHndl[0] = client.simxGetObjectHandle('Vision_sensor_L',client.simxServiceCall())
    retCode, visionHndl[1] = client.simxGetObjectHandle('Vision_sensor_M',client.simxServiceCall())
    retCode, visionHndl[2] = client.simxGetObjectHandle('Vision_sensor_R',client.simxServiceCall())

    retCode, leftMotorHndl = client.simxGetObjectHandle('LeftMotor',client.simxServiceCall())
    retCode, rightMotorHndl= client.simxGetObjectHandle('RightMotor',client.simxServiceCall())
    retCode, chuteCapMotorHndl = client.simxGetObjectHandle('ChuteMotor',client.simxServiceCall())
    retCode, chuteMotorHndl= client.simxGetObjectHandle('ChuteCapMotor',client.simxServiceCall())
    retCode, implrMotorHndl = client.simxGetObjectHandle('ImpellerMotor',client.simxServiceCall())
    retCode, augerMotorHndl= client.simxGetObjectHandle('AugerMotor',client.simxServiceCall())
    
    leftMtrSpd = 0
    rightMtrSpd = 0
    chuteCapMtrSpd = 0
    chuteMtrSpd = 0
    implrMtrSpd = 0
    augerMtrSpd = 0


    print("Vision Handler ret: {}, vHandler: {}".format(retCode, visionHndl))
    print("PRESS key 's' to start robot and 'x' to stop ")

    keySprev = 0;
    keyXprev = 0;
    ROBOT_STATE = 'x'
    while(1):
        keyS = win32api.GetKeyState(0x53)
        keyX = win32api.GetKeyState(0x58)
        if (keyS < 0 and keySprev >= 0):
            print("------------------ s pressed")
            ROBOT_STATE = 's'
        elif (keyX < 0 and keyXprev >= 0):
            print("------------------ x pressed")
            ROBOT_STATE = 'x'
            
        keySprev = keyS
        keyXprev = keyX


        if (ROBOT_STATE == 's'):
            #start motors
            leftMtrSpd = slowDownv    
            rightMtrSpd = slowDownv
            
        elif (ROBOT_STATE == 'x'):
            #stop motors
            leftMtrSpd = stopv     
            rightMtrSpd = stopv
        else :
            #stop motors
            leftMtrSpd = stopv     
            rightMtrSpd = stopv 

            
        sensor_values = [0,0,0];
        for i in range(0,3,1):
            
            retList = client.simxReadVisionSensor(visionHndl[i], client.simxServiceCall())
            
            retCode = retList[0]
            detectionState = retList[1]
            #print("vision sensor 1 retList: {}".format(retList))
            if detectionState > -1:
                Data = retList[2]
                #print("vision sensor 2 ret: {}, dstate: {}, d[11]: {}".format(retCode, detectionState, Data[11]))
                #if sensor detect snow
                if Data[11] < 0.4: 
                    sensor_values[i] = 1
                else :
                    sensor_values[i] = 0

        #if both all sensors are white auger motor can be turned on
        #if the sensor goes black the motor can be turned off
        if sensor_values[0] == 0:
            pass
        if sensor_values[2] == 0:
            pass
            
 
        retCode = client.simxSetJointTargetVelocity(leftMotorHndl, leftMtrSpd, client.simxServiceCall())
        retCode = client.simxSetJointTargetVelocity(rightMotorHndl, rightMtrSpd, client.simxServiceCall())

        retCode = client.simxSetJointTargetVelocity(chuteCapMotorHndl, chuteCapMtrSpd, client.simxServiceCall())
        retCode = client.simxSetJointTargetVelocity(chuteMotorHndl, chuteMtrSpd, client.simxServiceCall())
        retCode = client.simxSetJointTargetVelocity(implrMotorHndl, implrMtrSpd, client.simxServiceCall())
        retCode = client.simxSetJointTargetVelocity(augerMotorHndl, augerMtrSpd, client.simxServiceCall())

