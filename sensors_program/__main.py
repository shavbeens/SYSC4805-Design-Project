import b0RemoteApi
import time


norminalv = -1
slowDownv = -0.2

with b0RemoteApi.RemoteApiClient('b0RemoteApi_CoppeliaSim-addOn','b0RemoteApiAddOn',60) as client:
    
    """ display on coppelia """
    client.simxAddStatusbarMessage('Sensor Program Started',client.simxDefaultPublisher())
    
    visionHndl = [-1, -1, -1]
    retCode, visionHndl[0] = client.simxGetObjectHandle('Vision_sensor_L',client.simxServiceCall())
    retCode, visionHndl[1] = client.simxGetObjectHandle('Vision_sensor_M',client.simxServiceCall())
    retCode, visionHndl[2] = client.simxGetObjectHandle('Vision_sensor_R',client.simxServiceCall())
    print("Vision Handler ret: {}, vHandler: {}".format(retCode, visionHndl))
    while(1):
        sensor_values = [0,0,0];
        for i in range(0,3,1):
            
            retList = client.simxReadVisionSensor(visionHndl[i], client.simxServiceCall())
            
            retCode = retList[0]
            detectionState = retList[1]
            #print("vision sensor 1 retList: {}".format(retList))
            if detectionState > -1:
                Data = retList[2]
                print("vision sensor 2 ret: {}, dstate: {}, d[11]: {}".format(retCode, detectionState, Data[11]))
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
            
    
