# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
# Testing line

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import threading

# Robot State Variables


def getAverageColour(auxilary):
    red = auxilary[11]
    green = auxilary[12]
    blue = auxilary[13]
    return (red + green + blue) / 3




print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,3) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')
    
    
    # Initializing the objects that are gonna be used in the robot, starting with vision sensors
    [leftSensorReturnCode, leftHandle] = sim.simxGetObjectHandle(clientID, "Vision_sensor_L", sim.simx_opmode_blocking)
    [centerSensorReturnCode, centerHandle] = sim.simxGetObjectHandle(clientID, "Vision_sensor_M", sim.simx_opmode_blocking)
    [rightSensorReturnCode, rightHandle] = sim.simxGetObjectHandle(clientID, "Vision_sensor_R", sim.simx_opmode_blocking)
    # sensors are then saved in the following tuple
    visionSensor = [leftHandle, centerHandle, rightHandle]


    # Initialization of the Motor joints
    [leftMotorReturnCode, leftJoint] = sim.simxGetObjectHandle(clientID, "LeftMotor", sim.simx_opmode_blocking)
    [rightMotorReturnCode, rightJoint] = sim.simxGetObjectHandle(clientID, "RightMotor", sim.simx_opmode_blocking)
    # [chuteMotorReturnCode, chuteJoint] = sim.simxGetObjectHandle(clientID, "ChuteMotor", sim.simx_opmode_blocking)
    print(leftMotorReturnCode, rightMotorReturnCode)
    NOMINAL_VELOCITY = 2 # Nominal velocisty that will be used
    VAR = 1

    # Get Object Handles for CentreGravity and CentreGravity_1 reference points
    [centreGravityReturnCode, referencePoint] = sim.simxGetObjectHandle(clientID, "CentreGravity", sim.simx_opmode_blocking)
    [centreGravity_1ReturnCode, referencePoint2] = sim.simxGetObjectHandle(clientID, "CentreGravity_1", sim.simx_opmode_blocking)
    #Get Object handles for PlaneY and PlaneX
    [planeYReturnCode, planeY] = sim.simxGetObjectHandle(clientID, "PlaneY", sim.simx_opmode_blocking)
    [planeXReturnCode, planeX] = sim.simxGetObjectHandle(clientID, "PlaneX", sim.simx_opmode_blocking)

    # Main operating function of the robot
    while True:
        defaultReading = [False, False, False]
        sensorReading = list(defaultReading)
        collision = False


        # Checks if colour reading are correct from the vision Sensor
        for i in range(0, 3):
            returnBool, state, aux = sim.simxReadVisionSensor(clientID, visionSensor[i], sim.simx_opmode_blocking)
            if state > -1:
                if i == 1:
                    sensorReading[i] = getAverageColour(aux[0]) > 0.45
                    print(sensorReading[i], ": ", aux, "\n\n")
                else:
                    sensorReading[i] = getAverageColour(aux[0]) < 0.25


        # Find the readings from the object group data, Starting with X, Y, Z points for CentreGravity
        returnCode, x1 = sim.simxGetObjectPosition(clientID, referencePoint, planeX, sim.simx_opmode_blocking)
        returnCode, y1 = sim.simxGetObjectPosition(clientID, referencePoint, planeY, sim.simx_opmode_blocking)
        # Then Find X, Y, Z coordinates for CentreGravity_1
        returnCode, x2 = sim.simxGetObjectPosition(clientID, referencePoint2, planeX, sim.simx_opmode_blocking)
        returnCode, y2 = sim.simxGetObjectPosition(clientID, referencePoint2, planeY, sim.simx_opmode_blocking)
        # We only care about X value for x1 and x2, and Y value for y1 and y2; We have to be mindful of what value we take according to our reference point 
        x1 = x1[0]
        x2 = x2[0]
        y1 = y1[0]
        y2 = y2[0]

        print("CentreGravity: (%s, %s)" % (x1, y1))
        print("CentreGravity_1: (%s, %s)" % (x2, y2))





        # Sets velocity of the robot according to the calculated solutions above
        rightV = NOMINAL_VELOCITY
        leftV = NOMINAL_VELOCITY
        # chuteV = NOMINAL_VELOCITY

        if sensorReading[0] and not sensorReading[2]:
            leftV = 1.3
        if sensorReading[2] and not sensorReading[0]:
            rightV = 1.3
        # if sensorReading[0] and sensorReading[1] and sensorReading[2]:
        #     leftV = 0
        #     rightV = 0
        #     collision = True

        leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
        rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
        # chuteMotorReturnCode = sim.simxSetJointTargetVelocity(clientID, chuteJoint, chuteV, sim.simx_opmode_blocking)

    
    print("a collision is about to occur")






    # Moves left motor forward at 1 speed
    leftMotorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, -1, sim.simx_opmode_blocking)
    time.sleep(10) # sleep for 10 seconds and run
    leftMotorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, 0, sim.simx_opmode_blocking)
    # Off

    # Rest of the Code should be written here
    sim.simxGetPingTime(clientID)

    sim.simxFinish(clientID)
    # # Now try to retrieve data in a blocking fashion (i.e. a service call):
    # res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    # if res==sim.simx_return_ok:
    #     print ('Number of objects in the scene: ',len(objs))
    # else:
    #     print ('Remote API function call returned with error code: ',res)
    #
    # time.sleep(2)
    #
    # # Now retrieve streaming data (i.e. in a non-blocking fashion):
    # startTime=time.time()
    # sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
    # while time.time()-startTime < 5:
    #     returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
    #     if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
    #         print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
    #     time.sleep(0.005)
    #
    # # Now send some data to CoppeliaSim in a non-blocking fashion:
    # sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)
    #
    # # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    # sim.simxGetPingTime(clientID)
    #
    # # Now close the connection to CoppeliaSim:
    # sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
