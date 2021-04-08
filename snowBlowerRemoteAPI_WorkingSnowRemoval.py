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
import math

# Robot State Variables
STOP = 0
STANDBY = 1
MOVESTRAIGHT = 2
UTURNLEFT = 3
UTURNRIGHT = 4
AVOIDLEFT = 5
AVOIDRIGHT = 6

# Robot state variable
currentState = STANDBY

# Robot initialization variable
isStart = True

# Robot Checkpoint variables
nextCheckPoint = 1.0
checkPointIndex = 1.0

# Robot Heading Variables
NORTH = 0
SOUTH = 1
robotHeading = NORTH # Change this to south if the robot starts heading south initially 

# Robot Turning Variables
LEFT = 0
RIGHT = 1
robotNextTurn = LEFT # Change this to right if the first turn is a right turn
uturnComplete = False
avoidanceInit = True
headingXcoordinate = 0.0
headingXcoordinate2 = 0.0

def getAverageColour(auxilary):
    red = auxilary[11]
    green = auxilary[12]
    blue = auxilary[13]
    return (red + green + blue) / 3


class myThread(threading.Thread):
    def __init__(self, threadID, name, clientID, referencePoint, referencePoint2, planeX, planeY):
        threading.Thread.__init__(self)
        self.ThreadID = threadID
        self.name = name
        self.clientID = clientID
        self.referencePoint = referencePoint
        self.referencePoint2 = referencePoint2
        self.planeX = planeX
        self.planeY = planeY

    def run(self):
        print("Starting " + self.name)
        runThread(self.name, self.clientID, self.referencePoint, self.referencePoint2, self.planeX, self.planeY)
        print("%s Exited" % self.name)

# Thread Runnable
def runThread(threadName, clientID, referencePoint, referencePoint2, planeX, planeY):
    global currentState
    keepRunning = True

    print("running " + threadName)
    


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

    # Initializing the proximity sensors that are gonna be used in the robot
    [leftProximityReturnCode, proximityLeft_Handle] = sim.simxGetObjectHandle(clientID, "Proximity_sensor_L", sim.simx_opmode_blocking)
    [centerProximityReturnCode, proximityCenter_Handle] = sim.simxGetObjectHandle(clientID, "Proximity_sensor_M", sim.simx_opmode_blocking)
    [rightProximityReturnCode, proximityRight_Handle] = sim.simxGetObjectHandle(clientID, "Proximity_sensor_R", sim.simx_opmode_blocking)


    # Initialization of the Motor joints
    [leftMotorReturnCode, leftJoint] = sim.simxGetObjectHandle(clientID, "LeftMotor", sim.simx_opmode_blocking)
    [rightMotorReturnCode, rightJoint] = sim.simxGetObjectHandle(clientID, "RightMotor", sim.simx_opmode_blocking)
    # [chuteMotorReturnCode, chuteJoint] = sim.simxGetObjectHandle(clientID, "ChuteMotor", sim.simx_opmode_blocking)
    print(leftMotorReturnCode, rightMotorReturnCode)
    NOMINAL_VELOCITY = 1 # Nominal velocisty that will be used
    VAR = 1

    # Get Object Handles for CentreGravity and CentreGravity_1 reference points
    [centreGravityReturnCode, referencePoint] = sim.simxGetObjectHandle(clientID, "CentreGravity", sim.simx_opmode_blocking)
    [centreGravity_1ReturnCode, referencePoint2] = sim.simxGetObjectHandle(clientID, "CentreGravity_1", sim.simx_opmode_blocking)
    #Get Object handles for PlaneY and PlaneX
    [planeYReturnCode, planeY] = sim.simxGetObjectHandle(clientID, "PlaneY", sim.simx_opmode_blocking)
    [planeXReturnCode, planeX] = sim.simxGetObjectHandle(clientID, "PlaneX", sim.simx_opmode_blocking)

    returnCode, startingX = sim.simxGetObjectPosition(clientID, referencePoint2, planeX, sim.simx_opmode_blocking)
    returnCode, startingY = sim.simxGetObjectPosition(clientID, referencePoint2, planeY, sim.simx_opmode_blocking)

    # thread = myThread(1, "MovementThread", clientID, referencePoint, referencePoint2, planeX, planeY)
    # thread.start()

    # Main operating function of the robot
    while currentState != STOP:
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
        # Then Find theta between centreGravity and CentreGravity_1-----> theta = inverseTan(y2-y1 / x2-x1)
        theta = math.degrees(math.atan((y2-y1) / (x2-x1)))

        defaultReading = [False, False, False]
        sensorReading = list(defaultReading)
        collision = False

        print("Theta angle is: %s degrees" % theta)


        # Checks if colour reading are correct from the vision Sensor
        returnBool, state, aux = sim.simxReadVisionSensor(clientID, visionSensor[1], sim.simx_opmode_blocking)
        if state > -1:
            sensorReading[1] = getAverageColour(aux[0]) > 0.45

        # If sensorReading[1] is True, then that means we have detected snow, let's get the handle from Proximity sensor
        if sensorReading[1]:
            midProxReturnCode, detectionState_M, detectedPoint_M, detectedObjectHandle_M, detectedSurfaceNormal_M = sim.simxReadProximitySensor(clientID, proximityCenter_Handle, sim.simx_opmode_blocking)
            print("Shovel Activated, Pushing Snow")
            removeReturnCode = sim.simxRemoveObject(clientID, detectedObjectHandle_M, sim.simx_opmode_streaming)

        leftProxReturnCode, detectionState_L, detectedPoint_L, detectedObjectHandle_L, detectedSurfaceNormal_L = sim.simxReadProximitySensor(clientID, proximityLeft_Handle, sim.simx_opmode_blocking)
        rightProxReturnCode, detectionState_R, detectedPoint_R, detectedObjectHandle_R, detectedSurfaceNormal_R = sim.simxReadProximitySensor(clientID, proximityRight_Handle, sim.simx_opmode_blocking)
        print("Right state: ", detectionState_R, "\n", "Left state: ", detectionState_L, "\n")
        
        if currentState == STOP:
            print("exiting movement loop...")

        elif currentState == STANDBY:
            if isStart:
                currentState = MOVESTRAIGHT
                print("Currently in Standby but changed states to MoveStraigh")
                isStart = False
            else:
                # Firstly stop the movement on the robot
                leftV = 0
                rightV = 0
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

                # Check if we reached our checkpoint
                if robotHeading == NORTH:
                    if nextCheckPoint == 1.0:
                        print("Got to the first checkpoint")
                        nextCheckPoint = nextCheckPoint + checkPointIndex
                        currentState = MOVESTRAIGHT
                    elif nextCheckPoint == 2.0:
                        print("Got to the second checkpoint")
                        nextCheckPoint = nextCheckPoint + checkPointIndex
                        currentState = MOVESTRAIGHT
                    elif nextCheckPoint == 3.0:
                        print("Got to the third checkpoint, gonna u turn")
                        #nextCheckPoint = nextCheckPoint + checkPointIndex
                        currentState = UTURNLEFT
                elif robotHeading == SOUTH:
                    if nextCheckPoint == 3.0:
                        print("Got to the first checkpoint in new row")
                        nextCheckPoint = nextCheckPoint - checkPointIndex
                        currentState = MOVESTRAIGHT
                    elif nextCheckPoint == 2.0:
                        print("Got to the second checkpoint")
                        nextCheckPoint = nextCheckPoint - checkPointIndex
                        currentState = MOVESTRAIGHT
                    elif nextCheckPoint == 1.0:
                        print("Got to the third checkpoint, gonna stop")
                        currentState = UTURNRIGHT
                        


        elif currentState == MOVESTRAIGHT:
            if detectionState_R and not sensorReading[1]:
                currentState = AVOIDLEFT
            elif detectionState_L and not sensorReading[1]:
                currentState = AVOIDRIGHT
            # Sets velocity of the robot according to the calculated solutions above
            rightV = NOMINAL_VELOCITY
            leftV = NOMINAL_VELOCITY
            # chuteV = NOMINAL_VELOCITY

            if robotHeading == NORTH:
                # if sensorReading[0] and not sensorReading[2]:
                if x1 > x2:
                    leftV = 0.9
                # if sensorReading[2] and not sensorReading[0]:
                if x1 < x2:
                    rightV = 0.9
            elif robotHeading == SOUTH:
                # if sensorReading[0] and not sensorReading[2]:
                if x1 > x2:
                    rightV = 0.9
                # if sensorReading[2] and not sensorReading[0]:
                if x1 < x2:
                    leftV = 0.9
                
            print("CentreGravity: (%s, %s)" % (x1, y1))
            print("CentreGravity_1: (%s, %s)" % (x2, y2))
            print("Theta angle is: %s degrees" % theta)

            leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
            rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
            # chuteMotorReturnCode = sim.simxSetJointTargetVelocity(clientID, chuteJoint, chuteV, sim.simx_opmode_blocking)

            # If Y2 is at 1, then it is a checkpoint
            if robotHeading == NORTH:
                if y2 >= nextCheckPoint:
                    print("Reached a checkpoint")
                    currentState = STANDBY
            elif robotHeading == SOUTH:
                if y2 <= nextCheckPoint:
                    print("Reached a checkpoint")
                    currentState = STANDBY


        elif currentState == UTURNLEFT:
            # Sets velocity of the robot according to the calculated solutions above
            rightV = NOMINAL_VELOCITY
            leftV = NOMINAL_VELOCITY
            # chuteV = NOMINAL_VELOCITY

            # Use theta value to turn the Robot to the left
            # Initially turn 90 degrees and stop. 
            if theta <= 90 and theta > 25:
                rightV = NOMINAL_VELOCITY / 3
                leftV = -NOMINAL_VELOCITY / 4
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            elif theta <= 25 and theta > 0:
                rightV = NOMINAL_VELOCITY / 3
                leftV = -NOMINAL_VELOCITY / 4
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            elif theta > -45 and theta <= 0:
                rightV = NOMINAL_VELOCITY / 3
                leftV = -NOMINAL_VELOCITY / 4
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
            
            elif theta <= -45 and theta > -70:
                rightV = NOMINAL_VELOCITY / 16
                leftV = -NOMINAL_VELOCITY / 16
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            elif theta <= -70 and theta > -85:
                rightV = NOMINAL_VELOCITY / 32
                leftV = -NOMINAL_VELOCITY / 32
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
                uturnComplete = True

            elif theta <= -85:
                if uturnComplete:
                    uturnComplete = False
                    rightV = 0
                    leftV = 0
                    leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                    rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
                    robotHeading = SOUTH
                    robotNextTurn = RIGHT
                    currentState = STANDBY
                elif not uturnComplete:
                    rightV = NOMINAL_VELOCITY / 3
                    leftV = -NOMINAL_VELOCITY / 4
                    leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                    rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            else: 
                rightV = NOMINAL_VELOCITY / 3
                leftV = -NOMINAL_VELOCITY / 4
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
            
            print("turning left now: angle is %s" % theta)

            
            

        elif currentState == UTURNRIGHT:
            # Sets velocity of the robot according to the calculated solutions above
            rightV = NOMINAL_VELOCITY
            leftV = NOMINAL_VELOCITY
            # chuteV = NOMINAL_VELOCITY

            # Use theta value to turn the Robot to the left
            # Initially turn 90 degrees and stop. 
            if theta >= -90 and theta < -25:
                rightV = -NOMINAL_VELOCITY / 9
                leftV = NOMINAL_VELOCITY / 8
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            elif theta >= -25 and theta < 0:
                rightV = -NOMINAL_VELOCITY / 9
                leftV = NOMINAL_VELOCITY / 8
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            elif theta < 45 and theta >= 0:
                rightV = -NOMINAL_VELOCITY / 9
                leftV = NOMINAL_VELOCITY / 8
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
            
            elif theta >= 45 and theta < 70:
                rightV = -NOMINAL_VELOCITY / 16
                leftV = NOMINAL_VELOCITY / 16
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            elif theta >= 70 and theta < 85:
                rightV = -NOMINAL_VELOCITY / 32
                leftV = NOMINAL_VELOCITY / 32
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
                uturnComplete = True

            elif theta >= 85:
                if uturnComplete:
                    rightV = 0
                    leftV = 0
                    leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                    rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
                    robotHeading = NORTH
                    robotNextTurn = LEFT
                    currentState = STANDBY
                elif not uturnComplete:
                    rightV = -NOMINAL_VELOCITY / 9
                    leftV = NOMINAL_VELOCITY / 8
                    leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                    rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            else:
                rightV = -NOMINAL_VELOCITY / 9
                leftV = NOMINAL_VELOCITY / 8
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            
            print("turning right now: angle is %s" % theta)

        elif currentState == AVOIDLEFT:
            if detectionState_R:
                if avoidanceInit:
                    headingXcoordinate = x1
                    headingXcoordinate2 = x2
                    avoidanceInit = False
                rightV = NOMINAL_VELOCITY 
                leftV = NOMINAL_VELOCITY / 2
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            elif not detectionState_R:
                if robotHeading == NORTH:
                    if x1 >= headingXcoordinate:
                        rightV = NOMINAL_VELOCITY / 2
                        leftV = NOMINAL_VELOCITY
                        leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                        rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

                    else:
                        if x2 <= headingXcoordinate2:
                            print("x1: %s, x2: %s" % (x1, x2))
                            if(abs(x1-x2) <= 0.01):
                                avoidanceInit = True
                                currentState = MOVESTRAIGHT
                            rightV = NOMINAL_VELOCITY 
                            leftV = 0
                            leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                            rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
                elif robotHeading == SOUTH:
                    if x1 <= headingXcoordinate:
                        rightV = NOMINAL_VELOCITY / 2
                        leftV = NOMINAL_VELOCITY
                        leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                        rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

                    else:
                        if x2 >= headingXcoordinate2:
                            print("x1: %s, x2: %s" % (x1, x2))
                            if(abs(x1-x2) <= 0.01):
                                avoidanceInit = True
                                currentState = MOVESTRAIGHT
                            rightV = NOMINAL_VELOCITY
                            leftV = 0
                            leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                            rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
            
        elif currentState == AVOIDRIGHT:
            if detectionState_L:
                if avoidanceInit:
                    headingXcoordinate = x1
                    headingXcoordinate2 = x2
                    avoidanceInit = False
                rightV = NOMINAL_VELOCITY / 2
                leftV = NOMINAL_VELOCITY 
                leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

            elif not detectionState_L:
                if robotHeading == NORTH:
                    if x1 <= headingXcoordinate:
                        rightV = NOMINAL_VELOCITY 
                        leftV = NOMINAL_VELOCITY / 2
                        leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                        rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

                    else:
                        if x2 >= headingXcoordinate2:
                            print("x1: %s, x2: %s" % (x1, x2))
                            if(abs(x1-x2) <= 0.01):
                                avoidanceInit = True
                                currentState = MOVESTRAIGHT
                            rightV = 0
                            leftV = NOMINAL_VELOCITY 
                            leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                            rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)
                
                elif robotHeading == SOUTH:
                    if x1 >= headingXcoordinate:
                        rightV = NOMINAL_VELOCITY 
                        leftV = NOMINAL_VELOCITY / 2
                        leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                        rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)

                else:
                    if x2 <= headingXcoordinate2:
                        print("x1: %s, x2: %s" % (x1, x2))
                        if(abs(x1-x2) <= 0.01):
                            avoidanceInit = True
                            currentState = MOVESTRAIGHT
                        rightV = 0
                        leftV = NOMINAL_VELOCITY 
                        leftSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, leftJoint, leftV, sim.simx_opmode_blocking)
                        rightSensorReturnCode = sim.simxSetJointTargetVelocity(clientID, rightJoint, rightV, sim.simx_opmode_blocking)


        else:
            currentState = STOP
    

    
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
