# SYSC4805-Design-Project

## 1.0 INTRODUCTION
The team British Racing Green that consists of three enthusiastic Systems
Engineering students has determined to design a software prototype
autonomous snow blower as an innovative engineering solution to avoid
unnecessary fatalities associated with shovelling snow. This engineering
proposal briefly describes how the software prototype design could satisfy
customer’s requirements by applying engineering principles in a safe manner.

## 2.0 PROJECT CHARTER
### 2.1 Overall Objective
Every winter, there are about 100 people die in the United States by shovelling
the snow and in a six year span of study shows that about 1647 fatalities from
cardiac related injuries caused by snow shovelling [1].

### 2.2 Overall Deliverables
Team british racing green is attempting to tackle the above issue by designing a
robot capable of autonomously clearing snow. The foundation of this robot
design is a list of actions that have been resolved from a list of customer
requirements. These actions will be implemented and studied upon in a computer
simulation model using CoppeliaSim Edu robot simulation program. The time
period of this project will be February 2021-April 2021. By then end of this time
period, a simulation model of the Autonomous Snowblower will be delivered as
well as the necessary documents related to the management of the project.

## 3.0 SCOPE
### 3.1 List of requirements
* When the initiation signal is received, the robot shall enter snow clearing state
* When the signal to halt is received, the robot shall leave snow clearing state
* When in snow clearing state, the robot shall stay within east/west bounds of movement
* When in snow clearing state, the robot shall not exceed the north/south boundaries of movement
* When in snow clearing state, the robot shall detect and differentiate between a snow covered path and a cleared path
* When in snow clearing state, the robot shall differentiate between an obstacle and snow
* When in snow clearing state, the robot shall differentiate between snow that is deep enough for the robot to clear and snow that is deeper than the robot is capable of clearing
* When in snow clearing state, and an obstruction is detected, the robot shall halt clearing snow
* When in snow clearing state, and an obstruction is detected, the robot shall halt movement
* When in snow clearing state, and an obstruction is detected. The robot shall notify the client
* At all times, the robot must be in control of its physical movement.
* Given the depth requirement of snow meets the robots capabilities, the robot shall always clear the snow when asked to do so.
* At all times, the snow that the robot clears, must not be thrown into a path that has already been cleared.
* At all times, the robot must not partake in actions that might cause harm or disrespect to any personal, property, or itself.

### 3.2 List of activities that meet the requirements
* Remote API function that can initiate the robot to enter operating mode
* Remote API function that can instruct the robot to leave operating mode
* Implement a vision and proximity sensor group to detect left and right boundaries of operation. These sensors will do the job sensing the boundary of the path the robot will take during the operation.
* Implement multiple layers of optical and ultrasonic proximity sensors at the front and rear of the robot for obstacle detection and collision avoidance purposes. Each of these sensors are situated deeper into their own heated tubes in order to prevent them from being blocked by clogged or built up snow or ice. Multiple obstacle detection sensor phenomenon
(optical & ultrasonic) is used for each segmented angle to maximize sensor redundancy so the operational safety is enhanced.
* Implement a suitable snow texture in the simulation environment
* Implement a snow texture detection policy within the robot
* Implement a suitable tarmac texture in the simulation environment
* Implement a tarmac texture detection policy within the robot
* The robot is built with a powertrain that consists of two electric revolute joints which could independently drive both of the rear wheels in both forward and reverse directions. The angular velocity of the revolute joints is independently controlled by the system so the orientation and the speed of the robot can be adjusted as required.
* Implement the front wheel pair so they are free to rotate in any direction such that they could support the rear powertrain control and maintain the stability of the structure.
* Implement a rotating blade auger assembly at the front of the robot so it could collect the snow and crack the ice before pushing them into the rotating blade impeller assembly which is located behind the auger assembly. The auger blades and impeller blades are both driven by two independent revolute joints which could be controlled by the system. The auger blades rotate about the left-right horizontal axis and the impeller blades rotate about the front-rear horizontal axis.
* Install a chute assembly at a 30 degrees slant from the vertical axis above the impeller mechanism so the impeller could throw the snow forcefully through it. The chute is driven by a revolute joint so that it could be rotated about the vertical axis between -135 and +135 degrees. A wider chute angle is chosen so the robot could throw the snow equally into a wider area within a specified perimeter. This is to prevent building up a snow hill in the front yard that often causes poor visibility for drivers.
* The tip of the chute is attached with a horizontally rotatable cap that could be driven by a revolute joint so the snow throwing target distance could be controlled by adjusting its angle between +45 and +90 degrees. The farthest target distance is achieved by setting this cap at +45 degrees angle.
* Implement obstruction response from the robot to the client with appropriate error messaging

## 4.0 RESPONSIBILITY
|               Activity/Task                 | Responsibility of |     Aproved By    |
| ------------------------------------------- | ----------------- | ----------------- |
| Draw schematic diagram of proposed end unit | Shan Rameshkanna  | Chukwuka Ihedimbu |
| Design chassis                              | Chukwuka Ihedimbu | Shan Rameshkanna  |
| Design powertrain                           | Shan Rameshkanna  | Shaviyo Marasinghe|
| Design auger & impeller                     | Shan Rameshkanna  | Shaviyo Marasinghe|
| Design chute and cap                        | Shaviyo Marasinghe| Chukwuka Ihedimbu |
| Install multilayer sensors                  | Chukwuka Ihedimbu | Shan Rameshkanna  |
| Program powertrain                          | Shan Rameshkanna  | Shaviyo Marasinghe|
| Program maneuverability                     | Shaviyo Marasinghe| Chukwuka Ihedimbu |
| Program coll. avoidance/obstacle detection  | Shaviyo Marasinghe| Shan Rameshkanna  |
| Snow simulation environment                 | Shaviyo Marasinghe| Chukwuka Ihedimbu |
| Program auger & impeller assembly           | Shan Rameshkanna  | Shaviyo Marasinghe|
| Program chute assembly                      | Chukwuka Ihedimbu | Shan Rameshkanna  |
| Program pathfinding algorithm               | Shaviyo Marasinghe| Chukwuka Ihedimbu |
| Program communication                       | Shaviyo Marasinghe| Chukwuka Ihedimbu |
| Unit testing                                | Shaviyo Marasinghe| Shan Rameshkanna  |
| Ensure operational safety, failsafe mode    | Chukwuka Ihedimbu | Shaviyo Marasinghe|
| Iterative and intrusive test stages         | Shan Rameshkanna  | Chukwuka Ihedimbu |
| Find potential hazards when misused         | Chukwuka Ihedimbu | Shan Rameshkanna  |
