
#
# Definitions
#
robotGumstixObjects = \
GumstixControl.o cmu_main.o i2c_main.o serial_sdk.o

robotLinuxObjects = \
main.o Robot.o \
BaseTypes.o BitArray.o Bresenham.o BresenhamImpl.o Geometry.o Image.o List.o Maths.o Point.o RobotCore.o RobotTypes.o Uncertainty.o Vector.o \
CommCore.o MapCore.o \
Backtrack.o BehaviourAdoption.o BehaviourControl.o BehaviourCore.o CloseLoop.o Exploration.o ExplorationImpl.o FollowPath.o GotoExpPt.o GotoExpPtImpl.o Supervision.o \
RobotReadImpl.o RobotWriteImpl.o RobotRead.o RobotWrite.o RobotCommSerialize.o \
BehaviourData.o RobotDatabase.o RobotEnvironment.o RobotGroupData.o RobotPartnerData.o RobotSensorData.o RobotStatus.o \
Actuator.o DestCentric.o Ik.o TargetCentric.o \
CameraProcessing.o MapGridProcessing.o MapIntegration.o MapIntegrationImpl.o MapProcessing.o SensorProcessing.o SimulatedProcessing.o \
ObstacleRecognition.o ObstacleRecognitionCore.o ObstacleRecognitionModel.o \
RobotRecognition.o RobotRecognitionBlobDetection.o RobotRecognitionCore.o RobotRecognitionModel.o RobotRecognitionAccurateEstimation.o RobotRecognitionRoughEstimation.o

boardObjects = \
main.o \
RobotCore.o Board.o \
RobotTypes.o Bresenham.o List.o Image.o BaseTypes.o \
BoardManagement.o BoardDatabase.o BoardMapProcessing.o BoardComm.o \
Uncertainty.o Geometry.o \
BoardGroupData.o BoardSensorData.o BoardEnvironment.o BoardCoalitionData.o \
BoardMapIntegration.o MapCore.o \
CommCore.o RobotCommSerialize.o

# Make sure object files are always rebuilt, as the different executables share most objects but build them all with different flags, etc.
.PHONY: *.o

# Use a soft link to the cross compiler to tidy up the output
# ln -s /root/gumstix-buildroot/build_arm_nofpu/staging_dir/usr/bin/ccache /root/gumstix-buildroot/arm-linux-gcc-ln
# gumstixCC = /root/gumstix-buildroot/build_arm_nofpu/staging_dir/bin/arm-linux-gcc
gumstixCC = /root/gumstix-buildroot/arm-linux-gcc


gumstixFlags = -DROBOT_PLATFORM=2 -DROBOT_INDEX=0 -DROBOT -DIS_GUMSTIX -DIS_MOTOR
#gumstixFlags = -DROBOT_PLATFORM=3 -DROBOT_INDEX=0 -DROBOT -DIS_GUMSTIX -DIS_MOTOR
#gumstixFlags = -DROBOT_PLATFORM=4 -DROBOT_INDEX=0 -DROBOT -DIS_GUMSTIX -DIS_MOTOR

gumstixLinkFlags = --bluetooth -lm

robotFlags = -DROBOT_PLATFORM=2 -DROBOT_INDEX=0 -DROBOT
robotLinkFlags = -lm

boardFlags = -DROBOT_PLATFORM=2 -DROBOT_INDEX=0 -DBOARD
boardLinkFlags = -lm  -lpthread

compileFlags = -Wall -O3
source = ../

gumstix: cc = $(gumstixCC)
robot: cc = gcc
board: cc = gcc

gumstix: compile = $(cc) $(gumstixFlags) $(compileFlags) -c
robot: compile = $(cc) $(robotFlags) $(compileFlags) -c
board: compile = $(cc) $(boardFlags) $(compileFlags) -c

#
# Gumstix exe
#
gumstix: $(robotGumstixObjects) $(robotLinuxObjects)
	@echo Building robot to run on Gumstix...
	@echo Build flags: $(compileFlags) $(gumstixFlags) $(gumstixLinkFlags)
	$(cc) -o gumstix $(robotGumstixObjects) $(robotLinuxObjects) $(compileFlags) $(gumstixFlags) $(gumstixLinkFlags)

#
# Robot Exe
#
robot: $(robotLinuxObjects)
	@echo Building robot...
	@echo Build flags: $(compileFlags) $(robotFlags) $(robotLinkFlags)
	$(cc) -o robot $(robotLinuxObjects) $(compileFlags) $(robotFlags) $(robotLinkFlags)

#
# Board exe
#
board: $(boardObjects)
	@echo Building blackboard agent...
	@echo Build flags: $(compileFlags) $(boardFlags) $(boardLinkFlags)
	$(cc) -o board $(boardObjects) $(compileFlags) $(boardFlags) $(boardLinkFlags)

#
# Clean target
#
cleanAll:
	rm gumstix robot board *.o

cleanGumstix:
	rm gumstix $(robotGumstixObjects) $(robotLinuxObjects)

cleanRobot:
	rm robot $(robotLinuxObjects)

cleanBoard:
	rm board $(boardObjects)

#
# Board main
#
Board.o:
	$(compile) $(source)/Board/Board.c

#
# Board
#
BoardManagement.o:
	$(compile) $(source)/Board/BoardManagement.c

BoardDatabase.o:
	$(compile) $(source)/Board/Data/BoardDatabase.c

BoardMapProcessing.o:
	$(compile) $(source)/Board/Map/BoardMapProcessing.c

BoardComm.o:
	$(compile) $(source)/Board/Comm/BoardComm.c

BoardGroupData.o:
	$(compile) $(source)/Board/Data/BoardGroupData.c

BoardSensorData.o:
	$(compile) $(source)/Board/Data/BoardSensorData.c

BoardEnvironment.o:
	$(compile) $(source)/Board/Data/BoardEnvironment.c

BoardCoalitionData.o:
	$(compile) $(source)/Board/Data/BoardCoalitionData.c

BoardMapIntegration.o:
	$(compile) $(source)/Board/Map/BoardMapIntegration.c

#
# Robot main
#
Robot.o:
	$(compile) $(source)/Robot/Robot.c

GumstixControl.o:
	$(compile) $(source)/Robot/GumstixControl.c

main.o:
	$(compile) $(source)/main.c

#
# Common
#
BaseTypes.o:
	$(compile) $(source)/Common/BaseTypes.c

BitArray.o:
	$(compile) $(source)/Common/BitArray.c

Bresenham.o:
	$(compile) $(source)/Common/Bresenham.c

BresenhamImpl.o:
	$(compile) $(source)/Common/BresenhamImpl.c

Geometry.o:
	$(compile) $(source)/Common/Geometry.c

Image.o:
	$(compile) $(source)/Common/Image.c

List.o:
	$(compile) $(source)/Common/List.c

Maths.o:
	$(compile) $(source)/Common/Maths.c

Point.o:
	$(compile) $(source)/Common/Point.c

RobotCore.o:
	$(compile) $(source)/Common/RobotCore.c

RobotTypes.o:
	$(compile) $(source)/Common/RobotTypes.c
	
Uncertainty.o:
	$(compile) $(source)/Common/Uncertainty.c

Vector.o:
	$(compile) $(source)/Common/Vector.c

CommCore.o:
	$(compile) $(source)/Common/Comm/CommCore.c

MapCore.o:
	$(compile) $(source)/Common/Map/MapCore.c



#
# Robot Data
#
RobotStatus.o:
	$(compile) $(source)/Robot/Data/RobotStatus.c

BehaviourData.o:
	$(compile) $(source)/Robot/Data/BehaviourData.c

RobotEnvironment.o:
	$(compile) $(source)/Robot/Data/RobotEnvironment.c

RobotGroupData.o:
	$(compile) $(source)/Robot/Data/RobotGroupData.c

RobotPartnerData.o:
	$(compile) $(source)/Robot/Data/RobotPartnerData.c

RobotProposalData.o:
	$(compile) $(source)/Robot/Data/RobotProposalData.c

RobotSensorData.o:
	$(compile) $(source)/Robot/Data/RobotSensorData.c

RobotDatabase.o:
	$(compile) $(source)/Robot/Data/RobotDatabase.c

#
# Robot Comm
#
RobotReadImpl.o:
	$(compile) $(source)/Robot/Comm/RobotReadImpl.c

RobotWriteImpl.o:
	$(compile) $(source)/Robot/Comm/RobotWriteImpl.c

RobotRead.o:
	$(compile) $(source)/Robot/Comm/RobotRead.c

RobotWrite.o:
	$(compile) $(source)/Robot/Comm/RobotWrite.c

RobotCommSerialize.o:
	$(compile) $(source)/Robot/Comm/RobotCommSerialize.c

#
# IK
#
Actuator.o:
	$(compile) $(source)/Common/Actuator.c

DestCentric.o:
	$(compile) $(source)/Robot/Ik/DestCentric.c
	
TargetCentric.o:
	$(compile) $(source)/Robot/Ik/TargetCentric.c

Ik.o:
	$(compile) $(source)/Robot/Ik/Ik.c

#
# Gumstix
#
cmu_main.o:
	$(compile) $(source)/Gumstix/cmu_main.c

i2c_main.o:
	$(compile) $(source)/Gumstix/i2c_main.c

serial_sdk.o:
	$(compile) $(source)/Gumstix/serial_sdk.c

#
# Sensors
#
MapIntegration.o:
	$(compile) $(source)/Robot/Sensors/MapIntegration.c

MapIntegrationImpl.o:
	$(compile) $(source)/Robot/Sensors/MapIntegrationImpl.c

MapGridProcessing.o:
	$(compile) $(source)/Robot/Sensors/MapGridProcessing.c

MapProcessing.o:
	$(compile) $(source)/Robot/Sensors/MapProcessing.c

CameraProcessing.o:
	$(compile) $(source)/Robot/Sensors/CameraProcessing.c

SimulatedProcessing.o:
	$(compile) $(source)/Robot/Sensors/SimulatedProcessing.c

SensorProcessing.o:
	$(compile) $(source)/Robot/Sensors/SensorProcessing.c


#
# Robot Vision
#
ObstacleRecognition.o:
	$(compile) $(source)/Robot/RobotVision/ObstacleRecognition.c

ObstacleRecognitionCore.o:
	$(compile) $(source)/Robot/RobotVision/ObstacleRecognitionCore.c

ObstacleRecognitionModel.o:
	$(compile) $(source)/Robot/RobotVision/ObstacleRecognitionModel.c

RobotRecognition.o:
	$(compile) $(source)/Robot/RobotVision/RobotRecognition.c

RobotRecognitionBlobDetection.o:
	$(compile) $(source)/Robot/RobotVision/RobotRecognitionBlobDetection.c

RobotRecognitionCore.o:
	$(compile) $(source)/Robot/RobotVision/RobotRecognitionCore.c

RobotRecognitionModel.o:
	$(compile) $(source)/Robot/RobotVision/RobotRecognitionModel.c

RobotRecognitionAccurateEstimation.o:
	$(compile) $(source)/Robot/RobotVision/RobotRecognitionAccurateEstimation.c

RobotRecognitionRoughEstimation.o:
	$(compile) $(source)/Robot/RobotVision/RobotRecognitionRoughEstimation.c



#
# Behaviour
#
Backtrack.o:
	$(compile) $(source)/Robot/Behaviour/Backtrack.c

BehaviourAdoption.o:
	$(compile) $(source)/Robot/Behaviour/BehaviourAdoption.c

BehaviourControl.o:
	$(compile) $(source)/Robot/Behaviour/BehaviourControl.c

BehaviourCore.o:
	$(compile) $(source)/Robot/Behaviour/BehaviourCore.c

CloseLoop.o:
	$(compile) $(source)/Robot/Behaviour/CloseLoop.c

Exploration.o:
	$(compile) $(source)/Robot/Behaviour/Exploration.c	

ExplorationImpl.o:
	$(compile) $(source)/Robot/Behaviour/ExplorationImpl.c	

FollowPath.o:
	$(compile) $(source)/Robot/Behaviour/FollowPath.c

GotoExpPt.o:
	$(compile) $(source)/Robot/Behaviour/GotoExpPt.c

GotoExpPtImpl.o:
	$(compile) $(source)/Robot/Behaviour/GotoExpPtImpl.c

Supervision.o:
	$(compile) $(source)/Robot/Behaviour/Supervision.c



