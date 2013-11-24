#include "../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file Robot.h
\brief Autonomous mobile robot.
*/
#ifndef ROBOT_H
#define ROBOT_H

#include "Sensors/SensorProcessing.h"
#include "Sensors/MapProcessing.h"
#include "../Common/Actuator.h"
#include "Behaviour/BehaviourControl.h"
#include "Comm/RobotWrite.h"
#include "Comm/RobotRead.h"

#ifdef SIMULATION
#include "../Board/Data/BoardDatabase.h"
#endif


#ifdef IS_GUMSTIX
#include "../Gumstix/cmu.h"
#endif

//! Robot control logic and state data.
typedef struct Robot_
{
	RobotDatabase db;
} Robot;

//! Constructor
Robot Robot_init (
#ifdef SIMULATION
	BoardDatabase *b,
#endif
#if defined (SIMULATION) || defined (RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
	IplImage *localMapIplImage,
#endif
	const int index);

//!Destructor
void clearRobot (Robot *r);

void Robot_updateState (Robot *r, const int updateVisualisation);

void Robot_printStatus (Robot *r);

void Robot_finalUpdate (Robot *r);

//! Stop robot loop
void Robot_finish (Robot *r);

#ifdef ROBOT
void Robot_start();
#endif


#endif // ifndef ROBOT_H
#endif
