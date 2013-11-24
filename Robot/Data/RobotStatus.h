#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file RobotStatus.h
\brief Data on current robot status.
*/
#ifndef ROBOT_STATUS_H
#define ROBOT_STATUS_H

#include "../../Common/BaseTypes.h"
#include "../../Common/RobotTypes.h"

//! Robot's proprioceptive data
typedef struct RobotStatus_
{
	__int8 isNewPose;
	__int8 index;						//!< Index as assigned by board, starting from 0
	__int16 nIterations;				//!< Number of iterations carried out
	float stdDev;						//!< Robot's pose estimate
	Pose pose;							//!< Pose as estimated from odometry and cooperative localisation
	PointF scanCentrePt;				//!< Central point under supervision
	PointF target;
	__int8 isTimeElapsed;
	PointF actualLocOffset;				//!< Actual location of robot - after error inserted into simulation

	schar locWinName[12];				//!< String for displaying local image
	schar navWinName[12];				//!< String for displaying navigation image

	IKData ikData;						//!< Stores moves to execute.

#if defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
	char rerunning_experDirToRerun[256];
#if defined(RERUNNING_ROBOT)
	FILE *rerunning_file;				//!< Experiment file from which state is being read.
	char *rerunning_buffer;				//!< Buffer into which recorded log is read.
	char *rerunning_startPtr;
	char *rerunning_endPtr;
	int rerunning_atEnd;
	char rerunning_replacedChar;
#endif
#endif // defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)

#ifdef RECORDING_ROBOT
	int objIndex;						//!< Index of object getting serialized out
#endif // ifdef RECORDING_ROBOT
} RobotStatus;

//! Constructor
RobotStatus initRobotStatus (const int index);

//! Destructor
void RobotStatus_dtor (RobotStatus *r);


#endif // ifndef
#endif
