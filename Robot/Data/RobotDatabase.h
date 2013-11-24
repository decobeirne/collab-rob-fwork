#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file RobotDatabase.h
\brief All data stored on robot.
*/
#ifndef ROBOT_DATABASE_H
#define ROBOT_DATABASE_H

#include "RobotStatus.h"
#include "RobotGroupData.h"
#include "RobotPartnerData.h"
#include "BehaviourData.h"
#include "RobotSensorData.h"
#include "RobotEnvironment.h"

#include "../../Common/Geometry.h"
#include "../../Common/Uncertainty.h"

#ifdef SIMULATION
#include "../../Board/Board.h"
#endif

#ifdef RERUNNING_ROBOT
#include "../../Board/OpenCV/CV.h"
#endif

//! Contains data stored on robot
/*!
Contains may general items such as GeometryConstants that are
also stored in BoardDatabase_.
*/
typedef struct RobotDatabase_
{
	RobotStatus status;
	RobotGroupData groupData;
	BehaviourData behaviourData;
	PartnerData partnerData;
	RobotSensorData sensorData;
	RobotEnvironment environment;
	CommData commData;

	GeometryConstants geometryConstants;
	UncertaintyConstants uncertaintyConstants;
	IkConstants ikConstants;
	CamVectors camVectors;

#ifdef SIMULATION
	BoardDatabase* board;
#endif
#if defined (SIMULATION) || defined (RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
	IplImage *localMapIplImage;
	IplImage *cameraIplImage;
#endif

	FILE *xmlLog;
} RobotDatabase;

//! Constructor
RobotDatabase initRobotDatabase (
#ifdef SIMULATION
								 BoardDatabase *board,
#endif
#if defined (SIMULATION) || defined (RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
								 IplImage *localMapIplImage,
#endif
								 const int index);

//! Destructor
void RobotDatabase_dtor (RobotDatabase *r);



#endif // ifndef
#endif
