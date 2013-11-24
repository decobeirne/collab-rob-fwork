#include "../../Common/RobotDefs.h"

#ifndef BOARD_DATABASE_H
#define BOARD_DATABASE_H

/*!
\file BoardDatabase.h
\brief All data stored on blackboard.
*/

#ifndef ROBOT

#include "../../Common/Uncertainty.h"
#include "BoardGroupData.h"
#include "BoardSensorData.h"
#include "BoardEnvironment.h"
#include "BoardCoalitionData.h"

//! All data stored on the blackboard
typedef struct BoardDatabase_
{
	BoardGroupData groupData;
	BoardSensorData sensorData;
	BoardEnvironment environment;
	BoardCoalitionData coalitionData;
	CommData commData;

	UncertaintyConstants uncertaintyConstants; //!< \todo Consider removing this from board
//	GeometryConstants geometryConstants;
//	IkConstants ikConstants;
//	CamVectors camVectors;

#ifdef IS_LINUX
	pthread_t threads[MAX_N_ROBOTS];
#endif

#ifdef IS_WIN
	IplImage *globMapIplImage;
	IplImage *localMapIplImage;
#endif

#ifdef BOARD
	int updateFlag;
#endif

	FILE *xmlLog;
} BoardDatabase;

//! Constructor
BoardDatabase initBoardDatabase (
#ifdef IS_WIN
	IplImage *globMapIplImage,
	IplImage *localMapIplImage
#endif
	);

//! Destructor.
void BoardDatabase_dtor (BoardDatabase *data);

//! Print status of blackboard to log.
void BoardDatabase_printStatus (BoardDatabase *data);

void BoardDatabase_printExpGridSection (BoardDatabase *db,
										const PointI bl,
										const PointI tr);


void BoardDatabase_printExpGrid (BoardDatabase *db);

void BoardDatabase_printLocalMapGrid (BoardDatabase *db);

void BoardDatabase_printSupGrid (BoardDatabase *db);

void BoardDatabase_printAllGrids (BoardDatabase *db);

void BoardDatabase_printGlobalMapList (BoardDatabase *db);

void BoardDatabase_printGlobalMap (BoardDatabase *db);

#endif // ifndef ROBOT
#endif // ifndef BOARD_DATABASE_H
