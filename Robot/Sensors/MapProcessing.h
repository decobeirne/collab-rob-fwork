#include "../../Common/RobotDefs.h"

#ifndef BOARD


#ifndef ROBOT_MAP_PROC_H
#define ROBOT_MAP_PROC_H


#include "MapGridProcessing.h"
#include "MapIntegration.h"


PointI RobotMapProcessing_centreLocalMap2 (const PointI pt);
PointI RobotMapProcessing_centreLocalMap (const PointI pt1, const PointI pt2, const PointI pt3);

void RobotMapProcessing_setLocalMapOrigin (RobotDatabase *db);

void RobotMapProcessing_prepareLocalMap (RobotDatabase *db);

int RobotMapProcessing_isLocalMapCentreOK (RobotDatabase *db, const Pose pose);

void RobotMapProcessing_determineIsLocalMapReady (RobotDatabase *db);


void RobotMapProcessing_resetLocalMap (RobotDatabase *db);

#ifndef IS_GUMSTIX
void RobotMapProcessing_printNavMap (RobotDatabase *db);
#endif

#if defined (SIMULATION) || defined (RERUNNING_ROBOT)
void RobotMapProcessing_displayNavMap (Image *navMap,
									 const PointF loc,
									 const schar *winName,
									 int windowSize[4],
									 IplImage *localMapIplImage);
#endif // defined (SIMULATION) || defined (RERUNNING_ROBOT)

void RobotMapProcessing_updateNavMap (RobotDatabase *db);

//! Determine if a navigation map cells is marked as robot-occupied.
int RobotMapProcessing_isMarkedAsRobot (
	const uchar val);

//! Mark single robot on the nav map
void RobotMapProcessing_incRobotIntoNavMap (
	RobotDatabase *db,
	const int index,
	const PointI localMapCentre,
	const PointI localMapOrig,
	Image *navMap);

//! Mark locations of other robots as occupied in order to avoid collisions
void RobotMapProcessing_incRobotsIntoNavMap (RobotDatabase *db);

//! Remove (possibly temporarily) occupied pixels corresponding to a robot on the nav map
void RobotMapProcessing_removeRobotFromNavMap (
	RobotDatabase *db,
	const int robotIndex,
	const PointI localMapCentre,
	const PointI localMapOrig,
	Image *navMap);

//! Determine if other robots are located within the local map.
/*!
\return Whether or not nav map should be updated: are there currently other robots, or were there last iteration.
*/
int RobotMapProcessing_checkForRobotsInLocalMap (RobotDatabase *db);

void RobotMapProcessing_updateWholeObstructedGrid (
	RobotDatabase *db,
	Image *navMap);

void RobotMapProcessing_updateObstructedGridAroundRobot (
	RobotDatabase *db,
	Image *navMap,
	const int robotIndex,
	List *cellsToResetToObstd);

void RobotMapProcessing_resetObstdGridCellsAroundRobot (
	RobotDatabase *db,
	List *cellsToResetToObstd);

int RobotMapProcessing_detectNavMapCollision (RobotDatabase *db, const Pose pose, const int obstacleValue);

#endif // ROBOT_MAP_PROC_H
#endif // ifndef BOARD
