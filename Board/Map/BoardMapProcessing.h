#include "../../Common/RobotDefs.h"

#ifndef ROBOT

/*!
\file BoardMapProcessing.h
\brief Maintain global map.
*/
#ifndef MAP_PROCESSING_H
#define MAP_PROCESSING_H


#include "../Data/BoardDatabase.h"
#include "../../Common/Map/MapCore.h"
#include "../BoardManagement.h"
#include "BoardMapIntegration.h"



void BoardMapProcessing_processMapData (BoardDatabase *db);

void BoardMapProcessing_refreshSim (BoardDatabase *db);


//! Update the boundary grid indicating if map patches contain a boundary or not
/*!
Map patches are considered to contain a boundary if they contain n or more edge pixels. The
boundaries should be drawn as black on a white background
*/
void BoardMapProcessing_updateObstructedGrid (
	BoardDatabase *db,
	Image *navMap);

//! Given the points at which robot data was updated, calculate section of grid to update
/*!
This new version works with submaps incorporated from the robots. If the region list for a
robot has more that one element, then its centre point is examined. The dimensions of 
the submap are then incorporated to determine the bounds of the area to update. The points 
are converted to grid cell indexes and passed to the update function. 
*/
void BoardMapProcessing_updateMapGrid (
	BoardDatabase *db,
	const PointI updateBl,
	const PointI updateTr);


void BoardMapProcessing_compressMapSection (BoardDatabase *db, const PointI *localMapOrigin, FILE *fstream);

int BoardMapProcessing_calculateMappingCertainty (
	BoardDatabase *db,
	const float distance,
	float *certainty);

int BoardMapProcessing_updateMapGrid_submaps (BoardDatabase *db);


int BoardMapProcessing_determineLocalMapGridUpdateBounds (
	BoardDatabase *db,
	PointI *bl,
	PointI *tr,
	const PointI *expGridDims,
	const PointI *localMapGridDims);

int BoardMapProcessing_updateSupervisionGrid (
	BoardDatabase *db,
	const PointI *minPoint,
	const PointI *maxPoint);

void BoardMapProcessing_resetNewDataFlags (BoardDatabase *db);

void BoardMapProcessing_updNavMap (
	BoardDatabase *db,
	Image *map,
	Image *navMapHandle);

void BoardMapProcessing_updateEffectedAreaForRobots (
	BoardDatabase *db,
	const PointI bl,
	const PointI tr);








#endif // ifndef
#endif
