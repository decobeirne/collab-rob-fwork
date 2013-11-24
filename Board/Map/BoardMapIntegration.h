#include "../../Common/RobotDefs.h"

#ifndef ROBOT

#ifndef MAP_INTEGRATION_H
#define MAP_INTEGRATION_H

#include "../../Common/Map/MapCore.h"


void BoardMapIntegration_refreshMapArea (BoardDatabase *db,
										 const PointI bl,
										 const PointI tr);

void BoardMapIntegration_wipeProvisionalMapData (
	BoardDatabase *db,
	PointI *bl,
	PointI *tr);


void BoardMapIntegration_incorporateNewMapData (
	BoardDatabase *db,
	PointI *bl,
	PointI *tr);


void BoardMapIntegration_updateLocalMapGridCells (
	BoardDatabase *db,
	const __int16 expGrid[GLOB_EXP_GRID_DIMS][GLOB_EXP_GRID_DIMS],
	__int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS]);

void BoardMapIntegration_displayTerrain (
										Image *image,
										List *obstacleList);


void BoardMapIntegration_updateExpGridCells (BoardDatabase *db,
											const PointI bl,
											const PointI tr);





#endif
#endif