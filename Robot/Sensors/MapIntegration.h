#include "../../Common/RobotDefs.h"

#ifndef BOARD


#ifndef ROBOT_MAP_INTEGRATION_H
#define ROBOT_MAP_INTEGRATION_H

#include "SensorProcessing.h"
#include "../../Common/Map/MapCore.h"
#include "../Data/RobotDatabase.h"

#if defined(MAP_SCANS_HAVE_GRIDS)
int MapIntegration_intBlankScan (
	RobotDatabase *db,
	MapScan *mapScan,
	const float stdDev,
	Image *localMap,
	Image *navMap,
	CamVectors *camVectors,
	const int updateNavMap,
	const int intUnknownCells);

int MapIntegration_intCamScan (
	RobotDatabase *db,
	MapScan *mapScan,
	const float stdDev,
	Image *localMap,
	Image *navMap,
	CamVectors *camVectors,
	const int updateNavMap,
	const int intUnknownCells);

#else // defined(MAP_SCANS_HAVE_GRIDS)

int MapIntegration_intSimScan (
	RobotDatabase *db,
	MapScan *mapScan,
	const float stdDev,
	Image *localMap,
	Image *navMap,
	Image *sim,
	const int updateNavMap);
#endif // defined(MAP_SCANS_HAVE_GRIDS)

//! Integrate new mapped terrain from latest sensor data into local map.
void RobotMapIntegration_integrateMapData (RobotDatabase *db);

//! Update local map with terrain that another robot is assumed to have mapped
/*!
This terrain should be taken into account when determining where to explore,
but should not determine what areas are occupied or free.
*/
int RobotMapProcessing_intAssumedNeighbourScans (RobotDatabase *db);

void RobotMapIntegration_integrateNewMapDataFromBoard (RobotDatabase *db);

LocalMapInfo RobotMapIntegration_redrawLocalMap (RobotDatabase *db);



#endif // ifndef BOARD
#endif // ROBOT_MAP_INTEGRATION_H
