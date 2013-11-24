#include "../../Common/RobotDefs.h"



#ifndef EXPIMPL_H
#define EXPIMPL_H

#include "BehaviourCore.h"
#include "../Data/BehaviourData.h"



typedef struct ExpTemp_
{
	float profitToGtep;
	float profitToCoal;
} ExpTemp;

ExpTemp initExpTemp();






void ExplorationImpl_calcProfit (
	ExplorationData *e,
	const Image *navMap,
	Image *localMap,
	const __int16 expGrid[][EXP_GRID_DIMS],
	const uchar mappedCellGrid[((EXP_GRID_DIMS*EXP_GRID_DIMS)/8)+1],
	const uchar obstructedCellGrid[((NAV_GRID_DIMS*NAV_GRID_DIMS)/8)+1],
	uchar unreachableExpCellGrid[((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1], // Not const on purpose
	const List *unreachableIkTargets,
	const RobotData *robots,
	const Pose pose,
	const float stdDev,
	const int index,
	const int supervisor,
	const float explorationCoalitionBid,
	const int sessionIndex,
	//#if defined(SIMULATION) || defined(BOARD)
	//								  uchar *boardUnreachableLocalMapGrid,
	//								  BoardSensorData *boardSensorData,
	//#else
	//								  RobotDatabase *db,
	//#endif
	const float targetDist,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	FILE *xmlLog);





#endif // ifndef EXPIMPL_H


