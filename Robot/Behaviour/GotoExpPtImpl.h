#include "../../Common/RobotDefs.h"

#ifndef GOTO_EXP_PT_IMPL_H
#define GOTO_EXP_PT_IMPL_H

#include "../Data/BehaviourData.h"
#include "../../Common/Image.h"
#include "../../Common/Vector.h"



#if defined(USE_CLOUD)
void GotoExpPtImpl_allocPayload (GotoExpPtPayload **payload);

void GotoExpPtImpl_clearPayload (
								 GotoExpPtPayload *payload);
#endif // defined(USE_CLOUD)

float GotoExpPtImpl_calcBehaviourChangeOverhead (
	const int explorationCoalition,
	const float stdDev);



//! Loop over grid of local maps and determine potential profit in exploring each
void GotoExpPtImpl_calcProfit(
	const Image *navMap,
	Image *localMap, // Not const as we update it when calcing gain
	const uchar *obstructedCellGrid,
	uchar *unreachableLocalMapGrid, // Not const as it is updated (and also syncd with board)
	int *hasUnreachableLocalMapGridBeenUpdated,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const RobotData *robots,
	const List *unreachableIkDests,
	const Pose pose,
	const float stdDev,
	const float behaviourChangeOverhead, // Different for GotoExpPt and GotoExpPtCollab
	const int considerStdDevInc, // same
	const PointI originCellMidpt, // same
	const PointI gridDims, // same
	const int index,
	const int explorationCoalition,
	const __int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS],
	GotoExpPtData * gotoExpPtData,
	FILE *xmlLog);














#endif // GOTO_EXP_PT_IMPL_H


