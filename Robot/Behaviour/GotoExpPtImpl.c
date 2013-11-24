
#include "GotoExpPtImpl.h"
#include "BehaviourCore.h"
#include "../../Common/Geometry.h"
#include "../../Common/BitArray.h"



#if defined(USE_CLOUD)


void GotoExpPtImpl_allocPayload (GotoExpPtPayload **payload)
{
#ifdef PRINT_DEBUG
	printf (" - GotoExpPtPayload %d\n", sizeof(GotoExpPtPayload));
#endif

//	GotoExpPtPayload *p;
	*payload = (GotoExpPtPayload*)malloc (sizeof (GotoExpPtPayload));
//	p = *payload;
//	Image_ctor (&p->navMap, LOC_MAP_DIMS, LOC_MAP_DIMS, 1);
//	Image_ctor (&p->localMap, LOC_MAP_DIMS, LOC_MAP_DIMS, 1);
}

void GotoExpPtImpl_clearPayload (GotoExpPtPayload *payload)
{
//	Image_dtor (&payload->navMap);
//	Image_dtor (&payload->localMap);
	free (payload);
}


#endif // defined(USE_CLOUD)






/*!
From log files. y = 14x + 7, i.e. when sup area is completely it typically takes approx 7 moves
*/
float GotoExpPtImpl_estimateNSteps (const float proporationSupAreaMapped)
{
	return (14.0f * proporationSupAreaMapped + 7.0f);
}

void GotoExpPtImpl_calcMoves (
	FILE *xmlLog,
	FollowPathData *followPathData,
	const Image *navMap,
	Image *localMap, // Not const as updated (and reset) when calcing gain
	const uchar *obstructedCellGrid,
	uchar *unreachableLocalMapGrid, // Not const as updated, and also later syncd with board
	int *hasUnreachableLocalMapGridBeenUpdated,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const Pose pose,
	const float stdDev,
	const int index,
	ProfitTemp *temp,
	const PointI cellMidpt)
{
	temp->dest.dest.loc.x = (float)cellMidpt.x;
	temp->dest.dest.loc.y = (float)cellMidpt.y;
	temp->dest.dest.orient = 0.0f;
	temp->dest.leeway = GTEP_LEEWAY;
	temp->dest.maxDist = MAX_FLT;
	temp->dest.type = IS_COARSE_NAV_OK;

	BehaviourCore_calcMovesAndPathIfReqd (
		xmlLog,
		followPathData,
		navMap,
		localMap,
		obstructedCellGrid,
		unreachableLocalMapGrid,
		hasUnreachableLocalMapGridBeenUpdated,
		geometryConstants,
		camVectors,
		ikConstants,
		uncertaintyConstants,
		camPOffset,
		pose,
		stdDev,
		index,
		temp,
		GOTO_EXP_PT,
		0);
}

void GotoExpPtImpl_calcTargetProbability (
									   const int index,
									   const RobotData *robots,
									   ProfitTemp *temp,
									   const PointI cellMidpt)
{
	//	RobotGroupData *groupData = &db->groupData;
	int i, n;
	int isCompeting;

	n = 1;
	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i == index || STUCK == robots[i].behaviour || SUPERVISION == robots[i].baseBehaviour)
		{
			continue;
		}

		isCompeting = Geometry_isPtOnLocalMap (PointF_toPointI (robots[i].pose.loc), cellMidpt, ROBOT_ON_LOC_MAP_LEEWAY);
		isCompeting |= (GOTO_EXP_PT == robots[i].baseBehaviour && 0 == PointI_compareValues (PointF_toPointI (robots[i].target), cellMidpt));
		n += isCompeting;
	}
	temp->probability = 1.0f / n;
}

#define AVG_FRACT_EXP_TARS_PER_LOCAL_MAP 0.5f
void GotoExpPtImpl_calcRatio (ProfitTemp *temp, const float nExpCellsToMap)
{
	float expStdDev;

	if (temp->nSteps >= MAX_FLT ||
		temp->gainNMapped == 0.0f ||
		temp->stdDevAtDest >= STD_DEV_MAX)
	{
		temp->ratio = MIN_FLT;
	}
#if 0
	else
	{
		expStdDev = temp->stdDevAtDest + (nExpCellsToMap / (float)N_CELLS_LOC_MAP) * (0.5f * AVG_N_STEPS_PER_LOC_MAP * AVG_STD_DEV_PER_MOVE);
		temp->gross = calcMappingProfit (temp->gainNMapped, expStdDev);
//		temp->gross *= nExpCellsToMap;
		temp->gross *= (nExpCellsToMap * AVG_FRACT_EXP_TARS_PER_LOCAL_MAP);
		temp->gross *= temp->probability;

		// This behaviour will get k*totalGross from exploration
//		temp->gross *= (1.0f - GTEP_ALLOC_PROFIT);
		temp->gross *= GTEP_ALLOC_PROFIT;
		temp->expenditure = 0.0f;
//		temp->resources = (temp->stdDevInc * STD_DEV_COST_COEFF) + (temp->nSteps * BATTERY_LOSS_MOVE) + temp->switchOverhead;
//		temp->ratio = (temp->gross) / temp->resources;
		temp->resources = (temp->stdDevInc * STD_DEV_COST_COEFF) + (temp->nSteps * BATTERY_LOSS_MOVE);
		temp->ratio = ((temp->gross) / temp->resources) - temp->switchOverhead;
	}
#endif
	else
	{
		expStdDev = temp->stdDevAtDest + (nExpCellsToMap / (float)N_CELLS_LOC_MAP) * (0.5f * AVG_N_STEPS_PER_LOC_MAP * AVG_STD_DEV_PER_MOVE);
		temp->gross = calcMappingProfit (temp->gainNMapped, expStdDev);
//		temp->gross *= nExpCellsToMap;
//		temp->gross *= (nExpCellsToMap * AVG_FRACT_EXP_TARS_PER_LOCAL_MAP);
		temp->gross *= (nExpCellsToMap * 0.3f); // was 0.5
		temp->gross *= temp->probability;

		// This behaviour will get k*totalGross from exploration
//		temp->gross *= (1.0f - GTEP_ALLOC_PROFIT);
		temp->gross *= GTEP_ALLOC_PROFIT;
		temp->expenditure = 0.0f;
//		temp->resources = (temp->stdDevInc * STD_DEV_COST_COEFF) + (temp->nSteps * BATTERY_LOSS_MOVE) + temp->switchOverhead;
//		temp->ratio = (temp->gross) / temp->resources;
		temp->resources = (temp->stdDevInc * STD_DEV_COST_COEFF) + (temp->nSteps * BATTERY_LOSS_MOVE);
		temp->ratio = ((temp->gross) / temp->resources) - temp->switchOverhead;
	}


}

//! Check target against grid of unreachable targets.
int GotoExpPtImpl_checkUnreachableGrid (
	uchar *unreachableLocalMapGrid,
	const Image *navMap,
	const List *unreachableIkDests,
	const PointI cellMidpt,
	const PointI localMapCentre)
{
//	uchar pixelValue;
	PointI p;
	p = PointI_calcCellIndex (cellMidpt, LOC_MAP_DIMS, LOC_MAP_DIFF);

	if (BitArray_checkElement_pt (
		unreachableLocalMapGrid,
		p,
		GLOB_LOC_MAP_GRID_DIMS))
	{
		return 0;
	}

	if (abs (cellMidpt.x - localMapCentre.x) <= LOC_MAP_DIFF &&
		abs (cellMidpt.y - localMapCentre.y) <= LOC_MAP_DIFF)
	{
		return 0;
	}

	// Same as in ExplorationImpl_checkUnreachableGrid. Doesn't make sense to check
	// this all the time in the behaviour profit calc loop (and it also makes
	// impling profit calc on the cloud more complicated), therefore move this
	// check to where the navMap is actually updated: 
#if 0
	pixelValue = Image_getPixel_check (
		navMap,
		cellMidpt.x,
		cellMidpt.y);

	// checkMove() is called when calculating IK moves. This checks the LOS between
	// poses against these values.
	if (pixelValue == OCCUPIED_TERRAIN ||
		pixelValue == NARROW_OCCUPIED)
	{
		BitArray_setElement_pt (
			unreachableLocalMapGrid,
			p,
			GLOB_LOC_MAP_GRID_DIMS,
			GLOB_LOC_MAP_GRID_DIMS,
			1);

		return 0;
	}
#endif

	// Check if IK had previously failed for this dest
	if (1 == List_isElement (unreachableIkDests, &cellMidpt, PointI_comparePtrs))
	{
		return 0;
	}

	return 1;
}



































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
	FILE *xmlLog)
{
	PointI iter;
	PointI globalIter;
	PointI cellMidpt;
//	PointI originCellMidpt;
//	PointI gridDims;
	PointI localMapCentre;
	int distBetweenMidpts;
	int nExpCellsMapped, nExpCellsToMap;
	ProfitTemp temp = initProfitTemp();
	FollowPathData followPathData = initFollowPathData();

	TIMER_START ("gtepProfitImpl")

#ifdef PRINT_EVENTS
	fprintf (xmlLog, "<CalcProfitGOTO_EXP_PT>\n");
#endif

	localMapCentre.x = navMap->orig.x + LOC_MAP_DIMS/2;
	localMapCentre.y = navMap->orig.y + LOC_MAP_DIMS/2;

//	originCellMidpt.x = originCellMidpt.y = (LOC_MAP_DIMS / 2.0f); //
//	gridDims.x = GLOB_LOC_MAP_GRID_DIMS; //
//	gridDims.y = GLOB_LOC_MAP_GRID_DIMS; //
	distBetweenMidpts = LOC_MAP_DIFF;

//	temp.switchOverhead = GotoExpPtImpl_calcBehaviourChangeOverhead (explorationCoalition,  ); //
	temp.switchOverhead = behaviourChangeOverhead;
//	temp.gainNMapped = (EXP_AREA * EXP_AREA * 0.6f); // Only count n unmapped exp cells, not % free in each, => take mean (between completely unexplored and min accepted unexplored)
	temp.gainNMapped = (EXP_AREA * EXP_AREA * 0.5f);

#ifdef PRINT_EVENTS
	fprintf (xmlLog, "<CalcProfit>behaviour=\"GOTO_EXP_PT\" originCellMidpt=(%d,%d) dims=(%d,%d) midptDiff=%d</CalcProfit>\n",
		originCellMidpt.x, originCellMidpt.y, gridDims.x, gridDims.y, distBetweenMidpts);
#endif

	for (iter.x = 0,
		globalIter.x = ((originCellMidpt.x / (LOC_MAP_DIMS / 2)) - 1);
		iter.x < gridDims.x;
		++iter.x,
		++globalIter.x)
	{
		for (iter.y = 0,
			globalIter.y = ((originCellMidpt.y / (LOC_MAP_DIMS / 2)) - 1);
			iter.y < gridDims.y;
			++iter.y,
			++globalIter.y)
		{
			nExpCellsMapped = localMapGrid[globalIter.x][globalIter.y];
			nExpCellsToMap = (N_CELLS_LOC_MAP - nExpCellsMapped);
			cellMidpt = calcCellMidpt (iter, distBetweenMidpts, originCellMidpt);

			if (!GotoExpPtImpl_checkUnreachableGrid (
				unreachableLocalMapGrid,
				navMap,
				unreachableIkDests,
				cellMidpt,
				localMapCentre))
			{
				continue;
			}

			if (nExpCellsToMap < (N_CELLS_LOC_MAP / 2))
			{
				continue;
			}

			// is also passed unreachableLocalMapGrid - see if it does a write or just a read
			GotoExpPtImpl_calcMoves (
				xmlLog,
				&followPathData,
				navMap,
				localMap,
				obstructedCellGrid,
				unreachableLocalMapGrid,
				hasUnreachableLocalMapGridBeenUpdated,
				geometryConstants,
				camVectors,
				ikConstants,
				uncertaintyConstants,
				camPOffset,
				pose,
				stdDev,
				index,
				&temp,
				cellMidpt);
			gotoExpPtData->isFollowPathImpossible |= temp.isFollowPathImpossible;

			// was taking groupData before, now taking robots
			GotoExpPtImpl_calcTargetProbability (index, robots, &temp, cellMidpt);
			if (!considerStdDevInc)
			{
				temp.stdDevInc = 0.0f; // GotoExpPtForCoal
			}
			GotoExpPtImpl_calcRatio (&temp, nExpCellsToMap);
			if (temp.ratio > gotoExpPtData->profit.ratio)
			{
				setProfitData (&gotoExpPtData->profit, &temp, (float)nExpCellsMapped);
			}
#ifdef PRINT_PROFIT_DETAIL
			BehaviourCore_printBehaviourProfit (
				xmlLog, GOTO_EXP_PT,
				0,
				0,
				&temp.dest,
				temp.ratio,
				temp.gross,
				temp.expenditure,
				temp.resources,
				temp.nSteps,
				temp.gainNMapped,
				temp.stdDevAtDest,
				temp.stdDevInc,
				(float)nExpCellsMapped);
			printBehaviourProfit_tail (xmlLog, 0);
#endif
		}
	}

#ifdef PRINT_EVENTS
	fprintf (xmlLog, "</CalcProfitGOTO_EXP_PT>\n");
#endif

	FollowPathData_clear (&followPathData);

	TIMER_STOP (xmlLog, "gtepProfitImpl");
}


