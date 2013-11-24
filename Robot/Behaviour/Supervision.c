#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "Supervision.h"
#include "GotoExpPt.h"
#include "Backtrack.h"
#include "../../Common/BitArray.h"


typedef struct SupervisionTemp_
{
	float totalProfit;
	float duration;
} SupervisionTemp;

SupervisionTemp initSupervisionTemp()
{
	SupervisionTemp supTemp;
	supTemp.totalProfit = 0.0f;
	supTemp.duration = 0.0f;
	return supTemp;
}











#if 0
#define MY_INLINE __inline
#else
#define MY_INLINE
#endif


MY_INLINE PointI Supervision_supAreaIdFromCentre (const PointF centre)
{
	PointI pt;
	pt.x = (centre.x - SUP_DIMS / 2) / SUP_DIFF;
	pt.y = (centre.y - SUP_DIMS / 2) / SUP_DIFF;
	return pt;
}

MY_INLINE PointI Supervision_supAreaIdFromCentreI (const PointI centre)
{
	PointI pt;
	pt.x = (centre.x - SUP_DIMS / 2) / SUP_DIFF;
	pt.y = (centre.y - SUP_DIMS / 2) / SUP_DIFF;
	return pt;
}

MY_INLINE float Supervision_getNExpCellsMappedPerIter (const int currentNExpCellsMapped)
{
	/*!
	Value obtained from graphing nExpCellsMapped over iteration in grossAttribWrtAreaMapped.xlsx.
	Recent graph in dir: experiment_sim_1376174318-20130810-Sat-233838__2robs__collab.
	*/
//	float f = 0.58f ? (currentNExpCellsMapped < 70) : 0.1f;
//	float f = (currentNExpCellsMapped < 70) ? 0.58f : (currentNExpCellsMapped < 90 ? 0.2f : 0.05f);
	float f = (currentNExpCellsMapped < 120) ? 0.58f : 0.1f;
	return f;
}

MY_INLINE float Supervision_estAvgStdDevOverLoopClose (const float supStdDev)
{
	/*!
	Value obtained from graphing offsetSession0, etc. in closeLoopScans2.xlsx
	*/
	return (supStdDev + 1.659276455f);
}

__inline float Supervision_estResourcesPerIter()
{
	/*!
	From exploration and gtep tarets in logs. This is estimated resources per iteration.
	*/
	return 20.0f;
}

MY_INLINE float Supervision_getTypicalCloseLoopDuration()
{
	/*!
	Value obtained from closeLoopScans2.xlsx
	*/
	return 119.0f;
}

__inline float Supervision_getSupResourcesPerIter()
{
	/*!
	Had previously been using BATTERY_LOSS_IDLE (0.3f)
	*/
	return 0.2f;
}

MY_INLINE float Supervision_getAllocRatio()
{
//	return (BATTERY_LOSS_IDLE / BATTERY_LOSS_MOVE);
//	return (0.2f / BATTERY_LOSS_MOVE);
	return 0.2f; // Testing
}




































//! Check target against grid of unreachable targets.
int checkUnreachableGridSUPERVISION (RobotDatabase *db, const PointI cellMidpt)
{
	uchar pixelValue;
	PointI p;
	p = PointI_calcCellIndex (cellMidpt, SUP_DIMS, SUP_DIFF);

	if (BitArray_checkElement_pt (
		db->environment.unreachableSupCellGrid,
		p,
		SUP_GRID_DIMS))
	{
		return 0;
	}

	//! \todo Get board to collate list of current supervision targets
	if (0) //1 == List_isElement (&db->environment.currentActiveSupTargets, &cellMidpt, PointI_comparePtrs))
	{
		return 0;
	}

	pixelValue = Image_getPixel_check (
		db->environment.navMap,
		cellMidpt.x,
		cellMidpt.y);

	if (pixelValue == OCCUPIED_TERRAIN ||
		pixelValue == NARROW_OCCUPIED)
	{
		BitArray_setElement_pt (
			db->environment.unreachableSupCellGrid,
			p,
			SUP_GRID_DIMS,
			1);

		return 0;
	}

	return 1;
}

void calcMovesSUPERVISION (
	FILE *xmlLog,
	FollowPathData *followPathData,
	Image *navMap,
	Image *localMap,
	uchar *obstructedCellGrid,
	uchar *unreachableGrid, // May be expGrid, gtepGrid, etc.
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const Pose pose,
	const float stdDev,
	const int index,
#if defined(SIMULATION) || defined(BOARD)
	uchar *boardUnreachableLocalMapGrid,
	BoardSensorData *boardSensorData,
#else
	RobotDatabase *db,
#endif
	ProfitTemp *temp,
	const PointI cellMidpt)
{
	int hasUnreachableGridBeenUpdated;

	temp->dest.dest.loc.x = (float)cellMidpt.x;
	temp->dest.dest.loc.y = (float)cellMidpt.y;
	temp->dest.dest.orient = 0.0f;
//	temp->dest.leeway = SUPERVISION_LEEWAY;
	temp->dest.leeway = 60.0f; // Should be less than SUP_DIFF / 2
//	temp->dest.leeway = 100.0f;
	temp->dest.maxDist = MAX_FLT;
	temp->dest.type = IS_COARSE_NAV_OK;

	BehaviourCore_calcMovesAndPathIfReqd (
		xmlLog,
		followPathData,
		navMap,
		localMap,
		obstructedCellGrid,
		unreachableGrid,
		&hasUnreachableGridBeenUpdated,
		geometryConstants,
		camVectors,
		ikConstants,
		uncertaintyConstants,
		camPOffset,
		pose,
		stdDev,
		index,
		temp,
		SUPERVISION,
		0);
}


//PointI Supervision_supAreaIndexFromCentre (const PointI centre)
//{
//	PointI index;
//	index.x = (centre.x / SUP_DIFF) - 1; // If area centre = 126, then 126/126 = 1, but this is id = 0
//	index.x = max (0, min (index.x, 2)); // Max sup area id = 2, change this is dimensions change
//
//	index.y = (centre.y / SUP_DIFF) - 1;
//	index.y = max (0, min (index.y, 2));
//	return index;
//}

#define SUP_HALF_DIFF (SUP_DIFF / 2)
PointI Supervision_getMostApplicableSupArea (
	const PointF currentLoc)
{
	PointI loc;
	PointI centre;
	loc = PointF_toPointI (currentLoc);

	centre.x = (loc.x / SUP_DIFF) * SUP_DIFF;
	centre.x = (loc.x - centre.x) < SUP_HALF_DIFF ? centre.x : centre.x + SUP_DIFF;
	centre.x = max(SUP_DIFF, centre.x);
	centre.x = min((ENVIR_DIMS - SUP_DIFF), centre.x);

	centre.y = (loc.y / SUP_DIFF) * SUP_DIFF;
	centre.y = (loc.y - centre.y) < SUP_HALF_DIFF ? centre.y : centre.y + SUP_DIFF;
	centre.y = max(SUP_DIFF, centre.y);
	centre.y = min((ENVIR_DIMS - SUP_DIFF), centre.y);

//	return Supervision_supAreaIndexFromCentre (centre);
	return Supervision_supAreaIdFromCentreI (centre);
}

float Supervision_estGross (
	const int nExpCellsMapped,
	const float stdDevAtDest)
{
	float gross, avgStdDev, gain;

	avgStdDev = Supervision_estAvgStdDevOverLoopClose (stdDevAtDest);

	gain = (EXP_AREA * EXP_AREA * 0.7f) * // mapped threshold for expl is actually 0.3
		Supervision_getNExpCellsMappedPerIter (nExpCellsMapped);

	gross = calcMappingProfit (gain, avgStdDev) * Supervision_getAllocRatio();
//	gross = calcMappingProfit (gain, avgStdDev) * 0.2f;
	return gross;
}

float Supervision_estRatio (
	const float gross,
	const float additionalOverhead)
{
	float ratio;

	ratio = gross / (Supervision_getSupResourcesPerIter() + additionalOverhead);
	return ratio;
}

float Supervision_estCurrentCoalitionRatio (
	RobotDatabase *db)
{
	float ratio;
//	PointI area = Supervision_supAreaIndexFromCentre (PointF_toPointI (db->behaviourData.supervision.profit.dest.dest.loc));
	PointI area = Supervision_supAreaIdFromCentre (db->behaviourData.supervision.profit.dest.dest.loc);

	ratio = Supervision_estRatio (
		Supervision_estGross (
			db->environment.supGrid[area.x][area.y],
			db->status.stdDev),
		0.0f);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<EstCurrentCoalitionRatio>expCellsMapped=%d stdDev=%f</EstCurrentCoalitionRatio>\n",
		db->environment.supGrid[area.x][area.y],
		db->status.stdDev);
#endif

	return ratio;
}

void Supervision_calcRatio (
	RobotDatabase *db,
	ProfitTemp *temp,
	SupervisionTemp *supTemp,
	int nExpCellsMapped)
{
	float overhead;

	if (temp->nSteps == MAX_FLT ||
		temp->gainNMapped == 0 ||
		temp->stdDevAtDest >= STD_DEV_MAX)
	{
		temp->ratio = MIN_FLT;
	}
	else
	{
		temp->gross = Supervision_estGross (
			nExpCellsMapped,
			temp->stdDevAtDest);

		// Additional overhead (required to get to supervision loc - idle overhead will be added to this)
		overhead = ((temp->stdDevInc * STD_DEV_COST_COEFF) + (temp->nSteps * BATTERY_LOSS_MOVE)) / Supervision_getTypicalCloseLoopDuration();
		temp->resources = overhead; // Not used in coalitions, but may be useful when post-processing logs

		temp->ratio = Supervision_estRatio (
			temp->gross,
			overhead);
	}
}

//! Calculate profit estimate for target
/*!
Profit is calculated as:
	- Calculate probable duration of coalition - how long it will take a supervisor to map the area.
	- Calculate resources exhausted by adopting this behaviour. Calculate total based on value of resources.
	- Calculate gross profit. Record this as it will be shared with any potential partner.
	- ESTIMATE profit given to other robots or behaviours. The actual expenditure will depend on the bidding process.
	- Calculate profit ratio. This is (gross - expenditure) / resourcesExhausted
*/
void calcRatioSUPERVISION_OLD (RobotDatabase *db, ProfitTemp *temp, SupervisionTemp *supTemp, int nExpCellsToMap)
{
#if 0
	float likelyExpCellsToMap;

	if (temp->nSteps == MAX_FLT ||
		temp->gainNMapped == 0 ||
		temp->stdDevAtDest >= STD_DEV_MAX)
	{
		temp->ratio = MIN_FLT;
	}
	else
	{
		likelyExpCellsToMap = min (nExpCellsToMap * temp->probability, TYPICAL_SUP_N_CELLS_MAPPED);
		supTemp->duration = (likelyExpCellsToMap / (float)(N_CELLS_LOC_MAP)) * AVG_N_STEPS_PER_LOC_MAP * AVG_INC_MOVES_CLOSE_LOOP;
//		supTemp->duration = (likelyExpCellsToMap / (float)(N_CELLS_LOC_MAP)) * AVG_N_STEPS_PER_LOC_MAP * AVG_INC_MOVES_CLOSE_LOOP;
//		temp->resources = (temp->stdDevInc * STD_DEV_COST_COEFF) + (temp->nSteps * BATTERY_LOSS_MOVE) + (supTemp->duration * BATTERY_LOSS_IDLE);
		temp->resources = (temp->stdDevInc * STD_DEV_COST_COEFF) + (temp->nSteps * BATTERY_LOSS_MOVE) + (supTemp->duration * 1.0f);
		temp->gross = likelyExpCellsToMap * temp->probability * temp->gainNMapped;
		temp->gross = calcMappingProfit (temp->gross, temp->stdDevAtDest + AVG_CLOSE_LOOP_STD_DEV);
		supTemp->totalProfit = temp->gross;
		temp->gross *= SUP_ALLOC_PROFIT;
		temp->ratio = (temp->gross - temp->expenditure) / temp->resources;
	}
#endif
#if 0
/*
in current supervision:
have current sup std dev (and have typical avg std dev when submitting) => typical std dev of data getting subd
have n exp cells mapped => ideally want to get typical (gross, resources)

in propd sup session:
have std dev of other sup => avg std dev of data that we would be subing
have n exp cells mapped => gross and resources would be great

=>
check if mapGain, resources can be predicted well from nExpCellsMapped


*/

#endif
}

//! Any other robots doing supervision should be considered a competitor.
void calcTargetProbabilitySUPERVISION (RobotDatabase *db, ProfitTemp *temp, const PointI cellMidpt)
{
	RobotGroupData *groupData = &db->groupData;
	int i, n;
	int isCompeting;

	n = 1;
	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i == db->status.index || SUPERVISION != groupData->robots[i].baseBehaviour)
		{
			continue;
		}

		isCompeting = Geometry_isPtOnSupArea (PointF_toPointI (groupData->robots[i].pose.loc), cellMidpt, ROBOT_ON_LOC_MAP_LEEWAY);
		isCompeting |= PointI_compareValues (PointF_toPointI (groupData->robots[i].target), cellMidpt);
		n += isCompeting;
	}
	temp->probability = 1.0f / n;
}

int Supervision_getItersInCoalition (RobotDatabase *db)
{
	return max (0, db->status.nIterations - db->partnerData.coalitionIter);
}

int Supervision_getNPartners (RobotDatabase *db)
{
	return db->partnerData.supervisionCoalitions.size;
}

int Supervision_isPartnerRobot (
	RobotDatabase *db,
	const int robotId)
{
	ListNode *iter;
	Coalition *coal;

	iter = db->partnerData.supervisionCoalitions.front;
	while (iter)
	{
		coal = (Coalition*)iter->value;
		if (coal->explorer == robotId)
		{
			return 1;
		}

		iter = iter->next;
	}
	return 0;
}



int Supervision_alreadyBeingSupervised(
	RobotDatabase *db, const PointI cellMidpt)
{
	ListNode *iter;
	Coalition *coal;

	iter = db->board->coalitionData.coalitions.front;
	while (iter)
	{
		coal = (Coalition*)iter->value;

		if (PointI_compareValues (coal->area, cellMidpt))
		{
//			printf ("Robot %d sup area %d,%d, already done\n",
//				db->status.index,
//				cellMidpt.x, cellMidpt.y);
			return 1;
		}

		iter = iter->next;
	}
	return 0;
}

extern int GotoExpPt_isMapDataToAjust (RobotDatabase *db);
extern int Supervision_getNGoodGtepTarsInSupArea (RobotDatabase *db, const PointI supArea);

#if 0 // old
void calcProfitSUPERVISION (SupervisionData *supData, RobotDatabase *db)
{
	PointI iter;
	PointI cellMidpt;
	PointI originCellMidpt;
	PointI gridDims;
	int nExpCellsMapped;
	int distBetweenMidpts;
	ProfitTemp temp = initProfitTemp();
	SupervisionTemp supTemp = initSupervisionTemp();

	BehaviourCore_clearProfitData (&supData->profit);

#if 0
	// During testing, once an exploring robot is in a coalition, do not let it make a
	// proposal.
	if (db->partnerData.explorationCoalition.supervisor != -1)
	{
		return;
	}
#endif

#if 1
	if (db->partnerData.explorationCoalition.id != -1)
	{
//		iter = Supervision_supAreaIdFromCentre (db->partnerData.explorationCoalition.pt);
		iter = db->partnerData.explorationCoalition.area;

		if (db->environment.supGrid[iter.x][iter.y] < 70)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<DontCalcSupervision>reason=\"supGrid\" val=%d</DontCalcSupervision>\n", db->environment.supGrid[iter.x][iter.y]);
#endif
			return;
		}
	}
#endif
#if 1
	if (db->partnerData.explorationCoalition.supervisor != -1)
	{
		if (Supervision_getItersInCoalition (db) < 50)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<DontCalcSupervision>reason=\"itersInCoalition\" val=%d</DontCalcSupervision>\n", Supervision_getItersInCoalition (db));
#endif
			return;
		}
	}
#endif
#if 0
	if (GotoExpPt_isMapDataToAjust (db))
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<DontCalcSupervision>reason=\"mapDataToAdjust\" val=%d</DontCalcSupervision>\n", db->sensorData.mapScanList.size);
#endif
		return;
	}
#endif

	originCellMidpt.x = SUP_DIMS/2;
	originCellMidpt.y = SUP_DIMS/2;
	gridDims.x = SUP_GRID_DIMS;
	gridDims.y = SUP_GRID_DIMS;
	distBetweenMidpts = SUP_DIFF;

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<CalcProfit>behaviour=\"SUPERVISION\" originCellMidpt=(%d,%d) dims=(%d,%d) midptDiff=%d</CalcProfit>\n",
		originCellMidpt.x, originCellMidpt.y, gridDims.x, gridDims.y, distBetweenMidpts);
#endif

	temp.gainNMapped = (EXP_AREA * EXP_AREA * 0.6f); // Take mean, as we only store the n unmapped exp cells based on 0.2 threshold
	temp.switchOverhead = 0.0f;
	temp.expenditure = 0.0f; // Supervision is currently implemented as the top of the behaviour chain, so no profit is given to other behaviours.

	for (iter.x = 0; iter.x < gridDims.x; ++iter.x)
	{
		for (iter.y = 0; iter.y < gridDims.y; ++iter.y)
		{
			cellMidpt = calcCellMidpt (iter, distBetweenMidpts, originCellMidpt);

			if (!checkUnreachableGridSUPERVISION (db, cellMidpt))
			{
				continue;
			}

			nExpCellsMapped = db->environment.supGrid[iter.x][iter.y];

			if (nExpCellsMapped > 110)
			{
				continue;
			}

#ifdef SIMULATION
			if (Supervision_alreadyBeingSupervised(db, iter))
			{
				continue;
			}

			if (Supervision_getNGoodGtepTarsInSupArea (db, iter) < 4)
			{
				continue;
			}
#endif

			calcMovesSUPERVISION (
				db->xmlLog,
				&db->behaviourData.followPath,
				db->environment.navMap,
				db->sensorData.localMap,
				db->environment.obstructedCellGrid,
				db->environment.unreachableSupCellGrid,
				&db->geometryConstants,
				&db->camVectors,
				&db->ikConstants,
				&db->uncertaintyConstants,
				CAM_P_OFFSET,
				db->status.pose,
				db->status.stdDev,
				db->status.index,
#if defined(SIMULATION) || defined(BOARD)
				db->board->environment.unreachableLocalMapGrid,
				&db->board->sensorData,
#else
				db,
#endif
				&temp,
				cellMidpt);

			supData->isFollowPathImpossible |= temp.isFollowPathImpossible;
			calcTargetProbabilitySUPERVISION (db, &temp, cellMidpt);
			Supervision_calcRatio (db, &temp, &supTemp, nExpCellsMapped);

			if (temp.ratio > supData->profit.ratio)
			{
				setProfitData (&supData->profit, &temp, nExpCellsMapped);
				supData->proposalStdDev = temp.stdDevAtDest;
				supData->proposalExpCellsMapped = nExpCellsMapped;
				supData->area = iter;
//				supData->proposalTotalProfit = supTemp.totalProfit;
//				supData->proposalDuration = supTemp.duration;
			}
#ifdef PRINT_PROFIT_DETAIL
			BehaviourCore_printBehaviourProfit (
				db->xmlLog, SUPERVISION,
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
				nExpCellsMapped);
			printBehaviourProfit_tail (db->xmlLog, 0);
#endif
		}
	}

	if (supData->profit.gross == MIN_FLT && supData->isFollowPathImpossible)
	{
		db->behaviourData.shouldBacktrack = 1;
	}
#ifdef PRINT_EVENTS
	else
	{
		BehaviourCore_printBehaviourProfit (
			db->xmlLog, SUPERVISION,
			0,
			1,
			&supData->profit.dest,
			supData->profit.ratio,
			supData->profit.gross,
			supData->profit.expenditure,
			supData->profit.resources,
			supData->profit.nSteps,
			supData->profit.gainNMapped,
			supData->profit.stdDevAtDest,
			supData->profit.stdDevInc,
			supData->profit.initialNMapped);
		fprintf (db->xmlLog, " proposalStdDev=%f proposalExpCellsMapped=%d",
//		fprintf (db->xmlLog, " proposalTotalProfit=%f proposalStdDev=%f proposalDuration=%f",
//			db->behaviourData.supervision.proposalTotalProfit,
			db->behaviourData.supervision.proposalStdDev,
//			db->behaviourData.supervision.proposalDuration);
			db->behaviourData.supervision.proposalExpCellsMapped);
		printBehaviourProfit_tail (db->xmlLog, 1);
	}
#endif
}
#endif // old

extern void Supervision_getBestGtepTargets (RobotDatabase *db, const PointI supArea, int *bestTargets, const int n);

extern float ExplorationImpl_estimateProfitRatio (const float proportionLocalMapMapped);
extern float ExplorationImpl_calcProfitRatio (const float proportionLocalMapMapped, const float movesForExpTarget, const float avgStdDev);

extern float GotoExpPtImpl_estimateNSteps (const float proporationSupAreaMapped);

extern float ExplorationImpl_estimateNSteps (const float proportionLocalMapMapped);

extern float ExplorationImpl_updateProportionLocalMapMappedPerTar (const float proportionLocalMapMapped, const float nSteps);

extern float CloseLoop_estNSteps (const float proportionSupAreaMapped);

extern float CloseLoop_estProfit (const float nScans, const float supStdDev);

float Supervision_initialProfitProportionEstimate()
{
	return 0.5f;
}

void calcProfitSUPERVISION (SupervisionData *supData, RobotDatabase *db)
{
	PointI iter;
	PointI cellMidpt;
	PointI originCellMidpt;
	PointI gridDims;
	int nExpCellsMapped;
	int distBetweenMidpts;
	ProfitTemp temp = initProfitTemp();
//	SupervisionTemp supTemp = initSupervisionTemp();

	const int nGtepTargetsToConsider = 4;
	int bestGtepTargets[4];
	int i;
	float nStepsTotal;
	float netProfitTotal;
	float proportionSupAreaMapped;
	float nStepsForExp;
	float proportionLocalMapMapped;
	float movesForExpTarget;
	float ratioForExpTarget;
	float nStepsCloseLoop;
	float closeLoopProfit;
	float profitForSup;
	float avgStdDev;

	BehaviourCore_clearProfitData (&supData->profit);

	// Only letting a robot make a proposal when it is not in a coalition - This should work fine now
	// as robot's can queue coalitions to join
	//
	// Have to make sure that explorationCoalition values are reset when the robot leaves a coalition or
	// reads that it has ended from the board
	//
	// Robot's should leave coalitions once a loop-close has been successfully achieved
	if (db->partnerData.explorationCoalition.supervisor != -1)
	{
		return;
	}



	originCellMidpt.x = SUP_DIMS/2;
	originCellMidpt.y = SUP_DIMS/2;
	gridDims.x = SUP_GRID_DIMS;
	gridDims.y = SUP_GRID_DIMS;
	distBetweenMidpts = SUP_DIFF;





#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<CalcProfit>behaviour=\"SUPERVISION\" originCellMidpt=(%d,%d) dims=(%d,%d) midptDiff=%d</CalcProfit>\n",
		originCellMidpt.x, originCellMidpt.y, gridDims.x, gridDims.y, distBetweenMidpts);
#endif





	temp.gainNMapped = (EXP_AREA * EXP_AREA * 0.6f); // Take mean, as we only store the n unmapped exp cells based on 0.2 threshold
	temp.switchOverhead = 0.0f;
	temp.expenditure = 0.0f; // Supervision is currently implemented as the top of the behaviour chain, so no profit is given to other behaviours.









	for (iter.x = 0; iter.x < gridDims.x; ++iter.x)
	{
		for (iter.y = 0; iter.y < gridDims.y; ++iter.y)
		{
			cellMidpt = calcCellMidpt (iter, distBetweenMidpts, originCellMidpt);

			if (!checkUnreachableGridSUPERVISION (db, cellMidpt))
			{
				continue;
			}

#ifdef SIMULATION
			if (Supervision_alreadyBeingSupervised(db, iter))
			{
				continue;
			}
#endif

			nExpCellsMapped = db->environment.supGrid[iter.x][iter.y];

			// Look at grid of 9 corresponding GTEP targets. Assume that a loop-close will
			// cover 3 of these.
			Supervision_getBestGtepTargets (db, iter, bestGtepTargets, nGtepTargetsToConsider);
			if (bestGtepTargets[nGtepTargetsToConsider - 1] > 15)
			{
				continue;
			}

			// Estimate moves that the supervisor itself will have to make
			calcMovesSUPERVISION (
				db->xmlLog,
				&db->behaviourData.followPath,
				db->environment.navMap,
				db->sensorData.localMap,
				db->environment.obstructedCellGrid,
				db->environment.unreachableSupCellGrid,
				&db->geometryConstants,
				&db->camVectors,
				&db->ikConstants,
				&db->uncertaintyConstants,
				CAM_P_OFFSET,
				db->status.pose,
				db->status.stdDev,
				db->status.index,
#if defined(SIMULATION) || defined(BOARD)
				db->board->environment.unreachableLocalMapGrid,
				&db->board->sensorData,
#else
				db,
#endif
				&temp,
				cellMidpt);

			supData->isFollowPathImpossible |= temp.isFollowPathImpossible;

			if (temp.nSteps == MAX_FLT ||
				temp.stdDevAtDest >= STD_DEV_MAX)
			{
				continue;
			}

			// Calculate total profit
			proportionSupAreaMapped = (float)nExpCellsMapped / N_CELLS_SUP_AREA;
			nStepsTotal = 0.0f;
			nStepsForExp = 0.0f; // For loop-close, it is assumed that exp scans only will be benefitial
			netProfitTotal = 0.0f;

			avgStdDev = Supervision_estAvgStdDevOverLoopClose (temp.stdDevAtDest);

			// Consider cost of navigating between good GTEP targets based on the proportion
			// of the sup area explored
			for (i = 0; i < nGtepTargetsToConsider; ++i)
			{
				proportionLocalMapMapped = (float)bestGtepTargets[i] / N_CELLS_LOC_MAP;

				// N steps graphed against proportion of sup area mapped
				nStepsTotal += GotoExpPtImpl_estimateNSteps (proportionSupAreaMapped);

				do
				{
					movesForExpTarget = ExplorationImpl_estimateNSteps (proportionLocalMapMapped);
					nStepsTotal += movesForExpTarget;
					nStepsForExp += movesForExpTarget;

//					ratioForExpTarget = ExplorationImpl_estimateProfitRatio (proportionLocalMapMapped);
					ratioForExpTarget = ExplorationImpl_calcProfitRatio (proportionLocalMapMapped, movesForExpTarget, avgStdDev);

					netProfitTotal += (ratioForExpTarget * movesForExpTarget);

					proportionLocalMapMapped = ExplorationImpl_updateProportionLocalMapMappedPerTar (proportionLocalMapMapped, movesForExpTarget);
				}
				while (proportionLocalMapMapped < 0.65);
			}

			// Estimate close loop steps based on supervision area
			{
				nStepsCloseLoop = CloseLoop_estNSteps (proportionSupAreaMapped);

				closeLoopProfit = CloseLoop_estProfit (nStepsForExp, temp.stdDevAtDest); // Assumed that EXPLORATION targets only will have map gain

				nStepsTotal += nStepsCloseLoop;
				profitForSup = closeLoopProfit * Supervision_initialProfitProportionEstimate();
			}

			// Removed calcTargetProbabilitySUPERVISION, as instead a rule is imposed that robots cannot look at
			// an area currently being supervised by another robot

			// Calc profit ratio for supervisor. The resources will amount to those expended by moving to the loc,
			// and to those expended by standing idle while acting as an artificial landmark.
			//
			// As ratio is calculated per iteration, the total profit and total resources must be averaged over the
			// total duration.
			{
				temp.gross = profitForSup;

				temp.resources = ((temp.stdDevInc * STD_DEV_COST_COEFF) + (temp.nSteps * BATTERY_LOSS_MOVE) + (nStepsTotal * BATTERY_LOSS_IDLE));

				temp.ratio = temp.gross / temp.resources;
			}

			if (temp.ratio > supData->profit.ratio)
			{
				setProfitData (&supData->profit, &temp, nExpCellsMapped);
				supData->proposalStdDev = temp.stdDevAtDest;
//				supData->proposalExpCellsMapped = nExpCellsMapped;
				supData->area = iter;
				supData->estCloseLoopProfit = closeLoopProfit;
//				supData->proposalTotalProfit = supTemp.totalProfit;
//				supData->proposalDuration = supTemp.duration;
			}
#ifdef PRINT_PROFIT_DETAIL
			BehaviourCore_printBehaviourProfit (
				db->xmlLog, SUPERVISION,
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
				nExpCellsMapped);
			printBehaviourProfit_tail (db->xmlLog, 0);
#endif
		}
	}

	if (supData->profit.gross == MIN_FLT && supData->isFollowPathImpossible)
	{
		db->behaviourData.shouldBacktrack = 1;
	}
#ifdef PRINT_EVENTS
	else
	{
		BehaviourCore_printBehaviourProfit (
			db->xmlLog, SUPERVISION,
			0,
			1,
			&supData->profit.dest,
			supData->profit.ratio,
			supData->profit.gross,
			supData->profit.expenditure,
			supData->profit.resources,
			supData->profit.nSteps,
			supData->profit.gainNMapped,
			supData->profit.stdDevAtDest,
			supData->profit.stdDevInc,
			supData->profit.initialNMapped);
//		fprintf (db->xmlLog, " proposalStdDev=%f proposalExpCellsMapped=%d",
		fprintf (db->xmlLog, " proposalStdDev=%f",
//		fprintf (db->xmlLog, " proposalTotalProfit=%f proposalStdDev=%f proposalDuration=%f",
//			db->behaviourData.supervision.proposalTotalProfit,
			db->behaviourData.supervision.proposalStdDev);
//			db->behaviourData.supervision.proposalDuration);
//			db->behaviourData.supervision.proposalExpCellsMapped);
		printBehaviourProfit_tail (db->xmlLog, 1);
	}
#endif
}

void adoptSUPERVISION (RobotDatabase *db)
{
	PointF pt;

	List_clear (&db->behaviourData.stack, 0);
	List_pushValue (&db->behaviourData.stack, (BasicBehaviourData*)&db->behaviourData.supervision);
	db->behaviourData.behaviour = SUPERVISION;
	db->behaviourData.baseBehaviour = SUPERVISION;

//	db->behaviourData.supervision.currentCoalRatio = db->behaviourData.supervision.profit.ratio;
	db->behaviourData.supervision.isAtFinalDest = 0;
	db->behaviourData.supervision.arrivedAtDest = 0;
	db->behaviourData.supervision.wasPropSuccessful = 1;
	db->behaviourData.supervision.profit.dest.type |= IS_NEW_DEST;
	db->behaviourData.targetIndex = db->behaviourData.targetCounter;
	++db->behaviourData.targetCounter;

	db->partnerData.coalitionIter = db->status.nIterations;

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<AdoptBehaviour>behaviour=\"SUPERVISION\" targetIndex=%d</AdoptBehaviour>\n",
		db->behaviourData.targetIndex);
#endif


	pt.x = db->status.pose.loc.x - db->environment.navMap->orig.x;
	pt.y = db->status.pose.loc.y - db->environment.navMap->orig.y;
//	if (FREE_TERRAIN != Image_getPixel_dontCheck (db->environment.navMap, db->status.pose.loc.x, db->status.pose.loc.y))
	if (FREE_TERRAIN != Image_getPixel_dontCheck (db->environment.navMap, pt.x, pt.y))
	{
		fprintf (db->xmlLog, "<AdoptBacktrackBeforeSupervision />\n");
		adoptBacktrack (db);
		return;
	}

}

extern void RobotWrite_supervisorAtFinalDest (RobotDatabase *db);

void isAtDestSUPERVISION (RobotDatabase *db)
{
	if (!db->behaviourData.supervision.isAtFinalDest)
	{
		RobotWrite_supervisorAtFinalDest (db);
	}
	db->behaviourData.supervision.isAtFinalDest = 1;
	db->behaviourData.supervision.arrivedAtDest = 1; // Submit map, in order to avoid false negatives when calculating paths

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<SupervisorAtDest />\n");
#endif
}

extern void adoptAVAILABLE (RobotDatabase *db, const int leaveProvisionalMapData);

extern void addDestToUnreachableList (RobotDatabase *db);

void Supervision_ikFailed (RobotDatabase *db)
{
	//if (db->behaviourData.supervision.nPathTries < 5)
	//{
	//	++db->behaviourData.supervision.nPathTries;
	//}

	addDestToUnreachableList (db);

	// Similar to when an explorer robot adopts GOTO_EXP_PT and leaves a coalition, we must update this here
	Supervision_leaveCoalition (db, 1, "path failed for SUP");

	// If robot was performing loop closing, have to leave provisional data, as it cannot
	// be further processed.
	adoptAVAILABLE (db, 1);


}











































#include "../Comm/RobotRead.h"
#include "../Comm/RobotWrite.h"

void Supervision_setupProposal (RobotDatabase *db, Proposal *prop)
{
	time_t postedTime;
	postedTime = time (&postedTime);

	initProposal_ (prop);

	prop->supervisor = db->status.index;
	prop->area = db->behaviourData.supervision.area;
	prop->stdDev = db->behaviourData.supervision.proposalStdDev;
	prop->postedTime = postedTime;
	prop->estCloseLoopProfit = db->behaviourData.supervision.estCloseLoopProfit;
}

/*!
If currently a supervisor (size > 0), and there are any robots that
we are not in a coalition with.
*/
void Supervision_makeFurtherProposal (
	RobotDatabase *db,
	Proposal *prop)
{
	time_t postedTime;
	Coalition *coal;

	db->behaviourData.supervision.makeProposal = 1;

	postedTime = time (&postedTime);
	initProposal_ (prop);

	coal = (Coalition*)db->partnerData.supervisionCoalitions.front->value;

	prop->supervisor = coal->supervisor;
	prop->stdDev = coal->stdDev;
//	prop->expCellsMapped = db->environment.supGrid[coal->area.x][coal->area.y];
	prop->postedTime = postedTime;
	prop->area = coal->area;
	prop->estCloseLoopProfit = coal->estCloseLoopProfit;
}

PointI Supervision_getCurrentArea (RobotDatabase *db)
{
	Coalition *coal;
	coal = (Coalition*)db->partnerData.supervisionCoalitions.front->value;
	return coal->area;
}

void Supervision_makeProposal (RobotDatabase *db)
{
	Proposal prop;

	if (db->sensorData.isDataPendingInc || db->behaviourData.behaviour == STUCK)
	{
		return;
	}

	if (db->behaviourData.supervision.makeProposal)
	{
		Supervision_setupProposal (db, &prop);
	}
	else if (
		db->behaviourData.behaviour == SUPERVISION &&
		db->partnerData.supervisionCoalitions.size > 0 &&
		db->partnerData.supervisionCoalitions.size < (N_ROBOTS - 1) && 
//		db->partnerData.supervisionCoalitions.size < 1 // disabled for now - need to implement queued coalition to make this work
		db->partnerData.supervisionCoalitions.size < 1
		)
	{
		if (Supervision_getNGoodGtepTarsInSupArea (db, Supervision_getCurrentArea (db)) >= 4)
		{
			// If already a supervisor, will not have entered HighLevelBehaviour (as not AVAILABLE), but
			// can det proposal on the fly based on own state and the state of the environment
			Supervision_makeFurtherProposal (db, &prop);
	#ifdef PRINT_EVENTS
//			fprintf (db->xmlLog, "<MakeFurtherProposal>nSupervisionCoalitions=%d expCellsMapped=%d</MakeFurtherProposal>\n",
			fprintf (db->xmlLog, "<MakeFurtherProposal>nSupervisionCoalitions=%d</MakeFurtherProposal>\n",
				db->partnerData.supervisionCoalitions.size);
//				prop.expCellsMapped);
	#endif
		}
	}

	if (db->behaviourData.supervision.makeProposal)
	{
		db->behaviourData.supervision.makeProposal = 0;
		RobotWrite_writeProposal (db, &prop);
		db->behaviourData.supervision.isProposalMade = 1;
	}
}


extern void RobotRead_readCoalitions (RobotDatabase *db);

void Supervision_checkExplorers (RobotDatabase *db, const int readCoalitions)
{
	if (readCoalitions)
	{
		RobotRead_readCoalitions (db);
	}

	if (db->partnerData.supervisionCoalitions.size)
	{
#ifdef PRINT_PROFIT_DETAIL
		fprintf (db->xmlLog, "<ContinueSupervising />\n");
#endif
	}
	else
	{
#ifdef PRINT_PROFIT_DETAIL
		fprintf (db->xmlLog, "<ExplorersQuit />\n");
#endif
		adoptAVAILABLE (db, 1);
	}
}

PROPOSAL_STATUS Supervision_checkProposalStatus (RobotDatabase *db)
{
	int flags[2];
	List proposalList;
	List bidList;
	ListNode *iter;
	Proposal *proposal;
	Bid *bid;

	RobotRead_readCoalitionFlags (db, flags);

	if (flags[0])
	{
		return BID_SUCCESSFUL;
	}
	else if (flags[1])
	{
		return PROPOSAL_SUCCESSFUL;
	}

	if (db->behaviourData.supervision.isProposalMade)
	{
		db->behaviourData.supervision.isProposalMade = 0;

		proposalList = initList();
		RobotRead_readProposals (db, &proposalList);

		iter = proposalList.front;
		while (iter)
		{
			proposal = (Proposal*)iter->value;

			if (proposal->supervisor == db->status.index)
			{
				List_clear (&proposalList, 1);
				return PROPOSAL_PENDING;
			}

			iter = iter->next;
		}
		List_clear (&proposalList, 1);
		return NO_PENDING_COALITION;
	}
	// Should only get to this point if bid was made.
	else
	{
		EXPLICIT_DEBUG_ASSERT (db->behaviourData.supervision.isBidMade)

		bidList = initList();
		RobotRead_readBids (db, &bidList);

		iter = bidList.front;
		while (iter)
		{
			bid = (Bid*)iter->value;
			if (bid->explorer == db->status.index)
			{
				List_clear (&bidList, 1);
				return BID_PENDING;
			}

			iter = iter->next;
		}
		List_clear (&bidList, 1);
		return NO_PENDING_COALITION;
	}
}







































void Supervision_leaveCoalition (RobotDatabase *db, const int explorerOrSupervisor, const char *reason)
{
	db->partnerData.explorationCoalition.id = -1;
	db->partnerData.explorationCoalition.supervisor = -1;

	if (explorerOrSupervisor)
	{
		List_clear (&db->partnerData.supervisionCoalitions, 1);
	}

	RobotWrite_leaveCoalition (db, explorerOrSupervisor);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<LeaveCoalition>reason=\"%s\" iteration=%d </LeaveCoalition>\n", reason, db->status.nIterations);
#endif
}

//! Determine maximum bid that can be put forward to join coalition.
/*
DELETE ME DEPRECATED

Estimate profit that would be earned by independent exploration over the period of the proposed coalition.
It is not accurate to use the current profit as an average estimate, as profitPerStep will decrease as
accumulated error increases. Therefore calculate average stdDev over the period of the coalition. If the
robot would reach its max allowed stdDev before the end of this period, then the nSteps should be adjusted
accordingly.
*/
#if 0
float Supervision_calcBidSplitProfit (RobotDatabase *db, const ProfitTemp *temp, const Proposal *proposal, const float currentMaxProfit)
{
	float avgStdDevInc, avgProfit, avgCoalProfit, finalStdDev, resources, nSteps, ownRatioProfit, supRatioProfit;

	// Estimate independent profitPerStep over the period of the coalition.
	nSteps = (STD_DEV_MAX - db->status.stdDev) * (1.0f / AVG_STD_DEV_PER_MOVE);
	if (nSteps < temp->nSteps)
	{
		avgStdDevInc = (STD_DEV_MAX - db->status.stdDev) * 0.5f;
	}
	else
	{
		nSteps = temp->nSteps;
		avgStdDevInc = (nSteps * AVG_STD_DEV_PER_MOVE) * 0.5f;
	}
//	avgProfit = currentMaxProfit + PROFIT_PER_MOVE_OVER_STD_DEV_M * avgStdDevInc;
	avgProfit = currentMaxProfit + AVG_PROFIT_RED_PER_STD_DEV * avgStdDevInc;
	avgProfit *= (nSteps * BID_PROFIT);

	// Determine cost incurred with respect to error accumulated if coalition is joined. If the
	// explorer will improve its stdDev at the end of the coalition, then the cost may actually
	// be negative. Assume the explorer will be able to observe the supervisor from the optimum
	// viewing dist.
	finalStdDev = proposal->stdDev + db->uncertaintyConstants.visEst_typicalStdDev;
	resources = ((finalStdDev - db->status.stdDev) * STD_DEV_COST_COEFF) + (temp->nSteps * BATTERY_LOSS_MOVE) + temp->switchOverhead;

	ownRatioProfit = resources / proposal->supervisorResources;
	ownRatioProfit = max (0.1f, min (0.9f, ownRatioProfit));

	avgCoalProfit = ownRatioProfit * proposal->totalProfit - resources;

	if (avgCoalProfit > avgProfit)
	{
		supRatioProfit = 1.0f - ownRatioProfit;
#ifdef PRINT_PROFIT
		fprintf (db->xmlLog, "<CalculateBid>bid=%f super=%d ownProfitRatio=%f totalGross=%f currentMaxIndProfit=%f avgMaxIndProfit=%f avgCoalProfit=%f nSteps=%f nExpSteps=%f finalStdDev=%f resources=%f iteration=%d</CalculateBid>\n",
			supRatioProfit, proposal->supervisor, ownRatioProfit, proposal->totalProfit,
			currentMaxProfit, avgProfit, avgCoalProfit,
			temp->nSteps, proposal->duration, finalStdDev, resources, db->status.nIterations);
#endif
		return supRatioProfit;
	}
	else
	{
		return 0.0f;
	}
}
#endif

#define VERBOSE_PROPOSAL_ASSESSMENT 1
float Supervision_calcPartnerProfitRatio (
#if VERBOSE_PROPOSAL_ASSESSMENT
	FILE *xmlLog,
#endif
	const float nSteps,
	const float stdDev,
	const int expCellsMapped)
//	const Proposal *proposal)
{
	float sd, mg, partnerProfitRatio, overhead;

	overhead = 0.0f;
	if (nSteps)
	{
		overhead = nSteps * Supervision_estResourcesPerIter();
		overhead /= Supervision_getTypicalCloseLoopDuration(); // Rough, but conservative estimate of coalition duration
	}

//	sd = Supervision_estAvgStdDevOverLoopClose (proposal->stdDev);
//	mg = Supervision_getNExpCellsMappedPerIter (proposal->expCellsMapped);
	sd = Supervision_estAvgStdDevOverLoopClose (stdDev);
	mg = Supervision_getNExpCellsMappedPerIter (expCellsMapped);

//	mg *= (EXP_AREA * EXP_AREA);
	mg *= (EXP_AREA * EXP_AREA * 0.7f); // mapped threshold for expl is actually 0.3, but go with avg
	partnerProfitRatio = calcMappingProfit (mg, sd);
	partnerProfitRatio *= (1.0f - Supervision_getAllocRatio());
	partnerProfitRatio = partnerProfitRatio / (Supervision_estResourcesPerIter() + overhead);

#if VERBOSE_PROPOSAL_ASSESSMENT
	if (xmlLog)
		fprintf (xmlLog, "<CalcPartnerProfitRatio>propStdDev=%f propExpCellsMapped=%d avgStdDev=%f avgMapGain=%f nSteps=%f overhead=%f ratio=%f</CalcPartnerProfitRatio>\n",
	//		proposal->stdDev, proposal->expCellsMapped,
			stdDev, expCellsMapped,
			sd, mg, nSteps, overhead, partnerProfitRatio);
#endif

	return partnerProfitRatio;
}

void Supervision_calcMovesForProposal (
	FILE *xmlLog,
	FollowPathData *followPathData,
	Image *navMap,
	Image *localMap,
	uchar *obstructedCellGrid,
	uchar *unreachableGrid,
	int *hasUnreachableLocalMapGridBeenUpdated,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const Pose pose,
	const float stdDev,
	const int index,
#if defined(SIMULATION) || defined(BOARD)
	uchar *boardUnreachableLocalMapGrid,
	BoardSensorData *boardSensorData,
#else
	RobotDatabase *db,
#endif
	ProfitTemp *temp,
	const PointF pt)
{
	temp->dest.dest.loc = pt;
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
		NULL,
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
		CLOSE_LOOP,
		0);
}

int Supervision_getNGoodGtepTarsInSupArea (RobotDatabase *db, const PointI supArea)
{
	const int threshold = 18;
	int n;
	PointI pt;
	pt.x = supArea.x * 3;
	pt.y = supArea.y * 3;

	n = 0;
	n += (db->environment.localMapGrid[pt.x    ][pt.y    ] < threshold);
	n += (db->environment.localMapGrid[pt.x    ][pt.y + 2] < threshold);
	n += (db->environment.localMapGrid[pt.x    ][pt.y + 4] < threshold);
	n += (db->environment.localMapGrid[pt.x + 2][pt.y    ] < threshold);
	n += (db->environment.localMapGrid[pt.x + 2][pt.y + 2] < threshold);
	n += (db->environment.localMapGrid[pt.x + 2][pt.y + 4] < threshold);
	n += (db->environment.localMapGrid[pt.x + 4][pt.y    ] < threshold);
	n += (db->environment.localMapGrid[pt.x + 4][pt.y + 2] < threshold);
	n += (db->environment.localMapGrid[pt.x + 4][pt.y + 4] < threshold);
	return n;
}

void Supervision_shiftQueue (int *queue, const int pos, const int n)
{
	int i;

	for (i = n - 1; i > pos; --i)
	{
		queue[i] = queue[i - 1];
	}
}

void Supervision_insertQueue (int *queue, const int value, const int n)
{
	int i;
	for (i = 0; i < n; ++i)
	{
		if (value < queue[i])
		{
			Supervision_shiftQueue (queue, i, n);
			queue[i] = value;
			return;
		}
	}
}

void Supervision_getBestGtepTargets (RobotDatabase *db, const PointI supArea, int *bestTargets, const int n)
{
	PointI pt;
	int i, j;
	__int16 value;

	pt.x = supArea.x * 3;
	pt.y = supArea.y * 3;

	for (i = 0; i < n; ++i)
	{
		bestTargets[i] = MAX_INT32;
	}

	for (i = pt.x; i < pt.x + 5; i += 2) // pt.x, pt.x + 2, pt.x + 4
	{
		for (j = pt.y; j < pt.y + 5; j += 2)
		{
			value = db->environment.localMapGrid[i][j];

			Supervision_insertQueue (bestTargets, value, n);
		}
	}
}

extern float BehaviourAdoption_getBehaviourRatio (
	RobotDatabase *db,
	const BEHAVIOUR behaviour);

#if 0 // old
void Supervision_considerProposals (RobotDatabase *db)
{
	Proposal *proposal;
	ListNode *iter;
	ProfitTemp temp;
	float coalitionProfitRatio;
	float reserve;
	List proposalList;
	int flag;
	PointF pt;
	FollowPathData followPathData = initFollowPathData(); // We can/should not overwrite the actual path, as another behaviour may be using it currently
	int hasUnreachableLocalMapGridBeenUpdated = 0;

	proposalList = initList();

	// Even if robot has no intention of bidding on a proposal, if robots always read proposals, the board will
	// be able to determine when all potential bids are in.
	RobotRead_readProposals (db, &proposalList);

	// Return if stuck *after* reading proposals, so that proposing robots don't spin waiting for us.
	flag = (
		db->behaviourData.behaviour == STUCK ||
		db->behaviourData.behaviour == BACKTRACK);
//		(db->behaviourData.behaviour != AVAILABLE && db->behaviourData.behaviour != SUPERVISION)); // The only behaviours from which a robot can consider proposals

	flag |= (db->partnerData.explorationCoalition.id != -1 && Supervision_getItersInCoalition (db) < 40);

	if (flag)
	{
		RobotWrite_writeProposalsConsidered (db);
		List_clear (&proposalList, 1);
		return;
	}



//	if (db->behaviourData.behaviour != AVAILABLE ||
//		proposalList.size - db->behaviourData.supervision.isProposalMade < 1)
	if (proposalList.size - db->behaviourData.supervision.isProposalMade < 1)
	{
		List_clear (&proposalList, 1);
		RobotWrite_writeProposalsConsidered (db);

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<NoProposalsToConsider />\n");
#endif
		return;
	}

//	temp.switchOverhead = GotoExpPt_calcBehaviourChangeOverhead (
//		db->partnerData.explorationCoalition.id,
//		db->status.index);
	temp.switchOverhead  = 0.0f;

	if (db->behaviourData.behaviour == AVAILABLE)
	{
		EXPLICIT_DEBUG_ASSERT(db->behaviourData.maxProfit != MIN_FLT);
		reserve = db->behaviourData.maxProfit;
//		reserve = (1.5f * db->behaviourData.maxProfit);
	}
	else if (
		db->behaviourData.behaviour == SUPERVISION ||
		(db->behaviourData.baseBehaviour == SUPERVISION))
	{
		reserve = Supervision_estCurrentCoalitionRatio (db);
	}
	else
	{
		if (db->partnerData.explorationCoalition.supervisor == -1)
		{
			reserve = BehaviourAdoption_getBehaviourRatio (db, db->behaviourData.baseBehaviour);
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<ReserveBehaviour>reserve=%f baseBehaviour=%d</ReserveBehaviour>\n", reserve, db->behaviourData.baseBehaviour);
#endif
		}
		else
		{
			if (Supervision_getNGoodGtepTarsInSupArea (db, db->partnerData.explorationCoalition.area) > 2)
			{
				reserve = MAX_FLT;
			}
			else
			{
				reserve = Supervision_calcPartnerProfitRatio (
					db->xmlLog,
					0, // No overhead for coalition we're already a part of
					db->partnerData.explorationCoalition.stdDev,
					db->environment.supGrid[db->partnerData.explorationCoalition.area.x][db->partnerData.explorationCoalition.area.y]);
				reserve *= 1.3f;
			}

#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<ReserveBehaviour>reserve=%f baseBehaviour=%d</ReserveBehaviour>\n", reserve, db->behaviourData.baseBehaviour);
#endif
		}
	}

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<ConsiderProposals>reserveRatio=%f reserveThreshold=%f</ConsiderProposals>\n", reserve, reserve * 1.5);
#endif

	iter = proposalList.front; // Debugging to see why supervisors are quitting their coalitions
	while (iter)
	{
		proposal = (Proposal*)iter->value;

		// Flag means "don't consider"
		flag = (proposal->supervisor == db->status.index);

		// If a supervisor, only allowed to look at proposals from a partner, and only when it only has 1 partner
		flag |= (db->behaviourData.behaviour == SUPERVISION && (!Supervision_isPartnerRobot (db, proposal->supervisor) || (Supervision_getNPartners (db) != 1)));

		if (!flag)
		{
			pt = db->groupData.robots[proposal->supervisor].pose.loc;

			Supervision_calcMovesForProposal (
				db->xmlLog,
//				&db->behaviourData.followPath,
				&followPathData,
				db->environment.navMap,
				db->sensorData.localMap,
				db->environment.obstructedCellGrid,
				NULL,
				&hasUnreachableLocalMapGridBeenUpdated,
				&db->geometryConstants,
				&db->camVectors,
				&db->ikConstants,
				&db->uncertaintyConstants,
				CAM_P_OFFSET,
				db->status.pose,
				db->status.stdDev,
				db->status.index,
#if defined(SIMULATION) || defined(BOARD)
				db->board->environment.unreachableLocalMapGrid,
				&db->board->sensorData,
#else
				db,
#endif
				&temp,
//				proposal->pt); //< Proposal.pt is actually the centre pt of the sup area
				pt); //< The supervisor loc as calculated above

			coalitionProfitRatio = Supervision_calcPartnerProfitRatio (
#if VERBOSE_PROPOSAL_ASSESSMENT
				db->xmlLog,
#endif
				temp.nSteps,
				proposal->stdDev,
				1234.0f); // dummy - deprecated
//				proposal->expCellsMapped);
//				proposal);

//			if (coalitionProfitRatio > (1.1f * db->behaviourData.maxProfit))
			if (coalitionProfitRatio > (1.1f * reserve))
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<ConsiderProposal>event=\"writingBid\" coalitionRatio=%f nSteps=%f</ConsiderProposal>\n",
					coalitionProfitRatio,
					temp.nSteps);
#endif
				RobotWrite_writeBid (db, proposal->supervisor, temp.nSteps);
				db->behaviourData.supervision.isBidMade = 1;
			}
		}

		iter = iter->next;
	}

	RobotWrite_writeProposalsConsidered (db);

	FollowPathData_clear (&followPathData);

	List_clear (&proposalList, 1);
}
#endif // old

int Supervision_canConsiderProposals (RobotDatabase *db)
{
	int canNotConsider = 0;

	// All other conditions should be caught by these, e.g. behaviour is STUCK or BACKTRACK
	int hasASupervisor = db->partnerData.explorationCoalition.id != -1; // Disable this when queued coalitions have been impld
	int hasAQueuedCoalition = db->partnerData.explorationCoalitionQueued.id != -1;
	int isASupervisor = (Supervision_getNPartners (db) != 0);

	canNotConsider |= hasAQueuedCoalition;
	canNotConsider |= isASupervisor;
	canNotConsider |= hasASupervisor;
	canNotConsider |= db->sensorData.isDataPendingInc;

	return (!canNotConsider);
}

float Supervision_getClosestGoodGtepTarget (RobotDatabase *db, const PointI supArea)
{
	float dist;
	float minDist;
	const PointF loc = db->status.pose.loc;
	PointF centre;
	PointI pt;
	int i, j;
	int value;

	pt.x = supArea.x * 3;
	pt.y = supArea.y * 3;

	minDist = MAX_FLT;

	for (i = pt.x; i < pt.x + 5; i += 2) // pt.x, pt.x + 2, pt.x + 4
	{
		for (j = pt.y; j < pt.y + 5; j += 2)
		{
			value = db->environment.localMapGrid[i][j];

			if (value < 15) // Same value for "good enough" used below
			{
				centre.x = (i + 1) * LOC_MAP_DIFF;
				centre.y = (j + 1) * LOC_MAP_DIFF;

				dist = Geometry_dist (loc.x, loc.y, centre.x, centre.y);
				minDist = min (minDist, dist);
			}
		}
	}

	return minDist;
}

void Supervision_calcProfitFromProposal (
	RobotDatabase *db,
	const PointI area,
	const float stdDev,
	ProfitTemp *temp)
{
	const int nGtepTargetsToConsider = 4;
	int bestGtepTargets[4];
	int i;
	float dist;
	float nStepsTotal;
	float netProfitTotal;
	float proportionSupAreaMapped;
	float nStepsForExp;
	float proportionLocalMapMapped;
	float movesForExpTarget;
	float ratioForExpTarget;
	float nStepsCloseLoop;
	float closeLoopProfit;
	int nExpCellsMapped;
	float nSteps;
	float ratio;
	float avgStdDev;

	// Look at best n local maps in the sup area. Make rough estimate of how long it would take to get
	// to the first one
	Supervision_getBestGtepTargets (db, area, bestGtepTargets, nGtepTargetsToConsider);

	// Sanity check
	if (bestGtepTargets[nGtepTargetsToConsider - 1] > 15)
	{
		temp->ratio = MIN_FLT;
		temp->nSteps = MAX_FLT;
		return;
	}

	nExpCellsMapped = db->environment.supGrid[area.x][area.y];
	proportionSupAreaMapped = (float)nExpCellsMapped / N_CELLS_SUP_AREA;

	// Get closest good cell out of these
	dist = Supervision_getClosestGoodGtepTarget (db, area);

	// Estimate n steps as is done for loop-close
	nSteps = (dist / db->geometryConstants.nav_maxDist) * 1.5f;

	avgStdDev = Supervision_estAvgStdDevOverLoopClose (stdDev);

	nStepsTotal = 0.0f;
	nStepsForExp = 0.0f; // For loop-close, it is assumed that exp scans only will be benefitial
	netProfitTotal = 0.0f;

	// Consider cost of navigating between good GTEP targets based on the proportion
	// of the sup area explored
	for (i = 0; i < nGtepTargetsToConsider; ++i)
	{
		proportionLocalMapMapped = (float)bestGtepTargets[i] / N_CELLS_LOC_MAP;

		// Important to distinguish between robots at different distances from the sup area
		if (i == 0)
		{
			nStepsTotal += nSteps;
		}
		else
		{
			// N steps graphed against proportion of sup area mapped
			nStepsTotal += GotoExpPtImpl_estimateNSteps (proportionSupAreaMapped);
		}

		do
		{
			movesForExpTarget = ExplorationImpl_estimateNSteps (proportionLocalMapMapped);
			nStepsTotal += movesForExpTarget;
			nStepsForExp += movesForExpTarget;

//			ratioForExpTarget = ExplorationImpl_estimateProfitRatio (proportionLocalMapMapped);
			ratioForExpTarget = ExplorationImpl_calcProfitRatio (proportionLocalMapMapped, movesForExpTarget, avgStdDev);

			netProfitTotal += (ratioForExpTarget * movesForExpTarget);

			proportionLocalMapMapped = ExplorationImpl_updateProportionLocalMapMappedPerTar (proportionLocalMapMapped, movesForExpTarget);
		}
		while (proportionLocalMapMapped < 0.65f);
	}

	// Estimate close loop steps based on supervision area
	{
		nStepsCloseLoop = CloseLoop_estNSteps (proportionSupAreaMapped);

		closeLoopProfit = CloseLoop_estProfit (nStepsForExp, stdDev); // Assumed that EXPLORATION targets only will have map gain

		nStepsTotal += nStepsCloseLoop;

		netProfitTotal += (closeLoopProfit * (1.0f - Supervision_initialProfitProportionEstimate()));
	}

	ratio = netProfitTotal / nStepsTotal;

	temp->ratio = ratio;
	temp->nSteps = nSteps;
}

void Supervision_considerProposals (RobotDatabase *db)
{
	Proposal *proposal;
	ListNode *iter;
	ProfitTemp temp;
	float reserve;
	List proposalList;
	int hasUnreachableLocalMapGridBeenUpdated = 0;

	List proposalsToConsider = initList();
	List proposalsToCalcReserveFor = initList();

	proposalList = initList();

	// Even if robot has no intention of bidding on a proposal, if robots always read proposals, the board will
	// be able to determine when all potential bids are in.
	RobotRead_readProposals (db, &proposalList);

	if (!Supervision_canConsiderProposals (db))
	{
		RobotWrite_writeProposalsConsidered (db);
		List_clear (&proposalList, 1);

		return;
	}

	fprintf (db->xmlLog, "<ConsiderProposalsPre>iter=%d</ConsiderProposalsPre>\n", db->status.nIterations);

	// Determine profit ratio to compare this against
	//
	// If available, comp to best indep behaviour just calcd
	//
	// If in a coalition, compare it to the current (assumed) proposal from the current
	// supervisor robot. Do not read this coalition while this is still the supervisor
	// though, i.e. make a distinction between urgent and !urgent bids.

	EXPLICIT_DEBUG_ASSERT (db->partnerData.explorationCoalitionQueued.id == -1)

	if (db->partnerData.explorationCoalition.id == -1) // Not in a coalition, bid on everything
	{
		iter = proposalList.front;
		while (iter)
		{
			List_pushValue (&proposalsToConsider, iter->value);

			iter = iter->next;
		}
	}
	else // Currently in a coalition => only bid on urgent proposals
	{
		iter = proposalList.front;
		while (iter)
		{
			proposal = (Proposal*)iter->value;

			if (db->groupData.robots[proposal->supervisor].behaviour == SUPERVISION)
			{
				List_pushValue (&proposalsToCalcReserveFor, iter->value);
			}
			else
			{
				List_pushValue (&proposalsToConsider, iter->value);
			}
			iter = iter->next;
		}
	}

	if (db->partnerData.explorationCoalition.id == -1)
	{
		if (db->behaviourData.behaviour == AVAILABLE)
		{
			reserve = db->behaviourData.maxProfit;

			fprintf (db->xmlLog, "<Reserve>type=\"maxProfit\" value=%f</Reserve>\n", reserve);
		}
		// If doing an indep behaviour, comp to last max profit
		else
		{
			reserve = db->behaviourData.lastMaxProfit;

			fprintf (db->xmlLog, "<Reserve>type=\"lastMaxProfit\" value=%f</Reserve>\n", reserve);
		}
	}
	else
	{
		// Fix later... maybe
//		EXPLICIT_DEBUG_ASSERT (proposalsToCalcReserveFor.size > 0) // Should always have at least one from the current supervisor

		reserve = MIN_FLT;

		iter = proposalsToCalcReserveFor.front;
		while (iter)
		{
			proposal = (Proposal*)iter->value;
			if (proposal->supervisor == db->status.index)
			{
				iter = iter->next;
				continue;
			}

			temp = initProfitTemp();

			Supervision_calcProfitFromProposal (
				db,
				proposal->area,
				proposal->stdDev,
				&temp);

			reserve = max (reserve, temp.ratio);

			iter = iter->next;
		}

		fprintf (db->xmlLog, "<Reserve>type=\"proposalFromCurrentSupervisor\" value=%f</Reserve>\n", reserve);
	}

	// Loop through proposals available
	//
	// Assume that we will leave the current coalition when loop-close is done, can accept an offer
	// from the current supervisor then
	//
	// Also, don't look at a prop from any sup unless not in a coalition. Assume these will still be
	// available, and if not, a better prop should be available then anyway
	//
	// To calcaulate a reserve to compare urgent proposals to, look at proposals from all current
	// supervisors first
	iter = proposalsToConsider.front;
	while (iter)
	{
		proposal = (Proposal*)iter->value;
		if (proposal->supervisor == db->status.index)
		{
			iter = iter->next;
			continue;
		}

		temp = initProfitTemp();

		Supervision_calcProfitFromProposal (
			db,
			proposal->area,
			proposal->stdDev,
			&temp);

		if (temp.ratio > (1.1f * reserve))
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<ConsiderProposal>event=\"writingBid\" supervisor=%d coalitionRatio=%f nSteps=%f</ConsiderProposal>\n",
				proposal->supervisor,
				temp.ratio,
				temp.nSteps);
#endif
			RobotWrite_writeBid (db, proposal->supervisor, temp.nSteps);
			db->behaviourData.supervision.isBidMade = 1;
		}
		else
		{
			fprintf (db->xmlLog, "<ConsiderProposal>event=\"dontBid\" supervisor=%d coalitionRatio=%f</ConsiderProposal>\n",
				proposal->supervisor,
				temp.ratio);
		}

		iter = iter->next;
	}

	RobotWrite_writeProposalsConsidered (db);

	List_clear (&proposalList, 1);
	List_clear (&proposalsToConsider, 0);
	List_clear (&proposalsToCalcReserveFor, 0);
}


#if 0
void Supervision_considerProposals_OLD (RobotDatabase *db)
{
	Proposal *proposal;
	ListNode *iter;
	ProfitTemp temp;
	float bid;
	List proposalList;
	int hasUnreachableLocalMapGridBeenUpdated = 0;

	proposalList = initList();

	// Even if robot has no intention of bidding on a proposal, if robots always read proposals, the board will
	// be able to determine when all potential bids are in.
	RobotRead_readProposals (db, &proposalList);

	// Return if stuck *after* reading proposals, so that proposing robots don't spin waiting for us.
	if (db->behaviourData.behaviour == STUCK)
	{
		RobotWrite_writeProposalsConsidered (db);
		List_clear (&proposalList, 1);
		return;
	}

//	if (db->behaviourData.behaviour != AVAILABLE ||
//		proposalList.size - db->behaviourData.supervision.isProposalMade < 1)
	if (proposalList.size - db->behaviourData.supervision.isProposalMade < 1)
	{
		List_clear (&proposalList, 1);
		RobotWrite_writeProposalsConsidered (db);

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<NoProposalsToConsider />\n");
#endif
		return;
	}

	temp.switchOverhead = GotoExpPt_calcBehaviourChangeOverhead (
		db->partnerData.explorationCoalition.id,
		db->status.index);

	iter = proposalList.front;
	while (iter)
	{
		proposal = (Proposal*)iter->value;
		if (proposal->supervisor != db->status.index)
		{
			Supervision_calcMovesForProposal (
				db->xmlLog,
				&db->behaviourData.followPath,
				db->environment.navMap,
				db->sensorData.localMap,
				db->environment.obstructedCellGrid,
				NULL,
				&hasUnreachableLocalMapGridBeenUpdated,
				&db->geometryConstants,
				&db->camVectors,
				&db->ikConstants,
				&db->uncertaintyConstants,
				CAM_P_OFFSET,
				db->status.pose,
				db->status.stdDev,
				db->status.index,
#if defined(SIMULATION) || defined(BOARD)
				db->board->environment.unreachableLocalMapGrid,
				&db->board->sensorData,
#else
				db,
#endif
				&temp,
				proposal->pt);
			temp.nSteps += proposal->duration;

			bid = Supervision_calcBidSplitProfit (db, &temp, proposal, db->behaviourData.maxProfit);
			if (bid > 0.0f)
			{
				RobotWrite_writeBid (db, proposal->supervisor, bid);
				db->behaviourData.supervision.isBidMade = 1;
			}
		}

		iter = iter->next;
	}

	RobotWrite_writeProposalsConsidered (db);

	List_clear (&proposalList, 1);
}
#endif

void Supervision_checkCollaborationState (RobotDatabase *db)
{
	float dist;
	int isInSupervisionArea;
	float distFromEdge;
	PointF centre;

	if (db->partnerData.explorationCoalition.id != -1)
	{
		centre.x = (float)(SUP_DIFF + db->partnerData.explorationCoalition.area.x * SUP_DIFF);
		centre.y = (float)(SUP_DIFF + db->partnerData.explorationCoalition.area.y * SUP_DIFF);

		// Use same leeway as in RobotMapProcessing_isLocalMapCentreOK
//		distFromEdge = (CAM_P_OFFSET + db->geometryConstants.cam_minDist + BASE_MOVE_DELTA_FWD);
		distFromEdge = (CAM_P_OFFSET + db->geometryConstants.cam_minDist);

		isInSupervisionArea = 1;
		dist = fabs (db->status.pose.loc.x - centre.x);
		if (dist > (float)(SUP_DIMS * 0.5f) - distFromEdge)
		{
			isInSupervisionArea = 0;
		}
		dist = fabs (db->status.pose.loc.y - centre.y);
		if (dist > (float)(SUP_DIMS * 0.5f) - distFromEdge)
		{
			isInSupervisionArea = 0;
		}

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<IsInSupArea>res=%d loc=(%f,%f) centre=(%f,%f)</IsInSupArea>\n",
			isInSupervisionArea,
			db->status.pose.loc.x, db->status.pose.loc.y,
			centre.x, centre.y);
#endif

		db->behaviourData.closeLoop.isInSupervisionArea = isInSupervisionArea;
	}
}


















#endif // ifndef BOARD


