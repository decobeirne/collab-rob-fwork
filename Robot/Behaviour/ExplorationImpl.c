



#include "ExplorationImpl.h"
#include "FollowPath.h"
#include "../Sensors/MapGridProcessing.h"
#include "../Data/BehaviourData.h"
#include "../../Common/BitArray.h"
#include "../../Common/Geometry.h"
#include "../../Common/Map/MapCore.h"


ExpTemp initExpTemp()
{
	ExpTemp expTemp;
	expTemp.profitToGtep = 0.0f;
	expTemp.profitToCoal = 0.0f;
	return expTemp;
}


/*!
From expProfitWrtEnvir.xlsx. Estimate n steps per exploration target. y = 2x + 2, i.e.
targets typically take 2 steps when local map is not mapped.
*/
float ExplorationImpl_estimateNSteps (const float proportionLocalMapMapped)
{
	return (3.2f * proportionLocalMapMapped + 2.0f);
}

/*!
From expProfitWrtEnvir.xlsx.
*/
float ExplorationImpl_estimateProfitRatio (const float proportionLocalMapMapped)
{
	return (-4.9f * proportionLocalMapMapped + 5.2f);
}

float ExplorationImpl_estGain (const float proportionLocalMapMapped)
{
	return (-16.0f * proportionLocalMapMapped + 156.0f);
}

float ExplorationImpl_calcProfitRatio (const float proportionLocalMapMapped, const float movesForExpTarget, const float avgStdDev)
{
	float gain = ExplorationImpl_estGain (proportionLocalMapMapped);
	float gross = calcMappingProfit (gain, avgStdDev);
	float ratio = gross / (movesForExpTarget * BATTERY_LOSS_MOVE);
	return ratio;
}

float ExplorationImpl_updateProportionLocalMapMappedPerTar (const float proportionLocalMapMapped, const float nSteps)
{
	return min(1.0f, proportionLocalMapMapped + ((float)nSteps * 0.05f)); // Approx 400 occupancy cells explored per step
}


//! Check target against grid of unreachable targets.
int ExplorationImpl_checkUnreachableGrid (
	const PointI cellMidpt,
	const Image *navMap,
	const Image *localMap,
	const List *unreachableIkTargets,
	const uchar unreachableExpCellGrid[((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1],
	const uchar mappedCellGrid[((EXP_GRID_DIMS*EXP_GRID_DIMS)/8)+1])
{
	PointI p;

	p = PointI_calcExpCellIndex (cellMidpt, EXP_AREA);

	if (BitArray_checkElement_pt (
		unreachableExpCellGrid,
		p,
		GLOB_EXP_GRID_DIMS))
	{
		return 0;
	}

	// This isn't needed here anymore. This logic has been moved to
	// RobotMapProcessing_updateNavMap. This fits better, as nav maps
	// are only updated here, so stupid to keep rechecking them. Also
	// better to keep grid update steps in the one place. Also means
	// that for cloud_calcProfitEXLORATION, we don't have to update this
	// grid.
#if 0
	pixelValue = Image_getPixel_check (
		navMap,
		cellMidpt.x - navMap->orig.x,
		cellMidpt.y - navMap->orig.y);

	// checkMove() is called when calculating IK moves. This checks the LOS between
	// poses against these values.
	if (pixelValue == OCCUPIED_TERRAIN ||
		pixelValue == NARROW_OCCUPIED)
	{
		BitArray_setElement_pt (
			unreachableExpCellGrid,
			p,
			GLOB_EXP_GRID_DIMS,
			GLOB_EXP_GRID_DIMS,
			1);

		return 0;
	}
#endif

	// Check if IK had previously failed for this target
	if (1 == List_isElement (unreachableIkTargets, &cellMidpt, PointI_comparePtrs))
	{
		return 0;
	}

	// Check if already mapped. Grid is relative to local map here
	p = cellMidpt;
	p.x -= localMap->orig.x;
	p.y -= localMap->orig.y;
	p = PointI_calcExpCellIndex (p, EXP_AREA);

	if (BitArray_checkElement_pt (
		mappedCellGrid,
		p,
		EXP_GRID_DIMS))
	{
		return 0;
	}

	return 1;
}

extern float Supervision_getAllocRatio();

//! Calculate gross profit estimate for target
/*!
Calculation of profit takes into account the profit that must be attributed to the robot's 
supervisor, if it is in a coalition, and also to other behaviours, such as GOTO_EXP_PT, if
it is in such a session. If the robot is currently in a supervision session, the submitted
map data may be later updates, so the profit calculate here may not be accurate. It is still
necessary however, as a best guess at the profit the robot will obtain from this target
*/
void ExplorationImpl_calcRatio (const int supervisor,
						   const float explorationCoalitionBid,
						   const int sessionIndex,
						   ProfitTemp *temp,
						   ExpTemp *expTemp)
{
	if (temp->nSteps == MAX_FLT ||
		temp->gainNMapped == 0.0f ||
		temp->stdDevAtDest >= STD_DEV_MAX)
	{
		temp->ratio = MIN_FLT;
	}
	else
	{
		temp->gross = calcMappingProfit (temp->gainNMapped, temp->stdDevAtDest) * temp->probability;

		expTemp->profitToCoal = 0.0f;
		if (supervisor != -1)
		{
			temp->stdDevInc = 0.0f;
//			expTemp->profitToCoal = explorationCoalitionBid * temp->gross;
//			expTemp->profitToCoal = Supervision_getAllocRatio() * temp->gross;
			expTemp->profitToCoal = 0.01f * temp->gross; // Quick fix as Supervision_getAllocRatio was getting an undefined error on BOARD
		}

		expTemp->profitToGtep = 0.0f;
		if (sessionIndex != -1)
		{
			expTemp->profitToGtep = GTEP_ALLOC_PROFIT * temp->gross;
		}

		temp->resources = (temp->stdDevInc * STD_DEV_COST_COEFF) + (temp->nSteps * BATTERY_LOSS_MOVE);
		temp->expenditure = (expTemp->profitToCoal + expTemp->profitToGtep);
		temp->ratio = (temp->gross - temp->expenditure) / temp->resources;
	}
}

void ExplorationImpl_calcTargetProbability (const GeometryConstants *geometryConstants,
									   const Pose pose,
									   const int index,
									   const RobotData *robots,
									   ProfitTemp *temp,
									   const PointI cellMidpt,
									   const float targetDist)
{
	int i;
	float f;
	PointF robotPt;
	float robotDistSqrd;
	PointF neighbourPt;
	float neighbourDistSqrd;
	float minNeighbourDistSqrd;

//	Geometry_ptFromOrient (pose.loc.x, pose.loc.y, &robotPt.x, &robotPt.y, geometryConstants->exp_optDist + CAM_P_OFFSET, pose.orient);
//	Geometry_ptFromOrient (pose.loc.x, pose.loc.y, &robotPt.x, &robotPt.y, SCAN_CENTRE_DIST + CAM_P_OFFSET, pose.orient);
	Geometry_ptFromOrient (pose.loc.x, pose.loc.y, &robotPt.x, &robotPt.y, targetDist, pose.orient);
	robotDistSqrd = Geometry_distSqrd (robotPt.x, robotPt.y, temp->dest.target.x, temp->dest.target.y);
	minNeighbourDistSqrd = MAX_FLT;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i == index || STUCK == robots[i].behaviour)
		{
			continue;
		}

		// Another robot under any behaviour can map this target. Calculate the probability of a
		// neighbour achieving the target first.
		//
		// Limitation: only considering single neighbour robot that is closest to the target. Other robots should have
		// a negligible impact on profit here though.
//		Geometry_ptFromOrient (robots[i].pose.loc.x, robots[i].pose.loc.y, &neighbourPt.x, &neighbourPt.y, geometryConstants->exp_optDist + CAM_P_OFFSET, robots[i].pose.orient);
//		Geometry_ptFromOrient (robots[i].pose.loc.x, robots[i].pose.loc.y, &neighbourPt.x, &neighbourPt.y, SCAN_CENTRE_DIST + CAM_P_OFFSET, robots[i].pose.orient);
		Geometry_ptFromOrient (robots[i].pose.loc.x, robots[i].pose.loc.y, &neighbourPt.x, &neighbourPt.y, targetDist, robots[i].pose.orient);
		neighbourDistSqrd = Geometry_distSqrd (neighbourPt.x, neighbourPt.y, temp->dest.target.x, temp->dest.target.y);
		minNeighbourDistSqrd = min (neighbourDistSqrd, minNeighbourDistSqrd);
	}

	// If a neighbour is further away, it should not achieve the target first.
	// If within d of the target, then the neighbour may share an equal probability of achieving the target
	f = minNeighbourDistSqrd / (minNeighbourDistSqrd + robotDistSqrd);
	if (f > 0.7f)
	{
		f = 1.0f;
	}
	temp->probability = f;
}

void calcMovesEXPLORATION (
	FILE *xmlLog,
	FollowPathData *followPathData,
	const Image *navMap,
	Image *localMap, // Not const as we integrate scans to calculate map gain
	const uchar *obstructedCellGrid,
	uchar *unreachableGrid, // Not const as updated in BehaviourCore_calcMovesAndPathIfReqd->BehaviourCore_updateUnreachableGrid
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const Pose pose,
	const float stdDev,
	const int index,
	ProfitTemp *temp,
	const PointI cellMidpt,
	const float targetDist,
	const int calcGain)
{
	int hasUnreachableLocalMapGridBeenUpdated = 0;

	temp->dest.target.x = (float)cellMidpt.x;
	temp->dest.target.y = (float)cellMidpt.y;
//	temp->dest.leeway = EXPLORATION_LEEWAY;
	temp->dest.leeway = 5.0f;
	temp->dest.maxDist = MAX_FLT;
	temp->dest.targetDist = targetDist;
	temp->dest.type = IS_TARGET_CENTRIC;
	temp->dest.flags = NORMAL_IK;

	BehaviourCore_calcMovesAndPathIfReqd (
		xmlLog,
		followPathData,
		navMap,
		localMap,
		obstructedCellGrid,
		unreachableGrid,
		&hasUnreachableLocalMapGridBeenUpdated,
		geometryConstants,
		camVectors,
		ikConstants,
		uncertaintyConstants,
		camPOffset,
		pose,
		stdDev,
		index,
		temp,
		EXPLORATION,
		calcGain);
}














int ExplorationImpl_otherRobotsTooClose (
	const int index,
	const RobotData *robots,
	const PointI cellMidpt)
{
	PointF pt1;
	PointF pt2;
	int tooClose = 0;
	int i;

	pt2 = PointI_toPointF (cellMidpt);

	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i == index)
		{
			continue;
		}
		pt1 = robots[i].pose.loc;
		if (Geometry_dist (pt1.x, pt1.y, pt2.x, pt2.y) < 42)
		{
			return 1;
		}
	}

	return tooClose;
}


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
	const float targetDist,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	FILE *xmlLog)
{
	PointI iter;
	PointI cellMidpt;
	PointI originCellMidpt;
	PointI gridDims;
	float initialNMapped;
	int distBetweenMidpts;
	ProfitTemp temp = initProfitTemp();
	ExpTemp expTemp = initExpTemp();
	FollowPathData followPathData = initFollowPathData();

	TIMER_START ("explorationProfitImpl")

	originCellMidpt.x = localMap->orig.x + (EXP_AREA / 2);
	originCellMidpt.y = localMap->orig.y + (EXP_AREA / 2);
	gridDims.x = EXP_GRID_DIMS;
	gridDims.y = EXP_GRID_DIMS;
	distBetweenMidpts = EXP_AREA;

#ifdef PRINT_EVENTS
	fprintf (xmlLog, "<CalcProfit>behaviour=\"EXPLORATION\" originCellMidpt=(%d,%d) dims=(%d,%d) midptDiff=%d</CalcProfit>\n",
		originCellMidpt.x, originCellMidpt.y, gridDims.x, gridDims.y, distBetweenMidpts);
#endif

//	temp.dest.flags |= SIMPLE_IK_ONLY;
	temp.switchOverhead = 0.0f;

	for (iter.x = 0; iter.x < gridDims.x; ++iter.x)
	{
		for (iter.y = 0; iter.y < gridDims.y; ++iter.y)
		{
			initialNMapped = (float)expGrid[iter.x][iter.y];
			cellMidpt = calcCellMidpt (iter, distBetweenMidpts, originCellMidpt);

			if (!ExplorationImpl_checkUnreachableGrid (
					cellMidpt,
					navMap,
					localMap,
					unreachableIkTargets,
					unreachableExpCellGrid,
					mappedCellGrid)||
				!MapCore_checkIfCellValid (initialNMapped))
			{
				continue;
			}

#if 1
			if (ExplorationImpl_otherRobotsTooClose (
				index,
				robots,
				cellMidpt))
			{
//				printf ("Robot %d, cell %d,%d is too close\n", index, cellMidpt.x, cellMidpt.y);
				continue;
			}
#endif

			// Exploration is implemented such that it does not consider the certainty
			// and profit associated with independent or collaborative exploration, it
			// just looks at targets and determines the profit of each based on the
			// current state.
			calcMovesEXPLORATION (
				xmlLog,
				&followPathData,
				navMap,
				localMap,
				obstructedCellGrid,
				unreachableExpCellGrid,
				geometryConstants,
				camVectors,
				ikConstants,
				uncertaintyConstants,
				camPOffset,
				pose,
				stdDev,
				index,
				&temp,
				cellMidpt,
//				CAM_P_OFFSET + geometryConstants->exp_optDist,
//				CAM_P_OFFSET + SCAN_CENTRE_DIST,
				targetDist,
#ifdef JUST_LOOK_AT_GAIN_FROM_FINAL_TARGET
				0
#else
				1
#endif
				);

#ifdef JUST_LOOK_AT_GAIN_FROM_FINAL_TARGET
			temp.gainNMapped = max (0.0f, ((EXP_AREA * EXP_AREA) * 0.8f) - initialNMapped);
#endif

			e->isFollowPathImpossible |= temp.isFollowPathImpossible;

			ExplorationImpl_calcTargetProbability (
				geometryConstants,
				pose,
				index,
				robots,
				&temp,
				cellMidpt,
				targetDist);

			ExplorationImpl_calcRatio (
				supervisor,
				explorationCoalitionBid,
				sessionIndex,
				&temp,
				&expTemp);

			if (temp.ratio > e->profit.ratio)
			{
				setProfitData (&e->profit, &temp, initialNMapped);
				e->profitToGtep = expTemp.profitToGtep;
				e->profitToCoal = expTemp.profitToCoal;
			}

#ifdef PRINT_PROFIT_DETAIL
			BehaviourCore_printBehaviourProfit (
				xmlLog,
				EXPLORATION,
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
				initialNMapped);
			fprintf (xmlLog, " profitToGtep=%f profitToCoal=%f gtepSession=%d", expTemp.profitToGtep, expTemp.profitToCoal, sessionIndex/*, db->behaviourData.gotoExpPt.sessionIndex*/);
			printBehaviourProfit_tail (xmlLog, 0);
#endif
		}
	}

	FollowPathData_clear (&followPathData);

	TIMER_STOP (xmlLog, "explorationProfitImpl");
}





