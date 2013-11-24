#include "../../Common/RobotDefs.h"

#if !defined(BOARD)

i'm not using this naymore 

#include "GotoExpPtCollab.h"
#include "BehaviourCore.h"
#include "GotoExpPtImpl.h"
#include "../Comm/RobotWrite.h"
#include "../Comm/RobotRead.h"











void GotoExpPtCollab_calcProfit (GotoExpPtCollabData *g, RobotDatabase *db)
{
	PointI iter;
	PointI cellMidpt;
	PointI originCellMidpt;
	PointI gridDims;
	float nExpCellsMapped, nExpCellsToMap;
	float distBetweenMidpts;
	ProfitTemp temp = initProfitTemp();
	FollowPathData followPathData = initFollowPathData();

	BehaviourCore_clearProfitData (&g->profit);

	if (db->partnerData.explorationCoalition.id == -1/* || Exploration_checkIfExpTarIsValid (db)*/)
	{
		return;
	}

	originCellMidpt.x = db->partnerData.explorationCoalition.pt.x - (SUP_DIMS/2 - LOC_MAP_DIMS/2);
	originCellMidpt.y = db->partnerData.explorationCoalition.pt.y - (SUP_DIMS/2 - LOC_MAP_DIMS/2);
	gridDims.x = 5;
	gridDims.y = 5;
	distBetweenMidpts = LOC_MAP_DIFF;

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<CalcProfit>behaviour=\"GOTO_EXP_PT_COLLAB\" originCellMidpt=(%d,%d) dims=(%d,%d) midptDiff=%f</CalcProfit>\n",
		originCellMidpt.x, originCellMidpt.y, gridDims.x, gridDims.y, distBetweenMidpts);
#endif

	temp.stdDevInc = 0.0f;
	temp.gainNMapped = (EXP_AREA * EXP_AREA * 0.6f); // Only count n unmapped exp cells, not % free in each, => take mean
	temp.switchOverhead = 0.0f;

	for (iter.x = 0; iter.x < gridDims.x; ++iter.x)
	{
		for (iter.y = 0; iter.y < gridDims.y; ++iter.y)
		{
			nExpCellsMapped = (float)db->environment.localMapGrid[iter.x][iter.y];
			nExpCellsToMap = ((float)N_CELLS_LOC_MAP - nExpCellsMapped);
			cellMidpt = calcCellMidpt (iter, distBetweenMidpts, originCellMidpt);

			if (!GotoExpPtImpl_checkUnreachableGrid (
					db->environment.unreachableLocalMapGrid,
					db->environment.navMap,
					cellMidpt,
					db->sensorData.localMapCentre))
			{
				continue;
			}

			// When adopting GOTO_EXP_PT when in a coalition, the map data collected may
			// be adjusted later if the robot is able to improve its location estimate. This 
			// behaviour is adopt such that it does not consider the updated certainty as it
			// is not a certainty that the map data actually will be updated, and GOTO_EXP_PT
			// has very little claim to any of the increased profit. 
			GotoExpPtImpl_calcMoves (
				db->xmlLog,
				&followPathData,
				db->environment.navMap,
				db->sensorData.localMap,
				db->environment.obstructedCellGrid,
				db->environment.unreachableLocalMapGrid,
				hasUnreachableLocalMapGridBeenUpdated,
				&db->geometryConstants,
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
			g->isFollowPathImpossible |= temp.isFollowPathImpossible;

			GotoExpPtImpl_calcTargetProbability (
				db->status.index,
#if (defined(SIMULATION) && defined(USE_CLOUD)) || defined(BOARD)
				&db->board->groupData,
#else
				&db->groupData,
#endif
				&temp,
				cellMidpt);
			GotoExpPtImpl_calcRatio (&temp, nExpCellsToMap);
			if (temp.ratio > g->profit.ratio)
			{
				setProfitData (&g->profit, &temp, nExpCellsMapped);
			}
#ifdef PRINT_PROFIT_DETAIL
			BehaviourCore_printBehaviourProfit (
				db->xmlLog, GOTO_EXP_PT_COLLAB, 0,
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

	if (g->profit.gross == MIN_FLT)
	{
		if (g->isFollowPathImpossible)
		{
			db->behaviourData.shouldBacktrack = 1;
		}
	}
#ifdef PRINT_EVENTS
	else
	{
		BehaviourCore_printBehaviourProfit (
			db->xmlLog, GOTO_EXP_PT_COLLAB, 1,
			&g->profit.dest,
			g->profit.ratio,
			g->profit.gross,
			g->profit.expenditure,
			g->profit.resources,
			g->profit.nSteps,
			g->profit.gainNMapped,
			g->profit.stdDevAtDest,
			g->profit.stdDevInc,
			g->profit.initialNMapped);
		fprintf (db->xmlLog, " potentialGtepSessionId=%d", db->behaviourData.gotoExpPt.nextSessionIndex);
		printBehaviourProfit_tail (db->xmlLog, 1);
	}
#endif

	FollowPathData_clear (&followPathData);
}























#endif // !defined(BOARD)


