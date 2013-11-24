#include "../../Common/RobotDefs.h"

#if !defined(BOARD)


#include "Exploration.h"
#include "ExplorationImpl.h"
#include "BehaviourAdoption.h"
#include "../../Common/BitArray.h"



#ifdef PRINT_PROFIT_DETAIL
#define PRINT_CELL_PROFIT
#endif
extern void estimateCollabStdDev (RobotDatabase *db, ProfitTemp *temp, const PointF target);


#if defined(USE_CLOUD)

#if defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))
void Exploration_calcProfitDebug (ExplorationPayload *expPayload,
								  const Image *navMap,
								  Image *localMap,
								  const List *unreachableIkTargets,
								  const RobotData robots[MAX_N_ROBOTS],
								  FILE *xmlLog)
{
	ExplorationImpl_calcProfit (
		&expPayload->expData,
		navMap,
		localMap,
		expPayload->expGrid, // In GCC this gives: warning: passing arg _ of _ from incompatible pointer type. Afaik C does not support a const double pointer.
		expPayload->mappedCellGrid,
		expPayload->obstructedCellGrid,
		expPayload->unreachableExpCellGrid,
		unreachableIkTargets,
		robots,
		expPayload->pose,
		expPayload->stdDev,
		expPayload->index,
		expPayload->supervisor,
		expPayload->bid,
		expPayload->sessionIndex,
		expPayload->targetDist,
		&expPayload->geometryConstants,
		&expPayload->camVectors,
		&expPayload->ikConstants,
		&expPayload->uncertaintyConstants,
		expPayload->camPOffset,
		xmlLog);
}
#endif // defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))

void initExpPayload (ExplorationPayload **expPayload,
					 RobotDatabase *db,
					 ExplorationData *expData)
{
	int i;
	ExplorationPayload *ep;
#ifdef PRINT_DEBUG
	printf("ExplorationPayload %d\n", sizeof (ExplorationPayload));
#endif

	(*expPayload) = (ExplorationPayload*)malloc (sizeof (ExplorationPayload));
	ep = *expPayload;

	memcpy (&ep->expData, expData, sizeof (ExplorationData));
	for (i = 0; i < EXP_GRID_DIMS; ++i)
	{
		memcpy (ep->expGrid[i], db->environment.expGrid[i], sizeof (__int16) * EXP_GRID_DIMS);
	}
	memcpy (ep->mappedCellGrid, db->environment.mappedCellGrid, ((EXP_GRID_DIMS*EXP_GRID_DIMS)/8)+1);
	memcpy (ep->obstructedCellGrid, db->environment.obstructedCellGrid, ((NAV_GRID_DIMS*NAV_GRID_DIMS)/8)+1);
	memcpy (ep->unreachableExpCellGrid, db->environment.unreachableExpCellGrid, ((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1);
	memcpy (&ep->pose, &db->status.pose, sizeof (Pose));
	ep->camPOffset = CAM_P_OFFSET;
	ep->stdDev = db->status.stdDev;
	ep->index = db->status.index;
	ep->niter = db->status.nIterations;
	ep->supervisor = db->partnerData.explorationCoalition.supervisor;
	ep->bid = db->partnerData.explorationCoalition.bid;
	ep->sessionIndex = db->behaviourData.gotoExpPt.sessionIndex;
	ep->targetDist = CAM_P_OFFSET + SCAN_CENTRE_DIST; //< This is robot-specific, so have to setup in payload
	memcpy (&ep->geometryConstants, &db->geometryConstants, sizeof (GeometryConstants)); //< Remember geometryConstants are robot-specific!!!
	memcpy (&ep->camVectors, &db->camVectors, sizeof (CamVectors));
	memcpy (&ep->ikConstants, &db->ikConstants, sizeof (IkConstants));
	memcpy (&ep->uncertaintyConstants, &db->uncertaintyConstants, sizeof (UncertaintyConstants));
}

void clearExpPayload (ExplorationPayload *expPayload)
{
	free (expPayload);
}
#endif // defined(USE_CLOUD)















extern void GotoExpPt_setLocalMapExhausted (RobotDatabase *db);

#if defined(USE_CLOUD) && !defined(SIMULATION)
void RobotWrite_writeExpJob (
	RobotDatabase *db,
	ExplorationPayload *expPayload,
	ExplorationData *expData);
#endif

//! Calculate max achievable profit given current state.
void Exploration_calcProfit (ExplorationData *expData, RobotDatabase *db)
{
#if defined(USE_CLOUD)
	ExplorationPayload *expPayload;
#endif

	BehaviourCore_clearProfitData (&expData->profit);
	expData->isFollowPathImpossible = 0;

#ifdef COLLAB_EXP
	{
		if (-1 != db->partnerData.explorationCoalition.supervisor &&
			!db->behaviourData.closeLoop.isInSupervisionArea)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<ExplorationEarlyOut>reason=\"isInSupervisionArea\"</ExplorationEarlyOut>\n");
#endif
			return;
		}
	}
#endif

#if defined(USE_CLOUD)
	initExpPayload (
		&expPayload,
		db,
		expData);

#if defined(PRINT_CLOUD_DETAIL)
	ExplorationPayload_print (
		expPayload,
		&db->environment.unreachableIkDests,
		db->xmlLog);
#endif

	fprintf (db->xmlLog, "<CloudExploration />\n");

#if defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))

	Exploration_calcProfitDebug (
		expPayload,
		db->environment.navMap,
		db->sensorData.localMap,
		&db->environment.unreachableIkTargets,
		db->groupData.robots,
		db->xmlLog);

	memcpy (db->environment.unreachableExpCellGrid, expPayload->unreachableExpCellGrid, sizeof (db->environment.unreachableExpCellGrid));
	memcpy (expData, &expPayload->expData, sizeof (ExplorationData));

#if defined(PRINT_CLOUD_DETAIL)
	ExplorationData_printOutput (
		&expPayload->expData,
		db->xmlLog);
#endif

#else // defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))

	RobotWrite_writeExpJob (db, expPayload, expData);
#endif // defined(SIMULATION)

	clearExpPayload (expPayload);

#else // defined(USE_CLOUD)

	ExplorationImpl_calcProfit (
		expData,
		db->environment.navMap,
		db->sensorData.localMap,
		db->environment.expGrid,
		db->environment.mappedCellGrid,
		db->environment.obstructedCellGrid,
		db->environment.unreachableExpCellGrid,
		&db->environment.unreachableIkTargets,
		db->groupData.robots,
		db->status.pose,
		db->status.stdDev,
		db->status.index,
		db->partnerData.explorationCoalition.supervisor,
		db->partnerData.explorationCoalition.bid,
		db->behaviourData.gotoExpPt.sessionIndex,
		CAM_P_OFFSET + SCAN_CENTRE_DIST,
		&db->geometryConstants,
		&db->camVectors,
		&db->ikConstants,
		&db->uncertaintyConstants,
		CAM_P_OFFSET,
		db->xmlLog);
#endif // defined(USE_CLOUD)

	if (expData->profit.gross == MIN_FLT)
	{
		if (expData->isFollowPathImpossible)
		{
			db->behaviourData.shouldBacktrack = 1;
		}
	}
#ifdef PRINT_EVENTS
	else
	{
		BehaviourCore_printBehaviourProfit (
			db->xmlLog,
			EXPLORATION,
			0,
			1,
			&expData->profit.dest,
			expData->profit.ratio,
			expData->profit.gross,
			expData->profit.expenditure,
			expData->profit.resources,
			expData->profit.nSteps,
			expData->profit.gainNMapped,
			expData->profit.stdDevAtDest,
			expData->profit.stdDevInc,
			expData->profit.initialNMapped);
		fprintf (db->xmlLog, " profitToGtep=%f profitToCoal=%f gtepSession=%d", expData->profitToGtep, expData->profitToCoal, db->behaviourData.gotoExpPt.sessionIndex);
		printBehaviourProfit_tail (db->xmlLog, 1);
	}
#endif

#ifdef STRICT_EXP_BEH // When gtep can only be adopted AFTER exp targets are exhausted
	if (!Exploration_checkIfExpTarIsValid (db))
	{
		GotoExpPt_setLocalMapExhausted (db);
	}
#endif
}











int Exploration_checkIfExpTarIsValid (RobotDatabase *db)
{
	int isValid = (
		db->behaviourData.exploration.profit.ratio != MIN_FLT && 
		db->behaviourData.exploration.profit.initialNMapped < (EXP_AREA * EXP_AREA * 0.6f)
		);
	return isValid;
}

void Exploration_adopt (RobotDatabase *db)
{
	List_clear (&db->behaviourData.stack, 0);
	List_pushValue (&db->behaviourData.stack, (BasicBehaviourData*)&db->behaviourData.exploration);
	BehaviourCore_printStack (&db->behaviourData.stack, "adopt exp");
	db->behaviourData.behaviour = EXPLORATION;
	db->behaviourData.baseBehaviour = EXPLORATION;
	db->behaviourData.targetIndex = db->behaviourData.targetCounter;
	++db->behaviourData.targetCounter;
	db->behaviourData.exploration.profit.dest.type |= IS_NEW_DEST;
}

void Exploration_isAtDest (RobotDatabase *db)
{
	PointI pt;

	List_clear (&db->behaviourData.stack, 0);
	List_pushValue (&db->behaviourData.stack, &db->behaviourData.available);
	BehaviourCore_printStack (&db->behaviourData.stack, "exp is at dest");
	db->behaviourData.behaviour = AVAILABLE;
	db->behaviourData.baseBehaviour = AVAILABLE;
	db->behaviourData.targetIndex = -1;

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<AdoptBehaviour>behaviour=\"AVAILABLE\"</AdoptBehaviour>\n");
#endif

	pt = PointF_toPointI (db->behaviourData.exploration.profit.dest.target);
	pt.x -= db->sensorData.localMap->orig.x;
	pt.y -= db->sensorData.localMap->orig.y;
	pt = PointI_calcExpCellIndex (pt, EXP_AREA);
	BitArray_setElement_pt (db->environment.mappedCellGrid, pt, EXP_GRID_DIMS, 1);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<SetMapped>behaviour=\"EXPLORATION\" pt=(%d,%d)<SetMapped>\n", pt.x, pt.y);
#endif
}

extern void addDestToUnreachableList (RobotDatabase *db);

void Exploration_ikFailed (RobotDatabase *db)
{
	// May have to reset stuff for other behaviours. e.g. collabNav


	addDestToUnreachableList (db);

	// If robot was performing loop closing, have to leave provisional data, as it cannot
	// be further processed.
	adoptAVAILABLE (db, 1);

}


int Exploration_checkNavigationState (RobotDatabase *db, Dest *dest)
{
	PointI cell;
	Dest *prevDest = NULL;
	int currentMapped;

	if (FOLLOW_PATH == db->behaviourData.behaviour && EXPLORATION == ((BasicBehaviourData*)db->behaviourData.stack.back->prev->value)->id)
	{
		prevDest = getBehaviourDest (db, EXPLORATION);
		cell = PointF_toPointI (prevDest->target);
	}
	else
	{
		cell = PointF_toPointI (dest->target);
	}

	cell.x -= (db->sensorData.localMap->orig.x + (EXP_AREA / 2));
	cell.y -= (db->sensorData.localMap->orig.y + (EXP_AREA / 2));
	cell.x /= EXP_AREA;
	cell.y /= EXP_AREA;
	currentMapped = db->environment.expGrid[cell.x][cell.y];

	// This will have been updated in RobotMapGridProcessing_updateGrids()
	if (currentMapped > (EXP_AREA * EXP_AREA) * EXP_CELL_MAPPED_THRESHOLD)
	{
		dest->type |= IS_AT_DEST;
		if (prevDest)
		{
			prevDest->type |= IS_AT_DEST;
		}

		return 1;
	}
	else
	{
		return 0;
	}
}

#endif // if !defined(BOARD)



