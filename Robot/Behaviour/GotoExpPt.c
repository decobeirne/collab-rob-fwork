#include "../../Common/RobotDefs.h"

#if !defined(BOARD)


#include "GotoExpPt.h"
#include "Exploration.h"
#include "GotoExpPtImpl.h"
#include "BehaviourAdoption.h"
#include "../Data/BehaviourData.h"
#include "../Comm/RobotWrite.h"
#include "../../Common/BitArray.h"
#include "../../Common/Vector.h"




#if defined(USE_CLOUD)
void GotoExpPt_initPayload (
	GotoExpPtPayload **payload,
	RobotDatabase *db,
	const float behaviourChangeOverhead,
	const int considerStdDevInc,
	const PointI originCellMidpt,
	const PointI gridDims,
	GotoExpPtData *gotoExpPtData)
{
	int i;
	GotoExpPtPayload *p;
#ifdef PRINT_DEBUG
	printf("GotoExpPtPayload %d\n", sizeof(GotoExpPtPayload));
#endif
	*payload = (GotoExpPtPayload*)malloc (sizeof (GotoExpPtPayload));
	p = *payload;
	memcpy (p->obstructedCellGrid, db->environment.obstructedCellGrid, sizeof (db->environment.obstructedCellGrid));
	memcpy (&p->geometryConstants, &db->geometryConstants, sizeof (GeometryConstants));
	memcpy (&p->camVectors, &db->camVectors, sizeof (CamVectors));
	memcpy (&p->ikConstants, &db->ikConstants, sizeof (IkConstants));
	memcpy (&p->uncertaintyConstants, &db->uncertaintyConstants, sizeof (UncertaintyConstants));
	memcpy (&p->pose, &db->status.pose, sizeof (Pose));
	p->camPOffset = CAM_P_OFFSET,
	p->stdDev = db->status.stdDev;

	p->considerStdDevInc = considerStdDevInc;
	p->behaviourChangeOverhead = behaviourChangeOverhead;
	p->originCellMidpt = originCellMidpt;
	p->gridDims = gridDims;

	p->index = db->status.index;
	p->explorationCoalition = db->partnerData.explorationCoalition.id;
	p->niter = db->status.nIterations;
	for (i = 0; i < GLOB_LOC_MAP_GRID_DIMS; ++i)
	{
		memcpy (p->localMapGrid[i], db->environment.localMapGrid[i], sizeof (short) * GLOB_LOC_MAP_GRID_DIMS);
	}
	memcpy (&p->gotoExpPtData, gotoExpPtData, sizeof (GotoExpPtData));
}



#if defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))

void GotoExpPt_calcProfitDebug (
#if defined(SIMULATION)
	BoardDatabase *db,
#endif
	const RobotData *groupData,
	Image *navMap,
	Image *localMap,
	const List *unreachableIkDests,
	uchar *boardUnreachableLocalMapGrid,
	GotoExpPtPayload *gtepPayload,
	FILE *xmlLog)
{
	int hasUnreachableLocalMapGridBeenUpdated = 0;

	GotoExpPtImpl_calcProfit (
		navMap,
		localMap,
		gtepPayload->obstructedCellGrid,
		boardUnreachableLocalMapGrid,
		&hasUnreachableLocalMapGridBeenUpdated,
		&gtepPayload->geometryConstants,
		&gtepPayload->camVectors,
		&gtepPayload->ikConstants,
		&gtepPayload->uncertaintyConstants,
		gtepPayload->camPOffset,
//		db->groupData.robots,
		groupData,
		unreachableIkDests,
		gtepPayload->pose,
		gtepPayload->stdDev,
		gtepPayload->behaviourChangeOverhead, // Different for GotoExpPt and GotoExpPtCollab
		gtepPayload->considerStdDevInc, // same
		gtepPayload->originCellMidpt, // same
		gtepPayload->gridDims, // same
		gtepPayload->index,
		gtepPayload->explorationCoalition,
		gtepPayload->localMapGrid,
		&gtepPayload->gotoExpPtData,
		xmlLog);

// If we are rerunning, then we simulate all data returned from the board, so we don't
// have to write anything
#if defined(SIMULATION)
	// This function is meant to simulate actual functionality on cloud,
	// so the board's unreachableLocalMapGrid is used. Therefore we don't
	// sync the robot's grid to the board, but just reset the flags on the
	// board.
	if (hasUnreachableLocalMapGridBeenUpdated)
	{
		MapStatusFromBoard_resetNewUnreachableLocalMap (db->sensorData.statusFromBoard);
	}
#endif
}


#else // defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))

#endif // defined(SIMULATION)
#endif // defined(USE_CLOUD)







void GotoExpPt_setLocalMapExhausted (RobotDatabase *db)
{
	PointI pt;
	// This local map has been exhausted. It may not appear to be exhausted due to blocked terrain though, so
	// it is necessary to mark it on the grid.
	pt.x = (db->sensorData.localMapCentre.x - LOC_MAP_DIMS / 2.0f) / LOC_MAP_DIFF;
	pt.y = (db->sensorData.localMapCentre.y - LOC_MAP_DIMS / 2.0f) / LOC_MAP_DIFF;

	BitArray_setElement_pt (
		db->environment.unreachableLocalMapGrid,
		pt,
		GLOB_LOC_MAP_GRID_DIMS,
		1);

	RobotWrite_updateUnreachableLocalMapPt (
#if defined(SIMULATION) || defined(BOARD)
		db->board->environment.unreachableLocalMapGrid,
		&db->board->sensorData,
#else
		db,
#endif
		db->status.index,
		pt);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<LocalMapExhausted>index=(%d,%d) centre=(%d,%d)</LocalMapExhausted>\n",
		pt.x,pt.y,
		db->sensorData.localMapCentre.x,
		db->sensorData.localMapCentre.y);
#endif
}

int GotoExpPt_isMapDataToAjust (RobotDatabase *db)
{
	return db->sensorData.mapScanList.size > 8;
}

float GotoExpPt_calcBehaviourChangeOverhead (
	const int explorationCoalition,
	const float stdDev)
{
//	float profit;

//	if (db->partnerData.explorationCoalition.id != -1)
	if (explorationCoalition != -1)
	{
		// Estimate typical profitPerStep given current stdDev in location estimate.
//		profit = PROFIT_PER_MOVE_OVER_STD_DEV_B + stdDev * PROFIT_PER_MOVE_OVER_STD_DEV_M;
//		return profit * COALITION_QUIT_COEFF;
		return 5.0f; //! \todo May required tweaking
	}
	return 0.0f;
}



















void GotoExpPt_calcProfit (GotoExpPtData *g, RobotDatabase *db)
{
	PointI originCellMidpt;
	PointI gridDims;
	float behaviourChangeOverhead;
	const int considerStdDevInc = 1;

#if defined(USE_CLOUD)
	GotoExpPtPayload *payload;
#else
	int hasUnreachableLocalMapGridBeenUpdated = 0;
#endif

	FollowPathData followPathData = initFollowPathData();

	BehaviourCore_clearProfitData (&g->profit);
	g->isFollowPathImpossible = 0;

#ifdef STRICT_EXP_BEH // When gtep can only be adopted AFTER exp targets are exhausted
	if (Exploration_checkIfExpTarIsValid (db))
	{
		return;
	}
#endif

	// If robot is doing collaborative exploration, then GOTO_EXP_PT is considered an independent
	// behaviour (as opposed to GOTO_EXP_PT_COLLAB).
	// For simplicity's sake, the robot cannot adopt this behaviour if there is pending map data.
#ifdef COLLAB_EXP
	
	if (GotoExpPt_isMapDataToAjust (db) && db->partnerData.explorationCoalition.id != -1)
	{
		return;
	}
#endif

	originCellMidpt.x = originCellMidpt.y = (LOC_MAP_DIMS / 2.0f);
	gridDims.x = GLOB_LOC_MAP_GRID_DIMS;
	gridDims.y = GLOB_LOC_MAP_GRID_DIMS;
	behaviourChangeOverhead = GotoExpPt_calcBehaviourChangeOverhead (
		db->partnerData.explorationCoalition.id,
		db->status.stdDev);

#if defined(USE_CLOUD)
	GotoExpPt_initPayload (
		&payload,
		db,
		behaviourChangeOverhead,
		considerStdDevInc,
		originCellMidpt,
		gridDims,
		g);

#if defined(PRINT_CLOUD_DETAIL)
	GotoExpPtPayload_print (
		payload,
		db->xmlLog);
#endif

	fprintf (db->xmlLog, "<CloudGotoExpPt />\n");

#if defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))

	GotoExpPt_calcProfitDebug (
#if defined(SIMULATION)
		db->board,
		db->board->groupData.robots,
#else
		db->groupData.robots,
#endif
		db->environment.navMap,
		db->sensorData.localMap,
		&db->environment.unreachableIkDests,
#if defined(SIMULATION)
		db->board->environment.unreachableLocalMapGrid, // Board's grid - as when using cloud
#else
		db->environment.unreachableLocalMapGrid, // Rerunning - so don't need to keep in sync
#endif
		payload,
		db->xmlLog);

	// Everything the robot needs to get back from the board is included
	memcpy (g, &payload->gotoExpPtData, sizeof (GotoExpPtData));

#if defined(PRINT_CLOUD_DETAIL)
	GotoExpPtData_printOutput (
		&payload->gotoExpPtData,
		db->xmlLog);
#endif

#else // defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))



	RobotWrite_writeGtepJob (db, payload, g);
#endif // defined(SIMULATION)

	GotoExpPtImpl_clearPayload (payload);

#else // defined(USE_CLOUD)

	GotoExpPtImpl_calcProfit (
		db->environment.navMap,
		db->sensorData.localMap,
		db->environment.obstructedCellGrid,
		db->environment.unreachableLocalMapGrid,
		&hasUnreachableLocalMapGridBeenUpdated,
		&db->geometryConstants,
		&db->camVectors,
		&db->ikConstants,
		&db->uncertaintyConstants,
		CAM_P_OFFSET,
		db->groupData.robots,
		&db->environment.unreachableIkDests,
		db->status.pose,
		db->status.stdDev,
		behaviourChangeOverhead,
		considerStdDevInc,
		originCellMidpt,
		gridDims,
		db->status.index,
		db->partnerData.explorationCoalition.id,
		db->environment.localMapGrid,
		g,
		db->xmlLog);

	// Updates have been made on the robots unreachableLocalMapGrid. This must
	// be kept in sync with board, so write to the board's version.
	if (hasUnreachableLocalMapGridBeenUpdated)
	{
		RobotWrite_updateUnreachableLocalMapGrid (
			db,
			db->environment.unreachableLocalMapGrid,
			db->status.index);
	}
#endif // defined(USE_CLOUD)

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
			db->xmlLog, GOTO_EXP_PT,
			0,
			1,
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

		printBehaviourProfit_tail (db->xmlLog, 1);
	}
#endif

	FollowPathData_clear (&followPathData);
}













































GotoExpPtData GotoExpPt_getDataForCalcingCollabGtep (
	const GotoExpPtCollabData *gCollab,
	const RobotDatabase *db)
{
	GotoExpPtData g;

	// Get elements from GotoExpPtCollabData
	g.id = gCollab->id;
	g.isFollowPathImpossible = gCollab->isFollowPathImpossible;
	memcpy (&g.profit, &gCollab->profit, sizeof (ProfitData));

	// Get elements only stored in GotoExpPtData
	g.sessionIndex = db->behaviourData.gotoExpPt.sessionIndex;
	g.nextSessionIndex = db->behaviourData.gotoExpPt.nextSessionIndex;
	g.requestedLocalMap = db->behaviourData.gotoExpPt.requestedLocalMap;

	return g;
}

void GotoExpPt_calcProfitForCoal (GotoExpPtCollabData *g, RobotDatabase *db)
{
	PointI originCellMidpt;
	PointI gridDims;
	const float behaviourChangeOverhead = 0.0f;
	const int considerStdDevInc = 0;
	GotoExpPtData g_temp;

#if defined(USE_CLOUD)
	GotoExpPtPayload *payload;
#else
	int hasUnreachableLocalMapGridBeenUpdated = 0;
#endif

	FollowPathData followPathData = initFollowPathData();

	BehaviourCore_clearProfitData (&g->profit);
	g->isFollowPathImpossible = 0;

	if (db->partnerData.explorationCoalition.id == -1)
	{
		return;
	}

#ifdef STRICT_EXP_BEH // When gtep can only be adopted AFTER exp targets are exhausted
	if (Exploration_checkIfExpTarIsValid (db))
	{
		return;
	}
#endif

//	originCellMidpt.x = originCellMidpt.y = (LOC_MAP_DIMS / 2.0f);
//	gridDims.x = GLOB_LOC_MAP_GRID_DIMS;
//	gridDims.y = GLOB_LOC_MAP_GRID_DIMS;

//	originCellMidpt.x = db->partnerData.explorationCoalition.pt.x - (SUP_DIMS/2 - LOC_MAP_DIMS/2);
//	originCellMidpt.y = db->partnerData.explorationCoalition.pt.y - (SUP_DIMS/2 - LOC_MAP_DIMS/2);
	originCellMidpt.x = (SUP_DIFF + db->partnerData.explorationCoalition.area.x * SUP_DIFF) - (SUP_DIMS/2 - LOC_MAP_DIMS/2);
	originCellMidpt.y = (SUP_DIFF + db->partnerData.explorationCoalition.area.y * SUP_DIFF) - (SUP_DIMS/2 - LOC_MAP_DIMS/2);
	gridDims.x = 5;
	gridDims.y = 5;
//	gridDims.x = 3;
//	gridDims.y = 3;

	// Copy collab data into GotoExpPtData
	g_temp = GotoExpPt_getDataForCalcingCollabGtep (g, db);

#if defined(USE_CLOUD)
	GotoExpPt_initPayload (
		&payload,
		db,
		behaviourChangeOverhead,
		considerStdDevInc,
		originCellMidpt,
		gridDims,
		&g_temp);

	fprintf (db->xmlLog, "<CloudGotoExpPtCollab />\n");

#if defined(PRINT_CLOUD_DETAIL)
	GotoExpPtPayload_print (
		payload,
		db->xmlLog);
#endif

#if defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))

	GotoExpPt_calcProfitDebug (
#if defined(SIMULATION)
		db->board,
		db->board->groupData.robots,
#else
		db->groupData.robots,
#endif
		db->environment.navMap,
		db->sensorData.localMap,
		&db->environment.unreachableIkDests,
#if defined(SIMULATION)
		db->board->environment.unreachableLocalMapGrid, // Board's grid - as when using cloud
#else
		db->environment.unreachableLocalMapGrid, // Rerunning - so don't need to keep in sync
#endif
		payload,
		db->xmlLog);

	// Everything the robot needs to get back from the board is included
	memcpy (&g_temp, &payload->gotoExpPtData, sizeof (GotoExpPtData));

#if defined(PRINT_CLOUD_DETAIL)
	GotoExpPtData_printOutput (
		&payload->gotoExpPtData,
		db->xmlLog);
#endif

#else // defined(SIMULATION) || (defined(ROBOT) && defined(RERUNNING_ROBOT))

//#if defined(PRINT_CLOUD_DETAIL)
//	GotoExpPtPayload_print (
//		payload,
//		db->xmlLog);
//#endif

	RobotWrite_writeGtepJob (db, payload, &g_temp);
#endif // defined(SIMULATION)




	GotoExpPtImpl_clearPayload (payload);

#else // defined(USE_CLOUD)

	GotoExpPtImpl_calcProfit (
		db->environment.navMap,
		db->sensorData.localMap,
		db->environment.obstructedCellGrid,
		db->environment.unreachableLocalMapGrid,
		&hasUnreachableLocalMapGridBeenUpdated,
		&db->geometryConstants,
		&db->camVectors,
		&db->ikConstants,
		&db->uncertaintyConstants,
		CAM_P_OFFSET,
		db->groupData.robots,
		&db->environment.unreachableIkDests,
		db->status.pose,
		db->status.stdDev,
		behaviourChangeOverhead,
		considerStdDevInc,
		originCellMidpt,
		gridDims,
		db->status.index,
		db->partnerData.explorationCoalition.id,
		db->environment.localMapGrid,
		&g_temp,
		db->xmlLog);

	// Updates have been made on the robots unreachableLocalMapGrid. This must
	// be kept in sync with board, so write to the board's version.
	if (hasUnreachableLocalMapGridBeenUpdated)
	{
		RobotWrite_updateUnreachableLocalMapGrid (
			db,
			db->environment.unreachableLocalMapGrid,
			db->status.index);
	}
#endif // defined(USE_CLOUD)

	// Copy stuff back into collab struct
	{
		g->isFollowPathImpossible = g_temp.isFollowPathImpossible;
		memcpy (&g->profit, &g_temp.profit, sizeof (ProfitData));

#ifdef SIMULATION
		// Make sure that we weren't changing anything on the cloud
		EXPLICIT_DEBUG_ASSERT (db->behaviourData.gotoExpPt.sessionIndex == g_temp.sessionIndex)
		EXPLICIT_DEBUG_ASSERT (db->behaviourData.gotoExpPt.nextSessionIndex == g_temp.nextSessionIndex)
		EXPLICIT_DEBUG_ASSERT (1 == PointI_compareValues (db->behaviourData.gotoExpPt.requestedLocalMap, g_temp.requestedLocalMap))
#endif
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
			db->xmlLog, GOTO_EXP_PT_COLLAB,
			0,
			1,
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

		printBehaviourProfit_tail (db->xmlLog, 1);
	}
#endif

	FollowPathData_clear (&followPathData);
}





























void GotoExpPtCollab_adopt (RobotDatabase *db)
{
	List_clear (&db->behaviourData.stack, 0);
	List_pushValue (&db->behaviourData.stack, &db->behaviourData.gotoExpPtCollab);
	BehaviourCore_printStack (&db->behaviourData.stack, "gtepc adopt");
	db->behaviourData.behaviour = GOTO_EXP_PT_COLLAB;
	db->behaviourData.baseBehaviour = GOTO_EXP_PT_COLLAB;
	db->behaviourData.targetIndex = db->behaviourData.targetCounter;
	++db->behaviourData.targetCounter;
	db->behaviourData.gotoExpPt.sessionIndex = -1;
	db->behaviourData.gotoExpPtCollab.profit.dest.type |= IS_NEW_DEST;
}

void GotoExpPt_adopt (RobotDatabase *db)
{
	List_clear (&db->behaviourData.stack, 0);
	List_pushValue (&db->behaviourData.stack, &db->behaviourData.gotoExpPt);
	BehaviourCore_printStack (&db->behaviourData.stack, "gtep adopt");
	db->behaviourData.behaviour = GOTO_EXP_PT;
	db->behaviourData.baseBehaviour = GOTO_EXP_PT;
	db->behaviourData.targetIndex = db->behaviourData.targetCounter;
	++db->behaviourData.targetCounter;
	db->behaviourData.gotoExpPt.sessionIndex = -1;
	db->behaviourData.gotoExpPt.profit.dest.type |= IS_NEW_DEST;
}

void GotoExpPt_isAtDest (RobotDatabase *db, const int isCollabGtep)
{
	List_clear (&db->behaviourData.stack, 0);
	List_pushValue (&db->behaviourData.stack, &db->behaviourData.available);
	BehaviourCore_printStack (&db->behaviourData.stack, "gtep at dest");
	db->behaviourData.behaviour = AVAILABLE;
	db->behaviourData.baseBehaviour = AVAILABLE;
	db->behaviourData.targetIndex = -1;

	if (isCollabGtep)
	{
		db->behaviourData.gotoExpPt.requestedLocalMap = PointF_toPointI (db->behaviourData.gotoExpPtCollab.profit.dest.dest.loc);
	}
	else
	{
		db->behaviourData.gotoExpPt.requestedLocalMap = PointF_toPointI (db->behaviourData.gotoExpPt.profit.dest.dest.loc);
	}
	db->sensorData.isLocalMapResetRequired = SET_REQUESTED_LOC_MAP;

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<AdoptBehaviour>behaviour=\"AVAILABLE\"</AdoptBehaviour>\n");
#endif
}

extern void addDestToUnreachableList (RobotDatabase *db);

void GotoExpPt_ikFailed (RobotDatabase *db)
{

	addDestToUnreachableList (db);

	// If robot was performing loop closing, have to leave provisional data, as it cannot
	// be further processed.
	adoptAVAILABLE (db, 1);


}

#endif // !defined(BOARD)


