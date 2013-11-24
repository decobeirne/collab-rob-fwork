#include "../../Common/RobotDefs.h"


#include "RobotWrite.h"
#include "../../Common/BitArray.h"


#ifndef BOARD

void RobotWrite_writeFinished (RobotDatabase *db)
{
#ifndef RERUNNING_ROBOT
#ifdef SIMULATION

#else
	RobotWriteImpl_writeFinished (db);
#endif
#endif
}

void RobotWrite_writeLeaveProvisionalMapData (RobotDatabase *db)
{
#ifndef RERUNNING_ROBOT
#ifdef SIMULATION
	fprintf (db->board->xmlLog, "<LeaveProvisionalMapData>robotIndex=%d</LeaveProvisionalMapData>\n", db->status.index);
#else
	RobotWriteImpl_writeLeaveProvisionalMapData (db);
#endif
#endif

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<LeaveProvisionalMapData />\n");
#endif
}

#ifdef SIMULATION
extern void tidyCoalitions (BoardDatabase *db, const int id, const int explorerOrSupervisor);
#endif

void RobotWrite_leaveCoalition (RobotDatabase *db, const int explorerOrSupervisor)
{
	RobotWrite_writeLeaveProvisionalMapData (db);

#ifndef RERUNNING_ROBOT
#ifdef SIMULATION
	tidyCoalitions (db->board, db->status.index, explorerOrSupervisor);
#else
	RobotWriteImpl_writeLeaveCoalition (db, explorerOrSupervisor);
#endif
#endif
}

RobotData generateRobotData (RobotDatabase *db)
{
	RobotData d;
	d.pose = db->status.pose;
	d.actualLocOffset = db->status.actualLocOffset;
	d.behaviour = db->behaviourData.behaviour;
	d.baseBehaviour = db->behaviourData.baseBehaviour;
	d.localMapOrigin = db->sensorData.localMap->orig;
	d.stdDev = db->status.stdDev;
	d.target = db->status.target;
	return d;
}

void RobotWrite_writeRobotData (RobotDatabase *db)
{
#ifndef RERUNNING_ROBOT
	RobotData d = generateRobotData (db);

#ifdef SIMULATION
	db->board->groupData.robots[db->status.index] = d;
#else
	RobotWriteImpl_writeRobotData (db, &d);
#endif
#endif
}

extern char experimentDirName[128];

void RobotWrite_writeExperimentPath (RobotDatabase *db)
{
#ifndef RERUNNING_ROBOT
#ifdef SIMULATION
	fprintf (db->board->xmlLog, "<RobotLogFile>index=%d path=\"%s\"</RobotLogFile>\n", db->status.index, experimentDirName);
#else
	RobotWriteImpl_writeExperimentPath (db);
#endif
#endif
}

void RobotWrite_writeLocalMap (RobotDatabase *db)
{
#ifdef SIMULATION
	int index = db->status.index;
	CompressedImageWithMapInfo *compressedImageWithMapInfo;
	int sz;
#endif

	if (0 == db->sensorData.isMapDataReady ||
//		0 == db->sensorData.mapScanList.size) // This was stupid - we reset the map scan list whenever we don't have a supervisor
		0 == db->environment.mapInfoToSubmit.nScans)
	{
		return;
	}

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<WriteLocalMap>index=%d origin=(%d,%d) isMoreDataComing=%d</WriteLocalMap>\n",
		db->environment.mapInfoToSubmit.localMapIndex,
		db->environment.mapToSubmit.orig.x,
		db->environment.mapToSubmit.orig.y,
		db->behaviourData.closeLoop.isMoreMapDataComing);
#endif

	TIMER_START ("writeLocalMap")

#ifndef RERUNNING_ROBOT
#ifdef SIMULATION
	compressedImageWithMapInfo = (CompressedImageWithMapInfo*)malloc (sizeof (CompressedImageWithMapInfo));
	compressedImageWithMapInfo->mapInfo = db->environment.mapInfoToSubmit;

	sz = db->environment.mapToSubmit.usedSize;
	compressedImageWithMapInfo->image.bufferSize = sz;
	compressedImageWithMapInfo->image.usedSize = sz;
	compressedImageWithMapInfo->image.buffer = (uchar*)malloc (sz);
	compressedImageWithMapInfo->image.orig = db->environment.mapToSubmit.orig;

	memcpy (compressedImageWithMapInfo->image.buffer, db->environment.mapToSubmit.buffer, sz);

	List_pushValue (&db->board->sensorData.incorpMaps, compressedImageWithMapInfo);

	// Set flags on board
	db->board->sensorData.statusFromRobot[index].isMoreDataComing = db->behaviourData.closeLoop.isMoreMapDataComing;
//	db->board->sensorData.statusFromRobot[index].isNewMapData = 1;
	if (!db->behaviourData.closeLoop.isMoreMapDataComing)
	{
		db->board->sensorData.mapStatus.isNewMapData = 1;
	}
#else // SIMULATION
	RobotWriteImpl_writeLocalMap (
		db,
		db->behaviourData.closeLoop.isMoreMapDataComing);
#endif // SIMULATION
#endif // RERUNNING_ROBOT

	// Reset own flags
	db->sensorData.isMapDataReady = 0;
	db->sensorData.isDataPendingInc = 1;

	TIMER_STOP (db->xmlLog, "writeLocalMap")
}

void RobotWrite_updateBoard (RobotDatabase *db)
{
	RobotWrite_writeRobotData (db);

	RobotWrite_writeLocalMap (db);
}


void RobotWrite_writeProposal (
	RobotDatabase *db,
	Proposal *prop)
{
#ifdef SIMULATION
	Proposal *ptr;
#endif

#ifndef RERUNNING_ROBOT
#ifdef SIMULATION
	ptr = (Proposal*)malloc (sizeof (Proposal));
	memcpy (ptr, prop, sizeof (Proposal));
//	List_pushValue (&db->board->coalitionData.proposals, ptr);

	List_insertSorted (&db->board->coalitionData.proposals, ptr, Proposal_isGreater);

#else
	RobotWriteImpl_writeProposal (db, &prop);
#endif
#endif

#ifdef PRINT_PROFIT
//	fprintf (db->xmlLog, "<MakeProposal>stdDev=%f nExpCells=%d area=(%d,%d) iteration=%d</MakeProposal>\n",
	fprintf (db->xmlLog, "<MakeProposal>stdDev=%f area=(%d,%d) iteration=%d</MakeProposal>\n",
		prop->stdDev,
//		prop->expCellsMapped,
		prop->area.x,
		prop->area.y,
		db->status.nIterations);
#endif
}
#if 0
void RobotWrite_writeProposal (RobotDatabase *db)
{
	Proposal prop;
#ifdef SIMULATION
	Proposal *ptr;
#endif
	time_t postedTime;
	postedTime = time (&postedTime);

	prop = initProposal();
	prop.supervisor = db->status.index;
	prop.area = db->behaviourData.supervision.area;
//	prop.pt = db->behaviourData.supervision.profit.dest.dest.loc;
	prop.stdDev = db->behaviourData.supervision.proposalStdDev;
	prop.expCellsMapped = db->behaviourData.supervision.proposalExpCellsMapped;
//	prop.duration = db->behaviourData.supervision.proposalDuration;
//	prop.totalProfit = db->behaviourData.supervision.proposalTotalProfit;

//	prop.supervisorResources = db->behaviourData.supervision.profit.resources; // Supervisor's overhead involved in getting to the dest loc
//	prop.supervisorReserve = db->behaviourData.maxProfit;
	prop.postedTime = postedTime;

#ifndef RERUNNING_ROBOT
#ifdef SIMULATION
	ptr = (Proposal*)malloc (sizeof (Proposal));
	memcpy (ptr, &prop, sizeof (Proposal));
	List_pushValue (&db->board->coalitionData.proposals, ptr);
#else
	RobotWriteImpl_writeProposal (db, &prop);
#endif
#endif

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<MakeProposal>stdDev=%f nExpCells=%d iteration=%d</MakeProposal>\n",
		prop.stdDev,
		prop.expCellsMapped,
//		prop.totalProfit,
//		prop.duration,
//		prop.supervisorResources,
//		prop.supervisorReserve,
		db->status.nIterations);
#endif
}
#endif

//void RobotWrite_writeBid (RobotDatabase *db, const int supervisor, const float bid)
void RobotWrite_writeBid (RobotDatabase *db, const int supervisor, const float nSteps)
{
	Bid bidData;
#ifdef SIMULATION
	Bid *ptr;
#endif

	bidData.supervisor = supervisor;
	bidData.explorer = db->status.index;
//	bidData.bid = bid;
	bidData.nSteps = nSteps;

#ifdef SIMULATION
	ptr = (Bid*)malloc (sizeof (Bid));
	memcpy (ptr, &bidData, sizeof (Bid));
	List_pushValue (&db->board->coalitionData.bids, ptr);
#else
	RobotWriteImpl_writeBid (db, &bidData);
#endif

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<WriteBid />\n");
#endif
}

void RobotWrite_writeProposalsConsidered (RobotDatabase *db)
{
#ifdef SIMULATION
	ListNode *iter;
	Proposal *proposal;

	iter = db->board->coalitionData.proposals.front;
	while (iter)
	{
		proposal = (Proposal*)iter->value;
		if (proposal->supervisor != db->status.index)
		{
			proposal->considered[db->status.index] = 1;
		}
		iter = iter->next;
	}
#else
	RobotWriteImpl_writeProposalsConsidered (db);
#endif
}

void RobotWrite_supervisorAtFinalDest (RobotDatabase *db)
{
#ifdef SIMULATION
	ListNode *iter;
	Coalition *c;

	iter = db->board->coalitionData.coalitions.front;
	while (iter)
	{
		c = (Coalition*)iter->value;

		if (c->supervisor == db->status.index)
		{
			c->collabData.isSupAtFinalDest = 1;
		}

		iter = iter->next;
	}
#else
	RobotWriteImpl_writeSupAtFinalDest (db);
#endif
}

#if defined (USE_CLOUD)
void RobotWrite_writeGtepJob (RobotDatabase *db,
							  GotoExpPtPayload *payload,
							  GotoExpPtData *gtepData)
{
	RobotWriteImpl_writeGtepJob (db, payload, gtepData);
}

void RobotWrite_writeExpJob (RobotDatabase *db,
							 ExplorationPayload *expPayload,
							 ExplorationData *expData)
{
	RobotWriteImpl_writeExpJob (db, expPayload, expData);
}
#endif // defined (USE_CLOUD)

void RobotWrite_updateUnreachableLocalMapGrid (
#if defined(SIMULATION) || defined(BOARD)
	uchar *boardUnreachableLocalMapGrid,
	BoardSensorData *boardSensorData,
#else
	RobotDatabase *db,
#endif
	const uchar *unreachableLocalMapGrid,
	const int index)
{
#if defined(SIMULATION) || defined(BOARD)
	int i;
	memcpy (
		boardUnreachableLocalMapGrid,
		unreachableLocalMapGrid,
		((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1);

	// Update flags for other robots
	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i != index)
		{
			boardSensorData->statusFromBoard[i].isNewUnreachableLocalMap = 1;
		}
	}

#else // defined(SIMULATION) || defined(BOARD)

#ifndef RERUNNING_ROBOT
	RobotWriteImpl_updateUnreachableLocalMapGrid (
		db,
		unreachableLocalMapGrid,
		index);
#endif // RERUNNING_ROBOT
#endif // defined(SIMULATION) || defined(BOARD)
}

void RobotWrite_updateUnreachableLocalMapPt (
#if defined(SIMULATION) || defined(BOARD)
	uchar *boardUnreachableLocalMapGrid,
	BoardSensorData *boardSensorData,
#else
	RobotDatabase *db,
#endif
	const int index,
	const PointI pt)
{
	// i think we should be copying the whole grid over, rather than just
	// doing a pt at a time
#if defined(SIMULATION) || defined(BOARD)
	int i;
	BitArray_setElement_pt (
		boardUnreachableLocalMapGrid,
		pt,
		GLOB_LOC_MAP_GRID_DIMS,
		1);

	// Update flags for other robots
	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i != index)
		{
			boardSensorData->statusFromBoard[i].isNewUnreachableLocalMap = 1;
		}
	}

#else // defined(SIMULATION) || defined(BOARD)

#ifndef RERUNNING_ROBOT
	RobotWriteImpl_updateUnreachableLocalMapPt (
		db,
		pt,
		index);
#endif // RERUNNING_ROBOT
#endif // defined(SIMULATION) || defined(BOARD)
}
#endif // BOARD





































