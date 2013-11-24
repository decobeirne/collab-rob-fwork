#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "RobotRead.h"
#include "../../Common/BitArray.h"

#if defined(RERUNNING_ROBOT) || defined(RECORDING_ROBOT)
#include "RobotCommSerialize.h"
#endif





void RobotRead_readStatus (RobotDatabase *db)
{
	MapStatusFromBoard s;
	Vector4I area;
	PointI localMapOrigin;

#ifdef RERUNNING_ROBOT
	readStatus_serialize (db, &s);

#else // RERUNNING_ROBOT

#ifdef SIMULATION
	s = db->board->sensorData.statusFromBoard[db->status.index];
	db->board->sensorData.statusFromBoard[db->status.index] = initMapStatusFromBoard();
#else
	RobotReadImpl_readStatus (db, &s);
#endif

#ifdef RECORDING_ROBOT
	writeStatus_serialize (db, &s);
#endif // RECORDING_ROBOT
#endif // RERUNNING_ROBOT

#ifdef PRINT_COMM_DETAIL
	fprintf (db->xmlLog, "<ReadStatus>isTimeElapsed=%d isNewGlobalMap=%d</ReadStatus>\n", s.isTimeElapsed, s.isNewGlobalMap);
#endif

	db->status.isTimeElapsed = s.isTimeElapsed;
	db->environment.isNewGlobalMap = s.isNewGlobalMap;
	db->environment.isNewUnreachableLocalMap = s.isNewUnreachableLocalMap;

	if (s.isNewGlobalMap)
	{
		db->sensorData.hasDataBeenIncd = s.hasDataBeenIncd;

		if (!s.hasDataBeenIncd)
		{
			area = s.mapAreaEffectedForRobot;
			localMapOrigin = db->sensorData.localMap->orig;

			area.x -= LOC_MAP_DIMS;
			area.y -= LOC_MAP_DIMS;
			area.z += LOC_MAP_DIMS;
			area.w += LOC_MAP_DIMS;

			if (localMapOrigin.x > area.x &&
				localMapOrigin.y > area.y &&
				localMapOrigin.x < area.z &&
				localMapOrigin.y < area.w)
			{
				db->sensorData.isLocalMapEffected = 1;
			}
		}
	}
}

#ifdef RERUNNING_ROBOT
#include "RobotCommSerialize.h"
#endif

void RobotRead_readGrids (RobotDatabase *db)
{
#ifdef SIMULATION
	int i;
#endif

	if (db->environment.isNewGlobalMap)
	{
#ifdef RERUNNING_ROBOT
		readGrids_serialize (db);

#else // RERUNNING_ROBOT

#ifdef SIMULATION
		memcpy (db->environment.localMapGrid, db->board->environment.localMapGrid, sizeof (__int16) * GLOB_LOC_MAP_GRID_DIMS * GLOB_LOC_MAP_GRID_DIMS);
		memcpy (db->environment.supGrid, db->board->environment.supGrid, sizeof (__int16) * SUP_GRID_DIMS * SUP_GRID_DIMS);
		memcpy (db->environment.obstructedCellGrid, db->board->environment.obstructedGrid, ((NAV_GRID_DIMS * NAV_GRID_DIMS)/8)+1);
#else
		RobotReadImpl_readGrids (db);
#endif // SIMULATION

#ifdef RECORDING_ROBOT
		writeGrids_serialize (db);
#endif
#endif // RERUNNING_ROBOT


#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<ReadGrids />\n");
#endif
	}

	if (db->environment.isNewUnreachableLocalMap)
	{
#ifdef RERUNNING_ROBOT
		readUnreachableGrid_serialize (db);

#else // RERUNNING_ROBOT

#ifdef SIMULATION
		for (i = 0; i < GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS; ++i)
		{
			if (BitArray_checkElement_index (db->board->environment.unreachableLocalMapGrid, i))
			{
				BitArray_setElement_index (db->environment.unreachableLocalMapGrid, i, 1);
			}
		}
#else
		RobotReadImpl_readUnreachableLocalMapGrid (db);
#endif

#ifdef RECORDING_ROBOT
		writeUnreachableGrid_serialize (db);
#endif // RECORDING_ROBOT
#endif // RERUNNING_ROBOT

		db->environment.isNewUnreachableLocalMap = 0;
	}
}

void RobotRead_readMap (RobotDatabase *db)
{
#ifdef SIMULATION
	int sz;
#endif

	if (db->sensorData.hasDataBeenIncd || db->sensorData.isLocalMapEffected)
	{
#ifdef RERUNNING_ROBOT
		readMap_serialize (db);

#else // RERUNNING_ROBOT

#ifdef SIMULATION
		BoardMapProcessing_compressMapSection (
			db->board,
			&db->sensorData.localMap->orig,
			db->board->xmlLog);

		sz = db->board->sensorData.compressedMap.usedSize;
		db->environment.mapFromBoard.usedSize = sz;
		db->environment.mapFromBoard.orig = db->board->sensorData.compressedMap.orig;
		memcpy (db->environment.mapFromBoard.buffer, db->board->sensorData.compressedMap.buffer, sz);
#else
		RobotReadImpl_readMap (db);
#endif

#ifdef RECORDING_ROBOT
		writeMap_serialize (db);
#endif
#endif // RERUNNING_ROBOT

		if (db->sensorData.hasDataBeenIncd)
		{
			db->sensorData.isDataPendingInc = 0;
		}
	}
}

//! Read status of other robots in group
void RobotRead_readGroup (RobotDatabase *db)
{
#ifdef RERUNNING_ROBOT
	readGroup_serialize (db);

#else // RERUNNING_ROBOT

#ifdef SIMULATION
	memcpy (db->groupData.robots, db->board->groupData.robots, sizeof (RobotData) * N_ROBOTS);
#else
	RobotReadImpl_readGroup (db);
#endif

#ifdef RECORDING_ROBOT
	writeGroup_serialize (db);
#endif
#endif // RERUNNING_ROBOT
}

void RobotRead_readProposals (RobotDatabase *db, List *proposalList)
{
#ifdef RERUNNING_ROBOT
	// Later, if at all

#else // RERUNNING_ROBOT

#ifdef SIMULATION
	Proposal *src;
	Proposal *dest;
	ListNode *iter;
	List_clear (proposalList, 1);

	iter = db->board->coalitionData.proposals.front;
	while (iter)
	{
		src = (Proposal*)iter->value;
		dest = (Proposal*)malloc (sizeof (Proposal));
		memcpy (dest, src, sizeof (Proposal));

		List_pushValue (proposalList, dest);
		iter = iter->next;
	}
#else
	RobotReadImpl_readProposals (db, proposalList);
#endif
#endif // RERUNNING_ROBOT
}

void RobotRead_readBids (RobotDatabase *db, List *bidList)
{
#ifdef RERUNNING_ROBOT
	// Later

#else // RERUNNING_ROBOT

#ifdef SIMULATION
	Bid *src;
	Bid *dest;
	ListNode *iter;
	List_clear (bidList, 1);

	iter = db->board->coalitionData.bids.front;
	while (iter)
	{
		src = (Bid*)iter->value;
		dest = (Bid*)malloc (sizeof (Bid));
		memcpy (dest, src, sizeof (Bid));

		List_pushValue (bidList, dest);
		iter = iter->next;
	}
#else
	RobotReadImpl_readBids (db, bidList);
#endif
#endif // RERUNNING_ROBOT
}

void RobotRead_readCoalitionFlags (RobotDatabase *db, int flags[2])
{
#ifdef RERUNNING_ROBOT
	//! \todo Coalition flags when rerunning

#else // RERUNNING_ROBOT

#ifdef SIMULATION
	flags[0] = db->board->coalitionData.isBidSuccessful[db->status.index];
	flags[1] = db->board->coalitionData.isPropSuccessful[db->status.index];
	db->board->coalitionData.isBidSuccessful[db->status.index] = 0;
	db->board->coalitionData.isPropSuccessful[db->status.index] = 0;

#else
	RobotReadImpl_readCoalitionFlags (db, flags);
#endif
#endif // RERUNNING_ROBOT
}

extern Coalition findExplorationCoalition (List *coalitions, const int index);
extern void findSupervisionCoalitions (List *boardCoalitions, List *robotCoalitions, const int index);

void RobotRead_readCoalitions (RobotDatabase *db)
{
#ifdef SETUP_TEST_COALITION
	return;
#endif

	db->partnerData.explorationCoalition = initCoalition (-1);
	List_clear (&db->partnerData.supervisionCoalitions, 1);

#ifdef RERUNNING_ROBOT
	// Later

#else // RERUNNING_ROBOT

#ifdef SIMULATION
	db->partnerData.explorationCoalition = findExplorationCoalition (&db->board->coalitionData.coalitions, db->status.index);
	findSupervisionCoalitions (&db->board->coalitionData.coalitions, &db->partnerData.supervisionCoalitions, db->status.index);
#else
	RobotReadImpl_readCoalitions (db);
#endif

#ifdef RECORDING_ROBOT
	// Later
#endif
#endif

	if (db->partnerData.explorationCoalition.id == -1)
	{
		db->partnerData.explorationCoalition.supervisor = -1;
	}
}

















void RobotRead_readBoard (RobotDatabase *db)
{
	RobotRead_readStatus (db);

	RobotRead_readGrids (db);

	RobotRead_readMap (db);

	RobotRead_readGroup (db);

	RobotRead_readCoalitions (db);
}


#endif
