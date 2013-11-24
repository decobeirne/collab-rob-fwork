#include "../../Common/RobotDefs.h"

#ifndef ROBOT


#include "BoardComm.h"
#include "../../Robot/Data/BehaviourData.h"
#include "../../Common/BitArray.h"

#if defined(USE_CLOUD)
#include "../Map/BoardJobs.h"
#endif

void writeStatus ROBOT_REQUEST_PARAMS
{
	int res;
	int robotIndex;
	MapStatusFromBoard s;

	robotIndex = payload;

	s = db->sensorData.statusFromBoard[robotIndex];
	db->sensorData.statusFromBoard[robotIndex] = initMapStatusFromBoard();
	db->sensorData.statusFromBoard[robotIndex].isTimeElapsed = db->groupData.experimentStatus.isTimeElapsed;

	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, &s, sizeof (MapStatusFromBoard));

	res = CommCore_sendBuffer (commData, sizeof (MapStatusFromBoard));
	assert (0 < res);

#ifdef PRINT_COMM_DETAIL
	fprintf (f, "<WriteStatus>robotIndex=%d isTimeElapsed=%d</WriteStatus>\n", robotIndex, s.isTimeElapsed);
#endif
}

void writeGrids ROBOT_REQUEST_PARAMS
{
	int res;
	int sz;
	const int ackSz = sizeof (COMM_FLAG) + sizeof (int);

	// Local maps
	sz = sizeof (__int16) * GLOB_LOC_MAP_GRID_DIMS * GLOB_LOC_MAP_GRID_DIMS;

	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, db->environment.localMapGrid, sz);

	res = CommCore_sendBuffer (commData, sz);
	assert (0 < res);

	res = CommCore_recvBuffer (commData, ackSz);
	assert (0 < res);

	// Supervision areas
	sz = sizeof (__int16) * SUP_GRID_DIMS * SUP_GRID_DIMS;

	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, db->environment.supGrid, sz);

	res = CommCore_sendBuffer (commData, sz);
	assert (0 < res);

	res = CommCore_recvBuffer (commData, ackSz);
	assert (0 < res);

	// Obstructed cells
	sz = ((NAV_GRID_DIMS * NAV_GRID_DIMS)/8)+1;

	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, db->environment.obstructedGrid, sz);

	res = CommCore_sendBuffer (commData, sz);
	assert (0 < res);

#ifdef PRINT_EVENTS
	fprintf (f, "<WriteGrids />\n");
#endif
}
	
void writeUnreachableLocalMapGrid ROBOT_REQUEST_PARAMS
{
	int res, sz;
	sz = sizeof (uchar) * ((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1;

	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, db->environment.unreachableLocalMapGrid, sz);

	res = CommCore_sendBuffer (commData, sz);
	assert (0 < res);

#ifdef PRINT_EVENTS
	fprintf (f, "<writeUnreachableLocalMapGrid />\n");
#endif
}

void writeGroupData ROBOT_REQUEST_PARAMS
{
	int res;
	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, db->groupData.robots, sizeof (RobotData) * N_ROBOTS);

	res = CommCore_sendBuffer (commData, sizeof (RobotData) * N_ROBOTS);
	assert (0 < res);

#ifdef PRINT_COMM_DETAIL
	fprintf (f, "<WriteGroupData>robotIndex=%d</WriteGroupData>\n", payload);
#endif
}

void writeCoalitionFlags ROBOT_REQUEST_PARAMS
{
	int res;
	int robotIndex = payload;
	int flags[2];

	flags[0] = db->coalitionData.isPropSuccessful[robotIndex];
	flags[1] = db->coalitionData.isBidSuccessful[robotIndex];

	db->coalitionData.isPropSuccessful[robotIndex] = 0;
	db->coalitionData.isBidSuccessful[robotIndex] = 0;

	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, flags, sizeof (int) * 2);

	res = CommCore_sendBuffer (commData, sizeof (int) * 2);
	assert (0 < res);

#ifdef PRINT_COMM_DETAIL
	fprintf (f, "<WriteCoalitionFlags>robotIndex=%d isProposalSuccessful=%d isBidSuccessful=%d</WriteCoalitionFlags>\n", robotIndex, flags[0], flags[1]);
#endif
}

extern Coalition findExplorationCoalition (List *coalitions, const int index);
extern void findSupervisionCoalitions (List *boardCoalitions, List *robotCoalitions, const int index);

void writeCoalitions (BoardDatabase *db, CommData *commData, const int payload, FILE *f)
{
	int res;
	int robotIndex = payload;
	Coalition coalition;
	List coalitions = initList();
	ListNode *iter;

	coalition = findExplorationCoalition (&db->coalitionData.coalitions, robotIndex);

	findSupervisionCoalitions (&db->coalitionData.coalitions, &coalitions, robotIndex);

	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, &coalition, sizeof (Coalition));
	CommCore_addItemToBuffer (commData, &coalitions.size, sizeof (int));

	res = CommCore_sendBuffer (commData, sizeof (Coalition) + sizeof (int));
	assert (0 < res);

	if (coalitions.size)
	{
		res = CommCore_recvAck (commData, R_COALITIONS);
		assert (0 < res);

		CommCore_clearBuffer (commData);

		iter = coalitions.front;
		while (iter)
		{
			CommCore_addItemToBuffer (commData, iter->value, sizeof (Coalition));
			iter = iter->next;
		}

		res = CommCore_sendBuffer (commData, sizeof (Coalition) * coalitions.size);
		assert (0 < res);
	}
}

void writeProposals ROBOT_REQUEST_PARAMS
{
	int res;
	int nProposals;
	ListNode *iter;
	Proposal *proposal;

	nProposals = db->coalitionData.proposals.size;
	
	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, &nProposals, sizeof (int));

	res = CommCore_sendBuffer (commData, sizeof (int));
	assert (0 < res);

	if (nProposals)
	{
		res = CommCore_recvAck (commData, R_PROPOSALS);
		assert (0 < res);

		CommCore_clearBuffer (commData);

		iter = db->coalitionData.proposals.front;
		while (iter)
		{
			proposal = (Proposal*)iter->value;
			CommCore_addItemToBuffer (commData, proposal, sizeof (Proposal));

			iter = iter->next;
		}

		res = CommCore_sendBuffer (commData, sizeof (Proposal) * nProposals);
		assert (0 < res);
	}

#ifdef PRINT_COMM_DETAIL
	fprintf (f, "<WriteProposals>nProposals=%d</WriteProposals>\n", nProposals);
#endif
}

void writeBids ROBOT_REQUEST_PARAMS
{
	int res;
	int nBids;
	ListNode *iter;
	Bid *bid;

	nBids = db->coalitionData.bids.size;
	
	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, &nBids, sizeof (int));

	res = CommCore_sendBuffer (commData, sizeof (int));
	assert (0 < res);

	if (nBids)
	{
		res = CommCore_recvAck (commData, R_BIDS);
		assert (0 < res);

		CommCore_clearBuffer (commData);

		iter = db->coalitionData.bids.front;
		while (iter)
		{
			bid = (Bid*)iter->value;
			CommCore_addItemToBuffer (commData, bid, sizeof (Bid));

			iter = iter->next;
		}

		res = CommCore_sendBuffer (commData, sizeof (Bid) * nBids);
		assert (0 < res);
	}

#ifdef PRINT_COMM_DETAIL
	fprintf (f, "<WriteBids>nBids=%d</WriteBids>\n", nBids);
#endif
}

















void writeMapSection ROBOT_REQUEST_PARAMS
{
	int res;
	PointI localMapOrigin;

	// Recv map origin
	res = CommCore_sendAck (commData, R_GLOBAL_MAP_SECTION);
	assert (0 < res);

	res = CommCore_recvBuffer (commData, payload);
	assert (0 < res);

	memcpy (&localMapOrigin, commData->buffer, sizeof (PointI));

	// Send package info
	BoardMapProcessing_compressMapSection (db, &localMapOrigin, f);

	CommCore_sendMap (commData, &db->sensorData.compressedMap);

	// Simplify communication if the board always ends with a send
	res = CommCore_sendAck (commData, W_MAP);
	assert (0 < res);

#ifdef PRINT_EVENTS
	fprintf (f, "<WriteMapSection>localMapOrigin=(%d,%d)</WriteMapSection>\n", localMapOrigin.x, localMapOrigin.y);
#endif
}



































void readLeaveProvisionalMapData ROBOT_REQUEST_PARAMS
{
	int res;
	res = CommCore_sendAck (commData, W_LEAVE_PROV_DATA);
	assert (0 < res);

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<LeaveProvisionalMapData>robotIndex=%d</LeaveProvisionalMapData>\n", payload);
#endif
}

extern void tidyCoalitions (BoardDatabase *db, const int id, const int explorerOrSupervisor);

void readLeaveCoalition ROBOT_REQUEST_PARAMS
{
	int res;
	int index;
	int explorerOrSupervisor;
	res = CommCore_sendAck (commData, W_LEAVE_COALITION);
	assert (0 < res);

	explorerOrSupervisor = (payload >> 31) & 1;
	index = payload & 0x7ffffff;

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<LeaveCoalition>robotIndex=%d</LeaveCoalition>\n", index);
#endif

	tidyCoalitions (db, index, explorerOrSupervisor);
}


void readFinished ROBOT_REQUEST_PARAMS
{
	int res;
	commData->quitFlag = 1;

	res = CommCore_sendAck (commData, W_FINISHED);
	assert (0 < res);

#ifdef PRINT_EVENTS
	fprintf (f, "<ReadFinished>robotIndex=%d</ReadFinished>\n", payload);
#endif
}

void readRobotData ROBOT_REQUEST_PARAMS
{
	int res;
	int index;
	RobotData d;

	res = CommCore_sendAck (commData, W_ROBOT_DATA);
	assert (0 < res);

	res = CommCore_recvBuffer (commData, payload);
	assert (0 < res);

	memcpy (&index,		commData->buffer,					sizeof (int));
	memcpy (&d,			commData->buffer + sizeof (int),	sizeof (RobotData));
	db->groupData.robots[index] = d;

	res = CommCore_sendAck (commData, W_ROBOT_DATA);
	assert (0 < res);

#ifdef PRINT_COMM_DETAIL
	fprintf (f, "<ReadRobotData>robotIndex=%d</ReadRobotData>\n", index);
#endif
}

void readRobotLogfile ROBOT_REQUEST_PARAMS
{
	int res;
	int index;
	char temp[128];

	res = CommCore_sendAck (commData, W_ROBOT_LOGFILE);
	assert (0 < res);

	res = CommCore_recvBuffer (commData, payload);
	assert (0 < res);

	memcpy (&index,		commData->buffer,					sizeof (int));
	memcpy (temp,		commData->buffer + sizeof (int),	payload - sizeof (int));

	res = CommCore_sendAck (commData, W_ROBOT_LOGFILE);
	assert (0 < res);

#ifdef PRINT_EVENTS
	fprintf (f, "<RobotLogFile>index=%d path=\"%s\"</RobotLogFile>\n", index, temp);
#endif
}

void readProposal ROBOT_REQUEST_PARAMS
{
	int res;
	int index;
	Proposal *ptr;

	res = CommCore_sendAck (commData, W_PROPOSAL);
	assert (0 < res);

	res = CommCore_recvBuffer (commData, payload);
	assert (0 < res);

	ptr = (Proposal*)malloc (sizeof (Proposal));
	memcpy (&index,		commData->buffer,					sizeof (int));
	memcpy (ptr,		commData->buffer + sizeof (int),	sizeof (Proposal));

	List_pushValue (&db->coalitionData.proposals, ptr);

	res = CommCore_sendAck (commData, W_PROPOSAL);
	assert (0 < res);

#ifdef PRINT_COMM_DETAIL
	fprintf (f, "<ReadProposal>robotIndex=%d</ReadProposal>\n", index);
#endif
}

void readBid ROBOT_REQUEST_PARAMS
{
	int res;
	int index;
	Bid *ptr;

	res = CommCore_sendAck (commData, W_BID);
	assert (0 < res);

	res = CommCore_recvBuffer (commData, payload);
	assert (0 < res);

	ptr = (Bid*)malloc (sizeof (Bid));
	memcpy (&index,		commData->buffer,					sizeof (int));
	memcpy (ptr,		commData->buffer + sizeof (int),	sizeof (Bid));

	List_pushValue (&db->coalitionData.bids, ptr);

	res = CommCore_sendAck (commData, W_BID);
	assert (0 < res);

#ifdef PRINT_COMM_DETAIL
	fprintf (f, "<ReadBid>robotIndex=%d</ReadBid>\n", index);
#endif
}

void readProposalsConsidered ROBOT_REQUEST_PARAMS
{
	int res;
	ListNode *iter;
	Proposal *proposal;

	iter = db->coalitionData.proposals.front;
	while (iter)
	{
		proposal = (Proposal*)iter->value;
		if (proposal->supervisor != payload)
		{
			proposal->considered[payload] = 1;
		}
		iter = iter->next;
	}

	res = CommCore_sendAck (commData, W_PROPOSALS_CONSIDERED);
	assert (0 < res);

#ifdef PRINT_EVENTS
	fprintf (f, "<ReadProposalsConsidered>robotIndex=%d</ReadProposalsConsidered>\n", payload);
#endif
}

void readSupAtFinalDest ROBOT_REQUEST_PARAMS
{
	int res;
	ListNode *iter;
	Coalition *c;

	iter = db->coalitionData.coalitions.front;
	while (iter)
	{
		c = (Coalition*)iter->value;

		if (c->supervisor == payload)
		{
			c->collabData.isSupAtFinalDest = 1;
		}

		iter = iter->next;
	}

	res = CommCore_sendAck (commData, W_SUP_AT_FINAL_DEST);
	assert (0 < res);

	++commData->deleteme;
	if (commData->deleteme > 100)
	{
		printf ("I may be stuck\n");
	}

#ifdef PRINT_EVENTS
	fprintf (f, "<ReadSupAtFinalDest>robotIndex=%d</ReadSupAtFinalDest>\n", payload);
#endif
}


void readUnreachableLocalMapGrid ROBOT_REQUEST_PARAMS
{
	int res, robotIndex, sz, i;
	uchar *gridFromRobot, *boardGrid;

	robotIndex = payload;

	res = CommCore_sendAck (commData, W_LOC_MAP_GRID);
	assert (0 < res);

	sz = ((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1;
	res = CommCore_recvBuffer (commData, sz);
	assert (0 < res);

	gridFromRobot = (uchar*)commData->buffer;
	boardGrid = db->environment.unreachableLocalMapGrid;

	// Set each element in the board's grid if the element was set in either
	// the robot's grid or the board's grid.
	sz = GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS;
	for (i = 0; i < sz; ++i)
	{
		res =
			BitArray_checkElement_index (
				gridFromRobot,
				i/*,
				GLOB_LOC_MAP_GRID_DIMS,
				GLOB_LOC_MAP_GRID_DIMS*/) ||
			BitArray_checkElement_index (
				boardGrid,
				i/*,
				GLOB_LOC_MAP_GRID_DIMS,
				GLOB_LOC_MAP_GRID_DIMS*/);
		BitArray_setElement_index (
			boardGrid,
			i,
			/*GLOB_LOC_MAP_GRID_DIMS,
			GLOB_LOC_MAP_GRID_DIMS,*/
			res);
	}

	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i != robotIndex)
		{
			db->sensorData.statusFromBoard[i].isNewUnreachableLocalMap = 1;
		}
	}

	res = CommCore_sendAck (commData, W_LOC_MAP_GRID);
	assert (0 < res);

#ifdef PRINT_EVENTS
	fprintf (f, "<ReadUnreachableLocalMapGrid>robotIndex=%d</ReadUnreachableLocalMapGrid>\n", robotIndex);
#endif
}

void readUnreachableLocalMapPt ROBOT_REQUEST_PARAMS
{
	int res, i, robotIndex;
	PointI pt;

	robotIndex = (payload & 0xEE000000) >> 24;
	pt.x = (payload & 0x00EEE000) >> 12;
	pt.y = (payload & 0x00000EEE);
	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i != robotIndex)
		{
			db->sensorData.statusFromBoard[i].isNewUnreachableLocalMap = 1;
		}
	}

	res = CommCore_sendAck (commData, W_LOC_MAP_PT);
	assert (0 < res);

#ifdef PRINT_EVENTS
	fprintf (f, "<ReadUnreachableLocalMapPt>robotIndex=%d pt=(%d,%d)</ReadUnreachableLocalMapPt>\n", robotIndex, pt.x, pt.y);
#endif
}
































void readLocalMap ROBOT_REQUEST_PARAMS
{
	int res;
	int sz;
	int index;
	int isMoreDataComing;
	uchar *bufferPtr;
	CompressedImageWithMapInfo *compressedImageWithMapInfo;

	res = CommCore_sendAck (commData, W_LOCAL_MAP);
	assert (0 < res);

	// Recv profit data and (index, isMoreDataComing)
	compressedImageWithMapInfo = (CompressedImageWithMapInfo*)malloc (sizeof (CompressedImageWithMapInfo));

	sz = sizeof (LocalMapInfo) + (sizeof (int) * 2);
	res = CommCore_recvBuffer (commData, sz);
	assert (0 < res);

	bufferPtr = (uchar*)commData->buffer;
	memcpy (&compressedImageWithMapInfo->mapInfo, bufferPtr, sizeof (LocalMapInfo));
	bufferPtr += sizeof (LocalMapInfo);
	memcpy (&index, bufferPtr, sizeof (int));
	bufferPtr += sizeof (int);
	memcpy (&isMoreDataComing, bufferPtr, sizeof (int));

	res = CommCore_sendAck (commData, W_LOCAL_MAP);
	assert (0 < res);

	CommCore_readMap(commData, &compressedImageWithMapInfo->image, 1);

	List_pushValue (&db->sensorData.incorpMaps, compressedImageWithMapInfo);

	// Set flags on board
	db->sensorData.statusFromRobot[index].isMoreDataComing = isMoreDataComing;
//	db->sensorData.statusFromRobot[index].isNewMapData = 1;
	if (!isMoreDataComing)
	{
		db->sensorData.mapStatus.isNewMapData = 1;
	}

#ifdef PRINT_EVENTS
	fprintf (f, "<ReadLocalMap>robotIndex=%d isMoreDataComing=%d</ReadLocalMap>\n", index, isMoreDataComing);
#endif
}





























#define ROBOT_REQUEST_LOCAL_PARAMS (db, commData, payload, f); break
#ifdef BOARD
void BoardComm_processRobotRequest (BoardDatabase *db, CommData *commData, const COMM_FLAG flag, const int payload, FILE *f)
{
	enterBoardData();
	db->groupData.experimentStatus.updateFlag = 1;

	switch (flag)
	{

	case R_STATUS:
		writeStatus ROBOT_REQUEST_LOCAL_PARAMS;

	case R_GRIDS:
		writeGrids ROBOT_REQUEST_LOCAL_PARAMS;

	case R_LOC_MAP_GRID:
		writeUnreachableLocalMapGrid ROBOT_REQUEST_LOCAL_PARAMS;

	case R_GLOBAL_MAP_SECTION:
		writeMapSection ROBOT_REQUEST_LOCAL_PARAMS;

	case R_GROUP:
		writeGroupData ROBOT_REQUEST_LOCAL_PARAMS;

	case R_PROPOSALS:
		writeProposals ROBOT_REQUEST_LOCAL_PARAMS;

	case R_BIDS:
		writeBids ROBOT_REQUEST_LOCAL_PARAMS;

	case R_COALITION_FLAGS:
		writeCoalitionFlags ROBOT_REQUEST_LOCAL_PARAMS;

	case R_COALITIONS:
		writeCoalitions ROBOT_REQUEST_LOCAL_PARAMS;








	case W_FINISHED:
		readFinished ROBOT_REQUEST_LOCAL_PARAMS;

	case W_ROBOT_DATA:
		readRobotData ROBOT_REQUEST_LOCAL_PARAMS;

	case W_LOCAL_MAP:
		readLocalMap ROBOT_REQUEST_LOCAL_PARAMS;

	case W_LEAVE_PROV_DATA:
		readLeaveProvisionalMapData ROBOT_REQUEST_LOCAL_PARAMS;

	case W_LEAVE_COALITION:
		readLeaveCoalition ROBOT_REQUEST_LOCAL_PARAMS;

	case W_PROPOSAL:
		readProposal ROBOT_REQUEST_LOCAL_PARAMS;

	case W_BID:
		readBid ROBOT_REQUEST_LOCAL_PARAMS;

	case W_SUP_AT_FINAL_DEST:
		readSupAtFinalDest ROBOT_REQUEST_LOCAL_PARAMS;

	case W_LOC_MAP_GRID:
		readUnreachableLocalMapGrid ROBOT_REQUEST_LOCAL_PARAMS;

	case W_LOC_MAP_PT:
		readUnreachableLocalMapPt ROBOT_REQUEST_LOCAL_PARAMS;

	case W_PROPOSALS_CONSIDERED:
		readProposalsConsidered ROBOT_REQUEST_LOCAL_PARAMS;

	case W_ROBOT_LOGFILE:
		readRobotLogfile ROBOT_REQUEST_LOCAL_PARAMS;

#if defined(USE_CLOUD)
	case W_GTEP_JOB:
		BoardJobs_readGtepJob ROBOT_REQUEST_LOCAL_PARAMS;

	case W_EXP_JOB:
		BoardJobs_readExpJob ROBOT_REQUEST_LOCAL_PARAMS;
#endif



	default:
		break;
	}

	leaveBoardData();
}
#endif
#undef ROBOT_REQUEST_LOCAL_PARAMS
#endif
