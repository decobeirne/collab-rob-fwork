#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "RobotReadImpl.h"
#include "../../Common/BitArray.h"


#ifdef IS_LINUX
int RobotReadImpl_setupRobotSocket_linux (CommData *commData)
{
	int sockfd;
	struct sockaddr_in their_addr;

	printf ("Setting up linux socket connection to %s.\n", (char*)commData->address);

	if (-1 == (sockfd = ((int) socket (AF_INET, SOCK_STREAM, 0))))
	{
		printf ("Error socket\n");
		return -1;
	}

	their_addr.sin_family = AF_INET;
	their_addr.sin_port = htons (commData->port);

#ifdef IS_GUMSTIX
	inet_aton (commData->address, &their_addr.sin_addr);
#else
	inet_aton ((char*)commData->address, &their_addr.sin_addr);
#endif

	bzero (&(their_addr.sin_zero), sizeof (their_addr.sin_zero));

	if (-1 == connect (sockfd, (struct sockaddr*)&their_addr, sizeof (struct sockaddr)))
	{
		printf ("Error connect\n");
		return -1;
	}

	commData->socket = sockfd;

	printf ("Connected to host %s:%d\n", commData->address, commData->port);
	return 1;
}

#else

int RobotReadImpl_setupRobotSocket_win (CommData *commData)
{
	WSADATA wsaData;
	SOCKADDR_IN ServerAddr;
	int res;

	printf ("Setting up windows socket connection.\n");

	if (0 != WSAStartup(MAKEWORD(2,2), &wsaData))
	{
		printf ("Error starting up WSA\n");
		return -1;
	}

	commData->socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (commData->socket == INVALID_SOCKET)
	{
		printf ("Client: socket() failed! Error code: %ld\n", WSAGetLastError());
		WSACleanup();
		return -1;
	}

	ServerAddr.sin_family = AF_INET;
	ServerAddr.sin_port = htons(commData->port);
	ServerAddr.sin_addr.s_addr = inet_addr(commData->address);

	res = connect (commData->socket, (SOCKADDR *) &ServerAddr, sizeof(ServerAddr));
	if (res != 0)
	{
		printf ("Client: connect() failed! Error code: %ld\n", WSAGetLastError());
		closesocket (commData->socket);
		WSACleanup();
		return -1;
	}

	getsockname (commData->socket, (SOCKADDR*)&ServerAddr, (int *)sizeof (ServerAddr));
	printf ("Connected to %s:%d\n", inet_ntoa (ServerAddr.sin_addr),  htons (ServerAddr.sin_port));

	return 1;
}
#endif

//! Setup socket connection with board
int RobotReadImpl_setupSocket (CommData *commData)
{
#ifdef RERUNNING_ROBOT
	return 0;
#endif

#ifdef IS_LINUX
	return RobotReadImpl_setupRobotSocket_linux (commData);
#else
	return RobotReadImpl_setupRobotSocket_win (commData);
#endif
}

int RobotReadImpl_closeSocket (CommData *commData)
{
#ifdef RERUNNING_ROBOT
	return 0;
#endif

#ifdef IS_LINUX

#else

	if (shutdown (commData->socket, SD_SEND) != 0)
	{
		printf("Error shutting down %d\n", WSAGetLastError());
		return -1;
	}
	if (closesocket (commData->socket) != 0)
	{
		printf("Error closing %d\n", WSAGetLastError());
		return -1;
	}
	WSACleanup();
#endif

	return 1;
}




















void RobotReadImpl_readMap (RobotDatabase *db)
{
	int res;

	res = CommCore_sendInitPacket (&db->commData, R_GLOBAL_MAP_SECTION, sizeof (PointI));
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, R_GLOBAL_MAP_SECTION);
	assert (0 < res);

	// Send map origin
	CommCore_clearBuffer (&db->commData);
	CommCore_addItemToBuffer (&db->commData,&db->sensorData.localMap->orig, sizeof (PointI));

	res = CommCore_sendBuffer (&db->commData, sizeof (PointI));
	assert (0 < res);

	CommCore_readMap (&db->commData, &db->environment.mapFromBoard, 0);

	res = CommCore_recvAck (&db->commData, W_MAP);
	assert (0 < res);
}

void RobotReadImpl_readGroup (RobotDatabase *db)
{
	int res;
	int sz = sizeof (RobotData) * N_ROBOTS;

	res = CommCore_sendInitPacket (&db->commData, R_GROUP, db->status.index);
	assert (0 < res);

	res = CommCore_recvBuffer (&db->commData, sz);
	assert (0 < res);

	memcpy (db->groupData.robots, db->commData.buffer, sz);
}

void RobotReadImpl_readStatus (RobotDatabase *db, MapStatusFromBoard *s)
{
	int index;
	int res;
	index = db->status.index;

	res = CommCore_sendInitPacket (&db->commData, R_STATUS, index);
	assert (0 < res);

	res = CommCore_recvBuffer (&db->commData, sizeof (MapStatusFromBoard));
	assert (0 < res);

	memcpy (s, db->commData.buffer, sizeof (MapStatusFromBoard));
}

void RobotReadImpl_readGrids (RobotDatabase *db)
{
	int sz;
	int res;

	// Local maps
	res = CommCore_sendInitPacket (&db->commData, R_GRIDS, 0);
	assert (0 < res);

	sz = sizeof (__int16) * GLOB_LOC_MAP_GRID_DIMS * GLOB_LOC_MAP_GRID_DIMS;
	res = CommCore_recvBuffer (&db->commData, sz);
	assert (0 < res);

	memcpy (db->environment.localMapGrid, db->commData.buffer, sz);

	// Supervision areas
	res = CommCore_sendInitPacket (&db->commData, R_GRIDS, 0);
	assert (0 < res);

	sz = sizeof (__int16) * SUP_GRID_DIMS * SUP_GRID_DIMS;
	res = CommCore_recvBuffer (&db->commData, sz);
	assert (0 < res);

	memcpy (db->environment.supGrid, db->commData.buffer, sz);

	// Occupied cells
	res = CommCore_sendInitPacket (&db->commData, R_GRIDS, 0);
	assert (0 < res);

	sz = sizeof (uchar) * ((NAV_GRID_DIMS*NAV_GRID_DIMS)/8)+1;
	res = CommCore_recvBuffer (&db->commData, sz);
	assert (0 < res);

	memcpy (db->environment.obstructedCellGrid, db->commData.buffer, sz);
}

void RobotReadImpl_readUnreachableLocalMapGrid (RobotDatabase *db)
{
	int res, sz, i;
	uchar *grid;
	res = CommCore_sendInitPacket (&db->commData, R_LOC_MAP_GRID, 0);
	assert (0 < res);

	sz = sizeof (uchar) * ((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1;
	res = CommCore_recvBuffer (&db->commData, sz);
	assert (0 < res);

	grid = (uchar*)db->commData.buffer;

	for (i = 0; i < GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS; ++i)
	{
		if (BitArray_checkElement_index (grid, i))
		{
			BitArray_setElement_index (db->environment.unreachableLocalMapGrid, i, 1);
		}
	}
}

void RobotReadImpl_readProposals (RobotDatabase *db, List *proposalList)
{
	int res;
	int nProposals;
	int i;
	Proposal *ptr;
	Proposal *allocdProposal;

	List_clear (proposalList, 1);

	res = CommCore_sendInitPacket (&db->commData, R_PROPOSALS, db->status.index);
	assert (0 < res);

	res = CommCore_recvBuffer (&db->commData, sizeof (int));
	assert (0 < res);

	memcpy (&nProposals, db->commData.buffer, sizeof (int));

	if (nProposals)
	{
		res = CommCore_sendAck (&db->commData, R_PROPOSALS);
		assert (0 < res);

		res = CommCore_recvBuffer (&db->commData, nProposals * sizeof (Proposal));
		assert (0 < res);

		for (i = 0, ptr = (Proposal*)db->commData.buffer; i < nProposals; ++i, ptr += sizeof (Proposal))
		{
			allocdProposal = (Proposal*)malloc (sizeof (Proposal));
			memcpy (allocdProposal, ptr, sizeof (Proposal));
			List_pushValue (proposalList, allocdProposal);
		}
	}
}

void RobotReadImpl_readBids (RobotDatabase *db, List *bidList)
{
	int res;
	int nBids;
	Bid *ptr;
	Bid *allocdBid;

	List_clear (bidList, 1);

	res = CommCore_sendInitPacket (&db->commData, R_BIDS, db->status.index);
	assert (0 < res);

	res = CommCore_recvBuffer (&db->commData, sizeof (int));
	assert (0 < res);

	memcpy (&nBids, db->commData.buffer, sizeof (int));

	if (nBids)
	{
		res = CommCore_sendAck (&db->commData, R_BIDS);
		assert (0 < res);

		res = CommCore_recvBuffer (&db->commData, nBids * sizeof (Bid));
		assert (0 < res);

		ptr = (Bid*)db->commData.buffer;
		while (nBids--)
		{
			allocdBid = (Bid*)malloc (sizeof (Bid));
			memcpy (allocdBid, ptr, sizeof (Bid));
			List_pushValue (bidList, allocdBid);

			++ptr;
		}
	}
}

void RobotReadImpl_readCoalitionFlags (RobotDatabase *db, int flags[2])
{
	int res;
	int index = db->status.index;

	res = CommCore_sendInitPacket (&db->commData, R_COALITION_FLAGS, index);
	assert (0 < res);

	res = CommCore_recvBuffer (&db->commData, sizeof (int) * 2);
	assert (0 < res);

	memcpy (flags, db->commData.buffer, sizeof (int) * 2);
}

void RobotReadImpl_readCoalitions (RobotDatabase *db)
{
	int res;
	int nSupervisionCoalitions;
	int index = db->status.index;
	Coalition *coalition;
	Coalition *ptr;

	res = CommCore_sendInitPacket (&db->commData, R_COALITIONS, index);
	assert (0 < res);

	res = CommCore_recvBuffer (&db->commData, sizeof (Coalition) + sizeof (int));
	assert (0 < res);

	memcpy (&db->partnerData.explorationCoalition, db->commData.buffer, sizeof (Coalition));
	memcpy (&nSupervisionCoalitions, db->commData.buffer + sizeof (Coalition), sizeof (int));

	if (nSupervisionCoalitions)
	{
		res = CommCore_sendAck (&db->commData, R_COALITIONS);
		assert (0 < res);

		res = CommCore_recvBuffer (&db->commData, nSupervisionCoalitions * sizeof (Coalition));

		ptr = (Coalition*)db->commData.buffer;

		while (nSupervisionCoalitions--)
		{
			coalition = (Coalition*)malloc (sizeof (Coalition));
			memcpy (coalition, ptr, sizeof (Coalition));

			List_pushValue (&db->partnerData.supervisionCoalitions, coalition);

			++ptr;
		}
	}
}










#endif // ifndef BOARD

