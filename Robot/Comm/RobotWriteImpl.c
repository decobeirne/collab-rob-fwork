#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "RobotWriteImpl.h"
#include "../../Common/Map/MapCore.h"
#include "../../Common/Comm/CommCore.h"




void RobotWriteImpl_writeLocalMap (RobotDatabase *db,
							const int isMoreDataComing)
{
	int res;
	int index;
	int sz;

	res = CommCore_sendInitPacket (&db->commData, W_LOCAL_MAP, 0);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_LOCAL_MAP);
	assert (0 < res);

	// Send profit data
	CommCore_clearBuffer (&db->commData);
	CommCore_addItemToBuffer (&db->commData, &db->environment.mapInfoToSubmit, sizeof (LocalMapInfo));
	index = db->status.index;
	CommCore_addItemToBuffer (&db->commData, &index, sizeof (int));
	CommCore_addItemToBuffer (&db->commData, &isMoreDataComing, sizeof (int));

	sz = sizeof (LocalMapInfo) + (sizeof (int) * 2);
	res = CommCore_sendBuffer (&db->commData, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_LOCAL_MAP);
	assert (0 < res);

	CommCore_sendMap (&db->commData, &db->environment.mapToSubmit);
}
















void RobotWriteImpl_writeProposal (RobotDatabase *db, Proposal *proposal)
{
	int index;
	int res;
	int sz = sizeof (int) + sizeof (Proposal);

	res = CommCore_sendInitPacket (&db->commData, W_PROPOSAL, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_PROPOSAL);
	assert (0 < res);

	index = db->status.index;
	CommCore_clearBuffer (&db->commData);
	CommCore_addItemToBuffer (&db->commData, &index, sizeof (int));
	CommCore_addItemToBuffer (&db->commData, proposal, sizeof (Proposal));

	res = CommCore_sendBuffer (&db->commData, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_PROPOSAL);
	assert (0 < res);
}

void RobotWriteImpl_writeBid (RobotDatabase *db, Bid *bid)
{
	int index;
	int res;
	int sz = sizeof (int) + sizeof (Bid);

	res = CommCore_sendInitPacket (&db->commData, W_BID, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_BID);
	assert (0 < res);

	index = db->status.index;
	CommCore_clearBuffer (&db->commData);
	CommCore_addItemToBuffer (&db->commData, &index, sizeof (int));
	CommCore_addItemToBuffer (&db->commData, bid, sizeof (Bid));

	res = CommCore_sendBuffer (&db->commData, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_BID);
	assert (0 < res);
}

void RobotWriteImpl_writeProposalsConsidered (RobotDatabase *db)
{
	int res;
	int index = db->status.index;

	res = CommCore_sendInitPacket (&db->commData, W_PROPOSALS_CONSIDERED, index);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_PROPOSALS_CONSIDERED);
	assert (0 < res);
}

void RobotWriteImpl_writeSupAtFinalDest (RobotDatabase *db)
{
	int res;
	int index = db->status.index;

	res = CommCore_sendInitPacket (&db->commData, W_SUP_AT_FINAL_DEST, index);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_SUP_AT_FINAL_DEST);
	assert (0 < res);
}

void RobotWriteImpl_writeFinished (RobotDatabase *db)
{
	int res;
	int index = db->status.index;

	res = CommCore_sendInitPacket (&db->commData, W_FINISHED, index);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_FINISHED);
	assert (0 < res);
}

void RobotWriteImpl_writeLeaveProvisionalMapData (RobotDatabase *db)
{
	int res;
	int index = db->status.index;

	res = CommCore_sendInitPacket (&db->commData, W_LEAVE_PROV_DATA, index);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_LEAVE_PROV_DATA);
	assert (0 < res);
}

void RobotWriteImpl_writeLeaveCoalition (RobotDatabase *db, const int explorerOrSupervisor)
{
	int res;
	int index = db->status.index;
	int payload = (explorerOrSupervisor << 31) | index;

	res = CommCore_sendInitPacket (&db->commData, W_LEAVE_COALITION, payload);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_LEAVE_COALITION);
	assert (0 < res);
}

void RobotWriteImpl_writeRobotData (RobotDatabase *db, RobotData *d)
{
	int index;
	int res;
	int sz = sizeof (int) + sizeof (RobotData);

	res = CommCore_sendInitPacket (&db->commData, W_ROBOT_DATA, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_ROBOT_DATA);
	assert (0 < res);

	index = db->status.index;
	CommCore_clearBuffer (&db->commData);
	CommCore_addItemToBuffer (&db->commData, &index, sizeof (int));
	CommCore_addItemToBuffer (&db->commData, d, sizeof (RobotData));

	res = CommCore_sendBuffer (&db->commData, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_ROBOT_DATA);
	assert (0 < res);
}

extern char experimentDirName[128];
void RobotWriteImpl_writeExperimentPath (RobotDatabase *db)
{
	int index;
	int res;
	int sz = sizeof (int) + strlen (experimentDirName) + 1;

	res = CommCore_sendInitPacket (&db->commData, W_ROBOT_LOGFILE, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_ROBOT_LOGFILE);
	assert (0 < res);

	index = db->status.index;
	CommCore_clearBuffer (&db->commData);
	CommCore_addItemToBuffer (&db->commData, &index, sizeof (int));
	CommCore_addItemToBuffer (&db->commData, experimentDirName, strlen (experimentDirName) + 1);

	res = CommCore_sendBuffer (&db->commData, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_ROBOT_LOGFILE);
	assert (0 < res);
}

void RobotWriteImpl_updateUnreachableLocalMapGrid (
	RobotDatabase *db,
	const uchar *unreachableLocalMapGrid,
	const int index)
{
	int res;

	res = CommCore_sendInitPacket (&db->commData, W_LOC_MAP_GRID, index);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_LOC_MAP_GRID);
	assert (0 < res);

	CommCore_clearBuffer (&db->commData);
	CommCore_addItemToBuffer (
		&db->commData,
		unreachableLocalMapGrid,
		((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1);

	res = CommCore_recvAck (&db->commData, W_LOC_MAP_GRID);
	assert (0 < res);
}

void RobotWriteImpl_updateUnreachableLocalMapPt (
	RobotDatabase *db,
	const PointI pt,
	const int index)
{
	int res;
	int payload;

	// Put all values in 1 int: index, pt.x, pt.y
	payload = (index << 20) + (pt.x << 10) + pt.y;

	res = CommCore_sendInitPacket (&db->commData, W_LOC_MAP_PT, payload);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_LOC_MAP_PT);
	assert (0 < res);
}
















#if defined(USE_CLOUD)

void RobotWriteImpl_writeGtepJob (RobotDatabase *db,
						   GotoExpPtPayload *gtepPayload,
						   GotoExpPtData *gtepData)
{
	int index;
	int res;
	int sz;

	res = CommCore_sendInitPacket (&db->commData, W_GTEP_JOB, 0);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_GTEP_JOB);
	assert (0 < res);

	index = db->status.index;
	CommCore_clearBuffer (&db->commData);
	CommCore_addItemToBuffer (&db->commData, gtepPayload, sizeof (GotoExpPtPayload));

	sz = sizeof (GotoExpPtPayload);
	res = CommCore_sendBuffer (&db->commData, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_GTEP_JOB);
	assert (0 < res);

	MapCore_compressLocalMap (db->environment.navMap, &db->environment.mapToSubmit);
	CommCore_sendMap (&db->commData, &db->environment.mapToSubmit);

	MapCore_compressLocalMap (db->sensorData.localMap, &db->environment.mapToSubmit);
	CommCore_sendMap (&db->commData, &db->environment.mapToSubmit);

	CommCore_sendList (&db->commData, &db->environment.unreachableIkDests, sizeof(PointI), 1);

	sz = sizeof (GotoExpPtData);
	res = CommCore_recvBuffer (&db->commData, sz);
	assert (0 < res);

	memcpy (gtepData, db->commData.buffer, sz);
}

void RobotWriteImpl_writeExpJob (
	RobotDatabase *db,
	ExplorationPayload *expPayload,
	ExplorationData *expData)
{
	int index;
	int res;
	int sz;

	res = CommCore_sendInitPacket (&db->commData, W_EXP_JOB, 0);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_EXP_JOB);
	assert (0 < res);

	index = db->status.index;
	CommCore_clearBuffer (&db->commData);
	CommCore_addItemToBuffer (&db->commData, expPayload, sizeof (ExplorationPayload));

	sz = sizeof (ExplorationPayload);
	res = CommCore_sendBuffer (&db->commData, sz);
	assert (0 < res);

	res = CommCore_recvAck (&db->commData, W_EXP_JOB);
	assert (0 < res);

	MapCore_compressLocalMap (db->environment.navMap, &db->environment.mapToSubmit);
	CommCore_sendMap (&db->commData, &db->environment.mapToSubmit);

	MapCore_compressLocalMap (db->sensorData.localMap, &db->environment.mapToSubmit);
	CommCore_sendMap (&db->commData, &db->environment.mapToSubmit);

	CommCore_sendList (&db->commData, &db->environment.unreachableIkTargets, sizeof (PointI), 1);

	sz = sizeof (db->environment.unreachableExpCellGrid) + sizeof (ExplorationData);
	res = CommCore_recvBuffer (&db->commData, sz);
	assert (0 < res);

	sz = sizeof (db->environment.unreachableExpCellGrid);
	memcpy (db->environment.unreachableExpCellGrid, db->commData.buffer, sz);
	memcpy (expData, db->commData.buffer + sz, sizeof (ExplorationData));
}
#endif // defined(USE_CLOUD)


#endif // ifndef BOARD

