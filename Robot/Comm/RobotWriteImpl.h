#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file RobotWriteImpl.h
\brief Write data to blackboard.
*/
#ifndef ROBOT_WRITE_IMPL_H
#define ROBOT_WRITE_IMPL_H


#include "../Data/RobotDatabase.h"
//#include "../../Common/Comm/CommCore.h"

#ifdef IS_LINUX
//#include "../../Gumstix/cmu.h"
#endif




void RobotWriteImpl_writeFinished (RobotDatabase *db);

void RobotWriteImpl_writeLeaveProvisionalMapData (RobotDatabase *db);

void RobotWriteImpl_writeLeaveCoalition (RobotDatabase *db, const int explorerOrSupervisor);

void RobotWriteImpl_writeRobotData (RobotDatabase *db, RobotData *d);

void RobotWriteImpl_writeExperimentPath (RobotDatabase *db);

void RobotWriteImpl_writeLocalMap (RobotDatabase *db, const int isMoreDataComing);

void RobotWriteImpl_writeProposal (RobotDatabase *db, Proposal *proposal);

void RobotWriteImpl_writeBid (RobotDatabase *db, Bid *bid);

void RobotWriteImpl_writeProposalsConsidered (RobotDatabase *db);

void RobotWriteImpl_writeSupAtFinalDest (RobotDatabase *db);

void RobotWriteImpl_updateUnreachableLocalMapGrid (
	RobotDatabase *db,
	const uchar *unreachableLocalMapGrid,
	const int index);

void RobotWriteImpl_updateUnreachableLocalMapPt (
	RobotDatabase *db,
	const PointI pt,
	const int index);


#if defined(USE_CLOUD)
void RobotWriteImpl_writeGtepJob (
	RobotDatabase *db,
	GotoExpPtPayload *payload,
	GotoExpPtData *gtepData);

void RobotWriteImpl_writeExpJob (
	RobotDatabase *db,
	ExplorationPayload *expPayload,
	ExplorationData *expData);

#endif








#endif // if !defined(ROBOT_WRITE_IMPL_H)
#endif // BOARD
