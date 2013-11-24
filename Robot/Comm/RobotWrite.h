#include "../../Common/RobotDefs.h"



/*!
\file RobotWrite.h
\brief Send data to blackboard.
*/
#ifndef ROBOT_WRITE_H
#define ROBOT_WRITE_H

#include "RobotWriteImpl.h"
#include "../Data/RobotDatabase.h"
#include "../../Common/Uncertainty.h"


#ifndef BOARD

void RobotWrite_writeFinished (RobotDatabase *db);

void RobotWrite_writeLeaveProvisionalMapData (RobotDatabase *db);

void RobotWrite_leaveCoalition (RobotDatabase *db, const int explorerOrSupervisor);

void RobotWrite_writeRobotData (RobotDatabase *db);

//! Send the path to the robot's logfile to the board s.th. it can be written to the board's logfile for analysis afterwards.
void RobotWrite_writeExperimentPath (RobotDatabase *db);

void RobotWrite_writeLocalMap (RobotDatabase *db);

void RobotWrite_updateBoard (RobotDatabase *db);

void RobotWrite_writeProposal (
	RobotDatabase *db,
	Proposal *prop);
#if 0
void RobotWrite_writeProposal (RobotDatabase *db);
#endif

//void RobotWrite_writeBid (RobotDatabase *db, const int supervisor, const float bid);
void RobotWrite_writeBid (RobotDatabase *db, const int supervisor, const float nSteps);

void RobotWrite_writeProposalsConsidered (RobotDatabase *db);

void RobotWrite_supervisorAtFinalDest (RobotDatabase *db);

#if defined (USE_CLOUD)
void RobotWrite_writeGtepJob (
	RobotDatabase *db,
	GotoExpPtPayload *payload,
	GotoExpPtData *gtepData);
#endif // defined (USE_CLOUD)

#if (defined(SIMULATION) && defined(USE_CLOUD)) || defined(BOARD)
#include "../../Board/Data/BoardSensorData.h"
#endif

void RobotWrite_updateUnreachableLocalMapGrid (
#if defined(SIMULATION) || defined(BOARD)
	uchar *boardUnreachableLocalMapGrid,
	BoardSensorData *boardSensorData,
#else
	RobotDatabase *db,
#endif
	const uchar *unreachableLocalMapGrid,
	const int index);

void RobotWrite_updateUnreachableLocalMapPt (
#if defined(SIMULATION) || defined(BOARD)
	uchar *boardUnreachableLocalMapGrid,
	BoardSensorData *boardSensorData,
#else
	RobotDatabase *db,
#endif
	const int index,
	const PointI pt);



#endif // BOARD


#endif // ifndef

