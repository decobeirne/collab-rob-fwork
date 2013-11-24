#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file RobotCommImpl.h
\brief Send data to blackboard.
*/
#ifndef ROBOT_COMM_IMPL_H
#define ROBOT_COMM_IMPL_H


#include "../Data/RobotDatabase.h"
#include "../../Common/Comm/CommCore.h"

#ifdef IS_LINUX
#include "../../Gumstix/cmu.h"
#endif


int RobotReadImpl_setupSocket (CommData *commData);

int RobotReadImpl_closeSocket (CommData *commData);





















void RobotReadImpl_readGroup (RobotDatabase *db);

void RobotReadImpl_readStatus (RobotDatabase *db, MapStatusFromBoard *s);

void RobotReadImpl_readGrids (RobotDatabase *db);

void RobotReadImpl_readUnreachableLocalMapGrid (RobotDatabase *db);

void RobotReadImpl_readMap (RobotDatabase *db);

void RobotReadImpl_readProposals (RobotDatabase *db, List *proposalList);

void RobotReadImpl_readBids (RobotDatabase *db, List *bidList);

//! Read proposalSuccesful and bidSuccessful flags from board.
void RobotReadImpl_readCoalitionFlags (RobotDatabase *db, int flags[2]);

//! Read coalitions in which robot is an explorer or supervisor.
void RobotReadImpl_readCoalitions (RobotDatabase *db);

#endif // ifndef
#endif
