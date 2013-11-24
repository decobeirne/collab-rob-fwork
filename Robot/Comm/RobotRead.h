#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file RobotRead.h
\brief Read data from blackboard.
*/
#ifndef ROBOT_READ_H
#define ROBOT_READ_H

#include "RobotReadImpl.h"
#include "../Data/RobotDatabase.h"
#include "../../Common/Uncertainty.h"

#ifdef SIMULATION
#include "../../Board/Board.h"
#endif

void RobotRead_readGroup (RobotDatabase *db);

void RobotRead_readProposals (RobotDatabase *db, List *proposalList);

void RobotRead_readBids (RobotDatabase *db, List *bidList);

void RobotRead_readCoalitionFlags (RobotDatabase *db, int flags[2]);

void RobotRead_readCoalitions (RobotDatabase *db);

void RobotRead_readBoard (RobotDatabase *db);



















#endif // ifndef
#endif
