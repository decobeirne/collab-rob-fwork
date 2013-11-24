#include "../Common/RobotDefs.h"

#ifndef ROBOT
/*!
\file BoardManagement.h
\brief Provides coalition management functionality.
*/
#ifndef BOARD_MANAGEMENT_H
#define BOARD_MANAGEMENT_H

#include "../Common/RobotCore.h"
#include "Data/BoardDatabase.h"
#include "../Common/Uncertainty.h"


void BoardManagement_checkTimeElapsed (BoardDatabase *db);

//! Check if robots cannot continue
void BoardManagement_checkMissionStatus (BoardDatabase *db);

//! Assign robots to coalitions based on their proposals/offers
/*!
Robots post proposals and offers on the board. If there are any offers on 
proposals, then the board must examine the potential coalitions. Robots are 
allocated to coalitions in order to maximise the overall profit for the 
group. The appropriate flags are set on the board so that the robots can
determine their new coalition status.
*/
void BoardManagement_manageCoalitions (BoardDatabase *db);

#endif // ifndef
#endif

