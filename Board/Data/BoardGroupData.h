#include "../../Common/RobotDefs.h"

#ifndef ROBOT

/*!
\file BoardGroupData.h
\brief Status of group of robots.
*/
#ifndef BOARD_GROUP_DATA_H
#define BOARD_GROUP_DATA_H

#include "../../Common/RobotCore.h"


//! Boards data relating to the group of robots
typedef struct BoardGroupData
{
	ExperimentStatus experimentStatus;
	RobotData robots[MAX_N_ROBOTS];
} BoardGroupData;

//! Constructor
BoardGroupData initBoardGroupData();

//! Destructor
void BoardGroupData_dtor (BoardGroupData *g);

#endif // ifndef
#endif