#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file RobotGroupData.h
\brief Data on entire group stored on each robot.
*/
#ifndef ROBOT_GROUP_DATA_H
#define ROBOT_GROUP_DATA_H

#include "../../Common/RobotTypes.h"


//! Robot's data relating to other robots in group
typedef struct RobotGroupData_
{
	RobotData robots[MAX_N_ROBOTS];
	__int8 isRobotReachable[MAX_N_ROBOTS];		//!< Array of flags to indicate if a partner is reachable
	PoseSimple prevPoses[MAX_N_ROBOTS];			//!< Array of poses at last iteration
} RobotGroupData;

//! Constructor
RobotGroupData initRobotGroupData();

void RobotGroupData_dtor (RobotGroupData *r);


#endif // ifndef
#endif
