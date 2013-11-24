#include "../../Common/RobotDefs.h"

#ifndef ROBOT

/*!
\file BoardJobs.h
\brief Run jobs for robots that are too expensive for ARM processors.
*/
#ifndef BOARD_JOBS_H
#define BOARD_JOBS_H

#include "../Data/BoardDatabase.h"


void BoardJobs_readGtepJob ROBOT_REQUEST_PARAMS;

void BoardJobs_readExpJob ROBOT_REQUEST_PARAMS;


#endif // ifndef
#endif // ifndef ROBOT
