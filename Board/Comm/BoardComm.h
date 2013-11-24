#include "../../Common/RobotDefs.h"

#ifndef ROBOT

/*!
\file BoardComm.h
\brief Accept requests from robots.
*/
#ifndef BOARD_COMMUNICATION_H
#define BOARD_COMMUNICATION_H



#include "../Data/BoardDatabase.h"
#include "../Map/BoardMapProcessing.h"
#include "../../Common/Comm/CommCore.h"




//! Get packet from robot and determine what processing to do based on flag
void BoardComm_processRobotRequest (BoardDatabase *db, CommData *commData, const COMM_FLAG flag, const int payload, FILE *f);




#endif // ifndef
#endif
