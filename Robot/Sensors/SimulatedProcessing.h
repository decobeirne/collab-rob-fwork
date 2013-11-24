#include "../../Common/RobotDefs.h"

#if !defined(BOARD) && (!defined(IS_GUMSTIX) && !defined(RERUNNING_GUMSTIX))


#ifndef SIMD_PROC_H
#define SIMD_PROC_H

#include "../../Common/RobotCore.h"
#include "../Data/RobotDatabase.h"

//! Process simulated image, calculate occupied terrain and relative pose of closest visible robot.
void SimulatedProcessing_processSensorData (RobotDatabase *db, const int isFakeMove);

#endif 
#endif // !defined(BOARD) && !defined(IS_GUMSTIX) && !defined(RERUNNING_GUMSTIX)

