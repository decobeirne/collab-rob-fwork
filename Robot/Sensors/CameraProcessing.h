#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef CAM_PROC_H
#define CAM_PROC_H

#include "../../Common/RobotCore.h"
#include "../Data/RobotDatabase.h"

#if defined(IS_GUMSTIX) || defined(RERUNNING_GUMSTIX) || defined(RERUNNING_IMAGES_ONLY)
#include "../../Gumstix/cmu.h"

//! Process real image and compass data.
void CameraProcessing_processSensorData (RobotDatabase *db);

float SensorProcessing_calcOrient (const UINT16 reading);
#endif

#endif
#endif 
