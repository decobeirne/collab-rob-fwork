#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file SensorProcessing.h
\brief Processing simulated or actual camera, compass and range data.
*/
#ifndef SENSOR_PROCESSING_H
#define SENSOR_PROCESSING_H

#include "../../Common/RobotDefs.h"

#if defined(IS_GUMSTIX) || defined(RERUNNING_GUMSTIX) || defined(RERUNNING_IMAGES_ONLY)
#include "CameraProcessing.h"
#else
#include "SimulatedProcessing.h"
#endif

//! Process data before synchronising with board.
void SensorProcessing_processSensorData (RobotDatabase *db, const int isFakeMove);

void SensorProcessing_determineSensorRange (RobotDatabase *db);

//! Log any inconsistencies in the robot's state.
void SensorProcessing_sanityCheck (RobotDatabase *db, const char *message);

#endif // ifndef SENSOR_PROCESSING_H
#endif // ifndef BOARD
