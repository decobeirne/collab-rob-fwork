#include "../../Common/RobotDefs.h"

#ifndef ROBOT
/*!
\file BoardSensorData.h
\brief Sensor data posted to the blackboard by robots.
*/
#ifndef BOARD_SENSOR_DATA_H
#define BOARD_SENSOR_DATA_H

#include "../../Common/RobotCore.h"



//! Sensor data that robots post on board
typedef struct BoardSensorData
{
	MapStatus mapStatus;
	MapStatusFromRobot statusFromRobot[MAX_N_ROBOTS];
	MapStatusFromBoard statusFromBoard[MAX_N_ROBOTS];
	LocEstimateForRobot locEstimate[MAX_N_ROBOTS];

	List globalMaps;						//!< Full list of floating local maps making up global map
	List incorpMaps;						//!< List of maps to incorporate
	CompressedImage compressedMap;			//!< Temporary map for incorporating data from robots.
} BoardSensorData;

//! Constructor
BoardSensorData initBoardSensorData();

//! Destructor
void BoardSensorData_dtor (BoardSensorData *s);

void BoardSensorData_setMapFlags (BoardSensorData *s);

#endif // ifndef
#endif