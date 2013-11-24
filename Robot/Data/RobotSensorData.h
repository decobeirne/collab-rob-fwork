#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file RobotSensorData.h
\brief Simulated or actual sensor data gathered by robot.
*/
#ifndef ROBOT_SENSOR_DATA_H
#define ROBOT_SENSOR_DATA_H

#include "../../Common/RobotTypes.h"
#include "../../Common/Image.h"

//! Data captured by robot's sensors
typedef struct RobotSensorData_
{
	__int8 hasDataBeenIncd;								//! Flags if the robots data has just been incorp'd in the current map
	__int8 isLocalMapEffected;							//!< Flags if the loc map is effected by changes to the glob map
	__int8 isSearchGridUpdateRequired;					//!< Flags if the robot should update its search grids
	__int8 isLocalMapResetRequired;						//!< Flags if the local map has been centred at a particular location
	__int8 isMapDataReady;								//!< Flags if normal data is ready
	__int16 localMapDataIndex;								//!< Robot's index to keep track of subd loc maps
	__int8 areOtherRobotsInLocalMap;
	__int8 isUnincorporatedMapScan;						//!< Flags if the latest map scan has not been integrated into the map.

	UPDATE_NAV_MAP_FLAG isNavMapUpdateRequired;

#ifdef IS_GUMSTIX
	float compassOffset;
#endif
	int isObstacle;										//!< Flags if there is something immediately infront of the robot
	int visRobotIndex;									//!< Index (estd) of rob visible to sensors
	PointF visRobotRelLoc;								//!< Relative loc of visible rob
	UnionVector4F visRobotEstCov;						//!< Error covariance for visual estimate

	int isDataPendingInc;								//!< Flags if the robot is waiting for the board to incorp data

	int scanLimit;										//!< Max number of scans allowed to be incorporated into a local map

	PointI localMapCentre;								//!< Point on which local map is centred
//	PointI localMapResetPt;								//!< Point around which we have to reset/centre the local map.
	PointI localMapResetPt1;
	PointI localMapResetPt2;

	List mapScanList;									//!< List of sensor scans collected by robot

#if defined (IS_GUMSTIX) || defined (RERUNNING_GUMSTIX) || defined(RERUNNING_IMAGES_ONLY)
	Image *camImg;
#endif

	Image *localMap;									//!< Local map that robot writes immediate data to
	Image *tempLocalMap;								//!< Temp map of same dimensions as local map for approximating
} RobotSensorData;

//! Constructor
RobotSensorData initRobotSensorData();

//! Destructor
void RobotSensorData_dtor (RobotSensorData *r);



#endif // ifndef
#endif
