#include "../../Common/RobotDefs.h"

#ifndef BOARD

/*!
\file RobotEnvironment.h
\brief Environment data stored on robot.
*/
#ifndef ROBOT_ENVIRONMENT_H
#define ROBOT_ENVIRONMENT_H

#include "../../Common/RobotTypes.h"
#include "../../Common/Image.h"

//! Environment data read/collected by robot
typedef struct RobotEnvironment_
{
	//*** dimensions
	__int16 isNewGlobalMap;							//!< Flags if a new global map has just been read from the board
	__int16 isNewUnreachableLocalMap;					//!< Flags if new local maps have been flagged as exhausted/unreachable

	//*** map grid
	__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS];											//!< Grid of exp cells over loc map
	__int16 expGridInitialVals[EXP_GRID_DIMS][EXP_GRID_DIMS];								//!< Temp grid used for updating
	__int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS];						//!< Average uncertainty over local maps
	__int16 supGrid[SUP_GRID_DIMS][SUP_GRID_DIMS];											//!< Average uncertainty over supervision area
	uchar obstructedCellGrid[((NAV_GRID_DIMS*NAV_GRID_DIMS)/8)+1];							//!< Occupied navigation cells stored as bit array
	uchar unreachableExpCellGrid[((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1];			//!< Grid of unreachable exp cells on current loc map
	uchar unreachableLocalMapGrid[((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1];		//!< Grid of unreachable loc maps
	uchar unreachableSupCellGrid[((SUP_GRID_DIMS*SUP_GRID_DIMS)/8)+1];						//!< Grid of unreachable sup cells
	uchar mappedCellGrid[((EXP_GRID_DIMS*EXP_GRID_DIMS)/8)+1];

	//*** images
	Image *navMap;								//!< Boundary image for navigation, constructed from obstacles

	//*** lists
	List unreachableIkDests;
	List unreachableIkTargets;

	CompressedImage mapFromBoard;
	CompressedImage mapToSubmit;
	LocalMapInfo mapInfoToSubmit;

#if defined(ROBOT) && !defined(IS_GUMSTIX)
	Image *originalSim;							//!< Map to store original simulated environment
	List originalObstacleList;
#endif
} RobotEnvironment;

//! Constructor
RobotEnvironment initRobotEnvironment();

//! Destructor
void RobotEnvironment_dtor (RobotEnvironment *r);


#endif // ifndef
#endif
