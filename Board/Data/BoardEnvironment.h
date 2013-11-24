#include "../../Common/RobotDefs.h"

#ifndef ROBOT
/*!
\file BoardEnvironment.h
\brief Global environment.
*/
#ifndef ENVIRONMENT_DATA_H
#define ENVIRONMENT_DATA_H

#include "../../Common/RobotCore.h"


//! Simulated environment
/*!
Data is accumulated from data collected by robots and stored in
RobotEnvironment_ structs.
*/
typedef struct BoardEnvironment_
{
	//*** map grids
	__int16 expGrid[GLOB_EXP_GRID_DIMS][GLOB_EXP_GRID_DIMS];								//!< Average uncertainty calculated over map patches
	__int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS];						//!< Average uncertainty over local maps
	__int16 supGrid[SUP_GRID_DIMS][SUP_GRID_DIMS];											//!< Average uncertainty over supervision areas
	uchar unreachableLocalMapGrid[((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1];		//!< Grid of unreachable loc maps
	uchar obstructedGrid[((NAV_GRID_DIMS * NAV_GRID_DIMS)/8)+1];							//!< Grid of occupied nav cells stored as bit array

	//**** maps/images
	Image *map; //!< Occupancy map built by robots
	Image *tempMap; //!< Temporary map used in calcing nav grid, displaying sim
	Image *incMap; //!< Map for incorporting robots' local maps into global map
#if defined(USE_CLOUD)
	Image *incMap2; //!< Map for running jobs for robots
#endif

	//*** lists
	List origObstList; //!< Original list from file
} BoardEnvironment;

//! Constructor
BoardEnvironment initBoardEnvironment();

//! Destructor
void BoardEnvironment_dtor (BoardEnvironment *envir);


#endif // ifndef
#endif