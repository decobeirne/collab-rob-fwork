#include "../../Common/RobotDefs.h"

#ifndef ROBOT
#include "BoardEnvironment.h"
#include "../../Common/BitArray.h"

BoardEnvironment initBoardEnvironment()
{
	BoardEnvironment b;

	//*** create maps
	Image_ctor (
		&b.map,
		ENVIR_DIMS,
		ENVIR_DIMS,
		1);

	Image_ctor (
		&b.tempMap,
		ENVIR_DIMS,
		ENVIR_DIMS,
		1);

	Image_ctor (
		&b.incMap,
		LOC_MAP_DIMS,
		LOC_MAP_DIMS,
		1);

#if defined(USE_CLOUD)
	Image_ctor (
		&b.incMap2,
		LOC_MAP_DIMS,
		LOC_MAP_DIMS,
		1);
#endif

	b.origObstList = initList();

	b.expGrid[0][0] = 0;
	b.expGrid[29][29] = 0;

	BitArray_reset (b.unreachableLocalMapGrid, GLOB_LOC_MAP_GRID_DIMS, GLOB_LOC_MAP_GRID_DIMS);
	BitArray_reset (b.obstructedGrid, NAV_GRID_DIMS, NAV_GRID_DIMS);

	return b;
}

void BoardEnvironment_dtor (BoardEnvironment *envir)
{
	Image_dtor (&envir->map);
	Image_dtor (&envir->tempMap);
	Image_dtor (&envir->incMap);
#if defined(USE_CLOUD)
	Image_dtor (&envir->incMap2);
#endif

	List_clear (&envir->origObstList, 1);
}

#endif
