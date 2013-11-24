#include "RobotEnvironment.h"

#ifndef BOARD


#include "../../Common/Map/MapCore.h"
#include "../../Common/BitArray.h"


RobotEnvironment initRobotEnvironment()
{
	int i, j;
	RobotEnvironment r;

	r.isNewGlobalMap = 0;
	r.isNewUnreachableLocalMap = 0;

	//*** map grid
	for (i = 0; i < EXP_GRID_DIMS; ++i)
	{
		for (j = 0; j < EXP_GRID_DIMS; ++j)
		{
			r.expGrid[i][j] = 0;
			r.expGridInitialVals[i][j] = 0;
		}
	}

	for (i = 0; i < GLOB_LOC_MAP_GRID_DIMS; ++i)
	{
		for (j = 0; j < GLOB_LOC_MAP_GRID_DIMS; ++j)
		{
			r.localMapGrid[i][j] = 0;
		}
	}

	for (i = 0; i < SUP_GRID_DIMS; ++i)
	{
		for (j = 0; j < SUP_GRID_DIMS; ++j)
		{
			r.supGrid[i][j] = 0;
		}
	}

	BitArray_reset (r.unreachableExpCellGrid, GLOB_EXP_GRID_DIMS, GLOB_EXP_GRID_DIMS);
	BitArray_reset (r.unreachableLocalMapGrid, GLOB_LOC_MAP_GRID_DIMS, GLOB_LOC_MAP_GRID_DIMS);
	BitArray_reset (r.unreachableSupCellGrid, SUP_GRID_DIMS, SUP_GRID_DIMS);
	BitArray_reset (r.obstructedCellGrid, NAV_GRID_DIMS, NAV_GRID_DIMS);
	BitArray_reset (r.mappedCellGrid, EXP_GRID_DIMS, EXP_GRID_DIMS);

	r.unreachableIkDests = initList();
	r.unreachableIkTargets = initList();

	initCompressedImage (&r.mapToSubmit);
	r.mapToSubmit.buffer = (uchar*)malloc (LOC_MAP_DIMS * LOC_MAP_DIMS);
	r.mapToSubmit.bufferSize = LOC_MAP_DIMS * LOC_MAP_DIMS;

	initCompressedImage (&r.mapFromBoard);
	r.mapFromBoard.buffer = (uchar*)malloc (LOC_MAP_DIMS * LOC_MAP_DIMS);
	r.mapFromBoard.bufferSize = LOC_MAP_DIMS * LOC_MAP_DIMS;

	r.mapInfoToSubmit = initLocalMapInfo();

	Image_ctor (&r.navMap, LOC_MAP_DIMS, LOC_MAP_DIMS, 1);

#if defined(ROBOT) && !defined(IS_GUMSTIX)
	Image_ctor (&r.originalSim, ENVIR_DIMS, ENVIR_DIMS, 1);
	r.originalObstacleList = initList();

	MapCore_setupObstacles (&r.originalObstacleList);
	MapCore_displayTerrain (r.originalSim, &r.originalObstacleList);
#endif

	Image_fill (r.navMap, FREE_TERRAIN);

	return r;
}

void RobotEnvironment_dtor (RobotEnvironment *r)
{
	freeCompressedImage ((void*)&r->mapToSubmit);
	freeCompressedImage ((void*)&r->mapFromBoard);

	List_clear (&r->unreachableIkDests, 1);
	List_clear (&r->unreachableIkTargets, 1);

	Image_dtor (&r->navMap);

#if defined(ROBOT) && !defined(IS_GUMSTIX)
	Image_dtor (&r->originalSim);
	List_clear (&r->originalObstacleList, 1);
#endif
}

#endif
