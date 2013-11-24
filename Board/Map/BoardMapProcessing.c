#include "../../Common/RobotDefs.h"

#ifndef ROBOT

#include "BoardMapProcessing.h"
#include "../../Common/BitArray.h"

//! Copy area of global map onto local map
/*!
It is assumed that local maps are centered properly to begin with, i.e. they do
not go over the edge of the global map
*/
void copyLocalMap (Image *map, Image *localMap)
{
	int iLocal, jLocal;
	int iGlobal, jGlobal;

	for (iLocal = 0, iGlobal = localMap->orig.x;
		iLocal < localMap->width;
		++iLocal, ++iGlobal)
	{
		for (jLocal = 0, jGlobal = localMap->orig.y;
			jLocal < localMap->height;
			++jLocal, ++jGlobal)
		{
			Image_setPixel_dontCheck (
				localMap,
				iLocal,
				jLocal,
				Image_getPixel_dontCheck (
				map,
				iGlobal,
				jGlobal));
		}
	}
}

#ifdef CALC_LIVE_MAP_STATS
void BoardMapProcessing_calcAndPrintMapStats (
	BoardDatabase *db)
{
	int i, j, k, l;
	int n, val;
	PointI orig;
	const int nExpCellsPerSupArea = (EXP_GRID_DIMS * EXP_GRID_DIMS * 9);
	const int localMapExhaustedThreshold = (int)(EXP_GRID_DIMS * EXP_GRID_DIMS * 0.6); // note: arbitrary

	fprintf (db->xmlLog, "<GlobalMapStats>index=%d\n", db->groupData.experimentStatus.iteration);
	fprintf (db->xmlLog, "<SupAreasPixelsMapped>");
	for (i = 0; i < SUP_GRID_DIMS; ++i)
	{
		for (j = 0; j < SUP_GRID_DIMS; ++j)
		{
			orig.x = i * SUP_DIFF;
			orig.y = j * SUP_DIFF;

			n = 0;
			for (k = orig.x; k < orig.x + SUP_DIMS; ++k)
			{
				for (l = orig.y; l < orig.y + SUP_DIMS; ++l)
				{
					val = Image_getPixel_dontCheck (db->environment.map, k, l);
					val = abs (127 - val);
					n += (val != 0);
				}
			}
			fprintf (db->xmlLog, "id=(%d,%d) nPixelsMapped=%d; ", i, j, n);
		}
	}
	fprintf (db->xmlLog, "</SupAreasPixelsMapped>\n");

	fprintf (db->xmlLog, "<SupAreasExpCellsMapped>");
	for (i = 0; i < SUP_GRID_DIMS; ++i)
	{
		for (j = 0; j < SUP_GRID_DIMS; ++j)
		{
			n = 0;
			for (k = i * 3; k < (i * 3 + 5); ++k)
			{
				for (l = j * 3; l < (j * 3 + 5); ++l)
				{
					n += (db->environment.localMapGrid[k][l] > localMapExhaustedThreshold); // local map exhausted
				}
			}

			fprintf (db->xmlLog, "id=(%d,%d) nExpCellsMapped=%d nLocalMapsExhausted=%d; ",
				i, j,
//				(nExpCellsPerSupArea - db->environment.supGrid[i][j]),
				(db->environment.supGrid[i][j]),
				n);
		}
	}
	fprintf (db->xmlLog, "</SupAreasExpCellsMapped>\n");
	fprintf (db->xmlLog, "</GlobalMapStats>\n");
}
#endif

//! Incorporate new data into map and carry out processing
/*!
In the simulation, robots submit data to the board by copying approximations
of their local maps onto the boards array of region lists.
*/
void BoardMapProcessing_processMapData (BoardDatabase *db)
{
	// get handle to temp map for drawing navigation map and calculating obstacles
	BoardEnvironment *env = &db->environment;
	PointI updateBl, updateTr;

	if (1 == db->sensorData.mapStatus.isNewMapData)
	{
		updateBl.x = updateBl.y = MAX_INT32;
		updateTr.x = updateTr.y = MIN_INT32;

		BoardMapIntegration_wipeProvisionalMapData (db, &updateBl, &updateTr);

		if (updateBl.x != MAX_INT32)
		{
			BoardMapIntegration_refreshMapArea (db, updateBl, updateTr);
		}

		BoardMapIntegration_incorporateNewMapData (db, &updateBl, &updateTr);

		BoardMapProcessing_updateEffectedAreaForRobots (db, updateBl, updateTr);

		// the bottom left and top right now represent the most extreme origins of local maps
		updateTr.x += LOC_MAP_DIMS;
		updateTr.y += LOC_MAP_DIMS;

		updateBl = PointI_calcExpCellIndex_check (updateBl, EXP_AREA, GLOB_EXP_GRID_DIMS, GLOB_EXP_GRID_DIMS);
		updateTr = PointI_calcExpCellIndex_check (updateTr, EXP_AREA, GLOB_EXP_GRID_DIMS, GLOB_EXP_GRID_DIMS);

		BoardMapProcessing_updNavMap (db, env->map, env->tempMap);
		BoardMapProcessing_updateMapGrid (db, updateBl, updateTr);
		BoardMapProcessing_resetNewDataFlags (db);
	}

#ifdef CALC_LIVE_MAP_STATS
	BoardMapProcessing_calcAndPrintMapStats (db);
#endif
}


// Draw original sim for robots to read off
/*!
In simulation robots fabricate their sensor data from the environment 
maintained by the board. The board uses this map for other purposes, 
so it must be reset before robots use it again.
*/
void BoardMapProcessing_refreshSim (BoardDatabase *db)
{
	Image *simHandle = db->environment.tempMap;

	MapCore_displayTerrain (simHandle, &db->environment.origObstList);
}

void BoardMapProcessing_compressMapSection (BoardDatabase *db, const PointI *localMapOrigin, FILE *fstream)
{
	BoardEnvironment *be = &db->environment;

	be->incMap->orig = *localMapOrigin;
	copyLocalMap (be->map, be->incMap);

	MapCore_compressLocalMap (be->incMap, &db->sensorData.compressedMap);

#ifdef PRINT_EVENTS
	fprintf (fstream, "<CompressMapSection>localMapOrigin=(%d,%d) compressedSize=%d</CompressMapSection>\n",
		localMapOrigin->x, localMapOrigin->y, db->sensorData.compressedMap.usedSize);
#endif
}

void BoardMapProcessing_updateObstructedGrid (BoardDatabase *db,
											  Image *navMap)
{
	int i, j;
	int k, l;
	int kStart, lStart, kStop, lStop;
	int numOccupied;
	uchar *obstructedGrid = db->environment.obstructedGrid;
	int obstdDimsX = NAV_GRID_DIMS;
	int obstdDimsY = NAV_GRID_DIMS;

	// empty the grid of obstructed cells first off
	BitArray_reset (
		obstructedGrid,
		obstdDimsX,
		obstdDimsY);

	// iterate through cells: count number of cells that are definitely occupied
	// also calculate the average occupancy certainty for each
	for (i = 0; i < obstdDimsX; ++i)
	{
		for (j = 0; j < obstdDimsY; ++j)
		{
			numOccupied = 0;

			kStart = i * NAV_CELL_AREA;
			lStart = j * NAV_CELL_AREA;
			kStop = kStart + NAV_CELL_AREA;
			lStop = lStart + NAV_CELL_AREA;

			for (k = kStart; k < kStop; ++k)
			{
				for (l = lStart; l < lStop; ++l)
				{
					if (Image_getPixel_dontCheck (navMap, k, l) < BROAD_VEHICLE_TERRAIN)
					{
						numOccupied++;
					}
				}
			}

			// decision: if a cell contains n > 10 occupied points, flag as occupied
			// add this cell to the list of obstructed cells
			if (numOccupied > NAV_CELL_OCCUPIED_THRESHOLD)
			{
				BitArray_setCoords (
					obstructedGrid,
					i,
					j,
					obstdDimsX,
					1);
			}
		}
	}

#ifdef PRINT_MAP_DETAIL
	fprintf (db->xmlLog, "<ObstructedCellGrid>\n");

	BitArray_display (
		obstructedGrid,
		db->xmlLog,
		obstdDimsX,
		obstdDimsY);

	fprintf (db->xmlLog, "</ObstructedCellGrid>\n");
#endif
}

void BoardMapProcessing_updateMapGrid (BoardDatabase *db,
										const PointI updateBl,
										const PointI updateTr)
{
	BoardMapIntegration_updateExpGridCells (
		db,
		updateBl,
		updateTr);

	BoardMapIntegration_updateLocalMapGridCells (
		db,
		(const __int16 (*)[GLOB_EXP_GRID_DIMS])db->environment.expGrid,
		db->environment.localMapGrid);

	MapCore_updateSupGridCells (
		db->xmlLog,
		db->environment.localMapGrid,
		db->environment.supGrid);

#ifdef PRINT_MAP_DETAIL
	BoardDatabase_printExpGridSection (db, updateBl, updateTr);
	BoardDatabase_printLocalMapGrid (db);
	MapCore_printSupGridCells (db->xmlLog, db->environment.supGrid);
#endif
}


//! Determine bottom left and top right bounds of localMapGrid to update
/*!
Given that there are 6  non-overlapping exp cells for each loc map, the bl
and tr exp cells can be converted to bl and tr loc maps. Bottom left loc 
map is the loc map of lowest value that covers the bl exp cell. This is 
calculated as ((coord - 10) / 4). The tr loc map is calculated as (coord / 4).

The bl cells must align to the last non-overlapping cell in each loc map. The
coord - 10 should therefore be a multiple of 4. The tr cells are aligned to the 
first cell in each loc map. The coord should therefore itself be a multiple of 4.
*/
int BoardMapProcessing_determineLocalMapGridUpdateBounds (BoardDatabase *db,
														  PointI *bl,
														  PointI *tr,
														  const PointI *expGridDims,
														  const PointI *localMapGridDims)
{
	// bl
	if (bl->x <= 10)
	{
		bl->x = 0;
	}
	else if (0 != ((bl->x - 10) % 4))
	{
		bl->x = ((bl->x - 10) / 4) * 4 + 10;
		bl->x = (bl->x - 10) / 4;
	}

	if (bl->y <= 10)
	{
		bl->y = 0;
	}
	else if (0 != ((bl->y - 10) % 4))
	{
		bl->y = ((bl->y - 10) / 4) * 4 + 10;
		bl->y = (bl->y - 10) / 4;
	}
	
	// tr
	if (tr->x >= ((expGridDims->x - 1) - 10))
	{
		tr->x = (localMapGridDims->x - 1);
	}
	else if (0 != (tr->x % 4))
	{
		tr->x = (tr->x / 4) * 4;
		tr->x = tr->x / 4;
	}

	if (tr->y >= ((expGridDims->y - 1) - 10))
	{
		tr->y = (localMapGridDims->y - 1);
	}
	else if (0 != (tr->y % 4))
	{
		tr->y = (tr->y / 4) * 4;
		tr->y = tr->y / 4;
	}

	return 0;
}


//! Create nav map from map and calculate obstructed nav cells from this
void BoardMapProcessing_updNavMap (BoardDatabase *db,
									Image *map,
									Image *navMapHandle)
{
	// threshold the image onto a value other that 0, as this is the region colour
	MapCore_thresholdImage (
		map,
		navMapHandle,
		THRESHOLD_TERRAIN,
		OCCUPIED_TERRAIN,
		FREE_TERRAIN,
		1,
		ENVIR_DIMS);

	MapCore_dilateNavMap (
		navMapHandle,
		navMapHandle,
		OCCUPIED_TERRAIN,
		NARROW_OCCUPIED,
		BROAD_OCCUPIED,
		FREE_TERRAIN,
		NARROW_NAV_MAP_DILATE_DIST,
		BROAD_NAV_MAP_DILATE_DIST);

	BoardMapProcessing_updateObstructedGrid (db, navMapHandle);
}


//! Reset flags on board to reflect that all map data submitted has been processed
void BoardMapProcessing_resetNewDataFlags (BoardDatabase *db)
{
	int i;
	for (i = 0; i < N_ROBOTS; ++i)
	{
		db->sensorData.statusFromBoard[i].isNewGlobalMap = 1;
	}

	db->sensorData.mapStatus.isNewMapData = 0;
	db->sensorData.mapStatus.isNewDataToDisplay = 1;
}

//! Update area of global map that has new data in it.
/*!
Bl and tr represent the local map origins of new local maps submitted
*/
void BoardMapProcessing_updateEffectedAreaForRobots (BoardDatabase *db,
													 const PointI bl,
													 const PointI tr)
{
	int i;
	Vector4I *area;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		area = &db->sensorData.statusFromBoard[i].mapAreaEffectedForRobot;
		if (area->x > bl.x){ area->x = bl.x; }
		if (area->y > bl.y){ area->y = bl.y; }
		if (area->z < tr.x){ area->z = tr.x; }
		if (area->w < tr.y){ area->w = tr.y; }
	}
}

#endif
