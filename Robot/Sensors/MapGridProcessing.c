#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "MapGridProcessing.h"

void updateSectionOfExplorationGrid (FILE *f,
									 Image *localMap,
									 __int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS],
									 const PointI bl,
									 const PointI tr)
{
	int i, j;
	int k, l;
	int gridTotal;
	int val;
	PointI expCellOrigin;

	// Grid cells. Iterative through BL and TR inclusive.
	for (i = bl.x; i <= tr.x; ++i)
	{
		expCellOrigin.x = i * EXP_AREA;

		for (j = bl.y; j <= tr.y; ++j)
		{
			expCellOrigin.y = j * EXP_AREA;
			gridTotal = 0;

			for (k = expCellOrigin.x; k < (expCellOrigin.x + EXP_AREA); ++k)
			{
				for (l = expCellOrigin.y; l < (expCellOrigin.y + EXP_AREA); ++l)
				{
					val = Image_getPixel_dontCheck (localMap, k, l);

					val = abs (127 - val);
					if (val != 0)
					{
						++gridTotal;
					}
				}
			}

			expGrid[i][j] = gridTotal;
		}
	}

#ifdef PRINT_MAP_DETAIL
#if 1
	MapCore_printExplorationGrid (f, /*localMap,*/ expGrid);
#else
	MapCore_printExplorationGridSection (f, localMap, bl, tr, expGrid);
#endif
#endif
}

void updateExplorationGrid_localMap (RobotDatabase *db,
									 Image *localMap,
									 __int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS],
									 PointI *expCellBl,
									 PointI *expCellTr)
{
	expCellBl->x = 0;
	expCellBl->y = 0;
	expCellTr->x = EXP_GRID_DIMS - 1;
	expCellTr->y = EXP_GRID_DIMS - 1;

	updateSectionOfExplorationGrid (
		db->xmlLog,
		localMap,
		expGrid,
		*expCellBl,
		*expCellTr);
}

void updateExplorationGrid_currentScan (RobotDatabase *db,
									   Image *localMap,
									   __int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS],
									   PointI *bl,
									   PointI *tr)
{
	PointI pt;
	const int gap = 2;

	// The first cell in the local grid starts at maporig-(dims/2), therefore its
	// centre equals maporig
	pt = PointF_toPointI (db->status.scanCentrePt);

	pt.x -= localMap->orig.x;
	pt.y -= localMap->orig.y;

	// 'dims' is passed in for a safety measure here, so use dims of grid actually
	// in use, i.e. local grid as opposed to global dims
	pt = PointI_calcExpCellIndex (pt, EXP_AREA);

	bl->x = pt.x - gap;
	bl->y = pt.y - gap;
	tr->x = pt.x + gap;
	tr->y = pt.y + gap;

	bl->x = min (EXP_GRID_DIMS - 1, max (0, bl->x));
	bl->y = min (EXP_GRID_DIMS - 1, max (0, bl->y));
	tr->x = min (EXP_GRID_DIMS - 1, max (0, tr->x));
	tr->y = min (EXP_GRID_DIMS - 1, max (0, tr->y));

	updateSectionOfExplorationGrid (
		db->xmlLog,
		localMap,
		expGrid,
		*bl,
		*tr);
}

//! Determine if an exploration cell is inside a local map
int isExplorationCellInLocalMap (const PointI cellCentre,
								const PointI localMapCentre)
{
	return (
		abs (cellCentre.x - localMapCentre.x) < LOC_MAP_DIMS/2 &&
		abs (cellCentre.y - localMapCentre.y) < LOC_MAP_DIMS/2);
}

#define VERBOSE_LOC_MAP_GRID 0
void updateLocalMapGrid (FILE *f,
						__int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS],
						const PointI *localMapOrig,
						__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS],
						__int16 expGridInitialVals[EXP_GRID_DIMS][EXP_GRID_DIMS],
						const PointI *cell_bl,
						const PointI *cell_tr)
{
	PointI localMapBl;
	PointI localMapTr;
	PointI localMapIter;
	PointI localMapCentre;
	PointI cellIter;
	PointI cellCentre;
	int deltaNMappedCells;
	int diff;

	localMapBl = *localMapOrig;
	localMapBl.x /= LOC_MAP_DIFF;
	localMapBl.y /= LOC_MAP_DIFF;
	localMapTr.x = localMapBl.x + 2;
	localMapTr.y = localMapBl.y + 2;
	localMapBl.x -= 2;
	localMapBl.y -= 2;
	localMapBl.x = max (0, localMapBl.x);
	localMapBl.y = max (0, localMapBl.y);
	localMapTr.x = min (GLOB_LOC_MAP_GRID_DIMS - 1, localMapTr.x);
	localMapTr.y = min (GLOB_LOC_MAP_GRID_DIMS - 1, localMapTr.y);

	for (localMapIter.y = localMapTr.y; localMapIter.y >= localMapBl.y; --localMapIter.y)
	{
		for (localMapIter.x = localMapBl.x; localMapIter.x <= localMapTr.x; ++localMapIter.x)
		{
			localMapCentre.x = (localMapIter.x * LOC_MAP_DIFF) + (LOC_MAP_DIMS / 2);
			localMapCentre.y = (localMapIter.y * LOC_MAP_DIFF) + (LOC_MAP_DIMS / 2);
			deltaNMappedCells = 0;

#if VERBOSE_LOC_MAP_GRID
			fprintf (f, "locMap=(%d,%d)", localMapCentre.x, localMapCentre.y);
#endif

			for (cellIter.x = cell_bl->x; cellIter.x <= cell_tr->x; ++cellIter.x)
			{
				for (cellIter.y = cell_bl->y; cellIter.y <= cell_tr->y; ++cellIter.y)
				{
					if (expGrid[cellIter.x][cellIter.y] != expGridInitialVals[cellIter.x][cellIter.y])
					{
						cellCentre.x = (cellIter.x * EXP_AREA) + (EXP_AREA / 2) + localMapOrig->x;
						cellCentre.y = (cellIter.y * EXP_AREA) + (EXP_AREA / 2) + localMapOrig->y;

						if (isExplorationCellInLocalMap (cellCentre, localMapCentre))
						{
							diff = (int)(!MapCore_checkIfCellValid(expGrid[cellIter.x][cellIter.y]) && MapCore_checkIfCellValid(expGridInitialVals[cellIter.x][cellIter.y]));
//							diff = -1 & !MapCore_checkIfCellValid(expGrid[cellIter.x][cellIter.y]) && MapCore_checkIfCellValid(expGridInitialVals[cellIter.x][cellIter.y]);
//							diff = MapCore_checkIfCellValid(expGrid[cellIter.x][cellIter.y]) && !MapCore_checkIfCellValid(expGridInitialVals[cellIter.x][cellIter.y]);
							deltaNMappedCells += diff;
#if VERBOSE_LOC_MAP_GRID
							if (diff)
							{
								fprintf (f, "(%d,%d),", cellCentre.x, cellCentre.y);
							}
#endif
						}
					}
				}
			}
#if VERBOSE_LOC_MAP_GRID
			fprintf (f, "n=%d\n", deltaNMappedCells);
#endif
			localMapGrid[localMapIter.x][localMapIter.y] += deltaNMappedCells;
		}
	}

#ifdef PRINT_MAP_DETAIL
	fprintf (f, "<UpdateLocalMapGrid>\n");
	fprintf (f, "      ");
	for (localMapIter.x = localMapBl.x; localMapIter.x <= localMapTr.x; ++localMapIter.x)
	{
		fprintf (f, "%6d", localMapIter.x);
	}
	fprintf (f, "\n");
	for (localMapIter.y = localMapTr.y; localMapIter.y >= localMapBl.y; --localMapIter.y)
	{
		fprintf (f, "%6d", localMapIter.y);
		for (localMapIter.x = localMapBl.x; localMapIter.x <= localMapTr.x; ++localMapIter.x)
		{
			fprintf (f, "%6d", localMapGrid[localMapIter.x][localMapIter.y]);
		}
		fprintf (f, "\n");
	}
	fprintf (f, "</UpdateLocalMapGrid>\n");

#if 1
	fprintf (f, "<UpdateLocalMapGridExtra>\n");
	fprintf (f, "      ");
	for (localMapIter.x = 0; localMapIter.x < GLOB_LOC_MAP_GRID_DIMS; ++localMapIter.x)
	{
		fprintf (f, "%6d", localMapIter.x);
	}
	fprintf (f, "\n");
	for (localMapIter.y = GLOB_LOC_MAP_GRID_DIMS - 1; localMapIter.y >= 0; --localMapIter.y)
	{
		fprintf (f, "%6d", localMapIter.y);
		for (localMapIter.x = 0; localMapIter.x < GLOB_LOC_MAP_GRID_DIMS; ++localMapIter.x)
		{
			fprintf (f, "%6d", localMapGrid[localMapIter.x][localMapIter.y]);
		}
		fprintf (f, "\n");
	}
	fprintf (f, "</UpdateLocalMapGridExtra>\n");
	fflush (f);
#endif
#endif
}
#undef VERBOSE_LOC_MAP_GRID

//! Determine change in exp cells over local map
void getExpGridDiffVals (__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS],
						 __int16 expGridTemp[EXP_GRID_DIMS][EXP_GRID_DIMS])
{
	int i, j;

	for (i = 0; i < EXP_GRID_DIMS; ++i)
	{
		for (j = 0; j < EXP_GRID_DIMS; ++j)
		{
			if (expGrid[i][j] > expGridTemp[i][j])
			{
				expGridTemp[i][j] = expGrid[i][j] - expGridTemp[i][j];
			}
			else
			{
				expGridTemp[i][j] = 0;
			}
		}
	}
}

void getExpGridInitialVals (__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS],
							__int16 expGridTemp[EXP_GRID_DIMS][EXP_GRID_DIMS])
{
	int i;

	for (i = 0; i < EXP_GRID_DIMS; ++i)
	{
		memcpy (expGridTemp[i], expGrid[i], sizeof (__int16) * EXP_GRID_DIMS);
	}
}

#ifdef CALC_LIVE_MAP_STATS
void RobotMapGridProcessing_calcAndPrintMapStats (
	RobotDatabase *db)
{
	int i, j, n, sz;
	uchar *ptr;
	int val;

	// % mapped in local map
	n = 0;
	sz = db->sensorData.localMap->height * db->sensorData.localMap->wStep;
	for (i = 0, ptr = db->sensorData.localMap->data;
		i < sz; ++i, ++ptr)
	{
		val = *ptr;
		val = abs (127 - val);
		n += (val != 0);
	}
	fprintf (db->xmlLog, "<LocalMapStats>iter=%d nPixelsMapped=%d", db->status.nIterations, n);

	// n cells mapped in local map
#define EXP_CELL_THRESH ((EXP_AREA * EXP_AREA) * EXP_CELL_MAPPED_THRESHOLD)
	n = 0;
	for (i = 0; i < EXP_GRID_DIMS; ++i)
	{
		for (j = 0; j < EXP_GRID_DIMS; ++j)
		{
			n += (db->environment.expGrid[i][j] > EXP_CELL_THRESH);
		}
	}
	fprintf (db->xmlLog, " nExpCellsMapped=%d", n);
#undef EXP_CELL_THRESH

	fprintf (db->xmlLog, "</LocalMapStats>\n");

	// Note:
	// Board will have full access to the rest of the global map, so can just reference
	// the supervision area id (printed in <RobotStatus> each iteration).
}
#endif

void RobotMapGridProcessing_updateGrids (RobotDatabase *db)
{
	PointI expCellBl;
	PointI expCellTr;

	getExpGridInitialVals (db->environment.expGrid, db->environment.expGridInitialVals);

	// update around current map data
	if (UPDATE_SEARCH_GRID_PATCH == db->sensorData.isSearchGridUpdateRequired)
	{
		updateExplorationGrid_currentScan (
			db,
			db->sensorData.localMap,
			db->environment.expGrid,
			&expCellBl,
			&expCellTr);
	}
	else
	{
		updateExplorationGrid_localMap (
			db,
			db->sensorData.localMap,
			db->environment.expGrid,
			&expCellBl,
			&expCellTr);
	}

	updateLocalMapGrid (
		db->xmlLog,
		db->environment.localMapGrid,
		&db->sensorData.localMap->orig,
		db->environment.expGrid,
		db->environment.expGridInitialVals,
		&expCellBl,
		&expCellTr);

	MapCore_updateSupGridCells (
		db->xmlLog,
		db->environment.localMapGrid,
		db->environment.supGrid);

#ifdef PRINT_MAP_DETAIL
	MapCore_printSupGridCells (db->xmlLog, db->environment.supGrid);
#endif

#ifdef CALC_LIVE_MAP_STATS
	RobotMapGridProcessing_calcAndPrintMapStats (db);
#endif

	db->sensorData.isSearchGridUpdateRequired = DONT_UPDATE_SEARCH_GRID;
}

#endif
