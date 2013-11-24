#include "../../Common/RobotDefs.h"

#ifndef ROBOT

#include "BoardMapProcessing.h"


//! Incorporate map submitted by robot into global map
/*!
This operates under the assumption that local maps maintained by the robots 
are within the bounds of the global map. The function iterates through the 
map, updating pixels with greater certainty of their occupancy.
*/
void incorporateLocalMap (BoardDatabase *db,
							Image *incMap,
							Image *map,
							int *nPix_ptr,
							float *avgPixStdDev_ptr)
{
	int i, j;
	uchar incPtr;
	uchar mapPtr;
	float cellnew, cellOrig;
	int stdDevPixelValue; // val for calc'ing std dev from pix val
	int incMapWidth = incMap->width;
	int incMapHeight = incMap->height;
	int nPix;
	float avgPixStdDev;

	stdDevPixelValue = (int)(STD_DEV_MAX + STD_DEV_PIXEL_GAP);

	nPix = 0;
	avgPixStdDev = 0;

	for (i = 0; i < incMapWidth; ++i)
	{
		for (j = 0; j < incMapHeight; ++j)
		{
			incPtr = Image_getPixel_dontCheck (incMap, i, j);

			mapPtr = Image_getPixel_dontCheck (
				map,
				i + incMap->orig.x,
				j + incMap->orig.y);

			cellOrig = mapPtr;
			cellOrig = fabs (cellOrig - 127.0f);

			cellnew = incPtr;
			cellnew = fabs (cellnew - 127.0f);

			if (cellnew > cellOrig)
			{
				Image_setPixel_dontCheck (
					map,
					i + incMap->orig.x,
					j + incMap->orig.y,
					incPtr);

				if (127 != incPtr)
				{
					++nPix;
					avgPixStdDev += stdDevPixelValue - cellnew;
				}
			}
		}
	}

	avgPixStdDev /= max (1, nPix);

	*nPix_ptr = nPix;
	*avgPixStdDev_ptr = avgPixStdDev;
}


void BoardMapIntegration_refreshMapArea (BoardDatabase *db,
										 const PointI bl,
										 const PointI tr)
{
	ListNode *iterator;
	PointI orig;
	CompressedImageWithMapInfo *mapWithProfitInfo;
	int nPix;
	float avgPixStdDev;

	Image_drawRect (db->environment.map, bl.x, bl.y, tr.x + LOC_MAP_DIMS, tr.y + LOC_MAP_DIMS, 127, 1);

	iterator = db->sensorData.globalMaps.back;
	while (iterator)
	{
		mapWithProfitInfo = (CompressedImageWithMapInfo*)iterator->value;

		orig = mapWithProfitInfo->image.orig;
		if (orig.x >= bl.x &&
			orig.y >= bl.y &&
			orig.x <= tr.x &&
			orig.y <= tr.y)
		{
			MapCore_decompressLocalMap (db->environment.incMap, &mapWithProfitInfo->image);

			// incorporate recreated local map into global map, and figure
			// out gain in certainty
			incorporateLocalMap (
				db,
				db->environment.incMap,
				db->environment.map,
				&nPix,
				&avgPixStdDev);
		}

		iterator = iterator->prev;
	}
}

//! Attribute map profit based on stdDev of submitted map data
void attributeProfit (BoardDatabase *db,
					 const int nPix,
					 float avgPixStdDev,
					 CompressedImageWithMapInfo *imageWithProfitInfo)
{
	float profit;
	LocalMapInfo *mapInfo = &imageWithProfitInfo->mapInfo;
	profit = nPix * (1.0f - min (1.0f, avgPixStdDev / STD_DEV_MAX));

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<LocalMapProfit>robotIndex=%d localMapIndex=%d profit=%f mapGain=%d avgStdDev=%f</LocalMapProfit>\n",
		mapInfo->robotIndex, mapInfo->localMapIndex, profit, nPix, avgPixStdDev);
#endif
}

extern void BoardDatabase_getNMapped (BoardDatabase *db, int *nMapped, int *nOccupied, float *avgErrorMag);

void BoardMapIntegration_printCurrent (BoardDatabase *db)
{
	int nMapped, nOccupied;
	float avg;

	BoardDatabase_getNMapped (db, &nMapped, &nOccupied, &avg);
	fprintf (db->xmlLog, "<BoardMapAccuracy>iter=%d nMapped=%d nOccupied=%d avgErrorMag=%f</BoardMapAccuracy>\n",
		nMapped,
		nOccupied,
		avg);


}

//! Add new occupancy data submitted by the robots into the global map
void BoardMapIntegration_incorporateNewMapData (BoardDatabase *db,
											   PointI *bl,
											   PointI *tr)
{
	ListNode *iterator;
	ListNode *globalIterator;
	CompressedImageWithMapInfo *mapWithProfitInfo;
	PointI origin;
	int nPix;
	float avgPixStdDev;
	int flag;

	flag = 0;
	iterator = db->sensorData.incorpMaps.front;
	while (iterator)
	{
		mapWithProfitInfo = (CompressedImageWithMapInfo*)iterator->value;

		if (1 == db->sensorData.statusFromRobot[mapWithProfitInfo->mapInfo.robotIndex].isMoreDataComing)
		{
			iterator = iterator->next;
			continue;
		}
		flag = 1;

		origin = mapWithProfitInfo->image.orig;

		if (origin.x < bl->x){ bl->x = origin.x; }
		if (origin.x > tr->x){ tr->x = origin.x; }
		if (origin.y < bl->y){ bl->y = origin.y; }
		if (origin.y > tr->y){ tr->y = origin.y; }

		// Set flag so robot will know its data has been incorporated
		db->sensorData.statusFromBoard[mapWithProfitInfo->mapInfo.robotIndex].hasDataBeenIncd = 1;
//		db->sensorData.statusFromRobot[mapWithProfitInfo->mapInfo.robotIndex].isNewMapData = 0;

		MapCore_decompressLocalMap (db->environment.incMap, &mapWithProfitInfo->image);

		incorporateLocalMap (
			db,
			db->environment.incMap,
			db->environment.map,
			&nPix,
			&avgPixStdDev);

		attributeProfit (
			db,
			nPix,
			avgPixStdDev,
			mapWithProfitInfo);

		globalIterator = iterator;
		iterator = iterator->next;

		List_removeElement (&db->sensorData.incorpMaps, globalIterator);
		List_pushNode (&db->sensorData.globalMaps, globalIterator);
	}

	if (flag)
	{
		BoardMapIntegration_printCurrent (db);
	}
}

#ifdef SIMULATION
int explicitlyShowWipe = 0;
#endif

//! If any robot wants to remove provisional map data it submitted, remove & refresh the area
void BoardMapIntegration_wipeProvisionalMapData (BoardDatabase *db,
												PointI *bl,
												PointI *tr)
{
	ListNode *iterator;
	ListNode *deleteIterator;
	CompressedImageWithMapInfo *mapWithProfitInfo;
	CompressedImageWithMapInfo *mapToDelete;
	PointI origin;
	int wasProvisionalDataRemoved[MAX_N_ROBOTS];

	memset (wasProvisionalDataRemoved, 0, (MAX_N_ROBOTS * sizeof (int)));

	iterator = db->sensorData.incorpMaps.front;
	while (iterator)
	{
		mapWithProfitInfo = (CompressedImageWithMapInfo*)iterator->value;

		if (1 == db->sensorData.statusFromRobot[mapWithProfitInfo->mapInfo.robotIndex].isMoreDataComing)
		{
			iterator = iterator->next;
			continue;
		}

		if (ADJUSTED_CLOSE_LOOP_DATA == mapWithProfitInfo->mapInfo.isProvisional &&
			!wasProvisionalDataRemoved[mapWithProfitInfo->mapInfo.robotIndex])
		{
			deleteIterator = db->sensorData.globalMaps.back;

			// Get rid of provisional map data
			// Profit is deleted when the profit for the new maps is assigned
			while (deleteIterator)
			{
				mapToDelete =  (CompressedImageWithMapInfo*)deleteIterator->value;

				if (mapToDelete->mapInfo.robotIndex == mapWithProfitInfo->mapInfo.robotIndex &&
					PROVISIONAL_CLOSE_LOOP_DATA == mapToDelete->mapInfo.isProvisional &&
					mapToDelete->mapInfo.closeLoopSession == mapWithProfitInfo->mapInfo.closeLoopSession)
				{
#ifdef PRINT_EVENTS
					fprintf (db->xmlLog, "<WipeProvisionalMap>robotIndex=%d localMapIndex=%d closeLoopSession=%d iteration=%d</WipeProvisionalMap>\n",
						mapToDelete->mapInfo.robotIndex,
						mapToDelete->mapInfo.localMapIndex,
						mapWithProfitInfo->mapInfo.closeLoopSession,
						db->groupData.experimentStatus.iteration);
#endif
					origin = mapToDelete->image.orig;
					if (origin.x < bl->x){ bl->x = origin.x; }
					if (origin.x > tr->x){ tr->x = origin.x; }
					if (origin.y < bl->y){ bl->y = origin.y; }
					if (origin.y > tr->y){ tr->y = origin.y; }

					List_deleteElementWithDeallocator (&db->sensorData.globalMaps, &deleteIterator, 0, freeCompressedImageWithMapInfo);

#ifdef SIMULATION
					explicitlyShowWipe = 1;
#endif
				}
				else
				{
					deleteIterator = deleteIterator->prev;
				}
			}

			wasProvisionalDataRemoved[mapWithProfitInfo->mapInfo.robotIndex] = 1;
		}

		iterator = iterator->next;
	}
}

//! Calculate local map occupied value from exploration grid
int calcLocalMapValue (const int i,
					 const int j,
					 const __int16 expGrid[GLOB_EXP_GRID_DIMS][GLOB_EXP_GRID_DIMS])
{
	PointI bl;
	PointI tr;
	PointI iter;
	int temp;

	// How/why was this wrong before?
//	bl.x = i * 2;
//	bl.y = j * 2;
	bl.x = i * 3;
	bl.y = j * 3;

	tr.x = bl.x + 6;
	tr.y = bl.y + 6;

	temp = 0;
	for (iter.x = bl.x; iter.x < tr.x; ++iter.x)
	{
		for (iter.y = bl.y; iter.y < tr.y; ++iter.y)
		{
			temp += MapCore_checkIfCellValid (expGrid[iter.x][iter.y]);
		}
	}
	return N_CELLS_LOC_MAP - temp;
}

void BoardMapIntegration_updateLocalMapGridCells (BoardDatabase *db,
											   const __int16 expGrid[GLOB_EXP_GRID_DIMS][GLOB_EXP_GRID_DIMS],
											   __int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS])
{
	int i, j;
	for (j = GLOB_LOC_MAP_GRID_DIMS - 1; j >= 0; --j)
	{
		for (i = 0; i < GLOB_LOC_MAP_GRID_DIMS; ++i)
		{
			localMapGrid[i][j] = calcLocalMapValue (i, j, expGrid);
		}
	}
}

void BoardMapIntegration_updateExpGridCells (BoardDatabase *db,
											const PointI bl,
											const PointI tr)
{
	int i, j, k, l;
	int kStart, lStart, kStop, lStop;
	int gridTotal;
	int val;
	int reqdcert;

	reqdcert = 1;

	// iterate through cells in mapGrid from minPoint to maxPoint
	for (j = tr.y; j >= bl.y; --j)
	{
		for (i = bl.x; i <= tr.x; ++i)
		{
			// nullify counter for cell; the max value that this variable will have
			// to store is 20 * 20 * 127 = 50,800
			gridTotal = 0;

			// set start point for iterating through the occupancy cells
			kStart = i * EXP_AREA;
			lStart = j * EXP_AREA;
			kStop = kStart + EXP_AREA;
			lStop = lStart + EXP_AREA;

			// within each grid determine average occupancy certainty
			for (k = kStart; k < kStop; ++k)
			{
				for (l = lStart; l < lStop; ++l)
				{
					val = Image_getPixel_dontCheck (db->environment.map, k, l);

					if (abs (127 - val) >= reqdcert)
					{
						++gridTotal;
					}
				}
			}

			// update value
			db->environment.expGrid[i][j] = gridTotal;
		}
	}
}

#endif
