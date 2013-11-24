#include "../../Common/RobotDefs.h"

#include "MapIntegration.h"
#include "MapIntegrationImpl.h"
#include "../../Common/Geometry.h"
#include "../../Common/BitArray.h"
#include "../../Common/Bresenham.h"
#include "../../Common/Vector.h"



#ifndef BOARD

void intScan_pixel (
	Image *localMap,
	Image *navMap,
	const PointI pt,
	const int isOccupied,
	const uchar pixelValue,
	const int updateNavMap)
{
	int localMapValue = Image_getPixel_check (localMap, pt.x, pt.y);

	if (INVALID_CELL == localMapValue ||
		(localMapValue != FAKE_NBOUR_TERRAIN && abs (127 - localMapValue) > pixelValue))
	{
		return;
	}

	if (0 == isOccupied)
	{
		// new vers has only 1 level of cert
		// but std dev is taken into acc
		Image_setPixel_dontCheck (
			localMap,
			pt.x,
			pt.y,
			(127 + pixelValue));
	}
	else
	{
		//! \todo possibly rethink overwriting occupied cells

		Image_setPixel_dontCheck (
			localMap,
			pt.x,
			pt.y,
			(127 - pixelValue));

		if (navMap && updateNavMap)
		{
			Image_setPixel_dontCheck (
				navMap,
				pt.x,
				pt.y,
				OCCUPIED_TERRAIN);
		}
	}
}

//! Check point on map against occupancy grid. Return value indicates if the given pixel is in the grid.
int checkCamGrid (OccupancyGrid *occupancyGrid, const int pixelX, const int pixelY, uchar *val)
{
	uchar temp;
	PointI element;
	element.x = pixelX - CAM_OCCUPANCY_GRID_ORIGIN_X;
	element.y = pixelY - CAM_OCCUPANCY_GRID_ORIGIN_X;

	if (element.x >= 0 && element.x < (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_CELL_X) &&
		element.y >= 0 && element.y < (CAM_OCCUPANCY_GRID_Y_EXP * CAM_OCCUPANCY_CELL_Y))
	{
		element.x /= CAM_OCCUPANCY_CELL_X;
		element.y /= CAM_OCCUPANCY_CELL_Y;

		if (occupancyGrid)
		{
			temp = TwoBitArray_checkPt (occupancyGrid->grid, element, CAM_OCCUPANCY_GRID_X);
		}
		else
		{
			// Fake scan is being used, so return unoccupied
			temp = 2;
		}

#if 0
		printf ("(%3d,%3d) pixel -> (%2d,%2d) grid -> %d\n", pixelX, pixelY, element.x, element.y, temp);
#endif

		*val = temp;
		return 1;
	}
	return 0;
}

#define VERBOSE_INT_GRID_SCAN 0
#if VERBOSE_INT_GRID_SCAN
int countMappedCells (Image *localMap)
{
	int i;
	int n = 0;
	for (i = 0; i < localMap->width * localMap->height; ++i)
	{
		n += (localMap->data[i] != 127);
	}
	return n;
}
#endif

//! Returns whether or not there is new occupied terrain
int MapIntegration_intGridScan (
	RobotDatabase *db,
	PoseSimple *mapScanPose,
	OccupancyGrid *camOccupancyGrid,
	const float stdDev,
	Image *localMap,
	Image *navMap,
	CamVectors *camVectors,
	const int updateNavMap,
	const int intIntoGlobalMap,
	const int intUnknownCells)
{
	int i, j;
	PointF focalPt;
	PointF centrePt;
	PointI bl, tr;
	PointI pt;
	const int sensorRange = 30; //25;
	uchar pixelValue; // val for encoding std dev in image
	PointI pixel;
	int isNewOccupiedTerrain = 0;
	uchar occupancyGridVal;
	int isInCamOccupGrid;
	PointF relPt;
	Vector3F focalPtOnWorldMap;
//	Vector3F ptOnWorldMap;
	PointF imageSpaceOffsets;
#ifdef PRINT_MAP_DETAIL
	int nMapCellsInCamGrid = 0;
#endif

	EXPLICIT_DEBUG_ASSERT(db->geometryConstants.areCamDistsSetup == 1)

	Geometry_ptFromOrient (mapScanPose->loc.x, mapScanPose->loc.y, &focalPtOnWorldMap.x, &focalPtOnWorldMap.y, CAM_P_OFFSET, mapScanPose->orient);
	focalPtOnWorldMap.x *= MAP_SCALE;
	focalPtOnWorldMap.y *= MAP_SCALE;
	focalPtOnWorldMap.z = 0.0f;

	// Remember: whenever we translate between map/world and images, we have to set the orient
	// of the camera vectors first.
	CamVectors_rotateToRobotOrient (camVectors, mapScanPose->orient);

	Geometry_ptFromOrient (mapScanPose->loc.x, mapScanPose->loc.y, &focalPt.x, &focalPt.y, CAM_P_OFFSET, mapScanPose->orient);
//	Geometry_ptFromOrient (mapScanPose->loc.x, mapScanPose->loc.y, &centrePt.x, &centrePt.y, CAM_P_OFFSET + db->geometryConstants.exp_optDist, mapScanPose->orient);
	Geometry_ptFromOrient (mapScanPose->loc.x, mapScanPose->loc.y, &centrePt.x, &centrePt.y, CAM_P_OFFSET + SCAN_CENTRE_DIST, mapScanPose->orient);

	centrePt.x -= localMap->orig.x;
	centrePt.y -= localMap->orig.y;

	tr.x = (int)centrePt.x + sensorRange;
	tr.y = (int)centrePt.y + sensorRange;
	bl.x = (int)centrePt.x - sensorRange;
	bl.y = (int)centrePt.y - sensorRange;

	bl.x = max (0, bl.x);
	bl.y = max (0, bl.y);
	if (intIntoGlobalMap)
	{
		tr.x = min (tr.x, ENVIR_DIMS);
		tr.y = min (tr.y, ENVIR_DIMS);
	}
	else
	{
		tr.x = min (tr.x, LOC_MAP_DIMS);
		tr.y = min (tr.y, LOC_MAP_DIMS);
	}

	pixelValue = (uchar)(STD_DEV_PIXEL_GAP + STD_DEV_MAX - stdDev);

	EXPLICIT_DEBUG_ASSERT (camVectors->currentAdjustedOrient == mapScanPose->orient)
	for (i = bl.x; i < tr.x; ++i)
	{
		for (j = bl.y; j < tr.y; ++j)
		{
			relPt.x = (float)(i + localMap->orig.x) - focalPt.x;
			relPt.y = (float)(j + localMap->orig.y) - focalPt.y;

			//if (1 != Geometry_isVisualContact (
			//	focalPt.x,
			//	focalPt.y,
			//	mapScanPose->orient,
			//	(float)(i + localMap->orig.x),
			//	(float)(j + localMap->orig.y),
			//	db->geometryConstants.cam_minDist,
			//	db->geometryConstants.cam_maxDist,
			//	db->geometryConstants.cam_angleAtMinDist,
			//	db->geometryConstants.cam_deltaAngleWithDist))
			if (1 != Geometry_isVisualContactNEW (
				relPt,
				db->geometryConstants.cam_xLimit,
				db->geometryConstants.cam_yNegLimit,
				db->geometryConstants.cam_yPosLimit,
				camVectors))
			{
				continue;
			}

			imageSpaceOffsets = CamVectors_projectMapSpaceToImage (camVectors, relPt);

			//ptOnWorldMap.x = (float)((i + localMap->orig.x) * MAP_SCALE_I);
			//ptOnWorldMap.y = (float)((j + localMap->orig.y) * MAP_SCALE_I);
			//ptOnWorldMap.z = 0.0f;

			//imageSpaceOffsets = CamVectors_projectWorldMapToImage (
			//	camVectors,
			//	focalPtOnWorldMap,
			//	ptOnWorldMap);

			pixel = CamVectors_imageOffsetsToPixel (imageSpaceOffsets);
			DEBUG_ASSERT(pixel.x >= 0 && pixel.x < CAM_IMG_W)
			DEBUG_ASSERT(pixel.y >= 0 && pixel.y < CAM_IMG_H)

			isInCamOccupGrid = checkCamGrid (
				camOccupancyGrid,
				pixel.x,
				pixel.y,
				&occupancyGridVal);

#if VERBOSE_INT_GRID_SCAN
			fprintf (
				db->xmlLog,
				"<IntGridScan>relPtOnMap=(%f,%f) imageSpaceOffsets=(%f,%f) pixel=(%d,%d) isInCamOccupGrid=%d occupancyGridVal=%d</IntGridScan>\n",
				relPt.x,
				relPt.y,
//				"<IntGridScan>ptOnWorldMap=(%f,%f,%f) imageSpaceOffsets=(%f,%f) pixel=(%d,%d) isInCamOccupGrid=%d occupancyGridVal=%d</IntGridScan>\n",
//				ptOnWorldMap.x,
//				ptOnWorldMap.y,
//				ptOnWorldMap.z,
				imageSpaceOffsets.x,
				imageSpaceOffsets.y,
				pixel.x,
				pixel.y,
				isInCamOccupGrid,
				occupancyGridVal);
#endif

			// If (the cell was definitely marked as occupied) or (the cell was maybe
			// marked as occupied, and this is our local map for navigation), then add
			// this cell to the local map.
			if (isInCamOccupGrid && (occupancyGridVal == 0 || occupancyGridVal == 1 || (intUnknownCells && occupancyGridVal >= 2)))
			{
#ifdef PRINT_MAP_DETAIL
				++nMapCellsInCamGrid;
#endif
				isNewOccupiedTerrain |= (int)(occupancyGridVal != 0);

				// Incorp pixel into local map
				pt.x = i;
				pt.y = j;

				intScan_pixel (
					localMap,
					navMap,
					pt,
					(occupancyGridVal != 0),
					pixelValue,
					updateNavMap);
			}
		} 
	}

#ifdef PRINT_MAP_DETAIL
	fprintf (db->xmlLog, "<IntegrateScanCam>focalPt=(%f,%f) centrePt=(%f,%f) bl=(%d,%d) tr=(%d,%d) stdDev=%f pixelValue=%d nMapCellsInCamGrid=%d</IntegrateScanCam>\n",
		focalPt.x, focalPt.y, centrePt.x, centrePt.y, bl.x, bl.y, tr.x, tr.y, stdDev, pixelValue, nMapCellsInCamGrid);
#endif

#if VERBOSE_INT_GRID_SCAN
	printf ("%d mapped cells\n", countMappedCells (localMap));
#endif

	return isNewOccupiedTerrain;
}

#if defined(MAP_SCANS_HAVE_GRIDS)
//! Test cam scans with blank image grid. Also, when there is no obstacle - no need to store grid
int MapIntegration_intBlankScan (
	RobotDatabase *db,
	MapScan *mapScan,
	const float stdDev,
	Image *localMap,
	Image *navMap,
	CamVectors *camVectors,
	const int updateNavMap,
	const int intUnknownCells)
{
	OccupancyGrid camOccupancyGrid;
	TwoBitArray_reset (camOccupancyGrid.grid, SIZE_OCCUP_TWOBITARRAY);

	return MapIntegration_intGridScan (
		db,
		&mapScan->pose,
		&camOccupancyGrid,
		stdDev,
		localMap,
		navMap,
		camVectors,
		updateNavMap,
		0,
		intUnknownCells);
}

int MapIntegration_intCamScan (
	RobotDatabase *db,
	MapScan *mapScan,
	const float stdDev,
	Image *localMap,
	Image *navMap,
	CamVectors *camVectors,
	const int updateNavMap,
	const int intUnknownCells)
{
	return MapIntegration_intGridScan (
		db,
		&mapScan->pose,
		mapScan->grid,
		stdDev,
		localMap,
		navMap,
		camVectors,
		updateNavMap,
		0,
		intUnknownCells);
}

#else // defined(MAP_SCANS_HAVE_GRIDS)

//! Return whether or not new occupied terrain is found.
int intScanVector_simd (RobotDatabase *db,
						CamVectors *camVectors,
						 BresData *bresdata,
						 Image *localMap,
						 Image *navMap,
						 Image *sim,
						 const uchar pixelValue,
						 const int updateNavMap)
{
	int e;
	int dist;
	int isBlocked;
	uchar val;
	uchar val2;
	uchar val3;
	PointI bres2;
	PointI bres3;
	PointF relPt;
	int isNewOccupiedTerrain = 0;

	e = 0;

	for (dist = 0; dist < bresdata->count; ++dist)
	{
		bres2 = bresdata->bres;
		bres3 = bres2;

		Bresenham_step (
			&bresdata->bres,
			&e,
			&bresdata->step,
			&bresdata->d,
			bresdata->axis);

		// Point is out of bounds, so return
		if (0 == Image_isWithinBounds_ignoreMapOrig (
			localMap,
			bresdata->bres.x,
			bresdata->bres.y))
		{
			return isNewOccupiedTerrain;
		}

		// calc neighbouring pts for each bresenham pt
		if (1 == bresdata->axis)
		{
			// 1 is x, so see if y was also incr'd
			if (bres2.y == bresdata->bres.y)
			{
				bres2.y += bresdata->step.y;
				bres3.y -= bresdata->step.y;
			}
			else
			{
				bres3.y += 2 * bresdata->step.y;
			}

			bres2.x += bresdata->step.x;
			bres3.x += bresdata->step.x;
		}
		else
		{
			if (bres2.x == bresdata->bres.x)
			{
				bres2.x += bresdata->step.x;
				bres3.x -= bresdata->step.x;
			}
			else
			{
				bres3.x += 2 * bresdata->step.x;
			}

			bres2.y += bresdata->step.y;
			bres3.y += bresdata->step.y;
		}

		if (MAX_FLT == bresdata->blockedDist)
		{
			// will return INVALID_CELL if OOB
			val = Image_getPixel_check (
				sim,
				localMap->orig.x + bresdata->bres.x + bresdata->offset.x,
				localMap->orig.y + bresdata->bres.y + bresdata->offset.y);

			val2 = Image_getPixel_check (
				sim,
				localMap->orig.x + bres2.x + bresdata->offset.x,
				localMap->orig.y + bres2.y + bresdata->offset.y);

			val3 = Image_getPixel_check (
				sim,
				localMap->orig.x + bres3.x + bresdata->offset.x,
				localMap->orig.y + bres3.y + bresdata->offset.y);

			if (0 == val || 0 == val2 || 0 == val3)
			{
				bresdata->blockedDist = Geometry_dist (
					(float)bresdata->bres.x,
					(float)bresdata->bres.y,
					bresdata->lens.x,
					bresdata->lens.y);

				bresdata->count = min (bresdata->count, (dist + bresdata->obstacleWidth));
			}
		}

		relPt.x = (float)bresdata->bres.x - bresdata->lens.x;
		relPt.y = (float)bresdata->bres.y - bresdata->lens.y;

		// always adjust beforehand
		EXPLICIT_DEBUG_ASSERT (camVectors->currentAdjustedOrient == bresdata->poseOrient)

		if (1 == Geometry_isVisualContactNEW (
			relPt,
			db->geometryConstants.cam_xLimit,
			db->geometryConstants.cam_yNegLimit,
			db->geometryConstants.cam_yPosLimit,
			camVectors))
		{
			isBlocked = (int)(MAX_FLT != bresdata->blockedDist);

			isNewOccupiedTerrain |= isBlocked;

			// will just return 0 if OOB
			intScan_pixel (localMap, navMap, bresdata->bres,	isBlocked, pixelValue, updateNavMap);
			intScan_pixel (localMap, navMap, bres2,				isBlocked, pixelValue, updateNavMap);
			intScan_pixel (localMap, navMap, bres3,				isBlocked, pixelValue, updateNavMap);
		}
	}

	return isNewOccupiedTerrain;
}

int MapIntegration_intSimScan (
	RobotDatabase *db,
	MapScan *mapScan,
	const float stdDev,
	Image *localMap,
	Image *navMap,
	Image *sim,
	const int updateNavMap)
{
	BresData bresdata;

	float orientRange;
	float orientJump;
	float orient;
	float startDist;
	float endDist;
	float bres_startDist;
	float bres_endDist;
	PointF startPt, endPt;
	uchar pixelValue; // Value for encoding std deviation (in robot location estimate) in image
	int isNewOccupiedTerrain = 0;

	EXPLICIT_DEBUG_ASSERT(db->geometryConstants.areCamDistsSetup == 1)

	orientRange = db->geometryConstants.cam_angleAtMinDist + RADS_TWO;
	orientJump = RADS_TWO;

	// Start from 1 (not db->geometryConstants.cam_minDist - 1), as an obstacle directly in front of the
	// camera will still be visible
	startDist = 1.0f;
	endDist = db->geometryConstants.cam_maxDist + 1;
	pixelValue = (uchar)(STD_DEV_PIXEL_GAP + STD_DEV_MAX - stdDev);

	Geometry_ptFromOrient (
		mapScan->pose.loc.x - localMap->orig.x,
		mapScan->pose.loc.y - localMap->orig.y,
		&bresdata.lens.x,
		&bresdata.lens.y,
		CAM_P_OFFSET,
		mapScan->pose.orient);

#if defined(SIM_ERROR)
	bresdata.offset.x = (mapScan->actualPoseLoc.x - mapScan->pose.loc.x);
	bresdata.offset.y = (mapScan->actualPoseLoc.y - mapScan->pose.loc.y);
#else
	bresdata.offset.x = 0;
	bresdata.offset.y = 0;
#endif
	bresdata.poseOrient = mapScan->pose.orient;
	bresdata.blockedDist = MAX_FLT;
	bresdata.obstacleWidth = 3;
	bres_startDist = startDist;
	bres_endDist = endDist;

	for (orient = mapScan->pose.orient - orientRange;
		orient < mapScan->pose.orient + orientRange;
		orient += orientJump)
	{
		bresdata.blockedDist = MAX_FLT;

		Geometry_ptFromOrient (
			bresdata.lens.x,
			bresdata.lens.y,
			&startPt.x,
			&startPt.y,
			bres_startDist,
			orient);

		Geometry_ptFromOrient (
			bresdata.lens.x,
			bresdata.lens.y,
			&endPt.x,
			&endPt.y,
			bres_endDist,
			orient);

		Bresenham_setPts (
			localMap,
			startPt.x,
			startPt.y,
			endPt.x,
			endPt.y,
			&bresdata.bres.x,
			&bresdata.bres.y,
			&bresdata.d.x,
			&bresdata.d.y);

		Bresenham_setStep (
			&bresdata.d.x,
			&bresdata.d.y,
			&bresdata.step.x,
			&bresdata.step.y,
			&bresdata.axis,
			&bresdata.count);

		isNewOccupiedTerrain |= intScanVector_simd (
			db,
			&db->camVectors,
			&bresdata,
			localMap,
			navMap,
			sim,
			pixelValue,
			updateNavMap);
	}

	return isNewOccupiedTerrain;
}
#endif // defined(MAP_SCANS_HAVE_GRIDS)























//! Temporary struct to hold profit per target while generating map to submit to the blackboard.
typedef struct TargetProfit_
{
	int targetIndex;
	int nScans;
	float profit;
	int estdGain;
} TargetProfit;

//! Default constructor.
void initTargetProfit(TargetProfit *targetProfit)
{
	targetProfit->nScans = 0;
	targetProfit->targetIndex = -1;
	targetProfit->profit = 0.0f;
	targetProfit->estdGain = 0;
}

//! Print break down of estimated profit for a submitted local map.
void MapIntegration_printApproximatedMap (RobotDatabase *db, const float behaviourProfits[3], const List *targetProfits)
{
	ListNode *iterator;
	TargetProfit *targetProfit;
	fprintf (db->xmlLog, "<LocalMapProfitBreakdown>\n");
	fprintf (db->xmlLog, "<PerBehaviour>exploration=%f gotoExpPt=%f other=%f</PerBehaviour>\n", behaviourProfits[0], behaviourProfits[1], behaviourProfits[2]);
	iterator = targetProfits->front;
	while (iterator)
	{
		targetProfit = (TargetProfit*)iterator->value;
		fprintf (db->xmlLog, "<PerTarget>id=%d nScans=%d profit=%f estdGain=%d</PerTarget>\n", targetProfit->targetIndex, targetProfit->nScans, targetProfit->profit, targetProfit->estdGain);
		iterator = iterator->next;
	}

	fprintf (db->xmlLog, "</LocalMapProfitBreakdown>\n");
}


















extern int CloseLoop_isLoopClosePossible (RobotDatabase *db);

LocalMapInfo RobotMapIntegration_redrawLocalMap (RobotDatabase *db)
{
	LocalMapInfo localMapInfo;
	Image *lm = db->sensorData.localMap;
	List *msl = &db->sensorData.mapScanList;
	MapScan *ms;
	float profit;
	ListNode *iterator;
	float behaviourProfits[3] = {0.0f, 0.0f, 0.0f}; // EXPLORATION, GOTO_EXP_PT, OTHER
	List targetProfits = initList();
	ListNode *targetIterator;
	TargetProfit *targetProfit;
	int targetFound;

	Image_fill (lm, 127);

	localMapInfo = initLocalMapInfo();

	// If the most recent map scan was not integrated, write it to the log file now. Otherwise
	// the behaviour for this scan will not be attributed the appropriate 
	if (db->sensorData.isUnincorporatedMapScan)
	{
		db->sensorData.isUnincorporatedMapScan = 0;

//		MapCore_logMapScan (db->xmlLog, db->geometryConstants.exp_optDist, db->status.nIterations, (MapScan*)msl->back->value);
		MapCore_logMapScan (db->xmlLog, SCAN_CENTRE_DIST, db->status.nIterations, (MapScan*)msl->back->value);
	}

	iterator = msl->front;
	while (iterator != 0)
	{
		ms = (MapScan*)iterator->value;
		if (ms->stdDev >= STD_DEV_MAX)
		{
			iterator = iterator->next;
			continue;
		}

		CamVectors_rotateToRobotOrient (&db->camVectors, ms->pose.orient);

#if defined(MAP_SCANS_HAVE_GRIDS)
#ifdef IGNORE_CAM_DATA
		MapIntegration_intBlankScan (
			db,
			ms,
			ms->stdDev,
			lm,
			db->environment.navMap,
			&db->camVectors,
			0,
			0 /*intUnknownCells*/);

#else // IGNORE_CAM_DATA

		if (ms->grid)
		{
			MapIntegration_intCamScan (
				db,
				ms,
				ms->stdDev,
				lm,
				db->environment.navMap,
				&db->camVectors,
				0,
				0 /*intUnknownCells*/);
		}
		else
		{
			MapIntegration_intBlankScan (
				db,
				ms,
				ms->stdDev,
				lm,
				db->environment.navMap,
				&db->camVectors,
				0,
				0 /*intUnknownCells*/);
		}
#endif // IGNORE_CAM_DATA

#else // defined(MAP_SCANS_HAVE_GRIDS)

		MapIntegration_intSimScan (
			db,
			ms,
			ms->stdDev,
			lm,
			db->environment.navMap,
#ifdef SIMULATION
			db->board->environment.tempMap,
#else
			db->environment.originalSim, // have to give robot copy or simulated map if on separate processes
#endif
			0);
#endif // defined(MAP_SCANS_HAVE_GRIDS)

		profit = ms->estdGain * (1.0f - min (1.0f, ms->stdDev / STD_DEV_MAX));
		localMapInfo.gross += profit;
		localMapInfo.mapGain += ms->estdGain;
		++localMapInfo.nScans;
		localMapInfo.avgStdDev += ms->stdDev;

		targetFound = 0;
		targetIterator = targetProfits.front;
		while (targetIterator)
		{
			targetProfit = (TargetProfit*)targetIterator->value;
			if (targetProfit->targetIndex == ms->targetIndex)
			{
				targetProfit->profit += profit;
				targetProfit->estdGain += ms->estdGain;
				targetProfit->nScans += 1;
				targetFound = 1;
				break;
			}

			targetIterator = targetIterator->next;
		}
		if (!targetFound)
		{
			targetProfit = (TargetProfit*)malloc (sizeof (TargetProfit));
			initTargetProfit (targetProfit);

			targetProfit->targetIndex = ms->targetIndex;
			targetProfit->profit = profit;
			targetProfit->estdGain = ms->estdGain;
			targetProfit->nScans = 1;
//			targetProfit->iterWhenStarted = ms->targetIndex;
			List_pushValue (&targetProfits, targetProfit);
		}

		switch (ms->behaviour)
		{
		case EXPLORATION:
			behaviourProfits[0] += profit;
			break;
		case GOTO_EXP_PT:
			behaviourProfits[1] += profit;
			break;
		default:
			behaviourProfits[2] += profit;
		}

		iterator = iterator->next;
	}

	if (localMapInfo.nScans != 0)
	{
		localMapInfo.avgStdDev /= localMapInfo.nScans;
	}

#ifdef PRINT_PROFIT
	MapIntegration_printApproximatedMap (db, behaviourProfits, &targetProfits);
#endif

//	List_clearWithDeallocator (msl, freeMapScan);
	if (!CloseLoop_isLoopClosePossible (db))
	{
		List_clearWithDeallocator (msl, freeMapScan);
	}
	List_clear (&targetProfits, 1);

	return localMapInfo;
}

void RobotMapIntegration_integrateMapData (RobotDatabase *db)
{
	Image *lm;
	RobotSensorData *rsd;
	MapScan *curr;
	int isNewOccupiedTerrain;

	if ((0 == db->environment.isNewGlobalMap && 0 == db->status.isNewPose) ||
		db->status.stdDev > STD_DEV_MAX ||
		0 == db->sensorData.mapScanList.back)
	{
		return;
	}

	db->environment.isNewGlobalMap = 0;

	rsd = &db->sensorData;
	lm = rsd->localMap;
	curr = (MapScan*)rsd->mapScanList.back->value;

	CamVectors_rotateToRobotOrient (&db->camVectors, curr->pose.orient);

#if defined(MAP_SCANS_HAVE_GRIDS)
#if defined(IGNORE_CAM_DATA)
	isNewOccupiedTerrain = MapIntegration_intBlankScan (
		db,
		curr,
		db->status.stdDev,
		lm,
		db->environment.navMap,
		&db->camVectors,
		1,
		1); // Won't be any unknown cells, but set anyway

#else // defined(IGNORE_CAM_DATA)

	if (curr->grid)
	{
		isNewOccupiedTerrain = MapIntegration_intCamScan (
			db,
			curr,
			db->status.stdDev,
			lm,
			db->environment.navMap,
			&db->camVectors,
			1,
			1); // Integrate unknown cells to ensure collisions are avoided.
	}
	else
	{
		isNewOccupiedTerrain = MapIntegration_intBlankScan (
			db,
			curr,
			db->status.stdDev,
			lm,
			db->environment.navMap,
			&db->camVectors,
			1,
			1); // Integrate unknown cells to ensure collisions are avoided.
	}
#endif // defined(IGNORE_CAM_DATA)

#else // defined(MAP_SCANS_HAVE_GRIDS)

	isNewOccupiedTerrain = MapIntegration_intSimScan (
		db,
		curr,
		db->status.stdDev,
		lm,
		db->environment.navMap,
#ifdef SIMULATION
		db->board->environment.tempMap,
#else
		db->environment.originalSim, // Have to give robot copy of simulated map if on seperate process
#endif
		1);
#endif // defined(MAP_SCANS_HAVE_GRIDS)

	// Flag that the robot should at least update around the current
	// sensor data
	if (DONT_UPDATE_SEARCH_GRID == rsd->isSearchGridUpdateRequired)
	{
		rsd->isSearchGridUpdateRequired = UPDATE_SEARCH_GRID_PATCH;
	}

	if (isNewOccupiedTerrain && DONT_UPDATE_NAV_MAP == rsd->isNavMapUpdateRequired)
	{
		rsd->isNavMapUpdateRequired = UPDATE_OCCUPIED_TERRAIN;
	}

#ifdef PRINT_EVENTS
//	MapCore_logMapScan (db->xmlLog, db->geometryConstants.exp_optDist, db->status.nIterations, curr);
	MapCore_logMapScan (db->xmlLog, SCAN_CENTRE_DIST, db->status.nIterations, curr);
#endif
	db->sensorData.isUnincorporatedMapScan = 0;
}

int RobotMapProcessing_intAssumedNeighbourScans (RobotDatabase *db)
{
	int i;
	PointF pt;
	PointI pti;
	PointF loc;
	float orient;
	PointI localMapCentre = db->sensorData.localMapCentre;
//	float dist = db->geometryConstants.exp_optDist + CAM_P_OFFSET;
	float dist = SCAN_CENTRE_DIST + CAM_P_OFFSET;
	int haveToUpdateGrids = 0;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i == db->status.index)
		{
			continue;
		}

		loc = db->groupData.robots[i].pose.loc;
		orient = db->groupData.robots[i].pose.orient;
		Geometry_ptFromOrient (loc.x, loc.y, &pt.x, &pt.y, dist, orient);
		pti = PointF_toPointI (pt);
		if (Geometry_isPtOnLocalMap (pti, localMapCentre, ROBOT_ON_LOC_MAP_LEEWAY))
		{
			CamVectors_rotateToRobotOrient (&db->camVectors, orient);

			MapIntegrationImpl_intScan_neighbour (
				&db->geometryConstants,
				&db->camVectors,
				loc,
				orient,
				CAM_P_OFFSET,
				SCAN_CENTRE_DIST,
				db->sensorData.localMap,
				FAKE_NBOUR_TERRAIN);
			haveToUpdateGrids = 1;
		}
	}
	return haveToUpdateGrids;
}

void RobotMapIntegration_integrateNewMapDataFromBoard (RobotDatabase *db)
{
	if (db->environment.isNewGlobalMap)
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<IsNewGlobalMap />\n");
#endif
		if (db->sensorData.hasDataBeenIncd)
		{
			db->sensorData.hasDataBeenIncd = 0;

			MapCore_decompressLocalMap (
				db->sensorData.localMap,
				&db->environment.mapFromBoard);
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<HasDataBeenIncd />\n");
#endif
		}
		else if (db->sensorData.isLocalMapEffected)
		{
			db->sensorData.isLocalMapEffected = 0;

			db->sensorData.tempLocalMap->orig = db->sensorData.localMap->orig;

			MapCore_decompressLocalMap (
				db->sensorData.tempLocalMap,
				&db->environment.mapFromBoard);

			MapCore_copyMap_greaterCertainty (db->sensorData.tempLocalMap, db->sensorData.localMap);

#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<IsLocalMapEffected />\n");
#endif
		}

		if (!db->status.isNewPose)
		{
			db->status.isNewPose = 1;
			SensorProcessing_processSensorData (db, 1);
			db->status.isNewPose = 0;
		}

		db->sensorData.isNavMapUpdateRequired = UPDATE_WHOLE_NAV_MAP;
		db->sensorData.isSearchGridUpdateRequired = UPDATE_WHOLE_SEARCH_GRID;
	}
}

#endif // ifndef BOARD
