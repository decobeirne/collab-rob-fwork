#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "MapProcessing.h"
#include "../../Common/BitArray.h"

//! Calculate coordinate of loc map centre point given robots optimum mapping point
int calcLocalMapCentreCoordinate (const int ptCoord)
{
	int coord;
	int diff;
	diff = ((ptCoord - LOC_MAP_DIMS/2) % LOC_MAP_DIFF);

	// Align to DIMS/2 + n*DIFF
	if (diff < LOC_MAP_DIFF/2)
	{
		coord = (((ptCoord - LOC_MAP_DIMS/2) / LOC_MAP_DIFF) * LOC_MAP_DIFF) + LOC_MAP_DIMS/2;
	}
	else
	{
		coord = ((((ptCoord - LOC_MAP_DIMS/2) / LOC_MAP_DIFF) + 1) * LOC_MAP_DIFF) + LOC_MAP_DIMS/2;
	}

	if (coord < LOC_MAP_DIMS/2)
	{
		coord = LOC_MAP_DIMS/2;
	}
	else if (coord > (ENVIR_DIMS - LOC_MAP_DIMS/2))
	{
		coord = (ENVIR_DIMS - LOC_MAP_DIMS/2);
	}
	return coord;
}


PointI RobotMapProcessing_centreLocalMap2 (const PointI pt)
{
	PointI centre;
	centre.x = calcLocalMapCentreCoordinate (pt.x);
	centre.y = calcLocalMapCentreCoordinate (pt.y);
	return centre;
}

#define SAFE_DIST_FROM_CENTRE 32 // (84/2) - 10, i.e. 10 from edge of map

PointI RobotMapProcessing_centreLocalMap (const PointI pt1, const PointI pt2, const PointI pt3)
{
	PointI centre;

	PointI avg;
	PointF avgf;
	avg.x = (pt1.x + pt2.x + pt3.x);
	avg.y = (pt1.y + pt2.y + pt3.y);
	avgf = PointI_toPointF (avg);
	avgf.x *= 0.333333f;
	avgf.y *= 0.333333f;
	avg = PointF_toPointI (avgf); // This is ugly, but fix later

	centre.x = calcLocalMapCentreCoordinate (avg.x);
	centre.y = calcLocalMapCentreCoordinate (avg.y);

#ifdef SIMULATION
	if (abs (pt1.x - centre.x) >= SAFE_DIST_FROM_CENTRE ||
		abs (pt2.x - centre.x) >= SAFE_DIST_FROM_CENTRE ||
		abs (pt3.x - centre.x) >= SAFE_DIST_FROM_CENTRE)
	{
		printf("centre x %d too far from %d OR %d OR %d\n", centre.x, pt1.x, pt2.x, pt3.x);
	}

	if (abs (pt1.y - centre.y) >= SAFE_DIST_FROM_CENTRE ||
		abs (pt2.y - centre.y) >= SAFE_DIST_FROM_CENTRE ||
		abs (pt3.y - centre.y) >= SAFE_DIST_FROM_CENTRE)
	{
		printf("centre y %d too far from %d OR %d OR %d\n", centre.y, pt1.y, pt2.y, pt3.y);
	}
#endif

	return centre;
}

#if 0 // old
PointI RobotMapProcessing_centreLocalMap (const PointI pt1, const PointI pt2)
{
	PointI centre;
	centre.x = calcLocalMapCentreCoordinate (pt1.x);
	if (pt2.x < (centre.x - SAFE_DIST_FROM_CENTRE))
	{
		// The centre x value is too great, so shift it left. The left-most
		// point will be dims/2
		centre.x = max(centre.x - LOC_MAP_DIFF, LOC_MAP_DIMS/2);
#ifdef SIMULATION
		printf("Shifted local map centre x left to %d\n", centre.x);
#endif
	}
	else if (pt2.x >= (centre.x + SAFE_DIST_FROM_CENTRE))
	{
		// The centre x value is too small, so shift it right. The left-most
		// point will be globalDims-(localMapDims/2), i.e. the last local map
		// has coords from 420..503, therefore, the centre is at 462
		centre.x = min(centre.x + LOC_MAP_DIFF, ENVIR_DIMS - (LOC_MAP_DIMS/2));
#ifdef SIMULATION
		printf("Shifted local map centre x right to %d\n", centre.x);
#endif
	}

	centre.y = calcLocalMapCentreCoordinate (pt1.y);
	if (pt2.y < (centre.y - SAFE_DIST_FROM_CENTRE))
	{
		centre.y = max(centre.y - LOC_MAP_DIFF, LOC_MAP_DIMS/2);
#ifdef SIMULATION
		printf("Shifted local map centre y down to %d\n", centre.y);
#endif
	}
	else if (pt2.y >= (centre.y + SAFE_DIST_FROM_CENTRE))
	{
		centre.y = min(centre.y + LOC_MAP_DIFF, ENVIR_DIMS - (LOC_MAP_DIMS/2));
#ifdef SIMULATION
		printf("Shifted local map centre y up to %d\n", centre.y);
#endif
	}
	return centre;
}
#endif // old

#undef SAFE_DIST_FROM_CENTRE

void RobotMapProcessing_setLocalMapOrigin (RobotDatabase *db)
{
	db->sensorData.localMap->orig = db->sensorData.localMapCentre;
	db->sensorData.localMap->orig.x -= LOC_MAP_DIMS/2;
	db->sensorData.localMap->orig.y -= LOC_MAP_DIMS/2;

	db->sensorData.tempLocalMap->orig = db->sensorData.localMap->orig;
	db->environment.navMap->orig = db->sensorData.localMap->orig;
}

/*!
Profits are pushed onto this list in redrawLocalMap. Each gtepSessionId of each
EXPLORATION scan is checked. Profit is added to gtepProfitList based on the
GTEP_ALLOC_PROFIT coefficient.

When approximating the local map, the estimated profit for EXPLORATION as well as the
total estimated profit are written out. Based on the observed ratio actualProfit:accreditedProfit,
the amount of profit that EXPLORATION should give to each GTEP session can be calculated.
*/
void printSubmittedMap (RobotDatabase *db, LocalMapInfo *localMapInfo, const int loopClosing)
{
	if (loopClosing)
	{
		fprintf (db->xmlLog, "<SubmitCloseLoopLocalMap>");
	}
	else
	{
		fprintf (db->xmlLog, "<SubmitLocalMap>");
	}
	fprintf (db->xmlLog, "robotIndex=%d localMapIndex=%d nScans=%f isProvisional=%d closeLoopSession=%d estdGain=%f estdGross=%f avgStdDev=%f compressedSize=%d",
		localMapInfo->robotIndex, localMapInfo->localMapIndex, localMapInfo->nScans, localMapInfo->isProvisional,
		localMapInfo->closeLoopSession, localMapInfo->mapGain, localMapInfo->gross, localMapInfo->avgStdDev,
		db->environment.mapToSubmit.usedSize);
	if (loopClosing)
	{
		fprintf (db->xmlLog, "</SubmitCloseLoopLocalMap>\n");
	}
	else
	{
		fprintf (db->xmlLog, "</SubmitLocalMap>\n");
	}
}

#ifdef RECORDING_LOCAL_MAPS
extern char experimentDirName[128];
extern int CloseLoop_isLoopClosePossible (RobotDatabase *db);

void saveSubmittedMap (RobotDatabase *db, Image *localMap, const int localMapIndex)
{
	FILE *f;
	char filename[128];
	char fullpath[256];

	strcpy (fullpath, experimentDirName);
	sprintf (filename, "/localMap%05d.dat", localMapIndex);
	strcat (fullpath, filename);
	f = fopen (fullpath, "w");
	fwrite (localMap->data, 1, LOC_MAP_DIMS * LOC_MAP_DIMS * 1, f);
	fclose (f);

	fprintf (db->xmlLog, "<SerializedLocalMap>filename=\"%s\"</SerializedLocalMap>\n", filename);
}
#endif

#if 0
extern int robotMapWindowSize[4];
#endif

void RobotMapProcessing_prepareLocalMap (RobotDatabase *db)
{
	LocalMapInfo localMapInfo;
	localMapInfo = RobotMapIntegration_redrawLocalMap (db);

#if 0
	Image_show_givenIplImage (
		db->sensorData.localMap,
		db->status.locWinName,
		db->localMapIplImage,
		robotMapWindowSize);
	cvWaitKey (0);
#endif

	if (0 == localMapInfo.nScans)
	{
		return;
	}

	MapCore_compressLocalMap (db->sensorData.localMap, &db->environment.mapToSubmit);

	localMapInfo.localMapIndex = db->sensorData.localMapDataIndex;
	localMapInfo.robotIndex = db->status.index;
	++db->sensorData.localMapDataIndex;

//	if (db->behaviourData.closeLoop.isLoopClosePossible)
	if (CloseLoop_isLoopClosePossible (db))
	{
		localMapInfo.isProvisional = PROVISIONAL_CLOSE_LOOP_DATA;
		localMapInfo.closeLoopSession = db->behaviourData.closeLoop.currentCloseLoopSession;
	}
	else
	{
		localMapInfo.isProvisional = NOT_CLOSE_LOOP_DATA;
		localMapInfo.closeLoopSession = -1;
	}

	db->sensorData.isMapDataReady = 1;
	db->behaviourData.closeLoop.isMoreMapDataComing = 0;

#ifdef PRINT_PROFIT
	// Must print map data to XML so profit can be calculated in post-processing.
	printSubmittedMap (db, &localMapInfo, 0);
#endif

#ifdef RECORDING_LOCAL_MAPS
	saveSubmittedMap (db, db->sensorData.localMap, localMapInfo.localMapIndex);
#endif

	db->environment.mapInfoToSubmit = localMapInfo;
}

int checkPtAgainstLocalMap (const PointI pt, const PointI centre, const int distAllowed)
{
	if (pt.x < centre.x - distAllowed && centre.x > LOC_MAP_DIMS/2)
	{
		return 0;
	}
	if (pt.x >= centre.x + distAllowed && centre.x < ENVIR_DIMS - LOC_MAP_DIMS/2)
	{
		return 0;
	}
	if (pt.y < centre.y - distAllowed && centre.y > LOC_MAP_DIMS/2)
	{
		return 0;
	}
	if (pt.y >= centre.y + distAllowed && centre.y < ENVIR_DIMS - LOC_MAP_DIMS/2)
	{
		return 0;
	}
	return 1;
}

//! Determine if the current local map used by the robot is OK
int RobotMapProcessing_isLocalMapCentreOK (RobotDatabase *db, const Pose pose)
{
	int isPtOnMap;
	PointF pt;
	PointI poseI, mappedPtI;
	float closestMappedDist;

	poseI = PointF_toPointI (pose.loc);

#ifdef PRINT_EVENTS
	// Write warning to log if robot is actually outside local map, i.e. not using safety margin from edge.
	isPtOnMap = checkPtAgainstLocalMap (poseI, db->sensorData.localMapCentre, LOC_MAP_DIMS/2);
	if (!isPtOnMap)
	{
		fprintf (db->xmlLog, "<RobotOutsideLocalMap>loc=(%f,%f) localMapCentre=(%d,%d)</RobotOutsideLocalMap>\n",
			pose.loc.x, pose.loc.y,
			db->sensorData.localMapCentre.x, db->sensorData.localMapCentre.y);
	}
#endif

	isPtOnMap = checkPtAgainstLocalMap (poseI, db->sensorData.localMapCentre, (LOC_MAP_DIMS/2 - LOC_MAP_EDGE_LEEWAY_2));

//	if (isPtOnMap)
	{
		closestMappedDist = (CAM_P_OFFSET + db->geometryConstants.cam_minDist);
		Geometry_ptFromOrient (pose.loc.x, pose.loc.y, &pt.x, &pt.y, closestMappedDist, pose.orient);
		mappedPtI = PointF_toPointI (pt);
		isPtOnMap &= checkPtAgainstLocalMap (mappedPtI, db->sensorData.localMapCentre, (LOC_MAP_DIMS/2 - LOC_MAP_EDGE_LEEWAY_2));
	}

	if (!isPtOnMap)
	{
		db->sensorData.isLocalMapResetRequired = CENTRE_LOC_MAP;
		db->sensorData.localMapResetPt1 = mappedPtI;
		db->sensorData.localMapResetPt2 = poseI;
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<IsLocalMapCentreOK>nextPose=(%f,%f,%f) centre=(%d,%d)</IsLocalMapCentreOK>\n",
			pose.loc.x, pose.loc.y, pose.orient,
			db->sensorData.localMapCentre.x, db->sensorData.localMapCentre.y);
#endif
	}
	return isPtOnMap;
}

extern Dest* getBehaviourDest (RobotDatabase *db, const BEHAVIOUR behaviour);

void RobotMapProcessing_determineIsLocalMapReady (RobotDatabase *db)
{
	RobotSensorData *rsd = &db->sensorData;
	PointF pt;
	Dest *dest;
	IKData *ikData;
	int resetMapForSupervision;

	// Flag set when loop closed. Also if robot is at edge of local map just before move is made
	if (CENTRE_LOC_MAP == rsd->isLocalMapResetRequired)
	{
		rsd->isMapDataReady = 1;
		db->behaviourData.gotoExpPt.sessionIndex = -1;

		// I put this in to fix a bug, but this is obviously wrong. This should be set where we
		// set CENTRE_LOC_MAP, as we need to know what the next pose when we move will be etc.
//		Geometry_ptFromOrient (db->status.pose.loc.x, db->status.pose.loc.y, &pt.x, &pt.y, (CAM_P_OFFSET + SCAN_CENTRE_DIST), db->status.pose.orient);
//		db->sensorData.localMapResetPt1 = PointF_toPointI (pt);
//		db->sensorData.localMapResetPt2 = PointF_toPointI (db->status.pose.loc);

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<IsLocalMapReady>reason=\"CENTRE_LOC_MAP\"</IsLocalMapReady>\n");
#endif
		return;
	}

	// This will be set when GOTO_EXP_PT reaches its destination.
	if (SET_REQUESTED_LOC_MAP == rsd->isLocalMapResetRequired)
	{
		rsd->isMapDataReady = 1;
		db->behaviourData.gotoExpPt.sessionIndex = db->behaviourData.gotoExpPt.nextSessionIndex;
		++db->behaviourData.gotoExpPt.nextSessionIndex;
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<StartGTEPSession>gtepSessionId=%d</StartGTEPSession>\n", db->behaviourData.gotoExpPt.sessionIndex);
		fprintf (db->xmlLog, "<IsLocalMapReady>reason=\"SET_REQUESTED_LOC_MAP\"</IsLocalMapReady>\n");
#endif
		return;
	}

	rsd->isLocalMapResetRequired = DONT_RESET_LOC_MAP;
	rsd->isMapDataReady = 0;

	// Adopting supervision.
	resetMapForSupervision = 0;
	if (1 == db->behaviourData.supervision.wasPropSuccessful)
	{
		db->behaviourData.supervision.wasPropSuccessful = 0;
		db->behaviourData.gotoExpPt.sessionIndex = -1;
		resetMapForSupervision = 1;
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<IsLocalMapReady>reason=\"wasPropSuccesful\"</IsLocalMapReady>\n");
#endif
	}

	if (db->behaviourData.supervision.arrivedAtDest)
	{
		db->behaviourData.supervision.arrivedAtDest = 0;
		resetMapForSupervision = 1;
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<IsLocalMapReady>reason=\"arrivedAtSupDest\"</IsLocalMapReady>\n");
#endif
	}

	if (resetMapForSupervision)
	{
		rsd->isLocalMapResetRequired = CENTRE_LOC_MAP;
		rsd->isMapDataReady = 1;

//		Geometry_ptFromOrient (db->status.pose.loc.x, db->status.pose.loc.y, &pt.x, &pt.y, (CAM_P_OFFSET + db->geometryConstants.exp_optDist), db->status.pose.orient);
		Geometry_ptFromOrient (db->status.pose.loc.x, db->status.pose.loc.y, &pt.x, &pt.y, (CAM_P_OFFSET + SCAN_CENTRE_DIST), db->status.pose.orient);
		db->sensorData.localMapResetPt1 = PointF_toPointI (pt);
		db->sensorData.localMapResetPt2 = PointF_toPointI (db->status.pose.loc);
	}

	// Scan limit reached
	else if (
		rsd->mapScanList.size > rsd->scanLimit &&
//		0 == db->behaviourData.closeLoop.isLoopClosePossible)
		0 == CloseLoop_isLoopClosePossible (db))
	{
		// If the robot is following some complex path, don't submit the map yet.
		dest = getBehaviourDest (db, db->behaviourData.behaviour);
		ikData = &db->status.ikData;
		if (!dest || dest->type & IS_COARSE_NAV_OK || dest->type & IS_NEW_DEST || -1 == ikData->index || ikData->index >= ikData->len)
		{
			rsd->isLocalMapResetRequired = CENTRE_LOC_MAP;
			rsd->isMapDataReady = 1;
			db->behaviourData.gotoExpPt.sessionIndex = -1;

//			Geometry_ptFromOrient (db->status.pose.loc.x, db->status.pose.loc.y, &pt.x, &pt.y, (CAM_P_OFFSET + db->geometryConstants.exp_optDist), db->status.pose.orient);
			Geometry_ptFromOrient (db->status.pose.loc.x, db->status.pose.loc.y, &pt.x, &pt.y, (CAM_P_OFFSET + SCAN_CENTRE_DIST), db->status.pose.orient);
			db->sensorData.localMapResetPt1 = PointF_toPointI (pt);
			db->sensorData.localMapResetPt2 = PointF_toPointI (db->status.pose.loc);
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<IsLocalMapReady>reason=\"scanLimit\"</IsLocalMapReady>\n");
#endif
		}
		/*else
		{
			printf("mapScanList.size %d but dest->type %d and ikData->index %d ->len %d\n",
				rsd->mapScanList.size, dest->type, ikData->index, ikData->len);
		}*/
	}
}

//! Reset location of robot's local map
void RobotMapProcessing_resetLocalMap (RobotDatabase *db)
{
	RobotSensorData *rsd = &db->sensorData;

	if (SET_REQUESTED_LOC_MAP == rsd->isLocalMapResetRequired)
	{
//		rsd->localMapCentre = PointF_toPointI (db->behaviourData.gotoExpPt.profit.dest.dest.loc);
		rsd->localMapCentre = db->behaviourData.gotoExpPt.requestedLocalMap;
	}
	else if (CENTRE_LOC_MAP == rsd->isLocalMapResetRequired)
	{
		DEBUG_ASSERT (db->sensorData.localMapResetPt1.x != MIN_INT32);
		db->sensorData.localMapCentre = RobotMapProcessing_centreLocalMap (db->sensorData.localMapResetPt1, db->sensorData.localMapResetPt2, PointF_toPointI (db->status.pose.loc));
	}

	RobotMapProcessing_setLocalMapOrigin (db);

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<ResetLocalMap>isLocalMapResetRequired=%d localMapCentre=(%d,%d) localMapOrigin=(%d,%d)</ResetLocalMap>\n",
			(int)rsd->isLocalMapResetRequired,
			db->sensorData.localMapCentre.x, db->sensorData.localMapCentre.y,
			db->sensorData.localMap->orig.x, db->sensorData.localMap->orig.y);
#endif
	rsd->isLocalMapResetRequired = DONT_RESET_LOC_MAP;

	BitArray_reset (db->environment.mappedCellGrid, EXP_GRID_DIMS, EXP_GRID_DIMS);
}

#if defined (SIMULATION) || defined (RERUNNING_ROBOT)
void drawRobotOnNavMap (Image *navMap, const PointF loc, uchar temps[9])
{
	int x, y, i, j, k;
	uchar val1, val2;
	k = 0;
	x = (int)(loc.x - navMap->orig.x);
	y = (int)(loc.y - navMap->orig.y);
	{
//		printf ("b 52,20=%d\n", Image_getPixel_dontCheck (navMap, 52, 20));
	}
	for (i = (x - 1); i < (x + 2); ++i)
	{
		for (j = (y - 1); j < (y + 2); ++j)
		{
			val1 = Image_getPixel_check (navMap, i, j);
			temps[k] = val1;
			if (val1 != INVALID_CELL)
			{
				switch (val1)
				{
					case 0:
						val2 = 127;
						break;
					case 20:
						val2 = 128;
						break;
					case 70:
						val2 = 129;
						break;
					default:
						val2 = 130;
				}
				Image_setPixel_dontCheck (navMap, i, j, val2);
			}
			++k;
		}
	}
}

void removeRobotFromNavMap (Image *navMap, const PointF loc, uchar temps[9])
{
	int x, y, i, j, k;
	uchar val1;
//	uchar val2;
	k = 0;
	x = (int)(loc.x - navMap->orig.x);
	y = (int)(loc.y - navMap->orig.y);
	for (i = (x - 1); i < (x + 2); ++i)
	{
		for (j = (y - 1); j < (y + 2); ++j)
		{
			val1 = temps[k];
			Image_setPixel_dontCheck (navMap, i, j, val1); // Set the *original* vals from the temp array
/*			if (val1 != INVALID_CELL)
			{
				switch (val1)
				{
					
					case 127:
						val2 = 0;
						break;
					case 128:
						val2 = 20;
						break;
					case 129:
						val2 = 70;
						break;
					default:
						val2 = 254;
				}
				Image_setPixel_dontCheck (navMap, i, j, val2);
			}*/
			++k;
		}
	}
	{
//		printf ("a 52,20=%d\n", Image_getPixel_dontCheck (navMap, 52, 20));
	}
}

#ifndef IS_GUMSTIX
void RobotMapProcessing_printNavMap (
	RobotDatabase *db)
{
	int i, j;
	uchar val;

	fprintf (db->xmlLog, "<NavMap>\n");
	for (j = db->environment.navMap->height - 1; j >= 0; --j)
	{
		for (i = 0; i < db->environment.navMap->width; ++i)
		{
			val = Image_getPixel_dontCheck (db->environment.navMap, i, j);
//			fprintf (db->xmlLog, "%d,", val);
			fprintf (db->xmlLog, "(%d,%d)%d,", i, j, val);
		}
		fprintf (db->xmlLog, "\n");
	}
	fprintf (db->xmlLog, "</NavMap>\n");
}
#endif // ifndef IS_GUMSTIX

//#ifdef RERUNNING_ROBOT
//#include "../../Board/Data/BoardDatabase.h"
//#endif

#if 0
void bar(Image *navMap, int i, int j, char *str)
{
	printf ("%s 52,20=%d\n", str, Image_getPixel_dontCheck (navMap, i, j), i, j);
}
#endif

void RobotMapProcessing_displayNavMap (Image *navMap,
									 const PointF loc,
									 const schar *winName,
									 int windowSize[4],
									 IplImage *localMapIplImage)
{
	uchar temps[9];

	// The nav map can have the colours: 0, 254, 20, 70. Therefore use temporary
	// values to draw the robot: 127,128,129,130.

	drawRobotOnNavMap (navMap, loc, temps);

	Image_show_givenIplImage (
		navMap,
		winName,
		localMapIplImage,
		windowSize);

	removeRobotFromNavMap (navMap, loc, temps);
}
#endif // defined (SIMULATION) || defined (RERUNNING_ROBOT)

void RobotMapProcessing_updateNavMap (RobotDatabase *db)
{
	if (UPDATE_WHOLE_NAV_MAP == db->sensorData.isNavMapUpdateRequired)
	{
		MapCore_thresholdImage (
			db->sensorData.localMap,
			db->environment.navMap,
			THRESHOLD_TERRAIN,
			OCCUPIED_TERRAIN,
			FREE_TERRAIN,
			1,
			LOC_MAP_DIMS);
	}

	if (UPDATE_WHOLE_NAV_MAP == db->sensorData.isNavMapUpdateRequired ||
		UPDATE_OCCUPIED_TERRAIN == db->sensorData.isNavMapUpdateRequired)
	{
		MapCore_dilateNavMap (
			db->environment.navMap,
			db->environment.navMap,
			OCCUPIED_TERRAIN,
			NARROW_OCCUPIED,
			BROAD_OCCUPIED,
			FREE_TERRAIN,
			NARROW_NAV_MAP_DILATE_DIST,
			BROAD_NAV_MAP_DILATE_DIST);

		MapCore_dilateBorder (
			db->environment.navMap,
			db->environment.navMap,
			BROAD_OCCUPIED,
			LOC_MAP_EDGE_LEEWAY);
	}
}

int RobotMapProcessing_isMarkedAsRobot (
	const uchar val)
{
	return ((val >= 56 && val <= 59) || (val >= 6 && val <= 9));
}

//float RobotMapProcessing_robotManhattanDist (
//	const int ptx,
//	const int pty,
//	const int refx,
//	const int refy)
//{
//	return (fabs (ptx - refx) + fabs (pty - refy));
//}

void RobotMapProcessing_incRobotIntoNavMap (
	RobotDatabase *db,
	const int index,
	const PointI localMapCentre,
	const PointI localMapOrig,
	Image *navMap)
{
	int x, y;
	PointI loc;
	PointI bl, tr;
	uchar origVal, newVal;

	loc = PointF_toPointI (db->groupData.robots[index].pose.loc);
	if (Geometry_isPtOnLocalMap (loc, localMapCentre, ROBOT_ON_LOC_MAP_LEEWAY))
	{
		loc.x -= localMapOrig.x;
		loc.y -= localMapOrig.y;

		bl.x = loc.x - BROAD_VEHICLE_DIST;
		bl.y = loc.y - BROAD_VEHICLE_DIST;
		tr.x = loc.x + BROAD_VEHICLE_DIST;
		tr.y = loc.y + BROAD_VEHICLE_DIST;
		bl.x = max (0, bl.x);
		bl.y = max (0, bl.y);
		tr.x = min (LOC_MAP_DIMS, tr.x);
		tr.y = min (LOC_MAP_DIMS, tr.y);
		for (x = bl.x; x < tr.x; ++x)
		{
			for (y = bl.y; y < tr.y; ++y)
			{
//				if (RobotMapProcessing_robotManhattanDist (x, y, loc.x, loc.y) > 16.0f) // BROAD_VEHICLE_DIST
				if (Geometry_distSqrd (x, y, loc.x, loc.y) > 256.0f) // BROAD_VEHICLE_DIST^2
				{
					continue;
				}
//				Image_setPixel_dontCheck (navMap, x, y, BROAD_VEHICLE_TERRAIN);

				origVal = Image_getPixel_dontCheck (navMap, x, y);
				switch (origVal)
				{
				case FREE_TERRAIN:
					newVal = 59;
					break;
				case BROAD_OCCUPIED:
					newVal = 58;
					break;
				case NARROW_OCCUPIED:
					newVal = 57;
					break;
				case OCCUPIED_TERRAIN:
					newVal = 56;
					break;
				default:
					// if this was marked for another robot already, then just leave it
					newVal = origVal;
					break;
				}
				Image_setPixel_dontCheck (navMap, x, y, newVal);
			}
		}

		bl.x = loc.x - NARROW_VEHICLE_DIST;
		bl.y = loc.y - NARROW_VEHICLE_DIST;
		tr.x = loc.x + NARROW_VEHICLE_DIST;
		tr.y = loc.y + NARROW_VEHICLE_DIST;
		bl.x = max (0, bl.x);
		bl.y = max (0, bl.y);
		tr.x = min (LOC_MAP_DIMS, tr.x);
		tr.y = min (LOC_MAP_DIMS, tr.y);
		for (x = bl.x; x < tr.x; ++x)
		{
			for (y = bl.y; y < tr.y; ++y)
			{
//				if (RobotMapProcessing_robotManhattanDist (x, y, loc.x, loc.y) > 14.0f) // NARROW_VEHICLE_DIST
				if (Geometry_distSqrd (x, y, loc.x, loc.y) > 196.0f) // NARROW_VEHICLE_DIST^2
				{
					continue;
				}
//				Image_setPixel_dontCheck (navMap, x, y, NARROW_VEHICLE_TERRAIN);

				origVal = Image_getPixel_dontCheck (navMap, x, y);
				switch (origVal)
				{
				case FREE_TERRAIN:
				case 59:
					newVal = 9;
					break;
				case BROAD_OCCUPIED:
				case 58:
					newVal = 8;
					break;
				case NARROW_OCCUPIED:
				case 57:
					newVal = 7;
					break;
				case OCCUPIED_TERRAIN:
				case 56:
					newVal = 6;
					break;
				default:
					// if this was marked for another robot already, then just leave it
					newVal = origVal;
					break;
				}
				Image_setPixel_dontCheck (navMap, x, y, newVal);
			}
		}
	}


}

void RobotMapProcessing_incRobotsIntoNavMap (RobotDatabase *db)
{
	int i;
	PointI localMapCentre = db->sensorData.localMapCentre;
	PointI localMapOrig = db->sensorData.localMap->orig;
	Image *navMap = db->environment.navMap;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i == db->status.index)
		{
			continue;
		}

		RobotMapProcessing_incRobotIntoNavMap (
			db,
			i,
			localMapCentre,
			localMapOrig,
			navMap);
	}
}

void RobotMapProcessing_removeRobotFromNavMap (
	RobotDatabase *db,
	const int robotIndex,
	const PointI localMapCentre,
	const PointI localMapOrig,
	Image *navMap)
{
	int x, y;
	PointI loc;
	PointI bl, tr;
	uchar origVal, newVal;

	loc = PointF_toPointI (db->groupData.robots[robotIndex].pose.loc);
	if (Geometry_isPtOnLocalMap (loc, localMapCentre, ROBOT_ON_LOC_MAP_LEEWAY))
	{
		loc.x -= localMapOrig.x;
		loc.y -= localMapOrig.y;

		bl.x = loc.x - BROAD_VEHICLE_DIST;
		bl.y = loc.y - BROAD_VEHICLE_DIST;
		tr.x = loc.x + BROAD_VEHICLE_DIST;
		tr.y = loc.y + BROAD_VEHICLE_DIST;
		bl.x = max (0, bl.x);
		bl.y = max (0, bl.y);
		tr.x = min (LOC_MAP_DIMS, tr.x);
		tr.y = min (LOC_MAP_DIMS, tr.y);
		for (x = bl.x; x < tr.x; ++x)
		{
			for (y = bl.y; y < tr.y; ++y)
			{
				origVal = Image_getPixel_dontCheck (navMap, x, y);
				switch (origVal)
				{
				case 59:
				case 9:
					newVal = FREE_TERRAIN;
					break;
				case 58:
				case 8:
					newVal = BROAD_OCCUPIED;
					break;
				case 57:
				case 7:
					newVal = NARROW_OCCUPIED;
					break;
				case 56:
				case 6:
					newVal = OCCUPIED_TERRAIN;
					break;
				default:
					// This shouldn't occur - could assert to verify this
					newVal = origVal;
					break;
				}
				Image_setPixel_dontCheck (navMap, x, y, newVal);
			}
		}
	}
}

int RobotMapProcessing_checkForRobotsInLocalMap (RobotDatabase *db)
{
	int i;
	int haveToUpdateOtherRobots;
	int areThereCurrentlyRobots = 0;
	PointI loc;
	PointI localMapCentre = db->sensorData.localMapCentre;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		if (i == db->status.index)
		{
			continue;
		}

		loc = PointF_toPointI (db->groupData.robots[i].pose.loc);
		if (Geometry_isPtOnLocalMap (loc, localMapCentre, ROBOT_ON_LOC_MAP_LEEWAY))
		{
			areThereCurrentlyRobots = 1;
			break;
		}
	}

	haveToUpdateOtherRobots = areThereCurrentlyRobots | db->sensorData.areOtherRobotsInLocalMap;
	db->sensorData.areOtherRobotsInLocalMap = areThereCurrentlyRobots;

	return haveToUpdateOtherRobots;
}

PointI calcCellMidpt (
	const PointI pt,
	const int distBetweenMidpts,
	const PointI originCellMidpt);

void RobotMapProcessing_updateUnreachableExpCellGrid (RobotDatabase *db,
													  Image *navMap,
													  uchar unreachableExpCellGrid[((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1])
{
	PointI iter, globalIter, originIterOffset, cellMidpt, originCellMidpt;
	uchar pixelValue;

	originCellMidpt.x = db->sensorData.localMap->orig.x + (EXP_AREA / 2);
	originCellMidpt.y = db->sensorData.localMap->orig.y + (EXP_AREA / 2);
	originIterOffset.x = db->sensorData.localMap->orig.x / EXP_AREA;
	originIterOffset.y = db->sensorData.localMap->orig.y / EXP_AREA;

	for (iter.x = 0; iter.x < EXP_GRID_DIMS; ++iter.x)
	{
		for (iter.y = 0; iter.y < EXP_GRID_DIMS; ++iter.y)
		{
			cellMidpt = calcCellMidpt (iter, EXP_AREA, originCellMidpt);

			pixelValue = Image_getPixel_check (
				navMap,
				cellMidpt.x - navMap->orig.x,
				cellMidpt.y - navMap->orig.y);

//			cellIndex = PointI_calcExpCellIndex (cellMidpt, EXP_AREA);

			globalIter.x = originIterOffset.x + iter.x;
			globalIter.y = originIterOffset.y + iter.y;

			// checkMove() is called when calculating IK moves. This checks the LOS between
			// poses against these values.
			if (pixelValue == OCCUPIED_TERRAIN ||
				pixelValue == NARROW_OCCUPIED)
			{
				BitArray_setElement_pt (
					unreachableExpCellGrid,
					globalIter,
					GLOB_EXP_GRID_DIMS,
					1);
			}
		}
	}
}

int RobotMapProcessing_isLocMapCellObstructed (
	const int i, const int j,
	Image *navMap)
{
	int iStart, iEnd, jStart, jEnd;
	int iCell, jCell;
	int nOccupied;

	iStart = i * NAV_CELL_AREA;
	jStart = j * NAV_CELL_AREA;
	iEnd = iStart + NAV_CELL_AREA;
	jEnd = jStart + NAV_CELL_AREA;

	nOccupied = 0;
	for (iCell = iStart; iCell < iEnd; ++iCell)
	{
		for (jCell = jStart; jCell < jEnd; ++jCell)
		{
			if (Image_getPixel_dontCheck (navMap, iCell, jCell) <= BROAD_VEHICLE_TERRAIN)
			{
				++nOccupied;
			}
		}
	}

	return (nOccupied > NAV_CELL_OCCUPIED_THRESHOLD);
}

void RobotMapProcessing_updateWholeObstructedGrid (
	RobotDatabase *db,
	Image *navMap)
{
	int i, j;
	PointI localMapOriginIndex;
	PointI obstdGridIndex;
	const int nNavCellsLocalMap = LOC_MAP_DIMS/NAV_CELL_AREA;

	localMapOriginIndex = navMap->orig;
	localMapOriginIndex.x += NAV_CELL_AREA/2;
	localMapOriginIndex.y += NAV_CELL_AREA/2;
	localMapOriginIndex = PointI_calcNavCellMidptObstdCell (localMapOriginIndex);

	for (i = 0; i < nNavCellsLocalMap; ++i)
	{
		for (j = 0; j <nNavCellsLocalMap; ++j)
		{
			if (RobotMapProcessing_isLocMapCellObstructed (i, j, navMap))
			{
				obstdGridIndex.x = localMapOriginIndex.x + i;
				obstdGridIndex.y = localMapOriginIndex.y + j;
				BitArray_setElement_pt (
					db->environment.obstructedCellGrid,
					obstdGridIndex,
					NAV_GRID_DIMS,
					1);
			}
		}
	}

#ifdef PRINT_MAP_DETAIL
	fprintf (db->xmlLog, "<UpdateObstructedCellGrid>\n");
	fprintf (db->xmlLog, "localMapOriginIndex=(%d,%d)\n", localMapOriginIndex.x, localMapOriginIndex.y);
	fprintf (db->xmlLog, "<Cells>\n");
	for (j = nNavCellsLocalMap - 1; j >= 0; --j)
	{
		for (i = 0; i < nNavCellsLocalMap; ++i)
		{
			obstdGridIndex.x = localMapOriginIndex.x + i;
			obstdGridIndex.y = localMapOriginIndex.y + j;

			fprintf (db->xmlLog, "%d ",
				BitArray_checkElement_pt (
				db->environment.obstructedCellGrid,
				obstdGridIndex,
				NAV_GRID_DIMS));
		}
		fprintf (db->xmlLog, "\n");
	}
	fprintf (db->xmlLog, "</Cells>\n");
	fprintf (db->xmlLog, "</UpdateObstructedCellGrid>\n");
#endif
}

void RobotMapProcessing_updateObstructedGridAroundRobot (
	RobotDatabase *db,
	Image *navMap,
	const int robotIndex,
	List *cellsToResetToObstd)
{
	int i, j;
	PointI pt1, pt2;
	PointI localMapOriginIndex;
	PointI obstdGridIndex;
	int isObstd;
	PointI *ptr;

	localMapOriginIndex = navMap->orig;
	localMapOriginIndex.x += NAV_CELL_AREA/2;
	localMapOriginIndex.y += NAV_CELL_AREA/2;
	localMapOriginIndex = PointI_calcNavCellMidptObstdCell (localMapOriginIndex);

	pt1 = PointF_toPointI (db->groupData.robots[robotIndex].pose.loc);
	pt1.x -= BROAD_VEHICLE_DIST; 
	pt1.y -= BROAD_VEHICLE_DIST;
	pt1 = PointI_calcNavCellMidptObstdCell (pt1); // Get nav cell index on global map first
	pt1.x -= localMapOriginIndex.x;
	pt1.y -= localMapOriginIndex.y;
	pt1.x = max (pt1.x, 0);
	pt1.y = max (pt1.y, 0);
	pt2.x = pt1.x + 3;
	pt2.y = pt1.y + 3;
	pt2.x = min (pt2.x, (LOC_MAP_DIMS / NAV_CELL_AREA));
	pt2.y = min (pt2.y, (LOC_MAP_DIMS / NAV_CELL_AREA));

	for (i = pt1.x; i < pt2.x; ++i)
	{
		for (j = pt1.y; j < pt2.y; ++j)
		{
			isObstd = RobotMapProcessing_isLocMapCellObstructed (i, j, navMap);

			// Only interested in cells that aren't occupied here
			if (!isObstd)
			{
				obstdGridIndex.x = localMapOriginIndex.x + i;
				obstdGridIndex.y = localMapOriginIndex.y + j;

				if (BitArray_checkElement_pt (
					db->environment.obstructedCellGrid,
					obstdGridIndex,
					NAV_GRID_DIMS))
				{
					// Add to list to reset when required
					ptr = (PointI*)malloc (sizeof (PointI));
					ptr->x = obstdGridIndex.x;
					ptr->y = obstdGridIndex.y;
					List_pushValue (cellsToResetToObstd, ptr);
#ifdef PRINT_EVENTS
					fprintf (db->xmlLog, "<UpdateObstdGridForCloseLoop>unset=(%d,%d)</UpdateObstdGridForCloseLoop>\n",
						obstdGridIndex.x, obstdGridIndex.y);
#endif
				}

				BitArray_setElement_pt (
					db->environment.obstructedCellGrid,
					obstdGridIndex,
					NAV_GRID_DIMS,
					0);
			}
		}
	}
}

void RobotMapProcessing_resetObstdGridCellsAroundRobot (
	RobotDatabase *db,
	List *cellsToResetToObstd)
{
	ListNode *iter;
	PointI *ptr;

	iter = cellsToResetToObstd->front;
	while (iter)
	{
		ptr = (PointI*)iter->value;
		BitArray_setElement_pt (
			db->environment.obstructedCellGrid,
			*ptr,
			NAV_GRID_DIMS,
			1);
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<UpdateObstdGridForCloseLoop>reset=(%d,%d)</UpdateObstdGridForCloseLoop>\n",
			ptr->x, ptr->y);
#endif
		iter = iter->next;
	}
}

int RobotMapProcessing_detectNavMapCollision (RobotDatabase *db, const Pose pose, const int obstacleValue)
{
	uchar pixVal;
	int res;
	PointI pt;

	pt = PointF_toPointI (pose.loc);

	pixVal = Image_getPixel_check (
		db->environment.navMap,
		pt.x - db->environment.navMap->orig.x,
		pt.y - db->environment.navMap->orig.y);

	DEBUG_ASSERT(pixVal != 255)
	if (pixVal <= obstacleValue)
	{
		res = 1;

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<DetectNavMapCollision>immediate=true loc=(%d,%d) val=%d</DetectNavMapCollision>\n", pt.x, pt.y, pixVal);
#endif
	}
	else
	{
		res = 0;
	}
	return res;
}

#endif // ifndef BOARD

