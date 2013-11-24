#include "../Common/RobotDefs.h"

#ifndef BOARD

#include "Robot.h"
#include "Behaviour/BehaviourAdoption.h"
#include "Behaviour/CloseLoop.h"
#include "Behaviour/Supervision.h"


void setupRobotsLocalMap (Robot *r)
{
	PointF pt;
	PointI pti;
	float closestMappedDist = (CAM_P_OFFSET + r->db.geometryConstants.cam_minDist);

	SensorProcessing_determineSensorRange (&r->db);

	Geometry_ptFromOrient (r->db.status.pose.loc.x, r->db.status.pose.loc.y, &pt.x, &pt.y, closestMappedDist, r->db.status.pose.orient);
	pti = PointF_toPointI (pt);
	r->db.sensorData.localMapResetPt1 = pti;
	r->db.sensorData.localMapResetPt2 = PointF_toPointI (r->db.status.pose.loc);

	// Just include pose twice (so we don't have to write another function, as the points are just a sanity check anyway)
	r->db.sensorData.localMapCentre = RobotMapProcessing_centreLocalMap (r->db.sensorData.localMapResetPt1, r->db.sensorData.localMapResetPt2, r->db.sensorData.localMapResetPt2);

	RobotMapProcessing_setLocalMapOrigin (&r->db); 
}

extern float grobotsLocs[MAX_N_ROBOTS * 3];
#ifdef IS_GUMSTIX
extern float grabCompassReading (RobotDatabase *db);
#endif

#if defined(SIMULATION) && defined(RAND_ROBOT_LOCS)
void generateRandomLoc (RobotDatabase *db, BoardDatabase *bdb, const int index)
{
	const int border = 40;
	int j, isConflict;
	PointI pt;	
	PointI bl = {border, border};
	PointI area = {ENVIR_DIMS - border, ENVIR_DIMS - border};
	area.x = area.x - bl.x;
	area.y = area.y - bl.y;

	do
	{
		pt.x = (rand() % area.x) + bl.x;
		pt.y = (rand() % area.y) + bl.y;
		isConflict = 0;

		for (j = 0; j < index; ++j)
		{
			if (Geometry_distSqrd (
				pt.x,
				pt.y,
				bdb->groupData.robots[j].pose.loc.x,
				bdb->groupData.robots[j].pose.loc.y) < 100.0f)
			{
				isConflict = 1;
				break;
			}
		}

		if (!isConflict)
		{
			db->status.pose.loc = PointI_toPointF (pt);
			db->status.pose.orient = 0.0f;

			// Update own pose on Board Database, so that later robots can check for conflicts.
			bdb->groupData.robots[index].pose = db->status.pose;
		}
	}
	while (isConflict);
}
#endif // defined(SIMULATION) && defined(RAND_ROBOT_LOCS)

#if defined(RERUNNING_ROBOT)
#include "Comm/RobotCommSerialize.h"

void getExperLogToRerun (RobotDatabase *db, const char *experDirToRerun)
{
	WIN32_FIND_DATA FindFileData;
	HANDLE hFind;
	char temp[256];

//#define TEST_EXPERIMENT_LOG
#ifdef TEST_EXPERIMENT_LOG
	int nRead;
	int atEnd;
	int lenToKeep, lenToRead, currentBufLen;
	char *iterStart, *iterEnd;
#endif // ifdef TEST_EXPERIMENT_LOG

	strcpy (temp, experDirToRerun);
	strcat (temp, "/*.txt");
	hFind = FindFirstFileEx(temp, FindExInfoStandard, &FindFileData, FindExSearchNameMatch, NULL, 0);
	if (hFind == INVALID_HANDLE_VALUE) 
	{
		printf ("FindFirstFileEx failed (%d)\n", GetLastError());
		assert (0);
	}

	printf ("The first file found is %s\n", FindFileData.cFileName);
	strcpy (temp, experDirToRerun);
	strcat (temp, "/");
	strcat (temp, FindFileData.cFileName);
	db->status.rerunning_file = fopen (temp, "r");
	if (!db->status.rerunning_file)
	{
		printf ("ERROR: could not open %s\n", temp);
		FindClose(hFind);
		assert (0);
	}

	db->status.rerunning_buffer = (char*)malloc (65536);

#ifdef TEST_EXPERIMENT_LOG
	getNextRobotIter_serialize (db);
	atEnd = 0;
	lenToKeep = 0;
	lenToRead = 65536;
	nRead = fread (db->status.rerunning_buffer, 1, lenToRead, db->status.rerunning_file);
	currentBufLen = lenToKeep + nRead;
	atEnd = (nRead != lenToRead);
	printf ("first read: lenToRead %d nRead %d atEnd %d currentBufLen %d\n", lenToRead, nRead, atEnd, currentBufLen);

	iterEnd = db->status.rerunning_buffer;
	while (1)
	{
		// findIterationBoundsInLog
		{
			iterStart = iterEnd;
			iterEnd = strstr (iterEnd + 1, "<RobotStatus>");
			if (iterEnd)
			{
#ifdef PRINT_DEBUG
				printf ("start %d end %d\n", iterStart - db->status.rerunning_buffer, iterEnd - db->status.rerunning_buffer);
#endif
			}
			else
			{
				if (iterStart == db->status.rerunning_buffer)
				{
					assert (0);
				}
				if (atEnd)
				{
#ifdef PRINT_DEBUG
					printf ("Last iteration of recorded file reached\n");
#endif
					break;
				}
				else
				{
					lenToRead = (iterStart - db->status.rerunning_buffer);
					lenToKeep = 65536 - lenToRead;
					strcpy (db->status.rerunning_buffer, iterStart);
					nRead = fread (db->status.rerunning_buffer + lenToKeep, 1, lenToRead, db->status.rerunning_file);
					currentBufLen = lenToKeep + nRead;
					iterEnd = db->status.rerunning_buffer;
					atEnd = (nRead != lenToRead);
					printf ("lenToRead %d nRead %d atEnd %d currentBufLen %d\n", lenToRead, nRead, atEnd, currentBufLen);
				}
			}
		}
	}
#undef TEST_EXPERIMENT_LOG
#endif // ifdef TEST_EXPERIMENT_LOG

	FindClose(hFind);
}
#endif // defined(RERUNNING_ROBOT)

Robot Robot_init (
#ifdef SIMULATION
				  BoardDatabase *b,
#endif
#if defined (SIMULATION) || defined (RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
				  IplImage *localMapIplImage,
#endif
				  const int index)
{
	Robot r;
#ifdef IS_GUMSTIX
	float reading;
#endif

#if defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
	CvSize szLocal = {LOC_MAP_DIMS, LOC_MAP_DIMS};
#endif // defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)

#if defined(SIMULATION)
	// Setup camera parameters, global variables, if on windows.
	// The platform is just setup statically in RobotDefs.h, but we can override
	// this to at runtime to test different platforms, or when training
	// cooperative localisation
	setupCamParams (ROBOT_PLATFORM);
#endif

	r.db = initRobotDatabase (
#ifdef SIMULATION
		b,
#endif
#if defined (SIMULATION) || defined (RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
		localMapIplImage,
#endif
		index);

#if defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
	r.db.localMapIplImage = cvCreateImage (szLocal, 8, 1);
	r.db.localMapIplImage->origin = 1;

	szLocal.width = 176;
	szLocal.height = 143;
	r.db.cameraIplImage = cvCreateImage (szLocal, 8, 3);
	r.db.cameraIplImage->origin = 1;
#endif // defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)

#if defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
//	strcpy (r.db.status.rerunning_experDirToRerun, "Files/20130427_gumstixFiles/gum2/experiment_robot_0_440-19691231-Wed-160720");
//	strcpy (r.db.status.rerunning_experDirToRerun, "Files/experiment_robot_0_1367139768-20130428-Sun-100248");
//	strcpy (r.db.status.rerunning_experDirToRerun, "Files/20130428_gumstixFiles_0/gum2/experiment_robot_0_142-19691231-Wed-160222");
//	strcpy (r.db.status.rerunning_experDirToRerun, "Files/20130428_gumstixFiles_1__okButShortExperiment/gum3/experiment_robot_0_255-19691231-Wed-160415");
//	strcpy (r.db.status.rerunning_experDirToRerun, "Files/20130428_gumstixFiles_3__okBut2ndOneCrashes/gum3/experiment_robot_0_92-19691231-Wed-160132"); // ok - 10 iters - missed some obstacles
//	strcpy (r.db.status.rerunning_experDirToRerun, "Files/20130428_gumstixFiles_3__okBut2ndOneCrashes/gum3/experiment_robot_0_309-19691231-Wed-160509"); // crashed!!!

	// From 2013/05/16
//	strcpy (r.db.status.rerunning_experDirToRerun, "Files/20130519_gumstixFiles_2__indepAndCollabExpers/gum2/experiment_robot_0_1709-19691231-Wed-162829_independent");
	strcpy (r.db.status.rerunning_experDirToRerun, "Files/20130519_gumstixFiles_2__indepAndCollabExpers/gum2/experiment_robot_0_2093-19691231-Wed-163453_collab");
//	strcpy (r.db.status.rerunning_experDirToRerun, "Files/20130519_gumstixFiles_2__indepAndCollabExpers/gum2/experiment_robot_0_2769-19691231-Wed-164609_independent");

#if defined(RERUNNING_ROBOT)
	getExperLogToRerun (&r.db, r.db.status.rerunning_experDirToRerun);
	getNextRobotIter_serialize (&r.db);
#endif
#endif // defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)





#ifdef RERUNNING_ROBOT
	// Need to read this now s.th. local maps etc. can be setup. Also
	// want to keep everything logically as similar as possible.
	readInitialRobotLoc_serialize (&r.db);

#else // RERUNNING_ROBOT

#ifdef IS_GUMSTIX
	// grabCompassReading takes the robot's compass offset into account
//	r.db.status.pose.orient = grabCompassReading(&r.db);
//	r.db.status.pose.orient = Geometry_orientSum (r.db.status.pose.orient, COMPASS_ORIENT_OFFSET);

	r.db.status.pose.loc.x = 350.0f;
	r.db.status.pose.loc.y = 200.0f;
	r.db.status.pose.orient = 0.0f;

	reading = grabCompassReading(&r.db);
	r.db.sensorData.compassOffset = Geometry_orientDiff (
		reading,
		r.db.status.pose.orient,
		0); // Get what adjustment should be made to future readings

#ifdef PRINT_EVENTS
	fprintf (r.db.xmlLog, "<InitialRobotCompassReading>reading=%f orient=%f offset=%f</InitialRobotCompassReading>\n",
		reading,
		r.db.status.pose.orient,
		r.db.sensorData.compassOffset);
#endif

#else // IS_GUMSTIX

#if defined(SIMULATION) && defined(RAND_ROBOT_LOCS)
	generateRandomLoc (&r.db, b, index);

#else

	// Either robot (on pc, not gumstix) or not using random locs
	r.db.status.pose.loc.x = grobotsLocs[index * 3 + 0];
	r.db.status.pose.loc.y = grobotsLocs[index * 3 + 1];
//	r.db.status.pose.orient = grobotsLocs[index * 3 + 2];
	r.db.status.pose.orient = 0.0f;

#endif // defined(SIMULATION) && defined(RAND_ROBOT_LOCS)
#endif // IS_GUMSTIX

//#ifdef RECORDING_ROBOT
#if 1
	fprintf (r.db.xmlLog, "<InitialRobotLoc>loc=(%14.12f,%14.12f,%14.12f)</InitialRobotLoc>\n", r.db.status.pose.loc.x, r.db.status.pose.loc.y, r.db.status.pose.orient);
#endif // RECORDING_ROBOT
#endif // RERUNNING_ROBOT

	setupRobotsLocalMap (&r);

	return r;
}

void clearRobot (Robot *r)
{
#if defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
	cvReleaseImage (&r->db.localMapIplImage);
	cvReleaseImage (&r->db.cameraIplImage);
#endif

	RobotDatabase_dtor (&r->db);
}

#define VERBOSE_NAV_GRIDS 0
#ifdef VERBOSE_NAV_GRIDS
void Robot_printNavGrid (FILE *f, uchar *obstructedCellGrid)
{



}
#endif

extern int robotMapWindowSize[4];
extern int robotNavMapWindowSize[4];

void RobotMapProcessing_updateUnreachableExpCellGrid (
	RobotDatabase *db,
	Image *navMap,
	uchar unreachableExpCellGrid[((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1]);

#ifdef RERUNNING_ROBOT
#include "../Board/Data/BoardDatabase.h"
#endif

void Robot_updateState (Robot *r, const int updateVisualisation)
{
	int haveToUpdateOtherRobots;
//	int isUnreachableLocalMapGridUpdated;
#ifdef SHOW_ROBOT_IMGS
	int windowSize[4];
#endif

	if (r->db.behaviourData.behaviour == STUCK)
	{
		return;
	}

	TIMER_START ("updateState")

	SensorProcessing_sanityCheck (&r->db, "before");
	SensorProcessing_processSensorData (&r->db, 0);

#ifdef COLLAB_EXP
	{
		CloseLoop_processData (&r->db);
	}
#endif

	RobotMapProcessing_determineIsLocalMapReady (&r->db);

	if (r->db.sensorData.isMapDataReady &&
		r->db.sensorData.mapScanList.size)
	{
		TIMER_START ("prepareLocalMap")
		RobotMapProcessing_prepareLocalMap (&r->db);
		TIMER_STOP (r->db.xmlLog, "prepareLocalMap")
	}

	if (r->db.sensorData.isLocalMapResetRequired != DONT_RESET_LOC_MAP)
	{
		// In some instances, e.g. when the robot is just starting off, or when
		// it has just submitted its map, then the map will have to be re-centred
		RobotMapProcessing_resetLocalMap (&r->db);
	}

	RobotRead_readBoard (&r->db);
	RobotWrite_updateBoard (&r->db);

	if (!r->db.sensorData.isDataPendingInc)
	{
		RobotMapIntegration_integrateNewMapDataFromBoard (&r->db);

		RobotMapIntegration_integrateMapData (&r->db);

		haveToUpdateOtherRobots = RobotMapProcessing_checkForRobotsInLocalMap (&r->db);

		if (RobotMapProcessing_intAssumedNeighbourScans (&r->db))
		{
			r->db.sensorData.isSearchGridUpdateRequired = UPDATE_WHOLE_NAV_MAP;
		}

		if (DONT_UPDATE_SEARCH_GRID != r->db.sensorData.isSearchGridUpdateRequired)
		{
			RobotMapGridProcessing_updateGrids (&r->db);
		}

		if (r->db.sensorData.isNavMapUpdateRequired != DONT_UPDATE_NAV_MAP ||
			haveToUpdateOtherRobots)
		{
			fprintf (r->db.xmlLog, "<UpdateNavMap>isNavMapUpdateRequired=%d haveToUpdateOtherRobots=%d</UpdateNavMap>\n",
				r->db.sensorData.isNavMapUpdateRequired,
				haveToUpdateOtherRobots);

			RobotMapProcessing_updateNavMap (&r->db);

			if (haveToUpdateOtherRobots)
			{
				RobotMapProcessing_incRobotsIntoNavMap (&r->db);
			}

			RobotMapProcessing_updateWholeObstructedGrid (&r->db, r->db.environment.navMap);

			RobotMapProcessing_updateUnreachableExpCellGrid (
				&r->db,
				r->db.environment.navMap,
				r->db.environment.unreachableExpCellGrid);

			r->db.behaviourData.checkPaths = 1;
		}
	}

#ifndef IS_GUMSTIX
#if 0
	RobotMapProcessing_printNavMap (&r->db);
#endif
#endif

//	SensorProcessing_sanityCheck (&r->db, "after");

#ifdef SHOW_ROBOT_IMGS
	if (updateVisualisation)
	{
		memcpy (windowSize, robotMapWindowSize, sizeof (int) * 4);
		windowSize[0] += r->db.status.index * 100;
		Image_show_givenIplImage (
			r->db.sensorData.localMap,
			r->db.status.locWinName,
			r->db.localMapIplImage,
			windowSize);

		memcpy (windowSize, robotNavMapWindowSize, sizeof (int) * 4);
		windowSize[0] += r->db.status.index * 100;
		RobotMapProcessing_displayNavMap (
			r->db.environment.navMap,
			r->db.status.pose.loc,
			r->db.status.navWinName,
			windowSize,
			r->db.localMapIplImage);
	}

#ifndef IS_GUMSTIX
#if 0
	RobotMapProcessing_printNavMap (&r->db);
#endif
#endif

#ifdef RERUNNING_ROBOT
//	cvWaitKey(0);
	cvWaitKey(1);
#endif
#endif

#if VERBOSE_NAV_GRIDS
	Robot_printNavGrid (r->db.xmlLog, r->db.environment.obstructedCellGrid);
#endif

	TIMER_STOP (r->db.xmlLog, "updateState")
}


void Robot_printSupCoals (RobotDatabase *db)
{
	ListNode *iter;
	Coalition *coal;

	fprintf (db->xmlLog, " explorationCoalitionId=%d", db->partnerData.explorationCoalition.id);
	fprintf (db->xmlLog, " supervisionCoalitionIds=(");
	iter = db->partnerData.supervisionCoalitions.front;
	while (iter)
	{
		coal = (Coalition*)iter->value;
		fprintf (db->xmlLog, "%d,", coal->id);
		iter = iter->next;
	}
	fprintf (db->xmlLog, ")");
}

extern const char* behaviourHandles[];

void Robot_printStatus (Robot *r)
{
	Dest *dest;
	BasicBehaviourData *data;
	PointI pt;

	fprintf (r->db.xmlLog, "\n<RobotStatus>iteration=%d behaviour=\"%s\" pose=(%f,%f,%f) cov=(%f,%f,%f,%f) evals=(%f,%f) evec=(%f,%f) stdDev=%f localMapIndex=%d localMapOrigin=(%d,%d) nMapScans=%d",
		r->db.status.nIterations, behaviourHandles[r->db.behaviourData.behaviour],
		r->db.status.pose.loc.x,r->db.status.pose.loc.y, r->db.status.pose.orient,
		r->db.status.pose.mat[0], r->db.status.pose.mat[1], r->db.status.pose.mat[2], r->db.status.pose.mat[3],
		r->db.status.pose.evals[0], r->db.status.pose.evals[1],
		r->db.status.pose.evec.x, r->db.status.pose.evec.y,
		r->db.status.stdDev,
		r->db.sensorData.localMapDataIndex,
		r->db.sensorData.localMap->orig.x,
		r->db.sensorData.localMap->orig.y,
		r->db.sensorData.mapScanList.size);

	dest = getBehaviourDest (&r->db, r->db.behaviourData.behaviour);
	if (dest)
	{
		fprintf (r->db.xmlLog, " dest=(type=%d ", dest->type);
		if (dest->type & IS_TARGET_CENTRIC)
		{
			fprintf (r->db.xmlLog, "tar=(%f,%f))", dest->target.x, dest->target.y);
		}
		else
		{
			fprintf (r->db.xmlLog, "dest=(%f,%f,%f))", dest->dest.loc.x, dest->dest.loc.y, dest->dest.orient);
		}
	}
	if (FOLLOW_PATH == r->db.behaviourData.behaviour)
	{
		data = (BasicBehaviourData*)r->db.behaviourData.stack.back->prev->value;
		if (data)
		{
			dest = getBehaviourDest (&r->db, data->id);
			if (dest)
			{
				fprintf (r->db.xmlLog, " prevDest=(type=%d ", dest->type);
				if (dest->type & IS_TARGET_CENTRIC)
				{
					fprintf (r->db.xmlLog, "tar=(%f,%f))", dest->target.x, dest->target.y);
				}
				else
				{
					fprintf (r->db.xmlLog, "dest=(%f,%f,%f))", dest->dest.loc.x, dest->dest.loc.y, dest->dest.orient);
				}
			}
		}
	}
	if (r->db.partnerData.explorationCoalition.id != -1)
	{
//		pt = Supervision_supAreaIdFromCentreI (r->db.partnerData.explorationCoalition.pt);
		fprintf (r->db.xmlLog, " supAreaId=(%d,%d)", r->db.partnerData.explorationCoalition.area.x, r->db.partnerData.explorationCoalition.area.y);
	}
	else
	{
		pt = Supervision_getMostApplicableSupArea (r->db.status.pose.loc);
		fprintf (r->db.xmlLog, " supAreaId=(%d,%d)", pt.x, pt.y);
	}

#ifdef SIM_ERROR
	fprintf (r->db.xmlLog, " actLocOff=(%14.12f,%14.12f)", r->db.status.actualLocOffset.x, r->db.status.actualLocOffset.y);
#endif

	fprintf (r->db.xmlLog, "</RobotStatus>\n");

	fprintf (r->db.xmlLog, "<CoalitionStatus>supervisor=%d gotoExpPtSessionIndex=%d",
		r->db.partnerData.explorationCoalition.supervisor,
		r->db.behaviourData.gotoExpPt.sessionIndex);
	Robot_printSupCoals (&r->db);
	fprintf (r->db.xmlLog, "</CoalitionStatus>\n");

	BehaviourCore_printStackToLog (r->db.xmlLog, &r->db.behaviourData.stack);

	fflush (r->db.xmlLog);
}

void Robot_finalUpdate (Robot *r)
{
	SensorProcessing_processSensorData (&r->db, 0);

#ifdef PRINT_EVENTS
	fprintf (r->db.xmlLog, "<IsLocalMapReady>reason=\"finishedExperiment\"</IsLocalMapReady>\n");
#endif

	RobotMapProcessing_prepareLocalMap (&r->db);

	r->db.behaviourData.closeLoop.isMoreMapDataComing = 0;

	RobotWrite_writeLocalMap (&r->db);
}

//! Alert board that the robot is finished and print status to log
void Robot_finish (Robot *r)
{
	// alert the board that the robot thread is about to exit
	RobotWrite_writeFinished (&r->db);

#ifdef PRINT_EVENTS
	fprintf (r->db.xmlLog, "<RobotFinish />\n");

	Robot_printStatus (r);
#endif
}
























#ifdef SETUP_TEST_COALITION
void Robot_setupTestCoalition (RobotDatabase *db)
{
	db->behaviourData.behaviour = AVAILABLE;
	db->partnerData.explorationCoalition.id = 0;
	db->partnerData.explorationCoalition.bid = 0.1f;
	db->partnerData.explorationCoalition.pt.x = 252.0f;
	db->partnerData.explorationCoalition.pt.y = 126.0f;
	db->partnerData.explorationCoalition.stdDev = 0.0f;
	db->partnerData.explorationCoalition.explorer = 0;
	db->partnerData.explorationCoalition.supervisor = 1;
	db->partnerData.explorationCoalition.collabData.isSupAtFinalDest = 1;

	db->groupData.robots[1].actualLocOffset = initPointF (0.0f);
	db->groupData.robots[1].baseBehaviour = SUPERVISION;
	db->groupData.robots[1].behaviour = SUPERVISION;
	db->groupData.robots[1].localMapOrigin = initPointI (0, 0);
//	db->groupData.robots[1].pose.loc.x = grobotsLocs[1 * 3 + 0]; // Same as in Robot_init above
//	db->groupData.robots[1].pose.loc.y = grobotsLocs[1 * 3 + 1];
	db->groupData.robots[1].pose.loc.x = 250.0f;
	db->groupData.robots[1].pose.loc.y = 200.0f;
	db->groupData.robots[1].pose.orient = 0.0f;
	db->groupData.robots[1].pose.mat[0] = 0.0f;
	db->groupData.robots[1].pose.mat[1] = 0.0f;
	db->groupData.robots[1].pose.mat[2] = 0.0f;
	db->groupData.robots[1].pose.mat[3] = 0.0f;
	db->groupData.robots[1].target.x = 252.0f;
	db->groupData.robots[1].target.y = 126.0f;
	db->groupData.robots[1].stdDev = 0.0f;
	db->groupData.robots[1].localMapOrigin.x = 210;
	db->groupData.robots[1].localMapOrigin.y = 168;
}
#endif

#ifdef ROBOT
extern int g_forceQuit;
extern int g_nIters;

#if !defined(RERUNNING_ROBOT)
#if defined(IS_GUMSTIX)
#define PRINT_TICK if (i % 10 == 0){ printf ("Iteration %d\n", i); } fflush (stdout);
#define PRINT_END_TICK
#else
#define PRINT_TICK if (i % 10){ printf("."); }else{ printf ("%d", i); } fflush (stdout);
#define PRINT_END_TICK printf ("\n");
#endif
#else
#define PRINT_TICK
#define PRINT_END_TICK
#endif

#ifdef IS_WIN
extern void RobotMapProcessing_displayNavMap (Image *navMap,
									 const PointF loc,
									 const schar *winName,
									 int windowSize[4],
									 IplImage *localMapIplImage);
void foo(RobotDatabase *db)
{
	int windowSize[4] = {200,200,120,120};
	cvNamedWindow ("crap", windowSize);
	RobotMapProcessing_displayNavMap (
		db->environment.navMap,
		db->status.pose.loc,
		"crap",
		windowSize,
		db->localMapIplImage);
}
void printNavMap (FILE *f, Image *img)
{
	int i, j;
	for (i = 0; i < 84; ++i)
	{
		for (j = 0; j < 84; ++j)
		{
			fprintf (f, "%3d.", Image_getPixel_dontCheck (img, i, j));
		}
		fprintf (f, "\n");
	}
}
#endif

#define DEBUGGING_NAV_MAPS

void Robot_start()
{
	Robot r;
	RobotDatabase *db;
#ifdef SETUP_TEST_COALITION
	int coalSetup = 0;
#endif
#ifdef IS_WIN
	CvSize szLocal = {LOC_MAP_DIMS, LOC_MAP_DIMS};
	IplImage *localMapIplImage = cvCreateImage (szLocal, 8, 1);
#endif
#ifdef AUTONOMOUS
	int i = 0, nIters = g_nIters;
#if !defined(IS_GUMSTIX)
//	nIters = 20;
	nIters = 100;
#endif
#endif

	setupExperimentDir();

#ifdef IS_GUMSTIX
	if (0 == bCMU_Init ())
	{
		printf ("Error bCMU_Init\n");
		return;
	}

	bCMU_calibWhiteBalance();
#endif

	// Set before setting up database
#ifdef SETUP_TEST_COALITION
	SET_N_ROBOTS(2)
#else
	SET_N_ROBOTS(1)
#endif

#ifdef IS_WIN
	localMapIplImage->origin = 1;
	r = Robot_init (localMapIplImage, ROBOT_INDEX);
#else
	r = Robot_init (ROBOT_INDEX);
#endif
	db = &r.db;

#ifdef JUST_TEST_BT
	// Turn this on to test bluetooth connection and exit straight away.
	if (-1 == RobotReadImpl_setupSocket (&db->commData))
	{
		goto cleanupRobot;
	}
	Robot_finish (&r);

#else // JUST_TEST_BT

	if (-1 == RobotReadImpl_setupSocket (&db->commData))
	{
		goto cleanupRobot;
	}

	printf ("Initial sync with board.\n");

	RobotWrite_writeRobotData (db);
	RobotWrite_writeExperimentPath (db); // Allows board and robot log files to be paired when analyzing after the experiment.
	SLEEP_FOR_MS(10)
	RobotRead_readBoard (db);

	printf ("Starting robot main control loop.\n");
	//if (&r != NULL)
	//{
	//	goto cleanupRobot;
	//}
	do
	{
		if (i == 17)
		{
//			printf ("break\n");
		}
		WIPE_TIMERS

#ifdef SETUP_TEST_COALITION
		if (!coalSetup)
		{
			Robot_setupTestCoalition (db);
			coalSetup = 1;
		}
#endif

		TIMER_START ("fullLoop")

//		printf ("Iteration %d\n", i);

#ifdef RERUNNING_ROBOT
		getNextRobotIter_serialize (db);
#endif // RERUNNING_ROBOT

#ifdef PRINT_EVENTS
		printf ("%d\n", i);
//		fprintf (db->xmlLog, "%d\n", i);
		Robot_printStatus (&r);
#endif
		Robot_updateState (&r, 1);

#ifdef IS_WIN
#ifdef DEBUGGING_NAV_MAPS
		if (i > 50)
		{
//			printNavMap (db->xmlLog, db->environment.navMap);
//			foo (db);
//			cvWaitKey (1);
//			fprintf (db->xmlLog, "val=%d\n", Image_getPixel_dontCheck (db->environment.navMap, 51, 64));
//			Image_getPixel_dontCheck (db->environment.navMap, 51, 64);
		}
#endif
#endif

#ifdef AUTONOMOUS
		BehaviourControl_checkNavigationState (db);
		BehaviourControl_highLevelBehaviour (db);

#ifdef COLLAB_EXP
#ifdef SETUP_TEST_COALITION
		BehaviourControl_setCollaborativeBehaviour (db);
#else // SETUP_TEST_COALITION
		Supervision_makeProposal (db);
		Supervision_considerProposals (db);
		BehaviourControl_setCollaborativeBehaviour (db);
#endif // SETUP_TEST_COALITION
#endif // COLLAB_EXP

		BehaviourControl_lowLevelBehaviour (db);
		BehaviourControl_implementBehaviour (db);

#else // AUTONOMOUS

		BehaviourCore_manualBehaviour (db);
#endif // AUTONOMOUS

		BehaviourControl_processBehaviour (db);

		TIMER_STOP (db->xmlLog, "fullLoop")

#ifndef IS_GUMSTIX // Don't need to sleep if we're actually doing stuff with hardware
		SLEEP_FOR_MS(200)
#endif // ifndef IS_GUMSTIX

		PRINT_TICK
	}
#ifdef AUTONOMOUS
	while (++i <= nIters && !g_forceQuit);
	//while (!db->status.isTimeElapsed && !g_forceQuit);

#else // AUTONOMOUS

	while (!g_forceQuit);
#endif // AUTONOMOUS

#ifdef PRINT_EVENTS
	Robot_printStatus (&r);
#endif
	Robot_finalUpdate (&r);
	SLEEP_FOR_MS(100)
	Robot_finish (&r);

	PRINT_END_TICK
#endif // JUST_TEST_BT

	printf ("Finished robot main control loop.\n");

	RobotReadImpl_closeSocket (&db->commData);

	printf ("Robot socket closed.\n");

#ifdef IS_GUMSTIX
	bCMU_Close();
#endif

cleanupRobot:
	clearRobot (&r);

#ifdef IS_WIN
	cvReleaseImage (&localMapIplImage);
	cvCleanup();
#endif
#ifdef RERUNNING_ROBOT
	cvCleanup();
#endif
}
#endif // ifdef ROBOT


#endif // ifndef BOARD


