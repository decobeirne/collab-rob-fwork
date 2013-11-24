#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "CloseLoop.h"
#include "BehaviourCore.h"
#include "../Comm/RobotWrite.h"
#include "../Comm/RobotRead.h"
#include "../Sensors/MapIntegration.h"
#include "../Sensors/MapProcessing.h"

#if 0
void CloseLoop_adjustMapData (RobotDatabase *db, const PointF adjustment)
{
	ListNode *iter;
	MapScan *mapScan;
	PointF adjPerMove;
	PointF adj;
	float covariance[4];
	float covEstimate[4];
	float evals[2];
	PointF evec;
	float stdDev;
	int nMapScansAdjusted = 0;
	float sumStdDevRed = 0.0f;
	int nMoves = db->sensorData.mapScanList.size - 1;

	// Robot's own location-estimate-covariance should just have been updated (i.e., pivotal step of loop-closing).
	memcpy (covariance, db->status.pose.mat, sizeof (float) * 4);
	stdDev = db->status.stdDev;

	adjPerMove.x = adjustment.x / nMoves;
	adjPerMove.y = adjustment.y / nMoves;
	adj = adjustment;

	iter = db->sensorData.mapScanList.back;
	while (iter && iter != db->sensorData.mapScanList.front)
	{
		mapScan = (MapScan*)iter->value;

		if (stdDev < mapScan->stdDev)
		{
			sumStdDevRed += mapScan->stdDev - stdDev;
			++nMapScansAdjusted;
		}

		mapScan->pose.loc.x += adj.x;
		mapScan->pose.loc.y += adj.y;

		covEstimate[0] = db->uncertaintyConstants.closeLoop_varFwd;
		covEstimate[1] = db->uncertaintyConstants.closeLoop_covar;
		covEstimate[2] = covEstimate[1];
		covEstimate[3] = db->uncertaintyConstants.closeLoop_varLat;

		Uncertainty_updateCovariance (covEstimate, PI / 4.0f, covariance);
		Uncertainty_calcEigenvalues (covariance, evals, &evec, &stdDev);

		adj.x -= adjPerMove.x;
		adj.y -= adjPerMove.y;
		iter = iter->prev;
	}

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<CloseLoop>nMapScansTotal=%d nMapScansAdjusted=%d sumMapScanStdDevRed=%f</CloseLoop>\n",
		db->sensorData.mapScanList.size, nMapScansAdjusted, sumStdDevRed);
#endif
}
#endif

/*!
From closeLoopSessions.csv
*/
float CloseLoop_estNSteps (const float proportionSupAreaMapped)
{
	return (5.0f * proportionSupAreaMapped + 10.0f);
}

float CloseLoop_estProfit (const float nScans, const float supStdDev)
{
	float nAdjusted = nScans * 0.75f;
	float estdGainPerScan = max (0.0f, (-30.0f * (supStdDev / STD_DEV_MAX) + 40.0f));
	return (nAdjusted * estdGainPerScan);
}






void CloseLoop_adjustMapDataNEW (RobotDatabase *db, const PointF adjustment)
{
	ListNode *iter;
	MapScan *mapScan;
	PointF adjPerMove;
	PointF adj;
	float covariance[4];
	float covEstimate[4];
	float evals[2];
	PointF evec;
	float stdDev;
	int nMapScansAdjusted = 0;
	float sumStdDevRed = 0.0f;
	int nMoves = db->sensorData.mapScanList.size - 1;
#ifdef PRINT_PROFIT_DETAIL
	int scanIndex;
#ifdef SIM_ERROR
	PointF pt1, pt2;
#endif
#endif

	// Robot's own location-estimate-covariance should just have been updated (i.e., pivotal step of loop-closing).
	memcpy (covariance, db->status.pose.mat, sizeof (float) * 4);
	stdDev = db->status.stdDev;

	adjPerMove.x = adjustment.x / nMoves;
	adjPerMove.y = adjustment.y / nMoves;
	adj = adjustment;

#ifdef PRINT_PROFIT_DETAIL
	fprintf (db->xmlLog, "<CloseLoopAdjustMapData>\n");
	fprintf (db->xmlLog, "<AdjustMapScansPre>adjustment=(%f,%f) stdDev=%f nMapScans=%d</AdjustMapScansPre>\n", adj.x, adj.y, stdDev, db->sensorData.mapScanList.size);
	scanIndex = db->sensorData.mapScanList.size - 1;
#endif

	iter = db->sensorData.mapScanList.back;
	while (iter && iter != db->sensorData.mapScanList.front)
	{
		mapScan = (MapScan*)iter->value;

#ifdef SIM_ERROR
		pt1.x = mapScan->pose.loc.x - mapScan->actualPoseLoc.x;
		pt1.y = mapScan->pose.loc.y - mapScan->actualPoseLoc.y;
#endif

		if (stdDev >= mapScan->stdDev)
		{
#ifdef PRINT_PROFIT_DETAIL
			fprintf (db->xmlLog, "<AdjustScan>scan=%d event=\"dontAdjust\" origStdDev=%f adjStdDev=%f origLoc=(%f,%f)",
				scanIndex,
				mapScan->stdDev,
				stdDev,
				mapScan->pose.loc.x,
				mapScan->pose.loc.y);
			--scanIndex;
#endif

#ifdef SIM_ERROR
			fprintf (db->xmlLog, " actualLoc=(%f,%f) origDiff=(%f,%f)",
				mapScan->actualPoseLoc.x,
				mapScan->actualPoseLoc.y,
				pt1.x,
				pt1.y);
#endif
			fprintf (db->xmlLog, "</AdjustScan>\n");

			iter = iter->prev;
			continue;
		}

		sumStdDevRed += mapScan->stdDev - stdDev;
		++nMapScansAdjusted;

#ifdef PRINT_PROFIT_DETAIL
		fprintf (db->xmlLog, "<AdjustScan>scan=%d event=\"adjust\" origStdDev=%f adjStdDev=%f origLoc=(%f,%f) origOrient=%f",
			scanIndex,
			mapScan->stdDev,
			stdDev,
			mapScan->pose.loc.x,
			mapScan->pose.loc.y,
			mapScan->pose.orient);
#endif

		mapScan->pose.loc.x += adj.x;
		mapScan->pose.loc.y += adj.y;
		mapScan->stdDev = stdDev;

#ifdef PRINT_PROFIT_DETAIL
		fprintf (db->xmlLog, " adjLoc=(%f,%f)", mapScan->pose.loc.x, mapScan->pose.loc.y);

#ifdef SIM_ERROR
		pt2.x = mapScan->pose.loc.x - mapScan->actualPoseLoc.x;
		pt2.y = mapScan->pose.loc.y - mapScan->actualPoseLoc.y;

		fprintf (db->xmlLog, " actualLoc=(%f,%f) origDiff=(%f,%f) adjDiff=(%f,%f)", mapScan->actualPoseLoc.x, mapScan->actualPoseLoc.y, pt1.x, pt1.y, pt2.x, pt2.y);
#endif // ifdef SIM_ERROR
#endif // ifdef PRINT_PROFIT_DETAIL

		covEstimate[0] = db->uncertaintyConstants.closeLoop_varFwd;
		covEstimate[1] = db->uncertaintyConstants.closeLoop_covar;
		covEstimate[2] = covEstimate[1];
		covEstimate[3] = db->uncertaintyConstants.closeLoop_varLat;

		Uncertainty_updateCovariance (covEstimate, PI / 4.0f, covariance);
		Uncertainty_calcEigenvalues (covariance, evals, &evec, &stdDev);

#ifdef PRINT_PROFIT_DETAIL
		fprintf (db->xmlLog, "</AdjustScan>\n");
		--scanIndex;
#endif

		adj.x -= adjPerMove.x;
		adj.y -= adjPerMove.y;
		iter = iter->prev;
	}

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<AdjustMapScansPost>nMapScansTotal=%d nMapScansAdjusted=%d sumMapScanStdDevRed=%f</AdjustMapScansPost>\n",
		db->sensorData.mapScanList.size, nMapScansAdjusted, sumStdDevRed);
	fprintf (db->xmlLog, "</CloseLoopAdjustMapData>\n");
#endif
}

extern int checkPtAgainstLocalMap (const PointI pt, const PointI centre, const int distAllowed);

//! Check that all map scans fit inside a local map.
int checkMapScanSequence (RobotDatabase *db,
						  ListNode *start,
						  ListNode *end,
						  const PointI localMapCentre)
{
	ListNode *iter;
	MapScan *mapScan;
	PointF scanCentre;
//	const float scanCentreDist = db->geometryConstants.exp_optDist + CAM_P_OFFSET;
	const float scanCentreDist = SCAN_CENTRE_DIST + CAM_P_OFFSET;
	const int distAllowed = (LOC_MAP_DIMS / 2) - (EXP_AREA / 3);

	iter = start;
	while (iter && iter != end->next)
	{
		mapScan = (MapScan*)iter->value;

		Geometry_ptFromOrient (
			mapScan->pose.loc.x,
			mapScan->pose.loc.y,
			&scanCentre.x,
			&scanCentre.y,
			scanCentreDist,
			mapScan->pose.orient);

		if (!checkPtAgainstLocalMap (PointF_toPointI (scanCentre), localMapCentre, distAllowed))
		{
			return 0;
		}

		iter = iter->next;
	}

	return 1;
}



























#define VERBOSE_SUBMIT_CLOSE_LOOP_MAPS
#if defined(VERBOSE_SUBMIT_CLOSE_LOOP_MAPS)
void CloseLoop_printScanDetails (FILE *f, ListNode *start, ListNode *end)
{
	ListNode *iter;
//	PointI *pt;
	PointF pt;
	MapScan *mapScan;

	fprintf (f, "<CloseLoopScans>");
	iter = start;
	while (iter && iter != end->next)
	{
		mapScan = (MapScan*)iter->value;

		Geometry_ptFromOrient (
			mapScan->pose.loc.x,
			mapScan->pose.loc.y,
			&pt.x,
			&pt.y,
			(SCAN_CENTRE_DIST + CAM_P_OFFSET),
			mapScan->pose.orient);

//		fprintf (f, "(%f,%f)", mapScan->pose.loc.x, mapScan->pose.loc.y);
		fprintf (f, "(%f,%f)", pt.x, pt.y);
		iter = iter->next;
	}
	fprintf (f, "</CloseLoopScans>\n");
}
#endif

extern void RobotWrite_writeLocalMap (RobotDatabase *db);
extern void printSubmittedMap (RobotDatabase *db, LocalMapInfo *localMapInfo, const int loopClosing);
extern void saveSubmittedMap (RobotDatabase *db, Image *localMap, const int localMapIndex);
extern void MapCore_compressLocalMap (Image *image, CompressedImage *compressed);

void submitCloseLoopMap (RobotDatabase *db,
						 ListNode *start,
						 ListNode *end,
						 const PointI localMapCentre,
						 const int startIndex)
{
	ListNode *iter;
	MapScan *mapScan;
	const float stdDevMaxInv = 1.0f / STD_DEV_MAX;
	float profit;
#ifdef PRINT_PROFIT
	int scanIndex;
#endif
	LocalMapInfo localMapInfo = initLocalMapInfo();

#if defined(VERBOSE_SUBMIT_CLOSE_LOOP_MAPS)
	CloseLoop_printScanDetails (db->xmlLog, start, end);
#endif // defined(VERBOSE_SUBMIT_CLOSE_LOOP_MAPS)

	Image_fill (db->sensorData.localMap, 127);
	db->sensorData.localMap->orig = localMapCentre;
	db->sensorData.localMap->orig.x -= LOC_MAP_DIMS/2;
	db->sensorData.localMap->orig.y -= LOC_MAP_DIMS/2;

#ifdef PRINT_PROFIT
	scanIndex = startIndex;
	fprintf (db->xmlLog, "<RedrawCloseLoopMap>\n");
#endif
//	iter = db->sensorData.mapScanList.front;
//	while (iter != 0)
	iter = start;
	while (iter && iter != end->next)
	{
		mapScan = (MapScan*)iter->value;
		if (mapScan->stdDev >= STD_DEV_MAX)
		{
			iter = iter->next;
			++scanIndex;
			continue;
		}

		CamVectors_rotateToRobotOrient (&db->camVectors, mapScan->pose.orient);

#if defined(MAP_SCANS_HAVE_GRIDS)
#if defined(IGNORE_CAM_DATA)
		MapIntegration_intBlankScan (
			db,
			mapScan,
			mapScan->stdDev,
			db->sensorData.localMap,
			db->environment.navMap,
			&db->camVectors,
			0,
			0);

#else // defined(IGNORE_CAM_DATA)

		if (mapScan->grid)
		{
			MapIntegration_intCamScan (
				db,
				mapScan,
				mapScan->stdDev,
				db->sensorData.localMap,
				db->environment.navMap,
				&db->camVectors,
				0,
				0);
		}
		else
		{
			MapIntegration_intBlankScan (
				db,
				mapScan,
				mapScan->stdDev,
				db->sensorData.localMap,
				db->environment.navMap,
				&db->camVectors,
				0,
				0);
		}
#endif // defined(IGNORE_CAM_DATA)

#else // defined(MAP_SCANS_HAVE_GRIDS)

		MapIntegration_intSimScan (
			db,
			mapScan,
			mapScan->stdDev,
			db->sensorData.localMap,
			db->environment.navMap,
#ifdef SIMULATION
			db->board->environment.tempMap,
#else
			db->environment.originalSim, // have to give robot copy or simulated map if on seperate processes
#endif
			0);
#endif // defined(MAP_SCANS_HAVE_GRIDS)

		profit = mapScan->estdGain * (1.0f - min (1.0f, mapScan->stdDev * stdDevMaxInv));
		localMapInfo.gross += profit;
		localMapInfo.mapGain += mapScan->estdGain;
		localMapInfo.avgStdDev += mapScan->stdDev;
		++localMapInfo.nScans;

#ifdef PRINT_PROFIT
		fprintf (db->xmlLog, "<PerScan>scan=%d profit=%f gain=%d</PerScan>\n", scanIndex, profit, mapScan->estdGain);
#endif

		++scanIndex;
		iter = iter->next;
	}

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "</RedrawCloseLoopMap>\n");
#endif

	if (localMapInfo.nScans != 0)
	{
		localMapInfo.avgStdDev /= localMapInfo.nScans;
	}

	MapCore_compressLocalMap (db->sensorData.localMap, &db->environment.mapToSubmit);

	localMapInfo.localMapIndex = db->sensorData.localMapDataIndex;
	localMapInfo.robotIndex = db->status.index;
	++db->sensorData.localMapDataIndex;

	localMapInfo.isProvisional = ADJUSTED_CLOSE_LOOP_DATA;
	localMapInfo.closeLoopSession = db->behaviourData.closeLoop.currentCloseLoopSession;

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<IsLocalMapReady>reason=\"loopClosing\"</IsLocalMapReady>\n");
#endif

#ifdef RECORDING_LOCAL_MAPS
	saveSubmittedMap (db, db->sensorData.localMap, localMapInfo.localMapIndex);
#endif

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<SubmittingCloseLoopMap />\n");

	// Must print map data to XML so profit can be calculated in post-processing.
	printSubmittedMap (db, &localMapInfo, 1);
#endif

	db->environment.mapInfoToSubmit = localMapInfo;

	db->sensorData.isMapDataReady = 1;
	RobotWrite_writeLocalMap (db);
}

//extern PointI RobotMapProcessing_centreLocalMap (const PointF pt);

void submitCloseLoopMaps (RobotDatabase *db)
{
	ListNode *iter;
	MapScan *mapScan;
	ListNode *start = 0;
	ListNode *end = 0;
	int sequenceNumScans = 0;
	int isSequenceOK;
	PointF scanCentre;
	PointF sequenceCog;
	PointF sequenceTempCog;
	PointF pt;
	PointI sequenceLocalMapCentre;
	PointI sequenceLocalMapTempCentre;
	float temp;
	int iterIndex;
	int startIndex;
//	PointF sequenceBl, sequenceTr, sequenceTempBl, sequenceTempTr;
//	const float scanCentreDist = db->geometryConstants.exp_optDist + CAM_P_OFFSET;
	const float scanCentreDist = SCAN_CENTRE_DIST + CAM_P_OFFSET;

	iterIndex = 0;
	startIndex = 0;
	iter = db->sensorData.mapScanList.front;
	while (iter)
	{
		mapScan = (MapScan*)iter->value;

		Geometry_ptFromOrient (
			mapScan->pose.loc.x,
			mapScan->pose.loc.y,
			&scanCentre.x,
			&scanCentre.y,
			scanCentreDist,
			mapScan->pose.orient);

		if (!start)
		{
			startIndex = iterIndex;
			start = iter;
			end = iter;
			sequenceNumScans = 1;
//			sequenceCog = sequenceBl = sequenceTr = scanCentre;
			sequenceCog = scanCentre;
			sequenceLocalMapCentre = RobotMapProcessing_centreLocalMap2 (PointF_toPointI (scanCentre));
		}
		else
		{
			sequenceTempCog.x = sequenceCog.x * sequenceNumScans + scanCentre.x;
			sequenceTempCog.y = sequenceCog.y * sequenceNumScans + scanCentre.y;
			temp = 1.0f / (float)(sequenceNumScans + 1);
			sequenceTempCog.x *= temp;
			sequenceTempCog.y *= temp;

/*			sequenceTempBl.x = min (sequenceBl.x, scanCentre.x);
			sequenceTempBl.y = min (sequenceBl.y, scanCentre.y);
			sequenceTempTr.x = max (sequenceTr.x, scanCentre.x);
			sequenceTempTr.y = max (sequenceTr.y, scanCentre.y);
			sequenceTempCog.x = (sequenceTr.x + sequenceBl.x) * 0.5f;
			sequenceTempCog.y = (sequenceTr.y + sequenceBl.y) * 0.5f;*/

			sequenceLocalMapTempCentre = RobotMapProcessing_centreLocalMap2 (PointF_toPointI (sequenceTempCog));

			isSequenceOK = checkMapScanSequence (db, start, end, sequenceLocalMapTempCentre);
			if (isSequenceOK)
			{
				end = iter;
				++sequenceNumScans;
				sequenceCog = sequenceTempCog;
//				sequenceTr = sequenceTempTr;
//				sequenceBl = sequenceTempBl;
				sequenceLocalMapCentre = sequenceLocalMapTempCentre;
			}
			else
			{
				db->behaviourData.closeLoop.isMoreMapDataComing = 1;
				submitCloseLoopMap (db, start, end, sequenceLocalMapCentre, startIndex);

				startIndex = iterIndex;
				start = iter;
				end = iter;
				sequenceNumScans = 1;
				sequenceCog = scanCentre;
				sequenceLocalMapCentre = RobotMapProcessing_centreLocalMap2 (PointF_toPointI (scanCentre));
			}
		}

		++iterIndex;
		iter = iter->next;
	}

	db->behaviourData.closeLoop.isMoreMapDataComing = 0;
	submitCloseLoopMap (db, start, end, sequenceLocalMapCentre, startIndex);

	List_clearWithDeallocator (&db->sensorData.mapScanList, freeMapScan);

	// This should be processed in RobotMapProcessing_determineIsLocalMapReady.
	db->sensorData.isLocalMapResetRequired = CENTRE_LOC_MAP;

	// We ALWAYS have to set this whenever we set CENTRE_LOC_MAP
	Geometry_ptFromOrient (
		db->status.pose.loc.x,
		db->status.pose.loc.y,
		&pt.x,
		&pt.y,
//		(CAM_P_OFFSET + db->geometryConstants.cam_minDist),
		CAM_P_OFFSET,
		db->status.pose.orient);
	db->sensorData.localMapResetPt1 = PointF_toPointI (pt);
	db->sensorData.localMapResetPt2 = PointF_toPointI (db->status.pose.loc);

	db->sensorData.isUnincorporatedMapScan = 0;
}


















int CloseLoop_isLoopClosePossible (
	RobotDatabase *db)
{
	return (db->partnerData.explorationCoalition.supervisor != -1);
}



//! Return if loc was updated
int CloseLoop_updateOwnLocEst (
	RobotDatabase *db,
	PointF *adjustment)
{
	UnionVector4F cov;
	Pose *visRobotPose;
	float evals[2];
	PointF evec;
	float stdDev;
	PointF newLoc;
#ifdef SIM_ERROR
	PointF simdError;
#endif

	// Combine error for visual estimate with that of visible robot
	// from group data
	cov.vector = db->sensorData.visRobotEstCov.vector;

	// Covariance matrices are already rotated to global axes, so no need
	// to call Uncertainty_updateCovariance. Instead can just add the matrices.
	visRobotPose = &db->groupData.robots[db->sensorData.visRobotIndex].pose;
	addMatrices (cov.mat, visRobotPose->mat, cov.mat);

	// Determine if the location estimate from the visual estimate will
	// be more accurate
	Uncertainty_calcEigenvalues (cov.mat, evals, &evec, &stdDev);
	if (stdDev < db->status.stdDev)
	{
		newLoc.x = visRobotPose->loc.x - db->sensorData.visRobotRelLoc.x;
		newLoc.y = visRobotPose->loc.y - db->sensorData.visRobotRelLoc.y;

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<UpdateOwnLoc>relLoc=(%f,%f) visRobLoc=(%f,%f) newLocEst=(%f,%f) oldLoc=(%f,%f) oldCov=(%f,%f,%f,%f) oldStdDev=%f estCov=(%f,%f,%f,%f) newCov=(%f,%f,%f,%f) newStdDev=%f</UpdateOwnLoc>\n",
			db->sensorData.visRobotRelLoc.x, db->sensorData.visRobotRelLoc.y,
			visRobotPose->loc.x, visRobotPose->loc.y,
			newLoc.x, newLoc.y,
			db->status.pose.loc.x, db->status.pose.loc.y,
			db->status.pose.mat[0], db->status.pose.mat[1], db->status.pose.mat[2], db->status.pose.mat[3],
			db->status.stdDev,
			db->sensorData.visRobotEstCov.mat[0], db->sensorData.visRobotEstCov.mat[1], db->sensorData.visRobotEstCov.mat[2], db->sensorData.visRobotEstCov.mat[3],
			cov.mat[0], cov.mat[1], cov.mat[2], cov.mat[3],
			stdDev);
#endif

#ifdef SIM_ERROR
		db->status.actualLocOffset = db->groupData.robots[db->sensorData.visRobotIndex].actualLocOffset;

		simdError = Uncertainty_accumErrorForVisualEst (cov.mat, &db->status.actualLocOffset, db->xmlLog);

		// We already have calc'd (when simulating error) the actual loc offset (and thus the actual loc), so
		// this should not change. So when we update the actual loc offset based on error from the visual location
		// estimate, our estimated location should be updated accordingly (in the opposite direction)
		newLoc.x -= simdError.x;
		newLoc.y -= simdError.y;
#endif

		adjustment->x = newLoc.x - db->status.pose.loc.x;
		adjustment->y = newLoc.y - db->status.pose.loc.y;

		db->status.pose.loc = newLoc;
		memcpy (db->status.pose.mat, cov.mat, sizeof (float) * 4);
		db->status.stdDev = stdDev;

		return 1;
	}

	adjustment->x = 0.0f;
	adjustment->y = 0.0f;

	return 0;
}

extern void RobotWrite_writeLeaveProvisionalMapData (RobotDatabase *db);

void leaveProvisionalMapData (RobotDatabase *db)
{
	if (db->sensorData.mapScanList.size > db->sensorData.scanLimit &&
		!(db->behaviourData.behaviour == CLOSE_LOOP || (db->behaviourData.behaviour == FOLLOW_PATH && db->behaviourData.baseBehaviour == CLOSE_LOOP)))
	{
		RobotWrite_writeLeaveProvisionalMapData (db);
		++db->behaviourData.closeLoop.currentCloseLoopSession;
	}
}

void CloseLoop_processData (RobotDatabase *db)
{
	PointF adjustment;
	int locUpdated;

	if (-1 != db->sensorData.visRobotIndex &&
		(db->sensorData.visRobotIndex == db->partnerData.explorationCoalition.supervisor))
	{
		// Update robot's own location and location-estimate-certainty.
		locUpdated = CloseLoop_updateOwnLocEst (db, &adjustment);

		if (db->sensorData.mapScanList.size > 1 &&
			((FOLLOW_PATH == db->behaviourData.behaviour && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP) ||
			CLOSE_LOOP == db->behaviourData.behaviour))
		{
			leaveProvisionalMapData (db);
//			CloseLoop_adjustMapData (db, adjustment);
			CloseLoop_adjustMapDataNEW (db, adjustment);
			submitCloseLoopMaps (db);

			++db->behaviourData.closeLoop.currentCloseLoopSession;
		}

		if (CLOSE_LOOP == db->behaviourData.behaviour ||
			(FOLLOW_PATH == db->behaviourData.behaviour && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
		{
			if (db->behaviourData.closeLoop.initialSync)
			{
				fprintf (db->xmlLog, "<InitialCloseLoopPerformed />\n");
			}
			else
			{
				fprintf (db->xmlLog, "<CloseLoopPerformed />\n");
			}

			CloseLoop_processIsAtDest (db);
		}
	}
}



















#include "../Ik/Ik.h"

extern void adoptStuck (RobotDatabase *db);
extern void BehaviourControl_lowLevelBehaviour (RobotDatabase *db);

/*!
Follow "snail" path around target. Let distance between adjacent paths be 30cm,
or 15 units in simulation. Let there be 6 dests per revolution, therefore every
60 degrees. Dest at index=0 should be at dist 15. Dest at index=6 should be at
dist 30, therefore dist += 15/6 for each subsequent dest.
*/
PointF CloseLoop_getReconLoc (
	const PointF tar,
	const int index)
{
	PointF dest;
	float dist, orient;

//	dist = 15.0f + (index) * (15.0f / 6.0f);
	dist = 15.0f + (index / 6) * 15.0f; // Narrow circle around target should be more likely
	orient = (index) * 1.047197f; // 60 degrees in radians

	Geometry_ptFromOrient (tar.x, tar.y, &dest.x, &dest.y, dist, orient);

	return dest;
}

PointF PointF_add (const PointF pt1, const PointF pt2)
{
	PointF sum;
	sum.x = pt1.x + pt2.x;
	sum.y = pt1.y + pt2.y;

	return sum;
}

PointF PointF_sub (const PointF pt1, const PointF pt2)
{
	PointF sub;
	sub.x = pt1.x - pt2.x;
	sub.y = pt1.y - pt2.y;
	return sub;
}

PointF CloseLoop_getTestTarget (RobotDatabase *db)
{
	PointF target;
	target = PointF_add (
		db->groupData.robots[db->partnerData.explorationCoalition.supervisor].pose.loc,
		db->groupData.robots[db->partnerData.explorationCoalition.supervisor].actualLocOffset);
	target = PointF_sub (
		target,
		db->status.actualLocOffset);
	return target;
}

extern void Supervision_leaveCoalition (RobotDatabase *db, const int explorerOrSupervisor, const char *reason);

void CloseLoop_processIsAtDest (RobotDatabase *db)
{
	List obstdCellGridResetList;
	PointF target;
	int isResetReqd = 0;

	if (!db->sensorData.mapScanList.size)
	{
		List_clear (&db->behaviourData.stack, 0);
		List_pushValue (&db->behaviourData.stack, &db->behaviourData.available);
		BehaviourCore_printStack (&db->behaviourData.stack, "closeLoop is at dest");
		db->behaviourData.behaviour = AVAILABLE;
		db->behaviourData.baseBehaviour = AVAILABLE;
		db->behaviourData.targetIndex = -1;

		// Leave coalition
		Supervision_leaveCoalition (db, 0, "closeLoopDone");

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<AdoptBehaviour>behaviour=\"AVAILABLE\"</AdoptBehaviour>\n");
#endif
		return;
	}

/*	if (!db->behaviourData.closeLoop.isAttemptingRecon)
	{
		db->behaviourData.closeLoop.profit.dest.type &= ~IS_AT_DEST;
		db->status.ikData.index = 0;
		db->status.ikData.len = 1;
		db->status.ikData.moves[0].dir = 2; // Move left
		db->status.ikData.moves[0].moveDist = 0.0f;
		db->status.ikData.moves[0].n = 12;
		db->status.ikData.moves[0].usBurst = 0;
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<CloseLoopIsAtDest />\n");
		fprintf (db->xmlLog, "<TryingRecon />\n");
#endif
	}*/
/*	if (!db->behaviourData.closeLoop.isAttemptingRecon ||
		db->behaviourData.closeLoop.nReconSteps < 12)
	{
		if (!db->behaviourData.closeLoop.isAttemptingRecon)
		{
			db->behaviourData.closeLoop.isAttemptingRecon = 1;
			db->behaviourData.closeLoop.nReconSteps = 0;
		}
		++db->behaviourData.closeLoop.nReconSteps;

		db->behaviourData.closeLoop.profit.dest.type &= ~IS_AT_DEST;
		db->status.ikData.index = 0;
		db->status.ikData.len = 2;
		db->status.ikData.moves[0].dir = 0; // Move fwd
		db->status.ikData.moves[0].moveDist = 0.0f;
		db->status.ikData.moves[0].n = 1;
		db->status.ikData.moves[0].usBurst = 0;
		db->status.ikData.moves[1].dir = 2; // Move left
		db->status.ikData.moves[1].moveDist = 0.0f;
		db->status.ikData.moves[1].n = 1;
		db->status.ikData.moves[1].usBurst = 0;
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<CloseLoopIsAtDest />\n");
		fprintf (db->xmlLog, "<TryingRecon />\n");
#endif
	}*/

	if (db->behaviourData.closeLoop.reconFlag == NOT_DOING_RECON ||
		db->behaviourData.closeLoop.nReconSteps < 20)
	{

#if 0
		if (db->behaviourData.closeLoop.reconFlag == NOT_DOING_RECON)
		{
			db->behaviourData.closeLoop.reconFlag = INITIAL_RECON;
			db->behaviourData.closeLoop.nReconSteps = 0;

			// First attempt - just move towards where we think the supervisor is
			db->behaviourData.closeLoop.profit.dest.dest.loc = db->behaviourData.closeLoop.profit.dest.target;
			db->behaviourData.closeLoop.profit.dest.dest.orient = 0.0f;
			db->behaviourData.closeLoop.profit.dest.leeway = 5.0f;
			db->behaviourData.closeLoop.profit.dest.maxDist = BASE_MOVE_DELTA_FWD;
			db->behaviourData.closeLoop.profit.dest.type = IS_COARSE_NAV_OK; // Don't set as new dest, as we calc it straight away

#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CloseLoopIsAtDest />\n");
			fprintf (db->xmlLog, "<TryingInitialRecon />\n");
#endif

			{
				obstdCellGridResetList = initList();
				CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
				isResetReqd = 1;
			}

			IK_determineMove (db->xmlLog, db->environment.navMap, &db->status.pose, &db->behaviourData.closeLoop.profit.dest, &db->status.ikData, &db->geometryConstants, &db->ikConstants, CLOSE_LOOP, 1, 1, DONT_ALLOW_OUTSIDE_LOCAL_MAP, 1, 1, 0);
			BehaviourControl_lowLevelBehaviour (db);

			if (isResetReqd)
			{
				isResetReqd = 0;
				CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
				List_clear (&obstdCellGridResetList, 1);
			}
			return;
		}
#endif

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<CloseLoopIsAtDest />\n");
		fprintf (db->xmlLog, "<TryingSecondaryRecon>nReconSteps=%d</TryingSecondaryRecon>\n", db->behaviourData.closeLoop.nReconSteps);
#endif

#if 1
		// When doing secondary recon steps we follow the routine:
		// * Navigate to point (points follow an outwardly spiraling path around supervisor loc est)
		// * Perform full rotation
		//
		// Therefore, if nReconSteps % 2 == 0, we navigate to a point, otherwise we perform a rotation

		db->behaviourData.closeLoop.profit.dest.type = IS_COARSE_NAV_OK;

		if (db->behaviourData.closeLoop.nReconSteps % 2 == 0)
		{
			db->behaviourData.closeLoop.reconFlag = RECON_SPIRAL;

#if 0 // Testing
				target = db->behaviourData.closeLoop.profit.dest.target;
#else
				target = CloseLoop_getTestTarget (db);
#endif

			// Can leave other settings the same as for <TryingInitialRecon /> above, and
			// just change the dest loc
			//
			// We use nReconSteps / 2, as we navigate to a point on even steps, and rotate on odd steps
			db->behaviourData.closeLoop.profit.dest.dest.loc = CloseLoop_getReconLoc (
				target,
				db->behaviourData.closeLoop.nReconSteps / 2);

			{
				obstdCellGridResetList = initList();
				CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
				isResetReqd = 1;
			}

			IK_determineMove (db->xmlLog, db->environment.navMap, &db->status.pose, &db->behaviourData.closeLoop.profit.dest, &db->status.ikData, &db->geometryConstants, &db->ikConstants, CLOSE_LOOP, 1, 1, DONT_ALLOW_OUTSIDE_LOCAL_MAP, 1, 1, 0);

			// Don't require a path (checkDists=0) and allow longer coarse nav moves (allowOutsideLocalMap=1)
//			IK_determineMove (db->xmlLog, db->environment.navMap, &db->status.pose, &db->behaviourData.closeLoop.profit.dest, &db->status.ikData, &db->geometryConstants, &db->ikConstants, CLOSE_LOOP, 0, 1, DO_ALLOW_OUTSIDE_LOCAL_MAP, 1, 1, 0);
			BehaviourControl_lowLevelBehaviour (db);

			if (isResetReqd)
			{
				isResetReqd = 0;
				CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
				List_clear (&obstdCellGridResetList, 1);
			}
		}
		else
		{
			db->behaviourData.closeLoop.reconFlag = RECON_PANNING;

			db->status.ikData.index = 0;
			db->status.ikData.len = 1;
			db->status.ikData.moves[0].dir = 2; // Rotate left
			db->status.ikData.moves[0].moveDist = 0.0f;
			db->status.ikData.moves[0].n = 6;
			db->status.ikData.moves[0].usBurst = 0;

			fprintf (db->xmlLog, "<ReconPan />\n");
		}

		++db->behaviourData.closeLoop.nReconSteps;
#endif
	}
	else
	{
		db->behaviourData.closeLoop.reconFlag = NOT_DOING_RECON;
		db->behaviourData.closeLoop.nReconSteps = 0;

		List_clear (&db->behaviourData.stack, 0);
		List_pushValue (&db->behaviourData.stack, &db->behaviourData.available);
		BehaviourCore_printStack (&db->behaviourData.stack, "closeLoop is at dest");

		//db->behaviourData.behaviour = AVAILABLE;
		//db->behaviourData.baseBehaviour = AVAILABLE;
		//db->behaviourData.targetIndex = -1;

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<CloseLoopIsAtDest />\n");
		fprintf (db->xmlLog, "<ReconFailed />\n");
//		fprintf (db->xmlLog, "<AdoptBehaviour>behaviour=\"STUCK\"</AdoptBehaviour>\n");
#endif

		// Leave coalition
		Supervision_leaveCoalition (db, 0, "closeLoopFailed");

		adoptStuck (db);
	}
}

void calcMovesCLOSE_LOOP (
	FILE *xmlLog,
	FollowPathData *followPathData,
	Image *navMap,
	Image *localMap,
	uchar *obstructedCellGrid,
	uchar *unreachableGrid, // May be expGrid, gtepGrid, etc.
	int *hasUnreachableLocalMapGridBeenUpdated,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const Pose pose,
	const float stdDev,
	const int index,
#if defined(SIMULATION) || defined(BOARD)
	uchar *boardUnreachableLocalMapGrid,
	BoardSensorData *boardSensorData,
#else
	RobotDatabase *db,
#endif
	ProfitTemp *temp,
	const PointF supervisorLoc,
	const float targetDist)
{
	temp->dest.target = supervisorLoc;
	temp->dest.leeway = EXPLORATION_LEEWAY;
	temp->dest.maxDist = MAX_FLT;
	temp->dest.targetDist = targetDist;
	temp->dest.type = IS_TARGET_CENTRIC;

	BehaviourCore_calcMovesAndPathIfReqd (
		xmlLog,
		followPathData,
		navMap,
		localMap,
		obstructedCellGrid,
		unreachableGrid,
		hasUnreachableLocalMapGridBeenUpdated,
		geometryConstants,
		camVectors,
		ikConstants,
		uncertaintyConstants,
		camPOffset,
		pose,
		stdDev,
		index,
//#if defined(SIMULATION) || defined(BOARD)
//		boardUnreachableLocalMapGrid,
//		boardSensorData,
//#else
//		db,
//#endif
		temp,
		CLOSE_LOOP,
		0);
}

#include "Backtrack.h"

void adoptCLOSE_LOOP (RobotDatabase *db, const int initialSync)
{
	PointF ptf;
	PointI pt;

	db->behaviourData.closeLoop.initialSync = initialSync;

	// We adopt close loop without calcing the specific path first, so 
	List_clear (&db->behaviourData.stack, 0);
	List_pushValue (&db->behaviourData.stack, (BasicBehaviourData*)&db->behaviourData.closeLoop);
	db->behaviourData.behaviour = CLOSE_LOOP;
	db->behaviourData.baseBehaviour = CLOSE_LOOP;
	db->behaviourData.targetIndex = db->behaviourData.targetCounter;
	++db->behaviourData.targetCounter;
	db->behaviourData.closeLoop.profit.dest.type |= IS_NEW_DEST;
	db->behaviourData.closeLoop.reconFlag = NOT_DOING_RECON;

	// For normal behaviours, we want to have these submitted before closing the loop
	// so that the profit from them can be accumulated/analysed
	db->sensorData.isLocalMapResetRequired = CENTRE_LOC_MAP;

	// We ALWAYS have to set this whenever we set CENTRE_LOC_MAP
	Geometry_ptFromOrient (
		db->status.pose.loc.x,
		db->status.pose.loc.y,
		&ptf.x,
		&ptf.y,
//		(CAM_P_OFFSET + db->geometryConstants.cam_minDist),
		CAM_P_OFFSET,
		db->status.pose.orient);
	db->sensorData.localMapResetPt1 = PointF_toPointI (ptf);
	db->sensorData.localMapResetPt2 = PointF_toPointI (db->status.pose.loc);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<CentreLocalMapAdoptCloseLoop />\n");
#endif

	pt.x = db->status.pose.loc.x - db->environment.navMap->orig.x;
	pt.y = db->status.pose.loc.y - db->environment.navMap->orig.y;
//	if (FREE_TERRAIN != Image_getPixel_dontCheck (db->environment.navMap, db->status.pose.loc.x, db->status.pose.loc.y))
	if (FREE_TERRAIN != Image_getPixel_dontCheck (db->environment.navMap, pt.x, pt.y))
	{
		fprintf (db->xmlLog, "<AdoptBacktrackBeforeCloseLoop />\n");
		adoptBacktrack (db);
		return;
	}
}

void CloseLoop_removeSupFromNavMap (
	RobotDatabase *db,
	List *resetList)
{
	RobotMapProcessing_removeRobotFromNavMap (
		db,
		db->partnerData.explorationCoalition.supervisor,
		db->sensorData.localMapCentre,
		db->sensorData.localMap->orig,
		db->environment.navMap);
	RobotMapProcessing_updateObstructedGridAroundRobot (
		db,
		db->environment.navMap,
		db->partnerData.explorationCoalition.supervisor,
		resetList);
}

void CloseLoop_replaceSupInNavMap (
	RobotDatabase *db,
	List *obstdCellGridResetList)
{
	RobotMapProcessing_incRobotIntoNavMap (
		db,
		db->partnerData.explorationCoalition.supervisor,
		db->sensorData.localMapCentre,
		db->sensorData.localMap->orig,
		db->environment.navMap);
	RobotMapProcessing_resetObstdGridCellsAroundRobot (db, obstdCellGridResetList);
}

extern int Exploration_checkIfExpTarIsValid (RobotDatabase *db);

void calcProfitCLOSE_LOOP (CloseLoopData *c, RobotDatabase *db, const float maxProfit)
{
	ProfitTemp temp;
	PointF supervisorLoc;
	Pose visualEstPose;
	float covariance[4];
	float covMove[4];
	float evals[2];
	PointF evec;
	float visualEstStdDev, loopCloseStdDev, finalStdDev, stdDevRed, grossTemp, grossFromNewScans, grossFromAdjs;
	int i;
	MapScan *mapScan;
	ListNode *iter, *currentScanPlaceholder;
	int nCloseLoopUpdatedScans;
	Vector4F covVisualEst;
	int hasUnreachableLocalMapGridBeenUpdated = 0;
	List obstdGridCellsToReset;
	float largeMapScanListBonus;
	const float avgMoveBurst = 500000.0f;
#ifdef PRINT_PROFIT
	int scanIndex;
	int nInitialScansAdjusted;
#endif

	BehaviourCore_clearProfitData (&c->profit);

#ifndef SETUP_TEST_COALITION
	if (db->partnerData.explorationCoalition.id == -1 || !db->partnerData.explorationCoalition.collabData.isSupAtFinalDest)
	{
		return;
	}

//	if (db->sensorData.mapScanList.size < 20)
//	if (db->sensorData.mapScanList.size < 70 &&
	if (db->sensorData.mapScanList.size < 40 &&
//	if (db->sensorData.mapScanList.size < 20 &&
		maxProfit != MIN_FLT)
	{
		return;
	}

#if 0
	if (Exploration_checkIfExpTarIsValid (db))
	{
		return;
	}
#endif
#endif // ifndef SETUP_TEST_COALITION

	temp = initProfitTemp();

	//
	// Take supervisor out of nav map and obstructed cell grid
	//
	obstdGridCellsToReset = initList();
	CloseLoop_removeSupFromNavMap (db, &obstdGridCellsToReset);

	supervisorLoc = db->groupData.robots[db->partnerData.explorationCoalition.supervisor].pose.loc;
	calcMovesCLOSE_LOOP (
		db->xmlLog,
		&db->behaviourData.followPath,
		db->environment.navMap,
		db->sensorData.localMap,
		db->environment.obstructedCellGrid,
		NULL,
		&hasUnreachableLocalMapGridBeenUpdated,
		&db->geometryConstants,
		&db->camVectors,
		&db->ikConstants,
		&db->uncertaintyConstants,
		CAM_P_OFFSET,
		db->status.pose,
		db->status.stdDev,
		db->status.index,
#if defined(SIMULATION) || defined(BOARD)
		db->board->environment.unreachableLocalMapGrid,
		&db->board->sensorData,
#else
		db,
#endif
		&temp,
		supervisorLoc,
		CAM_P_OFFSET + (db->uncertaintyConstants.visEst_typicalDist * MAP_SCALE_INV));

	if (MAX_FLT == temp.nSteps)
	{
		fprintf (db->xmlLog, "<CloseLoopPathFailed>iter=%d</CloseLoopPathFailed>\n", db->status.nIterations);
		db->behaviourData.shouldBacktrack = 1;
		goto CALC_PROFIT_CLOSE_LOOP_REINSERT_SUP;
//		return;
	}

	//
	// Estimate error when visual contact is established
	//
	visualEstPose = db->status.pose;
	memcpy (visualEstPose.mat, db->groupData.robots[db->partnerData.explorationCoalition.supervisor].pose.mat, sizeof (float) * 4);

	// The calculated location-estimate covariance is scaled to map space
	covVisualEst = Uncertainty_updateCovarianceForVisualEst (
		&db->uncertaintyConstants,
		db->uncertaintyConstants.visEst_typicalDist,
		PI * 0.25f,
		&visualEstPose,
		&visualEstStdDev);

	//
	// Add map scans collected en route to supervisor.
	//
	currentScanPlaceholder = db->sensorData.mapScanList.back;
	memcpy (covariance, db->status.pose.mat, sizeof (float) * 4);

	// Take very-rough estimate of length of average move
	covMove[0] = BASE_MOVE_VAR_FWD + avgMoveBurst * MOVE_US_VAR_FWD;
	covMove[1] = BASE_MOVE_COVAR + avgMoveBurst * MOVE_US_COVAR;
	covMove[2] = covMove[1];
	covMove[3] = BASE_MOVE_VAR_LAT + avgMoveBurst * MOVE_US_VAR_LAT;
	i = -1;
//	temp.nSteps *= AVG_ACTUAL_CLOSE_LOOP_MOVES;
//	temp.nSteps *= 1.5;
//	temp.nSteps *= 1.0;
	temp.nSteps *= 0.5; // testing
	while (++i < temp.nSteps)
	{
		Uncertainty_updateCovariance (covMove, PI/4.0f, covariance);
		Uncertainty_calcEigenvalues (covariance, evals, &evec, &loopCloseStdDev);

		mapScan = (MapScan*)malloc (sizeof (MapScan));
		mapScan->stdDev = loopCloseStdDev;
		mapScan->estdGain = (EXP_AREA * EXP_AREA) * 0.01f; // This should be negligible
		mapScan->behaviour = CLOSE_LOOP;
		List_pushValue (&db->sensorData.mapScanList, mapScan);
	}

	//
	// Calculate stdDev from supervisor back to start of mapScanList.
	//
	loopCloseStdDev = visualEstStdDev;
	temp.gross = 0.0f;
	temp.expenditure = 0.0f;
	nCloseLoopUpdatedScans = 0;
	grossFromNewScans = 0.0f;
	grossFromAdjs = 0.0f;

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<CloseLoopScanProfit>\n");
	scanIndex = db->sensorData.mapScanList.size - 1;  // Scan ids should start at front of list
	nInitialScansAdjusted = 0;
#endif

	iter = db->sensorData.mapScanList.back;
	while (iter)
	{
		loopCloseStdDev += db->uncertaintyConstants.closeLoop_stdDev;
		mapScan = (MapScan*)iter->value;

		stdDevRed = mapScan->stdDev - loopCloseStdDev;
		finalStdDev = min (mapScan->stdDev, loopCloseStdDev);
		if (stdDevRed > 0.0f)
		{
			++nCloseLoopUpdatedScans;
		}
		else
		{
			stdDevRed = 0.0;
		}

		{
			if (CLOSE_LOOP == mapScan->behaviour)
			{
				// All profit for this scan is attributed to CLOSE_LOOP.
				grossTemp = calcMappingProfit (mapScan->estdGain, loopCloseStdDev);
				grossFromNewScans += grossTemp;
#ifdef PRINT_PROFIT
				fprintf (db->xmlLog, "<CloseLoopScan>scan=%d gross=%f behaviour=%d origStdDev=%f finalStdDev=%f red=0.0</CloseLoopScan>\n",
					scanIndex,
					grossTemp,
					mapScan->behaviour,
					mapScan->stdDev,
					loopCloseStdDev);
//				++nInitialScansAdjusted;
#endif
			}
			else
			{
				// Profit for reduction in stdDev is attributed to CLOSE_LOOP.
				grossTemp = calcStdDevReductionProfit (mapScan->estdGain, stdDevRed);
				grossFromAdjs += grossTemp;
#ifdef PRINT_PROFIT
				fprintf (db->xmlLog, "<CloseLoopScan>scan=%d gross=%f behaviour=%d origStdDev=%f finalStdDev=%f red=%f</CloseLoopScan>\n",
					scanIndex,
					grossTemp,
					mapScan->behaviour,
					mapScan->stdDev,
					loopCloseStdDev,
					stdDevRed);
#endif
			}

#ifdef PRINT_PROFIT
			--scanIndex;
#endif
//			++nCloseLoopUpdatedScans;
		}
//		else
//		{
//			break;
//		}

		iter = iter->prev;
	}

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "</CloseLoopScanProfit>\n");
	fprintf (db->xmlLog, "<CloseLoopProfit>visualEstStdDev=%f nScansInitial=%d nScansAdjusted=%d nScansCloseLoop=%d nScansTotal=%d grossNewScans=%f grossAdjs=%f</CloseLoopProfit>\n",
		visualEstStdDev,
		db->sensorData.mapScanList.size - (int)temp.nSteps,
		nCloseLoopUpdatedScans,
		(int)temp.nSteps,
		db->sensorData.mapScanList.size,
		grossFromNewScans,
		grossFromAdjs);
#endif
//
	c->profit.dest = temp.dest; // This is set in calcMovesCLOSE_LOOP
	c->profit.stdDevAtDest = visualEstStdDev;
	c->profit.gross = grossFromNewScans + grossFromAdjs;
//	c->profit.gross = grossFromAdjs; // Do *NOT* take gross from new scans into account
#if 0
	c->profit.gross += grossFromNewScans;
#endif 
	c->profit.expenditure = temp.expenditure;
	c->profit.nSteps = temp.nSteps;
	c->profit.resources = temp.nSteps * BATTERY_LOSS_MOVE;
	c->profit.ratio = (c->profit.gross - c->profit.expenditure) / c->profit.resources;

	largeMapScanListBonus = 0.0f;
	if (db->sensorData.mapScanList.size > 60)
	{
//		largeMapScanListBonus = 10.0f;
	}
	c->profit.ratio += largeMapScanListBonus;

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<MaxProfit>behaviour=\"CLOSE_LOOP\" target=(%f,%f) bonus=%f profitRatio=%f ownGross=%f profitAttribd=%f resources=%f nSteps=%f nUpdatedScans=%d stdDevAtDest=%f visEstCov=(%f,%f,%f,%f)</MaxProfit>\n",
		supervisorLoc.x, supervisorLoc.y,
		largeMapScanListBonus,
		c->profit.ratio,
		c->profit.gross,
		c->profit.expenditure,
		c->profit.resources,
		c->profit.nSteps,
		nCloseLoopUpdatedScans,
		c->profit.stdDevAtDest,
		covVisualEst.x, covVisualEst.y, covVisualEst.z, covVisualEst.w);
#endif

	iter = db->sensorData.mapScanList.back;
	while (iter && iter != currentScanPlaceholder)
	{
		List_deleteElement (&db->sensorData.mapScanList, &iter, 0, 1);
	}


CALC_PROFIT_CLOSE_LOOP_REINSERT_SUP:
	//
	// Put supervisor back into nav map and obstructed cell grid
	//
	CloseLoop_replaceSupInNavMap (db, &obstdGridCellsToReset);
//	RobotMapProcessing_resetObstdGridCellsAroundRobot (db, &obstdGridCellsToReset);
	List_clear (&obstdGridCellsToReset, 1);
}

void CloseLoop_calcDest (CloseLoopData *c, RobotDatabase *db)
{
	ProfitTemp temp;
	PointF supervisorLoc;
	Pose visualEstPose;
	float visualEstStdDev;
	Vector4F covVisualEst;
	int hasUnreachableLocalMapGridBeenUpdated = 0;
	List obstdGridCellsToReset;

	BehaviourCore_clearProfitData (&c->profit);

	temp = initProfitTemp();

	//
	// Take supervisor out of nav map and obstructed cell grid
	//
	obstdGridCellsToReset = initList();
	CloseLoop_removeSupFromNavMap (db, &obstdGridCellsToReset);

	supervisorLoc = db->groupData.robots[db->partnerData.explorationCoalition.supervisor].pose.loc;
	calcMovesCLOSE_LOOP (
		db->xmlLog,
		&db->behaviourData.followPath,
		db->environment.navMap,
		db->sensorData.localMap,
		db->environment.obstructedCellGrid,
		NULL,
		&hasUnreachableLocalMapGridBeenUpdated,
		&db->geometryConstants,
		&db->camVectors,
		&db->ikConstants,
		&db->uncertaintyConstants,
		CAM_P_OFFSET,
		db->status.pose,
		db->status.stdDev,
		db->status.index,
#if defined(SIMULATION) || defined(BOARD)
		db->board->environment.unreachableLocalMapGrid,
		&db->board->sensorData,
#else
		db,
#endif
		&temp,
		supervisorLoc,
		CAM_P_OFFSET + (db->uncertaintyConstants.visEst_typicalDist * MAP_SCALE_INV));

	if (MAX_FLT == temp.nSteps)
	{
		goto CALC_PROFIT_CLOSE_LOOP_REINSERT_SUP;
	}

	//
	// Estimate error when visual contact is established
	//
	visualEstPose = db->status.pose;
	memcpy (visualEstPose.mat, db->groupData.robots[db->partnerData.explorationCoalition.supervisor].pose.mat, sizeof (float) * 4);

	// The calculated location-estimate covariance is scaled to map space
	covVisualEst = Uncertainty_updateCovarianceForVisualEst (
		&db->uncertaintyConstants,
		db->uncertaintyConstants.visEst_typicalDist,
		PI * 0.25f,
		&visualEstPose,
		&visualEstStdDev);

#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<CloseLoopDest>nSteps=%d visualEstStdDev=%f</CloseLoopDest>\n",
		(int)temp.nSteps,
		visualEstStdDev);
#endif

	c->profit.dest = temp.dest; // This is set in calcMovesCLOSE_LOOP
	c->profit.stdDevAtDest = visualEstStdDev;
	c->profit.gross = 0.0f;
	c->profit.expenditure = 0.0f;
	c->profit.nSteps = temp.nSteps;
	c->profit.resources = temp.nSteps * BATTERY_LOSS_MOVE;
	c->profit.ratio = 0.0f;

CALC_PROFIT_CLOSE_LOOP_REINSERT_SUP:
	//
	// Put supervisor back into nav map and obstructed cell grid
	//
	CloseLoop_replaceSupInNavMap (db, &obstdGridCellsToReset);
//	RobotMapProcessing_resetObstdGridCellsAroundRobot (db, &obstdGridCellsToReset);
	List_clear (&obstdGridCellsToReset, 1);


}

#endif // BOARD


