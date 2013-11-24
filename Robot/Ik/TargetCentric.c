#include "../../Common/RobotDefs.h"



#include "TargetCentric.h"
#include "../../Common/Geometry.h"
#include "../../Common/Maths.h"

int l_allowOutsideLocalMap;

PointF targetCentricFwdKinematics_1Move (
	const PoseSimple startPose,
	const int rotationIndex,
	const float moveDist,
	const float us,
	const float targetDist,
	const IkConstants *ikConstants)
{
	PointF pt;
	float orientAfterMove = Geometry_orientSum (startPose.orient, BASE_MOVE_DELTA_ORIENT + us * MOVE_US_DELTA_ORIENT);
	float orientAfterRotate = Geometry_orientSum (orientAfterMove, ikConstants->rotations[rotationIndex]);
	float startOrientPlus90 = Geometry_orientSum (startPose.orient, PI/2);
	float orientAfterMovePlus90 = Geometry_orientSum (orientAfterMove, PI/2);

	pt.x = startPose.loc.x + moveDist * cos (startPose.orient);
	pt.y = startPose.loc.y + moveDist * sin (startPose.orient);

	pt.x += (BASE_MOVE_DELTA_LAT + (us * MOVE_US_DELTA_LAT)) * cos (startOrientPlus90);
	pt.y += (BASE_MOVE_DELTA_LAT + (us * MOVE_US_DELTA_LAT)) * sin (startOrientPlus90);

	pt.x += ikConstants->rotationOffsets[rotationIndex].x * cos (orientAfterMove);
	pt.y += ikConstants->rotationOffsets[rotationIndex].x * sin (orientAfterMove);

	pt.x += ikConstants->rotationOffsets[rotationIndex].y * cos (orientAfterMovePlus90);
	pt.y += ikConstants->rotationOffsets[rotationIndex].y * sin (orientAfterMovePlus90);

	pt.x += targetDist * cos (orientAfterRotate);
	pt.y += targetDist * sin (orientAfterRotate);
	return pt;
}

int targetCentricOptimumMoveDist_1Move (
	const PoseSimple startPose,
	const int rotationIndex,
	Dest *dest,
	float *move,
	float *moveErrorSqrd,
	const GeometryConstants *geometryConstants,
	const IkConstants *ikConstants)
{
	PointF pt;
	float errorSqrd, lastErrorSqrd;
	float moveDist;
	float lastMoveDist[2];
	float temp;
	float us;
	float bestErrorSqrd, bestMoveDist;

	bestErrorSqrd = MAX_FLT;
	bestMoveDist = MAX_FLT;

	// Start at median move dist.
	lastMoveDist[1] = ikConstants->moveMedian - 1.0f;
	lastMoveDist[0] = ikConstants->moveMedian;
	pt = targetCentricFwdKinematics_1Move(startPose, rotationIndex, ikConstants->moveMedian, ikConstants->usMedian, dest->targetDist, ikConstants);
	lastErrorSqrd = Geometry_distSqrd (pt.x, pt.y, dest->target.x, dest->target.y);

	moveDist = ikConstants->moveMedian + 1.0f;
	us = calcMoveMotorBurst (moveDist);
	pt = targetCentricFwdKinematics_1Move(startPose, rotationIndex, moveDist, us, dest->targetDist, ikConstants);
	errorSqrd = Geometry_distSqrd (pt.x, pt.y, dest->target.x, dest->target.y);

	// Continue while not settled on minimum.
	while (moveDist != lastMoveDist[0] && moveDist != lastMoveDist[1] && fabs (errorSqrd - lastErrorSqrd) > 0.05f)
	{
		temp = moveDist;
		if (errorSqrd < lastErrorSqrd)
		{
			moveDist += (moveDist - lastMoveDist[0]);
		}
		else
		{
			moveDist += (lastMoveDist[0] - moveDist) * 0.7f;
		}
		lastMoveDist[1] = lastMoveDist[0];
		lastMoveDist[0] = temp;
		lastErrorSqrd = errorSqrd;

		us = calcMoveMotorBurst (moveDist);
		pt = targetCentricFwdKinematics_1Move(startPose, rotationIndex, moveDist, us, dest->targetDist, ikConstants);
		errorSqrd = Geometry_distSqrd (pt.x, pt.y, dest->target.x, dest->target.y);

		if (errorSqrd < bestErrorSqrd && moveDist >= BASE_MOVE_DELTA_FWD && moveDist <= geometryConstants->nav_maxDist)
		{
			bestErrorSqrd = errorSqrd;
			bestMoveDist = moveDist;
		}
	}

	// If optimum move dist has moved outside acceptable bounds, see if value at bounds is ok.
	if (moveDist < BASE_MOVE_DELTA_FWD)
	{
		moveDist = BASE_MOVE_DELTA_FWD;
		pt = targetCentricFwdKinematics_1Move(startPose, rotationIndex, BASE_MOVE_DELTA_FWD, 0, dest->targetDist, ikConstants);
		errorSqrd = Geometry_distSqrd (pt.x, pt.y, dest->target.x, dest->target.y);

		if (errorSqrd < bestErrorSqrd)
		{
			bestMoveDist = moveDist;
			bestErrorSqrd = errorSqrd;
		}
	}
	else if (moveDist > geometryConstants->nav_maxDist)
	{
		moveDist = geometryConstants->nav_maxDist;
		us = calcMoveMotorBurst (moveDist);
		pt = targetCentricFwdKinematics_1Move(startPose, rotationIndex, moveDist, us, dest->targetDist, ikConstants);
		errorSqrd = Geometry_distSqrd (pt.x, pt.y, dest->target.x, dest->target.y);

		if (errorSqrd < bestErrorSqrd)
		{
			bestMoveDist = moveDist;
			bestErrorSqrd = errorSqrd;
		}
	}

	*move = bestMoveDist;
	*moveErrorSqrd = bestErrorSqrd;

	return (bestErrorSqrd < (dest->leeway * dest->leeway));
}

void print1MovePoses (
	FILE *f,
	const PoseSimple startPose,
	const int rotationIndex,
	Dest *dest,
	const float moveDist,
	const float moveErrorSqrd,
	const int nMoves,
	const IkConstants *ikConstants)
{
	PoseSimple tempPose1;
	float moveMS = (moveDist - BASE_MOVE_DELTA_FWD) * (1 / MOVE_US_DELTA_FWD);
	float orientAfterMove = Geometry_orientSum (startPose.orient, BASE_MOVE_DELTA_ORIENT + moveMS * MOVE_US_DELTA_ORIENT);
	float orientAfterRotate = Geometry_orientSum (orientAfterMove, ikConstants->rotations[rotationIndex]);
	float startOrientPlus90 = Geometry_orientSum (startPose.orient, PI/2);
	float orientAfterMovePlus90 = Geometry_orientSum (orientAfterMove, PI/2);

#ifdef PRINT_IK_DETAIL
	fprintf (f, "<TargetCentricMove>ik=1Move startPose=(%f,%f,%f) ",
		startPose.loc.x, startPose.loc.y, startPose.orient);
#endif

	tempPose1.loc.x = startPose.loc.x + moveDist * cos (startPose.orient);
	tempPose1.loc.y = startPose.loc.y + moveDist * sin (startPose.orient);
	tempPose1.loc.x += (BASE_MOVE_DELTA_LAT + (moveMS * MOVE_US_DELTA_LAT)) * cos (startOrientPlus90);
	tempPose1.loc.y += (BASE_MOVE_DELTA_LAT + (moveMS * MOVE_US_DELTA_LAT)) * sin (startOrientPlus90);

#ifdef PRINT_IK_DETAIL
	fprintf (f, "afterMove=(%f,%f,%f) ", tempPose1.loc.x, tempPose1.loc.y, orientAfterMove);
#endif

	tempPose1.loc.x += ikConstants->rotationOffsets[rotationIndex].x * cos (orientAfterMove);
	tempPose1.loc.y += ikConstants->rotationOffsets[rotationIndex].x * sin (orientAfterMove);
	tempPose1.loc.x += ikConstants->rotationOffsets[rotationIndex].y * cos (orientAfterMovePlus90);
	tempPose1.loc.y += ikConstants->rotationOffsets[rotationIndex].y * sin (orientAfterMovePlus90);

#ifdef PRINT_IK_DETAIL
	fprintf (f, "afterRotate=(%f,%f,%f) ", tempPose1.loc.x, tempPose1.loc.y, orientAfterRotate);
#endif

	tempPose1.loc.x += dest->targetDist * cos (orientAfterRotate);
	tempPose1.loc.y += dest->targetDist * sin (orientAfterRotate);

#ifdef PRINT_IK_DETAIL
	fprintf (f, "optimumPt=(%f,%f) ", tempPose1.loc.x, tempPose1.loc.y);
	fprintf (f, "errorSqrd=%f nMoves=%d", moveErrorSqrd, nMoves);
	fprintf (f, "</TargetCentricMove>\n");
#endif
}

//! Return collision.
int checkIKMoves_1Move (
	const Image *navMap,
	const PoseSimple startPose,
	const int rotationIndex,
	const float moveDist,
	const PointF target,
	const float targetDist,
	const float leewaySqrd,
	const IkConstants *ikConstants)
{
	PointF pt;
	PointF old;
	float us = calcMoveMotorBurst (moveDist);
	float orientAfterMove = Geometry_orientSum (startPose.orient, BASE_MOVE_DELTA_ORIENT + us * MOVE_US_DELTA_ORIENT);
	float startOrientPlus90 = Geometry_orientSum (startPose.orient, PI/2);
	float orientAfterMovePlus90 = Geometry_orientSum (orientAfterMove, PI/2);
#ifdef _DEBUG
	float finalOrient = Geometry_orientSum (orientAfterMove, ikConstants->rotations[rotationIndex]);
#endif

	pt.x = startPose.loc.x + moveDist * cos (startPose.orient);
	pt.y = startPose.loc.y + moveDist * sin (startPose.orient);

	pt.x += (BASE_MOVE_DELTA_LAT + (us * MOVE_US_DELTA_LAT)) * cos (startOrientPlus90);
	pt.y += (BASE_MOVE_DELTA_LAT + (us * MOVE_US_DELTA_LAT)) * sin (startOrientPlus90);

	if (checkMove (startPose.loc, pt, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
	{
		return 1;
	}
	old = pt;

	pt.x += ikConstants->rotationOffsets[rotationIndex].x * cos (orientAfterMove);
	pt.y += ikConstants->rotationOffsets[rotationIndex].x * sin (orientAfterMove);

	pt.x += ikConstants->rotationOffsets[rotationIndex].y * cos (orientAfterMovePlus90);
	pt.y += ikConstants->rotationOffsets[rotationIndex].y * sin (orientAfterMovePlus90);

	if (checkMove (old, pt, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
	{
		return 1;
	}

	// Just check LOS
	// Allow LOS to be outside local map
	if (checkMove (pt, target, navMap, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP))
	{
		return 1;
	}
	
#ifdef _DEBUG
	// Now verify that the optimum pt is where it is meant to be
	Geometry_ptFromOrient (pt.x, pt.y, &pt.x, &pt.y, targetDist, finalOrient);
	if (Geometry_distSqrd (pt.x, pt.y, target.x, target.y) >= leewaySqrd)
	{
		assert(0);
	}
#endif

	return 0;
}

void targetCentricIK_1Move (FILE *f,
							const Image *navMap,
							Dest *dest,
							const Pose *current,
							IKData *ikData,
							const GeometryConstants *geometryConstants,
							const IkConstants *ikConstants,
							const int verbose)
{
	PoseSimple startPose, tempPose1, endPose;
	PointF optimumPt;
	int i, j;
	float distSqrd;
	float moveDist;
	float targetDistSqrd = dest->targetDist * dest->targetDist;
	float leewaySqrd = dest->leeway * dest->leeway;
	float tempLeewaySqrd = (ikConstants->moveRange + 2.0f) * (ikConstants->moveRange + 2.0f);
	float moveErrorSqrd;
	int nMoves;
	float currentBestMoveErrorSqrd = MAX_FLT;
	int currentBestNMoves = MAX_INT32;


	for (i = 0; i < N_ROTATIONS_IN_TOTAL; ++i)
	{
		// x is the move in pose direction.
		// y is the move perpindicular to pose direction.
		Geometry_ptFromOrient (
			current->loc.x,
			current->loc.y,
			&startPose.loc.x,
			&startPose.loc.y,
			ikConstants->rotationOffsets[i].x,
			current->orient);

		Geometry_ptFromOrient (
			startPose.loc.x,
			startPose.loc.y,
			&startPose.loc.x,
			&startPose.loc.y,
			ikConstants->rotationOffsets[i].y,
			Geometry_orientSum (current->orient, PI/2.0f));

		if (checkMove (current->loc, startPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
		{
			continue;
		}

		startPose.orient = Geometry_orientSum (current->orient, ikConstants->rotations[i]);

		// Might not have to move; initial check if distance from current loc is ok.
		Geometry_ptFromOrient (
			startPose.loc.x,
			startPose.loc.y,
			&optimumPt.x,
			&optimumPt.y,
			dest->targetDist,
			startPose.orient);

		distSqrd = Geometry_distSqrd (optimumPt.x, optimumPt.y, dest->target.x, dest->target.y);
		if (distSqrd < leewaySqrd)
		{
			// Just check LOS
			if (!checkMove (startPose.loc, dest->target, navMap, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP))
			{
				ikData->index = 0;
				ikData->len = 0;
#ifdef PRINT_IK_DETAIL
				if (verbose)
				{
					fprintf (f, "<TargetCentricMove>ik=1Move pose=(%f,%f,%f) optimumPt=(%f,%f)</TargetCentricMove>\n",
						startPose.loc.x, startPose.loc.y, startPose.orient,
						optimumPt.x, optimumPt.y);
				}
#endif
				// moveDirs==0 means we didn't have to move
				if (0 != ikConstants->moveDirs[i])
				{
					ikData->moves[0].dir = ikConstants->moveDirs[i];
					ikData->moves[0].n = ikConstants->moveNRotations[i];
					ikData->moves[0].usBurst = 0;
					ikData->len = 1;
				}
				else
				{
					dest->type |= IS_AT_DEST;
				}
				return;
			}
		}

		// That didn't work...

		// Move median distance from initial orientation.
		Geometry_ptFromOrient (
			startPose.loc.x,
			startPose.loc.y,
			&tempPose1.loc.x,
			&tempPose1.loc.y,
			ikConstants->moveMedian,
			startPose.orient);

		Geometry_ptFromOrient (
			tempPose1.loc.x,
			tempPose1.loc.y,
			&tempPose1.loc.x,
			&tempPose1.loc.y,
			ikConstants->offsetMedian,
			Geometry_orientSum (startPose.orient, PI/2));

		if (checkMove (startPose.loc, tempPose1.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
		{
			continue;
		}

		distSqrd = Geometry_distSqrd (tempPose1.loc.x, tempPose1.loc.y, dest->target.x, dest->target.y);

		if (fabs (distSqrd - targetDistSqrd) > tempLeewaySqrd)
		{
			continue;
		}

		tempPose1.orient = Geometry_orientSum (startPose.orient, ikConstants->orientChangeMedian);

		for (j = 0; j < N_ROTATIONS_IN_TOTAL; ++j)
		{
			Geometry_ptFromOrient (
				tempPose1.loc.x,
				tempPose1.loc.y,
				&endPose.loc.x,
				&endPose.loc.y,
				ikConstants->rotationOffsets[j].x,
				tempPose1.orient);

			Geometry_ptFromOrient (
				endPose.loc.x,
				endPose.loc.y,
				&endPose.loc.x,
				&endPose.loc.y,
				ikConstants->rotationOffsets[j].y,
				Geometry_orientSum (tempPose1.orient, PI/2));

			if (checkMove (endPose.loc, tempPose1.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
			{
				continue;
			}

			endPose.orient = Geometry_orientSum (tempPose1.orient, ikConstants->rotations[j]);

			Geometry_ptFromOrient (
				endPose.loc.x,
				endPose.loc.y,
				&optimumPt.x,
				&optimumPt.y,
				dest->targetDist,
				endPose.orient);

			distSqrd = Geometry_distSqrd (optimumPt.x, optimumPt.y, dest->target.x, dest->target.y);

			if (distSqrd > tempLeewaySqrd)
			{
				continue;
			}

			if (targetCentricOptimumMoveDist_1Move(startPose, j, dest, &moveDist, &moveErrorSqrd, geometryConstants, ikConstants))
			{
				if (checkIKMoves_1Move (navMap, startPose, j, moveDist, dest->target, dest->targetDist, leewaySqrd, ikConstants))
				{
					continue;
				}

				nMoves = 1 + (int)(0 != ikConstants->moveDirs[i]) + (int)(0 != ikConstants->moveDirs[j]);

				if (verbose)
				{
					print1MovePoses (f, startPose, j, dest, moveDist, moveErrorSqrd, nMoves, ikConstants);
				}

				if (nMoves < currentBestNMoves || (nMoves == currentBestNMoves && moveErrorSqrd < currentBestMoveErrorSqrd))
				{
					currentBestNMoves = nMoves;
					currentBestMoveErrorSqrd = moveErrorSqrd;

					ikData->index = 0;
					ikData->len = 0;

					if (0 != ikConstants->moveDirs[i])
					{
						ikData->moves[0].dir = ikConstants->moveDirs[i];
						ikData->moves[0].n = ikConstants->moveNRotations[i];
						ikData->moves[0].usBurst = 0;
						ikData->len = 1;
					}

					ikData->moves[ikData->len].dir = 0; // Fwd
					ikData->moves[ikData->len].moveDist = moveDist;
					ikData->moves[ikData->len].usBurst = (moveDist - BASE_MOVE_DELTA_FWD) * (1 / MOVE_US_DELTA_FWD);
					ikData->moves[ikData->len].n = 1;
					DEBUG_ASSERT(ikData->moves[ikData->len].usBurst < 10000000)
					ikData->len += 1;

					if (0 != ikConstants->moveDirs[j])
					{
						ikData->moves[ikData->len].dir = ikConstants->moveDirs[j];
						ikData->moves[ikData->len].n = ikConstants->moveNRotations[j];
						ikData->moves[ikData->len].usBurst = 0;
						ikData->len += 1;
					}
				}
			}
		}
	}

	if (currentBestNMoves == MAX_INT32)
	{
		dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
	}
}















//! Trigonometry values to pass to 2-move IK functions. V expensive so calculate only once.
typedef struct TargetIKParams_
{
	float moveDist1;
	float moveDist2;
	float offset1;
	float offset2;
	float orient1PlusDelta;
	float orient1PlusOrient2;
	float orient2PlusDelta;
	float orient2PlusOrient3;
	float cosOrient1;
	float sinOrient1;
	float cosOrient1Plus90;
	float sinOrient1Plus90;
	float cosOrient1PlusDelta;
	float sinOrient1PlusDelta;
	float cosOrient1PlusDeltaPlus90;
	float sinOrient1PlusDeltaPlus90;
	float cosOrient1PlusOrient2;
	float sinOrient1PlusOrient2;
	float cosOrient1PlusOrient2Plus90;
	float sinOrient1PlusOrient2Plus90;
	float cosOrient2PlusDelta;
	float sinOrient2PlusDelta;
	float cosOrient2PlusDeltaPlus90;
	float sinOrient2PlusDeltaPlus90;
	float cosOrient2PlusOrient3;
	float sinOrient2PlusOrient3;
} TargetIKParams;

MatrixF targetCentricCalculateJacobian_2Moves (
	const float targetDist,
	const int rotationIndex1,
	const int rotationIndex2,
	TargetIKParams ikParams,
	const IkConstants *ikConstants)
{
	MatrixF fwdJac;

	// d(p.x)/d(dist1)
	fwdJac.val[0] =
		ikParams.cosOrient1 +
		(1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT  * ikParams.cosOrient1Plus90 +
		ikConstants->rotationOffsets[rotationIndex1].x * -ikParams.sinOrient1PlusDelta * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT + 
		ikConstants->rotationOffsets[rotationIndex1].y * -ikParams.sinOrient1PlusDeltaPlus90 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT + 
		ikParams.moveDist2 * -ikParams.sinOrient1PlusOrient2  * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		ikParams.offset2 * -ikParams.sinOrient1PlusOrient2Plus90  * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		ikConstants->rotationOffsets[rotationIndex2].x * -ikParams.sinOrient2PlusDelta * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		ikConstants->rotationOffsets[rotationIndex2].y * -ikParams.sinOrient2PlusDeltaPlus90 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		targetDist * -ikParams.sinOrient2PlusOrient3 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT;

	// d(p.x)/d(dist2)
	fwdJac.val[1] =
		ikParams.cosOrient1PlusOrient2 +
		(1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT  * ikParams.cosOrient1PlusOrient2Plus90 +
		ikConstants->rotationOffsets[rotationIndex2].x * -ikParams.sinOrient2PlusDelta * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		ikConstants->rotationOffsets[rotationIndex2].y * -ikParams.sinOrient2PlusDeltaPlus90 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		targetDist * -ikParams.sinOrient2PlusOrient3 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT;

	// d(p.y)/d(dist1)
	fwdJac.val[2] =
		ikParams.sinOrient1 +
		(1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT  * ikParams.sinOrient1Plus90 +
		ikConstants->rotationOffsets[rotationIndex1].x * ikParams.cosOrient1PlusDelta * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT + 
		ikConstants->rotationOffsets[rotationIndex1].y * ikParams.cosOrient1PlusDeltaPlus90 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT + 
		ikParams.moveDist2 * ikParams.cosOrient1PlusOrient2  * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		ikParams.offset2 * ikParams.cosOrient1PlusOrient2Plus90  * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		ikConstants->rotationOffsets[rotationIndex2].x * ikParams.cosOrient2PlusDelta * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		ikConstants->rotationOffsets[rotationIndex2].y * ikParams.cosOrient2PlusDeltaPlus90 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		targetDist * ikParams.cosOrient2PlusOrient3 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT;

	// d(p.y)/d(dist2)
	fwdJac.val[3] =
		ikParams.sinOrient1PlusOrient2 +
		(1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT  * ikParams.sinOrient1PlusOrient2Plus90 +
		ikConstants->rotationOffsets[rotationIndex2].x * ikParams.cosOrient2PlusDelta * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		ikConstants->rotationOffsets[rotationIndex2].y * ikParams.cosOrient2PlusDeltaPlus90 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT +
		targetDist * ikParams.cosOrient2PlusOrient3 * (1/MOVE_US_DELTA_FWD) * MOVE_US_DELTA_ORIENT;

	return fwdJac;
}

PointF targetCentricFwdKinematics_2Moves (
						const PoseSimple startPose,
						const float targetDist,
						const int rotationIndex1,
						const int rotationIndex2,
						TargetIKParams ikParams,
						const IkConstants *ikConstants)
{
	PointF pt;

	pt.x =	startPose.loc.x + ikParams.moveDist1 * ikParams.cosOrient1;
	pt.y =	startPose.loc.y + ikParams.moveDist1 * ikParams.sinOrient1;

	pt.x +=	ikParams.offset1 * ikParams.cosOrient1Plus90;
	pt.y +=	ikParams.offset1 * ikParams.sinOrient1Plus90;

	pt.x +=	ikConstants->rotationOffsets[rotationIndex1].x * ikParams.cosOrient1PlusDelta;
	pt.y +=	ikConstants->rotationOffsets[rotationIndex1].x * ikParams.sinOrient1PlusDelta;

	pt.x +=	ikConstants->rotationOffsets[rotationIndex1].y * ikParams.cosOrient1PlusDeltaPlus90;
	pt.y += ikConstants->rotationOffsets[rotationIndex1].y * ikParams.sinOrient1PlusDeltaPlus90;

	pt.x +=	ikParams.moveDist2 * ikParams.cosOrient1PlusOrient2;
	pt.y +=	ikParams.moveDist2 * ikParams.sinOrient1PlusOrient2;

	pt.x +=	ikParams.offset2 * ikParams.cosOrient1PlusOrient2Plus90;
	pt.y +=	ikParams.offset2 * ikParams.sinOrient1PlusOrient2Plus90;

	pt.x += ikConstants->rotationOffsets[rotationIndex2].x * ikParams.cosOrient2PlusDelta;
	pt.y += ikConstants->rotationOffsets[rotationIndex2].x * ikParams.sinOrient2PlusDelta;

	pt.x += ikConstants->rotationOffsets[rotationIndex2].y * ikParams.cosOrient2PlusDeltaPlus90;
	pt.y += ikConstants->rotationOffsets[rotationIndex2].y * ikParams.sinOrient2PlusDeltaPlus90;

	pt.x += ikParams.cosOrient2PlusOrient3 * targetDist;
	pt.y += ikParams.sinOrient2PlusOrient3 * targetDist;

	return pt;
}

void calcIKParams (TargetIKParams *ikParams,
				   const IkConstants *ikConstants,
				   const PoseSimple startPose,
				   const int rotationIndex1,
				   const int rotationIndex2)
{
	float o1, o1Plus90, o1PlusDelta, o1PlusDeltaPlus90, o2, o2Plus90, o2PlusDelta, o2PlusDeltaPlus90, o3;
	int us1 = calcMoveMotorBurst (ikParams->moveDist1);
	int us2 = calcMoveMotorBurst (ikParams->moveDist2);

	ikParams->offset1 = us1 * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT;
	ikParams->offset2 = us2 * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT;
	o1 = startPose.orient;
	ikParams->cosOrient1 = cos (o1);
	ikParams->sinOrient1 = sin (o1);
	o1Plus90 = Geometry_orientSum (o1, PI/2);
	ikParams->cosOrient1Plus90 = cos (o1Plus90);
	ikParams->sinOrient1Plus90 = sin (o1Plus90);
	o1PlusDelta = Geometry_orientSum (o1, us1 * MOVE_US_DELTA_ORIENT + BASE_MOVE_DELTA_ORIENT);
	ikParams->orient1PlusDelta = o1PlusDelta;
	ikParams->cosOrient1PlusDelta = cos (o1PlusDelta);
	ikParams->sinOrient1PlusDelta = sin (o1PlusDelta);
	o1PlusDeltaPlus90 = Geometry_orientSum (o1PlusDelta, PI/2);
	ikParams->cosOrient1PlusDeltaPlus90 = cos (o1PlusDeltaPlus90);
	ikParams->sinOrient1PlusDeltaPlus90 = sin (o1PlusDeltaPlus90);
	o2 = Geometry_orientSum (o1PlusDelta, ikConstants->rotations[rotationIndex1]);
	ikParams->orient1PlusOrient2 = o2;
	ikParams->cosOrient1PlusOrient2 = cos (o2);
	ikParams->sinOrient1PlusOrient2 = sin (o2);
	o2Plus90 = Geometry_orientSum (o2, PI/2);
	ikParams->cosOrient1PlusOrient2Plus90 = cos (o2Plus90);
	ikParams->sinOrient1PlusOrient2Plus90 = sin (o2Plus90);
	o2PlusDelta = Geometry_orientSum (o2, us2 * MOVE_US_DELTA_ORIENT + BASE_MOVE_DELTA_ORIENT);
	ikParams->orient2PlusDelta = o2PlusDelta;
	ikParams->cosOrient2PlusDelta = cos (o2PlusDelta);
	ikParams->sinOrient2PlusDelta = sin (o2PlusDelta);
	o2PlusDeltaPlus90 = Geometry_orientSum (o2PlusDelta, PI/2);
	ikParams->cosOrient2PlusDeltaPlus90 = cos (o2PlusDeltaPlus90);
	ikParams->sinOrient2PlusDeltaPlus90 = sin (o2PlusDeltaPlus90);
	o3 = Geometry_orientSum (o2PlusDelta, ikConstants->rotations[rotationIndex2]);
	ikParams->orient2PlusOrient3 = o3;
	ikParams->cosOrient2PlusOrient3 = cos (o3);
	ikParams->sinOrient2PlusOrient3 = sin (o3);
}

//! Return collision.
int checkIKMoves_2Moves (
						 const Image *navMap,
						 const PoseSimple startPose,
						 const int rotationIndex1,
						 const int rotationIndex2,
						 const float moveDist1,
						 const float moveDist2,
						 const PointF target,
						 const IkConstants *ikConstants)
{
	PointF pt;
	PointF old;
	TargetIKParams ikParams;
	ikParams.moveDist1 = moveDist1;
	ikParams.moveDist2 = moveDist2;
	calcIKParams (&ikParams, ikConstants, startPose, rotationIndex1, rotationIndex2);

	pt.x =	startPose.loc.x + ikParams.moveDist1 * ikParams.cosOrient1;
	pt.y =	startPose.loc.y + ikParams.moveDist1 * ikParams.sinOrient1;

	pt.x +=	ikParams.offset1 * ikParams.cosOrient1Plus90;
	pt.y +=	ikParams.offset1 * ikParams.sinOrient1Plus90;
	if (checkMove (startPose.loc, pt, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
	{
		return 1;
	}
	old = pt;

	pt.x +=	ikConstants->rotationOffsets[rotationIndex1].x * ikParams.cosOrient1PlusDelta;
	pt.y +=	ikConstants->rotationOffsets[rotationIndex1].x * ikParams.sinOrient1PlusDelta;

	pt.x +=	ikConstants->rotationOffsets[rotationIndex1].y * ikParams.cosOrient1PlusDeltaPlus90;
	pt.y += ikConstants->rotationOffsets[rotationIndex1].y * ikParams.sinOrient1PlusDeltaPlus90;
	if (checkMove (old, pt, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
	{
		return 1;
	}
	old = pt;

	pt.x +=	ikParams.moveDist2 * ikParams.cosOrient1PlusOrient2;
	pt.y +=	ikParams.moveDist2 * ikParams.sinOrient1PlusOrient2;

	pt.x +=	ikParams.offset2 * ikParams.cosOrient1PlusOrient2Plus90;
	pt.y +=	ikParams.offset2 * ikParams.sinOrient1PlusOrient2Plus90;
	if (checkMove (old, pt, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
	{
		return 1;
	}
	old = pt;

	pt.x += ikConstants->rotationOffsets[rotationIndex2].x * ikParams.cosOrient2PlusDelta;
	pt.y += ikConstants->rotationOffsets[rotationIndex2].x * ikParams.sinOrient2PlusDelta;

	pt.x += ikConstants->rotationOffsets[rotationIndex2].y * ikParams.cosOrient2PlusDeltaPlus90;
	pt.y += ikConstants->rotationOffsets[rotationIndex2].y * ikParams.sinOrient2PlusDeltaPlus90;
	if (checkMove (old, pt, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
	{
		return 1;
	}

	// Just check LOS
	if (checkMove (pt, target, navMap, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP))
	{
		return 1;
	}
	return 0;
}

int targetCentricOptimumMoveDist_2Moves (
	const PoseSimple startPose,
	const Dest *dest,
	const int rotationIndex1,
	const int rotationIndex2,
	float *move1,
	float *move2,
	float *moveErrorSqrd,
	const GeometryConstants *geometryConstants,
	const IkConstants *ikConstants,
	FILE *f)
{
	int nIters = 0;
	MatrixF jac, jacInv;
	PointF optimumPt, targetError;
	PointF bestValidDists;
	const float leewaySqrd = dest->leeway * dest->leeway;
	const float allowedErrorRange = ikConstants->moveRange * 2.0f;
	float bestErrorSqrd = MAX_FLT;
	float errorSqrd = MAX_FLT;
	float lastErrorSqrd = MAX_FLT - 10.0f;
	float step;
	float lastMoveDist1, lastMoveDist2;
	TargetIKParams ikParams;
	//int dummy;

	lastMoveDist1 = MAX_FLT;
	lastMoveDist2 = MAX_FLT;
	ikParams.moveDist1 = ikConstants->moveMedian;
	ikParams.moveDist2 = ikConstants->moveMedian;

	while (1)
	{
		if (nIters++ > 100)
		{
#ifdef PRINT_IK_DETAIL
//			printf ("Target-centric IK iteration exceeded!!!\n");
			if (f)
			{
				fprintf (f, "<IKIterationExceeded>type=\"targetCentric2Moves\" pose=(%f,%f,%f) target=(%f,%f) rot1=%d rot2=%d</IKIterationExceeded>\n", startPose.loc.x, startPose.loc.y, startPose.orient, dest->target.x, dest->target.y, rotationIndex1, rotationIndex2);
			}
#endif
			break;
		}

		calcIKParams (&ikParams, ikConstants, startPose, rotationIndex1, rotationIndex2);

		optimumPt = targetCentricFwdKinematics_2Moves (
			startPose,
			dest->targetDist,
			rotationIndex1,
			rotationIndex2,
			ikParams,
			ikConstants);
	
		lastErrorSqrd = errorSqrd;

		targetError.x = dest->target.x - optimumPt.x;
		targetError.y = dest->target.y - optimumPt.y;

		errorSqrd = fabs (targetError.x * targetError.y);
		if (fabs (errorSqrd - lastErrorSqrd) < 0.005f)
		{
			break;
		}

		// Record best valid input, in case IK leads outside of valid bounds.
		if (errorSqrd < bestErrorSqrd &&
			ikParams.moveDist1 <= geometryConstants->nav_maxDist && ikParams.moveDist1 >= BASE_MOVE_DELTA_FWD &&
			ikParams.moveDist2 <= geometryConstants->nav_maxDist && ikParams.moveDist2 >= BASE_MOVE_DELTA_FWD)
		{
			bestValidDists.x = ikParams.moveDist1;
			bestValidDists.y = ikParams.moveDist2;
			bestErrorSqrd = errorSqrd;
		}

		step = 0.7f;
		if (errorSqrd > 1.0f)
		{
			step = 0.7f / errorSqrd;
		}

		targetError.x *= step;
		targetError.y *= step;

		jac = targetCentricCalculateJacobian_2Moves (
			dest->targetDist,
			rotationIndex1,
			rotationIndex2,
			ikParams,
			ikConstants);

		jacInv = MatrixF_inverse (jac);

		lastMoveDist1 = ikParams.moveDist1;
		lastMoveDist2 = ikParams.moveDist2;

		ikParams.moveDist1 += jacInv.val[0] * targetError.x + jacInv.val[1] * targetError.y;
		ikParams.moveDist2 += jacInv.val[2] * targetError.x + jacInv.val[3] * targetError.y;

		//calcClosestMotorDist (ikParams.moveDist1, &ikParams.moveDist1, &dummy);
		//calcClosestMotorDist (ikParams.moveDist2, &ikParams.moveDist2, &dummy);

		//if (lastMoveDist1 == ikParams.moveDist1 && lastMoveDist2 == ikParams.moveDist2)
		if (0.05f > fabs (lastMoveDist1 - ikParams.moveDist1) && 0.05f > fabs (lastMoveDist2 - ikParams.moveDist2))
		{
			break;
		}

		if (fabs (ikParams.moveDist1 - ikConstants->moveMedian) > allowedErrorRange ||
			fabs (ikParams.moveDist2 - ikConstants->moveMedian) > allowedErrorRange)
		{
			break;
		}
	}

	if (bestErrorSqrd < leewaySqrd)
	{
		*move1 = bestValidDists.x;
		*move2 = bestValidDists.y;
		*moveErrorSqrd = bestErrorSqrd;
		return 1;
	}
	return 0;
}

void print2MovePoses (
	FILE *f,
	const PoseSimple startPose,
	const int rotationIndex1,
	const int rotationIndex2,
	const float targetDist,
	const float moveDist1,
	const float moveDist2,
	const PointF target,
	const IkConstants *ikConstants)
{
	PointF pt;
	TargetIKParams ikParams;
	float d;

	ikParams.moveDist1 = moveDist1;
	ikParams.moveDist2 = moveDist2;

	calcIKParams (&ikParams, ikConstants, startPose, rotationIndex1, rotationIndex2);

#ifdef PRINT_IK_DETAIL
	fprintf (f, "<TargetCentricMove>ik=2Move startPose=(%f,%f,%f) ",
		startPose.loc.x, startPose.loc.y, startPose.orient);
#endif

	pt.x =	startPose.loc.x + ikParams.moveDist1 * ikParams.cosOrient1;
	pt.y =	startPose.loc.y + ikParams.moveDist1 * ikParams.sinOrient1;

	pt.x +=	ikParams.offset1 * ikParams.cosOrient1Plus90;
	pt.y +=	ikParams.offset1 * ikParams.sinOrient1Plus90;

#ifdef PRINT_IK_DETAIL
	fprintf (f, "afterMove=(%f,%f,%f) ", pt.x, pt.y, ikParams.orient1PlusDelta);
#endif

	pt.x +=	ikConstants->rotationOffsets[rotationIndex1].x * ikParams.cosOrient1PlusDelta;
	pt.y +=	ikConstants->rotationOffsets[rotationIndex1].x * ikParams.sinOrient1PlusDelta;

	pt.x +=	ikConstants->rotationOffsets[rotationIndex1].y * ikParams.cosOrient1PlusDeltaPlus90;
	pt.y += ikConstants->rotationOffsets[rotationIndex1].y * ikParams.sinOrient1PlusDeltaPlus90;

#ifdef PRINT_IK_DETAIL
	fprintf (f, "afterRotate=(%f,%f,%f) ", pt.x, pt.y, ikParams.orient1PlusOrient2);
#endif

	pt.x +=	ikParams.moveDist2 * ikParams.cosOrient1PlusOrient2;
	pt.y +=	ikParams.moveDist2 * ikParams.sinOrient1PlusOrient2;

	pt.x +=	ikParams.offset2 * ikParams.cosOrient1PlusOrient2Plus90;
	pt.y +=	ikParams.offset2 * ikParams.sinOrient1PlusOrient2Plus90;

#ifdef PRINT_IK_DETAIL
	fprintf (f, "afterMove2=(%f,%f,%f) ", pt.x, pt.y, ikParams.orient2PlusDelta);
#endif

	pt.x += ikConstants->rotationOffsets[rotationIndex2].x * ikParams.cosOrient2PlusDelta;
	pt.y += ikConstants->rotationOffsets[rotationIndex2].x * ikParams.sinOrient2PlusDelta;

	pt.x += ikConstants->rotationOffsets[rotationIndex2].y * ikParams.cosOrient2PlusDeltaPlus90;
	pt.y += ikConstants->rotationOffsets[rotationIndex2].y * ikParams.sinOrient2PlusDeltaPlus90;

#ifdef PRINT_IK_DETAIL
	fprintf (f, "afterRotate2=(%f,%f,%f) ", pt.x, pt.y, ikParams.orient2PlusOrient3);
#endif

	pt.x += ikParams.cosOrient2PlusOrient3 * targetDist;
	pt.y += ikParams.sinOrient2PlusOrient3 * targetDist;

#ifdef PRINT_IK_DETAIL
	fprintf (f, "optimumPt=(%f,%f)", pt.x, pt.y);
#endif

	d = Geometry_dist (pt.x, pt.y, target.x, target.y);

#ifdef PRINT_IK_DETAIL
	fprintf (f, "error=%f", d);
	fprintf (f, "</TargetCentricMove>\n");
#endif
}

void targetCentricIK_2Moves (
	FILE *f,
	const Image *navMap,
	const Pose *current,
	Dest *dest,
	IKData *ikData,
	const GeometryConstants *geometryConstants,
	const IkConstants *ikConstants,
	const int verbose)
{
	int i, j, k;
	PoseSimple tempPose1, tempPose2, tempPose3, tempPose4, tempPose5;
	PointF optimumPt;
	float currentOrientPlus90 = Geometry_orientSum (current->orient, PI/2);
	float tempPose2OrientPlus90;
	float tempPose4OrientPlus90;
	float distSqrd;
	float moveDist1, moveDist2;
	float targetDistSqrd = dest->targetDist * dest->targetDist;
	float initialLeewaySqrd = (ikConstants->moveRange * 2.0f + 1.0f) * (ikConstants->moveRange * 2.0f + 1.0f);
	float moveErrorSqrd;
	int nMoves;
	float currentBestMoveErrorSqrd = MAX_FLT;
	int currentBestNMoves = MAX_INT32;

	// Rotate at start pose.
	for (i = 0; i < N_ROTATIONS_IN_TOTAL; ++i)
	{
		Geometry_ptFromOrient (current->loc.x, current->loc.y, &tempPose1.loc.x, &tempPose1.loc.y, ikConstants->rotationOffsets[i].x, current->orient);
		Geometry_ptFromOrient (tempPose1.loc.x, tempPose1.loc.y, &tempPose1.loc.x, &tempPose1.loc.y, ikConstants->rotationOffsets[i].y, currentOrientPlus90);
		tempPose1.orient = Geometry_orientSum (current->orient, ikConstants->rotations[i]);
		if (checkMove (current->loc, tempPose1.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
		{
			continue;
		}

		Geometry_ptFromOrient (tempPose1.loc.x, tempPose1.loc.y, &tempPose2.loc.x, &tempPose2.loc.y, ikConstants->moveMedian, tempPose1.orient);
		Geometry_ptFromOrient (tempPose2.loc.x, tempPose2.loc.y, &tempPose2.loc.x, &tempPose2.loc.y, ikConstants->offsetMedian, Geometry_orientSum (tempPose1.orient, PI/2));
		tempPose2.orient = Geometry_orientSum (tempPose1.orient, ikConstants->orientChangeMedian);
		tempPose2OrientPlus90 = Geometry_orientSum (tempPose2.orient, PI/2);
		if (checkMove (tempPose1.loc, tempPose2.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
		{
			continue;
		}

		// Rotate after making move.
		for (j = 1; j < N_ROTATIONS_IN_TOTAL; ++j)
		{
			Geometry_ptFromOrient (tempPose2.loc.x, tempPose2.loc.y, &tempPose3.loc.x, &tempPose3.loc.y, ikConstants->rotationOffsets[j].x, tempPose2.orient);
			Geometry_ptFromOrient (tempPose3.loc.x, tempPose3.loc.y, &tempPose3.loc.x, &tempPose3.loc.y, ikConstants->rotationOffsets[j].y, tempPose2OrientPlus90);
			tempPose3.orient = Geometry_orientSum (tempPose2.orient, ikConstants->rotations[j]);
			if (checkMove (tempPose2.loc, tempPose3.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
			{
				continue;
			}

			Geometry_ptFromOrient (tempPose3.loc.x, tempPose3.loc.y, &tempPose4.loc.x, &tempPose4.loc.y, ikConstants->moveMedian, tempPose3.orient);
			Geometry_ptFromOrient (tempPose4.loc.x, tempPose4.loc.y, &tempPose4.loc.x, &tempPose4.loc.y, ikConstants->offsetMedian, Geometry_orientSum (tempPose3.orient, PI/2));
			tempPose4.orient = Geometry_orientSum (tempPose3.orient, ikConstants->orientChangeMedian);
			if (checkMove (tempPose3.loc, tempPose4.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
			{
				continue;
			}

			// After 2 moves, pose should be within leeway of targetDist from target.
			distSqrd = Geometry_distSqrd (tempPose4.loc.x, tempPose4.loc.y, dest->target.x, dest->target.y);

			if (fabs (distSqrd - targetDistSqrd) > initialLeewaySqrd)
			{
				continue;
			}

			tempPose4OrientPlus90 = Geometry_orientSum (tempPose4.orient, PI/2);

			// Rotate after making second move, i.e., at end pose.
			for (k = 0; k < N_ROTATIONS_IN_TOTAL; ++k)
			{
				Geometry_ptFromOrient (tempPose4.loc.x, tempPose4.loc.y, &tempPose5.loc.x, &tempPose5.loc.y, ikConstants->rotationOffsets[k].x, tempPose4.orient);
				Geometry_ptFromOrient (tempPose5.loc.x, tempPose5.loc.y, &tempPose5.loc.x, &tempPose5.loc.y, ikConstants->rotationOffsets[k].y, tempPose4OrientPlus90);
				tempPose5.orient = Geometry_orientSum (tempPose4.orient, ikConstants->rotations[k]);
				if (checkMove (tempPose4.loc, tempPose5.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
				{
					continue;
				}

				Geometry_ptFromOrient (tempPose5.loc.x, tempPose5.loc.y, &optimumPt.x, &optimumPt.y, dest->targetDist, tempPose5.orient);

				// Check that point at optimum distance is within broad leeway of target.
				distSqrd = Geometry_distSqrd (optimumPt.x, optimumPt.y, dest->target.x, dest->target.y);

				if (distSqrd > initialLeewaySqrd)
				{
					continue;
				}

				if (targetCentricOptimumMoveDist_2Moves (tempPose1, dest, j, k, &moveDist1, &moveDist2, &moveErrorSqrd, geometryConstants, ikConstants, f))
				{
					if (checkIKMoves_2Moves (navMap, tempPose1, j, k, moveDist1, moveDist2, dest->target, ikConstants))
					{
						continue;
					}

					nMoves = 2 + (int)(0 != ikConstants->moveDirs[i]) + (int)(0 != ikConstants->moveDirs[j]) + (int)(0 != ikConstants->moveDirs[k]);

					if (verbose)
					{
						print2MovePoses (f, tempPose1, j, k, dest->targetDist, moveDist1, moveDist2, dest->target, ikConstants);
					}

					if (nMoves < currentBestNMoves || (nMoves == currentBestNMoves && moveErrorSqrd < currentBestMoveErrorSqrd))
					{
						currentBestNMoves = nMoves;
						currentBestMoveErrorSqrd = moveErrorSqrd;

						ikData->index = 0;
						ikData->len = 0;

						if (0 != ikConstants->moveDirs[i])
						{
							ikData->moves[0].dir = ikConstants->moveDirs[i];
							ikData->moves[0].n = ikConstants->moveNRotations[i];
							ikData->moves[0].usBurst = 0;
							ikData->len = 1;
						}

						ikData->moves[ikData->len].dir = 0;
						ikData->moves[ikData->len].moveDist = moveDist1;
						ikData->moves[ikData->len].usBurst = (moveDist1 - BASE_MOVE_DELTA_FWD) * (1 / MOVE_US_DELTA_FWD);
						ikData->moves[ikData->len].n = 1;
						DEBUG_ASSERT(ikData->moves[ikData->len].usBurst < 10000000)
						++ikData->len;

						if (0 != ikConstants->moveDirs[j])
						{
							ikData->moves[ikData->len].dir = ikConstants->moveDirs[j];
							ikData->moves[ikData->len].n = ikConstants->moveNRotations[j];
							ikData->moves[ikData->len].usBurst = 0;
							ikData->len += 1;
						}

						ikData->moves[ikData->len].dir = 0;
						ikData->moves[ikData->len].moveDist = moveDist2;
						ikData->moves[ikData->len].usBurst = (moveDist2 - BASE_MOVE_DELTA_FWD) * (1 / MOVE_US_DELTA_FWD);
						ikData->moves[ikData->len].n = 1;
						DEBUG_ASSERT(ikData->moves[ikData->len].usBurst < 10000000)
						++ikData->len;

						if (0 != ikConstants->moveDirs[k])
						{
							ikData->moves[ikData->len].dir = ikConstants->moveDirs[k];
							ikData->moves[ikData->len].n = ikConstants->moveNRotations[k];
							ikData->moves[ikData->len].usBurst = 0;
							ikData->len += 1;
						}
					}
				}
			}
		}
	}

	if (currentBestNMoves == MAX_INT32)
	{
		dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
	}
}

















extern float getMoveDistWithinLocalMap (const PointF pt1, const PointF pt2, const float moveDist, const Image *navMap);
extern float getMoveDistWithinEnvironment (const PointF pt1, const PointF pt2, const float moveDist);

//! Calculate move based on simple hill climbing.
void targetCentricCoarseMove (
	const Image *navMap,
	const Pose *current,
	Dest *dest,
	IKData *ikData,
	const GeometryConstants *geometryConstants,
	const float targetDist,
	const float desiredMoveDist)
{
	Pose nextPose;
	Pose subsequentPose;
	PointF optimumPt;
	Move nextMove;
	Move subsequentMove;
	float nextDist;
	float subsequentDist;
	float leewaySqrd = dest->leeway * dest->leeway;

	float moveDist, verifiedMoveDist;
	float us;
	float nextMoveDist;
	float orientToMove;
	float nextOrientToMove;
	float moveOffset;
	float nextMoveOffset;
	float orientDiff;

	ikData->len = 1;
	ikData->index = 0;

	// May be able to achieve optimum pt with a rotation.
	EXPLICIT_DEBUG_ASSERT(geometryConstants->areExpDistsSetup == 1)
	if (fabs (targetDist - dest->targetDist) < geometryConstants->nav_maxDist)
	{
		nextPose = *current;

		Geometry_ptFromOrient (nextPose.loc.x, nextPose.loc.y, &optimumPt.x, &optimumPt.y, dest->targetDist, nextPose.orient);
		nextDist = Geometry_distSqrd (optimumPt.x, optimumPt.y, dest->target.x, dest->target.y);

		nextMove.dir = -1;
		while (1)
		{
			// Try to left.
			subsequentPose = nextPose;
			subsequentMove.dir = 2;
			Actuator_executeMove (&subsequentPose, subsequentMove, 0, 0, 0);
			Geometry_ptFromOrient (subsequentPose.loc.x, subsequentPose.loc.y, &optimumPt.x, &optimumPt.y, dest->targetDist, subsequentPose.orient);
			subsequentDist = Geometry_distSqrd (optimumPt.x, optimumPt.y, dest->target.x, dest->target.y);

			if (subsequentDist < nextDist && !checkMove (nextPose.loc, subsequentPose.loc, navMap, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP))
			{
				if (-1 == nextMove.dir)
				{
					nextMove.dir = subsequentMove.dir;
					nextMove.n = 1;
				}
				nextPose = subsequentPose;
				nextDist = subsequentDist;
				continue;
			}

			// Right.
			subsequentPose = nextPose;
			subsequentMove.dir = 3;
			Actuator_executeMove (&subsequentPose, subsequentMove, 0, 0, 0);
			Geometry_ptFromOrient (subsequentPose.loc.x, subsequentPose.loc.y, &optimumPt.x, &optimumPt.y, dest->targetDist, subsequentPose.orient);
			subsequentDist = Geometry_distSqrd (optimumPt.x, optimumPt.y, dest->target.x, dest->target.y);

			if (subsequentDist < nextDist && !checkMove (nextPose.loc, subsequentPose.loc, navMap, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP))
			{
				if (-1 == nextMove.dir)
				{
					nextMove.dir = subsequentMove.dir;
					nextMove.n = 1;
				}
				nextPose = subsequentPose;
				nextDist = subsequentDist;
				continue;
			}

			// Rotation will not bring optimum point closer to target.
			break;
		}

		// Just check LOS to see if target can be achieved from here.
		// (Target centric IK targets should ALWAYS be within the robot's local map)
		if (nextDist < leewaySqrd && !checkMove (nextPose.loc, dest->target, navMap, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP))
		{
			ikData->moves[0] = nextMove;
			return;
		}
	}

	// Cannot achieve target without moving...

	moveDist = min (geometryConstants->nav_maxDist, desiredMoveDist);
	moveDist = max (BASE_MOVE_DELTA_FWD, moveDist);
	us = calcMoveMotorBurst (moveDist);
	//calcClosestMotorDist (moveDist, &motorDist, &motorBurst);

	// Take into account any lateral shift that the robot experiences when moving forward.
	moveOffset = BASE_MOVE_DELTA_LAT + us * MOVE_US_DELTA_LAT; // Offset
	moveOffset = atan (moveOffset / moveDist); // angle

	// Determine if heading is ok.
	orientToMove = Geometry_orient (current->loc.x, current->loc.y, dest->target.x, dest->target.y);
	if (targetDist < dest->targetDist)
	{
		orientToMove = Geometry_orientSum (orientToMove, PI);
	}
	orientToMove = Geometry_orientSum (orientToMove, moveOffset);
	orientDiff = Geometry_orientDiff (current->orient, orientToMove, 1);

	// Determine if rotation would be beneficial.
	if (orientDiff >= (float)ROT_AVG_DELTA_ORIENT/2.0f)
	{
		// Try left.
		nextPose = *current;
		nextMove.dir = 2;
		Actuator_executeMove (&nextPose, nextMove, 0, 0, 0);

		nextMoveDist = fabs (Geometry_dist (nextPose.loc.x, nextPose.loc.y, dest->target.x, dest->target.y) - dest->targetDist);
		nextMoveDist = min (geometryConstants->nav_maxDist, nextMoveDist);
		nextMoveDist = min (desiredMoveDist, nextMoveDist);
		nextMoveDist = max (BASE_MOVE_DELTA_FWD, nextMoveDist);
		if (nextMoveDist == moveDist)
		{
			nextMoveOffset = moveOffset;
		}
		else
		{
			us = calcMoveMotorBurst (nextMoveDist);
			//calcClosestMotorDist (nextMoveDist, &motorDist, &motorBurst);
			nextMoveOffset = BASE_MOVE_DELTA_LAT + us * MOVE_US_DELTA_LAT;						// offset
			nextMoveOffset = atan (nextMoveOffset / nextMoveDist);						// angle
		}
		nextOrientToMove = Geometry_orient (nextPose.loc.x, nextPose.loc.y, dest->target.x, dest->target.y);
		if (Geometry_distSqrd (nextPose.loc.x, nextPose.loc.y, dest->target.x, dest->target.y) < dest->targetDist * dest->targetDist)
		{
			nextOrientToMove = Geometry_orientSum (nextOrientToMove, PI);
		}
		nextOrientToMove = Geometry_orientSum (nextOrientToMove, nextMoveOffset);
		if (Geometry_orientDiff (nextPose.orient, nextOrientToMove, 1) < orientDiff)
		{
			ikData->moves[0] = initMoveParams (1, nextMove.dir, 0.0f, 0);
			return;
		}

		// Right
		nextPose = *current;
		nextMove.dir = 3;
		Actuator_executeMove (&nextPose, nextMove, 0, 0, 0);

		nextMoveDist = fabs (Geometry_dist (nextPose.loc.x, nextPose.loc.y, dest->target.x, dest->target.y) - dest->targetDist);
		nextMoveDist = min (geometryConstants->nav_maxDist, nextMoveDist);
		nextMoveDist = min (desiredMoveDist, nextMoveDist);
		nextMoveDist = max (BASE_MOVE_DELTA_FWD, nextMoveDist);
		if (nextMoveDist == moveDist)
		{
			nextMoveOffset = moveOffset;
		}
		else
		{
			us = calcMoveMotorBurst (nextMoveDist);
			//calcClosestMotorDist (nextMoveDist, &motorDist, &motorBurst);
			nextMoveOffset = BASE_MOVE_DELTA_LAT + us * MOVE_US_DELTA_LAT;						// offset
			nextMoveOffset = atan (nextMoveOffset / nextMoveDist);						// angle
		}
		nextOrientToMove = Geometry_orient (nextPose.loc.x, nextPose.loc.y, dest->target.x, dest->target.y);
		if (Geometry_distSqrd (nextPose.loc.x, nextPose.loc.y, dest->target.x, dest->target.y) < dest->targetDist * dest->targetDist)
		{
			nextOrientToMove = Geometry_orientSum (nextOrientToMove, PI);
		}
		nextOrientToMove = Geometry_orientSum (orientToMove, nextMoveOffset);
		if (Geometry_orientDiff (nextPose.orient, nextOrientToMove, 1) < orientDiff)
		{
			ikData->moves[0] = initMoveParams (1, nextMove.dir, 0.0f, 0);
			return;
		}

		// Rotation is not beneficial, so go with fwd movement.
	}

	// Ensure this move will not result in the robot going outside the environment or local map
	us = calcMoveMotorBurst (moveDist);
	DEBUG_ASSERT(us < 10000000)
	nextMove = initMoveParams (1, 0, moveDist, us);
	nextPose = *current;
	Actuator_executeMove (&nextPose, nextMove, 0, 0, 0);

	if (l_allowOutsideLocalMap)
	{
		verifiedMoveDist = getMoveDistWithinEnvironment (current->loc, nextPose.loc, moveDist);
	}
	else
	{
		verifiedMoveDist = getMoveDistWithinLocalMap (current->loc, nextPose.loc, moveDist, navMap);
	}
	if (0 == verifiedMoveDist)
	{
		dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
		return;
	}
	if (verifiedMoveDist != moveDist)
	{
		us = calcMoveMotorBurst (verifiedMoveDist);
		DEBUG_ASSERT(us < 10000000)
		nextMove = initMoveParams (1, 0, verifiedMoveDist, us);
		nextPose = *current;
		Actuator_executeMove (&nextPose, nextMove, 0, 0, 0);
	}

	ikData->moves[0] = nextMove;

	// Determine if movement will bring robot closer to targetDist

	// Could alternatively determine if optimumPt-target is closer, but would have to handle case
	// where robot is moving away from target.
	nextDist = Geometry_distSqrd (nextPose.loc.x, nextPose.loc.y, dest->target.x, dest->target.y);
	nextDist = fabs (nextDist - dest->targetDist * dest->targetDist);
	subsequentDist = fabs (targetDist * targetDist - dest->targetDist * dest->targetDist);

	if (nextDist > subsequentDist) // Actually the original dist, but don't bother with a local var
	{
		dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
		return;
	}

	if (checkMove (current->loc, nextPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, l_allowOutsideLocalMap))
	{
		dest->type |= IS_PATH_REQUIRED;
		return;
	}
}

















void IK_targetCentricAccurateMove (FILE *f,
								   const Image *navMap,
								   const Pose *pose,
								   Dest *dest,
								   IKData *ikData,
								   const GeometryConstants *geometryConstants,
								   const IkConstants *ikConstants,
								   const BEHAVIOUR behaviour,
								   const int verbose,
								   const int allowOutsideLocalMap,
								   const int isImplemented,
								   const int simpleIkOnly,
								   const int printLogic)
{
	float targetDist, distOffset;
	float optimumPtDist;
	Pose tempPose;
	PointF optimumPt;
	IKData tempIkData;
	float leewaySqrd = dest->leeway * dest->leeway;

	l_allowOutsideLocalMap = allowOutsideLocalMap;

	// Check if current pose is acceptable
	// (This would be caught anyway, but better to check here in case logic within function
	// changes in future).
	{
		Geometry_ptFromOrient (pose->loc.x, pose->loc.y, &optimumPt.x, &optimumPt.y, dest->targetDist, pose->orient);
		optimumPtDist = Geometry_distSqrd (optimumPt.x, optimumPt.y, dest->target.x, dest->target.y);
		if (optimumPtDist < leewaySqrd && !checkMove (pose->loc, dest->target, navMap, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP))
		{
			ikData->index = 0;
			ikData->len = 0;
			dest->type |= IS_AT_DEST;
			return;
		}
	}

	if (printLogic)
	{
#ifdef PRINT_EVENTS
		fprintf (f, "<CheckState>event=\"notAtDest\" dist=%f leeway=%f pose=(%f,%f,%f) pt=(%f,%f) target=(%f,%f)</CheckState>\n", optimumPtDist, leewaySqrd, pose->loc.x, pose->loc.y, pose->orient, optimumPt.x, optimumPt.y, dest->target.x, dest->target.y);
#endif
	}

	targetDist = Geometry_dist (pose->loc.x, pose->loc.y, dest->target.x, dest->target.y);
	distOffset = fabs (targetDist - dest->targetDist);
	if (distOffset > (geometryConstants->nav_maxDist * 1.8f))
	{
		if (printLogic)
		{
#ifdef PRINT_EVENTS
			fprintf (f, "<CheckState>event=\"destTooFar\" dist=%f</CheckState>\n", targetDist);
#endif
		}

		dest->type |= IS_PATH_REQUIRED;
		return;
	}

	if (distOffset < geometryConstants->nav_maxDist)
	{
		targetCentricIK_1Move (f, navMap, dest, pose, ikData, geometryConstants, ikConstants, verbose);
		if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE))
		{
			return;
		}
		dest->type &= ~IS_IK_CURRENTLY_IMPOSSIBLE;
	}

	if (simpleIkOnly)
	{
		if (printLogic)
		{
#ifdef PRINT_EVENTS
			fprintf (f, "<CheckState>event=\"simpleIkFailed\"</CheckState>\n");
#endif
		}
		dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
		return;
	}

	targetCentricIK_2Moves (f, navMap, pose, dest, ikData, geometryConstants, ikConstants, verbose);
	if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE))
	{
		return;
	}
	dest->type &= ~IS_IK_CURRENTLY_IMPOSSIBLE;

	// Attempt from current pose did not work (due to movement restrictions).
	// Calculate a path to a potentially suitable dest (from which to view target)
	// to avoid making to many "nudge" moves.
	if (distOffset > geometryConstants->nav_maxDist * 0.8f)
	{
		if (printLogic)
		{
#ifdef PRINT_EVENTS
			fprintf (f, "<CheckState>event=\"avoidNudgesCalcPath\" dist=%f</CheckState>\n", targetDist);
#endif
		}
		dest->type |= IS_PATH_REQUIRED;
		return;
	}

	// If a sequence of short moves will bring robot to a location where IK
	// can succeed, then return the first of these.
	tempPose = *pose;
	targetCentricCoarseMove (navMap, &tempPose, dest, ikData, geometryConstants, targetDist, BASE_MOVE_DELTA_FWD);
	if (dest->type & IS_IK_CURRENTLY_IMPOSSIBLE || dest->type & IS_PATH_REQUIRED)
	{
		if (printLogic)
		{
#ifdef PRINT_EVENTS
		fprintf (f, "<CheckState>event=\"firstTargetCentricNudgeFail\"</CheckState>\n");
#endif
		}
		return;
	}
	Actuator_executeMove (&tempPose, ikData->moves[0], 0, 0, 0);
#ifdef PRINT_IK_DETAIL
	if (verbose)
	{
		fprintf (f, "<TargetCentricMove>ik=nudge tempPose=(%f,%f,%f)</TargetCentricMove>\n",
			tempPose.loc.x, tempPose.loc.y, tempPose.orient);
	}
#endif
	while (1)
	{
		Geometry_ptFromOrient (tempPose.loc.x, tempPose.loc.y, &optimumPt.x, &optimumPt.y, dest->targetDist, tempPose.orient);
		optimumPtDist = Geometry_distSqrd (optimumPt.x, optimumPt.y, dest->target.x, dest->target.y);

		// Just check LOS
		if (optimumPtDist < leewaySqrd && !checkMove (tempPose.loc, dest->target, navMap, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP))
		{
			return;
		}

		targetDist = Geometry_dist (tempPose.loc.x, tempPose.loc.y, dest->target.x, dest->target.y);
		if (fabs (targetDist - dest->targetDist) < geometryConstants->nav_maxDist)
		{
			targetCentricIK_1Move (f, navMap, dest, &tempPose, &tempIkData, geometryConstants, ikConstants, verbose);
			if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE))
			{
				// These are from a temporary pose, so "isAtDest" isn't really valid.
				dest->type &= ~IS_AT_DEST;
				return;
			}
			dest->type &= ~IS_IK_CURRENTLY_IMPOSSIBLE;
		}

		targetCentricIK_2Moves (f, navMap, &tempPose, dest, &tempIkData, geometryConstants, ikConstants, verbose);
		if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE))
		{
			dest->type &= ~IS_AT_DEST;
			return;
		}
		dest->type &= ~IS_IK_CURRENTLY_IMPOSSIBLE;

		targetCentricCoarseMove (navMap, &tempPose, dest, &tempIkData, geometryConstants, targetDist, BASE_MOVE_DELTA_FWD);
		if (dest->type & IS_IK_CURRENTLY_IMPOSSIBLE || dest->type & IS_PATH_REQUIRED)
		{
			return;
		}
		Actuator_executeMove (&tempPose, tempIkData.moves[0], 0, 0, 0);

#ifdef PRINT_IK_DETAIL
		if (verbose)
		{
			fprintf (f, "<TargetCentricMove>ik=nudge tempPose=(%f,%f,%f)</TargetCentricMove>\n",
				tempPose.loc.x, tempPose.loc.y, tempPose.orient);
		}
#endif
	}

	return;
}



