#include "../../Common/RobotDefs.h"



#include "DestCentric.h"
#include "../../Common/Geometry.h"
#include "../../Common/Maths.h"

void destCentricIK_1Move (
	const Image *navMap,
	const Pose *current,
	Dest *dest,
	IKData *ikData,
	const GeometryConstants *geometryConstants,
	const IkConstants *ikConstants,
	const int allowOutsideLocalMap)
{
	PoseSimple startPose;
	PoseSimple endPose;
	int i;
	int j;
	int startMoveDir;
	int endMoveDir;
	int startMoveNRotations;
	int endMoveNRotations;
	Vector3F dir;
	Vector3F destOffset;
	PoseSimple temp;
	float dotProdDist, dist, distSqrd;
	float us;
	float leewaySqrd = dest->leeway * dest->leeway;
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
			Geometry_orientSum (current->orient, PI/2));

		if (checkMove (current->loc, startPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
		{
			continue;
		}

		startPose.orient = Geometry_orientSum (current->orient, ikConstants->rotations[i]);
		startMoveDir = ikConstants->moveDirs[i];
		startMoveNRotations = ikConstants->moveNRotations[i];

		for (j = 0; j < N_ROTATIONS_IN_TOTAL; ++j)
		{
			// Moving backwards from dest, so subtract offsets.
			Geometry_ptFromOrient (
				dest->dest.loc.x,
				dest->dest.loc.y,
				&endPose.loc.x,
				&endPose.loc.y,
				-ikConstants->rotationOffsets[j].x,
				dest->dest.orient);

			Geometry_ptFromOrient (
				endPose.loc.x,
				endPose.loc.y,
				&endPose.loc.x,
				&endPose.loc.y,
				-ikConstants->rotationOffsets[j].y,
				Geometry_orientSum (dest->dest.orient, PI/2));

			if (checkMove (dest->dest.loc, endPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
			{
				continue;
			}

			endPose.orient = Geometry_orientSum (dest->dest.orient, -ikConstants->rotations[j]);
			endMoveDir = ikConstants->moveDirs[j];
			endMoveNRotations = ikConstants->moveNRotations[j];

			// Determine if rotation without a forward move will achieve the destination.
			distSqrd = Geometry_distSqrd (startPose.loc.x, startPose.loc.y, endPose.loc.x, endPose.loc.y);
			if (distSqrd < leewaySqrd && Geometry_orientDiff (startPose.orient, endPose.orient, 1) < ROT_AVG_DELTA_ORIENT/2.0f)
			{
				if (!checkMove (startPose.loc, endPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
				{
					ikData->index = 0;
					ikData->len = 0;

					if (0 == startMoveDir && 0 == endMoveDir)
					{
						dest->type |= IS_AT_DEST;
						return;
					}

					nMoves = (int)(0 != startMoveDir) + (int)(0 != endMoveDir);

					if (nMoves < currentBestNMoves || (nMoves == currentBestNMoves && distSqrd < currentBestMoveErrorSqrd))
					{
						currentBestNMoves = nMoves;
						currentBestMoveErrorSqrd = distSqrd;

						if (0 != startMoveDir)
						{
							ikData->moves[0].dir = startMoveDir;
							ikData->moves[0].n = startMoveNRotations;
							ikData->moves[0].usBurst = 0;
							ikData->len = 1;
						}

						if (0 != endMoveDir)
						{
							ikData->moves[ikData->len].dir = endMoveDir;
							ikData->moves[ikData->len].n = endMoveNRotations;
							ikData->moves[ikData->len].usBurst = 0;
							ikData->len += 1;
						}
					}
				}
			}

			// Determine direction in which robot is pointing
			Geometry_ptFromOrient (startPose.loc.x, startPose.loc.y, &temp.loc.x, &temp.loc.y, 1.0f, startPose.orient);
			dir.x = temp.loc.x - startPose.loc.x;
			dir.y = temp.loc.y - startPose.loc.y;
			dir.z = 0;

			// Determine relative loc of dest pt
			destOffset.x = endPose.loc.x - startPose.loc.x;
			destOffset.y = endPose.loc.y - startPose.loc.y;
			destOffset.z = 0;

			dotProdDist = Vector3F_dot(dir, destOffset);
			if (dotProdDist >= BASE_MOVE_DELTA_FWD && dotProdDist <= geometryConstants->nav_maxDist)
			{
				us = calcMoveMotorBurst (dotProdDist);
				//calcClosestMotorDist (dotProdDist, &motorDist, &motorBurst);

				temp.orient = Geometry_orientSum (startPose.orient, BASE_MOVE_DELTA_ORIENT + us * MOVE_US_DELTA_ORIENT);

				if (Geometry_orientDiff (temp.orient, endPose.orient, 1) < ROT_AVG_DELTA_ORIENT/2.0f)
				{
					dist = BASE_MOVE_DELTA_LAT + us * MOVE_US_DELTA_LAT;
					Geometry_ptFromOrient (startPose.loc.x, startPose.loc.y, &temp.loc.x, &temp.loc.y, dotProdDist, startPose.orient);
					Geometry_ptFromOrient (temp.loc.x, temp.loc.y, &temp.loc.x, &temp.loc.y, dist, Geometry_orientSum (startPose.orient, PI/2.0f));

					distSqrd = Geometry_distSqrd (temp.loc.x, temp.loc.y, endPose.loc.x, endPose.loc.y);
					if (distSqrd < leewaySqrd)
					{
						if (checkMove (startPose.loc, temp.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
						{
							continue;
						}
						if (checkMove (temp.loc, endPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
						{
							continue;
						}

						nMoves = 1 + (int)(0 != startMoveDir) + (int)(0 != endMoveDir);

						if (nMoves < currentBestNMoves || (nMoves == currentBestNMoves && distSqrd < currentBestMoveErrorSqrd))
						{
							currentBestNMoves = nMoves;
							currentBestMoveErrorSqrd = distSqrd;

							ikData->index = 0;
							ikData->len = 0;

							if (0 != startMoveDir)
							{
								ikData->moves[0].dir = startMoveDir;
								ikData->moves[0].n = startMoveNRotations;
								ikData->moves[0].usBurst = 0;
								ikData->len = 1;
							}

							ikData->moves[ikData->len].dir = 0; // Fwd
							ikData->moves[ikData->len].moveDist = dotProdDist;
							ikData->moves[ikData->len].usBurst = us;
							ikData->moves[ikData->len].n = 1;
							DEBUG_ASSERT(ikData->moves[ikData->len].usBurst < 10000000)
							ikData->len += 1;

							if (0 != endMoveDir)
							{
								ikData->moves[ikData->len].dir = endMoveDir;
								ikData->moves[ikData->len].n = endMoveNRotations;
								ikData->moves[ikData->len].usBurst = 0;
								ikData->len += 1;
							}
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

//! Forward kinematics over 2 robot moves
PointF destCentricFwdKinematics_2Moves (const float dist1,
										const float dist2,
										const float orient1,
										const float orient1PlusDelta,
										const float orient1PlusOrient2,
										const int rotationIndex,
										const IkConstants *ikConstants)
{
	PointF pt;
	float orient1Plus90 = Geometry_orientSum (orient1, PI * 0.5f);
	float orient1PlusDeltaPlus90 = Geometry_orientSum (orient1PlusDelta, PI * 0.5f);
	float orient1PlusOrient2Plus90 = Geometry_orientSum (orient1PlusOrient2, PI * 0.5f);

	pt.x =		dist1 * cos (orient1);
	pt.y =		dist1 * sin (orient1);

	pt.x +=		(((dist1 - BASE_MOVE_DELTA_FWD) / MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * cos (orient1Plus90);
	pt.y +=		(((dist1 - BASE_MOVE_DELTA_FWD) / MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * sin (orient1Plus90);

	pt.x +=		ikConstants->rotationOffsets[rotationIndex].x * cos (orient1PlusDelta);
	pt.y +=		ikConstants->rotationOffsets[rotationIndex].x * sin (orient1PlusDelta);

	pt.x +=		ikConstants->rotationOffsets[rotationIndex].y * cos (orient1PlusDeltaPlus90);
	pt.y +=		ikConstants->rotationOffsets[rotationIndex].y * sin (orient1PlusDeltaPlus90);

	pt.x +=		dist2 * cos (orient1PlusOrient2);
	pt.y +=		dist2 * sin (orient1PlusOrient2);

	pt.x +=		(((dist2 - BASE_MOVE_DELTA_FWD) / MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * cos (orient1PlusOrient2Plus90);
	pt.y +=		(((dist2 - BASE_MOVE_DELTA_FWD) / MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * sin (orient1PlusOrient2Plus90);

	return pt;
}

//! Return collision
int checkDestIKMoves_2Moves (const Image *navMap,
							 const PoseSimple startPose,
							 const PoseSimple endPose,
							 const float dist1,
							 const float dist2,
							 const float orient1PlusDelta,
							 const float orient1PlusOrient2,
							 const int rotationIndex,
							 const int allowOutsideLocalMap,
							 const IkConstants *ikConstants)
{
	PointF pt;
	PointF old;
	float ms;
	float orient1Plus90 = Geometry_orientSum (startPose.orient, PI * 0.5f);
	float orient1PlusDeltaPlus90 = Geometry_orientSum (orient1PlusDelta, PI * 0.5f);
	float orient1PlusOrient2Plus90 = Geometry_orientSum (orient1PlusOrient2, PI * 0.5f);

	pt.x =		startPose.loc.x + dist1 * cos (startPose.orient);
	pt.y =		startPose.loc.y + dist1 * sin (startPose.orient);

	ms = (dist1 - BASE_MOVE_DELTA_FWD) * (1.0f / MOVE_US_DELTA_FWD);
	pt.x +=		(ms * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * cos (orient1Plus90);
	pt.y +=		(ms * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * sin (orient1Plus90);
	if (checkMove (startPose.loc, pt, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
	{
		return 1;
	}
	old = pt;

	pt.x +=		ikConstants->rotationOffsets[rotationIndex].x * cos (orient1PlusDelta);
	pt.y +=		ikConstants->rotationOffsets[rotationIndex].x * sin (orient1PlusDelta);

	pt.x +=		ikConstants->rotationOffsets[rotationIndex].y * cos (orient1PlusDeltaPlus90);
	pt.y +=		ikConstants->rotationOffsets[rotationIndex].y * sin (orient1PlusDeltaPlus90);
	if (checkMove (old, pt, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
	{
		return 1;
	}
	old = pt;

	pt.x +=		dist2 * cos (orient1PlusOrient2);
	pt.y +=		dist2 * sin (orient1PlusOrient2);

	pt.x +=		(((dist2 - BASE_MOVE_DELTA_FWD) / MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * cos (orient1PlusOrient2Plus90);
	pt.y +=		(((dist2 - BASE_MOVE_DELTA_FWD) / MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * sin (orient1PlusOrient2Plus90);
	if (checkMove (old, pt, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
	{
		return 1;
	}
	if (checkMove (pt, endPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
	{
		return 1;
	}
	return 0;
}

MatrixF destCentricCalculateJacobian_2Moves (const float dist1,
											 const float dist2,
											 const float orient1,
											 const float orient1PlusDelta,
											 const float orient1PlusOrient2,
											 const int rotationIndex,
											 const IkConstants *ikConstants)
{
	MatrixF fwdJac;
	float orient1Plus90 = Geometry_orientSum (orient1, PI * 0.5f);
	float orient1PlusDeltaPlus90 = Geometry_orientSum (orient1PlusDelta, PI * 0.5f);
	float orient1PlusOrient2Plus90 = Geometry_orientSum (orient1PlusOrient2, PI * 0.5f);
	float cosOrient1PlusOrient2Plus90 = cos (orient1PlusOrient2Plus90);
	float sinOrient1PlusOrient2Plus90 = cos (orient1PlusOrient2Plus90);

	//d(p.x)/d(dist1)
	fwdJac.val[0] =
		cos (orient1) +
		(MOVE_US_DELTA_LAT / MOVE_US_DELTA_FWD) * cos (orient1Plus90) +
		(MOVE_US_DELTA_ORIENT / MOVE_US_DELTA_FWD) * ikConstants->rotationOffsets[rotationIndex].x * -sin (orient1PlusDelta) +
		(MOVE_US_DELTA_ORIENT / MOVE_US_DELTA_FWD) * ikConstants->rotationOffsets[rotationIndex].y * -sin (orient1PlusDeltaPlus90) +
		(MOVE_US_DELTA_ORIENT / MOVE_US_DELTA_FWD) * dist2 * -sin (orient1PlusOrient2) +
		(MOVE_US_DELTA_ORIENT / MOVE_US_DELTA_FWD) * (((dist2 - BASE_MOVE_DELTA_FWD) / MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * -sinOrient1PlusOrient2Plus90;
	//d(p.x)/d(dist2)
	fwdJac.val[1] =
		cos (orient1PlusOrient2) +
		(MOVE_US_DELTA_LAT / MOVE_US_DELTA_FWD) * cosOrient1PlusOrient2Plus90;
	//d(p.y)/d(dist1)
	fwdJac.val[2] =
		sin (orient1) +
		(MOVE_US_DELTA_LAT / MOVE_US_DELTA_FWD) * sin (orient1Plus90) +
		(MOVE_US_DELTA_ORIENT / MOVE_US_DELTA_FWD) * ikConstants->rotationOffsets[rotationIndex].x * cos (orient1PlusDelta) +
		(MOVE_US_DELTA_ORIENT / MOVE_US_DELTA_FWD) * ikConstants->rotationOffsets[rotationIndex].x * cos (orient1PlusDeltaPlus90) +
		(MOVE_US_DELTA_ORIENT / MOVE_US_DELTA_FWD) * dist2 * cos (orient1PlusOrient2) +
		(MOVE_US_DELTA_ORIENT / MOVE_US_DELTA_FWD) * (((dist2 - BASE_MOVE_DELTA_FWD) / MOVE_US_DELTA_FWD) * MOVE_US_DELTA_LAT + BASE_MOVE_DELTA_LAT) * cosOrient1PlusOrient2Plus90;
	//d(p.y)/d(dist2)
	fwdJac.val[3] = 
		sin (orient1PlusOrient2) +
		(MOVE_US_DELTA_LAT / MOVE_US_DELTA_FWD) * sinOrient1PlusOrient2Plus90;

	return fwdJac;
}

void destCentricIK_2Moves (
	const Image *navMap,
	const Pose *current,
	Dest *dest,
	IKData *ikData,
	const GeometryConstants *geometryConstants,
	const IkConstants *ikConstants,
	const int allowOutsideLocalMap)
{
	PoseSimple startPose;
	PoseSimple endPose;
	int i, j, rotationIndex;
	float orient1, orient2, diff;
	float dist1, dist2;
	float us;
	float orient1PlusDelta, orient1PlusOrient2;
	PointF destError;
	float destErrorSqrd;
	PointF ikDest;
	MatrixF jac;
	MatrixF jacInv;
	float step;
	const float idealErrorSqrd = 0.01f;
	float allowedErrorSqrd = dest->leeway * dest->leeway;
	float testOrient;
	int jacIters;
	float bestDist1, bestDist2, bestErrorSqrd;
	int nMoves;
	float currentBestMoveErrorSqrd = MAX_FLT;
	int currentBestNMoves = MAX_INT32;

	// Orientation at current pose.
	for (i = 0; i < N_ROTATIONS_IN_TOTAL; ++i)
	{
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
			Geometry_orientSum (current->orient, PI/2));

		startPose.orient = Geometry_orientSum (current->orient, ikConstants->rotations[i]);

		if (checkMove (current->loc, startPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
		{
			continue;
		}

		// Orientation at intermediate pose. Do not include rotation of 0.
		for (rotationIndex = 1; rotationIndex < N_ROTATIONS_IN_TOTAL; ++rotationIndex)
		{
			// Calc orient at p1 (intermediate pose) and at p2 (dest)
			orient1 = Geometry_orientSum (startPose.orient, ikConstants->orientChangeMedian);
			orient2 = Geometry_orientSum (orient1, Geometry_orientSum (ikConstants->rotations[rotationIndex], ikConstants->orientChangeMedian));

			// Orientation at dest pose.
			for (j = 0; j < N_ROTATIONS_IN_TOTAL; ++j)
			{
				// Subtract offsets as we are moving backwards from destination.
				Geometry_ptFromOrient (
					dest->dest.loc.x,
					dest->dest.loc.y,
					&endPose.loc.x,
					&endPose.loc.y,
					-ikConstants->rotationOffsets[j].x,
					current->orient);

				Geometry_ptFromOrient (
					endPose.loc.x,
					endPose.loc.y,
					&endPose.loc.x,
					&endPose.loc.y,
					-ikConstants->rotationOffsets[j].y,
					Geometry_orientSum (current->orient, PI/2));

				if (checkMove (dest->dest.loc, endPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
				{
					continue;
				}

				endPose.orient = Geometry_orientSum (dest->dest.orient, -ikConstants->rotations[j]);

				diff = Geometry_orientDiff (orient2, endPose.orient, 1);

				// Determine if orient will match up with dest pose orient.
				// Greater leeway is permitted here, as the moves are checked later anyway.
				if (diff >= (ikConstants->orientChangeRange * 4 + ROT_AVG_DELTA_ORIENT/2.0f))
				{
					continue;
				}

				// Use intersection between lines as initial estimate for intermediate point.
				ikDest = Geometry_lineIntersect (startPose.loc, endPose.loc, orient1, Geometry_orientSum (PI, orient2), &dist1, &dist2);
				//calcClosestMotorDist (dist2, &dist2, &burst);
				//calcClosestMotorDist (dist1, &dist1, &burst);

				if (dist1 <= BASE_MOVE_DELTA_FWD || dist1 >= geometryConstants->nav_maxDist ||
					dist2 <= BASE_MOVE_DELTA_FWD || dist2 >= geometryConstants->nav_maxDist)
				{
					continue;
				}

				bestErrorSqrd = MAX_FLT;
				bestDist1 = MAX_FLT;
				bestDist2 = MAX_FLT;
				jacIters = 0;
				while (1)
				{
					if (jacIters++ > 50)
					{
						break;
					}
					us = calcMoveMotorBurst (dist1);
					orient1PlusDelta = Geometry_orientSum (startPose.orient, us * MOVE_US_DELTA_ORIENT + BASE_MOVE_DELTA_ORIENT);
					orient1PlusOrient2 = Geometry_orientSum (orient1PlusDelta, ikConstants->rotations[rotationIndex]);

					ikDest = destCentricFwdKinematics_2Moves (dist1, dist2, startPose.orient, orient1PlusDelta, orient1PlusOrient2, rotationIndex, ikConstants);
					ikDest.x += startPose.loc.x;
					ikDest.y += startPose.loc.y;
					destError.x = endPose.loc.x - ikDest.x;
					destError.y = endPose.loc.y - ikDest.y;
					destErrorSqrd = destError.x * destError.x + destError.y * destError.y;

					if (!checkDestIKMoves_2Moves (navMap, startPose, endPose, dist1, dist2, orient1PlusDelta, orient1PlusOrient2, rotationIndex, allowOutsideLocalMap, ikConstants))
					{
						// Move from current pose to dest, to check that moves orient is correct.
						testOrient = Geometry_orientSum (current->orient, ikConstants->rotations[i]);
						testOrient = Geometry_orientSum (testOrient, BASE_MOVE_DELTA_ORIENT + MOVE_US_DELTA_ORIENT * calcMoveMotorBurst(dist1));
						testOrient = Geometry_orientSum (testOrient, ikConstants->rotations[rotationIndex]);
						testOrient = Geometry_orientSum (testOrient, BASE_MOVE_DELTA_ORIENT + MOVE_US_DELTA_ORIENT * calcMoveMotorBurst(dist2));
						testOrient = Geometry_orientSum (testOrient, ikConstants->rotations[j]);

						if (Geometry_orientDiff (testOrient, dest->dest.orient, 1) < ROT_AVG_DELTA_ORIENT/2.0f)
						{
							if (destErrorSqrd < idealErrorSqrd)
							{
								bestErrorSqrd = destErrorSqrd;
								bestDist1 = dist1;
								bestDist2 = dist2;
								break;
							}
							else if (destErrorSqrd < allowedErrorSqrd && destErrorSqrd < bestErrorSqrd)
							{
								bestErrorSqrd = destErrorSqrd;
								bestDist1 = dist1;
								bestDist2 = dist2;
							}
						}
					}

					step = 1.0f;
					if (destErrorSqrd > 1.0f)
					{
						step = 1.0f / destErrorSqrd;
					}
					destError.x *= step;
					destError.y *= step;

					jac = destCentricCalculateJacobian_2Moves (dist1, dist2, startPose.orient, orient1PlusDelta, orient1PlusOrient2, rotationIndex, ikConstants);
					jacInv = MatrixF_inverse (jac);

					dist1 += jacInv.val[0] * destError.x + jacInv.val[1] * destError.y;
					dist2 += jacInv.val[2] * destError.x + jacInv.val[3] * destError.y;

					// Greater leeway in dist and orient diff is allowed
					// when starting IK in order to avoid false positives.
					// This may lead to convergence outside the accepted
					// bounds though, so check here and break from IK loop.
					if (dist1 <= BASE_MOVE_DELTA_FWD || dist1 >= geometryConstants->nav_maxDist ||
						dist2 <= BASE_MOVE_DELTA_FWD || dist2 >= geometryConstants->nav_maxDist)
					{
						break;
					}
				}

				if (bestErrorSqrd != MAX_FLT)
				{
					nMoves = 2 + (int)(ikConstants->moveDirs[i] != 0) + (int)(ikConstants->moveDirs[j] != 0);

					if (nMoves < currentBestNMoves || (nMoves == currentBestNMoves && bestErrorSqrd < currentBestMoveErrorSqrd))
					{
						currentBestNMoves = nMoves;
						currentBestMoveErrorSqrd = bestErrorSqrd;

						ikData->index = 0;
						ikData->len = 0;

						if (ikConstants->moveDirs[i] != 0)
						{
							ikData->moves[0].dir = ikConstants->moveDirs[i];
							ikData->moves[0].n = ikConstants->moveNRotations[i];
							ikData->moves[0].usBurst = 0;
							ikData->len = 1;
						}

						ikData->moves[ikData->len].dir = 0;
						ikData->moves[ikData->len].moveDist = bestDist1;
						ikData->moves[ikData->len].n = 1;
						ikData->moves[ikData->len++].usBurst = calcMoveMotorBurst(bestDist1);
						DEBUG_ASSERT(ikData->moves[ikData->len].usBurst < 10000000)
						++ikData->len;

						// Rotation of 0 is not allowed for move at intermediate pose.
						// 2x2 would not be possible in this case.
						ikData->moves[ikData->len].dir = ikConstants->moveDirs[rotationIndex];
						ikData->moves[ikData->len].usBurst = 0;
						ikData->moves[ikData->len++].n = ikConstants->moveNRotations[rotationIndex];

						ikData->moves[ikData->len].dir = 0;
						ikData->moves[ikData->len].moveDist = bestDist2;
						ikData->moves[ikData->len].n = 1;
						ikData->moves[ikData->len].usBurst = calcMoveMotorBurst(bestDist2);
						DEBUG_ASSERT(ikData->moves[ikData->len].usBurst < 10000000)
						++ikData->len;

						if (ikConstants->moveDirs[j] != 0)
						{
							ikData->moves[ikData->len].dir = ikConstants->moveDirs[j];
							ikData->moves[ikData->len].usBurst = 0;
							ikData->moves[ikData->len++].n = ikConstants->moveNRotations[j];
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

float getMoveDistWithinLocalMap (const PointF pt1,
								 const PointF pt2,
								 const float moveDist,
								 const Image *navMap)
{
	PointI pt1I, pt2I;
	int xOutside, yOutside;
	int xOutsideDist, yOutsideDist;
	int xDist, yDist;
	float xRatio, yRatio;

	{
		pt2I = PointF_toPointI (pt2);
		xOutside = (pt2I.x < (navMap->orig.x + LOC_MAP_EDGE_LEEWAY) || pt2I.x >= (navMap->orig.x + LOC_MAP_DIMS - LOC_MAP_EDGE_LEEWAY));
		yOutside = (pt2I.y < (navMap->orig.y + LOC_MAP_EDGE_LEEWAY) || pt2I.y >= (navMap->orig.y + LOC_MAP_DIMS - LOC_MAP_EDGE_LEEWAY));

		if (!xOutside && !yOutside)
		{
			return moveDist;
		}
	}

	pt1I = PointF_toPointI (pt1);
//	DEBUG_ASSERT(pt1I.x >= (navMap->orig.x + LOC_MAP_EDGE_LEEWAY) && pt1I.x < (navMap->orig.x + LOC_MAP_DIMS - LOC_MAP_EDGE_LEEWAY))
//	DEBUG_ASSERT(pt1I.y >= (navMap->orig.y + LOC_MAP_EDGE_LEEWAY) && pt1I.y < (navMap->orig.y + LOC_MAP_DIMS - LOC_MAP_EDGE_LEEWAY))
	if (pt1I.x < (navMap->orig.x + LOC_MAP_EDGE_LEEWAY) ||
		pt1I.x >= (navMap->orig.x + LOC_MAP_DIMS - LOC_MAP_EDGE_LEEWAY) ||
		pt1I.y < (navMap->orig.y + LOC_MAP_EDGE_LEEWAY) ||
		pt1I.y >= (navMap->orig.y + LOC_MAP_DIMS - LOC_MAP_EDGE_LEEWAY))
	{
		return 0; // fail
	}

	if (xOutside)
	{
		if (pt2I.x < (navMap->orig.x + LOC_MAP_EDGE_LEEWAY))
		{
			xOutsideDist = (navMap->orig.x + LOC_MAP_EDGE_LEEWAY) - pt2I.x;
		}
		else
		{
			xOutsideDist = pt2I.x - (navMap->orig.x + LOC_MAP_DIMS - LOC_MAP_EDGE_LEEWAY);
		}
		xDist = abs (pt2I.x - pt1I.x);
		xRatio = ((xDist - xOutsideDist) - 3) / (float)xDist;
		xRatio = max(0.0f, xRatio);

		if (!yOutside)
		{
			return moveDist * xRatio;
		}
	}

	if (yOutside)
	{
		if (pt2I.y < (navMap->orig.y + LOC_MAP_EDGE_LEEWAY))
		{
			yOutsideDist = (navMap->orig.y + LOC_MAP_EDGE_LEEWAY) - pt2I.y;
		}
		else
		{
			yOutsideDist = pt2I.y - (navMap->orig.y + LOC_MAP_DIMS - LOC_MAP_EDGE_LEEWAY);
		}
		yDist = abs (pt2I.y - pt1I.y);
		yRatio = ((yDist - yOutsideDist) - 3) / (float)yDist;
		yRatio = max(0.0f, yRatio);

		if (!xOutside)
		{
			return moveDist * yRatio;
		}
	}

	return moveDist * min (xRatio, yRatio);
}

float getMoveDistWithinEnvironment (const PointF pt1, const PointF pt2, const float moveDist)
{
	PointI pt1I, pt2I;
	int xOutside, yOutside;
	int xOutsideDist, yOutsideDist;
	int xDist, yDist;
	float xRatio, yRatio;

	{
		pt2I = PointF_toPointI (pt2);
		xOutside = (pt2I.x < LOC_MAP_EDGE_LEEWAY || pt2I.x >= (ENVIR_DIMS - LOC_MAP_EDGE_LEEWAY));
		yOutside = (pt2I.y < LOC_MAP_EDGE_LEEWAY || pt2I.y >= (ENVIR_DIMS - LOC_MAP_EDGE_LEEWAY));

		if (!xOutside && !yOutside)
		{
			return moveDist;
		}
	}

	pt1I = PointF_toPointI (pt1);
	DEBUG_ASSERT(pt1I.x >= LOC_MAP_EDGE_LEEWAY && pt1I.x < (ENVIR_DIMS - LOC_MAP_EDGE_LEEWAY))
	DEBUG_ASSERT(pt1I.y >= LOC_MAP_EDGE_LEEWAY && pt1I.y < (ENVIR_DIMS - LOC_MAP_EDGE_LEEWAY))

	if (xOutside)
	{
		if (pt2I.x < LOC_MAP_EDGE_LEEWAY)
		{
			xOutsideDist = LOC_MAP_EDGE_LEEWAY - pt2I.x;
		}
		else
		{
			xOutsideDist = pt2I.x - (ENVIR_DIMS - LOC_MAP_EDGE_LEEWAY);
		}
		xDist = abs (pt2I.x - pt1I.x);
		xRatio = ((xDist - xOutsideDist) - 3) / (float)xDist;
		xRatio = max(0.0f, xRatio);

		if (!yOutside)
		{
			return moveDist * xRatio;
		}
	}

	if (yOutside)
	{
		if (pt2I.y < LOC_MAP_EDGE_LEEWAY)
		{
			yOutsideDist = LOC_MAP_EDGE_LEEWAY - pt2I.y;
		}
		else
		{
			yOutsideDist = pt2I.y - (ENVIR_DIMS - LOC_MAP_EDGE_LEEWAY);
		}
		yDist = abs (pt2I.y - pt1I.y);
		yRatio = ((yDist - yOutsideDist) - 3) / (float)yDist;
		yRatio = max(0.0f, yRatio);

		if (!xOutside)
		{
			return moveDist * yRatio;
		}
	}

	return moveDist * min (xRatio, yRatio);
}

//! Try to perform fwd move from current pose. Return ikData from move, and resulting dist from dest or MAX_FLT for invalid.
float __coarseFwdMove (const Pose *currentPose,
					   IKData *ikData,
					   const GeometryConstants *geometryConstants,
					   const Dest *dest,
					   const float desiredMoveDist,
					   const int allowOutsideLocalMap,
					   const Image *navMap,
					   const BEHAVIOUR behaviour,
					   const int isImplemented)
{
	Move tempMove;
	Pose tempPose;
	float moveDist, usBurst, verifiedMoveDist;
	int i;

	DEBUG_ASSERT_IS_BOOL(allowOutsideLocalMap)
	DEBUG_ASSERT_IS_BOOL(isImplemented)

	moveDist = desiredMoveDist;
	if (0.0f == desiredMoveDist)
	{
		moveDist = Geometry_dist (currentPose->loc.x, currentPose->loc.y, dest->dest.loc.x, dest->dest.loc.y);
	}
	moveDist = min (geometryConstants->nav_maxDist, moveDist);
	if (dest->maxDist != MAX_FLT)
	{
		moveDist = min (dest->maxDist, moveDist);
	}
	moveDist = max (BASE_MOVE_DELTA_FWD, moveDist);
	usBurst = calcMoveMotorBurst (moveDist);
	DEBUG_ASSERT(usBurst < 10000000)
	tempMove = initMoveParams (1, 0, moveDist, usBurst); // Directions = 0123:FBLR
	tempPose = *currentPose;
	Actuator_executeMove (&tempPose, tempMove, 0, 0, 0);

	if (!isImplemented || behaviour != BACKTRACK)
	{
		// Ensure this move will not result in the robot going outside the environment, or the local map, depending
		// on whether or not the robot is following a path.
		// Check against local map or environment.
		if (allowOutsideLocalMap)
		{
			verifiedMoveDist = getMoveDistWithinEnvironment (currentPose->loc, tempPose.loc, moveDist);
		}
		else
		{
			verifiedMoveDist = getMoveDistWithinLocalMap (currentPose->loc, tempPose.loc, moveDist, navMap);
		}
		if (0 == verifiedMoveDist)
		{
			return MAX_FLT;
		}
		if (verifiedMoveDist != moveDist)
		{
			usBurst = calcMoveMotorBurst (verifiedMoveDist);
			DEBUG_ASSERT(usBurst < 10000000)
			tempMove = initMoveParams (1, 0, verifiedMoveDist, usBurst);
			tempPose = *currentPose;
			Actuator_executeMove (&tempPose, tempMove, 0, 0, 0);
		}
	}

	if (behaviour != BACKTRACK && checkMove (currentPose->loc, tempPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
	{
		return MAX_FLT;
	}

	// Append move to end of current IK move
	++ikData->len;
	i = ikData->len - 1;
	ikData->moves[i].n = 1;
	ikData->moves[i].dir = 0; // 0123:FBLR
	ikData->moves[i].moveDist = moveDist;
	ikData->moves[i].usBurst = usBurst;

	return Geometry_distSqrd (tempPose.loc.x, tempPose.loc.y, dest->dest.loc.x, dest->dest.loc.y);
}

//! Try to perform rotation and fwd move from current pose. Perform rotations in given direction until suitable move found.
float __coarseRotateFwdMove (const Pose *currentPose,
							 IKData *ikData,
							 const GeometryConstants *geometryConstants,
							 const Dest *dest,
							 const int direction,
							 const float desiredMoveDist,
							 const int allowOutsideLocalMap,
							 const Image *navMap,
							 const BEHAVIOUR behaviour,
							 const int isImplemented)
{
	Pose rotationPose, tempPose;
	Move rotationMove;
	IKData rotationIkData, moveIkData, minIkData;
	PointI pti;
	float orientToDest, orientDiff, tempOrientDiff;
	float dist, minDist;
	int isRotatingTowardsDest;

	DEBUG_ASSERT_IS_BOOL(allowOutsideLocalMap)
	DEBUG_ASSERT_IS_BOOL(isImplemented)

	// Robot will continue performing this move.
	rotationPose = *currentPose;
	rotationMove = initMoveParams (1, direction, 0.0f, 0);

	rotationIkData = initIKData();
	rotationIkData.index = 0;
	rotationIkData.len = 1;
	rotationIkData.moves[0] = initMoveParams (0, direction, 0.0f, 0);

	// Determine move (stored in an IKData) that will result in closest final dist to destination.
	minDist = MAX_FLT;

	// Continue trying to rotate until robot is facing away. Given coarseness of robot actuators this can
	// only be a few moves.
	isRotatingTowardsDest = 1;
	orientToDest = Geometry_orient (currentPose->loc.x, currentPose->loc.y, dest->dest.loc.x, dest->dest.loc.y);
	orientDiff = Geometry_orientDiff (orientToDest, currentPose->orient, 1);
	while (isRotatingTowardsDest || orientDiff < (PI * 0.5f))
	{
		tempPose = rotationPose;
		Actuator_executeMove (&tempPose, rotationMove, 0, 0, 0);

		if (behaviour != BACKTRACK && checkMove (tempPose.loc, rotationPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
		{
			break;
		}
		rotationPose = tempPose;

		if (!isImplemented || behaviour != BACKTRACK)
		{
			if (allowOutsideLocalMap)
			{
				if (!Geometry_isPtInEnvironment (PointF_toPointI (rotationPose.loc), LOC_MAP_EDGE_LEEWAY))
				{
					break;
				}
			}
			else
			{
				pti = navMap->orig;
				pti.x += (LOC_MAP_DIMS / 2);
				pti.y += (LOC_MAP_DIMS / 2);
				if (!Geometry_isPtOnLocalMap (PointF_toPointI (rotationPose.loc), pti, LOC_MAP_EDGE_LEEWAY))
				{
					break;
				}
			}
		}

		++rotationIkData.moves[0].n;
		moveIkData = rotationIkData;

		dist = __coarseFwdMove (&rotationPose, &moveIkData, geometryConstants, dest, desiredMoveDist, allowOutsideLocalMap, navMap, behaviour, isImplemented);
		if (dist < minDist)
		{
			minDist = dist;
			minIkData = moveIkData;
		}

		// Determine if still rotating towards the destination location.
		orientToDest = Geometry_orient (rotationPose.loc.x, rotationPose.loc.y, dest->dest.loc.x, dest->dest.loc.y);
		tempOrientDiff = Geometry_orientDiff (orientToDest, rotationPose.orient, 1);
		isRotatingTowardsDest = (tempOrientDiff < orientDiff);
		orientDiff = tempOrientDiff;
	}

	if (minDist < MAX_FLT)
	{
		*ikData = minIkData;
	}
	return minDist;
}

int __coarseRotate (const Pose *currentPose,
					IKData *ikData,
					const Dest *dest,
					const int allowOutsideLocalMap,
					const Image *navMap,
					const BEHAVIOUR behaviour,
					const int isImplemented)
{
	Pose rotationPose, tempPose;
	Move rotationMove;
	IKData rotationIkData;
	PointI pti;
	float orientToDest, orientDiff, tempOrientDiff;
	int direction;

	DEBUG_ASSERT_IS_BOOL(allowOutsideLocalMap)
	DEBUG_ASSERT_IS_BOOL(isImplemented)

	// Get best direction in which to rotate to face destination.
	orientToDest = Geometry_orient (currentPose->loc.x, currentPose->loc.y, dest->dest.loc.x, dest->dest.loc.y);
	orientDiff = Geometry_orientDiff (currentPose->orient, orientToDest, 0);
	direction = 2 + (orientDiff < 0.0f);
	orientDiff = Geometry_orientDiff (currentPose->orient, orientToDest, 1);

	rotationMove = initMoveParams (1, direction, 0.0f, 0);
	rotationPose = *currentPose;
	rotationIkData = initIKData();
	rotationIkData.index = 0;
	rotationIkData.len = 1;
	rotationIkData.moves[0] = initMoveParams (0, direction, 0.0f, 0);

	// I am going to assume that this should succeed, i.e. not fall back on rotating in the second
	// direction. Occupied terrain is already dilated, so rotation should not result result in
	// any contact.
	while (1)
	{
		tempPose = rotationPose;
		Actuator_executeMove (&tempPose, rotationMove, 0, 0, 0);
		orientToDest = Geometry_orient (tempPose.loc.x, tempPose.loc.y, dest->dest.loc.x, dest->dest.loc.y);
		tempOrientDiff = Geometry_orientDiff (tempPose.orient, orientToDest, 1);
		if (tempOrientDiff > orientDiff)
		{
			*ikData = rotationIkData;

			// Return result stating if the pose reached is valid, i.e. pointing at the dest.
			return tempOrientDiff < (ROT_AVG_DELTA_ORIENT * 1.5f);
		}
		orientDiff = tempOrientDiff;

		// Still moving towards destination, so this move is needed. Therefore check validity.
		if (behaviour != BACKTRACK && checkMove (rotationPose.loc, tempPose.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
		{
			return 0;
		}
		rotationPose = tempPose;

		if (!isImplemented || behaviour != BACKTRACK)
		{
			if (allowOutsideLocalMap)
			{
				if (!Geometry_isPtInEnvironment (PointF_toPointI (rotationPose.loc), LOC_MAP_EDGE_LEEWAY))
				{
					return 0;
				}
			}
			else
			{
				pti = navMap->orig;
				pti.x += (LOC_MAP_DIMS / 2);
				pti.y += (LOC_MAP_DIMS / 2);
				if (!Geometry_isPtOnLocalMap (PointF_toPointI (rotationPose.loc), pti, LOC_MAP_EDGE_LEEWAY))
				{
					return 0;
				}
			}
		}

		++rotationIkData.moves[0].n;
	}
}

//! Calculate move to destination location when current distance is large.
/*!
The (already simple) process is simplified by splitting up the actions of rotating to
face the destination, and moving towards it.

If pointing away from destination location, determine direction in which to rotate. Execute
rotation moves on a temporary pose to verify that the intermediate poses are not obstructed
and are on the map. Return boolean value signifying if the move can be made.

If pointing in the general direction of the destination location, check how close the robot
can get by moving forward from the current pose. Also rotate to left and determine final
proximity if forward move made from this pose. Continue rotating left and checking final proximity
until orientation has turned away from the destination. Repeat for right. Compare proximities for
current pose, poses to left and poses to right. Return the minimum final distance (squared, as it
is only being used for comparrison) from the destination. Return MAX_FLT if no move can be made.
*/
void IK_destCentricCoarseMove (
	const Image *navMap,
	const Pose *currentPose,
	Dest *dest,
	IKData *ikData,
	const GeometryConstants *geometryConstants,
	const float desiredMoveDist,
	const int allowOutsideLocalMap,
	const BEHAVIOUR behaviour,
	const int isImplemented,
	const int printLogic)
{
	IKData ikDatas[3];
	float dists[3];
	float currentDist, minDist, orientToDest, currentOrientDiff;
	int i, minIndex;

	DEBUG_ASSERT_IS_BOOL(allowOutsideLocalMap)
	DEBUG_ASSERT_IS_BOOL(isImplemented)

	currentDist = Geometry_distSqrd (currentPose->loc.x, currentPose->loc.y, dest->dest.loc.x, dest->dest.loc.y);

	orientToDest = Geometry_orient (currentPose->loc.x, currentPose->loc.y, dest->dest.loc.x, dest->dest.loc.y);
	currentOrientDiff = Geometry_orientDiff (currentPose->orient, orientToDest, 1);
	if (currentOrientDiff > (PI * 0.5f))
	{
		// Pointing away from target, so just calc rotation.
		if (!__coarseRotate (currentPose, ikData, dest, allowOutsideLocalMap, navMap, behaviour, isImplemented))
		{
			dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
		}
		return;
	}

	// Init IKData for fwd move, as __coarseFwdMove accumulates moves onto this s.th. it can be called from __coarseRotateFwdMove.
	ikDatas[0] = initIKData();
	ikDatas[0].len = 0;
	ikDatas[0].index = 0;
	dists[0] = __coarseFwdMove (currentPose, &ikDatas[0], geometryConstants, dest, desiredMoveDist, allowOutsideLocalMap, navMap, behaviour, isImplemented);
	dists[1] = __coarseRotateFwdMove (currentPose, &ikDatas[1], geometryConstants, dest, 2, desiredMoveDist, allowOutsideLocalMap, navMap, behaviour, isImplemented);
	dists[2] = __coarseRotateFwdMove (currentPose, &ikDatas[2], geometryConstants, dest, 3, desiredMoveDist, allowOutsideLocalMap, navMap, behaviour, isImplemented);

	DEBUG_ASSERT (currentDist < MAX_FLT)
	minIndex = -1;
	minDist = currentDist;
	for (i = 0; i < 3; ++i)
	{
		if (dists[i] < minDist)
		{
			minIndex = i;
			minDist = dists[i];
		}
	}

	// Enforce rule that robot must move closer to destination (if not adjusting orient in preparation for move).
	if (-1 == minIndex)
	{
		dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
		return;
	}

	// Tell robot how to move.
	*ikData = ikDatas[minIndex];
}

void IK_destCentricAccurateMove (FILE *f,
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
								 const int printLogic)
{
	float dist;
	Pose tempPose;
	IKData tempIkData;

	DEBUG_ASSERT_IS_BOOL(verbose)
	DEBUG_ASSERT_IS_BOOL(allowOutsideLocalMap)
	DEBUG_ASSERT_IS_BOOL(isImplemented)

	// Check if the current pose is acceptible
	{
		dist = Geometry_distSqrd (pose->loc.x, pose->loc.y, dest->dest.loc.x, dest->dest.loc.y);
		if (dist < (dest->leeway * dest->leeway) && Geometry_orientDiff (pose->orient, dest->dest.orient, 1) < ROT_AVG_DELTA_ORIENT/2.0f)
		{
			ikData->index = 0;
			ikData->len = 0;
			dest->type |= IS_AT_DEST;
			return;
		}
	}

	if (printLogic)
	{
		return;
	}

	dist = Geometry_dist (pose->loc.x, pose->loc.y, dest->dest.loc.x, dest->dest.loc.y);
	EXPLICIT_DEBUG_ASSERT(geometryConstants->areExpDistsSetup == 1)
	if (dist > (geometryConstants->nav_maxDist * 1.8f))
	{
		dest->type |= IS_PATH_REQUIRED;
		return;
	}

	if (dist < geometryConstants->nav_maxDist)
	{
		destCentricIK_1Move (navMap, pose, dest, ikData, geometryConstants, ikConstants, allowOutsideLocalMap);
		if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE))
		{
			return;
		}
		dest->type &= ~IS_IK_CURRENTLY_IMPOSSIBLE;
	}

	destCentricIK_2Moves (navMap, pose, dest, ikData, geometryConstants, ikConstants, allowOutsideLocalMap);
	if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE))
	{
		return;
	}
	dest->type &= ~IS_IK_CURRENTLY_IMPOSSIBLE;

	// First attempt failed. Calculate path to avoid making too many small moves.
	if (dist > geometryConstants->nav_maxDist * 0.8f)
	{
		dest->type |= IS_PATH_REQUIRED;
		return;
	}

	// If a sequence of short moves will bring robot to a location where IK
	// can succeed, then return the first of these.
	tempPose = *pose;
	IK_destCentricCoarseMove (navMap, &tempPose, dest, ikData, geometryConstants, BASE_MOVE_DELTA_FWD, allowOutsideLocalMap, behaviour, isImplemented, 0);
	if (dest->type & IS_IK_CURRENTLY_IMPOSSIBLE || dest->type & IS_PATH_REQUIRED)
	{
		return;
	}
	Actuator_executeMove (&tempPose, ikData->moves[0], 0, 0, 0);

	while (1)
	{
		dist = Geometry_dist (tempPose.loc.x, tempPose.loc.y, dest->dest.loc.x, dest->dest.loc.y);
		if (dist < dest->leeway && !checkMove (tempPose.loc, dest->dest.loc, navMap, 1, LOC_MAP_EDGE_LEEWAY, allowOutsideLocalMap))
		{
			return;
		}

		if (dist < geometryConstants->nav_maxDist)
		{
			destCentricIK_1Move (navMap, &tempPose, dest, &tempIkData, geometryConstants, ikConstants, allowOutsideLocalMap);
			if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE))
			{
				// This is calculated from a temporary pose, so "isAtDest" is not valid.
				dest->type &= ~IS_AT_DEST;
				return;
			}
			dest->type &= ~IS_IK_CURRENTLY_IMPOSSIBLE;
		}

		destCentricIK_2Moves (navMap, &tempPose, dest, &tempIkData, geometryConstants, ikConstants, allowOutsideLocalMap);
		if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE))
		{
			dest->type &= ~IS_AT_DEST;
			return;
		}
		dest->type &= ~IS_IK_CURRENTLY_IMPOSSIBLE;

		IK_destCentricCoarseMove (navMap, &tempPose, dest, &tempIkData, geometryConstants, BASE_MOVE_DELTA_FWD, allowOutsideLocalMap, behaviour, isImplemented, 0);
		if (dest->type & IS_IK_CURRENTLY_IMPOSSIBLE || dest->type & IS_PATH_REQUIRED)
		{
			return;
		}
		Actuator_executeMove (&tempPose, tempIkData.moves[0], 0, 0, 0);
	}
}


