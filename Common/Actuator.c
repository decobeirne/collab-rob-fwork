
#include "Actuator.h"
#include "Geometry.h"
#include "Maths.h"
#include "RobotCore.h"
#ifdef IS_LINUX
#include "../Gumstix/cmu.h"
#endif





//! Setup offsets for rotation moves carried out. Also setup median moves for IK.
void Actuator_setupIkConstants (const GeometryConstants *g,
								IkConstants *ikConstants)
{
#if 1
	Move m;
	Pose p;
	int i;
	float temp;
	FILE *f = 0;

	ikConstants->moveRange = (g->nav_maxDist - BASE_MOVE_DELTA_FWD)/2.0f;
	ikConstants->moveMedian = BASE_MOVE_DELTA_FWD + (g->nav_maxDist - BASE_MOVE_DELTA_FWD)/2.0f;
	ikConstants->usMedian = calcMoveMotorBurst(ikConstants->moveMedian);
	//calcClosestMotorDist (moveMedian, &moveMedian, &msMedian);

	ikConstants->offsetMedian = BASE_MOVE_DELTA_LAT + ikConstants->usMedian * MOVE_US_DELTA_LAT;

	ikConstants->orientChangeRange = fabs (ikConstants->usMedian * MOVE_US_DELTA_ORIENT);
	ikConstants->orientChangeMedian = BASE_MOVE_DELTA_ORIENT + (ikConstants->usMedian * MOVE_US_DELTA_ORIENT);

	ikConstants->rotationOffsets[0].x = 0;
	ikConstants->rotationOffsets[0].y = 0;

	// Rotations to left: 1..5
	m.dir = 2;
	p.loc.x = 0.0f;
	p.loc.y = 0.0f;
	p.orient = 0.0f;

	for (i = 1; i < (1 + N_ROTATIONS_FOR_180); ++i)
	{
		Actuator_executeMove (&p, m, 0, 0, f);

		ikConstants->rotationOffsets[i].x = p.loc.x; // X stores MOVE
		ikConstants->rotationOffsets[i].y = p.loc.y; // Y stores OFFSET (perpindicular to robot orientation)
	}

	// Rotations to right: 6..11
	m.dir = 3;
	p.loc.x = 0.0f;
	p.loc.y = 0.0f;
	p.orient = 0.0f;

	for (i = (1 + N_ROTATIONS_FOR_180); i < N_ROTATIONS_IN_TOTAL; ++i)
	{
		Actuator_executeMove (&p, m, 0, 0, f);

		ikConstants->rotationOffsets[i].x = p.loc.x; // X stores MOVE
		ikConstants->rotationOffsets[i].y = p.loc.y; // Y stores OFFSET (perpindicular to robot orientation)
	}

	ikConstants->rotations[0] = 0;
	temp = 0;
	ikConstants->moveDirs[0] = 0; // Fwd
	ikConstants->moveNRotations[0] = 0;

	for (i = 1; i < (1 + N_ROTATIONS_FOR_180); ++i)
	{
		temp += ROT_LEFT_DELTA_ORIENT;
		ikConstants->rotations[i] = temp;
		ikConstants->moveDirs[i] = 2; // Left
		ikConstants->moveNRotations[i] = i;
	}

	temp = 0;
	for (i = (1 + N_ROTATIONS_FOR_180); i < N_ROTATIONS_IN_TOTAL; ++i)
	{
		temp += ROT_RIGHT_DELTA_ORIENT;
		ikConstants->rotations[i] = temp;
		ikConstants->moveDirs[i] = 3; // Right
		ikConstants->moveNRotations[i] = i - (N_ROTATIONS_FOR_180);
	}

	// Sanity check
	temp = ikConstants->moveMedian;
	temp = calcMoveMotorBurst (temp);
	temp = BASE_MOVE_DELTA_ORIENT + temp * MOVE_US_DELTA_ORIENT;
	DEBUG_ASSERT (fabs (temp - ikConstants->orientChangeMedian) < FLT_EPSILON)
#endif
}

extern int g_forceQuit;

//! Execute move given direction and distance
void Actuator_executeMove (
								Pose *pose,
								const Move move,
								const int isExecuted,
								const int iteration,
								FILE *xmlLog)
{
	float f;
	PointF trans;
#ifdef IS_MOTOR
	int res;
#endif
#ifdef PRINT_EVENTS
	int isHBridge;
#endif

	// fblr = 0123
	switch (move.dir)
	{
	case 0:
			trans.x = 0.0f;
			trans.y = 0.0f;

			f = BASE_MOVE_DELTA_FWD + move.usBurst * MOVE_US_DELTA_FWD;

			Geometry_ptFromOrient (
				pose->loc.x,
				pose->loc.y,
				&pose->loc.x,
				&pose->loc.y,
				f,
				pose->orient);

			f = BASE_MOVE_DELTA_LAT + move.usBurst * MOVE_US_DELTA_LAT;

			Geometry_ptFromOrient (
				trans.x,
				trans.y,
				&trans.x,
				&trans.y,
				f,
				Geometry_orientSum (pose->orient, PI/2));

			pose->loc.x += trans.x;
			pose->loc.y += trans.y;

			f = BASE_MOVE_DELTA_ORIENT + move.usBurst * MOVE_US_DELTA_ORIENT;

			pose->orient = Geometry_orientSum (pose->orient, f);

#ifdef PRINT_EVENTS
			if (isExecuted)
			{
				fprintf (xmlLog, "<Move>iteration=%d direction=0 dist=%f burst=%d</Move>\n", iteration, move.moveDist, move.usBurst);
			}
#endif
			break;
	case 2:
		{
			trans.x = 0.0f;
			trans.y = 0.0f;

			Geometry_ptFromOrient ( // trans perp to rob orient
				trans.x,
				trans.y,
				&trans.x,
				&trans.y,
				ROT_LEFT_DELTA_LAT,
				Geometry_orientSum (pose->orient, PI/2));

			Geometry_ptFromOrient ( // trans along rob orient
				pose->loc.x,
				pose->loc.y,
				&pose->loc.x,
				&pose->loc.y,
				ROT_LEFT_DELTA_FWD,
				pose->orient);

			pose->loc.x += trans.x;
			pose->loc.y += trans.y;

			pose->orient = Geometry_orientSum (pose->orient, ROT_LEFT_DELTA_ORIENT);

#ifdef PRINT_EVENTS
			if (isExecuted)
			{
				fprintf (xmlLog, "<Move>iteration=%d direction=2 burst=0</Move>\n", iteration);
			}
#endif
		}
		break;
	case 3:
		{
			trans.x = 0.0f;
			trans.y = 0.0f;

			Geometry_ptFromOrient ( // trans perp to rob orient
				trans.x,
				trans.y,
				&trans.x,
				&trans.y,
				ROT_RIGHT_DELTA_LAT,
				Geometry_orientSum (pose->orient, PI/2));

			Geometry_ptFromOrient ( // trans along rob orient
				pose->loc.x,
				pose->loc.y,
				&pose->loc.x,
				&pose->loc.y,
				ROT_RIGHT_DELTA_FWD,
				pose->orient);

			pose->loc.x += trans.x;
			pose->loc.y += trans.y;

			pose->orient = Geometry_orientSum (pose->orient, ROT_RIGHT_DELTA_ORIENT);

#ifdef PRINT_EVENTS
			if (isExecuted)
			{
				fprintf (xmlLog, "<Move>iteration=%d direction=3 burst=0</Move>\n", iteration);
			}
#endif
		}
		break;
	}

	if (0 == move.dir || 2 == move.dir || 3 == move.dir)
	{
#ifdef PRINT_EVENTS
		isHBridge = 0;
#ifdef IS_H_BRIDGE
		isHBridge = 1;
#endif
		if (xmlLog)
		{
			fprintf (xmlLog, "<bCMU_MoveRobot>usBurst=%d dir=%d isHBridge=%d</bCMU_MoveRobot>\n", move.usBurst, move.dir, isHBridge);
		}
#endif

#ifdef IS_MOTOR
		if (isExecuted)
		{
			res = bCMU_MoveRobot (move.usBurst, move.dir);
			if (!res)
			{
				g_forceQuit = 1;
			}
		}
#endif
	}
}

// Global variable - easier than passing around through 20 functions
int G_COOP_LOC_DONT_CHECK_LOS = 0;

int checkMove (const PointF loc1,
			   const PointF loc2,
			   const Image *navMap,
			   const int navigation,
			   const int localMapEdge,
			   const int allowOutsideLocalMap)
{
	PointI pt1, pt2;
	int res;
	uchar hitValue;

	pt1 = PointF_toPointI (loc1);
	pt1.x -= navMap->orig.x;
	pt1.y -= navMap->orig.y;
	pt2 = PointF_toPointI (loc2);
	pt2.x -= navMap->orig.x;
	pt2.y -= navMap->orig.y;

	if (!allowOutsideLocalMap)
	{
		if (pt1.x < (0 + localMapEdge) || pt1.x > (LOC_MAP_DIMS - localMapEdge) ||
			pt1.y < (0 + localMapEdge) || pt1.y > (LOC_MAP_DIMS - localMapEdge) ||
			pt2.x < (0 + localMapEdge) || pt2.x > (LOC_MAP_DIMS - localMapEdge) ||
			pt2.y < (0 + localMapEdge) || pt2.y > (LOC_MAP_DIMS - localMapEdge))
		{
			return 1;
		}
	}

	if (navigation)
	{
		res = RobotCore_checkLine (pt1.x, pt1.y, pt2.x, pt2.y, navMap, RobotCore_equalsNarrowOccupied, 0, THREE_PIXELS, 0, &hitValue);
	}
	else
	{
		// Check line of signe (LOS)
		// If the COOP_LOC flag has been set, then we actually don't want to check this; the
		// LOS to a target will always be blocked.
		if (G_COOP_LOC_DONT_CHECK_LOS)
		{
			return 0;
		}
		res = RobotCore_checkLine (pt1.x, pt1.y, pt2.x, pt2.y, navMap, RobotCore_equalsActualOccupied, 0, THREE_PIXELS, 0, &hitValue);
	}
	return res;
}


