#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "Backtrack.h"
#include "BehaviourAdoption.h"
#include "BehaviourCore.h"
#include "../../Common/RobotCore.h"

int Backtrack_calcBacktrackDest (RobotDatabase *db)
{
	PointI loc;
	PointI locBL;
	PointI cellBL, cellTR, cellCentre;
	PointI minPt;
	int minDist;
	int distToFreeTerrain;
	int i, j;
	int n;
	int celli, cellj;
	Image *nav;
	Dest *dest;
	uchar hitValue;
	int cellHalfSize = db->behaviourData.backtrack.cellHalfSize;
	int cellSize = cellHalfSize * 2;
	int cellOffsets[] = {
		-4*cellSize, -3*cellSize, -2*cellSize, -1*cellSize,
		 1*cellSize,  2*cellSize,  3*cellSize,  4*cellSize};

	nav = db->environment.navMap;
	loc = PointF_toPointI (db->status.pose.loc);
	loc.x -= nav->orig.x;
	loc.y -= nav->orig.y;
	locBL.x = loc.x - cellHalfSize;
	locBL.y = loc.y - cellHalfSize;

	minDist = MAX_INT32;
	for (i = 0; i < 8; ++i)
	{
		cellBL.x = locBL.x + cellOffsets[i];
		cellTR.x = cellBL.x + cellSize;
		for (j = 0; j < 8; ++j)
		{
			cellBL.y = locBL.y + cellOffsets[j];
			cellTR.y = cellBL.y + cellSize;
			if (Image_isWithinBounds_ignoreMapOrig (nav, cellBL.x, cellBL.y) &&
				Image_isWithinBounds_ignoreMapOrig (nav, cellTR.x, cellTR.y))
			{
				n = 0;
				for (celli = cellBL.x; celli < cellTR.x; ++celli)
				{
					for (cellj = cellBL.y; cellj < cellTR.y; ++cellj)
					{
						n += (FREE_TERRAIN != Image_getPixel_dontCheck (nav, celli, cellj));
					}
				}
				if (!n)
				{
					cellCentre.x = cellBL.x + cellHalfSize;
					cellCentre.y = cellBL.y + cellHalfSize;
					distToFreeTerrain = RobotCore_checkLine (loc.x, loc.y, cellCentre.x, cellCentre.y, nav, uchar_equals, FREE_TERRAIN, THREE_PIXELS, 1, &hitValue);
					if (distToFreeTerrain && distToFreeTerrain < minDist)
					{
						minPt = cellCentre;
						minDist = distToFreeTerrain;
					}
				}
			}
		}
	}

	if (minDist != MAX_INT32)
	{
		minPt.x += nav->orig.x;
		minPt.y += nav->orig.y;
		dest = &db->behaviourData.backtrack.dest;
		dest->type = IS_COARSE_NAV_OK; // Not target-centric, coarse navigation
		dest->dest.loc = PointI_toPointF (minPt);
		dest->dest.orient = 0.0f;
		dest->leeway = BACKTRACK_LEEWAY;

		return 1;
	}
	return 0;
}

int isBacktrackSuccessful (RobotDatabase *db, const PointF pt, const int cellHalfSize)
{
	int cellSize = cellHalfSize * 2;
	PointI bl, tr;
	int i, j, n;
	Image *navMap = db->environment.navMap;

	bl = PointF_toPointI (pt);
	bl.x -= navMap->orig.x;
	bl.y -= navMap->orig.y;

	bl.x -= cellHalfSize;
	bl.y -= cellHalfSize;
	if (!Image_isWithinBounds_ignoreMapOrig (navMap, bl.x, bl.y))
	{
		return 0;
	}

	tr.x = bl.x + cellSize;
	tr.y = bl.y + cellSize;
	if (!Image_isWithinBounds_ignoreMapOrig (navMap, tr.x, tr.y))
	{
		return 0;
	}

	n = 0;
	for (i = bl.x; i < tr.x; ++i)
	{
		for (j = bl.y; j < tr.y; ++j)
		{
			n += (FREE_TERRAIN != Image_getPixel_dontCheck (navMap, i, j));
		}
	}

	return !n;
}

int revertToAvailable (RobotDatabase *db, const BEHAVIOUR behaviour)
{
	BasicBehaviourData *data;
	int revert = 0;

	if (behaviour == FOLLOW_PATH)
	{
		data = (BasicBehaviourData*)db->behaviourData.stack.back->prev->value;
		if (data->id == EXPLORATION || data->id == GOTO_EXP_PT || data->id == GOTO_EXP_PT_COLLAB)
		{
			revert = 1;
		}
	}
	else if (behaviour == EXPLORATION || behaviour == GOTO_EXP_PT || behaviour == GOTO_EXP_PT_COLLAB)
	{
		revert = 1;
	}

	// Only revert to AVAILABLE if not losing any state.
	if (revert)
	{
		// Do not leave provisional map data here. Backgrack was successful, so the robot may
		// be able to calc a path in order to perform loop closing.
		adoptAVAILABLE (db, 0);
	}
	return revert;
}

void ceaseBacktrack (RobotDatabase *db)
{
	ListNode *listNode;
	BasicBehaviourData *data;
	Dest *dest;

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<BacktrackSuccessful />\n");
#endif

	listNode = db->behaviourData.stack.back;
	List_deleteElement (&db->behaviourData.stack, &listNode, 0, 0);
	BehaviourCore_printStack (&db->behaviourData.stack, "cease bt");

	listNode = db->behaviourData.stack.back;
	data = (BasicBehaviourData*)listNode->value;

	db->behaviourData.backtrack.cellHalfSize = 4;

	if (!revertToAvailable (db, data->id))
	{
		db->behaviourData.behaviour = data->id;
		dest = getBehaviourDest (db, db->behaviourData.behaviour);
		if (data->id == FOLLOW_PATH)
		{
			dest->type |= IS_PATH_REQUIRED; // Recalculate path.
		}
		else if (dest)
		{
			dest->type |= IS_NEW_DEST; // Recalculate IK moves.
		}
	}
}

void Backtrack_checkBactrackState (RobotDatabase *db)
{
	if (isBacktrackSuccessful (db, db->status.pose.loc, 2))
	{
		ceaseBacktrack (db);
	}
	// Before we even get to the dest, verify that it is still a valid dest.
	else if (!isBacktrackSuccessful (db, db->behaviourData.backtrack.dest.dest.loc, 3))
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<BacktrackResetRequired />\n");
#endif
		// Current dest will not be ok, so reset.
		db->behaviourData.backtrack.dest.type |= DOES_DEST_REQ_BACKTRACK;
	}
}

void adoptStuckBACKTRACK (RobotDatabase *db)
{
	// Reset cellHalfSize; this may have been updated
	db->behaviourData.backtrack.cellHalfSize = 4;
	adoptStuck (db);
}

void isAtDestBACKTRACK (RobotDatabase *db)
{
	if (isBacktrackSuccessful (db, db->status.pose.loc, 2))
	{
		ceaseBacktrack (db);
	}
	else
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<BacktrackFail />\n");
#endif
//		adoptStuckBACKTRACK (db);

		// Current dest will not be ok, so reset. Ensure that next dest will be ok.
		++db->behaviourData.backtrack.cellHalfSize;
		db->behaviourData.backtrack.dest.type |= DOES_DEST_REQ_BACKTRACK;
	}
}

Dest* Backtrack_setBehaviour (RobotDatabase *db)
{
	Dest *dest;
	dest = &db->behaviourData.backtrack.dest;
	db->behaviourData.behaviour = BACKTRACK;
	db->behaviourData.targetIndex = -2;
	List_pushValue (&db->behaviourData.stack, &db->behaviourData.backtrack);
	BehaviourCore_printStack (&db->behaviourData.stack, "set bt");
	dest->type = IS_COARSE_NAV_OK | IS_NEW_DEST; // Coarse nav and new dest
	return dest;
}

int Backtrack_checkIfNecessary (RobotDatabase *db)
{
	PointI pt;
	uchar val;
	Dest *dest;

	pt = PointF_toPointI (db->status.pose.loc);
	pt.x -= db->environment.navMap->orig.x;
	pt.y -= db->environment.navMap->orig.y;

	val = Image_getPixel_dontCheck (db->environment.navMap, pt.x, pt.y);
	if (val < BROAD_VEHICLE_TERRAIN)
	{
/*		// If performing close loop, then we don't want to report collisions with the supervisor
		if (RobotMapProcessing_isMarkedAsRobot (val) &&
			(db->behaviourData.behaviour == CLOSE_LOOP ||
			(db->behaviourData.behaviour == FOLLOW_PATH &&
			BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP)))
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<BacktrackNotNecessary>value=%d pixel=(%d,%d)</BacktrackNotNecessary>\n", val, pt.x, pt.y);
#endif
			return 0;
		}*/

		dest = getBehaviourDest (db, db->behaviourData.behaviour);
		if (dest)
		{
			dest->type |= DOES_DEST_REQ_BACKTRACK;
		}
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<BacktrackNecessary>value=%d pixel=(%d,%d)</BacktrackNecessary>\n", val, pt.x, pt.y);
#endif
		return 1;
	}
	return 0;
}

void Backtrack_reset (RobotDatabase *db)
{
	ListNode *node;
	BasicBehaviourData *basicData;
	BEHAVIOUR id;

	node = db->behaviourData.stack.back;
	List_deleteElement (&db->behaviourData.stack, &node, 0, 0);
	BehaviourCore_printStack (&db->behaviourData.stack, "rest bt");
	basicData = (BasicBehaviourData*)db->behaviourData.stack.back->value;
	id = basicData->id;
	db->behaviourData.behaviour = id;
}

void adoptBacktrack (RobotDatabase *db)
{
	Dest *dest;
#ifdef PRINT_EVENTS
	PointF pt;
#endif

	if (db->behaviourData.behaviour == BACKTRACK)
	{
		Backtrack_reset (db);
		getBehaviourDest (db, db->behaviourData.behaviour);
	}

	if (Backtrack_calcBacktrackDest (db))
	{
		dest = Backtrack_setBehaviour (db);

#ifdef PRINT_EVENTS
		pt = getTargetToPrint (dest);
		fprintf (db->xmlLog, "<CalcBacktrackDest>pose=(%f,%f,%f) dest=(%f,%f)</CalcBacktrackDest>\n",
			db->status.pose.loc.x, db->status.pose.loc.y, db->status.pose.orient, pt.x, pt.y);
#endif
	}
	else
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<CalcBacktrackDestFail />\n");
#endif
		adoptStuckBACKTRACK (db);
	}
}

#endif

