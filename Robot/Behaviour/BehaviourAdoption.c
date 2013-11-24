#include "../../Common/RobotDefs.h"

#if !defined(BOARD)

#include "BehaviourAdoption.h"
#include "BehaviourCore.h"
#include "Supervision.h"
#include "../Ik/Ik.h"
#include "../../Common/Actuator.h"
#include "../../Common/Maths.h"

void RobotWrite_writeLeaveProvisionalMapData (RobotDatabase *db);

void adoptAVAILABLE (RobotDatabase *db, const int leaveProvisionalMapData)
{
	BEHAVIOUR behaviour;
	ListNode *iter;
	BasicBehaviourData *data;

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<AdoptBehaviour>behaviour=\"AVAILABLE\"</AdoptBehaviour>\n");
#endif

	behaviour = db->behaviourData.behaviour;
	if (leaveProvisionalMapData)
	{
		if (FOLLOW_PATH == behaviour || BACKTRACK == behaviour)
		{
			iter = db->behaviourData.stack.back->prev;
			data = (BasicBehaviourData*)iter->value;
			behaviour = data->id;
		}
		if (behaviour == CLOSE_LOOP)
		{
			RobotWrite_writeLeaveProvisionalMapData (db);
		}
	}
	if (behaviour == GOTO_EXP_PT ||
		behaviour == GOTO_EXP_PT_COLLAB)
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<AbortGTEPSession>reason=\"AVAILABLE\" potentialGtepSessionId=%d</AbortGTEPSession>\n", db->behaviourData.gotoExpPt.nextSessionIndex);
#endif
	}

	List_clear (&db->behaviourData.stack, 0);
	List_pushValue (&db->behaviourData.stack, &db->behaviourData.available);
	BehaviourCore_printStack (&db->behaviourData.stack, "adopt avail");
	db->behaviourData.behaviour = AVAILABLE;
	db->behaviourData.baseBehaviour = AVAILABLE;
	db->behaviourData.targetIndex = -1;
}



void adoptStuck (RobotDatabase *db)
{
	BEHAVIOUR behaviour;
	ListNode *iter;
	BasicBehaviourData *data;

	behaviour = db->behaviourData.behaviour;
	if (FOLLOW_PATH == behaviour || BACKTRACK == behaviour)
	{
		iter = db->behaviourData.stack.back->prev;
		data = (BasicBehaviourData*)iter->value;
		behaviour = data->id;
	}
	if (behaviour == CLOSE_LOOP)
	{
		RobotWrite_writeLeaveProvisionalMapData (db);
	}
	if (behaviour == GOTO_EXP_PT ||
		behaviour == GOTO_EXP_PT_COLLAB)
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<AbortGTEP>reason=\"STUCK\" potentialGtepSessionId=%d</AbortGTEP>\n", db->behaviourData.gotoExpPt.nextSessionIndex);
#endif
	}

	List_clear (&db->behaviourData.stack, 0);
	List_pushValue (&db->behaviourData.stack, &db->behaviourData.stuck);
	BehaviourCore_printStack (&db->behaviourData.stack, "adopt stuck");
	db->behaviourData.behaviour = STUCK;
	db->behaviourData.baseBehaviour = STUCK;
	db->behaviourData.targetIndex = -3;
	db->behaviourData.stuck.timeAdopted = db->status.nIterations;

	// May have been an explorer or supervisor
	Supervision_leaveCoalition (db, 1, "STUCK");

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<AdoptBehaviour>behaviour=\"STUCK\" timeAdopted=%d</AdoptBehaviour>\n", db->behaviourData.stuck.timeAdopted);
#endif
}


float BehaviourAdoption_getBehaviourRatio (
	RobotDatabase *db,
	const BEHAVIOUR behaviour)
{
	switch (behaviour)
	{
	case EXPLORATION:
		return db->behaviourData.exploration.profit.ratio;
	case GOTO_EXP_PT:
		return db->behaviourData.gotoExpPt.profit.ratio;
	case GOTO_EXP_PT_COLLAB:
		return db->behaviourData.gotoExpPtCollab.profit.ratio;
	case CLOSE_LOOP:
		return db->behaviourData.supervision.profit.ratio;
	case BACKTRACK:
		assert(0);
	default:
		assert(0);
	}

	return MAX_FLT;
}

Dest* getBehaviourDest (RobotDatabase *db, const BEHAVIOUR behaviour)
{
	switch (behaviour)
	{
	case EXPLORATION:
		return &db->behaviourData.exploration.profit.dest;
	case GOTO_EXP_PT:
		return &db->behaviourData.gotoExpPt.profit.dest;
	case GOTO_EXP_PT_COLLAB:
		return &db->behaviourData.gotoExpPtCollab.profit.dest;
	case SUPERVISION:
		return &db->behaviourData.supervision.profit.dest;
	case FOLLOW_PATH:
		return &db->behaviourData.followPath.dest;
	case BACKTRACK:
		return &db->behaviourData.backtrack.dest;
	case CLOSE_LOOP:
		return &db->behaviourData.closeLoop.profit.dest;
	default:
		return 0;
	}
}

void BehaviourAdoption_checkCoarseNavMoves (
	RobotDatabase *db,
	const Pose pose,
	Dest *dest,
	IKData *currentIKData)
{
	Pose tempPose = pose;
	Dest tempDest = *dest;
	IKData ikData = *currentIKData;

	// No point in checking state for recon=2, as we don't have a dest/target in
	// this situation, instead we as just moving to try to establish visual
	// contact.
	if (db->behaviourData.behaviour == CLOSE_LOOP &&
		db->behaviourData.closeLoop.reconFlag == RECON_PANNING)
	{
		return;
	}

	while (1)
	{
		if (tempDest.type & IS_NEW_DEST || ikData.index == -1 || ikData.index >= ikData.len)
		{
			// Reset dest and ikData each time we recalc move. Do not reset pose, as for
			// coarse nav, we want to be able to calc and move incrementally.
			tempDest = *dest;
			ikData = *currentIKData;

			tempDest.type &= ~IS_NEW_DEST;
			IK_determineMove (
				db->xmlLog,
				db->environment.navMap,
				&tempPose,
				&tempDest,
				&ikData,
				&db->geometryConstants,
				&db->ikConstants,
				db->behaviourData.behaviour,
				0,
				0,
//				DONT_ALLOW_OUTSIDE_LOCAL_MAP,
				DO_ALLOW_OUTSIDE_LOCAL_MAP, 
				0,
				1,
				0);

			if (tempDest.type & IS_AT_DEST)
			{
				// For coarse nav destinations, we iteratively calc and move, so when we achieve atDest, this just
				// means that we are on the right track, not that we are actually there
#ifdef PRINT_EVENTS
//				fprintf (db->xmlLog, "<CheckCoarseNavMoves>event=\"atDest\"</CheckCoarseNavMoves>\n");
#endif
				return;
			}
			else if (tempDest.type & IS_IK_CURRENTLY_IMPOSSIBLE)
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckCoarseNavMoves>event=\"ikImpossible\"</CheckCoarseNavMoves>\n");
#endif
				dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
				return;
			}
			else if (tempDest.type & IS_PATH_REQUIRED)
			{
				// In IK_determineMove for CLOSE_LOOP where recon=1/2, this should never happen
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckCoarseNavMoves>event=\"pathReqd\"</CheckCoarseNavMoves>\n");
#endif
				dest->type |= IS_PATH_REQUIRED;
				return;
			}

			// In checkMoves_checkState we reset dest and ikData at this point, but not
			// necessary here, as we can just do this at the top of each loop
		}

		Actuator_executeMove(
			&tempPose,
			ikData.moves[ikData.index],
			0,
			0,
			NULL);

		DEBUG_ASSERT(ikData.moves[ikData.index].n > 0)
		if (0 == --ikData.moves[ikData.index].n)
		{
			++ikData.index;
		}
	}
}

void checkMoves_checkState (RobotDatabase *db,
							const Pose pose,
							Dest *dest,
							IKData *currentIKData)
{
	Pose tempPose = pose;
	Dest tempDest = *dest;
	IKData ikData = *currentIKData;
	IKData backupIkData;
	int isNewIK = 0;
	int haveRecalcdIK = 0;

//	if (!(tempDest.type & IS_AT_DEST) && (tempDest.type & IS_NEW_DEST || ikData.index == -1))
	if (tempDest.type & IS_NEW_DEST || ikData.index == -1)
	{
		isNewIK = 1;

		tempDest.type &= ~IS_NEW_DEST;
		IK_determineMove (db->xmlLog, db->environment.navMap, &tempPose, &tempDest, &ikData, &db->geometryConstants, &db->ikConstants, db->behaviourData.behaviour, 0, 0, DONT_ALLOW_OUTSIDE_LOCAL_MAP, 0, 1, 0);

		if (tempDest.type & IS_AT_DEST)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CheckState>event=\"atDestStraightOff\"</CheckState>\n");
#endif
			return;
		}
		else if (tempDest.type & IS_IK_CURRENTLY_IMPOSSIBLE)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CheckState>event=\"ikImpossibleStraightOff\"</CheckState>\n");
#endif
			dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
			return;
		}
		else if (tempDest.type & IS_PATH_REQUIRED)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CheckState>event=\"pathReqdStraightOff\"</CheckState>\n");
#endif
			dest->type |= IS_PATH_REQUIRED;
			return;
		}
	}

	while (1)
	{
		if (db->behaviourData.behaviour == CLOSE_LOOP &&
			db->behaviourData.closeLoop.reconFlag == RECON_PANNING)
		{
			return;
		}

		if (ikData.index >= ikData.len)
		{
			if (haveRecalcdIK)
			{
				// If we have recalc'd, and this turns out to work, then we can use the new IK data
				backupIkData = ikData;
			}

			// Check if current moves have been successful.
			// Use slightly less strict leeway as numerical accuracy issues may occur.
			tempDest.type &= ~IS_NEW_DEST;
			IK_determineMove (db->xmlLog, db->environment.navMap, &tempPose, &tempDest, &ikData, &db->geometryConstants, &db->ikConstants, db->behaviourData.behaviour, 0, 0, DONT_ALLOW_OUTSIDE_LOCAL_MAP, 0, 0, 1);

			if (tempDest.type & IS_AT_DEST)
			{
				if (haveRecalcdIK)
				{
#ifdef PRINT_EVENTS
					fprintf (db->xmlLog, "<CheckState>event=\"switchIk\" ");
					IK_printMoves (&backupIkData, db->xmlLog);
					fprintf (db->xmlLog, "</CheckState>\n");
#endif
					// The current IK didn't work, but when we recalcd, this did work
					//dest->type |= IS_NEW_DEST;
					*currentIKData = backupIkData;
				}
				else
				{
#ifdef PRINT_EVENTS
					fprintf (db->xmlLog, "<CheckState>event=\"ikFine\"</CheckState>\n");
#endif
				}
				return;
			}
			if (haveRecalcdIK || isNewIK)
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckState>event=\"ikFailed\"</CheckState>\n");
#endif
				dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
				return;
			}
			if (tempDest.type & IS_IK_CURRENTLY_IMPOSSIBLE)
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckState>event=\"ikImpossible\"</CheckState>\n");
#endif
				dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
				return;
			}
			if (tempDest.type & IS_PATH_REQUIRED)
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckState>event=\"pathReqd\"</CheckState>\n");
#endif
				dest->type |= IS_PATH_REQUIRED;
				return;
			}

			// If the IK that we're currently using doesn't work (due to compass error) then either
			// recalc or quit, based on whatever we reckon will be more efficient
			if (1)
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckState>event=\"noDiceWithCurrentMovesSoRecalcing\"</CheckState>\n");
#endif
			}
			else
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckState>event=\"noDiceWithCurrentMovesSoQUITTING\"</CheckState>\n");
#endif
				dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
				return;
			}

			// Either not at dest, or an error didn't occur, so recalc IK, and if 
			// we are going to recalc, then do so from the actual (i.e. not temp) pose and dest
			tempPose = pose;
			tempDest = *dest;
			ikData = *currentIKData;

			// Use more strict leeway
			tempDest.type &= ~IS_NEW_DEST;
			IK_determineMove (db->xmlLog, db->environment.navMap, &tempPose, &tempDest, &ikData, &db->geometryConstants, &db->ikConstants, db->behaviourData.behaviour, 0, 0, DONT_ALLOW_OUTSIDE_LOCAL_MAP, 0, 1, 0);

			// Just check for failure or immediate success from recalc'd IK
			if (tempDest.type & IS_AT_DEST)
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckState>event=\"actuallyAtDest\"</CheckState>\n");
#endif
				// Set IK data such that BehaviourControl_implementBehaviour will realise that
				// we are currently at the dest
				currentIKData->index = -1;
				return;
			}
			if (tempDest.type & IS_IK_CURRENTLY_IMPOSSIBLE)
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckState>event=\"ikImpossibleAfterRecalc\"</CheckState>\n");
#endif
				dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
				return;
			}
			if (tempDest.type & IS_PATH_REQUIRED)
			{
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckState>event=\"pathReqdAfterRecalc\"</CheckState>\n");
#endif
				dest->type |= IS_PATH_REQUIRED;
				return;
			}

			haveRecalcdIK = 1;
		}

		Actuator_executeMove (&tempPose, ikData.moves[ikData.index], 0, 0, NULL);

		DEBUG_ASSERT(ikData.moves[ikData.index].n > 0)
		if (0 == --ikData.moves[ikData.index].n)
		{
			++ikData.index;
		}
	}
}

extern int g_forceQuit;

void BehaviourCore_manualBehaviour (RobotDatabase *db)
{
	Move move;
	char input[128];
	int quit = 0;

	printf ("wasd q=quit e=longMove >");
	gets (input);
	fflush (stdin);

	switch (input[0])
	{
	case 'w':
		move.dir = 0;
		move.n = 1;
		move.moveDist = BASE_MOVE_DELTA_FWD;
		move.usBurst = calcMoveMotorBurst (BASE_MOVE_DELTA_FWD);
		break;
	case 's':
		break;
	case 'a':
		move.dir = 2;
		move.n = 1;
		move.moveDist = 0.0f;
		move.usBurst = 0;
		break;
	case 'd':
		move.dir = 3;
		move.n = 1;
		move.moveDist = 0.0f;
		move.usBurst = 0;
		break;
	case 'e':
		move.dir = 0;
		move.n = 1;
		move.moveDist = db->geometryConstants.nav_maxDist;
		move.usBurst = calcMoveMotorBurst (db->geometryConstants.nav_maxDist);
		break;
	case 'q':
		quit = 1;
		break;
	}
	printf ("input=%c\n", input[0]);

	if (quit)
	{
		g_forceQuit = 1;
	}
	else
	{
		Actuator_executeMove (&db->status.pose, move, 1, db->status.nIterations, db->xmlLog);
		db->status.isNewPose = 1;
		db->behaviourData.behaviourAtLastMove = db->behaviourData.behaviour;
		db->behaviourData.targetAtLastMove = db->behaviourData.targetIndex;
	}
}

BEHAVIOUR getBehaviourToAccredit (RobotDatabase *db)
{
	BasicBehaviourData *data; 
	if (db->behaviourData.behaviourAtLastMove == FOLLOW_PATH && db->behaviourData.behaviour == FOLLOW_PATH)
	{
		data = (BasicBehaviourData*)db->behaviourData.stack.back->prev->value;
		return data->id;
	}
	return db->behaviourData.behaviourAtLastMove;
}

int getTargetToAccredit (RobotDatabase *db)
{
	return db->behaviourData.targetAtLastMove;
}












#endif // !defined(BOARD)



