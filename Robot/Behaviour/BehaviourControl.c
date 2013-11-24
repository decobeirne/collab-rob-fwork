#include "../../Common/RobotDefs.h"

#ifndef BOARD


#include "BehaviourControl.h"
#include "BehaviourAdoption.h"
#include "Exploration.h"
#include "GotoExpPt.h"
#include "GotoExpPtCollab.h"
#include "FollowPath.h"
#include "Backtrack.h"
#include "CloseLoop.h"
#include "Supervision.h"
#include "../Sensors/SensorProcessing.h"
#include "../Sensors/MapProcessing.h"
#include "../Ik/Ik.h"
#include "../Comm/RobotWrite.h"




BasicBehaviourData* getBehaviourData (RobotDatabase *db, const BEHAVIOUR behaviour)
{
	switch (behaviour)
	{
	case EXPLORATION:
	default:
		return (BasicBehaviourData*)&db->behaviourData.exploration;
		break;
	case GOTO_EXP_PT:
		return (BasicBehaviourData*)&db->behaviourData.gotoExpPt;
		break;
	}
}

extern const char* behaviourHandles[];

void setBehaviour (RobotDatabase *db, const BEHAVIOUR behaviour)
{
	switch (behaviour)
	{
	case AVAILABLE:
	default:
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<NoUsefulBehaviourFound />\n");
#endif
		adoptStuck (db);
		break;
	case BACKTRACK:
		adoptBacktrack (db);
		break;
	case EXPLORATION:
		Exploration_adopt (db);
		break;
	case GOTO_EXP_PT:
		GotoExpPt_adopt (db);
		break;
	case GOTO_EXP_PT_COLLAB:
		GotoExpPtCollab_adopt (db);
		break;
	case CLOSE_LOOP:
		adoptCLOSE_LOOP (db, 0);
	}

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<AdoptBehaviour>behaviour=\"%s\" targetIndex=%d", behaviourHandles[behaviour], db->behaviourData.targetIndex);
	if (behaviour == GOTO_EXP_PT ||
		behaviour == GOTO_EXP_PT_COLLAB)
	{
		fprintf (db->xmlLog, " potentialGtepSessionId=%d", db->behaviourData.gotoExpPt.nextSessionIndex);
	}
	fprintf (db->xmlLog, "</AdoptBehaviour>\n");
#endif
}

int BehaviourControl_retryPath (RobotDatabase *db)
{
	if (db->behaviourData.behaviour == FOLLOW_PATH)
	{
		if (db->behaviourData.baseBehaviour == SUPERVISION ||
			db->behaviourData.baseBehaviour == GOTO_EXP_PT ||
			db->behaviourData.baseBehaviour == CLOSE_LOOP)
		{
			if (db->behaviourData.followPath.nPathTries < 5)
			{
				++db->behaviourData.followPath.nPathTries;
				return 1;
			}
			else
			{
				// Reset, as we are adopting available
				db->behaviourData.followPath.nPathTries = 0;
			}
		}
	}
	return 0;
}

void BehaviourControl_checkNavigationState (RobotDatabase *db)
{
	BEHAVIOUR behaviour = db->behaviourData.behaviour;
	Dest *dest = getBehaviourDest (db, db->behaviourData.behaviour);
//	int checkPaths = db->behaviourData.checkPaths;
	float nMoves, stdDev;
	float dummy;
	ListNode *iterator;
	BasicBehaviourData *data;
	Dest *prevDest;
	Dest tempPrevDest;
	List obstdCellGridResetList;
	int isResetReqd = 0;
	int shouldReturn;

	if (db->sensorData.isDataPendingInc || db->behaviourData.behaviour == STUCK)
	{
		return;
	}

	if (db->behaviourData.behaviour == SUPERVISION && db->behaviourData.supervision.profit.dest.type & IS_AT_DEST)
	{
//		return;
	}

	TIMER_START ("checkNavigationState")

	db->behaviourData.checkPaths = 0;

	if (EXPLORATION == db->behaviourData.behaviour ||
		(FOLLOW_PATH == db->behaviourData.behaviour &&
		((BasicBehaviourData*)db->behaviourData.stack.back->prev->value)->id == EXPLORATION))
	{
//		if (Exploration_checkNavigationState (db, dest))
		{
#ifdef PRINT_EVENTS
//			fprintf (db->xmlLog, "<CheckState>event=\"targetMapped\"</CheckState>\n");
#endif
//			return;
		}
	}

	// Unfortunate that we may potentially do this more than once. If it looks like a performance hit, then
	// the structure can be re-worked later.
	if (CLOSE_LOOP == db->behaviourData.behaviour ||
		(FOLLOW_PATH == db->behaviourData.behaviour && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
	{
		obstdCellGridResetList = initList();
		CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
		isResetReqd = 1;
	}

	shouldReturn = 0;
	if (db->behaviourData.behaviour != BACKTRACK && Backtrack_checkIfNecessary (db))
	{
		shouldReturn = 1;
	}

	if (isResetReqd)
	{
		isResetReqd = 0;
		CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
		List_clear (&obstdCellGridResetList, 1);
	}
	if (shouldReturn)
	{
		return;
	}

	//if (CLOSE_LOOP == db->behaviourData.behaviour ||
	//	(FOLLOW_PATH == db->behaviourData.behaviour && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
	//{
	//	obstdCellGridResetList = initList();
	//	CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
	//	isResetReqd = 1;
	//}

	if (behaviour == BACKTRACK)
	{
		Backtrack_checkBactrackState (db);
	}
	//else if (behaviour == FOLLOW_PATH && checkPaths)
	else if (behaviour == FOLLOW_PATH)
	{
		iterator = db->behaviourData.stack.back->prev;
		data = (BasicBehaviourData*)iterator->value;
		prevDest = getBehaviourDest (db, data->id);
		tempPrevDest = *prevDest;
		nMoves = 0.0f;
		stdDev = db->status.stdDev;

		DEBUG_ASSERT(CAM_P_OFFSET != MIN_FLT)

		// Less-broad removal of sup from nav map
		// It was found that removing the sup at the top level within this function was
		// resulting in the robot running into the sup
		if (BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP)
		{
			obstdCellGridResetList = initList();
			CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
			isResetReqd = 1;
		}

		checkPathMoves (
			&db->behaviourData.followPath,
			db->xmlLog,
			db->environment.navMap,
			db->sensorData.localMap,
			&db->geometryConstants,
			&db->camVectors,
			&db->ikConstants,
			&db->uncertaintyConstants,
			CAM_P_OFFSET,
			db->status.pose,
			&tempPrevDest,
			&nMoves,
			&stdDev,
			1,
			0,
			&dummy);

		if (isResetReqd)
		{
			isResetReqd = 0;
			CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
			List_clear (&obstdCellGridResetList, 1);
		}

		if (tempPrevDest.type & IS_PATH_REQUIRED || tempPrevDest.type & IS_IK_CURRENTLY_IMPOSSIBLE)
		{


#if 0 // new
			if (BehaviourControl_retryPath (db)) // Increment n tries here
			{
				dest->type &= ~IS_IK_CURRENTLY_IMPOSSIBLE;
				dest->type |= IS_PATH_REQUIRED;
			}
			else
			{
				dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<CheckState>event=\"FAIL\" type=\"path\"</CheckState>\n");
#endif
			}
#endif // new


#if 1 // original - prob revert to this
			dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CheckState>event=\"FAIL\" type=\"path\"</CheckState>\n");
#endif

#endif // original - prob revert to this


#if 0 // crap
			// Do not fail immediately, as path may have failed because the robot is blocked by
			// another robot.
			tempPrevDest = *prevDest;
			nMoves = 0.0f;
			stdDev = db->status.stdDev;


			{
				obstdCellGridResetList = initList();
				CloseLoop_removeAllRobsFromNavMap (db, &obstdCellGridResetList);
				isResetReqd = 1;
			}

			checkPathMoves (
				&db->behaviourData.followPath,
				db->xmlLog,
				db->environment.navMap,
				db->sensorData.localMap,
				&db->geometryConstants,
				&db->camVectors,
				&db->ikConstants,
				&db->uncertaintyConstants,
				CAM_P_OFFSET,
				db->status.pose,
				&tempPrevDest,
				&nMoves,
				&stdDev,
				1,
				0,
				&dummy);

			if (isResetReqd)
			{
				isResetReqd = 0;
				CloseLoop_replaceAllRobsInNavMap (db, &obstdCellGridResetList);
				List_clear (&obstdCellGridResetList, 1);
			}
#endif // crap


		}
#if 0 // crap
		else
		{
			// If we have been trying to get around another robot that was blocking our path,
			// are we have now succeeded, then we should reset the try-count.
			db->behaviourData.followPath.nTries = 0;
		}
#endif // crap
	}
	else if (
		dest && dest->type & IS_COARSE_NAV_OK)
	{
		if (dest->type & IS_PATH_REQUIRED)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CheckState>event=\"SKIP_AS_IS_PATH_REQUIRED\"</CheckState>\n");
#endif
			// If this flag is set, then we will calc the path in lowLevelBehaviour just after checking nav state
			return;
		}

		if (db->behaviourData.behaviour == CLOSE_LOOP ||
			(FOLLOW_PATH == db->behaviourData.behaviour && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
		{
			// For recon, we are limited to small moves, and we know that our dest is the supervisor, so we
			// can remove it from the nav map completely
			if (db->behaviourData.closeLoop.reconFlag != NOT_DOING_RECON)
			{
				obstdCellGridResetList = initList();
				CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
				isResetReqd = 1;
			}
		}

		BehaviourAdoption_checkCoarseNavMoves (
			db,
			db->status.pose,
			dest,
			&db->status.ikData);

		if (isResetReqd)
		{
			isResetReqd = 0;
			CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
			List_clear (&obstdCellGridResetList, 1);
		}

		if (dest->type & IS_IK_CURRENTLY_IMPOSSIBLE)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CheckState>event=\"FAIL\" type=\"coarseNavIk\"</CheckState>\n");
#endif
			// May have to leave provisional map data as dest is currently not reachable
			adoptAVAILABLE (db, 1);
		}
	}
	// Check IK *every* time, not just when nav map has changed
	else if (dest /*&& checkPaths*/)
	{
		if (dest->type & IS_PATH_REQUIRED)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CheckState>event=\"SKIP_AS_IS_PATH_REQUIRED\"</CheckState>\n");
#endif
			return;
		}

		// Don't remove sup from nav map, as we don't want to run into it, but when checking
		// LOS, always return true. If we are performing COOP_LOC, then it can be assumed
		// that LOS will always fail otherwise
		if (db->behaviourData.behaviour == CLOSE_LOOP ||
			(FOLLOW_PATH == db->behaviourData.behaviour && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
		{
			// For recon, we are limited to small moves, and we know that our dest is the supervisor, so we
			// can remove it from the nav map completely
//			if (db->behaviourData.closeLoop.isAttemptingRecon)
//			{
//				obstdCellGridResetList = initList();
//				CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
//				isResetReqd = 1;
//			}
			G_COOP_LOC_DONT_CHECK_LOS = 1;
		}

		checkMoves_checkState (db, db->status.pose, dest, &db->status.ikData);

//		if (isResetReqd)
//		{
//			isResetReqd = 0;
//			CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
//			List_clear (&obstdCellGridResetList, 1);
//		}
		G_COOP_LOC_DONT_CHECK_LOS = 0; // Reset

		if (dest->type & IS_IK_CURRENTLY_IMPOSSIBLE)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CheckState>event=\"FAIL\" type=\"ik\"</CheckState>\n");
#endif
			// May have to leave provisional map data as dest is currently not reachable
			adoptAVAILABLE (db, 1);
		}
	}

	//if (isResetReqd)
	//{
	//	isResetReqd = 0;
	//	CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
	//	List_clear (&obstdCellGridResetList, 1);
	//}

	TIMER_STOP (db->xmlLog, "checkNavigationState")
}

extern void RobotRead_readCoalitions (RobotDatabase *db);
extern void RobotWrite_writeProposalsConsidered (RobotDatabase *db);

#ifdef PRINT_PROFIT_DETAIL
void BehaviourControl_printCoalitions (FILE *f, const Coalition *expCoal, const List *supCoals)
{
	ListNode *iter;
	Coalition *coal;

	fprintf (f, "<ReadCoalitions>expCoalition=(bid=%f exp=%d sup=%d id=%d area=(%d,%d) stdDev=%f) supCoalitions=(", expCoal->bid, expCoal->explorer, expCoal->supervisor, expCoal->id, expCoal->area.x, expCoal->area.y, expCoal->stdDev);
	iter = supCoals->front;
	while (iter)
	{
		coal = (Coalition*)iter->value;
		fprintf (f, "(bid=%f exp=%d sup=%d id=%d area=(%d,%d) stdDev=%f),", coal->bid, coal->explorer, coal->supervisor, coal->id, coal->area.x, coal->area.y, coal->stdDev);
		iter = iter->next;
	}
	fprintf (f, ")</ReadCoalitions>\n");
}

const char* BehaviourControl_getProposalStatusStr (const int status)
{
	switch (status)
	{
	case 0:
		return "PROPOSAL_SUCCESSFUL";
	case 1:
		return "BID_SUCCESSFUL";
	case 2:
		return "PROPOSAL_PENDING";
	case 3:
		return "BID_PENDING";
	default:
		return "NO_PENDING_COALITION";
	}
}
#endif

void BehaviourControl_setCollaborativeBehaviour (RobotDatabase *db)
{
	PROPOSAL_STATUS status;
	int isCurrentlyInCoalition;

	if (db->sensorData.isDataPendingInc || db->behaviourData.behaviour == STUCK)
	{
		return;
	}

	if (db->behaviourData.supervision.isProposalMade || db->behaviourData.supervision.isBidMade)
	{
		do
		{
			status = Supervision_checkProposalStatus (db);
#ifdef PRINT_PROFIT_DETAIL
			fprintf (db->xmlLog, "<CheckProposalStatus>status=%d statusStr=\"%s\"</CheckProposalStatus>\n", status, BehaviourControl_getProposalStatusStr (status));
			fflush (db->xmlLog);
#endif
			if (PROPOSAL_PENDING == status)
			{
				// Avoid deadlock
				RobotWrite_writeProposalsConsidered (db);
#ifdef PRINT_PROFIT_DETAIL
				fprintf (db->xmlLog, "<WriteProposalsConsidered />\n");
#endif
			}
			SLEEP_FOR_MS(10)
		} while (BID_PENDING == status || PROPOSAL_PENDING == status);

		if (PROPOSAL_SUCCESSFUL == status)
		{
			RobotRead_readCoalitions (db);
#ifdef PRINT_PROFIT_DETAIL
			BehaviourControl_printCoalitions (db->xmlLog, &db->partnerData.explorationCoalition, &db->partnerData.supervisionCoalitions);
#endif
			adoptSUPERVISION (db);
			return;
		}
		else if (BID_SUCCESSFUL == status)
		{
			RobotRead_readCoalitions (db);
#ifdef PRINT_PROFIT_DETAIL
			BehaviourControl_printCoalitions (db->xmlLog, &db->partnerData.explorationCoalition, &db->partnerData.supervisionCoalitions);
#endif
			db->partnerData.coalitionIter = db->status.nIterations;

#if 1 // old
			// Leave any provisional map data that the robot has submitted: it is changing coalitions
			// so it will not be able to adjust this data.
			adoptAVAILABLE (db, 1);
#else // old
			// Adopt close loop - as robot's are going around with too high error otherwise, i.e. not appropriately
			// evaluating the utility of reducing error
			//
			// Don't want this close loop session to get analysed in post-processing though - so shouldn't match the
			// same pattern, <CloseLoopPerformed />, as those sessions
			//
			fprintf (db->xmlLog, "<AdoptCloseLoopInitially />\n");

			CloseLoop_calcDest (&db->behaviourData.closeLoop, db);

			adoptCLOSE_LOOP (db, 1); // initialSync=1
#endif // old

			return;
		}
	}

	// Either no proposal/bid made, or made but not successful.

	isCurrentlyInCoalition = (db->partnerData.explorationCoalition.id != -1); // This was looking not-coherant, but maybe not the source of the issue
	RobotRead_readCoalitions (db);
#ifdef PRINT_PROFIT_DETAIL
	BehaviourControl_printCoalitions (db->xmlLog, &db->partnerData.explorationCoalition, &db->partnerData.supervisionCoalitions);
#endif

	if (SUPERVISION == db->behaviourData.behaviour)
	{
		Supervision_checkExplorers (db, 0);
		return;
	}
	else if (isCurrentlyInCoalition)
	{
		if (db->partnerData.explorationCoalition.supervisor != -1)
		{
			// It was determined that independent exploration would be more profitable.
			if (GOTO_EXP_PT == db->behaviourData.maxBehaviour)
			{
				Supervision_leaveCoalition (db, 0, "adopting GOTO_EXP_PT");

#ifndef STRICT_EXP_BEH // When gtep can be adopted BEFORE all exp targets are exhausted
				GotoExpPt_setLocalMapExhausted (db);

#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<SetLocalMapExhausted>reason=\"setCollaborativeBehavior\"</SetLocalMapExhausted>\n");
#endif
#endif // STRICT_EXP_BEH

				GotoExpPt_adopt (db);
			}

			// Continue with whatever we were doing.
			if (db->behaviourData.behaviour == AVAILABLE)
			{
#ifdef PRINT_PROFIT_DETAIL
				fprintf (db->xmlLog, "<SetBehaviourInCoal />\n");
#endif
				setBehaviour (db, db->behaviourData.maxBehaviour);
			}
			else
			{
#ifdef PRINT_PROFIT_DETAIL
				fprintf (db->xmlLog, "<MaintainCurrentBehaviourInCoal />\n");
#endif
			}
			return;
		}
		else
		{
#ifdef PRINT_PROFIT_DETAIL
			fprintf (db->xmlLog, "<SupervisorQuit />\n");
#endif
			adoptAVAILABLE (db, 1);
			return;
		}
	}
	else
	{
		// Should have selected a new independent behaviour.
		if (db->behaviourData.behaviour == AVAILABLE)
		{
#ifdef PRINT_PROFIT_DETAIL
			fprintf (db->xmlLog, "<SetIndependentBehaviour />\n");
#endif
			setBehaviour (db, db->behaviourData.maxBehaviour);
		}
		else
		{
#ifdef PRINT_PROFIT_DETAIL
			fprintf (db->xmlLog, "<MaintainCurrentIndependentBehaviour />\n");
#endif
		}
	}
}

void BehaviourControl_lowLevelBehaviour (RobotDatabase *db)
{
	Dest *dest = getBehaviourDest (db, db->behaviourData.behaviour);
	PATH_RESULT pathResult;
	uchar *unreachableGrid;
	int hasUnreachableLocalMapGridBeenUpdated = 0;
#ifdef PRINT_EVENTS
	PointF pt;
#endif
	List obstdCellGridResetList;
	int isResetReqd = 0;

	if (db->sensorData.isDataPendingInc || !dest || db->behaviourData.behaviour == STUCK)
	{
		return;
	}

	TIMER_START ("lowLevelBehavior")

	// New path required
	if (dest->type & IS_PATH_REQUIRED)
	{
		dest->type &= ~IS_PATH_REQUIRED;

		if (db->behaviourData.behaviour == FOLLOW_PATH)
		{
			FollowPath_reset (db);
			dest = getBehaviourDest (db, db->behaviourData.behaviour);
		}

		if (db->behaviourData.behaviour == CLOSE_LOOP ||
			(db->behaviourData.behaviour == FOLLOW_PATH &&
			BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
		{
			obstdCellGridResetList = initList();
			CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
			isResetReqd = 1;
		}

		pathResult = FollowPath_calcPath (
			&db->status.pose,
			dest,
			&db->behaviourData.followPath,
			db->environment.navMap,
			db->environment.obstructedCellGrid,
			1);

		if (isResetReqd)
		{
			CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
			List_clear (&obstdCellGridResetList, 1);
		}

		if (ROBOT_INVALID == pathResult)
		{
			// Adopt backtrack. It is not necessary to remember that path planning is
			// required, as IK will be calculated again.
			dest->type |= DOES_DEST_REQ_BACKTRACK;
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CalcPath>result=\"ROBOT_INVALID\"</CalcPath>\n");
#endif
		}
		else if (PATH_OK != pathResult)
		{
			switch (db->behaviourData.behaviour)
			{
			case EXPLORATION:
				unreachableGrid = db->environment.unreachableExpCellGrid;
				break;

			case GOTO_EXP_PT:
			case GOTO_EXP_PT_COLLAB:
				unreachableGrid = db->environment.unreachableLocalMapGrid;
				break;

			case SUPERVISION:
				unreachableGrid = db->environment.unreachableSupCellGrid;
				break;

			case CLOSE_LOOP:
			case GOTO_SUPERVISOR:
			case BACKTRACK:
			case AVAILABLE:
			case STUCK:
			case COLLAB_NAV:
			case FOLLOW_PATH:
			default:
				unreachableGrid = NULL;
				break;
			}

			BehaviourCore_updateUnreachableGrid (
				db->xmlLog,
				db->status.index,
				db->status.pose,
				unreachableGrid,
				&hasUnreachableLocalMapGridBeenUpdated,
				dest,
				db->behaviourData.behaviour);

			if (hasUnreachableLocalMapGridBeenUpdated)
			{
				DEBUG_ASSERT(db->behaviourData.behaviour == GOTO_EXP_PT || db->behaviourData.behaviour == GOTO_EXP_PT_COLLAB)
				RobotWrite_updateUnreachableLocalMapGrid (
#if defined(SIMULATION) || defined(BOARD)
					db->board->environment.unreachableLocalMapGrid,
					&db->board->sensorData,
#else
					db,
#endif
					db->environment.unreachableLocalMapGrid,
					db->status.index);
			}

			// May have to leave provisional map data. Current dest is marked as unreachable
			// so it is assumed that no further processing can be done.
			adoptAVAILABLE (db, 1);
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CalcPath>result=%d</CalcPath>\n", pathResult);
#endif
		}
		else
		{
#if 1 // new
			db->behaviourData.followPath.nPathTries = 0; // Reset
#endif // new

#ifdef PRINT_EVENTS
			pt = getTargetToPrint (dest);
			fprintf (db->xmlLog, "<CalcPath>result=\"Success\" pose=(%f,%f,%f) target=(%f,%f) targetCentric=%d ",
				db->status.pose.loc.x, db->status.pose.loc.y, db->status.pose.orient, pt.x, pt.y, (int)(dest->type & IS_TARGET_CENTRIC));
			FollowPath_printNodes (db->xmlLog, &db->behaviourData.followPath.nodes);
			fprintf(db->xmlLog, "</CalcPath>\n");
#endif
			dest = FollowPath_setBehaviour (db);
			FollowPath_updateDest (db, &db->behaviourData.followPath);
		}
	}

	// Backtrack required
	if (dest->type & DOES_DEST_REQ_BACKTRACK)
	{
		dest->type &= ~DOES_DEST_REQ_BACKTRACK;
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<BacktrackRequired reason=\"pathFailedInvalidLoc\" />\n");
#endif
		adoptBacktrack (db);
		return;
	}

	TIMER_STOP (db->xmlLog, "lowLevelBehavior")
}

// CRAP
extern void foo (RobotDatabase *db);
extern int isIterToBreakOn;

#ifdef SETUP_TEST_COALITION
extern int g_payload;
#define TEST_COALITION_CLOSE_LOOP_COEFF 10.0f
#endif

void BehaviourControl_highLevelBehaviour (RobotDatabase *db)
{
	BEHAVIOUR maxBehaviour = AVAILABLE;
	float profit;
	float maxProfit = MIN_FLT;

	// This shouldn't really go here, but not too bad
	if (db->behaviourData.behaviour == SUPERVISION)
	{
		Supervision_checkExplorers (db, 1);
	}

	if (db->sensorData.isDataPendingInc || db->behaviourData.behaviour != AVAILABLE)
	{
		return;
	}
#ifdef PRINT_PROFIT
	fprintf (db->xmlLog, "<HighLevelBehavior />\n");
#endif

#ifdef COLLAB_EXP
	{
		Supervision_checkCollaborationState (db);
	}
#endif

	TIMER_START ("highLevelbehaviour")

	TIMER_START ("explorationProfit")
	Exploration_calcProfit (&db->behaviourData.exploration, db);
	profit = db->behaviourData.exploration.profit.ratio;
	if (profit > maxProfit)
	{
		maxProfit = profit;
		maxBehaviour = EXPLORATION;
	}
	TIMER_STOP (db->xmlLog, "explorationProfit")

	TIMER_START ("gtepProfit")
	GotoExpPt_calcProfit (&db->behaviourData.gotoExpPt, db);
	profit = db->behaviourData.gotoExpPt.profit.ratio;
	if (profit > maxProfit)
	{
		maxProfit = profit;
		maxBehaviour = GOTO_EXP_PT;
	}
	TIMER_STOP (db->xmlLog, "gtepProfit")

#ifdef COLLAB_EXP
	{
		TIMER_START ("gtepProfitCollab")
		// If in a coalition, should also look at the profit of leaving the coalition.
		// WARNING: this will break if GotoExpPtCollabData is changed
		GotoExpPt_calcProfitForCoal (&db->behaviourData.gotoExpPtCollab, db);
		profit = db->behaviourData.gotoExpPtCollab.profit.ratio;
		if (profit > maxProfit)
		{
			maxProfit = profit;
			maxBehaviour = GOTO_EXP_PT_COLLAB;
		}
		TIMER_STOP (db->xmlLog, "gtepProfitCollab")

		calcProfitCLOSE_LOOP (&db->behaviourData.closeLoop, db, maxProfit);
		profit = db->behaviourData.closeLoop.profit.ratio;
#ifdef SETUP_TEST_COALITION
		if (db->status.nIterations > g_payload)
		{
			profit += TEST_COALITION_CLOSE_LOOP_COEFF;
		}
#endif
		if (profit > maxProfit)
		{
			maxProfit = profit;
			maxBehaviour = CLOSE_LOOP;
		}

		//if (isIterToBreakOn)
		//{
		//	foo(db);
		//	cvWaitKey (0);
		//}
	
// Don't need to calc profit for sup if we know we're the only robot
#if !defined(SETUP_TEST_COALITION)
		if (db->behaviourData.behaviour != SUPERVISION)
		{
			TIMER_START ("supervisionProfit")
			calcProfitSUPERVISION (&db->behaviourData.supervision, db);
			profit = db->behaviourData.supervision.profit.ratio;
			if (profit > maxProfit && N_ROBOTS > 1)
			{
				db->behaviourData.supervision.makeProposal = 1;
			}
			TIMER_STOP (db->xmlLog, "supervisionProfit")
		}
#endif // defined(IS_GUMSTIX) && !defined(SETUP_TEST_COALITION)
	}
#endif // ifdef COLLAB_EXP

//	if ((AVAILABLE == maxBehaviour || CLOSE_LOOP == CLOSE_LOOP) && db->behaviourData.shouldBacktrack)
	if ((AVAILABLE == maxBehaviour || CLOSE_LOOP == maxBehaviour) && db->behaviourData.shouldBacktrack) // Not sure what deal was with weird condition above
//	if ((AVAILABLE == maxBehaviour || CLOSE_LOOP == maxBehaviour || SUPERVISION == maxBehaviour) && db->behaviourData.shouldBacktrack)
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<BacktrackRequired reason=\"noBehaviourInvalidLoc\" />\n");
#endif
		db->behaviourData.shouldBacktrack = 0;
		db->behaviourData.maxBehaviour = BACKTRACK;
	}
	else
	{
		db->behaviourData.maxBehaviour = maxBehaviour;
		db->behaviourData.maxProfit = maxProfit;
		db->behaviourData.lastMaxProfit = maxProfit;
	}

#ifndef COLLAB_EXP
	{
		setBehaviour (db, db->behaviourData.maxBehaviour);

#ifndef STRICT_EXP_BEH // When gtep can be adopted BEFORE all exp targets are exhausted
		if (db->behaviourData.maxBehaviour == GOTO_EXP_PT ||
			db->behaviourData.maxBehaviour == GOTO_EXP_PT_COLLAB)
		{
			GotoExpPt_setLocalMapExhausted (db);

#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<SetLocalMapExhausted>reason=\"highLevelBehavior\"</SetLocalMapExhausted>\n");
#endif
		}
#endif // ifndef STRICT_EXP_BEH
	}
#endif // ifndef COLLAB_EXP

	TIMER_STOP(db->xmlLog, "highLevelbehaviour")
}

void processIsAtDest (RobotDatabase *db)
{
	BEHAVIOUR behaviour = db->behaviourData.behaviour;
	switch (behaviour)
	{
	case EXPLORATION:
		Exploration_isAtDest (db);
		break;
	case GOTO_EXP_PT:
		GotoExpPt_isAtDest (db, 0);
		break;
	case GOTO_EXP_PT_COLLAB:
		GotoExpPt_isAtDest (db, 1);
		break;
	case FOLLOW_PATH:
		isAtDestFOLLOW_PATH (db);
		break;
	case SUPERVISION:
		isAtDestSUPERVISION (db);
		break;
	case BACKTRACK:
		isAtDestBACKTRACK (db);
		break;
	case CLOSE_LOOP:
		CloseLoop_processIsAtDest (db);
		break;
	default:
		fprintf (db->xmlLog, "<UnhandledProcessIsAtDest>behaviour=%d</UnhandledProcessIsAtDest>\n", behaviour);
		break;
	}
}

//! Temporary list. Specific to current pose.
void addDestToUnreachableList (RobotDatabase *db)
{
	PointI *ptr;

	switch (db->behaviourData.behaviour)
	{
	case EXPLORATION:
		ptr = (PointI*)malloc (sizeof (PointF));
		*ptr = PointF_toPointI (db->behaviourData.exploration.profit.dest.target);
		List_pushValue (&db->environment.unreachableIkTargets, ptr);
		break;
	case GOTO_EXP_PT:
		ptr = (PointI*)malloc (sizeof (PointF));
		*ptr = PointF_toPointI (db->behaviourData.gotoExpPt.profit.dest.target);
		List_pushValue (&db->environment.unreachableIkTargets, ptr);
		break;
	default:
		break;
	}
}

void processIKFail (RobotDatabase *db)
{
	// If path planning had failed, this will have been dealt with already. Such a failure
	// would typically be related to the environment structure and would therefore not be
	// temporary.

//	switch (db->behaviourData.behaviour)
	switch (db->behaviourData.baseBehaviour)
	{
	case EXPLORATION:
	default:
		Exploration_ikFailed (db);
		break;
	case GOTO_EXP_PT:
		GotoExpPt_ikFailed (db);
		break;
	case SUPERVISION:
		Supervision_ikFailed (db);
		break;
	}
}

void updateTargetForCommunicating (const Dest *dest, RobotDatabase *db)
{
	if (dest)
	{
		if (dest->type & IS_TARGET_CENTRIC)
		{
			db->status.target = dest->target;
		}
		else
		{
			db->status.target = dest->dest.loc;
		}
	}
	else
	{
		db->status.target.x = MIN_FLT;
		db->status.target.y = MIN_FLT;
	}
}

//! Determine if the robot has accumulated too great an error to continue
void determineIfImmobilised (RobotDatabase *db)
{
//#if !defined(COLLAB_EXP)
#if 1
	if (db->status.stdDev > STD_DEV_MAX && db->partnerData.explorationCoalition.supervisor == -1)
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<RobotImmobilised />\n");
#endif
		db->behaviourData.behaviour = STUCK;
		db->behaviourData.state |= 2;
	}
#endif
}

int checkIsAtDestination (RobotDatabase *db, Dest *dest)
{
	PointF pt;
	float d;
	int isAtDest;

	if (dest->type & IS_TARGET_CENTRIC)
	{
		Geometry_ptFromOrient (db->status.pose.loc.x, db->status.pose.loc.y, &pt.x, &pt.y, dest->targetDist, db->status.pose.orient);
		d = Geometry_dist (pt.x, pt.y, dest->target.x, dest->target.y);
		isAtDest = (d <= dest->leeway);
	}
	else
	{
		d = Geometry_dist (db->status.pose.loc.x, db->status.pose.loc.y, dest->dest.loc.x, dest->dest.loc.y);
		isAtDest = (d <= dest->leeway);
		d = Geometry_orientDiff (db->status.pose.orient,  dest->dest.orient, 0);
		if (d > 0.0f)
		{
			isAtDest &= (d < (ROT_LEFT_DELTA_ORIENT/2.0f + 0.1745f));
		}
		else
		{
			isAtDest &= (-d < (-ROT_RIGHT_DELTA_ORIENT/2.0f + 0.1745f));
		}
	}

	return isAtDest;
}

#ifdef IS_MOTOR
extern int g_forceQuit;
#endif

void BehaviourControl_processBehaviour (RobotDatabase *db)
{
	Dest *dest;
	float stdDevTemp;
	UnionVector4F covMove;
	++db->status.nIterations;

	if (db->behaviourData.behaviour == STUCK)
	{
		return;
	}

	TIMER_START ("processBehavior")

#ifdef IS_MOTOR
	if (g_forceQuit)
	{
		adoptStuck (db);
		return;
	}
#endif

	if (db->status.isNewPose)
	{
		stdDevTemp = db->status.stdDev;

		covMove.vector = Uncertainty_updateCovarianceForMove (
			&db->status.ikData.moves[db->status.ikData.index],
			&db->uncertaintyConstants,
			&db->status.pose,
			&db->status.stdDev,
			db->xmlLog);

#ifdef SIM_ERROR
		Uncertainty_accumErrorForMove (covMove.mat, &db->status.actualLocOffset, db->xmlLog);
#endif // ifdef SIM_ERROR

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<MoveStdDev>before=%f after=%f inc=%f target=%d behaviour=\"%s\"</MoveStdDev>\n",
			stdDevTemp,
			db->status.stdDev,
			(db->status.stdDev - stdDevTemp),
			getTargetToAccredit (db),
			behaviourHandles[getBehaviourToAccredit (db)]);
#endif

		// Reset list of targets/dests which cannot be accurately reached due to constrained movement.
		// After executing a move, these should now be achieveable again.
		List_clear (&db->environment.unreachableIkDests, 1);
		List_clear (&db->environment.unreachableIkTargets, 1);
	}

	dest = getBehaviourDest (db, db->behaviourData.behaviour);
	if (dest && dest->type & IS_AT_DEST)
	{
		processIsAtDest (db);
	}

	if (dest && dest->type & IS_IK_CURRENTLY_IMPOSSIBLE)
	{
		// Failure flag may be set in IK or in path planning.
		processIKFail (db);
	}

	// Behaviour may have changed.
	dest = getBehaviourDest (db, db->behaviourData.behaviour);
	updateTargetForCommunicating (dest, db);

	// If behaviour changed, then dest should have been reset.
	if (db->status.isNewPose && dest && !(dest->type & IS_NEW_DEST))
	{
		// Point to next move in IK sequence
		if (0 == --db->status.ikData.moves[db->status.ikData.index].n)
		{
			++db->status.ikData.index;
		}

		// Check if all moves in IK array have been carried out.
		if (db->status.ikData.index >= db->status.ikData.len)
		{
			if (checkIsAtDestination (db, dest))
			{
#ifdef PRINT_EVENTS
				if (!(dest->type & IS_COARSE_NAV_OK))
				{
					fprintf (db->xmlLog, "<MoveComplete>\"Success\"</MoveComplete>\n");
				}
#endif
				dest->type |= IS_AT_DEST;
				processIsAtDest (db);
				dest = getBehaviourDest (db, db->behaviourData.behaviour);
				updateTargetForCommunicating (dest, db);
			}
			else
			{
#ifdef PRINT_EVENTS
				if (!(dest->type & IS_COARSE_NAV_OK))
				{
					fprintf (db->xmlLog, "<MoveComplete>\"Fail\"</MoveComplete>\n");

					// For EXPLORATION, we can just skip a target if it is going to be difficult to
					// map.
					if (db->behaviourData.behaviour == EXPLORATION)
					{
						processIKFail (db);
					}
				}
#endif
			}
		}
	}

	if (db->status.isNewPose)
	{
		determineIfImmobilised (db);
		SensorProcessing_determineSensorRange (db);
	}

	TIMER_STOP (db->xmlLog, "processBehavior")
}

void BehaviourControl_implementBehaviour (RobotDatabase *db)
{
	Dest *dest = getBehaviourDest (db, db->behaviourData.behaviour);
	IKData *ikData = &db->status.ikData;
	int isCollision, isLocalMapOK, backtrackRequired;
	Pose tempPose;
	List obstdCellGridResetList;
	int isResetReqd = 0;

	db->status.isNewPose = 0;

	if (db->sensorData.isDataPendingInc || !dest || db->behaviourData.behaviour == STUCK)
	{
		return;
	}

#if defined(RERUNNING_ROBOT)
	fflush (db->xmlLog);
#endif

	TIMER_START ("implementBehavior")

	//if (db->behaviourData.behaviour == CLOSE_LOOP ||
	//	(db->behaviourData.behaviour == FOLLOW_PATH && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
	//{
	//	resetList = initList();
	//	CloseLoop_removeSupFromNavMap (db, &resetList);
	//	isResetReqd = 1;
	//}

	// Calc IK moves required by dest.
	if (/*!(dest->type & IS_AT_DEST) &&*/
		(dest->type & IS_NEW_DEST || -1 == ikData->index || ikData->index >= ikData->len))
	{
#if 0
		// Recon is a special case behaviour. There isn't a particular target/dest
		// as it has been determined that the robot's loc est error is too large to
		// establish one. So instead, moves are carried out until either they are
		// completed, or a robot is detected and isAtDest is declared
		if (db->behaviourData.behaviour == CLOSE_LOOP &&
			db->behaviourData.closeLoop.isAttemptingRecon == 2) // There is a difference between 1 and 2
		{
			// Setting IS_AT_DEST here means we will catch this in processBehaviour and
			// adopt AVAILABLE
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CloseLoopReconMovesExhausted />\n");
#endif
			db->behaviourData.closeLoop.profit.dest.type |= IS_AT_DEST;
			goto IMPL_BEH_RESET_SUP;
//			return;
		}
#endif
#if 1
		// When doing secondary recon, we carry out blind searching for the supervisor robot. The
		// robot iteratively navigates to points in an outwardly spiraling circling around the initial
		// estimate of the supervisor robot location, and performs a full rotation to try to detect
		// the supervisor from each point.
		if (db->behaviourData.behaviour == CLOSE_LOOP &&
			db->behaviourData.closeLoop.reconFlag == RECON_PANNING)
		{
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CloseLoopReconMovesExhausted />\n");
#endif
			db->behaviourData.closeLoop.profit.dest.type |= IS_AT_DEST;
			goto IMPL_BEH_RESET_SUP;
		}
#endif

		if (db->behaviourData.behaviour == CLOSE_LOOP ||
			(FOLLOW_PATH == db->behaviourData.behaviour && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
		{
			G_COOP_LOC_DONT_CHECK_LOS = 1; // Don't check LOS when performing close loop

			if (db->behaviourData.closeLoop.reconFlag != NOT_DOING_RECON)
			{
				obstdCellGridResetList = initList();
				CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
				isResetReqd = 1;
			}
		}

		dest->type &= ~IS_NEW_DEST;
		IK_determineMove (db->xmlLog, db->environment.navMap, &db->status.pose, dest, ikData, &db->geometryConstants, &db->ikConstants, db->behaviourData.behaviour, 1, 1, DONT_ALLOW_OUTSIDE_LOCAL_MAP, 1, 1, 0);

		G_COOP_LOC_DONT_CHECK_LOS = 0; // Reset
		if (isResetReqd)
		{
			isResetReqd = 0;
			CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
			List_clear (&obstdCellGridResetList, 1);
		}

		if (dest->type & IS_PATH_REQUIRED)
		{
			goto IMPL_BEH_RESET_SUP;
//			return;
		}

		if (dest->type & IS_AT_DEST && db->behaviourData.behaviour == FOLLOW_PATH)
		{
			isAtDestFOLLOW_PATH (db);
			dest = getBehaviourDest (db, db->behaviourData.behaviour);
			EXPLICIT_DEBUG_ASSERT (dest->type & IS_NEW_DEST)

#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<SkipToNextPathDest />\n");
#endif

			if (db->behaviourData.behaviour == CLOSE_LOOP ||
				(FOLLOW_PATH == db->behaviourData.behaviour && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
			{
				G_COOP_LOC_DONT_CHECK_LOS = 1; // Don't check LOS when performing close loop
			}

			dest->type &= ~IS_NEW_DEST;
			IK_determineMove (db->xmlLog, db->environment.navMap, &db->status.pose, dest, ikData, &db->geometryConstants, &db->ikConstants, db->behaviourData.behaviour, 1, 1, DONT_ALLOW_OUTSIDE_LOCAL_MAP, 1, 1, 0);

			G_COOP_LOC_DONT_CHECK_LOS = 0; // Reset

			if (dest->type & IS_PATH_REQUIRED)
			{
				goto IMPL_BEH_RESET_SUP;
//				return;
			}
		}
	}

	// Execute IK move
	if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE) && !(dest->type & IS_AT_DEST))
	{
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<ImplIkMoves>index=%d\n<Moves>", ikData->index);
#endif
		IK_printMoves (ikData, db->xmlLog);
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "</Moves>\n</ImplIkMoves>\n");
#endif

		tempPose = db->status.pose;
		Actuator_executeMove (&tempPose, ikData->moves[ikData->index], 0, 0, NULL);

		if (db->behaviourData.behaviour == CLOSE_LOOP ||
			(FOLLOW_PATH == db->behaviourData.behaviour && BehaviourCore_getPreviousBehaviour (&db->behaviourData.stack) == CLOSE_LOOP))
		{
			obstdCellGridResetList = initList();
			CloseLoop_removeSupFromNavMap (db, &obstdCellGridResetList);
			isResetReqd = 1;
		}

		// Collisions are checked twice by this stage. Final quick check to ensure robot is not damaged.
		isCollision = 0;
		if (db->behaviourData.behaviour != BACKTRACK)
		{
			isCollision = RobotMapProcessing_detectNavMapCollision (db, tempPose, NARROW_OCCUPIED);
		}

		if (isCollision)
		{
			// Backtrack may actually be required if robot is too close to obstacle
			// at CURRENT pose (i.e., not next pose).
			backtrackRequired = RobotMapProcessing_detectNavMapCollision (db, db->status.pose, BROAD_OCCUPIED);
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<CollisionImminent>nextPose=(%f,%f,%f) backtrack=%d</CollisionImminent>\n", tempPose.loc.x, tempPose.loc.y, tempPose.orient, backtrackRequired);
			fprintf (db->xmlLog, "<BacktrackRequired reason=\"collisionImminent\" />\n");
#endif
			if (backtrackRequired)
			{
				dest->type |= DOES_DEST_REQ_BACKTRACK;
			}
			else
			{
				// Avoid situation caused by numerical error where path is flagged as required, but
				// the robot then determines that it can move straight to the dest/target.
				addDestToUnreachableList (db);

				// Flag that path is required.
				dest->type |= IS_PATH_REQUIRED;
			}
		}
		else
		{
			// See if the robot's mapped terrain will be within the current local map (it will not be saved otherwise).
			// If not, switch local maps before moving.
			isLocalMapOK = RobotMapProcessing_isLocalMapCentreOK (db, tempPose);
			if (isLocalMapOK)
			{
				Actuator_executeMove (&db->status.pose, ikData->moves[ikData->index], 1, db->status.nIterations, db->xmlLog);
#ifdef PRINT_EVENTS
				fprintf (db->xmlLog, "<PoseAfterMove>after=(%f,%f,%f)</PoseAfterMove>\n", db->status.pose.loc.x, db->status.pose.loc.y, db->status.pose.orient);
#endif
				db->status.isNewPose = 1;
				db->behaviourData.behaviourAtLastMove = db->behaviourData.behaviour;
				db->behaviourData.targetAtLastMove = db->behaviourData.targetIndex;
			}
			else
			{
#if 1 // new - quit exploration targets it for some reason a path brings it outside the local map
				if (db->behaviourData.baseBehaviour == EXPLORATION)
				{
					fprintf (db->xmlLog, "<InappropriateExplorationTarget />");
					adoptAVAILABLE (db, 0);
				}
#endif // new

//				db->status.possibleNextPose = tempPose;
//				adsf
			}
		}

		if (isResetReqd)
		{
			CloseLoop_replaceSupInNavMap (db, &obstdCellGridResetList);
			List_clear (&obstdCellGridResetList, 1);
		}
	}

IMPL_BEH_RESET_SUP:
	//if (isResetReqd)
	//{
	//	CloseLoop_replaceSupInNavMap (db, &resetList);
	//	List_clear (&resetList, 1);
	//}

	TIMER_STOP (db->xmlLog, "implementBehavior")
}
#endif
