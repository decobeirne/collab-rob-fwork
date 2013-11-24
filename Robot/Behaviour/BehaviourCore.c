#include "../../Common/RobotDefs.h"



#include "BehaviourCore.h"

#include "../Ik/Ik.h"
#include "../Data/RobotDatabase.h"
#include "../Sensors/MapIntegrationImpl.h"
#include "../../Common/RobotCore.h"
#include "../../Common/Uncertainty.h"
#include "../../Common/Actuator.h"
#include "../../Common/BitArray.h"



void BehaviourCore_clearProfitData (ProfitData *d)
{
	d->ratio = MIN_FLT;
	d->gross = MIN_FLT;
}

float calcTargetConflictDeduction (const float gross,
								   const float targetConflicts)
{
	float deduction = 0;
	if (gross > 0)
	{
		deduction = gross * min (1, targetConflicts);
	}
	return deduction;
}


float calcMappingProfit (const float gainNMapped, const float stdDev)
{
	return gainNMapped * (1.0f - min (1.0f, stdDev / STD_DEV_MAX));
}

float calcStdDevReductionProfit (const float gainNMapped, const float stdDevReduction)
{
	return gainNMapped * max (0.0f, stdDevReduction / STD_DEV_MAX);
}

PointF getTargetToPrint (const Dest *dest)
{
	if (dest->type & IS_TARGET_CENTRIC)
	{
		return dest->target;
	}
	else
	{
		return dest->dest.loc;
	}
}


extern const char* behaviourHandles[];

//! Print profit data for behaviour.
void BehaviourCore_printBehaviourProfit (
	FILE *f,
	const BEHAVIOUR behaviour,
	const int calledFromCloudFunc,
	const int isMaxProfit,
	const Dest *dest,
	const float ratio,
	const float gross,
	const float expenditure,
	const float resources,
	const float nSteps,
	const float gainNMapped,
	const float stdDevAtDest,
	const float stdDevInc,
	const float initialNMapped)
{
	const char *profitStr[2] = {"CellProfit", "MaxProfit"};
	const char *cloudStr[2] = {"", "_"};
	PointF targetToPrint = getTargetToPrint (dest);
//	fprintf (f, "<%s>behaviour=\"%s\" target=(%f,%f) ratio=%f gross=%f expenditure=%f resources=%f nSteps=%f gainNMapped=%f initialNMapped=%f stdDevAtDest=%f stdDevInc=%f switchOhead=%f",
	fprintf (f, "<%s%s>behaviour=\"%s\" target=(%f,%f) ratio=%f gross=%f expenditure=%f resources=%f nSteps=%f gainNMapped=%f initialNMapped=%f stdDevAtDest=%f stdDevInc=%f",
		profitStr[isMaxProfit],
		cloudStr[calledFromCloudFunc],
		behaviourHandles[behaviour],
		targetToPrint.x,
		targetToPrint.y,
		ratio,
		gross,
		expenditure,
		resources,
		nSteps,
		gainNMapped,
		initialNMapped,
		stdDevAtDest,
		stdDevInc);
}

void printBehaviourProfit_tail (FILE *f, const int isMaxProfit)
{
	const char *str[2] = {"CellProfit", "MaxProfit"};
	fprintf (f, "</%s>\n", str[isMaxProfit]);
}

PointF getDestPt (const Dest *dest)
{
	if (dest->type & IS_TARGET_CENTRIC)
	{
		return dest->target;
	}
	else
	{
		return dest->dest.loc;
	}
}

PointI calcCellMidpt (const PointI pt,
					  const int distBetweenMidpts,
					  const PointI originCellMidpt)
{
	PointI cellMidpt;
	cellMidpt.x = originCellMidpt.x + (pt.x * distBetweenMidpts);
	cellMidpt.y = originCellMidpt.y + (pt.y * distBetweenMidpts);
	return cellMidpt;
}

//! Calculate cells gained by moves required to map target.
void calcGainFromTempPixels (Image *localMap, float *gain)
{
	int i, j, n;
	uchar *ptr;
	n = 0;
	ptr = localMap->data;
	for (j = 0; j < LOC_MAP_DIMS; ++j)
	{
		for (i = 0; i < LOC_MAP_DIMS; ++i)
		{
			if (*ptr == FAKE_TEMP_TERRAIN)
			{
				*ptr = 127;
				++n;
			}
			++ptr;
		}
	}
	*gain = (float)n;
}

void calcMoves_evalBehaviour (FILE *xmlLog,
							  const Image *navMap,
							  Image *localMap,
							  const GeometryConstants *geometryConstants,
							  CamVectors *camVectors,
							  const IkConstants *ikConstants,
							  const UncertaintyConstants *uncertaintyConstants,
							  const float camPOffset,
							  const Pose pose,
							  Dest *dest,
							  float *nMoves,
							  float *stdDev,
							  const BEHAVIOUR behaviour,
							  const int calcGain,
							  float *gain)
{
	Pose tempPose = pose;
	Dest tempDest = *dest;
	IKData ikData = initIKData();
	Vector4F covMove;

	while (1)
	{
		if (tempDest.type & IS_NEW_DEST || ikData.index == -1 || ikData.index >= ikData.len)
		{
			tempDest.type &= ~IS_NEW_DEST;
			IK_determineMove (xmlLog, navMap, &tempPose, &tempDest, &ikData, geometryConstants, ikConstants, behaviour, 1, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP, 0, 1, 0);

			if (tempDest.type & IS_AT_DEST)
			{
				goto calcMoves_evalBehaviour_wipe;
			}
			else if (tempDest.type & IS_IK_CURRENTLY_IMPOSSIBLE)
			{
				dest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
				goto calcMoves_evalBehaviour_wipe;
			}
			else if (tempDest.type & IS_PATH_REQUIRED)
			{
				dest->type |= IS_PATH_REQUIRED;
				goto calcMoves_evalBehaviour_wipe;
			}
		}

		Actuator_executeMove (&tempPose, ikData.moves[ikData.index], 0, 0, NULL);

		if (calcGain)
		{
			CamVectors_rotateToRobotOrient (camVectors, tempPose.orient);

			// Only set pixels (occupancy map cells) if currently unmapped (127). Set to
			// FAKE_TEMP_TERRAIN so that they can be wiped later
			MapIntegrationImpl_intScan_simd_evalBehaviour (
				geometryConstants,
				camVectors,
				tempPose.loc,
				tempPose.orient,
				camPOffset,
				localMap,
				navMap,
				FAKE_TEMP_TERRAIN);

			//MapIntegrationImpl_intScan_neighbour (
			//	geometryConstants,
			//	tempPose.loc,
			//	tempPose.orient,
			//	localMap,
			//	FAKE_TEMP_TERRAIN);
		}

		++(*nMoves);
		covMove = Uncertainty_updateCovarianceForMove (
			&ikData.moves[ikData.index],
			uncertaintyConstants,
			&tempPose,
			stdDev,
			0);

		if (0 == --ikData.moves[ikData.index].n)
		{
			++ikData.index;
		}
	}

calcMoves_evalBehaviour_wipe:
	if (calcGain)
	{
		calcGainFromTempPixels (localMap, gain);
	}
	return;
}











PATH_RESULT FollowPath_calcPath (
	const Pose *pose,
	const Dest *dest,
	FollowPathData *followPath,
	const Image *navMap,
	const uchar *obstructedCellGrid,
	const int verbose);

// extern
int checkPathMoves (
	FollowPathData *followPathData,
	FILE *xmlLog,
	const Image *navMap,
	Image *localMap,
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const IkConstants *ikConstants,
	const UncertaintyConstants *uncertaintyConstants,
	const float camPOffset,
	const Pose pose,
	Dest *prevBehaviourDest,
	float *nMoves,
	float *stdDev,
	const int tryAdjustingNode,
	const int calcGain,
	float *gain);



void BehaviourCore_updateUnreachableGrid (
	FILE *xmlLog,
	const int index,
	const Pose pose,
	uchar *unreachableGrid,
	int *hasUnreachableLocalMapGridBeenUpdated,
	Dest *dest,
	const BEHAVIOUR behaviour)
{
	PointI pt;
#ifdef PRINT_EVENTS
	PointF ptf;
#endif

	switch (behaviour)
	{
	case EXPLORATION:
		pt = PointI_calcExpCellIndex (PointF_toPointI (dest->target), EXP_AREA);
		BitArray_setElement_pt (
			unreachableGrid,
			pt,
			GLOB_EXP_GRID_DIMS,
			1);
		break;

	case GOTO_EXP_PT:
	case GOTO_EXP_PT_COLLAB:
		pt = PointI_calcCellIndex (PointF_toPointI (dest->dest.loc), LOC_MAP_DIMS, LOC_MAP_DIFF);
		BitArray_setElement_pt (
			unreachableGrid,
			pt,
			GLOB_LOC_MAP_GRID_DIMS,
			1);

		// The sync with board has been moved to a separate function after the
		// profit calc loop. This avoids sending potentially multiple updates to
		// the board. It also greatly simplifies the implementation of behaviour
		// calculation on the cloud.
#if 0
		RobotWrite_updateUnreachableLocalMapGrid (
#if defined(SIMULATION) || defined(BOARD)
			boardUnreachableLocalMapGrid,
			boardSensorData,
#else
			db,
#endif
			index);
#endif // if 0

		// If the robot has updated the unreachable cell grid, then it may have to sync
		// this with the board after the behaviour calc loop is complete. This is done for
		// GTEP (and in future SUPERVISION).
		*hasUnreachableLocalMapGridBeenUpdated = 1;
		break;

	case SUPERVISION:
		pt = PointI_calcCellIndex (PointF_toPointI (dest->dest.loc), SUP_DIMS, SUP_DIFF);
		BitArray_setElement_pt (
			unreachableGrid,
			pt,
			SUP_GRID_DIMS,
			1);
		break;

	case CLOSE_LOOP:
		// Record that a coalition cannot be formed with this supervisor.
		break;

	default:
		break;
	}

#ifdef PRINT_EVENTS
	ptf = getTargetToPrint (dest);
	fprintf (
		xmlLog,
		"<CalcPath>result=\"Fail\" pose=(%f,%f,%f) target=(%f,%f) behaviour=\"%d\" targetCentric=%d<CalcPath>\n",
		pose.loc.x, pose.loc.y, pose.orient, ptf.x, ptf.y, (int)behaviour, (int)(dest->type & IS_TARGET_CENTRIC));
#endif
}

void BehaviourCore_calcMovesAndPathIfReqd (
	FILE *xmlLog,
	FollowPathData *followPathData,
	const Image *navMap,
	Image *localMap, // Not const, as scans are integrated in ordre to calc gain
	const uchar *obstructedCellGrid,
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
	ProfitTemp *temp,
	const BEHAVIOUR behaviour,
	const int calcGain)
{
	PATH_RESULT pathResult;
	float nMoves, stdDevTemp;
	float gain;
	int checkPathResult;

	nMoves = 0.0f;
	stdDevTemp = stdDev;
	calcMoves_evalBehaviour (
		xmlLog,
		navMap,
		localMap,
		geometryConstants,
		camVectors,
		ikConstants,
		uncertaintyConstants,
		camPOffset,
		pose,
		&temp->dest,
		&nMoves,
		&stdDevTemp,
		behaviour,
		calcGain,
		&gain);

	if (temp->dest.type & IS_IK_CURRENTLY_IMPOSSIBLE)
	{
		temp->nSteps = MAX_FLT;
		temp->stdDevAtDest = MAX_FLT;
		temp->stdDevInc = MAX_FLT;
		if (calcGain)
		{
			temp->gainNMapped = MIN_FLT;
		}
	}
	else if (temp->dest.type & IS_PATH_REQUIRED)
	{
		temp->dest.type &= ~IS_PATH_REQUIRED;

		pathResult = FollowPath_calcPath (
			&pose,
			&temp->dest,
			followPathData,
			navMap,
			obstructedCellGrid,
			0);

		if (PATH_OK == pathResult)
		{
			// Make sure that the robot is not stuck. This could be verified within FollowPath_calcPath,
			// but no need to duplicate code in checkPathMoves or introduce potential for bugs.
			nMoves = 0.0f;
			stdDevTemp = stdDev;
			checkPathResult = checkPathMoves (
				followPathData,
				xmlLog,
				navMap,
				localMap,
				geometryConstants,
				camVectors,
				ikConstants,
				uncertaintyConstants,
				camPOffset,
				pose,
				&temp->dest,
				&nMoves,
				&stdDevTemp,
				0,
				calcGain,
				&gain);

			// Path that was calculated actually still results in a collision, so do not consider this target.
			if (temp->dest.type & IS_PATH_REQUIRED || temp->dest.type & IS_IK_CURRENTLY_IMPOSSIBLE)
			{
				// Check if the path fails trying to reach the very first node.
				pathResult = TARGET_INVALID;
				if (1 == checkPathResult)
				{
					pathResult = ROBOT_INVALID;
				}
			}
			else
			{
				temp->nSteps = nMoves;
				temp->stdDevAtDest = stdDevTemp;
				temp->stdDevInc = max (0.0f, stdDevTemp - stdDev);
				if (calcGain)
				{
					temp->gainNMapped = gain;
				}
			}
		}
		else if (behaviour == CLOSE_LOOP)
		{
			fprintf (xmlLog, "<SupervisorMayBeInInvalidLoc />\n");
		}

		if (ROBOT_INVALID == pathResult)
		{
			// If the robot is currently in a location marked BROAD_OCCUPIED (or
			// BROAD_VEHICLE_TERRAIN), then path planning cannot be carried out from
			// here (backtracking should be carried out first), but this should
			// obviously not mean that the target is flagged as unreachable.
			temp->isFollowPathImpossible = 1;
			temp->nSteps = MAX_FLT;
			temp->stdDevAtDest = MAX_FLT;
			temp->stdDevInc = MAX_FLT;
			if (calcGain)
			{
				temp->gainNMapped = MIN_FLT;
			}
		}
		else if (PATH_OK != pathResult)
		{
			BehaviourCore_updateUnreachableGrid (
				xmlLog,
				index,
				pose,
				unreachableGrid,
				hasUnreachableLocalMapGridBeenUpdated,
				&temp->dest,
				behaviour);

			temp->nSteps = MAX_FLT;
			temp->stdDevAtDest = MAX_FLT;
			temp->stdDevInc = MAX_FLT;
			if (calcGain)
			{
				temp->gainNMapped = MIN_FLT;
			}
		}
	}
	else
	{
		temp->nSteps = nMoves;
		temp->stdDevAtDest = stdDevTemp;
		temp->stdDevInc = max (0.0f, stdDevTemp - stdDev);
		if (calcGain)
		{
			temp->gainNMapped = gain;
		}
	}

	temp->nSteps = max (temp->nSteps, 1.0f);
}

void BehaviourCore_printStackToLog (FILE *stream, List *stack)
{
	ListNode *iter;
	BasicBehaviourData *data;

	fprintf (stream, "<BehaviourStack>stack=(");

	iter = stack->front;
	while (iter)
	{
		data = (BasicBehaviourData*)iter->value;
		fprintf (stream, "%s,", behaviourHandles[data->id]);
		iter = iter->next;
	}
	fprintf (stream, ")</BehaviourStack>\n");
}

void BehaviourCore_printStack (List *stack, const char *msg)
{
#ifdef PRINT_DEBUG
	ListNode *iter;
	BasicBehaviourData *data;

	printf (" - behaviour stack - %s: ", msg);
	iter = stack->front;
	while (iter)
	{
		data = (BasicBehaviourData*)iter->value;
		printf("%d, ", data->id);
		iter = iter->next;
	}
	printf ("\n");
#endif
}

BEHAVIOUR BehaviourCore_getPreviousBehaviour (List *behaviourStack)
{
	BasicBehaviourData *data;
	EXPLICIT_DEBUG_ASSERT (behaviourStack->back)
	EXPLICIT_DEBUG_ASSERT (behaviourStack->back->prev)

	data = (BasicBehaviourData*)behaviourStack->back->prev->value;
	return data->id;
}








