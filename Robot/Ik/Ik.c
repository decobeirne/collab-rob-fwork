#include "../../Common/RobotDefs.h"

// #ifndef BOARD

#include "Ik.h"
#include "../../Common/Geometry.h"


void IK_printMoves (IKData *ikData,
					FILE *f)
{
	int i;
	fprintf (f, "len=%d moves=(", ikData->len);
	for (i = 0; i < ikData->len; ++i)
	{
		fprintf (f, "(dir=%d,n=%d,usBurst=%d)", ikData->moves[i].dir, ikData->moves[i].n, ikData->moves[i].usBurst);
	}
	fprintf (f, ")");
}

int IK_isCoarseDestOutsideLocalMap (Dest *dest, const Image *navMap)
{
	PointI centre;
	int isOnMap;
	centre.x = navMap->orig.x + 42; // LOC_MAP_DIMS/2
	centre.y = navMap->orig.y + 42; // LOC_MAP_DIMS/2
	isOnMap = Geometry_isPtOnLocalMap (
		PointF_toPointI (dest->dest.loc),
		centre,
		0);
	return (!isOnMap);
}

void IK_determineMove (FILE *f,
					   const Image *navMap,
					   const Pose *pose,
					   Dest *dest,
					   IKData *ikData,
					   const GeometryConstants *geometryConstants,
					   const IkConstants *ikConstants,
					   const BEHAVIOUR behaviour,
					   const int checkDists,
					   const int verbose,
					   const int allowOutsideLocalMap,
					   const int isImplemented,
					   const int useStricterLeeway,
					   const int printLogic)
{
	float dist, tempLeeway;
	const int resetTypeMasks[] = {~0, ~IS_PATH_REQUIRED};
//	const float targetLeeways[] = {0.8f, 0.75f, 3.0f}; // Crap vals
	const float targetLeeways[] = {0.8f, 0.75f, 3.0f}; // Test vals
//	const float targetLeeways[] = {0.75f, 0.75f}; // Actual proper vals
//	const float targetLeeways[] = {2.0f, 0.75f}; //< IK was failing (due to compass error) way too much
	const int simpleIkOnly = (int)(dest->flags & SIMPLE_IK_ONLY);
#ifdef PRINT_EVENTS
	PointF pt;
#endif

	DEBUG_ASSERT_IS_BOOL(checkDists)
	DEBUG_ASSERT_IS_BOOL(verbose)
	DEBUG_ASSERT_IS_BOOL(allowOutsideLocalMap)
	DEBUG_ASSERT_IS_BOOL(isImplemented)
	EXPLICIT_DEBUG_ASSERT(geometryConstants->areExpDistsSetup == 1)

	*ikData = initIKData();

	if (dest->type & IS_COARSE_NAV_OK)
	{
		dist = Geometry_dist (pose->loc.x, pose->loc.y, dest->dest.loc.x, dest->dest.loc.y);
		if (dist < dest->leeway)
		{
			if (verbose)
			{
				fprintf (f, "<IkDetermineMove>event=\"distLessThanLeeway\" leeway=%f</IkDetermineMove>\n", dest->leeway);
			}
			dest->type |= IS_AT_DEST;
			return;
		}
		else if (checkDists &&
			FOLLOW_PATH != behaviour &&
			BACKTRACK != behaviour &&
			(dist > geometryConstants->nav_maxDist * 1.8f || IK_isCoarseDestOutsideLocalMap (dest, navMap)))
		{
			if (verbose)
			{
				fprintf (f, "<IkDetermineMove>event=\"pathIsReqd\"</IkDetermineMove>\n");
			}
			// For longer moves, makes sense to check path before moving.
			dest->type |= IS_PATH_REQUIRED;
		}
		else
		{
			if (verbose)
			{
				fprintf (f, "<IkDetermineMove>event=\"destCentricCoarseMove\"</IkDetermineMove>\n");
			}
			IK_destCentricCoarseMove (navMap, pose, dest, ikData, geometryConstants, 0.0f, allowOutsideLocalMap, behaviour, isImplemented, 0);

			// Ignore (type & IS_PATH_REQUIRED), i.e. blocked, if behaviour is BACKTRACK
			dest->type &= resetTypeMasks[BACKTRACK == behaviour];
		}
	}
	else
	{
		// Avoid numerical accuracy issues by reducing leeway when calculating moves to reach destination.
		tempLeeway = dest->leeway;
		dest->leeway *= targetLeeways[useStricterLeeway];

		if (dest->type & IS_TARGET_CENTRIC)
		{
			if (verbose)
			{
				fprintf (f, "<IkDetermineMove>event=\"targetCentricAccurateMove\"</IkDetermineMove>\n");
			}
			IK_targetCentricAccurateMove (f, navMap, pose, dest, ikData, geometryConstants, ikConstants, behaviour, verbose, allowOutsideLocalMap, isImplemented, simpleIkOnly, printLogic);
		}
		else
		{
			if (verbose)
			{
				fprintf (f, "<IkDetermineMove>event=\"destCentricAccurateMove\"</IkDetermineMove>\n");
			}
			IK_destCentricAccurateMove (f, navMap, pose, dest, ikData, geometryConstants, ikConstants, behaviour, verbose, allowOutsideLocalMap, isImplemented, printLogic);
		}

		dest->leeway = tempLeeway;
	}

#ifdef PRINT_EVENTS
	if (verbose)
	{
		if (dest->type & IS_PATH_REQUIRED)
		{
			fprintf (f, "<IKCollision>pose=(%f,%f,%f) destType=%d</IKCollision>\n",
				pose->loc.x, pose->loc.y, pose->orient,
				dest->type);
		}
		else if (!(dest->type & IS_IK_CURRENTLY_IMPOSSIBLE) && !(dest->type & IS_AT_DEST))
		{
			fprintf (f, "<DetermineMove>");
			IK_printMoves (ikData, f);
			fprintf (f, "</DetermineMove>\n");
		}
		else if (dest->type & IS_AT_DEST)
		{
			if (dest->type & IS_TARGET_CENTRIC)
			{
				Geometry_ptFromOrient (pose->loc.x, pose->loc.y, &pt.x, &pt.y, dest->targetDist, pose->orient);
				fprintf (f, "<AtDestination>pose=(%f,%f,%f) optimumPt=(%f,%f) target=(%f,%f)</AtDestination>\n",
					pose->loc.x, pose->loc.y, pose->orient,
					pt.x, pt.y,
					dest->target.x, dest->target.y);
			}
			else
			{
				fprintf (f, "<AtDestination>pose=(%f,%f,%f) dest=(%f,%f,%f)</AtDestination>\n",
					pose->loc.x, pose->loc.y, pose->orient,
					dest->dest.loc.x, dest->dest.loc.y, dest->dest.orient);
			}
		}
		else
		{
			fprintf (f, "<IKFailed>pose=(%f,%f,%f) destType=%d</IKFailed>\n",
				pose->loc.x, pose->loc.y, pose->orient,
				dest->type);
		}
	}
#endif
}

// #endif // ifndef BOARD
