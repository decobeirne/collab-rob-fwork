#include "../../Common/RobotDefs.h"


#include "FollowPath.h"
#include "BehaviourAdoption.h"
#include "../Ik/Ik.h"
#include "../Sensors/MapIntegrationImpl.h"
#include "../../Common/BitArray.h"
#include "../../Common/RobotCore.h"
#include "../../Common/Actuator.h"
#include "../../Common/Geometry.h"
#include "../../Common/Uncertainty.h"


//
// Functions required to execute robot behaviour tasks on the cloud
//


//! Temporary values used when calculating a path.
typedef struct PathTemps_
{
	List path; //!< Path from pose
	List targetPath; //!< List of PointI nodes from target to rob
	List deadEnds; //! List of PointI dead ends
	PointF target;
	PointI navMapOriginIndex;
	const Image *navMap;
	const uchar *obstructedCellGrid;
#ifdef SHOW_PATHS
	IplImage *localMapIplImage;
#endif
} PathTemps;

//! Constructor
PathTemps initPathTemps (const Image *navMap,
						 const uchar *obstructedCellGrid,
						 const Dest *dest)
{
	PathTemps tempVals;
	tempVals.path = initList();
	tempVals.targetPath = initList();
	tempVals.deadEnds = initList();

	if (dest->type & IS_TARGET_CENTRIC)
	{
		tempVals.target = dest->target;
	}
	else
	{
		tempVals.target = dest->dest.loc;
	}

	tempVals.navMapOriginIndex = navMap->orig;
	tempVals.navMapOriginIndex.x += NAV_CELL_AREA/2;
	tempVals.navMapOriginIndex.y += NAV_CELL_AREA/2;
	tempVals.navMapOriginIndex = PointI_calcNavCellMidptObstdCell (tempVals.navMapOriginIndex);

	tempVals.navMap = navMap;
	tempVals.obstructedCellGrid = obstructedCellGrid;
#ifdef SHOW_PATHS
	tempVals.localMapIplImage = db->localMapIplImage;  // This will break - fix later
#endif
	return tempVals;
}

//! Destructor
void clearPathTemps (PathTemps *tempVals)
{
	List_clear (&tempVals->path, 1);
	List_clear (&tempVals->targetPath, 1);
	List_clear (&tempVals->deadEnds, 1);
}









void orderBresenhamCells (PointI *cell1, PointI *cell2)
{
	PointI temp;
	if (cell2->x > cell1->x || (cell2->x == cell1->x && cell2->y > cell1->y))
	{
		temp = *cell1;
		*cell1 = *cell2;
		*cell2 = temp;
	}
}

//! Check LOS between cells on obstructed cell grid
int checkObstructedGridVisualContact (const PointI pt1, const PointI pt2, const uchar *obstructedCellGrid)
{
	PointI cell1, cell2;
	PointI delta, step, iter, iter2;
	const int steps[2] = {-1, 1};
	int axis, count, e, i;

	cell1 = PointI_calcNavCellMidptObstdCell (pt1);
	cell2 = PointI_calcNavCellMidptObstdCell (pt2);
	DEBUG_ASSERT(cell1.x >= 0 && cell1.x < NAV_GRID_DIMS)
	DEBUG_ASSERT(cell1.y >= 0 && cell1.y < NAV_GRID_DIMS)
	DEBUG_ASSERT(cell2.x >= 0 && cell2.x < NAV_GRID_DIMS)
	DEBUG_ASSERT(cell2.y >= 0 && cell2.y < NAV_GRID_DIMS)

	orderBresenhamCells (&cell1, &cell2);

	delta.x = cell2.x - cell1.x;
	delta.y = cell2.y - cell1.y;
	step.x = steps[delta.x >= 0];
	step.y = steps[delta.y >= 0];
	delta.x = abs (delta.x);
	delta.y = abs (delta.y);
	axis = (delta.x > delta.y);
	count = max (delta.x, delta.y);

	iter = cell1;

	if (BitArray_checkElement_pt (obstructedCellGrid, iter, NAV_GRID_DIMS))
	{
		return 0; // Not visual contact.
	}

	if (axis)
	{
		for (e = 0, i = 0; i < count; ++i)
		{
			e += delta.y;
			if (e << 1	>= delta.x)
			{
				iter.y += step.y;
				e -= delta.x;
			}
			iter.x += step.x;

			if (BitArray_checkElement_pt (obstructedCellGrid, iter, NAV_GRID_DIMS))
			{
				return 0;
			}

			if (e)
			{
				iter2 = iter;
				if (e > 0)
				{
					iter2.y += step.y;
				}
				else
				{
					iter2.y -= step.y;
				}
				if (BitArray_checkElement_pt (obstructedCellGrid, iter2, NAV_GRID_DIMS))
				{
					return 0;
				}
			}
		}
	}
	else
	{
		for (e = 0, i = 0; i < count; ++i)
		{
			e += delta.x;
			if (e << 1	>= delta.y)
			{
				iter.x += step.x;
				e -= delta.y;
			}
			iter.y += step.y;

			if (BitArray_checkElement_pt (obstructedCellGrid, iter, NAV_GRID_DIMS))
			{
				return 0;
			}

			if (e)
			{
				iter2 = iter;
				if (e > 0)
				{
					iter2.x += step.x;
				}
				else
				{
					iter2.x -= step.x;
				}

				if (BitArray_checkElement_pt (obstructedCellGrid, iter2, NAV_GRID_DIMS))
				{
					return 0;
				}
			}
		}
	}
	return 1;
}

int checkPointVisualContact (
	PathTemps *tempVals,
	const PointI point,
	const PointI aim,
	const VISUAL_CONTACT_TYPE visualContactType,
	const Image *navMap,
	const uchar *obstructedCellGrid)
{
	PointI pt1l, pt2l;
	int res;
	int ptsWithinNavMap = 0;
	uchar hitValue;

	pt1l = point;
	pt1l.x -= navMap->orig.x;
	pt1l.y -= navMap->orig.y;

	pt2l = aim;
	pt2l.x -= navMap->orig.x;
	pt2l.y -= navMap->orig.y;

	if (Image_isWithinBounds_ignoreMapOrig (navMap, pt1l.x, pt1l.y))
	{
		ptsWithinNavMap |= 1;
	}
	if (Image_isWithinBounds_ignoreMapOrig (navMap, pt2l.x, pt2l.y))
	{
		ptsWithinNavMap |= 2;
	}

	if (ptsWithinNavMap)
	{
		if (ptsWithinNavMap & 1)
		{
			res = RobotCore_checkLine (pt1l.x, pt1l.y, pt2l.x, pt2l.y, navMap, uchar_lessThan, FREE_TERRAIN, THREE_PIXELS, 0, &hitValue);
		}
		else
		{
			res = RobotCore_checkLine (pt2l.x, pt2l.y, pt1l.x, pt1l.y, navMap, uchar_lessThan, FREE_TERRAIN, THREE_PIXELS, 0, &hitValue);
		}

		if (res)
		{
			return 0; // Not visual contact.
		}
	}

	if (CHECK_NAV_MAP_ONLY != visualContactType && (!(ptsWithinNavMap & 1) || !(ptsWithinNavMap & 2)))
	{
		if (CHECK_CELLS_ADJACENT == visualContactType)
		{
			return (NAV_CELL_AREA >= abs (point.x - aim.x) && NAV_CELL_AREA >= abs (point.y - aim.y));
		}
		else
		{
			return checkObstructedGridVisualContact (point, aim, obstructedCellGrid);
		}
	}
	return 1; // Visual contact
}

//! Check if the paths from start pose and target can be joined.
int checkPathVisualContact (
										   PathTemps *tempVals,
										 const PointI point,
										 List *oppositePath,
										 const Image *navMap)
{
	ListNode *iterator;
	iterator = oppositePath->front;
	while (iterator != 0)
	{
		// Return value of 1 indicates visual contact with the opposite path
		if (1 == checkPointVisualContact (
			tempVals,
			point,
			*(PointI*)iterator->value,
			CHECK_CELLS_ADJACENT,
			navMap,
			tempVals->obstructedCellGrid))
		{
			return 1;
		}
		iterator = iterator->next;
	}
	return 0;
}

//! Push neighbour node onto path
void pushNode (const PointI node, List *path)
{
	PointI *pt = (PointI*)malloc (sizeof (PointI));
	*pt = node;

	List_pushValue (path, pt);
}

//! If a node has no neighbours pop it from the path and push onto dead ends
int pushDeadEnd (PathTemps *tempVals, List *path)
{
	int res;
	ListNode *lastNode;

	lastNode = path->back;

	if (lastNode->prev)
	{
		// Set the return to indicate the path has to backtrack
		res = 0;
	}
	else
	{
		// If this is the first node, then set the return to indicate
		// that the target is unreachable
		res = -1;
	}

	List_removeElement (path, lastNode);
	List_pushNode (&tempVals->deadEnds, lastNode);

	return res;
}

//! Calculate cost for each potential next node.
int calcNeighbourSuitability (
									 PathTemps *tempVals,
									 List *path,
									 const PointI oppositeTarget,
									 const PointI *neighbourArray)
{
	int i;
	float minCost, cost;
	int bestNeighbourIndex;
	PointI cellIndex;
	PointF oppositeTargetF = PointI_toPointF (oppositeTarget);

	bestNeighbourIndex = -1;
	minCost = MAX_FLT;
	for (i = 0; i < 8; ++i)
	{
		if (neighbourArray[i].x < 0 || neighbourArray[i].x >= ENVIR_DIMS ||
			neighbourArray[i].y < 0 || neighbourArray[i].y >= ENVIR_DIMS)
		{
			continue;
		}

		if (1 == List_isElement (path, &neighbourArray[i], PointI_comparePtrs))
		{
			continue;
		}

		if (1 == List_isElement (&tempVals->deadEnds, &neighbourArray[i], PointI_comparePtrs))
		{
			continue;
		}

		cellIndex = PointI_calcNavCellMidptObstdCell (neighbourArray[i]);
		if (1 == BitArray_checkElement_pt (
			tempVals->obstructedCellGrid,
			cellIndex,
			NAV_GRID_DIMS))
		{
			continue;
		}

		if (0 == checkPointVisualContact (
			tempVals,
			*(PointI*)path->back->value,
			neighbourArray[i],
			CHECK_NAV_MAP_ONLY,
			tempVals->navMap,
			tempVals->obstructedCellGrid))
		{
			continue;
		}

		cost = Geometry_distSqrd (
			(float)neighbourArray[i].x,
			(float)neighbourArray[i].y,
			oppositeTargetF.x,
			oppositeTargetF.y);

		if (cost < minCost)
		{
			bestNeighbourIndex = i;
			minCost = cost;
		}
	}
	return bestNeighbourIndex;
}

//! Push next node onto path.
int calcPathNode (
	PathTemps *tempVals,
	List *path,
	PointI oppositeTarget)
{
	int neighbourIndex;
	PointI pt, midpt;
	PointI *lastNode;
	PointI neighbourArray[8];

	lastNode = (PointI*)path->back->value;
	pt = *lastNode;
	midpt = PointI_calcNavCellMidpt (pt);

	neighbourArray[0].x = midpt.x - NAV_CELL_AREA;		neighbourArray[0].y = midpt.y - NAV_CELL_AREA;
	neighbourArray[1].x = midpt.x - NAV_CELL_AREA;		neighbourArray[1].y = midpt.y;
	neighbourArray[2].x = midpt.x - NAV_CELL_AREA;		neighbourArray[2].y = midpt.y + NAV_CELL_AREA;
	neighbourArray[3].x = midpt.x;						neighbourArray[3].y = midpt.y - NAV_CELL_AREA;
	neighbourArray[4].x = midpt.x;						neighbourArray[4].y = midpt.y + NAV_CELL_AREA;
	neighbourArray[5].x = midpt.x + NAV_CELL_AREA;		neighbourArray[5].y = midpt.y - NAV_CELL_AREA;
	neighbourArray[6].x = midpt.x + NAV_CELL_AREA;		neighbourArray[6].y = midpt.y;
	neighbourArray[7].x = midpt.x + NAV_CELL_AREA;		neighbourArray[7].y = midpt.y + NAV_CELL_AREA;

	neighbourIndex = calcNeighbourSuitability (
		tempVals,
		path,
		oppositeTarget,
		neighbourArray);

	// If no neighbour, pop the current node from the path
	if (-1 == neighbourIndex)
	{
		return pushDeadEnd (tempVals, path);
	}
	// Otherwise push the most suitable neighbour to the path
	else
	{
		pushNode (neighbourArray[neighbourIndex], path);
		return 1;
	}
}

//! Push start nodes onto lists starting at robot and target
void pushStartNodes (PathTemps *tempVals, const Pose *pose, const Dest *dest)
{
	PointI *robStartNode;
	PointI *tarStartNode;
	robStartNode = (PointI*)malloc (sizeof (PointI));
	tarStartNode = (PointI*)malloc (sizeof (PointI));

	*robStartNode = PointF_toPointI (pose->loc);

	if (dest->type & IS_TARGET_CENTRIC)
	{
		*tarStartNode = PointF_toPointI (dest->target);
	}
	else
	{
		*tarStartNode = PointF_toPointI (dest->dest.loc);
	}

	List_pushValue (&tempVals->path, robStartNode);
	List_pushValue (&tempVals->targetPath, tarStartNode);
}






extern void calcGainFromTempPixels (Image *localMap, float *gain);

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
	float *gain)
{
	Pose tempPose = pose;
	Dest tempDest;
	ListNode *iterator;
	PointI node;
	IKData ikData = initIKData();
	int res = 1; // If fail occurs, it will initially be at target 1.
	Vector4F covMove; // Dummy

//	iterator = db->behaviourData.followPath.nodes.front;
	iterator = followPathData->nodes.front;
	node = *(PointI*)iterator->value;
	tempDest.dest.loc = PointI_toPointF (node);
	tempDest.dest.orient = 0.0f;
	tempDest.type = IS_COARSE_NAV_OK | IS_NEW_DEST;
	tempDest.flags = NORMAL_IK;
	tempDest.leeway = EXPLORATION_LEEWAY;
	tempDest.maxDist = MAX_FLT;

	while (1)
	{
		if (tempDest.type & IS_NEW_DEST || ikData.index == -1 || ikData.index >= ikData.len)
		{
			tempDest.type &= ~IS_NEW_DEST;
			IK_determineMove (xmlLog, navMap, &tempPose, &tempDest, &ikData, geometryConstants, ikConstants, FOLLOW_PATH, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP, 0, 1, 0);

			if (tempDest.type & IS_AT_DEST)
			{
				// If current dest is a node in the path, then move to next dest...
				if (iterator)
				{
					// Target along path at which fail occurs. Hopefully 0.
					++res;

					// Either next node or previous behaviour dest.
					iterator = iterator->next;
					if (iterator)
					{
						node = *(PointI*)iterator->value;
						tempDest.dest.loc = PointI_toPointF (node);
						tempDest.dest.orient = 0.0f;
						tempDest.type = IS_COARSE_NAV_OK | IS_NEW_DEST;
					}
					else
					{
						tempDest = *prevBehaviourDest;
					}
					ikData = initIKData();
				}
				// Have reached previous behaviour dest.
				else
				{
					res = 0;
					goto checkPathMoves_clean;
				}
			}
			else if (tempDest.type & IS_IK_CURRENTLY_IMPOSSIBLE || tempDest.type & IS_PATH_REQUIRED)
			{
#if 0
				if (tryAdjustingNode)
				{
					tempDest |= ~80;
//					IK_determineMove (db->xmlLog, db->environment.navMap, &tempPose, &tempDest, &ikData, ikConstants, FOLLOW_PATH, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP, 0, 1, 0);
					IK_determineMove (xmlLog, navMap, &tempPose, &tempDest, &ikData, ikConstants, FOLLOW_PATH, 0, 0, DO_ALLOW_OUTSIDE_LOCAL_MAP, 0, 1, 0);
				}
				else
#endif
				{
					prevBehaviourDest->type |= IS_IK_CURRENTLY_IMPOSSIBLE;
					goto checkPathMoves_clean;
				}
			}
		}

		if (ikData.index != -1)
		{
			Actuator_executeMove (&tempPose, ikData.moves[ikData.index], 0, 0, NULL);

			if (calcGain)
			{
				CamVectors_rotateToRobotOrient (camVectors, tempPose.orient);

				// Same approach as in calcMoves_evalBehaviour()
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
	}

checkPathMoves_clean:
	if (calcGain)
	{
//		calcGainFromTempPixels (db->sensorData.localMap, gain);
		calcGainFromTempPixels (localMap, gain);
	}

	return res;
}

//! Merge nodes from 2 paths into 1.
int mergePathSegments (PathTemps *tempVals,
									List *basePath,
									List *mergePath,
									const Image *navMap)
{
	PointI pt;
	ListNode *swapNode;
	ListNode *iterator;
	pt = *(PointI*)basePath->back->value;

	// Find the merge point: the earliest point in the merge path with visual contact with the
	// end of the base path
	iterator = mergePath->front;
	while (iterator)
	{
		if (1 == checkPointVisualContact (
			tempVals,
			pt,
			*(PointI*)iterator->value,
			CHECK_CELLS_ADJACENT,
			navMap,
			tempVals->obstructedCellGrid))
		{
			// From the point in the merge path pointed to by the iterator, merge the points
			// with the base path. Remove each element from the merge list and put push it 
			// onto the base list without going through a deletion each time.
			while (iterator)
			{
				List_removeElement (mergePath, iterator);

				swapNode = iterator;
				iterator = iterator->prev;
				List_pushNode (basePath, swapNode);
			}

			return 1;
		}

		iterator = iterator->next;
	}

	return 0;
}

//! Combine paths starting from pose and destination.
int combinePaths (PathTemps *tempVals, const int reverse)
{
	List *path1, *path2;
	ListNode *swapNode; // Pointer to front node to swap
	ListNode *iterator;

	if (reverse)
	{
		path1 = &tempVals->targetPath;
		path2 = &tempVals->path;
	}
	else
	{
		path1 = &tempVals->path;
		path2 = &tempVals->targetPath;
	}

	if (0 == mergePathSegments (tempVals, path1, path2, tempVals->navMap))
	{
		return 0;
	}

	if (reverse)
	{
		List_clear (&tempVals->path, 1);

		// If the target path is the base path, start at the back of this and push
		// the nodes onto the final path
		iterator = path1->back;
		while (iterator)
		{
			swapNode = iterator;
			List_removeElement (path1, swapNode);

			iterator = iterator->prev;

			List_pushNode (&tempVals->path, swapNode);
		}
	}
	return 1;
}

//! Calculate suitable last node in path.
/*!
For dest-centric, this is a pt from which the robot can assume the dest pose. For
tar-centric, this is a pt from which to observe the target.
*/
int calcLastNode (PathTemps *tempVals, const Dest *dest)
{
	float dOrient, pathOrient, orient;
	PointI *lastNode;
	PointF pt;
	PointI pti;
	ListNode *iterator;
	float idealDist;
	
	if (dest->type & IS_TARGET_CENTRIC)
	{
		// Project out a point slightly farther away than the actual desired dest. The
		// robot will navigate to this point using coarse IK, and from here will be in
		// a good position to use accurate IK to move to the correct point.
		idealDist = dest->targetDist + 3.0f;
	}
	else
	{
		// Should be a good distance from which to carry out accurate IK.
		idealDist = 8.0f;
	}

	// The last node in the path should be the (dest.loc|target) => can remove this now.
	List_deleteElement (&tempVals->path, &tempVals->path.back, 0, 1);

	// Get orient from target to last node in path.
	lastNode = (PointI*)tempVals->path.back->value;
	pt = PointI_toPointF (*lastNode);
	pathOrient = Geometry_orient (tempVals->target.x, tempVals->target.y, pt.x, pt.y);

	// Project destination points out from target.
	for (dOrient = 0.0f; dOrient < PI*2; dOrient += PI/30)
	{
		orient = Geometry_orientSum (pathOrient, dOrient);

		Geometry_ptFromOrient (
			tempVals->target.x,
			tempVals->target.y,
			&pt.x,
			&pt.y,
			idealDist,
			orient);

		pti = PointF_toPointI (pt);
		pti = PointI_calcNavCellMidpt (pti);

		if (pti.x < 0 || pti.x >= ENVIR_DIMS ||
			pti.y < 0 || pti.y >= ENVIR_DIMS)
		{
			continue;
		}

		if (0 == checkPointVisualContact (
			tempVals,
			PointF_toPointI (tempVals->target),
			pti,
			CHECK_CELLS_BRESENHAM,
			tempVals->navMap,
			tempVals->obstructedCellGrid))
		{
			continue;
		}

		// Calculate earliest node in path that has visual contact with the projected pt.
		iterator = tempVals->path.front;
		while (iterator)
		{
			if (checkPointVisualContact (
				tempVals,
				pti,
				*(PointI*)iterator->value,
				CHECK_CELLS_BRESENHAM,
				tempVals->navMap,
				tempVals->obstructedCellGrid))
			{
				// We now have visual contact between the pt and iterator.
				iterator = iterator->next;
				while (iterator)
				{
					List_deleteElement (&tempVals->path, &iterator, 1, 1);
				}

				// Push the destination pt onto the path.
				lastNode = (PointI*)malloc(sizeof(PointI));
				*lastNode = pti;

				List_pushValue (&tempVals->path, lastNode);

				return 1;
			}

			iterator = iterator->next;
		}
	}

	return 0;
}

//! Remove spurious nodes from path.
int refinePath (PathTemps *tempVals)
{
	ListNode *iterator;
	ListNode *currentAcceptedNode;
	ListNode *nextVisibleNode;
	ListNode *deleteIterator;

	currentAcceptedNode = tempVals->path.front;
	nextVisibleNode = 0;

	iterator = currentAcceptedNode->next;
	while (iterator)
	{
		if (checkPointVisualContact (
			tempVals,
			*(PointI*)currentAcceptedNode->value,
			*(PointI*)iterator->value,
			CHECK_CELLS_BRESENHAM,
			tempVals->navMap,
			tempVals->obstructedCellGrid))
		{
			nextVisibleNode = iterator;
			iterator = iterator->next;
		}
		else
		{
			// The last node that was visible should be used
			deleteIterator = currentAcceptedNode->next;
			while (deleteIterator && deleteIterator != nextVisibleNode)
			{
				List_deleteElement (&tempVals->path, &deleteIterator, 1, 1);
			}

			// We have found that 2 immediately adjacent nodes do not have a line
			// of visual contact, therefore return failure
			if (!nextVisibleNode)
			{
				return 0;
			}
	
			currentAcceptedNode = nextVisibleNode;
			nextVisibleNode = 0;

			// Do not increment iterator here, this was already incremented
			// last time, then it was compared to the old accepted node.
			// On the next iter it will be checked against the new accepted node.
		}
	}

	// The robot pose was pushed onto the front of the path when path calculation
	// was started, so can removed now.
	if (tempVals->path.front)
	{
		List_deleteElement (&tempVals->path, &tempVals->path.front, 1, 1);
	}

	return 1;
}

PATH_RESULT FollowPath_calcPath (
	const Pose *pose,
	const Dest *dest,
	FollowPathData *followPath,
	const Image *navMap,
	const uchar *obstructedCellGrid,
	const int verbose)
{
	PATH_RESULT res = PATH_OK;
	int isPathReachable;
	int robotRes, targetRes;
	PathTemps tempVals = initPathTemps (navMap, obstructedCellGrid, dest);

	pushStartNodes (&tempVals, pose, dest);

	isPathReachable = 1;
	while (1)
	{
		robotRes = 0;
		targetRes = 0;

		robotRes = calcPathNode (
			&tempVals,
			&tempVals.path,
			*(PointI*)tempVals.targetPath.front->value);

		// The path is empty, so there will be no node to aim towards.
		if (-1 != robotRes)
		{
			targetRes = calcPathNode (
				&tempVals,
				&tempVals.targetPath,
				*(PointI*)tempVals.path.front->value);
		}

#ifdef SHOW_PATHS
		if (verbose)
		{
			showPath (&tempVals);
		}
#endif

		if (-1 == robotRes || -1 == targetRes || tempVals.path.size > 100)
		{
			isPathReachable = 0;
			break;
		}

		robotRes = checkPathVisualContact (
			&tempVals,
			*(PointI*)tempVals.path.back->value,
			&tempVals.targetPath,
			tempVals.navMap);

		targetRes = checkPathVisualContact (
			&tempVals,
			*(PointI*)tempVals.targetPath.back->value,
			&tempVals.path,
			tempVals.navMap);

		if (1 == robotRes || 1 == targetRes)
		{
			break;
		}
	}

	if (!isPathReachable)
	{
		if (-1 == robotRes)
		{
			res = ROBOT_INVALID;
		}
		else
		{
			res = TARGET_INVALID;
		}
	}
	else
	{
		if (combinePaths (&tempVals, targetRes && !robotRes))
		{
#ifdef SHOW_PATHS
			if (verbose)
			{
				showPath (&tempVals);
			}
#endif
			if (calcLastNode (&tempVals, dest))
			{
				if (!refinePath (&tempVals))
				{
					res = CANT_REFINE_PATH;
					isPathReachable = 0;
				}
			}
			else
			{
				res = CANT_CALC_LAST_NODE;
				isPathReachable = 0;
			}
		}
		else
		{
			res = CANT_COMBINE;
			isPathReachable = 0;
		}
	}

	List_clear (&followPath->nodes, 1);

	if (isPathReachable)
	{
#ifdef SHOW_PATHS
		if (verbose)
		{
			showPath (&tempVals);
		}
#endif
		List_shallowCopyNodes (&followPath->nodes, &tempVals.path);
	}

	clearPathTemps (&tempVals);

	return res;
}




































//
// Functions not required on cloud, may contain references to robot database, etc.
//


#if !defined(BOARD)

#ifdef SHOW_PATHS
//! Display node on nav map
void showNode (PathTemps *tempVals, const PointI p)
{
	uchar u;
	u = Image_getPixel_dontCheck (
		tempVals->navMap,
		p.x - tempVals->navMap->orig.x,
		p.y - tempVals->navMap->orig.y);

	if (INVALID_CELL == u)
	{
		return;
	}

	if (FREE_TERRAIN == u)
	{
		Image_setPixel_dontCheck (
			tempVals->navMap,
			p.x - tempVals->navMap->orig.x,
			p.y - tempVals->navMap->orig.y,
			120);
	}
	else if (OCCUPIED_TERRAIN == u)
	{
		Image_setPixel_dontCheck (
			tempVals->navMap,
			p.x - tempVals->navMap->orig.x,
			p.y - tempVals->navMap->orig.y,
			130);
	}
}

//! Hide node on nav map
void hideNode (PathTemps *tempVals, const PointI p)
{
	uchar u;
	u = Image_getPixel_dontCheck (
		tempVals->navMap,
		p.x - tempVals->navMap->orig.x,
		p.y - tempVals->navMap->orig.y);

	if (INVALID_CELL == u)
	{
		return;
	}

	if (120 == u)
	{
		Image_setPixel_dontCheck (
			tempVals->navMap,
			p.x - tempVals->navMap->orig.x,
			p.y - tempVals->navMap->orig.y,
			FREE_TERRAIN);
	}
	else if (130 == u)
	{
		Image_setPixel_dontCheck (
			tempVals->navMap,
			p.x - tempVals->navMap->orig.x,
			p.y - tempVals->navMap->orig.y,
			OCCUPIED_TERRAIN);
	}
}

extern int pathWindowSize[4];

//! Show nodes in path and then hide again
void showPath (PathTemps *tempVals)
{
	PointI p;
	int res;
	ListNode *iterator;

	res = 0;
	iterator = tempVals->path.front;
	while (iterator)
	{
		p = *(PointI*)iterator->value;

		if (1 == Image_isWithinBounds_ignoreMapOrig (
			tempVals->navMap,
			p.x - tempVals->navMap->orig.x,
			p.y - tempVals->navMap->orig.y))
		{
			showNode (tempVals, p);
			res = 1;
		}
		iterator = iterator->next;
	}

	if (0 == res)
	{
		return;
	}

	Image_show_givenIplImage (tempVals->navMap, "path", tempVals->localMapIplImage, pathWindowSize);
	cvWaitKey (0);

	iterator = tempVals->path.front;
	while (iterator)
	{
		p = *(PointI*)iterator->value;

		if (1 == Image_isWithinBounds_ignoreMapOrig (
			tempVals->navMap,
			p.x - tempVals->navMap->orig.x,
			p.y - tempVals->navMap->orig.y))
		{
			hideNode (tempVals, p);
		}
		iterator = iterator->next;
	}
}
#endif // SHOW_PATHS






void FollowPath_updateDest (RobotDatabase *db, FollowPathData *followPath)
{
	ListNode *node;
	PointI *pt;

	node = db->behaviourData.followPath.nodes.front;
	if (node)
	{
		pt = (PointI*)node->value;

		followPath->dest.dest.loc = PointI_toPointF (*pt);
		followPath->dest.dest.orient = 0.0f;
		followPath->dest.type = IS_COARSE_NAV_OK | IS_NEW_DEST; // Coarse nav and new dest
		followPath->dest.leeway = FOLLOW_PATH_LEEWAY;
		followPath->dest.maxDist = MAX_FLT;
	}
	else
	{
		followPath->dest.type = IS_COARSE_NAV_OK | IS_AT_DEST;
	}
}


extern const char* behaviourHandles[];

void isAtDestFOLLOW_PATH (RobotDatabase *db)
{
	ListNode *node;
	BasicBehaviourData *basicBehaviour;
	Dest *dest;

	node = db->behaviourData.followPath.nodes.front;
	List_deleteElement (&db->behaviourData.followPath.nodes, &node, 1, 1);

	if (0 == db->behaviourData.followPath.nodes.size)
	{
		node = db->behaviourData.stack.back;
		List_deleteElement (&db->behaviourData.stack, &node, 0, 0);
		BehaviourCore_printStack (&db->behaviourData.stack, "fp is at dest");

		// Adopt previous behaviour
		basicBehaviour = (BasicBehaviourData*)node->value;

		db->behaviourData.behaviour = basicBehaviour->id;

		dest = getBehaviourDest (db, db->behaviourData.behaviour);
		if (dest)
		{
			dest->type |= IS_NEW_DEST;
		}
#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<PathComplete>prevBehaviour=%s</PathComplete>\n", behaviourHandles[basicBehaviour->id]);
#endif
	}
	else
	{
		FollowPath_updateDest (db, &db->behaviourData.followPath);
	}
}

void FollowPath_reset (RobotDatabase *db)
{
	ListNode *node;
	BasicBehaviourData *basicData;
	BEHAVIOUR id;

	node = db->behaviourData.stack.back;
	List_deleteElement (&db->behaviourData.stack, &node, 0, 0);
	BehaviourCore_printStack (&db->behaviourData.stack, "fp reset");
	basicData = (BasicBehaviourData*)db->behaviourData.stack.back->value;
	id = basicData->id;
	db->behaviourData.behaviour = id;
}

Dest* FollowPath_setBehaviour (RobotDatabase *db)
{
	Dest *dest;
	db->behaviourData.behaviour = FOLLOW_PATH;
	dest = &db->behaviourData.followPath.dest;
	List_pushValue (&db->behaviourData.stack, &db->behaviourData.followPath);
	BehaviourCore_printStack (&db->behaviourData.stack, "fp set beh");
	return dest;
}

void FollowPath_printNodes (FILE *f, List *nodes)
{
	ListNode *iterator;
	PointI *ptr;

	fprintf(f, "nodes=(");

	iterator = nodes->front;
	while (iterator)
	{
		ptr = (PointI*)iterator->value;
		fprintf (f, "(%d,%d)", ptr->x, ptr->y);
		iterator = iterator->next;
	}
	fprintf(f, ")");
}

//! Determine if the path can be nudged around an obstacle.
int tryAdjustingNode (RobotDatabase *db, Image *navMap, ListNode *iterator, const PointI nodePt, const PointI nodePt_global, const PointI prevPt, const PointI nextPt)
{
	float orient;
	PointF ptf1, ptf2;
	PointI newPt;
	PointI *ptr;
	uchar hitValue;

	ptf1 = PointI_toPointF (nodePt);
	ptr = iterator->value;

	for (orient = 0.0f; orient < PI * 2.0f; orient += (PI / 6.0f))
	{
		Geometry_ptFromOrient (ptf1.x, ptf1.y, &ptf2.x, &ptf2.y, 4.0f, orient);
		newPt = PointF_toPointI (ptf2);

		if (RobotCore_checkLine (prevPt.x, prevPt.y, newPt.x, newPt.y, navMap, RobotCore_equalsNarrowOccupied, 0, THREE_PIXELS, 0, &hitValue) &&
			RobotCore_checkLine (newPt.x, newPt.y, nextPt.x, nextPt.y, navMap, RobotCore_equalsNarrowOccupied, 0, THREE_PIXELS, 0, &hitValue))
		{
			newPt.x += navMap->orig.x;
			newPt.y += navMap->orig.y;
			*ptr = newPt;
			return 1;
		}

		Geometry_ptFromOrient (ptf1.x, ptf1.y, &ptf2.x, &ptf2.y, 7.0f, orient);
		newPt = PointF_toPointI (ptf2);
		if (newPt.x < 0 || newPt.x > ENVIR_DIMS -1 ||
			newPt.y < 0 || newPt.y > ENVIR_DIMS -1)
		{
			continue;
		}

		if (RobotCore_checkLine (prevPt.x, prevPt.y, newPt.x, newPt.y, navMap, RobotCore_equalsNarrowOccupied, 0, THREE_PIXELS, 0, &hitValue) &&
			RobotCore_checkLine (newPt.x, newPt.y, nextPt.x, nextPt.y, navMap, RobotCore_equalsNarrowOccupied, 0, THREE_PIXELS, 0, &hitValue))
		{
			newPt.x += navMap->orig.x;
			newPt.y += navMap->orig.y;
			*ptr = newPt;
			return 1;
		}
	}

	return 0;
}

#endif

