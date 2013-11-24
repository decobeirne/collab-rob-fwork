#include "../../Common/RobotDefs.h"



#ifndef FOLLOW_PATH_H
#define FOLLOW_PATH_H

#include "BehaviourCore.h"
#include "../Data/RobotDatabase.h"

//! Calculate path given current pose and dest.
PATH_RESULT FollowPath_calcPath (
	const Pose *pose,
	const Dest *dest,
	FollowPathData *followPath,
	const Image *navMap,
	const uchar *obstructedCellGrid,
	const int verbose);

#if defined(SIMULATION) || defined(BOARD)
#include "../../Board/Data/BoardSensorData.h"
#endif


//! Calc nMoves, stdDev and check for collision along path. Return 0 for success, otherwise target along path at which path traversal failed.
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

#if !defined(BOARD)

//! Set dest based on first node in path.
void FollowPath_updateDest (RobotDatabase *db, FollowPathData *followPath);

//! Update dest when node in path has been reached.
void isAtDestFOLLOW_PATH (RobotDatabase *db);

//! Pop FOLLOW_PATH off behaviour stack if path is blocked again when path following.
void FollowPath_reset (RobotDatabase *db);

//! Push behaviour onto stack, etc.
Dest* FollowPath_setBehaviour (RobotDatabase *db);

//! Print nodes to log
void FollowPath_printNodes (FILE *f, List *nodes);

#endif
#endif

