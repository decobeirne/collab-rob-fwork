#include "../../Common/RobotDefs.h"

//#ifndef BOARD

#ifndef IK_H
#define IK_H

#include "TargetCentric.h"
#include "DestCentric.h"

//! Print IK data to an output file.
void IK_printMoves (
	IKData *ikData,
	FILE *f);

//! Determine movements robot should make to achieve the given dest
/*!
\param checkDists Indicates that distances to the dest should be checked, and FOLLOW_PATH adopted where suitable
*/
void IK_determineMove (
	FILE *f,
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
	const int printLogic);

#endif

//#endif // ifndef BOARD
