#include "../../Common/RobotDefs.h"



#ifndef DEST_CENTRIC_H
#define DEST_CENTRIC_H

#include "../../Common/Actuator.h"


//! Coarse move to destination using hill climbing.
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
	const int printLogic);

//! Accurate move to destination using inverse Jacobian IK.
void IK_destCentricAccurateMove (
	FILE *f,
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
	const int printLogic);

void testDestCentricIK();

#endif

