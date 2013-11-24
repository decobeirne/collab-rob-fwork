#include "../../Common/RobotDefs.h"

//#ifndef BOARD

#ifndef TARGET_CENTRIC_H
#define TARGET_CENTRIC_H

#include "../../Common/Actuator.h"

//! Accurate move to target using inverse Jacobian IK.
void IK_targetCentricAccurateMove (
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
	const int simpleIkOnly,
	const int printLogic);

#endif

//#endif 
