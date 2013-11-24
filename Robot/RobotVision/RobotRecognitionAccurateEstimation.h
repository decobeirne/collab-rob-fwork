#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef ROBOT_RECOGNITION_ACCURATE_ESTIMATION_H
#define ROBOT_RECOGNITION_ACCURATE_ESTIMATION_H

#include "RobotRecognitionCore.h"

#if 0
void RobotRecognitionAccurateEstimation_estimateRobotLocations (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	CamVectors *camVectors,
	const RobotData *robotDataArray,
	UncertaintyConstants *uncertaintyConstants,
	const int ownIndex,
	List *robotEstimates,
	List *robotFinalEstimates);
#endif

void RobotRecognitionAccurateEstimation_estimateRobotLocationsNEW (
	FILE *outputFile,
	Image *camImg,
	CamVectors *camVectors,
	const RobotData *robotDataArray,
	UncertaintyConstants *uncertaintyConstants,
	const int ownIndex,
	List *robotEstimates);

#endif // ifndef ROBOT_RECOGNITION_ACCURATE_ESTIMATION_H
#endif

