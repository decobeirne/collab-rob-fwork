#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef ROBOT_RECOGNITION_ROUGH_ESTIMATION_H
#define ROBOT_RECOGNITION_ROUGH_ESTIMATION_H

#include "../../Common/RobotCore.h"
#include "../../Common/List.h"
#include "RobotRecognitionCore.h"

//! Used in both rough and accurate estimation processes
void RobotRecognitionRoughEstimation_setupFinalEstimate (
	FILE *outputFile,
	RobotEstimate *robotEstimate,
	const FaceEstType minEstTypeForFace);

void RobotRecognitionRoughEstimation_estimateRobotLocations (
	FILE *outputFile,
	CamVectors *camVectors,
	const RobotData *robotDataArray,
	UncertaintyConstants *uncertaintyConstants,
	const int ownIndex,
	List *colourBlobs,
	List *robotEstimates);

#endif






















#endif // ifndef ROBOT_RECOGNITION_ROUGH_ESTIMATION_H

