#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef ROBOT_RECOGNITION_MODEL_H
#define ROBOT_RECOGNITION_MODEL_H

#include "../../Common/RobotCore.h"

int RobotRecognitionModel_leaveOutColourInTraining (const int colour);

//! Naive, but only in testing/training so doesn't matter
int RobotRecognitionModel_mapColourIdToIndex(const int colourId);

float RobotRecognitionModel_calcSingleColourScore (
	const int colourIndex,
	ImgCellFeatures *cell);

float RobotRecognitionModel_calcColourScores (
	ImgCellFeatures *cellFeatures,
	float *colourScoreGrid);

#endif // ifndef ROBOT_RECOGNITION_MODEL_H
#endif
