#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef OBSTACLE_RECOGNITION_MODEL_H
#define OBSTACLE_RECOGNITION_MODEL_H

#include "../../Common/RobotCore.h"

void ObstacleRecognitionModel_calcObstacleGroupScores (
	ImgFeatures *imgFeatures,
	ImgCellFeatures *cellFeatures,
	float *camScoreGrid,
	const int camOccupancyGridY);

#endif // ifndef OBSTACLE_RECOGNITION_MODEL_H

#endif
