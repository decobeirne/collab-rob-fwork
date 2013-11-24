#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef OBSTACLE_RECOGNITION_H
#define OBSTACLE_RECOGNITION_H

#include "../../Common/RobotCore.h"

int ObstacleRecognition_estimateGridOccupancy (
	const int camOccupancyGridY,
	OccupancyGrid *camOccupancyGrid,
	const float *camScoreGrid,
	const int isAnyRobotCell);

//! Calculate image and cell features
/*!
We don't need to pass in occupancyGridOrig here, as this will already be setup
for the current robot; either hardcoded when running robot, or setup in
setupCamParams for testing/training.
*/
void ObstacleRecognition_calcImageFeatures (
	Image *camImg,
	ImgCellFeatures cellFeatures[SIZE_COOP_LOC_GRID],
	ImgFeatures *imgFeatures);

void ObstacleRecognition_calcOccupancy (
//	FILE *xmlLog,
	Image *camImg,
	ImgCellFeatures cellFeatures[SIZE_COOP_LOC_GRID],
	ImgFeatures *imgFeatures,
	OccupancyGrid *camOccupancyGrid,
	float camScoreGrid[SIZE_COOP_LOC_GRID],
	const int anyRobotCells,
	int *isObstacleInCamOccupancyGrid);

#endif // ifndef OBSTACLE_RECOGNITION_H
#endif
