#include "../../Common/RobotDefs.h"

#ifndef ROBOT_RECOGNITION_H
#define ROBOT_RECOGNITION_H

#include "RobotRecognitionCore.h"
#include "../../Common/RobotCore.h"

//! Return anyRobotCells - used when calculating occupancy
int RobotRecognition_detectRobotsNEW (
	FILE *outputFile,
#if VERBOSE_BLOB_DETECTION
	Image *displayImg,
	IplImage *iplImage,
	const int *windowSize,
	const uchar *colourIdsThisTrainingImg,
#endif
#if defined(DEBUGGING_ROBOT_REC)
	const int isLocSet,
#endif
	Image *originalImg,
	CamVectors *camVectors,
	UncertaintyConstants *uncertaintyConstants,
	const RobotData *robotDataArray,
	const int ownIndex,
	List *robotFinalEstimates,
	float *colourScoreGrid,
	ImgCellFeatures *cellFeatures,
	OccupancyGrid *camOccupancyGrid);



#endif // ifndef ROBOT_RECOGNITION_H
