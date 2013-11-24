#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef ROBOT_RECOGNITION_BLOB_DETECTION_H
#define ROBOT_RECOGNITION_BLOB_DETECTION_H

#include "../../Common/RobotCore.h"
#include "../../Common/List.h"
#include "RobotRecognitionCore.h"

int RobotRecognitionBlobDetection_isBlobValid (
	const ColourBlob *blob,
	const int includeSmallBlobs);

#if VERBOSE_BLOB_DETECTION
void RobotRecognitionBlobDetection_displayRobotCells (
	IplImage *iplImage,
	const int *windowSize,
	Image *originalImg,
	const OccupancyGrid *occupancyGrid);

void RobotRecognitionBlobDetection_displayBlobs (
	FILE *outputFile,
	IplImage *iplImage,
	const int *windowSize,
	Image *originalImg,
	List *colourBlobs,
	const int nValidBlobs);
#endif

int RobotRecognitionBlobDetection_analyseBlobs (
#if VERBOSE_BLOB_DETECTION
	const uchar *colourIdsThisTrainingImg,
#endif
	const RobotData *robotDataArray,
	const int ownIndex,
	CamVectors *camVectors,
	FILE *outputFile,
	List *colourBlobs);

void RobotRecognitionBlobDetection_createBlobs (
	FILE *outputFile,
#if VERBOSE_BLOB_DETECTION
	IplImage *iplImage,
	const int *windowSize,
	Image *originalImg,
	const uchar *colourIdsThisTrainingImg,
#endif
	const int ownRobotIndex,
	const float *colourScoreGrid,
	ImgCellFeatures *cellFeatures,
	OccupancyGrid *occupancyGrid,
	List *colourBlobs);

#endif // ifndef ROBOT_RECOGNITION_BLOB_DETECTION_H
#endif
