#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef OBSTACLE_RECOGNITION_CORE_H
#define OBSTACLE_RECOGNITION_CORE_H

#include "../../Common/RobotCore.h"


#if defined(IS_LINUX)
#define VERBOSE_BLOB_DETECTION 0
#else
// 0=no, 1=print, 2=displayMoreImages
//	#define VERBOSE_BLOB_DETECTION 2
	#define VERBOSE_BLOB_DETECTION 0
#endif

//! \todo May require tweaking if environment changes
/*!
May tweak this given further training images. At 3.0 it was found that
1/3 of cells were being estimated as unknown, but there were 0 errors.
At 4.0 only 1/100 were marked as unknown, for 1 error
*/
#define OCCUPANCY_CONFIDENCE_THRESHOLD 0.4f

int ObstacleRecognitionCore_cellXToPixelX (
	const int coord);

int ObstacleRecognitionCore_cellYToPixelY (
	const int coord);

//! Given index of cell in grid, return pixel at centre
PointI ObstacleRecognitionCore_cellCoordsToPixelCoords (
	const PointI coords);

//! Calculate features over entire image, looking at unoccupied and occupied cells.
void ObstacleRecognitionCore_calcImgFeatures (
	ImgFeatures *imgFeatures,
	ImgCellFeatures *cellFeatures,
	const int occupancyGridY);

//! Invert image features before processing to avoid repeating division operations.
void ObstacleRecognitionCore_calcInvImgFeatures (
	ImgFeatures *imgFeatures,
	ImgFeatures *invImgFeatures);

ImgCellFeatures ObstacleRecognitionCore_calcKernelFeatures (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	Image *img,
	const PointI kernelCentre);

//! Calculate appearance of image cell.
void ObstacleRecognitionCore_calcCellFeatures (
	Image *img,
	const int i,
	const int j,
	ImgCellFeatures *cellFeatures,
	const int gridOrigX,
	const int gridOrigY);

//! Calculate cell features for each cell in given image.
void ObstacleRecognitionCore_calcImgCellFeatures (
	Image *img,
	ImgCellFeatures *cellFeatures,
	const int occupancyGridY,
	const int gridOrigX,
	const int gridOrigY);

//! Generate a description of a cell in the context of its image.
void ObstacleRecognitionCore_calcCellDescRelToImg (
	ImgCellFeatures *cellFeatures,
	ImgFeatures *imgInvFeatures,
	const int imgGroupRefFeatures[6],
	float relFeatures[6]);

//! Calculate the difference between a cell description and a cell group description.
float ObstacleRecognitionCore_calcCellDiff (
	const float *cellDesc,
	const float *cellGroupDesc);

//! Calculate the difference between an image description and an image group description.
float ObstacleRecognitionCore_calcImgDiff (
	ImgFeatures *imgFeatures,
	const float *imgGroupDesc,
	const float *imgStdDevs);

//! Check for bad imgs based on typical issues found with camera in use - CMUCam2
int ObstacleRecognitionCore_isImageOk (
	Image *camImg,
	ImgCellFeatures *cellFeatures,
	ImgFeatures *imgFeatures);





#endif // ifndef OBSTACLE_RECOGNITION_CORE_H
#endif
