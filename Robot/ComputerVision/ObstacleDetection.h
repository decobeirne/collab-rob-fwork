#include "../../Common/RobotDefs.h"

#ifndef OBST_DET_H
#define OBST_DET_H

#include "../../Common/RobotCore.h"



//! Calculate cell features for each cell in given image.
void ObstacleDetection_calcImgCellFeatures (Image *img, ImgCellFeatures *cellFeatures, const int occupancyGridY, const int gridOrigX, const int gridOrigY);

//! Calculate appearance of image cell.
void ObstacleDetection_calcCellFeatures (Image *img, const int i, const int j, ImgCellFeatures *cellFeatures, const int gridOrigX, const int gridOrigY);

//! Calculate features over entire image, looking at unoccupied and occupied cells
void ObstacleDetection_calcImgFeatures (ImgFeatures *imgFeatures, ImgCellFeatures *cellFeatures, const int occupancyGridY);

//! Invert image features before processing to avoid repeating division operations.
void ObstacleDetection_calcInvImgFeatures (ImgFeatures *imgFeatures, ImgFeatures *invImgFeatures);

//! Given scores representing matches against unoccupied and occupied groups, and robot detection info, set cells in occupancy grid
int ObstacleDetection_calcOccupiedEst(
	const int camOccupancyGridY,
	OccupancyGrid *camOccupancyGrid,
	PointF *camScoreGrid,
	int *isObstacleInCamOccupancyGrid);

int ObstacleDetection_calcOccupiedEst2 (
	const int camOccupancyGridY,
	OccupancyGrid *camOccupancyGrid,
	PointF *camScoreGrid,
	ImgCellFeatures *cellFeatures,
	ImgFeatures *imgInvFeatures,
	const int closestImgGroupIndex,
	int *isObstacleInCamOccupancyGrid);

//! Calculate image occupancy, i.e. areas of the image representing unoccupied/occupied terrain
/*!
- Calculate diff between observed image and each image group
- For each cell in observed image:
-	Get cell features relative to image for the reference features used by this image group
-	Get diff between cell features and each unoccupied cell group
-	Get diff between cell features and each occupied cell group
-	Set value in occupancy grid based on closest match
*/
void ObstacleDetection_calcImgOccupancy (
	ImgFeatures *imgFeatures,
	ImgFeatures *imgInvFeatures,
	ImgCellFeatures *cellFeatures,
	PointF *camScoreGrid,
	int *closestImgGroupIndex,
	const int camOccupancyGridY);

#ifdef SIMULATION

//! For testing; extract OccupancyGrid into an array of uchars
void ObstacleDetection_extractOccupancyGrid (OccupancyGrid *occupancyGrid, uchar *values, const int gridX, const int gridY);

//! Set each cell in results grid to correct or not.
//void ObstacleDetection_compareCamOccupancyGrids (const int imageIndex, const uchar trainedGrid[CAM_OCCUPANCY_GRID_X * 18], uchar observedGrid[CAM_OCCUPANCY_GRID_X * 18], const int gridY, int results[7]);
void ObstacleDetection_compareCamOccupancyGrids (const int imageIndex, const uchar *trainedGrid, uchar *observedGrid, const int gridY, int results[7]);

//! Load training images, calculate occupancy grid using trained features and compare to original grids.
void ObstacleDetection_test();

//! Calculate cell features for each cell in image based on the given calcUnoccupied flag.
void ObstacleDetection_calcImgCellFeaturesForTraining (
	Image *img,
	ImgCellFeatures *cellFeatures,
	const uchar *occupancyGrid,
	const int gridY,
	const int gridOrigX,
	const int gridOrigY,
	const int calcUnoccupied);

//! Extract image and cell features from training images and write to file.
/*
The setup function initializes an array of TrainingData structs. The features are written
to files in the directory calibration/tempData. The image data can then be processed by running
Scripts/trainImageData.py, which will write out code for image groups that can be copied into
CameraProcessing.c.
*/
void ObstacleDetection_writeImageFeatures(const TrainingData *data, const int nTrainingImages, const char *trainingDataDir);

//! Extract and write features from obstacle detection images.
/*!
Calculate features h,s,v and texture for h,s,v.

The set of input images is set in ObstacleDetection_setupTrainingData. The output dir is set
in setupRootDir. The output file pattern must be provided.

Write single floating point value per line, s.th. it can be easily parsed by script on other side.
*/
void ObstacleDetection_writeFeatures();

//! Setup image training data for obstacle detection images.
/*
Images from the following directories are used:
 - robot2_20110827
 - robot4_20110807
 - robot2_20110812
 - robot3_20110918
 - 20110807_experiment_robot_0_2481
 - 20110807_experiment_robot_0_2618
 - 20110807_experiment_robot_0_2784
*/
int ObstacleDetection_setupTrainingData(TrainingData *data);

#endif // ifdef SIMULATION
#endif // ifndef OBST_DET_H
