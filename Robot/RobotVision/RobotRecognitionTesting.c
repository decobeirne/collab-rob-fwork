#include "RobotRecognitionTesting.h"
#include "RobotRecognitionTraining.h"
#include "ObstacleRecognitionTraining.h"
#include "RobotRecognitionModel.h"
#include "RobotRecognitionCore.h"
#include "RobotRecognition.h"
#include "ObstacleRecognition.h"
#include "ObstacleRecognitionCore.h"
#include "ObstacleRecognitionTesting.h"
#include "../../Common/RobotCore.h"
#include "../../Common/Geometry.h"

#if defined(IS_WIN) && defined(SIMULATION)

const char* RobotRecognitionTesting_mapColourIdToName (const int colourId)
{
	switch (colourId)
	{
	case 0:
		return "yellowGreen";
	case 1:
		return "darkOrange";
	case 2:
		return "yellow";
	case 3:
		return "lightPink";
	case 4:
		return "whiteGreen";
	case 5:
		return "aquaGreen";
	case 6:
		return "lightAqua";
	case 7:
		return "lightPurple";
	case 8:
		return "darkPurple";
	case 9:
		return "brown";
	case 10:
		return "darkAqua";
	case 11:
		return "mustard";
	case 12:
		return "darkPink";
	case 13:
		return "lightOrange";
	case 14:
		return "lightGreen";
	case 15:
		return "darkGreen";
	case 16:
		return "navy";
	case 20:
		return "background";
	default:
		return "NOT_A_COLOUR";
	}
}

int RobotRecognitionTesting_getColoursInTrainingImage (
	uchar *colours,
	const uchar *occupancyGrid)
{
	int i;
	int nColours;
	uchar val;

	memset (colours, 0, 256);
	nColours = 0;
	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		val = occupancyGrid[i];
		if (RobotRecognitionCore_isColourOkForTraining (val) &&
			!RobotRecognitionModel_leaveOutColourInTraining (val))
		{
			nColours += !colours[val];
			colours[occupancyGrid[i]] = 1;
		}
	}
	return nColours;
}

void RobotRecognitionTesting_printColoursInTrainingImage (
	FILE *outputFile,
	const int nColours,
	const uchar *colours)
{
	int i;
	if (nColours)
	{
		for (i = 0; i < 256; ++i)
		{
			if (colours[i])
			{
				fprintf (outputFile, "<RobotColourInTrainingImage>colourId=%d</RobotColourInTrainingImage>\n", i);
			}
		}
	}
	else
	{
		fprintf (outputFile, "<NoRobotColours />\n");
	}

	fflush (outputFile);
}

int RobotRecognitionTesting_anyRobotColoursInTrainingImage (
	const uchar *occupancyGrid)
{
	int nColours;
	uchar colours[256];

	nColours = RobotRecognitionTesting_getColoursInTrainingImage (
		colours,
		occupancyGrid);

	return (nColours != 0);
}

void RobotRecognitionTesting_displayTrainingImage (
	IplImage *iplImage,
	const int *windowSize,
	Image *originalImg,
	const uchar *trainingGrid,
	const int displayImgsWithNoColours)
{
	const int justShowImg = 1;
	int i;
	int nColours;
	uchar colours[256];

	nColours = RobotRecognitionTesting_getColoursInTrainingImage (
		colours,
		trainingGrid);

	if (justShowImg)
	{
		memcpy (iplImage->imageData, originalImg->data, originalImg->height * originalImg->wStep);
		cvNamedWindow ("robotRecTrainingImg", windowSize);
		cvShowImage ("robotRecTrainingImg", iplImage);
		cvWaitKey (0);
		return;
	}

	if (nColours)
	{
		for (i = 0; i < 256; ++i)
		{
			if (colours[i])
			{
				printf ("Colour id %d\n", i);

				// Refresh IplImage before drawing cells for this blob
				memcpy (iplImage->imageData, originalImg->data, originalImg->height * originalImg->wStep);

				Image_drawOccupancy (
					iplImage->imageData,
					trainingGrid,
					CAM_OCCUPANCY_GRID_ORIGIN_X,
					CAM_OCCUPANCY_GRID_ORIGIN_Y,
					CAM_OCCUPANCY_GRID_Y,
					i);

				cvNamedWindow ("robotRecTrainingImg", windowSize);
				cvShowImage ("robotRecTrainingImg", iplImage);
				cvWaitKey (0);
			}
		}
	}
	else if (displayImgsWithNoColours)
	{
		printf ("No colours\n");

		memcpy (iplImage->imageData, originalImg->data, originalImg->height * originalImg->wStep);

		cvNamedWindow ("robotRecTrainingImg", windowSize);
		cvShowImage ("robotRecTrainingImg", iplImage);
		cvWaitKey (0);
	}
}

extern char rootDirName[128];
extern const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS];
extern const float RobotRecognitionModel_colourAvgs[N_ROBOT_COLOURS];

void RobotRecognitionTesting_runRobotColourRecognitionOnDir()
{
	char dir[256];
	char buf[512];
	int i;
	IplImage *iplImage;
	CvSize size = {176, 143};
	int windowSize[] = {100, 10, 450, 450};
	FILE *f;
	const int sizeToRead = size.width * size.height * 3;
	Image *img, *img2;
	int exp;
	float *camScoreGrid;
	float *colourScoreGrid;
	ImgCellFeatures *cellFeatures;
	ImgFeatures imgFeatures;
	CamVectors camVectors;
	UncertaintyConstants uncertaintyConstants;
	OccupancyGrid *occupancyGrid = NULL;
	uchar occupancyGridTemp[SIZE_COOP_LOC_GRID];
	int anyRobotCells;
	int isObstacleInCamOccupancyGrid = 0;

	setupCamParams (ROBOT_PLATFORM);
	CamVectors_init (&camVectors);
	UncertaintyConstants_init (&uncertaintyConstants);

	iplImage = cvCreateImage (size, 8, 3);
	Image_ctor (&img, size.width, size.height, 3);
	Image_ctor (&img2, size.width, size.height, 3);

	cellFeatures = (ImgCellFeatures*)malloc (sizeof (ImgCellFeatures) * SIZE_COOP_LOC_GRID);
	camScoreGrid = (float*)malloc (sizeof (float) * SIZE_COOP_LOC_GRID);
	colourScoreGrid = (float*)malloc (sizeof (float) * N_ROBOT_COLOURS * SIZE_COOP_LOC_GRID);
	occupancyGrid = OccupancyGrid_alloc();


	i = 0; exp = 1; strcpy (dir, "20130728_gumstixFiles_2__collabOkButLoopCloseFailed/gum2/experiment_robot_0_194-19691231-Wed-160314");

	while (i < 500)
	{
		if (exp)
		{
			sprintf (buf, "%s/code/CollabExp/Files/%s/img%05d.dat", rootDirName, dir, i);
		}
		else
		{
			sprintf (buf, "%s/calibration/images/%s/img%05d.dat", rootDirName, dir, i);
		}
		f = fopen (buf, "r");

		if (f)
		{
			printf ("%s\n", buf);

			memset (img->data, 0, sizeToRead);
			fread (img->data, 1, sizeToRead, f);
			fclose (f);

			Image_rectify (img->data);
			Image_flip(img->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);

			memcpy (iplImage->imageData, img->data, sizeToRead);

			Image_toHSV (img->data);

			// As called from CameraProcessing_processSensorData (just before calling ObstacleRecognitionCore_isImageOk)
			ObstacleRecognition_calcImageFeatures (
				img,
				cellFeatures,
				&imgFeatures);

			// As called from CameraProcessing_processImageData...
			OccupancyGrid_init (occupancyGrid);

#if !defined(DEBUGGING_ROBOT_REC)
			assert(0);
#endif

			anyRobotCells = RobotRecognition_detectRobotsNEW (
				stdout,
#if VERBOSE_BLOB_DETECTION
				NULL,
				NULL,
				NULL,
				NULL,
#endif
#if defined(DEBUGGING_ROBOT_REC)
				0,
#endif
				img,
				&camVectors,
				&uncertaintyConstants,
				NULL, // robotDataArray
				0, // ownIndex
				NULL, // robotFinalEstimates
				colourScoreGrid,
				cellFeatures,
				occupancyGrid);

			printf ("RobotRecognition_detectRobotsNEW->anyRobotCells=%d\n", anyRobotCells);

			ObstacleRecognition_calcOccupancy (
				img,
				cellFeatures,
				&imgFeatures,
				occupancyGrid,
				camScoreGrid,
				anyRobotCells,
				&isObstacleInCamOccupancyGrid);
			
			ObstacleRecognitionTesting_gridFromOccupancyGrid (
				occupancyGrid,
				occupancyGridTemp);

			// These grids are usually set in SensorProcessing_processSensorData->CameraProcessing_processSensorData->asdf, and are used in
			// RobotMapIntegration_integrateMapData->MapIntegration_intCamScan->MapIntegration_intGridScan
			//
			// CameraProcessing_processSensorData
			//  ObstacleRecognition_calcImageFeatures
			//   ObstacleRecognitionCore_calcImgCellFeatures
			//   ObstacleRecognitionCore_calcImgFeatures
			//  CameraProcessing_processImageData
			//   OccupancyGrid_init
			//   RobotRecognition_detectRobotsNEW <-- given g_camOccupancyGrid
			//    RobotRecognitionCore_markRobotOccupiedCells
			//     (if colourScoreGrid[x] < threshold: set occupancyGrid->grid[x] == 3)
			//   ObstacleRecognition_calcOccupancy
			//    ObstacleRecognition_estimateGridOccupancy
			//     ObstacleRecognition_estimateGridOccupancy
			//      ObstacleRecognition_estimateCellOccupancy
			//       (set bit0:occupied/not, bit1:unknown/not, therefore vals are in range 0..3, so potential confusion with robot cells)
			//   CameraProcesing_setVisibleRobotRelativeLoc
			//   CameraProcessing_recordMapScan
			//    (copy g_camOccupancyGrid)
			//    (print <MapScan> to log)
			//
			// MapIntegration_intGridScan
			//  (if intUnknownCells: occupied = val != 0)
			//  (else: occupied = val == 1) <-- therefore conflict between robot cells and unknown cells won't matter in actual map
			//  MapIntegration_intCamScan
			//  MapIntegration_intBlankScan
			//   RobotMapIntegration_integrateMapData
			//    (set unknown cells to 1 (i.e. occupied))
			//   RobotMapIntegration_redrawLocalMap
			//    (set unknown cells to 0
			//
			// blue == 0 == unoccupied
			// white == 1 == defo occupied
			// green == 2 == unknown, unoccupied
			// red == 3 == unknow, occupied OR robot

			ObstacleRecognitionTesting_drawResultsGrid (
//				img->data,
				iplImage->imageData,
				occupancyGridTemp,
				CAM_OCCUPANCY_GRID_ORIGIN_X,
				CAM_OCCUPANCY_GRID_ORIGIN_Y,
				CAM_OCCUPANCY_GRID_Y);

			cvNamedWindow ("win", windowSize);
			cvShowImage ("win", iplImage);
			cvWaitKey (0);
		}
		++i;
	}

	free (occupancyGrid);
	free (camScoreGrid);
	free (colourScoreGrid);
	free (cellFeatures);

	Image_dtor (&img);
	Image_dtor (&img2);
	cvReleaseImage (&iplImage);
	cvCleanup();
}

void RobotRecognitionTesting_testRobotColourRecognition()
{
#if 1
	FILE *imgFile;
	FILE *outputFile;
	int i, nTrainingImages;
	int cellIndex, colourIndex;
	int trainedColour;
	float score;
	Image *img;
	char inputDir[256];
	float *colourScoreGrid;
	float *colourScorePtr;
	uchar *occupancyGridPtr;
	TrainingData *data;
	ImgCellFeatures *cellFeatures;

	int nCells[N_ROBOT_COLOURS];
	int nWrong[N_ROBOT_COLOURS];
	float avgScoreThisColour[N_ROBOT_COLOURS];
	float avgScoreOtherColour[N_ROBOT_COLOURS];
	int indexThisColour;
	float scoreThisColour;
	int bestOtherId;
	float bestOtherScore;

	Image_ctor (&img, 176, 143, 3);

	data = (TrainingData*)malloc (sizeof (TrainingData) * 500);
	TrainingData_init (data, 500);
	nTrainingImages = RobotRecognitionTraining_setupTrainingData (data);

	cellFeatures = (ImgCellFeatures*)malloc (sizeof (ImgCellFeatures) * SIZE_COOP_LOC_GRID);
	colourScoreGrid = (float*)malloc (sizeof (float) * N_ROBOT_COLOURS * SIZE_COOP_LOC_GRID);

	outputFile = fopen ("__RobotRecognitionTestOutput__.txt", "w");

	for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
	{
		nCells[colourIndex] = 0;
		nWrong[colourIndex] = 0;
		avgScoreThisColour[colourIndex] = 0.0f;
		avgScoreOtherColour[colourIndex] = 0.0f;
	}

	for (i = 0; i < nTrainingImages; ++i)
	{
		sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);

		imgFile = fopen (inputDir, "r");
		if (!imgFile)
		{
			printf ("Error: inputDir %s\n", inputDir);
		}
		assert (imgFile);
		fread (img->data, 1, img->width * img->height * 3, imgFile);
		fclose (imgFile);
		Image_rectify (img->data);
		Image_flip(img->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);
		Image_toHSV (img->data);

		ObstacleRecognitionCore_calcImgCellFeatures (img, cellFeatures, CAM_OCCUPANCY_GRID_Y, CAM_IMG_ORIG_X, CAM_IMG_ORIG_Y);

		RobotRecognitionModel_calcColourScores (cellFeatures, colourScoreGrid);

		colourScorePtr = colourScoreGrid;
		occupancyGridPtr = data[i].occupancyGrid;
		for (cellIndex = 0; cellIndex < SIZE_COOP_LOC_GRID; ++cellIndex, ++occupancyGridPtr)
		{
			trainedColour = *occupancyGridPtr;
			if (trainedColour == 20 || -1 == RobotRecognitionModel_mapColourIdToIndex(trainedColour))
			{
				// Background cells - or just not marked as a colour
				colourScorePtr += N_ROBOT_COLOURS;
				continue;
			}

			indexThisColour = RobotRecognitionModel_mapColourIdToIndex(trainedColour);
			bestOtherScore = MAX_FLT;
			bestOtherId = -1;

			for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
			{
				score = *colourScorePtr;
				if (colourIndex == indexThisColour)
				{
					scoreThisColour = score;
				}
				else
				{
					if (score < bestOtherScore)
					{
						bestOtherScore = score;
						bestOtherId = RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex];
					}
				}
				++colourScorePtr;
			}

			++nCells[indexThisColour];
			if (bestOtherScore < scoreThisColour)
			{
				++nWrong[indexThisColour];
			}
			avgScoreThisColour[indexThisColour] += scoreThisColour;
			avgScoreOtherColour[indexThisColour] += bestOtherScore;
		}
	}

	for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
	{
		avgScoreThisColour[colourIndex] /= nCells[colourIndex];
		avgScoreOtherColour[colourIndex] /= nCells[colourIndex];

		fprintf (
			outputFile,
			"index %2d id: %2d nCells:%6d nWrong: %6d (%12f) avgScore: %12f avgBestOtherScore: %12f\n",
			colourIndex,
			RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex],
			nCells[colourIndex],
			nWrong[colourIndex],
			(float)nWrong[colourIndex] / nCells[colourIndex],
			avgScoreThisColour[colourIndex],
			avgScoreOtherColour[colourIndex]);
	}

	fclose (outputFile);

	free (colourScoreGrid);
	free (data);
	free (cellFeatures);
	Image_dtor (&img);
#endif
}

void RobotRecognitionTesting_testRobotColourRecognitionAgainstBackgroundImages()
{
	FILE *imgFile;
	FILE *outputFile;
	int i, nTrainingImages;
	int cellIndex, colourIndex;
	float score;
	Image *img;
	CvSize size = {176, 143};
	IplImage *iplImage;
	int windowSize[] = {100, 10, 450, 450};
	char inputDir[256];
	float *colourScoreGrid;
	float *colourScorePtr;
	uchar *occupancyGridPtr;
	TrainingData *data;
	ImgCellFeatures *cellFeatures;
	int nCellsTooClose[N_ROBOT_COLOURS];
	int cellsTooCloseBins[N_ROBOT_COLOURS][50];
	const float colourTestingThreshold = 3.0f;
	const float colourTestingThreshold2 = 5.0f;
	int binNum;
	const int showImg = 0;

	Image_ctor (&img, 176, 143, 3);
	iplImage = cvCreateImage (size, 8, 3);

	data = (TrainingData*)malloc (sizeof (TrainingData) * 300);
	TrainingData_init (data, 300);
	nTrainingImages = ObstacleRecognitionTraining_setupTrainingData (data);

	cellFeatures = (ImgCellFeatures*)malloc (sizeof (ImgCellFeatures) * SIZE_COOP_LOC_GRID);
	colourScoreGrid = (float*)malloc (sizeof (float) * N_ROBOT_COLOURS * SIZE_COOP_LOC_GRID);

	outputFile = fopen ("__RobotRecognitionTestAgainstBackgroundOutput__.txt", "w");

	for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
	{
		nCellsTooClose[colourIndex] = 0;
		memset (cellsTooCloseBins[colourIndex], 0, sizeof (int) * 50);
	}

	for (i = 0; i < nTrainingImages; ++i)
//	for (i = 56; i < 57; ++i)
	{
		sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);

		imgFile = fopen (inputDir, "r");
		if (!imgFile)
		{
			printf ("Error: inputDir %s\n", inputDir);
		}
		assert (imgFile);
		fread (img->data, 1, img->width * img->height * 3, imgFile);
		fclose (imgFile);
		Image_rectify (img->data);
		Image_flip(img->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);
		if (showImg)
		{
			memcpy (iplImage->imageData, img->data, img->height * img->wStep);
		}
		Image_toHSV (img->data);

		ObstacleRecognitionCore_calcImgCellFeatures (img, cellFeatures, CAM_OCCUPANCY_GRID_Y, CAM_IMG_ORIG_X, CAM_IMG_ORIG_Y);

		RobotRecognitionModel_calcColourScores (cellFeatures, colourScoreGrid);

		// for testing...
		// compare colour scores for correct/incorrect cells
		colourScorePtr = colourScoreGrid;
		occupancyGridPtr = data[i].occupancyGrid;
		for (cellIndex = 0; cellIndex < SIZE_COOP_LOC_GRID; ++cellIndex, ++occupancyGridPtr)
		{
			for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
//			colourIndex=2; //5
			{
				score = *colourScorePtr;
//				colourAvg = RobotRecognitionModel_colourAvgs[colourIndex];
//				score /= colourAvg;
				if (score < colourTestingThreshold)
				{
					++nCellsTooClose[colourIndex];
					fprintf (
						outputFile,
						"img %3d cell %3d isOccupied %d colourIndex %2d colourId %2d score %12f\n",
						i,
						cellIndex,
						*occupancyGridPtr,
						colourIndex,
						RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex],
						score);
				}

				if (score < colourTestingThreshold2)
				{
					binNum = (int)(score / 0.1f);
					++cellsTooCloseBins[colourIndex][binNum];
				}
				++colourScorePtr;
			}
		}

		if (showImg)
		{
			cvNamedWindow ("bgImg", windowSize);
			cvShowImage ("bgImg", iplImage);
			cvWaitKey (0);
		}
	}

	fprintf (outputFile, "\n\n\n\n");

	fprintf (outputFile, "cells w score < %f\n", colourTestingThreshold);
	for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
	{
		fprintf (
			outputFile,
			"index %2d id: %2d nTooClose: %6d \n",
			colourIndex,
			RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex],
			nCellsTooClose[colourIndex]);
	}

	fprintf (outputFile, "\n\n\n\n");

	fprintf (outputFile, "cells w score < %f in steps of 0.1\n", colourTestingThreshold2);
	for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
	{
		fprintf (outputFile, "colourIndex=%2d colourId=%2d ", colourIndex, RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex]);
		for (i = 0; i < 50; ++i)
		{
			fprintf (outputFile, " %5d", cellsTooCloseBins[colourIndex][i]);
		}
		fprintf (outputFile, "\n");
	}



	fclose (outputFile);

	free (colourScoreGrid);
	free (data);
	free (cellFeatures);
	Image_dtor (&img);

	cvReleaseImage (&iplImage);
	cvCleanup();
}













int RobotRecognitionTesting_getRemainingRobotIndex (const int id1, const int id2)
{
	if ((id1 == 0 && id2 == 1) || (id1 == 1 && id2 == 0))
	{
		return 2;
	}
	else if ((id1 == 0 && id2 == 2) || (id1 == 2 && id2 == 0))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

extern void ObstacleRecognitionTesting_displayTrainingImage (
	IplImage *iplImage,
	const int *windowSize,
	Image *originalImg,
	const uchar *trainingGrid,
	const int displayImgsWithNoColours);

void RobotRecognitionTesting_testRobotRecognition (const int doRecognition)
{
	FILE *imgFile;
	FILE *outputFile;
	int i, nTrainingImages;
	Image *img;
	Image *displayImg;
	CvSize size = {176, 143};
	IplImage *iplImage;
	int windowSize[] = {100, 10, 450, 450};
	char inputDir[256];
	float *colourScoreGrid;
	float *camScoreGrid;
	TrainingData *data;
	ImgFeatures imgFeatures;
	ImgCellFeatures *cellFeatures;
	OccupancyGrid *occupancyGrid;
	int dummy;
	RobotData robotData[3]; // In normal code, will be taken from groupData
	int ownRobotIndex, visRobotIndex, remainingRobotIndex;
	int isLocSetInImg;
	int anyRobotCells;
	CamVectors camVectors;
	UncertaintyConstants uncertaintyConstants;
	List robotEstimates;
	RobotEstimate *robotEstimate;
	ListNode *iter;
	PointF focalPt, estLoc, error;
#if VERBOSE_BLOB_DETECTION
	int nColours;
	uchar colours[256];
#endif

	Image_ctor (&img, 176, 143, 3);
	Image_ctor (&displayImg, 176, 143, 3);
	iplImage = cvCreateImage (size, 8, 3);

	data = (TrainingData*)malloc (sizeof (TrainingData) * 500);
	TrainingData_init (data, 500);
	nTrainingImages = RobotRecognitionTraining_setupTrainingData (data);

	cellFeatures = (ImgCellFeatures*)malloc (sizeof (ImgCellFeatures) * SIZE_COOP_LOC_GRID);
	colourScoreGrid = (float*)malloc (sizeof (float) * N_ROBOT_COLOURS * SIZE_COOP_LOC_GRID);
	camScoreGrid = (float*)malloc (sizeof (float) * SIZE_COOP_LOC_GRID);
	occupancyGrid = OccupancyGrid_alloc();

	// Setup cam params for whatever robot was defined as ROBOT_PLATFORM in
	// RobotDefs.h for now, and then overwrite for the observer robot
	// specified for each training image
	setupCamParams (ROBOT_PLATFORM);
	CamVectors_init (&camVectors);
	UncertaintyConstants_init (&uncertaintyConstants);

	// Setup basic pose params for all robots - used in blob detection
	robotData[0].stdDev = 50.0f;
	robotData[1].stdDev = 50.0f;
	robotData[2].stdDev = 50.0f;

	if (doRecognition)
	{
		outputFile = fopen ("__RobotRecognitionBlobDetectionTestOutput__.txt", "w");
	}

	for (i = 0; i < nTrainingImages; ++i)
//	for (i = 7; i < 8; ++i)
//	for (i = 73; i < 74; ++i)
//	for (i = 213; i < 214; ++i)
	{
		sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);

		imgFile = fopen (inputDir, "r");
		if (!imgFile)
		{
			printf ("Error: inputDir %s\n", inputDir);
		}
		assert (imgFile);
		fread (img->data, 1, img->width * img->height * 3, imgFile);
		fclose (imgFile);
		Image_rectify (img->data);
		Image_flip(img->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);
		Image_clone (img, displayImg);
		Image_toHSV (img->data);

		assert (data[i].obs != -1 && data[i].vis != -1);

		robotEstimates = initList();

		// Set index of observer and set camera params for that robot platform
		ownRobotIndex = data[i].obs;
		visRobotIndex = data[i].vis;
		remainingRobotIndex = RobotRecognitionTesting_getRemainingRobotIndex (ownRobotIndex, visRobotIndex);
		setupCamParams (robotPlatformIds[ownRobotIndex]);
		CamVectors_init (&camVectors); // Setup cam vectors again once cam params are changed
		isLocSetInImg = data[i].isLoc;
		if (doRecognition && isLocSetInImg)
		{
			// Set pose of observer robot, with orient assumed to be 0
			robotData[ownRobotIndex].pose.loc.x = data[i].loc.x;
			robotData[ownRobotIndex].pose.loc.y = data[i].loc.y;
			robotData[ownRobotIndex].pose.orient = 0.0f;

			// Set pose of visible robot to origin, but with specified orient
			robotData[visRobotIndex].pose.loc.x = 0.0f;
			robotData[visRobotIndex].pose.loc.y = 0.0f;
			robotData[visRobotIndex].pose.orient = data[i].orient;

			// Set pose of remaining robot to something arbitrary
			robotData[remainingRobotIndex].pose.loc.x = 1000.0f;
			robotData[remainingRobotIndex].pose.loc.y = 1000.0f;
			robotData[remainingRobotIndex].pose.orient = 0.0f;

#if VERBOSE_BLOB_DETECTION
			fprintf (outputFile,
				"<RobotTrainingLoc>robotIndex=%d robotOrient=%f ownLoc=%f,%f</RobotTrainingLoc>\n",
				data[i].vis, data[i].orient,
				data[i].loc.x, data[i].loc.y);
#endif
		}
		else
		{
			// Required for blob detection - even if the obs/vis loc isn't set, we
			// still do blob detection. So set the visible robot to something reasonable.
			robotData[ownRobotIndex].pose.loc.x = 0.0f;
			robotData[ownRobotIndex].pose.loc.y = 0.0f;
			robotData[ownRobotIndex].pose.orient = 0.0f;
			robotData[visRobotIndex].pose.loc.x = 200.0f;
			robotData[visRobotIndex].pose.loc.y = 0.0f;
			robotData[visRobotIndex].pose.orient = 0.0f;
			robotData[remainingRobotIndex].pose.loc.x = 1000.0f;
			robotData[remainingRobotIndex].pose.loc.y = 1000.0f;
			robotData[remainingRobotIndex].pose.orient = 0.0f;
		}

		printf ("\nIMAGE %d\n", i);

		if (doRecognition)
		{
			// The functions called here should mirror CameraProcessing_processImageData

			fprintf (outputFile, "<TestRobotRecognition>image=%d file=%s</TestRobotRecognition>\n", i, inputDir);

			if (!RobotRecognitionTesting_anyRobotColoursInTrainingImage (
				data[i].occupancyGrid))
			{
				printf ("No robot colours\n");
				fprintf (outputFile, "<NoRobotColours />\n");
				continue;
			}

#if VERBOSE_BLOB_DETECTION
			nColours = RobotRecognitionTesting_getColoursInTrainingImage (
				colours,
				data[i].occupancyGrid);

			RobotRecognitionTesting_printColoursInTrainingImage (
				outputFile,
				nColours,
				colours);
#endif

			OccupancyGrid_init (occupancyGrid);

			ObstacleRecognition_calcImageFeatures (
				img,
				cellFeatures,
				&imgFeatures);

#if defined(DEBUGGING_ROBOT_REC)
#error "adsf"
			assert (1);
#else
			// DEBUGGING_ROBOT_REC should be defined in so params can be passed to functions
			assert (0);
#endif

			anyRobotCells = RobotRecognition_detectRobotsNEW (
				outputFile,
#if VERBOSE_BLOB_DETECTION
				displayImg,
				iplImage,
				windowSize,
				colours,
#endif
#if defined(DEBUGGING_ROBOT_REC)
				isLocSetInImg,
#endif
				img,
				&camVectors,
				&uncertaintyConstants,
				robotData,
				ownRobotIndex,
				&robotEstimates,
				colourScoreGrid,
				cellFeatures,
				occupancyGrid);

			if (robotEstimates.size)
			{
				Geometry_ptFromOrient (
					robotData[ownRobotIndex].pose.loc.x,
					robotData[ownRobotIndex].pose.loc.y,
					&focalPt.x,
					&focalPt.y,
					CAM_P_OFFSET_WORLD,
					robotData[ownRobotIndex].pose.orient);
				printf ("estimates: %d ownLoc=(%f,%f) focal=(%f,%f)\n",
					robotEstimates.size,
					robotData[ownRobotIndex].pose.loc.x, robotData[ownRobotIndex].pose.loc.y,
					focalPt.x, focalPt.y);

				iter = robotEstimates.front;
				while (iter)
				{
					robotEstimate = (RobotEstimate*)iter->value;

					if (robotEstimate->estType < FACE_EST_EDGE)
					{
						iter = iter->next;
						continue;
					}
					estLoc.x = focalPt.x + robotEstimate->estLoc.x;
					estLoc.y = focalPt.y + robotEstimate->estLoc.y;
					printf ("estimate: robotIndex=%d est=(%.2f,%.2f) cov=(%.2f,%.2f,%.2f,%.2f) estType=%d\n",
						robotEstimate->robotIndex,
						estLoc.x, estLoc.y,
						robotEstimate->estCov.mat[0], robotEstimate->estCov.mat[1], robotEstimate->estCov.mat[2], robotEstimate->estCov.mat[3],
						robotEstimate->estType);

					if (isLocSetInImg)
					{
						error.x = estLoc.x - robotData[data[i].vis].pose.loc.x;
						error.y = estLoc.y - robotData[data[i].vis].pose.loc.y;

						printf ("error=(%.2f,%.2f)\n", error.x, error.y);
					}
#if 1
					fprintf (outputFile,
						"<RobotTrainingEst>robotIndex=%d relEst=(%f,%f) focalPt=(%f,%f) finalEst=(%f,%f) cov=(%f,%f,%f,%f)",
						robotEstimate->robotIndex,
						robotEstimate->estLoc.x, robotEstimate->estLoc.y,
						focalPt.x, focalPt.y,
						estLoc.x, estLoc.y,
						robotEstimate->estCov.mat[0], robotEstimate->estCov.mat[1], robotEstimate->estCov.mat[2], robotEstimate->estCov.mat[3]);
					if (isLocSetInImg)
					{
						fprintf (outputFile, " error=(%f,%f,%f) actualLoc=(%f,%f)",
							error.x, error.y, sqrt (error.x * error.x + error.y * error.y),
							robotData[data[i].vis].pose.loc.x, robotData[data[i].vis].pose.loc.y);
					}
					fprintf (outputFile, "</RobotTrainingEst>\n");
#endif
					iter = iter->next;
				}

				List_clear (&robotEstimates, 1);
			}

			ObstacleRecognition_calcOccupancy (
//				outputFile,
				img,
				cellFeatures,
				&imgFeatures,
				occupancyGrid,
				camScoreGrid,
				anyRobotCells,
				&dummy);

			memcpy (iplImage->imageData, displayImg->data, iplImage->widthStep * iplImage->height);
			Image_drawOccupancy3 (
				iplImage->imageData,
				occupancyGrid,
				CAM_OCCUPANCY_GRID_ORIGIN_X,
				CAM_OCCUPANCY_GRID_ORIGIN_Y,
				CAM_OCCUPANCY_GRID_Y);
			cvNamedWindow ("occupGrid", windowSize);
			cvShowImage ("occupGrid", iplImage);
			cvWaitKey (1);

			fflush (outputFile);
		}
		else
		{
			RobotRecognitionTesting_displayTrainingImage (
				iplImage,
				windowSize,
				displayImg,
				data[i].occupancyGrid,
				1);
		}

		if (doRecognition)
		{
			fprintf (outputFile, "\n\n\n");
			fflush (outputFile);
		}

//		cvWaitKey (0);
		cvWaitKey (1);

//		break;
	}

	if (doRecognition)
	{
		fclose (outputFile);
	}

	free (occupancyGrid);
	free (camScoreGrid);
	free (colourScoreGrid);
	free (data);
	free (cellFeatures);
	Image_dtor (&displayImg);
	Image_dtor (&img);
	cvReleaseImage (&iplImage);
	cvCleanup();
}



















void RobotRecognitionTesting_verifyColourGrids()
{
	FILE *imgFile;
	int i, nTrainingImages;
	Image *img;
	Image *displayImg;
	CvSize size = {176, 143};
	IplImage *iplImage;
	int windowSize[] = {100, 10, 450, 450};
	char inputDir[256];
	TrainingData *data;
	CamVectors camVectors;
	UncertaintyConstants uncertaintyConstants;
	int colourId;
	int nColours;
	uchar colours[256];

	Image_ctor (&img, 176, 143, 3);
	Image_ctor (&displayImg, 176, 143, 3);
	iplImage = cvCreateImage (size, 8, 3);

	data = (TrainingData*)malloc (sizeof (TrainingData) * 500);
	TrainingData_init (data, 500);
	nTrainingImages = RobotRecognitionTraining_setupTrainingData (data);

	// Setup cam params for whatever robot was defined as ROBOT_PLATFORM in
	// RobotDefs.h for now, and then overwrite for the observer robot
	// specified for each training image
	setupCamParams (ROBOT_PLATFORM);
	CamVectors_init (&camVectors);
	UncertaintyConstants_init (&uncertaintyConstants);

//	for (colourId = 0; colourId < N_ROBOT_COLOURS_TOTAL; ++colourId)
	for (colourId = 12; colourId < N_ROBOT_COLOURS_TOTAL; ++colourId)
	{
		printf ("COLOUR %2d *********************************************************\n", colourId);

		for (i = 0; i < nTrainingImages; ++i)
		{
			nColours = RobotRecognitionTesting_getColoursInTrainingImage (
				colours,
				data[i].occupancyGrid);
			if (!colours[colourId])
			{
				continue;
			}
			printf ("img %3d\n", i);
			sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);
			imgFile = fopen (inputDir, "r");
			if (!imgFile)
			{
				printf ("Error: inputDir %s\n", inputDir);
			}
			assert (imgFile);
			fread (img->data, 1, img->width * img->height * 3, imgFile);
			fclose (imgFile);
			Image_rectify (img->data);
			Image_flip(img->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);
			Image_clone (img, displayImg);
			Image_toHSV (img->data);

			memcpy (iplImage->imageData, displayImg->data, displayImg->height * displayImg->wStep);
			Image_drawOccupancy (
				iplImage->imageData,
				data[i].occupancyGrid,
				CAM_OCCUPANCY_GRID_ORIGIN_X,
				CAM_OCCUPANCY_GRID_ORIGIN_Y,
				CAM_OCCUPANCY_GRID_Y,
				colourId);

			cvNamedWindow ("robotRecTrainingImg", windowSize);
			cvShowImage ("robotRecTrainingImg", iplImage);
			cvWaitKey (0);
		}
	}

	free (data);
	Image_dtor (&displayImg);
	Image_dtor (&img);
	cvReleaseImage (&iplImage);
	cvCleanup();
}

#endif // defined(IS_WIN) && defined(SIMULATION)
