#include "../../Common/RobotDefs.h"

#ifdef SIMULATION

#include "ObstacleDetection.h"
#include "../../Board/OpenCV/CV.h"
#include "../../Common/BitArray.h"


// Global to keep off stack
extern OccupancyGrid g_camOccupancyGrid;

//void ObstacleDetection_compareCamOccupancyGrids (const int imageIndex, const uchar trainedGrid[CAM_OCCUPANCY_GRID_X * 18], uchar observedGrid[CAM_OCCUPANCY_GRID_X * 18], const int gridY, int results[7])
void ObstacleDetection_compareCamOccupancyGrids (const int imageIndex, const uchar *trainedGrid, uchar *observedGrid, const int gridY, int results[7])
{
	int i;
	int correct;
	int observedVal, trainedVal;
	int nIncorrect = 0;

	for (i = 0; i < CAM_OCCUPANCY_GRID_X * gridY; ++i)
	{
		observedVal = observedGrid[i];
		trainedVal = trainedGrid[i];

		// Either correct, or the trained val was "don't know"
		correct = (observedVal == trainedVal || trainedVal == 2);

		observedGrid[i] = correct;
		//TwoBitArray_setIndex (resultsGrid, i, correct);

		// total, totalCorrect, totalIncorrect, totalPos, totalNeg, totalFalseNeg, totalFalsePos
		results[0] += 1;
		if (correct)
		{
			results[1] += 1;
		}
		else
		{
			results[2] += 1;
			nIncorrect += 1;
		}

		if (trainedVal)
		{
			results[3] += 1;
			if (!correct)
			{
				results[5] += 1;
			}
		}
		else
		{
			results[4] += 1;
			if (!correct)
			{
				results[6] += 1;
			}
		}
	}

	if (nIncorrect)
	{
#if 1
		printf ("img %d : %d incorrect\n", imageIndex, nIncorrect);
#endif
	}
}

void ObstacleDetection_extractOccupancyGrid (OccupancyGrid *occupancyGrid, uchar *values, const int gridX, const int gridY)
{
	int i, lim;
	uchar val;
	lim = gridX * gridY;

	for (i = 0; i < lim; ++i)
	{
		val = TwoBitArray_checkIndex (occupancyGrid->grid, i);
		values[i] = val;
	}
}

extern char rootDirName[128];
extern ImgCellFeatures g_cellFeatures[SIZE_COOP_LOC_GRID];
extern PointF g_camScoreGrid[SIZE_COOP_LOC_GRID];

void ObstacleDetection_test()
{
	TrainingData data[147];
	int nTrainingImages;
	int closestImgGroupIndex;
	int i;
	FILE *f;
	char path[256];
	Image *img;
	ImgFeatures imgFeatures;
	ImgFeatures imgInvFeatures;
	uchar resultsGrid[CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y];
	int isObstacleInCamOccupancyGrid;
	int results[7]; // total, totalCorrect, totalIncorrect, totalPos, totalNeg, totalFalseNeg, totalFalsePos
	PointI orig, flip;
#define DISPLAY_IMGS
#ifdef DISPLAY_IMGS
	Image *displayImg;
	CvSize size = {176, 143};
	const int nChannels = 3;
	IplImage *iplImage = cvCreateImage (size, 8, nChannels);
	int windowSize[] = {100, 10, 450, 450};
	const int sizeToRead = size.width * size.height * nChannels;

	Image_ctor (&displayImg, 176, 143, 3);
#endif

	Image_ctor (&img, 176, 143, 3);

	TrainingData_init (data, 147);
	nTrainingImages = ObstacleDetection_setupTrainingData (data);

	printf ("Testing %d images\n", nTrainingImages);

	{
		memset (results, 0, sizeof (int) * 7);

		for (i = 0; i < nTrainingImages; ++i)
		{
			sprintf (path, "%s/calibration/images/%s", rootDirName, data[i].imgName);
			f = fopen (path, "r");
			assert (f);
			fread (img->data, 1, img->width * img->height * 3, f);
			fclose (f);
			Image_rectify (img->data);
			orig.x = data[i].orig >> 8;
			orig.y = data[i].orig & 255;
			flip.x = data[i].flip >> 1;
			flip.y = data[i].flip & 1;
			Image_flip(img->data, flip.x, flip.y);
#ifdef DISPLAY_IMGS
			memcpy (displayImg->data, img->data, sizeToRead);
#endif
			Image_toHSV (img->data);

			ObstacleDetection_calcImgCellFeaturesForTraining (img, g_cellFeatures, data[i].occupancyGrid, data[i].gridY, orig.x, orig.y, 2);
			ObstacleDetection_calcImgFeatures (&imgFeatures, g_cellFeatures, data[i].gridY);
			ObstacleDetection_calcInvImgFeatures (&imgFeatures, &imgInvFeatures);

			ObstacleDetection_calcImgOccupancy (&imgFeatures, &imgInvFeatures, g_cellFeatures, g_camScoreGrid, &closestImgGroupIndex, data[i].gridY);

			ObstacleDetection_calcOccupiedEst (
				data[i].gridY,
				&g_camOccupancyGrid,
				g_camScoreGrid,
				&isObstacleInCamOccupancyGrid);

			//ObstacleDetection_calcOccupiedEst2 (
			//	data[i].gridY,
			//	&g_camOccupancyGrid,
			//	g_camScoreGrid,
			//	g_cellFeatures,
			//	&imgInvFeatures,
			//	closestImgGroupIndex,
			//	&isObstacleInCamOccupancyGrid);

			ObstacleDetection_extractOccupancyGrid (&g_camOccupancyGrid, resultsGrid, CAM_OCCUPANCY_GRID_X, data[i].gridY);

			// Compare the camera's occupancyGrid to training data: data[i].occupancyGrid.
			// Set the cells of resultsGrid to 0=false, 1=correct
			ObstacleDetection_compareCamOccupancyGrids (i, data[i].occupancyGrid, resultsGrid, data[i].gridY, results);

			fclose (f);

#ifdef DISPLAY_IMGS
			Image_drawOccupancy (displayImg->data, resultsGrid, orig.x, orig.y, data[i].gridY, 1);

			memcpy (iplImage->imageData, displayImg->data, sizeToRead);
			cvNamedWindow ("win", windowSize);
			cvShowImage ("win", iplImage);
			cvWaitKey (0);
#endif
		}

		printf("nCells %d\n", results[0]);
		printf("nCorrect %d\n", results[1]);
		printf("nIncorrect %d\n", results[2]);
		printf("nPos %d\n", results[3]);
		printf("nNeg %d\n", results[4]);
		printf("nFalseNeg %d\n", results[5]);
		printf("nFalsePos %d\n", results[6]);
	}

	Image_dtor (&img);
#ifdef DISPLAY_IMGS
	Image_dtor (&displayImg);

	cvReleaseImage (&iplImage);
	cvCleanup();
#undef DISPLAY_IMGS
#endif
}

#endif // SIMULATION
