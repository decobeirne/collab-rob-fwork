#include "ObstacleRecognitionTesting.h"
#include "ObstacleRecognitionCore.h"
#include "ObstacleRecognitionTraining.h"
#include "ObstacleRecognitionModel.h"
#include "ObstacleRecognition.h"
#include "../../Common/BitArray.h"

#if defined(IS_WIN) && defined(SIMULATION)

extern ImgCellFeatures g_cellFeatures[SIZE_COOP_LOC_GRID];
extern ImgFeatures g_imgFeatures;

int ObstacleRecognitionTesting_countColourInTrainingGrid (
	const int colour,
	const uchar *occupancyGrid)
{
	int i, n;
	n = 0;
	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		n += (int)(occupancyGrid[i] == colour);
	}
	return n;
}

void ObstacleRecognitionTesting_displayTrainingImage (
	IplImage *iplImage,
	const int *windowSize,
	Image *originalImg,
	const uchar *trainingGrid,
	const int displayImgsWithNoColours)
{
	int colour;
	int nColours[4];

	for (colour = 0; colour < 4; ++colour)
	{
		nColours[colour] = ObstacleRecognitionTesting_countColourInTrainingGrid (
			colour,
			trainingGrid);
	}

	if (nColours[0] || nColours[1] || nColours[2]|| nColours[3])
	{
		for (colour = 0; colour < 4; ++colour)
		{
			if (nColours[colour])
			{
				memcpy (iplImage->imageData, originalImg->data, originalImg->height * originalImg->wStep);

				Image_drawOccupancy (
					iplImage->imageData,
					trainingGrid,
					CAM_OCCUPANCY_GRID_ORIGIN_X,
					CAM_OCCUPANCY_GRID_ORIGIN_Y,
					CAM_OCCUPANCY_GRID_Y,
					colour);

				cvNamedWindow ("obstRecTrainingImg", windowSize);
				cvShowImage ("obstRecTrainingImg", iplImage);
				cvWaitKey (0);
			}
		}
	}
	else if (displayImgsWithNoColours)
	{
		memcpy (iplImage->imageData, originalImg->data, originalImg->height * originalImg->wStep);

		cvNamedWindow ("obstRecTrainingImg", windowSize);
		cvShowImage ("obstRecTrainingImg", iplImage);
		cvWaitKey (0);
	}
}

void ObstacleRecognitionTesting_checkNeighbours (
	const uchar *trainingOccupancyGrid,
	const float *camScoreGrid,
	const int i,
	const int j,
	const int gridY,
	FILE *outputFile)
{
	int k, l;
	int trainedOccup;
	float val;

	for (l = max (0, j - 1); l < min (gridY, j + 2); ++l)
	{
		for (k = max (0, i - 1); k < min (CAM_OCCUPANCY_GRID_X, i + 2); ++k)
		{
			if (l == j && k == i)
			{
				continue;
			}

			trainedOccup = trainingOccupancyGrid[k + l * CAM_OCCUPANCY_GRID_X];
			val = camScoreGrid[k + l * CAM_OCCUPANCY_GRID_X];

			fprintf (outputFile, "nbour (%2d,%2d) trained %d camScore %f\n",
				k, l,
				trainedOccup,
				val);
		}
	}

//	fprintf (outputFile, "\n");

}

void ObstacleRecognitionTesting_checkEstimatedOccupancy (
	const OccupancyGrid *camOccupancyGrid,
	const uchar *trainingOccupancyGrid,
	const float *camScoreGrid,
	const ImgCellFeatures *cellFeatures,
	uchar *resultsGrid,
	const int gridY,
	const int imgIndex,
	const char *imgPath,
	FILE *outputFile)
{
	int i, j;
	int estdOccup, trainedOccup, result;
	float val;

	//fprintf (outputFile, "img %3d %s\n", imgIndex, imgPath);

	for (j = 0; j < gridY; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			estdOccup = TwoBitArray_checkCoords (
				camOccupancyGrid->grid,
				i,
				j,
				CAM_OCCUPANCY_GRID_X);
			trainedOccup = trainingOccupancyGrid[i + j * CAM_OCCUPANCY_GRID_X];

			// See ObstacleRecognitionTesting.h for explanation.
			if (estdOccup == 2 && trainedOccup == 2)
			{
				result = 4;
			}
			else if (estdOccup == 2 && trainedOccup != 2)
			{
				result = 3;
			}
			else if (estdOccup != 2 && trainedOccup == 2)
			{
				result = 2;
			}
			else if (estdOccup != trainedOccup)
			{
				result = 1;
			}
			else
			{
				result = 0;
			}
			resultsGrid[i + j * CAM_OCCUPANCY_GRID_X] = result;

			//if (estdOccup != trainedOccup && trainedOccup != 2 /*2==dontKnow*/)
			if (result == 1 || result == 3)
			{
				val = camScoreGrid[i + j * CAM_OCCUPANCY_GRID_X];

				fprintf (
					outputFile,
					"cell (%2d,%2d) est %d trained %d camScore %f\n",
					i, j,
					estdOccup,
					trainedOccup,
					val);

				ObstacleRecognitionTesting_checkNeighbours (
					trainingOccupancyGrid,
					camScoreGrid,
					i,
					j,
					gridY,
					outputFile);
			}
		}
	}
}

void ObstacleRecognitionTesting_drawResult (
	uchar *ptr,
	const uchar result)
{
	// Values are set in ObstacleRecognitionTesting_checkEstimatedOccupancy, see
	// ObstacleRecognitionTesting.h for explanation.

	switch (result)
	{
	case 0:
		ptr[0] = 255;ptr[1] = 0;ptr[2] = 0;
		ptr[3] = 255;ptr[4] = 0;ptr[5] = 0;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 255;ptr[1] = 0;ptr[2] = 0;
		ptr[3] = 255;ptr[4] = 0;ptr[5] = 0;
		break;
	case 1:
		ptr[0] = 255;ptr[1] = 255;ptr[2] = 255;
		ptr[3] = 255;ptr[4] = 255;ptr[5] = 255;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 255;ptr[1] = 255;ptr[2] = 255;
		ptr[3] = 255;ptr[4] = 255;ptr[5] = 255;
		break;
	case 2:
		ptr[0] = 0;ptr[1] = 255;ptr[2] = 0;
		ptr[3] = 0;ptr[4] = 255;ptr[5] = 0;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 0;ptr[1] = 255;ptr[2] = 0;
		ptr[3] = 0;ptr[4] = 255;ptr[5] = 0;
		break;
	case 3:
		ptr[0] = 0;ptr[1] = 0;ptr[2] = 255;
		ptr[3] = 0;ptr[4] = 0;ptr[5] = 255;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 0;ptr[1] = 0;ptr[2] = 255;
		ptr[3] = 0;ptr[4] = 0;ptr[5] = 255;
		break;
	default: // 4
		ptr[0] = 0; ptr[1] = 255; ptr[2] = 255;
		ptr[3] = 0; ptr[4] = 255; ptr[5] = 255;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 0; ptr[1] = 255; ptr[2] = 255;
		ptr[3] = 0; ptr[4] = 255; ptr[5] = 255;
		break;
	}
}

void ObstacleRecognitionTesting_drawResultsGrid (
	uchar *data,
	const uchar *resultsGrid,
	const int origX,
	const int origY,
	const int gridY)
{
	int i, j, x, y;
	uchar *imgPtr;
	const uchar *gridPtr = resultsGrid;

	for (j = 0; j < gridY; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			x = origX + i * CAM_OCCUPANCY_CELL_X + CAM_OCCUPANCY_CELL_X / 2;
			y = origY + j * CAM_OCCUPANCY_CELL_Y + CAM_OCCUPANCY_CELL_Y / 2;
			imgPtr = data + (x * 3 + y * (CAM_IMG_W * 3));

			ObstacleRecognitionTesting_drawResult (imgPtr, *gridPtr);
			++gridPtr;
		}
	}
}

void ObstacleRecognitionTesting_gridFromOccupancyGrid (
	OccupancyGrid *occupancyGrid,
	uchar *ucharGrid)
{
	int i, j;
	uchar val;

	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			val = TwoBitArray_checkCoords (occupancyGrid->grid, i, j, CAM_OCCUPANCY_GRID_X);
			ucharGrid[i + j * CAM_OCCUPANCY_GRID_X] = val;
		}
	}
}

extern char rootDirName[128];

void ObstacleRecognitionTesting_testCorruptImages()
{
	char paths[20][256];
	int i, n = 0;
	Image *img, *displayImg;
	CvSize size = {176, 143};
	IplImage *iplImage;
	int windowSize[] = {100, 10, 450, 450};
	FILE *imgFile;
//	ImgCellFeatures *cellFeatures;
//	ImgFeatures imgFeatures;

	iplImage = cvCreateImage (size, 8, 3);
	Image_ctor (&img, 176, 143, 3);
	Image_ctor (&displayImg, 176, 143, 3);
//	cellFeatures = (ImgCellFeatures*)malloc (sizeof (ImgCellFeatures) * SIZE_COOP_LOC_GRID);

	sprintf (paths[n++], "%s/calibration/images/robot2_locRobot4_20130329/img00021.dat", rootDirName); // fine
	sprintf (paths[n++], "%s/calibration/images/robot2_locRobot4_20130329/img00022.dat", rootDirName); // speckles
//	sprintf (paths[n++], "%s/calibration/images/robot2_locRobot4_20130329/img00042.dat", rootDirName); // black
//	sprintf (paths[n++], "%s/calibration/images/robot2_locRobot4_20130329/img00043.dat", rootDirName); // black
	for (i = 0; i < n; ++i)
	{
		imgFile = fopen (paths[i], "r");
		assert (imgFile);
		fread (img->data, 1, img->width * img->height * 3, imgFile);
		fclose (imgFile);
		Image_rectify (img->data);
		Image_flip(img->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);
		Image_clone (img, displayImg);
		Image_toHSV (img->data);

		setupCamParams (2);

		ObstacleRecognition_calcImageFeatures (img, g_cellFeatures, &g_imgFeatures);
//		ObstacleRecognitionCore_calcImgFeatures (&g_imgFeatures, g_cellFeatures, CAMM_OCCUPANCY_GRID_Y);
//		ObstacleRecognitionCore_calcImgCellFeatures (img, cellFeatures, CAM_OCCUPANCY_GRID_Y, CAM_IMG_ORIG_X, CAM_IMG_ORIG_Y);

//		printf ("%f %f %f %f %f %f \n", g_cellFeatures[0].vals[0], g_cellFeatures[0].vals[1], g_cellFeatures[0].vals[2], g_cellFeatures[0].vals[3], g_cellFeatures[0].vals[4], g_cellFeatures[0].vals[5]);


		ObstacleRecognitionCore_isImageOk (img, g_cellFeatures, &g_imgFeatures);
		printf ("%f %f %f %f %f %f ", g_imgFeatures.vals[0], g_imgFeatures.vals[1], g_imgFeatures.vals[2], g_imgFeatures.vals[3], g_imgFeatures.vals[4], g_imgFeatures.vals[5]);
		printf ("%f %f %f %f %f %f\n", g_imgFeatures.vals[6], g_imgFeatures.vals[7], g_imgFeatures.vals[8], g_imgFeatures.vals[9], g_imgFeatures.vals[10], g_imgFeatures.vals[11]);
		printf ("%d\n", g_imgFeatures.nCellsWithWeirdPixels);
		printf ("\n");

		memcpy (iplImage->imageData, displayImg->data, displayImg->height * displayImg->wStep);
		cvNamedWindow ("testingCorruptImgs", windowSize);
		cvShowImage ("testingCorruptImgs", iplImage);
		cvWaitKey (0);
	}

//	free (cellFeatures);
	Image_dtor (&img);
	Image_dtor (&displayImg);
	cvReleaseImage (&iplImage);
	cvCleanup();
}

void ObstacleRecognitionTesting_testObstacleRecognition (
	const int doObstacleRecognition)
{
	TrainingData *data;
	ImgCellFeatures *cellFeatures;
	OccupancyGrid camOccupancyGrid;
	float *camScoreGrid;
	int nTrainingImages;
	int i;
	int j;
	FILE *imgFile, *outputFile;
	char path[256];
	Image *img;
	ImgFeatures imgFeatures;
	int isObstacleInCamOccupancyGrid;
	uchar resultsGrid[SIZE_COOP_LOC_GRID];
	int tempBin[5];
	int resultsBin[5];
	Image *displayImg;
	CvSize size = {176, 143};
	IplImage *iplImage;
	int windowSize[] = {100, 10, 450, 450};

	iplImage = cvCreateImage (size, 8, 3);
	Image_ctor (&displayImg, 176, 143, 3);

	data = (TrainingData*)malloc (sizeof (TrainingData) * 200);
	cellFeatures = (ImgCellFeatures*)malloc (sizeof (ImgCellFeatures) * SIZE_COOP_LOC_GRID);
	camScoreGrid = (float*)malloc (sizeof (float) * SIZE_COOP_LOC_GRID);

	Image_ctor (&img, 176, 143, 3);
	TrainingData_init (data, 200);
	nTrainingImages = ObstacleRecognitionTraining_setupTrainingData (data);

	memset (resultsBin, 0, sizeof (int) * 5);

	if (doObstacleRecognition)
	{
		outputFile = fopen ("__ObstacleRecognitionTestOutput__.txt", "w");
	}

	printf ("Testing obstacle recognition on %d images\n", nTrainingImages);

	for (i = 0; i < nTrainingImages; ++i)
//	for (i = 0; i < 1; ++i)
	{
		printf ("img %d\n", i);
		sprintf (path, "%s/calibration/images/%s", rootDirName, data[i].imgName);
		imgFile = fopen (path, "r");
		assert (imgFile);
		fread (img->data, 1, img->width * img->height * 3, imgFile);
		fclose (imgFile);
		Image_rectify (img->data);
		Image_flip(img->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);
		Image_clone (img, displayImg);
		Image_toHSV (img->data);

		if (doObstacleRecognition)
		{
			OccupancyGrid_init (&camOccupancyGrid);

			ObstacleRecognitionCore_calcImgCellFeatures (img, cellFeatures, CAM_OCCUPANCY_GRID_Y, CAM_IMG_ORIG_X, CAM_IMG_ORIG_Y);
			ObstacleRecognitionCore_calcImgFeatures (&imgFeatures, cellFeatures, CAM_OCCUPANCY_GRID_Y);

			ObstacleRecognitionModel_calcObstacleGroupScores (
				&imgFeatures,
				cellFeatures,
				camScoreGrid,
				CAM_OCCUPANCY_GRID_Y);

			isObstacleInCamOccupancyGrid = ObstacleRecognition_estimateGridOccupancy (
				CAM_OCCUPANCY_GRID_Y,
				&camOccupancyGrid,
				camScoreGrid,
				0); /*anyRobotCells*/

			ObstacleRecognitionTesting_checkEstimatedOccupancy (
				&camOccupancyGrid,
				data[i].occupancyGrid,
				camScoreGrid,
				cellFeatures,
				resultsGrid,
				CAM_OCCUPANCY_GRID_Y,
				i,
				path,
				outputFile);

			// See ObstacleRecognitionTesting_checkEstimatedOccupancy for results flags
			memset (tempBin, 0, sizeof (int) * 5);
			for (j = 0; j < (CAM_OCCUPANCY_GRID_Y * CAM_OCCUPANCY_GRID_X); ++j)
			{
				++tempBin[resultsGrid[j]];
			}

			fprintf (
				outputFile,
				"img %3d confident: correct:%5d incorrect:%5d notKnown:%5d notconfident: known:%5d notKnown:%5d\n",
				i, tempBin[0], tempBin[1], tempBin[2], tempBin[3], tempBin[4]);
			resultsBin[0] += tempBin[0];
			resultsBin[1] += tempBin[1];
			resultsBin[2] += tempBin[2];
			resultsBin[3] += tempBin[3];
			resultsBin[4] += tempBin[4];
		}
		else
		{
			ObstacleRecognitionTesting_displayTrainingImage (
				iplImage,
				windowSize,
				displayImg,
				data[i].occupancyGrid,
				1);
		}
	}

	if (doObstacleRecognition)
	{
		fprintf (
			outputFile,
			"TOTAL   confident: correct:%5d incorrect:%5d notKnown:%5d notconfident: known:%5d notKnown:%5d\n",
			resultsBin[0], resultsBin[1], resultsBin[2], resultsBin[3], resultsBin[4]);

		fclose (outputFile);
	}

	free (camScoreGrid);
	free (cellFeatures);
	free (data);
	Image_dtor (&img);
	Image_dtor (&displayImg);
	cvReleaseImage (&iplImage);
	cvCleanup();
}

void ObstacleRecognitionTesting_displayImagesInDir (
	const int drawGrid)
{
	char dir[256];
	char buf[512];
	int i;
	IplImage *iplImage;
	CvSize size = {176, 143};
	int windowSize[] = {100, 10, 450, 450};
	FILE *f;
	const int sizeToRead = size.width * size.height * 3;
	Image *img;
	int exp;

	iplImage = cvCreateImage (size, 8, 3);
	Image_ctor (&img, size.width, size.height, 3);

	// Cooperative localisation. Updated 2012/04/05
//	i = 0; exp = 0; strcpy (dir, "robot3_locRobot2_20111113"); // Probably useful for referencing for coopLoc
//	i = 0; exp = 0; strcpy (dir, "robot3_locRobot2_20111211"); // Probably useful for referencing for coopLoc
//	i = 0; exp = 0; strcpy (dir, "robot2_locRobot3_20120319"); // Blue and yellow blobs, maybe useful for verifying orients for edge detection
//	i = 0; exp = 0; strcpy (dir, "robot2_locRobot3_20120331"); // Blue and yellow blobs, maybe useful for verifying orients for edge detection
//	i = 0; exp = 0; strcpy (dir, "robot3_locRobot2_20120422"); // Orange and green blobs, maybe useful for verifying orients for edge detection

	// Cooperative localisation, updated 2012/05/08 (white balance a problem on some of these)
//	i = 0; exp = 0; strcpy (dir, "robot2_locRobot3_20120506_0"); // Colour cards, plus cardboard, colour settings look wrong
//	i = 0; exp = 0; strcpy (dir, "robot2_locRobot3_20120506_1"); // Colour cards, plus cardboard, colour settings look wrong
//	i = 0; exp = 0; strcpy (dir, "robot3_locRobot2_20120506_0"); // Measured imgs of robot on darker floor
//	i = 0; exp = 0; strcpy (dir, "robot3_locRobot2_20120506_1"); // More measured imgs of robot

	// Using these for robot recognition training as of 20130123. White balance is now off.
//	i = 0; exp = 0; strcpy (dir, "robot2_cards_2_20120527");
//	i = 0; exp = 0; strcpy (dir, "robot2_colours_20120715");

	// Using for robot recognition. Also, locs are setup but not exactly accurate, so can
	// use for verifying robot loc estimation. See readme file in dir for info
//	i = 0; exp = 0; strcpy (dir, "robot2_locRobot3_20130201");

	// Using for robot recognition. Accurate locs are setup. See readme in dir for more info
//	i = 0; exp = 0; strcpy (dir, "robot3_locRobot2_20130209");

	// Using for camera calibration on robot 3. Check readme files in dirs for more details
//	i = 0; exp = 0; strcpy (dir, "robot3_focus_20130210");
//	i = 0; exp = 0; strcpy (dir, "robot3_calib_20130210");
//	i = 0; exp = 0; strcpy (dir, "robot3_calib_20130328");
//	i = 0; exp = 0; strcpy (dir, "robot3_calib_20130328_2");

	// Using for obstacle recognition training as of 20130123
//	i = 0; exp = 0; strcpy (dir, "20120526_pcGumstixExperiments/experiment_robot_0_1005-19691231-Wed-161645");
//	i = 0; exp = 0; strcpy (dir, "20120526_pcGumstixExperiments/experiment_robot_0_1310-19691231-Wed-162150");
//	i = 0; exp = 0; strcpy (dir, "20120526_pcGumstixExperiments/experiment_robot_0_1714-19691231-Wed-162834");
//	i = 0; exp = 0; strcpy (dir, "20120703_pcGumstixExperiments/experiment_robot_0_1008-19691231-Wed-161648");
//	i = 0; exp = 0; strcpy (dir, "robot2_20120708");
//	i = 0; exp = 0; strcpy (dir, "robot2_envir_20120715");
//	i = 61; exp = 0; strcpy (dir, "robot2_envir_20130201");

	// Camera calib. More info in readme files
//	i = 0; exp = 0; strcpy (dir, "robot4_focus_20130329");
//	i = 0; exp = 0; strcpy (dir, "robot2_calib_20130329");
//	i = 0; exp = 0; strcpy (dir, "robot4_calib_20130329");
//	i = 0; exp = 0; strcpy (dir, "robot234_checkOrigin");
//	i = 0; exp = 0; strcpy (dir, "robot2_newColours_20130421");
//	i = 1; exp = 0; strcpy (dir, "robot3_newColours_20130421"); // Photos of robot recognition calibration setup are in dir

	// More robot recognition. Locs are set and described in readme.txt files in dirs
//	i = 0; exp = 0; strcpy (dir, "robot2_locRobot4_20130329");
//	i = 0; exp = 0; strcpy (dir, "robot4_locRobot3_20130329");

	// Check images from gumstix experiments
//	i = 0; exp = 1; strcpy (dir, "20130427_gumstixFiles/gum2/experiment_robot_0_440-19691231-Wed-160720");
	i = 0; exp = 1; strcpy (dir, "20130428_gumstixFiles_1__okButShortExperiment/gum3/experiment_robot_0_255-19691231-Wed-160415");
//	i = 0; exp = 1; strcpy (dir, "20130728_gumstixFiles_2__collabOkButLoopCloseFailed/gum2/experiment_robot_0_194-19691231-Wed-160314");

//	while (1)
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

			if (drawGrid)
			{
				Image_drawGrid (img->data, CAM_IMG_ORIG_X, CAM_IMG_ORIG_Y, CAM_OCCUPANCY_GRID_Y);
			}

			memcpy (iplImage->imageData, img->data, sizeToRead);
			cvNamedWindow ("win", windowSize);
			cvShowImage ("win", iplImage);
			cvWaitKey (0);
		}
		else
		{
//			printf ("%s: load failed\n", buf);
//			break;
		}

		++i;
	}

	Image_dtor (&img);
	cvReleaseImage (&iplImage);
	cvCleanup();
}













#endif // defined(IS_WIN) && defined(SIMULATION)
















