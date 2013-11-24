#include "../../Common/RobotDefs.h"

#ifdef SIMULATION

#include "CooperativeLocalisation.h"
#include "ObstacleDetection.h"
#include "../../Board/OpenCV/CV.h"


void __resetGroupPoses(PoseSimple *groupPoses)
{
	groupPoses[0] = initPoseSimple();
	groupPoses[0].loc.x = 300.0f;
	groupPoses[0].loc.y = 300.0f;
	groupPoses[0].orient = 0.0f;

	groupPoses[1] = initPoseSimple();
	groupPoses[1].loc.x = 200.0f;
	groupPoses[1].loc.y = 200.0f;
	groupPoses[1].orient = 0.0f;

	groupPoses[2] = initPoseSimple();
	groupPoses[2].loc.x = 100.0f;
	groupPoses[2].loc.y = 100.0f;
	groupPoses[2].orient = 0.0f;
}

void __directlySetBlobs (const uchar occupancyGrid[SIZE_COOP_LOC_GRID], uchar colourGrid[SIZE_COOP_LOC_GRID])
{
	memcpy (colourGrid, occupancyGrid, SIZE_COOP_LOC_GRID);
}

void __calcColourNaive (uchar trainingOccupancyGrid[SIZE_COOP_LOC_GRID],
						ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
						uchar colourGrid[SIZE_COOP_LOC_GRID],
						Image *image,
						IplImage *iplImage,
						const CvSize size,
						const int windowSize[4],
						const PointI orig,
						const int gridY)
{
	int i, j, k;
	int bestColourIndex;
	float bestColourScore;
	int coloursPresent[N_ROBOT_COLOURS];
	int colourPresentIndex = 0;
	const float colourScoreThresh = 10.0f;

	for (i = 0; i < N_ROBOT_COLOURS; ++i)
	{
		coloursPresent[i] = -1;
	}

	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		bestColourIndex = -1;
		bestColourScore = colourScoreThresh;
		for (j = 0; j < N_ROBOT_COLOURS; ++j)
		{
			if (colourResponseGrid[i].vals[j] < bestColourScore)
			{
				bestColourIndex = j;
				bestColourScore = colourResponseGrid[i].vals[j];
			}
		}
		if (bestColourIndex == -1)
		{
			colourGrid[i] = COLOUR_BACKGROUND;
		}
		else
		{
			colourGrid[i] = bestColourIndex;

			for (k = 0; k < colourPresentIndex; ++k)
			{
				if (coloursPresent[k] == bestColourIndex)
				{
					break;
				}
			}
			if (k == colourPresentIndex)
			{
				coloursPresent[colourPresentIndex++] = bestColourIndex;
			}
		}
	}

	printf ("red=match blue=not\n");
	printf ("%d colours present\n", colourPresentIndex);

	for (i = 0; i < colourPresentIndex; ++i)
	{
		printf ("colour == %d\n", coloursPresent[i]);

		Image_drawOccupancy (image->data, colourGrid, orig.x, orig.y, gridY, coloursPresent[i]);

		memcpy (iplImage->imageData, image->data, size.width * size.height * 3);
		cvNamedWindow ("robot colours", windowSize);
		cvShowImage ("robot colours", iplImage);
		cvWaitKey (0);
	}
}

//! Similar to earlier version. That calcd best colour for each cell
/*!
This looks at each blob colour in training image, and calculates response for cells that
actually are/aren't that colour
*/
void __calcColourNaive2 (uchar trainingOccupancyGrid[SIZE_COOP_LOC_GRID],
						 const int blobColoursThisImg[5],
						ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
						uchar colourGrid[SIZE_COOP_LOC_GRID],
						Image *image,
						IplImage *iplImage,
						const CvSize size,
						const int windowSize[4],
						const PointI orig,
						const int gridY)
{
	int i, c;
	int colour;
	int cellMatches;
	float val;
	int nMatch, nNotMatch;
	float totalMatch, totalNotMatch;
	float minMatch, maxMatch, minNotMatch, maxNotMatch;

	//for (c = 0; c < 5; ++c)
	for (c = 0; c < N_ROBOT_COLOURS; ++c)
	{
		/*colour = blobColoursThisImg[c];
		if (colour == -1)
		{
			break;
		}*/
		colour = c;

		nMatch = nNotMatch = 0;
		totalMatch = totalNotMatch = 0.0f;
		minMatch = minNotMatch = 10000.0f;
		maxMatch = maxNotMatch = -10000.0f;


		for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
		{
			cellMatches = (trainingOccupancyGrid[i] == colour);
			val = colourResponseGrid[i].vals[colour];

			if (cellMatches)
			{
				++nMatch;
				totalMatch += val;
				minMatch = min (val, minMatch);
				maxMatch = max (val, maxMatch);
			}
			else
			{
				++nNotMatch;
				totalNotMatch += val;
				minNotMatch = min (val, minNotMatch);
				maxNotMatch = max (val, maxNotMatch);
			}
		}

		printf ("\tcolour %d\n", colour);
		printf ("\t    n matches %3d, avgScore %10f, min %10f, max %10f\n", nMatch, totalMatch / nMatch, minMatch, maxMatch);
		printf ("\tn not matches %3d, avgScore %10f, min %10f, max %10f\n", nNotMatch, totalNotMatch / nNotMatch, minNotMatch, maxNotMatch);
	}
}

extern float __getColourBlobScore (
	const int c,
	ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
	const int i,
	const int j);

void __calcColourBlobScores (uchar trainingOccupancyGrid[SIZE_COOP_LOC_GRID],
							 ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
							 const int blobColoursThisImg[5],
							 Image *image,
							 IplImage *iplImage,
							 const CvSize size,
							 const int windowSize[4],
							 const PointI orig,
							 const int gridY)
{
	int c, colour;
	int i, j;
	float score;
	float tempGrid[SIZE_COOP_LOC_GRID];
	//uchar resultsGrid[SIZE_COOP_LOC_GRID];

	int cellMatches;
	float val;
	int nMatch, nNotMatch;
	float totalMatch, totalNotMatch;
	float minMatch, maxMatch, minNotMatch, maxNotMatch;

	for (c = 0; c < 5; ++c)
	{
		colour = blobColoursThisImg[c];
		if (colour == -1)
		{
			break;
		}

		nMatch = nNotMatch = 0;
		totalMatch = totalNotMatch = 0.0f;
		minMatch = minNotMatch = 10000.0f;
		maxMatch = maxNotMatch = -10000.0f;

		for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
		{
			for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
			{
				score = __getColourBlobScore (
					c,
					colourResponseGrid,
					i,
					j);
				tempGrid[OCCUP_COORD(i, j)] = score;
			}
		}

		for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
		{
			for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
			{
				cellMatches = (trainingOccupancyGrid[OCCUP_COORD(i, j)] == colour);
				val = tempGrid[OCCUP_COORD(i, j)];
				if (cellMatches)
				{
					++nMatch;
					totalMatch += val;
					minMatch = min (val, minMatch);
					maxMatch = max (val, maxMatch);
				}
				else
				{
					++nNotMatch;
					totalNotMatch += val;
					minNotMatch = min (val, minNotMatch);
					maxNotMatch = max (val, maxNotMatch);
				}

				//resultsGrid[OCCUP_COORD(i, j)] = tempGrid[OCCUP_COORD(i, j)] < 10.0f;
			}
		}

		printf ("\tcolour %d\n", colour);
		printf ("\t    n matches %3d, avgScore %10f, min %10f, max %10f\n", nMatch, totalMatch / nMatch, minMatch, maxMatch);
		printf ("\tn not matches %3d, avgScore %10f, min %10f, max %10f\n", nNotMatch, totalNotMatch / nNotMatch, minNotMatch, maxNotMatch);

		//printf ("red=match blue=not\n");
		//printf ("colour %d\n", colour);

		//Image_drawOccupancy (image->data, resultsGrid, orig.x, orig.y, gridY, 1);

		//memcpy (iplImage->imageData, image->data, size.width * size.height * 3);
		//cvNamedWindow ("robot colours", windowSize);
		//cvShowImage ("robot colours", iplImage);
		//cvWaitKey (0);
	}
}

void __displayRobotBlobs (
	ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
	List *colourBlobs,
	Image *image,
	IplImage *iplImage,
	const CvSize size,
	const int windowSize[4],
	const PointI orig)
{
	ListNode *iter;
	ColourBlob2 *blob2;
	int i;

	/*!
	\todo may need code to determine if blobs from different colours conflict,
	also, get rid of blobs that are too small. The bb might be useful for det'ing
	conflicts.
	*/

	printf ("%d blobs\n", colourBlobs->size);

	if (colourBlobs->size)
	{
		i = 0;
		iter = colourBlobs->front;
		while (iter)
		{
			blob2 = (ColourBlob2*)iter->value;

			printf ("colour %2d blobIndex %d nCells %d avgScore %f bl (%d,%d) tr (%d,%d)\n",
				blob2->colour,
				blob2->blobIndex,
				blob2->nCells,
				blob2->avgScore,
				blob2->bl.x, blob2->bl.y,
				blob2->tr.x, blob2->tr.y);

			// Refresh image
			memcpy (iplImage->imageData, image->data, size.width * size.height * 3);

			Image_drawBitArray (
				iplImage->imageData,
				blob2->blobGrid->grid,
				CAM_OCCUPANCY_GRID_X,
				CAM_OCCUPANCY_GRID_Y,
				orig.x,
				orig.y);

			cvNamedWindow ("blobs", windowSize);
			cvShowImage ("blobs", iplImage);
			cvWaitKey (0);

			iter = iter->next;
		}
	}
	else
	{
		printf ("no blobs\n");
		memcpy (iplImage->imageData, image->data, size.width * size.height * 3);
		cvNamedWindow ("blobs", windowSize);
		cvShowImage ("blobs", iplImage);
		cvWaitKey (0);
	}
}


extern char rootDirName[128];
extern int robotColours[3][2];
extern ImgCellFeatures g_cellFeatures[SIZE_COOP_LOC_GRID];
extern PointF g_camScoreGrid[SIZE_COOP_LOC_GRID];
extern OccupancyGrid g_camOccupancyGrid;

extern void __getBlobColoursThisImg (int blobColoursThisImg[5], const char occupancyGrid[CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y]);

void CooperativeLocalisation_testBlobDetection()
{
	TrainingData data[300];
	ImgFeatures imgFeatures;
	ImgFeatures imgInvFeatures;
	//int isObstacleInCamOccupancyGrid;
	ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID];
	uchar colourGrid[SIZE_COOP_LOC_GRID];
	uchar resultsGrid[SIZE_COOP_LOC_GRID];
	int nTrainingImages;
	FILE *f;
	PointI orig, flip;
	Image *img, *displayImg, *responseImg;
	CvSize size = {176, 143};
	IplImage *iplImage = cvCreateImage (size, 8, 3);
	int windowSize[] = {100, 10, 450, 450};
	char inputDir[256];
	int i;
	int closestImgGroupIndex;
//	int j;
	List colourBlobs;
//	VisibleRobot closestVisibleRobot;
	int ownIndex = 0;
	Pose ownPose;
//	PoseSimple groupPoses[3];
	int blobColoursThisImg[5] = {-1, -1, -1, -1, -1};
	const int justTestColours = 1;
	const int justTestBlobScores = 0;

	FILE *output = fopen ("__temp__.txt", "w");

	ownPose = initPose();
	ownPose.loc = initPointF(0.0f);
	ownPose.orient = 0.0f;

	TrainingData_init (data, 300);
	nTrainingImages = CooperativeLocalisation_setupTrainingData (data);

	Image_ctor (&img, 176, 143, 3);
	Image_ctor (&displayImg, 176, 143, 3);
	Image_ctor (&responseImg, 176, 143, 1);

	printf ("Displaying %d images\n", nTrainingImages);

	//for (i = 0; i < nTrainingImages; ++i)
	for (i = 0; i < 1; ++i)
	{
		sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);

		__getBlobColoursThisImg (blobColoursThisImg, data[i].occupancyGrid);
		//if (blobColoursThisImg[0] == -1)
		//{
		//	printf ("No robot colour blobs in img %d: %s\n", i, data[i].imgName);
		//	continue;
		//}

		f = fopen (inputDir, "r");
		if (!f)
		{
			printf ("Error: inputDir %s\n", inputDir);
		}
		assert (f);
		fread (img->data, 1, img->width * img->height * 3, f);
		fclose (f);
		Image_rectify (img->data);
		orig.x = data[i].orig >> 8;
		orig.y = data[i].orig & 255;
		flip.x = data[i].flip >> 1;
		flip.y = data[i].flip & 1;
		Image_flip(img->data, flip.x, flip.y);
		Image_clone (img, displayImg);
		Image_toHSV (img->data);

		printf ("%3d %s\n", i, inputDir);

		// gridY is 23 for all coopLoc images
		ObstacleDetection_calcImgCellFeaturesForTraining (img, g_cellFeatures, data[i].occupancyGrid, data[i].gridY, orig.x, orig.y, 2);
		//ObstacleDetection_calcImgCellFeatures (img, g_cellFeatures, CAM_OCCUPANCY_GRID_Y, orig.x, orig.y);

		if (0 && 1 == data[i].skipBlob)
		{
			__directlySetBlobs (data[i].occupancyGrid, colourGrid);
		}
		else
		{
			// Up to gridY for camera
			ObstacleDetection_calcImgFeatures (&imgFeatures, g_cellFeatures, data[i].gridY);
			ObstacleDetection_calcInvImgFeatures (&imgFeatures, &imgInvFeatures);
			ObstacleDetection_calcImgOccupancy (&imgFeatures, &imgInvFeatures, g_cellFeatures, g_camScoreGrid, &closestImgGroupIndex, data[i].gridY);

			fprintf (output, "%s\n", data[i].imgName);

			// Over whole grid
			colourBlobs = initList();

			CooperativeLocalisation_calcColourResponses (
				g_cellFeatures,
				&imgInvFeatures,
				data[i].occupancyGrid,
				colourResponseGrid,
				output);

			if (justTestColours)
			{
				//__calcColourNaive (
				//	data[i].occupancyGrid,
				//	colourResponseGrid,
				//	colourGrid,
				//	displayImg,
				//	iplImage,
				//	size,
				//	windowSize,
				//	orig,
				//	data[i].gridY);

				__calcColourNaive2 (
					data[i].occupancyGrid,
					blobColoursThisImg,
					colourResponseGrid,
					colourGrid,
					displayImg,
					iplImage,
					size,
					windowSize,
					orig,
					data[i].gridY);
			}
			else if (justTestBlobScores) // This needs some work, so see if we need it first
			{
				__calcColourBlobScores (
					data[i].occupancyGrid,
					colourResponseGrid,
					blobColoursThisImg,
					displayImg,
					iplImage,
					size,
					windowSize,
					orig,
					data[i].gridY);
			}
			else
			{
				CooperativeLocalisation_groupRobotBlobs (
					colourResponseGrid,
					&colourBlobs);

				__displayRobotBlobs (
					colourResponseGrid,
					&colourBlobs,
					displayImg,
					iplImage,
					size,
					windowSize,
					orig);

				// Up to gridY for camera
				//CooperativeLocalisation_calcOccupiedEst (
				//	data[i].gridY,
				//		&g_camOccupancyGrid,
				//		g_camScoreGrid,
				//		colourGrid,
				//		&isObstacleInCamOccupancyGrid);
			}

			// For testing, update the pose of the visible robot to match that in the image
			//__resetGroupPoses (groupPoses);
			//groupPoses[data[i].visIndex] = data[i].pose;
			//robotColours[data[i].visIndex][0] = data[i].visColours[0];
			//robotColours[data[i].visIndex][1] = data[i].visColours[1];

			//closestVisibleRobot = initVisibleRobot();
			//CooperativeLocalisation_determineVisibleRobotFromBlobs (&colourBlobs, &closestVisibleRobot, ownIndex);

			//if (closestVisibleRobot.valid)
			//{
			//	CooperativeLocalisation_detectRobotBlobEdges (img, responseImg, orig, colourGrid, &colourBlobs, &closestVisibleRobot, ownPose, groupPoses);
			//}

			List_clearWithDeallocator (&colourBlobs, freeColourBlob2);
			//List_clear(&colourBlobs, 1);
		}

		if (0 && !justTestColours && !justTestBlobScores)
		{
			ObstacleDetection_extractOccupancyGrid (&g_camOccupancyGrid, resultsGrid, CAM_OCCUPANCY_GRID_X, data[i].gridY);
			Image_drawOccupancy2 (displayImg->data, resultsGrid, orig.x, orig.y, data[i].gridY);

			memcpy (iplImage->imageData, displayImg->data, size.width * size.height * 3);
			cvNamedWindow ("robot colours", windowSize);
			cvShowImage ("robot colours", iplImage);
			cvWaitKey (0);
		}
	}

	fclose (output);
	Image_dtor (&img);
	Image_dtor (&displayImg);
	Image_dtor (&responseImg);
	cvReleaseImage (&iplImage);
	cvCleanup();
}





















extern const float robotColourDescs[N_ROBOT_COLOURS][12];

int __countColourInGrid (const int colour, const uchar occupancyGrid[SIZE_COOP_LOC_GRID])
{
	int i, n;
	n = 0;
	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		n += (int)(occupancyGrid[i] == colour);
	}
	return n;
}

float __calcDiffWithNewRep (const ImgCellFeatures *cellFeats, const float colourFeats[12])
{
	float diff;
	diff =
		fabs (cellFeats->vals[0] - colourFeats[0]) / colourFeats[6] +
		fabs (cellFeats->vals[1] - colourFeats[1]) / colourFeats[7] +
		fabs (cellFeats->vals[2] - colourFeats[2]) / colourFeats[8] +
		fabs (cellFeats->vals[3] - colourFeats[3]) / colourFeats[9] +
		fabs (cellFeats->vals[4] - colourFeats[4]) / colourFeats[10] +
		fabs (cellFeats->vals[5] - colourFeats[5]) / colourFeats[11];

	return diff;
}

extern const uchar coloursToLeaveOut[N_ROBOT_COLOURS];

void __calcColourResponses (ImgCellFeatures cellFeatures[SIZE_COOP_LOC_GRID],
						 ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID])
{
	int i, j, c;
	float diff;
	ImgCellFeatures *features;
	ColourResponse *response;

	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			features = &cellFeatures[i + j * CAM_OCCUPANCY_GRID_X];
			response = &colourResponseGrid[i + j * CAM_OCCUPANCY_GRID_X];

			for (c = 0; c < N_ROBOT_COLOURS; ++c)
			{
				if (coloursToLeaveOut[c])
				{
					diff = MAX_FLT;
				}
				else
				{
					diff = __calcDiffWithNewRep (features, robotColourDescs[c]);
				}

				response->vals[c] = diff;
			}
		}
	}
}


int __isColourOk (const uchar colour)
{
	return (colour != 20 && colour != 19 && colour != 18); // Background
}

void __printColourResponses (ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID],
							 const char *imgName,
							 FILE *f)
{
	int c, i, j;
	fprintf (f, "%s\n", imgName);

	for (c = 0; c < N_ROBOT_COLOURS; ++c)
	{
		fprintf (f, "colour %d\n", c);
		for (j = CAM_OCCUPANCY_GRID_Y - 1; j >= 0; --j)
		{
			for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
			{
				fprintf (f, "%3d,", (int)colourResponseGrid[OCCUP_COORD(i, j)].vals[c]);
			}
			fprintf (f, "\n");
		}
		fprintf (f, "\n\n\n");
	}
}

void temp__blobDet__useStuffFromPrevFuncs()
{
	TrainingData data[300];
//	ImgFeatures imgInvFeatures;
	ColourResponse colourResponseGrid[SIZE_COOP_LOC_GRID];
	int nTrainingImages;
	FILE *f;
	PointI orig, flip;
	Image *img, *displayImg, *responseImg;
	CvSize size = {176, 143};
	IplImage *iplImage = cvCreateImage (size, 8, 3);
	int windowSize[] = {100, 10, 450, 450};
	char inputDir[256];
	int i;
	int ownIndex = 0;
	Pose ownPose;
	int blobColoursThisImg[6] = {-1, -1, -1, -1, -1, -1};
	const int justTestColours = 1;
	const int justTestBlobScores = 0;
	List colourBlobs;

	FILE *output = fopen ("__temp__.txt", "w");

	ownPose = initPose();
	ownPose.loc = initPointF(0.0f);
	ownPose.orient = 0.0f;

	TrainingData_init (data, 300);
	nTrainingImages = CooperativeLocalisation_setupTrainingData (data);

	Image_ctor (&img, 176, 143, 3);
	Image_ctor (&displayImg, 176, 143, 3);
	Image_ctor (&responseImg, 176, 143, 1);

	colourBlobs = initList();

	printf ("Displaying %d images\n", nTrainingImages);

	for (i = 0; i < nTrainingImages; ++i)
//	for (i = 15; i < 16; ++i)
	{
		sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);

		f = fopen (inputDir, "r");
		if (!f)
		{
			printf ("Error: inputDir %s\n", inputDir);
		}
		assert (f);
		fread (img->data, 1, img->width * img->height * 3, f);
		fclose (f);
		Image_rectify (img->data);
		orig.x = data[i].orig >> 8;
		orig.y = data[i].orig & 255;
		flip.x = data[i].flip >> 1;
		flip.y = data[i].flip & 1;
		Image_flip(img->data, flip.x, flip.y);
		Image_clone (img, displayImg);
		Image_toHSV (img->data);

		printf ("%3d %s\n", i, inputDir);
		__getBlobColoursThisImg (blobColoursThisImg, data[i].occupancyGrid);

		ObstacleDetection_calcImgCellFeatures (
			img,
			g_cellFeatures,
			data[i].gridY,
			orig.x,
			orig.y);

		__calcColourResponses (g_cellFeatures, colourResponseGrid);
//		__printColourResponses (colourResponseGrid, inputDir, output);

		CooperativeLocalisation_groupRobotBlobs (colourResponseGrid, &colourBlobs);

		__displayRobotBlobs (colourResponseGrid, &colourBlobs, displayImg, iplImage, size, windowSize, orig);

		// find colours w any seed pts

		// iter through these, get blobs

		// check for conflicts, purge, return/present list of blobs

		List_clearWithDeallocator (&colourBlobs, freeColourBlob2);
	}

	List_clearWithDeallocator (&colourBlobs, freeColourBlob2);

	fclose (output);

	Image_dtor (&img);
	Image_dtor (&displayImg);
	Image_dtor (&responseImg);
	cvReleaseImage (&iplImage);
	cvCleanup();
}

void temp__testColourRecPerImg()
{
	TrainingData data[300];
	FILE *f;
	PointI orig, flip;
	int i, nTrainingImages;
	Image *img, *displayImg;
	char inputDir[256];
	int blobColoursThisImg[5] = {-1, -1, -1, -1, -1};
	int cell, colour;
	float diff;
	int nColourCells, nColourCellsCorrect;
	float avgCorrectColourDiff, avgIncorrectColourDiff, avgOtherColourDiff;
	float avgBgColourDiff;
	float colourDiff, minOtherColourDiff;
	FILE *output;

	output = fopen ("__colourRecPerImg__.txt", "w");

	Image_ctor (&img, 176, 143, 3);
	Image_ctor (&displayImg, 176, 143, 3);

	TrainingData_init (data, 300);
	nTrainingImages = CooperativeLocalisation_setupTrainingData (data);

	{
		for (i = 0; i < nTrainingImages; ++i)
		{
			nColourCells = 0;
			nColourCellsCorrect = 0;
			avgCorrectColourDiff = 0.0f;
			avgIncorrectColourDiff = 0.0f;
			avgOtherColourDiff = 0.0f;
			avgBgColourDiff = 0.0f;

			sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);

			fprintf (output, "%3d %s\n", i, inputDir);

			f = fopen (inputDir, "r");
			if (!f)
			{
				printf ("Error: inputDir %s\n", inputDir);
			}
			assert (f);
			fread (img->data, 1, img->width * img->height * 3, f);
			fclose (f);
			Image_rectify (img->data);
			orig.x = data[i].orig >> 8;
			orig.y = data[i].orig & 255;
			flip.x = data[i].flip >> 1;
			flip.y = data[i].flip & 1;
			Image_flip(img->data, flip.x, flip.y);
			Image_clone (img, displayImg);
			Image_toHSV (img->data);

			ObstacleDetection_calcImgCellFeatures (
				img,
				g_cellFeatures,
				data[i].gridY,
				orig.x,
				orig.y);

			for (cell = 0; cell < SIZE_COOP_LOC_GRID; ++cell)
			{
				colourDiff = 10000.0f;
				minOtherColourDiff = 10000.0f;

				for (colour = 0; colour < N_ROBOT_COLOURS; ++colour)
				{
					if (coloursToLeaveOut[colour])
						continue;
					if (!__isColourOk (colour))
					{
						continue;
					}
					diff = __calcDiffWithNewRep (&g_cellFeatures[cell], robotColourDescs[colour]);

					if (colour == data[i].occupancyGrid[cell])
					{
						++nColourCells;
						colourDiff = diff;
					}
					else
					{
						minOtherColourDiff = min (minOtherColourDiff, diff);
					}
				}

				if (colourDiff != 10000.0f)
				{
					if (colourDiff < minOtherColourDiff)
					{
						++nColourCellsCorrect;
						avgCorrectColourDiff += colourDiff;
					}
					else
					{
						avgIncorrectColourDiff += colourDiff;
						avgOtherColourDiff += minOtherColourDiff;
					}
				}
				else
				{
					avgBgColourDiff += minOtherColourDiff;
				}
			}

			if (nColourCells)
			{
				fprintf (output, "\tnColourCells %3d nCorrect %3d (%12f)\n", nColourCells, nColourCellsCorrect, (float)nColourCellsCorrect / (float)nColourCells);
				fprintf (output, "\tavgCorrect %12f avgIncorrect %12f avgOther %12f\n",
					avgCorrectColourDiff / nColourCellsCorrect,
					avgIncorrectColourDiff / (nColourCells - nColourCellsCorrect),
					avgOtherColourDiff / (nColourCells - nColourCellsCorrect));
				fprintf (output, "\tavgBg %12f\n", avgBgColourDiff / SIZE_COOP_LOC_GRID);
			}
			else
			{
				fprintf (output, "no colour cells\n");
			}
		}
	}

	fclose (output);

	Image_dtor (&img);
	Image_dtor (&displayImg);
}

void temp__testNewColourRep()
{
	TrainingData data[300];
	FILE *f;
	PointI orig, flip;
	int i, colour, nTrainingImages;
	int nThisImg;
	Image *img, *displayImg;
	char inputDir[256];
	int blobColoursThisImg[5] = {-1, -1, -1, -1, -1};
	int cell;
	float diffThisColour, minDiffOtherColours;
	int nCells, nCorrect, nIncorrect;
	float diff;
	int compColour, otherColour;
	float avgDiffCorrect, avgDiffIncorrect, avgOtherDiffIncorrect;
	FILE *output;


	Image_ctor (&img, 176, 143, 3);
	Image_ctor (&displayImg, 176, 143, 3);

	TrainingData_init (data, 300);
	nTrainingImages = CooperativeLocalisation_setupTrainingData (data);

	//for (colour = 0; colour < N_ROBOT_COLOURS; ++colour)
	//{
	//	for (i = 0 ; i < 6; ++i)
	//	{
	//		printf ("%f,", robotColourDescs[colour][i]);
	//	}
	//	printf ("\n");
	//}

	output = fopen ("__compColours__.txt", "w");


	for (colour = 0; colour < N_ROBOT_COLOURS; ++colour)
//	for (colour = 0; colour < 4; ++colour)
	{
		if (coloursToLeaveOut[colour])
			continue;

		if (!__isColourOk (colour))
		{
			continue;
		}
		printf ("colour %d\n", colour);
		fprintf (output, "colour %d\n", colour);

		nCells = 0;
		nCorrect = 0;
		nIncorrect = 0;
		avgDiffCorrect = 0.0f;
		avgDiffIncorrect = 0.0f;
		avgOtherDiffIncorrect = 0.0f;

		for (i = 0; i < nTrainingImages; ++i)
//		for (i = 0; i < 10; ++i)
		{
			sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);

			nThisImg = __countColourInGrid (colour, data[i].occupancyGrid);
			//nImgs += (nThisImg != 0);
			//n += nThisImg;

			if (nThisImg)
			{
				printf ("img %d\n", i);
				fprintf (output, "img %d\n", i);

				f = fopen (inputDir, "r");
				if (!f)
				{
					printf ("Error: inputDir %s\n", inputDir);
				}
				assert (f);
				fread (img->data, 1, img->width * img->height * 3, f);
				fclose (f);
				Image_rectify (img->data);
				orig.x = data[i].orig >> 8;
				orig.y = data[i].orig & 255;
				flip.x = data[i].flip >> 1;
				flip.y = data[i].flip & 1;
				Image_flip(img->data, flip.x, flip.y);
				Image_clone (img, displayImg);
				Image_toHSV (img->data);

				ObstacleDetection_calcImgCellFeaturesForTraining (img, g_cellFeatures, data[i].occupancyGrid, data[i].gridY, orig.x, orig.y, 2);


				for (cell = 0; cell < SIZE_COOP_LOC_GRID; ++cell)
				{
					//if (__isColourOk (g_cellFeatures[cell].isOccupied))
					if (g_cellFeatures[cell].isOccupied == colour)
					{
						otherColour = -1;
						minDiffOtherColours = 10000.0f;

						for (compColour = 0; compColour < N_ROBOT_COLOURS; ++compColour)
						{
							if (coloursToLeaveOut[compColour])
								continue;
							if (!__isColourOk (compColour))
								continue;

							diff = __calcDiffWithNewRep (&g_cellFeatures[cell], robotColourDescs[compColour]);

							if (g_cellFeatures[cell].isOccupied == compColour)
							{
								diffThisColour = diff;
							}
							else if (diff < minDiffOtherColours)
							{
								minDiffOtherColours = diff;
								otherColour = compColour;
							}
						}

						++nCells;
						if (diffThisColour < minDiffOtherColours)
						{
							++nCorrect;
							avgDiffCorrect += diffThisColour;
						}
						else
						{
							++nIncorrect;
							avgDiffIncorrect += diffThisColour;
							avgOtherDiffIncorrect += minDiffOtherColours;
							printf ("own diff %12f, other colour (%2d) %12f (img %d)\n", diffThisColour, otherColour, minDiffOtherColours, i);
							fprintf (output, "own diff %12f, other colour (%2d) %12f (img %d)\n", diffThisColour, otherColour, minDiffOtherColours, i);
						}
					}
				}

				printf ("nCells %4d nCorrect %4d nIncorrect %4d (%f%% correct)\n", nCells, nCorrect, nIncorrect, (float)nCorrect / (float)nCells);
				printf ("avgCorrect %12f avgIncorrect %12f avgOtherColour %12f\n\n\n", avgDiffCorrect/nCorrect, avgDiffIncorrect/nIncorrect, avgOtherDiffIncorrect/nIncorrect);

				fprintf (output, "###nCells %4d nCorrect %4d nIncorrect %4d (%f%% correct)\n", nCells, nCorrect, nIncorrect, (float)nCorrect / (float)nCells);
				fprintf (output, "avgCorrect %12f avgIncorrect %12f avgOtherColour %12f\n\n\n", avgDiffCorrect/nCorrect, avgDiffIncorrect/nIncorrect, avgOtherDiffIncorrect/nIncorrect);
			}
		}
	}


	fclose (output);


	Image_dtor (&img);
	Image_dtor (&displayImg);
}

void __recordFeatFromGrid (
	float vals[3][8000],
	int *index,
//	const int feat,
	const int colour,
	const ImgCellFeatures cellFeatures[SIZE_COOP_LOC_GRID],
	const uchar occupancyGrid[SIZE_COOP_LOC_GRID])
{
	int i, ind;
	ind = *index;
	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		if (occupancyGrid[i] == colour)
		{
			vals[0][ind++] = cellFeatures[i].vals[0];
			vals[1][ind++] = cellFeatures[i].vals[1];
			vals[2][ind++] = cellFeatures[i].vals[2];
		}
	}
	*index = ind;
}

void temp__printOutColourFeats()
{
	TrainingData data[300];
	FILE *f;
	PointI orig, flip;
	int i, feat, colour, nTrainingImages;
	int nCells, nThisImg, nImgs;
	int cellIndex;
	float vals[3][8000];
	float featAvg;
//	float featMin, featMax;
	float featStdDev;
	float featsThisColour[6];
	Image *img, *displayImg;
	char inputDir[256];
	int blobColoursThisImg[5] = {-1, -1, -1, -1, -1};
	FILE *output;

	output = fopen ("__coopLocParamsToCopyToCode__.txt", "w");


	Image_ctor (&img, 176, 143, 3);
	Image_ctor (&displayImg, 176, 143, 3);

	TrainingData_init (data, 300);
	nTrainingImages = CooperativeLocalisation_setupTrainingData (data);

	fprintf (output, "const float robotColourDescs[N_ROBOT_COLOURS][6] = {\n");

//	for (feat = 0; feat < 3; ++feat)
	for (colour = 0; colour < N_ROBOT_COLOURS; ++colour)
	{
//		for (colour = 0; colour < N_ROBOT_COLOURS; ++colour)
//		for (feat = 0; feat < 3; ++feat)
		{
//			printf ("FEATURE %d\n", feat);
//			printf ("COLOUR %2d\n", colour);

			nCells = 0;
			nImgs = 0;

			cellIndex = 0;
			for (i = 0; i < 8000; ++i)
			{
				for (feat = 0; feat < 3; ++feat)
				{
					vals[feat][i] = 0.0f;
				}
			}

			for (i = 0; i < nTrainingImages; ++i)
			{

				sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);

				nThisImg = __countColourInGrid (colour, data[i].occupancyGrid);
				nImgs += (nThisImg != 0);
				nCells += nThisImg;

				if (nThisImg)
				{
					f = fopen (inputDir, "r");
					if (!f)
					{
						printf ("Error: inputDir %s\n", inputDir);
					}
					assert (f);
					fread (img->data, 1, img->width * img->height * 3, f);
					fclose (f);
					Image_rectify (img->data);
					orig.x = data[i].orig >> 8;
					orig.y = data[i].orig & 255;
					flip.x = data[i].flip >> 1;
					flip.y = data[i].flip & 1;
					Image_flip(img->data, flip.x, flip.y);
					Image_clone (img, displayImg);
					Image_toHSV (img->data);

					ObstacleDetection_calcImgCellFeaturesForTraining (img, g_cellFeatures, data[i].occupancyGrid, data[i].gridY, orig.x, orig.y, 2);

					__recordFeatFromGrid (vals, &cellIndex, /*feat, */colour, g_cellFeatures, data[i].occupancyGrid);


				}

			}

//			if (feat == 0)
			{
				printf ("colour %d (%d,%d) (nCells,nImgs)\n", colour, nCells, nImgs);
			}

			for (feat = 0; feat < 3; ++feat)
			{
				featAvg = 0.0f;
//				featMin = 10000.0f;
//				featMax = -10000.0f;
				for (i = 0; i < cellIndex; ++i)
				{
					featAvg += vals[feat][i];
//					featMin = min (featMin, vals[i]);
//					featMax = max (featMax, vals[i]);
				}
				featAvg /= nCells;
				featStdDev = 0.0f;
				for (i = 0; i < cellIndex; ++i)
				{
					featStdDev += fabs (featAvg - vals[feat][i]);
				}
				featStdDev /= nCells;
				featsThisColour[feat] = featAvg;
				featsThisColour[feat + 3] = featStdDev;
//				printf ("%d avg %12.2f min %12.2f max %12.2f dev %12.2f\n",
//					feat, featAvg, featMin, featMax, featStdDev);
			}
		}

		fprintf (output, "#ifndef OWN_COLOUR_%d\n", colour);
		fprintf (output, "{ // From %d images, %d cells\n", nImgs, nCells);
		fprintf (output, "%12ff,%12ff,%12ff,\n", featsThisColour[0], featsThisColour[1], featsThisColour[2]);
		fprintf (output, "%12ff,%12ff,%12ff,\n", featsThisColour[3], featsThisColour[4], featsThisColour[5]);
		fprintf (output, "},\n#endif\n");

	}


	fprintf (output, "};");


	fclose (output);


	Image_dtor (&img);
	Image_dtor (&displayImg);
}

void temp__testColoursAgainstBackground()
{
	TrainingData data[100];
	int nTrainingImages;
	int i, colour, cell;
	FILE *f;
	char path[256];
	Image *img;
	PointI orig, flip;
	float diff;
	int diffBins[30]; // store diffs in 0-20, and then steps of 10 after that
	int diffInt, binIndex;
	int totalCells;
	FILE *output;

	Image_ctor (&img, 176, 143, 3);

	TrainingData_init (data, 100);
	nTrainingImages = ObstacleDetection_setupTrainingData (data);

	memset (diffBins, 0, sizeof (int) * 30);

	output = fopen ("__compColoursToObsts__.txt", "w");

	printf ("Testing colours against %d environment images\n", nTrainingImages);

	for (colour = 0; colour < N_ROBOT_COLOURS; ++colour)
	{
		if (coloursToLeaveOut[colour])
			continue;

		//if (!__isColourOk (colour)) // don't bother with navy, etc. as they're crap
		{
			//continue;
		}

		totalCells = 0;
		memset (diffBins, 0, sizeof (int) * 30);

		printf ("colour %d\n", colour);
		fprintf (output, "colour %d\n", colour);


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

			Image_toHSV (img->data);

			ObstacleDetection_calcImgCellFeatures (img, g_cellFeatures, CAM_OCCUPANCY_GRID_Y, orig.x, orig.y);
			//ObstacleDetection_calcImgCellFeaturesForTraining (img, g_cellFeatures, data[i].occupancyGrid, data[i].gridY, orig.x, orig.y, 2);

			for (cell = 0; cell < SIZE_COOP_LOC_GRID; ++cell)
			{
				diff = __calcDiffWithNewRep (&g_cellFeatures[cell], robotColourDescs[colour]);

				diffInt = (int)diff;
				if (diffInt < 20) // count 0..19 individually
				{
					++diffBins[diffInt];
				}
				else // count(20..29),(30..39) in 20..29
				{
					diffInt = min (100, diffInt);
					binIndex = 19 + ((diffInt - 20) / 10);
					++diffBins[binIndex];
				}
				++totalCells;
			}

		}

		for (i = 0; i < 10; ++i)
		{
			printf ("%3d,", i);
			fprintf (output, "%3d,", i);
		}
		printf ("\n");
		fprintf (output, "\n");
		for (i = 0; i < 10; ++i)
		{
			printf ("%3d,", diffBins[i]);
			fprintf (output, "%3d,", diffBins[i]);
		}
		printf ("\n\n");
		fprintf (output, "\n\n");
		for (i = 10; i < 20; ++i)
		{
			printf ("%3d,", i);
			fprintf (output, "%3d,", i);
		}
		printf ("\n");
		fprintf (output, "\n");
		for (i = 10; i < 20; ++i)
		{
			printf ("%3d,", diffBins[i]);
			fprintf (output, "%3d,", diffBins[i]);
		}
		printf ("\n\n");
		fprintf (output, "\n\n");
		for (i = 20; i < 30; ++i)
		{
			printf ("%3d,", i);
			fprintf (output, "%3d,", i);
		}
		printf ("\n");
		fprintf (output, "\n");
		for (i = 20; i < 30; ++i)
		{
			printf ("%3d,", diffBins[i]);
			fprintf (output, "%3d,", diffBins[i]);
		}
		printf ("\n\n");
		fprintf (output, "\n\n");
		if (i == 0)
		{
			printf ("total %d\n\n", totalCells);
			fprintf (output, "total %d\n\n", totalCells);
		}
	}


	fclose (output);

	Image_dtor (&img);
}



#endif // SIMULATION

