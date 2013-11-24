#include "RobotRecognitionTraining.h"
#include "RobotRecognitionCore.h"
#include "ObstacleRecognitionTraining.h"
#include "RobotRecognitionModel.h"
#include "ObstacleRecognition.h"
#include "ObstacleRecognitionCore.h"

#if defined(IS_WIN) && defined(SIMULATION)

/*
Note on robot recognition images:

Older sets of images are in archived versions of CollabExp.

Images of robot2 (taken from robot3) start at 700mm and move in jumps of
50mm. The images end at 250mm from front-on orientation.
*/








// Define which sets of images to use when generating robot appearance model.
//	#define USE_ROB2_IMAGES_FROM_0527
//	#define USE_ROB2_IMAGES_FROM_0715
//	#define USE_ROB2_IMAGES_FROM_0201 // From this up are just images of cards or don't have loc set, so can comment out when testing coop loc
	#define USE_ROB3_IMAGES_FROM_0209
	#define USE_ROB2_IMAGES_FROM_0329 // <-- RE ENABLE THESE
	#define USE_ROB4_IMAGES_FROM_0329
	#define USE_ROB2_IMAGES_FROM_0421 // These 2 have new purple colours 07 and 08
	#define USE_ROB3_IMAGES_FROM_0421


// Include occupancy grids corresponding to training images.
#include "Training/RobotRecognitionGrids2012_05_27.c"
#include "Training/RobotRecognitionGrids2012_07_15.c"
#include "Training/RobotRecognitionGrids2013_02_01.c"
#include "Training/RobotRecognitionGrids2013_02_09.c"
#include "Training/RobotRecognitionGrids2013_03_29_1.c"
#include "Training/RobotRecognitionGrids2013_03_29_2.c"
#include "Training/RobotRecognitionGrids2013_04_21.c"

int RobotRecognitionTraining_setupTrainingData(TrainingData *data)
{
	int nTrainingImages = 0;
	int i = -1;

	// Include code to populate TrainingData array.
	#include "Training/RobotRecognitionData2012_05_27.c"
	#include "Training/RobotRecognitionData2012_07_15.c"
	#include "Training/RobotRecognitionData2013_02_01.c"
	#include "Training/RobotRecognitionData2013_02_09.c"
	#include "Training/RobotRecognitionData2013_03_29_1.c"
	#include "Training/RobotRecognitionData2013_03_29_2.c"
	#include "Training/RobotRecognitionData2013_04_21.c"

	nTrainingImages = i + 1;
	return nTrainingImages;
}

int RobotRecognitionTraining_countColourInGrid (
	const int colour,
	const uchar occupancyGrid[SIZE_COOP_LOC_GRID])
{
	int i, n;
	n = 0;
	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		n += (int)(occupancyGrid[i] == colour);
	}
	return n;
}

void RobotRecognitionTraining_recordFeatFromGrid (
	float vals[3][8000],
	int *index,
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

extern char rootDirName[128];
//extern ImgCellFeatures *g_cellFeatures;
//extern ImgFeatures g_imgFeatures;

void RobotRecognitionTraining_calcAndWriteFeaturesNEW (
	const TrainingData *data,
	const int nTrainingImages,
	const char *trainingDataDir)
{
	ImgCellFeatures *cellFeatures;
	ImgFeatures imgFeatures;
	int i, colour, cell;
	int nThisImg;
	Image *img;
	char inputFileName[256];
	FILE *inputFile;
	char outputFileName[256];
	FILE *outputFile;

	cellFeatures = (ImgCellFeatures*)malloc (sizeof (ImgCellFeatures) * SIZE_COOP_LOC_GRID);
	Image_ctor (&img, 176, 143, 3);

	sprintf (outputFileName, "%s/calibration/%s/coloursToUse.txt", rootDirName, trainingDataDir);
	outputFile = fopen (outputFileName, "w");
	assert (outputFile);
	for (colour = 0; colour < N_ROBOT_COLOURS_TOTAL; ++colour)
	{
		if (!RobotRecognitionModel_leaveOutColourInTraining (colour))
		{
			fprintf (outputFile, "%d\n", colour);
		}
	}
	fclose (outputFile);

	for (colour = 0; colour < N_ROBOT_COLOURS_TOTAL; ++colour)
	{
		sprintf (outputFileName, "%s/calibration/%s/colourFeatures%03d.txt", rootDirName, trainingDataDir, colour);
		outputFile = fopen (outputFileName, "w");
		assert (outputFile);

		for (i = 0; i < nTrainingImages; ++i)
		{
			nThisImg = RobotRecognitionTraining_countColourInGrid (colour, data[i].occupancyGrid);
			if (nThisImg)
			{
				sprintf (inputFileName, "%s/calibration/images/%s", rootDirName, data[i].imgName);
				inputFile = fopen (inputFileName, "r");
				assert (inputFile);
				fread (img->data, 1, img->width * img->height * 3, inputFile);
				fclose (inputFile);

				Image_rectify (img->data);
				Image_flip(img->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);
				Image_toHSV (img->data);

				setupCamParams (2);

				ObstacleRecognition_calcImageFeatures (img, cellFeatures, &imgFeatures);
//				ObstacleRecognition_calcImageFeatures (img, g_cellFeatures, &g_imgFeatures);
//				ObstacleRecognitionCore_calcImgCellFeatures (img, cellFeatures, CAM_OCCUPANCY_GRID_Y, CAM_IMG_ORIG_X, CAM_IMG_ORIG_Y);
//				ObstacleRecognitionCore_calcImgCellFeatures (img, g_cellFeatures, CAM_OCCUPANCY_GRID_Y, CAM_IMG_ORIG_X, CAM_IMG_ORIG_Y);

				for (cell = 0; cell < SIZE_COOP_LOC_GRID; ++cell)
				{
					if (data[i].occupancyGrid[cell] == colour)
					{
						fprintf (
							outputFile,
							"%d\n%d\n%f\n%f\n%f\n%f\n%f\n%f\n",
							i,
							cell,
							cellFeatures[cell].vals[0],
							cellFeatures[cell].vals[1],
							cellFeatures[cell].vals[2],
							cellFeatures[cell].vals[3],
							cellFeatures[cell].vals[4],
							cellFeatures[cell].vals[5]);
/*							g_cellFeatures[cell].vals[0],
							g_cellFeatures[cell].vals[1],
							g_cellFeatures[cell].vals[2],
							g_cellFeatures[cell].vals[3],
							g_cellFeatures[cell].vals[4],
							g_cellFeatures[cell].vals[5]);*/
					}
				}
			}
		}

		fclose (outputFile);
	}

	free (cellFeatures);
	Image_dtor (&img);
}

void RobotRecognitionTraining_writeFeaturesForTrainingNEW2()
{
	TrainingData *data;
	int nTrainingImages;
	const char *trainingDataDir = "RobotRecognition";

	data = (TrainingData*)malloc (sizeof (TrainingData) * 500);

	TrainingData_init (data, 500);
	nTrainingImages = RobotRecognitionTraining_setupTrainingData (data);
	RobotRecognitionTraining_calcAndWriteFeaturesNEW (data, nTrainingImages, trainingDataDir);

	free (data);
}

#endif // defined(IS_WIN) && defined(SIMULATION)



