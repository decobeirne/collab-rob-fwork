#include "ObstacleRecognitionTraining.h"
#include "ObstacleRecognitionCore.h"

#if defined(IS_WIN) && defined(SIMULATION)

extern char rootDirName[128];

void ObstacleRecognitionTraining_calcAndWriteFeatures (
	const TrainingData *data,
	const int nTrainingImages,
	const char *trainingDataDir)
{
	ImgFeatures imgFeatures;
	ImgFeatures invImgFeatures;
	ImgCellFeatures *cellFeatures;
	Image *img;
	int i;
	char inputDir[256];
	char outputFile[256];
	int j;
	FILE *f;

	cellFeatures = (ImgCellFeatures*)malloc (sizeof (ImgCellFeatures) * SIZE_COOP_LOC_GRID);
	Image_ctor (&img, 176, 143, 3);

	printf ("Writing data for %d images\n", nTrainingImages);
	for (i = 0; i < nTrainingImages; ++i)
	{
		sprintf (inputDir, "%s/calibration/images/%s", rootDirName, data[i].imgName);
		f = fopen (inputDir, "r");
		assert (f);
		fread (img->data, 1, img->width * img->height * 3, f);
		fclose (f);
		Image_rectify (img->data);
		Image_flip(img->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);
		Image_toHSV (img->data);

		ObstacleRecognitionCore_calcImgCellFeatures (img, cellFeatures, CAM_OCCUPANCY_GRID_Y, CAM_IMG_ORIG_X, CAM_IMG_ORIG_Y);
		ObstacleRecognitionCore_calcImgFeatures (&imgFeatures, cellFeatures, CAM_OCCUPANCY_GRID_Y);
		ObstacleRecognitionCore_calcInvImgFeatures (&imgFeatures, &invImgFeatures);

		sprintf (outputFile, "%s/calibration/%s/imgFeatures%03d.txt", rootDirName, trainingDataDir, i);
		f = fopen (outputFile, "w");
		assert (f);
		fprintf (f, "%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n",
			imgFeatures.vals[0], imgFeatures.vals[1], imgFeatures.vals[2], imgFeatures.vals[3], imgFeatures.vals[4], imgFeatures.vals[5],
			imgFeatures.vals[6], imgFeatures.vals[7], imgFeatures.vals[8], imgFeatures.vals[9], imgFeatures.vals[10], imgFeatures.vals[11]);
		fclose(f);

		sprintf (outputFile, "%s/calibration/%s/imgInvFeatures%03d.txt", rootDirName, trainingDataDir, i);
		f = fopen (outputFile, "w");
		assert (f);
		fprintf (f, "%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n",
			invImgFeatures.vals[0], invImgFeatures.vals[1], invImgFeatures.vals[2], invImgFeatures.vals[3], invImgFeatures.vals[4], invImgFeatures.vals[5],
			invImgFeatures.vals[6], invImgFeatures.vals[7], invImgFeatures.vals[8], invImgFeatures.vals[9], invImgFeatures.vals[10], invImgFeatures.vals[11]);
		fclose(f);

		sprintf (outputFile, "%s/calibration/%s/cellFeatures%03d.txt", rootDirName, trainingDataDir, i);
		f = fopen (outputFile, "w");
		assert (f);
		fprintf (f, "%d\n", CAM_OCCUPANCY_GRID_Y);
		for (j = 0; j < CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y; ++j)
		{
			fprintf (f, "%d\n%f\n%f\n%f\n%f\n%f\n%f\n",
				data[i].occupancyGrid[j],
				cellFeatures[j].vals[0], cellFeatures[j].vals[1], cellFeatures[j].vals[2], cellFeatures[j].vals[3], cellFeatures[j].vals[4], cellFeatures[j].vals[5]);
		}
		fclose (f);
	}

	free (cellFeatures);
	Image_dtor (&img);
}

// Define which sets of images to use when generating environment appearance model.
	#define USE_OBST_DET_IMAGES_FROM_0527 // Some obstacle images amongst these
	#define USE_OBST_DET_IMAGES_FROM_EXP_FROM_0527 // Experiment
	#define USE_OBST_DET_IMAGES_FROM_EXP_FROM_0703 // Experiment
	#define USE_OBST_DET_IMAGES_FROM_EXP_FROM_0708 // Imgs of envir
	#define USE_OBST_DET_IMAGES_FROM_EXP_FROM_0715 // Imgs of envir
	#define USE_OBST_DET_IMAGES_FROM_FROM_0201 // Imgs of envir


// Include occupancy grids corresponding to training images.
#include "Training/RobotRecognitionGrids2012_05_27.c"
#include "Training/ObstacleRecognitionGrids2012_05_27.c"
#include "Training/ObstacleRecognitionGrids2012_07_03.c"
#include "Training/ObstacleRecognitionGrids2012_07_08.c"
#include "Training/ObstacleRecognitionGrids2012_07_15.c"
#include "Training/ObstacleRecognitionGrids2013_02_01.c"

int ObstacleRecognitionTraining_setupTrainingData (TrainingData *data)
{
	int nTrainingImages = 0;
	int i = -1;

	// Include code to populate TrainingData array.
	// Verified on 2013/03/02
	#include "Training/RobotRecognitionData2012_05_27.c"
	#include "Training/ObstacleRecognitionData2012_05_27.c"
	#include "Training/ObstacleRecognitionData2012_07_03.c"
	#include "Training/ObstacleRecognitionData2012_07_08.c"
	#include "Training/ObstacleRecognitionData2012_07_15.c"
	#include "Training/ObstacleRecognitionData2013_02_01.c"

	nTrainingImages = i + 1;
	return nTrainingImages;
}

void ObstacleRecognitionTraining_writeFeaturesForTraining()
{
	TrainingData *data;
	int nTrainingImages;
	const char *trainingDataDir = "ObstacleRecognition";

	data = (TrainingData*)malloc (sizeof (TrainingData) * 300);

	nTrainingImages = ObstacleRecognitionTraining_setupTrainingData (data);
	ObstacleRecognitionTraining_calcAndWriteFeatures (data, nTrainingImages, trainingDataDir);

	free (data);
}

#endif // defined(IS_WIN) && defined(SIMULATION)



