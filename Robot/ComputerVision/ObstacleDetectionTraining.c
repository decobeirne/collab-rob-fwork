#include "../../Common/RobotDefs.h"

#ifdef SIMULATION

#include "ObstacleDetection.h"


extern char rootDirName[128];
extern ImgCellFeatures g_cellFeatures[SIZE_COOP_LOC_GRID];

void ObstacleDetection_writeImageFeatures(const TrainingData *data,
										  const int nTrainingImages,
										  const char *trainingDataDir)
{
	ImgFeatures imgFeatures;
	ImgFeatures invImgFeatures;
	Image *img;
	int i;
	char inputDir[256];
	char outputFile[256];
	int j;
	FILE *f;
	PointI orig, flip;

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
		orig.x = data[i].orig >> 8;
		orig.y = data[i].orig & 255;
		flip.x = data[i].flip >> 1;
		flip.y = data[i].flip & 1;
		Image_flip(img->data, flip.x, flip.y);
		Image_toHSV (img->data);

		ObstacleDetection_calcImgCellFeaturesForTraining (img, g_cellFeatures, data[i].occupancyGrid, data[i].gridY, orig.x, orig.y, 2);
		ObstacleDetection_calcImgFeatures (&imgFeatures, g_cellFeatures, data[i].gridY);
		ObstacleDetection_calcInvImgFeatures (&imgFeatures, &invImgFeatures);

#if 1
		sprintf (outputFile, "%s/calibration/%s/imgFeatures%03d.txt", rootDirName, trainingDataDir, i);
		f = fopen (outputFile, "w");
		assert (f);
		fprintf (f, "%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n%f\n",
			imgFeatures.vals[0], imgFeatures.vals[1], imgFeatures.vals[2], imgFeatures.vals[3], imgFeatures.vals[4], imgFeatures.vals[5],
			imgFeatures.vals[6], imgFeatures.vals[7], imgFeatures.vals[8], imgFeatures.vals[9], imgFeatures.vals[10], imgFeatures.vals[11]);
		fclose(f);
#endif

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
		fprintf (f, "%d\n", data[i].gridY);
		for (j = 0; j < CAM_OCCUPANCY_GRID_X * data[i].gridY; ++j)
		{
			fprintf (f, "%d\n%f\n%f\n%f\n%f\n%f\n%f\n",
				g_cellFeatures[j].isOccupied,
				g_cellFeatures[j].vals[0], g_cellFeatures[j].vals[1], g_cellFeatures[j].vals[2], g_cellFeatures[j].vals[3], g_cellFeatures[j].vals[4], g_cellFeatures[j].vals[5]);
		}
		fclose (f);
	}

	Image_dtor (&img);
}

	#define USE_OBST_DET_IMAGES_ORIG
//	#define USE_OBST_DET_IMAGES_FROM_0506 // White balance is wrong; didn't setup the data anyway
	#define USE_OBST_DET_IMAGES_FROM_0527 // Some obstacle images amongst these
	#define USE_OBST_DET_IMAGES_FROM_EXP_FROM_0527 // Experiment
	#define USE_OBST_DET_IMAGES_FROM_EXP_FROM_0703 // Experiment
	#define USE_OBST_DET_IMAGES_FROM_EXP_FROM_0708 // Imgs of envir
	#define USE_OBST_DET_IMAGES_FROM_EXP_FROM_0715

//#include "ObstacleDetectionGridsOrig.c"
#include "ObstacleDetectionGrids2012_05_06.c"
#include "CooperativeLocalisationGrids2012_05_27.c"
#include "ObstacleDetectionGrids2012_05_27.c"
#include "ObstacleDetectionGrids2012_07_03.c"
#include "ObstacleDetectionGrids2012_07_08.c"
#include "ObstacleDetectionGrids2012_07_15.c"

int ObstacleDetection_setupTrainingData(TrainingData *data)
{
	int nTrainingImages = 0;
	int i = -1;
	unsigned int orig;
	unsigned int flip;

	orig = (2 << 8) | 4;
	flip = 2; // x = 0, y = 0

//	#include "ObstacleDetectionDataOrig.c"
	#include "ObstacleDetectionData2012_05_06.c"
	#include "CooperativeLocalisationData2012_05_27.c"
	#include "ObstacleDetectionData2012_05_27.c"
	#include "ObstacleDetectionData2012_07_03.c"
	#include "ObstacleDetectionData2012_07_08.c"
	#include "ObstacleDetectionData2012_07_15.c"

	nTrainingImages = i + 1;
	return nTrainingImages;
}

void ObstacleDetection_writeFeatures()
{
	TrainingData data[300];
	int nTrainingImages = ObstacleDetection_setupTrainingData (data);
	const char *trainingDataDir = "obstacleDetection";

	ObstacleDetection_writeImageFeatures(data, nTrainingImages, trainingDataDir);
}

void ObstacleDetection_calcImgCellFeaturesForTraining (Image *img, ImgCellFeatures *cellFeatures, const uchar *occupancyGrid, const int gridY, const int gridOrigX, const int gridOrigY, const int calcUnoccupied)
{
	int i, j;
	uchar isOccupied;

	for (j = 0; j < gridY; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			isOccupied = occupancyGrid[i + j * CAM_OCCUPANCY_GRID_X];
			cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].isOccupied = isOccupied;

			if (2 == calcUnoccupied ||
				(1 == calcUnoccupied && 0 == isOccupied) ||
				(0 == calcUnoccupied && 1 == isOccupied))
			{
				ObstacleDetection_calcCellFeatures (img, i, j, cellFeatures, gridOrigX, gridOrigY);
			}
			else
			{
				cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[0] = 0.0f;
				cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[1] = 0.0f;
				cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[2] = 0.0f;
				cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[3] = 0.0f;
				cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[4] = 0.0f;
				cellFeatures[i + j * CAM_OCCUPANCY_GRID_X].vals[5] = 0.0f;
			}
		}
	}
}



#endif // SIMULATION
