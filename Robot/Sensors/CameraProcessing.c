#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "CameraProcessing.h"
#include "../RobotVision/ObstacleRecognitionModel.h"
#include "../RobotVision/ObstacleRecognition.h"
#include "../RobotVision/ObstacleRecognitionCore.h"
#include "../RobotVision/RobotRecognition.h"
#include "../RobotVision/RobotRecognitionModel.h"
#include "../RobotVision/RobotRecognitionCore.h"
#include "../../Common/BitArray.h"


// These are made global to keep them off the stack
OccupancyGrid g_camOccupancyGrid;
ImgCellFeatures g_cellFeatures[SIZE_COOP_LOC_GRID];
ImgFeatures g_imgFeatures;
int g_featuresCalcdForImg = 0;
float g_camScoreGrid[SIZE_COOP_LOC_GRID];
float g_colourScoreGrid[SIZE_COOP_LOC_GRID * N_ROBOT_COLOURS];

#if defined (IS_GUMSTIX) || defined (RERUNNING_GUMSTIX) || defined(RERUNNING_IMAGES_ONLY)

extern BEHAVIOUR getBehaviourToAccredit (RobotDatabase *db);
extern int getTargetToAccredit (RobotDatabase *db);

//! Record sensor data in MapScan data structure
void CameraProcessing_recordMapScan (
	RobotDatabase *db,
	int isObstacleInCamOccupancyGrid,
	OccupancyGrid *camOccupancyGrid)
{
	Pose *p = &db->status.pose;
	MapScan *ms;
	OccupancyGrid *scanOccupancyGrid = 0;
	BEHAVIOUR behaviourToAccredit;
	int targetToAccredit;
#if defined(SIM_ERROR)
	PointF actualLoc;
#endif
#ifdef PRINT_EVENTS
	PointI pt;
#endif

	if (isObstacleInCamOccupancyGrid)
	{
		scanOccupancyGrid = OccupancyGrid_alloc();
		copyOccupancyGrid(scanOccupancyGrid, camOccupancyGrid);

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<MapScan>isObstacle=True</MapScan>\n");
		fprintf (db->xmlLog, "<CamOccupancyGrid>\n");
		for (pt.y = CAM_OCCUPANCY_GRID_Y_EXP - 1; pt.y >= 0; --pt.y)
		{
			for (pt.x = 0; pt.x < CAM_OCCUPANCY_GRID_X; ++pt.x)
			{
				fprintf (db->xmlLog, "%d ", TwoBitArray_checkPt (camOccupancyGrid->grid, pt, CAM_OCCUPANCY_GRID_X));
			}
			fprintf (db->xmlLog, "\n");
		}
		fprintf (db->xmlLog, "</CamOccupancyGrid>\n");
#endif
	}
#ifdef PRINT_EVENTS
	else
	{
		fprintf (db->xmlLog, "<MapScan>isObstacle=False</MapScan>\n");
	}
#endif

	behaviourToAccredit = getBehaviourToAccredit (db);
	targetToAccredit = getTargetToAccredit (db);

#ifdef SIM_ERROR
	actualLoc = p->loc;
	actualLoc.x += db->status.actualLocOffset.x;
	actualLoc.y += db->status.actualLocOffset.y;
#endif

	ms = (MapScan*)malloc (sizeof (MapScan));
	MapScan_init (
		ms,
#if defined(SIM_ERROR) // May occur when RERUNNING_IMAGES_ONLY is set
		actualLoc,
#endif
		scanOccupancyGrid,
		p->loc,
		p->orient,
		behaviourToAccredit,
		targetToAccredit,
		db->sensorData.localMap->orig,
		db->sensorData.localMapDataIndex,
		db->status.stdDev);

	ms->estdGain = estMapScanGain (
		db->status.scanCentrePt,
		db->sensorData.localMap->orig,
		db->environment.expGrid);

	List_pushValue (&db->sensorData.mapScanList, ms);

	db->sensorData.isUnincorporatedMapScan = 1;
}
#endif // defined (IS_GUMSTIX) || defined (RERUNNING_GUMSTIX) || defined(RERUNNING_IMAGES_ONLY)

//! Calculate the relative loc of the nearest visible robot in the current image
/*!
Don't calculate the absolute loc of the robot, as we may want the relative loc to update our
own location in CloseLoop_updateOwnLocEstNEW.
*/
void CameraProcesing_setVisibleRobotRelativeLoc (
	RobotDatabase *db,
	List *robotEstimates)
{
	ListNode *iter;
	RobotEstimate *thisEstimate, *estimate;
	Vector3F focalPt;
	float thisDist, dist;
	float *mat;

	db->sensorData.visRobotIndex = -1;

	if (robotEstimates->size)
	{
		if (robotEstimates->size == 1)
		{
			estimate = (RobotEstimate*)robotEstimates->front->value;
		}
		else
		{
			estimate = 0;
			dist = MAX_FLT;
			iter = robotEstimates->front;
			while (iter)
			{
				thisEstimate = (RobotEstimate*)iter->value;
				thisDist = estimate->estLoc.x * estimate->estLoc.x + estimate->estLoc.y * estimate->estLoc.y;
				if (thisDist < dist)
				{
					dist = thisDist;
					estimate = thisEstimate;
				}

				iter = iter->next;
			}
		}

		if (estimate)
		{
			db->sensorData.visRobotIndex = estimate->robotIndex;
			db->sensorData.visRobotEstCov = estimate->estCov;

			mat = db->sensorData.visRobotEstCov.mat;
			Uncertainty_scaleCovToSim (mat);

			// Estimate is relative to focal point, so calculate loc relative to our
			// own loc. Also scale from world space to sim space.
			focalPt = Vector3F_multiplyScalar (db->camVectors.robotDir, CAM_P_OFFSET);
//			focalPt.x += db->status.pose.loc.x;
//			focalPt.y += db->status.pose.loc.y;
			db->sensorData.visRobotRelLoc.x = focalPt.x + (estimate->estLoc.x / MAP_SCALE);
			db->sensorData.visRobotRelLoc.y = focalPt.y + (estimate->estLoc.y / MAP_SCALE);

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<DetectRobots>index=%d relativeLoc=(%f,%f) type=\"cam\" cov=(%f,%f,%f,%f)</DetectRobots>\n",
			db->sensorData.visRobotIndex, db->sensorData.visRobotRelLoc.x, db->sensorData.visRobotRelLoc.y,
			mat[0], mat[1], mat[2], mat[3]);
#endif
		}
	}
}



//! Process captured image, calculate occupied terrain and relative pose of closest visible robot
#define READY_FOR_ROBOT_REC_IN_EXPER 1
void CameraProcessing_processImageData (
	RobotDatabase *db,
	Image *camImg)
{
	int isObstacleInCamOccupancyGrid;
#if !defined(IGNORE_CAM_DATA)
	int anyRobotCells;
#endif
	List robotEstimates;

	robotEstimates = initList();

	OccupancyGrid_init (&g_camOccupancyGrid);
	isObstacleInCamOccupancyGrid = 0;

#if !defined(IGNORE_CAM_DATA)
	if (!g_featuresCalcdForImg)
	{
		ObstacleRecognition_calcImageFeatures (
			camImg,
			g_cellFeatures,
			&g_imgFeatures);
		g_featuresCalcdForImg = 1;
	}

#if READY_FOR_ROBOT_REC_IN_EXPER
	// First parameter is a dummy file ptr. We should turn off VERBOSE_BLOB_DETECTION
	// when calling from here.
#if VERBOSE_BLOB_DETECTION
	assert (0);
#endif
#if defined(DEBUGGING_ROBOT_REC)
	assert (0);
#endif

	anyRobotCells = RobotRecognition_detectRobotsNEW (
		db->xmlLog,
#if VERBOSE_BLOB_DETECTION
		NULL,
		NULL,
		NULL,
		NULL,
#endif
#if defined(DEBUGGING_ROBOT_REC)
		0,
#endif
		camImg,
		&db->camVectors,
		&db->uncertaintyConstants,
		db->groupData.robots,
		db->status.index,
		&robotEstimates,
		g_colourScoreGrid,
		g_cellFeatures,
		&g_camOccupancyGrid);
#else
	anyRobotCells = 0;
#endif

	ObstacleRecognition_calcOccupancy (
//		outputFile,
		camImg,
		g_cellFeatures,
		&g_imgFeatures,
		&g_camOccupancyGrid,
		g_camScoreGrid,
		anyRobotCells,
		&isObstacleInCamOccupancyGrid);

	CameraProcesing_setVisibleRobotRelativeLoc (db, &robotEstimates);

#endif // !defined(IGNORE_CAM_DATA)

#if defined (IS_GUMSTIX) || defined (RERUNNING_GUMSTIX) || defined(RERUNNING_IMAGES_ONLY)
	// We may call CameraProcessing_processImageData when testing, but don't want to record.
	if (db->status.isNewPose)
	{
		CameraProcessing_recordMapScan (
			db,
			isObstacleInCamOccupancyGrid,
			&g_camOccupancyGrid);
	}
#endif // ifdef IS_GUMSTIX

	List_clear (&robotEstimates, 1);
}
#undef READY_FOR_ROBOT_REC_IN_EXPER

#ifdef IS_GUMSTIX
extern char experimentDirName[128];

#ifdef IS_GUMSTIX
//! Get image from camera
int CameraProcessing_grabImage (RobotDatabase *db)
{
	Image *img = db->sensorData.camImg;
	uchar byCamX, byCamY;
#ifdef RECORDING_IMAGES
	FILE *f;
	char filename[128];
	char fullpath[256];
	const int i = db->status.nIterations;
#endif // RECORDING_IMAGES

	g_featuresCalcdForImg = 0;

	if (0 == bCMU_GetImage(&byCamX, &byCamY, img->data, (1024 * 74), 1))
	{
		fprintf (db->xmlLog, "<Error name=CameraProcessing_grabImage />");
		return -1;
	}

#ifdef PRINT_CAM_DETAIL
	fprintf (db->xmlLog, "<GrabbedImage />\n");
#endif

#ifdef RECORDING_IMAGES
	strcpy (fullpath, experimentDirName);
	sprintf (filename, "/img%05d.dat", i);
	strcat (fullpath, filename);
	f = fopen (fullpath, "w");
	fwrite (img->data, 1, 143 * 176 * 3, f);
	fclose (f);

	fprintf (db->xmlLog, "<SerializedImage>filename=\"%s\"</SerializedImage>\n", filename);
#endif // RECORDING_IMAGES

	return 1;
}
#endif // IS_GUMSTIX

//! Given measured robot orient, apply Harris offset function to account for errors.
/*!
The Harris offset function was fitted to compass points using zunzun.com. The function was
fit to the given data points with RMSE (root mean squared error) of 1.72190 and lowest peak
absolute value of absolute error of 3.019785.

The Harris with Offset 2D function is of the form: y = 1.0 / (a + bx^c) + offset.
*/
float applyHarrisOffsetFunction_robot2 (const float measuredOrientDegrees)
{
	const float a = 8.027535f;
	const float b = -7.577917f;
	const float c = -1.981032;
	const float offset = 2.982640f;
	float actualOrientDegrees;

	printf ("orient 0 %12f\n", measuredOrientDegrees);

	actualOrientDegrees = (1.0f / (a + pow (b * measuredOrientDegrees, c))) + offset;

#if 1
	printf ("Measured degrees %12f actual degrees %12f\n", measuredOrientDegrees, actualOrientDegrees);
#endif
	return actualOrientDegrees;
}

//! Given measured robot orient, apply function to account for errors.
/*!
The simple equation y = a*pow(x,b/x)+c*x + Offset was fitted to compass data using zunzun.com.
The function was fit to robot orientation data in degrees with RMSE (root mean squared error)
of 2.039716 and sum of squared absolute error 1.24813.
*/
float applyCompassFunction_robot2_crap (const float measuredOrientDegrees)
{
	const float a = -6.427004;
	const float b = -6.372626;
	const float c = 1.660491;
	const float offset = -3.427223;
	float actualOrientDegrees;

	actualOrientDegrees = (a * pow(measuredOrientDegrees, b / measuredOrientDegrees) + c * measuredOrientDegrees) + offset;

#if 1
	printf ("Measured degrees %12f actual degrees %12f\n", measuredOrientDegrees, actualOrientDegrees);
#endif
	return actualOrientDegrees;
}

//! Apply Lorentzian Peak E plus offset model to orientation data.
/*!
The model was obtained from zunzun.com: y = 1.0 / (a + ((x-b)/c)2) + Offset.

The curve was fit to the given data points with root mean squared error 2.2965280485.
*/
float applyCompassFunction_robot2 (const float inputOrient)
{
	const float a = 0.00135605f;
	const float b = 515.31996944f;
	const float c = -10653.60550022f;
	const float offset = -268.55039660f;
	float outputOrient;

	outputOrient = (1.0f / (a + pow((inputOrient - b) / c, 2.0f))) + offset;
	return outputOrient;
}

float SensorProcessing_calcOrient(const UINT16 reading)
{
	float f;
#ifdef IS_HMC6352
	unsigned char buf[2];
	memcpy (buf, &reading, 2);
	f = ((buf[0] << 8) + buf[1]) * 0.1f; // The reading is in units of .1 degrees => divide by 10
	printf ("orient 0 %12f\n", f);
#ifdef FLIP_COMPASS
	f = 360.0f - f; // Compass may be mounted upside down
#endif
	printf ("orient 1 %12f\n", f);
#if ROBOT_PLATFORM == 2


//	f = applyHarrisOffsetFunction_robot2 (f);
	f = applyCompassFunction_robot2 (f);


#endif

#else // ifdef IS_HMC6352

	f = (float)reading * 0.1f; // Again, the reading is in units of .1 degrees => divide by 10

#ifdef FLIP_COMPASS
	f = 360.0f - f; // Compass may be mounted upside down
#endif
#endif // ifdef IS_HMC6352

	// Convert to radians AFTER other processing steps. The compasses return valus in degrees,
	// and curve fitting, etc. have been implemented using these values.
	f *= (PI / 180.0f);

	return f;
}

#ifdef IS_GUMSTIX
float grabCompassReading (RobotDatabase *db)
{
	UINT16 compass;
	if (0 == bCMU_ReadCompass(&compass))
	{
		assert (0);
	}
	return SensorProcessing_calcOrient (compass);
}
#endif

void CameraProcessing_processSensorData (RobotDatabase *db)
{
#ifdef IS_GUMSTIX
	float compassReading;
#ifndef IGNORE_CAM_DATA
	int nTries;
#endif
#endif

#ifdef IS_GUMSTIX
	// Update compass reading to compensate for initial offset from 0.0f.
	compassReading = grabCompassReading (db);
//	compassReading = Geometry_orientSum (compassReading, COMPASS_ORIENT_OFFSET);
	compassReading = Geometry_orientSum (compassReading, db->sensorData.compassOffset);

#ifdef PRINT_CAM_DETAIL
	fprintf (db->xmlLog, "<CompassReading>currentOrient=%14.12f reading=%14.12f</CompassReading>\n", db->status.pose.orient, compassReading);
#endif

#ifndef IGNORE_COMPASS_DATA
	db->status.pose.orient = compassReading;
#endif

	// Re-rotate after changing orient
	CamVectors_rotateToRobotOrient (&db->camVectors, db->status.pose.orient);

#ifdef IGNORE_CAM_DATA
#ifdef RECORDING_IMAGES
	fprintf (db->xmlLog, "<SerializedImage>filename=\"fakeCamData\"</SerializedImage>\n");
#endif // RECORDING_IMAGES

	// This will record a map scan, but the occupancy grid will just be blank
	CameraProcessing_processImageData (db, db->sensorData.camImg);

#else // ifdef IGNORE_CAM_DATA

	// Grabbing an image takes about 8 seconds, so don't do it if we don't need
	if (!db->status.isNewPose &&
		(N_ROBOTS == 1 ||
		(N_ROBOTS == 2 && db->partnerData.explorationCoalition.id != -1 && db->partnerData.explorationCoalition.collabData.isSupAtFinalDest)))
	{
		fprintf (db->xmlLog, "<SkippingImage />\n");
		return;
	}

	nTries = 0;
	while (1)
	{
		CameraProcessing_grabImage (db); // Set g_featuresCalcdForImg = 0 in here

		Image_rectify (db->sensorData.camImg->data);
		Image_flip(db->sensorData.camImg->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);
		Image_toHSV (db->sensorData.camImg->data);

		ObstacleRecognition_calcImageFeatures (
			db->sensorData.camImg,
			g_cellFeatures,
			&g_imgFeatures);
		g_featuresCalcdForImg = 1;
		if (ObstacleRecognitionCore_isImageOk (
			db->sensorData.camImg,
			g_cellFeatures,
			&g_imgFeatures))
		{
			break;
		}
		else if (++nTries > 3)
		{
			fprintf (db->xmlLog, "<FatalCameraError />\n");
			fflush (db->xmlLog);
			assert (0);
		}
		tBOOL bCMU_Reset();
	}

	CameraProcessing_processImageData (db, db->sensorData.camImg);
#endif // ifdef IGNORE_CAM_DATA
#endif // IS_GUMSTIX
}
#endif // IS_GUMSTIX
#endif
