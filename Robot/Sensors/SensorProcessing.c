#include "../../Common/RobotDefs.h"

#ifndef BOARD

#include "SensorProcessing.h"

void SensorProcessing_determineSensorRange (RobotDatabase *db)
{
	RobotStatus *stat = &db->status;

	EXPLICIT_DEBUG_ASSERT(db->geometryConstants.areExpDistsSetup == 1)

	// Determine central supervised point
	Geometry_ptFromOrient (
		stat->pose.loc.x,
		stat->pose.loc.y,
		&stat->scanCentrePt.x,
		&stat->scanCentrePt.y,
		SCAN_CENTRE_DIST + CAM_P_OFFSET,
		stat->pose.orient);
}

#if defined(RERUNNING_ROBOT)

#include "../Comm/RobotCommSerialize.h"

#if defined(RERUNNING_GUMSTIX)
extern void CameraProcessing_processImageData (RobotDatabase *db, Image *camImg);

#else // defined(RERUNNING_GUMSTIX)

extern void detectRobots_sim (RobotDatabase *db, const int index);
extern void recordMapScan_sim (RobotDatabase *db);
#endif // defined(RERUNNING_GUMSTIX)

void SensorProcessing_processSensorData_rerunningRobot (RobotDatabase *db)
{
	int serializedImageSize[4] = {10, 10, 200, 200};
	int isImg;

	readImage_serialize (db, &isImg);
	readCompass_serialize (db);

	// Re-rotate, as we may have a different orient
	CamVectors_rotateToRobotOrient (&db->camVectors, db->status.pose.orient);

#if defined(RERUNNING_GUMSTIX)
#if defined(IGNORE_CAM_DATA)
	// The occupancy grid will just be set to all blank here.
	CameraProcessing_processImageData (db, db->sensorData.camImg);

#else // defined(IGNORE_CAM_DATA)

	assert (isImg);
	{
		// Copied these from CameraProcessing_processSensorData
		Image_rectify (db->sensorData.camImg->data);
		Image_flip(db->sensorData.camImg->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);

		Image_show_givenIplImage (
			db->sensorData.camImg,
			"SerializedImage",
			db->cameraIplImage,
			serializedImageSize);
//		cvWaitKey (0);
		cvWaitKey (1);

		Image_toHSV (db->sensorData.camImg->data);
		CameraProcessing_processImageData (db, db->sensorData.camImg);
	}
#endif // defined(IGNORE_CAM_DATA)

#else // defined(RERUNNING_GUMSTIX) || defined(RERUNNING_IMAGES_ONLY)

	assert (!isImg);
	{
		if (1 != N_ROBOTS)
		{
			detectRobots_sim (db, db->status.index);
		}

		if (db->status.isNewPose)
		{
			recordMapScan_sim (db);
		}
	}
#endif // defined(RERUNNING_GUMSTIX)
}
#endif // defined(RERUNNING_ROBOT)

#if defined(RERUNNING_IMAGES_ONLY)
#if defined(SIM_ERROR)
extern void insertOrientError (RobotDatabase *db);
#endif
extern void CameraProcessing_processImageData (
	RobotDatabase *db,
	Image *camImg);

extern void RobotCommSerialize_readImage_rerunningImagesOnly (RobotDatabase *db);

void SensorProcessing_processSensorData_rerunningImagesOnly (
	RobotDatabase *db,
	const int isFakeMove)
{
	int serializedImageSize[4] = {10, 10, 200, 200};

#if defined(SIM_ERROR)
	// Error in orient when rotation/move has huge variance, so for now
	// just insert error for any move.
	if (db->status.isNewPose && db->status.nIterations && !isFakeMove)
	{
		insertOrientError (db);
	}

	// Re-rotate
	CamVectors_rotateToRobotOrient (&db->camVectors, db->status.pose.orient);
#endif

	RobotCommSerialize_readImage_rerunningImagesOnly (db);
	{
		// Copied these from CameraProcessing_processSensorData
		Image_rectify (db->sensorData.camImg->data);
		Image_flip(db->sensorData.camImg->data, CAM_IMG_FLIP_V, CAM_IMG_FLIP_H);

		Image_show_givenIplImage (
			db->sensorData.camImg,
			"SerializedImage",
			db->cameraIplImage,
			serializedImageSize);
		cvWaitKey (0);

		Image_toHSV (db->sensorData.camImg->data);

		// MapScan will be recorded inside this
		CameraProcessing_processImageData (db, db->sensorData.camImg);
	}
}
#endif // if defined(RERUNNING_IMAGES_ONLY)

void SensorProcessing_processSensorData (RobotDatabase *db, const int isFakeMove)
{
	if (1 == db->sensorData.isDataPendingInc)
	{
		return;
	}

	CamVectors_rotateToRobotOrient (&db->camVectors, db->status.pose.orient);

#if defined(RERUNNING_ROBOT)
	SensorProcessing_processSensorData_rerunningRobot (db);

#elif defined(RERUNNING_IMAGES_ONLY)

	SensorProcessing_processSensorData_rerunningImagesOnly (db, isFakeMove);

#else // defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)

#ifdef IS_GUMSTIX
	CameraProcessing_processSensorData (db);
#else
	SimulatedProcessing_processSensorData (db, isFakeMove);
#endif

#ifdef RECORDING_ROBOT
	// Don't need to write anything else. If sim, have already
	// written <SimulateOrientError>. If gumstix, have already
	// written <CompassReading>.
#endif
#endif // defined(RERUNNING_ROBOT) || defined(RERUNNING_IMAGES_ONLY)
}

void SensorProcessing_sanityCheck (RobotDatabase *db, const char *message)
{
	PointF loc;
	PointF ptf;
	PointI pti;
	uchar pixel;
	Image *navMap;
	int isOutside;
	navMap = db->environment.navMap;

	// Check if outside local map
	loc = db->status.pose.loc;
	ptf = PointI_toPointF (navMap->orig);

	isOutside = (loc.x < ptf.x || loc.x >= ptf.x + (float)LOC_MAP_DIMS ||
		loc.y < ptf.y || loc.y >= ptf.y + (float)LOC_MAP_DIMS);

	if (isOutside)
	{
		fprintf (db->xmlLog, "<SanityCheck>when=\"%s\" isOutside=1</SanityCheck>\n", message);
		return;
	}

	pti = PointF_toPointI (loc);
	pti.x -= navMap->orig.x;
	pti.y -= navMap->orig.y;
	pixel = navMap->data[pti.x + pti.y * LOC_MAP_DIMS];

	switch (pixel)
	{
	case NARROW_VEHICLE_TERRAIN:
	case 9:
	case 8:
	case 7:
	case 6:
		fprintf (db->xmlLog, "<SanityCheck>when=\"%s\" navMap=\"beside vehicle\"</SanityCheck>\n", message); break;
	case BROAD_VEHICLE_TERRAIN:
	case 59:
	case 58:
	case 57:
	case 56:
		fprintf (db->xmlLog, "<SanityCheck>when=\"%s\" navMap=\"near vehicle\"</SanityCheck>\n", message); break;
	case OCCUPIED_TERRAIN:
		fprintf (db->xmlLog, "<SanityCheck>when=\"%s\" navMap=\"occupied\"</SanityCheck>\n", message); break;
	case NARROW_OCCUPIED:
		fprintf (db->xmlLog, "<SanityCheck>when=\"%s\" navMap=\"beside occupied\"</SanityCheck>\n", message); break;
	case BROAD_OCCUPIED:
		fprintf (db->xmlLog, "<SanityCheck>when=\"%s\" navMap=\"near occupied\"</SanityCheck>\n", message); break;
	case FREE_TERRAIN:
		break;
	default:
		fprintf (db->xmlLog, "<SanityCheck>when=\"%s\" navMap=\"UNHANDLED!!!\"</SanityCheck>\n", message); break;
	}
}

#endif
