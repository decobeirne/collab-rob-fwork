#include "RobotRecognition.h"
#include "RobotRecognitionModel.h"
#include "RobotRecognitionCore.h"
#include "RobotRecognitionBlobDetection.h"
#include "RobotRecognitionRoughEstimation.h"
#include "RobotRecognitionAccurateEstimation.h"
#include "../Data/RobotDatabase.h"
#include "../../Common/RobotCore.h"
#include "../../Common/List.h"
#include "../../Common/Geometry.h"

#ifndef BOARD

//! Calc absolute orient from a robot's focal point to a blob
/*!
When we detect a blob we want to determine which side of the robot we are
seeing. We have absolute orients of all robots, so we can use the relative
orient to determine if it is the front, back, etc. If the relative orient
is e.g. 90 degrees though, we can't tell just from this which side it is, so
it will be more useful to use the orient to the detected blob.
*/
float RobotRecognition_calcOrientToBlob (
	const float ownOrient,
	const ColourBlob *blob)
{
	float tr, bl, blobCentre, blobCentreOffset, angle;
	tr = CAM_OCCUPANCY_GRID_ORIGIN_X + (CAM_OCCUPANCY_CELL_X * 0.5f) + (blob->tr.x * CAM_OCCUPANCY_CELL_X);
	bl = CAM_OCCUPANCY_GRID_ORIGIN_X + (CAM_OCCUPANCY_CELL_X * 0.5f) + (blob->bl.x * CAM_OCCUPANCY_CELL_X);

	blobCentre = (tr + bl) * 0.5f;
	blobCentreOffset = blobCentre - CAM_IMG_W_HALF; // Not v accurate but doesn't matter
	angle = atan (fabs (blobCentreOffset) / FOCAL_LEN);
	if (blobCentreOffset > 0.0f)
	{
		angle = -angle;
	}
	return Geometry_orientSum (ownOrient, angle);
}

float RobotRecognition_calcRobotRelativeOrient (
	const RobotData *robotDataArray,
	const int robotIndex,
	const int ownIndex)
{
	float robotRelativeOrient;

	// absolute=0 will return a signed relative orient
	robotRelativeOrient = Geometry_orientDiff (
		robotDataArray[ownIndex].pose.orient,
		robotDataArray[robotIndex].pose.orient,
		0);
	return robotRelativeOrient;
}

float RobotRecognition_calcPerceivedRobotRelativeOrient (
	const float orientToBlob,
	const float otherRobotOrient)
{
	float perceivedRobotRelativeOrient;

	// absolute=0 will return a signed relative orient
	perceivedRobotRelativeOrient = Geometry_orientDiff (
		orientToBlob,
		otherRobotOrient,
		0);
	return perceivedRobotRelativeOrient;
}

int RobotRecognition_calcFaceIndexFromRobotOrient (
	const int faceTypeIndex,
	const float perceivedRobotRelativeOrient)
{
	int faceIndex;

	if (FACE_TYPE_FRONTBACK == faceTypeIndex)
	{
		if (fabs (perceivedRobotRelativeOrient) > RADS_90)
		{
			faceIndex = 0; // Opposite direction
		}
		else
		{
			faceIndex = 1; // Roughly same direction
		}
	}
	else
	{
		if (perceivedRobotRelativeOrient > 0.0f)
		{
			faceIndex = 2; // left
		}
		else
		{
			faceIndex = 3; // right
		}
	}

	return faceIndex;
}

extern const float RobotRecognitionCore_faceOrientOffset[4];

void RobotRecognition_setupFaceOrientGivenIndex (
	const float orientToBlob,
	const float robotOrient,
	const Vector3F robotDir,
	RobotFace *robotFace)
{
	float orientChange;
	orientChange = RobotRecognitionCore_faceOrientOffset[robotFace->faceIndex];

	robotFace->faceOrient = Geometry_orientSum (robotOrient, orientChange);
	//robotFace->faceNormal = Vector3F_rotate (
	//	robotDir,
	//	2,
	//	orientChange);

	// absolute=0 will return a signed relative orient
	robotFace->perceivedFaceRelativeOrient = Geometry_orientDiff (
		orientToBlob,
		robotFace->faceOrient,
		0);
}

extern const int RobotRecognitionCore_colourIndexToRobotIndex[N_ROBOT_COLOURS];
extern const int RobotRecognitionCore_colourIndexToFaceTypeIndex[N_ROBOT_COLOURS];
extern const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS];

void RobotRecognition_identifyRobotFaces (
	FILE *outputFile,
	CamVectors *camVectors,
	const RobotData *robotDataArray,
	const int ownIndex,
	List *colourBlobs,
	List *robotEstimates)
{
	ColourBlob *blobPtr;
	ListNode *iter;
	int colourIndex, colourId;
	int robotIndex;
	RobotFace robotFace;
	RobotEstimate *tempPtr;
	RobotEstimate *robotEstimate;
	ListNode *robotLocationIter;
	float orientToBlob, perceivedRobotRelativeOrient;

	iter = colourBlobs->front;
	while (iter)
	{
		blobPtr = (ColourBlob*)iter->value;
		if (!RobotRecognitionBlobDetection_isBlobValid (blobPtr, 1))
		{
			iter = iter->next;
			continue;
		}

		colourIndex = blobPtr->colourIndex;
		colourId = RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex];
		robotIndex = RobotRecognitionCore_colourIndexToRobotIndex[colourIndex];

		// See if blob for this robotIndex has already been found
		robotEstimate = NULL;
		robotLocationIter = robotEstimates->front;
		while (robotLocationIter)
		{
			tempPtr = (RobotEstimate*)robotLocationIter->value;
			if (tempPtr->robotIndex == robotIndex)
			{
				robotEstimate = tempPtr;
				break;
			}
			robotLocationIter = robotLocationIter->next;
		}

		// This is the first blob for given robotIndex, so setup new struct for robot location
		if (!robotEstimate)
		{
			robotEstimate = (RobotEstimate*)malloc (sizeof (RobotEstimate));
			*robotEstimate = RobotEstimate_init();
			robotEstimate->robotIndex = robotIndex;

			if (robotIndex != -1) // -1 marks blobs that can belong to any robot
			{
				// The robot relative orient is a proper orient, i.e. directed
				robotEstimate->robotOrient = robotDataArray[robotIndex].pose.orient;
				robotEstimate->robotDir = Vector3F_rotate (
					Vector3F_init (1.0f, 0.0f, 0.0f),
					2,
					robotEstimate->robotOrient);
			}

			List_pushValue (robotEstimates, robotEstimate);
		}

		robotFace = RobotFace_init();
		robotFace.faceTypeIndex = RobotRecognitionCore_colourIndexToFaceTypeIndex[colourIndex];

		// We only do accurate location estimates for front/back and left/right
		// robot faces. Calculating estimates using top faces would be prone to
		// huge error due to the large number of artefacts on the robots.
		if (robotFace.faceTypeIndex == FACE_TYPE_TOP)
		{
			robotFace.faceIndex = 4; // 0123 = fblr
			robotFace.estType = FACE_EST_TOP;

#if VERBOSE_BLOB_DETECTION
			printf ("idenfityFace: colourId=%d robotIndex=%d faceTypeIndex=TOP\n", colourId, robotEstimate->robotIndex);
#endif
			fprintf (
				outputFile,
				"<FaceRecognised>colourId=%d robotIndex=%d estType=%d faceTypeIndex=%d faceIndex=%d</FaceRecognised>\n",
				colourId, robotIndex, robotFace.estType,
				robotFace.faceTypeIndex, robotFace.faceIndex);
		}
		else
		{
			DEBUG_ASSERT(robotIndex != -1)

			robotFace.blob = blobPtr;

			orientToBlob = RobotRecognition_calcOrientToBlob (
				robotDataArray[ownIndex].pose.orient,
				blobPtr);
			perceivedRobotRelativeOrient = RobotRecognition_calcPerceivedRobotRelativeOrient (
				orientToBlob,
				robotDataArray[robotIndex].pose.orient);

			robotFace.faceIndex = RobotRecognition_calcFaceIndexFromRobotOrient (
				robotFace.faceTypeIndex,
				perceivedRobotRelativeOrient);
			RobotRecognition_setupFaceOrientGivenIndex (
				orientToBlob,
				robotEstimate->robotOrient,
				robotEstimate->robotDir,
				&robotFace);

			if (fabs (robotFace.perceivedFaceRelativeOrient) < (RADS_110))
			{
				robotFace.estType = FACE_EST_CANT_USE;
			}
			else
			{
				// If angle is less than 30, then we should be able to use for an
				// accurate location estimate.
				robotFace.estType = FACE_EST_PROV_OK;
			}

#if VERBOSE_BLOB_DETECTION
			printf ("idenfityFace: colourId=%d robotIndex=%d faceTypeIndex=%d faceIndex=%d percFaceRelOrient=%.2f\n",
				colourId, robotIndex,
				robotFace.faceTypeIndex, robotFace.faceIndex,
				robotFace.perceivedFaceRelativeOrient);
#endif
			fprintf (
				outputFile,
				"<FaceRecognised>colourId=%d robotIndex=%d estType=%d faceTypeIndex=%d faceIndex=%d faceOrient=%f robotOrient=%f percFaceRelOrient=%f</FaceRecognised>\n",
				colourId, robotIndex, robotFace.estType,
				robotFace.faceTypeIndex, robotFace.faceIndex,
				robotFace.faceOrient, robotEstimate->robotOrient, robotFace.perceivedFaceRelativeOrient);

			robotEstimate->robotFaces[robotFace.faceTypeIndex] = robotFace;
		}

		iter = iter->next;
	}
}













#define READY_FOR_ACCURATE_REC 1

int RobotRecognition_detectRobotsNEW (
	FILE *outputFile,
#if VERBOSE_BLOB_DETECTION
	Image *displayImg,
	IplImage *iplImage,
	const int *windowSize,
	const uchar *colourIdsThisTrainingImg,
#endif
#if defined(DEBUGGING_ROBOT_REC)
	const int isLocSet,
#endif
	Image *originalImg,
	CamVectors *camVectors,
	UncertaintyConstants *uncertaintyConstants,
	const RobotData *robotDataArray,
	const int ownIndex,
	List *robotEstimates,
	float *colourScoreGrid,
	ImgCellFeatures *cellFeatures,
	OccupancyGrid *camOccupancyGrid)
{
	float strongestScore;
	List colourBlobs;
	int nValidBlobs;
	int anyRobotCells = 0;

#if defined(SIMULATION)
#if defined(DEBUGGING_ROBOT_REC)
#else
	assert (robotEstimates);
	assert (robotDataArray);
#endif
#endif

	colourBlobs = initList();

	strongestScore = RobotRecognitionModel_calcColourScores (
		cellFeatures,
		colourScoreGrid);

	if (strongestScore < COLOUR_CONFIDENCE_THRESHOLD)
	{
		anyRobotCells = RobotRecognitionCore_markRobotOccupiedCells (
			outputFile,
			colourScoreGrid,
			camOccupancyGrid,
			COLOUR_CONFIDENCE_THRESHOLD);

#if VERBOSE_BLOB_DETECTION == 2
		RobotRecognitionBlobDetection_displayRobotCells (
			iplImage,
			windowSize,
			displayImg,
			camOccupancyGrid);
#endif

		RobotRecognitionBlobDetection_createBlobs (
			outputFile,
#if VERBOSE_BLOB_DETECTION
			iplImage,
			windowSize,
			originalImg,
			colourIdsThisTrainingImg,
#endif
			ownIndex,
			colourScoreGrid,
			cellFeatures,
			camOccupancyGrid,
			&colourBlobs);

		nValidBlobs = RobotRecognitionBlobDetection_analyseBlobs (
#if VERBOSE_BLOB_DETECTION
			colourIdsThisTrainingImg,
#endif
			robotDataArray,
			ownIndex,
			camVectors,
			outputFile,
			&colourBlobs);

#if VERBOSE_BLOB_DETECTION
		RobotRecognitionBlobDetection_displayBlobs (
			outputFile,
			iplImage,
			windowSize,
			displayImg,
			&colourBlobs,
			nValidBlobs);
#endif

#if defined(DEBUGGING_ROBOT_REC)
		if (isLocSet)
#endif
		{
			// Calc blob edge cells, make rough estimate, calc exact blob edges, make accurate estimate
			RobotRecognition_identifyRobotFaces (
				outputFile,
				camVectors,
				robotDataArray,
				ownIndex,
				&colourBlobs,
				robotEstimates);

			RobotRecognitionRoughEstimation_estimateRobotLocations (
				outputFile,
				camVectors,
				robotDataArray,
				uncertaintyConstants,
				ownIndex,
				&colourBlobs,
				robotEstimates);

#if READY_FOR_ACCURATE_REC
			RobotRecognitionAccurateEstimation_estimateRobotLocationsNEW (
				outputFile,
				originalImg,
				camVectors,
				robotDataArray,
				uncertaintyConstants,
				ownIndex,
				robotEstimates);
#endif // READY_FOR_ACCURATE_REC
		}
	}
#if VERBOSE_BLOB_DETECTION
	else
	{
		printf ("No colour responses in this image\n");

		memcpy (iplImage->imageData, originalImg->data, originalImg->height * originalImg->wStep);
		cvNamedWindow ("emptyImg", windowSize);
		cvShowImage ("emptyImg", iplImage);
		cvWaitKey (1);
	}
#endif

	List_clear (&colourBlobs, 1);

	return anyRobotCells;
}
#undef READY_FOR_ACCURATE_REC


#endif
