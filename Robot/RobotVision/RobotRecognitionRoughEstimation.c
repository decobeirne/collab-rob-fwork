#include "RobotRecognitionRoughEstimation.h"
#include "RobotRecognitionCore.h"
#include "../../Common/Geometry.h"
#include "../../Common/Uncertainty.h"
#include "../../Common/Maths.h"

#ifndef BOARD


void RobotRecognitionRoughEstimation_calcBlobEdgeCells (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	ColourBlob *blob,
	uchar *tempGrid)
{
	int i, j;
	int cellIndex;
	int k, l;
#if VERBOSE_BLOB_DETECTION
	int nSeedCells;
	int nBackgroundCells;
	int nBlobEdgeCells;
	int nBlobEdgeNbours;
#endif
	int isBlobEdgeCell;

	memcpy (tempGrid, blob->blobGrid, SIZE_COOP_LOC_GRID);

	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		if (tempGrid[i] == blob->id)
		{
			tempGrid[i] = 0; // blob
		}
		else
		{
			tempGrid[i] = 255; // bg
		}
	}

#if VERBOSE_BLOB_DETECTION
	nSeedCells = 0;
#endif

	// For blob edge cells, encode what side of face the cell is on
	// 1=isBlobEdgeCell
	// 2=top
	// 4=bottom
	// 8=left
	// 16=right

	// j = 0
	for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
	{
		if (tempGrid[i] == 255)
		{
			// Mark as valid background cell, i.e. not a gap within
			// a blob
			tempGrid[i] = 254;
#if VERBOSE_BLOB_DETECTION
			++nSeedCells;
#endif
		}
		else if (tempGrid[i] == 0)
		{
			// Mark as blob edge cell
			tempGrid[i] |= 5; // 1 | 4(bottom)
		}
	}

	// j = CAM_OCCUPANCY_GRID_Y - 1
	for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
	{
		cellIndex = i + ((CAM_OCCUPANCY_GRID_Y - 1) * CAM_OCCUPANCY_GRID_X);
		if (tempGrid[cellIndex] == 255)
		{
			tempGrid[cellIndex] = 254;
#if VERBOSE_BLOB_DETECTION
			++nSeedCells;
#endif
		}
		else if (tempGrid[cellIndex] == 0)
		{
			tempGrid[cellIndex] |= 3; // 1 | 2(top)
		}
	}

	// i = 0
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		cellIndex = j * CAM_OCCUPANCY_GRID_X;
		if (tempGrid[cellIndex] == 255)
		{
			tempGrid[cellIndex] = 254;
#if VERBOSE_BLOB_DETECTION
			++nSeedCells;
#endif
		}
		else if (tempGrid[cellIndex] == 0)
		{
			tempGrid[cellIndex] |= 9; // 1 | 8(left)
		}
	}

	// i = CAM_OCCUPANCY_GRID_X - 1
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		cellIndex = (CAM_OCCUPANCY_GRID_X - 1) + j * CAM_OCCUPANCY_GRID_X;
		if (tempGrid[cellIndex] == 255)
		{
			tempGrid[cellIndex] = 254;
#if VERBOSE_BLOB_DETECTION
			++nSeedCells;
#endif
		}
		else if (tempGrid[cellIndex] == 0)
		{
			tempGrid[cellIndex] |= 17; // 1 | 16(right)
		}
	}

#if VERBOSE_BLOB_DETECTION
	RELEASE_ASSERT(nSeedCells)
	nBackgroundCells = nSeedCells;
#endif

	// Flood fill from bottom left
	for (j = 1; j < CAM_OCCUPANCY_GRID_Y - 1; ++j)
	{
		for (i = 1; i < CAM_OCCUPANCY_GRID_X - 1; ++i)
		{
			if (tempGrid[i + j * CAM_OCCUPANCY_GRID_X] == 255)
			{
				for (l = j - 1; l < j + 2; ++l)
				{
					for (k = i - 1; k < i + 2; ++k)
					{
						if (tempGrid[k + l * CAM_OCCUPANCY_GRID_X] == 254)
						{
							tempGrid[i + j * CAM_OCCUPANCY_GRID_X] = 254;
#if VERBOSE_BLOB_DETECTION
							++nBackgroundCells;
#endif
							goto RobotRecognitionRoughEstimation_calcBlobEdgeCells_nbourFound1;
						}
					}
				}
			}
RobotRecognitionRoughEstimation_calcBlobEdgeCells_nbourFound1:
			// Break from 2 loops
			;
		}
	}

	// Flood fill from top right
	for (j = CAM_OCCUPANCY_GRID_Y - 2; j > 1; --j)
	{
		for (i = CAM_OCCUPANCY_GRID_X - 2; i > 1; --i)
		{
			if (tempGrid[i + j * CAM_OCCUPANCY_GRID_X] == 255)
			{
				for (l = j - 1; l < j + 2; ++l)
				{
					for (k = i - 1; k < i + 2; ++k)
					{
						if (tempGrid[k + l * CAM_OCCUPANCY_GRID_X] == 254)
						{
							tempGrid[i + j * CAM_OCCUPANCY_GRID_X] = 254;
#if VERBOSE_BLOB_DETECTION
							++nBackgroundCells;
#endif
							goto RobotRecognitionRoughEstimation_calcBlobEdgeCells_nbourFound2;
						}
					}
				}
			}
RobotRecognitionRoughEstimation_calcBlobEdgeCells_nbourFound2:
			// Break from 2 loops
			;
		}
	}

	// At this point, anything 255 is an invalid background cell, while anything
	// 254 is valid. Blob cells along the img boundary have been marked
#if VERBOSE_BLOB_DETECTION
	nBlobEdgeCells = 0;
	nBlobEdgeNbours = 0;
#endif

	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			if (tempGrid[i + j * CAM_OCCUPANCY_GRID_X] == 0)
			{
				isBlobEdgeCell = 0;
				for (l = max (0, j - 1); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
				{
					for (k = max (0, i - 1); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
					{
						if (k == i && l == j)
						{
							continue;
						}

						if (tempGrid[k + l * CAM_OCCUPANCY_GRID_X] == 254 ||
							tempGrid[k + l * CAM_OCCUPANCY_GRID_X] == 253)
						{
#if VERBOSE_BLOB_DETECTION
							if (tempGrid[k + l * CAM_OCCUPANCY_GRID_X] == 254)
							{
								++nBlobEdgeNbours;
							}
#endif
							tempGrid[k + l * CAM_OCCUPANCY_GRID_X] = 253; // bg cell that borders a blob

							// Encode what side of blob the cell is on in 4 bits of value
							if (l > j)
							{
								isBlobEdgeCell |= 2;
							}
							else if (l < j)
							{
								isBlobEdgeCell |= 4;
							}

							if (k < i)
							{
								isBlobEdgeCell |= 8;
							}
							else if (k > i)
							{
								isBlobEdgeCell |= 16;
							}
						}
					}
				}

				if (isBlobEdgeCell)
				{
					// Mark as blob edge cell, and encode possible sides: t/b/l/r
					tempGrid[i + j * CAM_OCCUPANCY_GRID_X] = (isBlobEdgeCell | 1);
#if VERBOSE_BLOB_DETECTION
					++nBlobEdgeCells;
#endif
				}
			}
		}
	}

#if VERBOSE_BLOB_DETECTION
	fprintf (outputFile, "<BlobEdgeCells>\n");
	fprintf (
		outputFile,
		"blobId=%d nBackgroundSeedCells=%d nBackgroundCells=%d nBlobEdgeCells=%d nBlobEdgeNbours=%d\n",
		blob->id,
		nSeedCells,
		nBackgroundCells,
		nBlobEdgeCells,
		nBlobEdgeNbours);

	fprintf (outputFile, "<Cells>");
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			cellIndex = i + j * CAM_OCCUPANCY_GRID_X;
			if (tempGrid[cellIndex] && !(tempGrid[cellIndex] & 64))
			{
				fprintf (
					outputFile,
					"(%d,(%d,%d),(%d,%d)), ",
					tempGrid[cellIndex],
					i,
					j,
					CAM_OCCUPANCY_GRID_ORIGIN_X + i * (CAM_OCCUPANCY_CELL_X / 2),
					CAM_OCCUPANCY_GRID_ORIGIN_Y + j * (CAM_OCCUPANCY_CELL_Y / 2));
			}
		}
	}
	fprintf (outputFile, "</Cells>\n</BlobEdgeCells>\n");
#if VERBOSE_BLOB_DETECTION > 2
	for (isBlobEdgeCell = 0, i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		isBlobEdgeCell += (int)(tempGrid[i] == 255);
	}
	fprintf (outputFile, "<VerifyBlobEdgeCells>type=255 n=%d</VerifyBlobEdgeCells>\n", isBlobEdgeCell);
	for (isBlobEdgeCell = 0, i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		isBlobEdgeCell += (int)(tempGrid[i] == 254);
	}
	fprintf (outputFile, "<VerifyBlobEdgeCells>type=254 n=%d</VerifyBlobEdgeCells>\n", isBlobEdgeCell);
	for (isBlobEdgeCell = 0, i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		isBlobEdgeCell += (int)(tempGrid[i] == 253);
	}
	fprintf (outputFile, "<VerifyBlobEdgeCells>type=253 n=%d</VerifyBlobEdgeCells>\n", isBlobEdgeCell);
	for (isBlobEdgeCell = 0, i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		isBlobEdgeCell += (int)(tempGrid[i] == 0);
	}
	fprintf (outputFile, "<VerifyBlobEdgeCells>type=0 n=%d</VerifyBlobEdgeCells>\n", isBlobEdgeCell);
	for (isBlobEdgeCell = 0, i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		isBlobEdgeCell += (int)(tempGrid[i] < 32 && tempGrid[i] != 0);
	}
	fprintf (outputFile, "<VerifyBlobEdgeCells>type=\"anyBlobEdge\" n=%d</VerifyBlobEdgeCells>\n", isBlobEdgeCell);
	for (isBlobEdgeCell = 0, i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		isBlobEdgeCell += (int)(tempGrid[i] < 32 && (tempGrid[i] & 2));
	}
	fprintf (outputFile, "<VerifyBlobEdgeCells>type=\"top\" n=%d</VerifyBlobEdgeCells>\n", isBlobEdgeCell);
	for (isBlobEdgeCell = 0, i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		isBlobEdgeCell += (int)(tempGrid[i] < 32 && (tempGrid[i] & 4));
	}
	fprintf (outputFile, "<VerifyBlobEdgeCells>type=\"bottom\" n=%d</VerifyBlobEdgeCells>\n", isBlobEdgeCell);
	for (isBlobEdgeCell = 0, i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		isBlobEdgeCell += (int)(tempGrid[i] < 32 && (tempGrid[i] & 8));
	}
	fprintf (outputFile, "<VerifyBlobEdgeCells>type=\"left\" n=%d</VerifyBlobEdgeCells>\n", isBlobEdgeCell);
	for (isBlobEdgeCell = 0, i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		isBlobEdgeCell += (int)(tempGrid[i] < 32 && (tempGrid[i] & 16));
	}
	fprintf (outputFile, "<VerifyBlobEdgeCells>type=\"right\" n=%d</VerifyBlobEdgeCells>\n", isBlobEdgeCell);
#endif // if 1/0
#endif

	memcpy (blob->blobGrid, tempGrid, SIZE_COOP_LOC_GRID);
}




















extern const RobotParams RobotRecognitionCore_robotParams[3];

void RobotRecognitionRoughEstimation_estLocFromFaceEdge (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	RobotEstimate *robotEstimate,
	RobotFace *robotFace,
	const float ownOrient,
	const PointI pixel,
	CamVectors *camVectors,
	UncertaintyConstants *uncertaintyConstants,
	const float heightWorldSpace,
	const float distToProject,
	const float orientToProject)
{
	PointI pixel2, slopeVals;
	PointF ptPixelSpace, ptPixelSpace2, robotLocWorldSpace;
	Vector3F ptWorldSpace, ptWorldSpace2;
	float slope, distFromFaceToCog;
	int useXAxis;

	// Project to pt in world space at expected height, i.e. intersect with
	// plane with normal (0,0,1) containing a pt (0,0,h)
	ptPixelSpace = CamVectors_pixelToImageOffsets (pixel);
	ptWorldSpace = CamVectors_projectImageToHorizontalPlane (
		camVectors,
		ptPixelSpace,
		heightWorldSpace);

	// Get a second pt on the place s.th. it can be projected back to the camera
	// space in order to determine the expected slope of edges
	//
	// This also happens to be the centre of the face => est the loc of the robot
	// from this as well.
	Geometry_ptFromOrient (
		ptWorldSpace.x,
		ptWorldSpace.y,
		&ptWorldSpace2.x,
		&ptWorldSpace2.y,
		distToProject,
		orientToProject);
	ptWorldSpace2.z = ptWorldSpace.z;

	// Calculate expected slope of horizontal edge of face in camera image
	ptPixelSpace2 = CamVectors_projectWorldSpaceToImage (
		camVectors,
		ptWorldSpace2);
	pixel2 = CamVectors_imageOffsetsToPixel (ptPixelSpace2);
	slopeVals.x = pixel2.x - pixel.x;
	slopeVals.y = pixel2.y - pixel.y;
	if (slopeVals.x < 0)
	{
		slopeVals.x = -slopeVals.x;
		slopeVals.y = -slopeVals.y;
	}
	slope = (float)slopeVals.y / slopeVals.x;
	useXAxis = 0;
	if (fabs (slope) > 1.0f)
	{
		slope = (float)slopeVals.x / slopeVals.y;
		useXAxis = 1;
	}

	robotFace->slopeCameraSpace = slope;
	robotFace->useCameraXAxis = useXAxis;

	// Estimate robot location from the face centre pt
	distFromFaceToCog = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].cogToFace[robotFace->faceIndex];
	Geometry_ptFromOrient (
		ptWorldSpace2.x,
		ptWorldSpace2.y,
		&robotLocWorldSpace.x,
		&robotLocWorldSpace.y,
//		distFromFaceToCog,
		-distFromFaceToCog,
		robotFace->faceOrient);
	
	robotFace->locEst = robotLocWorldSpace;

	robotFace->cov.vector = Uncertainty_calcVisualEstimateCovariance (
		uncertaintyConstants,
		ownOrient,
		uncertaintyConstants->visEst_coarseFace_haveCorner_varFwd,
		uncertaintyConstants->visEst_coarseFace_haveCorner_varLat,
		uncertaintyConstants->visEst_coarseFace_haveCorner_covar);

	robotFace->estType = FACE_EST_CORNER;

#if VERBOSE_BLOB_DETECTION
//	printf ("corner: pixel=(%d,%d) camPt=(%.2f,%.2f) worldPt=(%.2f,%.2f,%.2f) faceCentre=(%.2f,%.2f,%.2f) est=(%.2f,%.2f)\n",
	printf ("corner: pixel=(%d,%d) camPt=(%.2f,%.2f) worldPt=(%.2f,%.2f,%.2f) faceCentre=(%.2f,%.2f,%.2f)\n",
		pixel.x, pixel.y,
		ptPixelSpace.x, ptPixelSpace.y,
		ptWorldSpace.x, ptWorldSpace.y, ptWorldSpace.z,
		ptWorldSpace2.x, ptWorldSpace2.y, ptWorldSpace2.z);
//		robotLocWorldSpace.x, robotLocWorldSpace.y);
#endif
}

void RobotRecognitionRoughEstimation_estLocFromHorizontalEdge (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	RobotEstimate *robotEstimate,
	RobotFace *robotFace,
	const float ownOrient,
	const PointI pixel,
	CamVectors *camVectors,
	UncertaintyConstants *uncertaintyConstants,
	const float heightWorldSpace,
	const float distToProject,
	const float orientToProject)
{
	PointI pixel2, slopeVals;
	PointF ptPixelSpace, ptPixelSpace2, robotLocWorldSpace;
	Vector3F ptWorldSpace, ptWorldSpace2;
	float slope, distFromFaceToCog;
	int useXAxis;

	// Project to pt in world space at expected height, i.e. intersect with
	// plane with normal (0,0,1) containing a pt (0,0,h)
	ptPixelSpace = CamVectors_pixelToImageOffsets (pixel);
	ptWorldSpace = CamVectors_projectImageToHorizontalPlane (
		camVectors,
		ptPixelSpace,
		heightWorldSpace);

	// Get a second pt on the place s.th. it can be projected back to the camera
	// space in order to determine the expected slope of edges
	//
	// This also happens to be the centre of the face => est the loc of the robot
	// from this as well.
	Geometry_ptFromOrient (
		ptWorldSpace.x,
		ptWorldSpace.y,
		&ptWorldSpace2.x,
		&ptWorldSpace2.y,
		distToProject,
		orientToProject);
	ptWorldSpace2.z = ptWorldSpace.z;

	// Calculate expected slope of horizontal edge of face in camera image
	ptPixelSpace2 = CamVectors_projectWorldSpaceToImage (
		camVectors,
		ptWorldSpace2);
	pixel2 = CamVectors_imageOffsetsToPixel (ptPixelSpace2);
	slopeVals.x = pixel2.x - pixel.x;
	slopeVals.y = pixel2.y - pixel.y;
	if (slopeVals.x < 0)
	{
		slopeVals.x = -slopeVals.x;
		slopeVals.y = -slopeVals.y;
	}
	slope = (float)slopeVals.y / slopeVals.x;
	useXAxis = 0;
	if (fabs (slope) > 1.0f)
	{
		slope = slopeVals.x / slopeVals.y;
		useXAxis = 1;
	}

	robotFace->slopeCameraSpace = slope;
	robotFace->useCameraXAxis = useXAxis;

	// Here we cannot assume that the point we are given is a corner of the face,
	// instead we make a v rough est that it is the centre. The error is taken
	// into account in the covariance matrix below
	distFromFaceToCog = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].cogToFace[robotFace->faceIndex];
	Geometry_ptFromOrient (
		ptWorldSpace.x,
		ptWorldSpace.y,
		&robotLocWorldSpace.x,
		&robotLocWorldSpace.y,
		-distFromFaceToCog,
		robotFace->faceOrient);
//		distFromFaceToCog,
//		-robotFace->faceOrient);

	robotFace->locEst = robotLocWorldSpace;

	robotFace->cov.vector = Uncertainty_calcVisualEstimateCovariance (
		uncertaintyConstants,
		ownOrient,
		uncertaintyConstants->visEst_coarseFace_noCorner_varFwd,
		uncertaintyConstants->visEst_coarseFace_noCorner_varLat,
		uncertaintyConstants->visEst_coarseFace_noCorner_covar);

/*
	// Not calling this, as var calcd from script incorporates this error
	robotFace->cov.vector = Uncertainty_accumRobotFaceHorizonalEdgeEstimateCovariance (
		uncertaintyConstants,
		robotFace->cov.vector,
		robotFace->faceOrient,
		RobotRecognitionCore_getFaceHalfLen (robotEstimate->robotIndex, robotFace->faceIndex));
*/
	robotFace->estType = FACE_EST_EDGE;

#if VERBOSE_BLOB_DETECTION
	printf ("edgePt: pixel=(%d,%d) camPt=(%.2f,%.2f) worldPt=(%.2f,%.2f,%.2f)\n",
		pixel.x, pixel.y,
		ptPixelSpace.x, ptPixelSpace.y,
		ptWorldSpace.x, ptWorldSpace.y, ptWorldSpace.z);
#endif
}

extern const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS];

/*!
Calculate expected slope of horizontal edges of the face in the
robot's camera image.

Input orients are the visible face relative to the viewing robot. A
face with a relative orient of 0 will be in the exact same direction as
the robot and therefore not visible. Faces with a relative orient of less
than 30 degrees should already have been discounted by this point.

Determine if more straight-forward to take slope as y/x (as
typical) or as x/y. The range of values will be closer to the
bounds of the image for a smaller orient. Also for extreme cases
this will be more accureate.
*/
void RobotRecognitionRoughEstimation_estimateFaceRoughLocation (
	FILE *outputFile,
	CamVectors *camVectors,
	RobotFace *robotFace,
	const float ownOrient,
	UncertaintyConstants *uncertaintyConstants,
	const ColourBlob *blob,
	RobotEstimate *robotEstimate)
{
	float heightWorldSpace;
	float orientToProject;
	float distToProject;
	PointI gridPt, pixel;

	// Obviously, any visible face will have an abs orient diff of > PI/2
	// Any face with an orient diff > 0, will be anticlockwise from our own orient,
	// so the edge of the face furthest from us should be in the top-left of the blob aabb.
	//
	// If this is not visible, then the nearest corner should be in the bottom-right
	// of the blob AABB.
	//
	// If neither of these are available, then we can only make a rough estimate, so the
	// bottom-right corner of the blob is taken. This may result in a rough estimate that
	// is further in distance than the face actually is. This is ok because:
	//
	// If we can make an accurate estimate this will not be a factor
	//
	// We will dilate the visible robot location on our nav map in order to avoid any
	// conflict anyway
	if (robotFace->perceivedFaceRelativeOrient > 0.0f)
	{
		// Use top left corner of blob as a pt to project to world space
//		if (blob->tr.y != (CAM_OCCUPANCY_GRID_Y - 1) && blob->bl.x != (CAM_OCCUPANCY_GRID_X - 1))
		if (blob->bl.x != 0 && blob->tr.y != (CAM_OCCUPANCY_GRID_Y - 1))
		{
			gridPt.x = blob->bl.x;
			gridPt.y = blob->tr.y;
			pixel = RobotRecognitionCore_gridPtToCameraPixel (gridPt);

			heightWorldSpace = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].faceTopEdgeHeight;

			// We have edge of face furthest from viewing robot, so centre of face is
			// project towards us. So if our orient is 0, and face relative orient
			// (of face normal) is 30 degrees, then we project centre of face from this
			// point in the world at (30 + 90) degrees
			orientToProject = Geometry_orientSum (robotFace->faceOrient, RADS_90);

			distToProject = RobotRecognitionCore_getDistToSide (
				robotEstimate->robotIndex,
				robotFace->faceIndex,
				1); // Want dist from robot COG to left of face
			
			RobotRecognitionRoughEstimation_estLocFromFaceEdge (
#if VERBOSE_BLOB_DETECTION
				outputFile,
#endif
				robotEstimate,
				robotFace,
				ownOrient,
				pixel,
				camVectors,
				uncertaintyConstants,
				heightWorldSpace,
				distToProject,
				orientToProject);

#if VERBOSE_BLOB_DETECTION
			printf ("estFace: corner=tl est=(%f,%f) cov=(%f,%f,%f,%f)\n",
				robotFace->locEst.x, robotFace->locEst.y,
				robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w);
#endif
			fprintf (
				outputFile,
				"<FaceRoughLocation>colourId=%d perceivedRelOrient=%f cornerVisible=1 corner=\"tl\" pixel=(%d,%d) est=(%f,%f) cov=(%f,%f,%f,%f) slope=%f useXAxis=%d</FaceRoughLocation>\n",
				RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
				robotFace->perceivedFaceRelativeOrient, pixel.x, pixel.y,
				robotFace->locEst.x, robotFace->locEst.y,
				robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w,
				robotFace->slopeCameraSpace, robotFace->useCameraXAxis);
		}
		// Otherwise see can we use bottom right corner
		else if (blob->tr.x != (CAM_OCCUPANCY_GRID_X - 1) && blob->bl.y != 0)
		{
			gridPt.x = blob->tr.x;
			gridPt.y = blob->bl.y;
			pixel = RobotRecognitionCore_gridPtToCameraPixel (gridPt);
			heightWorldSpace = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].faceEdgeBottomHeight;

			// We have edge of face nearest to viewing robot, so centre of face is
			// projected away from us.
			orientToProject = Geometry_orientSum (robotFace->faceOrient, -RADS_90);

			distToProject = RobotRecognitionCore_getDistToSide (
				robotEstimate->robotIndex,
				robotFace->faceIndex,
				0); // Want dist from robot COG to right of face

			RobotRecognitionRoughEstimation_estLocFromFaceEdge (
#if VERBOSE_BLOB_DETECTION
				outputFile,
#endif
				robotEstimate,
				robotFace,
				ownOrient,
				pixel,
				camVectors,
				uncertaintyConstants,
				heightWorldSpace,
				distToProject,
				orientToProject);

#if VERBOSE_BLOB_DETECTION
			printf ("estFace: corner=br est=(%f,%f) cov=(%f,%f,%f,%f)\n",
				robotFace->locEst.x, robotFace->locEst.y,
				robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w);
#endif
			fprintf (
				outputFile,
				"<FaceRoughLocation>colourId=%d perceivedRelOrient=%f cornerVisible=1 corner=\"br\" pixel=(%d,%d) est=(%f,%f) cov=(%f,%f,%f,%f) slope=%f useXAxis=%d</FaceRoughLocation>\n",
				RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
				robotFace->perceivedFaceRelativeOrient, pixel.x, pixel.y,
				robotFace->locEst.x, robotFace->locEst.y,
				robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w,
				robotFace->slopeCameraSpace, robotFace->useCameraXAxis);
		}
		else
		{
			// We don't have a corner from which we can make an est, so the blob must border
			// either the left or right side of the image. We can however check if we have
			// any point on either the top of bottom edge of the face available. If this is the
			// case, then we can make a very rough estimation, taking the point to be at the
			// midpt of that edge.
			if (blob->tr.y != (CAM_OCCUPANCY_GRID_Y - 1))
			{
				// Take point on top edge, assume it to be the midpt
				gridPt.x = blob->bl.x;
				gridPt.y = blob->tr.y;
				pixel = RobotRecognitionCore_gridPtToCameraPixel (gridPt);
				heightWorldSpace = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].faceTopEdgeHeight;
				orientToProject = Geometry_orientSum (robotFace->faceOrient, RADS_90);
				distToProject = RobotRecognitionCore_getDistToSide (
					robotEstimate->robotIndex,
					robotFace->faceIndex,
					1); // We don't calc the face centre here, but we project a point anyway to get the slope of the hor edge

				RobotRecognitionRoughEstimation_estLocFromHorizontalEdge (
#if VERBOSE_BLOB_DETECTION
					outputFile,
#endif
					robotEstimate,
					robotFace,
					ownOrient,
					pixel,
					camVectors,
					uncertaintyConstants,
					heightWorldSpace,
					distToProject,
					orientToProject);
#if VERBOSE_BLOB_DETECTION
				printf ("estFace: rough-corner=tl est=(%f,%f) cov=(%f,%f,%f,%f)\n",
					robotFace->locEst.x, robotFace->locEst.y,
					robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w);
#endif
				fprintf (
					outputFile,
					"<FaceRoughLocation>colourId=%d perceivedRelOrient=%f cornerVisible=0 corner=\"tl\" pixel=(%d,%d) est=(%f,%f) cov=(%f,%f,%f,%f) slope=%f useXAxis=%d</FaceRoughLocation>\n",
					RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
					robotFace->perceivedFaceRelativeOrient, pixel.x, pixel.y,
					robotFace->locEst.x, robotFace->locEst.y,
					robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w,
					robotFace->slopeCameraSpace, robotFace->useCameraXAxis);
			}
			// Take point on bottom edge
			else
			{
				gridPt.x = blob->tr.x;
				gridPt.y = blob->bl.y;
				pixel = RobotRecognitionCore_gridPtToCameraPixel (gridPt);
				heightWorldSpace = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].faceEdgeBottomHeight;
				orientToProject = Geometry_orientSum (robotFace->faceOrient, -RADS_90);
				distToProject = RobotRecognitionCore_getDistToSide (
					robotEstimate->robotIndex,
					robotFace->faceIndex,
					0);
				RobotRecognitionRoughEstimation_estLocFromHorizontalEdge (
#if VERBOSE_BLOB_DETECTION
					outputFile,
#endif
					robotEstimate,
					robotFace,
					ownOrient,
					pixel,
					camVectors,
					uncertaintyConstants,
					heightWorldSpace,
					distToProject,
					orientToProject);
#if VERBOSE_BLOB_DETECTION
				printf ("estFace: rough-corner=br est=(%f,%f) cov=(%f,%f,%f,%f)\n",
					robotFace->locEst.x, robotFace->locEst.y,
					robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w);
#endif
				fprintf (
					outputFile,
					"<FaceRoughLocation>colourId=%d perceivedRelOrient=%f cornerVisible=0 corner=\"br\" pixel=(%d,%d) est=(%f,%f) cov=(%f,%f,%f,%f) slope=%f useXAxis=%d</FaceRoughLocation>\n",
					RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
					robotFace->perceivedFaceRelativeOrient, pixel.x, pixel.y,
					robotFace->locEst.x, robotFace->locEst.y,
					robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w,
					robotFace->slopeCameraSpace, robotFace->useCameraXAxis);
			}
		}
	}
	else
	{
		// Top right corner
		if (blob->tr.x != (CAM_OCCUPANCY_GRID_X - 1) && blob->tr.y != (CAM_OCCUPANCY_GRID_Y - 1))
		{
			gridPt.x = blob->tr.x;
			gridPt.y = blob->tr.y;
			pixel = RobotRecognitionCore_gridPtToCameraPixel (gridPt);
			heightWorldSpace = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].faceTopEdgeHeight;
			orientToProject = Geometry_orientSum (robotFace->faceOrient, -RADS_90);
			distToProject = RobotRecognitionCore_getDistToSide (
				robotEstimate->robotIndex,
				robotFace->faceIndex,
				0); // Want dist from robot COG to right of face
			RobotRecognitionRoughEstimation_estLocFromFaceEdge (
#if VERBOSE_BLOB_DETECTION
				outputFile,
#endif
				robotEstimate,
				robotFace,
				ownOrient,
				pixel,
				camVectors,
				uncertaintyConstants,
				heightWorldSpace,
				distToProject,
				orientToProject);
#if VERBOSE_BLOB_DETECTION
			printf ("estFace: corner=tr est=(%f,%f) cov=(%f,%f,%f,%f)\n",
				robotFace->locEst.x, robotFace->locEst.y,
				robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w);
#endif
			fprintf (
				outputFile,
				"<FaceRoughLocation>colourId=%d perceivedRelOrient=%f cornerVisible=1 corner=\"tr\" pixel=(%d,%d) est=(%f,%f) cov=(%f,%f,%f,%f) slope=%f useXAxis=%d</FaceRoughLocation>\n",
				RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
				robotFace->perceivedFaceRelativeOrient, pixel.x, pixel.y,
				robotFace->locEst.x, robotFace->locEst.y,
				robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w,
				robotFace->slopeCameraSpace, robotFace->useCameraXAxis);
		}
		// Bottom left corner
//		else if (blob->bl.x != (CAM_OCCUPANCY_GRID_X - 1) && blob->bl.y != (CAM_OCCUPANCY_GRID_Y - 1))
		else if (blob->bl.x != 0 && blob->bl.y != 0)
		{
			gridPt.x = blob->bl.x;
			gridPt.y = blob->bl.y;
			pixel = RobotRecognitionCore_gridPtToCameraPixel (gridPt);
			heightWorldSpace = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].faceEdgeBottomHeight;
			orientToProject = Geometry_orientSum (robotFace->faceOrient, RADS_90);
			distToProject = RobotRecognitionCore_getDistToSide (
				robotEstimate->robotIndex,
				robotFace->faceIndex,
				1); // Want dist from robot COG to left of face
			RobotRecognitionRoughEstimation_estLocFromFaceEdge (
#if VERBOSE_BLOB_DETECTION
				outputFile,
#endif
				robotEstimate,
				robotFace,
				ownOrient,
				pixel,
				camVectors,
				uncertaintyConstants,
				heightWorldSpace,
				distToProject,
				orientToProject);

#if VERBOSE_BLOB_DETECTION
			printf ("estFace: corner=bl est=(%f,%f) cov=(%f,%f,%f,%f)\n",
				robotFace->locEst.x, robotFace->locEst.y,
				robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w);
#endif
			fprintf (
				outputFile,
				"<FaceRoughLocation>colourId=%d perceivedRelOrient=%f cornerVisible=1 corner=\"bl\" pixel=(%d,%d) est=(%f,%f) cov=(%f,%f,%f,%f) slope=%f useXAxis=%d</FaceRoughLocation>\n",
				RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
				robotFace->perceivedFaceRelativeOrient, pixel.x, pixel.y,
				robotFace->locEst.x, robotFace->locEst.y,
				robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w,
				robotFace->slopeCameraSpace, robotFace->useCameraXAxis);
		}
		else
		{
			// Take point on top edge
			if (blob->tr.y != (CAM_OCCUPANCY_GRID_Y - 1))
			{
				gridPt.x = blob->tr.x;
				gridPt.y = blob->tr.y;
				pixel = RobotRecognitionCore_gridPtToCameraPixel (gridPt);
				heightWorldSpace = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].faceTopEdgeHeight;

				// We have edge of face furthest from viewing robot, so centre of face is
				// project towards us.
				orientToProject = Geometry_orientSum (robotFace->faceOrient, -RADS_90);
				distToProject = RobotRecognitionCore_getDistToSide (
					robotEstimate->robotIndex,
					robotFace->faceIndex,
					0);
				RobotRecognitionRoughEstimation_estLocFromHorizontalEdge (
#if VERBOSE_BLOB_DETECTION
					outputFile,
#endif
					robotEstimate,
					robotFace,
					ownOrient,
					pixel,
					camVectors,
					uncertaintyConstants,
					heightWorldSpace,
					distToProject,
					orientToProject);
#if VERBOSE_BLOB_DETECTION
				printf ("estFace: rough-corner=tr est=(%f,%f) cov=(%f,%f,%f,%f)\n",
					robotFace->locEst.x, robotFace->locEst.y,
					robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w);
#endif
				fprintf (
					outputFile,
					"<FaceRoughLocation>colourId=%d perceivedRelOrient=%f cornerVisible=0 corner=\"tr\" pixel=(%d,%d) est=(%f,%f) cov=(%f,%f,%f,%f) slope=%f useXAxis=%d</FaceRoughLocation>\n",
					RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
					robotFace->perceivedFaceRelativeOrient, pixel.x, pixel.y,
					robotFace->locEst.x, robotFace->locEst.y,
					robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w,
					robotFace->slopeCameraSpace, robotFace->useCameraXAxis);
			}
			// Take point on bottom edge
			else
			{
				gridPt.x = blob->bl.x;
				gridPt.y = blob->bl.y;
				pixel = RobotRecognitionCore_gridPtToCameraPixel (gridPt);
				heightWorldSpace = RobotRecognitionCore_robotParams[robotEstimate->robotIndex].faceEdgeBottomHeight;

				// We have edge of face nearest to viewing robot, so centre of face is
				// project away from us.
				orientToProject = Geometry_orientSum (robotFace->faceOrient, RADS_90);
				distToProject = RobotRecognitionCore_getDistToSide (
					robotEstimate->robotIndex,
					robotFace->faceIndex,
					1);
				RobotRecognitionRoughEstimation_estLocFromHorizontalEdge (
#if VERBOSE_BLOB_DETECTION
					outputFile,
#endif
					robotEstimate,
					robotFace,
					ownOrient,
					pixel,
					camVectors,
					uncertaintyConstants,
					heightWorldSpace,
					distToProject,
					orientToProject);
#if VERBOSE_BLOB_DETECTION
				printf ("estFace: rough-corner=bl est=(%f,%f) cov=(%f,%f,%f,%f)\n",
					robotFace->locEst.x, robotFace->locEst.y,
					robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w);
#endif
				fprintf (
					outputFile,
					"<FaceRoughLocation>colourId=%d perceivedRelOrient=%f cornerVisible=0 corner=\"bl\" pixel=(%d,%d) est=(%f,%f) cov=(%f,%f,%f,%f) slope=%f useXAxis=%d</FaceRoughLocation>\n",
					RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
					robotFace->perceivedFaceRelativeOrient, pixel.x, pixel.y,
					robotFace->locEst.x, robotFace->locEst.y,
					robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w,
					robotFace->slopeCameraSpace, robotFace->useCameraXAxis);
			}
		}
	}
}

/*!
Have a location estimate and a covariance for 0..2 faces. Combine these estimates
based on the magnitude of each covariance.
*/
void RobotRecognitionRoughEstimation_setupFinalEstimate (
	FILE *outputFile,
	RobotEstimate *robotEstimate,
	const FaceEstType minEstTypeForFace)
{
	UnionVector4F tempCov, combdCov;
	PointF tempEst, combdEst;
	int i;
	RobotFace *robotFace;

	robotEstimate->estType = FACE_EST_NOTHING;
	combdEst.x = 0.0f; combdEst.y = 0.0f;
	combdCov.mat[0] = 0.0f; combdCov.mat[1] = 0.0f; combdCov.mat[2] = 0.0f; combdCov.mat[3] = 0.0f;
	for (i = 0; i < 2; ++i)
	{
		// 0=f/b 1=l/r
		robotFace = &robotEstimate->robotFaces[i];

//		if (robotFace->estType == FACE_EST_CORNER || robotFace->estType == FACE_EST_EDGE)
		if (robotFace->estType >= minEstTypeForFace)
		{
			if (robotFace->estType > robotEstimate->estType)
			{
				robotEstimate->estType = robotFace->estType;
			}

			// Calculate c^-1 for this est
			tempCov = robotFace->cov;
			tempCov.matrix = MatrixF_inverse (tempCov.matrix);

			// Accumulate sum of c^-1 * x for each estimate
			tempEst = Maths_multMatrixByPoint (tempCov.mat, robotFace->locEst);
			combdEst.x += tempEst.x; combdEst.y += tempEst.y;

			// Accumulate sum of c^-1
			combdCov.vector = Maths_addMatrices (combdCov.mat, tempCov.mat);
		}
	}

	if (robotEstimate->estType != FACE_EST_NOTHING)
	{
		// Calculate (sum of c^-1)^-1
		combdCov.matrix = MatrixF_inverse (combdCov.matrix);
		robotEstimate->estCov.matrix = combdCov.matrix;

		// Calculate combined estimate: est(final) = c^-1(final) * sum(c^-1 * est)
		robotEstimate->estLoc = Maths_multMatrixByPoint (combdCov.mat, combdEst);
	}
}

void RobotRecognitionRoughEstimation_estimateRobotLocations (
	FILE *outputFile,
	CamVectors *camVectors,
	const RobotData *robotDataArray,
	UncertaintyConstants *uncertaintyConstants,
	const int ownIndex,
	List *colourBlobs,
	List *robotEstimates)
{
	int i;
	RobotEstimate *robotEstimate;
	RobotFace *robotFace;
	ListNode *iter;
	uchar tempGrid[SIZE_COOP_LOC_GRID];

	iter = robotEstimates->front;
	while (iter)
	{
		robotEstimate = (RobotEstimate*)iter->value;

		for (i = 0; i < 2; ++i)
		{
			// 0=f/b 1=l/r
			robotFace = &robotEstimate->robotFaces[i];

			if (robotFace->estType == FACE_EST_PROV_OK)
			{
				DEBUG_ASSERT (robotFace->blob)
				DEBUG_ASSERT (robotFace->blob->blobType == BLOB_VALID || robotFace->blob->blobType == BLOB_VALID_BUT_TOO_SMALL)

				RobotRecognitionRoughEstimation_calcBlobEdgeCells (
#if VERBOSE_BLOB_DETECTION
					outputFile,
#endif
					robotFace->blob,
					tempGrid);

				RobotRecognitionRoughEstimation_estimateFaceRoughLocation (
					outputFile,
					camVectors,
					robotFace,
					robotDataArray[ownIndex].pose.orient,
					uncertaintyConstants,
					robotFace->blob,
					robotEstimate);
			}
		}

		RobotRecognitionRoughEstimation_setupFinalEstimate (
			outputFile,
			robotEstimate,
			FACE_EST_EDGE);
		if (robotEstimate->estType != FACE_EST_NOTHING)
		{
#if 1 || VERBOSE_BLOB_DETECTION
			fprintf (outputFile,
				"<RobotRoughLocation>est=(%f,%f) cov=(%f,%f,%f,%f)</RobotRoughLocation>\n",
				robotEstimate->estLoc.x, robotEstimate->estLoc.y,
				robotEstimate->estCov.mat[0], robotEstimate->estCov.mat[1], robotEstimate->estCov.mat[2], robotEstimate->estCov.mat[3]);
#endif
		}

		iter = iter->next;
	}
}

#endif













