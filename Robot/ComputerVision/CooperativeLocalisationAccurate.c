#include "../../Common/RobotDefs.h"

#include "CooperativeLocalisation.h"
#include "ObstacleDetection.h"
#include "../../Common/Geometry.h"
#include "../../Common/Vector.h"



int robotColours[3][2] = {
{2, 3}, // f&b, l&r
{1, 0},
{4, 5},
};

extern int robotColours[3][2];

void CooperativeLocalisation_determineVisibleRobotFromBlobs (List *colourBlobs, VisibleRobot *closestVisibleRobot, const int ownIndex)
{
	int i, j, robotFound, closestBlob;
	ListNode *iterator, *iterator2;
	ColourBlob *blob;
	List visibleRobots;
	VisibleRobot *visibleRobot;

	visibleRobots = initList();

	// Iterate through blobs that have been detected in the image.
	iterator = colourBlobs->front;
	while (iterator)
	{
		blob = (ColourBlob*)iterator->value;

		// Iterate through robots to find matching colours.
		for (i = 0; i < 3; ++i)
		{
			for (j = 0; j < 2; ++j)
			{
				if (robotColours[i][j] == blob->colour)
				{
					blob->colourIndex = j;

					robotFound = 0;
					iterator2 = visibleRobots.front;
					while (iterator2)
					{
						visibleRobot = (VisibleRobot*)iterator2->value;

						// If there are more than 2 visible robot blobs per robot, than the
						// location estimate may be inaccurate, so the blob is marked as invalid.
						if (visibleRobot->robotIndex == i)
						{
							if (visibleRobot->blobs[0].colour == -1)
							{
								visibleRobot->blobs[0] = *blob;
							}
							else if (visibleRobot->blobs[1].colour == -1)
							{
								visibleRobot->blobs[1] = *blob;
							}
							else
							{
								printf ("Error: we should have max 2 blobs per visible robot\n");
								visibleRobot->valid = 0;
							}

							robotFound = 1;
							break;
						}

						iterator2 = iterator2->next;
					}

					if (!robotFound)
					{
						visibleRobot = (VisibleRobot*)malloc (sizeof (VisibleRobot));
						*visibleRobot = initVisibleRobot();
						visibleRobot->robotIndex = i;
						visibleRobot->blobs[0] = *blob;
						List_pushValue (&visibleRobots, visibleRobot);
					}

					// Cleaner than breaking from 2 loops.
					goto VisibleRobotFound;
				}
			}
		}

VisibleRobotFound:
		iterator = iterator->next;
	}

	// Determine closest valid robot
	closestBlob = CAM_OCCUPANCY_GRID_Y;
	closestVisibleRobot->valid = 0;

	iterator = visibleRobots.front;
	while (iterator)
	{
		visibleRobot = (VisibleRobot*)iterator->value;
		if (visibleRobot->valid)
		{
			if (visibleRobot->blobs[0].bl.y < closestBlob)
			{
				closestBlob = visibleRobot->blobs[0].bl.y;
				*closestVisibleRobot = *visibleRobot;
			}
			else if (visibleRobot->blobs[1].bl.y < closestBlob)
			{
				closestBlob = visibleRobot->blobs[1].bl.y;
				*closestVisibleRobot = *visibleRobot;
			}
		}
		iterator = iterator->next;
	}

	List_clear(&visibleRobots, 1);
}

int __getRobotVisibleSide (const PoseSimple *relPose, const int colourIndex)
{
	if (colourIndex == 0)
	{
		if (Geometry_orientDiff (0.0f, relPose->orient, 1) < PI)
		{
			// back
			return 1;
		}
		else
		{
			// front
			return 0;
		}
	}
	else
	{
		if (Geometry_orientDiff (0.0f, Geometry_orientSum (relPose->orient, PI * 0.5f), 1) < PI)
		{
			// right
			return 3;
		}
		else
		{
			// left
			return 2;
		}
	}
}

float __getRobotSideLength (const int side)
{
	float lengths[2] = {140.0f, 210.0f}; // Get measurements when in galway

	return lengths[(side == 2 || side == 3)];
}

float __getRobotSideOrientOLD (const float relRobotOrient, const int side)
{
	int sameGeneralDir = 1;
	float orient = relRobotOrient;
	if (orient > PI)
	{
		orient = 2.0f * PI - orient; // Get angle between 0 and 180
	}
	if (orient > PI * 0.5f)
	{
		sameGeneralDir = 0;
		orient = PI - orient; // Get angle between 0 and 90
	}

	// Have relative orient of other robot, now get orient of given side
	if (side == 0 || side == 1) // f/b
	{
		orient += (PI * 0.5f);
		if (orient > (PI * 0.5f)) // (This will always be true)
		{
			sameGeneralDir = 0;
			orient = PI - orient; // Again get angle between 0 and 90
		}
		else
		{
			sameGeneralDir = 1;
		}
	}

	if (!sameGeneralDir)
	{
		orient = -orient;
	}
	return orient;
}

/*
I want better blob detection:
 - a hierarchy would catch bigger areas better
 - to cut down on computation, could actually use smaller grid, then analyse blocks of smaller cells
 - this will give a better idea of contiguous areas
 - have list of large blocks that have been found
 - only append new large blocks if they include new cells

Forego this stupid step of estimating the robot location based on the rough blob bounds. There is no 
reason why edges can't be detected without this. 
 - Find *strong* edges for left, right, top, bottom (follow single-pixel rows and columns over image
 - Could then estimate the slope of edges from these before fitting hough lines (to cut down on computation)
 - Once lines are fit write function with bunch of conditions to estimate location of side
 - Combine estimates of visible sides to come up with robot loc estimate
*/

/*
use these points as seeds for hough lines.

use pts either side to restrict angle as well. start with resolution of c. 1 degree and see how slow this is

also, if we use the image and not these seed pts for hough transforms, then it wont matter if some seed pts
are anomalous

problem: top edge may actually be approaching PI/2 with ray from camera,
so these seed points will not be produced reliably
*/

/*
above note is crap.

should just get edge from response image

pick a pt, get local gradient, fit a line around this

iterate until another strong pt is found not within d of an existing pt

then just use local gradient around points to 
List_clear (&goodResponses, 1);

slight problem: not great at picking out edges at c. 70deg (ACTUALLY FINE)
slight problem: have to select seed pts from edges
	could use bresenham to filter at the estimated angle OR
	just pick a strong point, add to list, don't pick anything around this

prerequisite that would help: given loc of blob and of robot, determine what angle of
edges to expect
*/

//int __getBlobEdgeResponse (Image *responseImage, const int i, const int j, const int direction)
//{
//	int response;
//	const int wStep = responseImage->wStep;
//
//	//printf ("pt 100,20=%d pt 100,120=%d\n",
//	//	responseImage->data[100 + 20 * responseImage->wStep],
//	//	responseImage->data[100 + 120 * responseImage->wStep]);
//
//	switch (direction)
//	{
//	case 0: // Up
//		response =
// 			abs (responseImage->data[i + (j + 0) * wStep] - 0) + 
//			abs (responseImage->data[i + (j + 1) * wStep] - 0) + 
//			abs (responseImage->data[i + (j + 2) * wStep] - 0) + 
//			abs (255 - responseImage->data[i + (j + 3) * wStep]) + 
//			abs (255 - responseImage->data[i + (j + 4) * wStep]) + 
//			abs (255 - responseImage->data[i + (j + 5) * wStep]);
//		break;
//	case 1: // Down
//
//		break;
//	case 2: // Left
//
//		break;
//	case 3: // Right
//	default:
//
//
//		break;
//	}
//
//	return response;
//}

int __gridCellToPixelX (const int gridCell)
{
	return (CAM_OCCUPANCY_GRID_ORIGIN_X + (gridCell - 1) * CAM_OCCUPANCY_CELL_X + (CAM_OCCUPANCY_CELL_X / 2));
}

int __gridCellToPixelY (const int gridCell)
{
	return (CAM_OCCUPANCY_GRID_ORIGIN_Y + (gridCell - 1) * CAM_OCCUPANCY_CELL_Y + (CAM_OCCUPANCY_CELL_Y / 2));
}

float __getRobotFaceOrient (const float relRobotOrient, const int side)
{
	float orient = relRobotOrient;
	if (side == 0 || side == 1)
	{
		if (orient >= (PI * 1.5f))
		{
			orient = Geometry_orientSum (orient, -(PI * 0.5f));
		}
		else
		{
			orient = Geometry_orientSum (orient, (PI * 0.5f));
		}
	}

	// If the other robot's orient is in the lower 2 quadrant relative
	// to this robot, then reflect these orients to the upper 2 quadrants.
	if (relRobotOrient >= PI && relRobotOrient <= (PI * 1.5f))
	{
		orient -= PI;
	}
	else if (relRobotOrient > (PI * 0.5f) && relRobotOrient < PI)
	{
		orient += PI;
	}
	return orient;
}

void __estimateRobotFacePositionOLD (
	ColourBlob *blob,
	VisibleRobot *visibleRobot,
	Image *responseImage,
	const CamVectors *camVectors)
{
	PointI pt;
	PointF distEst;
	Vector3F ptInWorld;
	float dist, d1, d2, dTemp, dMin, dMax, orient;

	// Estimate whether or not edges of robot side are visible.
	blob->edgesVisible = initVector4I (1,1,1,1); // TBLR
	blob->edgesVisible.x = (blob->tr.y != CAM_OCCUPANCY_GRID_Y - 1);
	blob->edgesVisible.y = (blob->bl.y != 0);
	blob->edgesVisible.z = (blob->bl.x != 0);
	blob->edgesVisible.w = (blob->tr.x != CAM_OCCUPANCY_GRID_X - 1);

	dist = __getRobotSideLength (blob->robotSide); // Length of side
	orient = __getRobotFaceOrient (visibleRobot->relPose.orient, blob->robotSide); // Signed relative orient


	if (blob->edgesVisible.x && blob->edgesVisible.y) // Top and bottom visible
	{
		// Top pt
		pt.x = __gridCellToPixelX (blob->cog.x);
		pt.y = __gridCellToPixelY (blob->tr.y);
		ptInWorld = Vector3F_imagePtToPlane (camVectors, pt, ROBOT_FACE_TOTAL_HEIGHT);
		d1 = ptInWorld.x;

		// Bottom pt
		pt.x = __gridCellToPixelX (blob->cog.x);
		pt.y = __gridCellToPixelY (blob->bl.y);
		ptInWorld = Vector3F_imagePtToPlane (camVectors, pt, ROBOT_FACE_OFF_GROUND);
		d2 = ptInWorld.x;

		// Neither or both sides visible
		if (blob->edgesVisible.z + blob->edgesVisible.w == 2 ||
			blob->edgesVisible.z + blob->edgesVisible.w == 0)
		{
			distEst.y = (d1 + d2) * 0.5f;
		}
		else
		{
			dist = dist * cos (fabs (orient));
			if (orient < 0.0f)
			{
				// Side is not in same general dir as this robot, so take top visible pt
				dTemp = d1 - dist;
				if (dTemp > d2) // This shouldn't happen
				{
					dTemp = d2;
				}
				distEst.y = (d1 + dTemp) * 0.5f;
			}
			else
			{
				// Side is in same general dir as this robot, so take bottom visible pt
				dTemp = d2 + dist;
				if (dTemp < d1)
				{
					dTemp = d1;
				}
				distEst.y = (d1 + dTemp) * 0.5f;
			}
		}
	}
	else // Either top or bottom of blob is obscured
	{
		if (blob->edgesVisible.x) // Top visible
		{
			// Top pt
			pt.x = __gridCellToPixelX (blob->cog.x);
			pt.y = __gridCellToPixelY (blob->tr.y);
			ptInWorld = Vector3F_imagePtToPlane (camVectors, pt, ROBOT_FACE_TOTAL_HEIGHT);
			d1 = ptInWorld.x;

			// Either both or no sides obscured => pretty good estimate
			dMin = d1 - 50;
			dMax = d1 + 50;

			if (blob->edgesVisible.z + blob->edgesVisible.w == 1)
			{
			}
		}
		else // Bottom visible
		{
		}
	}
}

void __camGridIndexToPixel (const PointI orig, const PointI ptIn, PointI *ptOut)
{
	ptOut->x = orig.x + (CAM_OCCUPANCY_CELL_X / 2) + (CAM_OCCUPANCY_CELL_X * ptIn.x);
	ptOut->y = orig.y + (CAM_OCCUPANCY_CELL_Y / 2) + (CAM_OCCUPANCY_CELL_Y * ptIn.y);
}

int __canUseTopEdge (
	ColourBlob *blob,
	VisibleRobot *visibleRobot,
	const PointI orig,
	const CamVectors *camVectors,
	PointF *pt1,
	PointF *pt2)
{
	int width;
	float orient, dist;
	PointI ptInImage, ptTempI;
	Vector3F ptInWorld, ptTempF;

	// Check if blob is wide enough
	width = blob->tr.x - blob->bl.x;
	if (width < 3)
	{
		return 0;
	}

	// Check if orient in world is 'flat' enough
	orient = __getRobotFaceOrient (visibleRobot->relPose.orient, blob->robotSide); // Signed relative orient
	if (Geometry_absOrient (orient) < (PI / 6.0f)) // 30 degrees
	{
		return 0;
	}

	// Both sides visible
	if (blob->edgesVisible.z && blob->edgesVisible.w)
	{
		// Get furthest top corner of face
		if (orient < PI)
		{
			ptInImage.x = blob->bl.x; ptInImage.y = blob->tr.y;
			ptTempI.x = blob->tr.x; ptTempI.y = blob->tr.y;
		}
		else
		{
			ptInImage.x = blob->tr.x; ptInImage.y = blob->tr.y;
			ptTempI.x = blob->bl.x; ptTempI.y = blob->tr.y;
		}

		// Project furthest corner to world. Also project opposite corner to world
		// to estabish width of visible face. Given this width, project pt from first
		// corner along (the already known!!!) orient of the visible robot.
		__camGridIndexToPixel (orig, ptInImage, &ptInImage);
		__camGridIndexToPixel (orig, ptTempI, &ptTempI);
		ptInWorld = Vector3F_imagePtToPlane (camVectors, ptInImage, ROBOT_FACE_TOTAL_HEIGHT);
		ptTempF = Vector3F_imagePtToPlane (camVectors, ptTempI, ROBOT_FACE_TOTAL_HEIGHT);
		dist = Vector3F_dist (ptInWorld, ptTempF);
	}

	// Check angle in image that this edge would project to

	return 1;
}

void __estimateRobotFacePosition (
	ColourBlob *blob,
	VisibleRobot *visibleRobot,
	const PointI orig,
	Image *responseImage,
	const CamVectors *camVectors)
{
	PointF pt1, pt2;
	Vector4I edgeUsedForEst;

	// Estimate whether or not edges of robot side are visible.
	blob->edgesVisible = initVector4I (1,1,1,1); // TBLR
	blob->edgesVisible.x = (blob->tr.y != CAM_OCCUPANCY_GRID_Y - 1);
	blob->edgesVisible.y = (blob->bl.y != 0);
	blob->edgesVisible.z = (blob->bl.x != 0);
	blob->edgesVisible.w = (blob->tr.x != CAM_OCCUPANCY_GRID_X - 1);

	// As we're using very basic image processing techniques here, only estimate the robots
	// vertical position (in the image) from this blob if its width is large enough and if
	// the angle of the top/bottom edges are flat enough
	blob->canEstVertPos = 0;
	edgeUsedForEst = initVector4I (0, 0, 0, 0);

	if (blob->edgesVisible.x) // Top
	{
		if (__canUseTopEdge (blob, visibleRobot, orig, camVectors, &pt1, &pt2))
		{
			// Have 2 points (roughly on top edge) projected onto image; use these
			// to setup Hough lines



			edgeUsedForEst.w = 1;
		}
	}



}


//! Calc diff between cells of image and colour description.
void __getColourResponse (Image *img, const PointI orig, const float *colourDesc, Image *responseImg)
{
	int i, j;
	float diff;
	uchar *ptr;

	Image_fill (responseImg, 255);
	for (j = orig.y; j < (orig.y + CAM_OCCUPANCY_PIXELS_Y); ++j)
	{
		for (i = orig.x; i < (orig.x + CAM_OCCUPANCY_PIXELS_X); ++i)
		{
			Image_getPixel_dontCheck(img, i, j);
			ptr = Image_getPixelPointer(img, i, j);
			diff = abs (colourDesc[0] - ptr[0]) + abs (colourDesc[1] - ptr[1]) + abs (colourDesc[2] - ptr[2]);
			diff = min (255, diff);

			Image_setPixel_dontCheck (responseImg, i, j, diff);
		}
	}
}

#ifdef SIMULATION
#include "../../Board/OpenCV/CV.h"
#endif

extern const float robotColourDescs[N_ROBOT_COLOURS][6];

void CooperativeLocalisation_detectRobotBlobEdges (
	Image *img,
	Image *responseImg,
	const PointI orig,
	const uchar colourGrid[SIZE_COOP_LOC_GRID],
	List *colourBlobs,
	VisibleRobot *closestVisibleRobot,
	const Pose ownPose,
	const PoseSimple *groupPoses,
	const CamVectors *camVectors)
{
	ColourBlob *blob;
	const float *colourDesc;
	int i;
	//int j, k;
	//int diff;
	//uchar *ptr;
#ifdef SIMULATION
	CvSize size = {176, 143};
	IplImage *iplImage = cvCreateImage (size, 8, 1);
	int windowSize[] = {100, 10, 450, 450};
#endif

	if (closestVisibleRobot->valid)
	{
		closestVisibleRobot->relPose.loc.x = groupPoses[closestVisibleRobot->robotIndex].loc.x - ownPose.loc.x;
		closestVisibleRobot->relPose.loc.y = groupPoses[closestVisibleRobot->robotIndex].loc.y - ownPose.loc.y;
		closestVisibleRobot->relPose.orient = Geometry_orientDiff (groupPoses[closestVisibleRobot->robotIndex].orient, ownPose.orient, 0);
	}

	for (i = 0; i < 2; ++i)
	{
		blob = &closestVisibleRobot->blobs[i];
		DEBUG_ASSERT(blob->colour != -1)
		if (blob->colour != -1)
		{
			// Determine which side of robot is visible
			DEBUG_ASSERT(blob->colourIndex != -1)
			blob->robotSide = __getRobotVisibleSide (&closestVisibleRobot->relPose, blob->colourIndex);
			colourDesc = robotColourDescs[blob->colour];
			__getColourResponse (img, orig, colourDesc, responseImg);

			// Estimate location of face and orient of edges
			__estimateRobotFacePosition (blob, closestVisibleRobot, orig, responseImg, camVectors);
		}

#ifdef SIMULATION
		memcpy (iplImage->imageData, responseImg->data, size.width * size.height * 1);
		cvNamedWindow ("diff", windowSize);
		cvShowImage ("diff", iplImage);
		cvWaitKey (0);
#endif

	}


	//	memcpy (iplImage->imageData, tempImg->data, size.width * size.height * 1);
	//	cvNamedWindow ("diff", windowSize);
	//	cvShowImage ("diff", iplImage);
	//	cvWaitKey (0);

#ifdef SIMULATION
	cvReleaseImage (&iplImage);
#endif
}
