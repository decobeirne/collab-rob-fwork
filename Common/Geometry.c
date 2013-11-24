
#include "Geometry.h"
#include "RobotCore.h"


#if 0
//! Determine camera parameters given setup data
void GeometryConstants_detCamDists (GeometryConstants *g, FILE *f)
{
	float temp;
	float theta;
	float dist;
	const int pixelsy = CAM_OCCUPANCY_GRID_Y_EXP * CAM_OCCUPANCY_CELL_Y;

	// Min dist visible within image. Get angle from P to pixel relative to camera's orientation.
	// Add camTheta, i.e. the camera's orientation.
	// tan(theta) = camHeight / dist
	// dist = camHeight / tan
	// Bottom rows of image may be junk, so discard based on constant value.
	temp = IMAGE_CENTRE_PIXEL_Y - CAM_OCCUPANCY_GRID_ORIGIN_Y;
	theta = CAM_THETA + atan(temp / FOCAL_LEN);
	g->cam_minDist = CAM_HEIGHT / tan(theta);

	// Max dist for exploring. Pixel value is nCells * cellHeight.
	// Get angle of pixel relative to the camera's orientation, and SUBTRACT angle from camTheta.
	temp = (pixelsy + CAM_OCCUPANCY_GRID_ORIGIN_Y) - IMAGE_CENTRE_PIXEL_Y;
	theta = CAM_THETA - atan(temp / FOCAL_LEN);
	g->cam_maxDist = CAM_HEIGHT / tan(theta);

	// angle at min dist
	theta = atan(g->cam_minDist / CAM_HEIGHT); // angle to nearest pt on ground
	dist = CAM_HEIGHT / cos(theta); // dist along central ray from focal pt to nearest pt on ground
	temp = (86 - CAM_OCCUPANCY_GRID_ORIGIN_X);
	theta = atan(temp / FOCAL_LEN);// angle made by ray along row in image
	dist = dist * tan(theta); // dist perp to robot direction vector
	g->cam_angleAtMinDist = atan(dist / g->cam_minDist);// angle between vector to edge pixel and robot direction vector

	// angle at max dist
	theta = atan(g->cam_maxDist / CAM_HEIGHT); // angle to furthest pt on ground
	dist = CAM_HEIGHT / cos(theta); // dist along central ray from focal pt to furthest pt on ground
	// Use same 'temp' as above
	theta = atan(temp / FOCAL_LEN);// angle made by ray along row in image
	dist = dist * tan(theta); // dist perp to robot direction vector
	g->cam_deltaAngleWithDist = atan(dist / g->cam_maxDist);// angle between vector to edge pixel and robot direction vector
	g->cam_deltaAngleWithDist = (g->cam_deltaAngleWithDist - g->cam_angleAtMinDist) / (g->cam_maxDist - g->cam_minDist);

	// Max dist that the robot can move such that it will not traverse unmapped terrain.
	// We had used nav_maxDist = g->cam_maxDist - g->cam_minDist due to artificial obstacles
	// used in simulation, but this conservatism was unnecessary in real experiments, so
	// instead the robot is permitted to travel the full cam_maxDist
	g->nav_maxDist = g->cam_maxDist - 2.0f;

	if (f)
	{
		fprintf (f, "<CamDists>minDist=%f maxDist=%f orientAtMinDist=%f orientRedWithDist=%f navMaxDist=%f</CamDists>\n",
			g->cam_minDist,
			g->cam_maxDist,
			g->cam_angleAtMinDist,
			g->cam_deltaAngleWithDist,
			g->nav_maxDist);
	}

#if defined(EXPLICIT_DEBUG)
	g->areCamDistsSetup = 1;
#endif
}
#endif

#if !defined(BOARD)

//! Determine camera parameters given setup data
void GeometryConstants_detCamDists (
	GeometryConstants *g,
	CamVectors *v)
{
	PointI ptInPixelSpace;
	Vector3F visPtOnWorldMap, focalPtOnWorldMap;
	float maxDist, minDist;

	focalPtOnWorldMap = Vector3F_init (0.0f, 0.0f, 0.0f);

	// Determine min and max visible dists to determine how far we
	// can move in one go.
	ptInPixelSpace.x = -CAM_IMG_W_HALF + CAM_OCCUPANCY_GRID_ORIGIN_X;
	ptInPixelSpace.y = -CAM_IMG_H_HALF + CAM_OCCUPANCY_GRID_ORIGIN_Y;
//	ptInPixelSpace.y = -IMAGE_CENTRE_PIXEL_Y + CAM_OCCUPANCY_GRID_ORIGIN_Y;
	visPtOnWorldMap = CamVectors_projectImageToWorldMap (v, ptInPixelSpace, focalPtOnWorldMap);
	printf ("%f,%f,%f\n", visPtOnWorldMap.x, visPtOnWorldMap.y, visPtOnWorldMap.z);
	minDist = visPtOnWorldMap.x / MAP_SCALE;
	printf ("%f\n", minDist);

	g->cam_angleAtMinDist = atan (visPtOnWorldMap.y / visPtOnWorldMap.x);
	printf ("%f\n", g->cam_angleAtMinDist);

	ptInPixelSpace.x = -CAM_IMG_W_HALF + CAM_OCCUPANCY_GRID_ORIGIN_X;
	ptInPixelSpace.y = -CAM_IMG_H_HALF + (CAM_OCCUPANCY_GRID_ORIGIN_Y + CAM_OCCUPANCY_CELL_Y * CAM_OCCUPANCY_GRID_Y);
//	ptInPixelSpace.y = -IMAGE_CENTRE_PIXEL_Y + (CAM_OCCUPANCY_GRID_ORIGIN_Y + CAM_OCCUPANCY_CELL_Y * CAM_OCCUPANCY_GRID_Y);
	visPtOnWorldMap = CamVectors_projectImageToWorldMap (v, ptInPixelSpace, focalPtOnWorldMap);
	printf ("%f,%f,%f\n", visPtOnWorldMap.x, visPtOnWorldMap.y, visPtOnWorldMap.z);
	maxDist = visPtOnWorldMap.x / MAP_SCALE;
	printf ("%f\n", maxDist);

	g->cam_minDist = minDist;
	g->cam_maxDist = maxDist;
	g->nav_maxDist = maxDist - minDist - 0.5f;
	g->nav_maxDist = min (15.0f, g->nav_maxDist);

	// Set bounds of pts in image space that are within visual contact
	g->cam_xLimit = min (
		(CAM_IMG_W_HALF - CAM_OCCUPANCY_GRID_ORIGIN_X),
		(CAM_OCCUPANCY_GRID_ORIGIN_X + (CAM_OCCUPANCY_CELL_X * CAM_OCCUPANCY_GRID_X) - CAM_IMG_W_HALF));
	g->cam_yNegLimit = (CAM_IMG_H_HALF - CAM_OCCUPANCY_GRID_ORIGIN_Y);
	g->cam_yPosLimit = (CAM_OCCUPANCY_GRID_ORIGIN_Y + (CAM_OCCUPANCY_CELL_Y * CAM_OCCUPANCY_GRID_Y) - CAM_IMG_H_HALF);
//	g->cam_yNegLimit = (IMAGE_CENTRE_PIXEL_Y - CAM_OCCUPANCY_GRID_ORIGIN_Y);
//	g->cam_yPosLimit = (CAM_OCCUPANCY_GRID_ORIGIN_Y + (CAM_OCCUPANCY_CELL_Y * CAM_OCCUPANCY_GRID_Y) - IMAGE_CENTRE_PIXEL_Y);

	g->cam_xLimit -= 0.5f;
	g->cam_yNegLimit -= 0.5f;
	g->cam_yPosLimit -= 0.5f;

#if defined(EXPLICIT_DEBUG)
	g->areCamDistsSetup = 1;
#endif
}

#define VERBOSE_EXP_DISTS 0
//! Test routine to determine min, max, optimum dists for mapping
void GeometryConstants_detExpDists (
	GeometryConstants *g,
	CamVectors *v,
	FILE *f)
{
	float i, j;
	float ptDist;
	int halfArea;
	PointF loc;
	PointF pt;
	PointF focal;
	PointF relPt;
	float orient;
	int nMapped;
	int maxNMapped;
	int nAtMin, nAtMax;
#if VERBOSE_EXP_DISTS
	int biggestI, smallestI, biggestJ, smallestJ;
#endif
	int mind = (int)g->cam_minDist - 1;
	int maxd = (int)g->cam_maxDist + 1;

	// If dist d is acceptable for mapping, then we should make sure that an acceptable
	// number of cells are mapped in the target for that distance. Therefore add x to the
	// threshold when checking.
	int mappedThreshold = (int)((EXP_AREA * EXP_AREA) * (EXP_CELL_MAPPED_THRESHOLD + 0.2f));

	g->exp_minDist = MAX_FLT;
	g->exp_maxDist = MIN_FLT;
	g->exp_optDist = MIN_FLT;
	maxNMapped = MIN_INT32;
	nAtMin = MIN_INT32;
	nAtMax = MIN_INT32;

	halfArea = EXP_AREA >> 1;

	loc.x = 0;
	loc.y = 0;
//	orient = (float)(PI * 0.5f);
	orient = 0.0f;

	CamVectors_rotateToRobotOrient (v, orient);

	Geometry_ptFromOrient (
		loc.x,
		loc.y,
		&focal.x,
		&focal.y,
		CAM_P_OFFSET,
		orient);

	for (ptDist = mind; ptDist < maxd; ++ptDist)
	{
		Geometry_ptFromOrient (
			focal.x,
			focal.y,
			&pt.x,
			&pt.y,
			ptDist,
			orient);

		nMapped = 0;

#if VERBOSE_EXP_DISTS
		biggestI = MIN_INT32;
		smallestI = MAX_INT32;
		biggestJ = MIN_INT32;
		smallestJ = MAX_INT32;
#endif

		for (i = pt.x - halfArea; i < pt.x + halfArea; ++i)
		{
			for (j = pt.y - halfArea; j < pt.y + halfArea; ++j)
			{
				relPt.x = i - focal.x;
				relPt.y = j - focal.y;

				//if (1 == Geometry_isVisualContact (
				//	focal.x,
				//	focal.y,
				//	orient,
				//	(float)i,
				//	(float)j,
				//	g->cam_minDist,
				//	g->cam_maxDist,
				//	g->cam_angleAtMinDist,
				//	g->cam_deltaAngleWithDist))
				if (1 == Geometry_isVisualContactNEW (
					relPt,
					g->cam_xLimit,
					g->cam_yNegLimit,
					g->cam_yPosLimit,
					v))
				{
					nMapped++;
#if VERBOSE_EXP_DISTS
					biggestI = max (i, biggestI);
					smallestI = min (i, smallestI);
					biggestJ = max (j, biggestJ);
					smallestJ = min (j, smallestJ);
#endif
				}
			}
		}

		if (nMapped > mappedThreshold)
		{
			if (ptDist < g->exp_minDist)
			{
				g->exp_minDist = ptDist;
				nAtMin = nMapped;
			}
			if (ptDist > g->exp_maxDist)
			{
				g->exp_maxDist = ptDist;
				nAtMax = nMapped;
			}
		}

		if (nMapped > maxNMapped)
		{
			g->exp_optDist = ptDist;
			maxNMapped = nMapped;
		}
#if VERBOSE_EXP_DISTS
		printf ("d=%f -> i=%d..%d j=%d..%d\n", ptDist, smallestI, biggestI, smallestJ, biggestJ);
#endif
	}

	if (0)
	{
		assert (maxNMapped > mappedThreshold);
	}
	else
	{
		if (maxNMapped < mappedThreshold)
		{
			printf ("WARNING: At optimum dist, this robot maps only %d mapCells.\n", maxNMapped);
			printf ("         Should be able to map %d mapCells out of %d for an expCell\n", mappedThreshold, EXP_AREA * EXP_AREA);
		}
	}

	if (f)
	{
		fprintf (f, "<ExpDists>minDist=%f maxDist=%f optDist=%f nAtMin=%d nAtMax=%d</ExpDists>\n",
			g->exp_minDist,
			g->exp_maxDist,
			g->exp_optDist,
			nAtMin,
			nAtMax);
	}

#ifdef EXPLICIT_DEBUG
	g->areExpDistsSetup = 1;
#endif
}

//! More verbose routine.
void GeometryConstants_detExpDists_verbose (
	GeometryConstants *g,
	CamVectors *v,
	FILE *f)
{
	float i, j;
	float ptDist1, ptDist2;
	float minDist1, maxDist1, minDist2, maxDist2;
	float optDist1, optDist2;
	int halfArea;
	PointF loc;
	PointF pt1, pt2;
	PointF focal;
//	PointF projPt;
	PointF relPt;
	float orient, orient2;
	int nMapped;
	int maxNMapped1, maxNMapped2;
	int nAtMin, nAtMax;
	float mind = g->cam_minDist - 1.0f;
	float maxd = g->cam_maxDist + 1.0f;
	int mappedThreshold = (int)((EXP_AREA * EXP_AREA) * (EXP_CELL_MAPPED_THRESHOLD + 0.2f));

	g->exp_minDist = MAX_FLT;
	g->exp_maxDist = MIN_FLT;
	g->exp_optDist = MIN_FLT;
	maxNMapped1 = MIN_INT32;
	nAtMin = MIN_INT32;
	nAtMax = MIN_INT32;

	halfArea = EXP_AREA >> 1;

	loc.x = 20.0f;
	loc.y = 20.0f;
	orient = (float)(PI * 0.25f);
	orient2 = Geometry_orientSum (orient, (PI * 0.5f));

	CamVectors_rotateToRobotOrient (v, orient);

	Geometry_ptFromOrient (
		loc.x,
		loc.y,
		&focal.x,
		&focal.y,
		CAM_P_OFFSET,
		orient);

	//Geometry_ptFromOrient (
	//	focal.x,
	//	focal.y,
	//	&projPt.x,
	//	&projPt.y,
	//	10.0f,
	//	Geometry_orientSum (orient, -(PI * 0.5f)));

	optDist1 = MIN_FLT;
	optDist2 = MIN_FLT;

	minDist1 = MAX_FLT;
	maxDist1 = MIN_FLT;

	printf ("total=%d threshold=%d\n", (EXP_AREA * EXP_AREA), mappedThreshold);

	for (ptDist1 = mind; ptDist1 < maxd; ptDist1 += 1.0f)
	{
		Geometry_ptFromOrient (
			focal.x,
			focal.y,
			&pt1.x,
			&pt1.y,
			ptDist1,
			orient);

		minDist2 = MAX_FLT;
		maxDist2 = MIN_FLT;
		maxNMapped2 = MIN_FLT;

		for (ptDist2 = -20.0f; ptDist2 < 20.0f; ptDist2 += 1.0f)
		{
			Geometry_ptFromOrient (
				pt1.x,
				pt1.y,
				&pt2.x,
				&pt2.y,
				ptDist2,
				orient2);

			nMapped = 0;

			for (i = pt2.x - halfArea; i < pt2.x + halfArea; i += 1.0f)
			{
				for (j = pt2.y - halfArea; j < pt2.y + halfArea; j += 1.0f)
				{
					relPt.x = i - focal.x;
					relPt.y = j - focal.y;

					if (1 == Geometry_isVisualContactNEW (
						relPt,
						g->cam_xLimit,
						g->cam_yNegLimit,
						g->cam_yPosLimit,
						v))
					{
						nMapped++;
					}
				}
			}

			if (nMapped > mappedThreshold)
			{
				printf ("d1=%12f d2=%12f n=%3d\n", ptDist1, ptDist2, nMapped);

				minDist1 = min (ptDist1, minDist1);
				maxDist1 = max (ptDist1, maxDist1);

				if (ptDist2 < minDist2)
				{
					minDist2 = ptDist2;
					nAtMin = nMapped;
				}
				if (ptDist2 > maxDist2)
				{
					maxDist2 = ptDist2;
					nAtMax = nMapped;
				}

				maxNMapped2 = max (nMapped, maxNMapped2);
			}

			if (nMapped > maxNMapped1)
			{
				optDist1 = ptDist1;
				optDist2 = ptDist2;
				maxNMapped1 = nMapped;
			}
		}

		if (minDist2 != MAX_FLT)
		{
//			printf ("dist1 %f maxNMapped %d min/max dist: %f %f (%d %d)\n", ptDist1, maxNMapped2, minDist2, maxDist2, nAtMin, nAtMax);
		}
	}

	if (optDist1 == MIN_FLT)
	{
		printf ("ERROR: no dist was found that was better than EXP_CELL_MAPPED_THRESHOLD\n");
	}
	else
	{
		printf ("Optimum nMapped: %d dists: %f, %f\n", maxNMapped1, optDist1, optDist2);
	}
}

void GeometryConstants_init(
	GeometryConstants *g,
	CamVectors *v,
	FILE *f,
	const int verbose)
{
	GeometryConstants_detCamDists (g, v);
	if (verbose)
	{
		GeometryConstants_detExpDists_verbose (g, v, f);
	}
	else
	{
		GeometryConstants_detExpDists (g, v, f);
	}

	if (f)
	{
		GeometryConstants_print (g, f);
	}
}

#endif // !defined(BOARD)

float Geometry_orientDiff (const float orient1,
							const float orient2,
							const int absolute)
{
	float diff;
	float temp1, temp2;

	// return simple absolute difference between orientations
	if (1 == absolute)
	{
		return (float)min (
			fabs (orient1 - orient2),
			min (
				fabs (orient1 - (orient2 + (PI * 2.0f))),
				fabs ((orient1 + (PI * 2.0f)) - orient2)));
	}
	// return signed difference between orientations
	else
	{
		diff = orient2 - orient1;

		// if the difference is greater than 180, then the angles are either
		// side of the origin
		if (fabs (diff) > PI)
		{
			temp1 = orient1;
			temp2 = orient2;

			// if orient1 is to the right of the origin, then add 359 to this such
			// that the shortest difference between the orientations can be calculated
			if (temp1 < (float)PI)
			{
				temp1 += (float)(PI * 2.0f);
			}
			else
			{
				temp2 += (float)(PI * 2.0f);
			}

			diff = temp2 - temp1;
		}

		return diff;
	}
}

float Geometry_orientSum (const float orient1,
						  const float orient2)
{
	float sum;

	sum = orient1 + orient2;

	if (sum >= (float)(PI * 2.0f))
	{
		sum -= (float)(PI * 2.0f);
	}
	else if (sum < 0)
	{
		sum += (float)(PI * 2.0f);
	}

	return sum;
}

float Geometry_absOrient (const float orient_)
{
	float orient = orient_;
	if (orient > (PI * 2.0f))
	{
		orient -= (PI * 2.0f);
	}
	if (orient > PI)
	{
		orient = (PI * 2.0f) - orient;
	}
	return orient;
}

//! Dot product of vector1 (pt1.x, pt1.y, 0) and vector2
float Geometry_dot (const PointF pt1, const PointF pt2)
{
	float dist;
	dist = pt1.x * pt2.x + pt1.y * pt2.y; // + pt1.z * pt2.z
	return dist;
}


//! Rotate a point on the ground plane according to the orientation to the global axes
/*!
The input parameters are the offset from the source point; {width,height}, the return
parameters are the offset of the rotated point from the source; {x,y}
*/
void Geometry_rotatePoint (const float orient,
						   const float width,
						   const float  height,
						   float* x,
						   float* y)
{
	float angle, angle2;
	float dist;
	int xNeg, yNeg;
	int quadrant;

	// convert parameter orient from degrees to radians
//	angle = orient*(float)PI/(float)180;
	angle = orient;

	// get the angle from the source point to the point to be rotated {width,height}
	angle2 = (float)atan (fabs (width / height));

	// calculate combined angle
	if (0 < width)
	{
		angle -= angle2;
	}
	else 
	{
		angle += angle2;
	}

	// ensure the angle is within the range 0..2PI
	if (angle < 0)
	{
		angle += (float)(PI * 2.0f);
	}
	else if (angle > (float)(PI * 2.0f))
	{
		angle -= (float)(PI * 2.0f);
	}

	// determine what quadrant the resultant combined angle is
	quadrant = (int)(angle / (float)(PI * 0.5f));

	xNeg = 0;
	yNeg = 0;
	switch (quadrant)
	{
	case 1:
		xNeg = 1;
		angle = (float)PI - angle;
		break;
	case 2:
		xNeg = yNeg = 1;
		angle = angle - (float)PI;
		break;
	case 3:
		yNeg = 1;
		angle = (float)(PI * 2.0f) - angle;
		break;
	}

	// calculate the offset from the source point to the rotated point
	dist = (float)fastSqrt_Bab_2 (height * height + width * width);
	*x = dist * (float)cos (angle);
	*y = dist * (float)sin (angle);

	// depending on the quadrant, set the x and y values as +ve or -ve
	if (xNeg)
		*x = -*x;
	if (yNeg)
		*y = -*y;
}


//! Determine if point is within visual contact of camera
/*!
For new tech w calibrated cams, the pos loc should be the focal pt
All angles are in radians
*/
int Geometry_isVisualContact (const float lensX,
								const float lensY,
								const float poseOrient,
								const float x,
								const float y,
								const float mindist,
								const float maxdist,
								const float minorient,
								const float deltaOrient)
{
	float dist;
	float orient;
	float orientRange;

	dist = Geometry_dist (lensX, lensY, x, y);

	orient = Geometry_orient (lensX, lensY, x, y);
	orient = fabs(Geometry_orientDiff (orient, poseOrient, 0));

	dist = dist * cos(orient);

	if (dist < mindist ||
		dist > maxdist)
	{
		return 0;
	}

	dist -= mindist;
	orientRange = minorient + dist * deltaOrient;

	if (orient > orientRange)
	{
		return 0;
	}

	return 1;
}

int Geometry_isVisualContactNEW (
	const PointF relPtMapSpace,
	const float xLimit,
	const float yPosLimit,
	const float yNegLimit,
	CamVectors *v)
{
	int isContact;
	PointF imageDists;

	imageDists = CamVectors_projectMapSpaceToImage (v, relPtMapSpace);

	isContact = (fabs (imageDists.x) < xLimit);
	isContact &= ((imageDists.y < 0.0f) | (imageDists.y < yPosLimit));
	isContact &= ((imageDists.y > 0.0f) | (-imageDists.y < yNegLimit));
	return isContact;
}


//! Calculate eigenvalues and eigenvectors for a given covariance  matrix
/*!
To get eigenvalues

AX = lambdaX
(A - lambdaI)X = 0
det(A -lambdaI) = 0
|a b|
|c d|
(a - lambda)(d - lambda) - bc = 0
lambda^2 + ad - dlambda - alambda - bc = 0
lambda^2 - (d + a)lambda + ad - bc = 0

The equation to solve this is:

    -b +/- sqrt (b^2 -4ac)
x = ----------------------
             2a

therefore:

a = 1
b = -(d + a)
c = (ad - bc)
			 
should check that a != 0, and also that b^2 != 4ac
b^2 = 4ac should only occur if the ellipse is a flat line

         (d + a) +/- sqrt ((d + a)^2 -4(ad - bc))
lambda = ---------------------------------------
                           2

To calculate eigenvectors from eigenvalues given lambda, the corresponding 
eigenvector V = [v1, v2] is calculated as:

(A - lambdaI)V = 0
|a - lambda b         | |v1|
|c          d - lambda| |v2|

(a - lambda)v1 + bv2 = 0

The equation is satisfied if:

v1 = -b and v2 = (a - lambda)

Therefore the vector should be [-b, (a - lambda)]

http://149.170.199.144/multivar/eigen.htm
*/
void Geometry_calculateCovEllipseParams (const float *mat,
										 float *eigVal1,
										 float *eigVal2,
										 PointF *eigVec1,
										 PointF *eigVec2)
{
	float temp;

	temp = (float)pow ((mat[3] + mat[0]), 2) - (4 * (mat[0] * mat[3] - mat[1] * mat[2]));
	temp = fabs (temp);
	temp = (float)fastSqrt_Bab_2 (temp);

	*eigVal1 = ((mat[3] + mat[0]) + temp) / 2;
	*eigVal2 = ((mat[3] + mat[0]) - temp) / 2;

	eigVec1->x = -mat[1];
	eigVec1->y = (mat[0] - *eigVal1);

	eigVec2->x = -mat[1];
	eigVec2->y = (mat[0] - *eigVal2);
}


// inline functions moved in here because of stupid gcc

float Geometry_dist (const float x1,
					 const float y1,
					 const float x2,
					 const float y2)
{
	return fastSqrt_Bab_2 (
		(x1 - x2) * (x1 - x2) +
		(y1 - y2) * (y1 - y2));
}

float Geometry_distSqrd (const float x1,
						 const float y1,
						 const float x2,
						 const float y2)
{
	return (float)(
		(x1 - x2) * (x1 - x2) +
		(y1 - y2) * (y1 - y2));
}

PointF Geometry_lineIntersect (const PointF pt1, const PointF pt2, const float orient1, const float orient2, float *dist1, float *dist2)
{
	PointF u, v, vPerp, w, intersect;
	float s;

	// Line = p0 + su
	Geometry_ptFromOrient (pt1.x, pt1.y, &u.x, &u.y, 1.0f, orient1);
	u.x -= pt1.x;
	u.y -= pt1.y;

	// Line = q0 + tv
	Geometry_ptFromOrient (pt2.x, pt2.y, &v.x, &v.y, 1.0f, orient2);
	v.x -= pt2.x;
	v.y -= pt2.y;

	vPerp.x = -v.y;
	vPerp.y = v.x;

	// w = p0 - q0
	w.x = pt1.x - pt2.x;
	w.y = pt1.y - pt2.y;

	// Intersection point = (w + su).vPerp = 0
	s = -(vPerp.x * w.x + vPerp.y * w.y) / (vPerp.x * u.x + vPerp.y * u.y);

	intersect.x = pt1.x + s * u.x;
	intersect.y = pt1.y + s * u.y;

	*dist1 = s;

	w.x = intersect.x - pt2.x;
	w.y = intersect.y - pt2.y;
	*dist2 = Geometry_dot (w, v);

	return intersect;
}

float Geometry_orient (const float x1,
						const float y1,
						const float x2,
						const float y2)
{
	float x, y, angle;

	x = x2 - x1;
	y = y2 - y1;

	if (0 == x && 0 == y)
	{
		return 0;
	}

	if (0 == x)
	{
		return (float)PI * ((y > 0) ? 0.5f : 1.5f); // 90, 270 degrees
	}

	angle = fabs (atan (y / x));

	if (x < 0)
	{
		if (y > 0)
		{
			angle = PI - angle;
		}
		else
		{
			angle += PI;
		}
	}
	else if (y < 0)
	{
		angle = (float)PI * 2.0f - angle;
	}
	return angle;
}

void Geometry_ptFromOrient (const float sourceX,
								  const float sourceY,
								  float *destX,
								  float *destY,
								  const float dist,
								  const float orientation)
{
	*destX = sourceX + dist * (float)cos (orientation);
	*destY = sourceY + dist * (float)sin (orientation);
}


int Geometry_isPtOnLocalMap (const PointI loc, const PointI localMapCentre, const int internalBorder)
{
	int dist = (LOC_MAP_DIMS / 2) - internalBorder;
	return (abs (localMapCentre.x - loc.x) < dist && abs (localMapCentre.y - loc.y) < dist);
}

int Geometry_isPtInEnvironment (const PointI loc, const int internalBorder)
{
	int dist = (ENVIR_DIMS / 2) - internalBorder;
	return (abs ((ENVIR_DIMS / 2) - loc.x) < dist && abs ((ENVIR_DIMS / 2) - loc.y) < dist);
}

int Geometry_isPtOnSupArea (const PointI loc, const PointI centre, const int internalBorder)
{
	int dist = (SUP_DIMS / 2) - internalBorder;
	return (abs (centre.x - loc.x) < dist && abs (centre.y - loc.y) < dist);
}

