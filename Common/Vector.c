
#include "Vector.h"




//*****************************************************************************
MatrixF MatrixF_inverse (const MatrixF m)
{
	MatrixF m2;
	float d = 1 / (m.val[0] * m.val[3] - m.val[1] * m.val[2]);

	m2.val[0] = m.val[3] * d;
	m2.val[1] = -m.val[1] * d;
	m2.val[2] = -m.val[2] * d;
	m2.val[3] = m.val[0] * d;
	return m2;
}

MatrixF MatrixF_multiply (const MatrixF m1, const MatrixF m2)
{
	MatrixF m3;

	m3.val[0] = m1.val[0] * m2.val[0] + m1.val[1] * m2.val[2];
	m3.val[1] = m1.val[0] * m2.val[1] + m1.val[1] * m2.val[3];
	m3.val[2] = m1.val[2] * m2.val[0] + m1.val[3] * m2.val[2];
	m3.val[3] = m1.val[2] * m2.val[1] + m1.val[3] * m2.val[3];
	return m3;
}

void MatrixF_init(MatrixF *m1)
{
	m1->val[0] = m1->val[1] = m1->val[2] = m1->val[3] = 0;
}















//*****************************************************************************
Vector4I initEffectedArea()
{
	Vector4I v;
	v.x = MAX_INT32;
	v.y = MAX_INT32;
	v.z = MIN_INT32;
	v.w = MIN_INT32;
	return v;
}

Vector4I initVector4I (const int x_, const int y_, const int z_, const int w_)
{
	Vector4I v;
	v.x = x_;
	v.y = y_;
	v.z = z_;
	v.w = w_;
	return v;
}

Vector4F initVector4F (const float x_, const float y_, const float z_, const float w_)
{
	Vector4F v;
	v.x = x_;
	v.y = y_;
	v.z = z_;
	v.w = w_;
	return v;
}

Vector3F initVector3F (const float x, const float y, const float z)
{
	Vector3F v;
	v.x = x;
	v.y = y;
	v.z = z;
	return v;
}

Vector3F Vector3F_dirFromSlope (const float slope, const int xaxis)
{
	Vector3F v;
	if (xaxis)
	{
		v.x = 1.0f; v.y = slope; v.z = 0.0f;
	}
	else
	{
		v.x = slope; v.y = 1.0f; v.z = 0.0f;
	}
	v = Vector3F_norm (v);
	return v;
}

Vector3F Vector3F_perpVector (const Vector3F v1)
{
	Vector3F v2;
	DEBUG_ASSERT (v1.z == 0.0f)

	v2.x = -v1.y;
	v2.y = v1.x;
	v2.z = 0.0f;
	return v2;
}

Vector3F Vector3F_invertVector (const Vector3F v1)
{
	Vector3F v2;

	v2.x = -v1.x;
	v2.y = -v1.y;
	v2.z = v1.z;
	return v2;
}




#if !defined(BOARD)

void CamVectors_init (CamVectors *v)
{
//	Vector3F temp;
	float orientToRotate;

	DEBUG_ASSERT(FOCAL_LEN != MIN_FLT)

	v->_imageCentreInWorld = Vector3F_init (0.0f, 0.0f, FOCAL_LEN);

	// These are the inverse of what the actual axes should be, as we want
	// the image to be 'right way up' after it comes through the focal pt
	v->_imageXAxisInWorld = Vector3F_init (0.0f, 1.0f, 0.0f);
	v->_imageYAxisInWorld = Vector3F_init (-1.0f, 0.0f, 0.0f);

	// Assume the focal pt is at the origin first. Assume also that the robot orient is 0
	// at all times during setup. Setup the following pts as if the camera were pointing straight
	// down. Then Rotate about the y axis (if at orient 0, the robot will be pointing straight along
	// the x axis).
	orientToRotate = -(RADS_90 - CAM_THETA);
	v->_imageCentreInWorld = Vector3F_rotate (v->_imageCentreInWorld, 1, orientToRotate);
	v->_imageXAxisInWorld = Vector3F_rotate (v->_imageXAxisInWorld, 1, orientToRotate);
	v->_imageYAxisInWorld = Vector3F_rotate (v->_imageYAxisInWorld, 1, orientToRotate);
	v->_imageNormal = Vector3F_norm (v->_imageCentreInWorld);

	// imageCentreInWorld is an actual point, so we translate this to be relative to the
	// camera focal pt. The axes are vectors, i.e. directions, so their translation doesn't
	// matter
	v->focalPtAtOrigin = initVector3F (0.0f, 0.0f, CAM_HEIGHT_WORLD);
	v->_imageCentreInWorld = Vector3F_add (v->focalPtAtOrigin, v->_imageCentreInWorld);
//	temp = Vector3F_subtract (v->focalPtAtOrigin, v->_imageCentreInWorld);
//	v->_imageNormal = Vector3F_norm (temp);

	v->imageCentreInWorld = v->_imageCentreInWorld;
	v->imageXAxisInWorld = v->_imageXAxisInWorld;
	v->imageYAxisInWorld = v->_imageYAxisInWorld;
	v->imageNormal = v->_imageNormal;

	v->horizonNormal = initVector3F (0.0f, 0.0f, 1.0f);
	v->robotDir = initVector3F (1.0f, 0.0f, 0.0f); // Equivalent to orient=0

#ifdef EXPLICIT_DEBUG
	v->areCamVectorsSetup = 1;
#endif
}

#endif // !defined(BOARD)

void CamVectors_print (const CamVectors *v, FILE *f)
{
	fprintf (f, "<CamVectors>\n");
	fprintf (f, "focalPtAtOrigin=(%f,%f,%f)", v->focalPtAtOrigin.x, v->focalPtAtOrigin.y, v->focalPtAtOrigin.z);
	fprintf (f, " imageCentreInWorld=(%f,%f,%f)", v->imageCentreInWorld.x, v->imageCentreInWorld.y, v->imageCentreInWorld.z);
	fprintf (f, " imageXAxisInWorld=(%f,%f,%f)", v->imageXAxisInWorld.x, v->imageXAxisInWorld.y, v->imageXAxisInWorld.z);
	fprintf (f, " imageYAxisInWorld=(%f,%f,%f)", v->imageYAxisInWorld.x, v->imageYAxisInWorld.y, v->imageYAxisInWorld.z);
	fprintf (f, " imageNormal=(%f,%f,%f)", v->imageNormal.x, v->imageNormal.y, v->imageNormal.z);
	fprintf (f, " horizonNormal=(%f,%f,%f)\n", v->horizonNormal.x, v->horizonNormal.y, v->horizonNormal.z);
	fprintf (f, "</CamVectors>\n");
}

void CamVectors_rotateToRobotOrient (CamVectors *v, const float orient)
{
	Vector3F temp;
	const Vector3F defaultOrient = {1.0f, 0.0f, 0.0f};

	// Rotate initial values around z (up) axis by given theta
	v->imageCentreInWorld = Vector3F_rotate (v->_imageCentreInWorld, 2, orient);
	v->imageXAxisInWorld = Vector3F_rotate (v->_imageXAxisInWorld, 2, orient);
	v->imageYAxisInWorld = Vector3F_rotate (v->_imageYAxisInWorld, 2, orient);

	temp = Vector3F_subtract (v->imageCentreInWorld, v->focalPtAtOrigin);
	v->imageNormal = Vector3F_norm (temp);

	v->robotDir = Vector3F_rotate (defaultOrient, 2, orient);

#if defined(EXPLICIT_DEBUG)
	v->currentAdjustedOrient = orient;
#endif
}

#define TESTING_VECTORS 0

PointF CamVectors_projectWorldMapToImage (
	const CamVectors *camVectors,
	const Vector3F focalPtOnWorldMap,
	const Vector3F ptOnWorldMap)
{
	Vector3F ptRelToFocalPtOnWorldMap;
	Vector3F ptOnImagePlane;
	Vector3F ptInImageSpace;
	PointF imageSpaceOffsets;

	ptRelToFocalPtOnWorldMap = Vector3F_subtract (ptOnWorldMap, focalPtOnWorldMap);
	ptOnImagePlane = Vector3F_intersectPlane (
		ptRelToFocalPtOnWorldMap,
		camVectors->focalPtAtOrigin,
		camVectors->imageNormal,
		camVectors->imageCentreInWorld);
	ptInImageSpace = Vector3F_subtract (ptOnImagePlane, camVectors->imageCentreInWorld);
	imageSpaceOffsets.x = Vector3F_dot (ptInImageSpace, camVectors->imageXAxisInWorld);
	imageSpaceOffsets.y = Vector3F_dot (ptInImageSpace, camVectors->imageYAxisInWorld);

#if TESTING_VECTORS
	printf ("ptOnWorldMap=(%f,%f,%f)\n", ptOnWorldMap.x, ptOnWorldMap.y, ptOnWorldMap.z);
	printf ("focalPtOnWorldMap=(%f,%f,%f)\n", focalPtOnWorldMap.x, focalPtOnWorldMap.y, focalPtOnWorldMap.z);
	printf ("ptRelToFocalPtOnWorldMap=(%f,%f,%f)\n", ptRelToFocalPtOnWorldMap.x, ptRelToFocalPtOnWorldMap.y, ptRelToFocalPtOnWorldMap.z);
	printf ("ptOnImagePlane=(%f,%f,%f)\n", ptOnImagePlane.x, ptOnImagePlane.y, ptOnImagePlane.z);
	printf ("ptInImageSpace=(%f,%f,%f)\n", ptInImageSpace.x, ptInImageSpace.y, ptInImageSpace.z);
	printf ("imageSpaceOffsets=(%f,%f)\n", imageSpaceOffsets.x, imageSpaceOffsets.y);
	printf ("\n");
#endif

	return imageSpaceOffsets;
}

PointF CamVectors_projectWorldSpaceToImage (
	CamVectors *camVectors,
	const Vector3F ptWorldSpace)
{
	Vector3F ptOnImagePlane;
	Vector3F ptInImageSpace;
	PointF imageSpaceOffsets;

	// We are assuming that the robot's focal pt is directly above the
	// origin. CamVectors should have been adjusted for the robot's current
	// orient before calling this.
	ptOnImagePlane = Vector3F_intersectPlane (
		ptWorldSpace,
		camVectors->focalPtAtOrigin,
		camVectors->imageNormal,
		camVectors->imageCentreInWorld);
	ptInImageSpace = Vector3F_subtract (ptOnImagePlane, camVectors->imageCentreInWorld);
	imageSpaceOffsets.x = Vector3F_dot (ptInImageSpace, camVectors->imageXAxisInWorld);
	imageSpaceOffsets.y = Vector3F_dot (ptInImageSpace, camVectors->imageYAxisInWorld);

	return imageSpaceOffsets;
}

PointF CamVectors_projectMapSpaceToImage (
	CamVectors *camVectors,
	const PointF ptMapSpaceRelToFocalPt)
{
	Vector3F ptWorldSpace;
	ptWorldSpace.x = ptMapSpaceRelToFocalPt.x * MAP_SCALE;
	ptWorldSpace.y = ptMapSpaceRelToFocalPt.y * MAP_SCALE;
	ptWorldSpace.z = 0.0f;

	return CamVectors_projectWorldSpaceToImage (camVectors, ptWorldSpace);
}

Vector3F CamVectors_projectImageToWorldMap (
	const CamVectors *camVectors,
	const PointI ptInPixelSpace,
	const Vector3F focalPtOnWorldMap)
{
	const Vector3F horizonNormal = {0.0f, 0.0f, 1.0f};
	const Vector3F ptOnHorizon = {0.0f, 0.0f, 0.0f};
	Vector3F ptInImageSpace;
	Vector3F temp;
	Vector3F ptOnImagePlane;
	Vector3F ptRelToFocalPtOnWorldMap;
	Vector3F ptOnWorldMap;

	ptInImageSpace = Vector3F_multiplyScalar (camVectors->imageXAxisInWorld, (float)ptInPixelSpace.x);
	temp = Vector3F_multiplyScalar (camVectors->imageYAxisInWorld, (float)ptInPixelSpace.y);
	ptInImageSpace = Vector3F_add (ptInImageSpace, temp);
	ptOnImagePlane = Vector3F_add (ptInImageSpace, camVectors->imageCentreInWorld);
	ptRelToFocalPtOnWorldMap = Vector3F_intersectPlane (
		ptOnImagePlane,
		camVectors->focalPtAtOrigin,
		horizonNormal,
		ptOnHorizon);
	ptOnWorldMap = Vector3F_add (focalPtOnWorldMap, ptRelToFocalPtOnWorldMap);

#if TESTING_VECTORS == 1
	printf ("ptInPixelSpace=(%d,%d)\n", ptInPixelSpace.x, ptInPixelSpace.y);
	printf ("ptInImageSpace=(%f,%f,%f)\n", ptInImageSpace.x, ptInImageSpace.y, ptInImageSpace.z);
	printf ("ptOnImagePlane=(%f,%f,%f)\n", ptOnImagePlane.x, ptOnImagePlane.y, ptOnImagePlane.z);
	printf ("ptRelToFocalPtOnWorldMap=(%f,%f,%f)\n", ptRelToFocalPtOnWorldMap.x, ptRelToFocalPtOnWorldMap.y, ptRelToFocalPtOnWorldMap.z);
	printf ("ptOnWorldMap=(%f,%f,%f)\n", ptOnWorldMap.x, ptOnWorldMap.y, ptOnWorldMap.z);
	printf ("\n");
#endif

	return ptOnWorldMap;
}

Vector3F CamVectors_projectImageToHorizontalPlane (
	const CamVectors *camVectors,
	const PointF ptInPixelSpace,
	const float planeHeight)
{
	const Vector3F horizonNormal = {0.0f, 0.0f, 1.0f};
	Vector3F ptOnPlane;
	Vector3F ptInImageSpace;
	Vector3F temp;
	Vector3F ptOnImagePlane;
	Vector3F ptRelToFocalPtOnWorldMap;

	ptOnPlane = Vector3F_init (0.0f, 0.0f, planeHeight);

	ptInImageSpace = Vector3F_multiplyScalar (camVectors->imageXAxisInWorld, ptInPixelSpace.x);
	temp = Vector3F_multiplyScalar (camVectors->imageYAxisInWorld, ptInPixelSpace.y);
	ptInImageSpace = Vector3F_add (ptInImageSpace, temp);
	ptOnImagePlane = Vector3F_add (ptInImageSpace, camVectors->imageCentreInWorld);
	ptRelToFocalPtOnWorldMap = Vector3F_intersectPlane (
		ptOnImagePlane,
		camVectors->focalPtAtOrigin,
		horizonNormal,
		ptOnPlane);

	return ptRelToFocalPtOnWorldMap;
}

PointI CamVectors_imageOffsetsToPixel (
	const PointF imageSpaceOffsets)
{
	PointI pixel;
	pixel.x = CAM_IMG_W_HALF + (int)imageSpaceOffsets.x;
	pixel.y = CAM_IMG_H_HALF + (int)imageSpaceOffsets.y;
//	pixel.y = IMAGE_CENTRE_PIXEL_Y + (int)imageSpaceOffsets.y;
	return pixel;
}

PointF CamVectors_pixelToImageOffsets (
	const PointI pixel)
{
	PointF imageSpaceOffsets;
	imageSpaceOffsets.x = (float)(pixel.x - CAM_IMG_W_HALF);
	imageSpaceOffsets.y = (float)(pixel.y - CAM_IMG_H_HALF);
//	imageSpaceOffsets.y = (float)(pixel.y - IMAGE_CENTRE_PIXEL_Y);
	return imageSpaceOffsets;
}

PointF CamVectors_pixelfToImageOffsets (
	const PointF pixel)
{
	PointF imageSpaceOffsets;
	imageSpaceOffsets.x = pixel.x - CAM_IMG_W_HALF;
	imageSpaceOffsets.y = pixel.y - CAM_IMG_H_HALF;
//	imageSpaceOffsets.y = (float)(pixel.y - IMAGE_CENTRE_PIXEL_Y);
	return imageSpaceOffsets;
}



















Vector3F Vector3F_init (const int x, const int y, const int z)
{
	Vector3F v;
	v.x = x;
	v.y = y;
	v.z = z;
	return v;
}

float Vector3F_dot (const Vector3F v1, const Vector3F v2)
{
	float val = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
	return val;
}

Vector3F Vector3F_multiplyMat (const Vector3F vec, const float mat[9])
{
	Vector3F vec2;
	vec2.x = vec.x * mat[0] + vec.y * mat[1] + vec.z * mat[2];
	vec2.y = vec.x * mat[3] + vec.y * mat[4] + vec.z * mat[5];
	vec2.z = vec.x * mat[6] + vec.y * mat[7] + vec.z * mat[8];

	return vec2;
}

Vector3F Vector3F_multiplyScalar (const Vector3F vec, const float f)
{
	Vector3F vec2;
	vec2.x = vec.x * f;
	vec2.y = vec.y * f;
	vec2.z = vec.z * f;

	return vec2;
}

Vector3F Vector3F_add (const Vector3F vec1, const Vector3F vec2)
{
	Vector3F vec3;
	vec3.x = vec1.x + vec2.x;
	vec3.y = vec1.y + vec2.y;
	vec3.z = vec1.z + vec2.z;

	return vec3;
}

Vector3F Vector3F_subtract (const Vector3F vec1, const Vector3F vec2)
{
	Vector3F vec3;
	vec3.x = vec1.x - vec2.x;
	vec3.y = vec1.y - vec2.y;
	vec3.z = vec1.z - vec2.z;

	return vec3;
}

float Vector3F_len (const Vector3F vec)
{
	float len = sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
	return len;
}

Vector3F Vector3F_norm (const Vector3F vec1)
{
	float len;
	Vector3F vec2;
	len = Vector3F_len (vec1);
	vec2 = Vector3F_multiplyScalar (vec1, 1.0f / len);
	return vec2;
}

Vector3F Vector3F_rotate (const Vector3F vec, const int axis, const float theta)
{
	float mat[9];
	const float s = sin (theta);
	const float c = cos (theta);

	switch (axis)
	{
	case 0: // x axis
		mat[0] = 1.0f;		mat[1] = 0.0f;		mat[2] = 0.0f;
		mat[3] = 0.0f;		mat[4] = c;			mat[5] = -s;
		mat[6] = 0.0f;		mat[7] = s;			mat[8] = c;
		break;
	case 1: // y axis
		mat[0] = c;			mat[1] = 0.0f;		mat[2] = s;
		mat[3] = 0.0f;		mat[4] = 1.0f;		mat[5] = 0.0f;
		mat[6] = -s;		mat[7] = 0.0f;		mat[8] = c;
		break;
	case 2: // z axis
		mat[0] = c;			mat[1] = -s;		mat[2] = 0.0f;
		mat[3] = s;			mat[4] = c;			mat[5] = 0.0f;
		mat[6] = 0.0f;		mat[7] = 0.0f;		mat[8] = 1.0f;
		break;
	default:
		DEBUG_ASSERT(0)
		break;
	}

	return Vector3F_multiplyMat (vec, mat);
}

Vector3F Vector3F_intersectPlane (const Vector3F vecPt1, const Vector3F vecPt2, const Vector3F planeNormal, const Vector3F planePt)
{
	float d, f1, f2;
	Vector3F v1, v2, v3, v4;

	// If a plane is defined by the normal n, and a pt on it p0, then for any pt
	// p1 on the plane, (p1 - p0).n = 0

	// If a line is defined as a pt l0 on the line an a direction vector v, then
	// a pt on the vector l1 = l0 + dv

	// Find the pt of intersection by substituting the vector equation into the
	// line equation ((l0 + dv) - p0).n = 0
	// dv.n + (l0 - p0).n = 0
	// d = (p0 - l0).n / v.n

	// Get the direction of the line by subtracting pt1 from pt2
	v1 = Vector3F_subtract (vecPt2, vecPt1);

	// Calc v.n
	f1 = Vector3F_dot (v1, planeNormal);

	// Calc (l0 - p0).n
	v2 = Vector3F_subtract (planePt, vecPt1);
	f2 = Vector3F_dot (v2, planeNormal);

	// Calc d... (p0 - l0).n / v.n
	d = f2 / f1;

	// Return l0 + dv
	v3 = Vector3F_multiplyScalar (v1, d);
	v4 = Vector3F_add (vecPt1, v3);

	return v4;
}

float Vector3F_dist (const Vector3F pt1, const Vector3F pt2)
{
	Vector3F temp;
	temp.x = pt2.x - pt1.x;
	temp.y = pt2.y - pt1.y;
	temp.z = pt2.z - pt1.z;
	return (temp.x * temp.x + temp.y * temp.y + temp.z * temp.z);
}
















Line Line_init()
{
	Line b;
	b.axis = 0;
	b.c = 0;
	b.slope = 0.0f;
	return b;
}

LineAsVector LineAsVector_init (const float slope, const int c, const int xaxis)
{
	LineAsVector v;
	if (xaxis)
	{
		v.dir.x = 1.0f; v.dir.y = slope; v.dir.z = 0.0f;
		v.trans.x = c; v.trans.y = 0.0f; v.trans.z = 0.0f;
	}
	else
	{
		v.dir.x = slope; v.dir.y = 1.0f; v.dir.z = 0.0f;
		v.trans.x = 0.0f; v.trans.y = c; v.trans.z = 0.0f;
	}

	return v;
}



