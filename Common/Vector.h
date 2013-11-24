#ifndef VECTOR_H
#define VECTOR_H

#include "RobotDefs.h"
#include "Point.h"





//! 2x2 Matrix - floating point
typedef struct MatrixF
{
	float val[4];
} MatrixF;

//! Return inverse of matrix
MatrixF MatrixF_inverse (const MatrixF m);

//! Return product of matrices
MatrixF MatrixF_multiply (const MatrixF m1, const MatrixF m2);

//! Set matrix values to 0
void MatrixF_init(MatrixF *m1);

























//! Vector - int
typedef struct Vector4I_
{
	int x;
	int y;
	int z;
	int w;
} Vector4I;

//! Init bounding box of effected area when updating a map
Vector4I initEffectedArea();

//! Default init function
Vector4I initVector4I (const int x, const int y, const int z, const int w);

//! Vector - int
typedef struct Vector4F_
{
	float x;
	float y;
	float z;
	float w;
} Vector4F;

//! Default init function
Vector4F initVector4F (const float x, const float y, const float z, const float w);

typedef union
{
	Vector4F vector;
	float mat[4];
	MatrixF matrix;
} UnionVector4F;

//! Vector - float
typedef struct Vector3F_
{
	float x;
	float y;
	float z;
} Vector3F;

//! Constructor with parameters specified.
Vector3F initVector3F (const float x, const float y, const float z);

//! Convert slope to direction vector
Vector3F Vector3F_dirFromSlope (const float slope, const int xaxis);

//! Get perpindicular vector. Assumes z component is 0
Vector3F Vector3F_perpVector (const Vector3F v1);

//! Return vector in the opposite direction.
Vector3F Vector3F_invertVector (const Vector3F v1);

//! Pre-calculated vectors used when performing common translations between camera and world space
typedef struct CamVectors_
{
	Vector3F focalPtAtOrigin;

	// Values that may be rotated around focal pt. We record values at orient 0, and then
	// rotate these to a given robot orient
	Vector3F _imageCentreInWorld;
	Vector3F _imageNormal;
	Vector3F _imageXAxisInWorld;
	Vector3F _imageYAxisInWorld;

	Vector3F imageCentreInWorld;
	Vector3F imageNormal;
	Vector3F imageXAxisInWorld;
	Vector3F imageYAxisInWorld;
	Vector3F robotDir;

	Vector3F horizonNormal;
#ifdef EXPLICIT_DEBUG
	int areCamVectorsSetup;
#endif
#if defined(EXPLICIT_DEBUG)
	float currentAdjustedOrient;
#endif
} CamVectors;

#if !defined(BOARD)
//! Init camera vectors
void CamVectors_init (CamVectors *v);
#endif

void CamVectors_print (const CamVectors *v, FILE *stream);

//! Adjust camera vectors (using default values) given robot's orient about z (up) axis
void CamVectors_rotateToRobotOrient (CamVectors *v, const float orient);

//! Project pt at h=0 to image, given robot's focal pt in world
PointF CamVectors_projectWorldMapToImage (
	const CamVectors *camVectors,
	const Vector3F focalPtOnWorldMap,
	const Vector3F ptOnWorldMap);

//! Project pt from anywhere in world space to image, assuming focal pt above origin
PointF CamVectors_projectWorldSpaceToImage (
	CamVectors *camVectors,
	const Vector3F ptWorldSpace);

//! Convert map space (rel to focal pt) to world space and project to image.
PointF CamVectors_projectMapSpaceToImage (
	CamVectors *camVectors,
	const PointF ptMapSpaceRelToFocalPt);

//! Project pt in pixel space (origin=centreOfImage) to world map given robot's focal pt on map
Vector3F CamVectors_projectImageToWorldMap (
	const CamVectors *camVectors,
	const PointI ptInPixelSpace,
	const Vector3F focalPtOnWorldMap);

//! Project pt in pixel space (origin=centreOfImage) to plane assuming focal pt is at (above) origin
Vector3F CamVectors_projectImageToHorizontalPlane (
	const CamVectors *camVectors,
	const PointF ptInPixelSpace,
	const float planeHeight);

//! Convert pt in pixel space w origin at image centre to pixel w origin at bottom left
PointI CamVectors_imageOffsetsToPixel (
	const PointF imageSpaceOffsets);

//! Convert pixel to pt in pixel space w origin at image centre
PointF CamVectors_pixelToImageOffsets (
	const PointI pixel);

PointF CamVectors_pixelfToImageOffsets (
	const PointF pixel);










Vector3F Vector3F_init (const int x, const int y, const int z);

//! Dot product
float Vector3F_dot (const Vector3F v1, const Vector3F v2);

//! Multiply vector by 3x3 matrix
/*!
A B C * P = AP BQ CR
D E F   Q   DP EQ FR
G H I   R   GP HQ IR
*/
Vector3F Vector3F_multiplyMat (const Vector3F vec, const float mat[9]);

//! Multiply a vector by a given scalar value
Vector3F Vector3F_multiplyScalar (const Vector3F vec, const float f);

//! Add 2 const vectors
Vector3F Vector3F_add (const Vector3F vec1, const Vector3F vec2);

//! Subtract vector 2 from vector 1
Vector3F Vector3F_subtract (const Vector3F vec1, const Vector3F vec2);

//! Calculate regular length of vector.
float Vector3F_len (const Vector3F vec);

//! Return normalised vector.
Vector3F Vector3F_norm (const Vector3F vec1);

//! Rotate a vector around a given axis
Vector3F Vector3F_rotate (const Vector3F vec, const int axis, const float theta);

//! Find intersection between vector (p2 - p1) and plane (p3 - p).n = 0
/*!
p1 and p2 are points on a vector. n is the normal of a plane and p3 is a point on the plane. The aim
is to find the point pI where the line intersects the plane.

Write equation of plane: (pI - p3).n = 0

Write equation of line: pI = p1 + u(p2 - p1)

Remember that the dot product is distributive over vector addition. Therefore: n.(pI - p3) = n.pI - n.p3

The point of intersection can thus be found:
n.(p1 + u(p2 - p1) - p3) = 0
n.(p1 + u(p2 - p1) - p3) = 0
u(n.(p2 - p1)) = n.(p3 - p1)
u = n.(p3 - p1) / n.(p2 - p1)
*/
Vector3F Vector3F_intersectPlane (const Vector3F vecPt1, const Vector3F vecPt2, const Vector3F planeNormal, const Vector3F planePt);

//! Dist between 2 points in 3D
float Vector3F_dist (const Vector3F pt1, const Vector3F pt2);























//! Edge of robot face corresponding to blob
typedef struct Line_
{
	float slope;
	int c;
	int axis; //!< 0=xAxis, 1=xAxis
} Line;

//! Default ctor
Line Line_init();

typedef struct LineAsVector_
{
	Vector3F dir;
	Vector3F trans; //!< Offset of pt on line from origin
} LineAsVector;

//! Convert line to direction vector and translation vector
LineAsVector LineAsVector_init (const float slope, const int c, const int xaxis);









#endif // VECTOR_H
