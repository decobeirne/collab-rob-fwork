/*!
\file Geometry.h
\brief Geometry calculations used by robots 
*/
#ifndef GEOMETRY_H
#define GEOMETRY_H


#include "RobotTypes.h"



//! Setup parameters for vision and movement based on parameters of robot.
void GeometryConstants_init(
	GeometryConstants *g,
	CamVectors *v,
	FILE *f,
	const int verbose);

//!Distance between two coordinates with floating point parameters
float Geometry_dist (
	const float x1,
	const float y1,
	const float x2,
	const float y2);

//! Distance squared
float Geometry_distSqrd (
	const float x1,
	const float y1,
	const float x2,
	const float y2);

PointF Geometry_lineIntersect (const PointF pt1, const PointF pt2, const float orient1, const float orient2, float *dist1, float *dist2);

//! Distance orientation between two coordinates with floating point parameters
float Geometry_orient (
	const float x1,
	const float y1,
	const float x2,
	const float y2);

//! Calculate the rotation required to get from orient1 to orient2
/*!
Has to deal with the situation where the orientation are at either side 
of 0, e.g. 10 - 350 = 20 degrees. If the absolute difference is required, then
the function can merely calculate the smallest angle between the two given
orientations. Otherwise, the direction of the angle from orient1 to orient2
must be recorded
*/
float Geometry_orientDiff (
	const float orient1,
	const float orient2,
	const int absolute);

//! Sum two orientations, maintain range between 0 and 359
float Geometry_orientSum (
	const float orient1,
	const float orient2);

//! Absolute orient, i.e. difference form 0 or 360
float Geometry_absOrient (const float orient);

float Geometry_dot (
	const PointF pt1,
	const PointF pt2);

//! Calculate the transformation of a point with floating point parameters
/*!
0 degrees is 3:00 (3 o clock), while 90 degrees is 12:00. This is in keeping with matrix 
rotation functionality and ellipse rendering
*/
void Geometry_ptFromOrient (const float sourceX,
								  const float sourceY,
								  float *destX,
								  float *destY,
								  const float dist,
								  const float orientation);





int Geometry_projectMapToRobot (
	const int cellX,
	const int cellY,
	float *dist,
	float *offset,
	const int focalX,
	const int focalY,
	const float poseOrient);

void Geometry_rotatePoint (
	const float orient,
	const float width,
	const float height,
	float* x,
	float* y);

int Geometry_isVisualContact (
	const float lensX,
	const float lensY,
	const float poseOrient,
	const float x,
	const float y,
	const float mindist,
	const float maxdist,
	const float minorient,
	const float deltaOrient);

//! Determine if point is within visual contact of camera
/*!
For new tech w calibrated cams, the pos loc should be the focal pt
All angles are in radians
*/
int Geometry_isVisualContactNEW (
	const PointF relPtMapSpace,
	const float xLimit,
	const float yPosLimit,
	const float yNegLimit,
	CamVectors *v);

void Geometry_calculateCovEllipseParams (
	const float *mat,
	float *eigVal1,
	float *eigVal2,
	PointF *eigVec1,
	PointF *eigVec2);

//! Check if an int point is within a given local map.
int Geometry_isPtOnLocalMap (const PointI loc, const PointI localMapCentre, const int internalBorder);

//! Check if an int point is withing the environment.
int Geometry_isPtInEnvironment (const PointI loc, const int internalBorder);

//! Check if an int point is within a given supervision area.
int Geometry_isPtOnSupArea (const PointI loc, const PointI centre, const int internalBorder);


#endif // ifndef


