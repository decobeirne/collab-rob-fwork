#ifndef BRES_H
#define BRES_H

#include "RobotTypes.h"
#include "Image.h"

//! Setup begin and end points for Bresenham line iteration.
int Bresenham_setPts (
	const Image *img,
	const int beginX,
	const int beginY,
	const int endX,
	const int endY,
	int *bx,
	int *by,
	int *dx,
	int *dy);

//! Setup step values for Bresenham
void Bresenham_setStep (
	int *dx,
	int *dy,
	int *xstep,
	int *ystep,
	int *axis,
	int *count);

//! Most basic Bresenham step
void Bresenham_step (
	PointI *pt,
	int *e,
	const PointI *step,
	const PointI *d,
	const int axis);

//! Perform basic bresenham line iteration algorithm, searching for given value
/*!
This algorithm selects only the closest pixel to the ideal line. It is therefore
not suitable for obstacle detection along a line.
http://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html

beginX is the x coord of the pixel from which to start 

xStep is the change in the x axis that should be made at each step; either 1 or -1

dx is the difference in pixels between beginX and the x coord of the destination pixel

axis defines the base axis off of which the line iteration operates; 1 means x asis, 
0 means y axis
*/
int Bresenham_stepWithCondition (
	const int beginX,
	const int beginY,
	const int xStep,
	const int yStep,
	const int dx,
	const int dy,
	const int axis,
	const int count,
	const Image *img,
	int (*condition)(const uchar a, const void *b),
	const uchar searchValue,
	uchar *hitValue);

//! Perform complete bresenham line iteration algorithm, searching for given value
/*!
This algorithm selects all pixels that the ideal line passes through. It is suitable
for use in obstacle detection.
http://lifc.univ-fcomte.fr/~dedu/projects/bresenham/
*/
int Bresenham_stepCompleteWithCondition (
	const int beginX,
	const int beginY,
	const int xStep,
	const int yStep,
	const int dx,
	const int dy,
	const int axis,
	const int count,
	const Image *img,
	int (*condition)(const uchar a, const void *b),
	const uchar searchValue,
	uchar *hitValue);




#endif
