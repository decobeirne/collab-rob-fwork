#ifndef POINT_H
#define POINT_H

#include "RobotDefs.h"


//! Cartesian coordinates - floating point
typedef struct PointF_
{
	float x;
	float y;
} PointF;

//! Cartesian coordinated - int
typedef struct PointI_
{
	int x;
	int y;
} PointI;


//! Constructor
PointF initPointF (const float f);

//! Convert
PointF PointI_toPointF (const PointI pt);

//! Rotate 2D point given rotation matrix
PointF PointF_multiplyMat (const PointF pt, const float mat[4]);

PointF PointF_rotate (const PointF pt, const float theta);






//! Constructor
PointI initPointI (const int i, const int j);

//! Convert
PointI PointF_toPointI (const PointF pt);

//! Compare to integer coordinates
int PointI_comparePtrs (const void *p1_, const void *p2_);

//! Compare to integer coordinates
int PointI_compareValues (const PointI p1, const PointI p2);

//! Calculate cell index in grid corresponding to point
PointI PointI_calcExpCellIndex (const PointI p,
							 const int cellDims);

//! Calculate cell index, ensuring that cell is within bounds of given dimensions.
PointI PointI_calcExpCellIndex_check (const PointI p,
							 const int cellDims,
							 const int envirDimsX,
							 const int envirDimsY);

//! Calculate index of loc map given its centre pt (i.e. robot's target pt)
PointI PointI_calcCellIndex (const PointI centre,
							 const int cellDims,
							 const int cellMidptDiff);

//! Get mid point of cell in nav map containing point
PointI PointI_calcNavCellMidpt (const PointI ps);

//! Calculate index of cell in obstructed grid corresponding to point
PointI PointI_calcNavCellMidptObstdCell (const PointI ps);





#endif // POINT_H
