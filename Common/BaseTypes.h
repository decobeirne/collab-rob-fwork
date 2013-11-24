#ifndef BASE_TYPES_H
#define BASE_TYPES_H

#include "RobotDefs.h"
#include "Point.h"


//! Robot pose - location and covariance matrix
typedef struct PoseCov_
{
	PointF loc; //!< Robot's location.
	float orient; //!< Robot's orientation in degrees.
	float mat[4]; //!< Covariance mat.
	float evals[2]; //!< Eigenvalues of covariance matrix.
	PointF evec; //!< Eigenvector corresponding to first eigenvalue (not necessarily the largest).
} Pose;

//! Constructor
Pose initPose();

void Pose_print (const Pose *p, FILE *f);

//! Robot pose - location without covariance matrix
typedef struct PoseSimple_
{
	PointF loc;
	float orient; //!< Robot's orientation in degrees
} PoseSimple;

//! Constructor
PoseSimple initPoseSimple();





//! Obstacle object - contains list of PointI objects and bounding rectangle
typedef struct Obstacle_
{
	float x;
	float y;
	float orient;
	float len;
} Obstacle;

//! Constructor
void Obstacle_ctor (void **o);






//! Store id and util for gtep session 
typedef struct GTEPProfit_
{
	int id;
	float profit;
	int n;
} GTEPProfit;



//! Images captured by robots are descretised and evaluated
typedef struct OccupancyGrid_
{
	uchar grid[SIZE_COOP_LOC_TWOBITARRAY];
} OccupancyGrid;

//! Allocated array of uchars to store occupancy grid.
OccupancyGrid* OccupancyGrid_alloc();

void OccupancyGrid_init (OccupancyGrid *occupancyGrid);

//! Copy uchar array stored in occupancy grid from source to dest.s
void copyOccupancyGrid(OccupancyGrid *dest, const OccupancyGrid *source);






#endif
