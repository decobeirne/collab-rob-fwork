
#include "BaseTypes.h"
#include "BitArray.h"





//*****************************************************************************
Pose initPose()
{
	Pose p;
	p.loc.x = 0.0f;
	p.loc.y = 0.0f;
	p.orient = 0.0f;
	p.mat[0] = 0.0f;
	p.mat[1] = 0.0f;
	p.mat[2] = 0.0f;
	p.mat[3] = 0.0f;
	p.evals[0] = 0.0f;
	p.evals[1] = 0.0f;
	p.evec = initPointF (0.0f);

	return p;
}

void Pose_print (const Pose *p, FILE *f)
{
	fprintf (f, "<Pose>locX=%f locY=%f orient=%f mat0=%f mat1=%f mat2=%f mat3=%f</Pose>\n",
		p->loc.x,
		p->loc.y,
		p->orient,
		p->mat[0],
		p->mat[1],
		p->mat[2],
		p->mat[3]);
}

PoseSimple initPoseSimple()
{
	PoseSimple p;
	p.loc.x = 0.0f;
	p.loc.y = 0.0f;
	p.orient = 0.0f;
	return p;
}












//*****************************************************************************
void Obstacle_ctor (void **o_)
{
	Obstacle **o = (Obstacle**)o_;
	*o = (Obstacle*)malloc (sizeof (Obstacle));
	(*o)->x = 0.0f;
	(*o)->y = 0.0f;
	(*o)->orient = 0.0f;
	(*o)->len = 0.0f;
}





//*****************************************************************************
OccupancyGrid* OccupancyGrid_alloc()
{
	OccupancyGrid *grid = (OccupancyGrid*)malloc (sizeof (OccupancyGrid));
	OccupancyGrid_init (grid);
	return grid;
}

void OccupancyGrid_init (OccupancyGrid *occupancyGrid)
{
	memset (occupancyGrid->grid, 0, SIZE_COOP_LOC_TWOBITARRAY);
}

void copyOccupancyGrid(OccupancyGrid *dest, const OccupancyGrid *source)
{
	memcpy (dest->grid, source->grid, SIZE_COOP_LOC_TWOBITARRAY);
}




