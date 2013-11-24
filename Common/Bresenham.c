
#include "Bresenham.h"






int Bresenham_setPts (
							 const Image *img,
							 const int beginX,
							 const int beginY,
							 const int endX,
							 const int endY,
							 int *bx,
							 int *by,
							 int *dx,
							 int *dy)
{
	int isinside_begin;
	int isinside_end;
	int ex, ey;

	isinside_begin = Image_isWithinBounds_ignoreMapOrig (
		img,
		beginX,
		beginY);

	isinside_end = Image_isWithinBounds_ignoreMapOrig (
		img,
		endX,
		endY);

	if (0 == isinside_begin &&
		0 == isinside_end)
	{
		return 0;
	}

	if (0 == isinside_begin)
	{
		*bx = endX;
		*by = endY;
		ex = beginX;
		ey = beginY;
	}
	else
	{
		*bx = beginX;
		*by = beginY;
		ex = endX;
		ey = endY;
	}

	*dx = ex - *bx;
	*dy = ey - *by;

	return 1;
}

void Bresenham_setStep (int *dx,
							int *dy,
							int *xstep,
							int *ystep,
							int *axis,
							int *count)
{
	// calculate step that should be taken in x and y direction at each iteration
	if (*dx < 0)
		*xstep = -1;
	else
		*xstep = 1;

	if (*dy < 0)
		*ystep = -1;
	else
		*ystep = 1;

	// as the step is signed, the absolute value of dx and dy can be taken
	*dx = abs (*dx);
	*dy = abs (*dy);

	// check if the base axis will be x or y
	if (*dx > *dy)
		*axis = 1;
	else
		*axis = 0;

	// get number of pixels along base axis
	if (1 == *axis)
		*count = *dx;
	else
		*count = *dy;
}


void Bresenham_step (PointI *pt,
								 int *e,
								 const PointI *step,
								 const PointI *d,
								 const int axis)
{
	// perform incrementation depending on base axis
	if (1 == axis)
	{
		// increment error
		*e += d->y;

		// check if the second axis should be incremented by its step
		if ((*e) << 1 >= d->x)
		{
			pt->y += step->y;

			*e -= d->x;
		}

		// increment base axis
		pt->x += step->x;
	}
	else
	{
		*e += d->x;

		if ((*e) << 1 >= d->y)
		{
			pt->x += step->x;

			*e -= d->y;
		}

		pt->y += step->y;
	}
}
















