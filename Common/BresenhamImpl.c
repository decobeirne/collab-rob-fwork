
#include "Bresenham.h"
#include "Image.h"


//! Process pixel along line iterator
/*!
The coords of the pt are assumed to be already relative to the map orig
*/
int Bresenham_processPixel (
	const int x,
	const int y,
	const Image *img,
	int (*condition)(const uchar a, const void *b),
	const uchar searchValue)
{
	return condition (Image_getPixel_dontCheck (img, x, y), (void*)(&searchValue));
}

int Bresenham_manhattanDist (
						const int x1,
						 const int y1,
						 const int x2,
						 const int y2)
{
	return (abs (x2 - x1) + abs (y2 - y1));
}


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
	uchar *hitValue)
{
	// return value
	int isSearchValuePresent;
	int isWithinBounds;

	// create variables for iteration
	int x, y, e, i;
	int dist;

	x = beginX;
	y = beginY;
	e = 0;

	// iterate through pixels along base axis
	for (i = 0; i < count; ++i)
	{
		isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
			img,
			x,
			y);

		if (0 == isWithinBounds)
		{
			return 0;
		}

		isSearchValuePresent = Bresenham_processPixel (
			x,
			y,
			img,
			condition,
			searchValue);

		if (1 == isSearchValuePresent)
		{
			*hitValue = Image_getPixel_dontCheck (img, x, y);

			// even, if dist is 0, return 1 to avoid complications
			dist = Bresenham_manhattanDist (
				beginX,
				beginY,
				x,
				y);

			if (dist < 1)
			{
				dist = 1;
			}
			else if (dist > 127)
			{
				dist = 127;
			}

			return dist;
		}

		// perform incrementation depending on base axis
		if (axis)
		{
			// increment error
			e += dy;

			// check if the second axis should be incremented by its step
			if (e << 1 >= dx)
			{
				y += yStep;

				e -= dx;
			}

			// increment base axis
			x += xStep;
		}
		else
		{
			e += dx;

			if (e << 1 >= dy)
			{
				x += xStep;

				e -= dy;
			}

			y += yStep;
		}
	}

	return 0;
}


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
									uchar *hitValue)
{
	int isSearchValuePresent;
	int isWithinBounds;

	// create variables for iteration
	int x, y, e, ePrevious, i;
	int ddx, ddy;
	int dist;

	x = beginX;
	y = beginY;

	// set e for initial pixel
	if (axis == 1)
		e = dx;
	else
		e = dy;

	ePrevious = e;

	// double dx and dy
	ddx = dx << 1;
	ddy = dy << 1;

	// process first pixel
	isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
		img,
		x,
		y);

	if (0 == isWithinBounds)
	{
		return 0;
	}

	isSearchValuePresent = Bresenham_processPixel (
		x,
		y,
		img,
		condition,
		searchValue);

	if (1 == isSearchValuePresent)
	{
		*hitValue = Image_getPixel_dontCheck (img, x, y);

		// even, if dist is 0, return 1 to avoid complications
		dist = Bresenham_manhattanDist (
			beginX,
			beginY,
			x,
			y);

		if (dist < 1)
		{
			dist = 1;
		}
		else if (dist > 127)
		{
			dist = 127;
		}

		return (int)dist;
	}


	for (i = 0; i < count; ++i)
	{
		if (1 == axis)
		{
			x += xStep;

			e += ddy;

			if (e > ddx)
			{
				y += yStep;

				e -= ddx;

				if (e + ePrevious < ddx)
				{
					isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
						img,
						x,
						y-yStep);

					if (0 == isWithinBounds)
					{
						return 0;
					}

					isSearchValuePresent = Bresenham_processPixel (
						x,
						y-yStep,
						img,
						condition,
						searchValue);

					if (1 == isSearchValuePresent)
					{
						*hitValue = Image_getPixel_dontCheck (img, x, y-yStep);

						// even, if dist is 0, return 1 to avoid complications
						dist = Bresenham_manhattanDist (
							beginX,
							beginY,
							x,
							y-yStep);

						if (dist < 1)
						{
							dist = 1;
						}
						else if (dist > 127)
						{
							dist = 127;
						}

						return (int)dist;
					}
				}
				else if (e + ePrevious > ddx)
				{
					// process left square
					isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
						img,
						x-xStep,
						y);

					if (0 == isWithinBounds)
					{
						return 0;
					}

					isSearchValuePresent = Bresenham_processPixel (
						x-xStep,
						y,
						img,
						condition,
						searchValue);

					if (1 == isSearchValuePresent)
					{
						*hitValue = Image_getPixel_dontCheck (img, x-xStep, y);

						// even, if dist is 0, return 1 to avoid complications
						dist = Bresenham_manhattanDist (
							beginX,
							beginY,
							x-xStep,
							y);

						if (dist < 1)
						{
							dist = 1;
						}
						else if (dist > 127)
						{
							dist = 127;
						}

						return (int)dist;
					}
				}
				else
				{
					// process both squares
					isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
						img,
						x,
						y-yStep);

					if (0 == isWithinBounds)
					{
						return 0;
					}

					isSearchValuePresent = Bresenham_processPixel (
						x,
						y-yStep,
						img,
						condition,
						searchValue);

					if (1 == isSearchValuePresent)
					{
						*hitValue = Image_getPixel_dontCheck (img, x, y-yStep);

						// even, if dist is 0, return 1 to avoid complications
						dist = Bresenham_manhattanDist (
							beginX,
							beginY,
							x,
							y-yStep);

						if (dist < 1)
						{
							dist = 1;
						}
						else if (dist > 127)
						{
							dist = 127;
						}

						return (int)dist;
					}

					isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
						img,
						x-xStep,
						y);

					if (0 == isWithinBounds)
					{
						return 0;
					}

					isSearchValuePresent = Bresenham_processPixel (
						x-xStep,
						y,
						img,
						condition,
						searchValue);

					if (1 == isSearchValuePresent)
					{
						*hitValue = Image_getPixel_dontCheck (img, x-xStep, y);

						// even, if dist is 0, return 1 to avoid complications
						dist = Bresenham_manhattanDist (
							beginX,
							beginY,
							x-xStep,
							y);

						if (dist < 1)
						{
							dist = 1;
						}
						else if (dist > 127)
						{
							dist = 127;
						}

						return (int)dist;
					}
				}
			}
		}
		else
		{
			y += yStep;

			e += ddx;

			if (e > ddy)
			{
				x += xStep;

				e -= ddy;

				if (e + ePrevious < ddy)
				{
					isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
						img,
						x-xStep,
						y);

					if (0 == isWithinBounds)
					{
						return 0;
					}

					isSearchValuePresent = Bresenham_processPixel (
						x-xStep,
						y,
						img,
						condition,
						searchValue);

					if (1 == isSearchValuePresent)
					{
						*hitValue = Image_getPixel_dontCheck (img, x-xStep, y);

						// even, if dist is 0, return 1 to avoid complications
						dist = Bresenham_manhattanDist (
							beginX,
							beginY,
							x-xStep,
							y);

						if (dist < 1)
						{
							dist = 1;
						}
						else if (dist > 127)
						{
							dist = 127;
						}

						return (int)dist;
					}
				}
				else if (e + ePrevious > ddy)
				{
					isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
						img,
						x,
						y-yStep);

					if (0 == isWithinBounds)
					{
						return 0;
					}

					isSearchValuePresent = Bresenham_processPixel (
						x,
						y-yStep,
						img,
						condition,
						searchValue);

					if (1 == isSearchValuePresent)
					{
						*hitValue = Image_getPixel_dontCheck (img, x, y-yStep);

						// even, if dist is 0, return 1 to avoid complications
						dist = Bresenham_manhattanDist (
							beginX,
							beginY,
							x,
							y-yStep);

						if (dist < 1)
						{
							dist = 1;
						}
						else if (dist > 127)
						{
							dist = 127;
						}

						return (int)dist;
					}
				}
				else
				{
					isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
						img,
						x-xStep,
						y);

					if (0 == isWithinBounds)
					{
						return 0;
					}

					isSearchValuePresent = Bresenham_processPixel (
						x-xStep,
						y,
						img,
						condition,
						searchValue);

					if (1 == isSearchValuePresent)
					{
						*hitValue = Image_getPixel_dontCheck (img, x-xStep, y);

						// even, if dist is 0, return 1 to avoid complications
						dist = Bresenham_manhattanDist (
							beginX,
							beginY,
							x-xStep,
							y);

						if (dist < 1)
						{
							dist = 1;
						}
						else if (dist > 127)
						{
							dist = 127;
						}

						return (int)dist;
					}

					isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
						img,
						x,
						y-yStep);

					if (0 == isWithinBounds)
					{
						return 0;
					}

					isSearchValuePresent = Bresenham_processPixel (
						x,
						y-yStep,
						img,
						condition,
						searchValue);

					if (1 == isSearchValuePresent)
					{
						*hitValue = Image_getPixel_dontCheck (img, x, y-yStep);

						// even, if dist is 0, return 1 to avoid complications
						dist = Bresenham_manhattanDist (
							beginX,
							beginY,
							x,
							y-yStep);

						if (dist < 1)
						{
							dist = 1;
						}
						else if (dist > 127)
						{
							dist = 127;
						}

						return (int)dist;
					}
				}
			}
		}
		// process current pixel
		isWithinBounds = Image_isWithinBounds_ignoreMapOrig (
			img,
			x,
			y);

		if (0 == isWithinBounds)
		{
			return 0;
		}

		isSearchValuePresent = Bresenham_processPixel (
			x,
			y,
			img,
			condition,
			searchValue);

		if (1 == isSearchValuePresent)
		{
			*hitValue = Image_getPixel_dontCheck (img, x, y);

			// even, if dist is 0, return 1 to avoid complications
			dist = Bresenham_manhattanDist (
				beginX,
				beginY,
				x,
				y);

			if (dist < 1)
			{
				dist = 1;
			}
			else if (dist > 127)
			{
				dist = 127;
			}

			return (int)dist;
		}

		// set previous error for next iteration
		ePrevious = e;
	}

	return 0;
}






