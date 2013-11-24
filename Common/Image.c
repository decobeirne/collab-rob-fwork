

#include "Image.h"
#include "BitArray.h"


void Image_drawRect (Image *image,
						   const int bottomLeftX,
						   const int bottomLeftY,
						   const int topRightX,
						   const int topRightY,
						   const uchar colour,
						   const int isFilled)
{
	int i, j;

	if (MIN_INT32 == bottomLeftX)
	{
		return;
	}

	if (1 == isFilled)
	{
		for (j = bottomLeftY; j < topRightY; ++j)
		{	
			for (i = bottomLeftX; i < topRightX; ++i)
			{
				Image_setPixel_dontCheck (image, i, j, colour);
			}
		}
	}
	else
	{
		// bottom
		j = bottomLeftY;
		for (i = bottomLeftX; i < topRightX; ++i)
		{
			Image_setPixel_dontCheck (image, i, j, colour);
		}

		// top
		j = topRightY - 1;
		for (i = bottomLeftX; i < topRightX; ++i)
		{
			Image_setPixel_dontCheck (image, i, j, colour);
		}

		// left
		i = bottomLeftX;
		for (j = bottomLeftY; j < topRightY; ++j)
		{
			Image_setPixel_dontCheck (image, i, j, colour);
		}

		// right
		i = topRightX - 1;
		for (j = bottomLeftY; j < topRightY; ++j)
		{
			Image_setPixel_dontCheck (image, i, j, colour);
		}
	}
}

void Image_drawLine (Image *image,
							const int beginX,
							const int beginY,
							const int endX,
							const int endY,
							const uchar colour,
							const int fat)
{
	int dx;
	int dy;
	int stepX, stepY;
	int axis;
	int count;

	// make sure that the points are within the bounds of the image to avoid 
	// a run time error, this should never happen, but check anyway
	if (beginX < 0 ||
		beginX >= image->width ||
		beginY < 0 ||
		beginY >= image->height ||
		endX < 0 ||
		endX >= image->width ||
		endY < 0 ||
		endY >= image->height)
	{
		return;
	}

	// get change in x and y direction from p1 to p2
	dx = endX - beginX;
	dy = endY - beginY;

	// calculate step that should be taken in x and y direction at each iteration
	(dx < 0) ? (stepX = -1) : (stepX = 1);
	(dy < 0) ? (stepY = -1) : (stepY = 1);

	// as the step is signed, the absolute value of dx and dy can be taken
	dx = abs (dx);
	dy = abs (dy);
	
	// check if the base axis will be x or y
	(dx > dy) ? (axis = 1, count = dx) : (axis = 0, count = dy);

	Image_bresenham (
		image,
		beginX,
		beginY,
		stepX,
		stepY,
		dx,
		dy,
		axis,
		count,
		colour,fat);
}

void Image_bresenham (Image *image,
					   const int beginX,
					   const int beginY,
					   const int xStep,
					   const int yStep,
					   const int dx,
					   const int dy,
					   const int axis,
					   const int count,
					   const uchar colour,
					   const int fat)
{
	// create variables for iteration
	int x, y, e, i;

	x = beginX;
	y = beginY;
	e = 0;

	// iterate through pixels along base axis
	for (i = 0; i < count; ++i)
	{
		if (fat)
		{
			if (axis)
			{
				Image_setPixel_check (image, x, y-1, colour);
				Image_setPixel_check (image, x, y+1, colour);
			}
			else
			{
				Image_setPixel_check (image, x-1, y, colour);
				Image_setPixel_check (image, x+1, y, colour);
			}
		}
		Image_setPixel_check (image, x, y, colour);

		// perform incrementation depending on base axis
		if (axis)
		{
			// increment error
			e += dy;

			// check if the second axis should be incremented by its step
			if (e << 1	>= dx)
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

			if (e << 1	>= dy)
			{
				x += xStep;

				e -= dy;
			}

			y += yStep;
		}
	}
}

void Image_flip (uchar *data, const int vert, const int hor)
{
	int i, j;
	int h = CAM_IMG_H / 2;
	int w = CAM_IMG_W / 2;
	int wstep = CAM_IMG_W * 3;
	uchar buff[CAM_IMG_W * 3];
	uchar *ptri, *ptrj, *ptrk;
	uchar u[3];

	if (1 == vert)
	{
		for (i = 0, j = CAM_IMG_H-1; i < h; i++, j--)
		{
			ptri = data + i * wstep;
			ptrj = data + j * wstep;
			memcpy (buff, ptri, wstep);
			memcpy (ptri, ptrj, wstep);
			memcpy (ptrj, buff, wstep);
		}
	}

	if (1 == hor)
	{
		for (i = 0; i < CAM_IMG_H; ++i)
		{
			ptri = data + i * wstep;
			memcpy (buff, ptri, wstep);

			ptrj = ptri;
			ptrk = ptri + (CAM_IMG_W - 1) * 3;

			for (j = 0; j < w; j++, ptrj+=3, ptrk-=3)
			{
				u[0] = ptrj[0];
				u[1] = ptrj[1];
				u[2] = ptrj[2];
				ptrj[0] = ptrk[0];
				ptrj[1] = ptrk[1];
				ptrj[2] = ptrk[2];
				ptrk[0] = u[0];
				ptrk[1] = u[1];
				ptrk[2] = u[2];
			}
		}
	}
}

void Image_ctor (Image **image,
							   const int width,
							   const int height,
							   const int nChannels)
{
	*image = (Image*)malloc (sizeof (Image));

	(*image)->nChannels = nChannels;
	(*image)->width = width;
	(*image)->height = height;
	(*image)->wStep = (*image)->width * (*image)->nChannels;
	(*image)->orig.x = 0;
	(*image)->orig.y = 0;
	(*image)->data = (uchar*)malloc ((*image)->wStep * (*image)->height * sizeof (uchar));
}

void Image_dtor (Image **image)
{
	__try
	{
		free ((*image)->data);
	}
	__finally
	{
		//printf("MEMORY PROBLEM!!!\n");
	}
	free (*image);
}

void Image_rectify (uchar *data)
{
	int i, j;
	uchar temp[3];
	uchar *ptr = data;

	for (j = 0; j < 143; ++j)
	{
		for (i = 0; i < 176; ++i)
		{
			temp[0] = ptr[0];
			temp[1] = ptr[1];
			temp[2] = ptr[2];

			ptr[0] = temp[1];
			ptr[1] = temp[2];
			ptr[2] = temp[0];

			ptr += 3;
		}
	}
}

void Image_toHSV (uchar *data)
{
	int i;
	int camImgDataSize = 176 * 143;
	uchar *ptr;
	uchar maxColour, minColour;
	uchar delta;
	uchar hsv[3];

	i = 0;
	ptr = data;
	do
	{
		minColour = min (min (ptr[0], ptr[1]), ptr[2]);
		maxColour = max (max (ptr[0], ptr[1]), ptr[2]);

		if (maxColour == 0)
		{
			ptr[0] = 0;
			ptr[1] = 0;
			ptr[2] = 0;
		}
		else
		{
			delta = maxColour - minColour;

			// Value
			hsv[2] = maxColour;

			// Saturation
			hsv[1] = (255 * delta) / maxColour;

			// Hue
			if (0 == delta)
			{
				hsv[0] = 0;
			}
			else
			{
				if (maxColour == ptr[0])
				{
					hsv[0] = 0 + 43 * (ptr[1] - ptr[2]) / delta;
				}
				else if (maxColour == ptr[1])
				{
					hsv[0] = 85 + 43 * (ptr[2] - ptr[0]) / delta;
				}
				else
				{
					hsv[0] = 171 + 43 * (ptr[0] - ptr[1]) / delta;
				}
			}

			ptr[0] = hsv[0];
			ptr[1] = hsv[1];
			ptr[2] = hsv[2];
		}

		ptr += 3;
	}
	while (++i < camImgDataSize);
}

void Image_fill (Image *image,
				 const uchar colour)
{
	// single array of chars
	memset (
		image->data,
		colour,
		image->wStep * image->height);
}

int Image_isWithinBounds_ignoreMapOrig (const Image *image,
										const int x,
										const int y)
{
	return !(
		(x < 0 || x > (image->width - 1)) ||
		(y < 0 || y > (image->height - 1)) );
}

uchar* Image_getPixelPointer (Image *image, const int x, const int y)
{
	uchar *pixel = &(image->data[(y * image->wStep) + (x * image->nChannels)]);
	return pixel;
}

uchar Image_getPixel_check (const Image *image, const int x, const int y)
{
	if (0 == Image_isWithinBounds_ignoreMapOrig (image, x, y))
	{
		return INVALID_CELL;
	}
	return image->data[(y * image->wStep) + x];
}

uchar Image_getPixel_dontCheck (const Image *image, const int x, const int y)
{
	return image->data[(y * image->wStep) + x];
}

void Image_setPixel_check (Image *image, const int x, const int y, const uchar value)
{
	if (0 != Image_isWithinBounds_ignoreMapOrig (image, x, y))
	{
		image->data[(y * image->wStep) + x] = value;
	}
}

void Image_setPixel_dontCheck (Image *image, const int x, const int y, const uchar value)
{
	image->data[(y * image->wStep) + x] = value;
}

#ifdef IS_WIN

void Image_displayPixels (Image *image, FILE *f)
{
	int i, j;
	int h = image->height;
	int w = image->width;
	uchar *ptr = image->data;

	for (j = 0; j < h; ++j)
	{
		for (i = 0; i < w; ++i)
		{
			fprintf (f, "%3d ", *ptr);
			++ptr;
		}
		fprintf (f, "\n");
	}
}

void Image_drawGrid (uchar *data, const int origX, const int origY, const int gridY)
{
	int i, j, k, l;
	uchar *ptr;

	// Vertical
	for (i = 1; i < CAM_OCCUPANCY_GRID_X; ++i)
	{
		k = origX + i * CAM_OCCUPANCY_CELL_X;
		for (j = origY; j < origY + gridY * CAM_OCCUPANCY_CELL_Y; ++j)
		{
			ptr = data + (k * 3 + j * (CAM_IMG_W * 3));
			ptr[0] = 255;
			ptr[1] = 255;
			ptr[2] = 255;
		}
	}

	// Horizontal
	for (j = 1; j < gridY; ++j)
	{
		l = origY + j * CAM_OCCUPANCY_CELL_Y;
		for (i = origX; i < origX + CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_CELL_X; ++i)
		{
			ptr = data + (i * 3 + l * (CAM_IMG_W * 3));
			ptr[0] = 255;
			ptr[1] = 255;
			ptr[2] = 255;
		}
	}
}

//! Draw either red or blue dot in centre of grid cell indicating whether or not it is occupied
void __drawCellOccupancy (uchar *ptr, const int isMatch)
{
	if (isMatch == 1) // Red
	{
		ptr[0] = 255;ptr[1] = 0;ptr[2] = 0;ptr[3] = 255;ptr[4] = 0;ptr[5] = 0;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 255;ptr[1] = 0;ptr[2] = 0;ptr[3] = 255;ptr[4] = 0;ptr[5] = 0;
	}
	else // Blue
	{
		ptr[0] = 0;ptr[1] = 0;ptr[2] = 255;ptr[3] = 0;ptr[4] = 0;ptr[5] = 255;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 0;ptr[1] = 0;ptr[2] = 255;ptr[3] = 0;ptr[4] = 0;ptr[5] = 255;
	}
}

void Image_drawOccupancy (uchar *data, const uchar *occupancyGrid, const int origX, const int origY, const int gridY, const int desiredCategory)
{
	int i, x, y;
	uchar *ptr;
	int isMatch;

	for (i = 0; i < (CAM_OCCUPANCY_GRID_X * gridY); ++i)
	{
		y = i / 19;
		x = i % 19;
		x = origX + x * CAM_OCCUPANCY_CELL_X + CAM_OCCUPANCY_CELL_X / 2;
		y = origY + y * CAM_OCCUPANCY_CELL_Y + CAM_OCCUPANCY_CELL_Y / 2;

		ptr = data + (x * 3 + y * (CAM_IMG_W * 3));
		isMatch = (occupancyGrid[i] == desiredCategory);
		__drawCellOccupancy (ptr, isMatch);
	}
}

//! Draw red for bg, green for unknown, blue for robot
void __drawCellOccupancy2 (uchar *ptr, const uchar val)
{
	if (val == 0) // Free - red
	{
		ptr[0] = 255;ptr[1] = 0;ptr[2] = 0;ptr[3] = 255;ptr[4] = 0;ptr[5] = 0;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 255;ptr[1] = 0;ptr[2] = 0;ptr[3] = 255;ptr[4] = 0;ptr[5] = 0;
//		ptr[0] = 0;ptr[1] = 0;ptr[2] = 0;ptr[3] = 0;ptr[4] = 0;ptr[5] = 0;
//		ptr += (CAM_IMG_W * 3);
//		ptr[0] = 0;ptr[1] = 0;ptr[2] = 0;ptr[3] = 0;ptr[4] = 0;ptr[5] = 0;
	}
	else if (val == 1) // Obstacle - white
	{
		ptr[0] = 255;ptr[1] = 255;ptr[2] = 255;ptr[3] = 255;ptr[4] = 255;ptr[5] = 255;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 255;ptr[1] = 255;ptr[2] = 255;ptr[3] = 255;ptr[4] = 255;ptr[5] = 255;
	}
	else if (val == 2) // Unknown - green
	{
		ptr[0] = 0;ptr[1] = 255;ptr[2] = 0;ptr[3] = 0;ptr[4] = 255;ptr[5] = 0;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 0;ptr[1] = 255;ptr[2] = 0;ptr[3] = 0;ptr[4] = 255;ptr[5] = 0;
	}
	else // Robot - blue
	{
		ptr[0] = 0;ptr[1] = 0;ptr[2] = 255;ptr[3] = 0;ptr[4] = 0;ptr[5] = 255;
		ptr += (CAM_IMG_W * 3);
		ptr[0] = 0;ptr[1] = 0;ptr[2] = 255;ptr[3] = 0;ptr[4] = 0;ptr[5] = 255;
	}
}

void Image_drawOccupancy2 (
	uchar *data,
	const uchar *occupancyGrid,
	const int origX,
	const int origY,
	const int gridY)
{
	int i, x, y;
	uchar *imgPtr;
	const uchar *gridPtr;

	gridPtr = occupancyGrid;

	for (i = 0; i < (CAM_OCCUPANCY_GRID_X * gridY); ++i)
	{
		y = i / 19;
		x = i % 19;
		x = origX + x * CAM_OCCUPANCY_CELL_X + CAM_OCCUPANCY_CELL_X / 2;
		y = origY + y * CAM_OCCUPANCY_CELL_Y + CAM_OCCUPANCY_CELL_Y / 2;
		imgPtr = data + (x * 3 + y * (CAM_IMG_W * 3));

		__drawCellOccupancy2 (imgPtr, *gridPtr);

		++gridPtr;
	}
}

void Image_drawOccupancy3 (uchar *data, OccupancyGrid *occupancyGrid, const int origX, const int origY, const int gridY)
{
	int i, x, y;
	uchar *imgPtr;
	uchar val;

	for (i = 0; i < (CAM_OCCUPANCY_GRID_X * gridY); ++i)
	{
		y = i / 19;
		x = i % 19;
		x = origX + x * CAM_OCCUPANCY_CELL_X + CAM_OCCUPANCY_CELL_X / 2;
		y = origY + y * CAM_OCCUPANCY_CELL_Y + CAM_OCCUPANCY_CELL_Y / 2;

		val = TwoBitArray_checkIndex (occupancyGrid->grid, i);

		imgPtr = data + (x * 3 + y * (CAM_IMG_W * 3));

		__drawCellOccupancy2 (imgPtr, val);
	}
}

void Image_drawCooperativeLocalisation (uchar *data, const uchar *occupancyGrid, const uchar *colourGrid, const int origX, const int origY)
{
	int i, x, y;
	uchar *ptr;

	for (i = 0; i < (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_GRID_Y); ++i)
	{
		y = i / 19;
		x = i % 19;
		x = origX + x * CAM_OCCUPANCY_CELL_X + CAM_OCCUPANCY_CELL_X / 2;
		y = origY + y * CAM_OCCUPANCY_CELL_Y + CAM_OCCUPANCY_CELL_Y / 2;

		ptr = data + (x * 3 + y * (CAM_IMG_W * 3));
		if (occupancyGrid[i] == colourGrid[i])
		{
			// Correct
		}
		else if (occupancyGrid[i] == 20)
		{
			// False positive - red
			ptr[0] = 255;ptr[1] = 0;ptr[2] = 0;ptr[3] = 255;ptr[4] = 0;ptr[5] = 0;
			ptr += (CAM_IMG_W * 3);
			ptr[0] = 255;ptr[1] = 0;ptr[2] = 0;ptr[3] = 255;ptr[4] = 0;ptr[5] = 0;
		}
		else
		{
			// False negative - blue
			ptr[0] = 0;ptr[1] = 0;ptr[2] = 255;ptr[3] = 0;ptr[4] = 0;ptr[5] = 255;
			ptr += (CAM_IMG_W * 3);
			ptr[0] = 0;ptr[1] = 0;ptr[2] = 255;ptr[3] = 0;ptr[4] = 0;ptr[5] = 255;
		}
	}
}

void Image_drawBitArray (uchar *data,
						 const uchar *grid,
						 const int gridX,
						 const int gridY,
						 const int origX,
						 const int origY)
{
	int x, y;
	PointI pt;
	uchar *ptr;
	uchar isMatch;
	for (pt.y = 0; pt.y < gridY; ++pt.y)
	{
		for (pt.x = 0; pt.x < gridX; ++pt.x)
		{
			x = origX + pt.x * CAM_OCCUPANCY_CELL_X + CAM_OCCUPANCY_CELL_X / 2;
			y = origX + pt.y * CAM_OCCUPANCY_CELL_Y + CAM_OCCUPANCY_CELL_Y / 2;
			ptr = data + (x * 3 + y * (CAM_IMG_W * 3));

			isMatch = BitArray_checkElement_pt (grid, pt, gridX);

			__drawCellOccupancy (ptr, isMatch);
		}
	}
}

void Image_drawTwoBitArray (
	uchar *data,
	const uchar *grid,
	const int gridX,
	const int gridY,
	const int origX,
	const int origY,
	const int requiredColour)
{
	int x, y;
	PointI pt;
	uchar *ptr;
	uchar isMatch;
	for (pt.y = 0; pt.y < gridY; ++pt.y)
	{
		for (pt.x = 0; pt.x < gridX; ++pt.x)
		{
			x = origX + pt.x * CAM_OCCUPANCY_CELL_X + CAM_OCCUPANCY_CELL_X / 2;
			y = origX + pt.y * CAM_OCCUPANCY_CELL_Y + CAM_OCCUPANCY_CELL_Y / 2;
			ptr = data + (x * 3 + y * (CAM_IMG_W * 3));

			isMatch = (requiredColour == TwoBitArray_checkPt (grid, pt, gridX));

			__drawCellOccupancy (ptr, isMatch);
		}
	}
}

void Image_clone (Image *source, Image *dest)
{
	memcpy (dest->data, source->data, source->wStep * source->height);
	dest->height = source->height;
	dest->nChannels = source->nChannels;
	dest->orig = source->orig;
	dest->width = source->width;
	dest->wStep = source->wStep;
}

void Image_getIplImageFromData (Image *image,
								 IplImage **ipl)
{
	CvSize sz = {image->width, image->height};

	(*ipl) = cvCreateImage (
		sz,
		8, 
		image->nChannels);
	(*ipl)->origin = 1;
	memcpy( (*ipl)->imageData, image->data, image->wStep * image->height );
}

void Image_show_givenIplImage (Image *image,
							   const schar *winName,
							   IplImage *iplImage,
							   const int windowSize[4])
{
	memcpy( iplImage->imageData, image->data, image->wStep * image->height );
	cvNamedWindow ((const char*)winName, windowSize);
	cvShowImage ((const char*)winName, iplImage);
}

void Image_show (const Image *image,
				 const schar *winName,
				 const int windowSize[4])
{
	IplImage *iplImage;
	CvSize sz;
	sz.width = image->width;
	sz.height = image->height;

	iplImage = cvCreateImage (sz, 8, 1);

	memcpy( iplImage->imageData, image->data, image->wStep * image->height );
	cvNamedWindow ((const char*)winName, windowSize);
	cvShowImage ((const char*)winName, iplImage);
//	cvWaitKey (0);

	cvReleaseImage (&iplImage);
}

#endif // ifdef IS_WIN

