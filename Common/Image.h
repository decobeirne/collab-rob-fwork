#ifndef IMAGE_H
#define IMAGE_H


#include "BaseTypes.h"
#include "Point.h"


//! Originally image wrapper class for IplImage
typedef struct Image_
{
	int width; //!< Width of image in pixels
	int height; //!< Height of image in pixels
	int nChannels; //!< N channels in image, i.e. greyscale or colour
	int wStep; //!< N items per row
	PointI orig; //!< Origin of map rel to glob coords
	uchar *data; //!< 1D uchar array
} Image;

//! Create image header and data given specific dims and n channels per pixel
/*!
This does not create an OpenCV image, but rather a uchar buffer. This buffer
can be converted into an OpenCV image for displaying
*/
void Image_ctor (Image **image,
							   const int width,
							   const int height,
							   const int nChannels);

//! Release image
void Image_dtor (Image **image);

//! Swap elements such that RGB for each pixel are in correct order
void Image_rectify (uchar *data);

//! Convert RGB to HSV (Hue, Saturation, Value)
/*!
This assumes that RGB elements are ordered correctly in the original image.

Reference: http://en.literateprograms.org/RGB_to_HSV_color_space_conversion_%28C%29
*/
void Image_toHSV (uchar *data);

//! Fill the image with a given colour passed by reference
void Image_fill (Image *image, const uchar colour);

//! Draw rectangle on images
void Image_drawRect (Image *image,
	const int bottomLeftX,
	const int bottomLeftY,
	const int topRightX,
	const int topRightY,
	const uchar colour,
	const int isFilled);

//! Draw line on image
void Image_drawLine (
	Image *image,
	const int beginX,
	const int beginY,
	const int endX,
	const int endY,
	const uchar colour,
	const int fat);

//! Bresenham line iterator
/*!
This algorithm selects only the closest pixel to the ideal line.
\param beginX The x coord of the pixel from which to start.
\param xStep The change in the x axis that should be made at each step; either 1 or -1.
\param dx The difference in pixels between beginX and the x coord of the destination pixel.
\param axis Defines the base axis off of which the line iteration operates; 1 means x asis,
0 means y axis.

See http://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html
*/
void Image_bresenham (
	Image *image,
	const int beginX,
	const int beginY,
	const int xStep,
	const int yStep,
	const int dx,
	const int dy,
	const int axis,
	const int count,
	const uchar colour,
	const int fat);

//! Check if coordinates are within image bounds
int Image_isWithinBounds_ignoreMapOrig (
	const Image *image,
	const int x,
	const int y);

//! Return pointer to pixel value
uchar* Image_getPixelPointer (Image *image, const int x, const int y);

//! Return value of pixel, checking bounds
uchar Image_getPixel_check (const Image *image, const int x, const int y);

//! Return value of pixel without checking bounds
uchar Image_getPixel_dontCheck (const Image *image, const int x, const int y);

//! Set value of pixel, check bounds
void Image_setPixel_check (Image *image, const int x, const int y, const uchar value);

//! Set value of pixel without checking bounds
void Image_setPixel_dontCheck (Image *image, const int x, const int y, const uchar value);

//! Flip image vertically and/or horizontally
void Image_flip (uchar *data, const int vert, const int hor);

#if defined (IS_WIN)
#include "../Board/OpenCV/CV.h"

void Image_displayPixels (Image *image, FILE *f);

//! Draw grid over img captured by robot's camera.
void Image_drawGrid (uchar *data, const int origX, const int origY, const int gridY);

//! Mark (un)occupied cells from training data on given image
void Image_drawOccupancy (uchar *data, const uchar *occupancyGrid, const int origX, const int origY, const int gridY, const int desiredCategory);

//! Same as previous function, but draws 0,1,2,3 for each uchar as red,white,green,blue
void Image_drawOccupancy2 (uchar *data, const uchar *occupancyGrid, const int origX, const int origY, const int gridY);

//! Same as above, draw 0,1,2,3, but use an OccupancyGrid instead of uchar*
void Image_drawOccupancy3 (uchar *data, OccupancyGrid *occupancyGrid, const int origX, const int origY, const int gridY);

//! Mark erroneous detection of robot colours on given image.
void Image_drawCooperativeLocalisation (uchar *data, const uchar *occupancyGrid, const uchar *colourGrid, const int origX, const int origY);

//! Draw given bitArray given dimensions
void Image_drawBitArray (
	uchar *data,
	const uchar *grid,
	const int gridX,
	const int gridY,
	const int origX,
	const int origY);

//! Draw given twoBitArray given dimensions and required colour (in range 0..3)
void Image_drawTwoBitArray (
	uchar *data,
	const uchar *grid,
	const int gridX,
	const int gridY,
	const int origX,
	const int origY,
	const int requiredColour);

//! Copy all parameters and data from image to image.
void Image_clone (Image *source, Image *dest);

//! Create IplImage to display from pixel buffer
void Image_getIplImageFromData (Image *image, IplImage **ipl);

//! Display image buffer given IplImage to copy it to
void Image_show_givenIplImage (
	Image *image,
	const schar *winName,
	IplImage *iplImage,
	const int windowSize[4]);

//! Very inefficient wrt memory so purely for debugging.
void Image_show (
	const Image *image,
	const schar *winName,
	const int windowSize[4]);

#endif // defined (IS_WIN)
#endif // ifndef IMAGE_H

