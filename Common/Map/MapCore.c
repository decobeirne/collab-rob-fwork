
#include "MapCore.h"
#include "../Geometry.h"
#include "../BitArray.h"
#include "../RobotCore.h"


#if !defined(BOARD)
#ifdef PRINT_EVENTS
extern const char* behaviourHandles[];

void MapCore_logMapScan (FILE *f, const float optDist, const int nIterations, const MapScan *mapScan)
{
	PointF pt;
	Geometry_ptFromOrient (
		mapScan->pose.loc.x, mapScan->pose.loc.y, &pt.x, &pt.y,
		CAM_P_OFFSET + optDist,
		mapScan->pose.orient);

	fprintf (f, "<IntegrateScan>pose=(%f,%f,%f) localMapOrigin=(%d,%d) optimumPt=(%f,%f) estdGain=%d behaviour=\"%s\" targetIndex=%d iteration=%d</IntegrateScan>\n",
		mapScan->pose.loc.x, mapScan->pose.loc.y, mapScan->pose.orient,
		mapScan->localMapOrigin.x, mapScan->localMapOrigin.y, pt.x, pt.y,
		mapScan->estdGain, behaviourHandles[mapScan->behaviour],
		mapScan->targetIndex, nIterations);
}
#endif
#endif

void MapCore_thresholdImage (
									   Image *inputImg,
									   Image *outputImg,
									   const uchar thresholdColour,
									   const uchar occupiedColour,
									   const uchar freeColour,
									   const int thresholdBelow,
									   const int dims)
{
	int i, j;
	uchar inputPixel;
	int compareValues[2];
	uchar colourToSet[2] = {freeColour, occupiedColour};
	int isOccupied;

	for (i = 0; i < dims; ++i)
	{
		for (j = 0; j < dims; ++j)
		{
			inputPixel = Image_getPixel_dontCheck (inputImg, i, j);

			compareValues[0] = inputPixel >= thresholdColour;
			compareValues[1] = inputPixel <= thresholdColour;

			isOccupied = compareValues[thresholdBelow];

			Image_setPixel_dontCheck (outputImg, i, j, colourToSet[isOccupied]);
		}
	}
}

//! Calculate manhattan distance on a binary image
/*!
Same as function that takes a single image, but it does not alter the
input image.

http://ostermiller.org/dilate_and_erode.html
*/
void MapCore_manhattan (
	Image *inputImg,
	Image *outputImg,
	const uchar dilateColour)
{
	int i, j;
	int w, h;
	uchar val;

	w = inputImg->width;
	h = inputImg->height;

	// traverse in normal direction
	for (i = 0; i < w; ++i)
	{
		for (j = 0; j < h; ++j)
		{
			val = Image_getPixel_dontCheck (inputImg, i, j);

			// set dist to 0
			if (dilateColour == val)
			{
				val = 0;
			}
			else
			{
				// initially assign it the max value
				val = 255;

				// otherwise, it may be 1 more than the pixel below it
				if (j > 0)
				{
					val = min (
						val,
						(Image_getPixel_dontCheck (outputImg, i, j - 1) + 1));
				}
				// or to the left
				if (i > 0)
				{
					val = min (
						val,
						(Image_getPixel_dontCheck (outputImg, i - 1, j) + 1));
				}
			}

			Image_setPixel_dontCheck (outputImg, i, j, val);
		}
	}

	// traverse in opposite direction
	for (i = w - 1; i >= 0; i--)
	{
		for (j = h - 1; j >= 0; j--)
		{
			val = Image_getPixel_dontCheck (outputImg, i, j);

			// otherwise, it may be 1 more than the pixel abore it
			if (j < h - 1)
			{
				val = min (
					val,
					(Image_getPixel_dontCheck (outputImg, i, j + 1) + 1));
			}
			// or to the right
			if (i < w - 1)
			{
				val = min (
					val,
					(Image_getPixel_dontCheck (outputImg, i + 1, j) + 1));
			}

			Image_setPixel_dontCheck (outputImg, i, j, val);
		}
	}
}

void MapCore_dilateBorder (Image *inputImg, Image *outputImg,
						   const uchar broadOccupiedColour, const int broadDist)
{
	int i, j;
	uchar *ptr;

	// Bottom edge
	if (inputImg->orig.y == 0)
	{
		for (j = 0; j < broadDist; ++j)
		{
			ptr = outputImg->data + (j * outputImg->wStep);
			for (i = 0; i < LOC_MAP_DIMS; ++i, ++ptr)
			{
				if (*ptr > broadOccupiedColour){ *ptr = broadOccupiedColour; }
			}
		}
	}

	// Top edge
	if (inputImg->orig.y == ENVIR_DIMS - LOC_MAP_DIMS)
	{
		for (j = LOC_MAP_DIMS - broadDist; j < LOC_MAP_DIMS; ++j)
		{
			ptr = outputImg->data + (j * outputImg->wStep);
			for (i = 0; i < LOC_MAP_DIMS; ++i, ++ptr)
			{
				if (*ptr > broadOccupiedColour){ *ptr = broadOccupiedColour; }
			}
		}
	}

	// Left
	if (inputImg->orig.x == 0)
	{
		for (j = 0; j < LOC_MAP_DIMS; ++j)
		{
			ptr = outputImg->data + (j * outputImg->wStep);
			for (i = 0; i < broadDist; ++i, ++ptr)
			{
				if (*ptr > broadOccupiedColour){ *ptr = broadOccupiedColour; }
			}
		}
	}

	// Right
	if (inputImg->orig.x == ENVIR_DIMS - LOC_MAP_DIMS)
	{
		for (j = 0; j < LOC_MAP_DIMS; ++j)
		{
			ptr = outputImg->data + (j * outputImg->wStep) + (LOC_MAP_DIMS - broadDist);
//			for (i = (LOC_MAP_DIMS - broadDist) - 1; i < LOC_MAP_DIMS; ++i, ++ptr)
			for (i = 0; i < broadDist; ++i, ++ptr)
			{
				if (*ptr > broadOccupiedColour){ *ptr = broadOccupiedColour; }
			}
		}
	}
}

void MapCore_dilateNavMap (
											  Image *inputImg,
											  Image *outputImg,
											  const uchar occupiedColour,
											  const uchar narrowDilate,
											  const uchar broadDilate,
											  const uchar freeColour,
											  const int narrowDist,
											  const int broadDist)
{
	uchar val;
	uchar *ptr;

	MapCore_manhattan (inputImg, outputImg, occupiedColour);

	ptr = outputImg->data;
	ptr += (outputImg->width * outputImg->height) - 1;

	do
	{
		val = *ptr;

		if (val == 0)
		{
			*ptr = occupiedColour;
		}
		else if (val < narrowDist)
		{
			*ptr = narrowDilate;
		}
		else if (val < broadDist)
		{
			*ptr = broadDilate;
		}
		else
		{
			*ptr = freeColour;
		}
	}
	while (--ptr >= outputImg->data);
}

#if defined (RAND_ENVIR) && defined(ROBOT)
void genRandEnvir (List *origObstList)
{
	int i;
	PointI pt;
	int orient;
	int len;
	Obstacle *obst;
	int dens;

	dens = (int)(DENSITY * (ENVIR_DIMS / 100) * (ENVIR_DIMS / 100));

	for (i = 0; i < dens; ++i)
	{
		Obstacle_ctor ((void**)&obst);

		pt.x =			rand() % (ENVIR_DIMS / 2);
		pt.x *= 2;
		pt.y =			rand() % (ENVIR_DIMS / 2);
		pt.y *= 2;
		orient =		rand() % 12;
		orient *= (int)(PI / 6.0f);
		len =			rand() % 70;
		len += 5;

		obst->x = pt.x;
		obst->y = pt.y;
		obst->orient = orient;
		obst->len = len;

		List_pushValue (origObstList, obst);
	}
}
#endif

//! Calculate mapped value (n cells mapped) in supervision area
/*!
In our implementation, supervision areas are designated as an area of 2x the dimensions of a local
map and therefore 4x the area. Local maps are non-overlapping, therefore the value for each supervision
map simply be summed from the local maps over the same area. As with local maps, supervision areas
overlap by half their dimensions.
*/
int calcSupAreaVal (
					const int i,
					const int j,
					__int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS])
{
	int celli, cellj;
/*	celli = i * 2;
	cellj = j * 2;

	return (int)(
		localMapGrid[celli    ][cellj    ] +
		localMapGrid[celli + 2][cellj    ] +
		localMapGrid[celli    ][cellj + 2] +
		localMapGrid[celli + 2][cellj + 2]);*/

	celli = i * 3;
	cellj = j * 3;

	return (int)(
		localMapGrid[celli    ][cellj    ] +
		localMapGrid[celli    ][cellj + 2] +
		localMapGrid[celli    ][cellj + 4] +
		localMapGrid[celli + 2][cellj    ] +
		localMapGrid[celli + 2][cellj + 2] +
		localMapGrid[celli + 2][cellj + 4] +
		localMapGrid[celli + 4][cellj    ] +
		localMapGrid[celli + 4][cellj + 2] +
		localMapGrid[celli + 4][cellj + 4]);
}

void MapCore_printSupGridCells (FILE *f,
								__int16 supGrid[SUP_GRID_DIMS][SUP_GRID_DIMS])
{
	int i, j;

	fprintf (f, "<SupGrid>\n");
	fprintf (f, "      ");
	for (i = 0; i < SUP_GRID_DIMS; ++i)
	{
		fprintf (f, "%6d", i);
	}
	fprintf (f, "\n");

	for (j = SUP_GRID_DIMS - 1; j >= 0; --j)
	{
		fprintf (f, "%6d", j);
		for (i = 0; i < SUP_GRID_DIMS; ++i)
		{
			fprintf (f, "%6d", supGrid[i][j]);
		}
		fprintf (f, "\n");
	}
	fprintf (f, "</SupGrid>\n");
}

void MapCore_updateSupGridCells (FILE *f,
								 __int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS],
								 __int16 supGrid[SUP_GRID_DIMS][SUP_GRID_DIMS])
{
	int i, j;
	for (j = SUP_GRID_DIMS - 1; j >= 0; --j)
	{
		for (i = 0; i < SUP_GRID_DIMS; ++i)
		{
			supGrid[i][j] = calcSupAreaVal (i, j, localMapGrid);
		}
	}
}

int MapCore_isPtWithinLocalMap (
										   const int robx,
										   const int roby,
										   const PointI localMapCentrePt,
										   const int localMapRadius)
{
	if ((abs (robx - localMapCentrePt.x) < localMapRadius) &&
		(abs (roby - localMapCentrePt.y) < localMapRadius))
	{
		return 1;
	}

	return 0;
}

void MapCore_copyMap_greaterCertainty (Image *src, Image *dest)
{
	int i, j;
	int width = src->width;
	int height = src->height;
	uchar srcVal, destVal;

	for (i = 0; i < width; ++i)
	{
		for (j = 0; j < height; ++j)
		{
			srcVal = Image_getPixel_dontCheck (src, i, j);
			destVal = Image_getPixel_dontCheck (dest, i, j);

			if (abs (srcVal - 127) > abs (destVal - 127))
			{
				Image_setPixel_dontCheck (dest, i, j, srcVal);
			}
		}
	}
}

void MapCore_compressLocalMap (Image *image, CompressedImage *compressed)
{
	int len;
	uchar val;
	int i;
	int sz;
	uchar *ptr;
	uchar *bufferPtr;
	int bufferUsed;

	ptr = image->data;
	sz = image->width * image->height;

	bufferPtr = compressed->buffer;
	bufferUsed = 0;

	// Start encoding with first pixel.
	val = *ptr;
	len = 1;
	++ptr;
	i = 1;

	do
	{
		if (*ptr != val || len == 255)
		{
			*bufferPtr = val;
			++bufferPtr;
			*bufferPtr = (uchar)len;
			++bufferPtr;

			bufferUsed += 2;

			val = *ptr;
			len = 0;
		}

		++len;
		++ptr;
	} while (++i != sz);

	{
		*bufferPtr = val;
		++bufferPtr;
		*bufferPtr = (uchar)len;
		++bufferPtr;

		bufferUsed += 2;
	}

	compressed->usedSize = bufferUsed;
	compressed->orig = image->orig;
}

void MapCore_decompressLocalMap (Image *image, CompressedImage *compressed)
{
	int bufferUsed;
	uchar *bufferPtr;
	uchar *ptr;
	uchar val;
	int len;

	bufferUsed = 0;
	bufferPtr = compressed->buffer;
	ptr = image->data;

	do
	{
		val = *bufferPtr;
		++bufferPtr;
		len = (int)*bufferPtr;
		++bufferPtr;
		bufferUsed += 2;

		memset (ptr, val, len);
		ptr += len;
	}
	while (bufferUsed < compressed->usedSize);

	image->orig = compressed->orig; // Comment this back in after 0124
}

#ifndef IS_GUMSTIX
void Obstacle_setup (Obstacle *obst, const float x, const float y, const float orient, const float len)
{
	obst->x = x;
	obst->y = y;
	obst->orient = orient;
	obst->len = len;
}

#define OBSTC Obstacle_ctor ((void**)&obst);
#define OBSTP List_pushValue (obstacleList, obst);

void MapCore_setupObstacles (List *obstacleList)
{
#ifdef RAND_ENVIR
	genRandEnvir (list);
#else

//	#define USE_ENVIR_0
//	#define USE_ENVIR_1
//	#define USE_ENVIR_2
//	#define USE_ENVIR_3
	#define USE_ENVIR_5

#ifdef USE_ENVIR_0
	// Box in the middle of the environment. Tests environment where a lot of the terrain is
	// unreachable.
	Obstacle *obst;
	Obstacle_ctor ((void**)&obst);
	obst->x =				10;
	obst->y =				10;
	obst->orient =			0;
	obst->len =				300;
	List_pushValue (obstacleList, obst);

	Obstacle_ctor ((void**)&obst);
	obst->x =				295;
	obst->y =				10;
	obst->orient =			PI/2;
	obst->len =				300;
	List_pushValue (obstacleList, obst);

	Obstacle_ctor ((void**)&obst);
	obst->x =				10;
	obst->y =				10;
	obst->orient =			PI/2;
	obst->len =				300;
	List_pushValue (obstacleList, obst);

	Obstacle_ctor ((void**)&obst);
	obst->x =				10;
	obst->y =				295;
	obst->orient =			0;
	obst->len =				300;
	List_pushValue (obstacleList, obst);
#endif
#ifdef USE_ENVIR_1
	// Obstacle for testing path, etc. Narrow paths, sort of like a maize.
	Obstacle *obst;
	Obstacle_ctor ((void**)&obst);
	obst->x =				80.0f;
	obst->y =				0.0f;
	obst->orient =			PI/2.0f;
	obst->len =				250.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				83.0f;
	obst->y =				0.0f;
	obst->orient =			PI/2.0f;
	obst->len =				250.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				86.0f;
	obst->y =				0.0f;
	obst->orient =			PI/2.0f;
	obst->len =				250.0f;
	List_pushValue (obstacleList, obst);

	Obstacle_ctor ((void**)&obst);
	obst->x =				200.0f;
	obst->y =				150.0f;
	obst->orient =			PI/2.0f;
	obst->len =				269.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				203.0f;
	obst->y =				150.0f;
	obst->orient =			PI/2.0f;
	obst->len =				269.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				206.0f;
	obst->y =				150.0f;
	obst->orient =			PI/2.0f;
	obst->len =				269.0f;
	List_pushValue (obstacleList, obst);

	Obstacle_ctor ((void**)&obst);
	obst->x =				280.0f;
	obst->y =				110.0f;
	obst->orient =			0.0f;
	obst->len =				139.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				280.0f;
	obst->y =				113.0f;
	obst->orient =			0.0f;
	obst->len =				139.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				280.0f;
	obst->y =				116.0f;
	obst->orient =			0.0f;
	obst->len =				139.0f;
	List_pushValue (obstacleList, obst);

	Obstacle_ctor ((void**)&obst);
	obst->x =				280.0f;
	obst->y =				310.0f;
	obst->orient =			0.0f;
	obst->len =				139.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				280.0f;
	obst->y =				313.0f;
	obst->orient =			0.0f;
	obst->len =				139.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				280.0f;
	obst->y =				316.0f;
	obst->orient =			0.0f;
	obst->len =				139.0f;
	List_pushValue (obstacleList, obst);

	Obstacle_ctor ((void**)&obst);
	obst->x =				170.0f;
	obst->y =				80.0f;
	obst->orient =			0.0f;
	obst->len =				30.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				170.0f;
	obst->y =				83.0f;
	obst->orient =			0.0f;
	obst->len =				30.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				170.0f;
	obst->y =				86.0f;
	obst->orient =			0.0f;
	obst->len =				30.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				170.0f;
	obst->y =				89.0f;
	obst->orient =			0.0f;
	obst->len =				30.0f;
	List_pushValue (obstacleList, obst);
	Obstacle_ctor ((void**)&obst);
	obst->x =				170.0f;
	obst->y =				92.0f;
	obst->orient =			0.0f;
	obst->len =				30.0f;
	List_pushValue (obstacleList, obst);
#endif
#ifdef USE_ENVIR_2
	// Test gtep. Leave long corridors with both ends open to make robot retrace paths
	Obstacle *obst;
	OBSTC Obstacle_setup (obst,		5.0f,	107.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		5.0f,	110.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		5.0f,	113.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		5.0f,	116.0f,		0.0f,		100.0f); OBSTP

	OBSTC Obstacle_setup (obst,		167.0f,	68.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		170.0f,	68.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		173.0f,	68.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		176.0f,	68.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		168.0f,	167.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		168.0f,	170.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		168.0f,	173.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		168.0f,	176.0f,		0.0f,		100.0f); OBSTP
#if 1 // Testing
	OBSTC Obstacle_setup (obst,		67.0f,	178.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		70.0f,	178.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		73.0f,	178.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		76.0f,	178.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		68.0f,	227.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		68.0f,	230.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		68.0f,	233.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		68.0f,	236.0f,		0.0f,		100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		297.0f,	5.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		300.0f,	5.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		303.0f,	5.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		306.0f,	5.0f,		PI/2.0f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		238.0f,	247.0f,		0.0f,		120.0f); OBSTP
	OBSTC Obstacle_setup (obst,		238.0f,	250.0f,		0.0f,		120.0f); OBSTP
	OBSTC Obstacle_setup (obst,		238.0f,	253.0f,		0.0f,		120.0f); OBSTP
	OBSTC Obstacle_setup (obst,		238.0f,	256.0f,		0.0f,		120.0f); OBSTP
	OBSTC Obstacle_setup (obst,		57.0f,	433.0f,		PI*1.7f,	100.0f); OBSTP // New (20130805)
	OBSTC Obstacle_setup (obst,		58.0f,	434.0f,		PI*1.7f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		59.0f,	435.0f,		PI*1.7f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		60.0f,	436.0f,		PI*1.7f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		61.0f,	437.0f,		PI*1.7f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		62.0f,	438.0f,		PI*1.7f,	100.0f); OBSTP
	OBSTC Obstacle_setup (obst,		204.0f,	323.0f,		0.0f,		80.0f); OBSTP
	OBSTC Obstacle_setup (obst,		204.0f,	326.0f,		0.0f,		80.0f); OBSTP
	OBSTC Obstacle_setup (obst,		204.0f,	329.0f,		0.0f,		80.0f); OBSTP
	OBSTC Obstacle_setup (obst,		204.0f,	332.0f,		0.0f,		80.0f); OBSTP
	OBSTC Obstacle_setup (obst,		203.0f,	334.0f,		PI/2.0f,	80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		206.0f,	334.0f,		PI/2.0f,	80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		209.0f,	334.0f,		PI/2.0f,	80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		212.0f,	334.0f,		PI/2.0f,	80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		332.0f,	433.0f,		0.0f,		80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		332.0f,	436.0f,		0.0f,		80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		332.0f,	439.0f,		0.0f,		80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		332.0f,	442.0f,		0.0f,		80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		403.0f,	354.0f,		PI/2.0f,	80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		406.0f,	354.0f,		PI/2.0f,	80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		409.0f,	354.0f,		PI/2.0f,	80.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		412.0f,	354.0f,		PI/2.0f,	80.0f);  OBSTP
#endif
#endif
#ifdef USE_ENVIR_3
	// Test gtep. Leave long corridors with both ends open to make robot retrace paths
	Obstacle *obst;
	OBSTC Obstacle_setup (obst,		75.0f,	105.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		75.0f,	108.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		75.0f,	111.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		75.0f,	114.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		74.0f,	104.0f,		PI/2.0f,	20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		77.0f,	104.0f,		PI/2.0f,	20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		80.0f,	104.0f,		PI/2.0f,	20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		83.0f,	104.0f,		PI/2.0f,	20.0f); OBSTP

	OBSTC Obstacle_setup (obst,		175.0f,	73.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		175.0f,	75.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		175.0f,	78.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		175.0f,	81.0f,		0.0f,		30.0f); OBSTP

	OBSTC Obstacle_setup (obst,		400.0f,	162.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		400.0f,	165.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		400.0f,	168.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		400.0f,	171.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		420.0f,	161.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		423.0f,	161.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		426.0f,	161.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		429.0f,	161.0f,		PI/2.0f,	50.0f); OBSTP

	OBSTC Obstacle_setup (obst,		95.0f,	222.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		95.0f,	225.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		95.0f,	228.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		95.0f,	231.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		102.0f,	205.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		105.0f,	205.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		108.0f,	205.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		111.0f,	205.0f,		PI/2.0f,	50.0f); OBSTP

	OBSTC Obstacle_setup (obst,		125.0f,	432.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		125.0f,	435.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		125.0f,	438.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		125.0f,	441.0f,		0.0f,		30.0f); OBSTP

	OBSTC Obstacle_setup (obst,		200.0f,	352.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		200.0f,	355.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		200.0f,	358.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		200.0f,	361.0f,		0.0f,		20.0f); OBSTP

	OBSTC Obstacle_setup (obst,		340.0f,	392.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		340.0f,	395.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		340.0f,	398.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		340.0f,	401.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		352.0f,	370.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		355.0f,	370.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		358.0f,	370.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		361.0f,	370.0f,		PI/2.0f,	50.0f); OBSTP

	OBSTC Obstacle_setup (obst,		350.0f,	252.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		350.0f,	255.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		350.0f,	258.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		350.0f,	261.0f,		0.0f,		20.0f); OBSTP
#endif
#ifdef USE_ENVIR_5
	// Test gtep. Leave long corridors with both ends open to make robot retrace paths
	Obstacle *obst;
	OBSTC Obstacle_setup (obst,		5.0f,	107.0f,		0.0f,		50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		5.0f,	110.0f,		0.0f,		50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		5.0f,	113.0f,		0.0f,		50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		5.0f,	116.0f,		0.0f,		50.0f); OBSTP

	OBSTC Obstacle_setup (obst,		167.0f,	68.0f,		PI/2.0f,	20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		170.0f,	68.0f,		PI/2.0f,	20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		173.0f,	68.0f,		PI/2.0f,	20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		176.0f,	68.0f,		PI/2.0f,	20.0f); OBSTP

	OBSTC Obstacle_setup (obst,		168.0f,	167.0f,		0.0f,		25.0f); OBSTP
	OBSTC Obstacle_setup (obst,		168.0f,	170.0f,		0.0f,		25.0f); OBSTP
	OBSTC Obstacle_setup (obst,		168.0f,	173.0f,		0.0f,		25.0f); OBSTP
	OBSTC Obstacle_setup (obst,		168.0f,	176.0f,		0.0f,		25.0f); OBSTP

	OBSTC Obstacle_setup (obst,		67.0f,	178.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		70.0f,	178.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		73.0f,	178.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		76.0f,	178.0f,		PI/2.0f,	50.0f); OBSTP

	OBSTC Obstacle_setup (obst,		68.0f,	227.0f,		0.0f,		50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		68.0f,	230.0f,		0.0f,		50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		68.0f,	233.0f,		0.0f,		50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		68.0f,	236.0f,		0.0f,		50.0f); OBSTP

	OBSTC Obstacle_setup (obst,		297.0f,	5.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		300.0f,	5.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		303.0f,	5.0f,		PI/2.0f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		306.0f,	5.0f,		PI/2.0f,	50.0f); OBSTP

	OBSTC Obstacle_setup (obst,		238.0f,	247.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		238.0f,	250.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		238.0f,	253.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		238.0f,	256.0f,		0.0f,		30.0f); OBSTP

	OBSTC Obstacle_setup (obst,		57.0f,	433.0f,		PI*1.7f,	50.0f); OBSTP // New (20130805)
	OBSTC Obstacle_setup (obst,		58.0f,	434.0f,		PI*1.7f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		59.0f,	435.0f,		PI*1.7f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		60.0f,	436.0f,		PI*1.7f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		61.0f,	437.0f,		PI*1.7f,	50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		62.0f,	438.0f,		PI*1.7f,	50.0f); OBSTP

	OBSTC Obstacle_setup (obst,		204.0f,	323.0f,		0.0f,		50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		204.0f,	326.0f,		0.0f,		50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		204.0f,	329.0f,		0.0f,		50.0f); OBSTP
	OBSTC Obstacle_setup (obst,		204.0f,	332.0f,		0.0f,		50.0f); OBSTP

	OBSTC Obstacle_setup (obst,		203.0f,	334.0f,		PI/2.0f,	50.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		206.0f,	334.0f,		PI/2.0f,	50.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		209.0f,	334.0f,		PI/2.0f,	50.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		212.0f,	334.0f,		PI/2.0f,	50.0f);  OBSTP

	OBSTC Obstacle_setup (obst,		332.0f,	433.0f,		0.0f,		50.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		332.0f,	436.0f,		0.0f,		50.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		332.0f,	439.0f,		0.0f,		50.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		332.0f,	442.0f,		0.0f,		50.0f);  OBSTP

	OBSTC Obstacle_setup (obst,		403.0f,	354.0f,		PI/2.0f,	50.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		406.0f,	354.0f,		PI/2.0f,	50.0f);  OBSTP
	OBSTC Obstacle_setup (obst,		409.0f,	354.0f,		PI/2.0f,	50.0f);  OBSTP 
	OBSTC Obstacle_setup (obst,		412.0f,	354.0f,		PI/2.0f,	50.0f);  OBSTP

	OBSTC Obstacle_setup (obst,		327.0f,	140.0f,		PI/2.0f,	20.0f); OBSTP // Newer
	OBSTC Obstacle_setup (obst,		330.0f,	140.0f,		PI/2.0f,	20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		333.0f,	140.0f,		PI/2.0f,	20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		336.0f,	140.0f,		PI/2.0f,	20.0f); OBSTP

	OBSTC Obstacle_setup (obst,		417.0f,	240.0f,		PI/2.0f,	30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		420.0f,	240.0f,		PI/2.0f,	30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		423.0f,	240.0f,		PI/2.0f,	30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		426.0f,	240.0f,		PI/2.0f,	30.0f); OBSTP

	OBSTC Obstacle_setup (obst,		402.0f,	87.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		402.0f,	90.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		402.0f,	93.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		402.0f,	96.0f,		0.0f,		20.0f); OBSTP

	OBSTC Obstacle_setup (obst,		52.0f,	327.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		52.0f,	330.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		52.0f,	333.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		52.0f,	336.0f,		0.0f,		20.0f); OBSTP

	OBSTC Obstacle_setup (obst,		60.0f,	30.0f,		0.0f,		20.0f); OBSTP // Newer
	OBSTC Obstacle_setup (obst,		60.0f,	33.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		60.0f,	36.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		60.0f,	39.0f,		0.0f,		20.0f); OBSTP

	OBSTC Obstacle_setup (obst,		430.0f,	50.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		430.0f,	53.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		430.0f,	56.0f,		0.0f,		20.0f); OBSTP
	OBSTC Obstacle_setup (obst,		430.0f,	59.0f,		0.0f,		20.0f); OBSTP

	OBSTC Obstacle_setup (obst,		450.0f,	150.0f,		0.0f,		40.0f); OBSTP
	OBSTC Obstacle_setup (obst,		450.0f,	153.0f,		0.0f,		40.0f); OBSTP
	OBSTC Obstacle_setup (obst,		450.0f,	156.0f,		0.0f,		40.0f); OBSTP
	OBSTC Obstacle_setup (obst,		450.0f,	159.0f,		0.0f,		40.0f); OBSTP

	OBSTC Obstacle_setup (obst,		150.0f,	450.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		150.0f,	453.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		150.0f,	456.0f,		0.0f,		30.0f); OBSTP
	OBSTC Obstacle_setup (obst,		150.0f,	459.0f,		0.0f,		30.0f); OBSTP




#endif
#endif // RAND_ENVIR
}

void MapCore_displayTerrain (Image *image, List *obstacleList)
{
#if defined(SIMULATION) && !defined(NO_SIMULATED_ENVIRONMENT)
	Obstacle *obst;
	float endx, endy;
	ListNode *iterator;
#endif // defined(SIMULATION)

	Image_fill (image, 254);

#if defined(SIMULATION) && !defined(NO_SIMULATED_ENVIRONMENT)
	iterator = obstacleList->front;
	while (iterator)
	{
		obst = (Obstacle*)iterator->value;

		Geometry_ptFromOrient (obst->x, obst->y, &endx, &endy, obst->len, obst->orient);

		Image_drawLine (image, (int)obst->x, (int)obst->y, (int)endx, (int)endy, 0, 1);

		iterator = iterator->next;
	}
#endif // defined(SIMULATION)
}
#endif // ifndef IS_GUMSTIX

#define EXP_CELL_THRESH ((EXP_AREA * EXP_AREA) * EXP_CELL_MAPPED_THRESHOLD)
int MapCore_checkIfCellValid (const float cellNMapped)
{
	return (cellNMapped <= EXP_CELL_THRESH);
}
#undef EXP_CELL_THRESH


















void MapCore_printObstructedCellGrid (FILE *f, uchar obstructedCellGrid[((NAV_GRID_DIMS*NAV_GRID_DIMS)/8)+1])
{
	int i, j;
	PointI pt;

	fprintf (f, "<ObstructedCellGrid>\n");
	for (j = NAV_GRID_DIMS - 1; j >= 0; --j)
	{
		for (i = 0; i < NAV_GRID_DIMS; ++i)
		{
			pt.x = i;
			pt.y = j;

			fprintf (f, "%d ",
				BitArray_checkElement_pt (
				obstructedCellGrid,
				pt,
				NAV_GRID_DIMS));
		}
		fprintf (f, "\n");
	}
	fprintf (f, "</ObstructedCellGrid>\n");
}

void MapCore_printLocalMapGrid (FILE *f, __int16 localMapGrid[GLOB_LOC_MAP_GRID_DIMS][GLOB_LOC_MAP_GRID_DIMS])
{
	PointI localMapIter;

	fprintf (f, "<LocalMapGrid>\n");
	fprintf (f, "      ");
	for (localMapIter.x = 0; localMapIter.x < GLOB_LOC_MAP_GRID_DIMS; ++localMapIter.x)
	{
		fprintf (f, "%6d", localMapIter.x);
	}
	fprintf (f, "\n");
	for (localMapIter.y = GLOB_LOC_MAP_GRID_DIMS - 1; localMapIter.y >= 0; --localMapIter.y)
	{
		fprintf (f, "%6d", localMapIter.y);
		for (localMapIter.x = 0; localMapIter.x < GLOB_LOC_MAP_GRID_DIMS; ++localMapIter.x)
		{
			fprintf (f, "%6d", localMapGrid[localMapIter.x][localMapIter.y]);
		}
		fprintf (f, "\n");
	}

	fprintf (f, "</LocalMapGrid>\n");
}

void MapCore_printExplorationGridSection (FILE *f,
										  const PointI bl,
										  const PointI tr,
										  __int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS])
{
	int i, j;

	fprintf (f, "<UpdateExplorationGrid>\n");
	fprintf (f, "      ");

	for (i = bl.x; i <= tr.x; ++i)
	{
#ifdef PRINT_MIDPTS
		fprintf (f, "%6d", midpt.x + i * EXP_AREA);
#else
		fprintf (f, "%6d", i);
#endif
	}
	fprintf (f, "\n");

	for (j = tr.y; j >= bl.y; --j)
	{
#ifdef PRINT_MIDPTS
		fprintf (f, "%6d", midpt.y + j * EXP_AREA);
#else
		fprintf (f, "%6d", j);
#endif

		for (i = bl.x; i <= tr.x; ++i)
		{
			fprintf (f, "%6d", expGrid[i][j]);
		}

		fprintf (f, "\n");
	}

	fprintf (f, "</UpdateExplorationGrid>\n");

#undef PRINT_MIDPTS
}

void MapCore_printExplorationGrid (FILE *f,
								   __int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS])
{
	PointI bl, tr;
	bl.x = 0;
	bl.y = 0;
	tr.x = EXP_GRID_DIMS - 1;
	tr.y = EXP_GRID_DIMS - 1;

	MapCore_printExplorationGridSection (f, bl, tr, expGrid);
}

void MapCore_printUnreachableExpCellGrid (
	FILE *f,
	uchar unreachableExpCellGrid[((GLOB_EXP_GRID_DIMS*GLOB_EXP_GRID_DIMS)/8)+1])
{
	int i, j;
	PointI pt;

	fprintf (f, "<UnreachableExpCellGrid>\n");
	for (j = GLOB_EXP_GRID_DIMS - 1; j >= 0; --j)
	{
		for (i = 0; i < GLOB_EXP_GRID_DIMS; ++i)
		{
			pt.x = i;
			pt.y = j;

			fprintf (f, "%d ",
				BitArray_checkElement_pt (
				unreachableExpCellGrid,
				pt,
				GLOB_EXP_GRID_DIMS));
		}
		fprintf (f, "\n");
	}
	fprintf (f, "</UnreachableExpCellGrid>\n");
}

void MapCore_printUnreachableLocalMapGrid (FILE *f, uchar unreachableLocalMapGrid[((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1])
{
	int i, j;
	PointI pt;

	fprintf (f, "<UnreachableLocalMapGrid>\n");
	for (j = GLOB_LOC_MAP_GRID_DIMS - 1; j >= 0; --j)
	{
		for (i = 0; i < GLOB_LOC_MAP_GRID_DIMS; ++i)
		{
			pt.x = i;
			pt.y = j;

			fprintf (f, "%d ",
				BitArray_checkElement_pt (
				unreachableLocalMapGrid,
				pt,
				GLOB_LOC_MAP_GRID_DIMS));
		}
		fprintf (f, "\n");
	}
	fprintf (f, "</UnreachableLocalMapGrid>\n");
}

