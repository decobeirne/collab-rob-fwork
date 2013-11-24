#include "../../Common/RobotDefs.h"

#if defined(IS_WIN)

#ifndef CV_H
#define CV_H

#include "CVCore.h"

#define CV_8UC3 CV_MAKETYPE(CV_8U,3)

#define MAGIC_VAL 0x42FF0000
#define AUTO_STEP 0


typedef struct Mat
{
	// includes several bit-fields:
	//  * the magic signature
	//  * continuity flag
	//  * depth
	//  * number of channels
	int flags;

	// the number of rows and columns
	int rows;
	int cols;

	// a distance between successive rows in bytes; includes the gap if any
	size_t step;

	// pointer to the data
	uchar* data;

	// pointer to the reference counter;
	// when matrix points to user-allocated data, the pointer is NULL
	int* refcount;

	// helper fields used in locateROI and adjustROI
	uchar* datastart;
	uchar* dataend;
} Mat;

//////////////////

void cvShowImage( const char* name, const CvArr* arr );

void cvMoveWindow( const char* name, const int x, const int y );



void cvNamedWindow( const char* name, const int windowSize[4] );




#endif // CV_H
#endif // defined(IS_WIN)



