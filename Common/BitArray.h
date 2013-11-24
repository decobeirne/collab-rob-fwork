#ifndef BITARRAY_H
#define BITARRAY_H

#include "RobotDefs.h"
#include "Point.h"



//! Set grid stored as bit array to all 0
/*!
Values in the bit array can be 0 ro 1, so if the dims of the
grid are x by y, the size of the grid in bytes will be x*y/8
*/
void BitArray_reset (
	uchar *grid,
	const int dimsX,
	const int dimsY);

//! Set an element in a grid stored as a bit array to 0 or 1
/*!
Element is accessed as x*h + y, as the normal way of traversing
a grid is to iterate through the x values, then the y
*/
void BitArray_setElement_pt (
	uchar *grid,
	const PointI element,
	const int gridDimsX,
//	const int gridDimsY,
	const int val);

void BitArray_setCoords (
	uchar *grid,
	const int ptX,
	const int ptY,
	const int gridDimsX,
//	const int gridDimsY,
	const int val);

//! Set an element in a grid stored as a bit array to 0 or 1
void BitArray_setElement_index (
	uchar *grid,
	const int index,
//	const int gridDimsX,
//	const int gridDimsY,
	const int val);

//! Print bit array to log
void BitArray_display (
	uchar *grid,
	FILE *f,
//	const int dimsX);
	const int dimsX,
	const int dimsY);

//! Check if an element is set in a grid stored as a bit array
/*!
The index in the array is calcd based on the grid dims and the
element coords
*/
int BitArray_checkElement_pt (
	const uchar *grid,
	const PointI element,
	const int gridDimsX);
//	const int gridDimsX,
//	const int gridDimsY);

int BitArray_checkCoords (
	const uchar *grid,
	const int ptX,
	const int ptY,
	const int gridDimsX);

//! Check if an element is set in a grid stored as a bit array
int BitArray_checkElement_index (
	const uchar *grid,
	const int index);
//	const int index,
//	const int gridDimsX,
//	const int gridDimsY);










//! Reset grid of 2-bit values.
void TwoBitArray_reset(uchar *grid, const int sz);

//! Set 2-bit value in grid given index.
void TwoBitArray_setIndex(uchar *grid, const int index, const uchar val);

//! Set 2-bit value in grid given point.
void TwoBitArray_setPt(uchar *grid, const PointI pt, const int gridY, const uchar val);

//! Set 2-bit value in grid given point coordinates.
void TwoBitArray_setCoords(uchar *grid, const int ptX, const int ptY, const int gridX, const uchar val);

//! Return 2-bit value in grid given index.
uchar TwoBitArray_checkIndex (const uchar *grid, const int index);

//! Return 2-bit value in grid given point.
uchar TwoBitArray_checkPt (const uchar *grid, const PointI pt, const int gridX);

//! Return 2-bit value in grid given point coordinates.
uchar TwoBitArray_checkCoords (const uchar *grid, const int ptX, const int ptY, const int gridX);

//! Print grid of 2-bit values to given file stream.
void TwoBitArray_display (uchar *grid, FILE *f, const int gridX, const int gridY);









#endif // BITARRAY_H
