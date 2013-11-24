
#include "BitArray.h"






//*****************************************************************************
void BitArray_display (uchar *grid,
					 FILE *f,
					 const int dimsX,
					 const int dimsY)
{
	int i, j;
	PointI element;
	int isSet;

	// print so tl is printed first, then br
	for (j = dimsY - 1; j >= 0; --j)
	{
		for (i = 0; i < dimsX; ++i)
		{
			element.x = i;
			element.y = j;

			isSet = BitArray_checkElement_pt (
				grid,
				element,
				dimsX);
//				dimsX,
//				dimsY);

			if (isSet)
			{
				fprintf (f, "#");
			}
			else
			{
				fprintf (f, "0");
			}

			fprintf (f, " ");
		}

		fprintf (f, "\n");
	}
}

void BitArray_setElement_pt (uchar *grid,
							 const PointI element,
							 const int gridDimsX,
//							 const int gridDimsY,
							 const int val)
{
	int i;
	int iByte;
//	i = element.x * gridDimsY + element.y;
	i = element.x + gridDimsX * element.y;
	iByte = i * (1.0f / 8.0f);
	i = i % 8;
	if (1 == val)
	{
		grid[iByte] = grid[iByte] | (1 << i);
	}
	else
	{
		grid[iByte] = grid[iByte] & ~(1 << i);
	}
}

void BitArray_setCoords (
	uchar *grid,
	const int ptX,
	const int ptY,
	const int gridDimsX,
	const int val)
{
	int i;
	int iByte;
//	i = element.x * gridDimsY + element.y;
	i = ptX + gridDimsX * ptY;
	iByte = i * (1.0f / 8.0f);
	i = i % 8;
	if (1 == val)
	{
		grid[iByte] = grid[iByte] | (1 << i);
	}
	else
	{
		grid[iByte] = grid[iByte] & ~(1 << i);
	}
}

void BitArray_setElement_index (uchar *grid,
								const int index,
//								const int gridDimsX,
//								const int gridDimsY,
								const int val)
{
	int i;
	int iByte;
	iByte = index * (1.0f / 8.0f);
	i = index % 8;
	if (1 == val)
	{
		grid[iByte] = grid[iByte] | (1 << i);
	}
	else
	{
		grid[iByte] = grid[iByte] & ~(1 << i);
	}
}

int BitArray_checkElement_pt (const uchar *grid,
							  const PointI element,
							  const int gridDimsX)
//							  const int gridDimsX,
//							  const int gridDimsY)
{
	int i;
	int iByte;
//	i = element.x * gridDimsY + element.y;
	i = element.x + gridDimsX * element.y;
	iByte = i * (1.0f / 8.0f);
	i = i % 8;
	if ((grid[iByte] & (1 << i)) == (1 << i))
	{
		return 1;
	}
	return 0;
}

int BitArray_checkCoords (
	const uchar *grid,
	const int ptX,
	const int ptY,
//	const int gridDimsY)
	const int gridDimsX)
{
	int i;
	int iByte;
//	i = ptX * gridDimsY + ptY;
	i = ptX + gridDimsX * ptY;
	iByte = i * (1.0f / 8.0f);
	i = i % 8;
	if ((grid[iByte] & (1 << i)) == (1 << i))
	{
		return 1;
	}
	return 0;
}

int BitArray_checkElement_index (const uchar *grid,
								 const int index)
//								 const int index,
//								 const int gridDimsX,
//								 const int gridDimsY)
{
	int i;
	int iByte;
	iByte = index * (1.0f / 8.0f);
	i = index % 8;
	if ((grid[iByte] & (1 << i)) == (1 << i))
	{
		return 1;
	}
	return 0;
}

void BitArray_reset (uchar *grid,
					 const int dimsX,
					 const int dimsY)
{
	const int sz_ = (dimsX * dimsY) * (1.0f / 8.0f);
	int sz = sz_;
	if (0 != ((dimsX * dimsY) % 8))
	{
		sz++;
	}
	memset (grid, 0, sz);
}















//*****************************************************************************
void TwoBitArray_reset(uchar *grid, const int sz)
{
	memset (grid, 0, sz);
}

void TwoBitArray_setIndex(uchar *grid, const int index, const uchar val)
{
	int elem, shift;
	DEBUG_ASSERT(!(val & ~0x03))
	elem = index / 4;
	shift = (index % 4) * 2;

	grid[elem] = (val << shift) | (grid[elem] & ~(0x03 << shift));
}

void TwoBitArray_setPt(uchar *grid, const PointI pt, const int gridX, const uchar val)
{
	TwoBitArray_setIndex (grid, pt.y * gridX + pt.x, val);
}

void TwoBitArray_setCoords(uchar *grid, const int ptX, const int ptY, const int gridX, const uchar val)
{
	TwoBitArray_setIndex (grid, ptY * gridX + ptX, val);
}

uchar TwoBitArray_checkIndex (const uchar *grid, const int index)
{
	int elem, shift;
	elem = index / 4;
	shift = (index % 4) * 2;
	return (int)((grid[elem] & (0x03 << shift)) >> shift);
}

uchar TwoBitArray_checkPt (const uchar *grid, const PointI pt, const int gridX)
{
	return TwoBitArray_checkIndex (grid, pt.y * gridX + pt.x);
}

uchar TwoBitArray_checkCoords (const uchar *grid, const int ptX, const int ptY, const int gridX)
{
	return TwoBitArray_checkIndex (grid, ptY * gridX + ptX);
}

void TwoBitArray_display (uchar *grid, FILE *f, const int gridX, const int gridY)
{
	int i, j;
	uchar val;

	// Print such that the top left element is printed first, and the
	// bottom right corner last.
	for (j = gridY - 1; j >= 0; --j)
	{
		for (i = 0; i < gridX; ++i)
		{
			val = TwoBitArray_checkCoords (grid, i, j, gridX);
			fprintf (f, "%d ", val);
		}
		fprintf (f, "\n");
	}
}














