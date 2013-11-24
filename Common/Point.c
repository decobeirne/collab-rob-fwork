
#include "Point.h"




//*****************************************************************************
PointI PointF_toPointI (const PointF pt)
{
	PointI pti;
	pti.x = (int)pt.x;
	pti.y = (int)pt.y;
	return pti;
}

PointF initPointF (const float f)
{
	PointF p;
	p.x = f;
	p.y = f;
	return p;
}

PointF PointF_multiplyMat (const PointF pt, const float mat[4])
{
	PointF pt2;

	pt2.x = mat[0] * pt.x + mat[1] * pt.y;
	pt2.y= mat[2] * pt.x + mat[3] * pt.y;

	return pt2;
}

PointF PointF_rotate (const PointF pt, const float theta)
{
	float mat[4];
	mat[0] = cos (theta);
	mat[1] = -sin (theta);
	mat[2] = -mat[1];
	mat[3] = mat[0];

	return PointF_multiplyMat (pt, mat);
}



//*****************************************************************************
PointI initPointI (const int i, const int j)
{
	PointI p;
	p.x = i;
	p.y = j;
	return p;
}

PointF PointI_toPointF (const PointI pt)
{
	PointF ptf;
	ptf.x = (float)pt.x;
	ptf.y = (float)pt.y;
	return ptf;
}

int PointI_comparePtrs (const void *p1_, const void *p2_)
{
	const PointI *p1 = (const PointI*)p1_;
	const PointI *p2 = (const PointI*)p2_;
	return (p1->x == p2->x && p1->y == p2->y);
}

int PointI_compareValues (const PointI p1, const PointI p2)
{
	return p1.x == p2.x && p1.y == p2.y;
}

PointI PointI_calcNavCellMidpt (const PointI pt)
{
	PointI cellMidpoint;
	cellMidpoint.x = ((pt.x / NAV_CELL_AREA) * NAV_CELL_AREA) + NAV_CELL_AREA/2;
	cellMidpoint.y = ((pt.y / NAV_CELL_AREA) * NAV_CELL_AREA) + NAV_CELL_AREA/2;
	return cellMidpoint;
}

PointI PointI_calcNavCellMidptObstdCell (const PointI ps)
{
	PointI obstructedCell;
	obstructedCell.x = ps.x / NAV_CELL_AREA;
	obstructedCell.y = ps.y / NAV_CELL_AREA;
	return obstructedCell;
}

PointI PointI_calcExpCellIndex (const PointI p,
							 const int cellDims)
{
	PointI cell;
	cell.x = p.x / cellDims;
	cell.y = p.y / cellDims;
	return cell;
}

PointI PointI_calcExpCellIndex_check (const PointI p,
									const int cellDims,
									const int envirDimsX,
									const int envirDimsY)
{
	PointI cell;
	cell.x = p.x / cellDims;
	cell.y = p.y / cellDims;
	cell.x = max (0, min (cell.x, (envirDimsX - 1)));
	cell.y = max (0, min (cell.y, (envirDimsY - 1)));
	return cell;
}

PointI PointI_calcCellIndex (const PointI centre,
							 const int cellDims,
							 const int cellMidptDiff)
{
	PointI index;
	index.x = (centre.x - (cellDims / 2)) / cellMidptDiff;
	index.y = (centre.y - (cellDims / 2)) / cellMidptDiff;
	return index;
}




