#include "RobotRecognitionCore.h"
#include "../../Common/RobotCore.h"
#include "../../Common/BitArray.h"

#ifndef BOARD


RobotFace RobotFace_init()
{
	RobotFace r;
	r.blob = NULL;
	r.faceOrient = 0.0f;
	r.estType = FACE_EST_NOTHING;
	return r;
}

RobotEstimate RobotEstimate_init()
{
	RobotEstimate r;
	r.robotFaces[0] = RobotFace_init();
	r.robotFaces[1] = RobotFace_init();
	r.robotFaces[2] = RobotFace_init();
	r.robotOrient = 0.0f;
	r.robotRelativeOrient = 0.0f;
	r.robotIndex = -1;
	r.estType = FACE_EST_NOTHING;
	return r;
}

ColourBlob* ColourBlob3_alloc (
	const int colourIndex,
	const int id)
{
	ColourBlob *blob;

	blob = (ColourBlob*)malloc (sizeof (ColourBlob));
	*blob = ColourBlob3_init (colourIndex, id);

	return blob;
}

ColourBlob ColourBlob3_init (
	const int colourIndex,
	const int id)
{
	ColourBlob c;

	c.colourIndex = colourIndex;
	c.id = id;
	c.nCells = 0;
	c.colourScore = 0.0f;
	c.densityScore = 0.0f;
	c.bl.x = MAX_FLT;
	c.bl.y = MAX_FLT;
	c.tr.x = MIN_FLT;
	c.tr.y = MIN_FLT;
	c.isUnique = 1;
	c.isConflicting = 0;
	c.blobType = BLOB_NOT_SET;
#if VERBOSE_BLOB_DETECTION
	c.isColourInImg = 0;
#endif

	return c;
}


// Orient offset from robot orient to fblr
const float RobotRecognitionCore_faceOrientOffset[4] = {0.0f, PI, RADS_90, -RADS_90};

// Corner indices from faces fblr
const PointI RobotRecognitionCore_cornerIndicesPerFace[4] = {{3, 0}, {1, 2}, {0, 1}, {2, 3}};

//! Updated on 2013/03/28
const RobotParams RobotRecognitionCore_robotParams[3] = {
	{7.0f, 65.0f, 72.0f, {105.0f, 105.0f, 63.0f, 63.0f}},
	{7.0f, 65.0f, 72.0f, {105.0f, 105.0f, 63.0f, 63.0f}},
	{9.0f, 65.0f, 74.0f, {91.0f, 99.0f, 85.0f, 85.0f}},
};

/*!
Pre-calculated orient offsets between a robot's corner and its orient. Also the dist from
the corner to the robot's cog.

Remember to update if RobotRecognitionCore_robotParams above is changed.
*/
const CornerParams RobotRecognitionCore_cornerParams[3] = {
	{{0.540419f, 2.601173f, 3.682012f, 5.742765f}, {122.449989f, 122.449989f, 122.449989f, 122.449989f}},
	{{0.540419f, 2.601173f, 3.682012f, 5.742765f}, {122.449989f, 122.449989f, 122.449989f, 122.449989f}},
	{{0.751320f, 2.432135f, 3.851050f, 5.531864f}, {124.523090f, 130.483715f, 130.483715f, 124.523090f}},
};

float RobotRecognitionCore_getFaceHalfLen (
	const int robotIndex,
	const int faceIndex)
{
	float len;
	switch (faceIndex)
	{
	case 0:
	case 1:
		len = RobotRecognitionCore_robotParams[robotIndex].cogToFace[2] + RobotRecognitionCore_robotParams[robotIndex].cogToFace[3];
		break;
	default:
		len = RobotRecognitionCore_robotParams[robotIndex].cogToFace[0] + RobotRecognitionCore_robotParams[robotIndex].cogToFace[1];
		break;
	}
	return len / 2.0f;
}

float RobotRecognitionCore_getDistToSide (
	const int robotIndex,
	const int faceIndex,
	const int wantDistToLeftOfFace)
{
	switch (faceIndex)
	{
	case 0: // Front
		if (wantDistToLeftOfFace)
		{
			return RobotRecognitionCore_robotParams[robotIndex].cogToFace[2]; // Dist from COG to left
		}
		else
		{
			return RobotRecognitionCore_robotParams[robotIndex].cogToFace[3]; // Dist to right
		}
	case 1: // Back
		if (wantDistToLeftOfFace)
		{
			return RobotRecognitionCore_robotParams[robotIndex].cogToFace[3]; // Dist to right
		}
		else
		{
			return RobotRecognitionCore_robotParams[robotIndex].cogToFace[2]; // Left
		}
	case 2: // Left
		if (wantDistToLeftOfFace)
		{
			return RobotRecognitionCore_robotParams[robotIndex].cogToFace[0]; // Dist to front
		}
		else
		{
			return RobotRecognitionCore_robotParams[robotIndex].cogToFace[1]; // Dist to back
		}
	case 3: // Right
	default:
		if (wantDistToLeftOfFace)
		{
			return RobotRecognitionCore_robotParams[robotIndex].cogToFace[1]; // Dist to back
		}
		else
		{
			return RobotRecognitionCore_robotParams[robotIndex].cogToFace[0]; // Dist to front
		}
	}
}














/*!
Defined per-robot.
Remember to update this and also the data in RobotRecognitionModel.c when/if the
colour model is updated.

2,
3,
5,
6,
7,
8,
9,
11,
12,
13,
14,

(2,5,8) on r2 (6,11,8) on r3 and (12,14,8) on r4
*/
#if defined(SIMULATION)
//! Face indices signify which side of a robot a face corresponds to: 0=f/b 1=l/r 2=top
const int RobotRecognitionCore_colourIndexToFaceTypeIndex[N_ROBOT_COLOURS] = {
	0, // colId=2|yellow, r=0, side=fb
	2, // colId=3|lightPink, r=0, side=t
	1, // colId=5|aquaGreen, r=0, side=lr
	0, // colId=6|lightAqua, r=1, side=fb
	2, // colId=7|lightPurple, r=-1, side=t
	2, // colId=8|darkPurple, r=-1, side=t
	2, // colId=9|brown, r=1, side=t
	1, // colId=11|mustard, r=1, side=lr
	0, // colId=12|darkPink, r=2, side=fb
	2, // colId=13|lightOrange, r=2, side=t
	1, // colId=14|lightGreen, r=2, side=lr
};

const int RobotRecognitionCore_colourIndexToRobotIndex[N_ROBOT_COLOURS] = {
	0, // colourId=2,	colour=yellow
	0, // colourId=3,	colour=lightPink
	0, // colourId=5,	colour=aquaGreen
	1, // colourId=6,	colour=lightAqua
	-1, // colourId=7,	colour=lightPurple
	-1, // colourId=8,	colour=darkPurple
	1, // colourId=9,	colour=brown
	1, // colourId=11,	colour=mustard
	2, // colourId=12,	colour=darkPink
	2, // colourId=13,	colour=lightOrange
	2, // colourId=14,	colour=lightGreen
};
#else // if defined(SIMULATION)

#if ROBOT_PLATFORM == 2
//! Face indices signify which side of a robot a face corresponds to: 0=f/b 1=l/r 2=top
//! (6,11,8) on r3 (12,14,8) on r4
const int RobotRecognitionCore_colourIndexToFaceTypeIndex[N_ROBOT_COLOURS] = {
	0, // colId=6|lightAqua, r=1, side=fb
	2, // colId=8|darkPurple, r=-1, side=t
	1, // colId=11|mustard, r=1, side=lr
	0, // colId=12|darkPink, r=2, side=fb
	1, // colId=14|lightGreen, r=2, side=lr
};

const int RobotRecognitionCore_colourIndexToRobotIndex[N_ROBOT_COLOURS] = {
	1, // colourId=6,	colour=lightAqua
	-1, // colourId=8,	colour=darkPurple
	1, // colourId=11,	colour=mustard
	2, // colourId=12,	colour=darkPink
	2, // colourId=14,	colour=lightGreen
};
#endif
#if ROBOT_PLATFORM == 3
//! Face indices signify which side of a robot a face corresponds to: 0=f/b 1=l/r 2=top
//! (2,5,8) on r2 (12,14,8) on r4
const int RobotRecognitionCore_colourIndexToFaceTypeIndex[N_ROBOT_COLOURS] = {
	0, // colId=2|yellow, r=0, side=fb
	1, // colId=5|aquaGreen, r=0, side=lr
	2, // colId=8|darkPurple, r=-1, side=t
	0, // colId=12|darkPink, r=2, side=fb
	1, // colId=14|lightGreen, r=2, side=lr
};

const int RobotRecognitionCore_colourIndexToRobotIndex[N_ROBOT_COLOURS] = {
	0, // colourId=2,	colour=yellow
	0, // colourId=5,	colour=aquaGreen
	-1, // colourId=8,	colour=darkPurple
	2, // colourId=12,	colour=darkPink
	2, // colourId=14,	colour=lightGreen
};
#endif
#if ROBOT_PLATFORM == 4
//! Face indices signify which side of a robot a face corresponds to: 0=f/b 1=l/r 2=top
//! (2,5,8) on r2 (6,11,8) on r3
const int RobotRecognitionCore_colourIndexToFaceTypeIndex[N_ROBOT_COLOURS] = {
	0, // colId=2|yellow, r=0, side=fb
	1, // colId=5|aquaGreen, r=0, side=lr
	0, // colId=6|lightAqua, r=1, side=fb
	2, // colId=8|darkPurple, r=-1, side=t
	1, // colId=11|mustard, r=1, side=lr
};

const int RobotRecognitionCore_colourIndexToRobotIndex[N_ROBOT_COLOURS] = {
	0, // colourId=2,	colour=yellow
	0, // colourId=5,	colour=aquaGreen
	1, // colourId=6,	colour=lightAqua
	-1, // colourId=8,	colour=darkPurple
	1, // colourId=11,	colour=mustard
};
#endif

#endif // if defined(SIMULATION)






























int RobotRecognitionCore_isColourOkForTraining (const int colour)
{
	// Background colour
	return (colour != 20 && colour != 19 && colour != 18);
}

float RobotRecognitionCore_calcColourDiff (
	const float *cellDesc,
	const float *colourDesc)
{
	float diff;
	diff =
		fabs(cellDesc[0] - colourDesc[0]) +
		fabs(cellDesc[1] - colourDesc[1]) +
		fabs(cellDesc[2] - colourDesc[2]) +
		fabs(cellDesc[3] - colourDesc[3]) +
		fabs(cellDesc[4] - colourDesc[4]) +
		fabs(cellDesc[5] - colourDesc[5]);
	return diff;
}

//! If a camera image cell x,y is deemed to be occupied by a robot, fill cells x,z where z > y.
/*!
We do not want to analyse environment terrain that appears behind a robot, as there will
be some many artefacts it will likely result in incorrect occupancy estimates.
*/
int RobotRecognitionCore_fillRobotOccupiedCellsUpwards (
	OccupancyGrid *occupancyGrid,
	const uchar *lowestRobotCellInColumns)
{
	int columnIndex;
	int rowIndex;
	int anyRobotCell = 0;

	for (columnIndex = 0; columnIndex < CAM_OCCUPANCY_GRID_X; ++columnIndex)
	{
		rowIndex = lowestRobotCellInColumns[columnIndex];
		if (255 != rowIndex)
		{
			anyRobotCell = 1;

			for (++rowIndex; rowIndex < CAM_OCCUPANCY_GRID_Y; ++rowIndex)
			{
				TwoBitArray_setCoords (occupancyGrid->grid, columnIndex, rowIndex, CAM_OCCUPANCY_GRID_X, ROBOT_OCCUPIED_FLAG);
			}
		}
	}

	return anyRobotCell;
}

int RobotRecognitionCore_markRobotOccupiedCells (
	FILE *outputFile,
	const float *colourScoreGrid,
	OccupancyGrid *occupancyGrid,
	const float colourScoreThreshold)
{
#if 1 || VERBOSE_BLOB_DETECTION
	int n, nCols;
#endif
	int cellIndex, colourIndex;
	int thisRow, thisColumn;
	int anyRobotCells;
	uchar lowestRobotCellInColumns[CAM_OCCUPANCY_GRID_X];
	uchar lowestRobotCellThisColumn;

	memset (lowestRobotCellInColumns, 255, CAM_OCCUPANCY_GRID_X);

#if 1 || VERBOSE_BLOB_DETECTION
	n = 0;
	nCols = 0;
#endif
	for (cellIndex = 0; cellIndex < SIZE_COOP_LOC_GRID; ++cellIndex)
	{
		for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
		{
			if (colourScoreGrid[colourIndex + cellIndex * N_ROBOT_COLOURS] < colourScoreThreshold)
			{
#if 1 || VERBOSE_BLOB_DETECTION
				++n;
//				fprintf (outputFile, "%3d,%3d marked for colour %2d\n", cellIndex % CAM_OCCUPANCY_GRID_X, cellIndex / CAM_OCCUPANCY_GRID_X, colourIndex);
#endif
				TwoBitArray_setIndex (occupancyGrid->grid, cellIndex, ROBOT_OCCUPIED_FLAG);
				thisColumn = cellIndex % CAM_OCCUPANCY_GRID_X;
				thisRow = cellIndex / CAM_OCCUPANCY_GRID_X;
				lowestRobotCellThisColumn = lowestRobotCellInColumns[thisColumn];
				if (255 == lowestRobotCellThisColumn)
				{
#if 1 || VERBOSE_BLOB_DETECTION
					++nCols;
#endif
					lowestRobotCellInColumns[thisColumn] = thisRow;
				}
				break;
			}
		}
	}

#if VERBOSE_BLOB_DETECTION
	fprintf (outputFile, "<RobotCells>n=%d gridSize=%d nColumns=%d<RobotCells>\n", n, SIZE_COOP_LOC_GRID, nCols);
	fflush (outputFile);
#endif

	anyRobotCells = RobotRecognitionCore_fillRobotOccupiedCellsUpwards (
		occupancyGrid,
		lowestRobotCellInColumns);

	return anyRobotCells;
}

void RobotRecognitionCore_getCellsNbouringValue (
	const uchar *blobGrid,
	const uchar requiredBlobId,
	uchar *nbourGrid)
{
	int i, j, blobCellIndex, nbourIndex;
	int k, l;

	nbourIndex = 0;
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			/*if (blobGrid[nbourIndex] == requiredBlobId)
			{
				continue;
			}*/
			if (blobGrid[nbourIndex] != 255)
			{
				++nbourIndex;
				continue;
			}

			for (l = max (0, j - 1); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
			{
				for (k = max (0, i - 1); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
				{
					if (k == i && l == j)
					{
						continue;
					}

					blobCellIndex = k + l * CAM_OCCUPANCY_GRID_X;
					if (blobGrid[blobCellIndex] == requiredBlobId)
					{
						nbourGrid[nbourIndex] = (blobCellIndex - nbourIndex) + MAX_COOP_LOC_NBOUR_DIST;
						goto RobotRecognitionCore_getCellsNbouringColour_isNbour;
					}
				}
			}
RobotRecognitionCore_getCellsNbouringColour_isNbour:
			; // Break from 2 loops
			++nbourIndex;
		}
	}
}

void RobotRecognitionCore_calcNbourInfluenceGrid (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	const float *colourScoreGrid,
	const int colourIndex,
	float *nbourInfluenceGrid)
{
	int i, j, k, l;
	int cellIndex, nbourIndex;
	float accumDiff, nbourDiff;

	cellIndex = 0;
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			accumDiff = 0.0f;

			for (l = max (0, j - 1); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
			{
				for (k = max (0, i - 1); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
				{
					if (k == i && l == j)
					{
						continue;
					}
					nbourIndex = k + l * CAM_OCCUPANCY_GRID_X;

					nbourDiff = colourScoreGrid[colourIndex + nbourIndex * N_ROBOT_COLOURS] - COLOUR_SCORE_NORMAL_THRESHOLD;
					accumDiff += nbourDiff;
				}
			}

			nbourInfluenceGrid[cellIndex] = accumDiff;
			++cellIndex;
		}
	}
}

PointI RobotRecognitionCore_gridPtToCameraPixel (
	const PointI gridPt)
{
	PointI cameraPixel;
	cameraPixel.x = CAM_OCCUPANCY_GRID_ORIGIN_X + CAM_OCCUPANCY_CELL_X_HALF + (CAM_OCCUPANCY_CELL_X * gridPt.x);
	cameraPixel.y = CAM_OCCUPANCY_GRID_ORIGIN_Y + CAM_OCCUPANCY_CELL_Y_HALF + (CAM_OCCUPANCY_CELL_Y * gridPt.y);
	return cameraPixel;
}

#endif







