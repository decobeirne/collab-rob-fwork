#include "ObstacleRecognition.h"
#include "ObstacleRecognitionCore.h"
#include "ObstacleRecognitionModel.h"
#include "../../Common/BitArray.h"

#ifndef BOARD

uchar g_tempOccupancyGrid[SIZE_COOP_LOC_BITARRAY];

int ObstacleRecognition_estimateCellOccupancy (
	const float camScore)
{
	int isNotConfident, isOccupiedEst;
	int est;
	float ratio;
	ratio = camScore;
	if (ratio < 1.0f)
	{
		ratio = 1.0f / ratio;
	}

	// If not confident, then we want value=2, and it doesn't matter if the
	// estimate is occupied or not. If confident, then we can take the
	// occupancy estimate
	isNotConfident = (fabs (ratio - 1.0f) < OCCUPANCY_CONFIDENCE_THRESHOLD);
	isOccupiedEst = (camScore < 1.0f) && !isNotConfident;
	est = (isNotConfident << 1) + isOccupiedEst;

	return est;
}

//! If robot cells are marked on the grid, then fill any adjoining occupied cells as unknown
/*!
Return anyOccupiedEst
*/
void ObstacleRecognition_fillRobotCells (
	OccupancyGrid *camOccupancyGrid)
{
	int i, j;
	int k, l;

	memset (g_tempOccupancyGrid, 0, SIZE_COOP_LOC_BITARRAY);

	// Set robot cells in out fill grid
	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		j = TwoBitArray_checkIndex (camOccupancyGrid->grid, i);
		BitArray_setElement_index (g_tempOccupancyGrid, i, j);
	}

	// Fill from bottom left - mark cells in temp grid as 1 if they are either robot
	// or next to a robot cell, and are not 0, i.e. 1 (occupied) or 2 (unknown)
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y - 2; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X - 2; ++i)
		{
			if (BitArray_checkCoords (g_tempOccupancyGrid, i, j, CAM_OCCUPANCY_GRID_X))
			{
				for (l = j; l < j + 2; ++l)
				{
					for (k = i; k < i + 2; ++k)
					{
						if ((k != i && l != j) &&
							TwoBitArray_checkCoords (camOccupancyGrid->grid, k, l, CAM_OCCUPANCY_GRID_X) != 0)
						{
							BitArray_setCoords (g_tempOccupancyGrid, k, l, CAM_OCCUPANCY_GRID_X, 1);
						}
					}
				}
			}
		}
	}

	// Fill from top right
	for (j = CAM_OCCUPANCY_GRID_Y - 1; j > 0; --j)
	{
		for (i = CAM_OCCUPANCY_GRID_X - 1; i > 0; --i)
		{
			if (BitArray_checkCoords (g_tempOccupancyGrid, i, j, CAM_OCCUPANCY_GRID_X))
			{
				for (l = j; l < j + 2; ++l)
				{
					for (k = i; k < i + 2; ++k)
					{
						if ((k != i && l != j) &&
							TwoBitArray_checkCoords (camOccupancyGrid->grid, k, l, CAM_OCCUPANCY_GRID_X) != 0)
						{
							BitArray_setCoords (g_tempOccupancyGrid, k, l, CAM_OCCUPANCY_GRID_X, 1);
						}
					}
				}
			}
		}
	}

	// Diff cam grid and temp grid, anything that has been set in the temp grid and is not a robot
	// cell can be marked as unknown
	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		if (BitArray_checkElement_index (g_tempOccupancyGrid, i) &&
			TwoBitArray_checkIndex (camOccupancyGrid->grid, i) != ROBOT_OCCUPIED_FLAG)
		{
			TwoBitArray_setIndex (camOccupancyGrid->grid, i, 2/*unknown*/);
		}
	}
}

int ObstacleRecognition_estimateGridOccupancy (
//	FILE *xmlLog,
	const int camOccupancyGridY,
	OccupancyGrid *camOccupancyGrid,
	const float *camScoreGrid,
	const int isAnyRobotCell)
{
	int i, j;
	uchar gridVal;
	int isOccupiedEst;
	int isAnyOccupiedEst = 0;

	for (j = 0; j < camOccupancyGridY; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			gridVal = TwoBitArray_checkCoords (camOccupancyGrid->grid, i, j, CAM_OCCUPANCY_GRID_X);

			// If already marked as a robot, then do not need to determine occupancy.
			if (gridVal == ROBOT_OCCUPIED_FLAG)
			{
				continue;
			}

			isOccupiedEst = ObstacleRecognition_estimateCellOccupancy (camScoreGrid[i + j * CAM_OCCUPANCY_GRID_X]);
			isAnyOccupiedEst |= (int)(isOccupiedEst > (uchar)1);
			TwoBitArray_setCoords (camOccupancyGrid->grid, i, j, CAM_OCCUPANCY_GRID_X, (uchar)isOccupiedEst);
		}
	}

	if (isAnyRobotCell)
	{
		ObstacleRecognition_fillRobotCells (camOccupancyGrid);

		isAnyOccupiedEst = 0;
		for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
		{
			isOccupiedEst = TwoBitArray_checkIndex (camOccupancyGrid->grid, i);
			isAnyOccupiedEst |= (int)(isOccupiedEst > (uchar)1);
		}
	}

	return isAnyOccupiedEst;
}

void ObstacleRecognition_calcImageFeatures (
	Image *camImg,
	ImgCellFeatures cellFeatures[SIZE_COOP_LOC_GRID],
	ImgFeatures *imgFeatures)
{
	// Calculate features over entire grid. Robot recognition will typically
	// use all cells, while obstacle recognition will not use cells further
	// into the distance.
	ObstacleRecognitionCore_calcImgCellFeatures (camImg, cellFeatures, CAM_OCCUPANCY_GRID_Y, CAM_OCCUPANCY_GRID_ORIGIN_X, CAM_OCCUPANCY_GRID_ORIGIN_Y);
	ObstacleRecognitionCore_calcImgFeatures (imgFeatures, cellFeatures, CAM_OCCUPANCY_GRID_Y);
}

void ObstacleRecognition_calcOccupancy (
//	FILE *xmlLog,
	Image *camImg,
	ImgCellFeatures cellFeatures[SIZE_COOP_LOC_GRID],
	ImgFeatures *imgFeatures,
	OccupancyGrid *camOccupancyGrid,
	float camScoreGrid[SIZE_COOP_LOC_GRID],
	const int anyRobotCells,
	int *isObstacleInCamOccupancyGrid)
{
	ObstacleRecognitionModel_calcObstacleGroupScores (
		imgFeatures,
		cellFeatures,
		camScoreGrid,
		CAM_OCCUPANCY_GRID_Y);

	*isObstacleInCamOccupancyGrid = ObstacleRecognition_estimateGridOccupancy (
//		xmlLog,
		CAM_OCCUPANCY_GRID_Y_EXP,
		camOccupancyGrid,
		camScoreGrid,
		anyRobotCells);
}

#endif // #ifndef BOARD















