#include "RobotRecognitionBlobDetection.h"
#include "RobotRecognitionCore.h"
#include "../../Common/RobotCore.h"
#include "../../Common/List.h"
#include "../../Common/Image.h"
#include "../../Common/BitArray.h"
#include "../../Common/Geometry.h"

#ifndef BOARD

int RobotRecognitionBlobDetection_isBlobValid (
	const ColourBlob *blob,
	const int includeSmallBlobs)
{
	if (includeSmallBlobs)
	{
		return (blob->isUnique && !blob->isConflicting && (blob->blobType == BLOB_VALID_BUT_TOO_SMALL || blob->blobType == BLOB_VALID));
	}
	else
	{
		return (blob->isUnique && !blob->isConflicting && blob->blobType == BLOB_VALID);
	}
}

#if VERBOSE_BLOB_DETECTION
void RobotRecognitionBlobDetection_printBlobGridParams (
	FILE *outputFile,
	const uchar *blobGrid)
{
	int cellIndex;
	int nBlobs, nBlobCells;
	uchar blobIds[256];
	uchar blobGridVal;
	const uchar *blobGridPtr;
	int blobId;

	memset (blobIds, 0, 256);
	nBlobs = 0;
	nBlobCells = 0;
	blobGridPtr = blobGrid;
	for (cellIndex = 0; cellIndex < SIZE_COOP_LOC_GRID; ++cellIndex)
	{
		blobGridVal = *blobGridPtr;
		++blobGridPtr;

		if (blobGridVal != 255)
		{
			++nBlobCells;

			if (!blobIds[blobGridVal])
			{
				++nBlobs;
			}

			// Max num we can count - not important anyway
			if (blobIds[blobGridVal] != 255)
			{
				++blobIds[blobGridVal];
			}
		}
	}

	fprintf (outputFile, "<VerifyBlobs>nBlobs=%d nCells=%d<VerifyBlobs>\n", nBlobs, nBlobCells);

	if (nBlobs)
	{
		for (blobId = 0; blobId < 256; ++blobId)
		{
			if (blobIds[blobId] != 0)
			{
				fprintf (outputFile, "<VerifyBlob>id=%d nCells=%d</VerifyBlob>\n", blobId, blobIds[blobId]);
			}
		}
	}
}

void RobotRecognitionBlobDetection_displayRobotCells (
	IplImage *iplImage,
	const int *windowSize,
	Image *originalImg,
	const OccupancyGrid *occupancyGrid)
{
	// Refresh IplImage before drawing robot cells
	memcpy (iplImage->imageData, originalImg->data, originalImg->height * originalImg->wStep);

	Image_drawTwoBitArray (
		iplImage->imageData,
		occupancyGrid->grid,
		CAM_OCCUPANCY_GRID_X,
		CAM_OCCUPANCY_GRID_Y,
		CAM_OCCUPANCY_GRID_ORIGIN_X,
		CAM_OCCUPANCY_GRID_ORIGIN_Y,
		ROBOT_OCCUPIED_FLAG);

	//Image_drawOccupancy (
	//	iplImage->imageData,
	//	occupancyGrid->grid,
	//	CAM_OCCUPANCY_GRID_ORIGIN_X,
	//	CAM_OCCUPANCY_GRID_ORIGIN_Y,
	//	CAM_OCCUPANCY_GRID_Y,
	//	ROBOT_OCCUPIED_FLAG);

	cvNamedWindow ("robotCells", windowSize);
	cvShowImage ("robotCells", iplImage);
//	cvWaitKey (0);
	cvWaitKey (1);
}

void RobotRecognitionBlobDetection_displayBlobs (
	FILE *outputFile,
	IplImage *iplImage,
	const int *windowSize,
	Image *originalImg,
	List *colourBlobs,
	const int nValidBlobs)
{
	ColourBlob *colourBlobPtr;
	ListNode *iter;
	char win[24];
	int i, j;

	i = 0;
	j = 0;
	iter = colourBlobs->front;
	while (iter)
	{
		colourBlobPtr = (ColourBlob*)iter->value;
//		if (0)
		if (!RobotRecognitionBlobDetection_isBlobValid (colourBlobPtr, 1))
		{
			iter = iter->next;
			continue;
		}
		/*if (1 == nValidBlobs)
		{
			iter = iter->next;
			continue;
		}*/

		// Refresh IplImage before drawing cells for this blob
		memcpy (iplImage->imageData, originalImg->data, originalImg->height * originalImg->wStep);

		Image_drawOccupancy (
			iplImage->imageData,
			colourBlobPtr->blobGrid,
			CAM_OCCUPANCY_GRID_ORIGIN_X,
			CAM_OCCUPANCY_GRID_ORIGIN_Y,
			CAM_OCCUPANCY_GRID_Y,
			colourBlobPtr->id);

		if (RobotRecognitionBlobDetection_isBlobValid (colourBlobPtr, 1))
		{
			sprintf (win, "validBlob%02d", i++);
			cvNamedWindow (win, windowSize);
			cvShowImage (win, iplImage);
		}
		else
		{
			sprintf (win, "invalidBlob%02d", j++);
			cvNamedWindow (win, windowSize);
			cvShowImage (win, iplImage);
		}
//		cvWaitKey (0);
		cvWaitKey (1);

		iter = iter->next;
	}
}
#endif

void RobotRecognitionBlobDetection_growBlobAroundSeedPt (
	const int i,
	const int j,
	const float *colourScoreGrid,
	uchar *blobGrid,
	const int colourIndex)
{
	int k, l;
	int cellIndex;

	for (k = max (0, i - 1); k < min (CAM_OCCUPANCY_GRID_X - 1, i + 2); ++k)
	{
		for (l = max (0, j - 1); l < min (CAM_OCCUPANCY_GRID_Y - 1, j + 2); ++l)
		{
			if (k == i && l == j)
			{
				continue;
			}
			cellIndex = OCCUP_COORD(k, l);

			if (blobGrid[cellIndex] == 255 &&
				colourScoreGrid[cellIndex * N_ROBOT_COLOURS + colourIndex] < COLOUR_SCORE_NORMAL_THRESHOLD)
			{
				blobGrid[cellIndex] = 0;
			}
		}
	}
}

//! Add nbour influence grid
void RobotRecognitionBlobDetection_growBlobAroundSeedPtNEW (
	const int i,
	const int j,
	const float *colourScoreGrid,
	const float *nbourInfluenceGrid,
	uchar *blobGrid,
	const int colourIndex)
{
	int k, l;
	int cellIndex;
	float score;

	for (k = max (0, i - 1); k < min (CAM_OCCUPANCY_GRID_X - 1, i + 2); ++k)
	{
		for (l = max (0, j - 1); l < min (CAM_OCCUPANCY_GRID_Y - 1, j + 2); ++l)
		{
			if (k == i && l == j)
			{
				continue;
			}
			cellIndex = OCCUP_COORD(k, l);

			score =
				colourScoreGrid[cellIndex * N_ROBOT_COLOURS + colourIndex] + 
				nbourInfluenceGrid[cellIndex] * NBOUR_INFLUENCE_COEFF;

			if (blobGrid[cellIndex] == 255 &&
				score < COLOUR_SCORE_NORMAL_THRESHOLD)
			{
				blobGrid[cellIndex] = 0;
			}
		}
	}
}

void RobotRecognitionBlobDetection_createInitialBlobs (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	const float *colourScoreGrid,
	uchar *blobGrid,
	const int colourIndex)
{
	int i, j;
	int cellIndex;
#if VERBOSE_BLOB_DETECTION
	int nTotal;
	int nSeeds = 0;
#endif

	cellIndex = 0;
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			if (blobGrid[cellIndex] == 255 &&
				colourScoreGrid[cellIndex * N_ROBOT_COLOURS + colourIndex] < COLOUR_SCORE_SEED_THRESHOLD)
			{
				blobGrid[cellIndex] = 0;

				RobotRecognitionBlobDetection_growBlobAroundSeedPt (
					i,
					j,
					colourScoreGrid,
					blobGrid,
					colourIndex);
#if VERBOSE_BLOB_DETECTION
				++nSeeds;
#endif
			}

			++cellIndex;
		}
	}

#if VERBOSE_BLOB_DETECTION
	nTotal = 0;
	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		nTotal += (int)(blobGrid[i] == 0);
	}
	fprintf (outputFile, "%d seed pts, %d pts from initial blob seeds\n", nSeeds, nTotal);
#endif
}

void RobotRecognitionBlobDetection_createInitialBlobsNEW (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	const float *colourScoreGrid,
	const float *nbourInfluenceGrid,
	uchar *blobGrid,
	const int colourIndex)
{
	int i, j;
	int cellIndex;
	float score;
#if VERBOSE_BLOB_DETECTION
	int nTotal;
	int nSeeds = 0;
#endif

	cellIndex = 0;
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			score =
				colourScoreGrid[cellIndex * N_ROBOT_COLOURS + colourIndex] +
				nbourInfluenceGrid[cellIndex] * NBOUR_INFLUENCE_COEFF;

//			if (blobGrid[cellIndex] == 255 && score < COLOUR_SCORE_SEED_THRESHOLD)
			if ((blobGrid[cellIndex] == 255 || blobGrid[cellIndex] == 0) && score < COLOUR_SCORE_SEED_THRESHOLD)
			{
				blobGrid[cellIndex] = 0;

				RobotRecognitionBlobDetection_growBlobAroundSeedPtNEW (
					i,
					j,
					colourScoreGrid,
					nbourInfluenceGrid,
					blobGrid,
					colourIndex);
#if VERBOSE_BLOB_DETECTION
				++nSeeds;
#endif
			}

			++cellIndex;
		}
	}

#if VERBOSE_BLOB_DETECTION
	nTotal = 0;
	for (i = 0; i < SIZE_COOP_LOC_GRID; ++i)
	{
		nTotal += (int)(blobGrid[i] == 0);
	}
	fprintf (outputFile, " nSeedPts=%d nPts=%d\n", nSeeds, nTotal);
#endif
}

void RobotRecognitionBlobDetection_propogateBlobJoins (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	List *blobsToJoinList)
{
	int wasJoinPropogated;
	PointI *ptPtr, *ptPtr2, *ptPtr3, *ptPtr4;
	ListNode *iter, *iter2;

	do
	{
		wasJoinPropogated = 0;

#if VERBOSE_BLOB_DETECTION
		fprintf (outputFile, "<CurrentJoins>");
		iter = blobsToJoinList->front;
		while (iter)
		{
			ptPtr = (PointI*)iter->value;
			fprintf (outputFile, "%d<-%d, ", ptPtr->x, ptPtr->y);
			iter = iter->next;
		}
		fprintf (outputFile, "</CurrentJoins>\n");
		fflush (outputFile);
#endif
		// Propogate sequences of joins, e.g. 22<23 and 23<24 go to 22<23 and 22<24
		iter = blobsToJoinList->front;
		while (iter)
		{
			ptPtr = (PointI*)iter->value;

			iter2 = blobsToJoinList->front;
			while (iter2)
			{
				ptPtr2 = (PointI*)iter2->value;
				if (ptPtr2->x == ptPtr->y)
				{
#if VERBOSE_BLOB_DETECTION
					fprintf (
						outputFile,
						"<PropogateBlobJoin>%d<-%d %d<-%d<PropogateBlobJoin>\n",
						ptPtr2->x, ptPtr2->y,
						ptPtr->x, ptPtr->y);
					fflush (outputFile);
#endif
					ptPtr3 = ptPtr;
					ptPtr4 = ptPtr2;
					if (ptPtr->x > ptPtr2->x)
					{
						ptPtr3 = ptPtr2;
						ptPtr4 = ptPtr;
					}

					ptPtr4->x = ptPtr3->x;

					wasJoinPropogated = 1;
				}
				iter2 = iter2->next;
			}
			iter = iter->next;
		}

		List_uniqueify (blobsToJoinList, PointI_comparePtrs);

		// Propogate joins from common source blobColours, e.g. A<C and B<C go to A<B and A<C
		// So if 2 joins have a common source, get the dest from each and join the higher dest to
		// the lower. The original join to the higher dest can then be removed.
		iter = blobsToJoinList->front;
		while (iter)
		{
			ptPtr = (PointI*)iter->value;

			iter2 = blobsToJoinList->front;
			while (iter2)
			{
				ptPtr2 = (PointI*)iter2->value;
				if (ptPtr2 != ptPtr && !PointI_comparePtrs (ptPtr2, ptPtr) && ptPtr2->y == ptPtr->y)
				{
					// Both joins have the same source, but obviously a different dest. Find the
					// join with the higher dest, as this won't be needed anymore.
					ptPtr3 = ptPtr;
					ptPtr4 = ptPtr2;
					if (ptPtr->x > ptPtr2->x)
					{
						ptPtr3 = ptPtr2;
						ptPtr4 = ptPtr;
					}

#if VERBOSE_BLOB_DETECTION
					fprintf (
						outputFile,
						"<CombineBlobJoins>%d<-%d %d<-%d<CombineBlobJoins>\n",
						ptPtr4->x, ptPtr4->y,
						ptPtr3->x, ptPtr3->y);
					fflush (outputFile);
#endif

					// Instead of removing one join and adding another, just alter the contents
					// of the join we're removing. ptPtr4 pts to the join with the greater dest,
					// this will become A<B.
					ptPtr4->y = ptPtr4->x;
					ptPtr4->x = ptPtr3->x;

					wasJoinPropogated = 1;
				}
				iter2 = iter2->next;
			}
			iter = iter->next;
		}

		List_uniqueify (blobsToJoinList, PointI_comparePtrs);
	}
	while (wasJoinPropogated);

#if VERBOSE_BLOB_DETECTION
	if (blobsToJoinList->size)
	{
		iter = blobsToJoinList->front;
		while (iter)
		{
			ptPtr = (PointI*)iter->value;
			fprintf (outputFile, "<JoinBlobColours>dest=%d src=%d<JoinBlobColours>\n", ptPtr->x, ptPtr->y);
			iter = iter->next;
		}
	}
#endif
}

void RobotRecognitionBlobDetection_joinBlobPairs (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	uchar *blobGrid,
	List *blobstoJoinList)
{
	int i, j;
	uchar blobId, *blobGridPtr;
	ListNode *iter;
	PointI *pt;

	blobGridPtr = blobGrid;
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			blobId = *blobGridPtr;
			if (blobId != 255)
			{
				iter = blobstoJoinList->front;
				while (iter)
				{
					pt = (PointI*)iter->value;
					if (blobId == pt->y)
					{
						*blobGridPtr = pt->x;
						break;
					}
					iter = iter->next;
				}
			}
			++blobGridPtr;
		}
	}
}

#if 1 || VERBOSE_BLOB_DETECTION
extern const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS];
#endif

void RobotRecognitionBlobDetection_joinAdjacentBlobs (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	uchar *blobGrid,
	const int colourIndex/*,
	List *colourBlobs*/)
{
	int i, j, k, l, cellIndex;
	uchar idThisBlob, idNbour, idNextBlob;
	PointI blobsToJoin, *blobstoJoinPtr;
	List blobsToJoinList;

	blobsToJoinList = initList();

	idNextBlob = 1;
	cellIndex = 0;
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			if (blobGrid[cellIndex] == 0)
			{
				// Mark this cell as not joined to any blobs
				idThisBlob = 0;

				// Determine blob is for this cell
				for (l = max (j - 1, 0); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
				{
					for (k = max (i - 1, 0); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
					{
						if (k == i && l == j)
						{
							continue;
						}

						idNbour = blobGrid[OCCUP_COORD(k, l)];
						if (idNbour != 255 && idNbour != 0)
						{
							// This cell was not assigned to a blob yet, add to nbours blob
							if (idThisBlob == 0)
							{
								idThisBlob = idNbour;
							}

							// This cell and this nbour are in different groups
							else if (idThisBlob != idNbour)
							{
								blobsToJoin.x = min (idThisBlob, idNbour);
								blobsToJoin.y = max (idThisBlob, idNbour);
								if (!List_isElement(
									&blobsToJoinList,
									&blobsToJoin,
									PointI_comparePtrs))
								{
									blobstoJoinPtr = (PointI*)malloc (sizeof (PointI));
									*blobstoJoinPtr = blobsToJoin;
									List_pushValue (&blobsToJoinList, blobstoJoinPtr);
#if VERBOSE_BLOB_DETECTION
									fprintf (
										outputFile,
										"<JoinBlobs>dest=%d src=%d pt=(%d,%d)</JoinBlobs>\n",
										blobsToJoin.x, blobsToJoin.y,
										i, j);
#endif
								}
							}
						}
					}
				}

				// If cell hasn't been joined to any nbour, add a new blob
				if (idThisBlob == 0)
				{
					idThisBlob = (RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex]) + idNextBlob++;
#if VERBOSE_BLOB_DETECTION
					fprintf (outputFile, "<NewBlob>id=%d pt=(%d,%d)</NewBlob>\n", idThisBlob, i, j);
#endif
				}

				// Set id for this cell
				blobGrid[cellIndex] = idThisBlob;
			}
			++cellIndex;
		}
	}

	RobotRecognitionBlobDetection_propogateBlobJoins (
#if VERBOSE_BLOB_DETECTION
		outputFile,
#endif
		&blobsToJoinList);

	RobotRecognitionBlobDetection_joinBlobPairs (
#if VERBOSE_BLOB_DETECTION
		outputFile,
#endif
		blobGrid,
		&blobsToJoinList);

	List_clear (&blobsToJoinList, 1);
}

//! Calculate blob density score
/*!
Count number of cells that nbour a blob cell
*/
float RobotRecognitionBlobDetection_calcBlobDensityScore (
	const uchar blobId,
	const uchar *blobGrid,
	const int nCells,
	const PointI bl,
	const PointI tr)
{
	int i, j;
	int k, l;
	int cellIndex;
	int nNbourCells;
	float densityScore;

	nNbourCells = 0;
	for (j = max (0, bl.y - 1); j < min (tr.y + 2, CAM_OCCUPANCY_GRID_Y); ++j)
	{
		for (i = max (0, bl.x - 1); i < min (tr.x + 2, CAM_OCCUPANCY_GRID_X); ++i)
		{
			cellIndex = i + j * CAM_OCCUPANCY_GRID_X;

			if (blobGrid[cellIndex] != blobId)
			{
				for (l = max (0, max (bl.y - 1, j - 1)); l < min (min (tr.y + 2, j + 2), CAM_OCCUPANCY_GRID_Y); ++l)
				{
					for (k = max (0, max (bl.x - 1, i - 1)); k < min (min (tr.x + 2, i + 2), CAM_OCCUPANCY_GRID_X); ++k)
					{
						if (blobGrid[l * CAM_OCCUPANCY_GRID_X + k] == blobId)
						{
							// (i, j) is a nbour cell
							++nNbourCells;
							goto RobotRecognitionBlobDetection_calcBlobDensityScore_isNbour;
						}
					}
				}
RobotRecognitionBlobDetection_calcBlobDensityScore_isNbour:
				; // Break from 2 loops
			}
		}
	}

	densityScore = (float)nCells / nNbourCells;
	return densityScore;
}

//! Calculate blob density score
/*!
Number of blob cells with a non-blob nbour
*/
float RobotRecognitionBlobDetection_calcBlobDensityScoreNEW (
	const uchar blobId,
	const uchar *blobGrid,
	const int nCells,
	const PointI bl,
	const PointI tr)
{
	int i, j;
	int k, l;
	int cellIndex;
	int nNbourCells;
	float densityScore;

	nNbourCells = 0;
	for (j = bl.y; j < tr.y + 2; ++j)
	{
		for (i = bl.x; i < tr.x + 2; ++i)
		{
			cellIndex = i + j * CAM_OCCUPANCY_GRID_X;

			if (blobGrid[cellIndex] == blobId)
			{
				for (l = max (0, j - 1); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
				{
					for (k = max (0, i - 1); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
					{
						if (blobGrid[l * CAM_OCCUPANCY_GRID_X + k] != blobId)
						{
							// (k, l) is a nbour cell
							++nNbourCells;
							goto RobotRecognitionBlobDetection_calcBlobDensityScore_isNbour;
						}
					}
				}
RobotRecognitionBlobDetection_calcBlobDensityScore_isNbour:
				; // Break from 2 loops
			}
		}
	}

	densityScore = (float)nCells / nNbourCells;
	return densityScore;
}


//! Calculate blob density score
/*!
Avg num nbour cells
*/
float RobotRecognitionBlobDetection_calcBlobDensityScoreNEW2 (
	const uchar blobId,
	const uchar *blobGrid,
	const int nCells,
	const PointI bl,
	const PointI tr)
{
	int i, j;
	int k, l;
	int cellIndex;
	int thisCellHasNbours;
	int nCellsWithNbours;
	int nNbourCells;
	float densityScore;

	nCellsWithNbours = 0;
	nNbourCells = 0;
	for (j = bl.y; j < tr.y + 2; ++j)
	{
		for (i = bl.x; i < tr.x + 2; ++i)
		{
			cellIndex = i + j * CAM_OCCUPANCY_GRID_X;

			if (blobGrid[cellIndex] == blobId)
			{
				thisCellHasNbours = 0;
				for (l = max (0, j - 1); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
				{
					for (k = max (0, i - 1); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
					{
						if (blobGrid[l * CAM_OCCUPANCY_GRID_X + k] != blobId)
						{
							thisCellHasNbours = 1;
							++nNbourCells;
						}
					}
				}
				nCellsWithNbours += thisCellHasNbours;
			}
		}
	}

	densityScore = (float)nNbourCells / nCellsWithNbours;
	return densityScore;
}

//! Calculate blob density score
/*!
For each blob cell w nbours, accumulate (nNbours- idealNNbours)
*/
float RobotRecognitionBlobDetection_calcBlobDensityScoreNEW3 (
	const uchar blobId,
	const uchar *blobGrid,
	const int nCells,
	const PointI bl,
	const PointI tr)
{
	int i, j;
	int k, l;
	int cellIndex;
	int nNboursThisCell;
	int nCellsWithNbours;
	int accumOffset;
	float densityScore;


	nCellsWithNbours = 0;
	accumOffset = 0;
	for (j = bl.y; j < tr.y + 2; ++j)
	{
		for (i = bl.x; i < tr.x + 2; ++i)
		{
			cellIndex = i + j * CAM_OCCUPANCY_GRID_X;

			if (blobGrid[cellIndex] == blobId)
			{
				nNboursThisCell = 0;
				for (l = max (0, j - 1); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
				{
					for (k = max (0, i - 1); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
					{
						if (blobGrid[l * CAM_OCCUPANCY_GRID_X + k] != blobId)
						{
							++nNboursThisCell;
						}
					}
				}
				if (nNboursThisCell)
				{
					++nCellsWithNbours;
					accumOffset += abs(nNboursThisCell - 3);
				}
			}
		}
	}

	densityScore = (float)accumOffset / nCellsWithNbours;
	return densityScore;
}

//! Determine whether or not the blob meets the minimum requirements
BlobType RobotRecognitionBlobDetection_determineBlobType (
	const float colourScore,
	const float densityScore,
	const int nCells)
{
	float combdScore = (colourScore / BLOB_MAX_COLOUR_SCORE) + (densityScore / BLOB_MAX_DENSITY_SCORE);
//	if (colourScore > BLOB_MAX_COLOUR_SCORE || densityScore > BLOB_MAX_DENSITY_SCORE)
	if (combdScore >= 2.0f)
	{
		return BLOB_INVALID;
	}
	if (nCells >= BLOB_MIN_NCELLS)
	{
		return BLOB_VALID;
	}
	else if (nCells >= BLOB_MIN_NCELLS_SMALLER)
	{
		return BLOB_VALID_BUT_TOO_SMALL;
	}
	return BLOB_INVALID;
}

int RobotRecognitionBlobDetection_createBlobsFromGrid (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
	const uchar *colourIdsThisTrainingImg,
#endif
	const uchar *blobGrid,
	const float *colourScoreGrid,
	const int colourIndex,
	List *colourBlobs)
{
	int cellIndex, blobId;
	int i, j;
	uchar blobIdsVal, *blobIdsPtr;
	uchar blobIdsPresent[256];
	uchar blobGridVal;
	const uchar *blobGridPtr;
	ColourBlob *colourBlobPtr;
	ListNode *lastNode;
	ListNode *iter;
	PointI bl, tr;
	int nBlobs, areBlobsUnique;
	int nCells;
	float colourScoreAvg;

	memset (blobIdsPresent, 0, 256);

	blobGridPtr = blobGrid;
	for (cellIndex = 0; cellIndex < SIZE_COOP_LOC_GRID; ++cellIndex)
	{
		blobGridVal = *blobGridPtr;
		if (blobGridVal != 255)
		{
			blobIdsPresent[blobGridVal] = (uchar)1;
		}
		++blobGridPtr;
	}

	// Record where to start from in list when processing new blobs
	lastNode = colourBlobs->back;

	nBlobs = 0;
	blobIdsPtr = blobIdsPresent;
	for (blobId = 0; blobId < 256; ++blobId)
	{
		blobIdsVal = *blobIdsPtr;
		if (blobIdsVal == (uchar)1)
		{
			++nBlobs;
			colourBlobPtr = ColourBlob3_alloc (colourIndex, blobId);
#if VERBOSE_BLOB_DETECTION
			memcpy (colourBlobPtr->blobGrid, blobGrid, SIZE_COOP_LOC_GRID);
#endif
			List_pushValue (colourBlobs, colourBlobPtr);
		}
		++blobIdsPtr;
	}

	areBlobsUnique = (nBlobs < 2);
#if VERBOSE_BLOB_DETECTION
	if (!areBlobsUnique)
	{
		fprintf (
			outputFile,
			"<BlobsNotUnique>nBlobs=%d colourIndex=%d colourId=%d</BlobsNotUnique>\n",
			nBlobs,
			colourIndex,
			RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex]);
	}
#endif

	if (lastNode)
	{
		iter = lastNode->next;
	}
	else
	{
		iter = colourBlobs->front;
	}
	while (iter)
	{
		colourBlobPtr = (ColourBlob*)iter->value;
		blobIdsVal = (uchar)colourBlobPtr->id;
		bl.x = MAX_FLT;
		bl.y = MAX_FLT;
		tr.x = MIN_FLT;
		tr.y = MIN_FLT;

		nCells = 0;
		colourScoreAvg = 0.0f;
		cellIndex = 0;
		blobGridPtr = blobGrid;
		for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
		{
			for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i, ++cellIndex, ++blobGridPtr)
			{
				blobGridVal = *blobGridPtr;
				if (blobGridVal == blobIdsVal)
				{
					bl.x = min (bl.x, i);
					bl.y = min (bl.y, j);
					tr.x = max (tr.x, i);
					tr.y = max (tr.y, j);

					++nCells;
					colourScoreAvg += colourScoreGrid[cellIndex * N_ROBOT_COLOURS + colourIndex];
				}
			}
		}
		colourScoreAvg /= nCells;

		colourBlobPtr->isUnique = areBlobsUnique;
		colourBlobPtr->bl = bl;
		colourBlobPtr->tr = tr;
		colourBlobPtr->nCells = nCells;
		colourBlobPtr->colourScore = colourScoreAvg;

		/*colourBlobPtr->densityScore = RobotRecognitionBlobDetection_calcBlobDensityScore (
			blobIdsVal,
			blobGrid,
			nCells,
			bl,
			tr);*/

		/*colourBlobPtr->densityScore = RobotRecognitionBlobDetection_calcBlobDensityScoreNEW (
			blobIdsVal,
			blobGrid,
			nCells,
			bl,
			tr);*/

		/*colourBlobPtr->densityScore = RobotRecognitionBlobDetection_calcBlobDensityScoreNEW2 (
			blobIdsVal,
			blobGrid,
			nCells,
			bl,
			tr);*/

		colourBlobPtr->densityScore = RobotRecognitionBlobDetection_calcBlobDensityScoreNEW3 (
			blobIdsVal,
			blobGrid,
			nCells,
			bl,
			tr);

		colourBlobPtr->blobType = RobotRecognitionBlobDetection_determineBlobType (
			colourBlobPtr->colourScore,
			colourBlobPtr->densityScore,
			colourBlobPtr->nCells);

#if VERBOSE_BLOB_DETECTION
		colourBlobPtr->isColourInImg = colourIdsThisTrainingImg[RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex]];
#endif

		iter = iter->next;
	}

	return nBlobs;
}

void RobotRecognitionBobDetection_removeBlobsPerColourIndex (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	const int colourIndex,
	List *blobs)
{
	ColourBlob *blobPtr;
	ListNode *iter;
#if VERBOSE_BLOB_DETECTION
	int nRemoved = 0;
#endif

	iter = blobs->front;
	while (iter)
	{
		blobPtr = (ColourBlob*)iter->value;
		if (blobPtr->colourIndex == colourIndex)
		{
			// Delete element from list and pt to next element
			List_deleteElement (
				blobs,
				&iter,
				1,
				1);

#if VERBOSE_BLOB_DETECTION
			++nRemoved;
#endif
		}
		else
		{
			iter = iter->next;
		}
	}

#if VERBOSE_BLOB_DETECTION
	fprintf (outputFile, "Removed %d blobs for colourIndex %d\n", nRemoved, colourIndex);
#endif
}

void RobotRecognitionBobDetection_resetBlobGridIds (
	uchar *blobGrid)
{
	int i;
	uchar *ptr;
	for (i = 0, ptr = blobGrid; i < SIZE_COOP_LOC_GRID; ++i, ++ptr)
	{
		if ((*ptr) != 255)
		{
			(*ptr) = 0;
		}
	}
}


//! Simple AABB collision algorighm - boxes collide if they overlap along both x and y axes
int RobotRecognitionBlobDetection_doBlobsConflict (
	ColourBlob *blob1,
	ColourBlob *blob2)
{
	PointI dist;
	int isCollision;
//	isCollision =
//		abs((blob2->tr.x + blob2->bl.x) - (blob1->tr.x + blob1->bl.x)) <= ((blob2->tr.x - blob2->bl.x) + (blob1->tr.x - blob1->bl.x)) &&
//		abs((blob2->tr.y + blob2->bl.y) - (blob1->tr.y + blob1->bl.y)) <= ((blob2->tr.y - blob2->bl.y) + (blob1->tr.y - blob1->bl.y));

	// Less strict collision - check if either cog is inside the other AABB
	dist.x = abs((blob2->tr.x + blob2->bl.x) - (blob1->tr.x + blob1->bl.x)) / 2; // Dist between cogs
	dist.y = abs((blob2->tr.y + blob2->bl.y) - (blob1->tr.y + blob1->bl.y)) / 2;
	isCollision =
		(dist.x <= blob2->tr.x - blob2->bl.x) || (dist.x <= blob1->tr.x - blob1->bl.x) ||
		(dist.y <= blob2->tr.y - blob2->bl.y) || (dist.y <= blob1->tr.y - blob1->bl.y);
	return isCollision;
}

float RobotRecognitionBlobDetection_calcBlobStrength (
	const float colourScore,
	const float densityScore)
{
	// For colourScore, the smaller the better. For density, same.
	// Density for correct blob typically 0.
	return (colourScore * BLOB_STRENGTH_COLOUR_COEFF) + (densityScore * BLOB_STRENGTH_DENSITY_COEFF);
}

#if 1 || defined(SIMULATION)
void RobotRecognitionBlobDetection_printBlob (
	FILE *outputFile,
	ColourBlob *blob)
{
#if VERBOSE_BLOB_DETECTION
	int inImg = blob->isColourInImg;
#else
	int inImg = 0;
#endif

	fprintf (
		outputFile,
//		"<AnalyseBlob>colourIndex=%d colourId=%d id=%d isColourInImg=%d isValid=%d isUnique=%d isConflicting=%d blobType=%d strength=%f nCells=%d colourScore=%f densityScore=%f bl=(d,%d) tr=(%d,%d)</AnalyseBlob>\n",
		"<AnalyseBlob>colourIndex=%d colourId=%d colourInImg=%d id=%d isValid=%d isUnique=%d isConflicting=%d blobType=%d strength=%f nCells=%d colourScore=%f densityScore=%f bl=(%d,%d) tr=(%d,%d)</AnalyseBlob>\n",
		blob->colourIndex,
		RobotRecognitionModel_mapColourDescriptionArrayToColourId[blob->colourIndex],
//		blob->isColourInImg,
		inImg,
		blob->id,
//		blob->isColourInImg,
		RobotRecognitionBlobDetection_isBlobValid (blob, 1),
		blob->isUnique,
		blob->isConflicting,
		(int)blob->blobType,
		RobotRecognitionBlobDetection_calcBlobStrength (blob->colourScore, blob->densityScore),
		blob->nCells,
		blob->colourScore,
		blob->densityScore,
		blob->bl.x,
		blob->bl.y,
		blob->tr.x,
		blob->tr.y);
}
#endif

int RobotRecognitionBlobDetection_isFirstBlobStronger (
	ColourBlob *blob1,
	ColourBlob *blob2)
{
	float blob1Strength, blob2Strength;

	// Blob may already be conflicting with another different blob
	if (!blob2->isUnique || blob2->isConflicting)
	{
		return 1;
	}

	blob1Strength = RobotRecognitionBlobDetection_calcBlobStrength (blob1->colourScore, blob1->densityScore);
	blob2Strength = RobotRecognitionBlobDetection_calcBlobStrength (blob2->colourScore, blob2->densityScore);

	// Smaller value == better
	return (blob1Strength <= blob2Strength);
}

extern const RobotParams RobotRecognitionCore_robotParams[3];
extern const int RobotRecognitionCore_colourIndexToFaceTypeIndex[N_ROBOT_COLOURS];
extern const int RobotRecognitionCore_colourIndexToRobotIndex[N_ROBOT_COLOURS];


void RobotRecognitionBlobDetection_resolveConflict (
	FILE *outputFile,
	ColourBlob *blobPtr1,
	ColourBlob *blobPtr2,
	const RobotData *robotDataArray,
	const int ownRobotIndex,
	CamVectors *camVectors)
{
	float blob1Strength, blob2Strength;
	int faceTypeIndex1, faceTypeIndex2;
	int robotId1, robotId2;
	float locDist1, locDist2;
	float temp;
	PointI pixel;
	PointF ptPixelSpace;
	Vector3F relPtWorldSpace, ptWorldSpace;

	// Blob may already be conflicting with another different blob
	if (!blobPtr2->isUnique || blobPtr2->isConflicting)
	{
		return;
	}

	blob1Strength = RobotRecognitionBlobDetection_calcBlobStrength (blobPtr1->colourScore, blobPtr1->densityScore);
	blob2Strength = RobotRecognitionBlobDetection_calcBlobStrength (blobPtr2->colourScore, blobPtr2->densityScore);

	faceTypeIndex1 = RobotRecognitionCore_colourIndexToFaceTypeIndex[blobPtr1->colourIndex];
	faceTypeIndex2 = RobotRecognitionCore_colourIndexToFaceTypeIndex[blobPtr2->colourIndex];

	if (faceTypeIndex1 != 2 && faceTypeIndex2 != 2 &&
		fabs (blob1Strength / blob2Strength) - 1.0f < 0.2f)
	{
		robotId1 = RobotRecognitionCore_colourIndexToRobotIndex[blobPtr1->colourIndex];
		robotId2 = RobotRecognitionCore_colourIndexToRobotIndex[blobPtr2->colourIndex];
		DEBUG_ASSERT (robotId1 != ownRobotIndex)
		DEBUG_ASSERT (robotId2 != ownRobotIndex)

		pixel.x = ((blobPtr1->bl.x + blobPtr1->tr.x) / 2) * CAM_OCCUPANCY_CELL_X + CAM_OCCUPANCY_CELL_X_HALF;
		pixel.y = ((blobPtr1->bl.y + blobPtr1->tr.y) / 2) * CAM_OCCUPANCY_CELL_Y + CAM_OCCUPANCY_CELL_Y_HALF;
		ptPixelSpace = CamVectors_pixelToImageOffsets (pixel);
		relPtWorldSpace = CamVectors_projectImageToHorizontalPlane (
			camVectors,
			ptPixelSpace,
			35.0f); // Roughly half height of robot face types 0, 1
		ptWorldSpace.x = robotDataArray[ownRobotIndex].pose.loc.x + relPtWorldSpace.x;
		ptWorldSpace.y = robotDataArray[ownRobotIndex].pose.loc.y + relPtWorldSpace.y;

		locDist1 = Geometry_dist (
			ptWorldSpace.x, ptWorldSpace.y,
			robotDataArray[robotId1].pose.loc.x, robotDataArray[robotId1].pose.loc.y);

		locDist2 = Geometry_dist (
			ptWorldSpace.x, ptWorldSpace.y,
			robotDataArray[robotId2].pose.loc.x, robotDataArray[robotId2].pose.loc.y);

		// If both are within this dist, then possible/likely that either are in the image
		if (max (locDist1, locDist2) > 500.0f)
		{
			fprintf (outputFile, "<ResolveConflict>status=\"useDists\" strength1=%f strength2=%f dist1=%f dist2=%f",
				blob1Strength, blob2Strength, locDist1, locDist2);

			locDist1 /= robotDataArray[robotId1].stdDev;
			locDist2 /= robotDataArray[robotId2].stdDev;

			fprintf (outputFile, " stdDev1=%f stdDev2=%f val1=%f val2=%f",
				robotDataArray[robotId1].stdDev, robotDataArray[robotId2].stdDev,
				locDist1, locDist2);

			temp = max (locDist1, locDist2);
			blob1Strength *= (locDist1 / temp);
			blob2Strength *= (locDist2 / temp);

			fprintf (outputFile, " updatedStrength1=%f updatedStrength2=%f</ResolveConflict>\n", blob1Strength, blob2Strength);
		}
		else
		{
			fprintf (outputFile, "<ResolveConflict>status=\"tooSimilar\" strength1=%f strength2=%f dist1=%f dist2=%f</ResolveConflict>\n",
				blob1Strength, blob2Strength, locDist1, locDist2);
		}
	}

	if (blob1Strength <= blob2Strength) // Smaller scores are better
	{
		blobPtr2->isConflicting = 1;
	}
	else
	{
		blobPtr1->isConflicting = 1;
	}
}

extern const int RobotRecognitionCore_colourIndexToRobotIndex[N_ROBOT_COLOURS];

int RobotRecognitionBlobDetection_analyseBlobs (
#if VERBOSE_BLOB_DETECTION
	const uchar *colourIdsThisTrainingImg,
#endif
	const RobotData *robotDataArray,
	const int ownRobotIndex,
	CamVectors *camVectors,
	FILE *outputFile,
	List *colourBlobs)
{
	ColourBlob *blobPtr, *blobPtr2;
	ListNode *iter, *iter2;
	int isValid, nValidBlobs, isConflict;
	int r0, r1;
#if VERBOSE_BLOB_DETECTION
	int i;
#endif

	// Compare each pair of blobs to determine conflicts
	nValidBlobs = 0;
	iter = colourBlobs->front;
	while (iter)
	{
		blobPtr = (ColourBlob*)iter->value;

		iter2 = iter->next;
		while (iter2)
		{
			blobPtr2 = (ColourBlob*)iter2->value;

			isConflict = RobotRecognitionBlobDetection_doBlobsConflict (blobPtr, blobPtr2);
			if (isConflict)
			{
				r0 = RobotRecognitionCore_colourIndexToRobotIndex[blobPtr->colourIndex];
				r1 = RobotRecognitionCore_colourIndexToRobotIndex[blobPtr2->colourIndex];

				if (r0 == r1 || (r0 == -1 && r1 != -1) || (r0 != -1 && r1 == -1))
				{
					isConflict = 0;
				}
			}
			if (isConflict)
			{
				RobotRecognitionBlobDetection_resolveConflict (
					outputFile,
					blobPtr, blobPtr2,
					robotDataArray, ownRobotIndex, camVectors
					);
				/*if (RobotRecognitionBlobDetection_isFirstBlobStronger (blobPtr, blobPtr2))
				x
					blobPtr2->isConflicting = 1;
				}
				else
				{
					blobPtr->isConflicting = 1;
				}*/
			}

			iter2 = iter2->next;
		}

		iter = iter->next;
	}

	iter = colourBlobs->front;
	while (iter)
	{
		blobPtr = (ColourBlob*)iter->value;

		isValid = RobotRecognitionBlobDetection_isBlobValid (blobPtr, 1);
		nValidBlobs += isValid;

		iter = iter->next;
	}

	fprintf (outputFile, "<AnalyseBlobs>nValidBlobs=%d\n", nValidBlobs);

#if VERBOSE_BLOB_DETECTION
	if (!nValidBlobs)
	{
		printf ("No valid blobs in this image\n");
	}
#endif

	iter = colourBlobs->front;
	while (iter)
	{
		blobPtr = (ColourBlob*)iter->value;

		RobotRecognitionBlobDetection_printBlob (outputFile, blobPtr);

		iter = iter->next;
	}

#if VERBOSE_BLOB_DETECTION
	for (i = 0; i < 256; ++i)
	{
		if (colourIdsThisTrainingImg[i])
		{
			iter = colourBlobs->front;
			while (iter)
			{
				blobPtr = (ColourBlob*)iter->value;
				if (RobotRecognitionModel_mapColourDescriptionArrayToColourId[blobPtr->colourIndex] == i)
				{
					break;
				}
				iter = iter->next;
			}
			if (!iter)
			{
				printf ("colourId=%d should have a blob!\n", i);
				fprintf (outputFile, "<BlobMissing>colourId=%d</BlobMissing>\n", i);
			}
			else if (!RobotRecognitionBlobDetection_isBlobValid (blobPtr, 1))
			{
				printf ("colourId=%d has a blob but it is not valid!\n", i);
				fprintf (outputFile, "<BlobNotMissingButInvalid>colourId=%d</BlobNotMissingButInvalid>\n", i);
			}
		}
	}
	iter = colourBlobs->front;
	while (iter)
	{
		blobPtr = (ColourBlob*)iter->value;
		i = RobotRecognitionModel_mapColourDescriptionArrayToColourId[blobPtr->colourIndex];
		if (RobotRecognitionBlobDetection_isBlobValid (blobPtr, 1) && !colourIdsThisTrainingImg[i])
		{
			printf ("colourId=%d should not have a valid blob!\n", i);
			fprintf (outputFile, "<IncorrectBlob>colourId=%d</IncorrectBlob>\n", i);
		}
		iter = iter->next;
	}
#endif

	fprintf (outputFile, "</AnalyseBlobs>\n");

	return nValidBlobs;
}

extern const float RobotRecognitionModel_colourAvgs[N_ROBOT_COLOURS];

int RobotRecognitionBlobDetection_fillBlobNbours (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	const float *colourScoreGrid,
	ImgCellFeatures *cellFeatures,
	uchar *blobGrid,
	ColourBlob *blob,
	const int blobId,
	uchar *nbourGrid)
{
	int cellIndex, blobCellIndex, newNbourIndex;
	float nbourDiff;
	int i, j, k, l;
	int nAdded;
	float colourScore;

	nAdded = 0;
	for (cellIndex = 0; cellIndex < SIZE_COOP_LOC_GRID; ++cellIndex)
	{
		if (nbourGrid[cellIndex] != 255)
		{
			blobCellIndex = cellIndex + nbourGrid[cellIndex] - MAX_COOP_LOC_NBOUR_DIST;

			nbourDiff = RobotRecognitionCore_calcColourDiff (
				cellFeatures[cellIndex].vals,
				cellFeatures[blobCellIndex].vals);
			nbourDiff /= RobotRecognitionModel_colourAvgs[blob->colourIndex];

			colourScore = colourScoreGrid[blob->colourIndex + cellIndex * N_ROBOT_COLOURS];
			if (colourScore < COLOUR_SCORE_RELAXED_THRESHOLD_2 &&
				nbourDiff < NBOUR_SCORE_THRESHOLD)
			{
				++nAdded;

				DEBUG_ASSERT(blobGrid[cellIndex] == 255)
				blobGrid[cellIndex] = blobId;
				i = cellIndex % CAM_OCCUPANCY_GRID_X;
				j = cellIndex / CAM_OCCUPANCY_GRID_X;

#if VERBOSE_BLOB_DETECTION
				fprintf (
					outputFile,
					"Added %3d(%3d,%3d) for colour %d(id %d) score=%f nbourDiff=%f\n",
					cellIndex,
					i,
					j,
					blob->colourIndex,
					RobotRecognitionModel_mapColourDescriptionArrayToColourId[blob->colourIndex],
					colourScore,
					nbourDiff);
#endif

				for (l = max (0, j - 1); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
				{
					for (k = max (0, i - 1); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
					{
						if (k == i && l == j)
						{
							continue;
						}

						// Only add as a new nbour if it hasn't already been added as a nbour to something else, i.e.
						// we just added cellIndex, so better to be a nbour to an original cell
						newNbourIndex = k + l * CAM_OCCUPANCY_GRID_X;
						if (blobGrid[newNbourIndex] == 255 && nbourGrid[newNbourIndex] == 255)
						{
							nbourGrid[newNbourIndex] = (cellIndex - newNbourIndex) + MAX_COOP_LOC_NBOUR_DIST;
						}
					}
				}
			}
			// If the nbour wasn't added this iteration, do not consider it next time
			nbourGrid[cellIndex] = 255;
		}
	}

	return nAdded;
}

int RobotRecognitionBlobDetection_fillBlobNboursNEW (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	const float *colourScoreGrid,
	ImgCellFeatures *cellFeatures,
	uchar *blobGrid,
	ColourBlob *blob,
	const int blobId,
	uchar *nbourGrid)
{
	int cellIndex;
//	int blobCellIndex;
	int newNbourIndex;
	float nbourDiff;
	float bestNbourDiff;
	int i, j, k, l;
	int nAdded;
	float colourScore;

#if VERBOSE_BLOB_DETECTION
	fprintf (outputFile, "<FillBlob>blobId=%d colourIndex=%d cells=(", blob->id, blob->colourIndex);
#endif

	nAdded = 0;
	cellIndex = 0;
	for (j = 0; j < CAM_OCCUPANCY_GRID_Y; ++j)
	{
		for (i = 0; i < CAM_OCCUPANCY_GRID_X; ++i)
		{
			if (nbourGrid[cellIndex] != 255)
			{
				bestNbourDiff = MAX_FLT;
				for (l = max (0, j - 1); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
				{
					for (k = max (0, i - 1); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
					{
						if ((k == i && l == j) || blobGrid[k + l * CAM_OCCUPANCY_GRID_X] != blobId)
						{
							continue;
						}

						nbourDiff = RobotRecognitionCore_calcColourDiff (
							cellFeatures[cellIndex].vals,
							cellFeatures[k + l * CAM_OCCUPANCY_GRID_X].vals);
						nbourDiff /= RobotRecognitionModel_colourAvgs[blob->colourIndex];
						bestNbourDiff = min (nbourDiff, bestNbourDiff);
					}
				}
/*
				blobCellIndex = cellIndex + nbourGrid[cellIndex] - MAX_COOP_LOC_NBOUR_DIST;

				nbourDiff = RobotRecognitionCore_calcColourDiff (
					cellFeatures[cellIndex].vals,
					cellFeatures[blobCellIndex].vals);
				nbourDiff /= RobotRecognitionModel_colourAvgs[blob->colourIndex];
*/
				colourScore = colourScoreGrid[blob->colourIndex + cellIndex * N_ROBOT_COLOURS];
				if (colourScore < COLOUR_SCORE_NORMAL_THRESHOLD &&
//				if (colourScore < COLOUR_SCORE_RELAXED_THRESHOLD_2 &&
//					nbourDiff < NBOUR_SCORE_THRESHOLD)
					bestNbourDiff < NBOUR_SCORE_THRESHOLD)
				{
					++nAdded;

					DEBUG_ASSERT(blobGrid[cellIndex] == 255)
					blobGrid[cellIndex] = blobId;
					i = cellIndex % CAM_OCCUPANCY_GRID_X;
					j = cellIndex / CAM_OCCUPANCY_GRID_X;

#if VERBOSE_BLOB_DETECTION
					fprintf (outputFile, "(pt=(%d,%d) score=%f nbourDiff=%f), ",
						i, j, colourScore, nbourDiff);
#endif

					for (l = max (0, j - 1); l < min (j + 2, CAM_OCCUPANCY_GRID_Y); ++l)
					{
						for (k = max (0, i - 1); k < min (i + 2, CAM_OCCUPANCY_GRID_X); ++k)
						{
							if (k == i && l == j)
							{
								continue;
							}

							// Only add as a new nbour if it hasn't already been added as a nbour to something else, i.e.
							// we just added cellIndex, so better to be a nbour to an original cell
							newNbourIndex = k + l * CAM_OCCUPANCY_GRID_X;
							if (blobGrid[newNbourIndex] == 255 && nbourGrid[newNbourIndex] == 255)
							{
								nbourGrid[newNbourIndex] = (cellIndex - newNbourIndex) + MAX_COOP_LOC_NBOUR_DIST;
							}
						}
					}
				}
				// If the nbour wasn't added this iteration, do not consider it next time
				nbourGrid[cellIndex] = 255;
			}

			++cellIndex;
		}
	}

#if VERBOSE_BLOB_DETECTION
	fprintf (outputFile, ")</FillBlob>\n");
#endif

	return nAdded;
}

/*!
 - Given value (blob id), mark cells in grid that neighbour this.
 - fill blob - for each nbour cell, calc diff to all blob cells, get best nbour diff, get own colour score, add to blob if ok
 - if any blob was expanded, we then later delete all blobs for this colourIndex and create new blobs from the blobGrid
*/
int RobotRecognitionBlobDetection_fillBlobs (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
	const uchar *colourIdsThisTrainingImg,
#endif
	const float *colourScoreGrid,
	ImgCellFeatures *cellFeatures,
	uchar *blobGrid,
	List *colourBlobs,
	const int colourIndex)
{
	ColourBlob *blobPtr;
	ListNode *iter;
	uchar nbourGrid[SIZE_COOP_LOC_GRID];
	int nFillIterations;

	nFillIterations = 0;
	iter = colourBlobs->front;
	while (iter)
	{
		blobPtr = (ColourBlob*)iter->value;
//		if (blobPtr->colourIndex != colourIndex || !RobotRecognitionBlobDetection_isBlobValid (blobPtr, 1))
		if (blobPtr->colourIndex != colourIndex)
		{
			iter = iter->next;
			continue;
		}

		memset (nbourGrid, 255, SIZE_COOP_LOC_GRID);

		RobotRecognitionCore_getCellsNbouringValue (
			blobGrid,
			blobPtr->id,
			nbourGrid);

		while (RobotRecognitionBlobDetection_fillBlobNboursNEW (
#if VERBOSE_BLOB_DETECTION
			outputFile,
#endif
			colourScoreGrid,
			cellFeatures,
			blobGrid,
			blobPtr,
			blobPtr->id,
			nbourGrid))
		{
			++nFillIterations;
		}

		iter = iter->next;
	}

	return nFillIterations;
}

void RobotRecognitionBlobDetection_createBlobs (
	FILE *outputFile,
#if VERBOSE_BLOB_DETECTION
	IplImage *iplImage,
	const int *windowSize,
	Image *originalImg,
	const uchar *colourIdsThisTrainingImg,
#endif
	const int ownRobotIndex,
	const float *colourScoreGrid,
	ImgCellFeatures *cellFeatures,
	OccupancyGrid *occupancyGrid,
	List *colourBlobs)
{
	int colourIndex;
	uchar blobGrid[SIZE_COOP_LOC_GRID];
	float nbourInfluenceGrid[SIZE_COOP_LOC_GRID];
#if VERBOSE_BLOB_DETECTION
	int colourId;
#endif

	for (colourIndex = 0; colourIndex < N_ROBOT_COLOURS; ++colourIndex)
//	for (colourIndex = 6; colourIndex < 7; ++colourIndex)
	{
		if (RobotRecognitionCore_colourIndexToRobotIndex[colourIndex] == ownRobotIndex)
		{
#if VERBOSE_BLOB_DETECTION
			fprintf (outputFile, "<CreateBlobsSkipColour>colourIndex=%d robotId=%d</CreateBlobsSkipColour>\n", colourIndex, ownRobotIndex);
#endif
			continue;
		}
#if VERBOSE_BLOB_DETECTION
		// colourId isn't used in the rest of the processing, just printed here
		colourId = RobotRecognitionModel_mapColourDescriptionArrayToColourId[colourIndex];
		fprintf (outputFile, "<CreateBlobs>colourIndex=%d colourId=%d", colourIndex, colourId);
#endif

		RobotRecognitionCore_calcNbourInfluenceGrid (
#if VERBOSE_BLOB_DETECTION
			outputFile,
#endif
			colourScoreGrid,
			colourIndex,
			nbourInfluenceGrid);

		memset (blobGrid, 255, SIZE_COOP_LOC_GRID);

		RobotRecognitionBlobDetection_createInitialBlobsNEW (
#if VERBOSE_BLOB_DETECTION
			outputFile,
#endif
			colourScoreGrid,
			nbourInfluenceGrid,
			blobGrid,
			colourIndex);

		RobotRecognitionBlobDetection_joinAdjacentBlobs (
#if VERBOSE_BLOB_DETECTION
			outputFile,
#endif
			blobGrid,
			colourIndex/*,
			colourBlobs*/);

		if (RobotRecognitionBlobDetection_createBlobsFromGrid (
#if VERBOSE_BLOB_DETECTION
			outputFile,
			colourIdsThisTrainingImg,
#endif
			blobGrid,
			colourScoreGrid,
			colourIndex,
			colourBlobs))
		{
#if 1 // Enable/disale blob filling

			if (RobotRecognitionBlobDetection_fillBlobs (
#if VERBOSE_BLOB_DETECTION
				outputFile,
				colourIdsThisTrainingImg,
#endif
				colourScoreGrid,
				cellFeatures,
				blobGrid,
				colourBlobs,
				colourIndex))
			{
				RobotRecognitionBobDetection_removeBlobsPerColourIndex (
#if VERBOSE_BLOB_DETECTION
					outputFile,
#endif
					colourIndex,
					colourBlobs);

				RobotRecognitionBobDetection_resetBlobGridIds (
					blobGrid);

				RobotRecognitionBlobDetection_joinAdjacentBlobs (
#if VERBOSE_BLOB_DETECTION
					outputFile,
#endif
					blobGrid,
					colourIndex/*,
					colourBlobs*/);

				RobotRecognitionBlobDetection_createBlobsFromGrid (
#if VERBOSE_BLOB_DETECTION
					outputFile,
					colourIdsThisTrainingImg,
#endif
					blobGrid,
					colourScoreGrid,
					colourIndex,
					colourBlobs);
			}
#endif // 0/1
		}

#if VERBOSE_BLOB_DETECTION
		RobotRecognitionBlobDetection_printBlobGridParams (
			outputFile,
			blobGrid);

		fprintf (outputFile, "</CreateBlobs>\n");
#endif
	}
}






#endif
