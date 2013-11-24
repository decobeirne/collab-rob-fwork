#include "RobotRecognitionRoughEstimation.h"
#include "RobotRecognitionAccurateEstimation.h"
#include "ObstacleRecognitionCore.h"
#include "RobotRecognitionModel.h"
#include "../../Common/Vector.h"
#include "../../Common/Uncertainty.h"
#include "../../Common/Geometry.h"
#include "../../Common/Maths.h"

#ifndef BOARD

//! Line fit to edge in camera image
typedef struct _LineEst
{
	Line houghLine;
	int isEstOk;
} LineEst;

LineEst LineEst_init()
{
	LineEst e;
	e.houghLine = Line_init();
	e.isEstOk = 0;
	return e;
}

typedef struct _LocEst
{
	PointF locInWorld;
	UnionVector4F cov;
	int isEst;
} LocEst;

LocEst LocEst_init()
{
	LocEst l;
	l.locInWorld.x = MIN_FLT;
	l.locInWorld.y = MIN_FLT;
	l.isEst = 0;
	return l;
}

// Accumulate evidence for 2 lines at 5 degree intervals either side of
// the given expected orient. Let each bin count for 3 pixels. This gives
// a range of 240 pixels, which means that we can capture lines that
// intersect the x or y axis up to 40 pixels outside of the image bounds.
#define SIZE_SINGLE_HOUGH_BINS 90
#define N_HOUGH_ORIENTS 13
#define SIZE_HOUGH_BINS (SIZE_SINGLE_HOUGH_BINS * N_HOUGH_ORIENTS)
#define HOUGH_BINS_OFFSET 50

#define CHECK_HOUGH_LINE 1

#if CHECK_HOUGH_LINE
void RobotRecognitionAccurateEstimation_checkHoughLine (
	FILE *outputFile,
	const LineEst lineEst,
	const PointI *linePts,
	const int nLinePts)
{
	int i;
	PointF lineOrigin;
	PointF lineDir;
	PointF diff;
	PointF ptOnLine;
	float f;
	float avg;
	int n;

	if (lineEst.houghLine.axis)
	{
		lineOrigin.x = lineEst.houghLine.c;
		lineOrigin.y = 0.0f;
		lineDir.x = lineEst.houghLine.slope;
		lineDir.y = 1.0f;
	}
	else
	{
		lineOrigin.x = 0.0f;
		lineOrigin.y = lineEst.houghLine.c;
		lineDir.x = 1.0f;
		lineDir.y = lineEst.houghLine.slope;
	}
	f = sqrt(lineDir.x * lineDir.x + lineDir.y * lineDir.y);
	f = 1.0f / f;
	lineDir.x *= f;
	lineDir.y *= f;

	avg = 0.0f;
	n = 0;
	for (i = 0; i < nLinePts; ++i)
	{
		if (linePts[i].x != MIN_FLT && linePts[i].y != MIN_FLT)
		{
			diff.x = linePts[i].x - lineOrigin.x;
			diff.y = linePts[i].y - lineOrigin.y;

			// Can do a dot product on these if we just assume z is 0
			f = diff.x * lineDir.x + diff.y * lineDir.y;

			ptOnLine.x = lineOrigin.x + f * lineDir.x;
			ptOnLine.y = lineOrigin.y + f * lineDir.y;

			f = Geometry_dist (linePts[i].x, linePts[i].y, ptOnLine.x, ptOnLine.y);
			avg += f;
			++n;
		}
	}

	avg /= n;
	fprintf (outputFile, "<CheckHoughLine>nPts=%d avgDist=%f</CheckHoughLine>\n", n, avg);
}
#endif // if CHECK_HOUGH_LINE


LineEst RobotRecognitionAccurateEstimation_fitPtsToLine (
	FILE *outputFile,
	PointI *linePts,
	const int nLinePts,
	const float expectedEdgeSlope,
	const int useXAxis)
{
	int i;
	int c, bin;
	int bestSlopeIndex; // get rid of un initialised warning on this for linux
	int bestC; // this too
	int bestVal;
	float bestSlope;
	float slope;
	int slopeIndex;
	LineEst houghLineEst;
	uchar houghVals[SIZE_HOUGH_BINS];

	memset (houghVals, 0, SIZE_HOUGH_BINS);
	houghLineEst = LineEst_init();

	// Accumulate evidence for lines given points and orient. We
	// can rep a line as y=mx+c. We have m, and can take the x and y
	// from each pt to come up with c values.

	// Determine if more straight-forward to take slope as y/x (as
	// typical) or as x/y. The range of values will be closer to the
	// bounds of the image for a smaller orient. Also for extreme cases
	// this will be more accurate.

#if VERBOSE_BLOB_DETECTION > 2
	fprintf (outputFile, "<HoughLines>expectedSlope=%f xaxis=%d\n", expectedEdgeSlope, useXAxis);
#endif
	if (useXAxis)
	{
		for (
			slopeIndex = 0, slope = expectedEdgeSlope - 0.6f;
			slopeIndex < 13;
			++slopeIndex, slope += 0.1f)
		{
#if VERBOSE_BLOB_DETECTION > 2
			fprintf (outputFile, "<HoughLineForSlope>xaxis=1 slopeIndex=%d slope=%f\n", slopeIndex, slope);
#endif
			for (i = 0; i < nLinePts; ++i)
			{
				if (linePts[i].x == MIN_FLT || linePts[i].y == MIN_FLT)
				{
					continue;
				}
				// x=my+c... c=x-my
				c = linePts[i].x - (int)(slope * (float)linePts[i].y);
//				printf ("slope=%d", slopeIndex);
//				printf ("c=%d ", c);
//				bin = (c / 3) + HOUGH_BINS_OFFSET;
				bin = (c + HOUGH_BINS_OFFSET) / 3;
//				bin = ((linePts[i].x - (int)(slope * (float)linePts[i].y)) / 3) + HOUGH_BINS_OFFSET;
//				printf ("%d, ", bin);
//				printf ("bin=%d \n", bin);
				if (bin >= 0 && bin < (SIZE_SINGLE_HOUGH_BINS - 1))
				{
//				DEBUG_ASSERT((bin >=0) && (bin < (SIZE_SINGLE_HOUGH_BINS - 1)))
					++houghVals[bin + slopeIndex * SIZE_SINGLE_HOUGH_BINS];
				}
				else
				{
					linePts[i].x = MIN_FLT;
				}
#if VERBOSE_BLOB_DETECTION > 2
				fprintf (outputFile, "(%d,%d), ", c, bin);
#endif
			}
#if VERBOSE_BLOB_DETECTION > 2
			fprintf (outputFile, "\n</HoughLineForSlope>\n");
#endif
		}
	}
	else
	{
		for (
			slopeIndex = 0, slope = expectedEdgeSlope - 0.6f;
			slopeIndex < 13;
			++slopeIndex, slope += 0.1f)
		{
#if VERBOSE_BLOB_DETECTION > 2
			fprintf (outputFile, "<HoughLineForSlope>xaxis=0 slopeIndex=%d slope=%f\n", slopeIndex, slope);
#endif
			for (i = 0; i < nLinePts; ++i)
			{
				if (linePts[i].x == MIN_FLT || linePts[i].y == MIN_FLT)
				{
					continue;
				}
				// y=mx+c... c=y-mx
				c = linePts[i].y - (int)(slope * (float)linePts[i].x);
//				bin = (c / 3) + HOUGH_BINS_OFFSET;
				bin = (c + HOUGH_BINS_OFFSET) / 3;
				if (bin >= 0 && bin < (SIZE_SINGLE_HOUGH_BINS - 1))
				{
//				DEBUG_ASSERT((bin >=0) && (bin < (SIZE_SINGLE_HOUGH_BINS - 1)))
					++houghVals[bin + slopeIndex * SIZE_SINGLE_HOUGH_BINS];
				}
				else
				{
					linePts[i].x = MIN_FLT;
				}
#if VERBOSE_BLOB_DETECTION > 2
				fprintf (outputFile, "(%d,%d), ", c, bin);
#endif
			}
#if VERBOSE_BLOB_DETECTION > 2
			fprintf (outputFile, "\n</HoughLineForSlope>\n");
#endif
		}
	}

#if VERBOSE_BLOB_DETECTION > 2
	fprintf (outputFile, "<EvaluatingHoughLines>\n");
#endif
	// Find maxima in bins
	bestVal = 0;
	bestSlopeIndex = -1;
	bestC = -1;
	for (slopeIndex = 0; slopeIndex < 13; ++slopeIndex)
	{
#if VERBOSE_BLOB_DETECTION > 2
	fprintf (outputFile, "<EvaluatingHoughLinesForSlope>slopeIndex=%d\n", slopeIndex);
#endif
		for (i = 0; i < SIZE_SINGLE_HOUGH_BINS; ++i)
		{
			if (houghVals[i + slopeIndex * SIZE_SINGLE_HOUGH_BINS])
			{
#if VERBOSE_BLOB_DETECTION > 2
				fprintf (outputFile, "(%d,%d), ", i, houghVals[i + slopeIndex * SIZE_SINGLE_HOUGH_BINS]);
#endif
//				fprintf (outputFile, "slope=%d c=%d val=%d\n",
//					slopeIndex,
//					(i - HOUGH_BINS_OFFSET) * 3,
//					i);
//					(houghVals[i + slopeIndex * SIZE_SINGLE_HOUGH_BINS] - HOUGH_BINS_OFFSET) * 3,
//					houghVals[i + slopeIndex * SIZE_SINGLE_HOUGH_BINS]);
			}

			if (houghVals[i + slopeIndex * SIZE_SINGLE_HOUGH_BINS] > bestVal)
			{
				bestSlopeIndex = slopeIndex;
				bestC = i;
				bestVal = houghVals[i + slopeIndex * SIZE_SINGLE_HOUGH_BINS];
			}
		}
#if VERBOSE_BLOB_DETECTION > 2
		fprintf (outputFile, "\n</EvaluatingHoughLinesForSlope>\n");
#endif
	}
#if VERBOSE_BLOB_DETECTION > 2
	fprintf (outputFile, "</EvaluatingHoughLines>\n");
#endif

	if (bestVal == 0)
	{
#if VERBOSE_BLOB_DETECTION
		fprintf (outputFile, "<HoughLineFail />\n");
#endif
		return houghLineEst;
	}

//	bestC = (bestC - HOUGH_BINS_OFFSET) * 3;
	bestC = (bestC * 3) - HOUGH_BINS_OFFSET;
	bestSlope = expectedEdgeSlope + (bestSlopeIndex - 6) * 0.1f;

	houghLineEst.houghLine.axis = useXAxis;
	houghLineEst.houghLine.c = bestC;
	houghLineEst.houghLine.slope = bestSlope;

#if VERBOSE_BLOB_DETECTION > 2
	fprintf (outputFile, "<HoughLineResults>slope=%f c=%d axis=%d val=%d</HoughLineResults>\n",
		bestSlope,
		bestC,
		useXAxis,
		bestVal);
/*	fprintf (
		outputFile,
		"Best line: slope %f c %d xAxis %d val %d\n",
		bestSlope,
		bestC,
		useXAxis,
		bestVal);*/
#endif

#if VERBOSE_BLOB_DETECTION > 2
	fprintf (outputFile, "</HoughLines>\n");
	fflush (outputFile);
#endif

#if VERBOSE_BLOB_DETECTION
#if CHECK_HOUGH_LINE
	RobotRecognitionAccurateEstimation_checkHoughLine (
		outputFile,
		houghLineEst,
		linePts,
		nLinePts);
#endif // if CHECK_HOUGH_LINE
#endif // if VERBOSE_BLOB_DETECTION

	houghLineEst.isEstOk = 1;

	return houghLineEst;
}
#undef CHECK_HOUGH_LINE































int RobotRecognitionAccurateEstimation_getKernelStartPtsNEW (
	FILE *outputFile,
	const int edgeIndex,
	const float expectedEdgeSlope,
	const int useXAxis,
	const RobotFace *robotFace,
	PointI *linePts)
{
	int index = 0;
	PointI start, end, step, iter;
	if (edgeIndex == 0 || edgeIndex == 1)
	{
		// top/bottom edge
		if (edgeIndex == 0)
		{
			start.y = end.y = robotFace->blob->tr.y;
		}
		else
		{
			start.y = end.y = robotFace->blob->bl.y;
		}
		start.x = robotFace->blob->bl.x;
		end.x = robotFace->blob->tr.x;
		
		start.x = max (0, start.x - 3);
		end.x = min (CAM_OCCUPANCY_GRID_X - 1, end.x + 3);

		start = ObstacleRecognitionCore_cellCoordsToPixelCoords (start);
		end = ObstacleRecognitionCore_cellCoordsToPixelCoords (end);

		step.x = CAM_OCCUPANCY_CELL_X;
		step.y = 0;
	}
	else
	{
		// left/right
		if (edgeIndex == 2)
		{
			start.x = end.x = robotFace->blob->bl.x; // left
		}
		else
		{
			start.x = end.x = robotFace->blob->tr.x;
		}
		start.y = robotFace->blob->bl.y;
		end.y = robotFace->blob->tr.y;

		start.y = max (0, start.y - 3);
		end.y = min (CAM_OCCUPANCY_GRID_X - 1, end.y + 3);

		start = ObstacleRecognitionCore_cellCoordsToPixelCoords (start);
		end = ObstacleRecognitionCore_cellCoordsToPixelCoords (end);

		step.x = 0;
		step.y = CAM_OCCUPANCY_CELL_Y;
	}

#if VERBOSE_BLOB_DETECTION > 2
	fprintf (outputFile, "<KernelStartPts>edge=%d pts=(", edgeIndex);
#endif

	iter = start;
	while (1)
	{
		linePts[index++] = iter;
#if VERBOSE_BLOB_DETECTION > 2
		fprintf (outputFile, "(%d,%d),", iter.x, iter.y);
#endif
		iter.x += step.x;
		iter.y += step.y;
		if (iter.x > end.x || iter.y > end.y)
		{
			break;
		}
	}
#if VERBOSE_BLOB_DETECTION > 2
	fprintf (outputFile, ") nPts=%d</KernelStartPts>\n", index);
#endif

	return index;
}

int RobotRecognitionAccurateEstimation_isKernelOk (
	const PointI kernelIter)
{
	if (kernelIter.x < CAM_OCCUPANCY_GRID_ORIGIN_X + 2 ||
		kernelIter.x > (CAM_OCCUPANCY_GRID_ORIGIN_X + (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_CELL_X)) - 3 ||
		kernelIter.y < CAM_OCCUPANCY_GRID_ORIGIN_Y + 2 ||
		kernelIter.y > (CAM_OCCUPANCY_GRID_ORIGIN_Y + (CAM_OCCUPANCY_GRID_Y * CAM_OCCUPANCY_CELL_Y)) - 3)
	{
		return 0;
	}
	return 1;
}

float RobotRecognitionAccurateEstimation_getKernelResponse (
#if VERBOSE_BLOB_DETECTION
	FILE *outputFile,
#endif
	const int colourIndex,
	const PointI kernelIter,
	Image *camImg)
{
	float response;
	ImgCellFeatures cellFeatures;

	cellFeatures = ObstacleRecognitionCore_calcKernelFeatures (
#if VERBOSE_BLOB_DETECTION
		outputFile,
#endif
		camImg, kernelIter);

	response = RobotRecognitionModel_calcSingleColourScore (colourIndex, &cellFeatures);

	return response;
}

#define N_KERNELS_PER_PT 15
#define KERNEL_ITER_OFFSET 20
#define KERNEL_ITER_STEP 4
#define KERNEL_SEQ_LEN 4
#define KERNEL_SEQ_SCORE_THRESHOLD 3.0f
#define KERNEL_RATIO_THRESHOLD 0.2f
int RobotRecognitionAccurateEstimation_findKernelEdgeResponses (
	FILE *outputFile,
	const int edgeIndex,
	const int expectedEdgeSlope,
	const int useXAxis,
	const PointI *initialLinePts,
	PointI *responsePts,
	const int nLinePts,
	Image *camImg,
	const RobotFace *robotFace)
{
	int i;
	int nGoodResponses;
	int kernelIndex, nKernelResponses, bestIndex;
	float score, bestScore, blobResponsesStrength, responseDiff;
	PointI kernelIter, kernelIterOffset, kernelIterStart, kernelIterStep, responsePt;
	float kernelResponses[N_KERNELS_PER_PT];

	switch (edgeIndex)
	{
	case 0: // top
		kernelIterOffset.x = 0; kernelIterOffset.y = KERNEL_ITER_OFFSET;
		kernelIterStep.x = 0; kernelIterStep.y = -KERNEL_ITER_STEP;
		break;
	case 1: // bottom
		kernelIterOffset.x = 0; kernelIterOffset.y = -KERNEL_ITER_OFFSET;
		kernelIterStep.x = 0; kernelIterStep.y = KERNEL_ITER_STEP;
		break;
	case 2: // left
		kernelIterOffset.x = -KERNEL_ITER_OFFSET; kernelIterOffset.y = 0;
		kernelIterStep.x = KERNEL_ITER_STEP; kernelIterStep.y = 0;
		break;
	default: // right
		kernelIterOffset.x = KERNEL_ITER_OFFSET; kernelIterOffset.y = 0;
		kernelIterStep.x = -KERNEL_ITER_STEP; kernelIterStep.y = 0;
		break;
	}

	nGoodResponses = 0;
	for (i = 0; i < nLinePts; ++i)
	{
#if VERBOSE_BLOB_DETECTION > 2
		fprintf (outputFile, "<KernelResponse>\n");
#endif

		kernelIterStart.x = initialLinePts[i].x + kernelIterOffset.x;
		kernelIterStart.y = initialLinePts[i].y + kernelIterOffset.y;

		kernelIterStart.x = max (kernelIterStart.x, CAM_OCCUPANCY_GRID_ORIGIN_X + 2); // kernel is 5x5, so midpt can be min 2
		kernelIterStart.x = min (kernelIterStart.x, (CAM_OCCUPANCY_GRID_ORIGIN_X + (CAM_OCCUPANCY_GRID_X * CAM_OCCUPANCY_CELL_X)) - 3);

		kernelIterStart.y = max (kernelIterStart.y, CAM_OCCUPANCY_GRID_ORIGIN_Y + 2);
		kernelIterStart.y = min (kernelIterStart.y, (CAM_OCCUPANCY_GRID_ORIGIN_Y + (CAM_OCCUPANCY_GRID_Y * CAM_OCCUPANCY_CELL_Y)) - 3);

		kernelIter = kernelIterStart;
		kernelIndex = 0;
		while (kernelIndex < N_KERNELS_PER_PT)
		{
			kernelResponses[kernelIndex] = RobotRecognitionAccurateEstimation_getKernelResponse (
#if VERBOSE_BLOB_DETECTION
				outputFile,
#endif
				robotFace->blob->colourIndex,
				kernelIter,
				camImg);

			++kernelIndex;

			kernelIter.x += kernelIterStep.x;
			kernelIter.y += kernelIterStep.y;
			if (!RobotRecognitionAccurateEstimation_isKernelOk (kernelIter))
			{
				break;
			}
		}
		nKernelResponses = kernelIndex;

#if VERBOSE_BLOB_DETECTION > 2
		fprintf (outputFile, "<KernelResponseInput>edge=%d initialPt=(%d,%d) iterStartPt=(%d,%d), iterStep=(%d,%d)</KernelResponseInput>\n",
			edgeIndex,
			initialLinePts[i].x, initialLinePts[i].y,
			kernelIterStart.x, kernelIterStart.y,
			kernelIterStep.x, kernelIterStep.y);
#endif

#if VERBOSE_BLOB_DETECTION > 2
		fprintf (outputFile, "<KernelResponseScores>responseScores=(");
		for (kernelIndex = 0; kernelIndex < nKernelResponses; ++kernelIndex)
		{
			fprintf (outputFile, "%f,", kernelResponses[kernelIndex]);
		}
		fprintf (outputFile, ")</KernelResponseScores>\n");
#endif

#if VERBOSE_BLOB_DETECTION > 2
		fprintf (outputFile, "<KernelSequenceScores>sequenceScores=(");
#endif
		bestIndex = -1;
		bestScore = MAX_FLT;
		nKernelResponses = kernelIndex;
		for (kernelIndex = 0; kernelIndex < (nKernelResponses - KERNEL_SEQ_LEN); ++kernelIndex)
		{
			blobResponsesStrength = (kernelResponses[kernelIndex + 1] + kernelResponses[kernelIndex + 2] + kernelResponses[kernelIndex + 3]) / 3.0f;

			// subtract goodScore - poor, as smaller scores are better, so a large -ve number will be good
//			responseDiff = blobResponsesStrength - kernelResponses[kernelIndex];
			responseDiff = blobResponsesStrength / kernelResponses[kernelIndex];
//			score = (blobResponsesStrength + responseDiff);
#if VERBOSE_BLOB_DETECTION > 2
			fprintf (outputFile, "(%d,%f,%f),", kernelIndex, blobResponsesStrength, responseDiff);
#endif
			// Smaller scores are better
//			if (score < bestScore)
//			{
//				bestScore = score;
//				bestIndex = kernelIndex;
//			}
			if (blobResponsesStrength < COLOUR_CONFIDENCE_THRESHOLD &&
				responseDiff < KERNEL_RATIO_THRESHOLD)
			{
				// Both will be normalised to 1
				score = (blobResponsesStrength / COLOUR_CONFIDENCE_THRESHOLD) + responseDiff;
				if (score < bestScore)
				{
					bestScore = score;
					bestIndex = kernelIndex;
				}
			}
		}
#if VERBOSE_BLOB_DETECTION > 2
		fprintf (outputFile, ") bestScore=%f bestIndex=%d</KernelSequenceScores>\n",
			bestScore, bestIndex);
#endif

		if (bestScore < KERNEL_SEQ_SCORE_THRESHOLD)
		{
			responsePt = kernelIterStart;

			responsePt.x += kernelIterOffset.x;
			responsePt.y += kernelIterOffset.y;

			responsePt.x += (kernelIterStep.x) * bestIndex;
			responsePt.y += (kernelIterStep.y) * bestIndex;

			responsePts[i] = responsePt;
#if VERBOSE_BLOB_DETECTION > 2
			fprintf (outputFile, "<KernelResponseAlongLine>index=%d pt=(%d,%d) score=%f</KernelResponseAlongLine>\n",
				i,
				responsePts[i].x, responsePts[i].y,
				bestScore);
#endif
			++nGoodResponses;
		}
		else
		{
			responsePts[i].x = MIN_FLT;
			responsePts[i].y = MIN_FLT;
#if VERBOSE_BLOB_DETECTION > 2
			fprintf (outputFile, "<KernelResponseAlongLineFail>index=%d score=%f</KernelResponseAlongLineFail>\n",
				i, bestScore);
#endif
		}
#if VERBOSE_BLOB_DETECTION > 2
		fprintf (outputFile, "</KernelResponse>\n");
#endif
	}
	return nGoodResponses;
}
#undef N_KERNELS_PER_PT
#undef KERNEL_ITER_OFFSET
#undef KERNEL_ITER_STEP
#undef KERNEL_SEQ_LEN
#undef KERNEL_SEQ_SCORE_THRESHOLD









































// ! Determine pt on line that can be used for projecting into world space
/*!
If we have a vertical and horizonal line, we can intersect these to getter
a better corner estimate, otherwise we can pick the top/bottom or leftmost/rightmost
point from the given edge pts. The rough assumption made is that a pt is at the
corner of an edge if it is greater than cellDims/2 away from the image bounds.
*/
PointF RobotRecognitionAccurateEstimation_pickPtOnLine (
	FILE *outputFile,
	const int edgeIndex,
	const RobotFace *robotFace,
//	const PointI *responsePts,
//	const int nResponsePts,
	LineEst *lineEst)

{
	int i;
	float val1, val2;
	PointF pt;

	if (edgeIndex == 0 || edgeIndex == 1) // top/bottom
	{
		DEBUG_ASSERT(!lineEst->houghLine.axis)

		i = (robotFace->blob->tr.x + robotFace->blob->bl.x) / 2;
		val1 = (float)ObstacleRecognitionCore_cellXToPixelX (i);
		val2 = lineEst->houghLine.slope * val1 + lineEst->houghLine.c; // y = mx + c
		pt.x = val1;
		pt.y = val2;
	}
	else
	{
		DEBUG_ASSERT(lineEst->houghLine.axis)

		i = (robotFace->blob->tr.y + robotFace->blob->bl.y) / 2;
		val1 = (float)ObstacleRecognitionCore_cellYToPixelY (i);
		val2 = lineEst->houghLine.slope * val1 + lineEst->houghLine.c; // x = my + c
		pt.x = val2;
		pt.y = val1;
	}
	return pt;
}

int RobotRecognitionAccurateEstimation_checkIfAxisOk (
	const int edgeIndex,
	const int useXAxis)
{
	if (edgeIndex == 0 || edgeIndex == 1)
	{
		// Horizontal => should not use x axis
		return !useXAxis;
	}
	else
	{
		// Vertical => should use x axis
		return useXAxis;
	}
}

extern const int RobotRecognitionModel_mapColourDescriptionArrayToColourId[N_ROBOT_COLOURS];
extern const float RobotRecognitionCore_faceOrientOffset[4];
extern const PointI RobotRecognitionCore_cornerIndicesPerFace[4];
extern const RobotParams RobotRecognitionCore_robotParams[3];
extern const CornerParams RobotRecognitionCore_cornerParams[3];

#define FIT_EDGE_TO_RESPONSE_PTS 1
#define GET_KERNEL_RESPONSES 1
#define MIN_REQD_PTS_FOR_EDGE 4
LocEst RobotRecognitionAccurateEstimation_estFaceEdgeNEW (
	FILE *outputFile,
	CamVectors *camVectors,
	const UncertaintyConstants *uncertaintyConstants,
	Image *camImg,
	const RobotFace *robotFace,
	const float ownOrient,
	const int visibleRobotIndex,
	const float visibleRobotOrient,
	const int edgeIndex,
	const float _expectedEdgeSlope,
	const int _useXAxis)
{
	int nLinePts, nResponsePts;
	PointI initialLinePts[CALC_FACE_EDGE_MAX_N_PTS];
	PointI responsePts[CALC_FACE_EDGE_MAX_N_PTS];
	LocEst locEst;
	LineEst houghLineEst;
	PointF ptOnLine, ptPixelSpace;
	float heightWorldSpace;
	Vector3F ptWorldSpace;
	float distFromCog;
	float orientFromCog;
	int cornerIndex;
	float expectedEdgeSlope;
	int useXAxis;

	locEst.isEst = 0;

	if (edgeIndex == 0 || edgeIndex == 1) // top/bottom
	{
		useXAxis = _useXAxis;
		expectedEdgeSlope = _expectedEdgeSlope;
	}
	else
	{
		useXAxis = 1;
		expectedEdgeSlope = 0.0f; // left/right = straight up
	}

	if (!RobotRecognitionAccurateEstimation_checkIfAxisOk (
		edgeIndex,
		useXAxis))
	{
		fprintf (outputFile, "<EstFaceEdgeFail>reason=\"useXAxis\"</EstFaceEdgeFail>\n");
		return locEst;
	}

	nLinePts = RobotRecognitionAccurateEstimation_getKernelStartPtsNEW (
		outputFile,
		edgeIndex,
		expectedEdgeSlope,
		useXAxis,
		robotFace,
		initialLinePts);
	if (!nLinePts)
	{
		fprintf (outputFile, "<EstFaceEdgeFail>reason=\"nlinePts\"</EstFaceEdgeFail>\n");
		return locEst;
	}

#if GET_KERNEL_RESPONSES
	nResponsePts = RobotRecognitionAccurateEstimation_findKernelEdgeResponses (
		outputFile,
		edgeIndex,
		expectedEdgeSlope,
		useXAxis,
		initialLinePts,
		responsePts,
		nLinePts,
		camImg,
		robotFace);
	if (nResponsePts < MIN_REQD_PTS_FOR_EDGE)
	{
		fprintf (outputFile, "<EstFaceEdgeFail>reason=\"nResponsePts\"</EstFaceEdgeFail>\n");
		return locEst;
	}

#if FIT_EDGE_TO_RESPONSE_PTS
	houghLineEst = RobotRecognitionAccurateEstimation_fitPtsToLine (
		outputFile,
		responsePts,
		nLinePts,
		expectedEdgeSlope,
		useXAxis);
	if (!houghLineEst.isEstOk)
	{
		fprintf (outputFile, "<EstFaceEdgeFail>reason=\"isEstOk\"</EstFaceEdgeFail>\n");
		return locEst;
	}

	ptOnLine = RobotRecognitionAccurateEstimation_pickPtOnLine (
		outputFile,
		edgeIndex,
		robotFace,
		&houghLineEst);
	ptPixelSpace = CamVectors_pixelfToImageOffsets (ptOnLine);

	if (edgeIndex == 0 || edgeIndex == 1) // top/bottom
	{
		if (edgeIndex == 0) // top
		{
			heightWorldSpace = RobotRecognitionCore_robotParams[visibleRobotIndex].faceTopEdgeHeight;
		}
		else
		{
			heightWorldSpace = RobotRecognitionCore_robotParams[visibleRobotIndex].faceEdgeBottomHeight;
		}

		ptWorldSpace = CamVectors_projectImageToHorizontalPlane (
			camVectors,
			ptPixelSpace,
			heightWorldSpace);

		distFromCog = RobotRecognitionCore_robotParams[visibleRobotIndex].cogToFace[robotFace->faceIndex];
		Geometry_ptFromOrient (
			ptWorldSpace.x,
			ptWorldSpace.y,
			&locEst.locInWorld.x,
			&locEst.locInWorld.y,
			-distFromCog,
			robotFace->faceOrient);

		// Horizontal edge => we just take the midpt of the edge and estimate this
		// as the face location. A more accurate estimate along the perpindicular
		// axis can be obtained from vertical edges and combined with this.

		// Initial covariance represents the variance in error when detecting edges
		// and when projecting points into the world
		locEst.cov.vector = Uncertainty_calcVisualEstimateCovariance (
			uncertaintyConstants,
			ownOrient,
			uncertaintyConstants->visEst_accurateFace_horEdge_varFwd,
			uncertaintyConstants->visEst_accurateFace_horEdge_varLat,
			uncertaintyConstants->visEst_accurateFace_horEdge_covar);

		// We just take the midpt of the horizonatal edge and estimate this as the 
		// centre of the face, so accumulate the associated variance
		locEst.cov.vector = Uncertainty_accumRobotFaceHorizonalEdgeEstimateCovariance (
			uncertaintyConstants,
			locEst.cov.vector,
			robotFace->faceOrient,
			RobotRecognitionCore_getFaceHalfLen (visibleRobotIndex, robotFace->faceIndex));
	}
	else
	{
		heightWorldSpace = (RobotRecognitionCore_robotParams[visibleRobotIndex].faceTopEdgeHeight + RobotRecognitionCore_robotParams[visibleRobotIndex].faceEdgeBottomHeight) / 2.0f;

		ptWorldSpace = CamVectors_projectImageToHorizontalPlane (
			camVectors,
			ptPixelSpace,
			heightWorldSpace);

		if (edgeIndex == 2) // left
		{
			cornerIndex = RobotRecognitionCore_cornerIndicesPerFace[robotFace->faceIndex].x;
		}
		else // right edge of face
		{
			cornerIndex = RobotRecognitionCore_cornerIndicesPerFace[robotFace->faceIndex].y;
		}
		orientFromCog = RobotRecognitionCore_cornerParams[visibleRobotIndex].orientFromCogToCorner[cornerIndex];
		distFromCog = RobotRecognitionCore_cornerParams[visibleRobotIndex].distFromCogToCorner[cornerIndex];

		orientFromCog = Geometry_orientSum (visibleRobotOrient, orientFromCog);

		// Have orient from cog to corner, but have estimate for corner, so
		// project point in the opposite direction.
		Geometry_ptFromOrient (
			ptWorldSpace.x,
			ptWorldSpace.y,
			&locEst.locInWorld.x,
			&locEst.locInWorld.y,
			-distFromCog,
			orientFromCog);

		// Have a quite good variance laterally, but a large variance along
		// the direction of the estimate, as we just pick the midpt of the
		// vertical edge
		locEst.cov.vector = Uncertainty_calcVisualEstimateCovariance (
			uncertaintyConstants,
			ownOrient,
			uncertaintyConstants->visEst_accurateFace_vertEdge_varFwd,
			uncertaintyConstants->visEst_accurateFace_vertEdge_varLat,
			uncertaintyConstants->visEst_accurateFace_vertEdge_covar);
	}

	fprintf (
		outputFile,
		"<FaceEdgeAccLoc>colourId=%d faceIndex=%d edgeIndex=%d loc=(%f,%f) cov=(%f,%f,%f,%f)</FaceEdgeAccLoc>\n",
		RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
		robotFace->faceIndex,
		edgeIndex,
		locEst.locInWorld.x, locEst.locInWorld.y,
		locEst.cov.mat[0], locEst.cov.mat[1], locEst.cov.mat[2], locEst.cov.mat[3]);

	locEst.isEst = 1;

#endif // FIT_EDGE_TO_RESPONSE_PTS
#endif // GET_KERNEL_RESPONSES
	return locEst;
}
#undef FIT_EDGE_TO_RESPONSE_PTS
#undef GET_KERNEL_RESPONSES
#undef MIN_REQD_PTS_FOR_EDGE


void RobotRecognitionAccurateEstimation_estimateRobotLocationsNEW (
	FILE *outputFile,
	Image *camImg,
	CamVectors *camVectors,
	const RobotData *robotDataArray,
	UncertaintyConstants *uncertaintyConstants,
	const int ownIndex,
	List *robotEstimates)
{
	int faceIndex, edgeIndex, nEdgeEsts, nFaceEsts;
	RobotEstimate *robotEstimate;
	RobotFace *robotFace;
	ListNode *iter;
	LocEst edgeLoc;
	UnionVector4F tempCov, combdCov, combdFaceCov;
	PointF tempEst, combdEst, combdFaceEst;
	float ownOrient = robotDataArray[ownIndex].pose.orient;

	iter = robotEstimates->front;
	while (iter)
	{
		robotEstimate = (RobotEstimate*)iter->value;

		nFaceEsts = 0;
		combdFaceEst.x = 0.0f; combdFaceEst.y = 0.0f;
		combdFaceCov.mat[0] = 0.0f; combdFaceCov.mat[1] = 0.0f; combdFaceCov.mat[2] = 0.0f; combdFaceCov.mat[3] = 0.0f;

		for (faceIndex = 0; faceIndex < 2; ++faceIndex)
		{
			// 0=f/b 1=l/r
			robotFace = &robotEstimate->robotFaces[faceIndex];

			// If this face has a valid rough est, then try and get an accurate est
			if ((robotFace->estType == FACE_EST_CORNER || robotFace->estType == FACE_EST_EDGE) &&
				robotFace->blob->blobType == BLOB_VALID)
			{
				nEdgeEsts = 0;
				combdEst.x = 0.0f; combdEst.y = 0.0f;
				combdCov.mat[0] = 0.0f; combdCov.mat[1] = 0.0f; combdCov.mat[2] = 0.0f; combdCov.mat[3] = 0.0f;

				for (edgeIndex = 0; edgeIndex < 4; ++edgeIndex)
				{
					edgeLoc = RobotRecognitionAccurateEstimation_estFaceEdgeNEW (
						outputFile,
						camVectors,
						uncertaintyConstants,
						camImg,
						robotFace,
						ownOrient,
						robotEstimate->robotIndex,
						robotEstimate->robotOrient,
						edgeIndex,
						robotFace->slopeCameraSpace,
						robotFace->useCameraXAxis);

					if (edgeLoc.isEst)
					{
						// Combine estimates together

						// Calculate c^-1 for this est
						tempCov = edgeLoc.cov;
						tempCov.matrix = MatrixF_inverse (tempCov.matrix);

						// Accumulate sum of c^-1 * x for each estimate
						tempEst = Maths_multMatrixByPoint (tempCov.mat, edgeLoc.locInWorld);
						combdEst.x += tempEst.x; combdEst.y += tempEst.y;

						// Accumulate sum of c^-1
						combdCov.vector = Maths_addMatrices (combdCov.mat, tempCov.mat);

						++nEdgeEsts;
					}
				}

				if (nEdgeEsts)
				{
					// Calculate (sum of c^-1)^-1
					combdCov.matrix = MatrixF_inverse (combdCov.matrix);
					robotFace->cov = combdCov;

					// Calculate combined estimate: est(final) = c^-1(final) * sum(c^-1 * est)
					robotFace->locEst = Maths_multMatrixByPoint (combdCov.mat, combdEst);

					fprintf (
						outputFile,
						"<FaceAccLoc>colourId=%d faceIndex=%d nEdgeEsts=%d loc=(%f,%f) cov=(%f,%f,%f,%f)</FaceAccLoc>\n",
						RobotRecognitionModel_mapColourDescriptionArrayToColourId[robotFace->blob->colourIndex],
						robotFace->faceIndex,
						nEdgeEsts,
						robotFace->locEst.x, robotFace->locEst.y,
						robotFace->cov.vector.x, robotFace->cov.vector.y, robotFace->cov.vector.z, robotFace->cov.vector.w);

					robotFace->estType = FACE_EST_ACCURATE;
				}
			}

			// Acumulate face est
			if (robotFace->estType == FACE_EST_ACCURATE)
			{
				++nFaceEsts;

				// Get c^-1
				tempCov = robotFace->cov;
				tempCov.matrix = MatrixF_inverse (tempCov.matrix);

				// Accumulate sum of c^-1 * x for each estimate
				tempEst = Maths_multMatrixByPoint (tempCov.mat, robotFace->locEst);
				combdFaceEst.x += tempEst.x; combdFaceEst.y += tempEst.y;

				// Accumulate sum of c^-1
				combdFaceCov.vector = Maths_addMatrices (combdFaceCov.mat, tempCov.mat);
			}
		}

		// Accumulate initial rough est, and then write the final est back out to robotEstimate
		if (nFaceEsts)
		{
			// Get c^-1
			tempCov = robotEstimate->estCov;
			tempCov.matrix = MatrixF_inverse (tempCov.matrix);

			// Accumulate sum of c^-1 * x for each estimate
			tempEst = Maths_multMatrixByPoint (tempCov.mat, robotEstimate->estLoc);
			combdFaceEst.x += tempEst.x; combdFaceEst.y += tempEst.y;

			// Accumulate sum of c^-1
			combdFaceCov.vector = Maths_addMatrices (combdFaceCov.mat, tempCov.mat);

			// Calculate (sum of c^-1)^-1
			combdFaceCov.matrix = MatrixF_inverse (combdFaceCov.matrix);
			robotEstimate->estCov.matrix = combdFaceCov.matrix;

			// Calculate combined estimate: est(final) = c^-1(final) * sum(c^-1 * est)
			robotEstimate->estLoc = Maths_multMatrixByPoint (combdFaceCov.mat, combdFaceEst);
		}

		// If any acurate edges, then take est from rough est and
		// update type to FACE_EST_ACCURATE
		/*if (RobotRecognitionAccurateEstimation_setupFinalEstimate (
			outputFile,
			robotEstimate,
			FACE_EST_ACCURATE))
		{
#if 1 || VERBOSE_BLOB_DETECTION
			fprintf (outputFile,
				"<RobotAccurateLocation>est=(%f,%f) cov=(%f,%f,%f,%f)</RobotAccurateLocation>\n",
				finalEstimate.estLoc.x, finalEstimate.estLoc.y,
				finalEstimate.estCov.mat[0], finalEstimate.estCov.mat[1], finalEstimate.estCov.mat[2], finalEstimate.estCov.mat[3]);
#endif
		}*/

		iter = iter->next;
	}
}



#endif

