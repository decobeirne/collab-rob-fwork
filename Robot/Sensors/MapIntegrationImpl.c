#include "MapIntegrationImpl.h"

#include "../../Common/Geometry.h"
#include "../../Common/Bresenham.h"

void MapIntegrationImpl_intScan_neighbour (
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const PointF loc,
	const float orient,
	const float camPOffset,
	const float scanCentreDist,
	Image *localMap,
	const uchar pixelValueToSet)
{
	int i, j;
	PointF focalPt;
	PointF centrePt;
	PointI bl, tr;
	PointF relPt;
	const int sensorRange = 25;

//	Geometry_ptFromOrient (loc.x, loc.y, &focalPt.x, &focalPt.y, CAM_P_OFFSET, orient);
	Geometry_ptFromOrient (loc.x, loc.y, &focalPt.x, &focalPt.y, camPOffset, orient);
//	Geometry_ptFromOrient (loc.x, loc.y, &centrePt.x, &centrePt.y, CAM_P_OFFSET + geometryConstants->exp_optDist, orient);
//	Geometry_ptFromOrient (loc.x, loc.y, &centrePt.x, &centrePt.y, CAM_P_OFFSET + SCAN_CENTRE_DIST, orient);
	Geometry_ptFromOrient (loc.x, loc.y, &centrePt.x, &centrePt.y, camPOffset + scanCentreDist, orient);
	centrePt.x -= localMap->orig.x;
	centrePt.y -= localMap->orig.y;

	tr.x = centrePt.x + sensorRange;
	tr.y = centrePt.y + sensorRange;
	bl.x = centrePt.x - sensorRange;
	bl.y = centrePt.y - sensorRange;

	bl.x = max (0, bl.x);
	bl.y = max (0, bl.y);
	tr.x = min (tr.x, LOC_MAP_DIMS);
	tr.y = min (tr.y, LOC_MAP_DIMS);

	for (i = bl.x; i < tr.x; ++i)
	{
		for (j = bl.y; j < tr.y; ++j)
		{
			relPt.x = (float)(i + localMap->orig.x) - focalPt.x;
			relPt.y = (float)(j + localMap->orig.y) - focalPt.y;

			if (Geometry_isVisualContactNEW (
				relPt,
				geometryConstants->cam_xLimit,
				geometryConstants->cam_yNegLimit,
				geometryConstants->cam_yPosLimit,
				camVectors))
			{
				if (127 == Image_getPixel_dontCheck (localMap, i, j))
				{
					Image_setPixel_dontCheck (localMap, i, j, pixelValueToSet);
				}
			}
		}
	}
}


















void MapIntegration_intScanPixel_simd_evalBehaviour (
	Image *localMap,
	const PointI pt,
	const uchar pixelValueToSet)
{
	// Same logic as in MapIntegrationImpl_intScan_neighbour
	if (127 == Image_getPixel_check (localMap, pt.x, pt.y))
	{
		Image_setPixel_dontCheck (localMap, pt.x, pt.y, pixelValueToSet);
	}
}

void MapIntegration_intScanVector_simd_evalBehaviour (
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	BresData *bresdata,
	Image *localMap,
	const Image *navMap,
	const uchar pixelValueToSet)
{
	int e;
	int dist;
	uchar val, val2, val3;
	PointI bres2, bres3;
	PointF relPt;

	e = 0;

	for (dist = 0; dist < bresdata->count; ++dist)
	{
		bres2 = bresdata->bres;
		bres3 = bres2;

		Bresenham_step (
			&bresdata->bres,
			&e,
			&bresdata->step,
			&bresdata->d,
			bresdata->axis);

		// Point is out of bounds, so return
		if (0 == Image_isWithinBounds_ignoreMapOrig (
			localMap,
			bresdata->bres.x,
			bresdata->bres.y))
		{
			return;
		}

		// calc neighbouring pts for each bresenham pt
		if (1 == bresdata->axis)
		{
			// 1 is x, so see if y was also incr'd
			if (bres2.y == bresdata->bres.y)
			{
				bres2.y += bresdata->step.y;
				bres3.y -= bresdata->step.y;
			}
			else
			{
				bres3.y += 2 * bresdata->step.y;
			}

			bres2.x += bresdata->step.x;
			bres3.x += bresdata->step.x;
		}
		else
		{
			if (bres2.x == bresdata->bres.x)
			{
				bres2.x += bresdata->step.x;
				bres3.x -= bresdata->step.x;
			}
			else
			{
				bres3.x += 2 * bresdata->step.x;
			}

			bres2.y += bresdata->step.y;
			bres3.y += bresdata->step.y;
		}

		{
			// will return INVALID_CELL if OOB
			val = Image_getPixel_check (
				navMap,
				bresdata->bres.x,
				bresdata->bres.y);

			val2 = Image_getPixel_check (
				navMap,
				bres2.x,
				bres2.y);

			val3 = Image_getPixel_check (
				navMap,
				bres3.x,
				bres3.y);

			if (OCCUPIED_TERRAIN == val || OCCUPIED_TERRAIN == val2 || OCCUPIED_TERRAIN == val3)
			{
				// When integrating a scan for evalBehaviour, we don't care if pixels are
				// occupied or not. We just project out vectors and quit as soon as an
				// obstacle is hit. Obstacles are determined to be any pixel marked
				// OCCUPIED_TERRAIN in the navMap.
				return;
			}
		}

		relPt.x = (float)bresdata->bres.x - bresdata->lens.x;
		relPt.y = (float)bresdata->bres.y - bresdata->lens.y;

		EXPLICIT_DEBUG_ASSERT(camVectors->currentAdjustedOrient == bresdata->poseOrient)

		if (1 == Geometry_isVisualContactNEW (
			relPt,
			geometryConstants->cam_xLimit,
			geometryConstants->cam_yNegLimit,
			geometryConstants->cam_yPosLimit,
			camVectors))
		{
			MapIntegration_intScanPixel_simd_evalBehaviour (localMap, bresdata->bres,	pixelValueToSet);
			MapIntegration_intScanPixel_simd_evalBehaviour (localMap, bres2,			pixelValueToSet);
			MapIntegration_intScanPixel_simd_evalBehaviour (localMap, bres3,			pixelValueToSet);
		}
	}
}

void MapIntegrationImpl_intScan_simd_evalBehaviour (
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const PointF poseLoc,
	const float poseOrient,
	const float camPOffset,
	Image *localMap,
	const Image *navMap,
	const uchar pixelValueToSet)
{
	BresData bresdata;
	float orientRange;
	float orientJump;
	float bres_orient;
	float startDist;
	float endDist;
	float bres_startDist;
	float bres_endDist;
	PointF startPt, endPt;

	EXPLICIT_DEBUG_ASSERT(geometryConstants->areCamDistsSetup == 1)

	orientRange = geometryConstants->cam_angleAtMinDist + RADS_TWO;
	orientJump = RADS_TWO;

	// Start from 1 (not cam_minDist - 1), as an obstacle directly in front of the
	// camera will still be visible
	startDist = 1.0f;
	endDist = geometryConstants->cam_maxDist + 1;

	Geometry_ptFromOrient (
		poseLoc.x - localMap->orig.x,
		poseLoc.y - localMap->orig.y,
		&bresdata.lens.x,
		&bresdata.lens.y,
//		CAM_P_OFFSET,
		camPOffset,
		poseOrient);

	bresdata.poseOrient = poseOrient;
	bres_startDist = startDist;
	bres_endDist = endDist;

	for (bres_orient = poseOrient - orientRange;
		bres_orient < poseOrient + orientRange;
		bres_orient += orientJump)
	{
		Geometry_ptFromOrient (
			bresdata.lens.x,
			bresdata.lens.y,
			&startPt.x,
			&startPt.y,
			bres_startDist,
			bres_orient);

		Geometry_ptFromOrient (
			bresdata.lens.x,
			bresdata.lens.y,
			&endPt.x,
			&endPt.y,
			bres_endDist,
			bres_orient);

		Bresenham_setPts (
			localMap,
			startPt.x,
			startPt.y,
			endPt.x,
			endPt.y,
			&bresdata.bres.x,
			&bresdata.bres.y,
			&bresdata.d.x,
			&bresdata.d.y);

		Bresenham_setStep (
			&bresdata.d.x,
			&bresdata.d.y,
			&bresdata.step.x,
			&bresdata.step.y,
			&bresdata.axis,
			&bresdata.count);

		MapIntegration_intScanVector_simd_evalBehaviour (
			geometryConstants,
			camVectors,
			&bresdata,
			localMap,
			navMap,
			pixelValueToSet);
	}
}






