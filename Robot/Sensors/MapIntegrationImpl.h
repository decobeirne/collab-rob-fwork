#include "../../Common/RobotDefs.h"

#ifndef ROBOT_MAP_INTEGRATION_IMPL_H
#define ROBOT_MAP_INTEGRATION_IMPL_H

#include "../../Common/Map/MapCore.h"
#include "../../Common/Vector.h"

void MapIntegrationImpl_intScan_neighbour (
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const PointF loc,
	const float orient,
	const float camPOffset,
	const float scanCentreDist,
	Image *localMap,
	const uchar pixelValueToSet);

void MapIntegrationImpl_intScan_simd_evalBehaviour (
	const GeometryConstants *geometryConstants,
	CamVectors *camVectors,
	const PointF poseLoc,
	const float poseOrient,
	const float camPOffset,
	Image *localMap,
	const Image *navMap,
	const uchar pixelValueToSet);

#endif // ROBOT_MAP_INTEGRATION_IMPL_H
