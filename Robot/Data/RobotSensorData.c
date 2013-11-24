#include "RobotSensorData.h"


#ifndef BOARD




RobotSensorData initRobotSensorData ()
{
	PointI localMapDims;
	RobotSensorData r;
	localMapDims.x = LOC_MAP_DIMS;
	localMapDims.y = LOC_MAP_DIMS;

	r.isObstacle = 0;
	r.visRobotIndex = -1;
	r.visRobotRelLoc.x = 0;
	r.visRobotRelLoc.y = 0;
	r.isMapDataReady = 0;
	r.areOtherRobotsInLocalMap = 0;
	r.isDataPendingInc = 0;
	r.hasDataBeenIncd = 0;
	r.isSearchGridUpdateRequired = DONT_UPDATE_SEARCH_GRID;
	r.isNavMapUpdateRequired = DONT_UPDATE_NAV_MAP;
	r.isLocalMapEffected = 0;
	r.localMapDataIndex = 0;
	r.isUnincorporatedMapScan = 0;

	r.mapScanList = initList();

#if defined (IS_GUMSTIX) || defined (RERUNNING_GUMSTIX) || defined(RERUNNING_IMAGES_ONLY)
	r.camImg = (Image*)malloc (sizeof (Image)); // have to make data bigger by 1 byte
	r.camImg->nChannels = 3;
	r.camImg->width = 176;
	r.camImg->height = 143;
	r.camImg->wStep = r.camImg->width * r.camImg->nChannels;
	
	// Size should be 143 * 176 * 3.
	// More accurately: 4 + (176 + 1) * 3 * 143
	// => 75651, or 1024 * 74 = 75776
	r.camImg->data = (uchar*)malloc ((1024 * 74) * sizeof (uchar));
#endif

	Image_ctor (&r.localMap, localMapDims.x, localMapDims.y, 1);
	Image_ctor (&r.tempLocalMap, localMapDims.x, localMapDims.y, 1);
	Image_fill (r.localMap, 127);

	r.localMapCentre = initPointI (MIN_INT32, MIN_INT32);
	r.localMapResetPt1 = initPointI (MIN_INT32, MIN_INT32);
	r.localMapResetPt2 = initPointI (MIN_INT32, MIN_INT32);

	r.isLocalMapResetRequired = DONT_RESET_LOC_MAP;
	r.scanLimit = MAP_SCAN_LIST_LIMIT;
	return r;
}

void RobotSensorData_dtor (RobotSensorData *r)
{
	List_clearWithDeallocator (&r->mapScanList, freeMapScan);

	Image_dtor (&r->localMap);
	Image_dtor (&r->tempLocalMap);

#if defined (IS_GUMSTIX) || defined (RERUNNING_GUMSTIX) || defined(RERUNNING_IMAGES_ONLY)
	Image_dtor (&r->camImg);
#endif // IS_GUMSTIX
}
#endif

