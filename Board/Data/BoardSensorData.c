#include "../../Common/RobotDefs.h"

#ifndef ROBOT

#include "BoardSensorData.h"



MapStatus initMapStatus()
{
	MapStatus m;
	m.isNewDataToDisplay = 0;
	m.isNewMapData = 0;
	return m;
}



MapStatusFromRobot initMapStatusFromRobot()
{
	MapStatusFromRobot m;
	m.isMoreDataComing = 0;
	m.isMoreDataComing = 0;
	return m;
}

LocEstimateForRobot initLocEstimateForRobot()
{
	LocEstimateForRobot l;
	l.estimateLoc.x = MIN_FLT;
	l.estimateLoc.y = MIN_FLT;
	l.estimateStdDev = MAX_FLT;
	l.robotMakingEstimate = -1;
	return l;
}

BoardSensorData initBoardSensorData()
{
	int i;
	BoardSensorData s;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		s.statusFromBoard[i] = initMapStatusFromBoard();
		s.statusFromRobot[i] = initMapStatusFromRobot();
		s.locEstimate[i] = initLocEstimateForRobot();
	}

	s.mapStatus = initMapStatus();
	s.globalMaps = initList();
	s.incorpMaps = initList();
	initCompressedImage (&s.compressedMap);
	s.compressedMap.buffer = (uchar*)malloc (LOC_MAP_DIMS * LOC_MAP_DIMS);
	s.compressedMap.bufferSize = LOC_MAP_DIMS * LOC_MAP_DIMS;

	return s;
}

void BoardSensorData_dtor (BoardSensorData *s)
{
	List_clearWithDeallocator (&s->globalMaps, freeCompressedImageWithMapInfo);
	List_clearWithDeallocator (&s->incorpMaps, freeCompressedImageWithMapInfo);
	freeCompressedImage (&s->compressedMap);
}

void BoardSensorData_setMapFlags (BoardSensorData *s)
{
	int i;
	for (i = 0; i < N_ROBOTS; ++i)
	{
		s->statusFromBoard[i].isNewGlobalMap = 1;
	}
}
#endif
