#include "../../Common/RobotDefs.h"

#ifndef ROBOT

#include "BoardJobs.h"
#include "../Comm/BoardComm.h"
#include "../../Robot/Data/BehaviourData.h"
#include "../../Robot/Behaviour/BehaviourCore.h"
#include "../../Robot/Behaviour/GotoExpPtImpl.h"
#include "../../Robot/Behaviour/ExplorationImpl.h"
#include "../../Common/BitArray.h"

#if defined(USE_CLOUD)



void BoardJobs_calcProfitGotoExpPt (
	BoardDatabase *db,
	const Image *navMap,
	Image *localMap,
	const List *unreachableIkDests,
	uchar *boardUnreachableLocalMapGrid,
	GotoExpPtPayload *gtepPayload,
	FILE *f)
{
	int hasUnreachableLocalMapGridBeenUpdated = 0;
#if defined(PRINT_CLOUD_DETAIL)
//	int winsz[4] = {10, 10, 90, 130};
#endif

	fprintf (f, "<ReadGtepJob>\n");

#if defined(PRINT_CLOUD_DETAIL)
//	Image_show (navMap, "n0", winsz);
//	Image_show (localMap, "l0", winsz);
	GotoExpPtPayload_print (gtepPayload, f);
#endif // defined(PRINT_CLOUD_DETAIL)

	GotoExpPtImpl_calcProfit (
		navMap,
		localMap,
		gtepPayload->obstructedCellGrid,
		boardUnreachableLocalMapGrid,
		&hasUnreachableLocalMapGridBeenUpdated,
		&gtepPayload->geometryConstants,
		&gtepPayload->camVectors,
		&gtepPayload->ikConstants,
		&gtepPayload->uncertaintyConstants,
		gtepPayload->camPOffset,
		db->groupData.robots,
		unreachableIkDests,
		gtepPayload->pose,
		gtepPayload->stdDev,
		gtepPayload->behaviourChangeOverhead, // Different for GotoExpPt and GotoExpPtCollab
		gtepPayload->considerStdDevInc, // same
		gtepPayload->originCellMidpt, // same
		gtepPayload->gridDims, // same
		gtepPayload->index,
		gtepPayload->explorationCoalition,
		gtepPayload->localMapGrid,
		&gtepPayload->gotoExpPtData,
		f);

#if defined(PRINT_CLOUD_DETAIL)
	GotoExpPtData_printOutput (&gtepPayload->gotoExpPtData, f);
#endif

	if (hasUnreachableLocalMapGridBeenUpdated)
	{
		// Reset flag for ALL robots. Even the robot who commisioned this job
		// will not receive the updated unreachableLocalMapGrid back, so it
		// will have to re-sync afterwards.
		MapStatusFromBoard_resetNewUnreachableLocalMap (db->sensorData.statusFromBoard);
	}

	fprintf (f, "</ReadGtepJob>\n");
}

void BoardJobs_readGtepJob ROBOT_REQUEST_PARAMS
{
	int res;
	int sz;
	CompressedImage compressedImage;
	GotoExpPtPayload *gtepPayload;
	List unreachableIkDests;

	res = CommCore_sendAck (commData, W_GTEP_JOB);
	assert (0 < res);

	sz = sizeof (GotoExpPtPayload);
	res = CommCore_recvBuffer (commData, sz);
	assert (0 < res);

	GotoExpPtImpl_allocPayload (&gtepPayload);
	memcpy (gtepPayload, commData->buffer, sizeof (GotoExpPtPayload));

	res = CommCore_sendAck (commData, W_GTEP_JOB);
	assert (0 < res);

	CommCore_readMap(commData, &compressedImage, 1); // The buffer is freed if necessary

	MapCore_decompressLocalMap (db->environment.incMap, &compressedImage);
	free (compressedImage.buffer);

	// The images data and orig are read in readMap. All other params are assumed standard.
	db->environment.incMap->width = LOC_MAP_DIMS;
	db->environment.incMap->height = LOC_MAP_DIMS;
	db->environment.incMap->nChannels = 1;
	db->environment.incMap->wStep = LOC_MAP_DIMS;

	CommCore_readMap(commData, &compressedImage, 1); // The buffer is freed if necessary

	MapCore_decompressLocalMap (db->environment.incMap2, &compressedImage);
	free (compressedImage.buffer);

	// The images data and orig are read in readMap. All other params are assumed standard.
	db->environment.incMap2->width = LOC_MAP_DIMS;
	db->environment.incMap2->height = LOC_MAP_DIMS;
	db->environment.incMap2->nChannels = 1;
	db->environment.incMap2->wStep = LOC_MAP_DIMS;

	unreachableIkDests = initList();
	CommCore_readList (commData, &unreachableIkDests);

	BoardJobs_calcProfitGotoExpPt (
		db,
		db->environment.incMap,
		db->environment.incMap2,
		&unreachableIkDests,
		db->environment.unreachableLocalMapGrid,
		gtepPayload,
		f);

	// Return gtepData, which contains info on the best target to go for, and
	// unreachableLocalMapGrid, which is updated during the process.
	CommCore_clearBuffer (commData);

//	CommCore_addItemToBuffer (commData, gtepPayload->unreachableLocalMapGrid, sizeof (gtepPayload->unreachableLocalMapGrid));
	CommCore_addItemToBuffer (commData, &gtepPayload->gotoExpPtData, sizeof (GotoExpPtData));

//	sz = sizeof (gtepPayload->unreachableLocalMapGrid) + sizeof (GotoExpPtData);
	sz = sizeof (GotoExpPtData);
	res = CommCore_sendBuffer (commData, sz);
	assert (0 < res);

	GotoExpPtImpl_clearPayload (gtepPayload);
	List_clear (&unreachableIkDests, 1);
}













































void BoardJobs_calcProfitExploration (
	BoardDatabase *db,
	const Image *navMap,
	Image *localMap,
	List *unreachableIkTargets,
	ExplorationPayload *expPayload,
	FILE *f)
{
#if defined(PRINT_CLOUD_DETAIL)
//	int winsz[4] = {10, 10, 90, 130};
#endif

#ifdef PRINT_DEBUG
	printf ("ExplorationPayload %d\n", sizeof (ExplorationPayload));
#endif

	fprintf (f, "<ReadExpJob>\n");

#if defined(PRINT_CLOUD_DETAIL)
//	Image_show (navMap, "n0", winsz);
//	Image_show (localMap, "l0", winsz);
	ExplorationPayload_print (expPayload, unreachableIkTargets, f);
#endif // defined(PRINT_CLOUD_DETAIL)

	ExplorationImpl_calcProfit (
		&expPayload->expData,
		navMap,
		localMap,
		expPayload->expGrid,
		expPayload->mappedCellGrid,
		expPayload->obstructedCellGrid,
		expPayload->unreachableExpCellGrid,
		unreachableIkTargets,
		db->groupData.robots,
		expPayload->pose,
		expPayload->stdDev,
		expPayload->index,
		expPayload->supervisor,
		expPayload->bid,
		expPayload->sessionIndex,
		expPayload->targetDist,
		&expPayload->geometryConstants,
		&expPayload->camVectors,
		&expPayload->ikConstants,
		&expPayload->uncertaintyConstants,
		expPayload->camPOffset,
		f);

#if defined(PRINT_CLOUD_DETAIL)
	ExplorationData_printOutput (&expPayload->expData, f);
#endif

	fprintf (f, "</ReadExpJob>\n");
}



void BoardJobs_readExpJob ROBOT_REQUEST_PARAMS
{
	int res;
	int sz;
	CompressedImage compressedImage;
	ExplorationPayload *expPayload;
	List unreachableIkTargets;

	res = CommCore_sendAck (commData, W_EXP_JOB);
	assert (0 < res);

	sz = sizeof (ExplorationPayload);
	res = CommCore_recvBuffer (commData, sz);
	assert (0 < res);

	expPayload = (ExplorationPayload*)malloc (sizeof (ExplorationPayload));
	memcpy (expPayload, commData->buffer, sizeof (ExplorationPayload));
	EXPLICIT_DEBUG_ASSERT(expPayload->geometryConstants.areExpDistsSetup == 1)

	res = CommCore_sendAck (commData, W_EXP_JOB);
	assert (0 < res);

	CommCore_readMap(commData, &compressedImage, 1); // The buffer is freed if necessary

	MapCore_decompressLocalMap (db->environment.incMap, &compressedImage);
	free (compressedImage.buffer);

	// The images data and orig are read in readMap. All other params are assumed standard.
	db->environment.incMap->width = LOC_MAP_DIMS;
	db->environment.incMap->height = LOC_MAP_DIMS;
	db->environment.incMap->nChannels = 1;
	db->environment.incMap->wStep = LOC_MAP_DIMS;

	CommCore_readMap(commData, &compressedImage, 1); // The buffer is freed if necessary

	MapCore_decompressLocalMap (db->environment.incMap2, &compressedImage);
	free (compressedImage.buffer);

	// The images data and orig are read in readMap. All other params are assumed standard.
	db->environment.incMap2->width = LOC_MAP_DIMS;
	db->environment.incMap2->height = LOC_MAP_DIMS;
	db->environment.incMap2->nChannels = 1;
	db->environment.incMap2->wStep = LOC_MAP_DIMS;

	unreachableIkTargets = initList();
	CommCore_readList (commData, &unreachableIkTargets);

	BoardJobs_calcProfitExploration (
		db,
		db->environment.incMap,
		db->environment.incMap2,
		&unreachableIkTargets,
		expPayload,
		f);

	// Return expData, which contains info on the best target to go for
	// Unlike GTO_EXP_PT, we do not update the unreachable grid here
	CommCore_clearBuffer (commData);

	CommCore_addItemToBuffer (commData, expPayload->unreachableExpCellGrid, sizeof (expPayload->unreachableExpCellGrid));
	CommCore_addItemToBuffer (commData, &expPayload->expData, sizeof (ExplorationData));

	sz = sizeof (ExplorationData) + sizeof (expPayload->unreachableExpCellGrid);
	res = CommCore_sendBuffer (commData, sz);
	assert (0 < res);

	List_clear (&unreachableIkTargets, 1);
	free (expPayload);
}













#endif // defined(USE_CLOUD)

#endif // ifndef ROBOT
