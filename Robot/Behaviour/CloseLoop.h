#include "../../Common/RobotDefs.h"

#ifndef BOARD

#ifndef CLOSE_LOOP_H
#define CLOSE_LOOP_H

#include "../Data/RobotDatabase.h"

//! Wrapper function for map processing steps
void CloseLoop_removeSupFromNavMap (
	RobotDatabase *db,
	List *resetList);

void CloseLoop_replaceSupInNavMap (
	RobotDatabase *db,
	List *obstdCellGridResetList);

int CloseLoop_isLoopClosePossible (
	RobotDatabase *db);

//! Calculate profit of adopting behaviour.
void calcProfitCLOSE_LOOP (CloseLoopData *c, RobotDatabase *db, const float maxProfit);

//! Set the dest when we don't want to calculate the profit, but just do a close loop anyway
void CloseLoop_calcDest (CloseLoopData *c, RobotDatabase *db);

//! Adjust map data collected by robot given improved location estimate.
void CloseLoop_processData (RobotDatabase *db);

void CloseLoop_processIsAtDest (RobotDatabase *db);

//! Set behaviour and reset dest such that IK data is calculated.
void adoptCLOSE_LOOP (RobotDatabase *db, const int initialSync);

#endif
#endif
