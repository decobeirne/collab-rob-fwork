#include "../../Common/RobotDefs.h"

#if defined(ROBOT) && (defined(RECORDING_ROBOT) || defined(RERUNNING_ROBOT))

/*!
\file RobotCommSerialize.h
\brief Serialize chunks of data read from the blackboard.
*/
#ifndef ROBOT_COMM_SERIALIZE_H
#define ROBOT_COMM_SERIALIZE_H

typedef enum SERIALIZED_TYPES_
{
	SER_STATUS_FROM_BOARD = 0, // MapStatusFromBoard
	SER_GRIDS = 1,
	SER_MAP = 2,
	SER_GROUP = 3,
	SER_UNREACHABLE = 4,
	SER_CAM = 5,

} SERIALIZED_TYPES;


#include "../Data/RobotDatabase.h"
#include "../../Common/Comm/CommCore.h"


#if defined(RERUNNING_ROBOT)

//! Find start and end ptrs for the current iter in the recorded file.
void getNextRobotIter_serialize (RobotDatabase *db);

void readStatus_serialize (RobotDatabase *db, MapStatusFromBoard *s);

void readImage_serialize (RobotDatabase *db, int *isImg);

void readCompass_serialize (RobotDatabase *db);

void readGrids_serialize (RobotDatabase *db);

void readInitialRobotLoc_serialize (RobotDatabase *db);

void readUnreachableGrid_serialize (RobotDatabase *db);

void readMap_serialize (RobotDatabase *db);

void readGroup_serialize (RobotDatabase *db);

#endif // defined(RERUNNING_ROBOT)




#if defined(RECORDING_ROBOT)

void writeStatus_serialize (RobotDatabase *db, const MapStatusFromBoard *s);

void writeGrids_serialize (RobotDatabase *db);

void writeMap_serialize (RobotDatabase *db);

void writeGroup_serialize (RobotDatabase *db);

void writeUnreachableGrid_serialize (RobotDatabase *db);

#endif // defined(RECORDING_ROBOT)


#endif // defined(ROBOT) && (defined(RECORDING_ROBOT) || defined(RERUNNING_ROBOT))

#endif // ROBOT_COMM_SERIALIZE_H
