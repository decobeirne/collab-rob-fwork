
/*!
\file CommCore.h
\brief Core communication funcionality.
*/
#ifndef ROBOT_COMM_CORE_H
#define ROBOT_COMM_CORE_H

#include "../RobotTypes.h"


//! Send packet with flag to board to prepare it to send/receive data.
int CommCore_sendInitPacket (CommData *commData, const COMM_FLAG flag, const int payload);

int CommCore_recvAck (CommData *commData, const COMM_FLAG flag);

void CommCore_clearBuffer (CommData *commData);

void CommCore_addItemToBuffer (CommData *commData, const void *item, const int sz);

int CommCore_sendBuffer (CommData *commData, const int sz);

int CommCore_recvBuffer (CommData *commData, const int sz);

int CommCore_sendAck (CommData *commData, const COMM_FLAG flag);




void CommCore_sendList (
	CommData *commData,
	const List *list,
	const int valueSize,
	const int valueType);

void CommCore_readList (
	CommData *commData,
	List *list);


void CommCore_sendMap (
	CommData *commData,
	CompressedImage *compressedImage);

void CommCore_readMap (
	CommData *commData,
	CompressedImage *compressedImage,
	const int allocNewBuffer);





#endif // ifndef
