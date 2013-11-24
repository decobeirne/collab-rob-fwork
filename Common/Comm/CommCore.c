

#include "CommCore.h"
#ifdef SIMULATION
#include "../../Board/Data/BoardDatabase.h"
#include "../../Robot/Data/RobotDatabase.h"
#endif



int CommCore_sendInitPacket (CommData *commData, const COMM_FLAG flag, const int payload)
{
	int res;
	const int sz = sizeof (COMM_FLAG) + sizeof (int);

	memcpy (commData->buffer,							&flag,		sizeof (COMM_FLAG));
	memcpy (commData->buffer + sizeof (COMM_FLAG),		&payload,	sizeof (int));

	res = send (commData->socket, commData->buffer, sz, 0);

	return res;
}

int CommCore_recvAck (CommData *commData, const COMM_FLAG flag)
{
	int res;
	COMM_FLAG f;

	res = recv (commData->socket, commData->buffer, sizeof (COMM_FLAG), 0);

	memcpy (&f, commData->buffer, sizeof (COMM_FLAG));

	if (-1 == res || flag != f)
	{
		return -1;
	}
	return res;
}

void CommCore_clearBuffer (CommData *commData)
{
	commData->bufferCurrentSize = 0;
}

void CommCore_addItemToBuffer (CommData *commData, const void *item, const int sz)
{
	memcpy (commData->buffer + commData->bufferCurrentSize, item, sz);

	commData->bufferCurrentSize += sz;
}

int CommCore_sendBuffer (CommData *commData, const int sz)
{
	int res;

	if (sz != commData->bufferCurrentSize)
	{
		printf ("Error size is not correct\n");
		return -1;
	}

	res = send (commData->socket, commData->buffer, sz, 0);
	return res;
}

int CommCore_recvBuffer (CommData *commData, const int sz)
{
	int res;
	res = recv (commData->socket, commData->buffer, sz, 0);
	return res;
}

int CommCore_sendAck (CommData *commData, const COMM_FLAG flag)
{
	int res;

	memcpy (commData->buffer, &flag, sizeof (COMM_FLAG));

	res = send (commData->socket, commData->buffer, sizeof (COMM_FLAG), 0);
	return res;
}






















//#define DEBUG_LIST_COMM

void CommCore_sendList (CommData *commData,
				 const List *list,
				 const int valueSize,
				 const int valueType)
{
	int res;
	int sz;
	int i;
	int totalSize;
	int sizeThisWrite;
	int nThisWrite;
	int sizeLeftToWrite;
	int isAnotherWriteRequired;
	const ListNode *listNode;
#ifdef DEBUG_LIST_COMM
	int debugSz;
#endif

	// Send package info
	totalSize = list->size * valueSize;
	sizeLeftToWrite = totalSize;
	if (totalSize > BUFFER_SIZE)
	{
		isAnotherWriteRequired = 1;
		nThisWrite = BUFFER_SIZE / valueSize; // Integer division will round down => good
		sizeThisWrite = nThisWrite * valueSize;
	}
	else
	{
		isAnotherWriteRequired = 0;
		nThisWrite = list->size;
		sizeThisWrite = totalSize;
	}

	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, &totalSize, sizeof (int));
	CommCore_addItemToBuffer (commData, &sizeThisWrite, sizeof (int));
	CommCore_addItemToBuffer (commData, &nThisWrite, sizeof (int));
	CommCore_addItemToBuffer (commData, &valueSize, sizeof (int));
	CommCore_addItemToBuffer (commData, &valueType, sizeof (int));
	CommCore_addItemToBuffer (commData, &isAnotherWriteRequired, sizeof (int));
	sz = sizeof (int) * 6;

#ifdef DEBUG_LIST_COMM
	printf ("%s: totalSize=%d sizeThisWrite=%d nThisWrite=%d \
valueSize=%d valueType=%d isAnotherWriteRequired=%d\n",
			__func__, totalSize, sizeThisWrite, nThisWrite,
			valueSize, valueType, isAnotherWriteRequired);
	fflush (stdout);
#endif // DEBUG_LIST_COMM



#ifdef DEBUG_LIST_COMM
	printf ("%s: sending buffer sz:%d\n", __func__, sz);
	fflush (stdout);
#endif // DEBUG_LIST_COMM
	res = CommCore_sendBuffer (commData, sz);
	assert (0 < res);

#ifdef DEBUG_LIST_COMM
	printf ("%s: receiving ack\n", __func__);
	fflush (stdout);
#endif // DEBUG_LIST_COMM
	res = CommCore_recvAck (commData, W_LIST);
	assert (0 < res);

	// Send buffer
	CommCore_clearBuffer (commData);

#ifdef DEBUG_LIST_COMM
	debugSz = 0;
#endif // DEBUG_LIST_COMM

	listNode = list->front;
	for (i = 0; i < nThisWrite; ++i)
	{
		CommCore_addItemToBuffer (commData, listNode->value, valueSize);
		listNode = listNode->next;
#ifdef DEBUG_LIST_COMM
		debugSz += valueSize;
#endif // DEBUG_LIST_COMM
	}

	sizeLeftToWrite -= sizeThisWrite;
	commData->bufferCurrentSize = sizeThisWrite;

#ifdef DEBUG_LIST_COMM
	printf ("%s: sending list szThisWrite=%d debugSz=%d\n", __func__, sizeThisWrite, debugSz);
	fflush (stdout);
#endif // DEBUG_LIST_COMM

	if (sizeThisWrite)
	{
		res = CommCore_sendBuffer (commData, sizeThisWrite);
		assert (0 < res);

		res = CommCore_recvAck (commData, W_LIST);
		assert (0 < res);
	}
	else
	{
		DEBUG_ASSERT(!isAnotherWriteRequired)
	}

	while (isAnotherWriteRequired)
	{
		// Send package info
		if (sizeLeftToWrite > BUFFER_SIZE)
		{
			isAnotherWriteRequired = 1;
			nThisWrite = BUFFER_SIZE / valueSize;
			sizeThisWrite = nThisWrite * valueSize;
		}
		else
		{
			isAnotherWriteRequired = 0;
			nThisWrite = sizeLeftToWrite / valueSize;
			sizeThisWrite = sizeLeftToWrite;
		}

		CommCore_clearBuffer (commData);
		CommCore_addItemToBuffer (commData, &sizeThisWrite, sizeof (int));
		CommCore_addItemToBuffer (commData, &nThisWrite, sizeof (int));
		CommCore_addItemToBuffer (commData, &isAnotherWriteRequired, sizeof (int));

#ifdef DEBUG_LIST_COMM
		printf ("%s: sending buffer sizeThisWrite=%d nThisWrite=%d isAnotherWriteRequired=%d\n", __func__, sizeThisWrite, nThisWrite, isAnotherWriteRequired);
		fflush (stdout);
#endif // DEBUG_LIST_COMM

		res = CommCore_sendBuffer (commData, sizeof (int) * 3);
		assert (0 < res);

#ifdef DEBUG_LIST_COMM
		printf ("%s: receiving ack\n", __func__);
		fflush (stdout);
#endif // DEBUG_LIST_COMM

		res = CommCore_recvAck (commData, W_LIST);
		assert (0 < res);

		// Send buffer
		CommCore_clearBuffer (commData);

		for (i = 0; i < nThisWrite; ++i)
		{
			CommCore_addItemToBuffer (commData, listNode->value, valueSize);
			listNode = listNode->next;
		}

		sizeLeftToWrite -= sizeThisWrite;
		commData->bufferCurrentSize = sizeThisWrite;

#ifdef DEBUG_LIST_COMM
		printf ("%s: sending buffer sizeLeftToWrite=%d bcs=%d sizeThisWrite=%d\n", __func__, sizeLeftToWrite, commData->bufferCurrentSize, sizeThisWrite);
		fflush (stdout);
#endif // DEBUG_LIST_COMM

		res = CommCore_sendBuffer (commData, sizeThisWrite);
		assert (0 < res);

		res = CommCore_recvAck (commData, W_LIST);
		assert (0 < res);
	}
}

void CommCore_readList (CommData *commData, List *list)
{
	int i;
	int res;
	int sz;
	int totalSize;
	int sizeThisRead;
	int nThisRead;
	int valueSize;
	int valueType;
	int isAnotherReadRequired;
	schar *bufferPtr;
	PointF *ptfPtr;
	PointI *ptiPtr;

	// Recv 5 ints required to get list
	sz = sizeof (int) * 6;
#ifdef DEBUG_LIST_COMM
	printf ("%s: receiving buffer %d\n", __func__, sz);
	fflush (stdout);
#endif // DEBUG_LIST_COMM
	res = CommCore_recvBuffer (commData, sz);
	assert (0 < res);

	bufferPtr = (uchar*)commData->buffer;
	memcpy (&totalSize, bufferPtr, sizeof (int));
	bufferPtr += sizeof (int);
	memcpy (&sizeThisRead, bufferPtr, sizeof (int));
	bufferPtr += sizeof (int);
	memcpy (&nThisRead, bufferPtr, sizeof (int));
	bufferPtr += sizeof (int);
	memcpy (&valueSize, bufferPtr, sizeof (int));
	bufferPtr += sizeof (int);
	memcpy (&valueType, bufferPtr, sizeof (int));
	bufferPtr += sizeof (int);
	memcpy (&isAnotherReadRequired, bufferPtr, sizeof (int));

#ifdef DEBUG_LIST_COMM
	printf ("%s: sending ack\n", __func__);
	fflush (stdout);
#endif // DEBUG_LIST_COMM
	res = CommCore_sendAck (commData, W_LIST);
	assert (0 < res);

#ifdef DEBUG_LIST_COMM
	printf ("%s: receiving list totalSize=%d sizeThisRead=%d nThisRead=%d \
valueSize=%d valueType=%d isAnotherReadRequired=%d\n",
			__func__, totalSize, sizeThisRead, nThisRead,
			valueSize, valueType, isAnotherReadRequired);
	fflush (stdout);
#endif // DEBUG_LIST_COMM

	if (sizeThisRead != 0)
	{
		// Recv buffer
		res = CommCore_recvBuffer (commData, sizeThisRead);
		assert (0 < res);

	bufferPtr = commData->buffer;
	if (valueType == 0) // PointF
	{
		for (i = 0; i < nThisRead; ++i)
		{
			ptfPtr = (PointF*)malloc (sizeof (PointF));
			*ptfPtr = *(PointF*)bufferPtr;
			List_pushValue (list, ptfPtr);

			bufferPtr += valueSize;
		}
		
	}
	else if (valueType == 1) // PointI
	{
		for (i = 0; i < nThisRead; ++i)
		{
			ptiPtr = (PointI*)malloc (sizeof (PointI));
			*ptiPtr = *(PointI*)bufferPtr;
			List_pushValue (list, ptiPtr);

			bufferPtr += valueSize;
		}
	}
	else
	{
		assert (0);
	}

		res = CommCore_sendAck (commData, W_LIST);
		assert (0 < res);
	}
	else
	{
		DEBUG_ASSERT(!isAnotherReadRequired)
	}

	while (isAnotherReadRequired)
	{
		// Recv info
		res = CommCore_recvBuffer (commData, sizeof (int) * 3);
		assert (0 < res);

		memcpy (&sizeThisRead, commData->buffer, sizeof (int));
		memcpy (&nThisRead, commData->buffer + sizeof (int), sizeof (int));
		memcpy (&isAnotherReadRequired, commData->buffer + sizeof (int) * 2, sizeof (int));

#ifdef DEBUG_LIST_COMM
		printf ("%s: received buffer sizeThisRead=%d nThisRead=%d isAnotherReadRequired=%d\n", __func__, sizeThisRead, nThisRead, isAnotherReadRequired);
		fflush (stdout);
#endif // DEBUG_LIST_COMM

		res = CommCore_sendAck (commData, W_LIST);
		assert (0 < res);

		// Recv buffer
		res = CommCore_recvBuffer (commData, sizeThisRead);
		assert (0 < res);

		bufferPtr = commData->buffer;
		if (valueType == 0) // PointF
		{
			for (i = 0; i < nThisRead; ++i)
			{
				ptfPtr = (PointF*)malloc (sizeof (PointF));
				*ptfPtr = *(PointF*)bufferPtr;
				List_pushValue (list, ptfPtr);

				bufferPtr += valueSize;
			}
			
		}
		else
		{
			assert (0);
		}

		res = CommCore_sendAck (commData, W_LIST);
		assert (0 < res);
	}
}























void CommCore_sendMap (
				CommData *commData,
				CompressedImage *compressedImage)
{
	int res;
	int sz;
	int totalSize;
	int sizeThisWrite;
	int sizeLeftToWrite;
	int isAnotherWriteRequired;
	uchar *imagePtr;

	// Send package info
	totalSize = compressedImage->usedSize;
	sizeLeftToWrite = totalSize;
	if (sizeLeftToWrite > BUFFER_SIZE)
	{
		isAnotherWriteRequired = 1;
		sizeThisWrite = BUFFER_SIZE;
	}
	else
	{
		isAnotherWriteRequired = 0;
		sizeThisWrite = sizeLeftToWrite;
	}

	CommCore_clearBuffer (commData);
	CommCore_addItemToBuffer (commData, &totalSize, sizeof (int));
	CommCore_addItemToBuffer (commData, &sizeThisWrite, sizeof (int));
	CommCore_addItemToBuffer (commData, &isAnotherWriteRequired, sizeof (int));
	CommCore_addItemToBuffer (commData, &compressedImage->orig, sizeof (PointI));
	sz = (sizeof (int) * 3) + sizeof (PointI);
	res = CommCore_sendBuffer (commData, sz);
	assert (0 < res);

	res = CommCore_recvAck (commData, W_MAP);
	assert (0 < res);

	// Send buffer
	CommCore_clearBuffer (commData);

	imagePtr = compressedImage->buffer;
	memcpy (commData->buffer, imagePtr, sizeThisWrite);

	imagePtr += sizeThisWrite;
	sizeLeftToWrite -= sizeThisWrite;
	commData->bufferCurrentSize = sizeThisWrite;

	res = CommCore_sendBuffer (commData, sizeThisWrite);
	assert (0 < res);

	res = CommCore_recvAck (commData, W_MAP);
	assert (0 < res);

	while (isAnotherWriteRequired)
	{
		// Send package info
		if (sizeLeftToWrite > BUFFER_SIZE)
		{
			isAnotherWriteRequired = 1;
			sizeThisWrite = BUFFER_SIZE;
		}
		else
		{
			isAnotherWriteRequired = 0;
			sizeThisWrite = sizeLeftToWrite;
		}

		CommCore_clearBuffer (commData);
		CommCore_addItemToBuffer (commData, &sizeThisWrite, sizeof (int));
		CommCore_addItemToBuffer (commData, &isAnotherWriteRequired, sizeof (int));

		res = CommCore_sendBuffer (commData, sizeof (int) * 2);
		assert (0 < res);

		res = CommCore_recvAck (commData, W_MAP);
		assert (0 < res);

		// Send buffer
		CommCore_clearBuffer (commData);
		memcpy (commData->buffer, imagePtr, sizeThisWrite);

		imagePtr += sizeThisWrite;
		sizeLeftToWrite -= sizeThisWrite;
		commData->bufferCurrentSize = sizeThisWrite;

		res = CommCore_sendBuffer (commData, sizeThisWrite);
		assert (0 < res);

		res = CommCore_recvAck (commData, W_MAP);
		assert (0 < res);
	}
}


void CommCore_readMap (CommData *commData, CompressedImage *compressedImage, const int allocNewBuffer)
{
	int res;
	int sz;
	int totalSize;
	int sizeThisRead;
	int isAnotherReadRequired;
	schar *bufferPtr;

	if (allocNewBuffer)
	{
		initCompressedImage(compressedImage);
	}

	// Recv 3 ints required to get image, and also image origin
	sz = (sizeof (int) * 3) + sizeof (PointI);
	res = CommCore_recvBuffer (commData, sz);
	assert (0 < res);

	bufferPtr = (uchar*)commData->buffer;
	memcpy (&totalSize, bufferPtr, sizeof (int));
	bufferPtr += sizeof (int);
	memcpy (&sizeThisRead, bufferPtr, sizeof (int));
	bufferPtr += sizeof (int);
	memcpy (&isAnotherReadRequired, bufferPtr, sizeof (int));
	bufferPtr += sizeof (int);
	memcpy (&compressedImage->orig, bufferPtr, sizeof (PointI));

	res = CommCore_sendAck (commData, W_MAP);
	assert (0 < res);

	// Recv buffer
	res = CommCore_recvBuffer (commData, sizeThisRead);
	assert (0 < res);

	if (allocNewBuffer)
	{
		compressedImage->buffer = (uchar*)malloc (totalSize);
		compressedImage->bufferSize = totalSize;
	}

	memcpy (compressedImage->buffer, commData->buffer, sizeThisRead);
	compressedImage->usedSize = sizeThisRead;

	res = CommCore_sendAck (commData, W_MAP);
	assert (0 < res);

	while (isAnotherReadRequired)
	{
		// Recv info
		res = CommCore_recvBuffer (commData, sizeof (int) * 2);
		assert (0 < res);

		memcpy (&sizeThisRead, commData->buffer, sizeof (int));
		memcpy (&isAnotherReadRequired, commData->buffer + sizeof (int), sizeof (int));

		res = CommCore_sendAck (commData, W_MAP);
		assert (0 < res);

		// Recv buffer
		res = CommCore_recvBuffer (commData, sizeThisRead);
		assert (0 < res);

		memcpy (compressedImage->buffer + compressedImage->usedSize, commData->buffer, sizeThisRead);
		compressedImage->usedSize += sizeThisRead;

		res = CommCore_sendAck (commData, W_MAP);
		assert (0 < res);
	}
}
































