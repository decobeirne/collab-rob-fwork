#include "../Common/RobotDefs.h"

#ifndef ROBOT
#include "Board.h"

#ifdef IS_WIN
#include "process.h"
#else
#include "pthread.h"
#endif

extern char experimentDirName[128];

BoardThreadParams initBoardThreadParams (BoardDatabase *db,
										 const int index)
{
	char fileName[120];
	BoardThreadParams params;
	time_t t;
	time (&t);

	sprintf (fileName, "%s/thread_%d_%d.txt", experimentDirName, index,(int)t);

	params.commData = initCommData();
	params.db = db;
	params.index = index;
	params.xmlLog = fopen (fileName, "w");
	RELEASE_ASSERT(params.xmlLog)
	return params;
}

void clearBoardThreadParams (BoardThreadParams *params)
{
	if (params->xmlLog)
	{
		fclose (params->xmlLog);
	}
}

void setupEnvironment (Board *b)
{
	PointI bl, tr;

	Image_fill (b->db.environment.map, 127);

#if 1
	MapCore_setupObstacles (&b->db.environment.origObstList);
#endif
	MapCore_displayTerrain (b->db.environment.tempMap, &b->db.environment.origObstList);

	BoardMapProcessing_updNavMap (
		&b->db,
		b->db.environment.map,
		b->db.environment.tempMap);

	bl.x =0;
	bl.y = 0;
	tr.x = GLOB_EXP_GRID_DIMS - 1;
	tr.y = GLOB_EXP_GRID_DIMS - 1;

	BoardMapProcessing_updateMapGrid (&b->db, bl, tr);

	BoardMapProcessing_resetNewDataFlags (&b->db);
}

static int seeds[2] = {1340644673, 1340644685};
static int seedIndex = 0;

Board initBoard (
#ifdef IS_WIN
				 IplImage *globMapIplImage,
				 IplImage *localMapIplImage
#endif
				 )
{
	Board b;
	unsigned int seed;

	b.db = initBoardDatabase (
#ifdef IS_WIN
		globMapIplImage,
		localMapIplImage
#endif
		);

#ifdef BOARD
	initialiseCriticalSections();
#endif

	{
		seed = (unsigned int)time(0);
//		seed = 1380244778;
//		seed = seeds[seedIndex++];
		srand (seed);
		fprintf (b.db.xmlLog, "<RandomSeed>%d</RandomSeed>\n", seed);
	}

	setupEnvironment (&b);
	return b;
}



void clearBoard (Board *b)
{
#ifdef BOARD
	deleteCriticalSections();
#endif

	BoardDatabase_dtor (&b->db);
}

#ifdef SIMULATION
extern int explicitlyShowWipe;
#endif
void Board_processData (Board *b, const int updateVisualisation)
{
#ifdef PRINT_EVENTS
	BoardDatabase_printStatus (&b->db);
#endif

	BoardManagement_checkTimeElapsed (&b->db);

	BoardMapProcessing_processMapData (&b->db);

#ifdef SHOW_IMGS
	if (updateVisualisation)
	{
		Visualisation_showSim (&b->db);
		Visualisation_showMap (&b->db);
#ifdef SIMULATION
		if (explicitlyShowWipe)
		{
			printf ("Provisional data wiped on board!\n");
//			cvWaitKey (0);
			cvWaitKey (1);
			explicitlyShowWipe = 0;
		}
		else
		{
			cvWaitKey (1);
		}
#else
		cvWaitKey (1);
#endif
	}
#endif
}

void Board_finish (Board *b)
{
#ifdef PRINT_PROFIT
	PointI bl, tr;
	bl.x = 0;
	bl.y = 0;
	tr.x = ENVIR_DIMS - LOC_MAP_DIMS;
	tr.y = ENVIR_DIMS - LOC_MAP_DIMS;
	BoardMapIntegration_refreshMapArea (&b->db, bl, tr);
#endif

#ifdef PRINT_MAP_DETAIL
	BoardDatabase_printAllGrids (&b->db);
#endif

#ifdef PRINT_EVENTS
	BoardDatabase_printGlobalMapList (&b->db);
#endif

#ifdef PRINT_PROFIT
	BoardDatabase_printGlobalMap (&b->db);
#endif

#ifdef SHOW_IMGS
	printf ("\npress any key on window to continue\n");

	Visualisation_showSim (&b->db);
	Visualisation_showMap (&b->db);
//	cvWaitKey (0);
	cvWaitKey (1);
#endif
}











#ifdef BOARD
//! Control loop for thread on blackboard accepting requests from a single robot.
#ifdef IS_WIN
void runAcceptRequests (void *v)
#else
void* runAcceptRequests (void *v)
#endif
{
	BoardThreadParams* params = (BoardThreadParams*)v;
	BoardDatabase *db = params->db;
	CommData *commData = &params->commData;
	const int sz = sizeof (COMM_FLAG) + sizeof (int);
	COMM_FLAG flag;
	int payload;
	int res;

	commData->quitFlag = 0;
	while (!commData->quitFlag)
	{
		res = CommCore_recvBuffer (commData, sz);
		assert (0 < res);

		memcpy (&flag, commData->buffer, sizeof (COMM_FLAG));
		memcpy (&payload, commData->buffer + sizeof (COMM_FLAG), sizeof (int));

		BoardComm_processRobotRequest (db, commData, flag, payload, params->xmlLog);
	}

#ifdef IS_WIN
	if (shutdown (params->commData.socket, SD_SEND) != 0)
	{
		printf ("Error shutting down connection %d\n",  WSAGetLastError());
	}

#else

	if (close (params->commData.socket) != 0)
	{
		printf ("Error shutting down connection\n");
	}
#endif

	enterBoardData();
	db->groupData.experimentStatus.updateFlag = 1;
	++db->groupData.experimentStatus.nFinished;
	leaveBoardData();

#ifndef IS_WIN
	return NULL;
#endif
}












#ifdef IS_WIN
static int waitForConnection (SOCKET socket)
{
	struct fd_set fds;
	FD_ZERO(&fds);
	FD_SET(socket, &fds);

	return select (0, &fds, 0, 0, NULL);
}
#endif

int setupSockets (BoardDatabase *db, BoardThreadParams threadParams[MAX_N_ROBOTS])
{
	int i;
	int Port = 7171;
#ifdef IS_WIN
	int waitResult;
	WSADATA wsaData;
	SOCKADDR_IN serverAddr;
#else
	struct sockaddr_in serverAddr;
#endif

#ifdef IS_WIN
	if (WSAStartup(MAKEWORD(2,2), &wsaData) != 0)
	{
		return -1;
	}

	db->commData.socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (db->commData.socket == INVALID_SOCKET)
	{
		printf ("Error creating socket %d\n", WSAGetLastError());
		WSACleanup();
		return -1;
	}

	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(Port);
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

#else

	db->commData.socket = socket(AF_INET, SOCK_STREAM, 0);
	if (db->commData.socket < 0)
	{
		printf ("Error creating socket %d\n", db->commData.socket);
		return -1;
	}

	bzero ((char*)&serverAddr, sizeof(serverAddr));
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(Port);
	serverAddr.sin_addr.s_addr = INADDR_ANY;
#endif

#ifdef IS_WIN
	if (bind (db->commData.socket, (SOCKADDR *)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
	{
		printf ("Error binding socket %d\n", WSAGetLastError());
		closesocket(db->commData.socket);
		WSACleanup();
		return -1;
	}

	if (listen (db->commData.socket, 5) == SOCKET_ERROR)
	{
		printf ("Error listening %d\n", WSAGetLastError());
		closesocket (db->commData.socket);
		WSACleanup();
		return -1;
	}

#else

	if (bind (db->commData.socket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0)
	{
		printf ("Error binding socket\n");
		closesocket (db->commData.socket);
		WSACleanup();
		return -1;
	}

	if (listen (db->commData.socket, 5) != 0)
	{
		printf ("Error listening\n");
		closesocket (db->commData.socket);
		WSACleanup();
		return -1;
	}
#endif
	printf ("Blackboard ready.\n");
	printf ("Waiting for %d robots to connect.\n", N_ROBOTS);

	for (i = 0; i < N_ROBOT_THREADS; ++i)
//	for (i = 0; i < N_ROBOTS; ++i)
	{
		threadParams[i] = initBoardThreadParams (db, i);

#ifdef IS_WIN
		waitResult = waitForConnection (db->commData.socket);
		if (0 > waitResult)
		{
			printf ("Error waiting for connection %d\n", waitResult);
			closesocket (db->commData.socket);
			WSACleanup();
			return -1;
		}

		threadParams[i].commData.socket = accept (db->commData.socket, NULL, NULL);
		if (threadParams[i].commData.socket == SOCKET_ERROR)
		{
			printf ("Error accepting connection %d\n", WSAGetLastError());
			closesocket (db->commData.socket);
			WSACleanup();
			return -1;
		}

#else

		threadParams[i].commData.socket = accept(db->commData.socket, NULL, NULL); 
		if (threadParams[i].commData.socket < 0)
		{
			printf ("Error accepting connection\n");
			return -1;
		}
#endif

		printf ("Connection received for thread %d.\n", i);
	}

	return 1;
}

int setupThreads (BoardDatabase *db, BoardThreadParams threadParams[MAX_N_ROBOTS])
{
	int i;
	for (i = 0; i < N_ROBOT_THREADS; ++i)
//	for (i = 0; i < N_ROBOTS; ++i)
	{
#ifdef IS_WIN
		if (-1 == _beginthread (runAcceptRequests, 0, &threadParams[i]))
		{
			return -1;
		}

#else

		if (-1 == pthread_create(&db->threads[i], NULL, runAcceptRequests, &threadParams[i]))
		{
			return -1;
		}
#endif
		printf ("Setup thread for robot %d.\n", i);
	}
	return 0;
}

void closeSockets (BoardDatabase *db)
{
#ifdef IS_WIN
	if (closesocket (db->commData.socket) != 0)
	{
		printf ("Error closing listening socket %d.\n", WSAGetLastError());
	}
	WSACleanup();

#else

	int i;
	for (i = 0; i < N_ROBOT_THREADS; ++i)
//	for (i = 0; i < N_ROBOTS; ++i)
	{
		pthread_join(db->threads[i], NULL);
	}

	if (close (db->commData.socket) != 0)
	{
		printf ("Error closing listening socket.\n");
	}

	printf ("All sockets closed.\n");
#endif
}

void checkTime (BoardDatabase *db, const int experimentEndTime, int *updateFlag)
{
	db->groupData.experimentStatus.isTimeElapsed = (clock() > experimentEndTime);

	*updateFlag = db->groupData.experimentStatus.updateFlag;
	db->groupData.experimentStatus.updateFlag = 0;
}

int checkIfAllFinished (BoardDatabase *db)
{
	//if (db->groupData.experimentStatus.nFinished != 0)
	//{
	//	fprintf (db->xmlLog, "%d fin\n", db->groupData.experimentStatus.nFinished);
	//}
//	return (db->groupData.experimentStatus.nFinished == N_ROBOTS);
	return (db->groupData.experimentStatus.nFinished == N_ROBOT_THREADS);
}












#ifdef SETUP_TEST_COALITION
extern float grobotsLocs[MAX_N_ROBOTS * 3];

void Board_setupTestCoalRobots (BoardDatabase *db)
{
	db->groupData.robots[1].actualLocOffset = initPointF (0.0f);
	db->groupData.robots[1].baseBehaviour = SUPERVISION;
	db->groupData.robots[1].behaviour = SUPERVISION;
	db->groupData.robots[1].localMapOrigin = initPointI (0, 0);
//	db->groupData.robots[1].pose.loc.x = grobotsLocs[1 * 3 + 0]; // Same as in Robot_init above
//	db->groupData.robots[1].pose.loc.y = grobotsLocs[1 * 3 + 1];
	db->groupData.robots[1].pose.loc.x = 250.0f;
	db->groupData.robots[1].pose.loc.y = 200.0f;
	db->groupData.robots[1].pose.orient = 0.0f;
	db->groupData.robots[1].pose.mat[0] = 0.0f;
	db->groupData.robots[1].pose.mat[1] = 0.0f;
	db->groupData.robots[1].pose.mat[2] = 0.0f;
	db->groupData.robots[1].pose.mat[3] = 0.0f;
	db->groupData.robots[1].target.x = 252.0f;
	db->groupData.robots[1].target.y = 126.0f;
	db->groupData.robots[1].stdDev = 0.0f;
	db->groupData.robots[1].localMapOrigin.x = 210;
	db->groupData.robots[1].localMapOrigin.y = 168;
}
#endif


void Board_start()
{
	int experimentEndTime;
	int updateFlag;
	int quitFlag;
	Board b;
	BoardThreadParams threadParams[MAX_N_ROBOTS];
	BoardDatabase *db;

#ifdef IS_WIN
	CvSize szGlobal = {ENVIR_DIMS, ENVIR_DIMS};
	CvSize szLocal = {LOC_MAP_DIMS, LOC_MAP_DIMS};
	IplImage *globMapIplImage;
	IplImage *localMapIplImage;
#endif

	setupExperimentDir();

	// Do this before setting up the database
	SET_N_ROBOTS(1)
#ifdef SETUP_TEST_COALITION
	SET_N_ROBOTS(2)
	SET_N_ROBOTS_THREADS_FOR_TEST_COAL(1)
#endif

#ifdef IS_WIN
	globMapIplImage = cvCreateImage (szGlobal, 8, 1);
	localMapIplImage = cvCreateImage (szLocal, 8, 1);
	b = initBoard (globMapIplImage, localMapIplImage);
#else
	b = initBoard();
#endif
	db = &b.db;

#ifdef SETUP_TEST_COALITION
	Board_setupTestCoalRobots (&b.db);
#endif

#ifdef SHOW_IMGS
	Visualisation_showSim (db);
	Visualisation_showMap (db);
//	printf ("Adjust windows and click to start\n"); cvWaitKey (0);
#endif

	if (-1 == setupSockets (db, threadParams))
	{
		goto cleanupBoard;
	}

	experimentEndTime = clock() + 10000;
	printf ("Experiment ends at %d.\n", experimentEndTime);

	if (-1 == setupThreads (db, threadParams))
	{
		goto cleanupBoard;
	}

	printf ("All threads started. Entering blackboard server loop.\n");

	quitFlag = 0;
	while (!quitFlag)
	{
		enterBoardData();
		checkTime (db, experimentEndTime, &updateFlag);
		if (updateFlag)
		{
			Board_processData (&b, 1);

			//! \todo Allocate coalitions

			quitFlag = checkIfAllFinished (db);
		}
		leaveBoardData();

		if (!updateFlag)
		{
			SLEEP_FOR_MS(10)
		}
	}

	printf ("Finished loop. Closing sockets.\n");

	closeSockets (db);

	Board_finish (&b);

cleanupBoard:
	clearBoard (&b);
#ifdef IS_WIN
	cvReleaseImage (&globMapIplImage);
	cvReleaseImage (&localMapIplImage);
	cvCleanup();
#endif
}
#endif
#endif
