#include "../../Common/RobotDefs.h"

#if !defined (ROBOT) || defined (IS_WIN)
#include "BoardDatabase.h"
#include "../../Common/Geometry.h"
#include "../../Common/Vector.h"



#ifdef ON_LAPTOP
int simWindowSize[4] = {200, 10, 520, 560};
int mapWindowSize[4] = {720, 10, 520, 560};
int pathWindowSize[4] = {600, 10, 90, 130};
int robotMapWindowSize[4] = {10, 10, 90, 130};
int robotNavMapWindowSize[4] = {10, 150, 90, 130};

#else // Dell

int simWindowSize[4] = {500, 600, 520, 560};
int mapWindowSize[4] = {1020, 600, 520, 560};
int pathWindowSize[4] = {900, 600, 90, 130};
int robotMapWindowSize[4] = {200, 800, 90, 130};
int robotNavMapWindowSize[4] = {200, 950, 90, 130};
#endif // ON_LAPTOP
#endif // !defined (ROBOT) || defined (IS_WIN)

#ifndef ROBOT
extern char experimentDirName[128];

BoardDatabase initBoardDatabase (
#ifdef IS_WIN
								 IplImage *globMapIplImage,
								 IplImage *localMapIplImage
#endif
								 )
{
	BoardDatabase data;
	char filename[120];
	time_t t;

#ifdef IS_WIN
	data.globMapIplImage = globMapIplImage;
	data.localMapIplImage = localMapIplImage;
#endif

	data.groupData = initBoardGroupData();
	data.environment = initBoardEnvironment();
	data.coalitionData = initBoardCoalitionData();
	data.sensorData = initBoardSensorData();
	data.commData = initCommData();

	time (&t);
	sprintf (filename, "%s/board_%d.txt", experimentDirName, (int)t);
	data.xmlLog = fopen (filename, "w");

	//CamVectors_init (
	//	&data.camVectors);
	//GeometryConstants_init (
	//	&data.geometryConstants,
	//	&data.camVectors,
	//	data.xmlLog,
	//	0);
	//IkConstants_init (
	//	&data.geometryConstants,
	//	&data.ikConstants);
	UncertaintyConstants_init (
		&data.uncertaintyConstants);


#ifdef BOARD
	data.updateFlag = 0;
#endif
	return data;
}

void BoardDatabase_dtor (BoardDatabase *data)
{
	BoardGroupData_dtor (&data->groupData);
	BoardEnvironment_dtor (&data->environment);
	freeBoardCoalitionData (&data->coalitionData);
	BoardSensorData_dtor (&data->sensorData);

	fclose (data->xmlLog);
}

//! Display board information at each iteration
void BoardDatabase_printStatus (BoardDatabase *data)
{

}

void BoardDatabase_printExpGridSection (BoardDatabase *db,
										const PointI bl,
										const PointI tr)
{
	FILE *f = db->xmlLog;
	int i, j;
	fprintf (f, "<ExpGrid>\n");
	fprintf (f, "      ");
	for (i = 0; i < GLOB_EXP_GRID_DIMS; ++i)
	{
		fprintf (f, "%6d", i);
	}
	fprintf (f, "\n");
	for (j = GLOB_EXP_GRID_DIMS - 1; j >= 0; --j)
	{
		fprintf (f, "%6d", j);
		for (i = 0; i < GLOB_EXP_GRID_DIMS; ++i)
		{
			fprintf (f, "%6d", db->environment.expGrid[i][j]);
		}
		fprintf (f, "\n");
	}
	fprintf (f, "</ExpGrid>\n");
}

void BoardDatabase_printExpGrid (BoardDatabase *db)
{
	PointI bl, tr;
	bl.x = 0;
	bl.y = 0;
	tr.x = GLOB_EXP_GRID_DIMS - 1;
	tr.y = GLOB_EXP_GRID_DIMS - 1;

	BoardDatabase_printExpGridSection (db, bl, tr);
}

void BoardDatabase_printLocalMapGrid (BoardDatabase *db)
{
	FILE *f = db->xmlLog;
	int i, j;
	fprintf (f, "<LocalMapGrid>\n");
	fprintf (f, "      ");
	for (i = 0; i < GLOB_LOC_MAP_GRID_DIMS; ++i)
	{
		fprintf (f, "%6d", i);
	}
	fprintf (f, "\n");
	for (j = GLOB_LOC_MAP_GRID_DIMS - 1; j >= 0; --j)
	{
		fprintf (f, "%6d", j);
		for (i = 0; i < GLOB_LOC_MAP_GRID_DIMS; ++i)
		{
			fprintf (f, "%6d", db->environment.localMapGrid[i][j]);
		}
		fprintf (f, "\n");
	}
	fprintf (f, "</LocalMapGrid>\n");
}

void BoardDatabase_printSupGrid (BoardDatabase *db)
{
	FILE *f = db->xmlLog;
	int i, j;
	fprintf (f, "<SupGrid>\n");
	fprintf (f, "      ");
	for (i = 0; i < SUP_GRID_DIMS; ++i)
	{
		fprintf (f, "%6d", i);
	}
	fprintf (f, "\n");
	for (j = SUP_GRID_DIMS - 1; j >= 0; --j)
	{
		fprintf (f, "%6d", j);
		for (i = 0; i < SUP_GRID_DIMS; ++i)
		{
			fprintf (f, "%6d", db->environment.supGrid[i][j]);
		}
		fprintf (f, "\n");
	}
	fprintf (f, "</SupGrid>\n");
}

void BoardDatabase_printAllGrids (BoardDatabase *db)
{
	BoardDatabase_printExpGrid (db);
	BoardDatabase_printLocalMapGrid (db);
	BoardDatabase_printSupGrid (db);
}

void BoardDatabase_printGlobalMapList (BoardDatabase *db)
{
	int totalBufferSize, totalStructSize;
	ListNode *iterator;
	CompressedImageWithMapInfo *map;

	totalStructSize = db->sensorData.globalMaps.size * sizeof (CompressedImageWithMapInfo);
	totalBufferSize = 0;

	fprintf (db->xmlLog, "<MapList>\n");
	fprintf (db->xmlLog, "bufferSizes=(");
	iterator = db->sensorData.globalMaps.front;
	while (iterator)
	{
		map = (CompressedImageWithMapInfo*)iterator->value;

		fprintf (db->xmlLog, "%d ", map->image.bufferSize);
		totalBufferSize += map->image.bufferSize;

		iterator = iterator->next;
	}

	fprintf (db->xmlLog, ")\n");
	fprintf (db->xmlLog, "nMaps=%d structSize=%d totalStructSize=%d totalBufferSize=%d totalSize=%d\n",
		db->sensorData.globalMaps.size, sizeof (CompressedImageWithMapInfo),
		totalStructSize, totalBufferSize, totalBufferSize + totalStructSize);
	fprintf (db->xmlLog, "</MapList>\n");
}


void BoardDatabase_getNMapped (BoardDatabase *db, int *nMapped, int *nOccupied, float *avgErrorMag)
{
	int i;
	int nm;
	int no;
	int diff;
	float avg;
	uchar *ptr;
	uchar u;

	nm = 0;
	no = 0;
	avg = 0.0f;
	ptr = db->environment.map->data;

	for (i = 0; i < (ENVIR_DIMS * ENVIR_DIMS); ++i)
	{
		u = *ptr;
		++ptr;
		nm += (int)(u != 127);
		no += (int)(u < 127);
		if (u != 127)
		{
			diff = abs(u - 127);
			avg += (float)((STD_DEV_PIXEL_GAP + STD_DEV_MAX) - diff);
		}
	}
	avg /= nm;

	*nMapped = nm;
	*nOccupied = no;
	*avgErrorMag = avg;
}

void BoardDatabase_printGlobalMap (BoardDatabase *db)
{
	int i;
	int nMapped;
	int nOccupied;
	float avgErrorMag;
	const int nCells = ENVIR_DIMS * ENVIR_DIMS;
	uchar *ptr;
	uchar u;
	time_t t;
	FILE *f;
	char mapFilename[120];
	int stdDevHist[256];

	time (&t);
	sprintf (mapFilename, "%s/globalMap_%d.dat", experimentDirName, (int)t);

	f = fopen (mapFilename, "wb");
	fwrite (db->environment.map->data, 1, ENVIR_DIMS * ENVIR_DIMS, f);
	fclose (f);

	BoardDatabase_getNMapped (db, &nMapped, &nOccupied, &avgErrorMag);

	fprintf (db->xmlLog, "<GlobalMap>nCells=%d nMapped=%d nOccupied=%d avgErrorMag=%f filename=\"%s\" cells=[",
		nCells, nMapped, nOccupied, avgErrorMag, mapFilename);

	for (i = 0; i < 256; ++i)
	{
		stdDevHist[i] = 0;
	}

	ptr = db->environment.map->data;
	for (i = 0; i < nCells; ++i)
	{
		u = *ptr;
		if (u != 127)
		{
			++stdDevHist[u];
		}
		++ptr;
	}

	for (i = 0; i < 256; ++i)
	{
		if (stdDevHist[i] != 0)
		{
			fprintf(db->xmlLog, "(%d,%d),", i, stdDevHist[i]);
		}
	}

	fprintf (db->xmlLog, "]</GlobalMap>");
}
#endif // ifndef ROBOT

