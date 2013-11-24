#include "../../Common/RobotDefs.h"

#if defined(RERUNNING_IMAGES_ONLY)
#include "../Data/RobotDatabase.h"
#include "../../Common/Comm/CommCore.h"

extern int g_featuresCalcdForImg;
static int s_rerunningImagesIndex = 0;
void RobotCommSerialize_readImage_rerunningImagesOnly (RobotDatabase *db)
{
	char temp[256];
	char buf[256];
	FILE *f;

	strcpy (temp, db->status.rerunning_experDirToRerun);
	sprintf (buf, "/img%05d.dat", s_rerunningImagesIndex);
	strncat (temp, buf, 256 - 1);

	s_rerunningImagesIndex++;

	// Copied from below
	{
		{
			f = fopen (temp, "rb");
			assert (f);

			fread (db->sensorData.camImg->data, 1, 143 * 176 * 3, f);
			fclose (f);

			g_featuresCalcdForImg = 0;
		}
	}
}
#endif // defined(RERUNNING_IMAGES_ONLY

#if defined(ROBOT) && (defined(RECORDING_ROBOT) || defined(RERUNNING_ROBOT))

#include "RobotCommSerialize.h"

extern char experimentDirName[128];

#if defined(RERUNNING_ROBOT)

void getNextRobotIter_serialize (RobotDatabase *db)
{
	int nRead, lenToKeep, lenToRead, currentBufLen;

	if (!db->status.rerunning_startPtr)
	{
		db->status.rerunning_atEnd = 0;
		lenToKeep = 0;
		lenToRead = 65536;
		nRead = fread (db->status.rerunning_buffer, 1, lenToRead, db->status.rerunning_file);
		currentBufLen = lenToKeep + nRead;
		db->status.rerunning_atEnd = (nRead != lenToRead);
#ifdef PRINT_DEBUG
		printf ("first read for recorded log:\n");
		printf ("lenToKeep %d lenToRead %d nRead %d atEnd %d currentBufLen %d\n", lenToKeep, lenToRead, nRead, db->status.rerunning_atEnd, currentBufLen);
#endif

		db->status.rerunning_startPtr = db->status.rerunning_buffer;
		db->status.rerunning_endPtr = strstr (db->status.rerunning_buffer, "<RobotStatus>");
		assert (db->status.rerunning_endPtr);
		db->status.rerunning_replacedChar = *db->status.rerunning_endPtr;
		*db->status.rerunning_endPtr = '\0';
	}
	else
	{
		*db->status.rerunning_endPtr = db->status.rerunning_replacedChar;

getNextRobotIter_search:
		db->status.rerunning_startPtr = db->status.rerunning_endPtr;
		db->status.rerunning_endPtr = strstr (db->status.rerunning_endPtr + 1, "<RobotStatus>");
		if (db->status.rerunning_endPtr)
		{
#ifdef PRINT_DEBUG
			printf ("start %d end %d\n", db->status.rerunning_startPtr - db->status.rerunning_buffer, db->status.rerunning_endPtr - db->status.rerunning_buffer);
#endif
			db->status.rerunning_replacedChar = *db->status.rerunning_endPtr;
			*db->status.rerunning_endPtr = '\0';
		}
		else
		{
			// <RobotStatus> was not found from beginning of buffer
			assert (db->status.rerunning_startPtr != db->status.rerunning_buffer);

			if (db->status.rerunning_atEnd)
			{
#ifdef PRINT_DEBUG
				printf ("Last iteration of recorded file reached\n");
#endif
			}
			else
			{
				lenToRead = (db->status.rerunning_startPtr - db->status.rerunning_buffer);
				lenToKeep = 65536 - lenToRead;
				strcpy (db->status.rerunning_buffer, db->status.rerunning_startPtr);
				nRead = fread (db->status.rerunning_buffer + lenToKeep, 1, lenToRead, db->status.rerunning_file);
				currentBufLen = lenToKeep + nRead;
				db->status.rerunning_endPtr = db->status.rerunning_buffer;
				db->status.rerunning_atEnd = (nRead != lenToRead);
#ifdef PRINT_DEBUG
				printf ("lenToKeep %d lenToRead %d nRead %d atEnd %d currentBufLen %d\n", lenToKeep, lenToRead, nRead, db->status.rerunning_atEnd, currentBufLen);
#endif
				goto getNextRobotIter_search;
			}
		}
	}
}

void readStatus_serialize (RobotDatabase *db, MapStatusFromBoard *s)
{
	char *ptr1, *ptr2;
	char temp[256];
	FILE *f;
	int id, sz;

	ptr1 = strstr (db->status.rerunning_startPtr, "<WriteObject>type=\"SER_STATUS_FROM_BOARD\" file=\"");
	assert (ptr1);
	ptr1 += 48;

	ptr2 = strstr (ptr1, "\"");
	strcpy (temp, db->status.rerunning_experDirToRerun);
	strncat (temp, ptr1, ptr2 - ptr1);

	f = fopen (temp, "rb");
	assert (f);

	fread (&id, sizeof (int), 1, f);
	assert (id == (int)SER_STATUS_FROM_BOARD);

	fread (&sz, sizeof (int), 1, f);
	assert (sz == sizeof (MapStatusFromBoard));

	fread (s, sz, 1, f);
	fclose (f);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<Rerunning>type=\"SER_STATUS_FROM_BOARD\"</Rerunning>\n");
#endif
}

extern int g_featuresCalcdForImg;
void readImage_serialize (RobotDatabase *db, int *isImg)
{
	// Look for camera image
	char *ptr1, *ptr2;
	char temp[256];
	FILE *f;

	ptr1 = strstr (db->status.rerunning_startPtr, "<SerializedImage>filename=\"");
	if (ptr1)
	{
		ptr1 += 27;

		ptr2 = strstr (ptr1, "\"");
		strcpy (temp, db->status.rerunning_experDirToRerun);
		strncat (temp, ptr1, ptr2 - ptr1);

		if (strstr (temp, "fakeCamData"))
		{
#ifdef IGNORE_CAM_DATA
#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<Rerunning>type=\"IGNORE_CAM_DATA\"<Rerunning>\n");
#endif

#else // IGNORE_CAM_DATA

			// If the experiment that was recorded was using fake cam data, then
			// this must also be defined now.
			assert (0);
#endif // IGNORE_CAM_DATA
			*isImg = 0;
		}
		else
		{
			f = fopen (temp, "rb");
			assert (f);

			fread (db->sensorData.camImg->data, 1, 143 * 176 * 3, f);
			fclose (f);

#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<Rerunning>type=\"SER_CAM\"</Rerunning>\n");
#endif

			*isImg = 1;

			g_featuresCalcdForImg = 0;
		}
	}
	else
	{
		*isImg = 0;
	}
}

#include "../../Common/Maths.h"

void readCompass_serialize (RobotDatabase *db)
{
	char *ptr1;
	float val;
	float dummy0, dummy1;

#ifdef RERUNNING_ROBOT
	fflush (db->xmlLog);
#endif

	ptr1 = strstr (db->status.rerunning_startPtr, "<SimulateOrientError>currentOrient=");
	if (ptr1)
	{
		ptr1 += 35;
		val = (float)strtod (ptr1, NULL);
		assert (fabs (val - db->status.pose.orient) < 0.02f);

		ptr1 = strstr (ptr1, "reading=");
		assert (ptr1);
		ptr1 += 8;
		db->status.pose.orient = (float)strtod (ptr1, NULL);

		// Call boxMuller so random vals will stay in sync
		boxMuller (&dummy0, &dummy1, NULL);

#ifdef PRINT_EVENTS
		fprintf (db->xmlLog, "<Rerunning>type=\"SimulateOrientError\" currentOrient=%14.12f reading=%14.12f</Rerunning>\n",
			val, db->status.pose.orient);
#endif
	}
	else
	{
#ifndef IGNORE_COMPASS_DATA
		ptr1 = strstr (db->status.rerunning_startPtr, "<CompassReading>currentOrient=");
		if (ptr1)
		{
			ptr1 += 30;
			val = (float)strtod (ptr1, NULL);
			assert (fabs (val - db->status.pose.orient) < 0.0001f);

			ptr1 = strstr (ptr1, "reading=");
			assert (ptr1);
			ptr1 += 8;
			db->status.pose.orient = (float)strtod (ptr1, NULL);

#ifdef PRINT_EVENTS
			fprintf (db->xmlLog, "<Rerunning>type=\"CompassReadings\" currentOrient=%14.12f reading=%14.12f</Rerunning>\n",
				val, db->status.pose.orient);
#endif
		}
#endif // ifndef IGNORE_COMPASS_DATA
	}
}

void readGrids_serialize (RobotDatabase *db)
{
	char *ptr1, *ptr2;
	char temp[256];
	FILE *f;
	int id, sz;

	ptr1 = strstr (db->status.rerunning_startPtr, "<WriteObject>type=\"SER_GRIDS\" file=\"");
	assert (ptr1);
	ptr1 += 36;

	ptr2 = strstr (ptr1, "\"");
	strcpy (temp, db->status.rerunning_experDirToRerun);
	strncat (temp, ptr1, ptr2 - ptr1);

	f = fopen (temp, "rb");
	assert (f);

	fread (&id, sizeof (int), 1, f);
	assert (id == (int)SER_GRIDS);

	fread (&sz, sizeof (int), 1, f);
	assert (sz == sizeof (__int16) * GLOB_LOC_MAP_GRID_DIMS * GLOB_LOC_MAP_GRID_DIMS +
		sizeof (__int16) * SUP_GRID_DIMS * SUP_GRID_DIMS +
		((NAV_GRID_DIMS * NAV_GRID_DIMS) / 8) + 1);

	fread (db->environment.localMapGrid, sizeof (__int16) * GLOB_LOC_MAP_GRID_DIMS * GLOB_LOC_MAP_GRID_DIMS, 1, f);
	fread (db->environment.supGrid, sizeof (__int16) * SUP_GRID_DIMS * SUP_GRID_DIMS, 1, f);
	fread (db->environment.obstructedCellGrid, ((NAV_GRID_DIMS * NAV_GRID_DIMS) / 8) + 1, 1, f);
	fclose (f);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<Rerunning>type=\"SER_GRIDS\"</Rerunning>\n");
#endif
}

void readInitialRobotLoc_serialize (RobotDatabase *db)
{
	char *ptr1;

	ptr1 = strstr (db->status.rerunning_startPtr, "<InitialRobotLoc>loc=(");
	assert (ptr1);
	ptr1 += 22;
	db->status.pose.loc.x = (float)strtod (ptr1, NULL);

	ptr1 = strstr (ptr1, ",");
	assert (ptr1);
	ptr1 += 1;
	db->status.pose.loc.y = (float)strtod (ptr1, NULL);

	ptr1 = strstr (ptr1, ",");
	assert (ptr1);
	ptr1 += 1;
	db->status.pose.orient = (float)strtod (ptr1, NULL);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<Rerunning>type=\"InitialRobotLoc\" vals=(%14.12f,%14.12f,%14.12f)</Rerunning>\n",
		db->status.pose.loc.x, db->status.pose.loc.y, db->status.pose.orient);
#endif
}

void readUnreachableGrid_serialize (RobotDatabase *db)
{
	char *ptr1, *ptr2;
	char temp[256];
	FILE *f;
	int id, sz;

	ptr1 = strstr (db->status.rerunning_startPtr, "<WriteObject>type=\"SER_UNREACHABLE\" file=\"");
	assert (ptr1);
	ptr1 += 42;

	ptr2 = strstr (ptr1, "\"");
	strcpy (temp, db->status.rerunning_experDirToRerun);
	strncat (temp, ptr1, ptr2 - ptr1);

	f = fopen (temp, "rb");
	assert (f);

	fread (&id, sizeof (int), 1, f);
	assert (id == (int)SER_UNREACHABLE);

	fread (&sz, sizeof (int), 1, f);
	assert (sz == ((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1);

	fread (db->environment.unreachableLocalMapGrid, sz, 1, f);
	fclose (f);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<Rerunning>type=\"SER_UNREACHABLE\"</Rerunning>\n");
#endif
}

void readMap_serialize (RobotDatabase *db)
{
	char *ptr1, *ptr2;
	char temp[256];
	FILE *f;
	int id, sz, usedSize;

	ptr1 = strstr (db->status.rerunning_startPtr, "<WriteObject>type=\"SER_MAP\" file=\"");
	assert (ptr1);
	ptr1 += 34;

	ptr2 = strstr (ptr1, "\"");
	strcpy (temp, db->status.rerunning_experDirToRerun);
	strncat (temp, ptr1, ptr2 - ptr1);

	f = fopen (temp, "rb");
	assert (f);

	fread (&id, sizeof (int), 1, f);
	assert (id == (int)SER_MAP);

	fread (&db->environment.mapFromBoard.orig, sizeof (PointI), 1, f); // Comment this back in after 0124
	fread (&sz, sizeof (int), 1, f);
	fread (&usedSize, sizeof (int), 1, f);
	db->environment.mapFromBoard.usedSize = usedSize;

	fread (db->environment.mapFromBoard.buffer, usedSize, 1, f);
	fclose (f);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<Rerunning>type=\"SER_MAP\"</Rerunning>\n");
#endif
}

void readGroup_serialize (RobotDatabase *db)
{
	char *ptr1, *ptr2;
	char temp[256];
	FILE *f;
	int id, sz;

	ptr1 = strstr (db->status.rerunning_startPtr, "<WriteObject>type=\"SER_GROUP\" file=\"");
	assert (ptr1);
	ptr1 += 36;

	ptr2 = strstr (ptr1, "\"");
	strcpy (temp, db->status.rerunning_experDirToRerun);
	strncat (temp, ptr1, ptr2 - ptr1);

	f = fopen (temp, "rb");
	assert (f);

	fread (&id, sizeof (int), 1, f);
	assert (id == (int)SER_GROUP);

	fread (&sz, sizeof (int), 1, f);
	assert (sz == sizeof (RobotData) * N_ROBOTS);

	fread (db->groupData.robots, sz, 1, f);
	fclose (f);

#ifdef PRINT_EVENTS
	fprintf (db->xmlLog, "<Rerunning>type=\"SER_GROUP\"</Rerunning>\n");
#endif
}

#endif // defined(RERUNNING_ROBOT)








#if defined(RECORDING_ROBOT)

void writeStatus_serialize (RobotDatabase *db, const MapStatusFromBoard *s)
{
	char filename[128];
	char fullpath[256];
	FILE *f;
	int id, sz;
	sprintf (filename, "/obj%05d.dat", ++db->status.objIndex);
	strcpy (fullpath, experimentDirName);
	strcat (fullpath, filename);
	f = fopen (fullpath, "wb");
	id = (int)SER_STATUS_FROM_BOARD;
	sz = sizeof (MapStatusFromBoard);

	fwrite (&id, sizeof (int), 1, f);
	fwrite (&sz, sizeof (int), 1, f);

	fwrite (s, sz, 1, f);

	fprintf (db->xmlLog, "<WriteObject>type=\"SER_STATUS_FROM_BOARD\" file=\"%s\"</WriteObject>\n", filename);
	fclose (f);
}

void writeGrids_serialize (RobotDatabase *db)
{
	char filename[128];
	char fullpath[256];
	FILE *f;
	int id, sz;
	sprintf (filename, "/obj%05d.dat", ++db->status.objIndex);
	strcpy (fullpath, experimentDirName);
	strcat (fullpath, filename);
	strcpy (fullpath, experimentDirName);
	strcat (fullpath, filename);
	f = fopen (fullpath, "wb");
	id = (int)SER_GRIDS;
	sz = sizeof (__int16) * GLOB_LOC_MAP_GRID_DIMS * GLOB_LOC_MAP_GRID_DIMS +
		sizeof (__int16) * SUP_GRID_DIMS * SUP_GRID_DIMS +
		((NAV_GRID_DIMS * NAV_GRID_DIMS) / 8) + 1;

	fwrite (&id, sizeof (int), 1, f);
	fwrite (&sz, sizeof (int), 1, f);

	fwrite (db->environment.localMapGrid, sizeof (__int16) * GLOB_LOC_MAP_GRID_DIMS * GLOB_LOC_MAP_GRID_DIMS, 1, f);
	fwrite (db->environment.supGrid, sizeof (__int16) * SUP_GRID_DIMS * SUP_GRID_DIMS, 1, f);
	fwrite (db->environment.obstructedCellGrid, ((NAV_GRID_DIMS * NAV_GRID_DIMS) / 8) + 1, 1, f);

	fprintf (db->xmlLog, "<WriteObject>type=\"SER_GRIDS\" file=\"%s\"</WriteObject>\n", filename);
	fclose (f);
}

void writeMap_serialize (RobotDatabase *db)
{
	char filename[128];
	char fullpath[256];
	FILE *f;
	int id, sz, temp;
	sprintf (filename, "/obj%05d.dat", ++db->status.objIndex);
	strcpy (fullpath, experimentDirName);
	strcat (fullpath, filename);
	f = fopen (fullpath, "wb");
	id = (int)SER_MAP;
	sz = sizeof (int) + db->environment.mapFromBoard.usedSize;

	fwrite (&id, sizeof (int), 1, f);
	fwrite (&db->environment.mapFromBoard.orig, sizeof (PointI), 1, f); // Comment this back in after 0124
	fwrite (&sz, sizeof (int), 1, f);

	temp = (int)db->environment.mapFromBoard.usedSize;
	fwrite (&temp, sizeof (int), 1, f);
	fwrite (db->environment.mapFromBoard.buffer, temp, 1, f);

	fprintf (db->xmlLog, "<WriteObject>type=\"SER_MAP\" file=\"%s\"</WriteObject>\n", filename);
	fclose (f);
}

void writeGroup_serialize (RobotDatabase *db)
{
	char filename[128];
	char fullpath[256];
	FILE *f;
	int id, sz;
	sprintf (filename, "/obj%05d.dat", ++db->status.objIndex);
	strcpy (fullpath, experimentDirName);
	strcat (fullpath, filename);
	f = fopen (fullpath, "wb");
	id = (int)SER_GROUP;
	sz = sizeof (RobotData) * N_ROBOTS;

	fwrite (&id, sizeof (int), 1, f);
	fwrite (&sz, sizeof (int), 1, f);

	fwrite (db->groupData.robots, sz, 1, f);

	fprintf (db->xmlLog, "<WriteObject>type=\"SER_GROUP\" file=\"%s\"</WriteObject>\n", filename);
	fclose (f);
}

void writeUnreachableGrid_serialize (RobotDatabase *db)
{
	char filename[128];
	char fullpath[256];
	FILE *f;
	int id, sz;
	sprintf (filename, "/obj%05d.dat", ++db->status.objIndex);
	strcpy (fullpath, experimentDirName);
	strcat (fullpath, filename);
	f = fopen (fullpath, "wb");
	id = (int)SER_UNREACHABLE;
	sz = ((GLOB_LOC_MAP_GRID_DIMS*GLOB_LOC_MAP_GRID_DIMS)/8)+1;

	fwrite (&id, sizeof (int), 1, f);
	fwrite (&sz, sizeof (int), 1, f);

	fwrite (db->environment.unreachableLocalMapGrid, sz, 1, f);

	fprintf (db->xmlLog, "<WriteObject>type=\"SER_UNREACHABLE\" file=\"%s\"</WriteObject>\n", filename);
	fclose (f);
}

#endif // defined(RECORDING_ROBOT)




#endif // defined(ROBOT)

