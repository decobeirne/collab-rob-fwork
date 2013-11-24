
#include "RobotCore.h"
#include "Bresenham.h"



// Inline functions that gcc was complaining about
#if defined(BOARD) || defined(TEST_SOCKETS)

// Only need a mutex when running as a board; simulation is single threaded.
#ifdef IS_LINUX
static pthread_mutex_t csBoardData = PTHREAD_MUTEX_INITIALIZER;
#else
static CRITICAL_SECTION csBoardData;
#endif

void initialiseCriticalSections()
{
#ifdef IS_LINUX
	// Nothing
#else
	InitializeCriticalSection (&csBoardData);
#endif
}
void enterBoardData()
{
#ifdef IS_LINUX
	pthread_mutex_lock (&csBoardData);
#else
	EnterCriticalSection (&csBoardData);
#endif
}
void leaveBoardData()
{
#ifdef IS_LINUX
	pthread_mutex_unlock (&csBoardData);
#else
	LeaveCriticalSection (&csBoardData);
#endif
}
void deleteCriticalSections()
{
#ifdef IS_LINUX
	// Nothing
#else
	DeleteCriticalSection (&csBoardData);
#endif
}
#endif // defined(BOARD) || defined(TEST_SOCKETS)

void Robot_sleep (const int milliseconds)
{
#ifdef IS_LINUX
	usleep (milliseconds);
#else
	Sleep (milliseconds);
#endif
}

char experimentDirName[128];

void setupExperimentDir()
{
	char readableTime[64];
	time_t experimentTime;
	struct tm *timeInfo;

	experimentTime = time (&experimentTime);
	timeInfo = localtime (&experimentTime);
	strftime (readableTime, 64, "-%Y%m%d-%a-%H%M%S", timeInfo);

#ifdef SIMULATION
	sprintf (experimentDirName, "Files/experiment_sim_%d", experimentTime);
#elif defined(BOARD)
	sprintf (experimentDirName, "Files/experiment_board_%d", (int)experimentTime);
#else
	sprintf (experimentDirName, "Files/experiment_robot_%d_%d", ROBOT_INDEX, (int)experimentTime);
#endif

	strcat (experimentDirName, readableTime);

#ifdef IS_LINUX
	mkdir (experimentDirName, S_IRWXU | S_IRWXG | S_IRWXO);
#else
	mkdir (experimentDirName);
#endif
}

char rootDirName[128];

void setupRootDir()
{
#ifdef ON_LAPTOP
	strcpy (rootDirName, "D:/thesis");
#else
	strcpy (rootDirName, "C:/stuff/t");
#endif
}

//! Check along a line to determine if a search value is present
/*!
Determine first if both or either of the pts are outside the img. If both
then return 0, i.e. search value was not found. If either, determine section
of line that is inside the img. The line iterator called is either single pixel
Bresenham or Bresenham complete, the latter being preferable for collision
detection

Old version required that both pts be inside the img, return an error, -1, if not

Return dist from start pt at which search value is found. This is returned as
a int, so it is limited to 127. If the actual dist is 0, this is changed to 1,
so that a return value of 0 can be used to mean the path is not blocked
*/
int RobotCore_checkLine (
						   const int beginX,
						   const int beginY,
						   const int endX,
						   const int endY,
						   const Image *img,
						   int (*condition)(const uchar a, const void *b),
						   const int searchValue,
						   const BRESENHAM_TYPE type,
						   const int isDistReqd,
						   uchar *hitValue)
{
	int axis;
	int count;
	PointI pt;
	PointI d;
	PointI step;
	int res;

	if (0 == Bresenham_setPts (
		img,
		beginX,
		beginY,
		endX,
		endY,
		&pt.x,
		&pt.y,
		&d.x,
		&d.y))
	{
		return 0;
	}

	Bresenham_setStep (
		&d.x,
		&d.y,
		&step.x,
		&step.y,
		&axis,
		&count);

	switch (type)
	{
	case SINGLE_PIXEL:
		res = Bresenham_stepWithCondition (
			pt.x,
			pt.y,
			step.x,
			step.y,
			d.x,
			d.y,
			axis,
			count,
			img,
			condition,
			searchValue,
			hitValue);
		break;
	case THREE_PIXELS:
	default:
		res = Bresenham_stepCompleteWithCondition (
			pt.x,
			pt.y,
			step.x,
			step.y,
			d.x,
			d.y,
			axis,
			count,
			img,
			condition,
			searchValue,
			hitValue);
		break;
	}

	if (!isDistReqd && res)
	{
		res = 1;
	}

	return res;
}



//! Determine if a robots curr pose is the same as the prev pose 
int RobotCore_comparePrevRobotPose (
									  const Pose *currPose,
									  const PointI *prevLoc,
									  const int prevOrient)
{
	if ((int)currPose->loc.x != prevLoc->x ||
		(int)currPose->loc.y != prevLoc->y ||
		(int)currPose->orient != prevOrient)
	{
		return 0;
	}

	return 1;
}

//! Copy only a specified colour from src image to dest
int RobotCore_copyImage_specColour (
									  Image *src,
									  Image *dest,
									  const uchar specColour)
{
	PointI bl;
	PointI tr;
	int i, j;

	{
		bl.x = 0;
		bl.y = 0;
		tr.x = dest->width;
		tr.y = dest->height;
	}

	for (i = bl.x; i < tr.x; ++i)
	{
		for (j = bl.y; j < tr.y; ++j)
		{
			if (specColour == Image_getPixel_dontCheck (src, i, j))
			{
				Image_setPixel_dontCheck (dest, i, j, specColour);
			}
		}
	}

	return 1;
}



int RobotCore_equalsNarrowOccupied (const uchar a, const void *b)
{
	// b is just a dummy here
	return (a <= NARROW_OCCUPIED);
}

int RobotCore_equalsActualOccupied (const uchar a, const void *b)
{
	// b is just a dummy here
	return (a <= NARROW_VEHICLE_TERRAIN);
}

int estMapScanGain (const PointF optimumPt,
					const PointI localMapOrigin,
					__int16 expGrid[EXP_GRID_DIMS][EXP_GRID_DIMS])
{
	PointI p;

	p.x = (int)optimumPt.x - localMapOrigin.x;
	p.y = (int)optimumPt.y - localMapOrigin.y;

	p = PointI_calcExpCellIndex (p, EXP_AREA);

	if (p.x < 0 ||
		p.y < 0 ||
		p.x >= EXP_GRID_DIMS ||
		p.y >= EXP_GRID_DIMS)
	{
		return 0;
	}
	else
	{
		return max (0, (EXP_AREA * EXP_AREA) - expGrid[p.x][p.y]);
	}
}

void RobotCore_printPointFList (List *pointIList, FILE *f)
{
	ListNode *iterator;
	PointI pt;

	iterator = pointIList->front;
	while (iterator)
	{
		pt = *((PointI*)iterator->value);
		fprintf (f, "%d,%d ", pt.x, pt.y);
	}
}

Coalition findExplorationCoalition (List *coalitions, const int index)
{
	ListNode *iter;
	Coalition *coalition;

	iter = coalitions->front;
	while (iter)
	{
		coalition = (Coalition*)iter->value;
		if (coalition->explorer == index)
		{
			return *coalition;
		}
		iter = iter->next;
	}

	return initCoalition (-1);
}

void findSupervisionCoalitions (List *boardCoalitions, List *robotCoalitions, const int index)
{
	ListNode *iter;
	Coalition *coalition;
	Coalition *newCoalition;

	iter = boardCoalitions->front;
	while (iter)
	{
		coalition = (Coalition*)iter->value;

		if (coalition->supervisor == index)
		{
			newCoalition = (Coalition*)malloc (sizeof (Coalition));
			memcpy (newCoalition, coalition, sizeof (Coalition));

			List_pushValue (robotCoalitions, newCoalition);
		}

		iter = iter->next;
	}
}

