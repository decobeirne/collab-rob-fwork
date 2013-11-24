#include "RobotStatus.h"



#ifndef BOARD




RobotStatus initRobotStatus (const int index)
{
	int c = 48 + index;
	RobotStatus r;

	r.index = index;
	r.nIterations = 0;
	r.stdDev = 0;
	r.isTimeElapsed = 0;
	r.pose = initPose();
	r.isNewPose = 1;
	r.actualLocOffset = initPointF (0);
	r.scanCentrePt = initPointF (MIN_FLT);
	r.scanCentrePt = initPointF (MIN_FLT);
	r.target = initPointF (MIN_FLT);
	r.ikData.index = -1;

#ifdef IS_WIN
	strcpy_s ((char*)r.locWinName, 4, "loc");
	strcpy_s ((char*)r.navWinName, 4, "nav");
#elif defined (IS_GUMSTIX)
	strcpy (r.locWinName, "loc");
	strcpy (r.navWinName, "nav");
#else
	strcpy ((char*)r.locWinName, "loc");
	strcpy ((char*)r.navWinName, "nav");
#endif

	r.locWinName[3] = c;
	r.navWinName[3] = c;
	r.locWinName[4] = 0;
	r.navWinName[4] = 0;

#ifdef RERUNNING_ROBOT
	r.rerunning_file = NULL;
	r.rerunning_file = NULL;
	r.rerunning_startPtr = NULL;
#endif // ifdef RERUNNING_ROBOT

#ifdef RECORDING_ROBOT
	r.objIndex = -1;
#endif

	return r;
}

void RobotStatus_dtor (RobotStatus *r)
{
#ifdef RERUNNING_ROBOT
	if (r->rerunning_file)
	{
		fclose (r->rerunning_file);
	}
	if (r->rerunning_buffer)
	{
		free (r->rerunning_buffer);
	}
#endif // ifdef RERUNNING_ROBOT
}

#endif
