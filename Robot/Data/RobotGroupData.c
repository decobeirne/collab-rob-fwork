#include "RobotGroupData.h"

#ifndef BOARD



RobotGroupData initRobotGroupData()
{
	int i;
	RobotGroupData r;

	for (i = 0; i < N_ROBOTS; ++i)
	{
		r.robots[i] = initRobotData();
		r.prevPoses[i] = initPoseSimple();
		r.isRobotReachable[i] = 1;
	}

	return r;
}


#endif

