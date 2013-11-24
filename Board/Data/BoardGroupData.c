#include "../../Common/RobotDefs.h"

#ifndef ROBOT
#include "BoardGroupData.h"

ExperimentStatus initExperimentStatus()
{
	ExperimentStatus e;
	e.nFinished = 0;
	e.updateFlag = 0;
	e.iteration = 0;
	e.isTimeElapsed = 0;
	return e;
}

BoardGroupData initBoardGroupData()
{
	int i;
	BoardGroupData g;
	g.experimentStatus = initExperimentStatus();
	for (i = 0; i < N_ROBOTS; ++i)
	{
		g.robots[i] = initRobotData();
	}
	return g;
}

void BoardGroupData_dtor (BoardGroupData *g){}

#endif


